/**
  Copyright (C) Samsung Electronics Co. LTD

  This software is proprietary of Samsung Electronics.
  No part of this software, either material or conceptual may be copied or distributed, transmitted,
  transcribed, stored in a retrieval system or translated into any human or computer language in any form by any means,
  electronic, mechanical, manual or otherwise, or disclosed
  to third parties without the express written permission of Samsung Electronics.

  Alternatively, this program is free software in case of open source project
  you can redistribute it and/or modify
  it under the terms of the GNU General Public License version 2 as
  published by the Free Software Foundation.
**/

#include <Library/DebugLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationHelperLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/TimerLib.h>
#include <Library/IoLib.h>

#include <Protocol/EFII2c.h>
#include <Protocol/EFIGpio.h>
#include <Library/I2cLib.h>

#include "I2c.h"

//
// Global bus data from SoC library
//
STATIC EFI_I2C_BUS_DATA *mBusData;
STATIC UINT8             mBusCount;

typedef struct {
  UINT8   *Buffer;
  UINT32  Length;
  UINT32  Flags;
} I2C_MSG_SEGMENT;

#define I2C_MAX_MSGS  3

typedef struct {
  I2C_MSG_SEGMENT Msgs[I2C_MAX_MSGS];
  UINT32          MsgNum;           // Total number of messages
  UINT32          MsgIdx;           // Current message index
  UINT32          MsgPtr;           // Current byte pointer within message
  UINT8           State;            // Current state machine state
  UINT8           SlaveAddr;        // 7-bit slave address for repeated start
} I2C_XFER_STATE;

STATIC
VOID
I2cCalcDivisor (
  IN  UINT32   ClkIn,
  IN  UINT32   Wanted,
  OUT UINT32  *Div1,
  OUT UINT32  *Divs
  )
{
  UINT32 CalcDivs;
  UINT32 CalcDiv1;
  UINT32 ClkPrescaler;

  ClkPrescaler = 16;
  CalcDivs     = ClkIn / Wanted;

  if (CalcDivs > (16 * 16)) {
    CalcDiv1 = 512;
  } else {
    CalcDiv1 = ClkPrescaler;
  }

  CalcDivs += CalcDiv1 - 1;
  CalcDivs /= CalcDiv1;

  if (CalcDivs == 0) {
    CalcDivs = 1;
  }

  if (CalcDivs > (ClkPrescaler + 1)) {
    CalcDivs = ClkPrescaler + 1;
  }

  *Divs = CalcDivs;
  *Div1 = CalcDiv1;
}

STATIC
VOID
I2cClockRate (
  IN UINT32 BaseAddress,
  IN UINT32 ClkOut,
  IN UINT32 ClkIn
  )
{
  UINT32 Divs;
  UINT32 Div1;
  UINT32 TargetFreq;
  UINT32 I2cCon;

  TargetFreq = ClkOut ? ClkOut : I2C_STAND_TX_CLOCK;
  I2cCalcDivisor (ClkIn, TargetFreq, &Div1, &Divs);

  I2cCon  = MmioRead32 (BaseAddress + I2C_I2CCON);
  I2cCon &= ~(I2CCON_SCALEMASK | I2CCON_TXDIV_512);
  I2cCon |= (Divs - 1);

  if (Div1 == 512) {
    I2cCon |= I2CCON_TXDIV_512;
  }

  MmioWrite32 (BaseAddress + I2C_I2CCON, I2cCon);
}

STATIC
VOID
I2cBusInit (
  IN UINT32 BaseAddress,
  IN UINT32 InputClock,
  IN UINT32 SpeedHz
  )
{
  UINT32 I2cStat;

  // Disable TX/RX to reset state
  I2cStat  = MmioRead32 (BaseAddress + I2C_I2CSTAT);
  I2cStat &= ~I2CSTAT_TXRXEN;
  MmioWrite32 (BaseAddress + I2C_I2CSTAT, I2cStat);

  // Clear I2CSTAT
  MmioWrite32 (BaseAddress + I2C_I2CSTAT, 0);

  // Set clock rate only if input clock is known
  if (InputClock != 0) {
    MmioWrite32 (BaseAddress + I2C_I2CCON, 0);
    I2cClockRate (BaseAddress, SpeedHz, InputClock);
  }

  MmioWrite32 (BaseAddress + I2C_I2CLC, I2CLC_FILTER_ON);
}

STATIC
EFI_STATUS
I2cSetMaster (
  IN UINT32 BaseAddress
  )
{
  UINT32 Timeout;
  UINT32 I2cStat;

  Timeout = I2C_MASTER_TIMEOUT;

  while (Timeout > 0) {
    I2cStat = MmioRead32 (BaseAddress + I2C_I2CSTAT);

    if ((I2cStat & I2CSTAT_BUSBUSY) == 0) {
      return EFI_SUCCESS;
    }

    MicroSecondDelay (10);
    Timeout--;
  }

  return EFI_TIMEOUT;
}

STATIC
EFI_STATUS
I2cWaitIdle (
  IN UINT32 BaseAddress
  )
{
  UINT32 Timeout;
  UINT32 I2cStat;

  Timeout = I2C_IDLE_TIMEOUT;

  while (Timeout > 0) {
    I2cStat = MmioRead32 (BaseAddress + I2C_I2CSTAT);

    if ((I2cStat & I2CSTAT_START) == 0) {
      return EFI_SUCCESS;
    }

    MicroSecondDelay (1);
    Timeout--;
  }

  return EFI_TIMEOUT;
}

STATIC
BOOLEAN
I2cIsAck (
  IN UINT32 BaseAddress,
  IN UINT8  State
  )
{
  UINT32 Tries;
  UINT32 I2cCon;
  UINT32 I2cStat;

  for (Tries = I2C_ACK_POLL_COUNT; Tries > 0; Tries--) {
    I2cCon = MmioRead32 (BaseAddress + I2C_I2CCON);

    if ((I2cCon & I2CCON_IRQPEND) != 0) {
      // In read mode, IRQPEND = data available regardless of ACK
      if (State == I2C_STATE_READ) {
        return TRUE;
      }

      // In write/start mode, check LASTBIT for NACK
      I2cStat = MmioRead32 (BaseAddress + I2C_I2CSTAT);
      if ((I2cStat & I2CSTAT_LASTBIT) == 0) {
        return TRUE;  // ACK received
      }

      return FALSE;   // NACK received
    }

    MicroSecondDelay (10);
  }

  return FALSE;
}

STATIC
VOID
I2cClearIrqPending (
  IN UINT32 BaseAddress
  )
{
  UINT32 I2cCon;

  I2cCon  = MmioRead32 (BaseAddress + I2C_I2CCON);
  I2cCon &= ~I2CCON_IRQPEND;
  MmioWrite32 (BaseAddress + I2C_I2CCON, I2cCon);
}

STATIC
VOID
I2cDisableAck (
  IN UINT32 BaseAddress
  )
{
  UINT32 I2cCon;

  I2cCon  = MmioRead32 (BaseAddress + I2C_I2CCON);
  I2cCon &= ~I2CCON_ACKEN;
  MmioWrite32 (BaseAddress + I2C_I2CCON, I2cCon);
}

STATIC
VOID
I2cEnableAck (
  IN UINT32 BaseAddress
  )
{
  UINT32 I2cCon;

  I2cCon  = MmioRead32 (BaseAddress + I2C_I2CCON);
  I2cCon |= I2CCON_ACKEN;
  MmioWrite32 (BaseAddress + I2C_I2CCON, I2cCon);
}

STATIC
VOID
I2cEnableIrq (
  IN UINT32 BaseAddress
  )
{
  UINT32 I2cCon;

  I2cCon  = MmioRead32 (BaseAddress + I2C_I2CCON);
  I2cCon |= I2CCON_IRQEN;
  MmioWrite32 (BaseAddress + I2C_I2CCON, I2cCon);
}

STATIC
VOID
I2cDisableIrq (
  IN UINT32 BaseAddress
  )
{
  UINT32 I2cCon;

  I2cCon  = MmioRead32 (BaseAddress + I2C_I2CCON);
  I2cCon &= ~I2CCON_IRQEN;
  MmioWrite32 (BaseAddress + I2C_I2CCON, I2cCon);
}

STATIC
VOID
I2cDisableBus (
  IN UINT32 BaseAddress
  )
{
  UINT32 Tmp;

  Tmp  = MmioRead32 (BaseAddress + I2C_I2CSTAT);
  Tmp &= ~I2CSTAT_TXRXEN;
  MmioWrite32 (BaseAddress + I2C_I2CSTAT, Tmp);

  // Disable IRQ, clear pending, disable ACK
  Tmp  = MmioRead32 (BaseAddress + I2C_I2CCON);
  Tmp &= ~(I2CCON_IRQEN | I2CCON_IRQPEND | I2CCON_ACKEN);
  MmioWrite32 (BaseAddress + I2C_I2CCON, Tmp);
}

STATIC
VOID
I2cMessageStart (
  IN UINT32 BaseAddress,
  IN UINT8  SlaveAddr,
  IN UINT32 Flags
  )
{
  UINT32 Addr;
  UINT32 I2cStat;
  UINT32 I2cCon;

  Addr    = (SlaveAddr & 0x7F) << 1;   // 7-bit address shifted left
  I2cStat = I2CSTAT_TXRXEN;

  if ((Flags & I2C_M_RD) != 0) {
    I2cStat |= I2CSTAT_MASTER_RX;
    Addr    |= 1;   // Set R/W bit for read
  } else {
    I2cStat |= I2CSTAT_MASTER_TX;
  }

  // Enable ACK
  I2cEnableAck (BaseAddress);

  // Set mode in I2CSTAT (write without START first)
  MmioWrite32 (BaseAddress + I2C_I2CSTAT, I2cStat);

  // Write address byte to data register
  MmioWrite32 (BaseAddress + I2C_I2CDS, Addr);

  // Delay to ensure data is on bus before START
  MicroSecondDelay (1);

  // Write I2CCON (restore ACKEN from read-modify-write)
  I2cCon  = MmioRead32 (BaseAddress + I2C_I2CCON);
  I2cCon |= I2CCON_ACKEN;
  MmioWrite32 (BaseAddress + I2C_I2CCON, I2cCon);

  // Set START condition
  I2cStat |= I2CSTAT_START;
  MmioWrite32 (BaseAddress + I2C_I2CSTAT, I2cStat);
}

STATIC
VOID
I2cStop (
  IN UINT32 BaseAddress
  )
{
  UINT32 I2cStat;

  // Clear START bit to generate STOP
  I2cStat  = MmioRead32 (BaseAddress + I2C_I2CSTAT);
  I2cStat &= ~I2CSTAT_START;
  MmioWrite32 (BaseAddress + I2C_I2CSTAT, I2cStat);

  // Disable interrupt
  I2cDisableIrq (BaseAddress);
}

STATIC
BOOLEAN
I2cIsMsgLast (
  IN I2C_XFER_STATE *Xfer
  )
{
  return Xfer->MsgPtr >= Xfer->Msgs[Xfer->MsgIdx].Length - 1;
}

STATIC
BOOLEAN
I2cIsMsgEnd (
  IN I2C_XFER_STATE *Xfer
  )
{
  return Xfer->MsgPtr >= Xfer->Msgs[Xfer->MsgIdx].Length;
}

STATIC
BOOLEAN
I2cIsLastMsg (
  IN I2C_XFER_STATE *Xfer
  )
{
  return Xfer->MsgIdx >= (Xfer->MsgNum - 1);
}

STATIC
EFI_STATUS
I2cNextByte (
  IN     UINT32         BaseAddress,
  IN OUT I2C_XFER_STATE *Xfer
  )
{
  UINT8  Byte;
  UINT32 I2cStat;
  EFI_STATUS RetStatus;

  I2cStat = MmioRead32 (BaseAddress + I2C_I2CSTAT);
  RetStatus = EFI_SUCCESS;

  switch (Xfer->State) {

  case I2C_STATE_START:
    // Check if address was NACKed
    if ((I2cStat & I2CSTAT_LASTBIT) != 0) {
      I2cStop (BaseAddress);
      Xfer->State = I2C_STATE_STOP;
      RetStatus = EFI_NOT_FOUND;   // No device at address
      goto out_ack;
    }

    // Transition to read or write mode based on current message flags
    if ((Xfer->Msgs[Xfer->MsgIdx].Flags & I2C_M_RD) != 0) {
      Xfer->State = I2C_STATE_READ;
    } else {
      Xfer->State = I2C_STATE_WRITE;
    }

    // If zero-length message, handle according to position in sequence
    if (Xfer->Msgs[Xfer->MsgIdx].Length == 0) {
      if (I2cIsLastMsg (Xfer)) {
        I2cStop (BaseAddress);
        Xfer->State = I2C_STATE_STOP;
        goto out_ack;
      }
      // Advance to next message with repeated start
      Xfer->MsgIdx++;
      Xfer->MsgPtr = 0;
      I2cMessageStart (BaseAddress, Xfer->SlaveAddr,
                       Xfer->Msgs[Xfer->MsgIdx].Flags);
      Xfer->State = I2C_STATE_START;
      goto out_ack;
    }

    if (Xfer->State == I2C_STATE_READ) {
      goto ReadByte;
    }
    break;

  case I2C_STATE_WRITE:
    // Check NACK
    if ((I2cStat & I2CSTAT_LASTBIT) != 0) {
      // Device NACKed data byte
      I2cStop (BaseAddress);
      Xfer->State = I2C_STATE_STOP;
      RetStatus = EFI_DEVICE_ERROR;
      goto out_ack;
    }

    if (!I2cIsMsgEnd (Xfer)) {
      // Write next byte from current message
      Byte = Xfer->Msgs[Xfer->MsgIdx].Buffer[Xfer->MsgPtr];
      Xfer->MsgPtr++;
      MmioWrite32 (BaseAddress + I2C_I2CDS, Byte);
      MicroSecondDelay (1);
    } else if (!I2cIsLastMsg (Xfer)) {
      // More messages follow - advance to next with repeated start
      Xfer->MsgPtr = 0;
      Xfer->MsgIdx++;
      I2cMessageStart (BaseAddress, Xfer->SlaveAddr,
                       Xfer->Msgs[Xfer->MsgIdx].Flags);
      Xfer->State = I2C_STATE_START;
      goto out_ack;
    } else {
      // All messages complete, send STOP
      I2cStop (BaseAddress);
      Xfer->State = I2C_STATE_STOP;
      goto out_ack;
    }
    break;

  case I2C_STATE_READ:
ReadByte:
    if (I2cIsMsgEnd (Xfer)) {
      // Read complete
      if (I2cIsLastMsg (Xfer)) {
        // Last message - send STOP
        (VOID)MmioRead32 (BaseAddress + I2C_I2CDS);
        I2cStop (BaseAddress);
        Xfer->State = I2C_STATE_STOP;
        goto out_ack;
      } else {
        // More messages follow - advance to next with repeated start
        Xfer->MsgPtr = 0;
        Xfer->MsgIdx++;
        I2cMessageStart (BaseAddress, Xfer->SlaveAddr,
                         Xfer->Msgs[Xfer->MsgIdx].Flags);
        Xfer->State = I2C_STATE_START;
        goto out_ack;
      }
    }

    // If this byte is the LAST one in current message, disable ACK BEFORE reading it
    // so the controller sends NACK to signal the slave to stop sending
    if (I2cIsMsgLast (Xfer)) {
      I2cDisableAck (BaseAddress);
    }

    // Read byte from data register
    Byte = (UINT8)MmioRead32 (BaseAddress + I2C_I2CDS);
    Xfer->Msgs[Xfer->MsgIdx].Buffer[Xfer->MsgPtr] = Byte;
    Xfer->MsgPtr++;

    // If we've read all bytes in current message
    if (I2cIsMsgEnd (Xfer)) {
      if (I2cIsLastMsg (Xfer)) {
        I2cStop (BaseAddress);
        Xfer->State = I2C_STATE_STOP;
        goto out_ack;
      } else {
        // Advance to next message with repeated start
        Xfer->MsgPtr = 0;
        Xfer->MsgIdx++;
        I2cMessageStart (BaseAddress, Xfer->SlaveAddr,
                         Xfer->Msgs[Xfer->MsgIdx].Flags);
        Xfer->State = I2C_STATE_START;
        goto out_ack;
      }
    }
    break;

  default:
    RetStatus = EFI_DEVICE_ERROR;
    goto out_ack;
  }

out_ack:
  I2cClearIrqPending (BaseAddress);
  return RetStatus;
}

STATIC
EFI_STATUS
I2cDoXfer (
  IN     UINT32         BaseAddress,
  IN     UINT8          SlaveAddr,
  IN OUT I2C_XFER_STATE *Xfer
  )
{
  EFI_STATUS Status;
  BOOLEAN    Ack;

  // Write slave address to address register
  MmioWrite32 (BaseAddress + I2C_I2CADD, (SlaveAddr & 0x7F) << 1);

  // Wait for bus to be free
  Status = I2cSetMaster (BaseAddress);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "%a: Bus busy! I2CSTAT=0x%08X\n",
      __FUNCTION__, MmioRead32 (BaseAddress + I2C_I2CSTAT)));
    return Status;
  }

  // Initialize transfer state
  Xfer->MsgPtr    = 0;
  Xfer->MsgIdx    = 0;
  Xfer->State     = I2C_STATE_START;
  Xfer->SlaveAddr = SlaveAddr;

  // Enable interrupts
  I2cEnableIrq (BaseAddress);

  // Send START + slave address for first message
  I2cMessageStart (BaseAddress, SlaveAddr, Xfer->Msgs[0].Flags);

  // Polling state machine loop
  while (Xfer->State != I2C_STATE_STOP) {
    Ack = I2cIsAck (BaseAddress, Xfer->State);

    if (!Ack) {
      // Arbitration lost or NACK
      DEBUG ((EFI_D_ERROR, "%a: No ACK! State=%u, I2CSTAT=0x%08X\n",
        __FUNCTION__, Xfer->State, MmioRead32 (BaseAddress + I2C_I2CSTAT)));

      // Check arbitration
      if (MmioRead32 (BaseAddress + I2C_I2CSTAT) & I2CSTAT_ARBITR) {
        DEBUG ((EFI_D_WARN, "%a: Arbitration lost\n", __FUNCTION__));
      }

      Status = EFI_DEVICE_ERROR;
      break;
    }

    Status = I2cNextByte (BaseAddress, Xfer);

    if (EFI_ERROR (Status)) {
      break;
    }
  }

  // Wait for STOP condition to complete
  if (!EFI_ERROR (Status)) {
    I2cWaitIdle (BaseAddress);
  }

  // Clean up: disable bus
  I2cDisableBus (BaseAddress);
  Xfer->State = I2C_STATE_IDLE;

  return Status;
}

STATIC
EFI_STATUS
I2cXferSingle (
  IN UINT32  BaseAddress,
  IN UINT32  InputClock,
  IN UINT32  SpeedHz,
  IN UINT8   SlaveAddr,
  IN UINT32  Flags,
  IN UINT8  *Buffer,
  IN UINT32  Length
  )
{
  EFI_STATUS     Status;
  UINT32         Retry;
  I2C_XFER_STATE Xfer;

  Xfer.MsgNum        = 1;
  Xfer.Msgs[0].Buffer = Buffer;
  Xfer.Msgs[0].Length = Length;
  Xfer.Msgs[0].Flags  = Flags;

  for (Retry = 0; Retry < I2C_MAX_RETRY; Retry++) {
    // Re-initialize bus before each attempt
    I2cBusInit (BaseAddress, InputClock, SpeedHz);

    Status = I2cDoXfer (BaseAddress, SlaveAddr, &Xfer);

    if (!EFI_ERROR (Status)) {
      return EFI_SUCCESS;
    }

    // Don't retry if device not found (NACK on address)
    if (Status == EFI_NOT_FOUND) {
      break;
    }

    MicroSecondDelay (I2C_RETRY_DELAY_US);
  }

  return Status;
}

STATIC
EFI_STATUS
I2cXferMulti (
  IN UINT32         BaseAddress,
  IN UINT32         InputClock,
  IN UINT32         SpeedHz,
  IN UINT8          SlaveAddr,
  IN I2C_MSG_SEGMENT *Segments,
  IN UINT32         NumSegments
  )
{
  EFI_STATUS     Status;
  UINT32         Retry;
  I2C_XFER_STATE Xfer;

  if (NumSegments > I2C_MAX_MSGS) {
    return EFI_INVALID_PARAMETER;
  }

  Xfer.MsgNum = NumSegments;
  CopyMem (Xfer.Msgs, Segments, NumSegments * sizeof (I2C_MSG_SEGMENT));

  for (Retry = 0; Retry < I2C_MAX_RETRY; Retry++) {
    I2cBusInit (BaseAddress, InputClock, SpeedHz);

    Status = I2cDoXfer (BaseAddress, SlaveAddr, &Xfer);

    if (!EFI_ERROR (Status)) {
      return EFI_SUCCESS;
    }

    if (Status == EFI_NOT_FOUND) {
      break;
    }

    MicroSecondDelay (I2C_RETRY_DELAY_US);

    // Reinitialize Xfer state after each attempt
    Xfer.MsgNum = NumSegments;
    CopyMem (Xfer.Msgs, Segments, NumSegments * sizeof (I2C_MSG_SEGMENT));
  }

  return Status;
}

//
// Look up input clock for a given I2C base address.
// Returns 0 if not found (BL2 pre-config assumed).
//
STATIC
UINT32
I2cGetClock (
  IN UINT32 BaseAddress
  )
{
  for (UINT8 i = 0; i < mBusCount; i++) {
    if ((UINT32)mBusData[i].BaseAddress == BaseAddress) {
      return mBusData[i].Clk;
    }
  }
  return 0;
}

STATIC
EFI_STATUS
I2cCombinedRead (
  IN  UINT32 BaseAddress,
  IN  UINT8  Slave,
  IN  UINT8  Addr,
  OUT UINT8 *Data,
  IN  UINT8  Count
  )
{
  EFI_STATUS      Status;
  I2C_MSG_SEGMENT Segments[2];

  //
  // Message sequence:
  //   [0] Write register address (no STOP)  — Sr:W, Addr
  //   [1] Read data (with STOP)             — Sr:Rd, Data...
  //
  Segments[0].Buffer = &Addr;
  Segments[0].Length = 1;
  Segments[0].Flags  = I2C_M_WR;

  UINT32 ClkIn;

  ClkIn = I2cGetClock (BaseAddress);

  Segments[1].Buffer = Data;
  Segments[1].Length = Count;
  Segments[1].Flags  = I2C_M_RD;

  Status = I2cXferMulti (BaseAddress, ClkIn, I2C_STAND_TX_CLOCK,
                          Slave, Segments, 2);

  return Status;
}

STATIC
EFI_STATUS
EFIAPI
I2cRead (
  IN  UINT32 BaseAddress,
  IN  UINT8  Slave,
  IN  UINT8  Addr,
  OUT UINT8 *Data
  )
{
  return I2cCombinedRead (BaseAddress, Slave, Addr, Data, 1);
}

STATIC
EFI_STATUS
EFIAPI
I2cBurstRead (
  IN  UINT32 BaseAddress,
  IN  UINT8  Slave,
  IN  UINT8  Addr,
  OUT UINT8 *Data,
  IN  UINT8  Count
  )
{
  if (Count == 0) {
    return EFI_SUCCESS;
  }

  return I2cCombinedRead (BaseAddress, Slave, Addr, Data, Count);
}

STATIC
EFI_STATUS
EFIAPI
I2cWrite (
  IN UINT32 BaseAddress,
  IN UINT8  Slave,
  IN UINT8  Addr,
  IN UINT8  Data
  )
{
  UINT8 Buffer[2];
  UINT32 ClkIn;

  ClkIn = I2cGetClock (BaseAddress);
  Buffer[0] = Addr;
  Buffer[1] = Data;

  return I2cXferSingle (BaseAddress, ClkIn, I2C_STAND_TX_CLOCK,
                        Slave, I2C_M_WR, Buffer, 2);
}

STATIC
EFI_STATUS
EFIAPI
I2cBurstWrite (
  IN UINT32 BaseAddress,
  IN UINT8  Slave,
  IN UINT8  Addr,
  IN UINT8 *Data,
  IN UINT8  Count
  )
{
  UINT8  Buffer[257];
  UINT32 TotalLen;
  UINT32 ClkIn;

  if (Count == 0) {
    return EFI_SUCCESS;
  }

  if (Count > 256) {
    return EFI_INVALID_PARAMETER;
  }

  ClkIn = I2cGetClock (BaseAddress);
  Buffer[0] = Addr;
  CopyMem (&Buffer[1], Data, Count);
  TotalLen = Count + 1;

  return I2cXferSingle (BaseAddress, ClkIn, I2C_STAND_TX_CLOCK,
                        Slave, I2C_M_WR, Buffer, TotalLen);
}

STATIC CONST EFI_I2C_PROTOCOL mI2cProtocol = {
  I2cRead,
  I2cBurstRead,
  I2cWrite,
  I2cBurstWrite
};

STATIC
VOID
I2cGpioConfig (
  IN EFI_GPIO_PROTOCOL          *Gpio,
  IN CONST EFI_I2C_GPIO_DATA    *GpioData
  )
{
  EFI_STATUS Status;

  if (Gpio == NULL) {
    return;
  }

  Status = Gpio->SetPinFunction (GpioData->SclBankId, GpioData->SclBankNum,
                                 GpioData->SclPin, GpioData->Function);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "%a: SCL SetPinFunction failed! Status=%r\n",
      __FUNCTION__, Status));
  }

  Status = Gpio->SetPinFunction (GpioData->SdaBankId, GpioData->SdaBankNum,
                                 GpioData->SdaPin, GpioData->Function);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "%a: SDA SetPinFunction failed! Status=%r\n",
      __FUNCTION__, Status));
  }
}

EFI_STATUS
EFIAPI
InitI2c (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS           Status;
  EFI_GPIO_PROTOCOL   *Gpio;
  EFI_I2C_BUS_DATA    *BusData;
  UINT8                BusCount;
  UINT8                i;

  GetI2cBusData (&BusData, &BusCount);

  if (BusCount == 0) {
    DEBUG ((EFI_D_WARN, "%a: No I2C buses configured. Skipping.\n",
      __FUNCTION__));
    return EFI_SUCCESS;
  }

  // Save bus data globally
  mBusData  = BusData;
  mBusCount = BusCount;

  Gpio = NULL;
  gBS->LocateProtocol (&gEfiGpioProtocolGuid, NULL, (VOID *)&Gpio);

  for (i = 0; i < BusCount; i++) {
    CONST EFI_I2C_BUS_DATA *Bus = &BusData[i];

    // Configure GPIO pins
    I2cGpioConfig (Gpio, &Bus->Gpio);
  }

  DEBUG ((EFI_D_INFO, "%a: %u I2C channel(s) initialized\n",
    __FUNCTION__, BusCount));

  Status = gBS->InstallMultipleProtocolInterfaces (
                  &ImageHandle,
                  &gEfiI2cProtocolGuid,
                  (VOID *)&mI2cProtocol,
                  NULL);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "%a: Failed to install I2C protocol! Status=%r\n",
      __FUNCTION__, Status));
    ASSERT_EFI_ERROR (Status);
  }

  return EFI_SUCCESS;
}
