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
#include <Library/UefiBootServicesTableLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/ArmLib.h>

//#include <Protocol/EfiExynosHSI2C.h>
#include <Protocol/EfiExynosGpio.h>

#include "ExynosHSI2C.h"
#include "I2C_Common.h"

static struct HSI2CChannelInfo *ChannelInfo;
EFI_EXYNOS_GPIO_PROTOCOL *mExynosGpio;

// 9610 SPECIFIC START

#define MAX_HSI2C_CHAN	(1)
#define CLK_PERI_HSI2C	(200000000)
#define CLK_CMGP_USI	(200000000)

#define EXYNOS9610_BLK_PERI_HSI2C_BASE 0x10550000
#define EXYNOS9610_BLK_HSI2C_5 (EXYNOS9610_BLK_PERI_HSI2C_BASE + 0x50000)

#define EXYNOS9610_GPIO_PERIC0_BASE                     0x110C0000
#define EXYNOS9610_GPM1CON (EXYNOS9610_GPIO_PERIC0_BASE + 0x020)

#define EXYNOS9610_SYSREG_PERI_BASE            0x110C0000
#define EXYNOS9610_HSI2C_5_SW_CONF             (EXYNOS9610_SYSREG_PERI_BASE + 0x1018)

static struct HSI2CChannelInfo exynos9610_hsi2c_chan_info[] = {
	{
		/* USI_HSI2C_5 */
		.HSI2CAddress = EXYNOS9610_BLK_HSI2C_5,
		.SysregAddress = EXYNOS9610_HSI2C_5_SW_CONF,
		.GpioScl = (unsigned int *)EXYNOS9610_GPM1CON,
		.GpioSda = (unsigned int *)EXYNOS9610_GPM1CON,
		.SclPin = 2,
		.SdaPin = 3,
		.ConValue = 2,
		.Clock = CLK_PERI_HSI2C,
	}
};

static struct HSI2CChannelInfo *match_hsi2c_chan_info(void)
{
	return exynos9610_hsi2c_chan_info;
}

// 9610 SPECIFIC END

// Generic LK Functions

struct fp_32_64 {
    UINT32 l0;    /* unshifted value */
    UINT32 l32;   /* value shifted left 32 bits (or bit -1 to -32) */
    UINT32 l64;   /* value shifted left 64 bits (or bit -33 to -64) */
};


STATIC UINT64
mul_u32_u32(UINT32 a, UINT32 b, INTN a_shift, INTN b_shift)
{
    return (UINT64)a * b;
}

STATIC UINT32
u32_mul_u64_fp32_64(UINT64 a, struct fp_32_64 b)
{
    UINT32 a_r32 = a >> 32;
    UINT32 a_0 = a;
    UINT64 res_l32;
    UINT32 ret;

    /* mul_u32_u32(a_r32, b.l0, 32, 0) does not affect result */
    res_l32 = mul_u32_u32(a_0, b.l0, 0, 0) << 32;
    res_l32 += mul_u32_u32(a_r32, b.l32, 32, -32) << 32;
    res_l32 += mul_u32_u32(a_0, b.l32, 0, -32);
    res_l32 += mul_u32_u32(a_r32, b.l64, 32, -64);
    res_l32 += mul_u32_u32(a_0, b.l64, 0, -64) >> 32; /* Improve rounding accuracy */
    ret = (res_l32 >> 32) + ((UINT32)res_l32 >> 31); /* Round to nearest integer */

    return ret;
}

static void
fp_32_64_div_32_32(struct fp_32_64 *result, UINT32 dividend, UINT32 divisor)
{
    UINT64 tmp;
    UINT32 rem;

    tmp = ((UINT64)dividend << 32) / divisor;
    rem = ((UINT64)dividend << 32) % divisor;
    result->l0 = tmp >> 32;
    result->l32 = tmp;
    tmp = ((UINT64)rem << 32) / divisor;
    result->l64 = tmp;
}

struct fp_32_64 ms_per_cntpct;

static UINT32 cntpct_to_lk_time(UINT64 cntpct)
{
    return u32_mul_u64_fp32_64(cntpct, ms_per_cntpct);
}

UINT32 current_time()
{
    return cntpct_to_lk_time(ArmReadCntPct());
}

// Generic LK Functions end

STATIC VOID
SetBits (
  UINT32 Value,
  UINT32 Address,
  UINT32 Offset)
{
  UINT32 RegValue;

  RegValue = MmioRead32(Address);
  RegValue &= ~(0xF << Offset);
  RegValue |= Value << Offset;

  MmioWrite32(Address, RegValue);
}

VOID
HSI2CGpioSet (INTN Channel)
{
  ExynosGpioBank *BankScl, *BankSda;
  UINT32 SclPin, SdaPin, ConValue;

  BankScl = (ExynosGpioBank *)ChannelInfo[Channel].GpioScl;
  BankSda = (ExynosGpioBank *)ChannelInfo[Channel].GpioSda;

  SclPin = ChannelInfo[Channel].SclPin;
  SdaPin = ChannelInfo[Channel].SdaPin;
  ConValue = ChannelInfo[Channel].ConValue;

  mExynosGpio->ConfigurePin(BankScl, 0, SclPin, ConValue);
  mExynosGpio->ConfigurePin(BankSda, 0, SdaPin, ConValue);

  DEBUG((EFI_D_WARN, "HSI2C: Con - SCL(0x%08x, %u) -> %u\n",
	 BankScl->Con, SclPin, ConValue));

  DEBUG((EFI_D_WARN, "HSI2C: Con - SDA(0x%08x, %u) -> %u\n",
         BankSda->Con, SdaPin, ConValue));
}

VOID
HSI2CUsiSwConf (INTN Channel)
{
  UINT32 SysregAddress;

  SysregAddress = ChannelInfo[Channel].SysregAddress;
  SetBits(I2C_SW_CONF, SysregAddress, 0);

  DEBUG((EFI_D_WARN, "HSI2C: USI_SW_CONF: sysreg(0x%08x) -> %u\n",
         SysregAddress, MmioRead32(SysregAddress)));
}

VOID
HSI2CSetTiming (
  UINT32 HSI2CAddress,
  UINT32 IPClock,
  INTN Mode)
{
  UINT32 OPClock;

  UINT32 HsDiv, uTSCL_H_HS, uTSTART_HD_HS;
  UINT32 FsDiv, uTSCL_H_FS, uTSTART_HD_FS;

  UINT32 UTemp;

  switch(Mode)
  {
    case I2C_STAND_SPD:
      OPClock = I2C_STAND_TX_CLOCK;

      FsDiv = IPClock / (OPClock * 16);
      FsDiv &= 0xFF;
      UTemp = MmioRead32(HSI2CAddress + I2C_TIMING_FS3);
      UTemp &= ~0x00FF0000;
      UTemp |= FsDiv << 16;
      MmioWrite32(UTemp, HSI2CAddress + I2C_TIMING_FS3);

      uTSCL_H_FS = (25 * (IPClock / (1000 * 1000))) / ((FsDiv + 1) * 10);
      uTSCL_H_FS = (0xFF << uTSCL_H_FS) & 0xFF;
      UTemp = MmioRead32(HSI2CAddress + I2C_TIMING_FS2);
      UTemp &= ~0x000000FF;
      UTemp |= uTSCL_H_FS << 0;
      MmioWrite32(HSI2CAddress + I2C_TIMING_FS2, UTemp);

      uTSTART_HD_FS = (25 * (IPClock / (1000 * 1000))) / ((FsDiv + 1) * 10) - 1;
      uTSTART_HD_FS = (0xFF << uTSTART_HD_FS) & 0xFF;

      UTemp = MmioRead32(HSI2CAddress + I2C_TIMING_FS1);
      UTemp &= ~0x00FF0000;
      UTemp |= uTSTART_HD_FS << 16;
      MmioWrite32(HSI2CAddress + I2C_TIMING_FS1, UTemp);
        break;
    case I2C_FAST_PLUS_SPD:
      OPClock = I2C_FAST_PLUS_TX_CLOCK;

      FsDiv = IPClock / (OPClock * 16) - 1;

      FsDiv &= 0xFF;
      UTemp = MmioRead32(HSI2CAddress + I2C_TIMING_FS3);
      UTemp &= ~0x00FF0000;
      UTemp |= FsDiv << 16;
      MmioWrite32(HSI2CAddress + I2C_TIMING_FS3, UTemp);

      uTSCL_H_FS = (4 * (IPClock / (1000 * 1000))) / ((FsDiv + 1) * 10);
      uTSCL_H_FS = (0xFF << uTSCL_H_FS) & 0xFF;
      UTemp = MmioRead32(HSI2CAddress + I2C_TIMING_FS2);
      UTemp &= ~0x000000FF;
      UTemp |= uTSCL_H_FS << 0;
      MmioWrite32(HSI2CAddress + I2C_TIMING_FS2, UTemp);

      uTSTART_HD_FS = (4 * (IPClock / (1000 * 1000))) / ((FsDiv + 1) * 10) - 1;
      uTSTART_HD_FS = (0xFF << uTSTART_HD_FS) & 0xFF;
      UTemp = MmioRead32(HSI2CAddress + I2C_TIMING_FS1);
      UTemp &= ~0x00FF0000;
      UTemp |= uTSTART_HD_FS << 16;
      MmioWrite32(HSI2CAddress + I2C_TIMING_FS1, UTemp);
        break;
    case I2C_HIGH_SPD:
      /* IPClock's unit is in HZ, OPClock's unit is in HZ */
      OPClock = I2C_HS_TX_CLOCK;
      HsDiv = IPClock / (OPClock * 15);
      HsDiv &= 0xFF;
      UTemp = MmioRead32(HSI2CAddress + I2C_TIMING_HS3);
      UTemp &= ~0x00FF0000;
      UTemp |= HsDiv << 16;
      MmioWrite32(HSI2CAddress + I2C_TIMING_HS3, UTemp);

      uTSCL_H_HS = ((7 * IPClock) / (1000 * 1000)) / ((HsDiv + 1) * 100);
      /* Make to 0 into TSCL_H_HS from LSB */
      uTSCL_H_HS = (0xFFFFFFFF >> uTSCL_H_HS) << uTSCL_H_HS;
      uTSCL_H_HS &= 0xFF;
      UTemp = MmioRead32(HSI2CAddress + I2C_TIMING_HS2);
      UTemp &= ~0x000000FF;
      UTemp |= uTSCL_H_HS << 0;
      MmioWrite32(HSI2CAddress + I2C_TIMING_HS2, UTemp);

      uTSTART_HD_HS = (7 * IPClock / (1000 * 1000)) / ((HsDiv + 1) * 100) - 1;
      /* Make to 0 into uTSTART_HD_HS from LSB */
      uTSTART_HD_HS = (0xFFFFFFFF >> uTSTART_HD_HS) << uTSTART_HD_HS;
      uTSTART_HD_HS &= 0xFF;
      UTemp = MmioRead32(HSI2CAddress + I2C_TIMING_HS1);
      UTemp &= ~0x00FF0000;
      UTemp |= uTSTART_HD_HS << 16;
      MmioWrite32(HSI2CAddress + I2C_TIMING_HS1, UTemp);
        break;
    default:
      /* Fast speed mode */
      /* OPClock's unit is in HZ, OPClock's unit is HZ */
      OPClock = I2C_FS_TX_CLOCK;
      FsDiv = IPClock / (OPClock * 15);
      FsDiv &= 0xFF;
      UTemp = MmioRead32(HSI2CAddress + I2C_TIMING_FS3);
      UTemp &= ~0x00FF0000;
      UTemp |= FsDiv << 16;
      MmioWrite32(HSI2CAddress + I2C_TIMING_FS3, UTemp);

      uTSCL_H_FS = ((9 * IPClock) / (1000 * 1000)) / ((FsDiv + 1) * 10);
      /* make to 0 into TSCL_H_FS from LSB */
      uTSCL_H_FS = (0xFFFFFFFF >> uTSCL_H_FS) << uTSCL_H_FS;
      uTSCL_H_FS &= 0xFF;
      UTemp = MmioRead32(HSI2CAddress + I2C_TIMING_FS2);
      UTemp &= ~0x000000FF;
      UTemp |= uTSCL_H_FS << 0;

      uTSTART_HD_FS = (9 * IPClock / (1000 * 1000)) / ((FsDiv + 1) * 10) - 1;
      /* Make to 0 into uTSTART_HD_FS from LSB */
      uTSTART_HD_FS = (0xFFFFFFFF >> uTSTART_HD_FS) << uTSTART_HD_FS;
      uTSTART_HD_FS &= 0xFF;
      UTemp = MmioRead32(HSI2CAddress + I2C_TIMING_FS1);
      UTemp &= ~0x00FF0000;
      MmioWrite32(HSI2CAddress + I2C_TIMING_FS1, UTemp);
        break;
  }
}

VOID
HSI2CClockSetup (struct i2c_info *HSI2C)
{
  UINT32 IPClock = ChannelInfo[HSI2C->Channel].Clock;

  DEBUG((EFI_D_WARN, "HSI2C: 0x%08x: ClockSetup: SpeedMode=%d\n",
         HSI2C->Base, HSI2C->Mode));

  if(HSI2C->Mode == I2C_HIGH_SPD)
    HSI2CSetTiming(HSI2C->Base, IPClock, I2C_FAST_SPD);

  HSI2CSetTiming(HSI2C->Base, IPClock, HSI2C->Mode);
}

VOID
HSI2CResetI2C (struct i2c_info *HSI2C)
{
  UINT32 I2CControl, I2CConfig;

  DEBUG((EFI_D_WARN, "HSI2C: 0x%08x: I2CReset\n", HSI2C->Base));

  /* Set and clear the bit for reset */
  I2CControl = MmioRead32(HSI2C->Base + CTL);
  I2CControl |= HSI2C_SW_RST;
  MmioWrite32(HSI2C->Base + CTL, I2CControl);

  I2CControl = MmioRead32(HSI2C->Base + CTL);
  I2CControl &= ~HSI2C_SW_RST;
  MmioWrite32(HSI2C->Base + CTL, I2CControl);

  /* We don't expect calculations to fail during the run */
  HSI2CClockSetup(HSI2C);

  /* Configure registers */
  I2CConfig = MmioRead32(HSI2C->Base + I2C_CONF);

  MmioWrite32(HSI2C->Base + CTL, HSI2C_MASTER);
  MmioWrite32(HSI2C->Base + TRAILING_CTL, HSI2C_TRAILING_COUNT);

  if(HSI2C->Mode == I2C_HIGH_SPD)
  {
    MmioWrite32(HSI2C->Base + I2C_ADDR, HSI2C_MASTER_ID(MASTER_ID(HSI2C->Channel)));

    I2CConfig |= HSI2C_HS_MODE;
  }

  I2CConfig |= HSI2C_AUTO_MODE;
  MmioWrite32(HSI2C->Base + I2C_CONF, I2CConfig);
}

EFI_STATUS
HSI2CXferMessage(
  struct i2c_info *HSI2C,
  struct i2c_msg *Message,
  INTN Stop
)
{
  UINT32 I2CControl, I2CAutoConfiguration, I2CTimeout, I2CAddress, I2CFifoControl;
  UINT32 InterruptEnable, InterruptStatus;
  UINT32 FifoStatus;
  UINT32 TransferStatus;
  UINT8  Byte;
  UINT32 MessagePointer = 0;
  EFI_STATUS Status = EFI_SUCCESS;

  DEBUG((EFI_D_WARN, "HSI2C: 0x%08x XferMessage\n"));

  I2CControl = MmioRead32(HSI2C->Base + CTL);
  I2CAutoConfiguration = MmioRead32(HSI2C->Base + I2C_AUTO_CONF);
  I2CTimeout = MmioRead32(HSI2C->Base + I2C_TIMEOUT);
  I2CTimeout &= ~HSI2C_TIMEOUT_EN;
  MmioWrite32(HSI2C->Base + I2C_TIMEOUT, I2CTimeout);

  /*
  * In case of a short length request it's better to set
  * the trigger level as message length
  */

  if (Message->Length >= FIFO_TRIG_CRITERIA)
  {
    I2CFifoControl = HSI2C_RXFIFO_EN | HSI2C_TXFIFO_EN |
		     HSI2C_RXFIFO_TRIGGER_LEVEL(FIFO_TRIG_CRITERIA) |
		     HSI2C_TXFIFO_TRIGGER_LEVEL(FIFO_TRIG_CRITERIA);
  }
  else
  {
    I2CFifoControl = HSI2C_RXFIFO_EN | HSI2C_TXFIFO_EN |
		     HSI2C_RXFIFO_TRIGGER_LEVEL(Message->Length) |
		     HSI2C_TXFIFO_TRIGGER_LEVEL(Message->Length);
  }

  MmioWrite32(HSI2C->Base + FIFO_CTL, I2CFifoControl);

  if(Message->Flags == I2C_M_RD)
  {
    I2CControl &= ~HSI2C_TXCHON;
    I2CControl |= HSI2C_RXCHON;

    I2CAutoConfiguration |= HSI2C_READ_WRITE;
  }
  else
  {
    I2CControl &= ~HSI2C_RXCHON;
    I2CControl |= HSI2C_TXCHON;

    I2CAutoConfiguration &= HSI2C_READ_WRITE;
  }

  if (Stop)
  {
    I2CAutoConfiguration |= HSI2C_STOP_AFTER_TRANS;
  }
  else
  {
    I2CAutoConfiguration &= ~HSI2C_STOP_AFTER_TRANS;
  }

  I2CAddress = MmioRead32(HSI2C->Base + I2C_ADDR);
  I2CAddress &= ~(0x3FF << 10);
  I2CAddress &= ~(0x3FF << 0);
  I2CAddress &= ~(0xFF << 24);
  I2CAddress |= ((Message->Address & 0x7F) << 10);
  MmioWrite32(HSI2C->Base + I2C_ADDR, I2CAddress);

  MmioWrite32(HSI2C->Base + CTL, I2CControl);

  MmioWrite32(HSI2C->Base + INT_EN, HSI2C_INT_TRANSFER_DONE);

  I2CAutoConfiguration &= ~(0xFFFF);
  I2CAutoConfiguration |= Message->Length;
  MmioWrite32(HSI2C->Base + I2C_AUTO_CONF, I2CAutoConfiguration);

  I2CAutoConfiguration = MmioRead32(HSI2C->Base + I2C_AUTO_CONF);
  I2CAutoConfiguration |= HSI2C_MASTER_RUN;
  MmioWrite32(HSI2C->Base + I2C_AUTO_CONF, I2CAutoConfiguration);

  Status = RETURN_LOAD_ERROR;

  if(Message->Flags == I2C_M_RD)
  {
    UINT32 Timeout;

    Timeout = current_time() + MSEC_PER_SEC;
    /* 1sec polling */
    while (current_time() <= Timeout) 
    {
      if ((MmioRead32(HSI2C->Base + FIFO_STAT) & HSI2C_RX_FIFO_EMPTY) == 0)
      {
        /* RX FIFO is not empty */
        Byte = (UINT8)MmioRead32(HSI2C->Base + RXDATA);
        Message->Buffer[MessagePointer++] = Byte;

        DEBUG((EFI_D_WARN, "HSI2C 0x%08x: XferMsg: read data 0x%x from RXDATA reg\n",
              HSI2C->Base, Byte));
      }

      if (MessagePointer >= Message->Length)
      {
        InterruptStatus = MmioRead32(HSI2C->Base + INT_STAT);

        if(InterruptStatus & HSI2C_INT_TRANSFER_DONE)
        {
          MmioWrite32(HSI2C->Base + INT_STAT, InterruptStatus);
          Status = EFI_SUCCESS;
          break;
        }
      }
    }
  if (Status == RETURN_LOAD_ERROR) goto error;
  }
  else
  {
    UINT32 FifoStatus;
    UINT32 Timeout;

    DEBUG((EFI_D_WARN, "HSI2C 0x%08x: XferMessage: Message - Write\n", HSI2C->Base));

    Timeout = current_time() + 1000;

    while ((current_time() <= Timeout) && (MessagePointer < Message->Length))
    {
      if((MmioRead32(HSI2C->Base + FIFO_STAT) & HSI2C_TX_FIFO_LVL_MASK) < EXYNOS_FIFO_SIZE)
      {
        Byte = Message->Buffer[MessagePointer++];
        MmioWrite32(HSI2C->Base + TXDATA, Byte);

        DEBUG((EFI_D_WARN, "HSI2C 0x%08x: XferMessage: Write data 0x%x to TXDATA reg\n",
               HSI2C->Base, Byte));
      }
    }

    while (current_time() <= Timeout)
    {
      InterruptStatus = MmioRead32(HSI2C->Base + INT_STAT);
      FifoStatus = MmioRead32(HSI2C->Base + FIFO_STAT);

      if ((InterruptStatus & HSI2C_INT_TRANSFER_DONE) &&  (FifoStatus & HSI2C_TX_FIFO_EMPTY))
      {
        MmioWrite32(HSI2C->Base + INT_STAT, InterruptStatus);
        Status = EFI_SUCCESS;
        break;
      }
      MicroSecondDelay(1);
    }

    /* TX Error */
    if (Status == RETURN_LOAD_ERROR) goto error;
  }
  return Status;

  error:
    DEBUG((EFI_D_ERROR, "HSI2C 0X%08x: XferMessage: %s Error\n", HSI2C->Base, ((Message->Flags & I2C_M_RD) ? "RX" : "TX")));

    /* Dump HSI2C Registers */
    I2CControl = MmioRead32(HSI2C->Base + CTL);
    I2CFifoControl = MmioRead32(HSI2C->Base + FIFO_CTL);
    InterruptEnable = MmioRead32(HSI2C->Base + INT_EN);

    InterruptStatus = MmioRead32(HSI2C->Base + INT_STAT);
    FifoStatus = MmioRead32(HSI2C->Base + FIFO_STAT);
    TransferStatus = MmioRead32(HSI2C->Base + I2C_TRANS_STATUS);

    DEBUG((EFI_D_ERROR, "HSI2C 0x%08x: ctl=0x%x, fifoctl=0x%x, int_en=0x%x\n",
           HSI2C->Base, I2CControl, I2CFifoControl, InterruptEnable));

    DEBUG((EFI_D_ERROR, "HSI2C 0x%08x: int_stat=0x%x, fifo_stat=0x%x, trans_stat=0x%x\n",
           HSI2C->Base, InterruptStatus, FifoStatus, TransferStatus));

    HSI2CResetI2C(HSI2C);
    return Status;
}

EFI_STATUS
HSI2CXfer (
  struct i2c_info *HSI2C, struct i2c_msg *Messages, INTN Number
)
{
  EFI_STATUS Status = EFI_SUCCESS;
  struct i2c_msg *MessagesPointer = Messages;
  INTN i;
  INTN Stop;

  if(HSI2C->InitialisationDone)
  {
    DEBUG((EFI_D_ERROR, "HSI2C 0x%08x: HSI2CXfer: channel(%d) not initialized\n",
           HSI2C->Base, HSI2C->Channel));
    Status = RETURN_LOAD_ERROR;
    return Status;
  }

  for(INTN Retry = 0; Retry < 5; Retry++)
  {
    for (i = 0; i < Number; i++)
    {
      if (i == Number - 1) Stop = 1;

      Status = HSI2CXferMessage(HSI2C, MessagesPointer, Stop);
      MessagesPointer++;

      if (Status == RETURN_LOAD_ERROR)
      {
        MessagesPointer = Messages;
        break;
      }
    }

    if((i == Number) && (Status != RETURN_LOAD_ERROR)) break;

    DEBUG((EFI_D_WARN, "HSI2C 0x%08x: HSI2CXfer: retrying transfer (%d)\n", HSI2C->Base, Retry));

    MicroSecondDelay(100);
  }

  if (i == Number)
  {
    Status = EFI_SUCCESS;
  }
  else
  {
    Status = RETURN_LOAD_ERROR;
    DEBUG((EFI_D_ERROR, "HSI2C 0x%08x: HSI2CXfer: xfer message failed\n", HSI2C->Base));
  }

  return Status;
}

EFI_STATUS
HSI2CWrite
(
  struct i2c_info *HSI2C,
  UINT32 Address,
  UINT32 Reg,
  UINT32 Length,
  UINT8 *Data
)
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT8 Buffer[256];
  struct i2c_msg Message[] = 
  {
    {
      .Address = Address,
      .Flags = I2C_M_WR,
      .Length = Length + 1,
      .Buffer = Buffer
    }
  };

  Buffer[0] = Reg & 0xFF;
  CopyMem(Buffer + 1, Data, Length);

  DEBUG((EFI_D_WARN, "HSI2C %08x: HSI2CWrite: msg[0]: addr=0x%x, flags=%u, len=%u, buf=0x%x\n",
         HSI2C->Base, Message[0].Address, Message[0].Flags, Message[0].Length, Message[0].Buffer[0]));

  Status = HSI2CXfer(HSI2C, Message, 1);
  return Status;
}

EFI_STATUS
HSI2CRead
(
  struct i2c_info *HSI2C,
  UINT32 Address,
  UINT32 Reg,
  UINT32 Length,
  UINT8 *Data
)
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT8 Buffer;
  struct i2c_msg Message[] = 
  {
    {
      .Address = Address,
      .Flags = I2C_M_WR,
      .Length = 1,
      .Buffer = &Buffer
    },
    {
      .Address = Address,
      .Flags = I2C_M_RD,
      .Length = Length,
      .Buffer = Data
    }
  };

  Buffer = Reg & 0xFF;

  DEBUG((EFI_D_WARN, "HSI2C %08x: HSI2CWrite: msg[0]: addr=0x%x, flags=%u, len=%u, buf=0x%x\n",
         HSI2C->Base, Message[0].Address, Message[0].Flags, Message[0].Length, Message[0].Buffer[0]));

  DEBUG((EFI_D_WARN, "HSI2C %08x: HSI2CWrite: msg[1]: addr=0x%x, flags=%u, len=%u\n",
         HSI2C->Base, Message[1].Address, Message[1].Flags, Message[1].Length));

  Status = HSI2CXfer(HSI2C, Message, 2);
  return Status;
}

VOID
HSI2CInitialisation (struct i2c_info *HSI2C)
{
  DEBUG((EFI_D_WARN, "HSI2C: Initialising HSI2C Channel %d\n", HSI2C->Channel));

  if(HSI2C->Channel < 0 || (MAX_HSI2C_CHAN && HSI2C->Channel >= MAX_HSI2C_CHAN))
  {
    DEBUG((EFI_D_ERROR, "Wrong HSI2C Channel Number %d\n", HSI2C->Channel));
    return;
  }

    ChannelInfo = match_hsi2c_chan_info();
    if(!ChannelInfo) DEBUG((EFI_D_ERROR, "HSI2C: Cannot get register info\n"));

    HSI2C->Base = ChannelInfo[HSI2C->Channel].HSI2CAddress;
    DEBUG((EFI_D_WARN, "HSI2C: Address: 0x%08x", HSI2C->Base));

    HSI2CGpioSet(HSI2C->Channel);
    HSI2CUsiSwConf(HSI2C->Channel);

    MmioWrite32(HSI2C->Base + USI_CON, USI_RESET);

    HSI2CResetI2C(HSI2C);

    HSI2C->InitialisationDone = 1;

    DEBUG((EFI_D_WARN, "HSI2C: HSI2C Channel %d has been initialised\n", HSI2C->Channel));
}

/*STATIC EFI_EXYNOS_I2C_PROTOCOL mExynosHSI2C = {
  
};*/

EFI_STATUS
EFIAPI
InitHSI2CDriver (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable)
{
  EFI_STATUS Status;

  Status = gBS->LocateProtocol (&gEfiExynosGpioProtocolGuid, NULL, (VOID *)&mExynosGpio);

  fp_32_64_div_32_32(&ms_per_cntpct, 1000, 26000000);

  struct i2c_info HSI2C;

  HSI2C.Channel = 0;

  HSI2CInitialisation(&HSI2C);

  while(1){}

  // Register Exynos HSI2C Protocol
  /*Status = gBS->InstallMultipleProtocolInterfaces (&ImageHandle, &gEfiExynosHSI2CProtocolGuid, &mExynosHSI2C, NULL);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Failed to Register Exynos HSI2C Protocol!\n"));
    ASSERT_EFI_ERROR (Status);
  }*/

  return EFI_SUCCESS;
}
