/**
  Copyright (c) 2017 Samsung Electronics Co., Ltd.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License version 2 as
  published by the Free Software Foundation.
**/

#include <Library/DebugLib.h>
#include <Library/MemoryAllocationHelperLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/TimerLib.h>
#include <Library/IoLib.h>
#include <Library/ExynosUartLib.h>

#include <Protocol/EFIUsi.h>
#include <Protocol/EFIGpio.h>
#include <Protocol/SerialIo.h>

#include "Uart.h"

#define UART_SIGNATURE  SIGNATURE_32 ('U','A','R','T')

typedef struct {
  UINT32                   Signature;
  EFI_HANDLE               Handle;
  EFI_SERIAL_IO_PROTOCOL   SerialIo;
  EFI_PHYSICAL_ADDRESS     Base;
  UINT32                   Clock;
  UINT32                   BaudRate;
  EFI_STOP_BITS_TYPE       StopBits;
  EFI_PARITY_TYPE          Parity;
  UINT8                    DataBits;
} UART_INSTANCE;

STATIC EFI_USI_PROTOCOL   *mUsi;
STATIC EFI_GPIO_PROTOCOL  *mGpio;
STATIC UART_INSTANCE      *gUartInstances;
STATIC UINT8               gNumUarts;

STATIC
VOID
UartWaitForTxEmpty (
  IN EFI_PHYSICAL_ADDRESS Base
  )
{
  while (!(MmioRead32 (Base + UTRSTAT) & UTRSTAT_TX_EMPTY));
}

STATIC
VOID
UartWaitForRxReady (
  IN EFI_PHYSICAL_ADDRESS Base
  )
{
  while (!(MmioRead32 (Base + UTRSTAT) & UTRSTAT_RX_READY));
}

STATIC
VOID
UartPutChar (
  IN EFI_PHYSICAL_ADDRESS Base,
  IN CHAR8                Char
  )
{
  UartWaitForTxEmpty (Base);
  MmioWrite32 (Base + UTXH, (UINT32)Char);
}

STATIC
CHAR8
UartGetChar (
  IN EFI_PHYSICAL_ADDRESS Base
  )
{
  UartWaitForRxReady (Base);
  return (CHAR8)(MmioRead32 (Base + URXH) & 0xFF);
}

STATIC
EFI_STATUS
UartConfigurePins (
  IN EFI_UART_BUS_DATA *Bus
  )
{
  EFI_STATUS Status;

  Status = mGpio->SetPinFunction (Bus->Gpio.Tx.BankId, Bus->Gpio.Tx.BankNum,
                                  Bus->Gpio.Tx.Pin, Bus->Gpio.Tx.Function);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "UART: TX SetPinFunction failed: %r\n", Status));
    return Status;
  }
  Status = mGpio->SetPinPull (Bus->Gpio.Tx.BankId, Bus->Gpio.Tx.BankNum,
                              Bus->Gpio.Tx.Pin, Bus->Gpio.Tx.Pull);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "UART: TX SetPinPull failed: %r\n", Status));
    return Status;
  }

  Status = mGpio->SetPinFunction (Bus->Gpio.Rx.BankId, Bus->Gpio.Rx.BankNum,
                                  Bus->Gpio.Rx.Pin, Bus->Gpio.Rx.Function);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "UART: RX SetPinFunction failed: %r\n", Status));
    return Status;
  }
  Status = mGpio->SetPinPull (Bus->Gpio.Rx.BankId, Bus->Gpio.Rx.BankNum,
                              Bus->Gpio.Rx.Pin, Bus->Gpio.Rx.Pull);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "UART: RX SetPinPull failed: %r\n", Status));
    return Status;
  }

  if (Bus->Gpio.Rts.BankId != 0) {
    Status = mGpio->SetPinFunction (Bus->Gpio.Rts.BankId, Bus->Gpio.Rts.BankNum,
                                    Bus->Gpio.Rts.Pin, Bus->Gpio.Rts.Function);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "UART: RTS SetPinFunction failed: %r\n", Status));
      return Status;
    }
    Status = mGpio->SetPinPull (Bus->Gpio.Rts.BankId, Bus->Gpio.Rts.BankNum,
                                Bus->Gpio.Rts.Pin, Bus->Gpio.Rts.Pull);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "UART: RTS SetPinPull failed: %r\n", Status));
      return Status;
    }

    Status = mGpio->SetPinFunction (Bus->Gpio.Cts.BankId, Bus->Gpio.Cts.BankNum,
                                    Bus->Gpio.Cts.Pin, Bus->Gpio.Cts.Function);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "UART: CTS SetPinFunction failed: %r\n", Status));
      return Status;
    }
    Status = mGpio->SetPinPull (Bus->Gpio.Cts.BankId, Bus->Gpio.Cts.BankNum,
                                Bus->Gpio.Cts.Pin, Bus->Gpio.Cts.Pull);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "UART: CTS SetPinPull failed: %r\n", Status));
      return Status;
    }
  }

  return EFI_SUCCESS;
}

//
// UART hardware init
//
STATIC
EFI_STATUS
UartHardwareInit (
  IN UINT32                Clock,
  IN UINT32                BaudRate,
  IN EFI_PHYSICAL_ADDRESS  Base
  )
{
  UINT32 Div, Rem;

  // Reset USI, then set UART mode (USI_CON bit 0 = 0)
  MmioWrite32 (Base + USI_CON, USI_SET_RESET);
  gBS->Stall (1);
  MmioWrite32 (Base + USI_CON, USI_RESET);
  gBS->Stall (1);

  // 8N1, PCLK source, TX/RX level IRQ, FIFO enable
  MmioWrite32 (Base + ULCON,   ULCON_8N1);
  MmioWrite32 (Base + UCON,    UCON_DEFAULT);
  MmioWrite32 (Base + UFCON,   UFCON_FIFO_ENABLE);
  MmioWrite32 (Base + UMCON,   0x0);
  MmioWrite32 (Base + UTRSTAT, 0x6);

  // Baud rate
  Div = Clock / (16 * BaudRate);
  Rem = Clock % (16 * BaudRate);
  MmioWrite32 (Base + UBRDIV,   Div - 1);
  MmioWrite32 (Base + UFRACVAL, (Rem * 16) / (16 * BaudRate));

  // Mask all interrupts (polling mode)
  MmioWrite32 (Base + UINTM, 0xF);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
UartReset (
  IN EFI_SERIAL_IO_PROTOCOL *This
  )
{
  UART_INSTANCE *Instance;

  Instance = CR (This, UART_INSTANCE, SerialIo, UART_SIGNATURE);

  MmioWrite32 (Instance->Base + UFCON, 0x0);
  gBS->Stall (1);
  MmioWrite32 (Instance->Base + UFCON, UFCON_FIFO_ENABLE);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
UartSetAttributes (
  IN EFI_SERIAL_IO_PROTOCOL *This,
  IN UINT64                  BaudRate,
  IN UINT32                  ReceiveFifoDepth,
  IN UINT32                  Timeout,
  IN EFI_PARITY_TYPE         Parity,
  IN UINT8                   DataBits,
  IN EFI_STOP_BITS_TYPE      StopBits
  )
{
  UART_INSTANCE *Instance;

  Instance = CR (This, UART_INSTANCE, SerialIo, UART_SIGNATURE);

  if (Parity != DefaultParity || DataBits != 8 || StopBits != DefaultStopBits) {
    return EFI_UNSUPPORTED;
  }

  UartHardwareInit (Instance->Clock, (UINT32)BaudRate, Instance->Base);

  Instance->BaudRate = (UINT32)BaudRate;
  Instance->StopBits = StopBits;
  Instance->Parity   = Parity;
  Instance->DataBits = DataBits;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
UartSetControl (
  IN EFI_SERIAL_IO_PROTOCOL *This,
  IN UINT32                  Control
  )
{
  return EFI_UNSUPPORTED;
}

STATIC
EFI_STATUS
EFIAPI
UartGetControl (
  IN  EFI_SERIAL_IO_PROTOCOL *This,
  OUT UINT32                 *Control
  )
{
  if (Control == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  *Control = 0;
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
UartWrite (
  IN EFI_SERIAL_IO_PROTOCOL *This,
  IN OUT UINTN              *BufferSize,
  IN VOID                   *Buffer
  )
{
  UART_INSTANCE *Instance;
  CHAR8         *Data;
  UINTN          Count;

  if (Buffer == NULL || BufferSize == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  Instance = CR (This, UART_INSTANCE, SerialIo, UART_SIGNATURE);
  Data     = (CHAR8 *)Buffer;
  Count    = *BufferSize;

  while (Count--) {
    if (*Data == '\n') {
      UartPutChar (Instance->Base, '\r');
      UartPutChar (Instance->Base, '\n');
    } else {
      UartPutChar (Instance->Base, *Data);
    }
    Data++;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
UartRead (
  IN EFI_SERIAL_IO_PROTOCOL *This,
  IN OUT UINTN              *BufferSize,
  OUT VOID                  *Buffer
  )
{
  UART_INSTANCE *Instance;
  CHAR8         *Data;
  UINTN          Count;

  if (Buffer == NULL || BufferSize == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  Instance = CR (This, UART_INSTANCE, SerialIo, UART_SIGNATURE);
  Data     = (CHAR8 *)Buffer;
  Count    = *BufferSize;

  *BufferSize = 0;

  while (Count--) {
    *Data++ = UartGetChar (Instance->Base);
    (*BufferSize)++;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
InitializeUart (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS           Status;
  EFI_UART_BUS_DATA   *BusData;
  UINT8                BusCount;
  UINT8                i;

  Status = gBS->LocateProtocol (&gEfiUsiProtocolGuid, NULL, (VOID *)&mUsi);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "UART: Failed to locate USI protocol: %r\n", Status));
    return Status;
  }

  Status = gBS->LocateProtocol (&gEfiGpioProtocolGuid, NULL, (VOID *)&mGpio);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "UART: Failed to locate GPIO protocol: %r\n", Status));
    return Status;
  }

  GetUartBusData (&BusData, &BusCount);

  if (BusCount == 0) {
    DEBUG ((EFI_D_WARN, "UART: No UART buses configured. Skipping.\n"));
    return EFI_SUCCESS;
  }

  gUartInstances = AllocateZeroPool (sizeof (UART_INSTANCE) * BusCount);
  if (gUartInstances == NULL) {
    DEBUG ((EFI_D_ERROR, "UART: Failed to allocate instances\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  gNumUarts = 0;

  for (i = 0; i < BusCount; i++) {
    EFI_UART_BUS_DATA   *Bus = &BusData[i];
    EFI_PHYSICAL_ADDRESS Base;

    Status = mUsi->GetControllerAddr (i, BUS_UART, &Base);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "UART: Port %d not found in USI data: %r\n", i, Status));
      continue;
    }

    // Map MMIO
    Status = MapMemoryRegion (Base, UART_MMIO_LENGTH, EfiMemoryMappedIO);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "UART: Failed to map UART%d at 0x%llx: %r\n",
              i, Base, Status));
      continue;
    }

    // Configure GPIO pins
    Status = UartConfigurePins (Bus);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "UART: Failed to configure pins for UART%d: %r\n",
              i, Status));
      continue;
    }

    // Set USI mode
    Status = mUsi->SetMode (Base, MODE_UART);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "UART: SetMode for UART%d at 0x%llx: %r\n",
              i, Base, Status));
    }

    // Initialize hardware
    Status = UartHardwareInit (GetUartClock (), UART_DEFAULT_BAUD, Base);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "UART: Failed to init UART%d: %r\n", i, Status));
      continue;
    }

    UART_INSTANCE *Instance = &gUartInstances[gNumUarts];

    Instance->Signature = UART_SIGNATURE;
    Instance->Clock    = GetUartClock ();
    Instance->BaudRate = UART_DEFAULT_BAUD;
    Instance->SerialIo.Revision      = EFI_SERIAL_IO_PROTOCOL_REVISION;
    Instance->SerialIo.Reset         = UartReset;
    Instance->SerialIo.SetAttributes = UartSetAttributes;
    Instance->SerialIo.SetControl    = UartSetControl;
    Instance->SerialIo.GetControl    = UartGetControl;
    Instance->SerialIo.Write         = UartWrite;
    Instance->SerialIo.Read          = UartRead;

    Instance->Base     = Base;
    Instance->StopBits = DefaultStopBits;
    Instance->Parity   = DefaultParity;
    Instance->DataBits = 8;

    Status = gBS->InstallProtocolInterface (
                    &Instance->Handle,
                    &gEfiSerialIoProtocolGuid,
                    EFI_NATIVE_INTERFACE,
                    &Instance->SerialIo
                    );
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "UART: Failed to install SerialIo for UART%d: %r\n",
              i, Status));
      continue;
    }

    DEBUG ((EFI_D_INFO, "UART: Initialized UART%d at 0x%llx, baud=%d\n",
            i, Base, Instance->BaudRate));

    gNumUarts++;
  }

  if (gNumUarts == 0) {
    DEBUG ((EFI_D_ERROR, "UART: No UART ports initialized!\n"));
    FreePool (gUartInstances);
    gUartInstances = NULL;
    return EFI_NOT_FOUND;
  }

  DEBUG ((EFI_D_INFO, "UART: Initialized %d UART port(s)\n", gNumUarts));

  return EFI_SUCCESS;
}