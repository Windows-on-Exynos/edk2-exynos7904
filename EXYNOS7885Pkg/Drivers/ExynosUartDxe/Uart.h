/**
  Copyright (c) 2017 Samsung Electronics Co., Ltd.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License version 2 as
  published by the Free Software Foundation.
**/

#ifndef _UART_H_
#define _UART_H_

//
// UART MMIO
//
#define UART_MMIO_LENGTH                           0x1000

// UART Register Offsets
#define ULCON     0x00  // Line control
#define UCON      0x04  // Control
#define UFCON     0x08  // FIFO control
#define UMCON     0x0C  // Modem control
#define UTRSTAT   0x10  // TX/RX status
#define UERSTAT   0x14  // Error status
#define UFSTAT    0x18  // FIFO status
#define UMSTAT    0x1C  // Modem status
#define UTXH      0x20  // TX holding
#define URXH      0x24  // RX holding
#define UBRDIV    0x28  // Baud rate divisor
#define UFRACVAL  0x2C  // Fractional divider
#define UINTP     0x30  // Interrupt pending
#define UINTSP    0x34  // Interrupt source pending
#define UINTM     0x38  // Interrupt mask

// USI Registers (offsets from UART base)
#define USI_CON    0xC4  // USI config

#define ULCON_8N1  0x3   // 8 data bits, 1 stop bit, no parity

#define UCON_DEFAULT  0x3005  // PCLK source, TX/RX level + timeout + error IRQs

#define UFCON_FIFO_ENABLE  0x1

#define UTRSTAT_RX_READY  BIT0
#define UTRSTAT_TX_EMPTY  BIT1

#define UART_DEFAULT_BAUD  115200

#define USI_RESET      (0U << 0)
#define USI_SET_RESET  (1U << 0)

#endif /* _UART_H_ */