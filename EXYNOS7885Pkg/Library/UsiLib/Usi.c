/**
  Copyright (c) 2019 Samsung Electronics Co., Ltd.

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License version 2 as
  published by the Free Software Foundation.
**/

#include <Library/UsiLib.h>

STATIC
EFI_USI_DATA
gUsiData[] = {
  //
  // SYSREG SHUB
  //
  {
    .UsiAddress        = 0x11013000,
    .ControllerAddress = 0x110C0000,
    .I2cBus            = 0,
    .SpiBus            = 0,
    .UartBus           = 1
  },
  {
    .UsiAddress        = 0x11013004,
    .ControllerAddress = 0x110D0000,
    .I2cBus            = 1,
    .SpiBus            = MAX_UINT8,
    .UartBus           = MAX_UINT8
  },

  //
  // SYSREG CMGP
  //
  {
    .UsiAddress        = 0x11C12000,
    .ControllerAddress = 0x11D00000,
    .I2cBus            = 2,
    .SpiBus            = 1,
    .UartBus           = 2
  },
  {
    .UsiAddress        = 0x11C12004,
    .ControllerAddress = 0x11D10000,
    .I2cBus            = 3,
    .SpiBus            = MAX_UINT8,
    .UartBus           = MAX_UINT8
  },
  {
    .UsiAddress        = 0x11C12010,
    .ControllerAddress = 0x11D20000,
    .I2cBus            = 4,
    .SpiBus            = 2,
    .UartBus           = 3
  },
  {
    .UsiAddress        = 0x11C12014,
    .ControllerAddress = 0x11D30000,
    .I2cBus            = 5,
    .SpiBus            = MAX_UINT8,
    .UartBus           = MAX_UINT8
  },
  {
    .UsiAddress        = 0x11C12020,
    .ControllerAddress = 0x11D40000,
    .I2cBus            = 6,
    .SpiBus            = 3,
    .UartBus           = 4
  },
  {
    .UsiAddress        = 0x11C12024,
    .ControllerAddress = 0x11D50000,
    .I2cBus            = 7,
    .SpiBus            = MAX_UINT8,
    .UartBus           = MAX_UINT8
  },
  {
    .UsiAddress        = 0x11C12030,
    .ControllerAddress = 0x11D60000,
    .I2cBus            = 8,
    .SpiBus            = 4,
    .UartBus           = 5
  },
  {
    .UsiAddress        = 0x11C12034,
    .ControllerAddress = 0x11D70000,
    .I2cBus            = 9,
    .SpiBus            = MAX_UINT8,
    .UartBus           = MAX_UINT8
  },
  {
    .UsiAddress        = 0x11C12040,
    .ControllerAddress = 0x11D80000,
    .I2cBus            = 10,
    .SpiBus            = 5,
    .UartBus           = 6
  },
  {
    .UsiAddress        = 0x11C12044,
    .ControllerAddress = 0x11D90000,
    .I2cBus            = 11,
    .SpiBus            = MAX_UINT8,
    .UartBus           = MAX_UINT8
  },

  //
  // SYSREG PERI
  //
  {
    .UsiAddress        = 0x10011010,
    .ControllerAddress = 0x13820000,
    .I2cBus            = MAX_UINT8,
    .SpiBus            = MAX_UINT8,
    .UartBus           = 0
  },
  {
    .UsiAddress        = 0x10011020,
    .ControllerAddress = 0x138A0000,
    .I2cBus            = 12,
    .SpiBus            = MAX_UINT8,
    .UartBus           = MAX_UINT8
  },
  {
    .UsiAddress        = 0x10011024,
    .ControllerAddress = 0x138B0000,
    .I2cBus            = 13,
    .SpiBus            = MAX_UINT8,
    .UartBus           = MAX_UINT8
  },
  {
    .UsiAddress        = 0x10011028,
    .ControllerAddress = 0x138C0000,
    .I2cBus            = 14,
    .SpiBus            = MAX_UINT8,
    .UartBus           = MAX_UINT8
  },
  {
    .UsiAddress        = 0x1001102C,
    .ControllerAddress = 0x138D0000,
    .I2cBus            = 15,
    .SpiBus            = MAX_UINT8,
    .UartBus           = MAX_UINT8
  },
  {
    .UsiAddress        = 0x10011030,
    .ControllerAddress = 0x13900000,
    .I2cBus            = MAX_UINT8,
    .SpiBus            = 6,
    .UartBus           = MAX_UINT8
  },
  {
    .UsiAddress        = 0x10011034,
    .ControllerAddress = 0x13910000,
    .I2cBus            = MAX_UINT8,
    .SpiBus            = 7,
    .UartBus           = MAX_UINT8
  },
  {
    .UsiAddress        = 0x10011038,
    .ControllerAddress = 0x13940000,
    .I2cBus            = MAX_UINT8,
    .SpiBus            = 9,
    .UartBus           = MAX_UINT8
  },
  {
    .UsiAddress        = 0x1001103C,
    .ControllerAddress = 0x13920000,
    .I2cBus            = 16,
    .SpiBus            = 8,
    .UartBus           = 7
  },
  {
    .UsiAddress        = 0x10011040,
    .ControllerAddress = 0x13930000,
    .I2cBus            = 17,
    .SpiBus            = MAX_UINT8,
    .UartBus           = MAX_UINT8
  }
};

VOID
GetUsiData (
  OUT EFI_USI_DATA **Data,
  OUT UINT8         *Count)
{
  // Pass Data
  *Data  = gUsiData;
  *Count = ARRAY_SIZE (gUsiData);
}