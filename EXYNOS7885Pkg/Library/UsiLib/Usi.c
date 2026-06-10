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
  // USI Address, Controller Address, HSI2C Bus Number, SPI Bus Number, UART Bus Number

  // SYSREG SHUB
  {0x11013000, 0x110C0000,  0,  0,  1},
  {0x11013004, 0x110D0000,  1, -1, -1},

  // SYSREG CMGP
  {0x11C12000, 0x11D00000,  2,  1,  2},
  {0x11C12004, 0x11D10000,  3, -1, -1},
  {0x11C12010, 0x11D20000,  4,  2,  3},
  {0x11C12014, 0x11D30000,  5, -1, -1},
  {0x11C12020, 0x11D40000,  6,  3,  4},
  {0x11C12024, 0x11D50000,  7, -1, -1},
  {0x11C12030, 0x11D60000,  8,  4,  5},
  {0x11C12034, 0x11D70000,  9, -1, -1},
  {0x11C12040, 0x11D80000, 10,  5,  6},
  {0x11C12044, 0x11D90000, 11, -1, -1},

  // SYSREG PERI
  {0x10011010, 0x13820000, -1, -1,  0},
  {0x10011020, 0x138A0000, 12, -1, -1},
  {0x10011024, 0x138B0000, 13, -1, -1},
  {0x10011028, 0x138C0000, 14, -1, -1},
  {0x1001102C, 0x138D0000, 15, -1, -1},
  {0x10011030, 0x13900000, -1,  6, -1},
  {0x10011034, 0x13910000, -1,  7, -1},
  {0x10011038, 0x13940000, -1,  9, -1},
  {0x1001103C, 0x13920000, 16,  8,  7},
  {0x10011040, 0x13930000, 17, -1, -1}
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