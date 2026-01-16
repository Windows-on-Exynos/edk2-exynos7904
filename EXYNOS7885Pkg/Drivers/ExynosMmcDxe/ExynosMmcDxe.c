/*
 * Copyright (C) 2026 viZPilot.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <Uefi.h>

#include <Library/BaseLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/TimerLib.h>

#define MMC0_BASE  0x13500000

#define DW_MMC_CTRL    0x000
#define DW_MMC_PWREN   0x004
#define DW_MMC_CLKDIV  0x008
#define DW_MMC_CLKENA  0x010
#define DW_MMC_STATUS  0x048

STATIC
VOID
DwMmcEarlyInit (VOID)
{
    // Power
    MmioWrite32(MMC0_BASE + DW_MMC_PWREN, 1);

    // Clock interno
    MmioWrite32(MMC0_BASE + DW_MMC_CLKENA, 1);

    // Div = 0
    MmioWrite32(MMC0_BASE + DW_MMC_CLKDIV, 0);

    // Reset
    MmioWrite32(MMC0_BASE + DW_MMC_CTRL, 0x7);
}

STATIC
EFI_STATUS
DwMmcSendCmd (
    UINT32 Cmd,
    UINT32 Arg
    )
{
    MmioWrite32(MMC0_BASE + 0x02C, Cmd);
    MmioWrite32(MMC0_BASE + 0x028, Arg);

    // esperar listo...
    return EFI_SUCCESS;
}

/**
  Initialize the state information for the ExynosMmcDxe

  @param  ImageHandle   of the loaded driver
  @param  SystemTable   Pointer to the System Table

  @retval EFI_SUCCESS           Protocol registered
**/
EFI_STATUS
EFIAPI
ExynosMmcDxeInitialize (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable)
{
	DEBUG((EFI_D_INFO, "[ExynosMmcDxe]: Initializing eMMC Driver\n"));
	DwMmcEarlyInit();
	DwMmcSendCmd(0, 0);      // GO_IDLE
	DwMmcSendCmd(1, 0x40FF); // INIT
	return EFI_SUCCESS;
}