/*
 * Samsung Exynos Clock Driver
 * Copyright (C) 2025-2026 viZPilot.
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

#include <Library/ExynosClockLib.h>

#include <Protocol/EFIGpio.h>

#include <Protocol/ExynosClockProtocol.h>

STATIC EFI_GPIO_PROTOCOL *mGpioProtocol;

#define EXYNOS_CLKOUT_NR_CLKS		1
#define EXYNOS_CLKOUT_PARENTS		32

#define EXYNOS_PMU_DEBUG_REG		0xa00
#define EXYNOS_CLKOUT_DISABLE_SHIFT	0
#define EXYNOS_CLKOUT_MUX_SHIFT		8
#define EXYNOS4_CLKOUT_MUX_MASK		0xf
#define EXYNOS5_CLKOUT_MUX_MASK		0x1f



/**
  Initialize the state information for the ExynosClockDxe

  @param  ImageHandle   of the loaded driver
  @param  SystemTable   Pointer to the System Table

  @retval EFI_SUCCESS           Protocol registered
**/
EFI_STATUS
EFIAPI
ExynosClockDxeInitialize (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable)
{
	EFI_STATUS Status;

	DEBUG((EFI_D_INFO, "[ExynosClockDxe]: Initializing Exynos Clock Driver\n"));
	ExynosClockInit();

	DEBUG((EFI_D_INFO, "[ExynosClockDxe]: Locating Exynos GPIO Protocol\n"));

	Status = gBS->LocateProtocol (&gEfiGpioProtocolGuid, NULL, (VOID *)&mGpioProtocol);
	if (EFI_ERROR (Status)) {
		DEBUG ((EFI_D_ERROR, "Failed to Locate Exynos GPIO Protocol! Status = %r\n", Status));
		return Status;
	}

	return EFI_SUCCESS;
}