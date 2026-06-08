/*
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
#include <Library/DebugLib.h>
#include <Library/PrintLib.h>
#include <Library/UefiDriverEntryPoint.h>
#include <Library/TimerLib.h>
#include <Library/IoLib.h>

#include "reg.h"

#define GPIO_LEFT_BASE                  	 0x14000000
#define rGPK2CON                             (GPIO_LEFT_BASE + 0x00E0)
#define rGPK2DAT                             (GPIO_LEFT_BASE + 0x00E4)
#define rGPK2PUD                             (GPIO_LEFT_BASE + 0x00E8)
#define rGPK3CON                             (GPIO_LEFT_BASE + 0x0100)
#define rGPK3DAT                             (GPIO_LEFT_BASE + 0x0104)
#define rGPK3PUD                             (GPIO_LEFT_BASE + 0x0108)

#define TEST_PKT_SIZE 53

#define USB_CAP_20_EXT  0x2
#define USB_CAP_SS      0x3
#define USB_CAP_CID     0x4

#define PHY_EXYNOS_USB_PLL_CON0	0x01A0
#define PHY_EXYNOS_USB_PLL_CON1	0x01A4
#define PHY_EXYNOS_USB_PLL_CON2	0x01A8

#define PLL_ENABLE_BIT			0x80000000
#define PLL_LOCKED_BIT			0x20000000
#define PLL_MUX_SEL_BIT		(1<<4)
#define PLL_MUX_BUSY_BIT		(1<<7)

#define MDIV_BIT16			16
#define PDIV_BIT8			8
#define SDIV_BIT0			0



#define USB_INT_NUM	(186 + 32)

#define USBDEVICE3_LINK_CH0_BASE	0x13200000
#define USBDEVICE3_PHYCTRL_CH0_BASE	0x131D0000
#define USB_PHY_CONTROL_BASE		USBDEVICE3_PHYCTRL_CH0_BASE

// TODO: implement USB Functions on this DXE

/**
  Initialize the state information for the ExynosUSBDxe

  @param  ImageHandle   of the loaded driver
  @param  SystemTable   Pointer to the System Table

  @retval EFI_SUCCESS           Protocol registered
**/
EFI_STATUS
EFIAPI
ExynosUsbEntry (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable)
{
	DEBUG ((EFI_D_INFO, "[ExynosUSBDxe]: Initializing Exynos USB Driver\n"));

	return EFI_SUCCESS;
}