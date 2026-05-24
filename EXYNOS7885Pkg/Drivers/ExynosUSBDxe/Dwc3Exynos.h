#ifndef __DWC3_EXYNOS_H__
#define __DWC3_EXYNOS_H__

#include <Uefi.h>

#include <Library/UefiLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/IoLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/TimerLib.h>
#include <Library/BaseMemoryLib.h>

#define EXYNOS_USB3_BASE        0x12000000
#define EXYNOS_USB3_PHY_BASE    0x12010000
#define EXYNOS_USB3_CLK_BASE    0x12020000
#define EXYNOS_USB3_RST_BASE    0x12030000

#define USB_CLK_ENABLE_REG      0x0000
#define USB_RESET_REG           0x0004

#define DWC3_GCTL               0xC110
#define DWC3_GUSB3PIPECTL0      0xC2C0
#define DWC3_GUSB2PHYCFG0       0xC200

#define DWC3_GCTL_CORESOFTRESET BIT11

typedef struct {
    UINTN Dwc3Base;
    UINTN PhyBase;
    UINTN ClkBase;
    UINTN RstBase;

    BOOLEAN Initialized;
} DWC3_EXYNOS_DEV;

EFI_STATUS
ExynosUsbClockEnable (
    IN DWC3_EXYNOS_DEV *Dev
    );

EFI_STATUS
ExynosUsbReset (
    IN DWC3_EXYNOS_DEV *Dev
    );

EFI_STATUS
ExynosUsbPhyInit (
    IN DWC3_EXYNOS_DEV *Dev
    );

EFI_STATUS
Dwc3CoreInit (
    IN DWC3_EXYNOS_DEV *Dev
    );

#endif