#include "Dwc3Exynos.h"

STATIC
VOID
Dwc3WriteReg (
    IN UINTN Address,
    IN UINT32 Value
    )
{
    MmioWrite32(Address, Value);
}

STATIC
UINT32
Dwc3ReadReg (
    IN UINTN Address
    )
{
    return MmioRead32(Address);
}

EFI_STATUS
ExynosUsbClockEnable (
    IN DWC3_EXYNOS_DEV *Dev
    )
{
    DEBUG((EFI_D_INFO, "DWC3: Enable clocks\n"));

    MmioOr32(
        Dev->ClkBase + USB_CLK_ENABLE_REG,
        BIT0 | BIT1 | BIT2
    );

    MicroSecondDelay(100);

    return EFI_SUCCESS;
}

EFI_STATUS
ExynosUsbPhyInit (
    IN DWC3_EXYNOS_DEV *Dev
    )
{
	DEBUG((EFI_D_INFO, "DWC3: USB Phy init\n"));
	return EFI_SUCCESS;
}

EFI_STATUS
ExynosUsbReset (
    IN DWC3_EXYNOS_DEV *Dev
    )
{
    DEBUG((EFI_D_INFO, "DWC3: Reset controller\n"));

    MmioOr32(
        Dev->RstBase + USB_RESET_REG,
        BIT0
    );

    MicroSecondDelay(10000);

    MmioAnd32(
        Dev->RstBase + USB_RESET_REG,
        ~BIT0
    );

    MicroSecondDelay(10000);

    return EFI_SUCCESS;
}

EFI_STATUS
Dwc3ExynosEntryPoint (
	IN EFI_HANDLE        ImageHandle,
	IN EFI_SYSTEM_TABLE *SystemTable)
{
    DEBUG((EFI_D_INFO, "DWC3: USB init\n"));

	return EFI_SUCCESS;
}
