#include <Library/ExynosClockLib.h>

#include <Library/IoLib.h>
#include <Library/DebugLib.h>

EFI_STATUS
ExynosClockInit (
    VOID
    )
{
    DEBUG ((EFI_D_INFO, "[ExynosClockDxe]: Initializing specific clocks\n"));
    return EFI_SUCCESS;
}

EFI_STATUS
ExynosClockEnable (
    IN EXYNOS_CLOCK_ID ClockId
    )
{
    DEBUG ((EFI_D_INFO, "[ExynosClockDxe]: Enable clock %d\n", ClockId));

    switch (ClockId) {
    case CLK_UART0:
	
        DEBUG((EFI_D_INFO, "[ExynosClockDxe]: UART clock enabled"));
        break;

    case CLK_MMC0:
	
        DEBUG((EFI_D_INFO, "[ExynosClockDxe]: MMC clock enabled"));
        break;

    default:
        return EFI_UNSUPPORTED;
    }

    return EFI_SUCCESS;
}
