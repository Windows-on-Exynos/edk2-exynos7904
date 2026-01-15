#include <Library/IoLib.h>
#include <Library/DebugLib.h>

#include <Library/ExynosClockLib.h>

EFI_STATUS
ExynosEnableEmmcClock (
    VOID
    )
{
    UINT32 Val;

    DEBUG ((EFI_D_INFO, "[ExynosClockDxe]: Enabling eMMC clock...\n"));

    DEBUG ((EFI_D_INFO, "[ExynosClockDxe]: Setting Divisor clock...\n"));
    // 1. Configurar divisor
    Val = MmioRead32(CLK_DIV_MMC0);
    Val &= ~0xFF;          // limpia divisor
    Val |= 0x10;           // valor conservador
    MmioWrite32(CLK_DIV_MMC0, Val);

    // 2. Habilitar gate
    Val = MmioRead32(CLK_GATE_MMC0);
    Val |= (1 << 0);
    MmioWrite32(CLK_GATE_MMC0, Val);

    return EFI_SUCCESS;
}

EFI_STATUS
ExynosClockInit (
    VOID
    )
{
    DEBUG ((EFI_D_INFO, "[ExynosClockDxe]: Initializing specific clocks\n"));
	ExynosClockEnable(2);
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
		ExynosEnableEmmcClock();
        DEBUG((EFI_D_INFO, "[ExynosClockDxe]: MMC clock enabled"));
        break;

    default:
        return EFI_UNSUPPORTED;
    }

    return EFI_SUCCESS;
}
