#ifndef __EXYNOS_CLOCK_LIB_H__
#define __EXYNOS_CLOCK_LIB_H__

#include <Uefi.h>

#define CMU_PERI_BASE   0x12060000

#define CLK_GATE_MMC0   (CMU_PERI_BASE + 0x0104)
#define CLK_DIV_MMC0    (CMU_PERI_BASE + 0x00FB)

typedef enum {
    CLK_UART0,
    CLK_MMC0,
    CLK_I2C0,
    CLK_MAX
} EXYNOS_CLOCK_ID;

EFI_STATUS
ExynosClockInit (
    VOID
    );

EFI_STATUS
ExynosClockEnable (
    IN EXYNOS_CLOCK_ID ClockId
    );

#endif
