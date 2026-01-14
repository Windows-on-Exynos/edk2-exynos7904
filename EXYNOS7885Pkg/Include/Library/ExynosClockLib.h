#ifndef __EXYNOS_CLOCK_LIB_H__
#define __EXYNOS_CLOCK_LIB_H__

#include <Uefi.h>

#define CMU_PERI_BASE 0x10010000

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
