#ifndef __EXYNOS_ADC_H
#define __EXYNOS_ADC_H

#define MAX_CHANNEL		11
#define ADC_CON1		(0x0)
#define ADC_CON2		(0x4)
#define ADC_DAT			(0x8)
#define ADC_SUM_DAT		(0xC)
#define ADC_INT_EN		(0x10)
#define ADC_INT_STATUS		(0x14)
#define ADC_DEBUG_DATA		(0x1C)
#define ADC_BASE_ADDR		(0x11C30000)

#define ADC_SOFT_RESET		(1u << 2)
#define ADC_NON_SOFT_RESET	(1u << 1)
#define ADC_CON2_C_TIME(x)	(((x) & 7) << 4)
#define ADC_CON2_ACH_MASK	(0xf)
#define ADC_CON2_ACH_SEL(x)	(((x) & 0xF) << 0)
#define ADC_CON_EN_START	(1u << 0)
#define ADC_DAT_MASK		(0xFFF)
#define EN_INT			(1)

void exynos_adc_init_hw(void);
void exynos_adc_exit_hw(void);
void exynos_adc_start_conv(UINT32 chan);
int exynos_adc_read_raw(UINT32 chan);

#endif