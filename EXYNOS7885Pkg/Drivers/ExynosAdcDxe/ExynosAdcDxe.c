#include <Uefi.h>

#include <Library/BaseLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/DebugLib.h>
#include <Library/TimerLib.h>
#include <Library/PcdLib.h>
#include <Library/IoLib.h>
#include <Library/BaseMemoryLib.h>

#include "adc.h"

void exynos_adc_init_hw(void)
{
	UINT32 con1, con2;

	con1 = ADC_SOFT_RESET;
	MmioWrite32(con1, ADC_BASE_ADDR + ADC_CON1);

	con1 = ADC_NON_SOFT_RESET;
	MmioWrite32(con1, ADC_BASE_ADDR + ADC_CON1);

	con2 = ADC_CON2_C_TIME(6);
	MmioWrite32(con2, ADC_BASE_ADDR + ADC_CON2);

	MmioWrite32(EN_INT, ADC_BASE_ADDR + ADC_INT_EN);
}

void exynos_adc_exit_hw(void)
{
	UINT32 con2;

	con2 = MmioRead32(ADC_BASE_ADDR + ADC_CON2);
	con2 &= ~ADC_CON2_C_TIME(7);
	MmioWrite32(con2, ADC_BASE_ADDR + ADC_CON2);

	MmioWrite32(!EN_INT, ADC_BASE_ADDR + ADC_INT_EN);
}

void exynos_adc_start_conv(UINT32 chan)
{
	UINT32 con1, con2;

	con2 = MmioRead32(ADC_BASE_ADDR + ADC_CON2);
	con2 &= ~ADC_CON2_ACH_MASK;
	con2 |= ADC_CON2_ACH_SEL(chan);
	MmioWrite32(con2, ADC_BASE_ADDR + ADC_CON2);

	con1 = MmioRead32(ADC_BASE_ADDR + ADC_CON1);
	MmioWrite32(con1 | ADC_CON_EN_START, ADC_BASE_ADDR + ADC_CON1);
}

int exynos_adc_read_raw(UINT32 chan)
{
	int val = -1;

	exynos_adc_init_hw();
	exynos_adc_start_conv(chan);

	MicroSecondDelay(100);

	val = MmioRead32(ADC_BASE_ADDR + ADC_DAT) & ADC_DAT_MASK;

	exynos_adc_exit_hw();
	return val;
}

EFI_STATUS
EFIAPI
ExynosAdcDxeInitialize(
	IN EFI_HANDLE         ImageHandle,
	IN EFI_SYSTEM_TABLE   *SystemTable
)
{
	EFI_STATUS  Status = EFI_SUCCESS;

	exynos_adc_init_hw();

	DEBUG((EFI_D_ERROR, "ADC initialised!"));

	ASSERT_EFI_ERROR(Status);

	return Status;
}