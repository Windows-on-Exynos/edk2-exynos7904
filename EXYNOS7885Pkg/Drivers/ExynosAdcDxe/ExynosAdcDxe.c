#include <Library/DebugLib.h>
#include <Library/MemoryAllocationHelperLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/IoLib.h>

#include <Protocol/EFIAdc.h>

#include "Adc.h"


STATIC
VOID
ExynosAdcInitHw (VOID)
{
  EFI_ADC_REGISTER *Register = (EFI_ADC_REGISTER *)0x11C30000;

  Register->control_1 = ADC_SOFT_RESET;
  Register->control_1 = ADC_NON_SOFT_RESET;

  Register->control_2 = ADC_CON2_C_TIME(6);

  Register->interrupt_enable = ENABLE_INTERRUPT;
}

STATIC
VOID
ExynosAdcExitHw (VOID)
{
  EFI_ADC_REGISTER *Register = (EFI_ADC_REGISTER *)0x11C30000;

  Register->control_2 &= ~(ADC_CON2_C_TIME(7));
  Register->interrupt_enable = DISABLE_INTERRUPT;
}

STATIC
VOID
ExynosAdcStartConversion (IN UINT32 Channel)
{
  EFI_ADC_REGISTER *Register = (EFI_ADC_REGISTER *)0x11C30000;
  UINT32 control_2;

  control_2 = Register->control_2;
  control_2 &= ~ADC_CON2_ACH_MASK;
  control_2 |= ADC_CON2_ACH_SEL(Channel);

  Register->control_2 = control_2;  
  Register->control_1 |= ADC_CON_EN_START;
}

INTN
ExynosAdcReadRaw (IN UINT32 Channel)
{
  EFI_ADC_REGISTER *Register = (EFI_ADC_REGISTER *)0x11C30000;
  INTN Value = -1;

  if (Channel > MAX_CHANNEL)
    return Value;

  ExynosAdcInitHw();
  ExynosAdcStartConversion(Channel);
  gBS->Stall(100000);

  Value = Register->data & ADC_DAT_MASK;

  ExynosAdcExitHw();
  return Value;
}

STATIC EFI_ADC_PROTOCOL mAdc = {
	ExynosAdcReadRaw
};

EFI_STATUS
EFIAPI
RegisterAdc (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable)
{
  EFI_STATUS Status;

  // Register ADC Protocol
  Status = gBS->InstallProtocolInterface (&ImageHandle, &gEfiAdcProtocolGuid, EFI_NATIVE_INTERFACE, &mAdc);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Failed to Register ADC Protocol!\n"));
    return Status;
  }

  return EFI_SUCCESS;
}