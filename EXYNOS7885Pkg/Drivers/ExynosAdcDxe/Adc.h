#ifndef _ADC_H_
#define _ADC_H_

#define MAX_CHANNEL		    11

#define ADC_SOFT_RESET		BIT2
#define ADC_NON_SOFT_RESET	BIT1
#define ADC_CON2_C_TIME(x)	((x) & 7) << 4
#define ADC_CON2_ACH_MASK	0xF
#define ADC_CON2_ACH_SEL(x)	((x) & 0xF) << 0
#define ADC_CON_EN_START	BIT0
#define ADC_DAT_MASK		0xFFF
#define ENABLE_INTERRUPT	1
#define DISABLE_INTERRUPT	0


//
// ADC MMIO Register Structure
//
typedef struct {
  UINT32 control_1;
  UINT32 control_2;
  UINT32 data;
  UINT32 sum_data;
  UINT32 interrupt_enable;
  UINT32 interrupt_status;
  UINT32 debug_data;
} EFI_ADC_REGISTER;

#endif /* _ADC_H_ */