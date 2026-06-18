#ifndef _I2C_LIB_H_
#define _I2C_LIB_H_

#include <Device/Gpio.h>

//
// I2C GPIO Data
//
typedef struct {
  EFI_GPIO_FUNCTION Function;
  EFI_GPIO_BANK_ID  SclBankId;
  EFI_GPIO_BANK_ID  SdaBankId;
  UINT8             SclBankNum;
  UINT8             SdaBankNum;
  UINT8             SclPin;
  UINT8             SdaPin;
} EFI_I2C_GPIO_DATA;

//
// I2C Bus Data
//
typedef struct {
  EFI_PHYSICAL_ADDRESS BaseAddress;
  EFI_I2C_GPIO_DATA    Gpio;
  UINT32               Clk;          // PCLK Hz; 0 = BL2 pre-config
  BOOLEAN              Initialized;
} EFI_I2C_BUS_DATA;

/**
  This Function Returns the Platform I2C Buses.

  @param[out] Data                         - The I2C Data.
  @param[out] Count                        - The Number of I2C Buses.
**/
VOID
GetI2cBusData (
  OUT EFI_I2C_BUS_DATA **Data,
  OUT UINT8             *Count
  );

#endif /* _I2C_LIB_H_ */
