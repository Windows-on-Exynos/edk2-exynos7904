#include <Library/I2cLib.h>

STATIC
EFI_I2C_BUS_DATA
gI2cBusData[] = {
  {
    .BaseAddress = 0x13830000,
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_P,
      .SdaBankId  = BANK_ID_P,
      .SclBankNum = 0,
      .SdaBankNum = 0,
      .SclPin     = 1,
      .SdaPin     = 0
    },
    .Clk = 0
  },
  {
    .BaseAddress = 0x13840000,
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_P,
      .SdaBankId  = BANK_ID_P,
      .SclBankNum = 0,
      .SdaBankNum = 0,
      .SclPin     = 3,
      .SdaPin     = 2
    },
    .Clk = 0
  },
  {
    .BaseAddress = 0x13850000,
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_P,
      .SdaBankId  = BANK_ID_P,
      .SclBankNum = 0,
      .SdaBankNum = 0,
      .SclPin     = 5,
      .SdaPin     = 4
    },
    .Clk = 0
  },
  {
    .BaseAddress = 0x13860000,
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_P,
      .SdaBankId  = BANK_ID_P,
      .SclBankNum = 0,
      .SdaBankNum = 0,
      .SclPin     = 7,
      .SdaPin     = 6
    },
    .Clk = 0
  },
  {
    .BaseAddress = 0x13870000,
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_P,
      .SdaBankId  = BANK_ID_P,
      .SclBankNum = 1,
      .SdaBankNum = 1,
      .SclPin     = 1,
      .SdaPin     = 0
    },
    .Clk = 0
  },
  {
    .BaseAddress = 0x13880000,
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_P,
      .SdaBankId  = BANK_ID_P,
      .SclBankNum = 1,
      .SdaBankNum = 1,
      .SclPin     = 3,
      .SdaPin     = 2
    },
    .Clk = 0
  },
  {
    .BaseAddress = 0x13890000,
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_P,
      .SdaBankId  = BANK_ID_P,
      .SclBankNum = 1,
      .SdaBankNum = 1,
      .SclPin     = 5,
      .SdaPin     = 4
    },
    .Clk = 0
  },
};

VOID
GetI2cBusData (
  OUT EFI_I2C_BUS_DATA **Data,
  OUT UINT8             *Count)
{
  *Data  = gI2cBusData;
  *Count = ARRAY_SIZE (gI2cBusData);
}
