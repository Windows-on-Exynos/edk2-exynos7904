#include <Library/HsI2cLib.h>

STATIC
EFI_HSI2C_BUS_DATA
gHsI2cBusData[] = {
  {
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_H,
      .SdaBankId  = BANK_ID_H,
      .SclBankNum = 0,
      .SdaBankNum = 0,
      .SclPin     = 0,
      .SdaPin     = 1
    },
    {
      .Freq      = 200000000,
      .SpeedMode = FAST_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_3,
      .SclBankId  = BANK_ID_H,
      .SdaBankId  = BANK_ID_H,
      .SclBankNum = 0,
      .SdaBankNum = 0,
      .SclPin     = 2,
      .SdaPin     = 3
    },
    {
      .Freq      = 200000000,
      .SpeedMode = FAST_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_M,
      .SdaBankId  = BANK_ID_M,
      .SclBankNum = 0,
      .SdaBankNum = 1,
      .SclPin     = 0,
      .SdaPin     = 0
    },
    {
      .Freq      = 200000000,
      .SpeedMode = FAST_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_3,
      .SclBankId  = BANK_ID_M,
      .SdaBankId  = BANK_ID_M,
      .SclBankNum = 2,
      .SdaBankNum = 3,
      .SclPin     = 0,
      .SdaPin     = 0
    },
    {
      .Freq      = 200000000,
      .SpeedMode = FAST_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_3,
      .SclBankId  = BANK_ID_M,
      .SdaBankId  = BANK_ID_M,
      .SclBankNum = 4,
      .SdaBankNum = 5,
      .SclPin     = 0,
      .SdaPin     = 0
    },
    {
      .Freq      = 200000000,
      .SpeedMode = FAST_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_3,
      .SclBankId  = BANK_ID_M,
      .SdaBankId  = BANK_ID_M,
      .SclBankNum = 6,
      .SdaBankNum = 7,
      .SclPin     = 0,
      .SdaPin     = 0
    },
    {
      .Freq      = 400000,
      .SpeedMode = FAST_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_M,
      .SdaBankId  = BANK_ID_M,
      .SclBankNum = 8,
      .SdaBankNum = 9,
      .SclPin     = 0,
      .SdaPin     = 0
    },
    {
      .Freq      = 400000,
      .SpeedMode = FAST_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_3,
      .SclBankId  = BANK_ID_M,
      .SdaBankId  = BANK_ID_M,
      .SclBankNum = 10,
      .SdaBankNum = 11,
      .SclPin     = 0,
      .SdaPin     = 0
    },
    {
      .Freq      = 200000000,
      .SpeedMode = FAST_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_M,
      .SdaBankId  = BANK_ID_M,
      .SclBankNum = 12,
      .SdaBankNum = 13,
      .SclPin     = 0,
      .SdaPin     = 0
    },
    {
      .Freq      = 200000000,
      .SpeedMode = FAST_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_3,
      .SclBankId  = BANK_ID_M,
      .SdaBankId  = BANK_ID_M,
      .SclBankNum = 14,
      .SdaBankNum = 15,
      .SclPin     = 0,
      .SdaPin     = 0
    },
    {
      .Freq      = 200000000,
      .SpeedMode = FAST_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_M,
      .SdaBankId  = BANK_ID_M,
      .SclBankNum = 16,
      .SdaBankNum = 17,
      .SclPin     = 0,
      .SdaPin     = 0
    },
    {
      .Freq      = 200000000,
      .SpeedMode = FAST_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_3,
      .SclBankId  = BANK_ID_M,
      .SdaBankId  = BANK_ID_M,
      .SclBankNum = 18,
      .SdaBankNum = 19,
      .SclPin     = 0,
      .SdaPin     = 0
    },
    {
      .Freq      = 200000000,
      .SpeedMode = FAST_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_C,
      .SdaBankId  = BANK_ID_C,
      .SclBankNum = 0,
      .SdaBankNum = 0,
      .SclPin     = 1,
      .SdaPin     = 0
    },
    {
      .Freq      = 1000000,
      .SpeedMode = FAST_PLUS_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_C,
      .SdaBankId  = BANK_ID_C,
      .SclBankNum = 0,
      .SdaBankNum = 0,
      .SclPin     = 3,
      .SdaPin     = 2
    },
    {
      .Freq      = 1000000,
      .SpeedMode = FAST_PLUS_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_C,
      .SdaBankId  = BANK_ID_C,
      .SclBankNum = 0,
      .SdaBankNum = 0,
      .SclPin     = 5,
      .SdaPin     = 4
    },
    {
      .Freq      = 400000,
      .SpeedMode = FAST_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_C,
      .SdaBankId  = BANK_ID_C,
      .SclBankNum = 0,
      .SdaBankNum = 0,
      .SclPin     = 7,
      .SdaPin     = 6
    },
    {
      .Freq      = 400000,
      .SpeedMode = FAST_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_2,
      .SclBankId  = BANK_ID_C,
      .SdaBankId  = BANK_ID_C,
      .SclBankNum = 1,
      .SdaBankNum = 1,
      .SclPin     = 0,
      .SdaPin     = 1
    },
    {
      .Freq      = 400000,
      .SpeedMode = FAST_SPD
    }
  },
  {
    {
      .Function   = FUNCTION_3,
      .SclBankId  = BANK_ID_C,
      .SdaBankId  = BANK_ID_C,
      .SclBankNum = 1,
      .SdaBankNum = 1,
      .SclPin     = 2,
      .SdaPin     = 3
    },
    {
      .Freq      = 400000,
      .SpeedMode = FAST_SPD
    }
  },
};

VOID
GetHsI2cBusData (
  OUT EFI_HSI2C_BUS_DATA **Data,
  OUT UINT8               *Count)
{
  *Data  = gHsI2cBusData;
  *Count = ARRAY_SIZE (gHsI2cBusData);
}
