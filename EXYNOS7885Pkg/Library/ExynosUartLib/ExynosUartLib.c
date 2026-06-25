#include <Library/ExynosUartLib.h>

STATIC
EFI_UART_BUS_DATA
gUartBusData[] = {
  // UART0 — ALIVE
  {
    .Gpio = {
      .Tx = {
        .Function = FUNCTION_2,
        .Pull     = PULL_UP,
        .BankId   = BANK_ID_Q,
        .BankNum  = 0,
        .Pin      = 3,
      },
      .Rx = {
        .Function = FUNCTION_2,
        .Pull     = PULL_UP,
        .BankId   = BANK_ID_Q,
        .BankNum  = 0,
        .Pin      = 4,
      },
    },
  },
  // UART1 — USI_0_SHUB
  {
    .Gpio = {
      .Tx  = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_H,
        .BankNum  = 0,
        .Pin      = 0,
      },
      .Rx  = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_H,
        .BankNum  = 0,
        .Pin      = 1,
      },
      .Rts = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_H,
        .BankNum  = 0,
        .Pin      = 2,
      },
      .Cts = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_H,
        .BankNum  = 0,
        .Pin      = 3,
      },
    },
  },
  // UART2 — USI_0_CMGP
  {
    .Gpio = {
      .Tx  = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 0,
        .Pin      = 0,
      },
      .Rx  = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 1,
        .Pin      = 0,
      },
      .Rts = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 2,
        .Pin      = 0,
      },
      .Cts = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 3,
        .Pin      = 0,
      },
    },
  },
  // UART3 — USI_1_CMGP
  {
    .Gpio = {
      .Tx  = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 4,
        .Pin      = 0,
      },
      .Rx  = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 5,
        .Pin      = 0,
      },
      .Rts = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 6,
        .Pin      = 0,
      },
      .Cts = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 7,
        .Pin      = 0,
      },
    },
  },
  // UART4 — USI_2_CMGP
  {
    .Gpio = {
      .Tx  = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  =  8,
        .Pin      = 0,
      },
      .Rx  = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  =  9,
        .Pin      = 0,
      },
      .Rts = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 10,
        .Pin      = 0,
      },
      .Cts = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 11,
        .Pin      = 0,
      },
    },
  },
  // UART5 — USI_3_CMGP
  {
    .Gpio = {
      .Tx  = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 12,
        .Pin      = 0,
      },
      .Rx  = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 13,
        .Pin      = 0,
      },
      .Rts = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 14,
        .Pin      = 0,
      },
      .Cts = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 15,
        .Pin      = 0,
      },
    },
  },
  // UART6 — USI_4_CMGP
  {
    .Gpio = {
      .Tx  = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 16,
        .Pin      = 0,
      },
      .Rx  = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 17,
        .Pin      = 0,
      },
      .Rts = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 18,
        .Pin      = 0,
      },
      .Cts = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_M,
        .BankNum  = 19,
        .Pin      = 0,
      },
    },
  },
  // UART7 — USI_PERI_USI_0
  {
    .Gpio = {
      .Tx  = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_C,
        .BankNum  = 1,
        .Pin      = 0,
      },
      .Rx  = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_C,
        .BankNum  = 1,
        .Pin      = 1,
      },
      .Rts = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_C,
        .BankNum  = 1,
        .Pin      = 2,
      },
      .Cts = {
        .Function = FUNCTION_2,
        .Pull     = PULL_NONE,
        .BankId   = BANK_ID_C,
        .BankNum  = 1,
        .Pin      = 3,
      },
    },
  },
};

#define UART_CLK_NORMAL  133250000
#define UART_CLK_DEBUG   199875000

UINT32
GetUartClock (
  VOID
  )
{
  return UART_CLK_NORMAL;
}

VOID
GetUartBusData (
  OUT EFI_UART_BUS_DATA **Data,
  OUT UINT8              *Count)
{
  *Data  = gUartBusData;
  *Count = ARRAY_SIZE (gUartBusData);
}