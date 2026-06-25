#ifndef _EXYNOS_UART_LIB_H_
#define _EXYNOS_UART_LIB_H_

#include <Device/Gpio.h>

//
// UART Pin Data (one per signal: TX, RX, RTS, CTS)
//
typedef struct {
  EFI_GPIO_FUNCTION Function;
  EFI_GPIO_PULL_MODE Pull;
  EFI_GPIO_BANK_ID   BankId;
  UINT8              BankNum;
  UINT8              Pin;
} EFI_UART_PIN_DATA;

//
// UART GPIO Data (all pins for one port)
//
typedef struct {
  EFI_UART_PIN_DATA Tx;
  EFI_UART_PIN_DATA Rx;
  EFI_UART_PIN_DATA Rts;
  EFI_UART_PIN_DATA Cts;
} EFI_UART_GPIO_DATA;

//
// UART Bus Data
//
typedef struct {
  EFI_UART_GPIO_DATA   Gpio;
  BOOLEAN              Initialized;
} EFI_UART_BUS_DATA;

/**
  This Function Returns the Platform UART Buses.

  @param[out] Data                         - The UART Data.
  @param[out] Count                        - The Number of UART Buses.
**/
VOID
GetUartBusData (
  OUT EFI_UART_BUS_DATA **Data,
  OUT UINT8              *Count
  );

/**
  Returns the UART source clock frequency for this SoC.

  @return Clock frequency in Hz.
**/
UINT32
GetUartClock (
  VOID
  );

#endif /* _EXYNOS_UART_LIB_H_ */