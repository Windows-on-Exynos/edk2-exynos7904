#ifndef _MMC_PLATFORM_LIB_H_
#define _MMC_PLATFORM_LIB_H_

#include <Uefi.h>
#include <Protocol/DevicePath.h>

#define MMC_CHANNEL_EMMC  0
#define MMC_CHANNEL_SD    2
#define MMC_CHANNEL_MAX   3

typedef struct {
  UINTN   IoBase;
  UINT32  SdrClksel;
  UINT32  DdrClksel;
  UINT32  Sdr50Clksel;
  UINT32  Sdr104Clksel;
  UINT32  Hs200Clksel;
  UINT32  Hs400Clksel;
  UINT32  BusHz;
  UINT32  PhaseDivide;
  UINT32  Ciudiv;
  UINT32  FifoDepth;
} MMC_PLATFORM_HOST;

/**
  Initialize MMC platform hardware for the specified channel.

  This function handles:
    - PMIC regulator enable
    - GPIO pinmux configuration for the channel's pins
    - CMU clock setup (MUX, DIV, GATE)

  @param[in]  Channel   The MMC channel to init (MMC_CHANNEL_EMMC or MMC_CHANNEL_SD).
  @param[out] Host      Platform host data filled in by this function.

  @return EFI_SUCCESS on success, error code otherwise.
**/
EFI_STATUS
MmcPlatformInit (
  IN  UINT32              Channel,
  OUT MMC_PLATFORM_HOST  *Host
  );

EFI_STATUS
SdVoltageSwitch (VOID);

EFI_STATUS
MmcGetCardDetect (
  IN  UINT32   Channel,
  OUT BOOLEAN *Present
  );

typedef struct {
  VENDOR_DEVICE_PATH        VendorDp;
  UINT8                     Slot;
  EFI_DEVICE_PATH_PROTOCOL  End;
} EXYNOS_SD_DEVICE_PATH;

#define EXYNOS_SD_CARD_GUID \
  { 0x7A2C3B4D, 0xE5F6, 0x4A8B, \
    { 0x9C, 0x0D, 0x1E, 0x2F, 0x3A, 0x4B, 0x5C, 0x6D } \
  }

extern EFI_GUID gExynosSdCardGuid;

#endif // _MMC_PLATFORM_LIB_H_
