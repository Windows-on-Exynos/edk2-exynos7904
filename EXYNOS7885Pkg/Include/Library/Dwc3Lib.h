#ifndef _DWC3_LIB_H_
#define _DWC3_LIB_H_

#define EXYNOS9610_USB_PHY_BASE							0x131D0000
#define EXYNOS9610_USB_LINK_BASE						0x13200000

/*
 * USB Controller power registers (relative to USB_LINK_BASE 0x13200000)
 * Based on stock S-BOOT reverse engineering
 */
#define USB_REG_PWR1		0xC200
#define USB_REG_PWR2		0xC2C0
#define USB_PWR_BIT		(1 << 31)

//
// DWC3 Platform Configuration
//
typedef struct {
  UINT64       BaseAddress;
  UINT64       BaseSize;
  UINT64       CmuTopMuxAddr;
  UINT32       CmuTopMuxValue;
  UINT64       CmuTopDivAddr;
  UINT32       CmuTopDivValue;
  UINT64       CmuTopGateAddr;
  UINT32       CmuTopGateValue;
  UINT64       CmuTopGateDrdAddr;
  UINT32       CmuTopGateDrdValue;
  UINT64       PmuBase;
  UINT64       PmuPhyControlOffset;
  UINT32       PmuPhyMask;
  UINT32       GsbUsbCfg0;
  UINT32       Guctl1;
  UINT32       NumHsPhy;
  UINT32       NumSsPhy;
  UINT32	   MeBurstLength;
  BOOLEAN      SusPhySupported;
  UINT32       RefClk;
  UINT32       SuspendClk;
} DWC3_PLAT_CONFIG;

typedef struct {
  CHAR8		  Speed[16];
  UINT32	  m_uEventBufDepth;
  UINT32	  m_uCtrlBufSize;
  UINT8		  m_ucU1ExitValue;
  CHAR16	  m_usU2ExitValue;
  UINT32	  NeedCacheOps;
} DWC3_DEV_CONFIG;

//
// DWC3 operation mode
//
typedef enum {
  DWC3_MODE_HOST   = 0,
  DWC3_MODE_DEVICE = 1,
} DWC3_OP_MODE;

EFI_STATUS
Dwc3DevPlatInit (
  VOID **BaseAddr,
  DWC3_DEV_CONFIG **PlatConfig
  );

/**
  Returns the DWC3 Platform Configuration for the current SoC.
**/
EFI_STATUS
GetDwc3PlatConfig (
  OUT DWC3_PLAT_CONFIG  *Config
  );

/**
  Enable CMU clock gates and release PMU isolation.
  Uses platform config for register addresses.
**/
VOID
Dwc3CoreEnableClocks (
  IN DWC3_PLAT_CONFIG  *Config
  );

/**
  Soft-reset both USB2 and USB3 PHY interfaces inside DWC3 core.
**/
VOID
Dwc3CorePhyReset (
  IN DWC3_PLAT_CONFIG  *Config
  );

/**
  Verify DWC3 IP (GSNPSID) and configure GFLADJ 30MHz mode.
  Returns EFI_DEVICE_ERROR if no valid DWC3 core detected.
**/
EFI_STATUS
Dwc3CoreGblInit (
  IN DWC3_PLAT_CONFIG  *Config,
  OUT UINT32            *LinkVersion OPTIONAL
  );

/**
  DWC3 core soft reset (DCTL.CSFTRST) followed by GSBUSCFG0 + GUCTL1 config.
**/
VOID
Dwc3CoreSoftReset (
  IN DWC3_PLAT_CONFIG  *Config
  );

/**
  Configure GCTL for the specified operation mode (host or device).
  Sets PRTCAPDIR, U2RSTECN, scrambling, VBUS filter bypass,
  RAM clock select, power down scale, and auto clock gating.
**/
VOID
Dwc3CoreConfigGctl (
  IN DWC3_PLAT_CONFIG  *Config,
  IN DWC3_OP_MODE       Mode
  );

/**
  Configure PHY interface registers (GUSB2PHYCFG + GUSB3PIPECTL)
  for operation. Sets SUSPHY, USBTrdTim, PHYIf, freeclk quirk.
**/
VOID
Dwc3CoreConfigPhyIf (
  IN DWC3_PLAT_CONFIG  *Config
  );

#endif /* _DWC3_LIB_H_ */
