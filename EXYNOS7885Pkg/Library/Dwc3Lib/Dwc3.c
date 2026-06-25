#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/Dwc3Lib.h>
#include <Library/IoLib.h>
#include <Library/UefiBootServicesTableLib.h>

#define DWC3_GSBUSCFG0        0xC100
#define DWC3_GCTL             0xC110
#define DWC3_GUCTL1           0xC11C
#define DWC3_GSNPSID          0xC120
#define DWC3_GFLADJ           0xC630
#define DWC3_GUSB2PHYCFG0     0xC200
#define DWC3_GUSB3PIPECTL0    0xC2C0
#define DWC3_DCTL             0xC704

#define GUSB2PHYCFG_PHYSOFTRST BIT31
#define GUSB2PHYCFG_SUSPHY     BIT6
#define GUSB2PHYCFG_ENBLSLPM   BIT8
#define GUSB2PHYCFG_USBTRDTIM_SHIFT 10
#define GUSB2PHYCFG_USBTRDTIM_MASK  (0xF << 10)
#define GUSB2PHYCFG_PHYIF      BIT3
#define GUSB2PHYCFG_U2_FREECLK_EXISTS BIT14

#define GUSB3PIPECTL_PHYSOFTRST BIT31
#define GUSB3PIPECTL_SUSPEND    BIT17
#define GUSB3PIPECTL_DELAY_P1P2P3_SHIFT 19

#define GCTL_PRTCAPDIR_HOST     (0x1 << 12)
#define GCTL_PRTCAPDIR_DEV      (0x2 << 12)
#define GCTL_PRTCAPDIR_MASK     (0x3 << 12)
#define GCTL_U2RSTECN           BIT16
#define GCTL_DISSCRAMBLE        BIT3
#define GCTL_MASTERFILTBYPASS   BIT5
#define GCTL_PWRDWNSCALE_SHIFT  19
#define GCTL_PWRDWNSCALE_MASK   (0x1FFF << 19)
#define GCTL_RAMCLKSEL_MASK     (0x3 << 6)
#define GCTL_RAMCLKSEL_BUS      (0x0 << 6)
#define GCTL_DISCLKGATING       BIT0
#define GCTL_SCALEDOWN_MASK     (0x3 << 4)

#define GFLADJ_30MHZ_REG_SEL    BIT7
#define GFLADJ_30MHZ_MASK       0x3F

#define DCTL_CSFTRST            BIT30

#define DWC3_RD32(A)   MmioRead32  ((UINTN)(A))
#define DWC3_WR32(A,V) MmioWrite32 ((UINTN)(A), (V))

STATIC DWC3_PLAT_CONFIG  gDwc3PlatConfig = {
  .BaseAddress             = 0x13200000,
  .BaseSize                = 0x10000,
  .CmuTopMuxAddr           = 0x1210108C,
  .CmuTopMuxValue          = 0x00000000,
  .CmuTopDivAddr           = 0x12101894,
  .CmuTopDivValue          = 0x00000000,
  .CmuTopGateAddr          = 0x12102090,
  .CmuTopGateValue         = 0x00300000,
  .CmuTopGateDrdAddr       = 0x12102098,
  .CmuTopGateDrdValue      = 0x00300000,
  .PmuBase                 = 0x11860000,
  .PmuPhyControlOffset     = 0x00000704,
  .PmuPhyMask              = 0x00000003,
  .GsbUsbCfg0              = 0x00000000,
  .Guctl1                  = 0x00000008,
  .NumHsPhy                = 1,
  .NumSsPhy                = 0,
  .MeBurstLength		   = 0xF,
  .SusPhySupported         = FALSE,
  .RefClk                  = 0,
  .SuspendClk              = 0
};

STATIC DWC3_DEV_CONFIG gDwc3DevConfig = {
  .Speed				   = "high",
  .m_uEventBufDepth		   = 64,
  .m_uCtrlBufSize		   = 128,
  .m_ucU1ExitValue		   = 10,
  .m_usU2ExitValue		   = 257,
  .NeedCacheOps			   = 1
};

EFI_STATUS
Dwc3DevPlatInit(VOID **BaseAddr, DWC3_DEV_CONFIG **PlatConfig)
{
	*BaseAddr = (VOID *)EXYNOS9610_USB_LINK_BASE;
	*PlatConfig = &gDwc3DevConfig;

	return 0;
}

EFI_STATUS
GetDwc3PlatConfig (OUT DWC3_PLAT_CONFIG *Config)
{
  if (!Config) return EFI_INVALID_PARAMETER;
  *Config = gDwc3PlatConfig;
  return EFI_SUCCESS;
}

VOID
Dwc3CoreEnableClocks (IN DWC3_PLAT_CONFIG *Cfg)
{
  UINT64  GateAddr;
  UINT32  GateVal;

  GateAddr = Cfg->CmuTopGateAddr;
  GateVal  = MmioRead32 ((UINTN)GateAddr);
  GateVal |= Cfg->CmuTopGateValue;
  MmioWrite32 ((UINTN)GateAddr, GateVal);

  GateAddr = Cfg->CmuTopGateDrdAddr;
  GateVal  = MmioRead32 ((UINTN)GateAddr);
  GateVal |= Cfg->CmuTopGateDrdValue;
  MmioWrite32 ((UINTN)GateAddr, GateVal);

  gBS->Stall (10);
}

VOID
Dwc3CorePhyReset (IN DWC3_PLAT_CONFIG *Cfg)
{
  UINT64 B = Cfg->BaseAddress;
  UINT32 V;

  V  = DWC3_RD32 (B + DWC3_GUSB3PIPECTL0);
  V |= GUSB3PIPECTL_PHYSOFTRST;
  DWC3_WR32 (B + DWC3_GUSB3PIPECTL0, V);

  V  = DWC3_RD32 (B + DWC3_GUSB2PHYCFG0);
  V |= GUSB2PHYCFG_PHYSOFTRST;
  DWC3_WR32 (B + DWC3_GUSB2PHYCFG0, V);

  gBS->Stall (50);

  V  = DWC3_RD32 (B + DWC3_GUSB2PHYCFG0);
  V &= ~GUSB2PHYCFG_PHYSOFTRST;
  DWC3_WR32 (B + DWC3_GUSB2PHYCFG0, V);

  V  = DWC3_RD32 (B + DWC3_GUSB3PIPECTL0);
  V &= ~GUSB3PIPECTL_PHYSOFTRST;
  DWC3_WR32 (B + DWC3_GUSB3PIPECTL0, V);

  gBS->Stall (100);
}

EFI_STATUS
Dwc3CoreGblInit (IN DWC3_PLAT_CONFIG *Cfg, OUT UINT32 *LinkVersion)
{
  UINT64 B = Cfg->BaseAddress;
  UINT32 Id, V;

  Id = DWC3_RD32 (B + DWC3_GSNPSID);
  if (Id == 0 || Id == 0xFFFFFFFF) return EFI_DEVICE_ERROR;
  if (LinkVersion) *LinkVersion = Id & 0xFFFF;

  V  = DWC3_RD32 (B + DWC3_GFLADJ);
  V |= GFLADJ_30MHZ_REG_SEL;
  V &= ~GFLADJ_30MHZ_MASK;
  V |= 0x20;
  DWC3_WR32 (B + DWC3_GFLADJ, V);

  return EFI_SUCCESS;
}

VOID
Dwc3CoreSoftReset (IN DWC3_PLAT_CONFIG *Cfg)
{
  UINT64 B = Cfg->BaseAddress;
  UINT32 Dctl, V;

  Dctl  = DWC3_RD32 (B + DWC3_DCTL);
  Dctl |= DCTL_CSFTRST;
  DWC3_WR32 (B + DWC3_DCTL, Dctl);

  do {
    gBS->Stall (10);
    Dctl = DWC3_RD32 (B + DWC3_DCTL);
  } while (Dctl & DCTL_CSFTRST);

  gBS->Stall (10);

  DWC3_WR32 (B + DWC3_GSBUSCFG0, 0x22220000);

  V  = DWC3_RD32 (B + DWC3_GUCTL1);
  V |= Cfg->Guctl1;
  DWC3_WR32 (B + DWC3_GUCTL1, V);
}

VOID
Dwc3CoreConfigGctl (IN DWC3_PLAT_CONFIG *Cfg, IN DWC3_OP_MODE Mode)
{
  UINT64 B = Cfg->BaseAddress;
  UINT32 V;

  V  = DWC3_RD32 (B + DWC3_GCTL);
  V &= ~GCTL_PRTCAPDIR_MASK;
  V |= (Mode == DWC3_MODE_HOST) ? GCTL_PRTCAPDIR_HOST : GCTL_PRTCAPDIR_DEV;
  V |= GCTL_U2RSTECN;
  V &= ~GCTL_DISSCRAMBLE;
  V |= GCTL_MASTERFILTBYPASS;
  V &= ~GCTL_RAMCLKSEL_MASK;
  V |= GCTL_RAMCLKSEL_BUS;
  V &= ~GCTL_PWRDWNSCALE_MASK;
  if (Cfg->SuspendClk != 0)
    V |= ((Cfg->SuspendClk / 16000) << GCTL_PWRDWNSCALE_SHIFT);
  V |= GCTL_DISCLKGATING;
  V &= ~GCTL_SCALEDOWN_MASK;
  DWC3_WR32 (B + DWC3_GCTL, V);
}

VOID
Dwc3CoreConfigPhyIf (IN DWC3_PLAT_CONFIG *Cfg)
{
  UINT64 B = Cfg->BaseAddress;
  UINT32 V;

  V  = DWC3_RD32 (B + DWC3_GUSB2PHYCFG0);
  if (Cfg->SusPhySupported) {
    V |= GUSB2PHYCFG_SUSPHY;
    V |= GUSB2PHYCFG_ENBLSLPM;
  }
  V &= ~GUSB2PHYCFG_USBTRDTIM_MASK;
  V |= (9 << GUSB2PHYCFG_USBTRDTIM_SHIFT);
  V &= ~GUSB2PHYCFG_PHYIF;
  V &= ~GUSB2PHYCFG_U2_FREECLK_EXISTS;
  DWC3_WR32 (B + DWC3_GUSB2PHYCFG0, V);

  V  = DWC3_RD32 (B + DWC3_GUSB3PIPECTL0);
  if (Cfg->SusPhySupported) {
    V |= GUSB3PIPECTL_SUSPEND;
  } else {
    V &= ~GUSB3PIPECTL_SUSPEND;
  }
  V &= ~(0x7 << GUSB3PIPECTL_DELAY_P1P2P3_SHIFT);
  V |= (1 << GUSB3PIPECTL_DELAY_P1P2P3_SHIFT);
  DWC3_WR32 (B + DWC3_GUSB3PIPECTL0, V);
}
