#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UsbPhyLib.h>

//
// PHY version constants
//
#define USBCON_VER_03_0_0  0x0300  // HS PHY only
#define USBCON_VER_05_3_0  0x0530  // SS Dual PHY

//
// Version extraction
//
#define USBCON_VER_MAJOR_MASK  0xFF00
#define USBCON_VER_SS_CAP      0x0010

//
// PHY Register offsets
//
#define USBCON_CTRL_VER        0x00
#define USBCON_LINK_CTRL       0x04
#define USBCON_LINK_PORT       0x08
#define USBCON_CLKRST          0x20
#define USBCON_PWR             0x24
#define USBCON_DUALPHYSEL      0x28
#define USBCON_COMBO_PMA_CTRL  0x48
#define USBCON_UTMI            0x50
#define USBCON_HSP             0x54
#define USBCON_HSP_TUNE        0x58
#define USBCON_HSP_TEST        0x5C

//
// LINK_CTRL (0x04) bits
//
#define LINKCTRL_DIS_QACT_ID0         BIT11
#define LINKCTRL_DIS_QACT_VBUS_VALID  BIT10
#define LINKCTRL_DIS_QACT_BVALID      BIT9
#define LINKCTRL_DIS_QACT_LINKGATE    BIT12
#define LINKCTRL_FORCE_QACT           BIT8
#define LINKCTRL_BUS_FILTER_BYPASS(x) (((x) & 0xF) << 4)
#define LINKCTRL_BUS_FILTER_BYPASS_MASK (0xF << 4)
#define LINKCTRL_PIPE3_FORCE_RX_ELEC_IDLE BIT18
#define LINKCTRL_PIPE3_FORCE_PHY_STATUS    BIT17
#define LINKCTRL_PIPE3_FORCE_EN            BIT16

//
// LINK_PORT (0x08) bits
//
#define LINKPORT_HUB_PORT_SEL_OCD_U3  BIT3
#define LINKPORT_HUB_PORT_SEL_OCD_U2  BIT2

//
// CLKRST (0x20) bits
//
#define CLKRST_PHY_SW_RST      BIT3
#define CLKRST_PHY_RST_SEL     BIT2
#define CLKRST_PORT_RST        BIT1
#define CLKRST_LINK_SW_RST     BIT0
#define CLKRST_LINK_PCLK_SEL   BIT7

//
// PWR (0x24) bits
//
#define PWR_TEST_POWERDOWN_SSP BIT1
#define PWR_TEST_POWERDOWN_HSP BIT0

//
// DUALPHYSEL (0x28) bits
//
#define DUALPHYSEL_PHYSEL_CTRL    BIT0
#define DUALPHYSEL_PHYSEL_SSPHY   BIT1
#define DUALPHYSEL_PHYSEL_PIPECLK BIT4
#define DUALPHYSEL_PHYSEL_PIPERST BIT8

//
// COMBO_PMA_CTRL (0x48) bits
//
#define PMA_LOW_PWRN      BIT4
#define PMA_TRSV_SW_RST   BIT3
#define PMA_CMN_SW_RST    BIT2
#define PMA_INIT_SW_RST   BIT1
#define PMA_APB_SW_RST    BIT0
#define PMA_REF_FREQ_MASK  (0x3 << 8)
#define PMA_REF_FREQ_SET(x) (((x) & 0x3) << 8)

//
// UTMI (0x50) bits
//
#define UTMI_FORCE_VBUSVALID  BIT5
#define UTMI_FORCE_BVALID     BIT4
#define UTMI_DP_PULLDOWN      BIT3
#define UTMI_DM_PULLDOWN      BIT2
#define UTMI_FORCE_SUSPEND    BIT1
#define UTMI_FORCE_SLEEP      BIT0

//
// HSP (0x54) bits
//
#define HSP_EN_UTMISUSPEND     BIT9
#define HSP_COMMONONN          BIT8
#define HSP_VBUSVLDEXTSEL      BIT13
#define HSP_VBUSVLDEXT         BIT12
#define HSP_FSLS_SPEED_SEL     BIT25
#define HSP_RETENABLE_EN       BIT28
#define HSP_AUTORSM_ENB        BIT29

//
// HSP_TEST (0x5C) bits
//
#define HSP_TEST_SIDDQ   BIT24

//
// PMU register
//
#define EXYNOS_USBDEV_PHY_CONTROL    0x0704
#define EXYNOS_USBDRD_ENABLE         BIT0
#define EXYNOS_USBHOST_ENABLE        BIT1
#define ENABLE_TCXO_BUF_MASK         0x10000

//
// Reference clock select sources
//
#define USBPHY_REFSEL_CLKCORE  0x2

//
// FSEL values for reference clock
//
#define EXYNOS_FSEL_26MHZ  0x82

//
// Read/Write helpers
//
#define PHY_RD32(Addr)    MmioRead32 ((UINTN)(Addr))
#define PHY_WR32(Addr, V) MmioWrite32 ((UINTN)(Addr), (V))

//
// PMU base for Exynos 9610
//
#define EXYNOS9610_PMU_BASE  0x11860000

/**
  Release PMU PHY isolation.

  @param  PhyConfig   PHY configuration.
**/
STATIC
VOID
PhyReleasePmuIsolation (
  IN USB_PHY_CONFIG  *PhyConfig
  )
{
  UINT64  PmuReg;
  UINT32  RegVal;

  PmuReg = EXYNOS9610_PMU_BASE + PhyConfig->PmuOffset;

  //
  // SET PMU mask bits and TCXO buffer to release isolation.
  //
  RegVal  = PHY_RD32 (PmuReg);
  RegVal |= PhyConfig->PmuMask;
  RegVal |= ENABLE_TCXO_BUF_MASK;
  PHY_WR32 (PmuReg, RegVal);

  DEBUG ((EFI_D_INFO, "UsbPhy: PMU isolation released (reg=0x%lx, val=0x%08x)\n",
          PmuReg, PHY_RD32 (PmuReg)));
}

/**
  Apply PMU PHY isolation.

  @param  PhyConfig   PHY configuration.
**/
STATIC
VOID
PhyApplyPmuIsolation (
  IN USB_PHY_CONFIG  *PhyConfig
  )
{
  UINT64  PmuReg;
  UINT32  RegVal;

  PmuReg = EXYNOS9610_PMU_BASE + PhyConfig->PmuOffset;

  //
  // CLEAR USB enable bits to apply isolation.
  //
  RegVal  = PHY_RD32 (PmuReg);
  RegVal &= ~PhyConfig->PmuMask;
  PHY_WR32 (PmuReg, RegVal);

  DEBUG ((EFI_D_INFO, "UsbPhy: PMU isolation applied (reg=0x%lx, val=0x%08x)\n",
          PmuReg, PHY_RD32 (PmuReg)));
}

/**
  Force Q-channel activation for PHY v3.0.0.

  @param  RegsBase   PHY register base address.
**/
STATIC
VOID
PhyForceQchannel (
  IN UINT64  RegsBase
  )
{
  UINT32  PhyResume;

  PhyResume  = PHY_RD32 (RegsBase + USBCON_LINK_CTRL);
  PhyResume |= LINKCTRL_DIS_QACT_ID0;
  PhyResume |= LINKCTRL_DIS_QACT_VBUS_VALID;
  PhyResume |= LINKCTRL_DIS_QACT_BVALID;
  PhyResume |= LINKCTRL_DIS_QACT_LINKGATE;
  PhyResume &= ~LINKCTRL_FORCE_QACT;
  PHY_WR32 (RegsBase + USBCON_LINK_CTRL, PhyResume);

  gBS->Stall (500);

  PhyResume |= LINKCTRL_FORCE_QACT;
  PHY_WR32 (RegsBase + USBCON_LINK_CTRL, PhyResume);

  gBS->Stall (500);
}

/**
  PHY software reset: assert reset (set high).
**/
STATIC
VOID
PhySwResetHigh (
  IN USB_PHY_CONFIG  *PhyConfig
  )
{
  UINT64  RegsBase = PhyConfig->RegBase;
  UINT32  ClkRst;

  ClkRst  = PHY_RD32 (RegsBase + USBCON_CLKRST);
  ClkRst |= CLKRST_PHY_SW_RST;
  ClkRst |= CLKRST_PHY_RST_SEL;
  PHY_WR32 (RegsBase + USBCON_CLKRST, ClkRst);

  DEBUG ((EFI_D_INFO, "UsbPhy: Reset HIGH (CLKRST=0x%08x)\n",
          PHY_RD32 (RegsBase + USBCON_CLKRST)));
}

/**
  PHY software reset: deassert reset (set low).
**/
STATIC
VOID
PhySwResetLow (
  IN USB_PHY_CONFIG  *PhyConfig
  )
{
  UINT64  RegsBase = PhyConfig->RegBase;
  UINT32  ClkRst;

  ClkRst  = PHY_RD32 (RegsBase + USBCON_CLKRST);
  ClkRst |= CLKRST_PHY_RST_SEL;
  ClkRst &= ~CLKRST_PHY_SW_RST;
  ClkRst &= ~CLKRST_PORT_RST;
  PHY_WR32 (RegsBase + USBCON_CLKRST, ClkRst);

  DEBUG ((EFI_D_INFO, "UsbPhy: Reset LOW (CLKRST=0x%08x)\n",
          PHY_RD32 (RegsBase + USBCON_CLKRST)));
}

/**
  Enable PHY power.
**/
STATIC
VOID
PhyPowerEnable (
  IN USB_PHY_CONFIG  *PhyConfig
  )
{
  UINT64  RegsBase = PhyConfig->RegBase;
  UINT32  Reg;
  UINT32  MainVersion;

  MainVersion = PhyConfig->Version & USBCON_VER_MAJOR_MASK;

  if (MainVersion == USBCON_VER_03_0_0) {
    //
    // HS PHY: clear SIDDQ bit in HSP_TEST
    //
    Reg  = PHY_RD32 (RegsBase + USBCON_HSP_TEST);
    Reg &= ~HSP_TEST_SIDDQ;
    PHY_WR32 (RegsBase + USBCON_HSP_TEST, Reg);
  }

  DEBUG ((EFI_D_INFO, "UsbPhy: Power enabled (HSP_TEST=0x%08x)\n",
          PHY_RD32 (RegsBase + USBCON_HSP_TEST)));
}

/**
  Disable PHY power.
**/
STATIC
VOID
PhyPowerDisable (
  IN USB_PHY_CONFIG  *PhyConfig
  )
{
  UINT64  RegsBase = PhyConfig->RegBase;
  UINT32  MainVersion;

  MainVersion = PhyConfig->Version & USBCON_VER_MAJOR_MASK;

  if (MainVersion == USBCON_VER_03_0_0) {
    PHY_WR32 (RegsBase + USBCON_HSP_TEST,
              PHY_RD32 (RegsBase + USBCON_HSP_TEST) | HSP_TEST_SIDDQ);
  }
}

/**
  Configure UTMI interface for High-Speed operation.
**/
STATIC
VOID
PhyConfigureUtmi (
  IN USB_PHY_CONFIG  *PhyConfig
  )
{
  UINT64  RegsBase = PhyConfig->RegBase;
  UINT32  Reg;

  //
  // Clear UTMI force suspend/sleep and pulldowns
  //
  Reg  = PHY_RD32 (RegsBase + USBCON_UTMI);
  Reg &= ~UTMI_FORCE_SUSPEND;
  Reg &= ~UTMI_FORCE_SLEEP;
  Reg &= ~UTMI_DP_PULLDOWN;
  Reg &= ~UTMI_DM_PULLDOWN;
  PHY_WR32 (RegsBase + USBCON_UTMI, Reg);

  //
  // HSP: enable UTMI suspend, common block control, high-speed
  //
  Reg  = PHY_RD32 (RegsBase + USBCON_HSP);
  Reg |= HSP_EN_UTMISUSPEND;
  Reg |= HSP_FSLS_SPEED_SEL;  // High-Speed
  if (PhyConfig->CommonBlockDisable) {
    Reg |= HSP_COMMONONN;
  } else {
    Reg &= ~HSP_COMMONONN;
  }
  PHY_WR32 (RegsBase + USBCON_HSP, Reg);

  DEBUG ((EFI_D_INFO, "UsbPhy: UTMI configured (UTMI=0x%08x, HSP=0x%08x)\n",
          PHY_RD32 (RegsBase + USBCON_UTMI),
          PHY_RD32 (RegsBase + USBCON_HSP)));
}

/**
  Configure VBUS control.

  When VBUS pad is NOT used, force VBUS valid and B-valid
  to make the PHY think a host is connected.

  @param  PhyConfig   PHY configuration.
**/
STATIC
VOID
PhyConfigureVbus (
  IN USB_PHY_CONFIG  *PhyConfig
  )
{
  UINT64  RegsBase = PhyConfig->RegBase;
  UINT32  RegUtmi;
  UINT32  RegHsp;
  UINT32  RegLink;

  RegUtmi = PHY_RD32 (RegsBase + USBCON_UTMI);
  RegHsp  = PHY_RD32 (RegsBase + USBCON_HSP);

  if (PhyConfig->NotUsedVbusPad) {
    RegLink  = PHY_RD32 (RegsBase + USBCON_LINK_CTRL);
    RegLink |= LINKCTRL_BUS_FILTER_BYPASS (0xF);
    PHY_WR32 (RegsBase + USBCON_LINK_CTRL, RegLink);

    RegUtmi |= UTMI_FORCE_BVALID;
    RegUtmi |= UTMI_FORCE_VBUSVALID;
    RegHsp  |= HSP_VBUSVLDEXTSEL;
    RegHsp  |= HSP_VBUSVLDEXT;
  } else {
    //
    // Use real VBUS pad
    //
    RegLink  = PHY_RD32 (RegsBase + USBCON_LINK_CTRL);
    RegLink &= ~LINKCTRL_BUS_FILTER_BYPASS_MASK;
    PHY_WR32 (RegsBase + USBCON_LINK_CTRL, RegLink);

    RegUtmi &= ~UTMI_FORCE_BVALID;
    RegUtmi &= ~UTMI_FORCE_VBUSVALID;
    RegHsp  &= ~HSP_VBUSVLDEXT;
  }

  PHY_WR32 (RegsBase + USBCON_UTMI, RegUtmi);
  PHY_WR32 (RegsBase + USBCON_HSP, RegHsp);

  DEBUG ((EFI_D_INFO, "UsbPhy: VBUS %s (UTMI=0x%08x, HSP=0x%08x)\n",
          PhyConfig->NotUsedVbusPad ? "FORCED" : "external",
          PHY_RD32 (RegsBase + USBCON_UTMI),
          PHY_RD32 (RegsBase + USBCON_HSP)));
}

/**
  Configure over-current detection.

  @param  PhyConfig   PHY configuration.
**/
STATIC
VOID
PhyConfigureOvc (
  IN USB_PHY_CONFIG  *PhyConfig
  )
{
  UINT64  RegsBase = PhyConfig->RegBase;
  UINT32  Reg;

  Reg = PHY_RD32 (RegsBase + USBCON_LINK_PORT);

  if (PhyConfig->UseIoForOvc) {
    Reg &= ~LINKPORT_HUB_PORT_SEL_OCD_U3;
    Reg &= ~LINKPORT_HUB_PORT_SEL_OCD_U2;
  } else {
    Reg |= LINKPORT_HUB_PORT_SEL_OCD_U3;
    Reg |= LINKPORT_HUB_PORT_SEL_OCD_U2;
  }

  PHY_WR32 (RegsBase + USBCON_LINK_PORT, Reg);
}

/**
  Select PHY mux for dual PHY configuration.

  @param  PhyConfig   PHY configuration.
**/
STATIC
VOID
PhySelectDualPhy (
  IN USB_PHY_CONFIG  *PhyConfig
  )
{
  UINT64  RegsBase = PhyConfig->RegBase;
  UINT32  PhySel;

  PhySel = PHY_RD32 (RegsBase + USBCON_DUALPHYSEL);

  if (PhyConfig->UsedPhyPort == 0) {
    //
    // Port 0: route to main PHY
    //
    PhySel &= ~DUALPHYSEL_PHYSEL_CTRL;
    PhySel &= ~DUALPHYSEL_PHYSEL_SSPHY;
    PhySel &= ~DUALPHYSEL_PHYSEL_PIPECLK;
    PhySel &= ~DUALPHYSEL_PHYSEL_PIPERST;
  } else {
    //
    // Port 1: route to secondary PHY
    //
    PhySel |= DUALPHYSEL_PHYSEL_CTRL;
    PhySel |= DUALPHYSEL_PHYSEL_SSPHY;
    PhySel |= DUALPHYSEL_PHYSEL_PIPECLK;
    PhySel |= DUALPHYSEL_PHYSEL_PIPERST;
  }

  PHY_WR32 (RegsBase + USBCON_DUALPHYSEL, PhySel);
}

/**
  Initialize a USB PHY instance.
**/
EFI_STATUS
UsbPhyInit (
  IN USB_PHY_CONFIG  *PhyConfig
  )
{
  UINT64  RegsBase;
  UINT32  MainVersion;
  BOOLEAN SsOnlyCap;

  if (PhyConfig == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  RegsBase    = PhyConfig->RegBase;
  MainVersion = PhyConfig->Version & USBCON_VER_MAJOR_MASK;
  SsOnlyCap   = (PhyConfig->Version & USBCON_VER_SS_CAP) ? TRUE : FALSE;

  DEBUG ((EFI_D_INFO, "UsbPhy: Init PHY at 0x%lx (ver=0x%x, main=0x%x, SS=%d)\n",
          RegsBase, PhyConfig->Version, MainVersion, SsOnlyCap));

  PhyReleasePmuIsolation (PhyConfig);

  if (MainVersion == USBCON_VER_03_0_0) {
    //
    // Force Q-channel
    //
    PhyForceQchannel (RegsBase);

    //
    // Link reset
    //
    {
      UINT32  Reg;
      Reg  = PHY_RD32 (RegsBase + USBCON_CLKRST);
      Reg |= CLKRST_LINK_SW_RST;
      PHY_WR32 (RegsBase + USBCON_CLKRST, Reg);

      gBS->Stall (10);

      Reg &= ~CLKRST_LINK_SW_RST;
      PHY_WR32 (RegsBase + USBCON_CLKRST, Reg);
    }
  }

  PhySwResetHigh (PhyConfig);

  //
  // Configure UTMI + HSP while reset is still asserted,
  // then set VBUS BEFORE enabling PHY power.
  //
  if (!SsOnlyCap) {
    PhyConfigureUtmi (PhyConfig);
  }

  gBS->Stall (100);

  //
  // Set VBUS Valid and DP-Pull up control
  //
  PhyConfigureVbus (PhyConfig);

  //
  // Enable PHY power — clear SIDDQ
  //
  PhyPowerEnable (PhyConfig);

  gBS->Stall (10);

  //
  // Deassert PHY reset — PHY now fully operational
  //
  PhySwResetLow (PhyConfig);

  gBS->Stall (75);

  if (SsOnlyCap) {
    DEBUG ((EFI_D_INFO, "UsbPhy: SS-only mode, skipping HS config\n"));
    return EFI_SUCCESS;
  }

  if (PhyConfig->DualPhy) {
    PhySelectDualPhy (PhyConfig);
  }

  PhyConfigureOvc (PhyConfig);

  DEBUG ((EFI_D_INFO, "UsbPhy: Init complete\n"));
  return EFI_SUCCESS;
}

/**
  Power down a USB PHY instance.
**/
EFI_STATUS
UsbPhyExit (
  IN USB_PHY_CONFIG  *PhyConfig
  )
{
  if (PhyConfig == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  //
  // Power down PHY
  //
  PhyPowerDisable (PhyConfig);

  //
  // Apply PMU isolation
  //
  PhyApplyPmuIsolation (PhyConfig);

  DEBUG ((EFI_D_INFO, "UsbPhy: Exit complete\n"));
  return EFI_SUCCESS;
}

STATIC USB_PHY_CONFIG  gPhyConfig0 = {
  .Type               = UsbPhyUtmi,
  .RegBase            = 0x131D0000,
  .RegBase2nd         = 0,
  .Version            = 0x0300,
  .PmuOffset          = 0x0704,
  .PmuMask            = 0x03,
  .RefClk             = 26000000,
  .ExtRefClk          = EXYNOS_FSEL_26MHZ,
  .RefSel             = USBPHY_REFSEL_CLKCORE,
  .NotUsedVbusPad     = TRUE,    // is_not_vbus_pad = 1
  .UseIoForOvc        = FALSE,   // use_io_for_ovc = 0
  .CommonBlockDisable = TRUE,    // common_block_disable = 1
  .HsRewa             = FALSE,   // No HS ReWA for HS-only PHY
  .DualPhy            = FALSE,   // has_other_phy = 0
  .UsedPhyPort        = 0,
  .Usb3PhyIsolation   = TRUE,    // usb3phy-isolation = 1
};

STATIC USB_PHY_CONFIG  gPhyConfig1 = {
  .Type               = UsbPhyUtmi,   // UTMI for HS part
  .RegBase            = 0x131F0000,   // Main PHY regs
  .RegBase2nd         = 0x131E0000,   // Secondary PHY regs (SS)
  .Version            = 0x0530,
  .PmuOffset          = 0x0704,
  .PmuMask            = 0x03,
  .RefClk             = 26000000,
  .ExtRefClk          = EXYNOS_FSEL_26MHZ,
  .RefSel             = USBPHY_REFSEL_CLKCORE,
  .NotUsedVbusPad     = TRUE,
  .UseIoForOvc        = FALSE,
  .CommonBlockDisable = TRUE,
  .HsRewa             = TRUE,
  .DualPhy            = TRUE,
  .UsedPhyPort        = 0,
  .Usb3PhyIsolation   = TRUE,
};

//
// Array of PHY config pointers
//
STATIC USB_PHY_CONFIG  *gPhyConfigs[] = {
  &gPhyConfig0,
  &gPhyConfig1,
};

/**
  Get the number of PHY instances for Exynos 9610 and their configurations.
**/
EFI_STATUS
GetUsbPhyConfigs (
  OUT USB_PHY_CONFIG  ***Configs,
  OUT UINT8            *Count
  )
{
  if (Configs == NULL || Count == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  *Configs = gPhyConfigs;
  *Count   = 2;
  return EFI_SUCCESS;
}

VOID
UsbPhyConnect (VOID)
{
  UINT64  RegsBase = gPhyConfig0.RegBase;
  UINT32  RegHsp;

  RegHsp  = PHY_RD32 (RegsBase + USBCON_HSP);
  RegHsp |= HSP_VBUSVLDEXT;
  PHY_WR32 (RegsBase + USBCON_HSP, RegHsp);

  DEBUG ((EFI_D_INFO, "UsbPhy: Connect (HSP=0x%08x)\n",
          PHY_RD32 (RegsBase + USBCON_HSP)));
}

VOID
UsbPhyDisconnect (VOID)
{
  UINT64  RegsBase = gPhyConfig0.RegBase;
  UINT32  RegHsp;

  RegHsp  = PHY_RD32 (RegsBase + USBCON_HSP);
  RegHsp &= ~HSP_VBUSVLDEXT;
  PHY_WR32 (RegsBase + USBCON_HSP, RegHsp);
}
