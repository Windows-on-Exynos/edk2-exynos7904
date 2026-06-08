#include <Uefi.h>
#include <Library/DebugLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <Library/MmcPlatformLib.h>
#include <Library/DwMmcRegs.h>
#include <Protocol/EfiGpio.h>
#include <Protocol/EfiSpeedy.h>
#include <Drivers/PmicDxe/S2mpu09.h>

#define DW_MMC0_BASE   0x13500000
#define DW_MMC2_BASE   0x13550000

#define CLK_CON_MUX_MMC_CARD    0x12101040UL
#define CLK_CON_DIV_MMC_CARD    0x12101848UL
#define CLK_CON_GAT_MMC_CARD    0x12102048UL

#define CLK_CON_MUX_MMC_EMBD    0x12101044UL
#define CLK_CON_DIV_MMC_EMBD    0x1210184CUL
#define CLK_CON_GAT_MMC_EMBD    0x1210204CUL

#define CMU_MUX_SEL_PLL          0x1
#define CMU_BUSY_BIT             (1 << 16)
#define CMU_GATE_MANUAL          (1 << 20)
#define CMU_GATE_CG_VAL          (1 << 21)
#define CMU_DIV_MASK             0x1FF
#define CMU_TIMEOUT              100

#define EXYNOS9610_SDR_CLKSEL    EXYNOS_CLKSEL_TIMING (3, 0, 2, 0)
#define EXYNOS9610_DDR_CLKSEL    EXYNOS_CLKSEL_TIMING (3, 0, 2, 1)
#define EXYNOS9610_HS200_CLKSEL  EXYNOS_CLKSEL_TIMING (0, 0, 2, 0)
#define EXYNOS9610_HS400_CLKSEL  EXYNOS_CLKSEL_TIMING (0, 0, 2, 1)
#define EXYNOS9610_SDR50_CLKSEL  EXYNOS_CLKSEL_TIMING (3, 0, 4, 2)
#define EXYNOS9610_SDR104_CLKSEL EXYNOS_CLKSEL_TIMING (3, 0, 3, 0)

#define MMC_BUS_HZ              800000000
#define MMC_CIU_DIV             4
#define MMC_CIUDIV              3
#define MMC_FIFO_DEPTH          0x40


STATIC
EFI_STATUS
MmcPowerSet (VOID)
{
  EFI_STATUS                    Status;
  EFI_SPEEDY_PROTOCOL          *Speedy;
  UINT8                         Value;

  Status = gBS->LocateProtocol (&gEfiSpeedyProtocolGuid, NULL, (VOID *)&Speedy);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: Speedy protocol not found: %r\n", Status));
    return Status;
  }

  Status = Speedy->Read (0x11A10000, S2MPU09_PM_ADDR,
                         S2MPU09_PM_LDO2_CTRL, &Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: LDO2_CTRL read failed: %r\n", Status));
    return Status;
  }
  Value |= S2MPU09_OUTPUT_ON_NORMAL;
  Status = Speedy->Write (0x11A10000, S2MPU09_PM_ADDR,
                          S2MPU09_PM_LDO2_CTRL, Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: LDO2_CTRL write failed: %r\n", Status));
    return Status;
  }

  Status = Speedy->Read (0x11A10000, S2MPU09_PM_ADDR,
                         S2MPU09_PM_LDO35_CTRL, &Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: LDO35_CTRL read failed: %r\n", Status));
    return Status;
  }
  Value |= S2MPU09_OUTPUT_ON_NORMAL;
  Status = Speedy->Write (0x11A10000, S2MPU09_PM_ADDR,
                          S2MPU09_PM_LDO35_CTRL, Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: LDO35_CTRL write failed: %r\n", Status));
    return Status;
  }

  MicroSecondDelay (1200);
  return EFI_SUCCESS;
}

EFI_STATUS
MmcGetCardDetect (
  IN  UINT32   Channel,
  OUT BOOLEAN *Present
  )
{
  EFI_STATUS                Status;
  EFI_EXYNOS_GPIO_PROTOCOL *Gpio;
  UINT8                     GpioState;

  if (!Present)
    return EFI_INVALID_PARAMETER;

  if (Channel == MMC_CHANNEL_SD) {
    Status = gBS->LocateProtocol (&gEfiExynosGpioProtocolGuid, NULL, (VOID *)&Gpio);
    if (!EFI_ERROR (Status)) {
      Status = Gpio->GetPin (2, GPIO_BANK_ID_A, 6, &GpioState);
      if (!EFI_ERROR (Status)) {
        *Present = (GpioState == 0);
        DEBUG ((EFI_D_WARN, "MMC%u: GPIO CD (GPA2-6)=%d %s\n",
          Channel, GpioState, *Present ? "present" : "absent"));
        return EFI_SUCCESS;
      }
    }
  }

  *Present = FALSE;
  DEBUG ((EFI_D_WARN, "MMC%u: CD not implemented for this channel\n", Channel));
  return EFI_UNSUPPORTED;
}

EFI_STATUS
SdVoltageSwitch (VOID)
{
  EFI_STATUS                    Status;
  EFI_SPEEDY_PROTOCOL          *Speedy;
  UINT8                         Value;

  Status = gBS->LocateProtocol (&gEfiSpeedyProtocolGuid, NULL, (VOID *)&Speedy);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: Speedy protocol not found: %r\n", Status));
    return Status;
  }

  Status = Speedy->Read (0x11A10000, S2MPU09_PM_ADDR,
                         S2MPU09_PM_LDO2_VOLT, &Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: LDO2_VOLT read failed: %r\n", Status));
    return Status;
  }

  Value &= ~S2MPU09_LDO_VSEL_MASK;
  Value |= S2MPU09_LDO_VSEL_1V8;

  Status = Speedy->Write (0x11A10000, S2MPU09_PM_ADDR,
                          S2MPU09_PM_LDO2_VOLT, Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: LDO2_VOLT write (1.8V) failed: %r\n", Status));
    return Status;
  }

  MicroSecondDelay (500);
  DEBUG ((EFI_D_WARN, "MMC: LDO2 VQMMC switched to 1.8V\n"));
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MmcGpioInitEmmc (VOID)
{
  EFI_STATUS                Status;
  EFI_EXYNOS_GPIO_PROTOCOL *Gpio;
  UINT8                     Pin;

  Status = gBS->LocateProtocol (&gEfiExynosGpioProtocolGuid, NULL, (VOID *)&Gpio);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: GPIO protocol not found: %r\n", Status));
    return Status;
  }

  Status = Gpio->ConfigurePin (0, GPIO_BANK_ID_F, 0, 2);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: GPF0-0 config\n"));
    return Status;
  }
  Status = Gpio->SetPull (0, GPIO_BANK_ID_F, 0, GPIO_PULL_NONE);
  Status = Gpio->SetDrv  (0, GPIO_BANK_ID_F, 0, 4);

  Status = Gpio->ConfigurePin (0, GPIO_BANK_ID_F, 1, 2);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: GPF0-1 config\n"));
    return Status;
  }
  Status = Gpio->SetPull (0, GPIO_BANK_ID_F, 1, GPIO_PULL_UP);
  Status = Gpio->SetDrv  (0, GPIO_BANK_ID_F, 1, 4);

  for (Pin = 0; Pin <= 7; Pin++) {
    Status = Gpio->ConfigurePin (1, GPIO_BANK_ID_F, Pin, 2);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "MMC: GPF1-%d config skipped\n", Pin));
      continue;
    }
    Gpio->SetPull (1, GPIO_BANK_ID_F, Pin, GPIO_PULL_UP);
    Gpio->SetDrv  (1, GPIO_BANK_ID_F, Pin, 4);
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
MmcGpioInitSd (VOID)
{
  EFI_STATUS                Status;
  EFI_EXYNOS_GPIO_PROTOCOL *Gpio;
  UINT8                     Pin;

  Status = gBS->LocateProtocol (&gEfiExynosGpioProtocolGuid, NULL, (VOID *)&Gpio);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: GPIO protocol not found: %r\n", Status));
    return Status;
  }

  Status = Gpio->ConfigurePin (2, GPIO_BANK_ID_F, 0, 2);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: GPF2-0 config\n"));
    return Status;
  }
  Status = Gpio->SetPull (2, GPIO_BANK_ID_F, 0, GPIO_PULL_NONE);
  Status = Gpio->SetDrv  (2, GPIO_BANK_ID_F, 0, 6);

  Status = Gpio->ConfigurePin (2, GPIO_BANK_ID_F, 1, 2);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC: GPF2-1 config\n"));
    return Status;
  }
  Status = Gpio->SetPull (2, GPIO_BANK_ID_F, 1, GPIO_PULL_UP);
  Status = Gpio->SetDrv  (2, GPIO_BANK_ID_F, 1, 4);

  for (Pin = 2; Pin <= 5; Pin++) {
    Status = Gpio->ConfigurePin (2, GPIO_BANK_ID_F, Pin, 2);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "MMC: GPF2-%d config skipped\n", Pin));
      continue;
    }
    Gpio->SetPull (2, GPIO_BANK_ID_F, Pin, GPIO_PULL_UP);
    Gpio->SetDrv  (2, GPIO_BANK_ID_F, Pin, 4);
  }

  Status = Gpio->ConfigurePin (2, GPIO_BANK_ID_A, 6, 0);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_WARN, "MMC: GPA2-6 CD config: %r\n", Status));
  } else {
    Gpio->SetPull (2, GPIO_BANK_ID_A, 6, GPIO_PULL_UP);
  }

  return EFI_SUCCESS;
}

STATIC
VOID
MmcClockInitEmmc (VOID)
{
  UINT32 Reg, Timeout;

  Reg = MmioRead32 (CLK_CON_DIV_MMC_EMBD);
  Reg &= ~CMU_DIV_MASK;
  MmioWrite32 (CLK_CON_DIV_MMC_EMBD, Reg);
  for (Timeout = 0; Timeout < CMU_TIMEOUT; Timeout++) {
    if (!(MmioRead32 (CLK_CON_DIV_MMC_EMBD) & CMU_BUSY_BIT))
      break;
  }

  Reg = MmioRead32 (CLK_CON_MUX_MMC_EMBD);
  Reg |= CMU_MUX_SEL_PLL;
  MmioWrite32 (CLK_CON_MUX_MMC_EMBD, Reg);
  for (Timeout = 0; Timeout < CMU_TIMEOUT; Timeout++) {
    if (!(MmioRead32 (CLK_CON_MUX_MMC_EMBD) & CMU_BUSY_BIT))
      break;
  }

  MmioWrite32 (CLK_CON_GAT_MMC_EMBD, CMU_GATE_MANUAL | CMU_GATE_CG_VAL);
}

STATIC
VOID
MmcClockInitSd (VOID)
{
  UINT32 Reg, Timeout;

  Reg = MmioRead32 (CLK_CON_DIV_MMC_CARD);
  Reg &= ~CMU_DIV_MASK;
  MmioWrite32 (CLK_CON_DIV_MMC_CARD, Reg);
  for (Timeout = 0; Timeout < CMU_TIMEOUT; Timeout++) {
    if (!(MmioRead32 (CLK_CON_DIV_MMC_CARD) & CMU_BUSY_BIT))
      break;
  }

  Reg = MmioRead32 (CLK_CON_MUX_MMC_CARD);
  Reg |= CMU_MUX_SEL_PLL;
  MmioWrite32 (CLK_CON_MUX_MMC_CARD, Reg);
  for (Timeout = 0; Timeout < CMU_TIMEOUT; Timeout++) {
    if (!(MmioRead32 (CLK_CON_MUX_MMC_CARD) & CMU_BUSY_BIT))
      break;
  }

  MmioWrite32 (CLK_CON_GAT_MMC_CARD, CMU_GATE_MANUAL | CMU_GATE_CG_VAL);
}

EFI_STATUS
MmcPlatformInit (
  IN  UINT32              Channel,
  OUT MMC_PLATFORM_HOST  *Host
  )
{
  EFI_STATUS Status;

  if (!Host) {
    return EFI_INVALID_PARAMETER;
  }

  Status = MmcPowerSet ();
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "MMC%d: PMIC power set failed: %r\n", Channel, Status));
    return Status;
  }

  if (Channel == MMC_CHANNEL_EMMC) {
    Status = MmcGpioInitEmmc ();
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "MMC%d: GPIO init failed: %r\n", Channel, Status));
      return Status;
    }
    MmcClockInitEmmc ();

    Host->IoBase      = DW_MMC0_BASE;
    Host->SdrClksel   = EXYNOS9610_SDR_CLKSEL;
    Host->DdrClksel   = EXYNOS9610_DDR_CLKSEL;
    Host->Hs200Clksel = EXYNOS9610_HS200_CLKSEL;
    Host->Hs400Clksel = EXYNOS9610_HS400_CLKSEL;
    Host->BusHz       = MMC_BUS_HZ;
    Host->PhaseDivide = MMC_CIU_DIV;
    Host->Ciudiv      = MMC_CIUDIV;
    Host->FifoDepth   = MMC_FIFO_DEPTH;
	Host->Secure      = TRUE;
    Host->MpsSecurity = 0;

  } else if (Channel == MMC_CHANNEL_SD) {
    Status = MmcGpioInitSd ();
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "MMC%d: GPIO init failed: %r\n", Channel, Status));
      return Status;
    }
    MmcClockInitSd ();

    Host->IoBase       = DW_MMC2_BASE;
    Host->SdrClksel    = EXYNOS9610_SDR_CLKSEL;
    Host->DdrClksel    = EXYNOS9610_DDR_CLKSEL;
    Host->Sdr50Clksel  = EXYNOS9610_SDR50_CLKSEL;
    Host->Sdr104Clksel = EXYNOS9610_SDR104_CLKSEL;
    Host->Hs200Clksel  = 0;
    Host->Hs400Clksel  = 0;
    Host->BusHz        = MMC_BUS_HZ;
    Host->PhaseDivide  = MMC_CIU_DIV;
    Host->Ciudiv       = MMC_CIUDIV;
    Host->FifoDepth    = MMC_FIFO_DEPTH;
	Host->Secure       = FALSE;
    Host->MpsSecurity  = 0;

  } else {
    DEBUG ((EFI_D_ERROR, "MMC: Unknown channel: %d\n", Channel));
    return EFI_INVALID_PARAMETER;
  }

  return EFI_SUCCESS;
}
