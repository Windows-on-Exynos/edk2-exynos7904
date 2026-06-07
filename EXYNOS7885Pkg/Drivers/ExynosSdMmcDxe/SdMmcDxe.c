#include <Uefi.h>
#include <Library/DebugLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/CacheMaintenanceLib.h>
#include <Library/DevicePathLib.h>
#include <Library/DwMmcRegs.h>
#include <Library/MmcPlatformLib.h>
#include "SdMmcDxeInternal.h"
#include <Protocol/BlockIo.h>
#include <Protocol/DevicePath.h>

STATIC CONST UINT8 mTuningPattern4Bit[TUNING_BLOCK_SIZE] = {
  0xFF, 0x0F, 0xFF, 0x00, 0xFF, 0xCC, 0xC3, 0xCC,
  0xC3, 0x3C, 0xCC, 0xFF, 0xFE, 0xFF, 0xFE, 0xEF,
  0xFF, 0xDF, 0xFF, 0xDD, 0xFF, 0xFB, 0xFF, 0xFB,
  0xBF, 0xFF, 0x7F, 0xFF, 0x77, 0xF7, 0xBD, 0xEF,
  0xFF, 0xF0, 0xFF, 0xF0, 0x0F, 0xFC, 0xCC, 0x3C,
  0xCC, 0x33, 0xCC, 0xCF, 0xFF, 0xEF, 0xFF, 0xEE,
  0xFF, 0xFD, 0xFF, 0xFD, 0xDF, 0xFF, 0xBF, 0xFF,
  0xBB, 0xFF, 0xF7, 0xFF, 0xF7, 0x7F, 0x7B, 0xDE,
};

STATIC DW_MMC_HOST  mHost;
STATIC UINT32       mChannel = MMC_CHANNEL_SD;

#define RD(R)   MmioRead32 (mHost.IoBase + (R))
#define WR(R,V) MmioWrite32 (mHost.IoBase + (R), (V))
#define SET(R,V) WR (R, RD (R) | (V))
#define CLR(R,V) WR (R, RD (R) & ~(V))

STATIC
BOOLEAN
DwMmcResetCtrl (UINT32 Mask)
{
  UINT32 Timeout = 1000;
  SET (DWMCI_CTRL, Mask);
  while (Timeout--) {
    if (!(RD (DWMCI_CTRL) & Mask))
      return TRUE;
    MicroSecondDelay (10);
  }
  DEBUG ((EFI_D_ERROR, "DWMMC: reset_ctrl timeout mask=0x%x\n", Mask));
  return FALSE;
}

STATIC
BOOLEAN
DwMmcResetCtrlOnce (UINT32 Mask)
{
  UINT32 Timeout = 1000, ClkselReg;
  ClkselReg = RD (DWMCI_CLKSEL);
  CLR (DWMCI_CLKSEL, CLKSEL_SAMPLE_CLK_ALL);
  SET (DWMCI_CTRL, Mask);
  WR (DWMCI_RINTSTS, INT_ALL);
  while (Timeout--) {
    if (!(RD (DWMCI_CTRL) & Mask))
      break;
    MicroSecondDelay (10);
  }
  WR (DWMCI_CLKSEL, ClkselReg);
  return (Timeout > 0);
}

STATIC
VOID
DwMmcFifoInit (VOID)
{
  UINT32 Val, RxWmark, TxWmark, Threshold;

  if (!mHost.FifoDepth)
    mHost.FifoDepth = 0x20;

  Val       = RD (DWMCI_FIFOTH);
  Threshold = mHost.FifoDepth / 2;
  RxWmark   = ((Threshold - 1) << 16) & RX_WMARK;
  TxWmark   = Threshold & TX_WMARK;

  Val &= ~FIFOTH_ALL;
  Val |= (RxWmark | TxWmark | MSIZE_8);
  WR (DWMCI_FIFOTH, Val);
}

STATIC
EFI_STATUS
DwMmcCheckClockChange (VOID)
{
  UINT32 LoopCount, Retry;

  WR (DWMCI_CMD, 0);
  WR (DWMCI_CMD, CMD_ONLY_CLK);
  for (Retry = 10; Retry > 0; Retry--) {
    for (LoopCount = 1000; LoopCount > 0; LoopCount--) {
      if (!(RD (DWMCI_CMD) & CMD_STRT_BIT))
        return EFI_SUCCESS;
    }
    DwMmcResetCtrl (CTRL_RESET);
    WR (DWMCI_CMD, CMD_ONLY_CLK);
  }
  return EFI_TIMEOUT;
}

STATIC
EFI_STATUS
DwMmcControlClken (UINT32 Val)
{
  WR (DWMCI_CLKENA, Val);
  return DwMmcCheckClockChange ();
}

STATIC
UINT32
DwMmcGetClockDiv (UINT32 BoardClk, UINT32 TargetClk)
{
  UINT32 i;
  if (TargetClk >= BoardClk)
    return 0;
  for (i = 1; i <= 0xFF; i++) {
    if (TargetClk >= (BoardClk / (2 * i)))
      return i;
  }
  return 0xFF;
}

STATIC
EFI_STATUS
DwMmcChangeClock (UINT32 TargetClk)
{
  UINT32 BoardHz = mHost.BusHz / mHost.PhaseDivide;
  UINT32 Div;

  if (!BoardHz)
    BoardHz = mHost.BusHz;

  DwMmcControlClken (CLK_DISABLE);

  Div = DwMmcGetClockDiv (BoardHz, TargetClk);
  WR (DWMCI_CLKDIV, Div);
  DwMmcCheckClockChange ();

  if (EFI_ERROR (DwMmcControlClken (CLK_ENABLE)))
    return EFI_TIMEOUT;

  mHost.CardClock = (Div == 0) ? BoardHz : (BoardHz / (2 * Div));
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcSetIos (UINT32 Clock, UINT32 ClkselVal)
{
  UINT32 Reg;

  if (Clock < mHost.MinClock)
    Clock = mHost.MinClock;
  if (Clock > mHost.MaxClock)
    Clock = mHost.MaxClock;

  DwMmcChangeClock (Clock);

  if (mHost.BusWidth == 8)
    WR (DWMCI_CTYPE, CTYPE_8BIT);
  else if (mHost.BusWidth == 4)
    WR (DWMCI_CTYPE, CTYPE_4BIT);
  else
    WR (DWMCI_CTYPE, CTYPE_1BIT);

  if (mHost.BusMode == 1) {
    SET (DWMCI_UHS_REG, UHS_REG_DDR);
  } else if (mHost.BusMode == 3) {
    // HS400 — DDR + DQS strobe
    SET (DWMCI_UHS_REG, UHS_REG_DDR);
  } else {
    CLR (DWMCI_UHS_REG, UHS_REG_DDR);
  }

  Reg = ClkselVal ? ClkselVal : mHost.SdrClksel;

  if (mHost.CardClock <= 400000)
    Reg |= EXYNOS_CLKSEL_CCLK_DRIVE (7);

  if (mHost.IsTuned) {
    Reg = EXYNOS_CLKSEL_UP_SAMPLE (Reg, mHost.TunedSamplePhase);
    if ((mHost.TunedSamplePhase & 0x7) >= 6)
      SET (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
    else
      CLR (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
  }

  WR (DWMCI_CLKSEL, Reg);

  if (!mHost.IsTuned)
    CLR (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcSetIosSimple (UINT32 Clock)
{
  UINT32 ClkselVal;

  if (mHost.BusMode == 1)
    ClkselVal = mHost.DdrClksel;
  else
    ClkselVal = mHost.SdrClksel;

  return DwMmcSetIos (Clock, ClkselVal);
}

STATIC
BOOLEAN
DwMmcCheckCardPresent (UINT32 Channel)
{
  EFI_STATUS Status;
  BOOLEAN    Present;

  Status = MmcGetCardDetect (Channel, &Present);
  if (!EFI_ERROR (Status))
    return Present;

  Present = ((RD (DWMCI_CDETECT) & (1 << Channel)) == 0);
  DEBUG ((EFI_D_WARN, "DWMMC: CDETECT fallback ch%d=%s\n",
    Channel, Present ? "present" : "absent"));
  return Present;
}

STATIC
EFI_STATUS
DwMmcCheckDataBusy (UINT32 CmdIdx)
{
  UINT32 Timeout;

  if (CmdIdx == CMD13_SEND_STATUS)
    return EFI_SUCCESS;

  for (Timeout = 1000; Timeout > 0; Timeout--) {
    if (!(RD (DWMCI_STATUS) & DATA_BUSY)) {
      WR (DWMCI_RINTSTS, INT_ALL);
      return EFI_SUCCESS;
    }
    MicroSecondDelay (1000);
  }
  DEBUG ((EFI_D_ERROR, "DWMMC: data busy timeout\n"));
  return EFI_TIMEOUT;
}

STATIC
VOID
DwMmcSetIdmaDesc (
  DWMMC_IDMAC_DESC *Desc,
  UINT64            Control,
  UINT64            BufferSize,
  UINT64            BufferAddr
  )
{
  Desc->Des0 = (UINT32)Control;
  Desc->Des1 = 0;
  Desc->Des2 = (UINT32)BufferSize;
  Desc->Des3 = 0;
  Desc->Des4 = (UINT32)BufferAddr;
  Desc->Des5 = (UINT32)(BufferAddr >> 32);
  Desc->Des6 = (UINT32)((UINTN)(Desc + 1));
  Desc->Des7 = (UINT32)(((UINTN)(Desc + 1)) >> 32);
  Desc->Des8  = 0;
  Desc->Des9  = 0;
  Desc->Des10 = 0;
  Desc->Des11 = 0;
  Desc->Des12[0] = 0;
  Desc->Des12[1] = 0;
  Desc->Des12[2] = 0;
  Desc->Des12[3] = 0;
}

STATIC
VOID
DwMmcPrepareData (UINT32 BlockSize, UINT32 BlockCnt,
                  BOOLEAN IsWrite, VOID *Buf)
{
  DWMMC_IDMAC_DESC *Desc;
  UINT32            Flags, i, DataCnt, SendBytes, Reg;

  Reg = RD (DWMCI_FIFOTH);
  Reg &= ~MSIZE_MASK;
  if (BlockSize == 8)
    Reg |= MSIZE_1;
  else
    Reg |= MSIZE_8;
  WR (DWMCI_FIFOTH, Reg);

  DwMmcResetCtrl (FIFO_RESET);
  SET (DWMCI_BMOD, BMOD_IDMAC_RESET);

  ZeroMem (mDmaDesc, sizeof (mDmaDesc));

  mDmaBlockSize = BlockSize;
  mDmaBlockCnt  = BlockCnt;
  mDmaIsRead    = !IsWrite;
  mDmaBuffer    = Buf;

  Desc    = mDmaDesc;
  DataCnt = BlockCnt;

  for (i = 0; ; i++) {
    Flags = DWMCI_IDMAC_OWN | DWMCI_IDMAC_CH;
    if (i == 0)
      Flags |= DWMCI_IDMAC_FS;

    if (DataCnt <= 8) {
      Flags    |= DWMCI_IDMAC_LD;
      SendBytes = BlockSize * DataCnt;
      DwMmcSetIdmaDesc (Desc, Flags, SendBytes,
                        (UINT64)(UINTN)Buf + (UINT64)(i * 0x1000));
      break;
    }

    SendBytes = BlockSize * 8;
    DwMmcSetIdmaDesc (Desc, Flags, SendBytes,
                      (UINT64)(UINTN)Buf + (UINT64)(i * 0x1000));
    DataCnt -= 8;
    Desc++;
  }

  WriteBackDataCacheRange (mDmaDesc, sizeof (mDmaDesc));

  if (IsWrite) {
    WriteBackDataCacheRange (Buf, BlockSize * BlockCnt);
  } else {
    WriteBackDataCacheRange (Buf, BlockSize * BlockCnt);
    InvalidateDataCacheRange (Buf, BlockSize * BlockCnt);
  }

  WR (DWMCI_DBADDRL, (UINT32)(UINTN)mDmaDesc);
  WR (DWMCI_DBADDRU, (UINT32)((UINTN)mDmaDesc >> 32));

  CLR (DWMCI_CTRL, SEND_AS_CCSD);
  SET (DWMCI_CTRL, ENABLE_IDMAC | DMA_ENABLE);
  SET (DWMCI_BMOD, BMOD_IDMAC_ENABLE | BMOD_IDMAC_FB);

  WR (DWMCI_BLKSIZ, BlockSize);
  WR (DWMCI_BYTCNT, BlockSize * BlockCnt);
}

STATIC
UINT32
DwMmcReadyCmd (UINT32 CmdIdx, UINT32 Arg, UINT32 RespType,
               BOOLEAN HasData, BOOLEAN IsWrite)
{
  UINT32 Flag = 0;

  WR (DWMCI_CMDARG, Arg);

  if (RespType & MMC_RSP_CRC)
    Flag |= CMD_CHECK_CRC_BIT;
  if (RespType & MMC_RSP_PRESENT) {
    Flag |= CMD_RESP_EXP_BIT;
    if (RespType & MMC_RSP_136)
      Flag |= CMD_RESP_LENGTH_BIT;
  }

  Flag |= (CmdIdx | CMD_STRT_BIT | CMD_USE_HOLD_REG | CMD_WAIT_PRV_DAT_BIT);

  if (HasData) {
    Flag |= CMD_DATA_EXP_BIT;
    if (IsWrite)
      Flag |= CMD_RW_BIT;
  }

  return Flag;
}

STATIC
EFI_STATUS
DwMmcStartCmd (UINT32 Flag)
{
  UINT32 Mask, Timeout;

  for (Timeout = 0x200000; Timeout > 0; Timeout--) {
    if (!(RD (DWMCI_CMD) & CMD_STRT_BIT) &&
        !(RD (DWMCI_RINTSTS) & INT_CDONE))
      break;
  }
  if (!Timeout) {
    DEBUG ((EFI_D_ERROR, "DWMMC: prev cmd not cleared\n"));
    DwMmcResetCtrl (CTRL_RESET);
    return EFI_TIMEOUT;
  }

  WR (DWMCI_RINTSTS, CMD_STATUS);
  WR (DWMCI_CMD, Flag);

  for (Timeout = 0x200000; Timeout > 0; Timeout--) {
    if (!(RD (DWMCI_CMD) & CMD_STRT_BIT))
      break;
  }
  if (!Timeout) {
    DEBUG ((EFI_D_ERROR, "DWMMC: cmd start bit stuck\n"));
    DwMmcResetCtrl (CTRL_RESET);
    return EFI_TIMEOUT;
  }

  for (Timeout = 0x200000; Timeout > 0; Timeout--) {
    if (!(RD (DWMCI_STATUS) & CMD_FSMSTAT))
      break;
  }
  if (!Timeout) {
    DEBUG ((EFI_D_ERROR, "DWMMC: FSM stuck\n"));
    DwMmcResetCtrl (CTRL_RESET);
    return EFI_TIMEOUT;
  }

  for (Timeout = 0x200000; Timeout > 0; Timeout--) {
    Mask = RD (DWMCI_RINTSTS);
    if (Mask & INT_CDONE)
      break;
  }
  if (!Timeout) {
    DEBUG ((EFI_D_ERROR, "DWMMC: CMD_DONE timeout\n"));
    DwMmcResetCtrl (CTRL_RESET);
    return EFI_TIMEOUT;
  }

  if (Mask & INT_RTO) {
    DEBUG ((EFI_D_ERROR, "DWMMC: response timeout\n"));
    return EFI_TIMEOUT;
  }
  if (Mask & INT_RE) {
    DEBUG ((EFI_D_ERROR, "DWMMC: response error\n"));
    return EFI_DEVICE_ERROR;
  }
  if (Mask & INT_RCRC) {
    DEBUG ((EFI_D_ERROR, "DWMMC: CRC error\n"));
    return EFI_CRC_ERROR;
  }

  WR (DWMCI_RINTSTS, Mask & CMD_STATUS);

  return EFI_SUCCESS;
}

STATIC
VOID
DwMmcResponseParse (UINT32 RespType, UINT32 *OutResp)
{
  if (!(RespType & MMC_RSP_PRESENT))
    return;

  if (!OutResp)
    return;

  if (RespType & MMC_RSP_136) {
    OutResp[0] = RD (DWMCI_RESP3);
    OutResp[1] = RD (DWMCI_RESP2);
    OutResp[2] = RD (DWMCI_RESP1);
    OutResp[3] = RD (DWMCI_RESP0);
  } else {
    OutResp[0] = RD (DWMCI_RESP0);
  }
}

STATIC
VOID
DwMmcEndData (VOID)
{
  UINT32 Reg;

  CLR (DWMCI_CTRL, (DMA_ENABLE | ENABLE_IDMAC));
  DwMmcResetCtrl (DMA_RESET);
  DwMmcResetCtrl (FIFO_RESET);

  Reg = RD (DWMCI_BMOD);
  Reg &= ~(BMOD_IDMAC_FB | BMOD_IDMAC_ENABLE);
  Reg |= BMOD_IDMAC_RESET;
  WR (DWMCI_BMOD, Reg);

  WR (DWMCI_IDINTEN, 0);
}

STATIC
VOID
DwMmcSimpleRecovery (VOID)
{
  UINT32 Timeout;

  DwMmcResetCtrl (CTRL_RESET);
  DwMmcResetCtrl (FIFO_RESET);

  WR (DWMCI_RINTSTS, INT_ALL);
  WR (DWMCI_INTMSK,  0);

  DwMmcControlClken (CLK_DISABLE);

  WR (DWMCI_CMD, 0);
  WR (DWMCI_CMD, CMD_ONLY_CLK);
  for (Timeout = 1000; Timeout > 0; Timeout--) {
    if (!(RD (DWMCI_CMD) & CMD_STRT_BIT))
      break;
  }

  WR (DWMCI_CMD, RD (DWMCI_CMD) & ~CMD_SEND_CLK_ONLY);
  DwMmcControlClken (CLK_ENABLE);

  if (mHost.CardRca) {
    DwMmcSendCommand (CMD12_STOP_TRAN, 0,
      MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_BUSY, NULL);
  }
}

STATIC
EFI_STATUS
DwMmcDataTransfer (VOID)
{
  UINT32 Mask;
  UINT32 Timeout = 2000000;

  while (Timeout--) {
    Mask = RD (DWMCI_RINTSTS);

    if (Mask & (DATA_ERR | DATA_TOUT)) {
      DwMmcSimpleRecovery ();
      DwMmcEndData ();
      DEBUG ((EFI_D_ERROR, "DWMMC: data error RINTSTS=0x%08x\n", Mask));
      return EFI_DEVICE_ERROR;
    }

    if (Mask & INT_DTO) {
      UINT32 DmaTimeout = 50000;
      while (!(RD (DWMCI_IDSTS) & IDSTS_NIS) && DmaTimeout--)
        MicroSecondDelay (1);

      DwMmcEndData ();
      if (mDmaIsRead)
        InvalidateDataCacheRange (mDmaBuffer, mDmaBlockSize * mDmaBlockCnt);
      return EFI_SUCCESS;
    }

    MicroSecondDelay (1);
  }

  DEBUG ((EFI_D_ERROR, "DWMMC: data transfer sw timeout\n"));
  DwMmcEndData ();
  return EFI_TIMEOUT;
}

STATIC
EFI_STATUS
DwMmcSendCommandData (
  UINT32   CmdIdx,
  UINT32   Arg,
  UINT32   RespType,
  UINT32  *OutResp,
  BOOLEAN  IsWrite,
  UINT32   BlockSize,
  UINT32   BlockCnt,
  VOID    *Buf
  )
{
  EFI_STATUS Status;
  UINT32     Flag;

  Status = DwMmcCheckDataBusy (CmdIdx);
  if (EFI_ERROR (Status))
    return Status;

  if (BlockCnt)
    DwMmcPrepareData (BlockSize, BlockCnt, IsWrite, Buf);

  Flag = DwMmcReadyCmd (CmdIdx, Arg, RespType, BlockCnt > 0, IsWrite);

  Status = DwMmcStartCmd (Flag);
  if (EFI_ERROR (Status)) {
    if (BlockCnt)
      DwMmcEndData ();
    return Status;
  }

  DwMmcResponseParse (RespType, OutResp);

  if (BlockCnt) {
    Status = DwMmcDataTransfer ();
    if (EFI_ERROR (Status))
      return Status;
  }
  if (BlockCnt > 1) {
    UINT32 StopFlag = DwMmcReadyCmd (CMD12_STOP_TRAN, 0,
                       MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_BUSY,
                       FALSE, FALSE);
    DwMmcStartCmd (StopFlag);
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcSendCommand (
  UINT32   CmdIdx,
  UINT32   Arg,
  UINT32   RespType,
  UINT32  *OutResp
  )
{
  return DwMmcSendCommandData (CmdIdx, Arg, RespType, OutResp,
                               FALSE, 0, 0, NULL);
}

STATIC
EFI_STATUS
DwMmcSendAppCmd (UINT32 Rca)
{
  UINT32 Resp[4];
  return DwMmcSendCommand (CMD55_APP_CMD, Rca << 16,
           MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, Resp);
}

STATIC
EFI_STATUS
DwMmcGetCardStatus (UINT32 TimeoutMs, UINT32 *OutState)
{
  EFI_STATUS Status;
  UINT32     Resp[4];

  do {
    Status = DwMmcSendCommand (CMD13_SEND_STATUS, mHost.CardRca << 16,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, Resp);
    if (!EFI_ERROR (Status)) {
      if (Resp[0] & R1_STATUS_MASK) {
        DEBUG ((EFI_D_ERROR, "DWMMC: CMD13 err 0x%08x\n", Resp[0]));
        return EFI_DEVICE_ERROR;
      }
      if ((Resp[0] & R1_READY_FOR_DATA) &&
          ((Resp[0] & R1_CURRENT_STATE_MASK) != R1_STATE_PRG)) {
        *OutState = (Resp[0] & R1_CURRENT_STATE_MASK) >> 9;
        return EFI_SUCCESS;
      }
    }
    MicroSecondDelay (1000);
  } while (TimeoutMs--);

  return EFI_TIMEOUT;
}

STATIC
UINT32
change_endian (UINT32 Val)
{
  return ((Val & 0xFF) << 24) | ((Val & 0xFF00) << 8) |
         ((Val >> 8) & 0xFF00) | ((Val >> 24) & 0xFF);
}

STATIC
UINT32
DwMmcExtractBits (UINT32 *Value, UINT32 Start, UINT32 End)
{
  UINT32 Idx1 = Start / 32, Off = Start % 32, Ret = Value[Idx1] >> Off;
  UINT32 Idx2 = End / 32, Mask = (1 << (End - Start + 1)) - 1;
  if (Idx2 > Idx1)
    Ret |= (Value[Idx2] << (32 - Off));
  return Ret & Mask;
}

STATIC
UINT32
DwMmcExtractExtCsd (UINT8 *ExtCsd, UINT32 StartByte, UINT32 EndByte)
{
  return DwMmcExtractBits ((UINT32 *)ExtCsd, StartByte * 8, EndByte * 8 + 7);
}

STATIC
EFI_STATUS
DwMmcSwitchCmd (UINT8 Set, UINT8 Index, UINT8 Value)
{
  EFI_STATUS Status;
  UINT32     Arg, CardState;

  Arg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) | (Index << 16) | (Value << 8) | Set;

  Status = DwMmcSendCommand (CMD6_SWITCH_FUNC, Arg,
             MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE | MMC_RSP_BUSY, NULL);
  if (EFI_ERROR (Status))
    return Status;

  return DwMmcGetCardStatus (1000, &CardState);
}

STATIC
EFI_STATUS
DwMmcSetBusWidth (UINT32 BusWidth)
{
  EFI_STATUS Status;
  UINT32     ExtCsdBusWidth;

  switch (BusWidth) {
  case 1:
    mHost.BusWidth = 1;
    ExtCsdBusWidth = EXT_CSD_BUS_WIDTH_1BIT_SDR;
    WR (DWMCI_CTYPE, CTYPE_1BIT);
    break;
  case 4:
    if (mHost.BusMode == 1 || mHost.BusMode == 3) {
      ExtCsdBusWidth = EXT_CSD_BUS_WIDTH_4BIT_DDR;
    } else {
      ExtCsdBusWidth = EXT_CSD_BUS_WIDTH_4BIT_SDR;
    }
    mHost.BusWidth = 4;
    WR (DWMCI_CTYPE, CTYPE_4BIT);
    break;
  case 8:
    if (mHost.BusMode == 1 || mHost.BusMode == 3) {
      ExtCsdBusWidth = EXT_CSD_BUS_WIDTH_8BIT_DDR;
    } else {
      ExtCsdBusWidth = EXT_CSD_BUS_WIDTH_8BIT_SDR;
    }
    mHost.BusWidth = 8;
    WR (DWMCI_CTYPE, CTYPE_8BIT);
    break;
  default:
    return EFI_INVALID_PARAMETER;
  }

  if (mHost.CardType == CARD_TYPE_MMC) {
    Status = DwMmcSwitchCmd (MMC_SWITCH_MODE_WRITE_BYTE, EXT_CSD_BUS_WIDTH, ExtCsdBusWidth);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "DWMMC: EXT_CSD_BUS_WIDTH=%d failed: %r\n", ExtCsdBusWidth, Status));
      return Status;
    }
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcSetTimingAndBus (UINT32 Timing, UINT32 Clock, UINT32 BusWidth, UINT32 BusMode, UINT32 ClkselVal)
{
  EFI_STATUS Status;

  // Switch card timing via CMD6 (HS_TIMING)
  if (mHost.CardType == CARD_TYPE_MMC && Timing != 0xFF) {
    Status = DwMmcSwitchCmd (MMC_SWITCH_MODE_WRITE_BYTE, EXT_CSD_HS_TIMING, Timing);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: HS_TIMING=%d failed: %r\n", Timing, Status));
      return Status;
    }
  }

  // Set host controller IOS (clock, bus mode, bus width)
  mHost.BusMode = BusMode;
  mHost.MaxClock = Clock;
  mHost.CardClock = Clock;

  Status = DwMmcSetIos (Clock, ClkselVal);
  if (EFI_ERROR (Status))
    return Status;

  if (BusWidth == 1)
    WR (DWMCI_CTYPE, CTYPE_1BIT);
  else if (BusWidth == 4)
    WR (DWMCI_CTYPE, CTYPE_4BIT);
  else if (BusWidth == 8)
    WR (DWMCI_CTYPE, CTYPE_8BIT);

  mHost.BusWidth = BusWidth;

  MicroSecondDelay (5000);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcSdSwitchCmd (UINT32 Mode, UINT32 Group, UINT8 Value, UINT8 *Resp)
{
  UINT32 Arg;

  Arg  = (Mode << 31) | 0x00FFFFFF;
  Arg &= ~(0xF << (Group * 4));
  Arg |= (Value << (Group * 4));

  return DwMmcSendCommandData (CMD6_SWITCH_FUNC, Arg,
           MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE | MMC_RSP_BUSY,
           NULL, FALSE, 64, 1, Resp);
}

STATIC
EFI_STATUS
DwMmcSendExtCsd (UINT8 *Buf)
{
  UINT32 CardState;
  ZeroMem (Buf, 512);

  DwMmcGetCardStatus (1000, &CardState);

  return DwMmcSendCommandData (CMD8_SEND_EXT_CSD, 0,
           MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE,
           NULL, FALSE, 512, 1, Buf);
}

STATIC
EFI_STATUS
DwMmcExecuteTuning (UINT32 CmdIdx)
{
  EFI_STATUS Status;
  UINT32     ClkselBase, ClkselVal, Phase, Sample;
  UINT16     GoodBitmap;
  UINT32     GoodCount, BestStart, BestLen, Start, Len, Median;

  ClkselBase = RD (DWMCI_CLKSEL);
  GoodBitmap = 0;

  WR (DWMCI_CLKSEL, ClkselBase & ~0xFFU);
  CLR (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
  WR (DWMCI_CARDTHRCTL, (512 << 16) | 1);

  DEBUG ((EFI_D_WARN, "DWMMC: Tuning start (CMD%d), clksel=0x%08x\n",
    CmdIdx, ClkselBase));

  for (Phase = 0; Phase <= 15; Phase++) {
    Sample = Phase & 0x7;
    ClkselVal = (ClkselBase & ~0x7U) | Sample;

    if (Sample >= 6)
      SET (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
    else
      CLR (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);

    WR (DWMCI_CLKSEL, ClkselVal);

    ZeroMem (mTuningBuf, TUNING_BLOCK_SIZE);
    Status = DwMmcSendCommandData (CmdIdx, 0,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE,
               NULL, FALSE, TUNING_BLOCK_SIZE, 1, mTuningBuf);

    if (!EFI_ERROR (Status)) {
      if (CompareMem (mTuningBuf, (VOID *)mTuningPattern4Bit,
                      TUNING_BLOCK_SIZE) == 0) {
        GoodBitmap |= (1 << Phase);
      }
    }
  }

  WR (DWMCI_CARDTHRCTL, 0);
  WR (DWMCI_CLKSEL, ClkselBase);
  CLR (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);

  if (GoodBitmap == 0xFFFF) {
    BestStart = 0;
    BestLen   = 16;
    DEBUG ((EFI_D_WARN, "DWMMC: all 16 phases pass\n"));
    goto TuningCalcMedian;
  }

  {
    UINT32 GoodBitmap32 = GoodBitmap | ((UINT32)GoodBitmap << 16);
    UINT32 Phase32;

    BestStart = 0;
    BestLen   = 0;
    Start     = 0;
    Len       = 0;

    for (Phase32 = 0; Phase32 < 32; Phase32++) {
      if (GoodBitmap32 & (1U << Phase32)) {
        if (Len == 0)
          Start = Phase32;
        Len++;
      } else {
        if (Len > BestLen) {
          BestLen   = Len;
          BestStart = Start;
        }
        Len = 0;
      }
    }
    if (Len > BestLen) {
      BestLen   = Len;
      BestStart = Start;
    }
  }

TuningCalcMedian:
  GoodCount = BestLen;
  {
    INT32 MaskNum[8] = { 13, 11, 9, 7, 5, 4, 3, 0 };
    INT32 MaskBit[8] = {  9,  7, 6, 5, 3, 2, 1, 0 };
    INT32 i;
    INT32 Offset = 0;

    for (i = 0; i < 8; i++) {
      if ((INT32)GoodCount >= MaskNum[i]) {
        Offset = MaskBit[i];
        break;
      }
    }
    Median = (BestStart + (UINT32)Offset) % 16;
  }

  DEBUG ((EFI_D_WARN,
    "DWMMC: Tuning result: bitmap=0x%04x, best=%d-%d (%d phases), median=%d\n",
    GoodBitmap, BestStart, BestStart + GoodCount - 1, GoodCount, Median));

  if (GoodCount == 0) {
    DEBUG ((EFI_D_ERROR, "DWMMC: Tuning failed — no good phases\n"));
    return EFI_DEVICE_ERROR;
  }

  mHost.TunedSamplePhase = Median;
  mHost.IsTuned          = TRUE;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcConfigHs400 (VOID)
{
  UINT32 DqsEn, DlineCtrl;

  DqsEn = DATA_STROBE_EN |
          HS400_AXI_NON_BLOCKING_WRITE |
          HS400_DQS_EN_DEFAULT;

  WR (DWMCI_HS400_DQS_EN, DqsEn);

  DlineCtrl = HS400_DLINE_CTRL_DEFAULT;
  WR (DWMCI_HS400_DLINE_CTRL, DlineCtrl);

  DEBUG ((EFI_D_WARN, "DWMMC: HS400 regs: DQS_EN=0x%08x DLINE=0x%08x\n",
    DqsEn, DlineCtrl));
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcVoltageSwitch (VOID)
{
  EFI_STATUS Status;
  UINT32     Timeout;

  for (Timeout = 20; Timeout > 0; Timeout--) {
    if (RD (DWMCI_STATUS) & DATA_BUSY)
      break;
    MicroSecondDelay (1000);
  }
  if (!Timeout) {
    DEBUG ((EFI_D_WARN, "DWMMC: Voltage switch: DATA_BUSY not set (CMD11 may not have completed), continuing\n"));
  }

  WR (DWMCI_CLKENA, CLK_DISABLE);
  MicroSecondDelay (10000);  // 10ms

  Status = SdVoltageSwitch ();
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "DWMMC: SdVoltageSwitch failed: %r\n", Status));
    WR (DWMCI_CLKENA, CLK_ENABLE);
    return Status;
  }

  SET (DWMCI_UHS_REG, UHS_REG_V18);

  mHost.MaxClock = (mHost.BusHz / mHost.PhaseDivide) < 200000000
                   ? (mHost.BusHz / mHost.PhaseDivide)
                   : 200000000;

  MicroSecondDelay (10000);  // 10ms

  WR (DWMCI_CLKENA, CLK_ENABLE);
  MicroSecondDelay (1000);

  if (RD (DWMCI_STATUS) & DATA_BUSY) {
    DEBUG ((EFI_D_WARN, "DWMMC: Voltage switch — DATA_BUSY still set after 1.8V switch, continuing\n"));
  }

  DEBUG ((EFI_D_WARN, "DWMMC: Voltage switch to 1.8V complete, MaxClock=%d MHz\n",
    mHost.MaxClock / 1000000));
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcHostInit (VOID)
{
  mHost.Version = RD (DWMCI_VERID) & 0xFFFF;

  WR (DWMCI_PWREN, POWER_ENABLE);
  DwMmcCheckDataBusy (0);

  MicroSecondDelay (10000);

  DwMmcResetCtrlOnce (RESET_ALL);
  DwMmcFifoInit ();

  WR (DWMCI_RINTSTS, INT_ALL);
  WR (DWMCI_INTMSK,  0);

  WR (DWMCI_CLKSEL, mHost.SdrClksel);
  DwMmcSetIosSimple (400000);

  mHost.MinClock = 400000;
  mHost.MaxClock = 50000000;

  SET (DWMCI_CTRL, SEND_AS_CCSD);

  WR (DWMCI_DEBNCE, 0xFFFFF);
  WR (DWMCI_CTYPE,  CTYPE_1BIT);
  WR (DWMCI_BMOD,   BMOD_IDMAC_RESET);
  WR (DWMCI_IDINTEN, 0);
  WR (DWMCI_TMOUT,  0xFFFFFFFF);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcSetInitState (VOID)
{
  mHost.CardRca         = 0;
  mHost.BlockSize       = MMC_MAX_BLOCK_LEN;
  mHost.BusWidth        = 1;
  mHost.BusMode         = 0;
  mHost.CardClock       = 400000;
  mHost.CardPresent     = FALSE;
  mHost.IsSdhc          = FALSE;
  mHost.IsHighSpeed     = FALSE;
  mHost.InitDone        = FALSE;
  mHost.TunedSamplePhase = 0;
  mHost.IsTuned         = FALSE;
  mHost.IsFineTuned     = FALSE;

  DwMmcHostInit ();
  return DwMmcSetIosSimple (400000);
}

STATIC
EFI_STATUS
DwMmcSdInitCard (VOID)
{
  EFI_STATUS Status;
  UINT32     Resp[4], OcrArg, i;

  Status = DwMmcSendCommand (CMD0_GO_IDLE, 0, 0, NULL);
  if (EFI_ERROR (Status))
    return Status;
  MicroSecondDelay (2000);

  for (i = 0; i < 3; i++) {
    Status = DwMmcSendCommand (CMD8_SEND_IF_COND, SD_SEND_IF_COND_ARG,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, Resp);
    if (Status == EFI_SUCCESS) {
      if ((Resp[0] & 0xFFF) != 0x1AA)
        return EFI_DEVICE_ERROR;
      OcrArg = SD_HC_OCR;
      break;
    }
    MicroSecondDelay (1000);
  }

  if (Status != EFI_SUCCESS) {
    DEBUG ((EFI_D_WARN, "DWMMC: CMD8 failed, trying SDSC\n"));
    OcrArg = SD_OCR;
  }

  for (i = 0; i < 20; i++) {
    Status = DwMmcSendAppCmd (0);
    if (EFI_ERROR (Status))
      return Status;

    Status = DwMmcSendCommand (ACMD41_SD_SEND_OP, OcrArg,
               MMC_RSP_PRESENT, Resp);
    if (EFI_ERROR (Status))
      return Status;

    if (Resp[0] & OCR_BUSY) {
      mHost.OcR = Resp[0];
      mHost.CardType = (Resp[0] & OCR_HCS) ? CARD_TYPE_SDHC : CARD_TYPE_SD;
      mHost.IsSdhc   = (mHost.CardType == CARD_TYPE_SDHC);
      mHost.IsUhsSupported = (Resp[0] & OCR_S18R) ? TRUE :
                             (OcrArg == SD_HC_OCR);
      break;
    }
    MicroSecondDelay (1000);
  }

  if (i >= 20)
    return EFI_TIMEOUT;

  if (mHost.IsUhsSupported) {
    DEBUG ((EFI_D_WARN, "DWMMC: CMD11 voltage switch (early, after ACMD41)...\n"));
    Status = DwMmcSendCommand (CMD11_SWITCH_VOLTAGE, 0,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, Resp);
    if (!EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: CMD11 succeeded, executing voltage switch sequence\n"));
    } else {
      DEBUG ((EFI_D_WARN, "DWMMC: CMD11 failed (%r) — slot may already be at 1.8V, forcing voltage setup\n",
        Status));
    }
    Status = DwMmcVoltageSwitch ();
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: Voltage switch setup failed (%r), UHS modes will be unavailable\n",
        Status));
      mHost.IsUhsSupported = FALSE;
    }
  }

  DEBUG ((EFI_D_WARN,
    "DWMMC: SD card ready, OCR=0x%08x, SDHC=%d, UHS=%d\n",
    mHost.OcR, mHost.IsSdhc, mHost.IsUhsSupported));
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcEmmcInitCard (VOID)
{
  EFI_STATUS Status;
  UINT32     Resp[4], Retry;

  Status = DwMmcSendCommand (CMD0_GO_IDLE, 0, 0, NULL);
  if (EFI_ERROR (Status))
    return Status;
  MicroSecondDelay (2000);

  for (Retry = 0; Retry < 10000; Retry++) {
    Status = DwMmcSendCommand (CMD1_SEND_OP_COND,
               OCR_VOLT_27_36 | OCR_SEC_MODE,
               MMC_RSP_PRESENT, Resp);
    if (Status == EFI_SUCCESS) {
      mHost.OcR = Resp[0];
      mHost.CardType = (Resp[0] & OCR_SEC_MODE) ? CARD_TYPE_MMC : CARD_TYPE_MMC;
      mHost.IsSdhc = FALSE;
      break;
    }
    MicroSecondDelay (1000);
  }

  if (Retry >= 1000)
    return EFI_TIMEOUT;

  DEBUG ((EFI_D_WARN, "DWMMC: eMMC ready, OCR=0x%08x\n", mHost.OcR));
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcIdentifyCard (VOID)
{
  EFI_STATUS Status;
  UINT32     Resp[4], Csd[4];

  Status = DwMmcSendCommand (CMD2_ALL_SEND_CID, 0,
             MMC_RSP_PRESENT | MMC_RSP_136 | MMC_RSP_CRC, Resp);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "DWMMC: CMD2 failed\n"));
    return Status;
  }

  Status = DwMmcSendCommand (CMD3_SEND_REL_ADDR, 0,
             MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, Resp);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "DWMMC: CMD3 failed\n"));
    return Status;
  }

  mHost.CardRca = (Resp[0] >> 16) & 0xFFFF;
  DEBUG ((EFI_D_WARN, "DWMMC: RCA=0x%04x\n", mHost.CardRca));

  Status = DwMmcSendCommand (CMD9_SEND_CSD, mHost.CardRca << 16,
             MMC_RSP_PRESENT | MMC_RSP_136 | MMC_RSP_CRC, Csd);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "DWMMC: CMD9 failed\n"));
    return Status;
  }

  mHost.Csd[0] = Csd[3];  // RESP0 = bits  31:0
  mHost.Csd[1] = Csd[2];  // RESP1 = bits  63:32
  mHost.Csd[2] = Csd[1];  // RESP2 = bits  95:64
  mHost.Csd[3] = Csd[0];  // RESP3 = bits 127:96

  {
    UINT32 CidVal[4];
    CidVal[0] = Resp[3];
    CidVal[1] = Resp[2];
    CidVal[2] = Resp[1];
    CidVal[3] = Resp[0];  // Little-endian: MSW = CID[127:96]

    if (mHost.CardType == CARD_TYPE_MMC) {
      UINT8  Mmid = DwMmcExtractBits (CidVal, 120, 127);
      UINT32 Psn  = DwMmcExtractBits (CidVal, 8, 31);
      UINT16 Oid  = DwMmcExtractBits (CidVal, 104, 119);
      UINT8  Prv  = DwMmcExtractBits (CidVal, 96, 103);
      UINT8  Month = DwMmcExtractBits (CidVal, 0, 3);
      UINT8  Year  = DwMmcExtractBits (CidVal, 4, 7) + 1997;

      DEBUG ((EFI_D_WARN,
        "DWMMC: eMMC CID — MID=0x%02x OID=0x%04x PRV=%d.%d PSN=0x%08x %04d-%02d\n",
        Mmid, Oid, (Prv >> 4) & 0xF, Prv & 0xF, Psn, Year, Month));
    } else {
      UINT8  Mmid = DwMmcExtractBits (CidVal, 120, 127);
      UINT32 Psn  = DwMmcExtractBits (CidVal, 8, 31);
      UINT16 Oid  = DwMmcExtractBits (CidVal, 104, 119);
      UINT8  Prv  = DwMmcExtractBits (CidVal, 96, 103);
      UINT8  Month = DwMmcExtractBits (CidVal, 0, 3);
      UINT8  Year  = DwMmcExtractBits (CidVal, 4, 7) + 2000;

      DEBUG ((EFI_D_WARN,
        "DWMMC: SD CID — MID=0x%02x OID=%c%c PRV=%d.%d PSN=0x%08x %04d-%02d\n",
        Mmid, (Oid >> 8) & 0xFF, Oid & 0xFF,
        (Prv >> 4) & 0xF, Prv & 0xF, Psn, Year, Month));
    }
  }

  Status = DwMmcSendCommand (CMD7_SELECT_CARD, mHost.CardRca << 16,
             MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_BUSY, Resp);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "DWMMC: CMD7 failed\n"));
    return Status;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcDecodeMmcInfo (VOID)
{
  EFI_STATUS Status;
  UINT64     Capacity;

  mHost.Capacity  = 0;
  mHost.BootSize  = 0;
  mHost.RpmbSize  = 0;

  if (mHost.CardType == CARD_TYPE_MMC) {
    if ((mHost.Csd[3] >> 26) >= 4) {
      Status = DwMmcSendExtCsd (mExtCsdBuf);
      if (!EFI_ERROR (Status)) {
        Capacity = (UINT64)DwMmcExtractExtCsd (mExtCsdBuf,
                     EXT_CSD_SEC_CNT, EXT_CSD_SEC_CNT + 3);
        Capacity *= MMC_MAX_BLOCK_LEN;
        mHost.Capacity  = Capacity;
        mHost.NumBlocks = (UINT32)(Capacity / MMC_MAX_BLOCK_LEN);
        mHost.CardCaps  = mExtCsdBuf[EXT_CSD_CARD_TYPE];

        mHost.BootSize = mExtCsdBuf[EXT_CSD_BOOT_MULT] * 128 / 1024;
        mHost.RpmbSize = mExtCsdBuf[EXT_CSD_RPMB_MULT] * 128 / 1024;
      }
    }
  }

  mHost.BlockSize = MMC_MAX_BLOCK_LEN;

  DEBUG ((EFI_D_WARN,
    "DWMMC: eMMC capacity=%lld MB, boot=%d MB, rpmb=%d MB, caps=0x%02x\n",
    mHost.Capacity / (1024 * 1024),
    mHost.BootSize, mHost.RpmbSize, mHost.CardCaps));
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcDecodeSdInfo (VOID)
{
  EFI_STATUS Status;
  UINT64     CSize, CMult;
  UINT32     FuncGroup1, CsdVer;
  UINT8      SwitchStatus[64];
  UINT8      ScrData[8];

  mHost.Capacity  = 0;
  mHost.BlockSize = MMC_MAX_BLOCK_LEN;

  CsdVer = mHost.Csd[3] >> 30;

  if (CsdVer >= 1) {
    // SDHC/SDXC (CSD v2.0): 22-bit C_SIZE at concatenated bits [48:69]
    CSize = DwMmcExtractBits (mHost.Csd, 48, 69);
    if (CSize >= 0xFFFF)
      mHost.Capacity = CSize * 512 * 1024ULL;
    else
      mHost.Capacity = (CSize + 1) * 512 * 1024ULL;
  } else {
    // SDSC (CSD v1.0)
    UINT32 Rbl  = 1 << DwMmcExtractBits (mHost.Csd, 80, 83);
    CSize       = DwMmcExtractBits (mHost.Csd, 62, 73);
    CMult       = DwMmcExtractBits (mHost.Csd, 47, 49);
    mHost.Capacity = (CSize + 1) * (1ULL << (CMult + 2)) * Rbl;
  }
  mHost.NumBlocks = (UINT32)(mHost.Capacity / MMC_MAX_BLOCK_LEN);

  Status = DwMmcSendAppCmd (mHost.CardRca);
  if (!EFI_ERROR (Status)) {
    Status = DwMmcSendCommandData (ACMD51_SEND_SCR, 0,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE,
               NULL, FALSE, 8, 1, ScrData);
    if (!EFI_ERROR (Status)) {
      mHost.Scr[0] = change_endian (
                       (ScrData[0] << 24) | (ScrData[1] << 16) |
                       (ScrData[2] << 8)  |  ScrData[3]);
      mHost.Scr[1] = change_endian (
                       (ScrData[4] << 24) | (ScrData[5] << 16) |
                       (ScrData[6] << 8)  |  ScrData[7]);
    }
  }

  Status = DwMmcSdSwitchCmd (SD_SWITCH_MODE_CHECK, 0, 1,
             SwitchStatus);
  if (!EFI_ERROR (Status)) {
    FuncGroup1 = SwitchStatus[13];

    DEBUG ((EFI_D_WARN, "DWMMC: SD caps=0x%04x (b13=0x%02x)\n",
      FuncGroup1, SwitchStatus[13]));
    if (FuncGroup1 & 0x02) {
      mHost.IsHighSpeed = TRUE;
      mHost.CardCaps   |= SD_HS_SDR25;
      DEBUG ((EFI_D_WARN, "DWMMC: SD High Speed supported\n"));
    }
    if (FuncGroup1 & 0x1C) {  // Any UHS mode
      mHost.CardCaps |= SD_UHS_SDR50 | SD_UHS_SDR104;
      DEBUG ((EFI_D_WARN, "DWMMC: SD UHS-I modes supported (caps=0x%04x)\n",
        FuncGroup1));
    }
  }

  DEBUG ((EFI_D_WARN, "DWMMC: SD capacity=%lld MB, SCR=0x%08x, HS=%d\n",
    mHost.Capacity / (1024 * 1024), mHost.Scr[0], mHost.IsHighSpeed));
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcSdAdjustSpeed (VOID)
{
  EFI_STATUS Status;
  UINT32     Resp[4];

  DEBUG ((EFI_D_WARN, "DWMMC: Scr[0]=0x%08x Scr[1]=0x%08x BW4=%d\n",
    mHost.Scr[0], mHost.Scr[1],
    (mHost.Scr[0] & SCR_BUS_WIDTH_4) ? 1 : 0));

  mHost.BusMode = 0;   // SDR

  // Bus width: ACMD6
  if (mHost.Scr[0] & SCR_BUS_WIDTH_4) {
    Status = DwMmcSendAppCmd (mHost.CardRca);
    if (!EFI_ERROR (Status)) {
      Status = DwMmcSendCommand (ACMD6_SET_BUS_WIDTH, 2,
                 MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, Resp);
      if (!EFI_ERROR (Status)) {
        mHost.BusWidth = 4;
        DwMmcSetIosSimple (mHost.CardClock);
        DEBUG ((EFI_D_WARN, "DWMMC: ACMD6 4-bit OK\n"));
      } else {
        DEBUG ((EFI_D_ERROR, "DWMMC: ACMD6 4-bit failed: %r\n", Status));
      }
    } else {
      DEBUG ((EFI_D_ERROR, "DWMMC: CMD55 before ACMD6 failed: %r\n", Status));
    }
  } else {
    DEBUG ((EFI_D_WARN, "DWMMC: SD card does not support 4-bit\n"));
  }

  if (mHost.CardCaps & SD_UHS_SDR104) {
    UINT8 SwSt[64];

    DwMmcSdSwitchCmd (SD_SWITCH_MODE_SWITCH, 2, 0, SwSt);   // Group 3: Driver Strength = Type B
    DwMmcSdSwitchCmd (SD_SWITCH_MODE_SWITCH, 3, 3, SwSt);   // Group 4: Current Limit
    DwMmcSdSwitchCmd (SD_SWITCH_MODE_SWITCH, 0, 3, SwSt);   // Group 1: Access Mode

    if ((SwSt[16] & 0xF) != 3) {
      DEBUG ((EFI_D_WARN,
        "DWMMC: SDR104 NOT accepted (sts[16]=%d), fallback\n",
        SwSt[16] & 0xF));
      mHost.CardCaps &= ~SD_UHS_SDR104;
      goto SdTrySdr50;
    }

    mHost.MaxClock  = 200000000;
    mHost.CardClock = 200000000;
    DwMmcSetIos (200000000, mHost.Sdr104Clksel);
    MicroSecondDelay (5000);

    Status = DwMmcExecuteTuning (CMD19_SEND_TUNING);
    if (!EFI_ERROR (Status)) {
      {
        UINT32 Reg = mHost.Sdr104Clksel;
        Reg = EXYNOS_CLKSEL_UP_SAMPLE (Reg, mHost.TunedSamplePhase);
        if ((mHost.TunedSamplePhase & 0x7) >= 6)
          SET (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
        else
          CLR (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
        WR (DWMCI_CLKSEL, Reg);
      }
      DEBUG ((EFI_D_WARN, "DWMMC: UHS SDR104 @ %d MHz\n",
        mHost.CardClock / 1000000));
      goto SdSpeedDone;
    }

    DEBUG ((EFI_D_WARN, "DWMMC: SDR104 tuning failed, falling to SDR50\n"));
    mHost.CardCaps &= ~SD_UHS_SDR104;
  }

SdTrySdr50:
  if (mHost.CardCaps & SD_UHS_SDR50) {
    UINT8 SwSt[64];

    DwMmcSdSwitchCmd (SD_SWITCH_MODE_SWITCH, 2, 0, SwSt);   // Group 3: Driver Strength = Type B
    DwMmcSdSwitchCmd (SD_SWITCH_MODE_SWITCH, 3, 2, SwSt);   // Group 4: Current Limit
    DwMmcSdSwitchCmd (SD_SWITCH_MODE_SWITCH, 0, 2, SwSt);   // Group 1: Access Mode

    if ((SwSt[16] & 0xF) != 2) {
      DEBUG ((EFI_D_WARN,
        "DWMMC: SDR50 NOT accepted (sts[16]=%d), fallback to HS\n",
        SwSt[16] & 0xF));
      mHost.CardClock = mHost.IsHighSpeed ? 50000000 : 25000000;
      DwMmcSetIosSimple (mHost.CardClock);
      goto SdSpeedDone;
    }

    mHost.MaxClock  = 100000000;
    mHost.CardClock = 100000000;
    DwMmcSetIos (100000000, mHost.Sdr50Clksel);
    MicroSecondDelay (5000);

    Status = DwMmcExecuteTuning (CMD19_SEND_TUNING);
    if (!EFI_ERROR (Status)) {
      {
        UINT32 Reg = mHost.Sdr50Clksel;
        Reg = EXYNOS_CLKSEL_UP_SAMPLE (Reg, mHost.TunedSamplePhase);
        if ((mHost.TunedSamplePhase & 0x7) >= 6)
          SET (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
        else
          CLR (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
        WR (DWMCI_CLKSEL, Reg);
      }
      DEBUG ((EFI_D_WARN, "DWMMC: UHS SDR50 @ %d MHz\n",
        mHost.CardClock / 1000000));
      goto SdSpeedDone;
    }

    DEBUG ((EFI_D_WARN, "DWMMC: SDR50 tuning failed, falling to HS\n"));
  }

  // HS (50 MHz) or DS (25 MHz)
  mHost.CardClock = mHost.IsHighSpeed ? 50000000 : 25000000;
  DwMmcSetIosSimple (mHost.CardClock);

SdSpeedDone:
  DEBUG ((EFI_D_WARN, "DWMMC: SD speed: %s %d-bit %d MHz\n",
    mHost.IsHighSpeed ? "HS" : "DS",
    mHost.BusWidth, mHost.CardClock / 1000000));
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcEmmcAdjustSpeed (VOID)
{
  EFI_STATUS Status;
  BOOLEAN    Tuned = FALSE;

  mHost.BusMode = 0;           // SDR
  mHost.CardClock = 26000000;
  WR (DWMCI_CTYPE, CTYPE_8BIT);
  mHost.BusWidth = 8;
  DwMmcSetBusWidth (8);
  DwMmcSetIosSimple (26000000);

  if (mHost.CardCaps & MMC_HS_400MHZ_DDR) {
    Status = DwMmcSetTimingAndBus (EXT_CSD_HS_TIMING_HS, 52000000, 8, 1,
               mHost.DdrClksel);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: HS400: HS timing failed (%r)\n", Status));
      goto TryHs200;
    }
    Status = DwMmcSetBusWidth (8);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: HS400: BUS_WIDTH=8BIT_DDR failed (%r)\n", Status));
      goto TryHs200;
    }

    Status = DwMmcSetTimingAndBus (EXT_CSD_HS_TIMING_HS200, 200000000, 8, 2,
               mHost.Hs200Clksel);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: HS400: HS200 timing failed (%r)\n", Status));
      goto TryDdr52;
    }
    Status = DwMmcSetBusWidth (8);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: HS400: BUS_WIDTH=8BIT_SDR failed (%r)\n", Status));
      goto TryDdr52;
    }

    Status = DwMmcExecuteTuning (CMD21_SEND_TUNING_BLOCK);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: HS400: HS200 tuning failed (%r)\n", Status));
      goto TryDdr52;
    }
    Tuned = TRUE;

    {
      UINT32 Reg = mHost.Hs200Clksel;
      Reg = EXYNOS_CLKSEL_UP_SAMPLE (Reg, mHost.TunedSamplePhase);
      if ((mHost.TunedSamplePhase & 0x7) >= 6)
        SET (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
      else
        CLR (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
      WR (DWMCI_CLKSEL, Reg);
    }

    mHost.IsTuned = FALSE;
    Status = DwMmcSetTimingAndBus (EXT_CSD_HS_TIMING_HS, 52000000, 8, 1,
               mHost.DdrClksel);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: HS400: back-to-HS failed (%r), staying at HS200\n", Status));
      mHost.IsTuned = TRUE;
      // Re-apply tuned CLKSEL for HS200
      {
        UINT32 Reg = mHost.Hs200Clksel;
        Reg = EXYNOS_CLKSEL_UP_SAMPLE (Reg, mHost.TunedSamplePhase);
        if ((mHost.TunedSamplePhase & 0x7) >= 6)
          SET (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
        else
          CLR (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
        WR (DWMCI_CLKSEL, Reg);
      }
      mHost.BusMode  = 2;
      mHost.MaxClock  = 200000000;
      mHost.CardClock = 200000000;
      goto EmmcspeedDone;
    }
    Status = DwMmcSetBusWidth (8);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: HS400: BUS_WIDTH=8BIT_DDR post-HS failed (%r)\n", Status));
    }

    Status = DwMmcSetTimingAndBus (EXT_CSD_HS_TIMING_HS400, 200000000, 8, 3,
               mHost.Hs400Clksel);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: HS400: HS400 timing failed (%r), staying at HS200\n", Status));
      mHost.IsTuned = TRUE;
      {
        UINT32 Reg = mHost.Hs200Clksel;
        Reg = EXYNOS_CLKSEL_UP_SAMPLE (Reg, mHost.TunedSamplePhase);
        if ((mHost.TunedSamplePhase & 0x7) >= 6)
          SET (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
        else
          CLR (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
        WR (DWMCI_CLKSEL, Reg);
      }
      mHost.BusMode  = 2;
      mHost.MaxClock  = 200000000;
      mHost.CardClock = 200000000;
      goto EmmcspeedDone;
    }

    DwMmcConfigHs400 ();

    Status = DwMmcSetBusWidth (8);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: HS400: final BUS_WIDTH=8BIT_DDR failed (%r)\n", Status));
    }

    mHost.IsTuned = TRUE;

    DEBUG ((EFI_D_WARN, "DWMMC: eMMC HS400 @ 200 MHz DDR 8-bit\n"));
    goto EmmcspeedDone;
  }

TryHs200:
  if (mHost.CardCaps & MMC_HS_200MHZ_SDR) {
    Status = DwMmcSetTimingAndBus (EXT_CSD_HS_TIMING_HS200, 200000000, 8, 2,
               mHost.Hs200Clksel);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: HS200 timing failed (%r)\n", Status));
      goto TryDdr52;
    }
    Status = DwMmcSetBusWidth (8);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: HS200 BUS_WIDTH=8BIT_SDR failed (%r)\n", Status));
      goto TryDdr52;
    }

    //
    // Execute tuning at HS200
    //
    Status = DwMmcExecuteTuning (CMD21_SEND_TUNING_BLOCK);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: HS200 tuning failed (%r)\n", Status));
      goto TryDdr52;
    }

    {
      UINT32 Reg = mHost.Hs200Clksel;
      Reg = EXYNOS_CLKSEL_UP_SAMPLE (Reg, mHost.TunedSamplePhase);
      if ((mHost.TunedSamplePhase & 0x7) >= 6)
        SET (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
      else
        CLR (DWMCI_AXI_BURST_LEN, AXI_SAMPLING_PATH_SEL);
      WR (DWMCI_CLKSEL, Reg);
    }

    mHost.IsTuned = TRUE;
    DEBUG ((EFI_D_WARN, "DWMMC: eMMC HS200 @ 200 MHz SDR 8-bit\n"));
    goto EmmcspeedDone;
  }

TryDdr52:
  if (mHost.CardCaps & (MMC_HS_52MHZ_DDR_1_8V | MMC_HS_52MHZ_DDR_1_2V)) {
    Status = DwMmcSetTimingAndBus (EXT_CSD_HS_TIMING_HS, 52000000, 8, 1,
               mHost.DdrClksel);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: DDR52 timing failed (%r)\n", Status));
    } else {
      Status = DwMmcSetBusWidth (8);
      if (EFI_ERROR (Status)) {
        DEBUG ((EFI_D_WARN, "DWMMC: DDR52 BUS_WIDTH=8BIT_DDR failed (%r)\n", Status));
      }
      DEBUG ((EFI_D_WARN, "DWMMC: eMMC DDR52 @ 52 MHz DDR 8-bit\n"));
      goto EmmcspeedDone;
    }
  }

  if (mHost.CardCaps & MMC_HS_52MHZ) {
    Status = DwMmcSetTimingAndBus (EXT_CSD_HS_TIMING_HS, 52000000, 8, 0,
               mHost.SdrClksel);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "DWMMC: HS52 timing failed (%r)\n", Status));
    } else {
      Status = DwMmcSetBusWidth (8);
      if (EFI_ERROR (Status)) {
        DEBUG ((EFI_D_WARN, "DWMMC: HS52 BUS_WIDTH=8BIT_SDR failed (%r)\n", Status));
      }
      DEBUG ((EFI_D_WARN, "DWMMC: eMMC HS52 @ 52 MHz SDR 8-bit\n"));
      goto EmmcspeedDone;
    }
  }

  {
    Status = DwMmcSetTimingAndBus (0xFF, 26000000, 8, 0, mHost.SdrClksel);
    if (!EFI_ERROR (Status)) {
      Status = DwMmcSetBusWidth (8);
      if (EFI_ERROR (Status)) {
        DEBUG ((EFI_D_WARN, "DWMMC: Default BUS_WIDTH=8BIT_SDR failed (%r)\n", Status));
      }
    }
    DEBUG ((EFI_D_WARN, "DWMMC: eMMC default @ 26 MHz SDR 8-bit\n"));
  }

EmmcspeedDone:
  DEBUG ((EFI_D_WARN, "DWMMC: eMMC final: %d MHz %s %d-bit%s\n",
    mHost.CardClock / 1000000,
    (mHost.BusMode == 1 || mHost.BusMode == 3) ? "DDR" : "SDR",
    mHost.BusWidth,
    (mHost.BusMode == 2) ? " HS200" :
    (mHost.BusMode == 3) ? " HS400" : ""));
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcInitAndIdentify (VOID)
{
  EFI_STATUS Status;

  Status = DwMmcSetInitState ();
  if (EFI_ERROR (Status))
    return Status;

  if (mChannel == MMC_CHANNEL_SD) {
    Status = DwMmcSdInitCard ();
  } else {
    Status = DwMmcEmmcInitCard ();
  }

  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_WARN, "DWMMC: %s init failed: %r\n",
      (mChannel == MMC_CHANNEL_SD) ? "SD" : "eMMC", Status));
    return Status;
  }

  Status = DwMmcIdentifyCard ();
  if (EFI_ERROR (Status))
    return Status;

  if (mHost.CardType == CARD_TYPE_SD || mHost.CardType == CARD_TYPE_SDHC) {
    DwMmcDecodeSdInfo ();
    DwMmcSdAdjustSpeed ();
  } else {
    DwMmcDecodeMmcInfo ();
    DwMmcEmmcAdjustSpeed ();

    // Enable RST_n
    DwMmcSwitchCmd (MMC_SWITCH_MODE_WRITE_BYTE,
      EXT_CSD_RST_N_FUNCTION, EXT_CSD_RST_N_ENABLE);

    //
    // Configure boot bus width: x1 in boot, x8 in runtime (0x22)
    // Select user partition for normal I/O (PART_CONF = 0)
    // Enable erase group definition for enhanced erase
    //
    DwMmcSwitchCmd (MMC_SWITCH_MODE_WRITE_BYTE,
      EXT_CSD_BOOT_BUS_WIDTH, 0x22);
    DwMmcSwitchCmd (MMC_SWITCH_MODE_WRITE_BYTE,
      EXT_CSD_PART_CONF, 0x00);
    DwMmcSwitchCmd (MMC_SWITCH_MODE_WRITE_BYTE,
      EXT_CSD_ERASE_GROUP_DEF, 0x01);
  }

  {
    UINT32 Resp[4];
    Status = DwMmcSendCommand (CMD16_SET_BLOCKLEN, 512,
               MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, Resp);
    if (EFI_ERROR (Status) || (Resp[0] & R1_BLOCK_LEN_ERROR)) {
      DEBUG ((EFI_D_ERROR, "DWMMC: CMD16 failed\n"));
      return EFI_DEVICE_ERROR;
    }
  }

  mHost.CardPresent = TRUE;
  mHost.InitDone    = TRUE;

  DEBUG ((EFI_D_WARN, "DWMMC: Init complete — %s, %u blocks, %d-bit, %d MHz\n",
    (mHost.CardType == CARD_TYPE_SD || mHost.CardType == CARD_TYPE_SDHC) ?
      "SD" : "eMMC",
    mHost.NumBlocks, mHost.BusWidth, mHost.CardClock / 1000000));
  return EFI_SUCCESS;
}

typedef struct {
  EFI_BLOCK_IO_PROTOCOL  BlockIo;
  UINTN                  Signature;
  EFI_BLOCK_IO_MEDIA     Media;
} DW_BLKDEV;

#define DW_SIG  SIGNATURE_32 ('D','W','S','D')
STATIC DW_BLKDEV *mDev;

EFI_GUID gExynosSdCardGuid = EXYNOS_SD_CARD_GUID;

STATIC EFI_STATUS EFIAPI
BlkReset (EFI_BLOCK_IO_PROTOCOL *This, BOOLEAN ExtendedVerification)
{
  return EFI_SUCCESS;
}

STATIC EFI_STATUS EFIAPI
BlkRead (
  IN  EFI_BLOCK_IO_PROTOCOL *This,
  IN  UINT32                 MediaId,
  IN  EFI_LBA                Lba,
  IN  UINTN                  BufferSize,
  OUT VOID                  *Buffer
  )
{
  DW_BLKDEV *Dev = (DW_BLKDEV *)This;
  UINT32     N, Arg;

  if (MediaId != Dev->Media.MediaId)
    return EFI_MEDIA_CHANGED;
  if (!BufferSize || !Buffer)
    return EFI_SUCCESS;

  N   = (UINT32)(BufferSize / 512);
  Arg = mHost.IsSdhc ? (UINT32)Lba : (UINT32)(Lba * 512);

  if (N == 1)
    return DwMmcSendCommandData (CMD17_READ_SINGLE, Arg,
             MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE,
             NULL, FALSE, 512, 1, Buffer);

  if (mHost.CardType == CARD_TYPE_MMC) {
    DwMmcSendCommand (CMD23_SET_BLKCNT, N,
      MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, NULL);
  }
  return DwMmcSendCommandData (CMD18_READ_MULTI, Arg,
           MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE,
           NULL, FALSE, 512, N, Buffer);
}

STATIC EFI_STATUS EFIAPI
BlkWrite (
  IN EFI_BLOCK_IO_PROTOCOL *This,
  IN UINT32                 MediaId,
  IN EFI_LBA                Lba,
  IN UINTN                  BufferSize,
  IN VOID                  *Buffer
  )
{
  DW_BLKDEV *Dev = (DW_BLKDEV *)This;
  UINT32     N, Arg;

  if (MediaId != Dev->Media.MediaId)
    return EFI_MEDIA_CHANGED;
  if (!BufferSize || !Buffer)
    return EFI_SUCCESS;

  N   = (UINT32)(BufferSize / 512);
  Arg = mHost.IsSdhc ? (UINT32)Lba : (UINT32)(Lba * 512);

  if (N == 1)
    return DwMmcSendCommandData (CMD24_WRITE_SINGLE, Arg,
             MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE,
             NULL, TRUE, 512, 1, Buffer);

  if (mHost.CardType == CARD_TYPE_MMC) {
    DwMmcSendCommand (CMD23_SET_BLKCNT, N,
      MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE, NULL);
  }
  return DwMmcSendCommandData (CMD25_WRITE_MULTI, Arg,
           MMC_RSP_PRESENT | MMC_RSP_CRC | MMC_RSP_OPCODE,
           NULL, TRUE, 512, N, Buffer);
}

STATIC EFI_STATUS EFIAPI
BlkFlush (IN EFI_BLOCK_IO_PROTOCOL *This)
{
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
InitDwMmcDriver (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS         Status;
  EFI_HANDLE         Handle = NULL;
  MMC_PLATFORM_HOST  Plat;

  Status = MmcPlatformInit (MMC_CHANNEL_SD, &Plat);
  if (!EFI_ERROR (Status)) {
    mChannel = MMC_CHANNEL_SD;
    ZeroMem (&mHost, sizeof (mHost));
    mHost.IoBase       = Plat.IoBase;
    mHost.SdrClksel    = Plat.SdrClksel;
    mHost.DdrClksel    = Plat.DdrClksel;
    mHost.Sdr50Clksel  = Plat.Sdr50Clksel;
    mHost.Sdr104Clksel = Plat.Sdr104Clksel;
    mHost.Hs200Clksel  = Plat.Hs200Clksel;
    mHost.Hs400Clksel  = Plat.Hs400Clksel;
    mHost.BusHz        = Plat.BusHz;
    mHost.PhaseDivide  = Plat.PhaseDivide;
    mHost.Ciudiv       = Plat.Ciudiv;
    mHost.FifoDepth    = Plat.FifoDepth;

    DEBUG ((EFI_D_WARN, "DWMMC: Trying SD @ 0x%lx Bus=%d MHz\n",
      mHost.IoBase, Plat.BusHz / 1000000));

    Status = DwMmcInitAndIdentify ();
  }

  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_WARN, "DWMMC: SD init failed (%r), trying eMMC...\n", Status));

    Status = MmcPlatformInit (MMC_CHANNEL_EMMC, &Plat);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "DWMMC: eMMC platform init failed: %r\n", Status));
      return Status;
    }

    mChannel = MMC_CHANNEL_EMMC;
    ZeroMem (&mHost, sizeof (mHost));
    mHost.IoBase       = Plat.IoBase;
    mHost.SdrClksel    = Plat.SdrClksel;
    mHost.DdrClksel    = Plat.DdrClksel;
    mHost.Sdr50Clksel  = Plat.Sdr50Clksel;
    mHost.Sdr104Clksel = Plat.Sdr104Clksel;
    mHost.Hs200Clksel  = Plat.Hs200Clksel;
    mHost.Hs400Clksel  = Plat.Hs400Clksel;
    mHost.BusHz        = Plat.BusHz;
    mHost.PhaseDivide  = Plat.PhaseDivide;
    mHost.Ciudiv       = Plat.Ciudiv;
    mHost.FifoDepth    = Plat.FifoDepth;

    DEBUG ((EFI_D_WARN, "DWMMC: Trying eMMC @ 0x%lx Bus=%d MHz\n",
      mHost.IoBase, Plat.BusHz / 1000000));

    if (DwMmcCheckCardPresent (MMC_CHANNEL_EMMC))
      Status = DwMmcInitAndIdentify ();
    else {
      DEBUG ((EFI_D_WARN, "DWMMC: eMMC channel has no card (UFS platform)\n"));
      Status = EFI_NOT_FOUND;
    }

    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "DWMMC: eMMC init failed: %r\n", Status));
      return Status;
    }
  }

  mDev = AllocateZeroPool (sizeof (DW_BLKDEV));
  if (!mDev)
    return EFI_OUT_OF_RESOURCES;

  mDev->Signature             = DW_SIG;
  mDev->Media.MediaId         = 1;
  mDev->Media.RemovableMedia  = (mHost.CardType == CARD_TYPE_SD ||
                                 mHost.CardType == CARD_TYPE_SDHC);
  mDev->Media.MediaPresent    = mHost.CardPresent;
  mDev->Media.BlockSize       = mHost.BlockSize;
  mDev->Media.LastBlock       = mHost.NumBlocks > 0 ?
                                  (mHost.NumBlocks - 1) : 0;
  mDev->BlockIo.Revision      = EFI_BLOCK_IO_PROTOCOL_REVISION2;
  mDev->BlockIo.Media         = &mDev->Media;
  mDev->BlockIo.Reset         = BlkReset;
  mDev->BlockIo.ReadBlocks    = BlkRead;
  mDev->BlockIo.WriteBlocks   = BlkWrite;
  mDev->BlockIo.FlushBlocks   = BlkFlush;

  {
    EXYNOS_SD_DEVICE_PATH *Dp = AllocateZeroPool (sizeof (EXYNOS_SD_DEVICE_PATH));
    if (!Dp) {
      FreePool (mDev);
      return EFI_OUT_OF_RESOURCES;
    }

    Dp->VendorDp.Header.Type    = HARDWARE_DEVICE_PATH;
    Dp->VendorDp.Header.SubType = HW_VENDOR_DP;
    SetDevicePathNodeLength (&Dp->VendorDp.Header,
      sizeof (VENDOR_DEVICE_PATH) + sizeof (UINT8));
    CopyMem (&Dp->VendorDp.Guid, &gExynosSdCardGuid, sizeof (EFI_GUID));
    Dp->Slot = (UINT8)mChannel;
    SetDevicePathEndNode (&Dp->End);

    Status = gBS->InstallMultipleProtocolInterfaces (
                    &Handle,
                    &gEfiBlockIoProtocolGuid, &mDev->BlockIo,
                    &gEfiDevicePathProtocolGuid, Dp,
                    NULL);
    if (EFI_ERROR (Status)) {
      FreePool (mDev);
      FreePool (Dp);
      return Status;
    }
  }

  DEBUG ((EFI_D_INFO, "DWMMC: %a DiskIO Protocol Installed (%u blocks)\n",
    mChannel == MMC_CHANNEL_EMMC ? "eMMC" : "SD card", mHost.NumBlocks));
  return EFI_SUCCESS;
}