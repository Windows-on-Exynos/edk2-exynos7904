/** @file
  Exynos 9610 UFS HCI Core Driver.
  Adapted from Samsung lk3rd:
    dev/scsi/ufs.c           - core UFS/SCSI logic
    platform/exynos9610/ufs.c - platform board init

  Copyright (c) 2024, EDK2 Port Authors.
  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include "ExynosUfs.h"

/* ─── Query parameter table (mirrors LK ufs_query_params[][5]) ───────────────
   Columns: [function, opcode, idn, index, selector] */
u8 gQueryParams[][5] = {
  /* [0] unused */
  {0,                    0,                              0,                           0, 0},
  /* FLAG_W_FDEVICEINIT */
  {UFS_STD_WRITE_REQ,    UPIU_QUERY_OPCODE_SET_FLAG,     UPIU_FLAG_ID_DEVICEINIT,     0, 0},
  /* FLAG_R_FDEVICEINIT */
  {UFS_STD_READ_REQ,     UPIU_QUERY_OPCODE_READ_FLAG,    UPIU_FLAG_ID_DEVICEINIT,     0, 0},
  /* DESC_R_DEVICE_DESC */
  {UFS_STD_READ_REQ,     UPIU_QUERY_OPCODE_READ_DESC,    UPIU_DESC_ID_DEVICE,         0, 0},
  /* DESC_W_CONFIG_DESC */
  {UFS_STD_WRITE_REQ,    UPIU_QUERY_OPCODE_WRITE_DESC,   UPIU_DESC_ID_CONFIGURATION,  0, 0},
  /* DESC_R_CONFIG_DESC */
  {UFS_STD_READ_REQ,     UPIU_QUERY_OPCODE_READ_DESC,    UPIU_DESC_ID_CONFIGURATION,  0, 0},
  /* DESC_R_UNIT_DESC */
  {UFS_STD_READ_REQ,     UPIU_QUERY_OPCODE_READ_DESC,    UPIU_DESC_ID_UNIT,           0, 0},
  /* DESC_R_GEOMETRY_DESC */
  {UFS_STD_READ_REQ,     UPIU_QUERY_OPCODE_READ_DESC,    UPIU_DESC_ID_GEOMETRY,       0, 0},
  /* ATTR_W_BOOTLUNEN */
  {UFS_STD_WRITE_REQ,    UPIU_QUERY_OPCODE_WRITE_ATTR,   UPIU_ATTR_ID_BOOTLUNEN,      0, 0},
  /* ATTR_R_BOOTLUNEN */
  {UFS_STD_READ_REQ,     UPIU_QUERY_OPCODE_READ_ATTR,    UPIU_ATTR_ID_BOOTLUNEN,      0, 0},
  /* ATTR_W_REFCLKFREQ */
  {UFS_STD_WRITE_REQ,    UPIU_QUERY_OPCODE_WRITE_ATTR,   UPIU_ATTR_ID_REFCLKFREQ,     0, 0},
  /* ATTR_R_REFCLKFREQ */
  {UFS_STD_READ_REQ,     UPIU_QUERY_OPCODE_READ_ATTR,    UPIU_ATTR_ID_REFCLKFREQ,     0, 0},
};

/* ─── CAL LLD adaptors (called from CAL into our driver) ────────────────────*/

static int UfsSendUicCmd (struct ufs_host *Ufs);  /* forward decl */

void ufs_lld_dme_set (void *h, u32 addr, u32 val)
{
  struct ufs_host     *Ufs = (struct ufs_host *)h;
  struct ufs_uic_cmd   cmd = {UIC_CMD_DME_SET, 0, 0, 0};
  cmd.uiccmdarg1 = addr;
  cmd.uiccmdarg3 = val;
  Ufs->uic_cmd = &cmd;
  UfsSendUicCmd (Ufs);
}

void ufs_lld_dme_get (void *h, u32 addr, u32 *val)
{
  struct ufs_host     *Ufs = (struct ufs_host *)h;
  struct ufs_uic_cmd   cmd = {UIC_CMD_DME_GET, 0, 0, 0};
  cmd.uiccmdarg1 = addr;
  Ufs->uic_cmd = &cmd;
  UfsSendUicCmd (Ufs);
  *val = cmd.uiccmdarg3;
}

void ufs_lld_dme_peer_set (void *h, u32 addr, u32 val)
{
  struct ufs_host     *Ufs = (struct ufs_host *)h;
  struct ufs_uic_cmd   cmd = {UIC_CMD_DME_PEER_SET, 0, 0, 0};
  cmd.uiccmdarg1 = addr;
  cmd.uiccmdarg3 = val;
  Ufs->uic_cmd = &cmd;
  UfsSendUicCmd (Ufs);
}

void ufs_lld_pma_write (void *h, u32 val, u32 addr)
{
  struct ufs_host *Ufs = (struct ufs_host *)h;
  writel (val, (u8 *)Ufs->phy_pma + addr);
}

u32 ufs_lld_pma_read (void *h, u32 addr)
{
  struct ufs_host *Ufs = (struct ufs_host *)h;
  return readl ((u8 *)Ufs->phy_pma + addr);
}

void ufs_lld_unipro_write (void *h, u32 val, u32 addr)
{
  struct ufs_host *Ufs = (struct ufs_host *)h;
  writel (val, (u8 *)Ufs->unipro_addr + addr);
}

void ufs_lld_udelay (u32 val)
{
  MicroSecondDelay (val);
}

void ufs_lld_usleep_delay (u32 min, u32 max)
{
  MicroSecondDelay (max);
}

unsigned long ufs_lld_get_time_count (unsigned long offset)
{
  return offset;
}

unsigned long ufs_lld_calc_timeout (const unsigned int ms)
{
  return 1000UL * ms;
}

/* ─── Platform: Exynos 9610 board init (from platform/exynos9610/ufs.c) ─────*/

static void UfsVsSet1usToCnt (struct ufs_host *Ufs)
{
  u32 nVal = readl ((u8 *)Ufs->ioaddr + UFSHCI_VS_UFSHCI_V2P1_CTRL);
  nVal |= IA_TICK_SEL;
  writel (nVal, (u8 *)Ufs->ioaddr + UFSHCI_VS_UFSHCI_V2P1_CTRL);
  writel ((UFS_SCLK / 1000000) & CNT_VAL_1US_MASK,
          (u8 *)Ufs->ioaddr + UFSHCI_VS_1US_TO_CNT_VAL);
}

static void UfsSetUniProClk (struct ufs_host *Ufs)
{
  int timeout = 0;
  writel (3, DIV_CLKCMU_UFS_EMBD_MUX);
  do { timeout++; } while ((readl (DIV_CLKCMU_UFS_EMBD_MUX) & 0x10000) && timeout < UFS_CLKCMU_TIMEOUT);
  timeout = 0;
  writel (1, MUX_CLKCMU_UFS_EMBD_CON);
  do { timeout++; } while ((readl (MUX_CLKCMU_UFS_EMBD_CON) & 0x10000) && timeout < UFS_CLKCMU_TIMEOUT);
  UfsVsSet1usToCnt (Ufs);
}

static EFI_STATUS UfsBoardInit (struct ufs_host *Ufs)
{
  u32 reg;
  UINT32 rst_stat = readl (EXYNOS9610_POWER_RST_STAT);
  UINT32 dfd_en   = readl (EXYNOS9610_POWER_RESET_SEQUENCER_CONFIGURATION);

  DEBUG ((DEBUG_INFO, "UFS: Board init\n"));

  /* MMIO regions */
  Ufs->ioaddr      = (VOID *)(UINTN)UFS_BASE_ADDR;
  Ufs->vs_addr     = (VOID *)(UINTN)(UFS_BASE_ADDR + UFS_VS_ADDR_OFFSET);
  Ufs->fmp_addr    = (VOID *)(UINTN)UFS_FMP_BASE_ADDR;
  Ufs->unipro_addr = (VOID *)(UINTN)UFS_UNIPRO_BASE_ADDR;
  Ufs->phy_pma     = (VOID *)(UINTN)(UFS_BASE_ADDR + UFS_PHY_PMA_OFFSET);

  /* Power / PHY isolation addresses */
  Ufs->dev_pwr_addr  = (VOID *)(UINTN)(0x139B0000UL + 0x144);
  Ufs->dev_pwr_shift = 0;
  Ufs->phy_iso_addr  = (VOID *)(UINTN)(0x11860000UL + 0x724);

  Ufs->mclk_rate = 166 * 1000 * 1000;

  UfsSetUniProClk (Ufs);

  /* GPIO: RST_N and REFCLK */
  reg  = *(volatile u32 *)0x13040048UL;
  reg &= ~0xFFU;
  *(volatile u32 *)0x13040048UL = reg;

  reg  = *(volatile u32 *)0x13040040UL;
  reg &= ~0xFFU;
  reg |= 0x22U;
  *(volatile u32 *)0x13040040UL = reg;

  /* XBOOTLDO GPG1[0] */
  reg  = *(volatile u32 *)0x107300C0UL;
  reg &= ~0x7U;
  reg |= 0x1U;
  *(volatile u32 *)0x107300C0UL = reg;

  /* IO coherency in SYSREG (skip if warm/wdt reset with DFD) */
  if (!((rst_stat & (WARM_RESET | LITTLE_WDT_RESET)) &&
        (dfd_en & EXYNOS9610_EDPCSR_DUMP_EN))) {
    reg  = *(volatile u32 *)0x13020700UL;
    reg |= ((1U << 22) | (1U << 23));
    *(volatile u32 *)0x13020700UL = reg;
  }

  return EFI_SUCCESS;
}

/* ─── Interrupt handler ─────────────────────────────────────────────────────*/

static int UfsHandleUicInt (struct ufs_host *Ufs, u32 stat)
{
  int ret = UFS_IN_PROGRESS;
  struct ufs_uic_cmd *cmd = Ufs->uic_cmd;

  if (stat & UIC_COMMAND_COMPL) {
    if (cmd->uiccmdr == UIC_CMD_DME_LINK_STARTUP)
      ret = UFS_NO_ERROR;
    else if ((cmd->uiccmdr == UIC_CMD_DME_SET) &&
             (cmd->uiccmdarg1 == (0x1571U << 16))) {
      if (stat & UIC_POWER_MODE) ret = UFS_NO_ERROR;
    } else
      ret = UFS_NO_ERROR;
  }

  if ((stat & UIC_ERROR) && (cmd->uiccmdr != UIC_CMD_DME_LINK_STARTUP)) {
    DEBUG ((DEBUG_INFO, "UFS: UIC ERROR 0x%08x\n", stat));
    ret = UFS_ERROR;
  }
  return ret;
}

static int UfsHandleUtpInt (struct ufs_host *Ufs, u32 stat)
{
  if (stat & UTP_TRANSFER_REQ_COMPL) {
    if (!(readl ((u8 *)Ufs->ioaddr + REG_UTP_TRANSFER_REQ_DOOR_BELL) & 1))
      return UFS_NO_ERROR;
  }
  return UFS_IN_PROGRESS;
}

static int UfsHandleInt (struct ufs_host *Ufs, int IsUic)
{
  u32 stat = readl ((u8 *)Ufs->ioaddr + REG_INTERRUPT_STATUS);
  int ret;

  ret = IsUic ? UfsHandleUicInt (Ufs, stat) : UfsHandleUtpInt (Ufs, stat);

  if (stat & INT_FATAL_ERRORS) {
    DEBUG ((DEBUG_INFO, "UFS: FATAL ERROR 0x%08x\n", stat));
    ret = UFS_ERROR;
  }

  if (ret == UFS_IN_PROGRESS) {
    if (Ufs->timeout--)
      u_delay (1);
    else {
      ret = UFS_TIMEOUT;
      DEBUG ((DEBUG_INFO, "UFS: TIMEOUT\n"));
    }
  }

  return ret;
}

/* ─── UIC command send ───────────────────────────────────────────────────────*/

static int UfsSendUicCmd (struct ufs_host *Ufs)
{
  int err, error_code;

  writel (Ufs->uic_cmd->uiccmdarg1, (u8 *)Ufs->ioaddr + REG_UIC_COMMAND_ARG_1);
  writel (Ufs->uic_cmd->uiccmdarg2, (u8 *)Ufs->ioaddr + REG_UIC_COMMAND_ARG_2);
  writel (Ufs->uic_cmd->uiccmdarg3, (u8 *)Ufs->ioaddr + REG_UIC_COMMAND_ARG_3);
  writel (Ufs->uic_cmd->uiccmdr,    (u8 *)Ufs->ioaddr + REG_UIC_COMMAND);

  Ufs->timeout = Ufs->uic_cmd_timeout;
  while (UFS_IN_PROGRESS == (err = UfsHandleInt (Ufs, 1)));

  writel (readl ((u8 *)Ufs->ioaddr + REG_INTERRUPT_STATUS),
          (u8 *)Ufs->ioaddr + REG_INTERRUPT_STATUS);

  error_code = readl ((u8 *)Ufs->ioaddr + REG_UIC_COMMAND_ARG_2);

  if (Ufs->uic_cmd->uiccmdr == UIC_CMD_DME_GET ||
      Ufs->uic_cmd->uiccmdr == UIC_CMD_DME_PEER_GET) {
    Ufs->uic_cmd->uiccmdarg3 = readl ((u8 *)Ufs->ioaddr + REG_UIC_COMMAND_ARG_3);
  }

  return error_code | err;
}

/* ─── CAL init helpers ──────────────────────────────────────────────────────*/

static EFI_STATUS UfsInitCal (struct ufs_host *Ufs, int idx)
{
  Ufs->cal_param->host     = Ufs;
  Ufs->cal_param->board    = 0;
  return EFI_SUCCESS;
}

static EFI_STATUS UfsPreLink (struct ufs_host *Ufs, u8 lane)
{
  struct ufs_cal_param *p = Ufs->cal_param;
  p->mclk_rate      = Ufs->mclk_rate;
  p->available_lane = lane;
  if (ufs_cal_pre_link (p) != UFS_CAL_NO_ERROR)
    return EFI_DEVICE_ERROR;
  return EFI_SUCCESS;
}

static EFI_STATUS UfsPostLink (struct ufs_host *Ufs)
{
  if (ufs_cal_post_link (Ufs->cal_param) != UFS_CAL_NO_ERROR)
    return EFI_DEVICE_ERROR;
  return EFI_SUCCESS;
}

/* ─── UTP PRDT / UPIU construction ─────────────────────────────────────────*/

STATIC void UfsMapSg (struct ufs_host *Ufs)
{
  u32 i, len, sg_segs, remaining, seg_bytes;
  len     = Ufs->scsi_cmd->datalen;
  sg_segs = (len + UFS_SG_BLOCK_SIZE - 1) / UFS_SG_BLOCK_SIZE;
  for (i = 0; i < sg_segs; i++) {
    u64 phys = (u64)(UINTN)Ufs->scsi_cmd->buf + (u64)i * UFS_SG_BLOCK_SIZE;
    remaining = len - i * UFS_SG_BLOCK_SIZE;
    seg_bytes = (remaining < UFS_SG_BLOCK_SIZE) ? remaining : UFS_SG_BLOCK_SIZE;
    Ufs->cmd_desc_addr->prd_table[i].size       = seg_bytes - 1;
    Ufs->cmd_desc_addr->prd_table[i].base_addr  = (u32)(phys & 0xFFFFFFFFULL);
    Ufs->cmd_desc_addr->prd_table[i].upper_addr = (u32)(phys >> 32);
  }
}

static u32 UfsCmdGetDir (UFS_SCSI_CMD *Cmd)
{
  if (!Cmd->datalen) return UTP_NO_DATA_TRANSFER;
  switch (Cmd->cdb[0]) {
  case SCSI_OP_WRITE_10:
  case SCSI_OP_WRITE_BUFFER:
  case SCSI_OP_FORMAT_UNIT:
  case SCSI_OP_UNMAP:
  case SCSI_OP_SECU_PROT_OUT:
  case SCSI_OP_START_STOP_UNIT:
    return UTP_HOST_TO_DEVICE;
  default:
    return UTP_DEVICE_TO_HOST;
  }
}

static u32 UfsCmdGetFlags (UFS_SCSI_CMD *Cmd)
{
  if (!Cmd->datalen) return UPIU_CMD_FLAGS_NONE;
  switch (Cmd->cdb[0]) {
  case SCSI_OP_WRITE_10:
  case SCSI_OP_WRITE_BUFFER:
  case SCSI_OP_FORMAT_UNIT:
  case SCSI_OP_UNMAP:
  case SCSI_OP_SECU_PROT_OUT:
  case SCSI_MODE_SEL10:
    return UPIU_CMD_FLAGS_WRITE;
  case SCSI_OP_START_STOP_UNIT:
    return UPIU_CMD_FLAGS_NONE;
  default:
    return UPIU_CMD_FLAGS_READ;
  }
}

static void UfsWriteCmdUcd (struct ufs_host *Ufs)
{
  struct ufs_upiu        *cmd  = &Ufs->cmd_desc_addr->command_upiu;
  struct ufs_upiu_header *hdr  = &cmd->header;
  u8                     *tsf  = cmd->tsf;
  u32                     dlen = cpu_to_be32 (Ufs->scsi_cmd->datalen);

  hdr->type  = UPIU_TRANSACTION_COMMAND;
  hdr->flags = (u8)UfsCmdGetFlags (Ufs->scsi_cmd);
  hdr->lun   = (u8)Ufs->lun;
  hdr->tag   = 0;

  CopyMem (&tsf[0], &dlen, sizeof (u32));
  CopyMem (&tsf[4], Ufs->scsi_cmd->cdb, MAX_CDB_SIZE);
}

static int UfsWriteQueryUcd (struct ufs_host *Ufs, query_index qry)
{
  struct ufs_upiu        *cmd   = &Ufs->cmd_desc_addr->command_upiu;
  struct ufs_upiu_header *hdr   = &cmd->header;
  u8                     *tsf   = cmd->tsf;
  u16                     dlen;
  u32                     info;
  int                     lun;

  hdr->type     = UPIU_TRANSACTION_QUERY_REQ;
  hdr->flags    = UPIU_CMD_FLAGS_NONE;
  hdr->tag      = 0;
  hdr->function = gQueryParams[qry][0];

  tsf[0] = gQueryParams[qry][1];  /* OPCODE   */
  tsf[1] = gQueryParams[qry][2];  /* IDN      */
  tsf[2] = gQueryParams[qry][3];  /* INDEX    */
  tsf[3] = gQueryParams[qry][4];  /* SELECTOR */

  if (hdr->function == UFS_STD_WRITE_REQ &&
      tsf[0] == UPIU_QUERY_OPCODE_WRITE_DESC &&
      tsf[1] == UPIU_DESC_ID_CONFIGURATION) {
    /*
     * Config descriptor write: data_seg_len is the actual config payload size.
     * Only for this case do we gate on device_desc being populated.
     */
    Ufs->data_seg_len = Ufs->device_desc.bUD0BaseOffset
                      + (8 * Ufs->device_desc.bUDConfigPlength);
    dlen = Ufs->data_seg_len;
    if (dlen > UPIU_DATA_SIZE) return -1;
    hdr->datalength = cpu_to_be16 (dlen);
    info = cpu_to_be16 (dlen);
    CopyMem (&tsf[6], &info, sizeof (u16));
  } else if (hdr->function == UFS_STD_READ_REQ) {
    /* All read queries: ask device to return up to UPIU_DATA_SIZE bytes */
    info = cpu_to_be16 (UPIU_DATA_SIZE);
    CopyMem (&tsf[6], &info, sizeof (u16));
  } else if (hdr->function == UFS_STD_WRITE_REQ) {
    /* Attribute / flag writes have no data payload via data segment */
    hdr->datalength = 0;
  }

  if (tsf[0] == UPIU_QUERY_OPCODE_WRITE_ATTR) {
    info = cpu_to_be32 (Ufs->attributes.arry[tsf[1]]);
    CopyMem (&tsf[8], &info, sizeof (u32));
  } else if (tsf[0] == UPIU_QUERY_OPCODE_SET_FLAG)
    tsf[11] = Ufs->flags.arry[tsf[1]];

  if (tsf[0] == UPIU_QUERY_OPCODE_WRITE_DESC && tsf[1] == UPIU_DESC_ID_CONFIGURATION) {
    CopyMem (cmd->data, &Ufs->config_desc.header, Ufs->device_desc.bUD0BaseOffset);
    for (lun = 0; lun < 8; lun++)
      CopyMem (cmd->data + Ufs->device_desc.bUD0BaseOffset
               + lun * Ufs->device_desc.bUDConfigPlength,
               &Ufs->config_desc.unit[lun],
               Ufs->device_desc.bUDConfigPlength);
  }
  return 0;
}

static int UfsWriteUtrd (struct ufs_host *Ufs, u32 type)
{
  struct ufs_utrd *utrd = Ufs->utrd_addr;
  u32   len, sg_segs;

  switch (type) {
  case UPIU_TRANSACTION_COMMAND:
    len      = Ufs->scsi_cmd->datalen;
    sg_segs  = (len + UFS_SG_BLOCK_SIZE - 1) / UFS_SG_BLOCK_SIZE;

    utrd->dw[0] = UfsCmdGetDir (Ufs->scsi_cmd) | UTP_SCSI_COMMAND | UTP_REQ_DESC_INT_CMD;
    utrd->dw[2] = OCS_INVALID_COMMAND_STATUS;
    if (len) {
      utrd->prdt_len = (u16)(sg_segs * sizeof (struct ufs_prdt));
      utrd->prdt_off = ALIGNED_UPIU_SIZE * 2;
    } else {
      utrd->prdt_len = 0;
      utrd->prdt_off = 0;
    }
    break;
  case UPIU_TRANSACTION_QUERY_REQ:
  case UPIU_TRANSACTION_NOP_OUT:
    utrd->dw[0] = UTP_REQ_DESC_INT_CMD;
    utrd->dw[2] = OCS_INVALID_COMMAND_STATUS;
    break;
  default:
    return -1;
  }
  return 0;
}

/* ─── UTP command send/wait ─────────────────────────────────────────────────*/

static void UfsUtpSend (struct ufs_host *Ufs, u32 type)
{
  switch (type) {
  case UPIU_TRANSACTION_NOP_OUT:
  case UPIU_TRANSACTION_QUERY_REQ:
    writel (0x0, (u8 *)Ufs->vs_addr + VS_UTRL_NEXUS_TYPE);
    break;
  case UPIU_TRANSACTION_COMMAND:
    writel (0xFFFFFFFF, (u8 *)Ufs->vs_addr + VS_UTRL_NEXUS_TYPE);
    break;
  default:
    break;
  }

  switch (type & 0xFF) {
  case UPIU_TRANSACTION_NOP_OUT:
  case UPIU_TRANSACTION_COMMAND:
  case UPIU_TRANSACTION_QUERY_REQ:
    writel (1, (u8 *)Ufs->ioaddr + REG_UTP_TRANSFER_REQ_DOOR_BELL);
    break;
  default:
    break;
  }
}

static int UfsUtpWaitResponse (struct ufs_host *Ufs, u32 type)
{
  int err;

  if (type == UPIU_TRANSACTION_COMMAND &&
      Ufs->scsi_cmd->cdb[0] == SCSI_OP_FORMAT_UNIT)
    Ufs->timeout = FORMAT_CMD_TIMEOUT;
  else if (type == UPIU_TRANSACTION_NOP_OUT)
    Ufs->timeout = NOP_OUT_TIMEOUT;
  else
    Ufs->timeout = Ufs->ufs_cmd_timeout;

  while (UFS_IN_PROGRESS == (err = UfsHandleInt (Ufs, 0)));

  writel (readl ((u8 *)Ufs->ioaddr + REG_INTERRUPT_STATUS),
          (u8 *)Ufs->ioaddr + REG_INTERRUPT_STATUS);

  /* Nexus clear for NOP / Query */
  if (type == UPIU_TRANSACTION_NOP_OUT || type == UPIU_TRANSACTION_QUERY_REQ)
    writel ((readl ((u8 *)Ufs->ioaddr + 0x140) | 0x01),
            (u8 *)Ufs->ioaddr + 0x140);

  return err;
}

static int UfsUtpCheckResult (struct ufs_host *Ufs)
{
  struct ufs_utrd        *utrd = Ufs->utrd_addr;
  struct ufs_upiu        *resp = &Ufs->cmd_desc_addr->response_upiu;
  struct ufs_upiu_header *hdr  = &resp->header;

  if (Ufs->scsi_cmd)
    Ufs->scsi_cmd->status = hdr->status;

  if (utrd->dw[2] != OCS_SUCCESS) {
    DEBUG ((DEBUG_INFO, "UFS: OCS=0x%02x response=0x%02x type=0x%02x\n",
            utrd->dw[2], hdr->response, hdr->type));
    return (hdr->response != 0) ? -1 : 0;
  }

  /* OCS is good, but also check SCSI status for COMMAND UPIUs */
  if (Ufs->scsi_cmd && hdr->status != SCSI_STATUS_GOOD) {
    DEBUG ((DEBUG_WARN, "UFS: SCSI CHECK CONDITION status=0x%02x cdb0=0x%02x\n",
            hdr->status, Ufs->scsi_cmd->cdb[0]));
    return -1;
  }

  return 0;
}

/* ─── UTP init (zero cmd descriptor) ───────────────────────────────────────*/

static void UfsUtpInit (struct ufs_host *Ufs, u32 lun)
{
  Ufs->lun      = lun;
  Ufs->scsi_cmd = NULL;
  ZeroMem (Ufs->cmd_desc_addr, sizeof (struct ufs_cmd_desc));
  /* Also zero the UTRD so stale direction/OCS/PRDT fields don't carry over */
  ZeroMem (Ufs->utrd_addr, sizeof (struct ufs_utrd));
  /* Restore the static cmd_desc pointer fields that UfsPreSetup wrote */
  Ufs->utrd_addr->cmd_desc_addr_l = (u32)(UINTN)Ufs->cmd_desc_addr;
  Ufs->utrd_addr->cmd_desc_addr_h = (u32)((UINTN)Ufs->cmd_desc_addr >> 32);
  Ufs->utrd_addr->rsp_upiu_off    = OFFSET_OF (struct ufs_cmd_desc, response_upiu);
  Ufs->utrd_addr->rsp_upiu_len    = ALIGNED_UPIU_SIZE;
}

/* ─── Query response decode ─────────────────────────────────────────────────*/

static void UfsQueryReadInfo (struct ufs_host *Ufs, u8 idn)
{
  struct ufs_upiu *resp = &Ufs->cmd_desc_addr->response_upiu;
  u8              *data = resp->data;
  VOID            *dst  = NULL;
  UINTN            len  = 0;
  int              lun;

  switch (idn) {
  case UPIU_DESC_ID_UNIT:
    if (data[2] >= 8) {
      DEBUG ((DEBUG_INFO, "UFS: unit desc response INDEX %d out of range\n", data[2]));
      return;
    }
    dst = &Ufs->unit_desc[data[2]];
    len = MIN (data[0], sizeof (struct ufs_unit_desc));
    break;
  case UPIU_DESC_ID_DEVICE:
    dst = &Ufs->device_desc;
    len = MIN (data[0], sizeof (struct ufs_device_desc));
    break;
  case UPIU_DESC_ID_CONFIGURATION:
    Ufs->data_seg_len = Ufs->device_desc.bUD0BaseOffset
                      + 8 * Ufs->device_desc.bUDConfigPlength;
    len = MIN (data[0], (UINT8)Ufs->data_seg_len);
    /* Copy header + per-LU params */
    CopyMem (&Ufs->config_desc.header, resp->data, Ufs->device_desc.bUD0BaseOffset);
    for (lun = 0; lun < 8; lun++)
      CopyMem (&Ufs->config_desc.unit[lun],
               resp->data + Ufs->device_desc.bUD0BaseOffset
                          + lun * Ufs->device_desc.bUDConfigPlength,
               Ufs->device_desc.bUDConfigPlength);
    return;
  case UPIU_DESC_ID_GEOMETRY:
    dst = &Ufs->geometry_desc;
    len = MIN (data[0], sizeof (struct ufs_geometry_desc));
    break;
  default:
    return;
  }

  if (dst) CopyMem (dst, resp->data, len);
}

static void UfsQueryGetData (struct ufs_host *Ufs, query_index qry)
{
  struct ufs_upiu *resp   = &Ufs->cmd_desc_addr->response_upiu;
  u8               opcode = gQueryParams[qry][1];
  u8              *tsf    = resp->tsf;
  u32              val;

  switch (opcode) {
  case UPIU_QUERY_OPCODE_READ_DESC:
    UfsQueryReadInfo (Ufs, gQueryParams[qry][2]);
    break;
  case UPIU_QUERY_OPCODE_READ_ATTR:
    val = UPIU_HEADER_DWORD ((u32)tsf[8], (u32)tsf[9], (u32)tsf[10], (u32)tsf[11]);
    Ufs->attributes.arry[gQueryParams[qry][2]] = val;
    break;
  case UPIU_QUERY_OPCODE_READ_FLAG:
    Ufs->flags.arry[gQueryParams[qry][2]] = (u8)tsf[11];
    break;
  default:
    break;
  }
}

/* ─── High-level UTP flows ──────────────────────────────────────────────────*/

static int UfsUtpNopProcess (struct ufs_host *Ufs)
{
  UfsUtpInit (Ufs, 0);
  wmb ();
  UfsUtpSend (Ufs, UPIU_TRANSACTION_NOP_OUT);
  if (UfsUtpWaitResponse (Ufs, UPIU_TRANSACTION_NOP_OUT))
    return -1;
  return UfsUtpCheckResult (Ufs);
}

static int UfsUtpQueryProcess (struct ufs_host *Ufs, query_index qry, u32 lun)
{
  UfsUtpInit (Ufs, lun);
  if (UfsWriteQueryUcd (Ufs, qry)) return -1;
  if (UfsWriteUtrd (Ufs, UPIU_TRANSACTION_QUERY_REQ)) return -1;
  wmb ();
  UfsUtpSend (Ufs, UPIU_TRANSACTION_QUERY_REQ);
  if (UfsUtpWaitResponse (Ufs, UPIU_TRANSACTION_QUERY_REQ)) return -1;
  if (UfsUtpCheckResult (Ufs)) return -1;
  UfsQueryGetData (Ufs, qry);
  return 0;
}

int UfsUtpQueryRetry (struct ufs_host *Ufs, query_index qry, u32 lun)
{
  int r, retries;
  for (retries = QUERY_REQ_RETRIES; retries > 0; retries--) {
    r = UfsUtpQueryProcess (Ufs, qry, lun);
    if (!r) break;
    DEBUG ((DEBUG_WARN, "UFS: Query retry (left=%d)\n", retries - 1));
  }
  return r;
}

int UfsUtpCmdProcess (struct ufs_host *Ufs, UFS_SCSI_CMD *Cmd)
{
  UfsUtpInit (Ufs, 0);
  Ufs->scsi_cmd = Cmd;
  Ufs->lun      = Cmd->lun;

  UfsWriteCmdUcd (Ufs);
  UfsMapSg (Ufs);
  if (UfsWriteUtrd (Ufs, UPIU_TRANSACTION_COMMAND)) return -1;

  wmb ();
  UfsUtpSend (Ufs, UPIU_TRANSACTION_COMMAND);
  if (UfsUtpWaitResponse (Ufs, UPIU_TRANSACTION_COMMAND)) return -1;
  return UfsUtpCheckResult (Ufs);
}

/* ─── Device power / GPIO ───────────────────────────────────────────────────*/

static void UfsDevicePower (struct ufs_host *Ufs, int on)
{
  if (Ufs->dev_pwr_addr)
    UFS_SET_SFR (Ufs->dev_pwr_addr, on, 0x1, Ufs->dev_pwr_shift);
}

static void UfsDeviceReset (struct ufs_host *Ufs)
{
  writel (0, (u8 *)Ufs->vs_addr + VS_GPIO_OUT);
  u_delay (5);
  writel (1, (u8 *)Ufs->vs_addr + VS_GPIO_OUT);
}

/* ─── HCE / pre-setup ───────────────────────────────────────────────────────*/

static EFI_STATUS UfsPreSetup (struct ufs_host *Ufs)
{
  u32 reg, val;

  /* PHY isolation bypass */
  val = readl (Ufs->phy_iso_addr);
  if (!(val & 0x1)) { val |= 1; writel (val, Ufs->phy_iso_addr); }

  /* VS_SW_RST */
  if ((readl ((u8 *)Ufs->vs_addr + VS_FORCE_HCS) >> 4) & 0xF)
    writel (0, (u8 *)Ufs->vs_addr + VS_FORCE_HCS);

  writel (3, (u8 *)Ufs->vs_addr + VS_SW_RST);
  while (readl ((u8 *)Ufs->vs_addr + VS_SW_RST));

  /* Clear idle indicator */
  reg = readl ((u8 *)Ufs->vs_addr + VS_IS);
  if ((reg >> 20) & 0x1)
    writel (reg, (u8 *)Ufs->vs_addr + VS_IS);

  UfsDevicePower (Ufs, 1);
  u_delay (1000);
  UfsDeviceReset (Ufs);

  /* HCE enable */
  writel (1, (u8 *)Ufs->ioaddr + REG_CONTROLLER_ENABLE);
  while (!(readl ((u8 *)Ufs->ioaddr + REG_CONTROLLER_ENABLE) & 0x1))
    u_delay (1);

  /* Refclk out control */
  writel (readl ((u8 *)Ufs->vs_addr + VS_CLKSTOP_CTRL) & ~(1 << 4),
          (u8 *)Ufs->vs_addr + VS_CLKSTOP_CTRL);

  /* Clock gating */
  writel (0xDE0, (u8 *)Ufs->vs_addr + VS_FORCE_HCS);
  writel (readl ((u8 *)Ufs->vs_addr + VS_UFS_ACG_DISABLE) | 1,
          (u8 *)Ufs->vs_addr + VS_UFS_ACG_DISABLE);

  /* Descriptor base addresses */
  ZeroMem (Ufs->cmd_desc_addr, UFS_NUTRS * sizeof (struct ufs_cmd_desc));
  ZeroMem (Ufs->utrd_addr, UFS_NUTRS * sizeof (struct ufs_utrd));

  Ufs->utrd_addr->cmd_desc_addr_l = (u32)(UINTN)Ufs->cmd_desc_addr;
  Ufs->utrd_addr->cmd_desc_addr_h = (u32)((UINTN)Ufs->cmd_desc_addr >> 32);
  Ufs->utrd_addr->rsp_upiu_off    = OFFSET_OF (struct ufs_cmd_desc, response_upiu);
  Ufs->utrd_addr->rsp_upiu_len    = ALIGNED_UPIU_SIZE;

  writel ((u32)(UINTN)Ufs->utmrd_addr, (u8 *)Ufs->ioaddr + REG_UTP_TASK_REQ_LIST_BASE_L);
  writel ((u32)((UINTN)Ufs->utmrd_addr >> 32), (u8 *)Ufs->ioaddr + REG_UTP_TASK_REQ_LIST_BASE_H);

  writel ((u32)(UINTN)Ufs->utrd_addr, (u8 *)Ufs->ioaddr + REG_UTP_TRANSFER_REQ_LIST_BASE_L);
  writel ((u32)((UINTN)Ufs->utrd_addr >> 32), (u8 *)Ufs->ioaddr + REG_UTP_TRANSFER_REQ_LIST_BASE_H);

  /* Cport config */
  writel (0x22, (u8 *)Ufs->vs_addr + 0x114);
  writel (0x01, (u8 *)Ufs->vs_addr + 0x110);

  return EFI_SUCCESS;
}

/* ─── Vendor setup (after NOP/DeviceInit) ────────────────────────────────── */

static void UfsVendorSetup (struct ufs_host *Ufs)
{
  writel (0xA, (u8 *)Ufs->vs_addr + VS_DATA_REORDER);
  writel (1, (u8 *)Ufs->ioaddr + REG_UTP_TASK_REQ_LIST_RUN_STOP);
  writel (1, (u8 *)Ufs->ioaddr + REG_UTP_TRANSFER_REQ_LIST_RUN_STOP);
  writel (UFS_SG_BLOCK_SIZE_BIT, (u8 *)Ufs->vs_addr + VS_TXPRDT_ENTRY_SIZE);
  writel (UFS_SG_BLOCK_SIZE_BIT, (u8 *)Ufs->vs_addr + VS_RXPRDT_ENTRY_SIZE);
  writel (0xFFFFFFFF, (u8 *)Ufs->vs_addr + VS_UTRL_NEXUS_TYPE);
  writel (0xFFFFFFFF, (u8 *)Ufs->vs_addr + VS_UMTRL_NEXUS_TYPE);
}

/* Reads PA_ConnectedTxDataLanes/PA_ConnectedRxDataLanes, sets the active
   lane count via DME_SET, stores it in cal_param and pmd->lane.
   Mirrors lk3rd ufs_check_2lane(). */
static int UfsCheck2Lane (struct ufs_host *Ufs)
{
  struct ufs_uic_cmd tx_get = {UIC_CMD_DME_GET, UIC_ARG_MIB (0x1561), 0, 0}; /* PA_ConnectedTxDataLanes */
  struct ufs_uic_cmd rx_get = {UIC_CMD_DME_GET, UIC_ARG_MIB (0x1581), 0, 0}; /* PA_ConnectedRxDataLanes */
  struct ufs_uic_cmd tx_set, rx_set;
  int tx, rx;

  Ufs->uic_cmd = &tx_get;
  if (UfsSendUicCmd (Ufs)) return -1;
  tx = (int)tx_get.uiccmdarg3;

  Ufs->uic_cmd = &rx_get;
  if (UfsSendUicCmd (Ufs)) return -1;
  rx = (int)rx_get.uiccmdarg3;

  Ufs->cal_param->connected_tx_lane = (u8)tx;
  Ufs->cal_param->connected_rx_lane = (u8)rx;
  DEBUG ((DEBUG_INFO, "UFS: connected TX=%d RX=%d\n", tx, rx));

  /* DME_SET PA_ActiveTxDataLanes / PA_ActiveRxDataLanes */
  tx_set.uiccmdr    = UIC_CMD_DME_SET;
  tx_set.uiccmdarg1 = UIC_ARG_MIB (0x1560);
  tx_set.uiccmdarg2 = 0;
  tx_set.uiccmdarg3 = (u32)tx;

  rx_set.uiccmdr    = UIC_CMD_DME_SET;
  rx_set.uiccmdarg1 = UIC_ARG_MIB (0x1580);
  rx_set.uiccmdarg2 = 0;
  rx_set.uiccmdarg3 = (u32)rx;

  Ufs->uic_cmd = &tx_set;
  if (UfsSendUicCmd (Ufs)) return -1;
  Ufs->uic_cmd = &rx_set;
  if (UfsSendUicCmd (Ufs)) return -1;

  if (tx == 1 && rx == 1)
    Ufs->quirks |= UFS_QUIRK_USE_1LANE;

  /* pmd->lane driven by TX count (lk3rd: ufs->pmd_cxt.lane = tx) */
  Ufs->pmd_cxt.lane = (u8)tx;
  return 0;
}

/* ─── NOP + fDeviceInit ─────────────────────────────────────────────────────*/

static int UfsEndBootMode (struct ufs_host *Ufs)
{
  int  retry = 100;
  u8   flag;
  int  res = -1;  /* initialise: guards against NOP_OUT_RETRY ever being 0 */

  /* 1. NOP OUT */
  for (int i = 0; i < NOP_OUT_RETRY; i++) {
    res = UfsUtpNopProcess (Ufs);
    if (!res) break;
  }
  if (res) {
    DEBUG ((DEBUG_INFO, "UFS: NOP OUT failed\n"));
    return res;
  }

  /* 2. Set fDeviceInit */
  Ufs->flags.arry[UPIU_FLAG_ID_DEVICEINIT] = 1;
  res = UfsUtpQueryRetry (Ufs, FLAG_W_FDEVICEINIT, 0);
  if (res) return res;

  /* 3. Poll fDeviceInit == 0 */
  do {
    res = UfsUtpQueryRetry (Ufs, FLAG_R_FDEVICEINIT, 0);
    if (res) return res;
    flag = Ufs->flags.arry[UPIU_FLAG_ID_DEVICEINIT];
    if (!flag) break;
    mdelay (1);
  } while (retry-- > 0);

  if (flag) { DEBUG ((DEBUG_INFO, "UFS: fDeviceInit timeout\n")); return -1; }
  return 0;
}

/* ─── Power mode change ─────────────────────────────────────────────────────*/

static EFI_STATUS UfsPmcCommon (struct ufs_host *Ufs, struct uic_pwr_mode *pmd)
{
  struct ufs_uic_cmd cmd[] = {
    {UIC_CMD_DME_SET, (0x1583U << 16), 0, Ufs->gear_mode}, /* PA_RxGear       */
    {UIC_CMD_DME_SET, (0x1568U << 16), 0, Ufs->gear_mode}, /* PA_TxGear       */
    {UIC_CMD_DME_SET, (0x1580U << 16), 0, pmd->lane},      /* PA_ActiveRxLanes*/
    {UIC_CMD_DME_SET, (0x1560U << 16), 0, pmd->lane},      /* PA_ActiveTxLanes*/
    {UIC_CMD_DME_SET, (0x1584U << 16), 0, 1},              /* PA_RxTermination*/
    {UIC_CMD_DME_SET, (0x1569U << 16), 0, 1},              /* PA_TxTermination*/
    {UIC_CMD_DME_SET, (0x156AU << 16), 0, UFS_RATE},       /* PA_HSSeries     */
    {0, 0, 0, 0},
  };
  struct ufs_uic_cmd pmc = {UIC_CMD_DME_SET, (0x1571U << 16), 0, UFS_RXTX_POWER_MODE};
  u32 reg;
  int i;

  for (i = 0; cmd[i].uiccmdr; i++) {
    Ufs->uic_cmd = &cmd[i];
    if (UfsSendUicCmd (Ufs)) return EFI_DEVICE_ERROR;
  }

  Ufs->uic_cmd = &pmc;
  if (UfsSendUicCmd (Ufs)) return EFI_DEVICE_ERROR;

  reg = readl ((u8 *)Ufs->ioaddr + REG_CONTROLLER_STATUS);
  if (UPMCRS (reg) != PWR_LOCAL) {
    DEBUG ((DEBUG_INFO, "UFS: Gear change failed, UPMCRS=0x%x\n", UPMCRS (reg)));
    return EFI_DEVICE_ERROR;
  }
  return EFI_SUCCESS;
}

/* ─── Public: UfsInitInterface ──────────────────────────────────────────────*/

EFI_STATUS UfsInitInterface (struct ufs_host *Ufs)
{
  struct ufs_uic_cmd   link_cmd = {UIC_CMD_DME_LINK_STARTUP, 0, 0, 0};
  struct ufs_uic_cmd   lane_cmd = {UIC_CMD_DME_GET, (0x1540U << 16), 0, 0};
  struct uic_pwr_mode *pmd      = &Ufs->pmd_cxt;
  EFI_STATUS           Status;

  /* 1. pre-setup */
  Status = UfsPreSetup (Ufs);
  if (EFI_ERROR (Status)) return Status;

  /* 2. Get available lanes */
  Ufs->uic_cmd = &lane_cmd;
  if (UfsSendUicCmd (Ufs)) return EFI_DEVICE_ERROR;
  Ufs->cal_param->available_lane = (u8)lane_cmd.uiccmdarg3;
  if (Ufs->cal_param->available_lane == 0) Ufs->cal_param->available_lane = 2;

  /* 3. pre-link CAL */
  Status = UfsPreLink (Ufs, Ufs->cal_param->available_lane);
  if (EFI_ERROR (Status)) return Status;

  /* 4. Link startup */
  Ufs->uic_cmd = &link_cmd;
  if (UfsSendUicCmd (Ufs)) {
    DEBUG ((DEBUG_INFO, "UFS: Link startup failed\n"));
    return EFI_DEVICE_ERROR;
  }
  DEBUG ((DEBUG_INFO, "UFS: Link established\n"));

  /* 5. post-link CAL */
  Status = UfsPostLink (Ufs);
  if (EFI_ERROR (Status)) return Status;

  /* 6. Vendor setup (enables UTRL/UMTRL run) */
  UfsVendorSetup (Ufs);

  /* 7. NOP + fDeviceInit */
  if (UfsEndBootMode (Ufs)) return EFI_DEVICE_ERROR;
  DEBUG ((DEBUG_INFO, "UFS: Device initialized\n"));

  /* 8. Check connected lane count, DME_SET active lanes, set pmd->lane */
  if (UfsCheck2Lane (Ufs)) {
    DEBUG ((DEBUG_INFO, "UFS: 2-lane check failed\n"));
    return EFI_DEVICE_ERROR;
  }

  /* 9. Read device descriptor */
  if (UfsUtpQueryRetry (Ufs, DESC_R_DEVICE_DESC, 0)) return EFI_DEVICE_ERROR;
  if (UfsUtpQueryRetry (Ufs, DESC_R_GEOMETRY_DESC, 0)) return EFI_DEVICE_ERROR;

  /* 10. PMC: pre-pmc CAL — pmd->lane already set by UfsCheck2Lane */
  pmd->gear      = (u8)Ufs->gear_mode;
  pmd->mode      = UFS_POWER_MODE;
  pmd->hs_series = UFS_RATE;
  /* pmd->lane set by UfsCheck2Lane above, do not override */

  /* 11. PMC */
  Status = UfsPmcCommon (Ufs, pmd);
  if (EFI_ERROR (Status)) return Status;

  DEBUG ((DEBUG_INFO, "UFS: Power mode G%d M%d L%d Series%d\n",
          pmd->gear, pmd->mode, pmd->lane, pmd->hs_series));

  return EFI_SUCCESS;
}

/* ─── Public: UfsInitHost ────────────────────────────────────────────────── */

EFI_STATUS UfsInitHost (struct ufs_host *Ufs)
{
  Ufs->ufs_cmd_timeout        = UTP_CMD_TIMEOUT;
  Ufs->uic_cmd_timeout        = UIC_CMD_TIMEOUT;
  Ufs->ufs_query_req_timeout  = QUERY_REQ_TIMEOUT;

  /* Platform board init (MMIO, clock, GPIO) */
  EFI_STATUS Status = UfsBoardInit (Ufs);
  if (EFI_ERROR (Status)) return Status;

  /* CAL init */
  return UfsInitCal (Ufs, 0);
}

/* ─── Public: UfsAllocHost ───────────────────────────────────────────────── */

struct ufs_host *UfsAllocHost (VOID)
{
  struct ufs_host     *Ufs;
  struct ufs_cmd_desc *CmdDesc = NULL;
  struct ufs_utrd     *Utrd    = NULL;
  struct ufs_utmrd    *Utmrd   = NULL;
  struct ufs_cal_param *Cal    = NULL;

  Ufs = AllocateZeroPool (sizeof (struct ufs_host));
  if (!Ufs) return NULL;

  Cal = AllocateZeroPool (sizeof (struct ufs_cal_param));
  if (!Cal) goto err;
  Ufs->cal_param = Cal;

  /* 4KB-aligned command descriptors */
  CmdDesc = AllocateAlignedPages (
              EFI_SIZE_TO_PAGES (UFS_NUTRS * sizeof (struct ufs_cmd_desc)),
              SIZE_4KB);
  if (!CmdDesc) goto err;
  ZeroMem (CmdDesc, UFS_NUTRS * sizeof (struct ufs_cmd_desc));
  Ufs->cmd_desc_addr = CmdDesc;

  Utrd = AllocateAlignedPages (
           EFI_SIZE_TO_PAGES (UFS_NUTRS * sizeof (struct ufs_utrd)),
           SIZE_4KB);
  if (!Utrd) goto err;
  ZeroMem (Utrd, UFS_NUTRS * sizeof (struct ufs_utrd));
  Ufs->utrd_addr = Utrd;

  Utmrd = AllocateAlignedPages (
            EFI_SIZE_TO_PAGES (sizeof (struct ufs_utmrd)),
            SIZE_4KB);
  if (!Utmrd) goto err;
  ZeroMem (Utmrd, sizeof (struct ufs_utmrd));
  Ufs->utmrd_addr = Utmrd;

  return Ufs;

err:
  if (Utmrd)   FreeAlignedPages (Utmrd,   EFI_SIZE_TO_PAGES (sizeof (struct ufs_utmrd)));
  if (Utrd)    FreeAlignedPages (Utrd,     EFI_SIZE_TO_PAGES (UFS_NUTRS * sizeof (struct ufs_utrd)));
  if (CmdDesc) FreeAlignedPages (CmdDesc,  EFI_SIZE_TO_PAGES (UFS_NUTRS * sizeof (struct ufs_cmd_desc)));
  if (Cal)     FreePool (Cal);
  FreePool (Ufs);
  return NULL;
}

/* ─── Public: UfsReadCapacity ────────────────────────────────────────────── */

EFI_STATUS UfsReadCapacity (struct ufs_host *Ufs, u32 Lun, u64 *BlkCnt, u32 *BlkSize)
{
  UFS_SCSI_CMD  Cmd;
  u8           *Buf;
  EFI_STATUS    Status;

  /* Must be page-aligned for DMA — stack buffer is not safe on ARM */
  Buf = AllocateAlignedPages (1, SIZE_4KB);
  if (!Buf) return EFI_OUT_OF_RESOURCES;
  ZeroMem (Buf, SIZE_4KB);

  ZeroMem (&Cmd, sizeof (Cmd));
  Cmd.cdb[0]  = SCSI_OP_READ_CAPACITY;
  Cmd.buf     = Buf;
  Cmd.datalen = 8;
  Cmd.lun     = Lun;

  if (UfsUtpCmdProcess (Ufs, &Cmd)) {
    FreeAlignedPages (Buf, 1);
    return EFI_DEVICE_ERROR;
  }

  /* Big-endian decode */
  *BlkCnt  = (u64)(((u32)Buf[0] << 24) | ((u32)Buf[1] << 16) |
                   ((u32)Buf[2] << 8)  |  (u32)Buf[3]) + 1;
  *BlkSize = ((u32)Buf[4] << 24) | ((u32)Buf[5] << 16) |
             ((u32)Buf[6] << 8)  |  (u32)Buf[7];

  DEBUG ((DEBUG_INFO, "UFS: LUN%d: %Lu blocks * %u bytes\n", Lun, *BlkCnt, *BlkSize));
  FreeAlignedPages (Buf, 1);
  Status = EFI_SUCCESS;
  return Status;
}

/* ─── Public: UfsRequestSense ────────────────────────────────────────────── */

/*
 * Issues REQUEST SENSE to clear any pending Unit Attention condition on the
 * given LUN.  Mirrors the lk3rd scsi_mode_sense() / scsi_scan_common() sequence
 * which always issues REQUEST SENSE before READ CAPACITY.
 * Failures are silently ignored -- a UA is non-fatal; the important thing is
 * that the subsequent READ CAPACITY has a clean slate.
 */
EFI_STATUS UfsRequestSense (struct ufs_host *Ufs, u32 Lun)
{
  UFS_SCSI_CMD Cmd;
  UINT8       *Buf;

  Buf = AllocateAlignedPages (1, SIZE_4KB);
  if (!Buf) return EFI_OUT_OF_RESOURCES;
  ZeroMem (Buf, SIZE_4KB);

  ZeroMem (&Cmd, sizeof (Cmd));
  Cmd.cdb[0]  = SCSI_OP_REQUEST_SENSE;
  Cmd.cdb[4]  = 18;            /* Allocation Length */
  Cmd.buf     = Buf;
  Cmd.datalen = 18;
  Cmd.lun     = Lun;

  /* Ignore return value -- UA cleared even on CHECK CONDITION */
  UfsUtpCmdProcess (Ufs, &Cmd);

  DEBUG ((DEBUG_INFO, "UFS: LUN%d REQUEST SENSE sense_key=0x%02x ASC=0x%02x ASCQ=0x%02x\n",
          Lun, Buf[2] & 0x0F, Buf[12], Buf[13]));

  FreeAlignedPages (Buf, 1);
  return EFI_SUCCESS;
}

EFI_STATUS UfsRead (struct ufs_host *Ufs, u32 Lun, u64 Lba, UINTN BlkCnt, u32 BlkSize, VOID *Buf)
{
  UFS_SCSI_CMD Cmd;

  ZeroMem (&Cmd, sizeof (Cmd));
  Cmd.cdb[0]  = SCSI_OP_READ_10;
  Cmd.cdb[1]  = 0;
  Cmd.cdb[2]  = (u8)((Lba >> 24) & 0xFF);
  Cmd.cdb[3]  = (u8)((Lba >> 16) & 0xFF);
  Cmd.cdb[4]  = (u8)((Lba >>  8) & 0xFF);
  Cmd.cdb[5]  = (u8)( Lba        & 0xFF);
  Cmd.cdb[6]  = 0;
  Cmd.cdb[7]  = (u8)((BlkCnt >>  8) & 0xFF);
  Cmd.cdb[8]  = (u8)( BlkCnt        & 0xFF);
  Cmd.cdb[9]  = 0;
  Cmd.buf     = Buf;
  Cmd.datalen = (u32)(BlkCnt * BlkSize);
  Cmd.lun     = Lun;

  return UfsUtpCmdProcess (Ufs, &Cmd) ? EFI_DEVICE_ERROR : EFI_SUCCESS;
}

/* ─── Public: UfsWrite ───────────────────────────────────────────────────── */

EFI_STATUS UfsWrite (struct ufs_host *Ufs, u32 Lun, u64 Lba, UINTN BlkCnt, u32 BlkSize, VOID *Buf)
{
  UFS_SCSI_CMD Cmd;

  ZeroMem (&Cmd, sizeof (Cmd));
  Cmd.cdb[0]  = SCSI_OP_WRITE_10;
  Cmd.cdb[1]  = 0;
  Cmd.cdb[2]  = (u8)((Lba >> 24) & 0xFF);
  Cmd.cdb[3]  = (u8)((Lba >> 16) & 0xFF);
  Cmd.cdb[4]  = (u8)((Lba >>  8) & 0xFF);
  Cmd.cdb[5]  = (u8)( Lba        & 0xFF);
  Cmd.cdb[6]  = 0;
  Cmd.cdb[7]  = (u8)((BlkCnt >>  8) & 0xFF);
  Cmd.cdb[8]  = (u8)( BlkCnt        & 0xFF);
  Cmd.cdb[9]  = 0;
  Cmd.buf     = (u8 *)Buf;
  Cmd.datalen = (u32)(BlkCnt * BlkSize);
  Cmd.lun     = Lun;

  return UfsUtpCmdProcess (Ufs, &Cmd) ? EFI_DEVICE_ERROR : EFI_SUCCESS;
}
