#ifndef __DW_MMC_REGS_H__
#define __DW_MMC_REGS_H__

#define DWMCI_CTRL              0x000
#define DWMCI_PWREN             0x004
#define DWMCI_CLKDIV            0x008
#define DWMCI_CLKSRC            0x00C
#define DWMCI_CLKENA            0x010
#define DWMCI_TMOUT             0x014
#define DWMCI_CTYPE             0x018
#define DWMCI_BLKSIZ            0x01C
#define DWMCI_BYTCNT            0x020
#define DWMCI_INTMSK            0x024
#define DWMCI_CMDARG            0x028
#define DWMCI_CMD               0x02C
#define DWMCI_RESP0             0x030
#define DWMCI_RESP1             0x034
#define DWMCI_RESP2             0x038
#define DWMCI_RESP3             0x03C
#define DWMCI_MINTSTS           0x040
#define DWMCI_RINTSTS           0x044
#define DWMCI_STATUS            0x048
#define DWMCI_FIFOTH            0x04C
#define DWMCI_CDETECT           0x050
#define DWMCI_WRTPRT            0x054
#define DWMCI_GPIO              0x058
#define DWMCI_TCBCNT            0x05C
#define DWMCI_TBBCNT            0x060
#define DWMCI_DEBNCE            0x064
#define DWMCI_USRID             0x068
#define DWMCI_VERID             0x06C
#define DWMCI_HCON              0x070
#define DWMCI_UHS_REG           0x074
#define DWMCI_RST_N             0x078
#define DWMCI_BMOD              0x080
#define DWMCI_PLDMND            0x084
#define DWMCI_DBADDRL           0x088
#define DWMCI_DBADDRU           0x08C
#define DWMCI_IDSTS             0x090
#define DWMCI_IDINTEN           0x094
#define DWMCI_DSCADDRL          0x098
#define DWMCI_DSCADDRU          0x09C
#define DWMCI_BUFADDRL          0x0A0
#define DWMCI_BUFADDRU          0x0A4
#define DWMCI_CLKSEL            0x0A8
#define DWMCI_RESP_TAT          0x0AC
#define DWMCI_FORCE_CLK_STP     0x0B0
#define DWMCI_AXI_BURST_LEN     0x0B4
#define DWMCI_CARDTHRCTL        0x100
#define DWMCI_BACK_END_POWER    0x104

#define DWMCI_HS400_DQS_EN          0x180
#define DWMCI_HS400_ASYNC_FIFO_CTRL 0x184
#define DWMCI_HS400_DLINE_CTRL      0x188

#define DWMCI_FIFODAT_240A      0x200

#define CTRL_RESET              (1<<0)
#define FIFO_RESET              (1<<1)
#define DMA_RESET               (1<<2)
#define INT_ENABLE              (1<<4)
#define DMA_ENABLE              (1<<5)
#define READ_WAIT               (1<<6)
#define SEND_IRQ_RESP           (1<<7)
#define ABRT_READ_DATA          (1<<8)
#define SEND_CCSD               (1<<9)
#define SEND_AS_CCSD            (1<<10)
#define CEATA_INTSTAT           (1<<11)
#define CARD_VOLA               (0xF<<16)
#define CARD_VOLB               (0xF<<20)
#define ENABLE_OD_PULLUP        (1<<24)
#define ENABLE_IDMAC            (1<<25)

#define RESET_ALL               (CTRL_RESET | FIFO_RESET | DMA_RESET)

#define POWER_ENABLE            (1<<0)

#define CLK_DIVIDER0            (0xFF<<0)
#define CLK_DIVIDER1            (0xFF<<8)
#define CLK_ENABLE              (1<<0)
#define CLK_DISABLE             0

#define RSP_TIMEOUT             (0xFF<<0)
#define DATA_TIMEOUT            (0xFFFFFF<<8)

#define CARD_WIDTH14            (0xFFFF<<0)
#define CARD_WIDTH8_16          (0xFFFF<<16)
#define CTYPE_1BIT              0
#define CTYPE_4BIT              1
#define CTYPE_8BIT              (1<<16)

#define BLK_SIZ                 (0xFFFF<<0)

#define CMD_RESP_EXP_BIT        (1<<6)
#define CMD_RESP_LENGTH_BIT     (1<<7)
#define CMD_CHECK_CRC_BIT       (1<<8)
#define CMD_DATA_EXP_BIT        (1<<9)
#define CMD_RW_BIT              (1<<10)
#define CMD_TRANSMODE_BIT       (1<<11)
#define CMD_SENT_AUTO_STOP_BIT  (1<<12)
#define CMD_WAIT_PRV_DAT_BIT    (1<<13)
#define CMD_ABRT_CMD_BIT        (1<<14)
#define CMD_SEND_INIT_BIT       (1<<15)
#define CMD_CARD_NUM_BITS       (0x1F<<16)
#define CMD_SEND_CLK_ONLY       (1<<21)
#define CMD_READ_CEATA          (1<<22)
#define CMD_CCS_EXPECTED        (1<<23)
#define CMD_USE_HOLD_REG        (1<<29)
#define CMD_STRT_BIT            (1<<31)

#define CMD_ONLY_CLK            (CMD_STRT_BIT | CMD_SEND_CLK_ONLY | \
                                 CMD_WAIT_PRV_DAT_BIT)

#define INTMSK_CDETECT          (1<<0)
#define INTMSK_RE               (1<<1)
#define INTMSK_CDONE            (1<<2)
#define INTMSK_DTO              (1<<3)
#define INTMSK_TXDR             (1<<4)
#define INTMSK_RXDR             (1<<5)
#define INTMSK_RCRC             (1<<6)
#define INTMSK_DCRC             (1<<7)
#define INTMSK_RTO              (1<<8)
#define INTMSK_DRTO             (1<<9)
#define INTMSK_HTO              (1<<10)
#define INTMSK_FRUN             (1<<11)
#define INTMSK_HLE              (1<<12)
#define INTMSK_SBE              (1<<13)
#define INTMSK_ACD              (1<<14)
#define INTMSK_EBE              (1<<15)
#define INTMSK_DMA              (INTMSK_ACD | INTMSK_RXDR | INTMSK_TXDR)
#define INTMSK_ALL              0xFFFFFFFF

// Short names
#define INT_CDET                INTMSK_CDETECT
#define INT_RE                  INTMSK_RE
#define INT_CDONE               INTMSK_CDONE
#define INT_DTO                 INTMSK_DTO
#define INT_RCRC                INTMSK_RCRC
#define INT_DCRC                INTMSK_DCRC
#define INT_RTO                 INTMSK_RTO
#define INT_DRTO                INTMSK_DRTO
#define INT_HTO                 INTMSK_HTO
#define INT_HLE                 INTMSK_HLE
#define INT_SBE                 INTMSK_SBE
#define INT_EBE                 INTMSK_EBE
#define INT_ALL                 INTMSK_ALL

// Err groups
#define DATA_ERR                (INTMSK_EBE | INTMSK_SBE | INTMSK_HLE | \
                                 INTMSK_FRUN | INTMSK_DCRC)
#define DATA_TOUT               (INTMSK_HTO | INTMSK_DRTO)
#define DATA_STATUS             (DATA_ERR | DATA_TOUT | INTMSK_RXDR | \
                                 INTMSK_TXDR | INTMSK_DTO)
#define CMD_STATUS              (INTMSK_RTO | INTMSK_RCRC | \
                                 INTMSK_CDONE | INTMSK_RE)
#define CMD_ERROR               (INTMSK_RCRC | INTMSK_RTO | INTMSK_RE)

#define FIFO_RXWTRMARK          (1<<0)
#define FIFO_TXWTRMARK          (1<<1)
#define FIFO_EMPTY              (1<<2)
#define FIFO_FULL               (1<<3)
#define CMD_FSMSTAT             (0xF<<4)
#define DATA_3STATUS            (1<<8)
#define DATA_BUSY               (1<<9)
#define DATA_MCBUSY             (1<<10)
#define RSP_INDEX               (0x3F<<11)
#define FIFO_COUNT              (0x1FFF<<17)
#define DMA_ACK                 (1<<30)
#define DMA_REQ                 (1<<31)

#define TX_WMARK                (0xFFF<<0)
#define RX_WMARK                (0xFFF<<16)
#define MSIZE_MASK              (0x7<<28)
#define FIFOTH_ALL              (TX_WMARK | RX_WMARK | MSIZE_MASK)

#define MSIZE_1                 (0<<28)
#define MSIZE_4                 (1<<28)
#define MSIZE_8                 (2<<28)
#define MSIZE_16                (3<<28)
#define MSIZE_32                (4<<28)
#define MSIZE_64                (5<<28)
#define MSIZE_128               (6<<28)
#define MSIZE_256               (7<<28)

#define BMOD_IDMAC_RESET        (1<<0)
#define BMOD_IDMAC_FB           (1<<1)
#define BMOD_IDMAC_ENABLE       (1<<7)

#define IDSTS_FSM               (0xF<<13)
#define IDSTS_EB                (0x7<<10)
#define IDSTS_AIS               (1<<9)
#define IDSTS_NIS               (1<<8)
#define IDSTS_CES               (1<<5)
#define IDSTS_DU                (1<<4)
#define IDSTS_FBE               (1<<2)
#define IDSTS_RI                (1<<1)
#define IDSTS_TI                (1<<0)

#define CLKSEL_SELCLK_SAMPLE        (0x7<<0)
#define CLKSEL_SAMPLE_CLK_TUNING    (0x3<<6)
#define CLKSEL_SAMPLE_CLK_TUNING_1  (1<<6)
#define CLKSEL_SAMPLE_CLK_ALL       (CLKSEL_SELCLK_SAMPLE | \
                                     CLKSEL_SAMPLE_CLK_TUNING)

#define EXYNOS_CLKSEL_CCLK_SAMPLE(x)       (((x) & 0x7) << 0)
#define EXYNOS_CLKSEL_CCLK_FINE_SAMPLE(x)  (((x) & 0xF) << 0)
#define EXYNOS_CLKSEL_CCLK_DRIVE(x)        (((x) & 0x7) << 16)
#define EXYNOS_CLKSEL_CCLK_FINE_DRIVE(x)   (((x) & 0x3) << 22)
#define EXYNOS_CLKSEL_CCLK_DIVIDER(x)      (((x) & 0x7) << 24)
#define EXYNOS_CLKSEL_FINE_TUNED           (1 << 6)
#define EXYNOS_CLKSEL_WAKEUP_INT           (1 << 11)
#define EXYNOS_CLKSEL_ULP_MODE             ((1 << 30) | (1 << 19))

#define EXYNOS_CLKSEL_TIMING(div, f_drv, drv, sample) \
  (EXYNOS_CLKSEL_CCLK_DIVIDER (div)    | \
   EXYNOS_CLKSEL_CCLK_FINE_DRIVE (f_drv) | \
   EXYNOS_CLKSEL_CCLK_DRIVE (drv)      | \
   EXYNOS_CLKSEL_CCLK_SAMPLE (sample))

#define EXYNOS_CLKSEL_TIMING_MASK \
  (EXYNOS_CLKSEL_TIMING (0x7, 0x3, 0x7, 0x7))

#define EXYNOS_CLKSEL_UP_SAMPLE(x, y) \
  (((x) & ~EXYNOS_CLKSEL_CCLK_SAMPLE (7)) | EXYNOS_CLKSEL_CCLK_SAMPLE (y))

#define DATA_STROBE_EN                  (1 << 0)
#define HS400_DQS_RCLK_MODE             (1 << 5)
#define HS400_AXI_NON_BLOCKING_WRITE    (1 << 7)
#define HS400_TXDT_CRC_TIMER_FASTLIMIT(x) (((x) & 0xFF) << 16)
#define HS400_TXDT_CRC_TIMER_INITVAL(x)   (((x) & 0xFF) << 8)
#define HS400_TXDT_CRC_TIMER_SET(x, y) \
  (HS400_TXDT_CRC_TIMER_FASTLIMIT (x) | HS400_TXDT_CRC_TIMER_INITVAL (y))
#define HS400_DQS_EN_DEFAULT \
  (HS400_TXDT_CRC_TIMER_SET (0x13, 0x15))

#define HS400_FIFO_CLK_DELAY_CTRL(x)  (((x) & 0x3) << 16)
#define HS400_RD_DQS_DELAY_CTRL(x)    ((x) & 0x3FF)
#define HS400_DLINE_CTRL_DEFAULT \
  (HS400_FIFO_CLK_DELAY_CTRL (0x2) | HS400_RD_DQS_DELAY_CTRL (0x40))

#define AXI_WRDMA_BURST_LEN(x)  ((0xF & (x)) << 16)
#define AXI_RDDMA_BURST_LEN(x)  ((0xF & (x)) << 0)
#define AXI_BURST_LEN(x)        (AXI_WRDMA_BURST_LEN(x) | \
                                 AXI_RDDMA_BURST_LEN(x))
#define AXI_SAMPLING_PATH_SEL   (1<<31)

#define UHS_REG_DDR             (1<<16)
#define UHS_REG_V18             (1<<0)

#define CMD0_GO_IDLE            0
#define CMD1_SEND_OP_COND       1
#define CMD2_ALL_SEND_CID       2
#define CMD3_SEND_REL_ADDR      3
#define CMD6_SWITCH_FUNC        6
#define CMD7_SELECT_CARD        7
#define CMD8_SEND_EXT_CSD       8
#define CMD8_SEND_IF_COND       8
#define CMD9_SEND_CSD           9
#define CMD11_SWITCH_VOLTAGE    11
#define CMD12_STOP_TRAN         12
#define CMD13_SEND_STATUS       13
#define CMD16_SET_BLOCKLEN      16
#define CMD17_READ_SINGLE       17
#define CMD18_READ_MULTI        18
#define CMD19_SEND_TUNING       19
#define CMD21_SEND_TUNING_BLOCK 21
#define CMD23_SET_BLKCNT        23
#define CMD24_WRITE_SINGLE      24
#define CMD25_WRITE_MULTI       25
#define CMD55_APP_CMD           55

#define ACMD6_SET_BUS_WIDTH     6
#define ACMD41_SD_SEND_OP       41
#define ACMD51_SEND_SCR         51

#define MMC_RSP_PRESENT         (1<<0)
#define MMC_RSP_136             (1<<1)
#define MMC_RSP_CRC             (1<<2)
#define MMC_RSP_BUSY            (1<<3)
#define MMC_RSP_OPCODE          (1<<4)

#define OCR_BUSY                (1UL<<31)
#define OCR_HCS                 (1<<30)
#define OCR_S18R                (1<<24)
#define OCR_XPC                 (1<<28)
#define OCR_SEC_MODE            (2<<29)
#define OCR_VOLT_27_36          (0x1FF<<15)
#define OCR_VOLT_MASK           0xFF8000
#define SD_HC_OCR               (OCR_VOLT_27_36 | OCR_HCS | OCR_XPC | OCR_S18R)
#define SD_OCR                  (OCR_VOLT_27_36)
#define SD_SEND_IF_COND_ARG     0x000001AA

#define R1_STATUS_MASK          (~0x0206BF7F)
#define R1_AKE_SEQ_ERROR        (1<<3)
#define R1_APP_CMD              (1<<5)
#define R1_READY_FOR_DATA       (1<<8)
#define R1_CURRENT_STATE_MASK   (0xF<<9)
#define R1_STATE_IDLE           (0<<9)
#define R1_STATE_STBY           (3<<9)
#define R1_STATE_TRAN           (4<<9)
#define R1_STATE_DATA           (5<<9)
#define R1_STATE_RCV            (6<<9)
#define R1_STATE_PRG            (7<<9)
#define R1_STATE_DIS            (8<<9)
#define R1_ERASE_RESET          (1<<13)
#define R1_ERROR                (1<<19)
#define R1_ILLEGAL_COMMAND      (1<<22)
#define R1_COM_CRC_ERROR        (1<<23)
#define R1_WP_VIOLATION         (1<<26)
#define R1_BLOCK_LEN_ERROR      (1<<29)
#define R1_ADDRESS_ERROR        (1<<30)
#define R1_OUT_OF_RANGE         (1<<31)

#define WRTPRT_WRITE_PROTECT    (1<<0)

#define SCR_BUS_WIDTH_1         (1 << 8)
#define SCR_BUS_WIDTH_4         (1 << 10)
#define SCR_BUS_WIDTH_8         (1 << 11)

#define EXT_CSD_SEC_CNT             212
#define EXT_CSD_HS_TIMING           185
#define EXT_CSD_BUS_WIDTH           183
#define EXT_CSD_CARD_TYPE           196
#define EXT_CSD_REV                 192
#define EXT_CSD_BOOT_MULT           226
#define EXT_CSD_RPMB_MULT           168
#define EXT_CSD_ERASE_GROUP_DEF     175
#define EXT_CSD_HC_ERASE_GRP_SIZE   224
#define EXT_CSD_SEC_FEATURE         231
#define EXT_CSD_RST_N_FUNCTION      162
#define EXT_CSD_BOOT_WP             173
#define EXT_CSD_BOOT_BUS_WIDTH      177
#define EXT_CSD_PART_CONF           179
#define EXT_CSD_POWER_OFF_NOTIFY    34
#define EXT_CSD_HC_WP_GRP_SIZE      221

// EXT_CSD HS_TIMING values
#define EXT_CSD_RST_N_ENABLE    1

//
#define EXT_CSD_HS_TIMING_DEFAULT    0
#define EXT_CSD_HS_TIMING_HS         1
#define EXT_CSD_HS_TIMING_HS200      2
#define EXT_CSD_HS_TIMING_HS400      3

// EXT_CSD BUS_WIDTH values
#define EXT_CSD_BUS_WIDTH_1BIT_SDR    0
#define EXT_CSD_BUS_WIDTH_4BIT_SDR    1
#define EXT_CSD_BUS_WIDTH_8BIT_SDR    2
#define EXT_CSD_BUS_WIDTH_4BIT_DDR    5
#define EXT_CSD_BUS_WIDTH_8BIT_DDR    6

// EXT_CSD card types
#define MMC_HS_26MHZ           (1<<0)
#define MMC_HS_52MHZ           (1<<1)
#define MMC_HS_52MHZ_DDR_1_8V  (1<<2)
#define MMC_HS_52MHZ_DDR_1_2V  (1<<3)
#define MMC_HS_200MHZ_SDR      (1<<4)
#define MMC_HS_400MHZ_DDR      (1<<5)

// SD card capabilities
#define SD_HS_SDR25            (1<<1)
#define SD_UHS_SDR50           (1<<2)
#define SD_UHS_SDR104          (1<<3)

// Tuning block size (64 bytes for CMD21)
#define TUNING_BLOCK_SIZE       64

#define MMC_SWITCH_MODE_WRITE_BYTE 0x03
#define SD_SWITCH_MODE_CHECK        0
#define SD_SWITCH_MODE_SWITCH       1

#define CARD_TYPE_MMC           0
#define CARD_TYPE_SD            1
#define CARD_TYPE_SDHC          2
#define CARD_TYPE_UNKNOWN       0xFF

#define MMC_MAX_BLOCK_LEN       512

typedef struct {
  UINT32 Des0;
  UINT32 Des1;
  UINT32 Des2;
  UINT32 Des3;
  UINT32 Des4;
  UINT32 Des5;
  UINT32 Des6;
  UINT32 Des7;
  UINT32 Des8;
  UINT32 Des9;
  UINT32 Des10;
  UINT32 Des11;
  UINT32 Des12[4];
} DWMMC_IDMAC_DESC;

#define DWMCI_IDMAC_OWN         (1<<31)
#define DWMCI_IDMAC_ER          (1<<5)
#define DWMCI_IDMAC_CH          (1<<4)
#define DWMCI_IDMAC_FS          (1<<3)
#define DWMCI_IDMAC_LD          (1<<2)
#define DWMCI_IDMAC_DIC         (1<<1)

typedef struct {
  //
  // Hardware configuration (from platform library)
  //
  UINTN       IoBase;
  UINT32      BusHz;
  UINT32      PhaseDivide;
  UINT32      SdrClksel;
  UINT32      DdrClksel;
  UINT32      Sdr50Clksel;
  UINT32      Sdr104Clksel;
  UINT32      Hs200Clksel;
  UINT32      Hs400Clksel;
  UINT32      Ciudiv;
  UINT32      FifoDepth;

  //
  // Clock state
  //
  UINT32      CardClock;
  UINT32      MinClock;
  UINT32      MaxClock;

  //
  // Tuning state (HS200/HS400)
  //
  UINT32      TunedSamplePhase;
  BOOLEAN     IsTuned;
  BOOLEAN     IsFineTuned;

  //
  // Card state
  //
  UINT32      CardRca;
  UINT32      CardType;
  UINT32      BusWidth;
  UINT32      BusMode;
  UINT32      OcR;
  UINT32      Csd[4];
  UINT32      Scr[2];
  UINT32      Version;
  BOOLEAN     CardPresent;
  BOOLEAN     IsSdhc;
  BOOLEAN     IsHighSpeed;
  BOOLEAN     IsUhsSupported;
  BOOLEAN     InitDone;
  UINT32      BootSize;
  UINT32      RpmbSize;
  UINT32      CardCaps;
  UINT32      NumBlocks;
  UINT32      BlockSize;
  UINT64      Capacity;
} DW_MMC_HOST;

#endif // __DW_MMC_REGS_H__
