/** @file
  Exynos 9830 UFS Host Controller Driver - Common Header
  Adapted from Samsung lk3rd UFS driver (dev/scsi/ufs.c + platform/exynos9830/)

  Copyright (c) 2024, EDK2 Port Authors.
  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef EXYNOS_UFS_H_
#define EXYNOS_UFS_H_

#include <Uefi.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/DebugLib.h>
#include <Library/TimerLib.h>
#include <Library/IoLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Protocol/BlockIo.h>
#include <Protocol/DevicePath.h>

#include <Library/DebugLib.h>

// ─── LK type aliases ────────────────────────────────────────────────────────
typedef UINT8   u8;
typedef UINT16  u16;
typedef UINT32  u32;
typedef UINT64  u64;

// ─── Register macros ────────────────────────────────────────────────────────
#define readl(addr)    MmioRead32 ((UINTN)(addr))
#define writel(val, addr)    MmioWrite32 ((UINTN)(addr), (UINT32)(val))

// ─── cpu_to_be / swab helpers ───────────────────────────────────────────────
#define ___swab16(x) \
  ((((x) & 0xff00u) >> 8) | (((x) & 0x00ffu) << 8))

#define ___swab32(x) \
  ((u32)( \
    (((u32)(x) & 0x000000ffUL) << 24) | \
    (((u32)(x) & 0x0000ff00UL) <<  8) | \
    (((u32)(x) & 0x00ff0000UL) >>  8) | \
    (((u32)(x) & 0xff000000UL) >> 24)))

#define cpu_to_be16(x)   ___swab16(x)
#define cpu_to_be32(x)   ___swab32(x)
#define be32_to_cpu(x)   ___swab32(x)

// ─── Delay helpers ──────────────────────────────────────────────────────────
#define u_delay(us)      MicroSecondDelay (us)
#define mdelay(ms)       MicroSecondDelay ((ms) * 1000ULL)

// ─── Memory barriers ────────────────────────────────────────────────────────
#define wmb()            MemoryFence ()

// ─── Error codes ────────────────────────────────────────────────────────────
#define NO_ERROR    0
#define ERR_GENERIC (-1)
#define MIN(a,b)    (((a) < (b)) ? (a) : (b))

// ─── UFS Timeouts ───────────────────────────────────────────────────────────
#define NOP_OUT_TIMEOUT             30000UL      /* 30 ms  */
#define NOP_OUT_RETRY               10
#define UTP_CMD_TIMEOUT             10000000UL   /* 10 s   */
#define UIC_CMD_TIMEOUT             1500000UL    /* 1500 ms */
#define QUERY_REQ_TIMEOUT           1500000UL
#define FORMAT_CMD_TIMEOUT          (10UL * 60UL * 1000UL * 1000UL)
#define QUERY_REQ_RETRIES           3

// ─── Buffer sizing ──────────────────────────────────────────────────────────
#define MAX_CDB_SIZE                16
#define ALIGNED_UPIU_SIZE           1024
#define SCSI_MAX_SG_SEGMENTS        128
#define UFS_SG_BLOCK_SIZE_BIT       12
#define UFS_SG_BLOCK_SIZE           (1 << UFS_SG_BLOCK_SIZE_BIT)
#define UFS_NUTRS                   2
#define UFS_BIT_LEN_OF_DWORD        32
#define UPIU_DATA_SIZE              (ALIGNED_UPIU_SIZE - 20 - sizeof(struct ufs_upiu_header))

// ─── Platform addresses (Exynos 9610) ───────────────────────────────────────
#define UFS_BASE_ADDR               0x13520000UL
#define UFS_VS_ADDR_OFFSET          0x1100UL
#define UFS_FMP_BASE_ADDR           0x13530000UL
#define UFS_UNIPRO_BASE_ADDR        0x13510000UL
#define UFS_PHY_PMA_OFFSET          0x4000UL

#define UFS_SCLK                    166000000UL
#define CNT_VAL_1US_MASK            0x3FFU
#define UFSHCI_VS_1US_TO_CNT_VAL    0x110CU
#define UFSHCI_VS_UFSHCI_V2P1_CTRL  0x118CU
#define IA_TICK_SEL                 (1U << 16)

#define MUX_CLKCMU_UFS_EMBD_CON    0x1A331098UL
#define DIV_CLKCMU_UFS_EMBD_MUX    0x1A331890UL
#define UFS_CLKCMU_TIMEOUT          100

#define EXYNOS9610_POWER_BASE                            0x11860000UL
#define EXYNOS9610_POWER_RST_STAT 						 (EXYNOS9610_POWER_BASE + 0x0404UL)
#define EXYNOS9610_POWER_RESET_SEQUENCER_CONFIGURATION	 (EXYNOS9610_POWER_BASE + 0x0500UL)
#define WARM_RESET                  (1U << 28)
#define LITTLE_WDT_RESET            (1U << 24)
#define EXYNOS9610_EDPCSR_DUMP_EN   (1U << 0)

// ─── UFSHCI Standard Registers ──────────────────────────────────────────────
enum {
  REG_CONTROLLER_CAPABILITIES              = 0x00,
  REG_UFS_VERSION                          = 0x08,
  REG_CONTROLLER_PID                       = 0x10,
  REG_CONTROLLER_MID                       = 0x14,
  REG_INTERRUPT_STATUS                     = 0x20,
  REG_INTERRUPT_ENABLE                     = 0x24,
  REG_CONTROLLER_STATUS                    = 0x30,
  REG_CONTROLLER_ENABLE                    = 0x34,
  REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER     = 0x38,
  REG_UIC_ERROR_CODE_DATA_LINK_LAYER       = 0x3C,
  REG_UIC_ERROR_CODE_NETWORK_LAYER         = 0x40,
  REG_UIC_ERROR_CODE_TRANSPORT_LAYER       = 0x44,
  REG_UIC_ERROR_CODE_DME                   = 0x48,
  REG_UTP_TRANSFER_REQ_INT_AGG_CONTROL     = 0x4C,
  REG_UTP_TRANSFER_REQ_LIST_BASE_L         = 0x50,
  REG_UTP_TRANSFER_REQ_LIST_BASE_H         = 0x54,
  REG_UTP_TRANSFER_REQ_DOOR_BELL          = 0x58,
  REG_UTP_TRANSFER_REQ_LIST_CLEAR         = 0x5C,
  REG_UTP_TRANSFER_REQ_LIST_RUN_STOP      = 0x60,
  REG_UTP_TRANSFER_REQ_LIST_CNR           = 0x64,
  REG_UTP_TASK_REQ_LIST_BASE_L            = 0x70,
  REG_UTP_TASK_REQ_LIST_BASE_H            = 0x74,
  REG_UTP_TASK_REQ_DOOR_BELL             = 0x78,
  REG_UTP_TASK_REQ_LIST_CLEAR            = 0x7C,
  REG_UTP_TASK_REQ_LIST_RUN_STOP         = 0x80,
  REG_UIC_COMMAND                         = 0x90,
  REG_UIC_COMMAND_ARG_1                   = 0x94,
  REG_UIC_COMMAND_ARG_2                   = 0x98,
  REG_UIC_COMMAND_ARG_3                   = 0x9C,
  REG_CRYPTO_CAPABILITY                   = 0x100,
};

// ─── Vendor-Specific Offsets (from vs_addr = base + 0x1100) ─────────────────
enum {
  VS_TXPRDT_ENTRY_SIZE      = 0x000,
  VS_RXPRDT_ENTRY_SIZE      = 0x004,
  VS_IS                     = 0x038,
  VS_UTRL_NEXUS_TYPE        = 0x040,
  VS_UMTRL_NEXUS_TYPE       = 0x044,
  VS_SW_RST                 = 0x050,
  VS_DATA_REORDER           = 0x060,
  VS_AXIDMA_RWDATA_BURST_LEN= 0x06C,
  VS_GPIO_OUT               = 0x070,
  VS_CLKSTOP_CTRL           = 0x0B0,
  VS_FORCE_HCS              = 0x0B4,
  VS_UFS_ACG_DISABLE        = 0x0FC,
  VS_MPHY_REFCLK_SEL        = 0x108,
};

// ─── Interrupt bits ─────────────────────────────────────────────────────────
#define UFS_BIT(x)                      (1UL << (x))
#define UTP_TRANSFER_REQ_COMPL          UFS_BIT(0)
#define UIC_ERROR                       UFS_BIT(2)
#define UIC_POWER_MODE                  UFS_BIT(4)
#define UIC_LINK_LOST                   UFS_BIT(7)
#define UIC_LINK_STARTUP                UFS_BIT(8)
#define UTP_TASK_REQ_COMPL              UFS_BIT(9)
#define UIC_COMMAND_COMPL               UFS_BIT(10)
#define DEVICE_FATAL_ERROR              UFS_BIT(11)
#define CONTROLLER_FATAL_ERROR          UFS_BIT(16)
#define SYSTEM_BUS_FATAL_ERROR          UFS_BIT(17)
#define INT_FATAL_ERRORS                (DEVICE_FATAL_ERROR | CONTROLLER_FATAL_ERROR | SYSTEM_BUS_FATAL_ERROR | UIC_LINK_LOST)

// ─── UIC Commands ───────────────────────────────────────────────────────────
enum {
  UIC_CMD_DME_GET          = 0x01,
  UIC_CMD_DME_SET          = 0x02,
  UIC_CMD_DME_PEER_GET     = 0x03,
  UIC_CMD_DME_PEER_SET     = 0x04,
  UIC_CMD_DME_POWERON      = 0x10,
  UIC_CMD_DME_POWEROFF     = 0x11,
  UIC_CMD_DME_ENABLE       = 0x12,
  UIC_CMD_DME_RESET        = 0x14,
  UIC_CMD_DME_LINK_STARTUP = 0x16,
  UIC_CMD_DME_HIBER_ENTER  = 0x17,
  UIC_CMD_DME_HIBER_EXIT   = 0x18,
};

// ─── Lane / MIB addressing ──────────────────────────────────────────────────
enum {
  TX_LANE_0 = 0, TX_LANE_1, TX_LANE_2, TX_LANE_3,
  RX_LANE_0 = 4, RX_LANE_1, RX_LANE_2, RX_LANE_3,
};
#define UIC_ARG_MIB_SEL(attr, sel) ((((attr) & 0xFFFFU) << 16) | ((sel) & 0xFFFFU))
#define UIC_ARG_MIB(attr)           UIC_ARG_MIB_SEL(attr, 0)

// ─── UPIU Transaction Types ─────────────────────────────────────────────────
enum {
  UPIU_TRANSACTION_NOP_OUT    = 0x00,
  UPIU_TRANSACTION_COMMAND    = 0x01,
  UPIU_TRANSACTION_DATA_OUT   = 0x02,
  UPIU_TRANSACTION_TASK_REQ   = 0x04,
  UPIU_TRANSACTION_QUERY_REQ  = 0x16,
  UPIU_TRANSACTION_NOP_IN     = 0x20,
  UPIU_TRANSACTION_RESPONSE   = 0x21,
  UPIU_TRANSACTION_DATA_IN    = 0x22,
  UPIU_TRANSACTION_QUERY_RSP  = 0x36,
};

// ─── UTP Transfer Request flags ─────────────────────────────────────────────
enum {
  UTP_NO_DATA_TRANSFER  = 0x00000000,
  UTP_HOST_TO_DEVICE    = 0x02000000,
  UTP_DEVICE_TO_HOST    = 0x04000000,
  UTP_SCSI_COMMAND      = 0x00000000,
  UTP_NATIVE_UFS_COMMAND= 0x10000000,
  UTP_DEVICE_MANAGEMENT = 0x20000000,
  UTP_REQ_DESC_INT_CMD  = 0x01000000,
};

// ─── OCS values ─────────────────────────────────────────────────────────────
enum {
  OCS_SUCCESS                 = 0x0,
  OCS_INVALID_CMD_TABLE_ATTR  = 0x1,
  OCS_INVALID_PRDT_ATTR       = 0x2,
  OCS_MISMATCH_DATA_BUF_SIZE  = 0x3,
  OCS_MISMATCH_RESP_UPIU_SIZE = 0x4,
  OCS_PEER_COMM_FAILURE       = 0x5,
  OCS_ABORTED                 = 0x6,
  OCS_FATAL_ERROR             = 0x7,
  OCS_INVALID_COMMAND_STATUS  = 0x0F,
  MASK_OCS                    = 0x0F,
};

// ─── UPIU Flags ─────────────────────────────────────────────────────────────
enum {
  UPIU_CMD_FLAGS_NONE  = 0x00,
  UPIU_CMD_FLAGS_WRITE = 0x20,
  UPIU_CMD_FLAGS_READ  = 0x40,
};

// ─── Query function types ────────────────────────────────────────────────────
enum {
  UFS_STD_READ_REQ    = 0x01,
  UFS_STD_WRITE_REQ   = 0x81,
};

// ─── Query opcodes ──────────────────────────────────────────────────────────
enum {
  UPIU_QUERY_OPCODE_NOP          = 0x0,
  UPIU_QUERY_OPCODE_READ_DESC    = 0x1,
  UPIU_QUERY_OPCODE_WRITE_DESC   = 0x2,
  UPIU_QUERY_OPCODE_READ_ATTR    = 0x3,
  UPIU_QUERY_OPCODE_WRITE_ATTR   = 0x4,
  UPIU_QUERY_OPCODE_READ_FLAG    = 0x5,
  UPIU_QUERY_OPCODE_SET_FLAG     = 0x6,
  UPIU_QUERY_OPCODE_CLEAR_FLAG   = 0x7,
  UPIU_QUERY_OPCODE_TOGGLE_FLAG  = 0x8,
};

// ─── Descriptor IDs ─────────────────────────────────────────────────────────
enum {
  UPIU_DESC_ID_DEVICE        = 0x0,
  UPIU_DESC_ID_CONFIGURATION = 0x1,
  UPIU_DESC_ID_UNIT          = 0x2,
  UPIU_DESC_ID_INTERCONNECT  = 0x4,
  UPIU_DESC_ID_STRING        = 0x5,
  UPIU_DESC_ID_GEOMETRY      = 0x7,
  UPIU_DESC_ID_POWER         = 0x8,
};

// ─── Flag IDs ───────────────────────────────────────────────────────────────
enum {
  UPIU_FLAG_ID_DEVICEINIT       = 0x1,
  UPIU_FLAG_ID_PER_WRITEPROTECT = 0x2,
  UPIU_FLAG_ID_POW_WRITEPROTECT = 0x3,
  UPIU_FLAG_ID_BG_OPERATRION    = 0x4,
  UPIU_FLAG_ID_PURGE_ENABLE     = 0x6,
};

// ─── Attribute IDs ──────────────────────────────────────────────────────────
enum {
  UPIU_ATTR_ID_BOOTLUNEN        = 0x0,
  UPIU_ATTR_ID_POWERMODE        = 0x2,
  UPIU_ATTR_ID_ACTIVECCLEVEL    = 0x3,
  UPIU_ATTR_ID_MAXDATAIN        = 0x7,
  UPIU_ATTR_ID_MAXDATAOUT       = 0x8,
  UPIU_ATTR_ID_REFCLKFREQ       = 0xa,
  UPIU_ATTR_ID_CONFIGDESCLOCK   = 0xb,
  UPIU_ATTR_ID_MAXNUMRTT        = 0xc,
};

#define UPMCRS(x)      (((x) >> 8) & 7)
#define PWR_LOCAL       1

// ─── UFS transfer status ────────────────────────────────────────────────────
enum {
  UFS_NO_ERROR    = 0,
  UFS_TIMEOUT,
  UFS_ERROR,
  UFS_IN_PROGRESS,
};

// ─── UFS quirks ─────────────────────────────────────────────────────────────
#define UFS_QUIRK_USE_1LANE    (1 << 1)
#define UFS_QUIRK_BROKEN_HCE   (1 << 2)

// ─── UFS rate/power ──────────────────────────────────────────────────────────
#define UFS_RATE              2
#define UFS_POWER_MODE        1
#define UFS_RXTX_POWER_MODE   ((UFS_POWER_MODE << 4) | UFS_POWER_MODE)

// ─── SCSI opcodes used ───────────────────────────────────────────────────────
#define SCSI_OP_REQUEST_SENSE  0x03
#define SCSI_OP_INQUIRY        0x12
#define SCSI_OP_READ_10        0x28
#define SCSI_OP_WRITE_10       0x2A
#define SCSI_OP_READ_CAPACITY  0x25

// ─── SCSI status codes ───────────────────────────────────────────────────────
#define SCSI_STATUS_GOOD            0x00
#define SCSI_STATUS_CHECK_CONDITION 0x02
#define SCSI_OP_FORMAT_UNIT    0x04
#define SCSI_OP_UNMAP          0x42
#define SCSI_OP_WRITE_BUFFER   0x3B
#define SCSI_OP_SECU_PROT_OUT  0xB5
#define SCSI_OP_START_STOP_UNIT 0x1B
#define SCSI_MODE_SEL10        0x55
#define SCSI_MODE_SEN10        0x5A

// ─── Query index enum (mirrors LK) ──────────────────────────────────────────
typedef enum {
  FLAG_W_FDEVICEINIT = 1,
  FLAG_R_FDEVICEINIT,
  DESC_R_DEVICE_DESC,
  DESC_W_CONFIG_DESC,
  DESC_R_CONFIG_DESC,
  DESC_R_UNIT_DESC,
  DESC_R_GEOMETRY_DESC,
  ATTR_W_BOOTLUNEN,
  ATTR_R_BOOTLUNEN,
  ATTR_W_REFCLKFREQ,
  ATTR_R_REFCLKFREQ,
} query_index;

// ─── UPIU header dword helper ────────────────────────────────────────────────
#define UPIU_HEADER_DWORD(b3,b2,b1,b0) \
  (((u32)(b3) << 24) | ((u32)(b2) << 16) | ((u32)(b1) << 8) | (u32)(b0))

// ─── CAL types (from ufs-cal-9830.h) ────────────────────────────────────────
struct uic_pwr_mode {
  u8  lane;
  u8  gear;
  u8  mode;
  u8  hs_series;
};

enum { HOST_EMBD = 0, HOST_CARD = 1 };
enum { GEAR_1 = 1, GEAR_2, GEAR_3, GEAR_4 };

struct ufs_cal_param {
  void  *host;
  u8 available_lane;
  u8 target_lane;
  u8     connected_tx_lane;
  u8     connected_rx_lane;
  u8     active_tx_lane;
  u8     active_rx_lane;
  u32    mclk_rate;
  u8     tbl;
  u8     board;
  struct uic_pwr_mode *pmd;
};

typedef enum {
  UFS_CAL_NO_ERROR = 0,
  UFS_CAL_TIMEOUT,
  UFS_CAL_ERROR,
  UFS_CAL_INV_ARG,
} ufs_cal_errno;

enum {
  __BRD_SMDK, __BRD_ASB, __BRD_HSIE, __BRD_ZEBU, __BRD_UNIV, __BRD_MAX,
};
#define BRD_SMDK  (1U << __BRD_SMDK)
#define BRD_ASB   (1U << __BRD_ASB)
#define BRD_HSIE  (1U << __BRD_HSIE)
#define BRD_ZEBU  (1U << __BRD_ZEBU)
#define BRD_UNIV  (1U << __BRD_UNIV)
#define BRD_MAX   (1U << __BRD_MAX)
#define BRD_ALL   ((1U << __BRD_MAX) - 1)

// ─── UPIU structures ─────────────────────────────────────────────────────────
#pragma pack(1)

struct ufs_upiu_header {
  u8   type;
  u8   flags;
  u8   lun;
  u8   tag;
  u8   cmdtype;
  u8   function;
  u8   response;
  u8   status;
  u8   ehslength;
  u8   deviceinfo;
  u16  datalength;
};

#define DW_NUM_OF_TSF 20

struct ufs_upiu {
  struct ufs_upiu_header header;
  u8     tsf[DW_NUM_OF_TSF];
  u8     data[UPIU_DATA_SIZE];
};

struct ufs_prdt {
  u32  base_addr;
  u32  upper_addr;
  u32  reserved;
  u32  size;
};

struct ufs_cmd_desc {
  struct ufs_upiu command_upiu;
  struct ufs_upiu response_upiu;
  struct ufs_prdt prd_table[SCSI_MAX_SG_SEGMENTS];
};

struct ufs_utrd {
  u32  dw[4];
  u32  cmd_desc_addr_l;
  u32  cmd_desc_addr_h;
  u16  rsp_upiu_len;
  u16  rsp_upiu_off;
  u16  prdt_len;
  u16  prdt_off;
};

struct ufs_utmrd {
  u32  dw[8];
};

#pragma pack()

// ─── UFS Descriptor structures (from lk3rd include/dev/ufs.h) ───────────────
#pragma pack(1)

struct ufs_config_desc_header {
  u8   bLength;
  u8   bDescriptorType;
  u8   bConfDescContinue;
  u8   bBootEnable;
  u8   bDescrAccessEn;
  u8   bInitPowerMode;
  u8   bHighPriorityLUN;
  u8   bSecureRemovalType;
  u8   bInitActiveICCLevel;
  u16  wPeriodicRTCUpdate;
  u8   reserved[5];
  u8   bTurboWriteBufferNoUserSpaceReductionEn;
  u8   bTurboWriteBufferType;
};

struct ufs_unit_desc_param {
  u8   bLUEnable;
  u8   bBootLunID;
  u8   bLUWriteProtect;
  u8   bMemoryType;
  u32  dNumAllocUnits;
  u8   bDataReliability;
  u8   bLogicalBlockSize;
  u8   bProvisioningType;
  u16  wContextCapabilities;
  u8   reserved[3];
  u8   reserved_1[6];
  u32  dLUNumTurboWriteBufferAllocUnits;
};

struct ufs_config_desc {
  struct ufs_config_desc_header header;
  struct ufs_unit_desc_param    unit[128];
};

struct ufs_device_desc {
  u8   bLength;
  u8   bDescriptorType;
  u8   bDevice;
  u8   bDeviceClass;
  u8   bDeviceSubClass;
  u8   bProtocol;
  u8   bNumberLU;
  u8   iNumberWLU;
  u8   bBootEnable;
  u8   bDescrAccessEn;
  u8   bInitPowerMode;
  u8   bHighPriorityLUN;
  u8   bSecureRemovalType;
  u8   bSecurityLU;
  u8   reserved;
  u8   bInitActiveICCLevel;
  u16  wSpecVersion;
  u16  wManufactureData;
  u8   iManufacturerName;
  u8   iProductName;
  u8   iSerialNumber;
  u8   iOemID;
  u16  wManufacturerID;
  u8   bUD0BaseOffset;
  u8   bUDConfigPlength;
  u8   bDeviceRTTCap;
  u16  wPeriodicRTCUpdate;
  u8   reserved_1[48];
  u32  dExtendedUFSFeaturesSupport;
  u8   reserved_2[4];
};

struct ufs_unit_desc {
  u8   bLength;
  u8   bDescriptorType;
  u8   bUnitIndex;
  u8   bLUEnable;
  u8   bBootLunID;
  u8   bLUWriteProtect;
  u8   bLUQueueDepth;
  u8   Reserved;
  u8   bMemoryType;
  u8   bDataReliability;
  u8   bLogicalBlockSize;
  u32  qLogicalBlockCount_h;
  u32  qLogicalBlockCount_l;
  u32  dEraseBlockSize;
  u8   bProvisioningType;
  u32  qPhyMemResourceCount_h;
  u32  qPhyMemResourceCount_l;
  u16  wContextCapabilities;
  u8   bLargeUnitSize_M1;
};

struct ufs_geometry_desc {
  u8   bLength;
  u8   bDescriptorType;
  u8   bMediaTechnology;
  u8   Reserved_03;
  u32  qTotalRawDeviceCapacity_h;
  u32  qTotalRawDeviceCapacity_l;
  u8   Reserved_0c;
  u32  dSegmentSize;
  u8   bAllocationUnitSize;
  u8   bMinAddrBlockSize;
  u8   bOptimalReadBlockSize;
  u8   bOptimalWriteBlockSize;
  u8   bMaxInBufferSize;
  u8   bMaxOutBufferSize;
  u8   bRPMB_ReadWriteSize;
  u8   Reserved_18;
  u8   bDataOrdering;
  u8   bMaxContexIDNumber;
  u8   bSysDataTagUnitSize;
  u8   bSysDataTagResSize;
  u8   bSupportedSecRTypes;
  u16  wSupportedMemoryTypes;
  u32  dSystemCodeMaxNAllocU;
  u16  wSystemCodeCapAdjFac;
  u32  dNonPersistMaxNAllocU;
  u16  wNonPersistCapAdjFac;
  u32  dEnhanced1MaxNAllocU;
  u16  wEnhanced1CapAdjFac;
  u32  dEnhanced2MaxNAllocU;
  u16  wEnhanced2CapAdjFac;
  u32  dEnhanced3MaxNAllocU;
  u16  wEnhanced3CapAdjFac;
  u32  dEnhanced4MaxNAllocU;
  u16  wEnhanced4CapAdjFac;
  u32  Reserved_44;
  u8   reserved[7];
  u32  dTurboWriteBufferMaxNAllocUnits;
  u8   reserved_1[2];
  u8   bSupportedTurboWriteBufferUserSpaceReductionTypes;
  u8   bSupportedTurboWriteBufferTypes;
  u8   reserved_2;
};

struct ufs_flag_bit {
  u8 reserved_0;
  u8 fDeviceInit;
  u8 fPermanentWPEn;
  u8 fPowerOnWPEn;
  u8 fBackgroundOpsEn;
  u8 reserved_5;
  u8 fPurgeEnable;
  u8 reserved_7[25];
};

union ufs_flags {
  u8 arry[32];
  struct ufs_flag_bit flag;
};

union ufs_attributes {
  u32 arry[18];
};

#pragma pack()

// ─── UIC Command ─────────────────────────────────────────────────────────────
#pragma pack(1)
struct ufs_uic_cmd {
  u32 uiccmdr;
  u32 uiccmdarg1;
  u32 uiccmdarg2;
  u32 uiccmdarg3;
};
#pragma pack()

// ─── Simple SCSI command context (replaces LK scm) ──────────────────────────
typedef struct {
  u8   cdb[16];
  u8   *buf;
  u32  datalen;
  u8   status;
  u8   sense_buf[64];
  u32  lun;
} UFS_SCSI_CMD;

// ─── Main UFS Host structure ─────────────────────────────────────────────────
struct ufs_host {
  CHAR8                    host_name[16];
  UINTN                    irq;
  VOID                    *ioaddr;       /* HCI base          */
  VOID                    *vs_addr;      /* Vendor-specific   */
  VOID                    *fmp_addr;     /* FMP               */
  VOID                    *unipro_addr;  /* UniPro            */
  VOID                    *phy_pma;      /* PHY PMA           */
  VOID                    *dev_pwr_addr;
  VOID                    *phy_iso_addr;

  int                      host_index;

  UFS_SCSI_CMD            *scsi_cmd;
  u32                      lun;
  int                      scsi_status;

  struct ufs_cmd_desc     *cmd_desc_addr;
  struct ufs_utrd         *utrd_addr;
  struct ufs_utmrd        *utmrd_addr;
  struct ufs_uic_cmd      *uic_cmd;

  u32                      capabilities;
  int                      nutrs;
  int                      nutmrs;
  u32                      ufs_version;
  u32                      int_enable_mask;
  u32                      quirks;
  u32                      errors;

  u32                      ufs_cmd_timeout;
  u32                      uic_cmd_timeout;
  u32                      ufs_query_req_timeout;
  u32                      timeout;

  union ufs_flags          flags;
  union ufs_attributes     attributes;

  struct ufs_config_desc   config_desc;
  struct ufs_device_desc   device_desc;
  struct ufs_geometry_desc geometry_desc;
  struct ufs_unit_desc     unit_desc[8];
  u16                      data_seg_len;
  u8                       upiu_data[UPIU_DATA_SIZE * 4];

  struct ufs_cal_param    *cal_param;
  u32                      mclk_rate;
  struct uic_pwr_mode      pmd_cxt;
  u32                      dev_pwr_shift;
  u32                      support_tw;
  u32                      gear_mode;
  u16                      wManufactureID;
};

// ─── UFS set/get SFR helpers ─────────────────────────────────────────────────
#define UFS_GET_SFR(addr,mask,shift)    ((readl(addr) >> (shift)) & (mask))
#define UFS_SET_SFR(addr,value,mask,shift) \
  writel((readl(addr) & ~((mask) << (shift))) | ((value) << (shift)), (addr))

// ─── CAL interface ───────────────────────────────────────────────────────────
ufs_cal_errno ufs_cal_post_h8_enter  (struct ufs_cal_param *p);
ufs_cal_errno ufs_cal_pre_h8_exit    (struct ufs_cal_param *p);
ufs_cal_errno ufs_cal_post_pmc       (struct ufs_cal_param *p);
ufs_cal_errno ufs_cal_pre_pmc        (struct ufs_cal_param *p);
ufs_cal_errno ufs_cal_post_link      (struct ufs_cal_param *p);
ufs_cal_errno ufs_cal_pre_link       (struct ufs_cal_param *p);
ufs_cal_errno ufs_cal_init           (struct ufs_cal_param *p, int idx);

// CAL LLD adaptor callbacks
void           ufs_lld_dme_set        (void *h, u32 addr, u32 val);
void           ufs_lld_dme_get        (void *h, u32 addr, u32 *val);
void           ufs_lld_dme_peer_set   (void *h, u32 addr, u32 val);
void           ufs_lld_pma_write      (void *h, u32 val, u32 addr);
u32            ufs_lld_pma_read       (void *h, u32 addr);
void           ufs_lld_unipro_write   (void *h, u32 val, u32 addr);
void           ufs_lld_udelay         (u32 val);
void           ufs_lld_usleep_delay   (u32 min, u32 max);
unsigned long  ufs_lld_get_time_count (unsigned long offset);
unsigned long  ufs_lld_calc_timeout   (const unsigned int ms);

// ─── HCI interface ──────────────────────────────────────────────────────────
struct ufs_host  *UfsAllocHost        (VOID);
EFI_STATUS        UfsInitHost         (struct ufs_host *Ufs);
EFI_STATUS        UfsInitInterface    (struct ufs_host *Ufs);
EFI_STATUS        UfsRead             (struct ufs_host *Ufs, u32 Lun, u64 Lba, UINTN BlkCnt, u32 BlkSize, VOID *Buf);
EFI_STATUS        UfsWrite            (struct ufs_host *Ufs, u32 Lun, u64 Lba, UINTN BlkCnt, u32 BlkSize, VOID *Buf);
EFI_STATUS        UfsReadCapacity     (struct ufs_host *Ufs, u32 Lun, u64 *BlkCnt, u32 *BlkSize);
EFI_STATUS        UfsRequestSense     (struct ufs_host *Ufs, u32 Lun);

// ─── BlockIO private context ─────────────────────────────────────────────────
#define EXYNOS_UFS_SIGNATURE  SIGNATURE_32('E','U','F','S')

typedef struct {
  UINT32                   Signature;
  EFI_BLOCK_IO_PROTOCOL    BlockIo;
  EFI_BLOCK_IO_MEDIA       Media;
  struct ufs_host         *Ufs;
  u32                      Lun;
} EXYNOS_UFS_DEV;

#define EXYNOS_UFS_FROM_BLOCKIO(a) \
  CR(a, EXYNOS_UFS_DEV, BlockIo, EXYNOS_UFS_SIGNATURE)

#endif /* EXYNOS_UFS_H_ */

// ─── Additional internal functions exposed across translation units ───────────
extern u8   gQueryParams[][5];
int         UfsUtpCmdProcess   (struct ufs_host *Ufs, UFS_SCSI_CMD *Cmd);
int         UfsUtpQueryRetry   (struct ufs_host *Ufs, query_index qry, u32 lun);
