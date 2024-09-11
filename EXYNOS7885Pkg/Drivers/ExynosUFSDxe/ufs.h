#ifndef __EXYNOS_UFS__
#define __EXYNOS_UFS__

#include <Uefi.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiHiiServicesLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/BaseLib.h>
#include <Library/UefiLib.h>
#include <Library/DevicePathLib.h>
#include <Library/DebugLib.h>
#include <Library/DxeServicesLib.h>
#include <Library/HiiLib.h>
#include <Library/PrintLib.h>
#include <Library/PcdLib.h>

#include <IndustryStandard/Scsi.h>
#include <Protocol/BlockIo.h>

#include <Drivers/exynos9610.h>

#include "scsi.h"

#define RET_SUCCESS		0	/* 0 = Success */
#define RET_FAILURE		1	/* 1 = Failure */

#define SWP_LOCK		0
#define SWP_UNLOCK		1

#define SAMSUNG		0xce
#define TOSHIBA		0x98
#define HYNIX			0xad
#define MICRON		0x2c

#define UFS_BIT_LEN_OF_DWORD	(sizeof(UINT32) * 8)

#define UFS_QUIRK_USE_1LANE		(1<<1)
#define UFS_QUIRK_BROKEN_HCE	(1<<2)

#define ___swab16(x) \
	((((x) & 0xff00) >> 8) | \
	 (((x) & 0x00ff) << 8))

#define ___swab32(x) \
	((UINT32)( \
		(((UINT32)(x) & (UINT32)0x000000ffUL) << 24) | \
		(((UINT32)(x) & (UINT32)0x0000ff00UL) <<  8) | \
		(((UINT32)(x) & (UINT32)0x00ff0000UL) >>  8) | \
		(((UINT32)(x) & (UINT32)0xff000000UL) >> 24)))

#define MAX_CDB_SIZE		16
#define ALIGNED_UPIU_SIZE	1024
#define SCSI_MAX_SG_SEGMENTS	128

#define UFS_SG_BLOCK_SIZE_BIT	12
#define UFS_SG_BLOCK_SIZE	(1<<UFS_SG_BLOCK_SIZE_BIT)

#define UFS_NUTRS		2

#define UFS_DEBUG_UPIU		0x00323110
#define UFS_DEBUG_UPIU_ALL	0x00743112
#define UFS_DEBUG_UPIU_ALL2	0x00743110
#define UFS_DEBUG_UPIU_MIN	0x00111110

#define UPIU_HEADER_DWORD(byte3, byte2, byte1, byte0)\
			((byte3 << 24) | (byte2 << 16) |\
			 (byte1 << 8) | (byte0))

#define UFS_GET_SFR(addr, mask, shift)	((readl(addr)>>(shift))&(mask))
#define UFS_SET_SFR(addr, value, mask, shift)	writel((readl(addr)&(~((mask)<<(shift))))|((value)<<(shift)), (addr))

enum {
	REG_CONTROLLER_CAPABILITIES = 0x00,
	REG_UFS_VERSION = 0x08,
	REG_CONTROLLER_PID = 0x10,
	REG_CONTROLLER_MID = 0x14,
	REG_INTERRUPT_STATUS = 0x20,
	REG_INTERRUPT_ENABLE = 0x24,
	REG_CONTROLLER_STATUS = 0x30,
#define UPMCRS(x)		(((x) >> 8) & 7)
#define PWR_LOCAL		1
	REG_CONTROLLER_ENABLE = 0x34,
	REG_UIC_ERROR_CODE_PHY_ADAPTER_LAYER = 0x38,
	REG_UIC_ERROR_CODE_DATA_LINK_LAYER = 0x3C,
	REG_UIC_ERROR_CODE_NETWORK_LAYER = 0x40,
	REG_UIC_ERROR_CODE_TRANSPORT_LAYER = 0x44,
	REG_UIC_ERROR_CODE_DME = 0x48,
	REG_UTP_TRANSFER_REQ_INT_AGG_CONTROL = 0x4C,
	REG_UTP_TRANSFER_REQ_LIST_BASE_L = 0x50,
	REG_UTP_TRANSFER_REQ_LIST_BASE_H = 0x54,
	REG_UTP_TRANSFER_REQ_DOOR_BELL = 0x58,
	REG_UTP_TRANSFER_REQ_LIST_CLEAR = 0x5C,
	REG_UTP_TRANSFER_REQ_LIST_RUN_STOP = 0x60,
	REG_UTP_TRANSFER_REQ_LIST_CNR	= 0x64,
	REG_UTP_TASK_REQ_LIST_BASE_L = 0x70,
	REG_UTP_TASK_REQ_LIST_BASE_H = 0x74,
	REG_UTP_TASK_REQ_DOOR_BELL = 0x78,
	REG_UTP_TASK_REQ_LIST_CLEAR = 0x7C,
	REG_UTP_TASK_REQ_LIST_RUN_STOP = 0x80,
	REG_UIC_COMMAND = 0x90,
	REG_UIC_COMMAND_ARG_1 = 0x94,
	REG_UIC_COMMAND_ARG_2 = 0x98,
	REG_UIC_COMMAND_ARG_3 = 0x9C,

	REG_CRYPTO_CAPABILITY			= 0x100,

	VS_TXPRDT_ENTRY_SIZE		= 0x000,
	VS_RXPRDT_ENTRY_SIZE		= 0x004,
	VS_IS				= 0x038,
	VS_UTRL_NEXUS_TYPE		= 0x040,
	VS_UMTRL_NEXUS_TYPE		= 0x044,
	VS_SW_RST			= 0x050,
	VS_DATA_REORDER			= 0x060,
	VS_AXIDMA_RWDATA_BURST_LEN	= 0x06C,
	VS_GPIO_OUT			= 0x070,
	VS_CLKSTOP_CTRL			= 0x0B0,
	VS_FORCE_HCS			= 0x0B4,
	VS_UFS_ACG_DISABLE		= 0x0FC,
	VS_MPHY_REFCLK_SEL		= 0x108,

	UFSP_UPRSECURITY	= 0x10,
	UFSP_UPSBEGIN0		= 0x2000,
	UFSP_UPSEND0		= 0x2004,
	UFSP_UPLUN0		= 0x2008,
	UFSP_UPSCTRL0		= 0x200c,
};

/* Interrupt status */
#define UFS_BIT(x)	(1L << (x))

#define UTP_TRANSFER_REQ_COMPL			UFS_BIT(0)
#define UIC_ERROR				UFS_BIT(2)
#define UIC_POWER_MODE				UFS_BIT(4)
#define UIC_LINK_LOST				UFS_BIT(7)
#define UIC_LINK_STARTUP			UFS_BIT(8)
#define UTP_TASK_REQ_COMPL			UFS_BIT(9)
#define UIC_COMMAND_COMPL			UFS_BIT(10)
#define DEVICE_FATAL_ERROR			UFS_BIT(11)
#define CONTROLLER_FATAL_ERROR			UFS_BIT(16)
#define SYSTEM_BUS_FATAL_ERROR			UFS_BIT(17)

#define INT_FATAL_ERRORS	(DEVICE_FATAL_ERROR |\
				CONTROLLER_FATAL_ERROR |\
				SYSTEM_BUS_FATAL_ERROR |\
				UIC_LINK_LOST)

/* UIC Commands */
enum {
	UIC_CMD_DME_GET = 0x01,
	UIC_CMD_DME_SET = 0x02,
	UIC_CMD_DME_PEER_GET = 0x03,
	UIC_CMD_DME_PEER_SET = 0x04,
	UIC_CMD_DME_POWERON = 0x10,
	UIC_CMD_DME_POWEROFF = 0x11,
	UIC_CMD_DME_ENABLE = 0x12,
	UIC_CMD_DME_RESET = 0x14,
	UIC_CMD_DME_END_PT_RST = 0x15,
	UIC_CMD_DME_LINK_STARTUP = 0x16,
	UIC_CMD_DME_HIBER_ENTER = 0x17,
	UIC_CMD_DME_HIBER_EXIT = 0x18,
	UIC_CMD_DME_TEST_MODE = 0x1A,
	UIC_CMD_WAIT = 0x80,
	UIC_CMD_WAIT_ISR = 0x90,
	PHY_PMA_COMN_SET = 0xf0,
	PHY_PMA_TRSV_SET = 0xf1,
	PHY_PMA_COMN_WAIT = 0xf2,
	PHY_PMA_TRSV_WAIT = 0xf3,
	UIC_CMD_REGISTER_SET = 0xff,
};

enum {
	TX_LANE_0 = 0,
	TX_LANE_1 = 1,
	TX_LANE_2 = 2,
	TX_LANE_3 = 3,
	RX_LANE_0 = 4,
	RX_LANE_1 = 5,
	RX_LANE_2 = 6,
	RX_LANE_3 = 7,
};

#define UIC_ARG_MIB_SEL(attr, sel)	((((attr) & 0xFFFF) << 16) |\
					 ((sel) & 0xFFFF))
#define UIC_ARG_MIB(attr)		UIC_ARG_MIB_SEL(attr, 0)


/* Well known logical unit id in LUN field of UPIU */
enum {
	UFS_UPIU_REPORT_LUNS_WLUN	= 0x81,
	UFS_UPIU_UFS_DEVICE_WLUN	= 0xD0,
	UFS_UPIU_BOOT_WLUN		= 0xB0,
	UFS_UPIU_RPMB_WLUN		= 0xC4,
};

enum {
	UIC_CMD_RESULT_SUCCESS = 0x00,
	UIC_CMD_RESULT_INVALID_ATTR = 0x01,
	UIC_CMD_RESULT_FAILURE = 0x01,
	UIC_CMD_RESULT_INVALID_ATTR_VALUE = 0x02,
	UIC_CMD_RESULT_READ_ONLY_ATTR = 0x03,
	UIC_CMD_RESULT_WRITE_ONLY_ATTR = 0x04,
	UIC_CMD_RESULT_BAD_INDEX = 0x05,
	UIC_CMD_RESULT_LOCKED_ATTR = 0x06,
	UIC_CMD_RESULT_BAD_TEST_FEATURE_INDEX = 0x07,
	UIC_CMD_RESULT_PEER_COMM_FAILURE = 0x08,
	UIC_CMD_RESULT_BUSY = 0x09,
	UIC_CMD_RESULT_DME_FAILURE = 0x0A,
};

enum {
	UTP_SCSI_COMMAND = 0x00000000,
	UTP_NATIVE_UFS_COMMAND = 0x10000000,
	UTP_DEVICE_MANAGEMENT_FUNCTION = 0x20000000,
	UTP_REQ_DESC_INT_CMD = 0x01000000,
};

/* UTP Transfer Request Data Direction (DD) */
enum {
	UTP_NO_DATA_TRANSFER = 0x00000000,
	UTP_HOST_TO_DEVICE = 0x02000000,
	UTP_DEVICE_TO_HOST = 0x04000000,
};

/* Overall command status values */
enum {
	OCS_SUCCESS = 0x0,
	OCS_INVALID_CMD_TABLE_ATTR = 0x1,
	OCS_INVALID_PRDT_ATTR = 0x2,
	OCS_MISMATCH_DATA_BUF_SIZE = 0x3,
	OCS_MISMATCH_RESP_UPIU_SIZE = 0x4,
	OCS_PEER_COMM_FAILURE = 0x5,
	OCS_ABORTED = 0x6,
	OCS_FATAL_ERROR = 0x7,
	OCS_INVALID_COMMAND_STATUS = 0x0F,
	MASK_OCS = 0x0F,
};

/* Task management functions */
enum {
	UFS_ABORT_TASK = 0x01,
	UFS_ABORT_TASK_SET = 0x02,
	UFS_CLEAR_TASK_SET = 0x04,
	UFS_LOGICAL_RESET = 0x08,
	UFS_QUERY_TASK = 0x80,
	UFS_QUERY_TASK_SET = 0x81,
};

/* Query request functions */
enum {
	UFS_STD_READ_REQ = 0x01,
	UFS_STD_WRITE_REQ = 0x81,
	UFS_VENDOR_READ_REQ = 0x40,
	UFS_VENDOR_WRITE_REQ = 0xc0,
	UFS_VENDOR_FORMAT_REQ = 0xc2,
};

#define UFS_QUERY_FUNCTION_MASK	0xc0

/* UTP UPIU Transaction Codes Initiator to Target */
enum ufs_transaction_code {
	UPIU_TRANSACTION_NOP_OUT = 0x00,
	UPIU_TRANSACTION_COMMAND = 0x01,
	UPIU_TRANSACTION_DATA_OUT = 0x02,
	UPIU_TRANSACTION_TASK_REQ = 0x04,
	UPIU_TRANSACTION_QUERY_REQ = 0x16,
};

/* UTP UPIU Transaction Codes Target to Initiator */
enum {
	UPIU_TRANSACTION_NOP_IN = 0x20,
	UPIU_TRANSACTION_RESPONSE = 0x21,
	UPIU_TRANSACTION_DATA_IN = 0x22,
	UPIU_TRANSACTION_TASK_RSP = 0x24,
	UPIU_TRANSACTION_READY_XFER = 0x31,
	UPIU_TRANSACTION_QUERY_RSP = 0x36,
	UPIU_TRANSACTION_REJECT_UPIU = 0x3f,
};

/* UPIU Read/Write flags */
enum {
	UPIU_CMD_FLAGS_NONE = 0x00,
	UPIU_CMD_FLAGS_WRITE = 0x20,
	UPIU_CMD_FLAGS_READ = 0x40,
};

/* UPIU Task Attributes */
enum {
	UPIU_TASK_ATTR_SIMPLE = 0x00,
	UPIU_TASK_ATTR_ORDERED = 0x01,
	UPIU_TASK_ATTR_HEADQ = 0x02,
	UPIU_TASK_ATTR_ACA = 0x03,
};

/* Descriptor ID */
enum {
	UPIU_DESC_ID_DEVICE = 0x0,
	UPIU_DESC_ID_CONFIGURATION = 0x1,
	UPIU_DESC_ID_UNIT = 0x2,
	UPIU_DESC_ID_INTERCONNECT = 0x4,
	UPIU_DESC_ID_STRING = 0x5,
	UPIU_DESC_ID_GEOMETRY = 0x7,
	UPIU_DESC_ID_POWER = 0x8,
};

/* Flag ID */
enum {
	UPIU_FLAG_ID_DEVICEINIT = 0x1,
	UPIU_FLAG_ID_PER_WRITEPROTECT = 0x2,
	UPIU_FLAG_ID_POW_WRITEPROTECT = 0x3,
	UPIU_FLAG_ID_BG_OPERATRION = 0x4,
	UPIU_FLAG_ID_PURGE_ENABLE = 0x6,
	UPIU_FLAG_ID_ALL = 0xff,
};

/* Attribute ID */
enum {
	UPIU_ATTR_ID_BOOTLUNEN = 0x0,
	UPIU_ATTR_ID_POWERMODE = 0x2,
	UPIU_ATTR_ID_ACTIVECCLEVEL = 0x3,
	UPIU_ATTR_ID_OOODATAEN = 0x4,
	UPIU_ATTR_ID_BACKGROUNDOP = 0x5,
	UPIU_ATTR_ID_PURGESTATUS = 0x6,
	UPIU_ATTR_ID_MAXDATAIN = 0x7,
	UPIU_ATTR_ID_MAXDATAOUT = 0x8,
	UPIU_ATTR_ID_DYNCAPNEEDED = 0x9,
	UPIU_ATTR_ID_REFCLKFREQ = 0xa,
	UPIU_ATTR_ID_CONFIGDESCLOCK = 0xb,
	UPIU_ATTR_ID_MAXNUMRTT = 0xc,
	UPIU_ATTR_ID_EXCEPTIONEVENTCTL = 0xd,
	UPIU_ATTR_ID_EXCEPTIONEVENTSTATUS = 0xe,
	UPIU_ATTR_ID_SECONDPASSED = 0xf,
	UPIU_ATTR_ID_CONTEXTCONF = 0x10,
	UPIU_ATTR_ID_CORRPRGBLKNUM = 0x11,
	UPIU_ATTR_ID_ALL = 0xff,
};

/* UTP QUERY Transaction Specific Fields OpCode */
enum {
	UPIU_QUERY_OPCODE_NOP = 0x0,
	UPIU_QUERY_OPCODE_READ_DESC = 0x1,
	UPIU_QUERY_OPCODE_WRITE_DESC = 0x2,
	UPIU_QUERY_OPCODE_READ_ATTR = 0x3,
	UPIU_QUERY_OPCODE_WRITE_ATTR = 0x4,
	UPIU_QUERY_OPCODE_READ_FLAG = 0x5,
	UPIU_QUERY_OPCODE_SET_FLAG = 0x6,
	UPIU_QUERY_OPCODE_CLEAR_FLAG = 0x7,
	UPIU_QUERY_OPCODE_TOGGLE_FLAG = 0x8,
};

enum {
	MASK_SCSI_STATUS = 0xFF,
	MASK_TASK_RESPONSE = 0xFF00,
	MASK_RSP_UPIU_RESULT = 0xFFFF,
};

/* Task management service response */
enum {
	UPIU_TASK_MANAGEMENT_FUNC_COMPL = 0x00,
	UPIU_TASK_MANAGEMENT_FUNC_NOT_SUPPORTED = 0x04,
	UPIU_TASK_MANAGEMENT_FUNC_SUCCEEDED = 0x08,
	UPIU_TASK_MANAGEMENT_FUNC_FAILED = 0x05,
	UPIU_INCORRECT_LOGICAL_UNIT_NO = 0x09,
};

enum {
	UFS_TRANSFER_COMMAND = (1 << 0),
	UFS_TRANSFER_UIC = (1 << 1),
	UFS_TRANSFER_TASK = (1 << 2),
	UFS_TRANSFER_COMM_DONE = (1 << 3),
	UFS_TRANSFER_TASK_DONE = (1 << 4),
	UFS_TRANSFER_ERROR = (1 << 16),
	UFS_TRANSFER_FATAL_ERROR = (1 << 20),
	UFS_TRANSFER_ERROR_RETRY = (1 << 21),
	UFS_TRANSFER_TIMEOUT = (1 << 24),
	UFS_TRANSFER_COMPLETE = (1 << 31),
};

enum {
	UFS_NO_ERROR = 0,
	UFS_TIMEOUT,
	UFS_ERROR,
	UFS_IN_PROGRESS,
};

struct ufs_upiu_header {
	/* DW0 */
	UINT8 type;		/* [7]HD, [6]DD, [5:0]Transaction Type : UFS1.1 HD/DD should '0' */
	UINT8 flags;		/* Task Attribute : simple / ordered / head of queue */
	UINT8 lun;
	UINT8 tag;

	/* DW1 */
	UINT8 cmdtype;		/* Command Set Type */
	UINT8 function;		/* Query Function / Task Manag. Function */
	UINT8 response;
	UINT8 status;

	/* DW2 */
	UINT8 ehslength;		/* Total EHS length */
	UINT8 deviceinfo;		/* Device Information */
	UINT16 datalength;		/* Data Seqment Length (MSB|LSB) */
} __attribute__ ((__packed__));

#define DW_NUM_OF_TSF		20

#define UPIU_DATA_SIZE		(ALIGNED_UPIU_SIZE - \
		sizeof(UINT8) * DW_NUM_OF_TSF - sizeof(struct ufs_upiu_header))

struct ufs_upiu {
	struct ufs_upiu_header header;
	/* DW3 ~ DW7 */
	UINT8 tsf[DW_NUM_OF_TSF];		/* Transaction Specific Fields */
	UINT8 data[UPIU_DATA_SIZE];
} __attribute__ ((__packed__));

struct ufs_config_desc_header {
	UINT8 bLength;
	UINT8 bDescriptorType;
	UINT8 bConfDescContinue;
	UINT8 bBootEnable;
	UINT8 bDescrAccessEn;
	UINT8 bInitPowerMode;
	UINT8 bHighPriorityLUN;
	UINT8 bSecureRemovalType;
	UINT8 bInitActiveICCLevel;
	UINT16 wPeriodicRTCUpdate;
	UINT8 reserved[5];
	UINT8 bTurboWriteBufferNoUserSpaceReductionEn;
	UINT8 bTurboWriteBufferType;
} __attribute__ ((__packed__));

struct ufs_unit_desc_param {
	UINT8 bLUEnable;
	UINT8 bBootLunID;
	UINT8 bLUWriteProtect;
	UINT8 bMemoryType;
	UINT32 dNumAllocUnits;
	UINT8 bDataReliability;
	UINT8 bLogicalBlockSize;
	UINT8 bProvisioningType;
	UINT16 wContextCapabilities;
	UINT8 reserved[3];
	UINT8 reserved_1[6];
	UINT32 dLUNumTurboWriteBufferAllocUnits;
} __attribute__ ((__packed__));

struct ufs_config_desc {
	struct ufs_config_desc_header header;
	struct ufs_unit_desc_param unit[128];
} __attribute__ ((__packed__));

struct ufs_device_desc {
	UINT8 bLength;		/* offset : 0x00 */
	UINT8 bDescriptorType;
	UINT8 bDevice;
	UINT8 bDeviceClass;
	UINT8 bDeviceSubClass;
	UINT8 bProtocol;
	UINT8 bNumberLU;
	UINT8 iNumberWLU;
	UINT8 bBootEnable;		/* offset : 0x08 */
	UINT8 bDescrAccessEn;
	UINT8 bInitPowerMode;
	UINT8 bHighPriorityLUN;
	UINT8 bSecureRemovalType;
	UINT8 bSecurityLU;
	UINT8 reserved;
	UINT8 bInitActiveICCLevel;
	UINT16 wSpecVersion;	/* offset : 0x10 */
	UINT16 wManufactureData;
	UINT8 iManufacturerName;
	UINT8 iProductName;
	UINT8 iSerialNumber;
	UINT8 iOemID;
	UINT16 wManufacturerID;	/* offset : 0x18 */
	UINT8 bUD0BaseOffset;
	UINT8 bUDConfigPlength;
	UINT8 bDeviceRTTCap;
	UINT16 wPeriodicRTCUpdate;	/* offset : 0x1D */
	UINT8 reserved_1[48];	/* 0x1F ~ 0x4E */
	UINT32 dExtendedUFSFeaturesSupport;	/* offset : 0x4F */
	UINT8 reserved_2[4];			/* 0x53 ~ 0x56 */
} __attribute__ ((__packed__));

struct ufs_unit_desc {
	UINT8 bLength;		/* offset : 0x00 */
	UINT8 bDescriptorType;
	UINT8 bUnitIndex;
	UINT8 bLUEnable;
	UINT8 bBootLunID;

	UINT8 bLUWriteProtect;
	UINT8 bLUQueueDepth;
	UINT8 Reserved;
	UINT8 bMemoryType;		/* offset : 0x08 */
	UINT8 bDataReliability;

	UINT8 bLogicalBlockSize;
	UINT32 qLogicalBlockCount_h;
	UINT32 qLogicalBlockCount_l;
	UINT32 dEraseBlockSize;	/* offset : 0x13 */
	UINT8 bProvisioningType;
	UINT32 qPhyMemResourceCount_h;	/* offset : 0x18 */
	UINT32 qPhyMemResourceCount_l;
	UINT16 wContextCapabilities;	/* offset : 0x20 */
	UINT8 bLargeUnitSize_M1;
} __attribute__ ((__packed__));

struct ufs_geometry_desc {
	UINT8 bLength;		/* offset : 0x00 */
	UINT8 bDescriptorType;
	UINT8 bMediaTechnology;
	UINT8 Reserved_03;
	UINT32 qTotalRawDeviceCapacity_h;
	UINT32 qTotalRawDeviceCapacity_l;
	UINT8 Reserved_0c;
	UINT32 dSegmentSize;	/* offset : 0x0D */
	UINT8 bAllocationUnitSize;
	UINT8 bMinAddrBlockSize;
	UINT8 bOptimalReadBlockSize;
	UINT8 bOptimalWriteBlockSize;
	UINT8 bMaxInBufferSize;
	UINT8 bMaxOutBufferSize;
	UINT8 bRPMB_ReadWriteSize;
	UINT8 Reserved_18;		/* offset : 0x18 */
	UINT8 bDataOrdering;
	UINT8 bMaxContexIDNumber;
	UINT8 bSysDataTagUnitSize;
	UINT8 bSysDataTagResSize;
	UINT8 bSupportedSecRTypes;
	UINT16 wSupportedMemoryTypes;
	UINT32 dSystemCodeMaxNAllocU;	/* offset : 0x20 */
	UINT16 wSystemCodeCapAdjFac;
	UINT32 dNonPersistMaxNAllocU;	/* offset : 0x26 */
	UINT16 wNonPersistCapAdjFac;
	UINT32 dEnhanced1MaxNAllocU;
	UINT16 wEnhanced1CapAdjFac;	/* offset : 0x30 */
	UINT32 dEnhanced2MaxNAllocU;
	UINT16 wEnhanced2CapAdjFac;
	UINT32 dEnhanced3MaxNAllocU;	/* offset : 0x38 */
	UINT16 wEnhanced3CapAdjFac;
	UINT32 dEnhanced4MaxNAllocU;
	UINT16 wEnhanced4CapAdjFac;	/* offset : 0x42 */
	UINT32	Reserved_44;			/* 0x44 ~ 0x47 */
	UINT8 reserved[7];			/* 0x48 ~ 0x4E */
	UINT32 dTurboWriteBufferMaxNAllocUnits;	/* offset : 0x4f */
	UINT8 reserved_1[2];			/* 0x53 ~ 0x54 */
	UINT8 bSupportedTurboWriteBufferUserSpaceReductionTypes;	/* offset : 0x55 */
	UINT8 bSupportedTurboWriteBufferTypes;	/* offset : 0x56 */
	UINT8 reserved_2;		/* 0x57 */
} __attribute__ ((__packed__));

struct ufs_flag_bit {
	UINT8 reserved_0;		// bit[0] : Reserved
	UINT8 fDeviceInit;		// bit[1] : Device Initialization
	UINT8 fPermanentWPEn;	// bit[2] : Permanent Write Protect Enable
	UINT8 fPowerOnWPEn;	// bit[3] : Power On Write Protection Enable
	UINT8 fBackgroundOpsEn;	// bit[4] : Background Operation Enable
	UINT8 reserved_5;		// bit[5] : Reserved
	UINT8 fPurgeEnable;	// bit[6] : Purge Enable
	UINT8 reserved_7;		// bit[7] : Reserved
	UINT8 fPhyResourceRemoval;	// bit[8] : Physical Resource Removal
	UINT8 fBusyRTC;		// bit[9] : Busy Real Time Clock
	UINT8 reserved_10_31[22];	// bit[31:10] : Reserved
} __attribute__ ((__packed__));

union ufs_flags {
	UINT8 arry[32];
	struct ufs_flag_bit flag;
};

struct __ufs_attributes {
	UINT32 bBootLunEn;		// id : 0
	UINT32 reserved;		// id : 1
	UINT32 bCurrentPowerMode;	// id : 2
	UINT32 bActiveICCLevel;	// id : 3
	UINT32 bOutOfOrderDataEn;	// id : 4
	UINT32 bBackgroundOpStatus;	// id : 5
	UINT32 bPurgeStatus;	// id : 6
	UINT32 bMaxDataInSize;	// id : 7
	UINT32 bMaxDataOutSize;	// id : 8
	UINT32 dDynCapNeeded;	// id : 9
	UINT32 bRefClkFreq;	// id : 10
	UINT32 bConfigDescrLock;	// id : 11
	UINT32 bMaxNumOfRTT;	// id : 12
	UINT32 wExceptionEventControl;	// id : 13
	UINT32 wExceptionEventStatus;	// id : 14
	UINT32 dSecondsPassed;	// id : 15
	UINT32 wContextConf;	// id : 16
	UINT32 dCorrPrgBlkNum;	// id : 17
};

/*	Attributes		*/
union ufs_attributes {
	UINT32 arry[18];
	struct __ufs_attributes attr;
};

#ifndef __iomem
#define __iomem
#endif

#define UFS_RATE		2
#define UFS_POWER_MODE	1
#define UFS_RXTX_POWER_MODE	((UFS_POWER_MODE << 4)|UFS_POWER_MODE)

struct exynos_ufs_sfr_log {
	const char *name;
	const UINT32 offset;
#define LOG_STD_HCI_SFR		0xFFFFFFF0
#define LOG_VS_HCI_SFR		0xFFFFFFF1
#define LOG_FMP_SFR		0xFFFFFFF2
#define LOG_UNIPRO_SFR		0xFFFFFFF3
#define LOG_PMA_SFR		0xFFFFFFF4
	UINT32 val;
};

struct exynos_ufs_attr_log {
	const UINT32 offset;
	UINT32 res;
	UINT32 val;
};

struct exynos_ufs_debug {
	struct exynos_ufs_sfr_log *sfr;
	struct exynos_ufs_attr_log *attr;
};

/* Main structure for UFS core */
struct ufs_host {
	char host_name[16];
	unsigned int irq;
	void __iomem *ioaddr;
	void __iomem *vs_addr;
	void __iomem *fmp_addr;
	void __iomem *unipro_addr;
	void __iomem *ufspaddr;
	void __iomem *phy_pma;
	void __iomem *dev_pwr_addr;
	void __iomem *phy_iso_addr;

	int host_index;

	scm *scsi_cmd;
	UINT8 *ufs_descriptor;
	UINT8 *arglist;
	UINT32 lun;
	int scsi_status;
	UINT8 *sense_buffer;
	UINT32 sense_buflen;

	struct ufs_cmd_desc *cmd_desc_addr;
	struct ufs_utrd *utrd_addr;
	struct ufs_utmrd *utmrd_addr;
	struct ufs_uic_cmd *uic_cmd;

	UINT32 capabilities;
	int nutrs;
	int nutmrs;
	UINT32 ufs_version;

	UINT32 int_enable_mask;

	UINT32 quirks;		/* Quirk flags */

	/* HBA Errors */
	UINT32 errors;

	/* timeout */
	UINT32 ufs_cmd_timeout;
	UINT32 uic_cmd_timeout;
	UINT32 ufs_query_req_timeout;
	UINT32 timeout;

	union ufs_flags flags;
	union ufs_attributes attributes;

	struct ufs_config_desc config_desc;
	struct ufs_device_desc device_desc;
	struct ufs_geometry_desc geometry_desc;
	struct ufs_unit_desc unit_desc[8];
	UINT16 data_seg_len;
	// TODO:
	UINT8 upiu_data[UPIU_DATA_SIZE * 4];
	struct ufs_cal_param	*cal_param;
	UINT32 mclk_rate;
	struct uic_pwr_mode pmd_cxt;
	UINT32 dev_pwr_shift;
	UINT32 support_tw;
	UINT32 gear_mode;
	UINT16 wManufactureID;
	int swp;

	struct exynos_ufs_debug  debug;
};

int ufs_alloc_memory(void);
int ufs_init(int mode);
int ufs_set_configuration_descriptor(void);
int ufs_board_init(int host_index, struct ufs_host *ufs);
void ufs_pre_vendor_setup(struct ufs_host *ufs);
int ufs_device_reset(void);
int scsi_swp_check(int lun);
void do_swp_lock(void);
void do_swp_unlock(void);

void print_ufs_upiu(struct ufs_host *ufs, int print_level);
void print_ufs_desc(UINT8 * desc);
void print_ufs_device_desc(UINT8 * desc);
void print_ufs_configuration_desc(UINT8 * desc);
void print_ufs_geometry_desc(UINT8 * desc);
void print_ufs_flags(union ufs_flags *flags);

void exynos_ufs_get_sfr(struct ufs_host *ufs,
		struct exynos_ufs_sfr_log *cfg);
void exynos_ufs_get_attr(struct ufs_host *ufs,
		struct exynos_ufs_attr_log *cfg);
void exynos_ufs_get_uic_info(struct ufs_host *ufs);

int handle_ufs_int(struct ufs_host *ufs, int is_uic);

typedef enum {
	UFS_CAL_NO_ERROR = 0,
	UFS_CAL_TIMEOUT,
	UFS_CAL_ERROR,
	UFS_CAL_INV_ARG,
} ufs_cal_errno;

#endif				/* __UFS__ */