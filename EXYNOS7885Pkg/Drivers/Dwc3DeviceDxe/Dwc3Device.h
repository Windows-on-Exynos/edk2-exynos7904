/**
  DWC3 USB Device Mode Driver — Internal Definitions

  Register bit definitions, TRB structures, EP0 state machine, and
  device context. Based on:
    - lk3rd maestro9610: usbd3-ss.h, usbd3-ss.c, usb.h
    - Kernel: drivers/usb/dwc3/core.h, dwc3-reg.h, gadget.h, ep0.h
    - Synopsys DWC3 Databook v3.30a

  Copyright (c) 2024, Denzeel Oliva. All rights reserved.
  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#ifndef _DWC3_DEVICE_DXE_H_
#define _DWC3_DEVICE_DXE_H_

#include <Uefi.h>
#include <Library/Dwc3Lib.h>

//
// =========================================================================
// Register Offsets (DWC3 base relative)
// =========================================================================
//
#define DWC3_GCTL             0xC110
#define DWC3_GUCTL1           0xC11C
#define DWC3_GUCTL2           0xC19C
#define DWC3_GSNPSID          0xC120
#define DWC3_GFLADJ           0xC630
#define DWC3_GUSB2PHYCFG0     0xC200
#define DWC3_GUSB3PIPECTL0    0xC2C0

//
// Device Registers
//
#define DWC3_DCFG             0xC700
#define DWC3_DCTL             0xC704
#define DWC3_DEVTEN           0xC708
#define DWC3_DSTS             0xC70C
#define DWC3_DGCMDPAR          0xC710
#define DWC3_DGCMD            0xC714
#define DWC3_DALEPENA         0xC720

//
// Event Buffer Registers (event index 0)
//
#define DWC3_GEVNTADR_LO      0xC400
#define DWC3_GEVNTADR_HI      0xC404
#define DWC3_GEVNTSIZ         0xC408
#define DWC3_GEVNTCOUNT       0xC40C
#define DWC3_GHWPARAMS0       0xC140
#define DWC3_GHWPARAMS1       0xC144
#define DWC3_GHWPARAMS2       0xC148
#define DWC3_GHWPARAMS3       0xC14C
#define DWC3_GHWPARAMS4       0xC150
#define DWC3_GHWPARAMS5       0xC154
#define DWC3_GHWPARAMS6       0xC158
#define DWC3_GHWPARAMS7       0xC15C
#define DWC3_GHWPARAMS8       0xC600

//
// Endpoint Command/Parameter Registers
//   EP0 OUT: 0xC800-0xC80C
//   EP0 IN:  0xC810-0xC81C
//   EP(n) OUT: 0xC800 + (0x20 * n)
//   EP(n) IN:  0xC810 + (0x20 * n)
//
#define DWC3_DOEP_BASE(n)     (0xC800 + (0x20 * (n)))
#define DWC3_DIEP_BASE(n)     (0xC810 + (0x20 * (n)))
#define DWC3_DOEPCMDPAR2(n)   (DWC3_DOEP_BASE (n) + 0x00)
#define DWC3_DOEPCMDPAR1(n)   (DWC3_DOEP_BASE (n) + 0x04)
#define DWC3_DOEPCMDPAR0(n)   (DWC3_DOEP_BASE (n) + 0x08)
#define DWC3_DOEPCMD(n)       (DWC3_DOEP_BASE (n) + 0x0C)
#define DWC3_DIEPCMDPAR2(n)   (DWC3_DIEP_BASE (n) + 0x00)
#define DWC3_DIEPCMDPAR1(n)   (DWC3_DIEP_BASE (n) + 0x04)
#define DWC3_DIEPCMDPAR0(n)   (DWC3_DIEP_BASE (n) + 0x08)
#define DWC3_DIEPCMD(n)       (DWC3_DIEP_BASE (n) + 0x0C)

//
// =========================================================================
// GCTL (Global Control) bits
// =========================================================================
//
#define GCTL_PRTCAPDIR_MASK     (0x3 << 12)
#define GCTL_PRTCAPDIR_HOST     (0x1 << 12)
#define GCTL_PRTCAPDIR_DEV      (0x2 << 12)
#define GCTL_U2RSTECN           BIT16
#define GCTL_DISSCRAMBLE        BIT3
#define GCTL_MASTERFILTBYPASS   BIT5
#define GCTL_PWRDWNSCALE_SHIFT  19
#define GCTL_PWRDWNSCALE_MASK   (0x1FFF << 19)
#define GCTL_RAMCLKSEL_MASK     (0x3 << 6)
#define GCTL_RAMCLKSEL_BUS      (0x0 << 6)
#define GCTL_DISCLKGATING       BIT0
#define GCTL_CORESOFTRESET      BIT11
#define GCTL_SCALEDOWN_MASK     (0x3 << 4)

//
// GUCTL2 bits
//
#define GUCTL2_RST_ACTBITLATER  BIT14

//
// =========================================================================
// GUSB2PHYCFG0 (USB2 PHY Configuration) bits
// =========================================================================
//
#define GUSB2PHYCFG_PHYSOFTRST  BIT31
#define GUSB2PHYCFG_SUSPHY      BIT6
#define GUSB2PHYCFG_ENBLSLPM    BIT8
#define GUSB2PHYCFG_USBTRDTIM_SHIFT 10
#define GUSB2PHYCFG_USBTRDTIM_MASK  (0xF << 10)
#define GUSB2PHYCFG_PHYIF       BIT3
#define GUSB2PHYCFG_U2_FREECLK_EXISTS BIT14

//
// =========================================================================
// GUSB3PIPECTL0 (USB3 PIPE Control) bits
// =========================================================================
//
#define GUSB3PIPECTL_PHYSOFTRST BIT31
#define GUSB3PIPECTL_SUSPEND    BIT17
#define GUSB3PIPECTL_DELAY_P1P2P3_SHIFT 19

//
// =========================================================================
// DCFG (Device Configuration) bits
// =========================================================================
//
#define DCFG_DEV_SPEED_HS       0x0
#define DCFG_DEV_SPEED_FS       0x1
#define DCFG_DEV_SPEED_LS       0x2
#define DCFG_DEV_SPEED_SS       0x4
#define DCFG_DEV_SPEED_SHIFT    0
#define DCFG_DEV_SPEED_MASK     (0x7 << 0)
#define DCFG_DEV_ADDR_SHIFT     3
#define DCFG_DEV_ADDR_MASK      (0x7F << 3)
#define DCFG_NUMP_SHIFT         10
#define DCFG_NUMP_MASK          (0x3 << 10)
#define DCFG_INTNUM_SHIFT       12
#define DCFG_INTNUM_MASK        (0x1F << 12)
#define DCFG_NUMRXBUF_SHIFT     17
#define DCFG_NUMRXBUF_MASK      (0xF << 17)
#define DCFG_LPM_CAP            BIT22
#define DCFG_IGNSTRMPP          BIT23

//
// =========================================================================
// DCTL (Device Control) bits
// =========================================================================
//
#define DCTL_RUN_STOP           BIT31
#define DCTL_CSFTRST            BIT30
#define DCTL_INIT_U1U2_DIS      (BIT9 | BIT10 | BIT11 | BIT12 | BIT13)
#define DCTL_TSTCTL_SHIFT       1
#define DCTL_TSTCTL_MASK        (0x7 << 1)

//
// =========================================================================
// DSTS (Device Status) bits
// =========================================================================
//
#define DSTS_CONNECTSPD_SHIFT   0
#define DSTS_CONNECTSPD_MASK    (0x7 << 0)
#define DSTS_RX_FIFO_EMPTY      BIT17

//
// =========================================================================
// DEVTEN (Device Event Enable) bits
// =========================================================================
//
#define DEVTEN_DISCONN          BIT0
#define DEVTEN_USB_RESET        BIT1
#define DEVTEN_CONN_DONE        BIT2
#define DEVTEN_WAKEUP           BIT4
#define DEVTEN_U3L2L1_SUSP      BIT6
#define DEVTEN_LNK_STS_CHNG     BIT5
#define DEVTEN_OVERFLOW         BIT11

//
// =========================================================================
// DGCMD (Device Generic Command) bits
// =========================================================================
//
#define DGCMD_CMD_TYPE_SHIFT    0
#define DGCMD_CMD_TYPE_MASK     (0xFF << 0)
#define DGCMD_CMD_ACTIVE        BIT8
#define DGCMD_ALL_FIFO_FLUSH    0x0D

//
// =========================================================================
// GEVNTSIZ bits
// =========================================================================
//
#define GEVNTSIZ_EVENT_INT_MASK  BIT31
#define GEVNTSIZ_SIZE_MASK      0xFFFF

//
// =========================================================================
// GEVNTCOUNT bits
// =========================================================================
//
#define GEVNTCOUNT_MASK         0xFFFF

//
// =========================================================================
// Event types (4-byte event word)
// =========================================================================
//
#define DWC3_EVENT_TYPE_MASK    (0xFF << 0)
#define DWC3_EVENT_DEV          (0 << 0)
#define DWC3_EVENT_EP_OUT       (6 << 0)

//
// Device Event indices (event_info field)
//
#define DEVT_DISCONNECT_LEVEL   0
#define DEVT_USB_RESET          1
#define DEVT_CONN_DONE          2
#define DEVT_LINK_STS_CHG       3
#define DEVT_WAKEUP             4
#define DEVT_OVERFLOW           6
#define DEVT_ERRATIC_ERR        9

//
// Endpoint Event types (event_type within DEPEVT)
//
#define DEPEVT_XFER_CMPL        0
#define DEPEVT_XFER_IN_PROG     1
#define DEPEVT_XFER_NRDY        3
#define DEPEVT_FIFO_UNDERRUN    5
#define DEPEVT_STRM_EVT         6
#define DEPEVT_CMD_CMPL         7

//
// =========================================================================
// DEPCMD (Endpoint Command) bits
// =========================================================================
//
#define DEPCMD_CMD_TYPE_SHIFT   0
#define DEPCMD_CMD_TYPE_MASK    (0xF << 0)
#define DEPCMD_CMD_PARAM_SHIFT  16
#define DEPCMD_CMD_PARAM_MASK   (0x7FFF << 16)
#define DEPCMD_CMD_ACTIVE       BIT10
#define DEPCMD_HIPRI_FORCERM    BIT11

//
// Endpoint command types
//
#define DEPCMD_SET_EP_CFG       1
#define DEPCMD_SET_EP_XFER_RSRC 2
#define DEPCMD_START_NEW_CFG    9
#define DEPCMD_END_XFER         8
#define DEPCMD_SET_STALL        4
#define DEPCMD_CLR_STALL        5
#define DEPCMD_START_XFER       6

//
// =========================================================================
// DEPCMDPAR0 for SetEpCfg
// =========================================================================
//
#define DEPCFG_EP_TYPE_SHIFT    1
#define DEPCFG_EP_TYPE_CTRL     0
#define DEPCFG_EP_TYPE_BULK     2
#define DEPCFG_MPS_SHIFT        3
#define DEPCFG_FIFO_NUM_SHIFT   17
#define DEPCFG_BRST_SIZ_SHIFT   25
#define DEPCFG_CFG_ACTION_SHIFT 30
#define DEPCFG_CFG_ACTION_INIT  0
#define DEPCFG_CFG_ACTION_MODIFY 2
#define DEPCFG_IGN_DSTNUM       BIT29

//
// =========================================================================
// DEPCMDPAR1 for SetEpCfg
// =========================================================================
//
#define DEPCFG1_XFER_CMPL_EN    BIT8
#define DEPCFG1_XFER_IN_PROG_EN BIT9
#define DEPCFG1_XFER_NRDY_EN    BIT10
#define DEPCFG1_INTR_NUM_SHIFT  0
#define DEPCFG1_INTR_NUM_MASK   (0x1F << 0)
#define DEPCFG1_EP_DIR_SHIFT    25
#define DEPCFG1_EP_DIR_OUT      0
#define DEPCFG1_EP_DIR_IN       1
#define DEPCFG1_EP_NUM_SHIFT    26
#define DEPCFG1_EP_NUM_MASK     (0x1F << 26)

//
// =========================================================================
// EP direction enum
// =========================================================================
//
typedef enum {
  Dwc3EpDirOut = 0,
  Dwc3EpDirIn  = 1
} DWC3_EP_DIR;

//
// =========================================================================
// TRB Control types
// =========================================================================
//
#define TRB_CTRL_NORMAL         1
#define TRB_CTRL_SETUP          2
#define TRB_CTRL_CTLDATA_1ST    3
#define TRB_CTRL_CTLDATA_2ND    4
#define TRB_CTRL_STATUS_2       5
#define TRB_CTRL_STATUS_3       6

#define TRB_CTRL_LST            BIT4
#define TRB_CTRL_IOC            BIT5
#define TRB_CTRL_ISP_IMI        BIT10
#define TRB_CTRL_HWO            BIT0

//
// =========================================================================
// TRB Status
// =========================================================================
//
#define TRB_STS_OK              0

//
// =========================================================================
// Transfer Resource Index mask
// =========================================================================
//
#define TRI_MASK                0x7F

//
// =========================================================================
// TRB Structure (16 bytes, aligned to 16)
// =========================================================================
//
#pragma pack (1)
typedef struct {
  UINT32    BufPtrLo;
  UINT32    BufPtrHi;
  UINT32    Size;
  UINT32    Ctrl;
} DWC3_TRB;
#pragma pack ()

//
// =========================================================================
// EP0 States (from lk3rd maestro9610)
// =========================================================================
//
typedef enum {
  EP0_STATE_INIT              = 0,
  EP0_STATE_IN_DATA_PHASE     = 1,
  EP0_STATE_OUT_DATA_PHASE    = 2,
  EP0_STATE_IN_WAIT_NRDY      = 3,
  EP0_STATE_OUT_WAIT_NRDY     = 4,
  EP0_STATE_IN_STATUS_PHASE   = 5,
  EP0_STATE_OUT_STATUS_PHASE  = 6
} DWC3_EP0_STATE;

//
// =========================================================================
// USB Device States
// =========================================================================
//
typedef enum {
  USBDEV_STATE_DEFAULT    = 0,
  USBDEV_STATE_ADDRESSED  = 1,
  USBDEV_STATE_CONFIGURED = 2
} DWC3_USB_STATE;

//
// =========================================================================
// USB Device Request (8 bytes, standard USB format)
// =========================================================================
//
#pragma pack (1)
typedef struct {
  UINT8     Type;
  UINT8     Request;
  UINT8     ValueL;
  UINT8     ValueH;
  UINT8     IndexL;
  UINT8     IndexH;
  UINT8     LengthL;
  UINT8     LengthH;
} USB_DEV_REQ;
#pragma pack ()

//
// =========================================================================
// USB Standard Descriptors (packed)
// =========================================================================
//
#pragma pack (1)
typedef struct {
  UINT8     Length;
  UINT8     Type;
  UINT8     UsbL;
  UINT8     UsbH;
  UINT8     DevClass;
  UINT8     DevSubClass;
  UINT8     DevProtocol;
  UINT8     MaxPkt0;
  UINT8     VendorL;
  UINT8     VendorH;
  UINT8     ProductL;
  UINT8     ProductH;
  UINT8     DevRelL;
  UINT8     DevRelH;
  UINT8     VendorStr;
  UINT8     ProductStr;
  UINT8     SerialStr;
  UINT8     NumConfigs;
} USB_DEV_DESC;

typedef struct {
  UINT8     Length;
  UINT8     Type;
  UINT8     TotalLenL;
  UINT8     TotalLenH;
  UINT8     NumIfs;
  UINT8     CfgVal;
  UINT8     CfgStr;
  UINT8     Attr;
  UINT8     MaxPower;
} USB_CFG_DESC;

typedef struct {
  UINT8     Length;
  UINT8     Type;
  UINT8     IfNum;
  UINT8     AltSetting;
  UINT8     NumEps;
  UINT8     IfClass;
  UINT8     IfSubClass;
  UINT8     IfProtocol;
  UINT8     IfStr;
} USB_IF_DESC;

typedef struct {
  UINT8     Length;
  UINT8     Type;
  UINT8     EpAddr;
  UINT8     Attr;
  UINT8     MaxPktL;
  UINT8     MaxPktH;
  UINT8     Interval;
} USB_EP_DESC;

//
// Composite: Config + Interface + EP0 + EP1
//
typedef struct {
  USB_CFG_DESC    Config;
  USB_IF_DESC     Interface;
  USB_EP_DESC     EpBulkIn;
  USB_EP_DESC     EpBulkOut;
} USB_CFG_FULL_DESC;

#pragma pack ()

#define FULL_CFG_DESC_SIZE  (sizeof (USB_CFG_FULL_DESC))  // 9+9+7+7 = 32

//
// =========================================================================
// GET_STATUS structure
// =========================================================================
//
#pragma pack (1)
typedef struct {
  UINT8     Device;
  UINT8     Interface;
  UINT8     EpCtrl;
  UINT8     EpIn;
  UINT8     EpOut;
} USB_GET_STATUS;

typedef struct {
  UINT8     AltSetting;
} USB_GET_IF;
#pragma pack ()

//
// =========================================================================
// Driver Context
// =========================================================================
//
#define MAX_EPS              16
#define EVENT_BUF_DEPTH      64
#define EP0_BUF_SIZE         128

#define DWC3_DEV_FROM_PROTO(This)  BASE_CR (This, DWC3_DEV_CTX, UsbfnIo)

typedef struct _DWC3_DEV_CTX {
  //
  // Protocol
  //
  EFI_USBFN_IO_PROTOCOL       UsbfnIo;

  //
  // Platform config
  //
  DWC3_PLAT_CONFIG            PlatConfig;
  UINT64                      Dwc3Base;

  DWC3_DEV_CONFIG			  DevConfig;

  //
  // DWC3 state
  //
  BOOLEAN                     Initialized;
  BOOLEAN                     Started;
  DWC3_EP0_STATE              Ep0State;
  DWC3_USB_STATE              UsbState;
  BOOLEAN                     Ep0ThreeStage;

  //
  // Event buffer
  //
  UINT32                     *EventBuffer;
  UINT32                      EventCount;

  //
  // EP0 buffers and state
  //
  VOID                       *Ep0SetupBuf;     // for SETUP TRB/request
  DWC3_TRB                   *Ep0SetupTrb;
  VOID                       *Ep0InBuf;
  DWC3_TRB                   *Ep0InTrb;
  VOID                       *Ep0OutBuf;
  DWC3_TRB                   *Ep0OutTrb;

  USB_DEV_REQ                 DeviceRequest;
  UINT32                      DeviceRequestLen;
  BOOLEAN                     SetupPending;

  //
  // Transfer resource indexes
  //
  UINT32                      TriOut[MAX_EPS];
  UINT32                      TriIn[MAX_EPS];

  //
  // Endpoint state
  //
  BOOLEAN                     EpEnabled[MAX_EPS * 2];
  BOOLEAN                     EpStalled[MAX_EPS * 2];
  BOOLEAN                     EpsActive;
  UINT32                      ControlMps;

  //
  // Speed
  //
  UINT32                      BusSpeed;      // 0=unknown, 1=Low, 2=Full, 3=High, 4=Super
  UINT32                      BulkMps;

  //
  // USB descriptors (allocated on heap)
  //
  USB_DEV_DESC               *DevDesc;
  USB_CFG_FULL_DESC          *CfgDesc;

  //
  // GET_STATUS / GET_INTERFACE state
  //
  USB_GET_STATUS              GetStatus;
  USB_GET_IF                  GetInterface;
  UINT8                       RemoteWakeup;

  //
  // Bulk transfer state
  //
  BOOLEAN                     BulkOutActive;
  VOID                       *BulkOutBuf;
  UINTN                       BulkOutLen;
  BOOLEAN                     BulkInActive;
  VOID                       *BulkInBuf;
  UINTN                       BulkInLen;
} DWC3_DEV_CTX;

//
// =========================================================================
// Internal helper functions
// =========================================================================
//
VOID
Dwc3DevSetEpCfg (
  IN DWC3_DEV_CTX   *Dev,
  IN DWC3_EP_DIR     Dir,
  IN UINT8           EpNum,
  IN UINT32          EpType,
  IN UINT32          Mps,
  IN UINT32          FifoNum,
  IN UINT32          Burst,
  IN UINT32          CfgAction
  );

VOID
Dwc3DevSetEpXferRsrc (
  IN DWC3_DEV_CTX   *Dev,
  IN DWC3_EP_DIR     Dir,
  IN UINT8           EpNum,
  IN UINT32          XferRscIdx
  );

VOID
Dwc3DevStartXfer (
  IN  DWC3_DEV_CTX  *Dev,
  IN  DWC3_EP_DIR    Dir,
  IN  UINT8          EpNum,
  IN  DWC3_TRB      *Trb,
  IN  UINT32         StrmId,
  OUT UINT32        *Tri
  );

VOID
Dwc3DevActivateEp (
  IN DWC3_DEV_CTX   *Dev,
  IN DWC3_EP_DIR     Dir,
  IN UINT8           EpNum
  );

#endif /* _DWC3_DEVICE_DXE_H_ */
