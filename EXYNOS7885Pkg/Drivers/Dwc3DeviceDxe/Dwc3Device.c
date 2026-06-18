#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/Dwc3Lib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationHelperLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiDriverEntryPoint.h>
#include <Library/UsbPhyLib.h>
#include <Protocol/UsbFunctionIo.h>

#include "Dwc3Device.h"

//
// MMIO helpers
//
#define DWC3_RD32(A)  MmioRead32  ((UINTN)(A))
#define DWC3_WR32(A,V) MmioWrite32 ((UINTN)(A), (V))
//
// Write memory barrier — LK3RD/kernel do wmb() before DEPCMD
// to ensure PARAM writes are visible before the command.
//
#define DWC3_WMB()    MemoryFence ()

STATIC USB_DEV_DESC gDeviceDescriptor = {
  .Length        = sizeof (USB_DEV_DESC),
  .Type          = 1,               // DEVICE
  .UsbL          = 0x00,
  .UsbH          = 0x02,           // USB 2.0
  .DevClass      = 0xFF,           // Vendor-specific
  .DevSubClass   = 0,
  .DevProtocol   = 0,
  .MaxPkt0       = 64,
  .VendorL       = 0xE8,
  .VendorH       = 0x04,           // Samsung: 0x04E8
  .ProductL      = 0x34,
  .ProductH      = 0x12,           // Product: 0x1234
  .DevRelL       = 0x00,
  .DevRelH       = 0x01,
  .VendorStr     = 1,
  .ProductStr    = 2,
  .SerialStr     = 0,              // No serial string
  .NumConfigs    = 1
};

STATIC USB_CFG_FULL_DESC gConfigDescriptor = {
  .Config = {
    .Length       = sizeof (USB_CFG_DESC),
    .Type         = 2,             // CONFIGURATION
    .TotalLenL    = FULL_CFG_DESC_SIZE & 0xFF,
    .TotalLenH    = (FULL_CFG_DESC_SIZE >> 8) & 0xFF,
    .NumIfs       = 1,
    .CfgVal       = 1,
    .CfgStr       = 0,
    .Attr         = 0x80 | 0x40,   // Default + Self-powered
    .MaxPower     = 25             // 50 mA
  },
  .Interface = {
    .Length       = sizeof (USB_IF_DESC),
    .Type         = 4,             // INTERFACE
    .IfNum        = 0,
    .AltSetting   = 0,
    .NumEps       = 2,
    .IfClass      = 0xFF,
    .IfSubClass   = 0,
    .IfProtocol   = 0,
    .IfStr        = 0
  },
  .EpBulkIn = {
    .Length       = sizeof (USB_EP_DESC),
    .Type         = 5,             // ENDPOINT
    .EpAddr       = 0x81,          // IN, ep1
    .Attr         = 2,             // BULK
    .MaxPktL      = 512 & 0xFF,
    .MaxPktH      = (512 >> 8) & 0xFF,
    .Interval     = 0
  },
  .EpBulkOut = {
    .Length       = sizeof (USB_EP_DESC),
    .Type         = 5,             // ENDPOINT
    .EpAddr       = 0x02,          // OUT, ep2
    .Attr         = 2,             // BULK
    .MaxPktL      = 512 & 0xFF,
    .MaxPktH      = (512 >> 8) & 0xFF,
    .Interval     = 0
  }
};

//
// String descriptor 0 (language IDs): US English (0x0409)
//
STATIC UINT8 gString0[] = { 4, 3, 0x09, 0x04 };

//
// String descriptor "Mu-Silicium UEFI"
//
STATIC UINT8 gString1[] = {
  30, 3,
  'M',0,'u',0,'-',0,'S',0,'i',0,'l',0,'i',0,'c',0,'i',0,'u',0,'m',0,
  ' ',0,'U',0,'E',0,'F',0,'I',0
};

//
// String descriptor "Exynos DWC3 Gadget"
//
STATIC UINT8 gString2[] = {
  46, 3,
  'E',0,'x',0,'y',0,'n',0,'o',0,'s',0,' ',0,'D',0,'W',0,'C',0,'3',0,
  ' ',0,'G',0,'a',0,'d',0,'g',0,'e',0,'t',0
};

//
// Device Qualifier descriptor (for USB 2.0 compliance)
//
STATIC UINT8 gQualifierDesc[] = {
  10, 6, 0x00, 0x02, 0xFF, 0x00, 0x00, 64, 1, 0
};

//
// Other Speed Configuration (full-speed version of our config)
//
STATIC USB_CFG_FULL_DESC gOtherSpeedConfig = {
  .Config = {
    .Length       = 9,
    .Type         = 7,             // OTHER_SPEED_CONFIGURATION
    .TotalLenL    = FULL_CFG_DESC_SIZE & 0xFF,
    .TotalLenH    = (FULL_CFG_DESC_SIZE >> 8) & 0xFF,
    .NumIfs       = 1,
    .CfgVal       = 1,
    .CfgStr       = 0,
    .Attr         = 0x80 | 0x40,
    .MaxPower     = 25
  },
  .Interface = {
    .Length       = 9,
    .Type         = 4,
    .IfNum        = 0,
    .AltSetting   = 0,
    .NumEps       = 2,
    .IfClass      = 0xFF,
    .IfSubClass   = 0,
    .IfProtocol   = 0,
    .IfStr        = 0
  },
  .EpBulkIn = {
    .Length       = 7,
    .Type         = 5,
    .EpAddr       = 0x81,
    .Attr         = 2,
    .MaxPktL      = 64,            // Full-speed MPS = 64
    .MaxPktH      = 0,
    .Interval     = 0
  },
  .EpBulkOut = {
    .Length       = 7,
    .Type         = 5,
    .EpAddr       = 0x02,
    .Attr         = 2,
    .MaxPktL      = 64,            // Full-speed MPS = 64
    .MaxPktH      = 0,
    .Interval     = 0
  }
};

/**
  Read-modify-write poll loop for endpoint command completion.

  Returns EFI_TIMEOUT if the command doesn't complete within 50000us.
**/
STATIC
EFI_STATUS
Dwc3EpWaitCmd (
  IN DWC3_DEV_CTX  *Dev,
  IN DWC3_EP_DIR    Dir,
  IN UINT8          EpNum
  )
{
  UINT64  Base;
  UINT32  Cmd;
  UINT32  Timeout;

  Base    = Dev->Dwc3Base;
  Timeout = 50000;

  //
  // Determine command register address
  //
  if (Dir == Dwc3EpDirIn) {
    Base += DWC3_DIEPCMD (EpNum);
  } else {
    Base += DWC3_DOEPCMD (EpNum);
  }

  do {
    Cmd = DWC3_RD32 (Base);
    if (!(Cmd & DEPCMD_CMD_ACTIVE)) {
      return EFI_SUCCESS;
    }
    gBS->Stall (1);
  } while (Timeout-- > 0);

  DEBUG ((DEBUG_ERROR, "Dwc3Dev: EP%d %s cmd timeout (cmd=0x%08x reg=0x%llx)\n",
          EpNum, (Dir == Dwc3EpDirIn) ? "IN" : "OUT", Cmd, Base));
  return EFI_TIMEOUT;
}

/**
  Issue a Set Endpoint Configuration command.

  Writes DEPCMDPAR0/1 then issues DEPCMD_SET_EP_CFG.
**/
VOID
Dwc3DevSetEpCfg (
  IN DWC3_DEV_CTX  *Dev,
  IN DWC3_EP_DIR    Dir,
  IN UINT8          EpNum,
  IN UINT32         EpType,
  IN UINT32         Mps,
  IN UINT32         FifoNum,
  IN UINT32         Burst,
  IN UINT32         CfgAction
  )
{
  UINT64  Base;
  UINT32  Par0;
  UINT32  Par1;

  Base  = Dev->Dwc3Base;

  //
  // Build DEPCMDPAR0
  //
  Par0  = (EpType << DEPCFG_EP_TYPE_SHIFT);
  Par0 |= (Mps << DEPCFG_MPS_SHIFT);
  Par0 |= (FifoNum << DEPCFG_FIFO_NUM_SHIFT);
  Par0 |= (Burst << DEPCFG_BRST_SIZ_SHIFT);
  Par0 |= (CfgAction << DEPCFG_CFG_ACTION_SHIFT);

  //
  // Build DEPCMDPAR1
  //
  Par1  = DEPCFG1_XFER_CMPL_EN;
  Par1 |= DEPCFG1_XFER_NRDY_EN;
  Par1 |= ((Dir == Dwc3EpDirIn) ? DEPCFG1_EP_DIR_IN : DEPCFG1_EP_DIR_OUT) << DEPCFG1_EP_DIR_SHIFT;
  Par1 |= ((UINT32)EpNum << DEPCFG1_EP_NUM_SHIFT);

  //
  // Write parameters
  //
  if (Dir == Dwc3EpDirIn) {
    DWC3_WR32 (Base + DWC3_DIEPCMDPAR0 (EpNum), Par0);
    DWC3_WR32 (Base + DWC3_DIEPCMDPAR1 (EpNum), Par1);
    DWC3_WR32 (Base + DWC3_DIEPCMDPAR2 (EpNum), 0);
  } else {
    DWC3_WR32 (Base + DWC3_DOEPCMDPAR0 (EpNum), Par0);
    DWC3_WR32 (Base + DWC3_DOEPCMDPAR1 (EpNum), Par1);
    DWC3_WR32 (Base + DWC3_DOEPCMDPAR2 (EpNum), 0);
  }

  //
  // Issue command — wmb() first to flush PARAM writes
  //
  DWC3_WMB ();
  if (Dir == Dwc3EpDirIn) {
    DWC3_WR32 (Base + DWC3_DIEPCMD (EpNum),
               DEPCMD_SET_EP_CFG | DEPCMD_CMD_ACTIVE);
  } else {
    DWC3_WR32 (Base + DWC3_DOEPCMD (EpNum),
               DEPCMD_SET_EP_CFG | DEPCMD_CMD_ACTIVE);
  }

  Dwc3EpWaitCmd (Dev, Dir, EpNum);
}

/**
  Issue a Set Endpoint Transfer Resource Configuration command.
**/
VOID
Dwc3DevSetEpXferRsrc (
  IN DWC3_DEV_CTX  *Dev,
  IN DWC3_EP_DIR    Dir,
  IN UINT8          EpNum,
  IN UINT32         XferRscIdx
  )
{
  UINT64  Base;

  Base = Dev->Dwc3Base;

  //
  // Write DEPCMDPAR0 with transfer resource count
  //
  if (Dir == Dwc3EpDirIn) {
    DWC3_WR32 (Base + DWC3_DIEPCMDPAR0 (EpNum), XferRscIdx);
    DWC3_WR32 (Base + DWC3_DIEPCMDPAR1 (EpNum), 0);
    DWC3_WR32 (Base + DWC3_DIEPCMDPAR2 (EpNum), 0);
    DWC3_WMB ();
    DWC3_WR32 (Base + DWC3_DIEPCMD (EpNum),
               DEPCMD_SET_EP_XFER_RSRC | DEPCMD_CMD_ACTIVE);
  } else {
    DWC3_WR32 (Base + DWC3_DOEPCMDPAR0 (EpNum), XferRscIdx);
    DWC3_WR32 (Base + DWC3_DOEPCMDPAR1 (EpNum), 0);
    DWC3_WR32 (Base + DWC3_DOEPCMDPAR2 (EpNum), 0);
    DWC3_WMB ();
    DWC3_WR32 (Base + DWC3_DOEPCMD (EpNum),
               DEPCMD_SET_EP_XFER_RSRC | DEPCMD_CMD_ACTIVE);
  }

  Dwc3EpWaitCmd (Dev, Dir, EpNum);
}

/**
  Fill a TRB and issue Start Transfer.

  The TRB is programmed with the buffer physical address, transfer size,
  TRB control word, and HWO bit. Then DEPCMD_START_XFER is issued.
**/
VOID
Dwc3DevStartXfer (
  IN  DWC3_DEV_CTX  *Dev,
  IN  DWC3_EP_DIR    Dir,
  IN  UINT8          EpNum,
  IN  DWC3_TRB      *Trb,
  IN  UINT32         StrmId,
  OUT UINT32        *Tri
  )
{
  UINT64  Base;
  UINT64  TrbPhys;
  UINT32  Cmd;

  Base = Dev->Dwc3Base;

  //
  // Write TRB physical address to DEPCMDPAR1 (64-bit: hi=0 for 32-bit)
  //
  TrbPhys = (UINT64)(UINTN)Trb;

  if (Dir == Dwc3EpDirIn) {
    DWC3_WR32 (Base + DWC3_DIEPCMDPAR0 (EpNum), 0);
    DWC3_WR32 (Base + DWC3_DIEPCMDPAR1 (EpNum), (UINT32)TrbPhys);
    DWC3_WR32 (Base + DWC3_DIEPCMDPAR2 (EpNum), 0);
  } else {
    DWC3_WR32 (Base + DWC3_DOEPCMDPAR0 (EpNum), 0);
    DWC3_WR32 (Base + DWC3_DOEPCMDPAR1 (EpNum), (UINT32)TrbPhys);
    DWC3_WR32 (Base + DWC3_DOEPCMDPAR2 (EpNum), 0);
  }

  //
  // Issue Start Transfer
  //
  Cmd  = DEPCMD_START_XFER;
  Cmd |= DEPCMD_CMD_ACTIVE;
  Cmd |= DEPCMD_HIPRI_FORCERM;
  Cmd |= (StrmId << DEPCMD_CMD_PARAM_SHIFT);

  DWC3_WMB ();
  if (Dir == Dwc3EpDirIn) {
    DWC3_WR32 (Base + DWC3_DIEPCMD (EpNum), Cmd);
  } else {
    DWC3_WR32 (Base + DWC3_DOEPCMD (EpNum), Cmd);
  }

  Dwc3EpWaitCmd (Dev, Dir, EpNum);

  //
  // Read Transfer Resource Index
  //
  if (Tri != NULL) {
    if (Dir == Dwc3EpDirIn) {
      *Tri = (DWC3_RD32 (Base + DWC3_DIEPCMD (EpNum)) >> DEPCMD_CMD_PARAM_SHIFT) & TRI_MASK;
    } else {
      *Tri = (DWC3_RD32 (Base + DWC3_DOEPCMD (EpNum)) >> DEPCMD_CMD_PARAM_SHIFT) & TRI_MASK;
    }
  }
}

/**
  Enable an endpoint in DALEPENA register.

  For EP(n) OUT: bit (2*n)
  For EP(n) IN:  bit (2*n + 1)
**/
VOID
Dwc3DevActivateEp (
  IN DWC3_DEV_CTX  *Dev,
  IN DWC3_EP_DIR    Dir,
  IN UINT8          EpNum
  )
{
  UINT64  Base;
  UINT32  Bit;
  UINT32  Reg;

  Base = Dev->Dwc3Base;

  Bit  = EpNum * 2;
  if (Dir == Dwc3EpDirIn) {
    Bit++;
  }

  Reg  = DWC3_RD32 (Base + DWC3_DALEPENA);
  Reg |= (1u << Bit);
  DWC3_WR32 (Base + DWC3_DALEPENA, Reg);
}

/**
  Issue End Transfer command (abort active transfer).
**/
STATIC
VOID
Dwc3DevEndXfer (
  IN DWC3_DEV_CTX  *Dev,
  IN DWC3_EP_DIR    Dir,
  IN UINT8          EpNum,
  IN UINT32         Tri
  )
{
  UINT64  Base;
  UINT32  Cmd;

  Base = Dev->Dwc3Base;

  Cmd  = DEPCMD_END_XFER;
  Cmd |= DEPCMD_CMD_ACTIVE;
  Cmd |= (Tri << DEPCMD_CMD_PARAM_SHIFT);

  DWC3_WMB ();
  if (Dir == Dwc3EpDirIn) {
    DWC3_WR32 (Base + DWC3_DIEPCMD (EpNum), Cmd);
  } else {
    DWC3_WR32 (Base + DWC3_DOEPCMD (EpNum), Cmd);
  }

  Dwc3EpWaitCmd (Dev, Dir, EpNum);
}

/**
  Issue All FIFO Flush via DGCMD.
**/
STATIC
VOID
Dwc3FlushAllFifos (
  IN DWC3_DEV_CTX  *Dev
  )
{
  UINT64  Base;
  UINT32  Cmd;

  Base = Dev->Dwc3Base;

  Cmd  = (DGCMD_ALL_FIFO_FLUSH << DGCMD_CMD_TYPE_SHIFT);
  Cmd |= DGCMD_CMD_ACTIVE;
  DWC3_WR32 (Base + DWC3_DGCMD, Cmd);

  //
  // Wait for command completion
  //
  do {
    gBS->Stall (1);
    Cmd = DWC3_RD32 (Base + DWC3_DGCMD);
  } while (Cmd & DGCMD_CMD_ACTIVE);

  //
  // Wait until Rx FIFO empty
  //
  do {
    gBS->Stall (1);
  } while (!(DWC3_RD32 (Base + DWC3_DSTS) & DSTS_RX_FIFO_EMPTY));
}

/**
  Set/clear Run/Stop bit.
**/
STATIC
VOID
Dwc3SetRunStop (
  IN DWC3_DEV_CTX  *Dev,
  IN BOOLEAN         Run
  )
{
  UINT64  Base;
  UINT32  V;

  Base = Dev->Dwc3Base;

  V = DWC3_RD32 (Base + DWC3_DCTL);
  if (Run) {
    V |= DCTL_RUN_STOP;
  } else {
    V &= ~DCTL_RUN_STOP;
  }
  DWC3_WR32 (Base + DWC3_DCTL, V);
}

/**
  Disable events (mask in GEVNTSIZ).
**/
STATIC
VOID
Dwc3EventDisable (
  IN DWC3_DEV_CTX  *Dev
  )
{
  UINT64  Base;
  UINT32  V;

  Base = Dev->Dwc3Base;

  V  = DWC3_RD32 (Base + DWC3_GEVNTSIZ);
  V |= GEVNTSIZ_EVENT_INT_MASK;
  DWC3_WR32 (Base + DWC3_GEVNTSIZ, V);
}

/**
  Enable events (clear mask in GEVNTSIZ).
**/
STATIC
VOID
Dwc3EventEnable (
  IN DWC3_DEV_CTX  *Dev
  )
{
  UINT64  Base;
  UINT32  V;

  Base = Dev->Dwc3Base;

  V  = DWC3_RD32 (Base + DWC3_GEVNTSIZ);
  V &= ~GEVNTSIZ_EVENT_INT_MASK;
  DWC3_WR32 (Base + DWC3_GEVNTSIZ, V);
}

/**
  Flush event buffer: write current count back to GEVNTCOUNT.
**/
STATIC
VOID
Dwc3EventFlush (
  IN DWC3_DEV_CTX  *Dev
  )
{
  UINT64  Base;
  UINT32  Count;

  Base  = Dev->Dwc3Base;
  Count = DWC3_RD32 (Base + DWC3_GEVNTCOUNT) & GEVNTCOUNT_MASK;
  DWC3_WR32 (Base + DWC3_GEVNTCOUNT, Count);
}

/**
  Read one event from the event buffer.
  Returns number of events remaining, or 0 if buffer is empty.
**/
STATIC
UINT32
Dwc3ReadEvent (
  IN  DWC3_DEV_CTX  *Dev,
  OUT UINT32        *Event
  )
{
  UINT64  Base;
  UINT32  Count;
  UINT32  Pos;

  Base  = Dev->Dwc3Base;
  Count = DWC3_RD32 (Base + DWC3_GEVNTCOUNT) & GEVNTCOUNT_MASK;
  if (Count == 0) {
    return 0;
  }

  Pos = Dev->EventCount % EVENT_BUF_DEPTH;
  *Event = Dev->EventBuffer[Pos];
  Dev->EventCount++;

  //
  // Acknowledge event consumption
  //
  DWC3_WR32 (Base + DWC3_GEVNTCOUNT, 1);

  Count--;
  return Count;
}

/**
  Start EP0 OUT to receive the next SETUP packet.

  Programs a TRB of type SETUP to receive the 8-byte USB device request.
**/
STATIC
EFI_STATUS
Dwc3Ep0StartSetupRx (
  IN DWC3_DEV_CTX  *Dev
  )
{
  DWC3_TRB  *Trb;
  UINT32     Ctrl;

  Trb = Dev->Ep0SetupTrb;

  //
  // Fill TRB: SETUP type, IOC, ISP_IMI, LST
  //
  Trb->BufPtrLo  = (UINT32)(UINTN)&Dev->DeviceRequest;
  Trb->BufPtrHi  = 0;
  Trb->Size       = 8;             // SETUP packet is always 8 bytes
  Ctrl            = TRB_CTRL_SETUP;
  Ctrl           |= TRB_CTRL_LST;
  Ctrl           |= TRB_CTRL_IOC;
  Ctrl           |= TRB_CTRL_ISP_IMI;
  Ctrl           |= TRB_CTRL_HWO;  // Hardware owns this TRB now
  Trb->Ctrl       = Ctrl;

  //
  // Issue Start Transfer for EP0-OUT
  //
  Dwc3DevStartXfer (Dev, Dwc3EpDirOut, 0, Trb, 0,
                    &Dev->TriOut[0]);

  return EFI_SUCCESS;
}

/**
  Start EP0 IN data phase transfer.

  For IN data phase: TRB_CTRL_CTLDATA_1ST
  For IN status phase: TRB_CTRL_STATUS_3 (3-stage) or STATUS_2 (2-stage)
**/
STATIC
EFI_STATUS
Dwc3Ep0StartInXfer (
  IN DWC3_DEV_CTX  *Dev,
  IN VOID           *Buf,
  IN UINT32          Len
  )
{
  DWC3_TRB  *Trb;
  UINT32     Ctrl;

  Trb = Dev->Ep0InTrb;

  //
  // Determine TRB type based on EP0 state
  //
  Ctrl = 0;
  if (Dev->Ep0State == EP0_STATE_IN_STATUS_PHASE) {
    if (Dev->Ep0ThreeStage) {
      Ctrl = TRB_CTRL_STATUS_3;
    } else {
      Ctrl = TRB_CTRL_STATUS_2;
    }
  } else {
    Ctrl = TRB_CTRL_CTLDATA_1ST;
  }

  Ctrl |= TRB_CTRL_LST;
  Ctrl |= TRB_CTRL_IOC;
  Ctrl |= TRB_CTRL_ISP_IMI;

  Trb->BufPtrLo  = (UINT32)(UINTN)Buf;
  Trb->BufPtrHi  = 0;
  Trb->Size       = Len;
  Trb->Ctrl       = Ctrl | TRB_CTRL_HWO;

  Dwc3DevStartXfer (Dev, Dwc3EpDirIn, 0, Trb, 0,
                    &Dev->TriIn[0]);

  return EFI_SUCCESS;
}

/**
  Start EP0 OUT data phase transfer.
**/
STATIC
EFI_STATUS
Dwc3Ep0StartOutXfer (
  IN DWC3_DEV_CTX  *Dev,
  IN VOID           *Buf,
  IN UINT32          Len
  )
{
  DWC3_TRB  *Trb;
  UINT32     Ctrl;

  Trb = Dev->Ep0OutTrb;

  Ctrl = 0;
  if (Dev->Ep0State == EP0_STATE_OUT_STATUS_PHASE) {
    if (Dev->Ep0ThreeStage) {
      Ctrl = TRB_CTRL_STATUS_3;
    } else {
      Ctrl = TRB_CTRL_STATUS_2;
    }
  } else {
    Ctrl = TRB_CTRL_CTLDATA_1ST;
  }

  Ctrl |= TRB_CTRL_LST;
  Ctrl |= TRB_CTRL_IOC;
  Ctrl |= TRB_CTRL_ISP_IMI;

  Trb->BufPtrLo  = (UINT32)(UINTN)Buf;
  Trb->BufPtrHi  = 0;
  Trb->Size       = Len;
  Trb->Ctrl       = Ctrl | TRB_CTRL_HWO;

  Dwc3DevStartXfer (Dev, Dwc3EpDirOut, 0, Trb, 0,
                    &Dev->TriOut[0]);

  return EFI_SUCCESS;
}

/**
  Setup IN status phase (send ZLP on EP0-IN after OUT data).
**/
STATIC
VOID
Dwc3SetupInStatusPhase (
  IN DWC3_DEV_CTX  *Dev
  )
{
  Dev->Ep0State = EP0_STATE_IN_STATUS_PHASE;
  Dwc3Ep0StartInXfer (Dev, (VOID *)&Dev->DeviceRequest, 0);
}

/**
  Setup OUT status phase (send ZLP on EP0-OUT after IN data).
**/
STATIC
VOID
Dwc3SetupOutStatusPhase (
  IN DWC3_DEV_CTX  *Dev
  )
{
  Dev->Ep0State = EP0_STATE_OUT_STATUS_PHASE;
  Dwc3Ep0StartOutXfer (Dev, (VOID *)&Dev->DeviceRequest, 0);
}

/**
  Initialize Endpoint 0 for control transfers.
**/
STATIC
EFI_STATUS
Dwc3Ep0Init (
  IN DWC3_DEV_CTX  *Dev
  )
{
  UINT64  Base;
  EFI_STATUS  Status;

  Base = Dev->Dwc3Base;

  //
  // Start New Configuration — assigns resources for all endpoints.
  //
  DWC3_WR32 (Base + DWC3_DOEPCMDPAR2 (0), 0);
  DWC3_WR32 (Base + DWC3_DOEPCMDPAR1 (0), 0);
  DWC3_WR32 (Base + DWC3_DOEPCMDPAR0 (0), 0);
  DWC3_WMB ();
  DWC3_WR32 (Base + DWC3_DOEPCMD (0),
             DEPCMD_START_NEW_CFG | DEPCMD_CMD_ACTIVE);
  Status = Dwc3EpWaitCmd (Dev, Dwc3EpDirOut, 0);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Dwc3Dev: EP0 StartNewCfg failed (%r)\n", Status));
    return Status;
  }

  //
  // SetEpCfg EP0 OUT: Control, MPS=64, INIT
  //
  Dwc3DevSetEpCfg (Dev, Dwc3EpDirOut, 0,
                    DEPCFG_EP_TYPE_CTRL, Dev->ControlMps,
                    0, 0, DEPCFG_CFG_ACTION_INIT);

  //
  // Transfer resources: 1 for EP0 OUT
  //
  Dwc3DevSetEpXferRsrc (Dev, Dwc3EpDirOut, 0, 1);

  //
  // SetEpCfg EP0 IN: Control, MPS=64, INIT
  //
  Dwc3DevSetEpCfg (Dev, Dwc3EpDirIn, 0,
                    DEPCFG_EP_TYPE_CTRL, Dev->ControlMps,
                    0, 0, DEPCFG_CFG_ACTION_INIT);

  //
  // Transfer resources: 1 for EP0 IN
  //
  Dwc3DevSetEpXferRsrc (Dev, Dwc3EpDirIn, 0, 1);

  return EFI_SUCCESS;
}

/**
  Handle a Standard USB Device Request received via EP0 SETUP.
**/
STATIC
VOID
Dwc3HandleSetup (
  IN DWC3_DEV_CTX  *Dev
  )
{
  UINT32    ReqLen;

  Dev->SetupPending = FALSE;

  //
  // Determine direction and request length
  //
  if (Dev->DeviceRequest.Type & 0x80) {
    Dev->Ep0State = EP0_STATE_IN_DATA_PHASE;
  } else {
    Dev->Ep0State = EP0_STATE_OUT_DATA_PHASE;
  }

  ReqLen = ((UINT32)Dev->DeviceRequest.LengthH << 8) |
            (UINT32)Dev->DeviceRequest.LengthL;
  Dev->DeviceRequestLen = ReqLen;

  Dev->Ep0ThreeStage = TRUE;
  if (ReqLen == 0) {
    Dev->Ep0State = EP0_STATE_IN_WAIT_NRDY;
    Dev->Ep0ThreeStage = FALSE;
  }

  //
  // Only handle Standard type requests; pass others to class driver
  //
  if ((Dev->DeviceRequest.Type & 0x60) != 0) {
    DEBUG ((DEBUG_WARN, "Dwc3Dev: Non-standard request type 0x%02x → class driver\n",
            Dev->DeviceRequest.Type));
    Dev->SetupPending = TRUE;
    return;
  }

  //
  // Dispatch by request code
  //
  switch (Dev->DeviceRequest.Request) {

  case 5:  // SET_ADDRESS
    {
      UINT64  Base = Dev->Dwc3Base;
      UINT32  Dcfg;

      Dcfg  = DWC3_RD32 (Base + DWC3_DCFG);
      Dcfg &= ~DCFG_DEV_ADDR_MASK;
      Dcfg |= ((UINT32)Dev->DeviceRequest.ValueL << DCFG_DEV_ADDR_SHIFT);
      DWC3_WR32 (Base + DWC3_DCFG, Dcfg);

      Dev->UsbState = USBDEV_STATE_ADDRESSED;
      DEBUG ((DEBUG_WARN, "Dwc3Dev: Set Address = %d\n", Dev->DeviceRequest.ValueL));
    }
    break;

  case 6:  // GET_DESCRIPTOR
    switch (Dev->DeviceRequest.ValueH) {
    case 1:  // DEVICE
      Dwc3Ep0StartInXfer (Dev, (VOID *)Dev->DevDesc,
                          MIN (ReqLen, sizeof (USB_DEV_DESC)));
      break;

    case 2:  // CONFIGURATION
      if (ReqLen > 9) {
        Dwc3Ep0StartInXfer (Dev, (VOID *)Dev->CfgDesc,
                            MIN (ReqLen, FULL_CFG_DESC_SIZE));
      } else {
        Dwc3Ep0StartInXfer (Dev, (VOID *)Dev->CfgDesc, 9);
      }
      break;

    case 3:  // STRING
      switch (Dev->DeviceRequest.ValueL) {
      case 0:
        Dwc3Ep0StartInXfer (Dev, gString0, sizeof (gString0));
        break;
      case 1:
        Dwc3Ep0StartInXfer (Dev, gString1, sizeof (gString1));
        break;
      case 2:
        Dwc3Ep0StartInXfer (Dev, gString2, sizeof (gString2));
        break;
      default:
        break;
      }
      break;

    case 5:  // ENDPOINT (interface-specific EP descriptor)
      // Return first EP descriptor from config
      Dwc3Ep0StartInXfer (Dev,
                          (VOID *)&Dev->CfgDesc->EpBulkIn,
                          sizeof (USB_EP_DESC));
      break;

    case 6:  // DEVICE_QUALIFIER
      Dwc3Ep0StartInXfer (Dev, gQualifierDesc,
                          MIN (ReqLen, sizeof (gQualifierDesc)));
      break;

    case 7:  // OTHER_SPEED_CONFIGURATION
      Dwc3Ep0StartInXfer (Dev, (VOID *)&gOtherSpeedConfig,
                          MIN (ReqLen, FULL_CFG_DESC_SIZE));
      break;
    }
    break;

  case 8:  // GET_CONFIGURATION
    {
      UINT8  CfgVal = (Dev->UsbState == USBDEV_STATE_CONFIGURED) ? 1 : 0;
      Dwc3Ep0StartInXfer (Dev, &CfgVal, 1);
    }
    break;

  case 9:  // SET_CONFIGURATION
    Dev->UsbState = USBDEV_STATE_CONFIGURED;
    DEBUG ((DEBUG_WARN, "Dwc3Dev: Set Configuration = %d\n",
            Dev->DeviceRequest.ValueL));
    break;

  case 0:  // GET_STATUS
    switch (Dev->DeviceRequest.Type & 0x03) {
    case 0:  // Device
      Dev->GetStatus.Device = (UINT8)((Dev->RemoteWakeup << 1) | 0x01);
      Dwc3Ep0StartInXfer (Dev, &Dev->GetStatus.Device, 1);
      break;
    case 1:  // Interface
      Dev->GetStatus.Interface = 0;
      Dwc3Ep0StartInXfer (Dev, &Dev->GetStatus.Interface, 1);
      break;
    case 2:  // Endpoint
      if (Dev->DeviceRequest.IndexL == 0) {
        Dwc3Ep0StartInXfer (Dev, &Dev->GetStatus.EpCtrl, 1);
      }
      break;
    }
    break;

  case 1:  // CLEAR_FEATURE
    if ((Dev->DeviceRequest.ValueL == 0) &&
        ((Dev->DeviceRequest.Type & 0x03) == 2)) {
      // Endpoint Halt clear
      if (Dev->DeviceRequest.IndexL == 1) {  // EP1 IN
        Dev->GetStatus.EpIn = 0;
      } else if (Dev->DeviceRequest.IndexL == 2) {  // EP2 OUT
        Dev->GetStatus.EpOut = 0;
      }
    }
    break;

  case 3:  // SET_FEATURE
    if ((Dev->DeviceRequest.ValueL == 0) &&
        ((Dev->DeviceRequest.Type & 0x03) == 2)) {
      // Endpoint Halt set
      if (Dev->DeviceRequest.IndexL == 1) {
        Dev->GetStatus.EpIn = 1;
      } else if (Dev->DeviceRequest.IndexL == 2) {
        Dev->GetStatus.EpOut = 1;
      }
    }
    break;

  case 10:  // GET_INTERFACE
    Dev->GetInterface.AltSetting = 0;
    Dwc3Ep0StartInXfer (Dev, &Dev->GetInterface.AltSetting, 1);
    break;

  case 11:  // SET_INTERFACE
    break;

  default:
    DEBUG ((DEBUG_WARN, "Dwc3Dev: Unhandled request 0x%02x\n",
            Dev->DeviceRequest.Request));
    break;
  }
}

/**
  Handle EP0 OUT transfer complete event.
  Dispatches based on EP0 state machine.
**/
STATIC
VOID
Dwc3HandleEp0OutXferComplete (
  IN DWC3_DEV_CTX  *Dev
  )
{
  switch (Dev->Ep0State) {

  case EP0_STATE_INIT:
    //
    // SETUP packet was received, parse and handle it
    //
    Dwc3HandleSetup (Dev);
    break;

  case EP0_STATE_OUT_DATA_PHASE:
    //
    // OUT data phase complete → go to IN wait for status
    //
    Dev->Ep0State = EP0_STATE_IN_WAIT_NRDY;
    break;

  case EP0_STATE_OUT_STATUS_PHASE:
    //
    // OUT status complete → back to IDLE, rearm SETUP
    //
    Dev->Ep0State = EP0_STATE_INIT;
    Dwc3Ep0StartSetupRx (Dev);
    break;

  default:
    break;
  }
}

/**
  Handle EP0 IN transfer complete event.
**/
STATIC
VOID
Dwc3HandleEp0InXferComplete (
  IN DWC3_DEV_CTX  *Dev
  )
{
  switch (Dev->Ep0State) {

  case EP0_STATE_IN_DATA_PHASE:
    //
    // IN data sent, wait for host to acknowledge (NRDY)
    //
    Dev->Ep0State = EP0_STATE_OUT_WAIT_NRDY;
    break;

  case EP0_STATE_IN_STATUS_PHASE:
    //
    // IN status phase complete → IDLE, rearm SETUP
    //
    Dev->Ep0State = EP0_STATE_INIT;
    Dwc3Ep0StartSetupRx (Dev);
    break;

  default:
    break;
  }
}

/**
  Handle EP0 IN transfer Not Ready event.
**/
STATIC
VOID
Dwc3HandleEp0InXferNotReady (
  IN DWC3_DEV_CTX  *Dev
  )
{
  if (Dev->Ep0State == EP0_STATE_IN_WAIT_NRDY) {
    //
    // Host wants status phase → send ZLP on EP0 IN
    //
    Dwc3SetupInStatusPhase (Dev);
  }
}

/**
  Handle EP0 OUT transfer Not Ready event.

  When the device sent IN data (DEVICE_TO_HOST), the host sends an
  OUT token as status handshake. The EP0 OUT goes NRDY, and we
  transition to the OUT status phase (send ZLP on EP0 OUT).
**/
STATIC
VOID
Dwc3HandleEp0OutXferNotReady (
  IN DWC3_DEV_CTX  *Dev
  )
{
  if (Dev->Ep0State == EP0_STATE_OUT_WAIT_NRDY) {
    //
    // Host wants status phase → send ZLP on EP0 OUT
    //
    Dwc3SetupOutStatusPhase (Dev);
  }
}

/**
  Handle device event (DWC3_EVENT_DEV type).
  Dispatches based on event_info field.
**/
STATIC
EFI_STATUS
Dwc3HandleDeviceEvent (
  IN DWC3_DEV_CTX  *Dev,
  IN UINT32         EventWord
  )
{
  UINT32  EvtType;

  //
  // DWC3 Device Event word layout:
  //   bits[31:8] = DevEventParam (24 bits)
  //   bits[7:1]  = DevEventType  (7 bits)
  //   bit[0]     = NonZero
  //
  EvtType = (EventWord >> 1) & 0x7F;

  switch (EvtType) {

  case DEVT_DISCONNECT_LEVEL:
    DEBUG ((DEBUG_WARN, "Dwc3Dev: Disconnect\n"));
    Dev->UsbState = USBDEV_STATE_DEFAULT;
    Dev->Ep0State = EP0_STATE_INIT;
    break;

  case DEVT_USB_RESET:
    {
      UINT64  Base = Dev->Dwc3Base;
      UINT32  i;

      DEBUG ((DEBUG_WARN, "Dwc3Dev: USB Reset\n"));

      //
      // Stop any active non-EP0 transfers
      //
      for (i = 1; i < MAX_EPS; i++) {
        if (Dev->TriOut[i] != 0) {
          Dwc3DevEndXfer (Dev, Dwc3EpDirOut, (UINT8)i, Dev->TriOut[i]);
          Dev->TriOut[i] = 0;
        }
        if (Dev->TriIn[i] != 0) {
          Dwc3DevEndXfer (Dev, Dwc3EpDirIn, (UINT8)i, Dev->TriIn[i]);
          Dev->TriIn[i] = 0;
        }
      }

      //
      // Flush all FIFOs
      //
      Dwc3FlushAllFifos (Dev);

      //
      // Reset device address to 0
      //
      {
        UINT32  Dcfg;
        Dcfg  = DWC3_RD32 (Base + DWC3_DCFG);
        Dcfg &= ~DCFG_DEV_ADDR_MASK;
        DWC3_WR32 (Base + DWC3_DCFG, Dcfg);
      }

      //
      // Reset state
      //
      Dev->UsbState = USBDEV_STATE_DEFAULT;
      Dev->Ep0State = EP0_STATE_INIT;
    }
    break;

  case DEVT_CONN_DONE:
    {
      UINT64  Base = Dev->Dwc3Base;
      UINT32  Dsts;
      UINT32  Speed;

      DEBUG ((DEBUG_WARN, "Dwc3Dev: Connect Done\n"));

      //
      // Read connected speed from DSTS
      //
      Dsts  = DWC3_RD32 (Base + DWC3_DSTS);
      Speed = (Dsts >> DSTS_CONNECTSPD_SHIFT) & 0x7;

      //
      // Set packet sizes based on speed
      //
      if (Speed == 4) {         // SuperSpeed
        Dev->ControlMps = 512;
        Dev->BulkMps    = 1024;
        Dev->BusSpeed   = 4;
      } else if (Speed == 0) {  // High-Speed
        Dev->ControlMps = 64;
        Dev->BulkMps    = 512;
        Dev->BusSpeed   = 3;
      } else {                  // Full-Speed
        Dev->ControlMps = 64;
        Dev->BulkMps    = 64;
        Dev->BusSpeed   = 2;
      }

      DEBUG ((DEBUG_WARN, "Dwc3Dev: Speed=%d, ControlMPS=%d, BulkMPS=%d\n",
              Dev->BusSpeed, Dev->ControlMps, Dev->BulkMps));

      //
      // Reconfigure EP0 with the connected speed's MPS
      //
      Dwc3DevSetEpCfg (Dev, Dwc3EpDirOut, 0,
                        DEPCFG_EP_TYPE_CTRL, Dev->ControlMps,
                        0, 0, DEPCFG_CFG_ACTION_MODIFY);
      Dwc3DevSetEpCfg (Dev, Dwc3EpDirIn, 0,
                        DEPCFG_EP_TYPE_CTRL, Dev->ControlMps,
                        0, 0, DEPCFG_CFG_ACTION_MODIFY);

      Dev->Ep0State = EP0_STATE_INIT;
    }
    break;

  case DEVT_LINK_STS_CHG:
    DEBUG ((DEBUG_WARN, "Dwc3Dev: Link Status Change (info=%d)\n",
            (EventWord >> 4) & 0xF));
    break;

  case DEVT_WAKEUP:
    DEBUG ((DEBUG_WARN, "Dwc3Dev: Wakeup\n"));
    break;

  case DEVT_OVERFLOW:
    DEBUG ((DEBUG_WARN, "Dwc3Dev: Event buffer overflow\n"));
    break;

  default:
    DEBUG ((DEBUG_WARN, "Dwc3Dev: Unknown device event 0x%x\n", EvtType));
    break;
  }

  return EFI_SUCCESS;
}

/**
  Handle endpoint event (DEPEVT).
  Dispatches based on endpoint event type and direction.
**/
STATIC
EFI_STATUS
Dwc3HandleEpEvent (
  IN DWC3_DEV_CTX  *Dev,
  IN UINT32         EventWord
  )
{
  UINT32  EpNum;
  UINT32  EpDir;  // 0=OUT, 1=IN
  UINT32  EvtType;

  //
  // DWC3 DEPEVT (Endpoint Event) word layout:
  //   bits[31:12] = DepEventParam   (20 bits)
  //   bits[11:6]  = DepEventType    (6 bits)
  //   bits[5:1]   = EndpointNumber  (5 bits)
  //   bit[0]      = EndpointDirection (0=OUT, 1=IN)
  //
  EpNum   = (EventWord >> 1) & 0x1F;
  EpDir   = EventWord & 0x01;

  //
  // DepEventType field
  //
  EvtType = (EventWord >> 6) & 0x3F;

  //
  // EP0 control handling
  //
  if (EpNum == 0) {
    if (EpDir == 0) {  // EP0 OUT
      switch (EvtType) {
      case DEPEVT_XFER_CMPL:
        Dwc3HandleEp0OutXferComplete (Dev);
        break;
      case DEPEVT_XFER_NRDY:
        Dwc3HandleEp0OutXferNotReady (Dev);
        break;
      default:
        break;
      }
    } else {  // EP0 IN
      switch (EvtType) {
      case DEPEVT_XFER_CMPL:
        Dwc3HandleEp0InXferComplete (Dev);
        break;
      case DEPEVT_XFER_NRDY:
        Dwc3HandleEp0InXferNotReady (Dev);
        break;
      default:
        break;
      }
    }
  } else {
    //
    // Non-EP0 endpoint events
    //
    switch (EvtType) {
    case DEPEVT_XFER_CMPL:
      if (EpDir == 0) {  // OUT xfer complete
        Dev->BulkOutActive = FALSE;
      } else {            // IN xfer complete
        Dev->BulkInActive = FALSE;
      }
      break;
    default:
      break;
    }
  }

  return EFI_SUCCESS;
}

/**
  Main event polling loop: reads all pending events from the buffer.
**/
STATIC
VOID
Dwc3PollEvents (
  IN DWC3_DEV_CTX  *Dev
  )
{
  UINT32  Event;
  UINT32  EvtType;

  while (Dwc3ReadEvent (Dev, &Event) > 0) {
    EvtType = Event & DWC3_EVENT_TYPE_MASK;

    if (EvtType == DWC3_EVENT_DEV) {
      Dwc3HandleDeviceEvent (Dev, Event);
    } else {
      Dwc3HandleEpEvent (Dev, Event);
    }
  }
}

/**
  Complete DWC3 device mode initialization.
**/
STATIC
EFI_STATUS
Dwc3DeviceInit (
  IN DWC3_DEV_CTX  *Dev
  )
{
  UINT64      Base;
  UINT32      RegVal;
  EFI_STATUS  Status;

  Base = Dev->Dwc3Base;

  DEBUG ((EFI_D_WARN, "Dwc3Dev: Device init start (base=0x%llx)\n", Base));

  Dwc3CoreEnableClocks (&Dev->PlatConfig);

  RegVal = DWC3_RD32 (Base + DWC3_DCTL);
  if (RegVal & DCTL_RUN_STOP) {
    Dwc3SetRunStop (Dev, FALSE);
    gBS->Stall (30000);
  }

  Dwc3CoreGblInit (&Dev->PlatConfig, NULL);

  Dwc3CoreConfigGctl (&Dev->PlatConfig, DWC3_MODE_DEVICE);

  Dwc3CoreSoftReset (&Dev->PlatConfig);

  Dwc3CoreConfigPhyIf (&Dev->PlatConfig);

  {
    UINT32 Guctl2;
    Guctl2  = DWC3_RD32 (Base + DWC3_GUCTL2);
    Guctl2 |= GUCTL2_RST_ACTBITLATER;
    DWC3_WR32 (Base + DWC3_GUCTL2, Guctl2);
  }

  if (!Dev->PlatConfig.SusPhySupported) {
    RegVal  = DWC3_RD32 (Base + DWC3_GUSB2PHYCFG0);
    RegVal &= ~GUSB2PHYCFG_SUSPHY;
    RegVal &= ~GUSB2PHYCFG_ENBLSLPM;
    DWC3_WR32 (Base + DWC3_GUSB2PHYCFG0, RegVal);
  }

  RegVal  = DCFG_DEV_SPEED_HS << DCFG_DEV_SPEED_SHIFT;
  RegVal |= (0 << DCFG_DEV_ADDR_SHIFT);
  RegVal |= (2 << DCFG_NUMP_SHIFT);       // per_fr_int = 2 (90% periodic)
  RegVal |= (0 << DCFG_INTNUM_SHIFT);
  RegVal |= (4 << DCFG_NUMRXBUF_SHIFT);   // 4 receive buffers
  RegVal |= DCFG_LPM_CAP;                 // LPM capable
  DWC3_WR32 (Base + DWC3_DCFG, RegVal);

  Dwc3EventDisable (Dev);

  //
  // Flush event counter
  //
  Dwc3EventFlush (Dev);

  //
  // Set event buffer address (32-bit physical)
  //
  DWC3_WR32 (Base + DWC3_GEVNTADR_LO,
             (UINT32)(UINTN)Dev->EventBuffer);
  DWC3_WR32 (Base + DWC3_GEVNTADR_HI, 0);

  //
  // Set event buffer size (4 bytes * depth)
  //
  RegVal  = DWC3_RD32 (Base + DWC3_GEVNTSIZ);
  RegVal &= ~GEVNTSIZ_SIZE_MASK;
  RegVal |= (EVENT_BUF_DEPTH * 4);
  DWC3_WR32 (Base + DWC3_GEVNTSIZ, RegVal);

  //
  // Enable events
  //
  Dwc3EventEnable (Dev);

  RegVal  = DEVTEN_DISCONN;
  RegVal |= DEVTEN_USB_RESET;
  RegVal |= DEVTEN_CONN_DONE;
  RegVal |= DEVTEN_U3L2L1_SUSP;
  RegVal |= DEVTEN_LNK_STS_CHNG;
  RegVal |= DEVTEN_OVERFLOW;
  DWC3_WR32 (Base + DWC3_DEVTEN, RegVal);

  Status = Dwc3Ep0Init (Dev);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Dwc3Dev: EP0 init failed (%r)\n", Status));
    return Status;
  }

  Dwc3DevActivateEp (Dev, Dwc3EpDirOut, 0);
  Dwc3DevActivateEp (Dev, Dwc3EpDirIn, 0);

  RegVal  = DWC3_RD32 (Base + DWC3_DCTL);
  RegVal &= ~DCTL_INIT_U1U2_DIS;
  DWC3_WR32 (Base + DWC3_DCTL, RegVal);

  Dwc3SetRunStop (Dev, TRUE);

  Dev->Initialized = TRUE;

  DEBUG ((EFI_D_WARN, "Dwc3Dev: Device init complete\n"));
  DEBUG ((EFI_D_WARN, "Dwc3Dev: GSNPSID=0x%08x\n", DWC3_RD32 (Base + DWC3_GSNPSID)));
  DEBUG ((EFI_D_WARN, "Dwc3Dev: HWPARAMS0=0x%08x HWPARAMS1=0x%08x HWPARAMS3=0x%08x\n",
          DWC3_RD32 (Base + DWC3_GHWPARAMS0),
          DWC3_RD32 (Base + DWC3_GHWPARAMS1),
          DWC3_RD32 (Base + DWC3_GHWPARAMS3)));
  DEBUG ((EFI_D_WARN, "Dwc3Dev: GUSB2PHYCFG0=0x%08x\n", DWC3_RD32 (Base + DWC3_GUSB2PHYCFG0)));
  DEBUG ((EFI_D_WARN, "Dwc3Dev: GCTL=0x%08x\n", DWC3_RD32 (Base + DWC3_GCTL)));
  DEBUG ((EFI_D_WARN, "Dwc3Dev: DCFG=0x%08x\n", DWC3_RD32 (Base + DWC3_DCFG)));
  DEBUG ((EFI_D_WARN, "Dwc3Dev: DCTL=0x%08x\n", DWC3_RD32 (Base + DWC3_DCTL)));
  DEBUG ((EFI_D_WARN, "Dwc3Dev: DSTS=0x%08x\n", DWC3_RD32 (Base + DWC3_DSTS)));
  DEBUG ((EFI_D_WARN, "Dwc3Dev: DEVTEN=0x%08x\n", DWC3_RD32 (Base + DWC3_DEVTEN)));

  //
  // Force VBUS now that DWC3 is running (PHY init deferred VBUS).
  // This creates the VBUS rising edge the DWC3 needs to connect.
  //
  UsbPhyConnect ();

  return EFI_SUCCESS;
}

/**
  DetectPort: Returns the USB port type.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnDetectPort (
  IN  EFI_USBFN_IO_PROTOCOL  *This,
  OUT EFI_USBFN_PORT_TYPE    *PortType
  )
{
  DEBUG ((EFI_D_WARN, "Dwc3FnDetectPort: called\n"));
  if (PortType == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  *PortType = EfiUsbStandardDownstreamPort;
  return EFI_SUCCESS;
}

/**
  ConfigureEnableEndpoints: Initialize controller + configure endpoints.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnConfigureEnableEndpoints (
  IN  EFI_USBFN_IO_PROTOCOL  *This,
  OUT EFI_USB_DEVICE_INFO    *DeviceInfo
  )
{
  DWC3_DEV_CTX  *Dev;
  EFI_STATUS     Status;

  Dev = BASE_CR (This, DWC3_DEV_CTX, UsbfnIo);

  DEBUG ((EFI_D_WARN, "Dwc3FnConfigureEnableEndpoints: entry\n"));

  if (DeviceInfo == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  //
  // Initialize device on first call
  //
  if (!Dev->Initialized) {
    Status = Dwc3DeviceInit (Dev);
    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  //
  // Ensure Run/Stop is set
  //
  Dwc3SetRunStop (Dev, TRUE);

  Dev->Started = TRUE;
  return EFI_SUCCESS;
}

/**
  GetEndpointMaxPacketSize: Returns MPS for the given endpoint type and speed.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnGetEndpointMaxPacketSize (
  IN  EFI_USBFN_IO_PROTOCOL  *This,
  IN  EFI_USB_ENDPOINT_TYPE   EndpointType,
  IN  EFI_USB_BUS_SPEED       BusSpeed,
  OUT UINT16                 *MaxPacketSize
  )
{
  if (MaxPacketSize == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  if (EndpointType == UsbEndpointControl) {
    *MaxPacketSize = 64;
  } else if (EndpointType == UsbEndpointBulk) {
    *MaxPacketSize = 512;
  } else {
    return EFI_INVALID_PARAMETER;
  }

  return EFI_SUCCESS;
}

/**
  GetDeviceInfo: Return device info strings.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnGetDeviceInfo (
  IN     EFI_USBFN_IO_PROTOCOL     *This,
  IN     EFI_USBFN_DEVICE_INFO_ID   Id,
  IN OUT UINTN                     *BufferSize,
  OUT    VOID                      *Buffer OPTIONAL
  )
{
  if (BufferSize == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  if ((*BufferSize != 0) && (Buffer == NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  return EFI_UNSUPPORTED;
}

/**
  GetVendorIdProductId: Return VID and PID.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnGetVendorIdProductId (
  IN  EFI_USBFN_IO_PROTOCOL  *This,
  OUT UINT16                 *Vid,
  OUT UINT16                 *Pid
  )
{
  if ((Vid == NULL) || (Pid == NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  *Vid = 0x04E8;   // Samsung
  *Pid = 0x1234;
  return EFI_SUCCESS;
}

/**
  AbortTransfer: Cancel an active transfer on the specified endpoint.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnAbortTransfer (
  IN EFI_USBFN_IO_PROTOCOL          *This,
  IN UINT8                           EndpointIndex,
  IN EFI_USBFN_ENDPOINT_DIRECTION    Direction
  )
{
  DWC3_DEV_CTX  *Dev;
  DWC3_EP_DIR    Dir;

  Dev = BASE_CR (This, DWC3_DEV_CTX, UsbfnIo);

  if (EndpointIndex >= MAX_EPS) {
    return EFI_INVALID_PARAMETER;
  }

  Dir = (Direction == EfiUsbEndpointDirectionDeviceTx) ?
         Dwc3EpDirIn : Dwc3EpDirOut;

  if (Dir == Dwc3EpDirIn) {
    if (Dev->TriIn[EndpointIndex] != 0) {
      Dwc3DevEndXfer (Dev, Dir, EndpointIndex, Dev->TriIn[EndpointIndex]);
      Dev->TriIn[EndpointIndex] = 0;
      Dev->BulkInActive = FALSE;
    }
  } else {
    if (Dev->TriOut[EndpointIndex] != 0) {
      Dwc3DevEndXfer (Dev, Dir, EndpointIndex, Dev->TriOut[EndpointIndex]);
      Dev->TriOut[EndpointIndex] = 0;
      Dev->BulkOutActive = FALSE;
    }
  }

  return EFI_SUCCESS;
}

/**
  GetEndpointStallState: Return stall state.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnGetEndpointStallState (
  IN     EFI_USBFN_IO_PROTOCOL         *This,
  IN     UINT8                          EndpointIndex,
  IN     EFI_USBFN_ENDPOINT_DIRECTION   Direction,
  IN OUT BOOLEAN                       *State
  )
{
  DWC3_DEV_CTX  *Dev;
  UINT32         EpIdx;

  Dev  = BASE_CR (This, DWC3_DEV_CTX, UsbfnIo);

  if ((State == NULL) || (EndpointIndex >= MAX_EPS)) {
    return EFI_INVALID_PARAMETER;
  }

  EpIdx  = EndpointIndex * 2;
  if (Direction == EfiUsbEndpointDirectionDeviceTx) {
    EpIdx++;
  }

  *State = Dev->EpStalled[EpIdx];
  return EFI_SUCCESS;
}

/**
  SetEndpointStallState: Set or clear stall.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnSetEndpointStallState (
  IN     EFI_USBFN_IO_PROTOCOL         *This,
  IN     UINT8                          EndpointIndex,
  IN     EFI_USBFN_ENDPOINT_DIRECTION   Direction,
  IN OUT BOOLEAN                       *State
  )
{
  DWC3_DEV_CTX  *Dev;
  DWC3_EP_DIR    Dir;
  UINT32         EpIdx;
  UINT64         Base;
  UINT32         Cmd;

  Dev  = BASE_CR (This, DWC3_DEV_CTX, UsbfnIo);
  Base = Dev->Dwc3Base;

  if ((State == NULL) || (EndpointIndex >= MAX_EPS)) {
    return EFI_INVALID_PARAMETER;
  }

  Dir   = (Direction == EfiUsbEndpointDirectionDeviceTx) ?
           Dwc3EpDirIn : Dwc3EpDirOut;
  EpIdx = EndpointIndex * 2;
  if (Direction == EfiUsbEndpointDirectionDeviceTx) {
    EpIdx++;
  }

  if (*State) {
    Cmd = DEPCMD_SET_STALL | DEPCMD_CMD_ACTIVE;
  } else {
    Cmd = DEPCMD_CLR_STALL | DEPCMD_CMD_ACTIVE;
  }

  if (Dir == Dwc3EpDirIn) {
    DWC3_WMB ();
    DWC3_WR32 (Base + DWC3_DIEPCMD (EndpointIndex), Cmd);
  } else {
    DWC3_WMB ();
    DWC3_WR32 (Base + DWC3_DOEPCMD (EndpointIndex), Cmd);
  }

  Dwc3EpWaitCmd (Dev, Dir, EndpointIndex);
  Dev->EpStalled[EpIdx] = *State;

  return EFI_SUCCESS;
}

/**
  EventHandler: Poll for USB events and return them.

  Reads the event buffer via GEVNTCOUNT polling, dispatches device/endpoint
  events, and returns EFI_USBFN_MESSAGE notifications for the USB class driver.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnEventHandler (
  IN     EFI_USBFN_IO_PROTOCOL     *This,
  OUT    EFI_USBFN_MESSAGE         *Message,
  IN OUT UINTN                     *PayloadSize,
  OUT    EFI_USBFN_MESSAGE_PAYLOAD *Payload
  )
{
  DWC3_DEV_CTX  *Dev;

  Dev = BASE_CR (This, DWC3_DEV_CTX, UsbfnIo);

  if ((Message == NULL) || (PayloadSize == NULL) || (Payload == NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  //
  // Poll for new events (GEVNTCOUNT register read)
  //
  Dwc3PollEvents (Dev);

  //
  // Check for pending SETUP
  //
  if (Dev->SetupPending) {
    Dev->SetupPending = FALSE;
    *Message    = EfiUsbMsgSetupPacket;
    *PayloadSize = sizeof (EFI_USB_DEVICE_REQUEST);
    CopyMem (&Payload->udr, &Dev->DeviceRequest, sizeof (EFI_USB_DEVICE_REQUEST));
    return EFI_SUCCESS;
  }

  //
  // Check bulk transfer completion
  //
  if (!Dev->BulkInActive && Dev->BulkInLen > 0) {
    *Message    = EfiUsbMsgEndpointStatusChangedTx;
    *PayloadSize = sizeof (EFI_USBFN_TRANSFER_RESULT);
    Payload->utr.BytesTransferred = Dev->BulkInLen;
    Payload->utr.TransferStatus   = UsbTransferStatusComplete;
    Payload->utr.EndpointIndex    = 1;  // BULK IN EP
    Payload->utr.Direction        = EfiUsbEndpointDirectionDeviceTx;
    Payload->utr.Buffer           = Dev->BulkInBuf;
    Dev->BulkInLen = 0;
    return EFI_SUCCESS;
  }

  if (!Dev->BulkOutActive && Dev->BulkOutLen > 0) {
    *Message    = EfiUsbMsgEndpointStatusChangedRx;
    *PayloadSize = sizeof (EFI_USBFN_TRANSFER_RESULT);
    Payload->utr.BytesTransferred = Dev->BulkOutLen;
    Payload->utr.TransferStatus   = UsbTransferStatusComplete;
    Payload->utr.EndpointIndex    = 2;  // BULK OUT EP
    Payload->utr.Direction        = EfiUsbEndpointDirectionDeviceRx;
    Payload->utr.Buffer           = Dev->BulkOutBuf;
    Dev->BulkOutLen = 0;
    return EFI_SUCCESS;
  }

  //
  // No pending events
  //
  *Message    = EfiUsbMsgNone;
  *PayloadSize = 0;
  return EFI_SUCCESS;
}

/**
  Transfer: Start a transfer on the specified endpoint.

  For RX (Host→Device): Programs an OUT TRB with the supplied buffer.
  For TX (Device→Host): Programs an IN TRB with the supplied data.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnTransfer (
  IN     EFI_USBFN_IO_PROTOCOL          *This,
  IN     UINT8                           EndpointIndex,
  IN     EFI_USBFN_ENDPOINT_DIRECTION    Direction,
  IN OUT UINTN                          *BufferSize,
  IN OUT VOID                           *Buffer
  )
{
  DWC3_DEV_CTX  *Dev;
  DWC3_EP_DIR    Dir;
  DWC3_TRB      *Trb;
  UINT32         Ctrl;
  UINT32        *Tri;

  Dev = BASE_CR (This, DWC3_DEV_CTX, UsbfnIo);

  if ((BufferSize == NULL) || (Buffer == NULL) || (EndpointIndex >= MAX_EPS)) {
    return EFI_INVALID_PARAMETER;
  }

  if (!Dev->Started) {
    return EFI_NOT_READY;
  }

  Dir = (Direction == EfiUsbEndpointDirectionDeviceTx) ?
         Dwc3EpDirIn : Dwc3EpDirOut;

  //
  // Allocate a TRB for this transfer (simple single-TRB transfer)
  //
  Trb = AllocateZeroPool (sizeof (DWC3_TRB));
  if (Trb == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  //
  // Build TRB control word
  //
  Ctrl  = TRB_CTRL_NORMAL;
  Ctrl |= TRB_CTRL_LST;
  Ctrl |= TRB_CTRL_IOC;
  Ctrl |= TRB_CTRL_ISP_IMI;

  Trb->BufPtrLo  = (UINT32)(UINTN)Buffer;
  Trb->BufPtrHi  = 0;
  Trb->Size       = (UINT32)*BufferSize;
  Trb->Ctrl       = Ctrl | TRB_CTRL_HWO;

  //
  // Start the transfer
  //
  if (Dir == Dwc3EpDirIn) {
    Tri = &Dev->TriIn[EndpointIndex];
    Dev->BulkInActive = TRUE;
    Dev->BulkInBuf    = Buffer;
    Dev->BulkInLen    = *BufferSize;
  } else {
    Tri = &Dev->TriOut[EndpointIndex];
    Dev->BulkOutActive = TRUE;
    Dev->BulkOutBuf    = Buffer;
    Dev->BulkOutLen    = *BufferSize;
  }

  Dwc3DevStartXfer (Dev, Dir, EndpointIndex, Trb, 0, Tri);

  return EFI_SUCCESS;
}

/**
  GetMaxTransferSize: Returns the maximum transfer size supported.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnGetMaxTransferSize (
  IN  EFI_USBFN_IO_PROTOCOL  *This,
  OUT UINTN                  *MaxTransferSize
  )
{
  if (MaxTransferSize == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  *MaxTransferSize = 0x10000;  // 64KB max per TRB
  return EFI_SUCCESS;
}

/**
  Allocates a transfer buffer that satisfies controller requirements.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnAllocateTransferBuffer (
  IN  EFI_USBFN_IO_PROTOCOL  *This,
  IN  UINTN                   Size,
  OUT VOID                   **Buffer
  )
{
  if ((Buffer == NULL) || (Size == 0)) {
    return EFI_INVALID_PARAMETER;
  }

  *Buffer = AllocatePool (Size);
  if (*Buffer == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  return EFI_SUCCESS;
}

/**
  Deallocates a transfer buffer previously allocated by AllocateTransferBuffer.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnFreeTransferBuffer (
  IN  EFI_USBFN_IO_PROTOCOL  *This,
  IN  VOID                   *Buffer
  )
{
  if (Buffer == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  FreePool (Buffer);
  return EFI_SUCCESS;
}

/**
  Supplies power to the USB controller if needed and initializes the
  hardware and internal data structures. The port must NOT be activated
  by this function (that's ConfigureEnableEndpoints' job).
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnStartController (
  IN  EFI_USBFN_IO_PROTOCOL  *This
  )
{
  DWC3_DEV_CTX  *Dev;
  EFI_STATUS     Status;

  Dev  = DWC3_DEV_FROM_PROTO (This);

  DEBUG ((EFI_D_WARN, "Dwc3FnStartController: entry (Started=%d)\n", Dev->Started));

  //
  // Already started?
  //
  if (Dev->Started) {
    DEBUG ((EFI_D_WARN, "Dwc3FnStartController: already started, return\n"));
    return EFI_SUCCESS;
  }

  //
  // Delegate full hardware init to Dwc3DeviceInit
  //
  Status = Dwc3DeviceInit (Dev);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Dwc3DeviceDxe: DeviceInit failed (%r)\n", Status));
    return Status;
  }

  Dev->Initialized = TRUE;
  Dev->Started     = TRUE;
  DEBUG ((EFI_D_WARN, "Dwc3DeviceDxe: Controller started\n"));
  return EFI_SUCCESS;
}

/**
  Stops the USB hardware device.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnStopController (
  IN  EFI_USBFN_IO_PROTOCOL  *This
  )
{
  DWC3_DEV_CTX  *Dev;
  UINT64         Base;
  UINT32         i;

  Dev  = DWC3_DEV_FROM_PROTO (This);
  Base = Dev->Dwc3Base;

  if (!Dev->Started) {
    return EFI_SUCCESS;
  }

  DWC3_WR32 (Base + DWC3_DEVTEN, 0);

  for (i = 1; i < MAX_EPS; i++) {
    if (Dev->TriOut[i] != 0) {
      Dwc3DevEndXfer (Dev, Dwc3EpDirOut, (UINT8)i, Dev->TriOut[i]);
      Dev->TriOut[i] = 0;
    }
    if (Dev->TriIn[i] != 0) {
      Dwc3DevEndXfer (Dev, Dwc3EpDirIn, (UINT8)i, Dev->TriIn[i]);
      Dev->TriIn[i] = 0;
    }
  }

  Dwc3SetRunStop (Dev, FALSE);

  Dwc3FlushAllFifos (Dev);

  Dev->Ep0State     = EP0_STATE_INIT;
  Dev->UsbState     = USBDEV_STATE_DEFAULT;
  Dev->EpsActive    = FALSE;
  Dev->BulkOutActive = FALSE;
  Dev->BulkInActive  = FALSE;

  Dev->Started = FALSE;
  DEBUG ((DEBUG_WARN, "Dwc3DeviceDxe: Controller stopped\n"));
  return EFI_SUCCESS;
}

/**
  Sets the configuration policy for the specified non-control endpoint.
  Not supported for DWC3 bulk endpoints.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnSetEndpointPolicy (
  IN  EFI_USBFN_IO_PROTOCOL         *This,
  IN  UINT8                          EndpointIndex,
  IN  EFI_USBFN_ENDPOINT_DIRECTION   Direction,
  IN  EFI_USBFN_POLICY_TYPE          PolicyType,
  IN  UINTN                          BufferSize,
  IN  VOID                          *Buffer
  )
{
  return EFI_UNSUPPORTED;
}

/**
  Retrieves the configuration policy for the specified non-control endpoint.
  Not supported for DWC3 bulk endpoints.
**/
STATIC
EFI_STATUS
EFIAPI
Dwc3FnGetEndpointPolicy (
  IN     EFI_USBFN_IO_PROTOCOL         *This,
  IN     UINT8                          EndpointIndex,
  IN     EFI_USBFN_ENDPOINT_DIRECTION   Direction,
  IN     EFI_USBFN_POLICY_TYPE          PolicyType,
  IN OUT UINTN                         *BufferSize,
  IN OUT VOID                          *Buffer
  )
{
  return EFI_UNSUPPORTED;
}

//
// =========================================================================
// Protocol instance
// =========================================================================
//

STATIC EFI_USBFN_IO_PROTOCOL  gUsbfnIoProtocol = {
  EFI_USBFN_IO_PROTOCOL_REVISION,
  Dwc3FnDetectPort,
  Dwc3FnConfigureEnableEndpoints,
  Dwc3FnGetEndpointMaxPacketSize,
  Dwc3FnGetDeviceInfo,
  Dwc3FnGetVendorIdProductId,
  Dwc3FnAbortTransfer,
  Dwc3FnGetEndpointStallState,
  Dwc3FnSetEndpointStallState,
  Dwc3FnEventHandler,
  Dwc3FnTransfer,
  Dwc3FnGetMaxTransferSize,
  Dwc3FnAllocateTransferBuffer,
  Dwc3FnFreeTransferBuffer,
  Dwc3FnStartController,
  Dwc3FnStopController,
  Dwc3FnSetEndpointPolicy,
  Dwc3FnGetEndpointPolicy,
};

//
// =========================================================================
// Driver Entry Point
// =========================================================================
//

/**
  Entry point: Initialize DWC3 device mode and install EFI_USBFN_IO_PROTOCOL.
**/
EFI_STATUS
EFIAPI
InitializeDwc3Device (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS     Status;
  DWC3_DEV_CTX  *Dev;

  DEBUG ((EFI_D_WARN, "Dwc3DeviceDxe: Entry\n"));

  //
  // Allocate driver context
  //
  Dev = AllocateZeroPool (sizeof (DWC3_DEV_CTX));
  if (Dev == NULL) {
    DEBUG ((DEBUG_ERROR, "Dwc3DeviceDxe: Context allocation failed\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  //
  // Load platform config from SoC library
  //
  Status = GetDwc3PlatConfig (&Dev->PlatConfig);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Dwc3DeviceDxe: GetDwc3PlatConfig failed (%r)\n",
            Status));
    goto ErrorExit;
  }

  //
  // Map DWC3 controller MMIO region before any access
  //
  Status = MapMemoryRegion (Dev->PlatConfig.BaseAddress,
                            Dev->PlatConfig.BaseSize,
                            EfiMemoryMappedIO);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Dwc3DeviceDxe: MapMemoryRegion failed (%r)\n",
            Status));
    goto ErrorExit;
  }

  Dev->Dwc3Base = Dev->PlatConfig.BaseAddress;

  //
  // Allocate event buffer (4 bytes * 64 events = 256 bytes, aligned to 4)
  //
  Dev->EventBuffer = AllocateZeroPool (4 * EVENT_BUF_DEPTH);
  if (Dev->EventBuffer == NULL) {
    DEBUG ((DEBUG_ERROR, "Dwc3DeviceDxe: Event buffer alloc failed\n"));
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }

  //
  // Allocate EP0 TRBs (each 16 bytes)
  //
  Dev->Ep0SetupTrb = AllocateZeroPool (sizeof (DWC3_TRB));
  Dev->Ep0InTrb     = AllocateZeroPool (sizeof (DWC3_TRB));
  Dev->Ep0OutTrb    = AllocateZeroPool (sizeof (DWC3_TRB));
  if ((Dev->Ep0SetupTrb == NULL) ||
      (Dev->Ep0InTrb    == NULL) ||
      (Dev->Ep0OutTrb   == NULL)) {
    DEBUG ((DEBUG_ERROR, "Dwc3DeviceDxe: EP0 TRB alloc failed\n"));
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }

  //
  // Allocate descriptor storage (allocated, not static, so class driver can modify)
  //
  Dev->DevDesc = AllocateZeroPool (sizeof (USB_DEV_DESC));
  Dev->CfgDesc = AllocateZeroPool (sizeof (USB_CFG_FULL_DESC));
  if ((Dev->DevDesc == NULL) || (Dev->CfgDesc == NULL)) {
    DEBUG ((DEBUG_ERROR, "Dwc3DeviceDxe: Descriptor alloc failed\n"));
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }

  CopyMem (Dev->DevDesc, &gDeviceDescriptor,  sizeof (USB_DEV_DESC));
  CopyMem (Dev->CfgDesc, &gConfigDescriptor, sizeof (USB_CFG_FULL_DESC));

  //
  // Set initial state
  //
  Dev->ControlMps  = 64;
  Dev->BulkMps     = 512;
  Dev->Ep0State    = EP0_STATE_INIT;
  Dev->UsbState    = USBDEV_STATE_DEFAULT;
  Dev->GetStatus.Device = 0x01;  // Self-powered

  //
  // Copy protocol table
  //
  CopyMem (&Dev->UsbfnIo, &gUsbfnIoProtocol, sizeof (EFI_USBFN_IO_PROTOCOL));

  //
  // Install protocol
  //
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &ImageHandle,
                  &gEfiUsbFunctionIoProtocolGuid,
                  &Dev->UsbfnIo,
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Dwc3DeviceDxe: InstallProtocol failed (%r)\n",
            Status));
    goto ErrorExit;
  }

  DEBUG ((EFI_D_WARN, "Dwc3DeviceDxe: Installed EFI_USBFN_IO_PROTOCOL\n"));

  //
  // Auto-start controller on driver load
  //
  Status = Dwc3FnStartController (&Dev->UsbfnIo);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Dwc3DeviceDxe: Auto-start failed (%r)\n", Status));
  }

  DEBUG ((DEBUG_INFO, "Dwc3DeviceDxe: Exit\n"));
  return EFI_SUCCESS;

ErrorExit:
  if (Dev->EventBuffer   != NULL) FreePool (Dev->EventBuffer);
  if (Dev->Ep0SetupTrb  != NULL) FreePool (Dev->Ep0SetupTrb);
  if (Dev->Ep0InTrb     != NULL) FreePool (Dev->Ep0InTrb);
  if (Dev->Ep0OutTrb    != NULL) FreePool (Dev->Ep0OutTrb);
  if (Dev->DevDesc      != NULL) FreePool (Dev->DevDesc);
  if (Dev->CfgDesc      != NULL) FreePool (Dev->CfgDesc);
  FreePool (Dev);
  return Status;
}
