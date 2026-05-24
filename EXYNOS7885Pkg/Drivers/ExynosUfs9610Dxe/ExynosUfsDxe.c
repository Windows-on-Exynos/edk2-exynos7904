/** @file
  Exynos 9610 UFS DXE Driver - UEFI entry point and EFI_BLOCK_IO_PROTOCOL.

  This driver:
    1. Initializes the UFS host controller (HCE, CAL, link, gear change)
    2. Enumerates LUNs 0..7 and exposes each as an EFI_BLOCK_IO_PROTOCOL instance.

  Copyright (c) 2024, EDK2 Port Authors.
  SPDX-License-Identifier: BSD-2-Clause-Patent
**/

#include "ExynosUfs.h"
#include <Library/DevicePathLib.h>
#include <Protocol/DiskIo.h>
#include <Protocol/SimpleFileSystem.h>

/* ─── Vendor device path node (used as path for each LUN) ───────────────────*/
#pragma pack(1)
typedef struct {
  VENDOR_DEVICE_PATH        VendorDp;
  UINT8                     Lun;
  EFI_DEVICE_PATH_PROTOCOL  End;
} EXYNOS_UFS_DEVICE_PATH;
#pragma pack()

STATIC EFI_GUID gExynosUfsGuid = {
  0x7320D216, 0xEBD9, 0x4863,
  {0xB6, 0xC2, 0x13, 0x59, 0xE3, 0xBA, 0x05, 0xE2}
};

/*
 * Installed LUN handles - kept module-scope so that the DiskIo protocol-notify
 * callback and the ReadyToBoot event can reconnect them without rescanning the
 * entire handle database.
 */
STATIC EFI_HANDLE  gLunHandles[8]  = { NULL };
STATIC UINT32      gLunHandleCount = 0;

/* ─── Forward declaration ────────────────────────────────────────────────────*/
STATIC VOID EFIAPI ExynosUfsConnectLuns (VOID);

/* ─── BlockIO protocol handlers ─────────────────────────────────────────────*/

STATIC EFI_STATUS EFIAPI
ExynosUfsReset (
  IN EFI_BLOCK_IO_PROTOCOL *This,
  IN BOOLEAN                ExtendedVerification)
{
  return EFI_SUCCESS;
}

/*
 * Maximum number of PRDT entries per UTP command.
 *
 * The PRDT region of the command descriptor starts at offset 0x800 and each
 * entry is 16 bytes.  The cmd_desc allocation must be large enough to hold
 * all entries; 128 entries x 16 bytes = 2 KB which fits safely in a typical
 * 4 KB cmd_desc page.  Each entry covers one 4 KB page, so 128 entries gives
 * a maximum of 512 KB per UTP command.
 *
 * Windows winload reads boot.wim in large (often > 512 KB) chunks.  Without
 * chunking the PRDT loop in UfsUtpCmdProcess would write past the end of the
 * cmd_desc buffer and fault.  We split every caller-supplied buffer into
 * UFS_MAX_XFER_BYTES slices and issue one UTP command per slice.
 */
#define UFS_MAX_PRDT_ENTRIES  128U
#define UFS_MAX_XFER_BYTES    (UFS_MAX_PRDT_ENTRIES * SIZE_4KB)   /* 512 KB */

STATIC EFI_STATUS EFIAPI
ExynosUfsReadBlocks (
  IN  EFI_BLOCK_IO_PROTOCOL *This,
  IN  UINT32                 MediaId,
  IN  EFI_LBA                Lba,
  IN  UINTN                  BufferSize,
  OUT VOID                  *Buffer)
{
  EXYNOS_UFS_DEV *Dev = EXYNOS_UFS_FROM_BLOCKIO (This);
  UINTN           BlkSize;
  UINTN           MaxBytes;
  UINTN           ThisBytes;
  UINTN           ThisBlkCnt;
  EFI_LBA         CurLba;
  UINT8          *CurBuf;
  UFS_SCSI_CMD    Cmd;

  if (!Buffer)                        return EFI_INVALID_PARAMETER;
  if (MediaId != Dev->Media.MediaId)  return EFI_MEDIA_CHANGED;
  if (BufferSize == 0)                return EFI_SUCCESS;

  BlkSize = Dev->Media.BlockSize;
  if (BufferSize % BlkSize)          return EFI_BAD_BUFFER_SIZE;

  /* Round MaxBytes down to a whole number of blocks */
  MaxBytes = (UFS_MAX_XFER_BYTES / BlkSize) * BlkSize;

  CurLba = Lba;
  CurBuf = (UINT8 *)Buffer;

  DEBUG ((DEBUG_VERBOSE, "UFS: ReadBlocks LUN%u LBA=%Lu total=%Lu\n",
          Dev->Lun, Lba, (UINT64)(BufferSize / BlkSize)));

  while (BufferSize > 0) {
    ThisBytes  = MIN (BufferSize, MaxBytes);
    ThisBlkCnt = ThisBytes / BlkSize;

    ZeroMem (&Cmd, sizeof (Cmd));
    Cmd.cdb[0] = SCSI_OP_READ_10;
    Cmd.cdb[2] = (u8)((CurLba >> 24) & 0xFF);
    Cmd.cdb[3] = (u8)((CurLba >> 16) & 0xFF);
    Cmd.cdb[4] = (u8)((CurLba >>  8) & 0xFF);
    Cmd.cdb[5] = (u8)( CurLba        & 0xFF);
    Cmd.cdb[7] = (u8)((ThisBlkCnt >>  8) & 0xFF);
    Cmd.cdb[8] = (u8)( ThisBlkCnt        & 0xFF);
    Cmd.buf     = CurBuf;
    Cmd.datalen = (UINT32)ThisBytes;
    Cmd.lun     = Dev->Lun;

    if (UfsUtpCmdProcess (Dev->Ufs, &Cmd)) {
      DEBUG ((DEBUG_INFO, "UFS: ReadBlocks LUN%u LBA=%Lu cnt=%Lu FAILED\n",
              Dev->Lun, CurLba, (UINT64)ThisBlkCnt));
      return EFI_DEVICE_ERROR;
    }

    BufferSize -= ThisBytes;
    CurBuf     += ThisBytes;
    CurLba     += (EFI_LBA)ThisBlkCnt;
  }

  return EFI_SUCCESS;
}

STATIC EFI_STATUS EFIAPI
ExynosUfsWriteBlocks (
  IN EFI_BLOCK_IO_PROTOCOL *This,
  IN UINT32                 MediaId,
  IN EFI_LBA                Lba,
  IN UINTN                  BufferSize,
  IN VOID                  *Buffer)
{
  EXYNOS_UFS_DEV *Dev = EXYNOS_UFS_FROM_BLOCKIO (This);
  UINTN           BlkSize;
  UINTN           MaxBytes;
  UINTN           ThisBytes;
  UINTN           ThisBlkCnt;
  EFI_LBA         CurLba;
  UINT8          *CurBuf;
  UFS_SCSI_CMD    Cmd;

  if (!Buffer)                        return EFI_INVALID_PARAMETER;
  if (MediaId != Dev->Media.MediaId)  return EFI_MEDIA_CHANGED;
  if (Dev->Media.ReadOnly)            return EFI_WRITE_PROTECTED;
  if (BufferSize == 0)                return EFI_SUCCESS;

  BlkSize = Dev->Media.BlockSize;
  if (BufferSize % BlkSize)          return EFI_BAD_BUFFER_SIZE;

  /* Round MaxBytes down to a whole number of blocks */
  MaxBytes = (UFS_MAX_XFER_BYTES / BlkSize) * BlkSize;

  CurLba = Lba;
  CurBuf = (UINT8 *)Buffer;

  while (BufferSize > 0) {
    ThisBytes  = MIN (BufferSize, MaxBytes);
    ThisBlkCnt = ThisBytes / BlkSize;

    ZeroMem (&Cmd, sizeof (Cmd));
    Cmd.cdb[0] = SCSI_OP_WRITE_10;
    Cmd.cdb[2] = (u8)((CurLba >> 24) & 0xFF);
    Cmd.cdb[3] = (u8)((CurLba >> 16) & 0xFF);
    Cmd.cdb[4] = (u8)((CurLba >>  8) & 0xFF);
    Cmd.cdb[5] = (u8)( CurLba        & 0xFF);
    Cmd.cdb[7] = (u8)((ThisBlkCnt >>  8) & 0xFF);
    Cmd.cdb[8] = (u8)( ThisBlkCnt        & 0xFF);
    Cmd.buf     = CurBuf;
    Cmd.datalen = (UINT32)ThisBytes;
    Cmd.lun     = Dev->Lun;

    if (UfsUtpCmdProcess (Dev->Ufs, &Cmd)) {
      DEBUG ((DEBUG_INFO, "UFS: WriteBlocks LUN%u LBA=%Lu cnt=%Lu FAILED\n",
              Dev->Lun, CurLba, (UINT64)ThisBlkCnt));
      return EFI_DEVICE_ERROR;
    }

    BufferSize -= ThisBytes;
    CurBuf     += ThisBytes;
    CurLba     += (EFI_LBA)ThisBlkCnt;
  }

  return EFI_SUCCESS;
}

STATIC EFI_STATUS EFIAPI
ExynosUfsFlushBlocks (
  IN EFI_BLOCK_IO_PROTOCOL *This)
{
  return EFI_SUCCESS;
}

/* ─── Connect helper ─────────────────────────────────────────────────────────
 * Calls ConnectController only on our installed LUN handles.  Much cheaper
 * than a full system-wide pass and safe to call from callbacks.
 */
STATIC VOID EFIAPI
ExynosUfsConnectLuns (VOID)
{
  UINT32      i;
  EFI_STATUS  Status;

  for (i = 0; i < gLunHandleCount; i++) {
    Status = gBS->ConnectController (gLunHandles[i], NULL, NULL, TRUE);
    DEBUG ((DEBUG_INFO, "UFS: ConnectController LUN[%u]: %r\n", i, Status));
  }
}

/* ─── DiskIo protocol-notify callback ───────────────────────────────────────
 *
 * Fires whenever EFI_DISK_IO_PROTOCOL is installed on ANY handle.
 *
 * The sequence that gets ReadBlocks called is:
 *   BlockIo installed (us) -> DiskIoDxe connects -> DiskIo installed (on our handle)
 *   -> PartitionDxe connects (needs both BlockIo + DiskIo) -> child partition handles
 *   -> FatDxe/Ext2Dxe mount SimpleFileSystem -> BDS sees bootable volumes.
 *
 * PartitionDxe::Supported() checks for BOTH BlockIo and DiskIo on the handle.
 * If we call ConnectController on our LUN handle before DiskIo is present,
 * PartitionDxe rejects it.  This notify fires after DiskIo appears, giving
 * PartitionDxe a second chance to connect and read the partition table.
 */
STATIC VOID EFIAPI
ExynosUfsDiskIoNotify (
  IN EFI_EVENT  Event,
  IN VOID      *Context)
{
  DEBUG ((DEBUG_INFO, "UFS: DiskIo notify - connecting LUN handles\n"));
  ExynosUfsConnectLuns ();
}

/* ─── LUN enumeration helpers ────────────────────────────────────────────────*/

STATIC EFI_STATUS
UfsReadCapacityInternal (
  struct ufs_host *Ufs,
  UINT32           Lun,
  UINT64          *BlkCnt,
  UINT32          *BlkSize)
{
  UFS_SCSI_CMD Cmd;
  UINT8       *Buf;

  /* Must be page-aligned for DMA - stack buffer is not safe on ARM */
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

  *BlkCnt  = (UINT64)(((UINT32)Buf[0] << 24) | ((UINT32)Buf[1] << 16) |
                      ((UINT32)Buf[2] <<  8) |  (UINT32)Buf[3]) + 1;
  *BlkSize = ((UINT32)Buf[4] << 24) | ((UINT32)Buf[5] << 16) |
             ((UINT32)Buf[6] <<  8) |  (UINT32)Buf[7];

  FreeAlignedPages (Buf, 1);
  return EFI_SUCCESS;
}

STATIC EXYNOS_UFS_DEVICE_PATH *
CreateDevicePath (UINT8 Lun)
{
  EXYNOS_UFS_DEVICE_PATH *Dp;

  Dp = AllocateZeroPool (sizeof (*Dp));
  if (!Dp) return NULL;

  Dp->VendorDp.Header.Type    = HARDWARE_DEVICE_PATH;
  Dp->VendorDp.Header.SubType = HW_VENDOR_DP;
  SetDevicePathNodeLength (&Dp->VendorDp.Header, sizeof (VENDOR_DEVICE_PATH) + sizeof (UINT8));
  CopyMem (&Dp->VendorDp.Guid, &gExynosUfsGuid, sizeof (EFI_GUID));
  Dp->Lun = Lun;

  SetDevicePathEndNode (&Dp->End);
  return Dp;
}

/* ─── Per-LUN install ────────────────────────────────────────────────────────*/

STATIC EFI_STATUS
InstallLun (
  IN  EFI_HANDLE    ImageHandle,
  struct ufs_host  *Ufs,
  UINT32            Lun,
  OUT EFI_HANDLE   *InstalledHandle)
{
  EXYNOS_UFS_DEV          *Dev;
  EXYNOS_UFS_DEVICE_PATH  *Dp;
  UINT64                   BlkCnt = 0;
  UINT32                   BlkSize = 4096;
  EFI_STATUS               Status;
  EFI_HANDLE               Handle = NULL;

  *InstalledHandle = NULL;

  /* Read unit descriptor to check if LUN is enabled */
  gQueryParams[DESC_R_UNIT_DESC][3] = (UINT8)Lun;
  if (UfsUtpQueryRetry (Ufs, DESC_R_UNIT_DESC, Lun)) {
    DEBUG ((DEBUG_WARN, "UFS: LUN %d unit desc read failed, skipping\n", Lun));
    return EFI_NOT_FOUND;
  }

  if (!Ufs->unit_desc[Lun].bLUEnable) {
    DEBUG ((DEBUG_INFO, "UFS: LUN %d not enabled, skipping\n", Lun));
    return EFI_NOT_FOUND;
  }

  /* REQUEST SENSE clears Unit Attention (power-on UA) before READ CAPACITY */
  UfsRequestSense (Ufs, Lun);

  /* READ CAPACITY */
  Status = UfsReadCapacityInternal (Ufs, Lun, &BlkCnt, &BlkSize);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_WARN, "UFS: LUN %d read capacity failed\n", Lun));
    BlkCnt  = 0;
    BlkSize = 4096;
  }

  DEBUG ((DEBUG_INFO, "UFS: LUN %d: %Lu blocks of %u bytes\n", Lun, BlkCnt, BlkSize));

  Dev = AllocateZeroPool (sizeof (EXYNOS_UFS_DEV));
  if (!Dev) return EFI_OUT_OF_RESOURCES;

  Dev->Signature = EXYNOS_UFS_SIGNATURE;
  Dev->Ufs       = Ufs;
  Dev->Lun       = Lun;

  /* EFI_BLOCK_IO_MEDIA */
  Dev->Media.MediaId          = 1;
  Dev->Media.RemovableMedia   = FALSE;
  Dev->Media.MediaPresent     = TRUE;
  Dev->Media.LogicalPartition = FALSE;
  Dev->Media.ReadOnly         = FALSE;
  Dev->Media.WriteCaching     = FALSE;
  Dev->Media.BlockSize        = BlkSize;
  Dev->Media.IoAlign          = 0;   /* no alignment requirement */
  Dev->Media.LastBlock        = (BlkCnt > 0) ? (BlkCnt - 1) : 0;

  /* EFI_BLOCK_IO_PROTOCOL - REVISION2 only; we don't implement BlockIo2 */
  Dev->BlockIo.Revision    = EFI_BLOCK_IO_PROTOCOL_REVISION2;
  Dev->BlockIo.Media       = &Dev->Media;
  Dev->BlockIo.Reset       = ExynosUfsReset;
  Dev->BlockIo.ReadBlocks  = ExynosUfsReadBlocks;
  Dev->BlockIo.WriteBlocks = ExynosUfsWriteBlocks;
  Dev->BlockIo.FlushBlocks = ExynosUfsFlushBlocks;

  Dp = CreateDevicePath ((UINT8)Lun);
  if (!Dp) { FreePool (Dev); return EFI_OUT_OF_RESOURCES; }

  Status = gBS->InstallMultipleProtocolInterfaces (
                  &Handle,
                  &gEfiBlockIoProtocolGuid,   &Dev->BlockIo,
                  &gEfiDevicePathProtocolGuid, Dp,
                  NULL);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_INFO, "UFS: InstallProtocol LUN %d failed: %r\n", Lun, Status));
    FreePool (Dev);
    FreePool (Dp);
    return Status;
  }

  *InstalledHandle = Handle;
  return EFI_SUCCESS;
}

/* ─── Driver entry point ─────────────────────────────────────────────────────*/

EFI_STATUS EFIAPI
ExynosUfs9610DxeEntry (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable)
{
  struct ufs_host *Ufs;
  EFI_STATUS       Status;
  UINT32           Lun;
  UINT32           FoundLuns = 0;
  UINT32           LinkRetry;
  EFI_EVENT        Event;
  VOID            *Registration;

  DEBUG ((DEBUG_INFO, "Exynos UFS 9610 Driver starting\n"));

  Ufs = UfsAllocHost ();
  if (!Ufs) {
    DEBUG ((DEBUG_INFO, "UFS: Failed to allocate host\n"));
    return EFI_OUT_OF_RESOURCES;
  }

  Status = UfsInitHost (Ufs);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_INFO, "UFS: Host init failed: %r\n", Status));
    return Status;
  }

  for (LinkRetry = 0; LinkRetry < 5; LinkRetry++) {
    Status = UfsInitInterface (Ufs);
    if (!EFI_ERROR (Status)) break;
    DEBUG ((DEBUG_WARN, "UFS: Link retry %u\n", LinkRetry + 1));
  }

  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_INFO, "UFS: Interface init failed after retries: %r\n", Status));
    return Status;
  }

  if (!UfsUtpQueryRetry (Ufs, ATTR_R_BOOTLUNEN, 0)) {
    DEBUG ((DEBUG_INFO, "UFS: bBootLunEn=0x%x\n",
            Ufs->attributes.arry[UPIU_ATTR_ID_BOOTLUNEN]));
  }

  /* Enumerate LUNs 0-7, save handles for use in callbacks */
  for (Lun = 0; Lun < 8; Lun++) {
    EFI_HANDLE Handle = NULL;
    if (!EFI_ERROR (InstallLun (ImageHandle, Ufs, Lun, &Handle))) {
      gLunHandles[gLunHandleCount++] = Handle;
      FoundLuns++;
    }
  }

  if (FoundLuns == 0) {
    DEBUG ((DEBUG_WARN, "UFS: No LUNs found\n"));
    return EFI_NOT_FOUND;
  }

  DEBUG ((DEBUG_INFO, "UFS: Installed %u LUN(s) as BlockIO\n", FoundLuns));

  /*
   * Attempt an immediate connect pass.  If DiskIoDxe and PartitionDxe were
   * dispatched before us, this succeeds and we're done.  If not, the two
   * event registrations below handle it.
   */
  ExynosUfsConnectLuns ();

  /*
   * Register a protocol-install notification on EFI_DISK_IO_PROTOCOL.
   *
   * The connect chain that makes ReadBlocks get called is:
   *   1. Our BlockIo handle is installed.
   *   2. DiskIoDxe::Start() sees BlockIo -> installs DiskIo on our handle.
   *   3. PartitionDxe::Supported() sees BlockIo + DiskIo -> connects, reads MBR/GPT.
   *   4. PartitionDxe::Start() calls ReadBlocks to probe the partition table.
   *   5. Child handles with LogicalPartition=TRUE and BlockIo are installed.
   *   6. FatDxe/filesystem drivers mount SimpleFileSystem on those children.
   *   7. BDS enumerates SimpleFileSystem handles as boot candidates.
   *
   * If DiskIoDxe dispatches AFTER us, step 2 fires later and PartitionDxe
   * was never retried on our handle.  This notify kicks ConnectController
   * again the moment DiskIo appears on any handle, giving PartitionDxe the
   * second chance it needs.
   */
  Status = gBS->CreateEvent (
                  EVT_NOTIFY_SIGNAL,
                  TPL_CALLBACK,
                  ExynosUfsDiskIoNotify,
                  NULL,
                  &Event);
  if (!EFI_ERROR (Status)) {
    gBS->RegisterProtocolNotify (
           &gEfiDiskIoProtocolGuid,
           Event,
           &Registration);
    DEBUG ((DEBUG_INFO, "UFS: DiskIo protocol notify registered\n"));
  } else {
    DEBUG ((DEBUG_WARN, "UFS: Failed to register DiskIo notify: %r\n", Status));
  }

  /*
   * Register a ReadyToBoot event as the final guarantee.
   *
   * EVT_SIGNAL_READY_TO_BOOT fires after ALL DXE drivers are dispatched and
   * before BDS attempts any boot.  Even if the platform's PlatformBootManagerLib
   * skips EfiBootManagerConnectAll() (which many downstream ports do), this
   * ensures PartitionDxe has connected and produced child handles before BDS
   * examines the system for boot candidates.
   */
/*  Status = gBS->CreateEventEx (
                  EVT_NOTIFY_SIGNAL,
                  TPL_CALLBACK,
                  ExynosUfsReadyToBoot,
                  NULL,
                  &gEfiEventReadyToBootGuid,
                  &Event);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_WARN, "UFS: Failed to create ReadyToBoot event: %r\n", Status));
  } else {
    DEBUG ((DEBUG_INFO, "UFS: ReadyToBoot event registered\n"));
  }*/

  return EFI_SUCCESS;
}
