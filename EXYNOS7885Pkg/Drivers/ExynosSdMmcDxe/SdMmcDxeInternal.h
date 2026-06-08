#ifndef __SD_MMC_DXE_INTERNAL_H__
#define __SD_MMC_DXE_INTERNAL_H__

#include <Uefi.h>
#include <Protocol/BlockIo.h>
#include <Library/DwMmcRegs.h>

// IDMAC descriptor table — 256 entries × 64 bytes = 16KB, cache-aligned
STATIC DWMMC_IDMAC_DESC  mDmaDesc[256] __attribute__((aligned(64)));

// EXT_CSD buffer for eMMC (512 bytes)
STATIC UINT8             mExtCsdBuf[512]     __attribute__((aligned(64)));

// Tuning data buffer (128 bytes for CMD21 eMMC 8-bit tuning, 64 for SD CMD19)
STATIC UINT8             mTuningBuf[128]       __attribute__((aligned(64)));

// Current DMA transfer state
STATIC UINT32            mDmaBlockSize;
STATIC UINT32            mDmaBlockCnt;
STATIC BOOLEAN           mDmaIsRead;
STATIC VOID             *mDmaBuffer;

// eMMC partition state (active partition access)
STATIC UINT8             mActivePartition = EXT_CSD_PART_ACCESS_USER;

// Boot/RPMB partition sizes (in sectors)
STATIC UINT32            mBoot1Blocks;
STATIC UINT32            mBoot2Blocks;
STATIC UINT32            mRpmbBlocks;
STATIC UINT32            mGp1Blocks;
STATIC UINT32            mGp2Blocks;
STATIC UINT32            mGp3Blocks;
STATIC UINT32            mGp4Blocks;

// Multiple BlockIo instances for partitions
STATIC VOID             *mPartitionDevs[MMC_NUM_PARTITIONS];

// Erase Block Protocol types
typedef struct {
  EFI_EVENT   Event;
  EFI_STATUS  TransactionStatus;
} EFI_ERASE_BLOCK_TOKEN;

typedef
EFI_STATUS
(EFIAPI *EFI_BLOCK_ERASE)(
  IN EFI_BLOCK_IO_PROTOCOL  *This,
  IN UINT32                  MediaId,
  IN EFI_LBA                 LBA,
  IN OUT EFI_ERASE_BLOCK_TOKEN *Token,
  IN UINTN                   Size
  );

typedef struct {
  UINT64              Revision;
  UINT32              EraseLengthGranularity;
  EFI_BLOCK_ERASE     EraseBlocks;
} EFI_ERASE_BLOCK_PROTOCOL;

#define EFI_ERASE_BLOCK_PROTOCOL_REVISION  ((2 << 16) | (60))

// Forward declarations for static helper functions
STATIC EFI_STATUS
DwMmcSendCommand (
  UINT32   CmdIdx,
  UINT32   Arg,
  UINT32   RespType,
  UINT32  *OutResp
  );

STATIC EFI_STATUS
DwMmcSendCommandData (
  UINT32   CmdIdx,
  UINT32   Arg,
  UINT32   RespType,
  UINT32  *OutResp,
  BOOLEAN  IsWrite,
  UINT32   BlockSize,
  UINT32   BlockCnt,
  VOID    *Buf
  );

STATIC EFI_STATUS
DwMmcChangeClock (UINT32 TargetClk);

STATIC EFI_STATUS
DwMmcControlClken (UINT32 Val);

STATIC EFI_STATUS
DwMmcSwitchCmd (UINT8 Set, UINT8 Index, UINT8 Value);

STATIC EFI_STATUS
DwMmcGetCardStatus (UINT32 TimeoutMs, UINT32 *OutState);

STATIC EFI_STATUS
DwMmcSetBusWidth (UINT32 BusWidth);

STATIC EFI_STATUS
DwMmcSetTimingAndBus (UINT32 Timing, UINT32 Clock, UINT32 BusWidth, UINT32 BusMode, UINT32 ClkselVal);

STATIC EFI_STATUS
DwMmcSetIos (UINT32 Clock, UINT32 ClkselVal);

STATIC EFI_STATUS
DwMmcSetIosSimple (UINT32 Clock);

STATIC EFI_STATUS
DwMmcExecuteTuning (UINT32 CmdIdx);

STATIC EFI_STATUS
DwMmcSelectPartition (
  UINT8 Partition
  );

STATIC EFI_STATUS
DwMmcErase (
  EFI_LBA Lba,
  UINTN   BlockCount,
  BOOLEAN SecurePurge
  );

STATIC EFI_STATUS EFIAPI
EraseBlocks (
  IN EFI_BLOCK_IO_PROTOCOL  *This,
  IN UINT32                  MediaId,
  IN EFI_LBA                 LBA,
  IN OUT EFI_ERASE_BLOCK_TOKEN *Token,
  IN UINTN                   Size
  );

// eMMC specific extension functions
STATIC EFI_STATUS
DwMmcSetCacheCtrl (
  BOOLEAN Enable
  );

STATIC EFI_STATUS
DwMmcSanitize (VOID);

STATIC EFI_STATUS
DwMmcTrim (
  EFI_LBA Lba,
  UINTN   BlockCount,
  BOOLEAN Secure
  );

STATIC EFI_STATUS
DwMmcDiscard (
  EFI_LBA Lba,
  UINTN   BlockCount
  );

STATIC EFI_STATUS
DwMmcPowerOffNotification (
  BOOLEAN PowerOffLong
  );

STATIC EFI_STATUS
DwMmcSetBkops (
  BOOLEAN Enable
  );

STATIC EFI_STATUS
DwMmcSetBootWp (
  UINT8 WpConfig
  );

STATIC EFI_STATUS
DwMmcSetHpi (VOID);

STATIC EFI_STATUS
DwMmcPowerOnNotification (VOID);

STATIC EFI_STATUS
DwMmcFlushCache (VOID);

STATIC VOID
DwMmcChangeClksel (
  UINT32 PassIndex
  );

STATIC EFI_STATUS
DwMmcSendExtCsd (UINT8 *Buf);

#endif // __SD_MMC_DXE_INTERNAL_H__