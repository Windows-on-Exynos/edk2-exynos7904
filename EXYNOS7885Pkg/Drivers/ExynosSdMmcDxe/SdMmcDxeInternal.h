#ifndef __SD_MMC_DXE_INTERNAL_H__
#define __SD_MMC_DXE_INTERNAL_H__

#include <Uefi.h>
#include <Library/DwMmcRegs.h>

// IDMAC descriptor table — 256 entries × 64 bytes = 16KB, cache-aligned
STATIC DWMMC_IDMAC_DESC  mDmaDesc[256] __attribute__((aligned(64)));

// EXT_CSD buffer for eMMC (512 bytes)
STATIC UINT8             mExtCsdBuf[512]     __attribute__((aligned(64)));

// Tuning data buffer (64 bytes for CMD21/CMD19)
STATIC UINT8             mTuningBuf[64]       __attribute__((aligned(64)));

// Current DMA transfer state
STATIC UINT32            mDmaBlockSize;
STATIC UINT32            mDmaBlockCnt;
STATIC BOOLEAN           mDmaIsRead;
STATIC VOID             *mDmaBuffer;

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

#endif // __SD_MMC_DXE_INTERNAL_H__