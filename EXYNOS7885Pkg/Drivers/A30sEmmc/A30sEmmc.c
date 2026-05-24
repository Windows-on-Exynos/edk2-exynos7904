#include <Uefi.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiDriverEntryPoint.h>
#include <Library/DebugLib.h>
#include <Library/BaseLib.h>
#include <Library/PcdLib.h>
#include <Library/IoLib.h>
#include <Library/CacheMaintenanceLib.h>
#include <Library/TimerLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Protocol/BlockIo.h>

#define EMMC_BASE       PcdGet64(PcdEmmcBaseAddress)
#define EMMC_CLOCK      PcdGet32(PcdEmmcClockFrequency)
#define EMMC_BUS        PcdGet32(PcdEmmcBusWidth)
#define EMMC_FIFO_DEPTH PcdGet32(PcdEmmcFifoDepth)

typedef struct {
    EFI_BLOCK_IO_PROTOCOL BlockIo;
    EFI_HANDLE Handle;
    EFI_BLOCK_IO_MEDIA Media;
} EMMC_DEVICE;

STATIC EMMC_DEVICE mEmmcDevice;

// Offsets DW-MSHC (Exynos)
#define DW_MSHC_CTRL        0x00
#define DW_MSHC_PWREN       0x04
#define DW_MSHC_CLKDIV      0x08
#define DW_MSHC_CLKSRC      0x0C
#define DW_MSHC_CLKENA      0x10
#define DW_MSHC_TMOUT       0x14
#define DW_MSHC_CTYPE       0x18
#define DW_MSHC_BLKSIZ      0x1C
#define DW_MSHC_BYTCNT      0x20
#define DW_MSHC_CMDARG      0x24
#define DW_MSHC_CMD         0x28
#define DW_MSHC_RESP0       0x2C
#define DW_MSHC_RESP1       0x30
#define DW_MSHC_RESP2       0x34
#define DW_MSHC_RESP3       0x38
#define DW_MSHC_DATA        0x200 // FIFO

STATIC VOID DwMshcHwInit(VOID)
{
    // Reset control
    MmioWrite32(EMMC_BASE + DW_MSHC_CTRL, 0x01);
    MicroSecondDelay(100);

    // Clock: setup default
    MmioWrite32(EMMC_BASE + DW_MSHC_CLKDIV, 0);
    MmioWrite32(EMMC_BASE + DW_MSHC_CLKSRC, 0);
    MmioWrite32(EMMC_BASE + DW_MSHC_CLKENA, 1);

    // Bus width 8-bit
    MmioWrite32(EMMC_BASE + DW_MSHC_CTYPE, (EMMC_BUS == 8) ? 0x02 : 0x00);

    // Timeout
    MmioWrite32(EMMC_BASE + DW_MSHC_TMOUT, 0xFFFFFFFF);

    DEBUG((EFI_D_INFO, "DW-MSHC hardware initialized\n"));
}

STATIC EFI_STATUS DwMshcCardInit(VOID)
{
    // Secuencia básica CMD0 -> CMD1 -> CMD2 -> CMD3 -> CMD7 -> CMD16
    // Placeholder: requiere implementación completa basada en el estándar eMMC
    DEBUG((EFI_D_INFO, "DW-MSHC card init placeholder\n"));
    return EFI_SUCCESS;
}

// Block IO protocol functions
STATIC EFI_STATUS EFIAPI A30sEmmcReset(IN EFI_BLOCK_IO_PROTOCOL *This, IN BOOLEAN ExtendedVerification)
{
    DwMshcHwInit();
    DwMshcCardInit();
    return EFI_SUCCESS;
}

STATIC EFI_STATUS EFIAPI A30sEmmcReadBlocks(IN EFI_BLOCK_IO_PROTOCOL *This,
                                            IN UINT32 MediaId,
                                            IN EFI_LBA Lba,
                                            IN UINTN BufferSize,
                                            OUT VOID *Buffer)
{
    // Placeholder: copia dummy
    SetMem(Buffer, BufferSize, 0xFF);
    DEBUG((EFI_D_INFO, "DW-MSHC ReadBlocks LBA=%lu, Size=%lu\n", Lba, BufferSize));
    return EFI_SUCCESS;
}

STATIC EFI_STATUS EFIAPI A30sEmmcWriteBlocks(IN EFI_BLOCK_IO_PROTOCOL *This,
                                             IN UINT32 MediaId,
                                             IN EFI_LBA Lba,
                                             IN UINTN BufferSize,
                                             IN VOID *Buffer)
{
    // Placeholder: no escribe realmente
    DEBUG((EFI_D_INFO, "DW-MSHC WriteBlocks LBA=%lu, Size=%lu\n", Lba, BufferSize));
    return EFI_SUCCESS;
}

STATIC EFI_STATUS EFIAPI A30sEmmcFlushBlocks(IN EFI_BLOCK_IO_PROTOCOL *This)
{
    return EFI_SUCCESS;
}

EFI_STATUS EFIAPI A30sEmmcEntryPoint(IN EFI_HANDLE ImageHandle, IN EFI_SYSTEM_TABLE *SystemTable)
{
    EFI_STATUS Status;

    DEBUG((EFI_D_INFO, "A30s DW-MSHC eMMC Driver Loaded\n"));

    ZeroMem(&mEmmcDevice, sizeof(mEmmcDevice));

    // Media info
    mEmmcDevice.Media.MediaId = 0;
    mEmmcDevice.Media.BlockSize = 512;
    mEmmcDevice.Media.LastBlock = (1024 * 1024) - 1; // ejemplo 512MB
    mEmmcDevice.Media.ReadOnly = FALSE;
    mEmmcDevice.Media.MediaPresent = TRUE;
    mEmmcDevice.Media.LogicalPartition = FALSE;
    mEmmcDevice.Media.RemovableMedia = FALSE;
    mEmmcDevice.Media.WriteCaching = FALSE;

    // Protocol
    mEmmcDevice.BlockIo.Revision = EFI_BLOCK_IO_PROTOCOL_REVISION;
    mEmmcDevice.BlockIo.Media = &mEmmcDevice.Media;
    mEmmcDevice.BlockIo.Reset = A30sEmmcReset;
    mEmmcDevice.BlockIo.ReadBlocks = A30sEmmcReadBlocks;
    mEmmcDevice.BlockIo.WriteBlocks = A30sEmmcWriteBlocks;
    mEmmcDevice.BlockIo.FlushBlocks = A30sEmmcFlushBlocks;

    Status = gBS->InstallProtocolInterface(
        &mEmmcDevice.Handle,
        &gEfiBlockIoProtocolGuid,
        EFI_NATIVE_INTERFACE,
        &mEmmcDevice.BlockIo
    );

    if (EFI_ERROR(Status)) {
        DEBUG((EFI_D_ERROR, "Failed to install Block IO protocol: %r\n", Status));
        return Status;
    }

    DEBUG((EFI_D_INFO, "DW-MSHC Block IO protocol installed!\n"));
    return EFI_SUCCESS;
}
