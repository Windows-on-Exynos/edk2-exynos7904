#include <Uefi.h>
#include <Library/UefiLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/DebugLib.h>  // Incluye DebugLib

EFI_STATUS EFIAPI BiosEntry(EFI_HANDLE ImageHandle, EFI_SYSTEM_TABLE *SystemTable)
{
    // Muestra un mensaje en la pantalla usando DebugLib
    DEBUG((EFI_D_INFO, "¡Hello from the UEFI app on Debug mode!\n"));

    Print(L"Hello, world!\r\n");

    return EFI_SUCCESS;
}
