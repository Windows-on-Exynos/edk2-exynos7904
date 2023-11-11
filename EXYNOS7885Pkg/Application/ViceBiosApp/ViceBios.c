#include <Uefi.h>
#include <Library/UefiLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/DebugLib.h>  // Incluye DebugLib
#include <Protocol/GraphicsOutput.h>

EFI_STATUS EFIAPI UefiMain(EFI_HANDLE ImageHandle, EFI_SYSTEM_TABLE *SystemTable)
{
	EFI_STATUS Status;

    // Muestra un mensaje en la pantalla usando DebugLib
    DEBUG((EFI_D_INFO, "¡Hello from the ViceBios app on Debug mode!\n"));

    Print(L"Hello, world!\r\n");
    Print(L"I am ViceBios\r\n");

    // Configura el modo gráfico (por ejemplo, resolución de 800x600)
    Status = Gop->SetMode(Gop, 0); // Modo 1 suele ser 800x600
    if (EFI_ERROR(Status))
    {
        Print(L"Failed to setup Graphics Mode: %r\n", Status);
        return Status;
    }

    // Dibuja un rectángulo rojo en la pantalla
    EFI_GRAPHICS_OUTPUT_BLT_PIXEL Color = {255, 0, 0, 0}; // Rojo (RGBA)
    UINTN RectWidth = 200; // Ancho del rectángulo
    UINTN RectHeight = 100; // Alto del rectángulo
    UINTN XPos = (1080 - RectWidth) / 2; // Centrado horizontalmente
    UINTN YPos = (2340 - RectHeight) / 2; // Centrado verticalmente

    for (UINTN y = YPos; y < YPos + RectHeight; y++)
    {
        for (UINTN x = XPos; x < XPos + RectWidth; x++)
        {
            Gop->Blt(Gop, &Color, EfiBltVideoFill, 0, 0, x, y, RectWidth, RectHeight, 0);
        }
    }

    return EFI_SUCCESS;
}
