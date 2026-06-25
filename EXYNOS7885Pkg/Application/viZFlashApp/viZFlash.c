#include <Uefi.h>

#include <Library/UefiLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>

#include <Protocol/UsbFunctionIo.h>
#include <Protocol/UsbDevice.h>

#define VIZFLASH_USB_WAIT_STEP_MS   200
#define VIZFLASH_USB_WAIT_TOTAL_MS  10000

STATIC
VOID
viZFlashWaitForKey (
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  UINTN Index;

  Print(L"\r\nPress any key to exit...\r\n");
  SystemTable->ConIn->Reset(SystemTable->ConIn, FALSE);
  gBS->WaitForEvent(1, &SystemTable->ConIn->WaitForKey, &Index);
}

STATIC
EFI_STATUS
WaitForProtocolHandles (
  IN EFI_GUID  *ProtocolGuid,
  IN CHAR16    *ProtocolName,
  IN UINTN      TimeoutMs
  )
{
  EFI_STATUS  Status;
  EFI_HANDLE *HandleBuffer;
  UINTN       HandleCount;
  UINTN       Elapsed;

  Print(L"[WAIT] Waiting for %s (timeout %u ms)...\r\n",
        ProtocolName,
        (UINT32)TimeoutMs);

  for (Elapsed = 0; Elapsed < TimeoutMs; Elapsed += VIZFLASH_USB_WAIT_STEP_MS) {
    HandleBuffer = NULL;
    HandleCount  = 0;

    Status = gBS->LocateHandleBuffer(
                    ByProtocol,
                    ProtocolGuid,
                    NULL,
                    &HandleCount,
                    &HandleBuffer
                    );

    if (!EFI_ERROR(Status) && HandleCount > 0) {
      Print(L"  -> %s became available after %u ms (%u handle(s))\r\n",
            ProtocolName,
            (UINT32)Elapsed,
            (UINT32)HandleCount);

      FreePool(HandleBuffer);
      return EFI_SUCCESS;
    }

    if (HandleBuffer != NULL) {
      FreePool(HandleBuffer);
    }

    gBS->Stall(VIZFLASH_USB_WAIT_STEP_MS * 1000);
  }

  Print(L"  -> Timeout waiting for %s\r\n", ProtocolName);
  return EFI_NOT_FOUND;
}

STATIC
VOID
DumpUsbFnHandles (
  VOID
  )
{
  EFI_STATUS             Status;
  EFI_HANDLE            *HandleBuffer;
  UINTN                  HandleCount;
  UINTN                  Index;
  EFI_USBFN_IO_PROTOCOL *UsbFn;

  HandleBuffer = NULL;
  HandleCount  = 0;

  Print(L"\r\n[USBFN] Searching handles for EFI_USBFN_IO_PROTOCOL...\r\n");

  Status = gBS->LocateHandleBuffer(
                  ByProtocol,
                  &gEfiUsbFunctionIoProtocolGuid,
                  NULL,
                  &HandleCount,
                  &HandleBuffer
                  );

  if (EFI_ERROR(Status)) {
    Print(L"  -> No USBFN handles found: %r\r\n", Status);
    return;
  }

  Print(L"  -> Found %u USBFN handle(s)\r\n", (UINT32)HandleCount);

  for (Index = 0; Index < HandleCount; Index++) {
    UsbFn = NULL;

    Print(L"     Handle[%u] = %p\r\n", (UINT32)Index, HandleBuffer[Index]);

    Status = gBS->HandleProtocol(
                    HandleBuffer[Index],
                    &gEfiUsbFunctionIoProtocolGuid,
                    (VOID **)&UsbFn
                    );
    if (EFI_ERROR(Status)) {
      Print(L"       HandleProtocol failed: %r\r\n", Status);
    } else {
      Print(L"       Protocol interface = %p\r\n", UsbFn);
    }
  }

  FreePool(HandleBuffer);
}

STATIC
VOID
DumpUsbDeviceHandles (
  VOID
  )
{
  EFI_STATUS           Status;
  EFI_HANDLE          *HandleBuffer;
  UINTN                HandleCount;
  UINTN                Index;
  USB_DEVICE_PROTOCOL *UsbDevice;

  HandleBuffer = NULL;
  HandleCount  = 0;

  Print(L"\r\n[USBDEV] Searching handles for USB_DEVICE_PROTOCOL...\r\n");

  Status = gBS->LocateHandleBuffer(
                  ByProtocol,
                  &gUsbDeviceProtocolGuid,
                  NULL,
                  &HandleCount,
                  &HandleBuffer
                  );

  if (EFI_ERROR(Status)) {
    Print(L"  -> No USB_DEVICE handles found: %r\r\n", Status);
    return;
  }

  Print(L"  -> Found %u USB_DEVICE handle(s)\r\n", (UINT32)HandleCount);

  for (Index = 0; Index < HandleCount; Index++) {
    UsbDevice = NULL;

    Print(L"     Handle[%u] = %p\r\n", (UINT32)Index, HandleBuffer[Index]);

    Status = gBS->HandleProtocol(
                    HandleBuffer[Index],
                    &gUsbDeviceProtocolGuid,
                    (VOID **)&UsbDevice
                    );
    if (EFI_ERROR(Status)) {
      Print(L"       HandleProtocol failed: %r\r\n", Status);
    } else {
      Print(L"       Protocol interface = %p\r\n", UsbDevice);
    }
  }

  FreePool(HandleBuffer);
}

EFI_STATUS
EFIAPI
viZFlashMain (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS UsbFnWaitStatus;
  EFI_STATUS UsbDevWaitStatus;

  Print(L"viZFlashApp v1.0\r\n\r\n");

  UsbFnWaitStatus = WaitForProtocolHandles(
                      &gEfiUsbFunctionIoProtocolGuid,
                      L"EFI_USBFN_IO_PROTOCOL",
                      VIZFLASH_USB_WAIT_TOTAL_MS
                      );

  UsbDevWaitStatus = WaitForProtocolHandles(
                       &gUsbDeviceProtocolGuid,
                       L"USB_DEVICE_PROTOCOL",
                       VIZFLASH_USB_WAIT_TOTAL_MS
                       );

  Print(L"\r\n[SUMMARY]\r\n");
  Print(L"  EFI_USBFN_IO_PROTOCOL : %r\r\n", UsbFnWaitStatus);
  Print(L"  USB_DEVICE_PROTOCOL   : %r\r\n", UsbDevWaitStatus);

  DumpUsbFnHandles();
  DumpUsbDeviceHandles();

  viZFlashWaitForKey(SystemTable);
  return EFI_SUCCESS;
}