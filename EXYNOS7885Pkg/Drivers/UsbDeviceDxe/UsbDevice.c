#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiDriverEntryPoint.h>
#include <Protocol/UsbFunctionIo.h>
#include <Protocol/UsbDevice.h>

//
// Global state
//
STATIC EFI_USBFN_IO_PROTOCOL      *mUsbfnIo   = NULL;
STATIC USB_DEVICE_RX_CALLBACK       mRxCallback = NULL;
STATIC USB_DEVICE_TX_CALLBACK       mTxCallback = NULL;
STATIC BOOLEAN                      mStarted    = FALSE;

/**
  Send data to the host via bulk IN endpoint.
**/
STATIC
EFI_STATUS
UsbDevSend (
  IN       UINT8  EndpointIndex,
  IN       UINTN  Size,
  IN CONST VOID  *Buffer
  )
{
  EFI_STATUS  Status;
  UINTN       BufSize;

  if ((mUsbfnIo == NULL) || (Buffer == NULL) || (Size == 0)) {
    return EFI_INVALID_PARAMETER;
  }

  BufSize = Size;
  Status  = mUsbfnIo->Transfer (
                        mUsbfnIo,
                        EndpointIndex,
                        EfiUsbEndpointDirectionDeviceTx,
                        &BufSize,
                        (VOID *)Buffer
                        );
  return Status;
}

/**
  Start the USB device and handle enumeration.

  This function blocks until the device is enumerated (SET_CONFIGURATION)
  or an error occurs.
**/
STATIC
EFI_STATUS
UsbDevStart (
  IN USB_DEVICE_DESCRIPTOR   *DeviceDescriptor,
  IN VOID                    **Descriptors,
  IN USB_DEVICE_RX_CALLBACK   RxCallback,
  IN USB_DEVICE_TX_CALLBACK   TxCallback
  )
{
  EFI_STATUS               Status;
  EFI_USBFN_MESSAGE         Message;
  UINTN                     PayloadSize;
  EFI_USBFN_MESSAGE_PAYLOAD Payload;
  EFI_USB_DEVICE_INFO       DevInfo;
  EFI_USB_CONFIG_INFO      *CfgInfo;

  if (mUsbfnIo == NULL) {
    return EFI_NOT_READY;
  }

  //
  // Store callbacks
  //
  mRxCallback = RxCallback;
  mTxCallback = TxCallback;

  //
  // Start the DWC3 controller
  //
  if (!mStarted) {
    Status = mUsbfnIo->StartController (mUsbfnIo);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "UsbDev: StartController failed (%r)\n", Status));
      return Status;
    }
  }

  //
  // Configure endpoints
  //
  CfgInfo = AllocateZeroPool (sizeof (EFI_USB_CONFIG_INFO));
  if (CfgInfo == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  CfgInfo->ConfigDescriptor = AllocateZeroPool (sizeof (EFI_USB_CONFIG_DESCRIPTOR));
  if (CfgInfo->ConfigDescriptor == NULL) {
    FreePool (CfgInfo);
    return EFI_OUT_OF_RESOURCES;
  }

  //
  // Build minimal config info from the descriptors passed by the fastboot transport
  //
  CopyMem (CfgInfo->ConfigDescriptor, Descriptors[0], sizeof (EFI_USB_CONFIG_DESCRIPTOR));

  CfgInfo->InterfaceInfoTable = AllocateZeroPool (sizeof (EFI_USB_INTERFACE_INFO *));
  if (CfgInfo->InterfaceInfoTable == NULL) {
    FreePool (CfgInfo->ConfigDescriptor);
    FreePool (CfgInfo);
    return EFI_OUT_OF_RESOURCES;
  }

  CfgInfo->InterfaceInfoTable[0] = AllocateZeroPool (sizeof (EFI_USB_INTERFACE_INFO));
  if (CfgInfo->InterfaceInfoTable[0] == NULL) {
    FreePool (CfgInfo->InterfaceInfoTable);
    FreePool (CfgInfo->ConfigDescriptor);
    FreePool (CfgInfo);
    return EFI_OUT_OF_RESOURCES;
  }

  DevInfo.DeviceDescriptor  = (EFI_USB_DEVICE_DESCRIPTOR *)DeviceDescriptor;
  DevInfo.ConfigInfoTable   = &CfgInfo;

  Status = mUsbfnIo->ConfigureEnableEndpoints (mUsbfnIo, &DevInfo);

  //
  // Free temporary allocations
  //
  if (CfgInfo->InterfaceInfoTable[0] != NULL) FreePool (CfgInfo->InterfaceInfoTable[0]);
  FreePool (CfgInfo->InterfaceInfoTable);
  FreePool (CfgInfo->ConfigDescriptor);
  FreePool (CfgInfo);

  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "UsbDev: ConfigureEnableEndpoints failed (%r)\n", Status));
    return Status;
  }

  mStarted = TRUE;
  DEBUG ((EFI_D_WARN, "UsbDev: Device started, waiting for USB cable...\n"));

  //
  // Poll for enumeration events
  //
  while (TRUE) {
    PayloadSize = sizeof (Payload);
    Status = mUsbfnIo->EventHandler (mUsbfnIo, &Message, &PayloadSize, &Payload);
    if (EFI_ERROR (Status)) {
      continue;
    }

    switch (Message) {
    case EfiUsbMsgSetupPacket:
      //
      // Standard setup packets (enumeration) are already handled by
      // the DWC3 driver internally. The class driver setup packets
      // will be forwarded.
      //
      break;

    case EfiUsbMsgEndpointStatusChangedRx:
      if (mRxCallback != NULL) {
        mRxCallback (
          Payload.utr.BytesTransferred,
          Payload.utr.Buffer
          );
        //
        // Re-arm bulk OUT transfer
        //
        {
          UINTN  RxSize = 512;
          VOID  *RxBuf  = AllocatePool (RxSize);
          if (RxBuf != NULL) {
            mUsbfnIo->Transfer (
                        mUsbfnIo,
                        Payload.utr.EndpointIndex,
                        Payload.utr.Direction,
                        &RxSize,
                        RxBuf
                        );
          }
        }
      }
      break;

    case EfiUsbMsgEndpointStatusChangedTx:
      if (mTxCallback != NULL) {
        mTxCallback ((UINT8)Payload.utr.EndpointIndex);
      }
      break;

    case EfiUsbMsgBusEventReset:
      break;

    case EfiUsbMsgBusEventAttach:
      DEBUG ((EFI_D_WARN, "UsbDev: Cable attached!\n"));
      break;

    case EfiUsbMsgBusEventDetach:
      DEBUG ((EFI_D_WARN, "UsbDev: Cable detached\n"));
      break;

    default:
      break;
    }
  }

  return EFI_SUCCESS;
}

//
// Protocol instance
//
STATIC USB_DEVICE_PROTOCOL  mUsbDeviceProtocol = {
  UsbDevStart,
  UsbDevSend,
};

/**
  Entry point: Locate EFI_USBFN_IO_PROTOCOL and install USB_DEVICE_PROTOCOL.
**/
EFI_STATUS
EFIAPI
InitializeUsbDevice (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS  Status;

  DEBUG ((EFI_D_WARN, "UsbDeviceDxe: Entry\n"));

  //
  // Locate EFI_USBFN_IO_PROTOCOL
  //
  Status = gBS->LocateProtocol (
                  &gEfiUsbFunctionIoProtocolGuid,
                  NULL,
                  (VOID **)&mUsbfnIo
                  );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "UsbDeviceDxe: USBFN protocol not found (%r)\n", Status));
    return Status;
  }

  DEBUG ((EFI_D_WARN, "UsbDeviceDxe: Found USBFN protocol at 0x%p\n", mUsbfnIo));

  //
  // Install USB_DEVICE_PROTOCOL
  //
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &ImageHandle,
                  &gUsbDeviceProtocolGuid,
                  &mUsbDeviceProtocol,
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "UsbDeviceDxe: InstallProtocol failed (%r)\n", Status));
    return Status;
  }

  DEBUG ((EFI_D_WARN, "UsbDeviceDxe: Installed USB_DEVICE_PROTOCOL\n"));
  return EFI_SUCCESS;
}
