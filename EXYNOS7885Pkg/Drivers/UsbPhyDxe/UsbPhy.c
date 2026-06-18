#include <Library/DebugLib.h>
#include <Library/MemoryAllocationHelperLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiDriverEntryPoint.h>
#include <Library/UsbPhyLib.h>

EFI_STATUS
EFIAPI
InitializeUsbPhy (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS        Status;
  USB_PHY_CONFIG   **Configs;
  UINT8              Count;
  UINT8              i;

  DEBUG ((DEBUG_WARN, "UsbPhyDxe: Entry\n"));

  //
  // Get PHY configurations from SoC library
  //
  Status = GetUsbPhyConfigs (&Configs, &Count);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_WARN, "UsbPhy: No PHY instances found (%r)\n", Status));
    return Status;
  }

  DEBUG ((DEBUG_WARN, "UsbPhy: Found %d PHY instance(s)\n", Count));

  for (i = 0; i < Count; i++) {
    DEBUG ((DEBUG_WARN, "UsbPhy: Mapping PHY %d base=0x%llx\n",
            i, Configs[i]->RegBase));

    Status = MapMemoryRegion (Configs[i]->RegBase, 0x1000, EfiMemoryMappedIO);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "UsbPhy: Failed to map PHY %d base (%r)\n",
              i, Status));
      return Status;
    }

    if (Configs[i]->RegBase2nd != 0) {
      DEBUG ((DEBUG_WARN, "UsbPhy: Mapping PHY %d 2nd base=0x%llx\n",
              i, Configs[i]->RegBase2nd));

      Status = MapMemoryRegion (Configs[i]->RegBase2nd, 0x1000, EfiMemoryMappedIO);
      if (EFI_ERROR (Status)) {
        DEBUG ((DEBUG_ERROR, "UsbPhy: Failed to map PHY %d 2nd base (%r)\n",
                i, Status));
        return Status;
      }
    }
  }

  //
  // Initialize each PHY
  //
  for (i = 0; i < Count; i++) {
    DEBUG ((DEBUG_WARN, "UsbPhy: Initializing PHY %d (base=0x%llx, ver=0x%x)\n",
            i, Configs[i]->RegBase, Configs[i]->Version));

    Status = UsbPhyInit (Configs[i]);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "UsbPhy: PHY %d init failed (%r)\n", i, Status));
      //
      // Continue with remaining PHYs even if one fails
      //
    }
  }

  DEBUG ((DEBUG_WARN, "UsbPhy: Exit\n"));
  return EFI_SUCCESS;
}
