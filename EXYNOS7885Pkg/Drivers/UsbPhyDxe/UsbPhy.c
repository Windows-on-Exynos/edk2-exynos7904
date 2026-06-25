#include <Library/DebugLib.h>
#include <Library/MemoryAllocationHelperLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiDriverEntryPoint.h>
#include <Library/UsbPhyLib.h>

#define USB_REG_GSBUSCFG0	0xC100
#define USB_REG_GSBUSCFG1	0xC104
#define USB_REG_GCTL		0xC110
#define USB_REG_GUSB2PHYCFG	0xC200
#define USB_REG_GUSB3PIPECTL	0xC2C0
#define USB_REG_USB2PHYCFG_MASK	0xFFFFC000
#define USB_REG_USB2PHYCFG_KEEP	0x000002BF
#define USB_REG_USB2PHYCFG_BL	0x00002400
#define USB_REG_GUSB3_SUSPEND	(1 << 17)
#define USB_REG_GCTL_KEEP_HIGH	0x0007C000
#define USB_REG_GCTL_KEEP_LOW	0x00000F3F
#define USB_REG_GCTL_DEVICE	(2 << 12)
#define USB_REG_GCTL_U2RST_ECN	(1 << 16)
#define USB_REG_GCTL_MASTER_FILT_BYPASS	(1 << 18)
#define USB_PHY_PMU_ENABLE	0x3

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

  DEBUG ((EFI_D_INFO, "UsbPhyDxe: Entry\n"));

  //
  // Get PHY configurations from SoC library
  //
  Status = GetUsbPhyConfigs (&Configs, &Count);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "UsbPhy: No PHY instances found (%r)\n", Status));
    return Status;
  }

  DEBUG ((EFI_D_INFO, "UsbPhy: Found %d PHY instance(s)\n", Count));

  for (i = 0; i < Count; i++) {
    DEBUG ((EFI_D_INFO, "UsbPhy: Mapping PHY %d base=0x%llx\n",
            i, Configs[i]->RegBase));

    Status = MapMemoryRegion (Configs[i]->RegBase, 0x1000, EfiMemoryMappedIO);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "UsbPhy: Failed to map PHY %d base (%r)\n",
              i, Status));
      return Status;
    }

    if (Configs[i]->RegBase2nd != 0) {
      DEBUG ((EFI_D_INFO, "UsbPhy: Mapping PHY %d 2nd base=0x%llx\n",
              i, Configs[i]->RegBase2nd));

      Status = MapMemoryRegion (Configs[i]->RegBase2nd, 0x1000, EfiMemoryMappedIO);
      if (EFI_ERROR (Status)) {
        DEBUG ((EFI_D_ERROR, "UsbPhy: Failed to map PHY %d 2nd base (%r)\n",
                i, Status));
        return Status;
      }
    }
  }

  //
  // Initialize each PHY
  //
  for (i = 0; i < Count; i++) {
    DEBUG ((EFI_D_INFO, "UsbPhy: Initializing PHY %d (base=0x%llx, ver=0x%x)\n",
            i, Configs[i]->RegBase, Configs[i]->Version));

    Status = UsbPhyInit (Configs[i]);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_ERROR, "UsbPhy: PHY %d init failed (%r)\n", i, Status));
      //
      // Continue with remaining PHYs even if one fails
      //
    }
  }

  DEBUG ((EFI_D_INFO, "UsbPhy: Exit\n"));
  return EFI_SUCCESS;
}
