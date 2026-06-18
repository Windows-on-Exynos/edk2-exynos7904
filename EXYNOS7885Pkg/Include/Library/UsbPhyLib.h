#ifndef _USB_PHY_LIB_H_
#define _USB_PHY_LIB_H_

//
// PHY instance type
//
typedef enum {
  UsbPhyUtmi  = 0,   // USB 2.0 UTMI PHY
  UsbPhyPipe3 = 1,   // USB 3.0 PIPE3 PHY
} USB_PHY_TYPE;

//
// PHY configuration for one instance
//
typedef struct {
  USB_PHY_TYPE    Type;
  UINT64          RegBase;         // PHY control register base
  UINT64          RegBase2nd;      // Secondary reg base (for dual PHY)
  UINT32          Version;         // PHY version (e.g. 0x300, 0x530)
  UINT32          PmuOffset;       // PMU offset for isolation
  UINT32          PmuMask;         // PMU mask bits for isolation
  UINT32          RefClk;          // Reference clock frequency (Hz)
  UINT32          ExtRefClk;       // FSEL value for reference clock
  UINT32          RefSel;          // Reference clock source selection
  BOOLEAN         NotUsedVbusPad;  // VBUS pad not used (force VBUS valid)
  BOOLEAN         UseIoForOvc;     // Use I/O for over-current
  BOOLEAN         CommonBlockDisable; // Disable common block
  BOOLEAN         HsRewa;          // HS Remote Wake-up Advisor
  BOOLEAN         DualPhy;         // Dual PHY configuration
  UINT32          UsedPhyPort;     // Which PHY port is used
  UINT32          Usb3PhyIsolation; // USB3 PHY isolation enabled
} USB_PHY_CONFIG;

/**
  Initialize a USB PHY instance.

  @param[in]  PhyConfig   Pointer to the PHY configuration.

  @retval EFI_SUCCESS           Initialization successful.
  @retval EFI_INVALID_PARAMETER PhyConfig is NULL.
  @retval EFI_DEVICE_ERROR      PHY init failed.
**/
EFI_STATUS
UsbPhyInit (
  IN USB_PHY_CONFIG  *PhyConfig
  );

/**
  Power down a USB PHY instance.

  @param[in]  PhyConfig   Pointer to the PHY configuration.

  @retval EFI_SUCCESS           Power down successful.
  @retval EFI_INVALID_PARAMETER PhyConfig is NULL.
**/
EFI_STATUS
UsbPhyExit (
  IN USB_PHY_CONFIG  *PhyConfig
  );

/**
  Get the number of PHY instances for this SoC and their configurations.

  @param[out] Configs      Pointer to array of PHY config pointers.
  @param[out] Count        Number of PHY instances.

  @retval EFI_SUCCESS           Configurations retrieved.
  @retval EFI_UNSUPPORTED       PHY not supported on this platform.
**/
EFI_STATUS
GetUsbPhyConfigs (
  OUT USB_PHY_CONFIG  ***Configs,
  OUT UINT8            *Count
  );

/**
  Signal VBUS valid + D+ pull-up to start USB device connection.
  Must be called AFTER DWC3 controller Run/Stop is set.
**/
VOID
UsbPhyConnect (VOID);

/**
  Clear VBUS valid signal (USB device disconnect).
**/
VOID
UsbPhyDisconnect (VOID);

#endif /* _USB_PHY_LIB_H_ */
