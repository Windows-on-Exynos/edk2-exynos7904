/**
  Copyright@ Samsung Electronics Co. LTD

  This software is proprietary of Samsung Electronics.
  No part of this software, either material or conceptual may be copied or distributed, transmitted,
  transcribed, stored in a retrieval system or translated into any human or computer language in any form by any means,
  electronic, mechanical, manual or otherwise, or disclosed
  to third parties without the express written permission of Samsung Electronics.
**/

#include <Library/DebugLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include <Protocol/EFISpeedy.h>

#include "S2mpu09.h"

//
// Global Variables
//
STATIC EFI_SPEEDY_PROTOCOL *mSpeedyProtocol = NULL;
STATIC UINT8                mBusNumber      = 0;

EFI_STATUS
S2mpu09SetLdo (
  IN UINT8   LdoNumber,
  IN UINT8   Mode,
  IN BOOLEAN Enable)
{
  EFI_STATUS Status;
  UINT8      CtrlReg;
  UINT8      Value;

  // Verify SPEEDY Protocol
  if (mSpeedyProtocol == NULL) {
    return EFI_NOT_READY;
  }

  // Map LDO Number to Control Register
  switch (LdoNumber) {
    case 2:  CtrlReg = S2MPU09_PM_LDO2_CTRL;  break;
    case 12: CtrlReg = S2MPU09_PM_LDO12_CTRL; break;
    case 13: CtrlReg = S2MPU09_PM_LDO13_CTRL; break;
    case 14: CtrlReg = S2MPU09_PM_LDO14_CTRL; break;
    case 35: CtrlReg = S2MPU09_PM_LDO35_CTRL; break;
    case 38: CtrlReg = S2MPU09_PM_LDO38_CTRL; break;
    case 39: CtrlReg = S2MPU09_PM_LDO39_CTRL; break;
    default: return EFI_UNSUPPORTED;
  }

  // Mode = raw control register value (VSEL + config)
  // Enable overrides only the ON/OFF bits [7:6]
  Value = Mode;

  if (Enable) {
    Value |=  S2MPU09_OUTPUT_ON_NORMAL;
  } else {
    Value &= ~S2MPU09_OUTPUT_ON_NORMAL;
  }

  Status = mSpeedyProtocol->Write (mBusNumber, S2MPU09_PM_ADDR, CtrlReg, Value);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // LDO2 also needs voltage written to separate VOLT register
  if (LdoNumber == 2) {
    Status = mSpeedyProtocol->Read (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_LDO2_VOLT, &Value);
    if (EFI_ERROR (Status)) {
      return Status;
    }

    Value &= ~S2MPU09_LDO_VSEL_MASK;
    Value |=  (Mode & S2MPU09_LDO_VSEL_MASK);

    Status = mSpeedyProtocol->Write (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_LDO2_VOLT, Value);
  }

  return Status;
}

EFI_STATUS
S2mpu09SetWtsr (IN BOOLEAN Enable)
{
  EFI_STATUS Status;
  UINT8      Value;

  // Verify SPEEDY Protocol
  if (mSpeedyProtocol == NULL) {
    return EFI_NOT_READY;
  }

  // Get current WTSR Config
  Status = mSpeedyProtocol->Read (mBusNumber, S2MPU09_RTC_ADDR, S2MPU09_RTC_WTSR_SMPL, &Value);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // Enable/Disable WTSR
  if (Enable) {
    Value |=  S2MPU09_RTC_WTSR_MASK;
  } else {
    Value &= ~S2MPU09_RTC_WTSR_MASK;
  }

  // Write new WTSR Config
  Status = mSpeedyProtocol->Write (mBusNumber, S2MPU09_RTC_ADDR, S2MPU09_RTC_WTSR_SMPL, Value);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
S2mpu09SetSmpl (IN BOOLEAN Enable)
{
  EFI_STATUS Status;
  UINT8      Value;

  // Verify SPEEDY Protocol
  if (mSpeedyProtocol == NULL) {
    return EFI_NOT_READY;
  }

  // Get current SMPL Config
  Status = mSpeedyProtocol->Read (mBusNumber, S2MPU09_RTC_ADDR, S2MPU09_RTC_WTSR_SMPL, &Value);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // Enable/Disable SMPL
  if (Enable) {
    Value |=  S2MPU09_RTC_SMPL_MASK;
  } else {
    Value &= ~S2MPU09_RTC_SMPL_MASK;
  }

  // Write new SMPL Config
  Status = mSpeedyProtocol->Write (mBusNumber, S2MPU09_RTC_ADDR, S2MPU09_RTC_WTSR_SMPL, Value);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return EFI_SUCCESS;
}

STATIC
VOID
DisplayPmicInfo ()
{
  UINT8 Value;

  // Read and Display PMIC Registers
  if (!EFI_ERROR (mSpeedyProtocol->Read (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_INT1, &Value))) {
    DEBUG ((EFI_D_WARN, "S2MPU09 INT1      = 0x%x\n", Value));
  }

  if (!EFI_ERROR (mSpeedyProtocol->Read (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_INT2, &Value))) {
    DEBUG ((EFI_D_WARN, "S2MPU09 INT2      = 0x%x\n", Value));
  }

  if (!EFI_ERROR (mSpeedyProtocol->Read (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_PWRONSRC, &Value))) {
    DEBUG ((EFI_D_WARN, "S2MPU09 PWRONSRC  = 0x%x\n", Value));
  }

  if (!EFI_ERROR (mSpeedyProtocol->Read (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_OFFSRC, &Value))) {
    DEBUG ((EFI_D_WARN, "S2MPU09 OFFSRC    = 0x%x\n", Value));
  }

  if (!EFI_ERROR (mSpeedyProtocol->Read (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_RTC_BUF, &Value))) {
    DEBUG ((EFI_D_WARN, "S2MPU09 RTC_BUF   = 0x%x\n", Value));
  }

  if (!EFI_ERROR (mSpeedyProtocol->Read (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_CTRL1, &Value))) {
    DEBUG ((EFI_D_WARN, "S2MPU09 CTRL1     = 0x%x\n", Value));
  }

  if (!EFI_ERROR (mSpeedyProtocol->Read (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_CTRL3, &Value))) {
    DEBUG ((EFI_D_WARN, "S2MPU09 CTRL3     = 0x%x\n", Value));
  }

  if (!EFI_ERROR (mSpeedyProtocol->Read (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_LDO38_CTRL, &Value))) {
    DEBUG ((EFI_D_WARN, "S2MPU09 LDO38_CTRL= 0x%x\n", Value));
  }

  if (!EFI_ERROR (mSpeedyProtocol->Read (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_LDO39_CTRL, &Value))) {
    DEBUG ((EFI_D_WARN, "S2MPU09 LDO39_CTRL= 0x%x\n", Value));
  }

  if (!EFI_ERROR (mSpeedyProtocol->Read (mBusNumber, S2MPU09_RTC_ADDR, S2MPU09_RTC_WTSR_SMPL, &Value))) {
    DEBUG ((EFI_D_WARN, "S2MPU09 WTSR_SMPL = 0x%x\n", Value));
  }
}

EFI_STATUS
InitS2mpu09 (
  IN EFI_SPEEDY_PROTOCOL *SpeedyProtocol,
  IN UINT8                BusNumber)
{
  EFI_STATUS Status;
  UINT8      Value;

  // Save SPEEDY Details
  mSpeedyProtocol = SpeedyProtocol;
  mBusNumber      = BusNumber;

  // Disable Manual Reset
  Status = mSpeedyProtocol->Read (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_CTRL1, &Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "%a: Failed to Read CTRL1! Status = %r\n", __FUNCTION__, Status));
    return Status;
  }

  Value &= ~MRSTB_EN;

  Status = mSpeedyProtocol->Write (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_CTRL1, Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "%a: Failed to Disable Manual Reset! Status = %r\n", __FUNCTION__, Status));
    return Status;
  }

  // Enable Warm Reset
  Status = mSpeedyProtocol->Read (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_CTRL3, &Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "%a: Failed to Read CTRL3! Status = %r\n", __FUNCTION__, Status));
    return Status;
  }

  Value |= WRSTEN;

  Status = mSpeedyProtocol->Write (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_CTRL3, Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "%a: Failed to Enable Warm Reset! Status = %r\n", __FUNCTION__, Status));
    return Status;
  }

  // Enable AP Warm Reset Detection
  Status = mSpeedyProtocol->Read (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_CTRL3, &Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "%a: Failed to Read CTRL3! Status = %r\n", __FUNCTION__, Status));
    return Status;
  }

  Value |= WRSTBIEN;

  Status = mSpeedyProtocol->Write (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_CTRL3, Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "%a: Failed to Enable WRSTBIEN! Status = %r\n", __FUNCTION__, Status));
    return Status;
  }

  // PERI 32kHz on, AP 32kHz on
  Status = mSpeedyProtocol->Read (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_RTC_BUF, &Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "%a: Failed to Read RTC_BUF! Status = %r\n", __FUNCTION__, Status));
    return Status;
  }

  Value |= (_32KHZPERI_EN | _32KHZAP_EN);

  Status = mSpeedyProtocol->Write (mBusNumber, S2MPU09_PM_ADDR, S2MPU09_PM_RTC_BUF, Value);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "%a: Failed to Enable 32KHz! Status = %r\n", __FUNCTION__, Status));
    return Status;
  }

  // Enable USB PHY LDOs (vdd_ldo12, vdd_ldo13, vdd_ldo14 from A50 DTS)
  S2mpu09SetLdo (12, 0x00, TRUE);
  S2mpu09SetLdo (13, 0x00, TRUE);
  S2mpu09SetLdo (14, 0x00, TRUE);

  // Enable WTSR
  Status = S2mpu09SetWtsr (TRUE);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "%a: Failed to Enable WTSR!\n", __FUNCTION__));
    return Status;
  }

  // Enable SMPL
  Status = S2mpu09SetSmpl (TRUE);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "%a: Failed to Enable SMPL!\n", __FUNCTION__));
    return Status;
  }

  // Display PMIC Info
  DisplayPmicInfo ();

  return EFI_SUCCESS;
}
