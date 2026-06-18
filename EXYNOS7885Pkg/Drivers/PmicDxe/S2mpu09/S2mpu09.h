/**
  Copyright@ Samsung Electronics Co. LTD

  This software is proprietary of Samsung Electronics.
  No part of this software, either material or conceptual may be copied or distributed, transmitted,
  transcribed, stored in a retrieval system or translated into any human or computer language in any form by any means,
  electronic, mechanical, manual or otherwise, or disclosed
  to third parties without the express written permission of Samsung Electronics.
**/

#ifndef _S2MPU09_H_
#define _S2MPU09_H_

//
// Slave Addresses
//
#define S2MPU09_COMMON_ADDR       0x000
#define S2MPU09_PM_ADDR           0x001
#define S2MPU09_RTC_ADDR          0x002

//
// PM Register Addresses
//
#define S2MPU09_PM_INT1           0x000
#define S2MPU09_PM_INT2           0x001
#define S2MPU09_PM_PWRONSRC       0x00C
#define S2MPU09_PM_OFFSRC         0x00D
#define S2MPU09_PM_RTC_BUF        0x00F
#define S2MPU09_PM_CTRL1          0x010
#define S2MPU09_PM_CTRL3          0x012
#define S2MPU09_PM_LDO2_CTRL      0x03A
#define S2MPU09_PM_LDO2_VOLT      0x03B
#define S2MPU09_PM_LDO12_CTRL     0x045
#define S2MPU09_PM_LDO13_CTRL     0x046
#define S2MPU09_PM_LDO14_CTRL     0x047
#define S2MPU09_PM_LDO35_CTRL     0x05C
#define S2MPU09_PM_LDO38_CTRL     0x05F
#define S2MPU09_PM_LDO39_CTRL     0x060

//
// RTC Register Addresses
//
#define S2MPU09_RTC_WTSR_SMPL     0x001
#define S2MPU09_RTC_UPDATE        0x002
#define S2MPU09_RTC_CAP_SEL       0x003
#define S2MPU09_RTC_MSEC          0x004
#define S2MPU09_RTC_SEC           0x005
#define S2MPU09_RTC_MIN           0x006
#define S2MPU09_RTC_HOUR          0x007
#define S2MPU09_RTC_WEEK          0x008
#define S2MPU09_RTC_DAY           0x009
#define S2MPU09_RTC_MON           0x00A
#define S2MPU09_RTC_YEAR          0x00B

//
// WTSR & SMPL Register Bits
//
#define S2MPU09_RTC_WTSR_MASK     BIT6
#define S2MPU09_RTC_SMPL_MASK     BIT7

//
// CTRL1
//
#define MRSTB_EN                  BIT4

//
// CTRL3
//
#define WRSTBIEN                  BIT6
#define WRSTEN                    BIT4

//
// RTC_BUF
//
#define _32KHZPERI_EN             BIT2
#define _32KHZAP_EN               BIT0

//
// LDOx_CTRL
//
#define S2MPU09_OUTPUT_ON_NORMAL  (BIT7 | BIT6)

//
// LDO Voltage Selection
//
#define S2MPU09_LDO_VSEL_MASK     0x03F
#define S2MPU09_LDO_VSEL_1V8      0x000
#define S2MPU09_LDO_VSEL_3V3      0x03C

#endif /* _S2MPU09_H_ */
