/**
  Copyright (C) Samsung Electronics Co. LTD

  This software is proprietary of Samsung Electronics.
  No part of this software, either material or conceptual may be copied or distributed, transmitted,
  transcribed, stored in a retrieval system or translated into any human or computer language in any form by any means,
  electronic, mechanical, manual or otherwise, or disclosed
  to third parties without the express written permission of Samsung Electronics.

  Alternatively, this program is free software in case of open source project
  you can redistribute it and/or modify
  it under the terms of the GNU General Public License version 2 as
  published by the Free Software Foundation.
**/

#ifndef _I2C_H_
#define _I2C_H_

//
// I2C MMIO
//
#define I2C_MMIO_LENGTH                           0x1000

// I2C Register Offsets
#define I2C_I2CCON                                0x00
#define I2C_I2CSTAT                               0x04
#define I2C_I2CADD                                0x08
#define I2C_I2CDS                                 0x0C
#define I2C_I2CLC                                 0x10   // Line Control (S3C2440+)

// I2CCON Register Bits
#define I2CCON_ACKEN                              (1U << 7)
#define I2CCON_TXDIV_16                           (0U << 6)
#define I2CCON_TXDIV_512                          (1U << 6)
#define I2CCON_IRQEN                              (1U << 5)
#define I2CCON_IRQPEND                            (1U << 4)
#define I2CCON_BUS_RELEASE                        (1U << 4)
#define I2CCON_SCALE(x)                           ((x) & 0xFU)
#define I2CCON_SCALEMASK                          (0xFU)

// I2CSTAT Register Bits
#define I2CSTAT_MASTER_RX                         (2U << 6)
#define I2CSTAT_MASTER_TX                         (3U << 6)
#define I2CSTAT_SLAVE_RX                          (0U << 6)
#define I2CSTAT_SLAVE_TX                          (1U << 6)
#define I2CSTAT_MODEMASK                          (3U << 6)

#define I2CSTAT_START                             (1U << 5)
#define I2CSTAT_BUSBUSY                           (1U << 5)
#define I2CSTAT_TXRXEN                            (1U << 4)
#define I2CSTAT_ARBITR                            (1U << 3)
#define I2CSTAT_ASSLAVE                           (1U << 2)
#define I2CSTAT_ADDR0                             (1U << 1)
#define I2CSTAT_LASTBIT                           (1U << 0)    // Last bit (ACK=0, NACK=1)

// I2CLC (Line Control) Register Bits
#define I2CLC_SDA_DELAY0                          (0U << 0)
#define I2CLC_SDA_DELAY5                          (1U << 0)
#define I2CLC_SDA_DELAY10                         (2U << 0)
#define I2CLC_SDA_DELAY15                         (3U << 0)
#define I2CLC_SDA_DELAY_MASK                      (3U << 0)

#define I2CLC_FILTER_ON                           (1U << 2)    // Line filter enable

// I2C Timing Constants
#define I2C_IDLE_TIMEOUT                          (5000)
#define I2C_MASTER_TIMEOUT                        (400)
#define I2C_ACK_POLL_COUNT                        (5000)

// I2C State Machine
#define I2C_STATE_IDLE                            0
#define I2C_STATE_STOP                            1
#define I2C_STATE_START                           2
#define I2C_STATE_READ                            3
#define I2C_STATE_WRITE                           4

// I2C GPIO Configuration
#define I2C_GPIO_SCL_SDA                          (0x02)       // Function value 2 = I2C

// I2C Speed Modes
#define I2C_STAND_TX_CLOCK                        (100000)
#define I2C_FS_TX_CLOCK                           (400000)

// I2C Message Flags
#define I2C_M_WR                                  (0x0000)
#define I2C_M_RD                                  (0x0001)

// I2C Transfer Constants
#define I2C_MAX_BUF                               (256)
#define I2C_MAX_RETRY                             (3)
#define I2C_RETRY_DELAY_US                        (100)

#endif /* _I2C_H_ */
