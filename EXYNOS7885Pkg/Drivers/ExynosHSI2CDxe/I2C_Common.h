#ifndef __I2C_COMMON_H__
#define __I2C_COMMON_H__

#define I2C_M_WR		(0x0000)
#define I2C_M_RD		(0x0001)

/*
 * Controller operating frequency, timing values for operation
 * are calculated against this frequency
 */
#define I2C_HS_TX_CLOCK		(2500000)
#define I2C_FAST_PLUS_TX_CLOCK	(1000000)
#define I2C_FS_TX_CLOCK		(400000)
#define I2C_STAND_TX_CLOCK	(100000)

#define I2C_STAND_SPD		(3)
#define I2C_FAST_PLUS_SPD	(2)
#define I2C_HIGH_SPD		(1)
#define I2C_FAST_SPD		(0)

struct i2c_msg {
  UINT32 Address;
  UINT32 Flags;
  UINT32 Length;
  UINT8  *Buffer;
};

struct i2c_info {
  INTN Channel;
  UINT32 Clock;
  UINT32 Base;
  UINT32 InitialisationDone;

  /* for Exynos HSI2C */
  INTN Mode;

	/* for Exynos I2C */
  INTN State;
  struct i2c_msg *Message;
  UINT32 MessageNumber;
  UINT32 MessageIndex;
  UINT32 MessagePointer;
};

#endif  // __I2C_COMMON_H__
