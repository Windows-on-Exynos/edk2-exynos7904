#ifndef _EFI_I2C_H_
#define _EFI_I2C_H_

//
// Global GUID for the I2C Protocol
//
#define EFI_I2C_PROTOCOL_GUID { 0xB4E7D1A2, 0x5C3F, 0x4E82, { 0xA1, 0x9B, 0xDE, 0x67, 0xF8, 0x2C, 0x43, 0x91 } }

//
// Forward reference
//
typedef struct _EFI_I2C_PROTOCOL EFI_I2C_PROTOCOL;

typedef
EFI_STATUS
(EFIAPI *EFI_I2C_READ) (
  IN  UINT32 BaseAddress,
  IN  UINT8  Slave,
  IN  UINT8  Addr,
  OUT UINT8 *Data
  );

typedef
EFI_STATUS
(EFIAPI *EFI_I2C_BURST_READ) (
  IN  UINT32 BaseAddress,
  IN  UINT8  Slave,
  IN  UINT8  Addr,
  OUT UINT8 *Data,
  IN  UINT8  Count
  );

typedef
EFI_STATUS
(EFIAPI *EFI_I2C_WRITE) (
  IN UINT32 BaseAddress,
  IN UINT8  Slave,
  IN UINT8  Addr,
  IN UINT8  Data
  );

typedef
EFI_STATUS
(EFIAPI *EFI_I2C_BURST_WRITE) (
  IN UINT32 BaseAddress,
  IN UINT8  Slave,
  IN UINT8  Addr,
  IN UINT8 *Data,
  IN UINT8  Count
  );

struct _EFI_I2C_PROTOCOL {
  EFI_I2C_READ        Read;
  EFI_I2C_BURST_READ  BurstRead;
  EFI_I2C_WRITE       Write;
  EFI_I2C_BURST_WRITE BurstWrite;
};

extern EFI_GUID gEfiI2cProtocolGuid;

#endif /* _EFI_I2C_H_ */
