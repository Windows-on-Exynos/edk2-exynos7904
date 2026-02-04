/*
 * Copyright (C) 2025-2026 viZPilot.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef UFS_H
#define UFS_H

#define	SCSI_MAX_INITIATOR	1
#define	SCSI_MAX_DEVICE		8

#define DW_NUM_OF_TSF		20
#define UPIU_DATA_SIZE		(ALIGNED_UPIU_SIZE - \
		sizeof(UINT8) * DW_NUM_OF_TSF - sizeof(struct UfsUpiuHeader))

#define UFS_GEAR		3
#define UFS_RATE		2
#define UFS_POWER_MODE	1
#define UFS_RXTX_POWER_MODE	((UFS_POWER_MODE << 4)|UFS_POWER_MODE)

typedef struct {
	CHAR8 HostName[16];
	UINTN Irq;
	
	INTN HostIndex;

	UINT8 *UFSDescriptor;
	UINT8 *ArgList;
	UINT32 Lun;
	INTN SCSIStatus;
	UINT8 *SenseBuffer;
	UINT32 SenseBufLen;

	UINT32 Capabilities;
	INTN Nutrs;
	INTN Nutmrs;
	UINT32 UFSVersion;

	UINT32 IntEnableMask;

	UINT32 Quirks;

	UINT32 Errors;

	UINT32 UFSCmdTimeout;
	UINT32 UICCmdTimeout;
	UINT32 UFSQueryReqTimeout;
	UINT32 Timeout;

	UINT16 DataSegLen;
	UINT8 UpiuData[UPIU_DATA_SIZE * 4];
	
	UINT32 DevPwrShift;
} EXYNOS_UFS_HOST;

#endif
