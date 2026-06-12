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

#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>

#include <Library/ExynosClockLib.h>

#include "ExynosClock.h"

EFI_STATUS
ExynosClockInit (VOID)
{
	DEBUG ((EFI_D_INFO, "[ExynosClockDxe]: Clock Init\n"));

	return EFI_SUCCESS;
}