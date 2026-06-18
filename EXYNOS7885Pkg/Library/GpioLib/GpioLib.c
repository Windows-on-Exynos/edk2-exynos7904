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

#include <Library/GpioLib.h>

STATIC
EFI_GPIO_CONTROLLER_DATA
gGpioControllers[] = {
  //
  // Pinctrl SHUB (0x11080000)
  //
  {
    .Address    = 0x11080000,
    {
      {
        .Id     = BANK_ID_H,
        .Number = 0,
        .Offset = 0x0
      },
      {
        .Id     = BANK_ID_H,
        .Number = 1,
        .Offset = 0x20
      }
    }
  },

  //
  // Pinctrl ALIVE (0x11850000)
  //
  {
    .Address    = 0x11850000,
    {
      {
        .Id     = BANK_ID_A,
        .Number = 0,
        .Offset = 0x20
      },
      {
        .Id     = BANK_ID_A,
        .Number = 1,
        .Offset = 0x40
      },
      {
        .Id     = BANK_ID_A,
        .Number = 2,
        .Offset = 0x60
      },
      {
        .Id     = BANK_ID_Q,
        .Number = 0,
        .Offset = 0x80
      }
    }
  },

  //
  // Pinctrl CMGP (0x11C20000)
  //
  {
    .Address    = 0x11C20000,
    {
      {
        .Id     = BANK_ID_M,
        .Number = 0,
        .Offset = 0x0
      },
      {
        .Id     = BANK_ID_M,
        .Number = 1,
        .Offset = 0x20
      },
      {
        .Id     = BANK_ID_M,
        .Number = 2,
        .Offset = 0x40
      },
      {
        .Id     = BANK_ID_M,
        .Number = 3,
        .Offset = 0x60
      },
      {
        .Id     = BANK_ID_M,
        .Number = 4,
        .Offset = 0x80
      },
      {
        .Id     = BANK_ID_M,
        .Number = 5,
        .Offset = 0xA0
      },
      {
        .Id     = BANK_ID_M,
        .Number = 6,
        .Offset = 0xC0
      },
      {
        .Id     = BANK_ID_M,
        .Number = 7,
        .Offset = 0xE0
      },
      {
        .Id     = BANK_ID_M,
        .Number = 8,
        .Offset = 0x100
      },
      {
        .Id     = BANK_ID_M,
        .Number = 9,
        .Offset = 0x120
      },
      {
        .Id     = BANK_ID_M,
        .Number = 10,
        .Offset = 0x140
      },
      {
        .Id     = BANK_ID_M,
        .Number = 11,
        .Offset = 0x160
      },
      {
        .Id     = BANK_ID_M,
        .Number = 12,
        .Offset = 0x180
      },
      {
        .Id     = BANK_ID_M,
        .Number = 13,
        .Offset = 0x1A0
      },
      {
        .Id     = BANK_ID_M,
        .Number = 14,
        .Offset = 0x1C0
      },
      {
        .Id     = BANK_ID_M,
        .Number = 15,
        .Offset = 0x1E0
      },
      {
        .Id     = BANK_ID_M,
        .Number = 16,
        .Offset = 0x200
      },
      {
        .Id     = BANK_ID_M,
        .Number = 17,
        .Offset = 0x220
      },
      {
        .Id     = BANK_ID_M,
        .Number = 18,
        .Offset = 0x240
      },
      {
        .Id     = BANK_ID_M,
        .Number = 19,
        .Offset = 0x260
      },
      {
        .Id     = BANK_ID_M,
        .Number = 20,
        .Offset = 0x280
      },
      {
        .Id     = BANK_ID_M,
        .Number = 21,
        .Offset = 0x2A0
      },
      {
        .Id     = BANK_ID_M,
        .Number = 22,
        .Offset = 0x2C0
      },
      {
        .Id     = BANK_ID_M,
        .Number = 23,
        .Offset = 0x2E0
      },
      {
        .Id     = BANK_ID_M,
        .Number = 24,
        .Offset = 0x300
      },
      {
        .Id     = BANK_ID_M,
        .Number = 25,
        .Offset = 0x320
      }
    }
  },

  //
  // Pinctrl FSYS (0x13490000)
  //
  {
    .Address    = 0x13490000,
    {
      {
        .Id     = BANK_ID_F,
        .Number = 0,
        .Offset = 0x0
      },
      {
        .Id     = BANK_ID_F,
        .Number = 1,
        .Offset = 0x20
      },
      {
        .Id     = BANK_ID_F,
        .Number = 2,
        .Offset = 0x40
      }
    }
  },

  //
  // Pinctrl TOP (0x139B0000)
  //
  {
    .Address    = 0x139B0000,
    {
      {
        .Id     = BANK_ID_C,
        .Number = 0,
        .Offset = 0x60
      },
      {
        .Id     = BANK_ID_C,
        .Number = 1,
        .Offset = 0x80
      },
      {
        .Id     = BANK_ID_C,
        .Number = 2,
        .Offset = 0xA0
      },
      {
        .Id     = BANK_ID_G,
        .Number = 0,
        .Offset = 0xC0
      },
      {
        .Id     = BANK_ID_G,
        .Number = 1,
        .Offset = 0xE0
      },
      {
        .Id     = BANK_ID_G,
        .Number = 2,
        .Offset = 0x100
      },
      {
        .Id     = BANK_ID_G,
        .Number = 3,
        .Offset = 0x120
      },
      {
        .Id     = BANK_ID_G,
        .Number = 4,
        .Offset = 0x140
      },
      {
        .Id     = BANK_ID_P,
        .Number = 0,
        .Offset = 0x0
      },
      {
        .Id     = BANK_ID_P,
        .Number = 1,
        .Offset = 0x20
      },
      {
        .Id     = BANK_ID_P,
        .Number = 2,
        .Offset = 0x40
      }
    }
  },

  //
  // Pinctrl DISPAUD (0x14A60000)
  //
  {
    .Address    = 0x14A60000,
    {
      {
        .Id     = BANK_ID_B,
        .Number = 0,
        .Offset = 0x0
      },
      {
        .Id     = BANK_ID_B,
        .Number = 1,
        .Offset = 0x20
      },
      {
        .Id     = BANK_ID_B,
        .Number = 2,
        .Offset = 0x40
      }
    }
  },
};

VOID
GetGpioControllerData (
  OUT EFI_GPIO_CONTROLLER_DATA **Data,
  OUT UINT8                     *Count)
{
  // Pass Data
  *Data  = gGpioControllers;
  *Count = ARRAY_SIZE (gGpioControllers);
}