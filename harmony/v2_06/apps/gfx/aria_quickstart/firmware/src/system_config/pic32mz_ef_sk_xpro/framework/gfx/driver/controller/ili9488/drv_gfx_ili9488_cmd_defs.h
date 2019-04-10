/*******************************************************************************
 Module for Microchip Graphics Library - Hardware Abstraction Layer

  Company:
    Microchip Technology Inc.

  File Name:
    drv_gfx_ili9488_cmd_defs.h

  Summary:
    Contains ILI9488 command definitions.

  Description:
    Command definitions for ILI9488 commands.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef DRV_GFX__ILI9488_CMD_DEFS_H    /* Guard against multiple inclusion */
#define DRV_GFX__ILI9488_CMD_DEFS_H

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

/* ************************************************************************** */
/** ILI9488_CMD_*
  Summary:
    Command defines.
*/
#define ILI9488_CMD_SLEEP_OUT                       0x11
#define ILI9488_CMD_DISPLAY_ON                      0x29
#define ILI9488_CMD_INTERFACE_PIXEL_FORMAT_SET      0x3A
#define ILI9488_CMD_SET_IMAGE_FUNCTION              0xE9
#define ILI9488_CMD_INTERFACE_MODE_CONTROL          0xB0
#define ILI9488_CMD_MEMORY_ACCESS_CONTROL           0x36
#define ILI9488_CMD_COLUMN_ADDRESS_SET              0x2A
#define ILI9488_CMD_PAGE_ADDRESS_SET                0x2B
#define ILI9488_CMD_MEMORY_WRITE                    0x2C
#define ILI9488_CMD_MEMORY_READ                     0x2E

/** ILI9488_COLOR_PIX_FMT_18BPP
  @Summary
    Color pixel format value for 18BPP
*/
#define ILI9488_COLOR_PIX_FMT_16BPP          0x5
#define ILI9488_COLOR_PIX_FMT_18BPP          0x6

/** ILI9488_MADCTL_*
  Summary:
    Memory access control command parameter bit values.
*/
#define ILI9488_MADCTL_RGB_BGR_ORDER_CTRL           (1 << 3)
#define ILI9488_MADCTL_ROW_COLUMN_EXCHANGE          (1 << 5)
#define ILI9488_MADCTL_COL_ADDR_ORDER               (1 << 6)
#define ILI9488_MADCTL_ROW_ADDR_ORDER               (1 << 7)

#ifdef __cplusplus
}
#endif

#endif /* DRV_GFX__ILI9488_CMD_DEFS_H */

/* *****************************************************************************
 End of File
 */
