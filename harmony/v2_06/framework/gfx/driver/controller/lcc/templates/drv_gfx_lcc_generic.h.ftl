/*******************************************************************************
  MPLAB Harmony LCC Generated Driver Header File

  File Name:
    drv_gfx_lcc_generic.h

  Summary:
    Build-time generated header file
	Interface for the graphics library where the primitives are rendered and 
	sent to the graphics controller either external or internal

  Description:
    This header file contains the function prototypes and definitions of
    the data types and constants that make up the interface to the Low-Cost
    Controllerless (LCC) Graphics Controller.

    Created with MPLAB Harmony Version ${CONFIG_MPLAB_HARMONY_VERSION_STRING}
*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2016 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _DRV_GFX_LCC_H
#define _DRV_GFX_LCC_H

#include "gfx/hal/inc/gfx_common.h"
#include "gfx/hal/inc/gfx_driver_interface.h"
#include "gfx/hal/inc/gfx_default_impl.h"

#include "system/dma/sys_dma.h"
#include "system/int/sys_int.h"

#ifdef __cplusplus
    extern "C" {
#endif

// function pointer definition for VSYNC callback
typedef void (*LCC_VSYNC_Callback_FnPtr)(void);

typedef enum
{
	LCC_VSYNC_TRUE = 0,
	LCC_VSYNC_FALSE
} LCC_VSYNC_STATE;
 
typedef enum
{
    /* */
    ACTIVE_PERIOD = 0,

    /* */
    BLANKING_PERIOD,
    
    /* */
    FINISH_LINE,
    
    /* */
    OVERFLOW,

    /* */
    PIP,

    /* */
    SCROLL,

} DMA_ISR_TASK;

//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data Types and Constants
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* LCC Driver Module Index Count

  Summary:
    Number of valid LCC driver indices.

  Description:
    This constant identifies LCC driver index definitions.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is device-specific.
*/

#define DRV_GFX_LCC_INDEX_COUNT     DRV_GFX_LCC_NUMBER_OF_MODULES

// *****************************************************************************
/* LCC Driver Module Index Count

  Summary:

  Description:

  Remarks:

 */
typedef enum
{
    /* */
    DRV_GFX_LCC_FB_WRITE_BUS_TYPE_NONE = 0,

    /* */
    DRV_GFX_LCC_FB_WRITE_BUS_TYPE_PMP,

    /* */
    DRV_GFX_LCC_FB_WRITE_BUS_TYPE_EBI,

} DRV_GFX_LCC_FB_WRITE_BUS_TYPE;

// *****************************************************************************
/* LCC Driver Module Index Count

  Summary:

  Description:

  Remarks:

*/
typedef enum
{
    /* */
    DRV_GFX_LCC_DISPLAY_WRITE_BUS_TYPE_NONE = 0,

    /* */
    DRV_GFX_LCC_DISPLAY_WRITE_BUS_TYPE_PMP,

    /* */
    DRV_GFX_LCC_DISPLAY_WRITE_BUS_TYPE_EBI,

} DRV_GFX_LCC_DISPLAY_WRITE_BUS_TYPE;

// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************

GFX_Result driverLCCInfoGet(GFX_DriverInfo* info);
GFX_Result driverLCCContextInitialize(GFX_Context* context);

#ifdef __cplusplus
    }
#endif
    
#endif