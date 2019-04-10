/*******************************************************************************
  emWin Touch Wrapper Local Header File

  Company:
    Microchip Technology Inc.

  File Name:
    emwin_touch_static_local.h

  Summary:
    This header file provides prototypes and definitions local to emWin 
    touch wrapper

  Description:
    
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#ifndef _EMWIN_TOUCH_STATIC_LOCAL_H_
#define _EMWIN_TOUCH_STATIC_LOCAL_H_

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "third_party/gfx/emwin/touch/emwin_touch_static.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************


typedef enum
{
    /* Init state */
    EMWIN_TOUCH_STATE_INIT = 0,

    /* Touch process state */
    EMWIN_TOUCH_STATE_PROCESS,
            


}EMWIN_TOUCH_STATES;


typedef struct
{
    /* Pointer input device state */
    GUI_PID_STATE           pidState;
    
    /* layer index */
    uint32_t                layerIndex;

    /* Touch orientation */
    EMWIN_TOUCH_ORIENTATION orientation;

    /* Display width */
    uint32_t                displayWidth;

    /* Display Height */
    uint32_t                displayHeight;    

	EMWIN_TOUCH_STATES      touchState;
	
} EMWIN_TOUCH_DATA;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
<#if CONFIG_USE_SYS_TOUCH_NEEDED>
/* emWin Touch Message Call back 

  Summary:

  Description:

  Remarks:

 */

static void _emWin_TouchMessageCallback( TOUCH_MSG_OBJ *pMsg );
</#if>
//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _EMWIN_TOUCH_STATIC_LOCAL_H_

/*******************************************************************************
 End of File
 */

