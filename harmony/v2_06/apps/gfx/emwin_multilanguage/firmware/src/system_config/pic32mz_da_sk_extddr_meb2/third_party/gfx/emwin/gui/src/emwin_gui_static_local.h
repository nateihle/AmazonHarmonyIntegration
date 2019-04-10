/*******************************************************************************
  emWin GUI Wrapper local header

  Company:
    Microchip Technology Inc.

  File Name:
    emwin_gui_static_local.h

  Summary:
    This header file provides prototypes and definitions for the emWin GUI
    wrapper.

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

#ifndef _EMWIN_GUI_STATIC_LOCAL_H_
#define _EMWIN_GUI_STATIC_LOCAL_H_

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "third_party/gfx/emwin/gui/emwin_gui_static.h"
#include "gfx/hal/inc/gfx_driver_interface.h"

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

// *****************************************************************************
/* emWin GUI Wrapper States

  Summary:

  Description:

  Remarks:

 */

typedef enum
{
    /* Init display state */
    EMWIN_GUI_STATE_DISPLAY_INIT = 0,

    /* GUI Init state */
    EMWIN_GUI_STATE_INIT,
            
    /* Screen Init state */
    EMWIN_GUI_STATE_SCREEN_INIT,
            
    /* Tasks state */
    EMWIN_GUI_STATE_TASKS,

}EMWIN_GUI_STATES;

// *****************************************************************************
/* emWin GUI wrapper data object 

  Summary:

  Description:

  Remarks:

 */

typedef struct
{
    /* GUI wrapper states */
    EMWIN_GUI_STATES            state;
    
    /* GUI wrapper status */
    int32_t                     status;
    
    /* screen change flag */
    bool                        screenChanged;
    
    /* screen Id */
    int32_t                     screenId;
    
    /* screen create function array */
    EMWIN_GUI_SCREEN_CREATE     screenCreate[ EMWIN_GUI_NUM_SCREENS ];

    /* screen window handle */
    WM_HWIN                     hScreen[ EMWIN_GUI_NUM_SCREENS ];
        
    /* screen initialize function pointer */
    EMWIN_GUI_SCREEN_INITIALIZE screenInitialize;

} EMWIN_GUI_DATA;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _EMWIN_GUI_STATIC_LOCAL_H_

/*******************************************************************************
 End of File
 */


