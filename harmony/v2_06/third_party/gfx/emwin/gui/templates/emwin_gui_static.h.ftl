/*******************************************************************************
  emWin GUI wrapper Header File

  Company:
    Microchip Technology Inc.

  File Name:
    emwin_gui_static.h

  Summary:
    This header file provides prototypes and definitions for emwin GUI wrapper.

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

#ifndef _EMWIN_GUI_STATIC_H_
#define _EMWIN_GUI_STATIC_H_

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "GUI.h"
#include "DIALOG.h"

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
/* 

  Summary:

  Description:

  Remarks:

 */

typedef WM_HWIN (* EMWIN_GUI_SCREEN_CREATE)(void);

// *****************************************************************************
/* 

  Summary:

  Description:

  Remarks:

 */

typedef void    (* EMWIN_GUI_SCREEN_INITIALIZE )( void );

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:

  Summary:

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    </code>

  Remarks:

 */
void emWin_GuiInitialize(void);

/*******************************************************************************
  Function:

  Summary:

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    </code>

  Remarks:

 */

void emWin_GuiScreenInitializeRegister( EMWIN_GUI_SCREEN_INITIALIZE screenInit );

/*******************************************************************************
  Function:

  Summary:

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    </code>

  Remarks:

 */

void emWin_GuiScreenRegister( int32_t screenId, EMWIN_GUI_SCREEN_CREATE screen );

/*******************************************************************************
  Function:

  Summary:

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    </code>

  Remarks:

 */

void emWin_GuiStartScreenSet( int32_t screenId );

/*******************************************************************************
  Function:

  Summary:

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    </code>

  Remarks:

 */

void emWin_GuiScreenChange( int32_t screenId );

/*******************************************************************************
  Function:

  Summary:

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    </code>

  Remarks:

 */

WM_HWIN emWin_GuiScreenGet( int32_t screenId );

/*******************************************************************************
  Function:

  Summary:

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    </code>

  Remarks:

 */

void emWin_GuiScreenEnd( int32_t screenId );

/*******************************************************************************
  Function:

  Summary:

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    </code>

  Remarks:

 */
void emWin_GuiTasks(void);

/*******************************************************************************
  Function:

  Summary:

  Description:

  Precondition:

  Parameters:

  Returns:

  Example:
    <code>
    </code>

  Remarks:

 */
void emWin_Tasks(void);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _EMWIN_GUI_STATIC_H_

/*******************************************************************************
 End of File
 */

