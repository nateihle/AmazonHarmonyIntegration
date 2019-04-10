/*******************************************************************************
  emWin GUI wrapper Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    emwin_gui_static.c

  Summary:
    This file contains the source code for the emWin GUI wrapper.

  Description:

 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "emwin_gui_static_local.h"
<#if CONFIG_SEGGER_EMWIN_GUI_WRAPPER == true>
#include "../third_party/gfx/emwin/touch/emwin_touch_static.h"
</#if>
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

static EMWIN_GUI_DATA emWinGuiData;

/*******************************************************************************
  Function:
    void emWin_GuiInitialize(void)

  Remarks:
    See prototype in emwin_gui_static.h.
 */

void emWin_GuiInitialize(void)
{   
    emWinGuiData.state         = EMWIN_GUI_STATE_DISPLAY_INIT;
    emWinGuiData.screenChanged = true;
    emWinGuiData.screenId      = 0;
    
	if (GFX_Open(0, 0, 0, GFX_NULL) != NULL)
    {
        emWinGuiData.state = EMWIN_GUI_STATE_INIT;                
    }
	
	emWinGuiData.status = GUI_Init();
	if( 0 == emWinGuiData.status )
    {
                emWinGuiData.state = EMWIN_GUI_STATE_SCREEN_INIT;
    }
	
	
    return;
}

/*******************************************************************************
  Function:
    void emWin_GuiScreenInitializeRegister( 
                                       EMWIN_GUI_SCREEN_INITIALIZE screenInit )

  Remarks:
    See prototype in emwin_gui_static.h.
*/

void emWin_GuiScreenInitializeRegister( EMWIN_GUI_SCREEN_INITIALIZE screenInit )
{
    if( NULL == screenInit )
    {
        return;
    }
    
    emWinGuiData.screenInitialize = screenInit;
}

/*******************************************************************************
  Function:
    void emWin_GuiScreenRegister( int32_t screenId, 
                                  EMWIN_GUI_SCREEN_CREATE screen )

  Remarks:
    See prototype in emwin_gui_static.h.
 */

void emWin_GuiScreenRegister( int32_t screenId, EMWIN_GUI_SCREEN_CREATE screen )
{
    if( screenId >= EMWIN_GUI_NUM_SCREENS )
    {
        return;
    }
    
    if( NULL == screen )
    {
        return;
    }
    
    emWinGuiData.screenCreate[screenId] = screen;
    
    return;

}

/*******************************************************************************
  Function:
    void emWin_GuiStartScreenSet( int32_t screenId )

  Remarks:
    See prototype in emwin_gui_static.h.
*/

void emWin_GuiStartScreenSet( int32_t screenId )
{
    if( screenId >= EMWIN_GUI_NUM_SCREENS )
    {
        return;
    }
    
    emWinGuiData.screenId = screenId;
}

/*******************************************************************************
  Function:
    void emWin_GuiScreenChange( int32_t screenId )

  Remarks:
    See prototype in emwin_gui_static.h.
 */
void emWin_GuiScreenChange( int32_t screenId )
{
    if( screenId >= EMWIN_GUI_NUM_SCREENS )
    {
        return;
    }
    
    emWinGuiData.screenChanged = true;
    emWinGuiData.screenId      = screenId;
    
    return;
}

/*******************************************************************************
  Function:
    WM_HWIN emWin_GuiScreenGet( int32_t screenId )

  Remarks:
    See prototype in emwin_gui_static.h.
 */

WM_HWIN emWin_GuiScreenGet( int32_t screenId )
{
    if( screenId >= EMWIN_GUI_NUM_SCREENS )
    {
        return 0;
    }

    return ( emWinGuiData.hScreen[screenId] );
}

/*******************************************************************************
  Function:
    void emWin_GuiScreenEnd ( int32_t screenId )

  Remarks:
    See prototype in emwin_gui_static.h.
 */

void emWin_GuiScreenEnd( int32_t screenId )
{
    if( screenId >= EMWIN_GUI_NUM_SCREENS )
    {
        return;
    }

    if( 0 != emWinGuiData.hScreen[screenId] )
    {

        GUI_EndDialog( emWinGuiData.hScreen[screenId], 0 );
        emWinGuiData.hScreen[screenId] = 0;

    }

    return;
}


/*********************************************************************
*
*       emWinTasks
*/

void emWin_Tasks(void)
{
    emWin_GuiTasks();
    <#if CONFIG_SEGGER_EMWIN_GUI_WRAPPER == true>
    emWin_TouchTasks();
	</#if>
}

void emWin_GuiTasks(void) 
{
    switch( emWinGuiData.state )
    {
        case EMWIN_GUI_STATE_DISPLAY_INIT:
		{
            if (GFX_Open(0, 0, 0, GFX_NULL) != NULL)
            {
                emWinGuiData.state = EMWIN_GUI_STATE_INIT;                
            }
            break;
        }        
        case EMWIN_GUI_STATE_INIT:
        {
            /* Initialize SEGGER emWin GUI Library */
            emWinGuiData.status = GUI_Init();
  
            if( 0 == emWinGuiData.status )
            {
                emWinGuiData.state = EMWIN_GUI_STATE_SCREEN_INIT;
            }

            break;
        }
        
        case EMWIN_GUI_STATE_SCREEN_INIT:
        {
            /* Initialize first screen */
            if( NULL != emWinGuiData.screenInitialize )
            {
                emWinGuiData.screenInitialize();
            }
            
            emWinGuiData.state = EMWIN_GUI_STATE_TASKS;
            
            break;
        }
        
        case EMWIN_GUI_STATE_TASKS:
        {
            if(emWinGuiData.screenChanged)
	        {
		        emWinGuiData.screenChanged = 0;
                
                if( NULL != emWinGuiData.screenCreate[emWinGuiData.screenId] )
                {
                    if ( !emWinGuiData.hScreen[emWinGuiData.screenId] )
                    {
                        emWinGuiData.hScreen[emWinGuiData.screenId] = 
                           emWinGuiData.screenCreate[emWinGuiData.screenId]();
                    }
                    else
                    {
                        WM_BringToTop(emWinGuiData.hScreen[emWinGuiData.screenId]);
                    }
                }
	        }
            
            GUI_Exec();
            
            break;
        }
        
        default:
        {
            break;
        }
    }
    
    return;
}

/*******************************************************************************
 End of File
 */



