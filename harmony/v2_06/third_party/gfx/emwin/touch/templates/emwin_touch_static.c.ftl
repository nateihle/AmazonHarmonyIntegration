/*******************************************************************************
  emWin Touch Wrapper Source File

  Company:
    Microchip Technology Inc.

  File Name:
    emwin_touch_static.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
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
#include "emwin_touch_static_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
static EMWIN_TOUCH_DATA emWinTouchData;

<#if CONFIG_USE_SYS_INPUT == true>
SYS_INP_InputListener inputListener_emwin;
GUI_PID_STATE * pidState = NULL;

static void touchDownHandler_emwin(const SYS_INP_TouchStateEvent* const evt);
static void touchUpHandler_emwin(const SYS_INP_TouchStateEvent* const evt);
static void touchMoveHandler_emwin(const SYS_INP_TouchMoveEvent* const evt);
</#if>

<#if CONFIG_USE_SYS_TOUCH_NEEDED>
static TOUCH_MSG_OBJ * pMessage;
</#if>
/*******************************************************************************/
/*  Function:
    void emWin_TouchInitialize ( void )

  Remarks:
    See prototype in app.h.
 */
 <#if CONFIG_USE_SYS_INPUT == true>
 void emWin_TouchInitialize( )
 </#if>
 <#if CONFIG_USE_SYS_TOUCH_NEEDED>
void emWin_TouchInitialize( const SYS_MODULE_INIT * const init )
</#if>
{
    <#if CONFIG_USE_SYS_TOUCH_NEEDED>
    if( NULL == init )
    {
        return;
    }
    </#if>

    <#if CONFIG_USE_SYS_INPUT == true>
    pidState = &emWinTouchData.pidState;
    inputListener_emwin.handleTouchDown = &touchDownHandler_emwin;
    inputListener_emwin.handleTouchUp = &touchUpHandler_emwin;
    inputListener_emwin.handleTouchMove = &touchMoveHandler_emwin;
    </#if>

    emWinTouchData.layerIndex    = 0;
    emWinTouchData.orientation   = EMWIN_TOUCH_ORIENTATION_0_DEGREE;
    emWinTouchData.displayWidth  = ${CONFIG_DRV_GFX_DISPLAY_WIDTH};
    emWinTouchData.displayHeight = ${CONFIG_DRV_GFX_DISPLAY_HEIGHT};
    emWinTouchData.touchState = EMWIN_TOUCH_STATE_INIT;
    return;
}

// *****************************************************************************
// *****************************************************************************
// Section: Local Functions
// *****************************************************************************
// *****************************************************************************

<#if CONFIG_USE_SYS_TOUCH_NEEDED>
/*******************************************************************************/

/*  Function:
    void emWin_TouchMailBoxCreate ( void )

  Remarks:
    See prototype in app.h.
*/

void emWin_TouchMailBoxCreate( void )
{
    pMessage = SYS_TOUCH_DrvObjGet(SYS_TOUCH_INDEX_0);
        if(pMessage!= NULL )
        {
            emWinTouchData.touchState = EMWIN_TOUCH_STATE_PROCESS;
        }
    return;
}
</#if>
/*******************************************************************************
  Function:
    void emWin_TouchLayerIndexSet ( void )

  Remarks:
    See prototype in app.h.
*/
void emWin_TouchLayerIndexSet( uint32_t layerIndex )
{
    emWinTouchData.layerIndex = layerIndex;
}

/*******************************************************************************
  Function:
    void emWin_TouchOrientationSet ( void )

  Remarks:
    See prototype in app.h.
*/
void emWin_TouchOrientationSet( EMWIN_TOUCH_ORIENTATION orientation )
{
    emWinTouchData.orientation = orientation;
}

/*******************************************************************************
  Function:
    void emWin_TouchResolutionSet ( void )

  Remarks:
    See prototype in app.h.
*/
void emWin_TouchResolutionSet( uint32_t displayWidth, uint32_t displayHeight )
{
    emWinTouchData.displayWidth  = displayWidth;
    emWinTouchData.displayHeight = displayHeight;
}

<#if CONFIG_USE_SYS_TOUCH_NEEDED>
/*******************************************************************************
  Function:
    void _emWin_TouchMessageCallback ( void )

  Remarks:
    See prototype in app.h.
*/

static void _emWin_TouchMessageCallback( TOUCH_MSG_OBJ *pMsg )
{
    GUI_PID_STATE * pidState = NULL;

    if( NULL == pMsg )
    {
        return;
    }

    if( TYPE_TOUCHSCREEN != pMsg->nMessageTypeID )
    {
        return;
    }

    pidState = &emWinTouchData.pidState;

    switch( pMsg->param0 )
    {
        case EVENT_PRESS:
        {
            pidState->Pressed = 1;
            break;
        }

        case EVENT_RELEASE:
        {
            pidState->Pressed = 0;
            break;
        }

        default:
            return;
    }

    pidState->Layer = emWinTouchData.layerIndex;

    switch( emWinTouchData.orientation )
    {
        case EMWIN_TOUCH_ORIENTATION_0_DEGREE:
        {
            pidState->x = pMsg->param1;
            pidState->y = pMsg->param2;
            break;
        }

        case EMWIN_TOUCH_ORIENTATION_90_DEGREE:
        {
            pidState->x = emWinTouchData.displayHeight - pMsg->param2 - 1;
            pidState->y = pMsg->param1;
            break;
        }

        case EMWIN_TOUCH_ORIENTATION_180_DEGREE:
        {
            pidState->x = emWinTouchData.displayWidth  - pMsg->param1 - 1;
            pidState->y = emWinTouchData.displayHeight - pMsg->param2 - 1;
            break;
        }

        case EMWIN_TOUCH_ORIENTATION_270_DEGREE:
        {
            pidState->x = pMsg->param2;
            pidState->y = emWinTouchData.displayWidth - pMsg->param1 - 1;
            break;
        }
    }

    GUI_TOUCH_StoreStateEx( pidState );

    return;
}
</#if>
/*******************************************************************************/
/*
  Function:
    void emWin_TouchTasks ( void )

  Remarks:
    See prototype in app.h.
*/
void emWin_TouchTasks(void)
{
    /* Check the application's current state. */
    switch ( emWinTouchData.touchState)
    {

        case EMWIN_TOUCH_STATE_INIT:
            <#if CONFIG_USE_SYS_INPUT == true>
            SYS_INP_AddListener(&inputListener_emwin);
            emWinTouchData.touchState = EMWIN_TOUCH_STATE_PROCESS;
            </#if>
            <#if CONFIG_USE_SYS_TOUCH_NEEDED>
            emWin_TouchMailBoxCreate();
            </#if>
            break;

        case EMWIN_TOUCH_STATE_PROCESS:
            <#if CONFIG_USE_SYS_TOUCH_NEEDED>
            _emWin_TouchMessageCallback(pMessage);
            </#if>
            break;

        default:
            break;
    }

}

<#if CONFIG_USE_SYS_INPUT == true>
void touchDownHandler_emwin(const SYS_INP_TouchStateEvent* const evt)
{

    pidState->Pressed = 1;


    pidState->Layer = emWinTouchData.layerIndex;

    switch( emWinTouchData.orientation )
    {
        case EMWIN_TOUCH_ORIENTATION_0_DEGREE:
        {
            pidState->x = evt->x;
            pidState->y = evt->y;
            break;
        }

        case EMWIN_TOUCH_ORIENTATION_90_DEGREE:
        {
            pidState->x = emWinTouchData.displayHeight - evt->x - 1;
            pidState->y = evt->y;
            break;
        }

        case EMWIN_TOUCH_ORIENTATION_180_DEGREE:
        {
            pidState->x = emWinTouchData.displayWidth  - evt->x;
            pidState->y = emWinTouchData.displayHeight - evt->y;
            break;
        }

        case EMWIN_TOUCH_ORIENTATION_270_DEGREE:
        {
            pidState->x = evt->x;
            pidState->y = emWinTouchData.displayHeight - evt->y - 1;
            break;
        }
    }

    GUI_TOUCH_StoreStateEx( pidState );
}

void touchUpHandler_emwin(const SYS_INP_TouchStateEvent* const evt)
{
    pidState->Pressed = 0;


    pidState->Layer = emWinTouchData.layerIndex;

    switch( emWinTouchData.orientation )
    {
        case EMWIN_TOUCH_ORIENTATION_0_DEGREE:
        {
            pidState->x = evt->x;
            pidState->y = evt->y;
            break;
        }

        case EMWIN_TOUCH_ORIENTATION_90_DEGREE:
        {
            pidState->x = emWinTouchData.displayHeight - evt->x - 1;
            pidState->y = evt->y;
            break;
        }

        case EMWIN_TOUCH_ORIENTATION_180_DEGREE:
        {
            pidState->x = emWinTouchData.displayWidth  - evt->x;
            pidState->y = emWinTouchData.displayHeight - evt->y;
            break;
        }

        case EMWIN_TOUCH_ORIENTATION_270_DEGREE:
        {
            pidState->x = evt->x;
            pidState->y = emWinTouchData.displayHeight - evt->y - 1;
            break;
        }
    }

    GUI_TOUCH_StoreStateEx( pidState );

}

void touchMoveHandler_emwin(const SYS_INP_TouchMoveEvent* const evt)
{
    GUI_TOUCH_StoreStateEx( pidState );
}

</#if>
/*******************************************************************************
 End of File
 */



