/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Implementation File

  File Name:
    libaria_harmony.c

  Summary:
    Build-time generated implementation from the MPLAB Harmony
    Graphics Composer.

  Description:
    Build-time generated implementation from the MPLAB Harmony
    Graphics Composer.

    Created with MPLAB Harmony Version ${CONFIG_MPLAB_HARMONY_VERSION_STRING}
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "gfx/libaria/libaria_harmony.h"
#include "gfx/libaria/libaria_init.h"

#include "gfx/libaria/libaria.h"
<#if CONFIG_LIBARIA_DEMO_MODE_ENABLED?? && CONFIG_LIBARIA_DEMO_MODE_ENABLED == true>
#include "gfx/libaria/libaria_demo_mode.h"
</#if>

<#if CONFIG_USE_LIBARIA_RTOS_EXTENSIONS == true>
#include "gfx/libaria/inc/libaria_context_rtos.h"
#include "gfx/libaria/inc/libaria_input_rtos.h"
#include "gfx/libaria/libaria_rtos.h"
</#if>

<#if CONFIG_LIBARIA_USE_SYSINPUT?? && CONFIG_LIBARIA_USE_SYSINPUT == true>
SYS_INP_InputListener inputListener;

static void touchDownHandler(const SYS_INP_TouchStateEvent* const evt);
static void touchUpHandler(const SYS_INP_TouchStateEvent* const evt);
static void touchMoveHandler(const SYS_INP_TouchMoveEvent* const evt);
</#if>

/*** libaria Object Global ***/
libaria_objects libariaObj;
static LIBARIA_STATES libariaState;

<#if CONFIG_LIBARIA_GENERATE_TOUCH?? && CONFIG_LIBARIA_GENERATE_TOUCH == true>
/*** Message System Service Globals ***/
TOUCH_MSG_OBJ *pMessage;

void LibAria_TouchMessageCallback(TOUCH_MSG_OBJ *pMsg);

</#if>
<#if CONFIG_LIBARIA_MEMORY_INTERFACE?? && CONFIG_LIBARIA_MEMORY_INTERFACE == true>
GFXU_MemoryIntf memIntf;

static GFX_Result LibAria_MediaOpenRequest(GFXU_AssetHeader* asset);

static GFX_Result LibAria_MediaReadRequest(GFXU_ExternalAssetReader* reader,
                                           GFXU_AssetHeader* asset,
                                           void* address,
                                           uint32_t readSize,
                                           uint8_t* destBuffer,
                                           GFXU_MediaReadRequestCallback_FnPtr cb);

static void LibAria_MediaCloseRequest(GFXU_AssetHeader* asset);

<#if CONFIG_LIBARIA_MEDIA_OPEN_FUNCTION?? && CONFIG_LIBARIA_MEDIA_OPEN_FUNCTION?has_content>
GFX_Result ${CONFIG_LIBARIA_MEDIA_OPEN_FUNCTION}(GFXU_AssetHeader* asset);
</#if>

<#if CONFIG_LIBARIA_MEDIA_READ_FUNCTION?? && CONFIG_LIBARIA_MEDIA_READ_FUNCTION?has_content>
GFX_Result ${CONFIG_LIBARIA_MEDIA_READ_FUNCTION}(GFXU_ExternalAssetReader* reader,
        GFXU_AssetHeader* asset,
        void* address,
        uint32_t readSize,
        uint8_t* destBuffer,
        GFXU_MediaReadRequestCallback_FnPtr cb);
</#if>

<#if CONFIG_LIBARIA_MEDIA_CLOSE_FUNCTION?? && CONFIG_LIBARIA_MEDIA_CLOSE_FUNCTION?has_content>
void ${CONFIG_LIBARIA_MEDIA_CLOSE_FUNCTION}(GFXU_AssetHeader* asset);
</#if>

</#if>
int32_t LibAria_Initialize(void)
{
    if(laInitialize() == LA_FAILURE)
        return -1;

<#if CONFIG_LIBARIA_MEMORY_INTERFACE?? && CONFIG_LIBARIA_MEMORY_INTERFACE == true>
    memIntf.heap.malloc = &malloc;
    memIntf.heap.coherent_alloc = &__pic32_alloc_coherent;
    memIntf.heap.calloc = &calloc;
    memIntf.heap.free = &free;

    memIntf.heap.coherent_free = &__pic32_free_coherent;

    memIntf.heap.memcpy = &memcpy;
    memIntf.heap.memset = &memset;
    memIntf.heap.realloc = &realloc;
    memIntf.open = &LibAria_MediaOpenRequest;
    memIntf.read = &LibAria_MediaReadRequest;
    memIntf.close = &LibAria_MediaCloseRequest;

<#if CONFIG_LIBARIA_CONTEXT_COLOR_MODE??>
    libariaObj.context = laContext_Create(0, 0, 0, ${CONFIG_LIBARIA_CONTEXT_COLOR_MODE}, &memIntf);
<#else>
    libariaObj.context = laContext_Create(0, 0, 0, GFX_COLOR_MODE_RGB_565, &memIntf);
</#if>

<#else>
<#if CONFIG_LIBARIA_CONTEXT_COLOR_MODE??>
    libariaObj.context = laContext_Create(0, 0, 0, ${CONFIG_LIBARIA_CONTEXT_COLOR_MODE}, NULL);
<#else>
    libariaObj.context = laContext_Create(0, 0, 0, GFX_COLOR_MODE_RGB_565, NULL);
</#if>

</#if>
    if(libariaObj.context == NULL)
        return -1;

    laContext_SetActive(libariaObj.context);

<#if CONFIG_USE_LIBARIA_RTOS_EXTENSIONS == true>
    laContext_Create_RTOS(libariaObj.context);
</#if>

<#if CONFIG_MHGC_ENABLE?? && CONFIG_MHGC_ENABLE == true>
    libaria_initialize(); // use auto-generated initialization functions
</#if>

<#if CONFIG_LIBARIA_USE_SYSINPUT?? && CONFIG_LIBARIA_USE_SYSINPUT == true>
    inputListener.handleTouchDown = &touchDownHandler;
    inputListener.handleTouchUp = &touchUpHandler;
    inputListener.handleTouchMove = &touchMoveHandler;
</#if>

    libariaState = LIBARIA_STATE_INIT;

    return 0;
}

void LibAria_Tasks(void)
{
    switch(libariaState)
    {
        case LIBARIA_STATE_INIT:
        {
<#if CONFIG_LIBARIA_GENERATE_TOUCH?? && CONFIG_LIBARIA_GENERATE_TOUCH == true>
            pMessage = SYS_TOUCH_DrvObjGet(SYS_TOUCH_INDEX_0);
            if(pMessage!= NULL )
            {
                libariaState = LIBARIA_STATE_RUNNING;
            }
            break;
<#elseif CONFIG_LIBARIA_USE_SYSINPUT?? && CONFIG_LIBARIA_USE_SYSINPUT == true>
            SYS_INP_AddListener(&inputListener);

            libariaState = LIBARIA_STATE_RUNNING;

            break;
<#else>
            libariaState = LIBARIA_STATE_RUNNING;
            break;
</#if>
        }
        case LIBARIA_STATE_RUNNING:
        {
            laContext_SetActive(libariaObj.context);

            <#if CONFIG_LIBARIA_DEMO_MODE_ENABLED?? && CONFIG_LIBARIA_DEMO_MODE_ENABLED == true>
            LibAria_DemoModeProcessEvents();
            </#if>

            <#if CONFIG_LIBARIA_GENERATE_TOUCH?? && CONFIG_LIBARIA_GENERATE_TOUCH == true>
            LibAria_TouchMessageCallback(pMessage);
            </#if>

<#if CONFIG_USE_LIBARIA_RTOS_EXTENSIONS == true>
<#if CONFIG_LIBARIA_RTOS_FULL_BLOCKING == true>
            laUpdate_RTOS(LA_TRUE, 0);
<#else>
            laUpdate_RTOS(LA_FALSE, 0);
</#if>
<#else>
            laUpdate(0);
</#if>

            break;
        }

        default:
        {
            break;
        }
    }
}

<#if CONFIG_LIBARIA_GENERATE_TOUCH?? && CONFIG_LIBARIA_GENERATE_TOUCH == true>
void LibAria_TouchMessageCallback(TOUCH_MSG_OBJ *pMsg)
{
    if(pMsg->nMessageTypeID == TYPE_TOUCHSCREEN)
    {
        if(pMsg->param0 == EVENT_PRESS)
        {
<#if CONFIG_USE_LIBARIA_RTOS_EXTENSIONS == true>
            laInput_SendTouchDown_Ext_RTOS(0, pMsg->param1, pMsg->param2);
<#else>
            laInput_InjectTouchDown(0, pMsg->param1, pMsg->param2);
</#if>

<#if CONFIG_LIBARIA_DEMO_MODE_RECORD?? && CONFIG_LIBARIA_DEMO_MODE_RECORD == true>
            LibAria_DemoModeRecordInputEvent(DEMO_MODE_INPUT_PRESS,
                                             pMsg->param1,
                                             pMsg->param2);
</#if>
<#if CONFIG_LIBARIA_DEMO_MODE_ENABLED?? && CONFIG_LIBARIA_DEMO_MODE_ENABLED == true>
            LibAria_DemoModeSendEvent(DEMO_EVENT_INPUT);
</#if>
        }
        else if(pMsg->param0 == EVENT_RELEASE)
        {

<#if CONFIG_USE_LIBARIA_RTOS_EXTENSIONS == true>
            laInput_SendTouchUp_Ext_RTOS(0, pMsg->param1, pMsg->param2);
<#else>
            laInput_InjectTouchUp(0, pMsg->param1, pMsg->param2);
</#if>

<#if CONFIG_LIBARIA_DEMO_MODE_RECORD?? && CONFIG_LIBARIA_DEMO_MODE_RECORD == true>
            LibAria_DemoModeRecordInputEvent(DEMO_MODE_INPUT_RELEASE,
                                             pMsg->param1,
                                             pMsg->param2);
</#if>
<#if CONFIG_LIBARIA_DEMO_MODE_ENABLED?? && CONFIG_LIBARIA_DEMO_MODE_ENABLED == true>
            LibAria_DemoModeSendEvent(DEMO_EVENT_INPUT);
</#if>
        }
        else if(pMsg->param0 == EVENT_MOVE)
        {
<#if CONFIG_USE_LIBARIA_RTOS_EXTENSIONS == true>
            laInput_SendTouchMoved_Ext_RTOS(0, pMsg->param1, pMsg->param2);
<#else>
            laInput_InjectTouchMoved(0, pMsg->param1, pMsg->param2);
</#if>

<#if CONFIG_LIBARIA_DEMO_MODE_RECORD?? && CONFIG_LIBARIA_DEMO_MODE_RECORD == true>
            LibAria_DemoModeRecordInputEvent(DEMO_MODE_INPUT_MOVE,
                                             pMsg->param1,
                                             pMsg->param2);
</#if>
<#if CONFIG_LIBARIA_DEMO_MODE_ENABLED?? && CONFIG_LIBARIA_DEMO_MODE_ENABLED == true>
            LibAria_DemoModeSendEvent(DEMO_EVENT_INPUT);
</#if>
        }
    }
    else if(pMsg->nMessageTypeID == TYPE_KEYBOARD)
    {

    }
    else if(pMsg->nMessageTypeID == TYPE_MOUSE)
    {

    }
}

</#if>

<#if CONFIG_LIBARIA_USE_SYSINPUT?? && CONFIG_LIBARIA_USE_SYSINPUT == true>
void touchDownHandler(const SYS_INP_TouchStateEvent* const evt)
{
<#if CONFIG_USE_LIBARIA_RTOS_EXTENSIONS == true>
    laInput_SendTouchDown_Ext_RTOS(evt->index, evt->x, evt->y);
<#else>
    laInput_InjectTouchDown(evt->index, evt->x, evt->y);
</#if>
}

void touchUpHandler(const SYS_INP_TouchStateEvent* const evt)
{
<#if CONFIG_USE_LIBARIA_RTOS_EXTENSIONS == true>
    laInput_SendTouchUp_Ext_RTOS(evt->index, evt->x, evt->y);
<#else>
    laInput_InjectTouchUp(evt->index, evt->x, evt->y);
</#if>
}

void touchMoveHandler(const SYS_INP_TouchMoveEvent* const evt)
{
<#if CONFIG_USE_LIBARIA_RTOS_EXTENSIONS == true>
    laInput_SendTouchMoved_Ext_RTOS(evt->index, evt->x, evt->y);
<#else>
    laInput_InjectTouchMoved(evt->index, evt->x, evt->y);
</#if>
}
</#if>

<#if CONFIG_LIBARIA_MEMORY_INTERFACE?? && CONFIG_LIBARIA_MEMORY_INTERFACE == true>

static GFX_Result LibAria_MediaOpenRequest(GFXU_AssetHeader* asset)
{
<#if CONFIG_LIBARIA_MEDIA_OPEN_FUNCTION?? && CONFIG_LIBARIA_MEDIA_OPEN_FUNCTION?has_content>
    return ${CONFIG_LIBARIA_MEDIA_OPEN_FUNCTION}(asset);
<#else>
    return GFX_FAILURE;
</#if>
}

static GFX_Result LibAria_MediaReadRequest(GFXU_ExternalAssetReader* reader,
                                           GFXU_AssetHeader* asset,
                                           void* address,
                                           uint32_t readSize,
                                           uint8_t* destBuffer,
                                           GFXU_MediaReadRequestCallback_FnPtr cb)
{
<#if CONFIG_LIBARIA_MEDIA_READ_FUNCTION?? && CONFIG_LIBARIA_MEDIA_READ_FUNCTION?has_content>
    return ${CONFIG_LIBARIA_MEDIA_READ_FUNCTION}(reader, asset, address, readSize, destBuffer, cb);
<#else>
    return GFX_FAILURE;
</#if>
}

static void LibAria_MediaCloseRequest(GFXU_AssetHeader* asset)
{
<#if CONFIG_LIBARIA_MEDIA_CLOSE_FUNCTION?? && CONFIG_LIBARIA_MEDIA_CLOSE_FUNCTION?has_content>
    ${CONFIG_LIBARIA_MEDIA_CLOSE_FUNCTION}(asset);
</#if>
}

</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
