/*******************************************************************************
 Module for Microchip Graphics Library - Graphic Object Layer

  Company:
    Microchip Technology Inc.

  File Name:
    libaria_context_rtos.c

  Summary:
    This file contains the RTOS extensions of libaria_context.c

  Description:
    This module implements the common routines for the Graphics Object Layer
    of the Microchip Graphics Library. The routines are independent of the
    Display Driver Layer and should be compatible with any Display Driver
    that is compliant with the requirements of the Display Driver
    Layer of the Graphics Library.
    The module utilizes the Graphics Primitive Layer to render the objects.
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
#include "gfx/libaria/inc/libaria_context.h"

#include "gfx/hal/gfx.h"
#include "gfx/utils/gfx_utils.h"

#include "gfx/libaria/inc/libaria_draw.h"
#include "gfx/libaria/inc/libaria_layer.h"
#include "gfx/libaria/inc/libaria_list.h"
#include "gfx/libaria/inc/libaria_screen.h"
#include "gfx/libaria/inc/libaria_utils.h"

#include "gfx/libaria/inc/libaria_event_rtos.h"

extern laContext* _activeContext;

typedef struct laContext_ScreenChangeEvent_t
{
    laEvent event;
    
    uint32_t index;
} laContext_ScreenChangeEvent;

typedef struct laContext_ScreenRefreshEvent_t
{
    laEvent event;
} laContext_ScreenRefreshEvent;

laResult laContext_Create_RTOS(laContext* context)
{
    return laEvent_RTOS_Initialize();
}

laResult laContext_Destroy_RTOS(laContext* context)
{
    return laEvent_RTOS_DeInitialize();
}

laResult laContext_RefreshActiveScreen_RTOS()
{
    laContext_ScreenRefreshEvent * evt;
    
    if(_activeContext == NULL)
        return LA_FAILURE;

    evt = _activeContext->memIntf.heap.malloc(sizeof(laContext_ScreenRefreshEvent));
        
    //Send a dummy event for now
    evt->event.id = LA_EVENT_NONE;;
    
    laEvent_AddEvent_RTOS((laEvent*)evt);

    return LA_SUCCESS;
}

laResult laContext_RefreshActiveScreen_Ext_RTOS()
{
    laContext_ScreenRefreshEvent * evt;
    
    if(_activeContext == NULL)
        return LA_FAILURE;

    evt = _activeContext->memIntf.heap.malloc(sizeof(laContext_ScreenRefreshEvent));
        
    //Send a dummy event for now
    evt->event.id = LA_EVENT_NONE;;
    
    laEvent_SendEvent_Ext_RTOS((laEvent*)evt);

    return LA_SUCCESS;
}

laResult laContext_SetActiveScreen_RTOS(uint32_t idx)
{
    laContext_ScreenChangeEvent* evt;
    
    if(_activeContext == NULL || idx < 0 || idx >= _activeContext->screenList.size)
        return LA_FAILURE;

    if(laContext_GetActiveScreenIndex() == idx)
        return LA_SUCCESS;
    
    evt = _activeContext->memIntf.heap.malloc(sizeof(laContext_ScreenChangeEvent));
        
    evt->event.id = LA_EVENT_SCREEN_CHANGE;
    evt->index = idx;
        
    laEvent_AddEvent_RTOS((laEvent*)evt);
    
    return LA_SUCCESS;
}

laResult laContext_SendSetActiveScreenEvent_Ext_RTOS(uint32_t idx)
{
    laContext_ScreenChangeEvent* evt;
    
    if(_activeContext == NULL || idx < 0 || idx >= _activeContext->screenList.size)
        return LA_FAILURE;

    if(laContext_GetActiveScreenIndex() == idx)
        return LA_SUCCESS;
    
    evt = _activeContext->memIntf.heap.malloc(sizeof(laContext_ScreenChangeEvent));
        
    evt->event.id = LA_EVENT_SCREEN_CHANGE;
    evt->index = idx;
        
    laEvent_SendEvent_Ext_RTOS((laEvent*)evt);
    
    return LA_SUCCESS;
}

void laContext_Update_RTOS(laBool fullBlock, uint32_t dt)
{
    laResult result;
    laContextUpdateState updateState;
    
    if(_activeContext == NULL)
        return;
    
    if (fullBlock == LA_TRUE)
    {
        OSAL_SEM_Pend(&_activeContext->event.eventCountSem, OSAL_WAIT_FOREVER);
    }
    
    do
    {
        OSAL_MUTEX_Lock(&_activeContext->event.eventLock, OSAL_WAIT_FOREVER);
    
        //Process as much events as we can, before we block at the event semaphore
        result = laEvent_ProcessEvents_RTOS();
        
        updateState = _laContext_Update(dt);
        //Context has pending updates for next cycle, send a future refresh event
        if (updateState == LA_CONTEXT_UPDATE_PENDING)
        {
            laContext_RefreshActiveScreen_RTOS();
        }
        
        //TODO: explore possibility of having the paint loop in a separate thread
        while (_activeContext->frameState != LA_CONTEXT_FRAME_READY)
        {
            _laContext_Paint();
        }
        
        OSAL_MUTEX_Unlock(&_activeContext->event.eventLock);
        
    } while (result != LA_FAILURE);
}