/*******************************************************************************
 Module for Microchip Graphics Library - Aria User Interface Library

  Company:
    Microchip Technology Inc.

  File Name:
    libaria_event_rtos.h

  Summary:
    Defines events and APIs that can be safely used for communicating with the UI
    library in an RTOS environment. All RTOS tasks should use these APIs for thread-safe
    operation.

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

#ifndef LIBARIA_EVENT_RTOS_H
#define LIBARIA_EVENT_RTOS_H

#include "gfx/libaria/inc/libaria_common.h"
#include "gfx/libaria/inc/libaria_list.h"

#ifdef __cplusplus
    extern "C" {
#endif

typedef enum
{
    LA_EXT_EVENT_NONE,
    LA_EXT_EVENT_SCREEN_UPDATE,
} laExternalEventID;

typedef struct laExternalEvent_t
{
    laExternalEventID id;
} laExternalEvent;

laResult laEvent_RTOS_Initialize(void);
laResult laEvent_RTOS_DeInitialize(void);
uint32_t laEvent_GetCount_RTOS();
laResult laEvent_SetFilter_RTOS(laEvent_FilterEvent cb);
laResult laEvent_ClearList_RTOS();
laResult laEvent_AddEvent_RTOS(laEvent* evt);
laResult laEvent_SendEvent_Ext_RTOS(laEvent* evt);
laResult laEvent_ProcessEvents_RTOS();

#ifdef __cplusplus
    }
#endif

#endif //LIBARIA_EVENT_RTOS_H