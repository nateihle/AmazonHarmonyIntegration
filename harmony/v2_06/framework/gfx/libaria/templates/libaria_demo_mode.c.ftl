/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Implementation File

  File Name:
    libaria_demo_mode.c

  Summary:
    Build-time generated implementation of the graphics demo mode.

  Description:
    Build-time generated implementation of the graphics demo mode.

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
#include "gfx/libaria/libaria_demo_mode.h"
#include "gfx/libaria/libaria.h"

typedef enum
{
    DEMO_TASK_INIT = 0,
    DEMO_INIT,
    DEMO_RECORDING,
    DEMO_IDLE,            
    DEMO_STARTING,
    DEMO_RUNNING,
    DEMO_RESTARTING,
    DEMO_STOPPING,
    DEMO_STOPPED,
} LIBARIA_DEMO_MODE_STATE;

typedef struct 
{
    DEMO_MODE_INPUT_TYPE touchEvent;
    uint32_t delayMSECS;
    int32_t index;
    int32_t x;
    int32_t y;
} LIBARIA_DEMO_EVENT_t;

typedef struct
{
    LIBARIA_DEMO_MODE_STATE state;
<#if CONFIG_LIBARIA_DEMO_MODE_RECORD == true>
    laBool recordEnabled;
</#if>
    uint32_t prevEventTick;
    volatile uint32_t recordTicks;
    volatile uint32_t demoEventFlags;
    uint32_t idleTimeOutMSECS;
    uint32_t numEvents;
    uint32_t maxEvents;
    int32_t pendingEvent;
    SYS_TMR_HANDLE demoRunTimer;
    SYS_TMR_HANDLE demoTimeoutTimer;
    SYS_TMR_HANDLE recordTickTimer;
    LIBARIA_DEMO_EVENT_t demoEvents[MAX_DEMO_EVENTS];
} LIBARIA_DEMO_EVENTS_t;

static LIBARIA_DEMO_EVENTS_t demoModeEvents;
<#if CONFIG_LIBARIA_USE_SYSINPUT?? && CONFIG_LIBARIA_USE_SYSINPUT == true>
static SYS_INP_InputListener inputListener;
</#if>

static void LibAria_RestartDemoModeTimerCallback(uintptr_t context, uint32_t currTick)
{
    LibAria_DemoModeSendEvent(DEMO_EVENT_START);
}
static void LibAria_DemoModeRunTimerCallback(uintptr_t context, uint32_t currTick)
{
    LibAria_DemoModeSendEvent(DEMO_EVENT_NEXT_EVENT);
}

static void LibAria_DemoModeStartTimerCallback (uintptr_t context, uint32_t currTick)
{
    LibAria_DemoModeSendEvent(DEMO_EVENT_START);
}

<#if CONFIG_LIBARIA_DEMO_MODE_RECORD == true>
static void LibAria_DemoModeRecordTickTimerCallback (uintptr_t context, uint32_t currTick)
{
    demoModeEvents.recordTicks++;
}
</#if>

void LibAria_DemoModeSendEvent(uint32_t event)
{
    demoModeEvents.demoEventFlags |= (event);
}

void LibAria_DemoModeAddInputEvent(uint32_t dt_ms,
                                   DEMO_MODE_INPUT_TYPE te,
                                   int32_t index,
                                   int32_t x,
                                   int32_t y)
{
    if (demoModeEvents.numEvents < demoModeEvents.maxEvents)
    {
        dt_ms = (dt_ms == 0) ? 1 : dt_ms;
        demoModeEvents.demoEvents[demoModeEvents.numEvents].delayMSECS = dt_ms;
        demoModeEvents.demoEvents[demoModeEvents.numEvents].touchEvent = te;
        demoModeEvents.demoEvents[demoModeEvents.numEvents].index = index;
        demoModeEvents.demoEvents[demoModeEvents.numEvents].x = x;
        demoModeEvents.demoEvents[demoModeEvents.numEvents].y = y;
        demoModeEvents.numEvents++;
    }
}

void LibAria_DemoModeRecordInputEvent(DEMO_MODE_INPUT_TYPE te,
                                      int32_t index,
                                      int32_t x,
                                      int32_t y)
{
<#if CONFIG_LIBARIA_DEMO_MODE_RECORD == true>
    if (demoModeEvents.state == DEMO_RECORDING)
    {
        LibAria_DemoModeAddInputEvent(((demoModeEvents.recordTicks - 
                                  demoModeEvents.prevEventTick) * 
                                  RECORD_TICK_PERIOD_MS),
                                  te,
                                  index,
                                  x, 
                                  y);
        
        demoModeEvents.prevEventTick = demoModeEvents.recordTicks;
    }
</#if>
}

static void LibAria_LoadDefaultEvents(void)
{
    // Add default input events here. These will be loaded if no events were
    // recorded.
    // ex. LibAria_DemoModeAddInputEvent(delay, index, event, x, y);
}

<#if CONFIG_LIBARIA_USE_SYSINPUT?? && CONFIG_LIBARIA_USE_SYSINPUT == true>
void LibAria_DemoModeTouchDownHandler(const SYS_INP_TouchStateEvent* const evt)
{
<#if CONFIG_LIBARIA_DEMO_MODE_RECORD == true>
    LibAria_DemoModeRecordInputEvent(DEMO_MODE_INPUT_PRESS, evt->index, evt->x, evt->y);
</#if>
    LibAria_DemoModeSendEvent(DEMO_EVENT_INPUT);
}

void LibAria_DemoModeTouchUpHandler(const SYS_INP_TouchStateEvent* const evt)
{
<#if CONFIG_LIBARIA_DEMO_MODE_RECORD == true>
    LibAria_DemoModeRecordInputEvent(DEMO_MODE_INPUT_RELEASE, evt->index, evt->x, evt->y);
</#if>
    LibAria_DemoModeSendEvent(DEMO_EVENT_INPUT);
}

void LibAria_DemoModeTouchMoveHandler(const SYS_INP_TouchMoveEvent* const evt)
{
<#if CONFIG_LIBARIA_DEMO_MODE_RECORD == true>
    LibAria_DemoModeRecordInputEvent(DEMO_MODE_INPUT_MOVE, evt->index, evt->x, evt->y);
</#if>
    LibAria_DemoModeSendEvent(DEMO_EVENT_INPUT);
}
</#if>

void LibAria_DemoModeProcessEvents(void)
{
    switch (demoModeEvents.state) 
    {
        case DEMO_INIT:
        {
            demoModeEvents.numEvents = 0; 
<#if CONFIG_LIBARIA_DEMO_MODE_RECORD == true>
            demoModeEvents.recordEnabled = LA_TRUE;
</#if>
            demoModeEvents.idleTimeOutMSECS = DEMO_IDLE_TIMEOUT_S*1000;
            demoModeEvents.maxEvents = MAX_DEMO_EVENTS;
            demoModeEvents.pendingEvent = 0;
            demoModeEvents.demoEventFlags = 0;

<#if CONFIG_LIBARIA_DEMO_MODE_RECORD == true>
            // Recording is enabled
            if (demoModeEvents.recordEnabled == LA_TRUE) 
            {
                //Start recording tick timer
                demoModeEvents.recordTickTimer = SYS_TMR_CallbackPeriodic(
                        RECORD_TICK_PERIOD_MS,
                        (uintptr_t) & demoModeEvents,
                        LibAria_DemoModeRecordTickTimerCallback);
                
                demoModeEvents.state = DEMO_RECORDING;
            }
            else
            {
                demoModeEvents.state = DEMO_IDLE;
            }
<#else>
            demoModeEvents.state = DEMO_IDLE;
</#if>
            
            //Start timeout timer
            demoModeEvents.demoTimeoutTimer = SYS_TMR_CallbackSingle(
                                                 demoModeEvents.idleTimeOutMSECS,
                                                 (uintptr_t) & demoModeEvents,
                                                 LibAria_DemoModeStartTimerCallback);
            break;
        }
<#if CONFIG_LIBARIA_DEMO_MODE_RECORD == true>
        case DEMO_RECORDING:
        {
            // idle timeout timer triggered, stop record tick timer
            // and start demo
            if (demoModeEvents.demoEventFlags & DEMO_EVENT_START)
            {
                SYS_TMR_ObjectDelete(demoModeEvents.recordTickTimer);
                demoModeEvents.demoEventFlags &= ~DEMO_EVENT_START;
                demoModeEvents.state = DEMO_STARTING;
            }
            
            if (demoModeEvents.demoEventFlags & DEMO_EVENT_INPUT)
            {
                demoModeEvents.demoEventFlags = 0;
                
                //Restart the idle timeout timer
                SYS_TMR_ObjectDelete(demoModeEvents.demoTimeoutTimer);
                demoModeEvents.demoTimeoutTimer = SYS_TMR_CallbackSingle(
                        demoModeEvents.idleTimeOutMSECS,
                        (uintptr_t) & demoModeEvents,
                        LibAria_DemoModeStartTimerCallback);
                
                demoModeEvents.state = DEMO_RECORDING;
            }

            // Events list is full, switch to idle
            if (demoModeEvents.numEvents >= demoModeEvents.maxEvents)
            {
                SYS_TMR_ObjectDelete(demoModeEvents.recordTickTimer);
                demoModeEvents.state = DEMO_IDLE;
            }
            
            break;
        }
</#if>
        case DEMO_IDLE:
        {
            if (demoModeEvents.demoEventFlags & DEMO_EVENT_START)
            {
                // idle timeout timer triggered, start demo
                demoModeEvents.demoEventFlags &= ~DEMO_EVENT_START;
                demoModeEvents.state = DEMO_STARTING;
            }
            
            if (demoModeEvents.demoEventFlags & DEMO_EVENT_INPUT)
            {
                demoModeEvents.demoEventFlags &= ~DEMO_EVENT_INPUT;
                
                //Restart the idle timeout timer
                SYS_TMR_ObjectDelete(demoModeEvents.demoTimeoutTimer);
                demoModeEvents.demoTimeoutTimer = SYS_TMR_CallbackSingle(
                                        demoModeEvents.idleTimeOutMSECS,
                                        (uintptr_t) &demoModeEvents,
                                        LibAria_DemoModeStartTimerCallback);
                
                demoModeEvents.state = DEMO_IDLE;
            }
            
            if (demoModeEvents.demoEventFlags & DEMO_EVENT_STOP)
            {
                demoModeEvents.demoEventFlags &= ~DEMO_EVENT_STOP;
                demoModeEvents.state = DEMO_STOPPED;
            }
            break;
        }
        case DEMO_STARTING:
        {
            LIBARIA_DEMO_EVENT_t * currEvent;
            
            if (demoModeEvents.numEvents == 0) 
            {
                LibAria_LoadDefaultEvents();
            }

            // START Demo Mode
            if (demoModeEvents.numEvents > 0) 
            {
                demoModeEvents.pendingEvent = 0;
                
                // Initialize app and switch to first screen
			<#if CONFIG_APP_IDX_0?has_content>
				${CONFIG_APP_NAME_0?upper_case}_Initialize();
			<#else>
				APP_Initialize();
			</#if>
                laContext_SetActiveScreen(0);

                currEvent = &demoModeEvents.demoEvents[0];

                SYS_TMR_ObjectDelete(demoModeEvents.demoRunTimer);
                demoModeEvents.demoRunTimer = SYS_TMR_CallbackSingle(
                                                currEvent->delayMSECS,
                                                (uintptr_t) &demoModeEvents,
                                                LibAria_DemoModeRunTimerCallback);
                
                demoModeEvents.state = DEMO_RUNNING;
            }
            else
            {
                demoModeEvents.state = DEMO_IDLE;
            }

            break;
        }
        case DEMO_STOPPING:
        {
            SYS_TMR_ObjectDelete(demoModeEvents.demoTimeoutTimer);
            SYS_TMR_ObjectDelete(demoModeEvents.demoRunTimer);
            SYS_TMR_ObjectDelete(demoModeEvents.recordTickTimer);
            
            demoModeEvents.state = DEMO_STOPPED;
        }
        case DEMO_STOPPED:
        {
            if (demoModeEvents.demoEventFlags & DEMO_EVENT_RESTART)
            {
                // received start event
                demoModeEvents.demoEventFlags &= ~DEMO_EVENT_RESTART;
                
                demoModeEvents.state = DEMO_INIT;
            }
            break;
        }
        case DEMO_RUNNING:
        {
            if (demoModeEvents.demoEventFlags & DEMO_EVENT_INPUT)
            {
                // Clear all events
                demoModeEvents.demoEventFlags = 0;
                
                //Restart the idle timeout timer
                SYS_TMR_ObjectDelete(demoModeEvents.demoRunTimer);
                SYS_TMR_ObjectDelete(demoModeEvents.demoTimeoutTimer);
                demoModeEvents.demoTimeoutTimer = SYS_TMR_CallbackSingle(
                                        demoModeEvents.idleTimeOutMSECS,
                                        (uintptr_t) &demoModeEvents,
                                        LibAria_DemoModeStartTimerCallback);
                
                demoModeEvents.state = DEMO_IDLE;
            }
            
            if (demoModeEvents.demoEventFlags & DEMO_EVENT_STOP)
            {
                demoModeEvents.demoEventFlags = 0;

                demoModeEvents.state = DEMO_STOPPING;
                break;
            }
            
            if (demoModeEvents.demoEventFlags & DEMO_EVENT_NEXT_EVENT)
            {
                LIBARIA_DEMO_EVENT_t * currEvent, * nextEvent;
                
                demoModeEvents.demoEventFlags &= ~DEMO_EVENT_NEXT_EVENT;

                currEvent = &demoModeEvents.demoEvents[demoModeEvents.pendingEvent];

                switch (currEvent->touchEvent) {
                    case DEMO_MODE_INPUT_PRESS:
                        laInput_InjectTouchDown(currEvent->index, currEvent->x, currEvent->y);
                        break;
                    case DEMO_MODE_INPUT_RELEASE:
                        laInput_InjectTouchUp(currEvent->index, currEvent->x, currEvent->y);
                        break;
                    case DEMO_MODE_INPUT_MOVE:
                        laInput_InjectTouchMoved(currEvent->index, currEvent->x, currEvent->y);
                        break;
                    default:
                        break;
                }

                demoModeEvents.pendingEvent++;

                // End of demo events, restart
                if (demoModeEvents.pendingEvent >= demoModeEvents.numEvents) 
                {
                    demoModeEvents.state = DEMO_RESTARTING;
                    demoModeEvents.demoRunTimer = SYS_TMR_CallbackSingle(
                                                (DEMO_REPEAT_TIMEOUT_S*1000),
                                                (uintptr_t) &demoModeEvents,
                                                LibAria_RestartDemoModeTimerCallback);
                }
                else 
                {
                    nextEvent = &demoModeEvents.demoEvents[demoModeEvents.pendingEvent];
                    SYS_TMR_ObjectDelete(demoModeEvents.demoRunTimer);
                    demoModeEvents.demoRunTimer = SYS_TMR_CallbackSingle(
                                            nextEvent->delayMSECS,
                                            (uintptr_t) & demoModeEvents,
                                            LibAria_DemoModeRunTimerCallback);
                    
                    demoModeEvents.state = DEMO_RUNNING;
                }
            }
            break;
        }
        case DEMO_RESTARTING:
        {
            if (demoModeEvents.demoEventFlags & DEMO_EVENT_START)
            {
                demoModeEvents.demoEventFlags &= ~DEMO_EVENT_START;

                demoModeEvents.state = DEMO_STARTING;
            }

            if (demoModeEvents.demoEventFlags & DEMO_EVENT_INPUT)
            {
                // Clear all events
                demoModeEvents.demoEventFlags = 0;

                //Restart the idle timeout timer
                SYS_TMR_ObjectDelete(demoModeEvents.demoRunTimer);
                SYS_TMR_ObjectDelete(demoModeEvents.demoTimeoutTimer);
                demoModeEvents.demoTimeoutTimer = SYS_TMR_CallbackSingle(
                                        demoModeEvents.idleTimeOutMSECS,
                                        (uintptr_t) &demoModeEvents,
                                        LibAria_DemoModeStartTimerCallback);

                demoModeEvents.state = DEMO_IDLE;
            }

            break;
        }
        //One time demo mode task initialization
        case DEMO_TASK_INIT:
        {
<#if CONFIG_LIBARIA_USE_SYSINPUT?? && CONFIG_LIBARIA_USE_SYSINPUT == true>
            SYS_INP_ListenerInit(&inputListener);

            inputListener.handleTouchDown = &LibAria_DemoModeTouchDownHandler;
            inputListener.handleTouchUp = &LibAria_DemoModeTouchUpHandler;
            inputListener.handleTouchMove = &LibAria_DemoModeTouchMoveHandler;

            SYS_INP_AddListener(&inputListener);
</#if>

            demoModeEvents.state = DEMO_INIT;

            break;
        }
        default:
        {
            break;
        }
    }
}