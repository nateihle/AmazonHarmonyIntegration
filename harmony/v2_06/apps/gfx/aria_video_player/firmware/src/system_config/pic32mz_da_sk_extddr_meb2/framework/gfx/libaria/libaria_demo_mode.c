/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Implementation File

  File Name:
    libaria_demo_mode.c

  Summary:
    Build-time generated implementation of the graphics demo mode.

  Description:
    Build-time generated implementation of the graphics demo mode.

    Created with MPLAB Harmony Version 2.04
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
    DEMO_INIT = 0,
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
    SYS_INPUT_DEVICE_EVENT touchEvent;
    uint32_t delayMSECS;
    int32_t x;
    int32_t y;
} LIBARIA_DEMO_EVENT_t;

typedef struct
{
    LIBARIA_DEMO_MODE_STATE state;
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


void LibAria_DemoModeSendEvent(uint32_t event)
{
    demoModeEvents.demoEventFlags |= (event);
}

void LibAria_DemoModeAddInputEvent(uint32_t dt_ms,
                                   SYS_INPUT_DEVICE_EVENT te,
                                   int32_t x,
                                   int32_t y)
{
    if (demoModeEvents.numEvents < demoModeEvents.maxEvents)
    {
        dt_ms = (dt_ms == 0) ? 1 : dt_ms;
        demoModeEvents.demoEvents[demoModeEvents.numEvents].delayMSECS = dt_ms;
        demoModeEvents.demoEvents[demoModeEvents.numEvents].touchEvent = te;
        demoModeEvents.demoEvents[demoModeEvents.numEvents].x = x;
        demoModeEvents.demoEvents[demoModeEvents.numEvents].y = y;
        demoModeEvents.numEvents++;
    }
}

void LibAria_DemoModeRecordInputEvent(SYS_INPUT_DEVICE_EVENT te,
                                      int32_t x,
                                      int32_t y)
{
}

static void LibAria_LoadDefaultEvents(void)
{
    // Add default input events here. These will be loaded if no events were
    // recorded.
    // ex. LibAria_DemoModeAddInputEvent(delay, event, x, y);
}

void LibAria_DemoModeProcessEvents(void)
{
    switch (demoModeEvents.state) 
    {
        case DEMO_INIT:
        {
            demoModeEvents.numEvents = 0; 
            demoModeEvents.idleTimeOutMSECS = DEMO_IDLE_TIMEOUT_S*1000;
            demoModeEvents.maxEvents = MAX_DEMO_EVENTS;
            demoModeEvents.pendingEvent = 0;
            demoModeEvents.demoEventFlags = 0;
            demoModeEvents.state = DEMO_IDLE;
            
            //Start timeout timer
            demoModeEvents.demoTimeoutTimer = SYS_TMR_CallbackSingle(
                                                 demoModeEvents.idleTimeOutMSECS,
                                                 (uintptr_t) & demoModeEvents,
                                                 LibAria_DemoModeStartTimerCallback);
            break;
        }
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
				APP_Initialize();
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
                    case EVENT_PRESS:
                        laInput_InjectTouchDown(0, currEvent->x, currEvent->y);
                        break;
                    case EVENT_RELEASE:
                        laInput_InjectTouchUp(0, currEvent->x, currEvent->y);
                        break;
                    case EVENT_MOVE:
                        laInput_InjectTouchMoved(0, currEvent->x, currEvent->y);
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
        default:
        {
            break;
        }
    }
}