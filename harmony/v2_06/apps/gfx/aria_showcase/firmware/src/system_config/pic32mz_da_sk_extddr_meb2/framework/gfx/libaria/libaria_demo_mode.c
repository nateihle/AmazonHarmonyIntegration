/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Implementation File

  File Name:
    libaria_demo_mode.c

  Summary:
    Build-time generated implementation of the graphics demo mode.

  Description:
    Build-time generated implementation of the graphics demo mode.

    Created with MPLAB Harmony Version 2.06
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
static SYS_INP_InputListener inputListener;

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
}

static void LibAria_LoadDefaultEvents(void)
{
    // START OF CUSTOM CODE. DO NOT MODIFY OR REMOVE!!!
    LibAria_DemoModeAddInputEvent(11210, DEMO_MODE_INPUT_PRESS, 0, 44, 124);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_RELEASE, 0, 44, 124);
    LibAria_DemoModeAddInputEvent(3090, DEMO_MODE_INPUT_PRESS, 0, 133, 157);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_MOVE, 0, 131, 155);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0, 129, 140);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 128, 138);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 128, 134);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 128, 127);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 128, 118);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 128, 113);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 130, 95);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 133, 77);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 137, 55);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_RELEASE, 0, 137, 55);
    LibAria_DemoModeAddInputEvent(1270, DEMO_MODE_INPUT_PRESS, 0, 192, 93);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0, 193, 95);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0, 193, 99);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 193, 103);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 193, 107);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 193, 113);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 193, 117);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 193, 123);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 192, 129);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 192, 135);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 192, 141);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 191, 156);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 190, 172);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 190, 181);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 188, 198);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 187, 210);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_RELEASE, 0, 187, 210);
    LibAria_DemoModeAddInputEvent(570, DEMO_MODE_INPUT_PRESS, 0, 260, 153);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0, 260, 150);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0, 261, 146);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 261, 143);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 261, 139);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 261, 135);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 262, 131);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 262, 126);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 262, 116);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 262, 111);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 262, 100);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 262, 89);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 262, 78);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 264, 64);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_RELEASE, 0, 264, 64);
    LibAria_DemoModeAddInputEvent(1020, DEMO_MODE_INPUT_PRESS, 0, 335, 144);
    LibAria_DemoModeAddInputEvent(60, DEMO_MODE_INPUT_MOVE, 0, 334, 141);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0, 333, 135);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 333, 129);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 332, 122);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 332, 114);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 332, 105);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 332, 97);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_RELEASE, 0, 332, 97);
    LibAria_DemoModeAddInputEvent(1320, DEMO_MODE_INPUT_PRESS, 0, 399, 252);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_RELEASE, 0, 399, 252);
    LibAria_DemoModeAddInputEvent(1240, DEMO_MODE_INPUT_PRESS, 0, 172, 175);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0, 176, 173);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 282, 128);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0, 292, 124);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 302, 120);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_MOVE, 0, 311, 116);
    LibAria_DemoModeAddInputEvent(110, DEMO_MODE_INPUT_MOVE, 0, 319, 113);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 327, 111);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 333, 108);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 339, 106);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 344, 104);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 349, 102);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 354, 101);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 358, 99);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 362, 98);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 366, 97);
    LibAria_DemoModeAddInputEvent(0, DEMO_MODE_INPUT_MOVE, 0, 369, 95);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 373, 95);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 375, 94);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 378, 93);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 382, 92);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 384, 91);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 386, 90);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 389, 90);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0, 391, 89);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 393, 88);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0, 389, 90);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 387, 92);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 382, 94);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 377, 97);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 371, 101);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 363, 106);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 353, 111);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 341, 117);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 327, 123);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 312, 130);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 296, 137);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 281, 145);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 266, 152);
    LibAria_DemoModeAddInputEvent(0, DEMO_MODE_INPUT_MOVE, 0, 252, 158);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 239, 164);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 227, 170);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 215, 176);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 205, 181);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 195, 186);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 185, 191);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 176, 195);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 168, 199);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 161, 203);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_RELEASE, 0, 161, 203);
    LibAria_DemoModeAddInputEvent(570, DEMO_MODE_INPUT_PRESS, 0, 398, 255);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_RELEASE, 0, 398, 255);
    LibAria_DemoModeAddInputEvent(1420, DEMO_MODE_INPUT_PRESS, 0, 305, 202);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_RELEASE, 0, 305, 202);
    LibAria_DemoModeAddInputEvent(1310, DEMO_MODE_INPUT_PRESS, 0, 337, 66);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_RELEASE, 0, 337, 66);
    LibAria_DemoModeAddInputEvent(700, DEMO_MODE_INPUT_PRESS, 0, 301, 204);
    LibAria_DemoModeAddInputEvent(110, DEMO_MODE_INPUT_RELEASE, 0, 301, 204);
    LibAria_DemoModeAddInputEvent(510, DEMO_MODE_INPUT_PRESS, 0, 360, 132);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_RELEASE, 0, 360, 132);
    LibAria_DemoModeAddInputEvent(500, DEMO_MODE_INPUT_PRESS, 0, 125, 206);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_RELEASE, 0, 125, 206);
    LibAria_DemoModeAddInputEvent(300, DEMO_MODE_INPUT_PRESS, 0, 169, 136);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_RELEASE, 0, 169, 136);
    LibAria_DemoModeAddInputEvent(620, DEMO_MODE_INPUT_PRESS, 0, 397, 139);
    LibAria_DemoModeAddInputEvent(120, DEMO_MODE_INPUT_RELEASE, 0, 397, 139);
    LibAria_DemoModeAddInputEvent(800, DEMO_MODE_INPUT_PRESS, 0, 119, 205);
    LibAria_DemoModeAddInputEvent(120, DEMO_MODE_INPUT_RELEASE, 0, 119, 205);
    LibAria_DemoModeAddInputEvent(640, DEMO_MODE_INPUT_PRESS, 0, 260, 171);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_RELEASE, 0, 260, 171);
    LibAria_DemoModeAddInputEvent(520, DEMO_MODE_INPUT_PRESS, 0, 350, 134);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_RELEASE, 0, 350, 134);
    LibAria_DemoModeAddInputEvent(500, DEMO_MODE_INPUT_PRESS, 0, 451, 132);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_RELEASE, 0, 451, 132);
    LibAria_DemoModeAddInputEvent(750, DEMO_MODE_INPUT_PRESS, 0, 353, 206);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_RELEASE, 0, 353, 206);
    LibAria_DemoModeAddInputEvent(790, DEMO_MODE_INPUT_PRESS, 0, 174, 135);
    LibAria_DemoModeAddInputEvent(110, DEMO_MODE_INPUT_RELEASE, 0, 174, 135);
    LibAria_DemoModeAddInputEvent(490, DEMO_MODE_INPUT_PRESS, 0, 409, 131);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_RELEASE, 0, 409, 131);
    LibAria_DemoModeAddInputEvent(640, DEMO_MODE_INPUT_PRESS, 0, 122, 208);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_RELEASE, 0, 122, 208);
    LibAria_DemoModeAddInputEvent(490, DEMO_MODE_INPUT_PRESS, 0, 356, 167);
    LibAria_DemoModeAddInputEvent(110, DEMO_MODE_INPUT_RELEASE, 0, 356, 167);
    LibAria_DemoModeAddInputEvent(490, DEMO_MODE_INPUT_PRESS, 0, 78, 174);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_RELEASE, 0, 78, 174);
    LibAria_DemoModeAddInputEvent(2850, DEMO_MODE_INPUT_PRESS, 0, 392, 253);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_RELEASE, 0, 392, 253);
    LibAria_DemoModeAddInputEvent(1760, DEMO_MODE_INPUT_PRESS, 0, 401, 129);
    LibAria_DemoModeAddInputEvent(530, DEMO_MODE_INPUT_MOVE, 0, 400, 127);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0, 400, 124);
    LibAria_DemoModeAddInputEvent(60, DEMO_MODE_INPUT_MOVE, 0, 400, 121);
    LibAria_DemoModeAddInputEvent(50, DEMO_MODE_INPUT_MOVE, 0, 400, 118);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 401, 116);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 401, 113);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 402, 110);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 402, 107);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0, 402, 104);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 402, 101);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0, 403, 98);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0, 403, 95);
    LibAria_DemoModeAddInputEvent(50, DEMO_MODE_INPUT_MOVE, 0, 403, 91);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_MOVE, 0, 403, 88);
    LibAria_DemoModeAddInputEvent(140, DEMO_MODE_INPUT_MOVE, 0, 403, 91);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 403, 94);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 403, 98);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 402, 102);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 401, 105);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 401, 110);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 400, 113);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 400, 118);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 400, 123);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 400, 126);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 400, 129);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 400, 132);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 399, 137);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 399, 140);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 399, 143);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 399, 148);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 399, 151);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 399, 156);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 399, 160);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 399, 165);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 399, 168);
    LibAria_DemoModeAddInputEvent(50, DEMO_MODE_INPUT_MOVE, 0, 399, 171);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0, 399, 174);
    LibAria_DemoModeAddInputEvent(160, DEMO_MODE_INPUT_MOVE, 0, 399, 171);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 399, 168);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 399, 165);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0, 399, 161);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 399, 158);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 399, 155);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 399, 152);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 398, 150);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 398, 147);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 398, 144);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 397, 139);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 397, 135);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 397, 132);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0, 397, 129);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 396, 127);
    LibAria_DemoModeAddInputEvent(60, DEMO_MODE_INPUT_MOVE, 0, 396, 124);
    LibAria_DemoModeAddInputEvent(170, DEMO_MODE_INPUT_RELEASE, 0, 396, 124);
    LibAria_DemoModeAddInputEvent(1020, DEMO_MODE_INPUT_PRESS, 0, 394, 250);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_RELEASE, 0, 394, 250);
    LibAria_DemoModeAddInputEvent(1360, DEMO_MODE_INPUT_PRESS, 0, 459, 127);
    LibAria_DemoModeAddInputEvent(60, DEMO_MODE_INPUT_RELEASE, 0, 459, 127);
    LibAria_DemoModeAddInputEvent(660, DEMO_MODE_INPUT_PRESS, 0, 470, 125);
    LibAria_DemoModeAddInputEvent(120, DEMO_MODE_INPUT_RELEASE, 0, 470, 125);
    LibAria_DemoModeAddInputEvent(1010, DEMO_MODE_INPUT_PRESS, 0, 307, 248);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_RELEASE, 0, 307, 248);
    LibAria_DemoModeAddInputEvent(2720, DEMO_MODE_INPUT_PRESS, 0, 166, 238);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_RELEASE, 0, 166, 238);
    LibAria_DemoModeAddInputEvent(6600, DEMO_MODE_INPUT_PRESS, 0, 191, 240);
    LibAria_DemoModeAddInputEvent(120, DEMO_MODE_INPUT_RELEASE, 0, 191, 240);
    LibAria_DemoModeAddInputEvent(750, DEMO_MODE_INPUT_PRESS, 0, 465, 251);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_RELEASE, 0, 465, 251);
    LibAria_DemoModeAddInputEvent(2540, DEMO_MODE_INPUT_PRESS, 0, 12, 249);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_RELEASE, 0, 12, 249);
    LibAria_DemoModeAddInputEvent(2660, DEMO_MODE_INPUT_PRESS, 0, 459, 28);
    LibAria_DemoModeAddInputEvent(120, DEMO_MODE_INPUT_RELEASE, 0, 459, 28);
    LibAria_DemoModeAddInputEvent(1850, DEMO_MODE_INPUT_PRESS, 0, 463, 254);
    LibAria_DemoModeAddInputEvent(60, DEMO_MODE_INPUT_RELEASE, 0, 463, 254);
    LibAria_DemoModeAddInputEvent(1260, DEMO_MODE_INPUT_PRESS, 0, 310, 56);
    LibAria_DemoModeAddInputEvent(160, DEMO_MODE_INPUT_RELEASE, 0, 310, 56);
    LibAria_DemoModeAddInputEvent(1170, DEMO_MODE_INPUT_PRESS, 0, 464, 249);
    LibAria_DemoModeAddInputEvent(110, DEMO_MODE_INPUT_RELEASE, 0, 464, 249);
    LibAria_DemoModeAddInputEvent(2520, DEMO_MODE_INPUT_PRESS, 0, 16, 249);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_RELEASE, 0, 16, 249);
    LibAria_DemoModeAddInputEvent(1970, DEMO_MODE_INPUT_PRESS, 0, 463, 21);
    LibAria_DemoModeAddInputEvent(120, DEMO_MODE_INPUT_RELEASE, 0, 463, 21);
    LibAria_DemoModeAddInputEvent(1430, DEMO_MODE_INPUT_PRESS, 0, 459, 254);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_RELEASE, 0, 459, 254);
    LibAria_DemoModeAddInputEvent(1240, DEMO_MODE_INPUT_PRESS, 0, 168, 54);
    LibAria_DemoModeAddInputEvent(140, DEMO_MODE_INPUT_MOVE, 0, 171, 54);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_RELEASE, 0, 171, 54);
    LibAria_DemoModeAddInputEvent(830, DEMO_MODE_INPUT_PRESS, 0, 178, 52);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_RELEASE, 0, 178, 52);
    LibAria_DemoModeAddInputEvent(2310, DEMO_MODE_INPUT_PRESS, 0, 467, 249);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_RELEASE, 0, 467, 249);
    // END OF CUSTOM CODE
}

void LibAria_DemoModeTouchDownHandler(const SYS_INP_TouchStateEvent* const evt)
{
    LibAria_DemoModeSendEvent(DEMO_EVENT_INPUT);
}

void LibAria_DemoModeTouchUpHandler(const SYS_INP_TouchStateEvent* const evt)
{
    LibAria_DemoModeSendEvent(DEMO_EVENT_INPUT);
}

void LibAria_DemoModeTouchMoveHandler(const SYS_INP_TouchMoveEvent* const evt)
{
    LibAria_DemoModeSendEvent(DEMO_EVENT_INPUT);
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
            SYS_INP_ListenerInit(&inputListener);
            
            inputListener.handleTouchDown = &LibAria_DemoModeTouchDownHandler;
            inputListener.handleTouchUp = &LibAria_DemoModeTouchUpHandler;
            inputListener.handleTouchMove = &LibAria_DemoModeTouchMoveHandler;      
            
            SYS_INP_AddListener(&inputListener);
            
            demoModeEvents.state = DEMO_INIT;
            
            break;
        }
        default:
        {
            break;
        }
    }
}