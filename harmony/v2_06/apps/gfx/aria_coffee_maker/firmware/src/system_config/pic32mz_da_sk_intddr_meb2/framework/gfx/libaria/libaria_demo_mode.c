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
    laBool recordEnabled;
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

static void LibAria_DemoModeRecordTickTimerCallback (uintptr_t context, uint32_t currTick)
{
    demoModeEvents.recordTicks++;
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
}

static void LibAria_LoadDefaultEvents(void)
{
    // Add default input events here. These will be loaded if no events were
    // recorded.
    // ex. LibAria_DemoModeAddInputEvent(delay, event, x, y);
    // START OF CUSTOM CODE. DO NOT MODIFY OR REMOVE!!!
    LibAria_DemoModeAddInputEvent(19360, DEMO_MODE_INPUT_PRESS, 0, 127, 232);
    LibAria_DemoModeAddInputEvent(175, DEMO_MODE_INPUT_MOVE, 0, 127, 229);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 129, 228);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 131, 227);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 133, 226);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 144, 224);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 148, 223);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 153, 222);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 174, 219);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 206, 218);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 210, 218);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 214, 218);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 232, 219);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 236, 219);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 239, 219);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 252, 219);
    LibAria_DemoModeAddInputEvent(55, DEMO_MODE_INPUT_MOVE, 0, 257, 219);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 271, 219);
    LibAria_DemoModeAddInputEvent(55, DEMO_MODE_INPUT_MOVE, 0, 274, 220);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0, 278, 220);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0, 282, 222);
    LibAria_DemoModeAddInputEvent(75, DEMO_MODE_INPUT_MOVE, 0, 287, 224);
    LibAria_DemoModeAddInputEvent(130, DEMO_MODE_INPUT_RELEASE, 0, 287, 224);
    LibAria_DemoModeAddInputEvent(1515, DEMO_MODE_INPUT_PRESS, 0, 151, 95);
    LibAria_DemoModeAddInputEvent(105, DEMO_MODE_INPUT_RELEASE, 0, 151, 95);
    LibAria_DemoModeAddInputEvent(435, DEMO_MODE_INPUT_PRESS, 0, 150, 90);
    LibAria_DemoModeAddInputEvent(120, DEMO_MODE_INPUT_RELEASE, 0, 150, 90);
    LibAria_DemoModeAddInputEvent(705, DEMO_MODE_INPUT_PRESS, 0, 155, 88);
    LibAria_DemoModeAddInputEvent(55, DEMO_MODE_INPUT_RELEASE, 0, 155, 88);
    LibAria_DemoModeAddInputEvent(910, DEMO_MODE_INPUT_PRESS, 0, 154, 88);
    LibAria_DemoModeAddInputEvent(55, DEMO_MODE_INPUT_RELEASE, 0, 154, 88);
    LibAria_DemoModeAddInputEvent(1275, DEMO_MODE_INPUT_PRESS, 0, 48, 78);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_RELEASE, 0, 48, 78);
    LibAria_DemoModeAddInputEvent(780, DEMO_MODE_INPUT_PRESS, 0, 44, 83);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_RELEASE, 0, 44, 83);
    LibAria_DemoModeAddInputEvent(770, DEMO_MODE_INPUT_PRESS, 0, 53, 84);
    LibAria_DemoModeAddInputEvent(115, DEMO_MODE_INPUT_RELEASE, 0, 53, 84);
    LibAria_DemoModeAddInputEvent(1095, DEMO_MODE_INPUT_PRESS, 0, 52, 96);
    LibAria_DemoModeAddInputEvent(85, DEMO_MODE_INPUT_RELEASE, 0, 52, 96);
    LibAria_DemoModeAddInputEvent(1035, DEMO_MODE_INPUT_PRESS, 0, 144, 153);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_RELEASE, 0, 144, 153);
    LibAria_DemoModeAddInputEvent(420, DEMO_MODE_INPUT_PRESS, 0, 136, 160);
    LibAria_DemoModeAddInputEvent(105, DEMO_MODE_INPUT_RELEASE, 0, 136, 160);
    LibAria_DemoModeAddInputEvent(590, DEMO_MODE_INPUT_PRESS, 0, 146, 162);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_RELEASE, 0, 146, 162);
    LibAria_DemoModeAddInputEvent(1290, DEMO_MODE_INPUT_PRESS, 0, 143, 237);
    LibAria_DemoModeAddInputEvent(105, DEMO_MODE_INPUT_RELEASE, 0, 143, 237);
    LibAria_DemoModeAddInputEvent(1090, DEMO_MODE_INPUT_PRESS, 0, 417, 53);
    LibAria_DemoModeAddInputEvent(95, DEMO_MODE_INPUT_MOVE, 0, 417, 56);
    LibAria_DemoModeAddInputEvent(25, DEMO_MODE_INPUT_MOVE, 0, 417, 61);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 419, 80);
    LibAria_DemoModeAddInputEvent(45, DEMO_MODE_INPUT_MOVE, 0, 419, 86);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 418, 92);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 417, 107);
    LibAria_DemoModeAddInputEvent(25, DEMO_MODE_INPUT_MOVE, 0, 416, 114);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 415, 122);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 414, 139);
    LibAria_DemoModeAddInputEvent(35, DEMO_MODE_INPUT_MOVE, 0, 413, 149);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 412, 159);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 411, 170);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_RELEASE, 0, 411, 170);
    LibAria_DemoModeAddInputEvent(1920, DEMO_MODE_INPUT_PRESS, 0, 137, 241);
    LibAria_DemoModeAddInputEvent(105, DEMO_MODE_INPUT_RELEASE, 0, 137, 241);
    LibAria_DemoModeAddInputEvent(980, DEMO_MODE_INPUT_PRESS, 0, 421, 65);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0, 422, 68);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 423, 70);
    LibAria_DemoModeAddInputEvent(0, DEMO_MODE_INPUT_MOVE, 0, 425, 74);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 426, 79);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 427, 84);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 428, 91);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 429, 111);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 429, 127);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 429, 146);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 427, 168);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_RELEASE, 0, 427, 168);
    LibAria_DemoModeAddInputEvent(1815, DEMO_MODE_INPUT_PRESS, 0, 131, 239);
    LibAria_DemoModeAddInputEvent(85, DEMO_MODE_INPUT_RELEASE, 0, 131, 239);
    LibAria_DemoModeAddInputEvent(1420, DEMO_MODE_INPUT_PRESS, 0, 401, 255);
    LibAria_DemoModeAddInputEvent(290, DEMO_MODE_INPUT_RELEASE, 0, 401, 255);
    LibAria_DemoModeAddInputEvent(920, DEMO_MODE_INPUT_PRESS, 0, 405, 84);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0, 405, 88);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 405, 91);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 405, 95);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 406, 101);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 406, 107);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 406, 125);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_RELEASE, 0, 406, 125);
    LibAria_DemoModeAddInputEvent(1165, DEMO_MODE_INPUT_PRESS, 0, 422, 254);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_RELEASE, 0, 422, 254);
    LibAria_DemoModeAddInputEvent(650, DEMO_MODE_INPUT_PRESS, 0, 404, 77);
    LibAria_DemoModeAddInputEvent(95, DEMO_MODE_INPUT_MOVE, 0, 404, 81);
    LibAria_DemoModeAddInputEvent(25, DEMO_MODE_INPUT_MOVE, 0, 404, 84);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 405, 86);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 405, 92);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 405, 95);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 405, 103);
    LibAria_DemoModeAddInputEvent(25, DEMO_MODE_INPUT_MOVE, 0, 405, 107);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 405, 110);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 405, 118);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 405, 122);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 405, 126);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 405, 134);
    LibAria_DemoModeAddInputEvent(25, DEMO_MODE_INPUT_MOVE, 0, 405, 139);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 405, 143);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 405, 154);
    LibAria_DemoModeAddInputEvent(35, DEMO_MODE_INPUT_MOVE, 0, 405, 159);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_RELEASE, 0, 405, 159);
    LibAria_DemoModeAddInputEvent(1505, DEMO_MODE_INPUT_PRESS, 0, 418, 252);
    LibAria_DemoModeAddInputEvent(315, DEMO_MODE_INPUT_MOVE, 0, 418, 249);
    LibAria_DemoModeAddInputEvent(215, DEMO_MODE_INPUT_RELEASE, 0, 418, 249);
    LibAria_DemoModeAddInputEvent(410, DEMO_MODE_INPUT_PRESS, 0, 401, 39);
    LibAria_DemoModeAddInputEvent(110, DEMO_MODE_INPUT_MOVE, 0, 401, 42);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 401, 45);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0, 401, 49);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 401, 55);
    LibAria_DemoModeAddInputEvent(45, DEMO_MODE_INPUT_MOVE, 0, 401, 61);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 401, 67);
    LibAria_DemoModeAddInputEvent(25, DEMO_MODE_INPUT_MOVE, 0, 401, 71);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 401, 74);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 401, 82);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 400, 85);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 400, 88);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 400, 95);
    LibAria_DemoModeAddInputEvent(25, DEMO_MODE_INPUT_MOVE, 0, 400, 99);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 399, 102);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 399, 110);
    LibAria_DemoModeAddInputEvent(35, DEMO_MODE_INPUT_MOVE, 0, 399, 113);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 398, 117);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_RELEASE, 0, 398, 117);
    LibAria_DemoModeAddInputEvent(1470, DEMO_MODE_INPUT_PRESS, 0, 419, 250);
    LibAria_DemoModeAddInputEvent(465, DEMO_MODE_INPUT_RELEASE, 0, 419, 250);
    LibAria_DemoModeAddInputEvent(2870, DEMO_MODE_INPUT_PRESS, 0, 55, 240);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0, 53, 238);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_RELEASE, 0, 53, 238);
    LibAria_DemoModeAddInputEvent(1080, DEMO_MODE_INPUT_PRESS, 0, 104, 237);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0, 104, 233);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 104, 230);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 104, 227);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 105, 219);
    LibAria_DemoModeAddInputEvent(35, DEMO_MODE_INPUT_MOVE, 0, 105, 215);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 105, 210);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 103, 195);
    LibAria_DemoModeAddInputEvent(45, DEMO_MODE_INPUT_MOVE, 0, 103, 190);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 103, 184);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 104, 172);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 105, 166);
    LibAria_DemoModeAddInputEvent(25, DEMO_MODE_INPUT_MOVE, 0, 106, 159);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 107, 153);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 110, 141);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 111, 135);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 113, 129);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 114, 123);
    LibAria_DemoModeAddInputEvent(25, DEMO_MODE_INPUT_MOVE, 0, 118, 112);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 120, 106);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 127, 85);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 128, 80);
    LibAria_DemoModeAddInputEvent(5, DEMO_MODE_INPUT_MOVE, 0, 133, 63);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 134, 59);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 135, 51);
    LibAria_DemoModeAddInputEvent(35, DEMO_MODE_INPUT_MOVE, 0, 136, 49);
    LibAria_DemoModeAddInputEvent(75, DEMO_MODE_INPUT_MOVE, 0, 136, 46);
    LibAria_DemoModeAddInputEvent(690, DEMO_MODE_INPUT_MOVE, 0, 136, 49);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 135, 51);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 134, 54);
    LibAria_DemoModeAddInputEvent(55, DEMO_MODE_INPUT_MOVE, 0, 134, 60);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 134, 68);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 136, 91);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 140, 128);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 141, 139);
    LibAria_DemoModeAddInputEvent(25, DEMO_MODE_INPUT_MOVE, 0, 143, 149);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 144, 158);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 145, 176);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 145, 184);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 146, 192);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 146, 199);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 148, 217);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 148, 221);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 149, 235);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 149, 238);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 149, 241);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 149, 248);
    LibAria_DemoModeAddInputEvent(55, DEMO_MODE_INPUT_MOVE, 0, 149, 251);
    LibAria_DemoModeAddInputEvent(25, DEMO_MODE_INPUT_MOVE, 0, 149, 254);
    LibAria_DemoModeAddInputEvent(265, DEMO_MODE_INPUT_MOVE, 0, 149, 257);
    LibAria_DemoModeAddInputEvent(340, DEMO_MODE_INPUT_RELEASE, 0, 149, 257);
    LibAria_DemoModeAddInputEvent(495, DEMO_MODE_INPUT_PRESS, 0, 448, 229);
    LibAria_DemoModeAddInputEvent(215, DEMO_MODE_INPUT_RELEASE, 0, 448, 229);
    LibAria_DemoModeAddInputEvent(1675, DEMO_MODE_INPUT_PRESS, 0, 116, 240);
    LibAria_DemoModeAddInputEvent(160, DEMO_MODE_INPUT_MOVE, 0, 119, 240);
    LibAria_DemoModeAddInputEvent(25, DEMO_MODE_INPUT_MOVE, 0, 122, 240);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 133, 241);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 137, 242);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 142, 243);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 166, 247);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 171, 247);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 176, 248);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 201, 251);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 207, 252);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 212, 252);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 242, 253);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 246, 254);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 264, 254);
    LibAria_DemoModeAddInputEvent(110, DEMO_MODE_INPUT_MOVE, 0, 267, 254);
    LibAria_DemoModeAddInputEvent(315, DEMO_MODE_INPUT_MOVE, 0, 271, 254);
    LibAria_DemoModeAddInputEvent(65, DEMO_MODE_INPUT_MOVE, 0, 274, 254);
    LibAria_DemoModeAddInputEvent(330, DEMO_MODE_INPUT_MOVE, 0, 271, 253);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 269, 252);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 266, 252);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 262, 251);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 257, 251);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 251, 250);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 245, 249);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 236, 247);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 225, 245);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 213, 243);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 152, 235);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 141, 235);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 91, 232);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 84, 232);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 50, 233);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 39, 233);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 34, 233);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0, 29, 233);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0, 25, 233);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0, 21, 234);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0, 17, 234);
    LibAria_DemoModeAddInputEvent(35, DEMO_MODE_INPUT_MOVE, 0, 12, 234);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 9, 234);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 7, 235);
    LibAria_DemoModeAddInputEvent(35, DEMO_MODE_INPUT_MOVE, 0, 4, 235);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0, 1, 235);
    LibAria_DemoModeAddInputEvent(85, DEMO_MODE_INPUT_RELEASE, 0, 1, 235);
    LibAria_DemoModeAddInputEvent(525, DEMO_MODE_INPUT_PRESS, 0, 467, 218);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0, 469, 219);
    LibAria_DemoModeAddInputEvent(45, DEMO_MODE_INPUT_MOVE, 0, 471, 220);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0, 467, 220);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 464, 219);
    LibAria_DemoModeAddInputEvent(0, DEMO_MODE_INPUT_MOVE, 0, 459, 219);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 454, 218);
    LibAria_DemoModeAddInputEvent(45, DEMO_MODE_INPUT_MOVE, 0, 448, 216);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 441, 216);
    LibAria_DemoModeAddInputEvent(0, DEMO_MODE_INPUT_MOVE, 0, 433, 215);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 424, 215);
    LibAria_DemoModeAddInputEvent(35, DEMO_MODE_INPUT_MOVE, 0, 415, 215);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 407, 215);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 398, 215);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 383, 216);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 376, 216);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 368, 216);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 331, 217);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 324, 217);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 318, 217);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 312, 217);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 276, 214);
    LibAria_DemoModeAddInputEvent(75, DEMO_MODE_INPUT_MOVE, 0, 271, 214);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0, 266, 214);
    LibAria_DemoModeAddInputEvent(60, DEMO_MODE_INPUT_MOVE, 0, 261, 213);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 255, 213);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 250, 213);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 245, 213);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 239, 212);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 234, 212);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 228, 212);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 223, 212);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 218, 212);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 212, 212);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 207, 211);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 203, 211);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 198, 211);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 193, 211);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 189, 211);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 184, 211);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 180, 211);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 176, 211);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 172, 211);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_RELEASE, 0, 172, 211);
    LibAria_DemoModeAddInputEvent(505, DEMO_MODE_INPUT_PRESS, 0, 412, 31);
    LibAria_DemoModeAddInputEvent(145, DEMO_MODE_INPUT_MOVE, 0, 412, 34);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 412, 38);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 412, 42);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 411, 45);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 409, 55);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 408, 60);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0, 408, 67);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0, 407, 75);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 407, 87);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 407, 104);
    LibAria_DemoModeAddInputEvent(15, DEMO_MODE_INPUT_MOVE, 0, 405, 130);
    LibAria_DemoModeAddInputEvent(5, DEMO_MODE_INPUT_RELEASE, 0, 405, 130);
    // END OF CUSTOM CODE
}

void LibAria_DemoModeTouchDownHandler(const SYS_INP_TouchStateEvent* const evt)
{
    LibAria_DemoModeRecordInputEvent(DEMO_MODE_INPUT_PRESS, evt->index, evt->x, evt->y);
    LibAria_DemoModeSendEvent(DEMO_EVENT_INPUT);
}

void LibAria_DemoModeTouchUpHandler(const SYS_INP_TouchStateEvent* const evt)
{
    LibAria_DemoModeRecordInputEvent(DEMO_MODE_INPUT_RELEASE, evt->index, evt->x, evt->y);
    LibAria_DemoModeSendEvent(DEMO_EVENT_INPUT);
}

void LibAria_DemoModeTouchMoveHandler(const SYS_INP_TouchMoveEvent* const evt)
{
    LibAria_DemoModeRecordInputEvent(DEMO_MODE_INPUT_MOVE, evt->index, evt->x, evt->y);
    LibAria_DemoModeSendEvent(DEMO_EVENT_INPUT);
}

void LibAria_DemoModeProcessEvents(void)
{
    switch (demoModeEvents.state) 
    {
        case DEMO_INIT:
        {
            demoModeEvents.numEvents = 0; 
            demoModeEvents.recordEnabled = LA_TRUE;
            demoModeEvents.idleTimeOutMSECS = DEMO_IDLE_TIMEOUT_S*1000;
            demoModeEvents.maxEvents = MAX_DEMO_EVENTS;
            demoModeEvents.pendingEvent = 0;
            demoModeEvents.demoEventFlags = 0;

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
            
            //Start timeout timer
            demoModeEvents.demoTimeoutTimer = SYS_TMR_CallbackSingle(
                                                 demoModeEvents.idleTimeOutMSECS,
                                                 (uintptr_t) & demoModeEvents,
                                                 LibAria_DemoModeStartTimerCallback);
            break;
        }
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