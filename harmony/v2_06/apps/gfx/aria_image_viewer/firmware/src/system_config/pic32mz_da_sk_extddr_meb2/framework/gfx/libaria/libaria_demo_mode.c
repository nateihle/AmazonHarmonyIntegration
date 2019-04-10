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
    //CUSTOM CODE - DO NOT MODIFY OR REMOVE
    LibAria_DemoModeAddInputEvent(9150, 2, 0, 251, 22);
    LibAria_DemoModeAddInputEvent(1620, 4, 0, 251, 22);
    LibAria_DemoModeAddInputEvent(2050, 2, 0, 72, 153);
    LibAria_DemoModeAddInputEvent(130, 1, 0, 67, 153);
    LibAria_DemoModeAddInputEvent(60, 1, 0, 73, 153);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 79, 150);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 92, 147);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 120, 139);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 159, 132);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 205, 124);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 257, 117);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 309, 112);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 361, 109);
    LibAria_DemoModeAddInputEvent(20, 4, 0, 361, 109);
    LibAria_DemoModeAddInputEvent(1620, 2, 0, 55, 170);
    LibAria_DemoModeAddInputEvent(80, 1, 0, 61, 170);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 67, 170);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 77, 167);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 101, 161);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 135, 155);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 175, 148);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 220, 140);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 268, 133);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 314, 129);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 360, 127);
    LibAria_DemoModeAddInputEvent(20, 4, 0, 360, 127);
    LibAria_DemoModeAddInputEvent(1620, 2, 0, 62, 167);
    LibAria_DemoModeAddInputEvent(120, 1, 0, 71, 167);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 80, 164);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 99, 159);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 132, 153);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 171, 146);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 215, 139);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 260, 134);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 303, 131);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 345, 129);
    LibAria_DemoModeAddInputEvent(20, 4, 0, 345, 129);
    LibAria_DemoModeAddInputEvent(1630, 2, 0, 55, 175);
    LibAria_DemoModeAddInputEvent(90, 1, 0, 62, 175);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 68, 175);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 80, 173);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 107, 168);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 143, 163);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 184, 158);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 228, 155);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 274, 154);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 318, 154);
    LibAria_DemoModeAddInputEvent(20, 4, 0, 318, 154);
    LibAria_DemoModeAddInputEvent(1710, 2, 0, 263, 168);
    LibAria_DemoModeAddInputEvent(90, 1, 0, 257, 168);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 255, 168);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 254, 168);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 253, 168);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 252, 168);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 250, 168);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 249, 168);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 256, 166);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 264, 162);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 266, 161);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 268, 160);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 270, 159);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 271, 159);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 273, 158);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 275, 157);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 276, 156);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 277, 155);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 279, 154);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 280, 154);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 282, 153);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 286, 151);
    LibAria_DemoModeAddInputEvent(10, 2, 1, 180, 182);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 288, 150);
    LibAria_DemoModeAddInputEvent(50, 1, 0, 290, 149);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 291, 148);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 292, 148);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 147);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 294, 147);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 295, 146);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 174, 184);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 296, 146);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 172, 185);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 171, 185);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 297, 145);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 170, 186);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 298, 145);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 169, 186);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 298, 144);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 167, 187);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 299, 144);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 165, 187);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 299, 143);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 164, 187);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 300, 143);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 162, 188);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 161, 189);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 301, 142);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 159, 189);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 157, 190);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 302, 142);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 302, 141);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 155, 191);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 303, 141);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 303, 140);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 153, 192);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 304, 140);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 152, 192);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 151, 193);
    LibAria_DemoModeAddInputEvent(70, 1, 0, 305, 139);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 150, 194);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 149, 194);
    LibAria_DemoModeAddInputEvent(40, 1, 1, 148, 194);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 148, 195);
    LibAria_DemoModeAddInputEvent(60, 1, 1, 147, 195);
    LibAria_DemoModeAddInputEvent(60, 1, 0, 306, 139);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 147, 196);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 146, 196);
    LibAria_DemoModeAddInputEvent(60, 1, 1, 145, 196);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 145, 197);
    LibAria_DemoModeAddInputEvent(80, 1, 1, 144, 197);
    LibAria_DemoModeAddInputEvent(70, 1, 1, 144, 198);
    LibAria_DemoModeAddInputEvent(50, 4, 1, 144, 198);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 306, 142);
    LibAria_DemoModeAddInputEvent(20, 4, 0, 306, 142);
    LibAria_DemoModeAddInputEvent(470, 2, 0, 267, 204);
    LibAria_DemoModeAddInputEvent(110, 1, 0, 267, 198);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 267, 197);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 267, 195);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 269, 192);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 270, 190);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 271, 187);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 272, 184);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 273, 182);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 273, 179);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 274, 176);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 275, 173);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 276, 171);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 277, 168);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 277, 165);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 278, 162);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 279, 159);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 279, 157);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 279, 154);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 280, 151);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 280, 149);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 281, 146);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 281, 143);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 281, 140);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 282, 138);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 282, 135);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 282, 132);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 282, 129);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 282, 126);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 283, 123);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 283, 121);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 283, 118);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 283, 115);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 284, 112);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 284, 109);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 285, 107);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 285, 104);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 285, 101);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 286, 98);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 286, 96);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 286, 93);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 287, 91);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 287, 89);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 288, 87);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 289, 85);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 291, 84);
    LibAria_DemoModeAddInputEvent(10, 4, 0, 291, 84);
    LibAria_DemoModeAddInputEvent(380, 2, 0, 286, 222);
    LibAria_DemoModeAddInputEvent(120, 1, 0, 286, 216);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 286, 215);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 286, 212);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 286, 209);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 286, 206);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 288, 203);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 289, 200);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 290, 196);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 290, 193);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 291, 189);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 292, 186);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 292, 182);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 292, 178);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 175);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 293, 171);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 293, 168);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 164);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 161);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 293, 157);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 154);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 151);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 148);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 293, 145);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 142);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 139);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 293, 136);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 293, 134);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 293, 131);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 128);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 126);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 293, 123);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 120);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 118);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 116);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 293, 113);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 111);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 109);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 293, 107);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 293, 105);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 103);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 293, 101);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 99);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 293, 97);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 95);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 94);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 93);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 293, 92);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 91);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 90);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 293, 89);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 293, 88);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 294, 88);
    LibAria_DemoModeAddInputEvent(40, 4, 0, 294, 88);
    LibAria_DemoModeAddInputEvent(540, 2, 0, 195, 187);
    LibAria_DemoModeAddInputEvent(30, 2, 1, 279, 183);
    LibAria_DemoModeAddInputEvent(20, 4, 0, 195, 187);
    LibAria_DemoModeAddInputEvent(50, 2, 0, 192, 192);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 279, 176);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 279, 174);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 282, 173);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 283, 172);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 284, 170);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 285, 170);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 286, 168);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 288, 167);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 288, 166);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 290, 165);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 291, 164);
    LibAria_DemoModeAddInputEvent(50, 1, 1, 292, 163);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 293, 163);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 294, 162);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 295, 161);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 296, 161);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 297, 160);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 186, 192);
    LibAria_DemoModeAddInputEvent(50, 1, 1, 298, 159);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 185, 192);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 184, 192);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 299, 158);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 300, 158);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 183, 192);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 300, 157);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 182, 192);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 301, 157);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 181, 192);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 301, 156);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 180, 192);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 302, 156);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 179, 192);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 302, 155);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 303, 155);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 178, 192);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 178, 194);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 303, 154);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 177, 195);
    LibAria_DemoModeAddInputEvent(40, 1, 1, 304, 154);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 176, 195);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 176, 196);
    LibAria_DemoModeAddInputEvent(330, 1, 1, 301, 154);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 178, 196);
    LibAria_DemoModeAddInputEvent(80, 1, 1, 300, 154);
    LibAria_DemoModeAddInputEvent(140, 1, 1, 299, 154);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 179, 196);
    LibAria_DemoModeAddInputEvent(100, 1, 1, 299, 156);
    LibAria_DemoModeAddInputEvent(210, 1, 0, 179, 191);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 299, 159);
    LibAria_DemoModeAddInputEvent(10, 4, 0, 179, 191);
    LibAria_DemoModeAddInputEvent(10, 4, 1, 299, 159);
    LibAria_DemoModeAddInputEvent(1410, 2, 0, 449, 238);
    LibAria_DemoModeAddInputEvent(100, 4, 0, 449, 238);
    LibAria_DemoModeAddInputEvent(450, 2, 0, 449, 237);
    LibAria_DemoModeAddInputEvent(110, 4, 0, 449, 237);
    LibAria_DemoModeAddInputEvent(400, 2, 0, 451, 239);
    LibAria_DemoModeAddInputEvent(100, 4, 0, 451, 239);
    LibAria_DemoModeAddInputEvent(450, 2, 0, 459, 237);
    LibAria_DemoModeAddInputEvent(40, 4, 0, 459, 237);
    LibAria_DemoModeAddInputEvent(700, 2, 0, 70, 209);
    LibAria_DemoModeAddInputEvent(50, 1, 0, 75, 209);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 86, 203);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 107, 196);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 144, 187);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 190, 178);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 241, 172);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 291, 169);
    LibAria_DemoModeAddInputEvent(10, 4, 0, 291, 169);
    LibAria_DemoModeAddInputEvent(2610, 2, 0, 275, 175);
    LibAria_DemoModeAddInputEvent(100, 2, 1, 192, 191);
    LibAria_DemoModeAddInputEvent(140, 1, 0, 280, 171);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 281, 171);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 282, 170);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 283, 169);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 284, 169);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 187, 191);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 285, 168);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 186, 191);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 286, 168);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 185, 191);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 287, 167);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 184, 191);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 183, 191);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 288, 167);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 182, 191);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 289, 166);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 181, 191);
    LibAria_DemoModeAddInputEvent(40, 1, 0, 290, 165);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 180, 191);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 179, 191);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 291, 165);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 178, 191);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 292, 164);
    LibAria_DemoModeAddInputEvent(40, 1, 1, 177, 193);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 293, 164);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 176, 194);
    LibAria_DemoModeAddInputEvent(40, 1, 0, 294, 163);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 175, 195);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 174, 195);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 295, 163);
    LibAria_DemoModeAddInputEvent(40, 1, 1, 173, 196);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 172, 196);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 296, 163);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 297, 162);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 171, 197);
    LibAria_DemoModeAddInputEvent(40, 1, 0, 298, 162);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 170, 197);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 170, 198);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 169, 198);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 299, 161);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 168, 198);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 300, 161);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 167, 199);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 301, 161);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 166, 199);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 166, 200);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 302, 161);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 165, 200);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 302, 160);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 165, 201);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 303, 160);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 164, 201);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 304, 160);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 163, 201);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 305, 160);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 163, 202);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 162, 202);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 305, 159);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 306, 159);
    LibAria_DemoModeAddInputEvent(70, 1, 0, 307, 159);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 162, 203);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 161, 203);
    LibAria_DemoModeAddInputEvent(70, 1, 1, 160, 203);
    LibAria_DemoModeAddInputEvent(50, 1, 1, 160, 204);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 307, 158);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 159, 204);
    LibAria_DemoModeAddInputEvent(70, 1, 0, 308, 158);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 158, 204);
    LibAria_DemoModeAddInputEvent(40, 1, 1, 158, 205);
    LibAria_DemoModeAddInputEvent(20, 1, 1, 157, 205);
    LibAria_DemoModeAddInputEvent(50, 1, 1, 156, 206);
    LibAria_DemoModeAddInputEvent(60, 1, 1, 155, 206);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 155, 207);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 154, 207);
    LibAria_DemoModeAddInputEvent(60, 1, 1, 153, 207);
    LibAria_DemoModeAddInputEvent(30, 1, 1, 153, 208);
    LibAria_DemoModeAddInputEvent(60, 1, 1, 152, 208);
    LibAria_DemoModeAddInputEvent(90, 1, 1, 152, 209);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 151, 209);
    LibAria_DemoModeAddInputEvent(150, 1, 1, 150, 209);
    LibAria_DemoModeAddInputEvent(430, 1, 1, 150, 210);
    LibAria_DemoModeAddInputEvent(20, 4, 0, 308, 158);
    LibAria_DemoModeAddInputEvent(50, 1, 1, 149, 210);
    LibAria_DemoModeAddInputEvent(10, 1, 1, 149, 207);
    LibAria_DemoModeAddInputEvent(20, 4, 1, 149, 207);
    LibAria_DemoModeAddInputEvent(340, 2, 0, 139, 215);
    LibAria_DemoModeAddInputEvent(220, 1, 0, 142, 210);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 142, 209);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 143, 208);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 143, 207);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 144, 206);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 144, 205);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 145, 204);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 145, 203);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 146, 202);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 147, 201);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 147, 200);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 148, 199);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 148, 198);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 149, 198);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 150, 197);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 150, 196);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 151, 195);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 152, 194);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 152, 193);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 153, 192);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 154, 192);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 154, 191);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 155, 190);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 156, 189);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 157, 189);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 158, 188);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 159, 187);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 160, 186);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 162, 186);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 163, 185);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 164, 184);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 165, 184);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 166, 183);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 167, 182);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 168, 182);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 169, 181);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 170, 181);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 171, 180);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 172, 179);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 173, 179);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 174, 178);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 175, 178);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 176, 177);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 177, 177);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 179, 176);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 180, 176);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 181, 176);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 182, 175);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 183, 175);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 184, 175);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 185, 174);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 186, 174);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 188, 174);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 189, 173);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 190, 173);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 192, 173);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 193, 173);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 194, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 195, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 196, 172);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 198, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 199, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 200, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 202, 172);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 203, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 205, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 206, 172);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 207, 172);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 208, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 210, 172);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 211, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 212, 172);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 213, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 215, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 216, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 217, 172);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 219, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 220, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 221, 172);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 223, 172);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 224, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 226, 172);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 227, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 229, 172);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 230, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 232, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 233, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 235, 174);
    LibAria_DemoModeAddInputEvent(30, 1, 0, 236, 174);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 238, 174);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 239, 174);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 240, 174);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 241, 175);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 243, 175);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 244, 175);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 245, 175);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 246, 175);
    LibAria_DemoModeAddInputEvent(20, 1, 0, 247, 175);
    LibAria_DemoModeAddInputEvent(10, 1, 0, 248, 175);
    //END OF CUSTOM CODE
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