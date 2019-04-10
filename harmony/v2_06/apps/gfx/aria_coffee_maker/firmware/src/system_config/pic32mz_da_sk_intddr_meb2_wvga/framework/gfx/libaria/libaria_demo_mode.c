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
    // Add default input events here. These will be loaded if no events were
    // recorded.
    // ex. LibAria_DemoModeAddInputEvent(delay, index, event, x, y);

    // START OF CUSTOM CODE. DO NOT MODIFY OR REMOVE!!!
    LibAria_DemoModeAddInputEvent(57700, DEMO_MODE_INPUT_PRESS, 0,27, 369);
    LibAria_DemoModeAddInputEvent(140, DEMO_MODE_INPUT_MOVE, 0,22, 369);
    LibAria_DemoModeAddInputEvent(50, DEMO_MODE_INPUT_MOVE, 0,25, 369);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,28, 367);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0,34, 367);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,55, 366);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,64, 366);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0,71, 366);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,96, 369);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,102, 370);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,112, 371);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,147, 377);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,181, 381);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,208, 383);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,231, 383);
    LibAria_DemoModeAddInputEvent(110, DEMO_MODE_INPUT_MOVE, 0,236, 383);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,258, 383);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,261, 383);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_MOVE, 0,267, 383);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,297, 383);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,300, 383);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,304, 383);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,307, 383);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,311, 383);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,314, 383);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,316, 381);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,322, 381);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,325, 381);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,330, 380);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,333, 380);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,337, 379);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,342, 379);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,345, 379);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,348, 379);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,352, 378);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,355, 378);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,358, 377);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,361, 377);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,364, 376);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,367, 376);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,369, 375);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,373, 375);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,377, 375);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,380, 375);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_RELEASE, 0,380, 375);
    LibAria_DemoModeAddInputEvent(920, DEMO_MODE_INPUT_PRESS, 0,129, 89);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_MOVE, 0,134, 86);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_RELEASE, 0,134, 86);
    LibAria_DemoModeAddInputEvent(1120, DEMO_MODE_INPUT_PRESS, 0,142, 102);
    LibAria_DemoModeAddInputEvent(130, DEMO_MODE_INPUT_RELEASE, 0,142, 102);
    LibAria_DemoModeAddInputEvent(1040, DEMO_MODE_INPUT_PRESS, 0,136, 101);
    LibAria_DemoModeAddInputEvent(110, DEMO_MODE_INPUT_RELEASE, 0,136, 101);
    LibAria_DemoModeAddInputEvent(1070, DEMO_MODE_INPUT_PRESS, 0,143, 96);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_RELEASE, 0,143, 96);
    LibAria_DemoModeAddInputEvent(1370, DEMO_MODE_INPUT_PRESS, 0,127, 308);
    LibAria_DemoModeAddInputEvent(110, DEMO_MODE_INPUT_RELEASE, 0,127, 308);
    LibAria_DemoModeAddInputEvent(800, DEMO_MODE_INPUT_PRESS, 0,125, 308);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_RELEASE, 0,125, 308);
    LibAria_DemoModeAddInputEvent(840, DEMO_MODE_INPUT_PRESS, 0,132, 311);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_RELEASE, 0,132, 311);
    LibAria_DemoModeAddInputEvent(950, DEMO_MODE_INPUT_PRESS, 0,134, 406);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_RELEASE, 0,134, 406);
    LibAria_DemoModeAddInputEvent(1480, DEMO_MODE_INPUT_PRESS, 0,716, 68);
    LibAria_DemoModeAddInputEvent(150, DEMO_MODE_INPUT_MOVE, 0,718, 72);
    LibAria_DemoModeAddInputEvent(140, DEMO_MODE_INPUT_MOVE, 0,711, 93);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,711, 97);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0,710, 102);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,710, 130);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0,710, 137);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,713, 173);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,713, 177);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0,713, 182);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,715, 215);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,715, 219);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0,715, 225);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,716, 256);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,716, 259);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0,717, 265);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,717, 289);
    LibAria_DemoModeAddInputEvent(130, DEMO_MODE_INPUT_MOVE, 0,717, 312);
    LibAria_DemoModeAddInputEvent(50, DEMO_MODE_INPUT_MOVE, 0,719, 336);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,720, 342);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,720, 346);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_RELEASE, 0,720, 346);
    LibAria_DemoModeAddInputEvent(1980, DEMO_MODE_INPUT_PRESS, 0,714, 180);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,714, 185);
    LibAria_DemoModeAddInputEvent(110, DEMO_MODE_INPUT_MOVE, 0,714, 188);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,717, 201);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,717, 205);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_MOVE, 0,717, 223);
    LibAria_DemoModeAddInputEvent(110, DEMO_MODE_INPUT_MOVE, 0,717, 228);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,717, 243);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,717, 246);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_MOVE, 0,717, 262);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,717, 265);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_MOVE, 0,716, 279);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_MOVE, 0,716, 282);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,715, 288);
    LibAria_DemoModeAddInputEvent(190, DEMO_MODE_INPUT_MOVE, 0,715, 291);
    LibAria_DemoModeAddInputEvent(150, DEMO_MODE_INPUT_MOVE, 0,715, 285);
    LibAria_DemoModeAddInputEvent(50, DEMO_MODE_INPUT_MOVE, 0,715, 276);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,714, 267);
    LibAria_DemoModeAddInputEvent(140, DEMO_MODE_INPUT_MOVE, 0,714, 258);
    LibAria_DemoModeAddInputEvent(120, DEMO_MODE_INPUT_MOVE, 0,714, 255);
    LibAria_DemoModeAddInputEvent(130, DEMO_MODE_INPUT_MOVE, 0,713, 247);
    LibAria_DemoModeAddInputEvent(120, DEMO_MODE_INPUT_MOVE, 0,713, 241);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0,713, 238);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,713, 223);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_MOVE, 0,713, 219);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,713, 205);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_MOVE, 0,713, 201);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,713, 189);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,713, 186);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_MOVE, 0,713, 170);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,714, 168);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,714, 165);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,716, 152);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0,716, 148);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0,718, 132);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,719, 121);
    LibAria_DemoModeAddInputEvent(50, DEMO_MODE_INPUT_MOVE, 0,720, 112);
    LibAria_DemoModeAddInputEvent(140, DEMO_MODE_INPUT_RELEASE, 0,720, 112);
    LibAria_DemoModeAddInputEvent(1800, DEMO_MODE_INPUT_PRESS, 0,725, 449);
    LibAria_DemoModeAddInputEvent(580, DEMO_MODE_INPUT_RELEASE, 0,725, 449);
    LibAria_DemoModeAddInputEvent(510, DEMO_MODE_INPUT_PRESS, 0,722, 301);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0,722, 295);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0,722, 290);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,722, 277);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,722, 270);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,720, 237);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,720, 232);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,720, 228);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,723, 197);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,723, 194);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,724, 191);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,728, 164);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,729, 161);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,730, 157);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,733, 133);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,734, 131);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0,734, 127);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0,735, 110);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_RELEASE, 0,735, 110);
    LibAria_DemoModeAddInputEvent(1840, DEMO_MODE_INPUT_PRESS, 0,714, 451);
    LibAria_DemoModeAddInputEvent(360, DEMO_MODE_INPUT_MOVE, 0,714, 447);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_RELEASE, 0,714, 447);
    LibAria_DemoModeAddInputEvent(360, DEMO_MODE_INPUT_PRESS, 0,730, 278);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,730, 273);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,730, 269);
    LibAria_DemoModeAddInputEvent(50, DEMO_MODE_INPUT_MOVE, 0,731, 266);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,732, 253);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,732, 250);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,732, 243);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,732, 205);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,733, 198);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,734, 193);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,739, 160);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,740, 154);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,741, 150);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,748, 119);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,749, 116);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_RELEASE, 0,749, 116);
    LibAria_DemoModeAddInputEvent(2000, DEMO_MODE_INPUT_PRESS, 0,728, 459);
    LibAria_DemoModeAddInputEvent(190, DEMO_MODE_INPUT_MOVE, 0,728, 454);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0,728, 451);
    LibAria_DemoModeAddInputEvent(240, DEMO_MODE_INPUT_MOVE, 0,728, 448);
    LibAria_DemoModeAddInputEvent(60, DEMO_MODE_INPUT_RELEASE, 0,728, 448);
    LibAria_DemoModeAddInputEvent(850, DEMO_MODE_INPUT_PRESS, 0,601, 359);
    LibAria_DemoModeAddInputEvent(160, DEMO_MODE_INPUT_MOVE, 0,606, 359);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_MOVE, 0,609, 359);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,627, 359);
    LibAria_DemoModeAddInputEvent(120, DEMO_MODE_INPUT_MOVE, 0,631, 359);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,666, 363);
    LibAria_DemoModeAddInputEvent(120, DEMO_MODE_INPUT_MOVE, 0,669, 363);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,702, 367);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,706, 368);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_MOVE, 0,709, 369);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,739, 375);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_MOVE, 0,759, 378);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,763, 378);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,765, 379);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,778, 380);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,782, 381);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,784, 382);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,788, 383);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,791, 383);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0,794, 383);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,796, 384);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0,798, 385);
    LibAria_DemoModeAddInputEvent(200, DEMO_MODE_INPUT_RELEASE, 0,798, 385);
    LibAria_DemoModeAddInputEvent(1250, DEMO_MODE_INPUT_PRESS, 0,223, 408);
    LibAria_DemoModeAddInputEvent(150, DEMO_MODE_INPUT_MOVE, 0,219, 408);
    LibAria_DemoModeAddInputEvent(60, DEMO_MODE_INPUT_MOVE, 0,215, 408);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,204, 407);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_MOVE, 0,201, 407);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,175, 403);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,169, 403);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,139, 399);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,135, 399);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,129, 399);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,100, 399);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,94, 398);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0,90, 398);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,59, 398);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,56, 398);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,51, 398);
    LibAria_DemoModeAddInputEvent(60, DEMO_MODE_INPUT_MOVE, 0,30, 399);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,16, 399);
    LibAria_DemoModeAddInputEvent(50, DEMO_MODE_INPUT_MOVE, 0,14, 400);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,8, 402);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_MOVE, 0,6, 403);
    LibAria_DemoModeAddInputEvent(210, DEMO_MODE_INPUT_MOVE, 0,5, 401);
    LibAria_DemoModeAddInputEvent(300, DEMO_MODE_INPUT_MOVE, 0,7, 399);
    LibAria_DemoModeAddInputEvent(50, DEMO_MODE_INPUT_MOVE, 0,10, 399);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,16, 399);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0,19, 399);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,28, 398);
    LibAria_DemoModeAddInputEvent(50, DEMO_MODE_INPUT_MOVE, 0,34, 398);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,62, 397);
    LibAria_DemoModeAddInputEvent(60, DEMO_MODE_INPUT_MOVE, 0,67, 397);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,100, 397);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,109, 397);
    LibAria_DemoModeAddInputEvent(80, DEMO_MODE_INPUT_MOVE, 0,139, 397);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0,174, 397);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,211, 397);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,215, 397);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0,222, 397);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,254, 397);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,258, 397);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_MOVE, 0,263, 397);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,297, 400);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,301, 401);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,304, 401);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,309, 401);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,312, 402);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,315, 402);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,318, 403);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,324, 403);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,330, 403);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,334, 404);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,337, 404);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,341, 405);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,344, 405);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,347, 405);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,350, 406);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0,354, 406);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0,356, 407);
    LibAria_DemoModeAddInputEvent(50, DEMO_MODE_INPUT_MOVE, 0,359, 407);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_MOVE, 0,362, 407);
    LibAria_DemoModeAddInputEvent(50, DEMO_MODE_INPUT_RELEASE, 0,362, 407);
    LibAria_DemoModeAddInputEvent(1990, DEMO_MODE_INPUT_PRESS, 0,129, 419);
    LibAria_DemoModeAddInputEvent(130, DEMO_MODE_INPUT_RELEASE, 0,129, 419);
    LibAria_DemoModeAddInputEvent(1260, DEMO_MODE_INPUT_PRESS, 0,25, 153);
    LibAria_DemoModeAddInputEvent(140, DEMO_MODE_INPUT_RELEASE, 0,25, 153);
    LibAria_DemoModeAddInputEvent(750, DEMO_MODE_INPUT_PRESS, 0,35, 150);
    LibAria_DemoModeAddInputEvent(130, DEMO_MODE_INPUT_RELEASE, 0,35, 150);
    LibAria_DemoModeAddInputEvent(820, DEMO_MODE_INPUT_PRESS, 0,30, 162);
    LibAria_DemoModeAddInputEvent(130, DEMO_MODE_INPUT_MOVE, 0,35, 159);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_RELEASE, 0,35, 159);
    LibAria_DemoModeAddInputEvent(980, DEMO_MODE_INPUT_PRESS, 0,28, 152);
    LibAria_DemoModeAddInputEvent(100, DEMO_MODE_INPUT_RELEASE, 0,28, 152);
    LibAria_DemoModeAddInputEvent(1710, DEMO_MODE_INPUT_PRESS, 0,35, 364);
    LibAria_DemoModeAddInputEvent(50, DEMO_MODE_INPUT_MOVE, 0,35, 359);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_RELEASE, 0,35, 359);
    LibAria_DemoModeAddInputEvent(1490, DEMO_MODE_INPUT_PRESS, 0,180, 437);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,180, 431);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,178, 428);
    LibAria_DemoModeAddInputEvent(280, DEMO_MODE_INPUT_MOVE, 0,172, 350);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0,178, 250);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,179, 245);
    LibAria_DemoModeAddInputEvent(140, DEMO_MODE_INPUT_MOVE, 0,179, 237);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0,185, 168);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,186, 165);
    LibAria_DemoModeAddInputEvent(150, DEMO_MODE_INPUT_MOVE, 0,186, 160);
    LibAria_DemoModeAddInputEvent(120, DEMO_MODE_INPUT_MOVE, 0,189, 123);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,192, 105);
    LibAria_DemoModeAddInputEvent(70, DEMO_MODE_INPUT_MOVE, 0,193, 103);
    LibAria_DemoModeAddInputEvent(130, DEMO_MODE_INPUT_MOVE, 0,195, 93);
    LibAria_DemoModeAddInputEvent(190, DEMO_MODE_INPUT_MOVE, 0,193, 95);
    LibAria_DemoModeAddInputEvent(120, DEMO_MODE_INPUT_MOVE, 0,191, 106);
    LibAria_DemoModeAddInputEvent(120, DEMO_MODE_INPUT_MOVE, 0,190, 125);
    LibAria_DemoModeAddInputEvent(90, DEMO_MODE_INPUT_MOVE, 0,190, 128);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,190, 153);
    LibAria_DemoModeAddInputEvent(220, DEMO_MODE_INPUT_MOVE, 0,190, 186);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,190, 226);
    LibAria_DemoModeAddInputEvent(120, DEMO_MODE_INPUT_MOVE, 0,190, 230);
    LibAria_DemoModeAddInputEvent(10, DEMO_MODE_INPUT_MOVE, 0,192, 280);
    LibAria_DemoModeAddInputEvent(20, DEMO_MODE_INPUT_MOVE, 0,192, 286);
    LibAria_DemoModeAddInputEvent(130, DEMO_MODE_INPUT_MOVE, 0,192, 290);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,189, 338);
    LibAria_DemoModeAddInputEvent(130, DEMO_MODE_INPUT_MOVE, 0,189, 345);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0,191, 387);
    LibAria_DemoModeAddInputEvent(170, DEMO_MODE_INPUT_MOVE, 0,192, 391);
    LibAria_DemoModeAddInputEvent(40, DEMO_MODE_INPUT_MOVE, 0,195, 420);
    LibAria_DemoModeAddInputEvent(160, DEMO_MODE_INPUT_MOVE, 0,195, 423);
    LibAria_DemoModeAddInputEvent(60, DEMO_MODE_INPUT_MOVE, 0,197, 437);
    LibAria_DemoModeAddInputEvent(170, DEMO_MODE_INPUT_MOVE, 0,201, 448);
    LibAria_DemoModeAddInputEvent(190, DEMO_MODE_INPUT_MOVE, 0,203, 457);
    LibAria_DemoModeAddInputEvent(30, DEMO_MODE_INPUT_MOVE, 0,204, 459);
    LibAria_DemoModeAddInputEvent(140, DEMO_MODE_INPUT_MOVE, 0,204, 462);
    LibAria_DemoModeAddInputEvent(250, DEMO_MODE_INPUT_RELEASE, 0,204, 462);
    LibAria_DemoModeAddInputEvent(750, DEMO_MODE_INPUT_PRESS, 0,746, 437);
    LibAria_DemoModeAddInputEvent(140, DEMO_MODE_INPUT_RELEASE, 0,746, 437);
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
				APP_WVGA_INTDDR_Initialize();
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