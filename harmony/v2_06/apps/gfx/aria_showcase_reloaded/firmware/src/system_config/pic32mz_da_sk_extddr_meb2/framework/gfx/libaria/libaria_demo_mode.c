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
// CUSTOM CODE - DO NOT MODIFY OR REMOVE!!!    
    LibAria_DemoModeAddInputEvent(6110, 2, 0,68, 76);
    LibAria_DemoModeAddInputEvent(260, 4, 0,68, 76);
    LibAria_DemoModeAddInputEvent(1780, 2, 0,239, 163);
    LibAria_DemoModeAddInputEvent(1730, 4, 0,239, 163);
    LibAria_DemoModeAddInputEvent(4460, 2, 0,454, 38);
    LibAria_DemoModeAddInputEvent(100, 4, 0,454, 38);
    LibAria_DemoModeAddInputEvent(1110, 2, 0,327, 102);
    LibAria_DemoModeAddInputEvent(490, 1, 0,332, 105);
    LibAria_DemoModeAddInputEvent(10, 1, 0,334, 105);
    LibAria_DemoModeAddInputEvent(30, 1, 0,336, 106);
    LibAria_DemoModeAddInputEvent(30, 1, 0,338, 107);
    LibAria_DemoModeAddInputEvent(20, 1, 0,340, 108);
    LibAria_DemoModeAddInputEvent(20, 1, 0,347, 113);
    LibAria_DemoModeAddInputEvent(20, 1, 0,355, 120);
    LibAria_DemoModeAddInputEvent(20, 1, 0,359, 125);
    LibAria_DemoModeAddInputEvent(20, 1, 0,365, 133);
    LibAria_DemoModeAddInputEvent(20, 1, 0,368, 138);
    LibAria_DemoModeAddInputEvent(20, 1, 0,371, 144);
    LibAria_DemoModeAddInputEvent(20, 1, 0,373, 153);
    LibAria_DemoModeAddInputEvent(20, 1, 0,373, 158);
    LibAria_DemoModeAddInputEvent(20, 1, 0,373, 165);
    LibAria_DemoModeAddInputEvent(20, 1, 0,373, 169);
    LibAria_DemoModeAddInputEvent(20, 1, 0,373, 173);
    LibAria_DemoModeAddInputEvent(20, 1, 0,368, 180);
    LibAria_DemoModeAddInputEvent(20, 1, 0,365, 186);
    LibAria_DemoModeAddInputEvent(20, 1, 0,358, 195);
    LibAria_DemoModeAddInputEvent(20, 1, 0,354, 200);
    LibAria_DemoModeAddInputEvent(20, 1, 0,349, 205);
    LibAria_DemoModeAddInputEvent(20, 1, 0,343, 211);
    LibAria_DemoModeAddInputEvent(20, 1, 0,340, 213);
    LibAria_DemoModeAddInputEvent(20, 1, 0,335, 217);
    LibAria_DemoModeAddInputEvent(20, 1, 0,333, 218);
    LibAria_DemoModeAddInputEvent(20, 1, 0,332, 219);
    LibAria_DemoModeAddInputEvent(20, 1, 0,330, 220);
    LibAria_DemoModeAddInputEvent(20, 1, 0,330, 221);
    LibAria_DemoModeAddInputEvent(20, 1, 0,329, 221);
    LibAria_DemoModeAddInputEvent(460, 1, 0,328, 222);
    LibAria_DemoModeAddInputEvent(260, 1, 0,331, 222);
    LibAria_DemoModeAddInputEvent(40, 1, 0,333, 222);
    LibAria_DemoModeAddInputEvent(40, 1, 0,338, 218);
    LibAria_DemoModeAddInputEvent(10, 1, 0,340, 216);
    LibAria_DemoModeAddInputEvent(30, 1, 0,342, 215);
    LibAria_DemoModeAddInputEvent(10, 1, 0,347, 212);
    LibAria_DemoModeAddInputEvent(30, 1, 0,354, 206);
    LibAria_DemoModeAddInputEvent(20, 1, 0,359, 201);
    LibAria_DemoModeAddInputEvent(20, 1, 0,367, 189);
    LibAria_DemoModeAddInputEvent(20, 1, 0,370, 184);
    LibAria_DemoModeAddInputEvent(20, 1, 0,373, 178);
    LibAria_DemoModeAddInputEvent(20, 1, 0,375, 168);
    LibAria_DemoModeAddInputEvent(20, 1, 0,376, 161);
    LibAria_DemoModeAddInputEvent(20, 1, 0,376, 151);
    LibAria_DemoModeAddInputEvent(20, 1, 0,373, 145);
    LibAria_DemoModeAddInputEvent(20, 1, 0,371, 140);
    LibAria_DemoModeAddInputEvent(20, 1, 0,368, 132);
    LibAria_DemoModeAddInputEvent(20, 1, 0,364, 127);
    LibAria_DemoModeAddInputEvent(20, 1, 0,359, 121);
    LibAria_DemoModeAddInputEvent(20, 1, 0,355, 117);
    LibAria_DemoModeAddInputEvent(20, 1, 0,350, 114);
    LibAria_DemoModeAddInputEvent(20, 1, 0,342, 112);
    LibAria_DemoModeAddInputEvent(20, 1, 0,337, 112);
    LibAria_DemoModeAddInputEvent(20, 1, 0,328, 112);
    LibAria_DemoModeAddInputEvent(20, 1, 0,321, 115);
    LibAria_DemoModeAddInputEvent(20, 1, 0,315, 117);
    LibAria_DemoModeAddInputEvent(20, 1, 0,306, 122);
    LibAria_DemoModeAddInputEvent(20, 1, 0,300, 126);
    LibAria_DemoModeAddInputEvent(20, 1, 0,293, 132);
    LibAria_DemoModeAddInputEvent(20, 1, 0,289, 137);
    LibAria_DemoModeAddInputEvent(20, 1, 0,285, 143);
    LibAria_DemoModeAddInputEvent(20, 1, 0,281, 152);
    LibAria_DemoModeAddInputEvent(20, 1, 0,279, 159);
    LibAria_DemoModeAddInputEvent(20, 1, 0,277, 171);
    LibAria_DemoModeAddInputEvent(20, 1, 0,277, 180);
    LibAria_DemoModeAddInputEvent(20, 1, 0,277, 187);
    LibAria_DemoModeAddInputEvent(20, 1, 0,279, 196);
    LibAria_DemoModeAddInputEvent(20, 1, 0,281, 202);
    LibAria_DemoModeAddInputEvent(20, 1, 0,283, 207);
    LibAria_DemoModeAddInputEvent(20, 1, 0,289, 214);
    LibAria_DemoModeAddInputEvent(20, 1, 0,294, 218);
    LibAria_DemoModeAddInputEvent(20, 1, 0,304, 223);
    LibAria_DemoModeAddInputEvent(20, 1, 0,311, 225);
    LibAria_DemoModeAddInputEvent(20, 1, 0,320, 225);
    LibAria_DemoModeAddInputEvent(20, 1, 0,333, 221);
    LibAria_DemoModeAddInputEvent(20, 1, 0,341, 217);
    LibAria_DemoModeAddInputEvent(20, 1, 0,354, 211);
    LibAria_DemoModeAddInputEvent(20, 1, 0,361, 206);
    LibAria_DemoModeAddInputEvent(130, 1, 0,368, 202);
    LibAria_DemoModeAddInputEvent(130, 1, 0,374, 197);
    LibAria_DemoModeAddInputEvent(120, 1, 0,377, 195);
    LibAria_DemoModeAddInputEvent(130, 1, 0,381, 192);
    LibAria_DemoModeAddInputEvent(250, 1, 0,384, 189);
    LibAria_DemoModeAddInputEvent(130, 1, 0,387, 186);
    LibAria_DemoModeAddInputEvent(30, 1, 0,390, 182);
    LibAria_DemoModeAddInputEvent(10, 1, 0,393, 178);
    LibAria_DemoModeAddInputEvent(20, 1, 0,396, 174);
    LibAria_DemoModeAddInputEvent(20, 4, 0,396, 174);
    LibAria_DemoModeAddInputEvent(1060, 2, 0,453, 38);
    LibAria_DemoModeAddInputEvent(90, 4, 0,453, 38);
    LibAria_DemoModeAddInputEvent(1280, 2, 0,410, 168);
    LibAria_DemoModeAddInputEvent(6120, 4, 0,410, 168);
    LibAria_DemoModeAddInputEvent(6530, 2, 0,449, 36);
    LibAria_DemoModeAddInputEvent(150, 4, 0,449, 36);
    LibAria_DemoModeAddInputEvent(1040, 2, 0,284, 142);
    LibAria_DemoModeAddInputEvent(100, 4, 0,284, 142);
    LibAria_DemoModeAddInputEvent(370, 2, 0,212, 116);
    LibAria_DemoModeAddInputEvent(90, 4, 0,212, 116);
    LibAria_DemoModeAddInputEvent(320, 2, 0,160, 160);
    LibAria_DemoModeAddInputEvent(100, 4, 0,160, 160);
    LibAria_DemoModeAddInputEvent(310, 2, 0,200, 232);
    LibAria_DemoModeAddInputEvent(90, 4, 0,200, 232);
    LibAria_DemoModeAddInputEvent(370, 2, 0,285, 211);
    LibAria_DemoModeAddInputEvent(110, 4, 0,285, 211);
    LibAria_DemoModeAddInputEvent(390, 2, 0,302, 138);
    LibAria_DemoModeAddInputEvent(100, 4, 0,302, 138);
    LibAria_DemoModeAddInputEvent(320, 2, 0,223, 111);
    LibAria_DemoModeAddInputEvent(110, 4, 0,223, 111);
    LibAria_DemoModeAddInputEvent(310, 2, 0,156, 155);
    LibAria_DemoModeAddInputEvent(90, 4, 0,156, 155);
    LibAria_DemoModeAddInputEvent(310, 2, 0,183, 228);
    LibAria_DemoModeAddInputEvent(100, 4, 0,183, 228);
    LibAria_DemoModeAddInputEvent(340, 2, 0,290, 212);
    LibAria_DemoModeAddInputEvent(120, 4, 0,290, 212);
    LibAria_DemoModeAddInputEvent(370, 2, 0,206, 236);
    LibAria_DemoModeAddInputEvent(110, 4, 0,206, 236);
    LibAria_DemoModeAddInputEvent(580, 2, 0,233, 90);
    LibAria_DemoModeAddInputEvent(100, 4, 0,233, 90);
    LibAria_DemoModeAddInputEvent(370, 2, 0,298, 128);
    LibAria_DemoModeAddInputEvent(100, 4, 0,298, 128);
    LibAria_DemoModeAddInputEvent(330, 2, 0,156, 153);
    LibAria_DemoModeAddInputEvent(110, 4, 0,156, 153);
    LibAria_DemoModeAddInputEvent(310, 2, 0,305, 214);
    LibAria_DemoModeAddInputEvent(110, 4, 0,305, 214);
    LibAria_DemoModeAddInputEvent(290, 2, 0,202, 234);
    LibAria_DemoModeAddInputEvent(110, 4, 0,202, 234);
    LibAria_DemoModeAddInputEvent(310, 2, 0,234, 107);
    LibAria_DemoModeAddInputEvent(110, 4, 0,234, 107);
    LibAria_DemoModeAddInputEvent(320, 2, 0,303, 133);
    LibAria_DemoModeAddInputEvent(80, 4, 0,303, 133);
    LibAria_DemoModeAddInputEvent(330, 2, 0,165, 155);
    LibAria_DemoModeAddInputEvent(90, 4, 0,165, 155);
    LibAria_DemoModeAddInputEvent(330, 2, 0,294, 205);
    LibAria_DemoModeAddInputEvent(110, 4, 0,294, 205);
    LibAria_DemoModeAddInputEvent(1050, 2, 0,457, 32);
    LibAria_DemoModeAddInputEvent(90, 4, 0,457, 32);
    LibAria_DemoModeAddInputEvent(2560, 2, 0,44, 243);
    LibAria_DemoModeAddInputEvent(110, 1, 0,49, 240);
    LibAria_DemoModeAddInputEvent(40, 1, 0,51, 239);
    LibAria_DemoModeAddInputEvent(10, 1, 0,56, 237);
    LibAria_DemoModeAddInputEvent(20, 1, 0,58, 236);
    LibAria_DemoModeAddInputEvent(10, 1, 0,61, 235);
    LibAria_DemoModeAddInputEvent(10, 1, 0,68, 233);
    LibAria_DemoModeAddInputEvent(10, 1, 0,71, 232);
    LibAria_DemoModeAddInputEvent(10, 1, 0,78, 230);
    LibAria_DemoModeAddInputEvent(20, 1, 0,82, 229);
    LibAria_DemoModeAddInputEvent(10, 1, 0,89, 226);
    LibAria_DemoModeAddInputEvent(10, 1, 0,97, 224);
    LibAria_DemoModeAddInputEvent(10, 1, 0,101, 223);
    LibAria_DemoModeAddInputEvent(10, 1, 0,108, 220);
    LibAria_DemoModeAddInputEvent(10, 1, 0,112, 219);
    LibAria_DemoModeAddInputEvent(10, 1, 0,119, 216);
    LibAria_DemoModeAddInputEvent(10, 1, 0,123, 215);
    LibAria_DemoModeAddInputEvent(10, 1, 0,131, 212);
    LibAria_DemoModeAddInputEvent(10, 1, 0,139, 209);
    LibAria_DemoModeAddInputEvent(10, 1, 0,142, 208);
    LibAria_DemoModeAddInputEvent(10, 1, 0,150, 205);
    LibAria_DemoModeAddInputEvent(10, 1, 0,154, 204);
    LibAria_DemoModeAddInputEvent(10, 1, 0,161, 201);
    LibAria_DemoModeAddInputEvent(10, 1, 0,169, 198);
    LibAria_DemoModeAddInputEvent(10, 1, 0,173, 197);
    LibAria_DemoModeAddInputEvent(10, 1, 0,181, 194);
    LibAria_DemoModeAddInputEvent(20, 1, 0,185, 193);
    LibAria_DemoModeAddInputEvent(10, 1, 0,192, 190);
    LibAria_DemoModeAddInputEvent(10, 1, 0,200, 187);
    LibAria_DemoModeAddInputEvent(10, 1, 0,204, 185);
    LibAria_DemoModeAddInputEvent(10, 1, 0,213, 182);
    LibAria_DemoModeAddInputEvent(20, 1, 0,217, 181);
    LibAria_DemoModeAddInputEvent(10, 1, 0,226, 177);
    LibAria_DemoModeAddInputEvent(10, 1, 0,235, 173);
    LibAria_DemoModeAddInputEvent(10, 1, 0,240, 171);
    LibAria_DemoModeAddInputEvent(10, 1, 0,251, 168);
    LibAria_DemoModeAddInputEvent(20, 1, 0,256, 166);
    LibAria_DemoModeAddInputEvent(10, 1, 0,266, 162);
    LibAria_DemoModeAddInputEvent(10, 1, 0,276, 158);
    LibAria_DemoModeAddInputEvent(10, 1, 0,281, 156);
    LibAria_DemoModeAddInputEvent(10, 1, 0,292, 153);
    LibAria_DemoModeAddInputEvent(10, 1, 0,297, 151);
    LibAria_DemoModeAddInputEvent(10, 1, 0,307, 147);
    LibAria_DemoModeAddInputEvent(10, 1, 0,317, 143);
    LibAria_DemoModeAddInputEvent(10, 1, 0,322, 141);
    LibAria_DemoModeAddInputEvent(10, 1, 0,332, 137);
    LibAria_DemoModeAddInputEvent(10, 1, 0,337, 135);
    LibAria_DemoModeAddInputEvent(10, 1, 0,346, 132);
    LibAria_DemoModeAddInputEvent(20, 1, 0,355, 128);
    LibAria_DemoModeAddInputEvent(10, 1, 0,359, 126);
    LibAria_DemoModeAddInputEvent(10, 1, 0,367, 122);
    LibAria_DemoModeAddInputEvent(10, 1, 0,371, 121);
    LibAria_DemoModeAddInputEvent(10, 1, 0,380, 117);
    LibAria_DemoModeAddInputEvent(20, 1, 0,389, 114);
    LibAria_DemoModeAddInputEvent(10, 1, 0,393, 112);
    LibAria_DemoModeAddInputEvent(10, 1, 0,402, 109);
    LibAria_DemoModeAddInputEvent(10, 1, 0,406, 107);
    LibAria_DemoModeAddInputEvent(10, 1, 0,414, 104);
    LibAria_DemoModeAddInputEvent(20, 1, 0,422, 102);
    LibAria_DemoModeAddInputEvent(10, 1, 0,425, 101);
    LibAria_DemoModeAddInputEvent(10, 1, 0,431, 98);
    LibAria_DemoModeAddInputEvent(10, 1, 0,433, 97);
    LibAria_DemoModeAddInputEvent(10, 1, 0,437, 96);
    LibAria_DemoModeAddInputEvent(10, 1, 0,440, 94);
    LibAria_DemoModeAddInputEvent(10, 1, 0,441, 94);
    LibAria_DemoModeAddInputEvent(10, 1, 0,444, 93);
    LibAria_DemoModeAddInputEvent(10, 1, 0,445, 92);
    LibAria_DemoModeAddInputEvent(10, 1, 0,446, 91);
    LibAria_DemoModeAddInputEvent(10, 1, 0,448, 91);
    LibAria_DemoModeAddInputEvent(10, 1, 0,449, 90);
    LibAria_DemoModeAddInputEvent(20, 1, 0,450, 90);
    LibAria_DemoModeAddInputEvent(10, 1, 0,451, 89);
    LibAria_DemoModeAddInputEvent(130, 1, 0,452, 89);
    LibAria_DemoModeAddInputEvent(820, 4, 0,452, 89);
    LibAria_DemoModeAddInputEvent(2120, 2, 0,456, 22);
    LibAria_DemoModeAddInputEvent(120, 4, 0,456, 22);
    LibAria_DemoModeAddInputEvent(1640, 2, 0,42, 243);
    LibAria_DemoModeAddInputEvent(140, 1, 0,49, 238);
    LibAria_DemoModeAddInputEvent(10, 1, 0,51, 237);
    LibAria_DemoModeAddInputEvent(10, 1, 0,53, 236);
    LibAria_DemoModeAddInputEvent(10, 1, 0,57, 234);
    LibAria_DemoModeAddInputEvent(20, 1, 0,63, 232);
    LibAria_DemoModeAddInputEvent(10, 1, 0,68, 230);
    LibAria_DemoModeAddInputEvent(10, 1, 0,79, 225);
    LibAria_DemoModeAddInputEvent(10, 1, 0,88, 222);
    LibAria_DemoModeAddInputEvent(20, 1, 0,97, 219);
    LibAria_DemoModeAddInputEvent(10, 1, 0,109, 216);
    LibAria_DemoModeAddInputEvent(20, 1, 0,121, 213);
    LibAria_DemoModeAddInputEvent(10, 1, 0,133, 209);
    LibAria_DemoModeAddInputEvent(10, 1, 0,148, 204);
    LibAria_DemoModeAddInputEvent(10, 1, 0,160, 199);
    LibAria_DemoModeAddInputEvent(10, 1, 0,179, 192);
    LibAria_DemoModeAddInputEvent(10, 1, 0,197, 185);
    LibAria_DemoModeAddInputEvent(10, 1, 0,212, 179);
    LibAria_DemoModeAddInputEvent(10, 1, 0,230, 172);
    LibAria_DemoModeAddInputEvent(10, 1, 0,244, 166);
    LibAria_DemoModeAddInputEvent(10, 1, 0,260, 159);
    LibAria_DemoModeAddInputEvent(10, 1, 0,281, 149);
    LibAria_DemoModeAddInputEvent(20, 1, 0,302, 140);
    LibAria_DemoModeAddInputEvent(10, 1, 0,324, 130);
    LibAria_DemoModeAddInputEvent(20, 1, 0,341, 124);
    LibAria_DemoModeAddInputEvent(10, 1, 0,356, 117);
    LibAria_DemoModeAddInputEvent(10, 1, 0,369, 112);
    LibAria_DemoModeAddInputEvent(20, 1, 0,382, 106);
    LibAria_DemoModeAddInputEvent(10, 1, 0,394, 102);
    LibAria_DemoModeAddInputEvent(10, 1, 0,406, 98);
    LibAria_DemoModeAddInputEvent(10, 1, 0,416, 95);
    LibAria_DemoModeAddInputEvent(10, 1, 0,427, 90);
    LibAria_DemoModeAddInputEvent(10, 1, 0,439, 85);
    LibAria_DemoModeAddInputEvent(10, 1, 0,451, 81);
    LibAria_DemoModeAddInputEvent(10, 1, 0,459, 77);
    LibAria_DemoModeAddInputEvent(20, 4, 0,466, 74);
    LibAria_DemoModeAddInputEvent(3980, 2, 0,456, 35);
    LibAria_DemoModeAddInputEvent(80, 4, 0,456, 35);
    LibAria_DemoModeAddInputEvent(1900, 2, 0,367, 215);
    LibAria_DemoModeAddInputEvent(230, 1, 0,361, 215);
    LibAria_DemoModeAddInputEvent(10, 1, 0,359, 215);
    LibAria_DemoModeAddInputEvent(20, 1, 0,356, 215);
    LibAria_DemoModeAddInputEvent(20, 1, 0,354, 215);
    LibAria_DemoModeAddInputEvent(10, 1, 0,347, 213);
    LibAria_DemoModeAddInputEvent(40, 1, 0,344, 212);
    LibAria_DemoModeAddInputEvent(20, 1, 0,340, 212);
    LibAria_DemoModeAddInputEvent(30, 1, 0,336, 212);
    LibAria_DemoModeAddInputEvent(20, 1, 0,309, 211);
    LibAria_DemoModeAddInputEvent(20, 1, 0,296, 211);
    LibAria_DemoModeAddInputEvent(30, 1, 0,283, 211);
    LibAria_DemoModeAddInputEvent(30, 1, 0,253, 208);
    LibAria_DemoModeAddInputEvent(30, 1, 0,220, 204);
    LibAria_DemoModeAddInputEvent(20, 1, 0,190, 199);
    LibAria_DemoModeAddInputEvent(20, 1, 0,165, 196);
    LibAria_DemoModeAddInputEvent(30, 4, 0,144, 195);
    LibAria_DemoModeAddInputEvent(2150, 2, 0,112, 147);
    LibAria_DemoModeAddInputEvent(300, 1, 0,117, 151);
    LibAria_DemoModeAddInputEvent(10, 1, 0,118, 151);
    LibAria_DemoModeAddInputEvent(20, 1, 0,119, 152);
    LibAria_DemoModeAddInputEvent(50, 1, 0,120, 152);
    LibAria_DemoModeAddInputEvent(30, 1, 0,121, 154);
    LibAria_DemoModeAddInputEvent(20, 1, 0,123, 155);
    LibAria_DemoModeAddInputEvent(20, 1, 0,124, 155);
    LibAria_DemoModeAddInputEvent(20, 1, 0,131, 160);
    LibAria_DemoModeAddInputEvent(10, 1, 0,135, 162);
    LibAria_DemoModeAddInputEvent(20, 1, 0,139, 165);
    LibAria_DemoModeAddInputEvent(20, 1, 0,162, 178);
    LibAria_DemoModeAddInputEvent(20, 1, 0,181, 188);
    LibAria_DemoModeAddInputEvent(20, 1, 0,203, 198);
    LibAria_DemoModeAddInputEvent(20, 1, 0,228, 208);
    LibAria_DemoModeAddInputEvent(30, 1, 0,257, 218);
    LibAria_DemoModeAddInputEvent(30, 1, 0,321, 232);
    LibAria_DemoModeAddInputEvent(20, 1, 0,437, 240);
    LibAria_DemoModeAddInputEvent(20, 4, 0,437, 240);
    LibAria_DemoModeAddInputEvent(3790, 2, 0,450, 30);
    LibAria_DemoModeAddInputEvent(110, 4, 0,450, 30);
    LibAria_DemoModeAddInputEvent(1490, 2, 0,19, 38);
    LibAria_DemoModeAddInputEvent(150, 4, 0,19, 38);
//END OF CUSTOM CODE
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