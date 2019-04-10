/*******************************************************************************
    BT Timer

  Company:
    Microchip Technology Inc.

  File Name:
    bttimer.c

  Summary:
    Contains the functional implementation of BT Timer.

  Description:
    This file contains the functional implementation of BT Timer.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#include "app.h"

static volatile bt_ulong        mTicks;
static volatile bt_ulong        mTimerCounter;
static TimerData                mTimers[TOTAL_TIMERS];
static bt_bool                  mProcessingTimers;
static volatile bool            intStatus;
extern unsigned char            mVolumeEvent ;

static void setTimer(bt_uint timerId, bt_ulong milliseconds, bttimer_TimerCallback callback);
static void clearTimer(bt_uint timerId);
static void updateTimerCounter(bt_ulong currentTicks);

// *****************************************************************************
// *****************************************************************************
// Section: Fuction Implementation
// *****************************************************************************
void bttask_pal_initTimer(void)
{
    memset(mTimers, 0, sizeof(mTimers));
    mTimerCounter = 0;
    mProcessingTimers = BT_FALSE;
}

void bttask_pal_handleTimerSignal(void)
{
    int i;
    bt_ulong ticks;

    mProcessingTimers = BT_TRUE;
    intStatus = SYS_INT_Disable();
    ticks = mTicks;
    if(intStatus)
    {
        SYS_INT_Enable();
    }

    for (i = 0; i < TOTAL_TIMERS; i++)
    {
        if (mTimers[i].duration != 0 &&
            (ticks - mTimers[i].startTime) >= mTimers[i].duration)
        {
            bt_timer_callback_fp callback = mTimers[i].callback;
            assert(callback != NULL);
            mTimers[i].duration = 0;
            mTimers[i].callback = NULL;
            (*callback)();
        }
    }
    updateTimerCounter(ticks);
    mProcessingTimers = BT_FALSE;
}


void bttimer_setTimer(bt_uint timerId, bt_ulong milliseconds, bttimer_TimerCallback callback)
{
    setTimer(timerId + BT_TIMER_MAX, milliseconds, callback);
}


void bttimer_clearTimer(bt_uint timerId)
{
    clearTimer(timerId + BT_TIMER_MAX);
}


void btapp_setTimer(BTAPP_TIMER_ID timerId, bt_ulong milliseconds, BTAPP_TIMER_CALLBACK callback)
{
    setTimer((int)timerId + (int)BT_TIMER_MAX + BTTIMER_MAX_TIMERS, milliseconds, (bttimer_TimerCallback)callback);
}


void btapp_clearTimer(BTAPP_TIMER_ID timerId)
{
    clearTimer((int)timerId + (int)BT_TIMER_MAX + BTTIMER_MAX_TIMERS);
}

// This function is called by dotstack to set a timer
void bt_oem_timer_set(bt_uint timerId, bt_ulong milliseconds, bt_timer_callback_fp callback)
{
    setTimer(timerId, milliseconds, (bttimer_TimerCallback)callback);
}

// This function is called by dotstack to clear a timer
void bt_oem_timer_clear(bt_uint timerId)
{
    clearTimer(timerId);
}

void bttimer_onSystemTick(void)
{
    mTicks++;
    if (mTimerCounter)
    {
        if (--mTimerCounter == 0)
    {
            bttask_setSignal(BTTASK_SIG_TIMER);
    }
    }
}


static void setTimer(bt_uint timerId, bt_ulong milliseconds, bttimer_TimerCallback callback)
{
    bt_ulong duration;
    bt_ulong tmpTimerCounter;
    bt_ulong tmpTicks;

    duration = (milliseconds + APP_BT_TICK_TIMER_MS - 1) / APP_BT_TICK_TIMER_MS; // number of ticks
    if (duration == 0) 
    {
        duration = 1; // set to at least 1 tick
    }
    intStatus = SYS_INT_Disable();
    tmpTicks = mTicks;
    tmpTimerCounter = mTimerCounter;
    if (tmpTimerCounter == 0 || tmpTimerCounter > duration)
    {
            mTimerCounter = duration;
    }
    if(intStatus)
    {
        SYS_INT_Enable();
    }
    mTimers[timerId].startTime = tmpTicks;
    mTimers[timerId].duration = duration;
    mTimers[timerId].callback = callback;
    if (!mProcessingTimers)
    {
            updateTimerCounter(tmpTicks);
    }
}


static void clearTimer(bt_uint timerId)
{
    mTimers[timerId].duration = 0;
    mTimers[timerId].callback = NULL;
    if (!mProcessingTimers)
    {
        bt_ulong ticks;
        intStatus = SYS_INT_Disable();
        ticks = mTicks;
        if(intStatus)
        {
            SYS_INT_Enable();
        }
        updateTimerCounter(ticks);
    }
}


static void updateTimerCounter(bt_ulong currentTicks)
{
    int i;
    bt_ulong tmp;
    bt_ulong timerCounter = 0xFFFFFFFF;

    for (i = 0; i < TOTAL_TIMERS; i++)
    {
        if (mTimers[i].duration != 0)
        {
            tmp = (mTimers[i].startTime + mTimers[i].duration) - currentTicks;
            if (tmp != 0 && tmp < timerCounter)
            {
                    timerCounter = tmp;
            }
        }
    }
    if (timerCounter != 0)
    {
        intStatus = SYS_INT_Disable();
        mTimerCounter = timerCounter;
        if(intStatus)
        {
            SYS_INT_Enable();
        }
    }
}
/*******************************************************************************
 End of File
 */



