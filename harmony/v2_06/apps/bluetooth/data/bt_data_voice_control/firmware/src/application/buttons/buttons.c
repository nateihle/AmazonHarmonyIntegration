/*******************************************************************************
    Buttons application support

  Company:
    Microchip Technology Inc.

  File Name:
    buttons.c

  Summary:
    Contains the functional implementation of buttons.

  Description:
    This file contains the functional implementation of buttons.
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
// DOM-IGNORE-END

#include "app.h"


static uint32_t mButtonState;
static uint32_t mNewButtonState;
static uint8_t  mRepeatButton;
static uint16_t mRepeatCount;
static volatile uint32_t intStatus;
static uint8_t  mButtonEvent = 0;

static void setRepeatTimer(unsigned delay);
static void clearRepeatTimer(void);

// *****************************************************************************
// *****************************************************************************
// Section: Fuction Implementation
// *****************************************************************************
void buttonsInit(void)
{
    mButtonState = APP_READ_BUTTON_PORTS();
    APP_ENABLE_BUTTON_CHANGE_NOTICE();

    mRepeatButton = 0;
    mRepeatCount = 0;
    mButtonEvent = 0;
}

void buttonsTask(void)
{
    unsigned int newState = mNewButtonState;
    unsigned int changed = mButtonState ^ newState;

    if (mButtonEvent == 1)
    {
        mButtonEvent = 0;
        
        if (changed & APP_BUTTON1_PIN)
        {
            if ((newState & APP_BUTTON1_PIN) == 0)
            {
                //DOWN
                btapp_onButtonDown(BTAPP_BUTTON_S1, 0);
                if (mRepeatButton == 0)
                {
                    mRepeatButton = BTAPP_BUTTON_S1;
                    mRepeatCount = 0;
                    setRepeatTimer(APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD);
                }
            }
            else
            {
                //UP
                if (mRepeatButton == BTAPP_BUTTON_S1)
                {
                    btapp_onButtonUp(BTAPP_BUTTON_S1, mRepeatCount);
                    mRepeatButton = 0;
                    clearRepeatTimer();
                }
                else
                {
                    btapp_onButtonUp(BTAPP_BUTTON_S1, 0);
                }
            }
        }

        if (changed & APP_BUTTON2_PIN)
        {
            if ((newState & APP_BUTTON2_PIN) == 0)
            {
                //DOWN
                btapp_onButtonDown(BTAPP_BUTTON_S2, 0);
                if (mRepeatButton == 0)
                {
                    mRepeatButton = BTAPP_BUTTON_S2;
                    mRepeatCount = 0;
                    setRepeatTimer(APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD);
                }
            }
            else
            {
                //UP
                if (mRepeatButton == BTAPP_BUTTON_S2)
                {
                    btapp_onButtonUp(BTAPP_BUTTON_S2, mRepeatCount);
                    mRepeatButton = 0;
                    clearRepeatTimer();
                }
                else
                {
                    btapp_onButtonUp(BTAPP_BUTTON_S2, 0);
                }
            }
        }

        if (changed & APP_BUTTON3_PIN)
        {
            if ((newState & APP_BUTTON3_PIN) == 0)
            {
                //DOWN
                btapp_onButtonDown(BTAPP_BUTTON_S3, 0);
                if (mRepeatButton == 0)
                {
                    mRepeatButton = BTAPP_BUTTON_S3;
                    mRepeatCount = 0;
                    setRepeatTimer(APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD);
                }
            }
            else
            {
                //UP
                if (mRepeatButton == BTAPP_BUTTON_S3)
                {
                    btapp_onButtonUp(BTAPP_BUTTON_S3, mRepeatCount);
                    mRepeatButton = 0;
                    clearRepeatTimer();
                }
                else
                {
                    btapp_onButtonUp(BTAPP_BUTTON_S3, 0);
                }
            }
        }
        if (changed & APP_BUTTON4_PIN)
        {
            if ((newState & APP_BUTTON4_PIN) == 0)
            {
                //DOWN
                btapp_onButtonDown(BTAPP_BUTTON_S4, 0);
                if (mRepeatButton == 0)
                {
                    mRepeatButton = BTAPP_BUTTON_S4;
                    mRepeatCount = 0;
                    setRepeatTimer(APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD);
                }
            }
            else
            {
                //UP
                if (mRepeatButton == BTAPP_BUTTON_S4)
                {
                    btapp_onButtonUp(BTAPP_BUTTON_S4, mRepeatCount);
                    mRepeatButton = 0;
                    clearRepeatTimer();
                }
                else
                {
                    btapp_onButtonUp(BTAPP_BUTTON_S4, 0);
                }
            }
        }
        if (changed & APP_BUTTON5_PIN)
        {
            if ((newState & APP_BUTTON5_PIN) == 0)
            {
                //DOWN
                btapp_onButtonDown(BTAPP_BUTTON_S5, 0);
                if (mRepeatButton == 0)
                {
                    mRepeatButton = BTAPP_BUTTON_S5;
                    mRepeatCount = 0;
                    setRepeatTimer(APP_BT_BUTTON_REPEAT_TIMER_INIT_PERIOD);
                }
            }
            else
            {
                //UP
                if (mRepeatButton == BTAPP_BUTTON_S5)
                {
                    btapp_onButtonUp(BTAPP_BUTTON_S5, mRepeatCount);
                    mRepeatButton = 0;
                    clearRepeatTimer();
                }
                else
                {
                    btapp_onButtonUp(BTAPP_BUTTON_S5, 0);
                }
            }
        }
        mButtonState = newState;
    }
}

void bttask_pal_handleButtonRepeatSignal(void)
{
    if (mRepeatButton)
    {
        mRepeatCount++;
        setRepeatTimer(APP_BT_BUTTON_REPEAT_TIMER_REPEAT_PERIOD);
        btapp_onButtonDown(mRepeatButton, mRepeatCount);
    }
}

static void setRepeatTimer(uint32_t delay)
{
    intStatus = SYS_INT_Disable();
    {
        DRV_TMR_CounterValueSet(appData.repeatTmrHandle, delay);
        DRV_TMR_Start(appData.repeatTmrHandle);
    }
    if(intStatus)
    {
        SYS_INT_Enable();
    }
}

static void clearRepeatTimer(void)
{
    DRV_TMR_Stop(appData.repeatTmrHandle);
}

void buttons_handleInterrupt(unsigned int newButtonState)
{
    // Remember new buttons state
    mNewButtonState = newButtonState;
    // Set BTTASK_SIG_BUTTONS signal. This will eventualy invoke its hanlder
    mButtonEvent = 1;
}
/*******************************************************************************
 End of File
 */
