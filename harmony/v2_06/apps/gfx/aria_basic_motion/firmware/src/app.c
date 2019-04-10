/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define GET_TICKS() __builtin_mfc0(9, 0)
#define BATTERY_DISCHARGE_RATE 240000000
#define BATTERY_CHARGE_RATE 80000000

static uint32_t app_lastTick;
static int app_batteryIdx;
static int app_logoIdx;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
/* Touch event callbacks */
static void App_PowerOn(laButtonWidget* btn);
static void App_PowerOff(laButtonWidget* btn);
static void App_BatteryChargeOn(laButtonWidget* btn);
static void App_SaveMotion(laScreen* scr);
static void App_RestoreMotion(laScreen* scr);

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    app_batteryIdx = 0;
    app_logoIdx = 0;
    app_lastTick = 0;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    uint32_t currentTick;
    
    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true; 
            
            if (appInitialized)
            {
                appData.state = APP_STATE_IDLE;
                laButtonWidget_SetPressedEventCallback(ButtonWidget1, &App_PowerOn);
                laButtonWidget_SetReleasedEventCallback(ButtonWidget1, &App_PowerOff);
                laButtonWidget_SetPressedEventCallback(ButtonWidget2, &App_BatteryChargeOn);
                laScreen_SetHideEventCallback(default_ID, &App_SaveMotion);
                laScreen_SetShowEventCallback(default_ID, &App_RestoreMotion);
                laImageSequenceWidget_ShowImage(ImageSequenceWidget1, 0);
            }
            break;
        }

        case APP_STATE_IDLE:
        {
            break;
        }
        
        case APP_STATE_PLAY_ANIMATION:
        {           
            currentTick = GET_TICKS();

            if ( app_logoIdx++ < 16 )
                laImageSequenceWidget_ShowImage(ImageSequenceWidget, app_logoIdx); 
            else
                app_logoIdx=0;
            
            if ((currentTick - app_lastTick) > BATTERY_DISCHARGE_RATE)
            {
                app_lastTick = currentTick;

                laImageSequenceWidget_ShowImage(ImageSequenceWidget1, app_batteryIdx); 
                if ( app_batteryIdx == 6)
                {
                    laImageSequenceWidget_Stop(ImageSequenceWidget);
                    laButtonWidget_SetReleasedImage(ButtonWidget1, &greyPowerBut);
                    laButtonWidget_SetPressed(ButtonWidget1, false);
                    laWidget_SetEnabled((laWidget*) ButtonWidget1, false);
                    laWidget_SetEnabled((laWidget*) ButtonWidget2, true);
                    laWidget_SetVisible((laWidget*)ButtonWidget2, true);
                    appData.state = APP_STATE_IDLE;
                } else
                {
                    app_batteryIdx++;
                } 
            }
            break;
        }
        
        case APP_STATE_CHARGE_BATTERY:
        {           
             currentTick = GET_TICKS();

            if ((currentTick - app_lastTick) > BATTERY_CHARGE_RATE)
            {
                app_lastTick = currentTick;

                laImageSequenceWidget_ShowImage(ImageSequenceWidget1, app_batteryIdx); 
                if ( app_batteryIdx == 0)
                {
                    laButtonWidget_SetReleasedImage(ButtonWidget1, &blackPowerBut);
                    laWidget_SetEnabled((laWidget*) ButtonWidget1, true);
                    laButtonWidget_SetPressed(ButtonWidget2, false);
                    laWidget_SetVisible((laWidget*)ButtonWidget2, false);
                    appData.state = APP_STATE_IDLE;
                } else
                {
                    app_batteryIdx--;
                }
            }
            break;
        }
                
        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}


void App_PowerOn(laButtonWidget* btn)
 {
//    laImageSequenceWidget_Play(ImageSequenceWidget);
    appData.state = APP_STATE_PLAY_ANIMATION;
    app_lastTick = GET_TICKS();
 }
 
void App_PowerOff(laButtonWidget* btn)
 {
//    laImageSequenceWidget_Stop(ImageSequenceWidget);
    appData.state = APP_STATE_IDLE;
 }
  
void App_BatteryChargeOn(laButtonWidget* btn)
 {
    appData.state = APP_STATE_CHARGE_BATTERY;
    laWidget_SetEnabled((laWidget*) ButtonWidget2, false);
    app_batteryIdx=6;
 }

void App_SaveMotion(laScreen* scr)
{
    
}

void App_RestoreMotion(laScreen* scr)
{
    laButtonWidget_SetPressedEventCallback(ButtonWidget1, &App_PowerOn);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget1, &App_PowerOff);
    laButtonWidget_SetPressedEventCallback(ButtonWidget2, &App_BatteryChargeOn);    
}

/*******************************************************************************
 End of File
 */
