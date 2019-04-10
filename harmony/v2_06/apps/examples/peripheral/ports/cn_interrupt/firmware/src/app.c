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

#include "system_config.h"
//#include "bsp_config.h"
#include "app.h"
#include "peripheral/ports/plib_ports.h"
#include "system/int/sys_int.h"
#include "system/tmr/sys_tmr.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

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

// *****************************************************************************
/* void APP_TimerCallBack ( uintptr_t context, uint32_t tickCount )

 Summary:
   Called back by timer system service.  Toggles LED.

 Remarks:
    context and tickCount parameters ignored.
*/
void APP_TimerCallBack ( uintptr_t context, uint32_t tickCount )
{
    uint32_t portBits;

    /* If we're in a debounce state, check port to see if switch is pressed. */
    if ( appData.state == APP_STATE_DEBOUNCE_START ||
         appData.state == APP_STATE_DEBOUNCE_STOP )
    {
        appData.timeoutCounter++;

        portBits = PLIB_PORTS_Read(PORTS_ID_0, APP_CN_PORT_CHANNEL);
        if ( (1 << APP_CN_PORT_BIT) & portBits)
        {
            appData.debounceCounter = 0;
        }
        else
        {
            appData.debounceCounter++;
        }

        /* If switch is pressed for APP_DEBOUNCE_COUNT counts, it's real. */
        if (appData.debounceCounter >= APP_DEBOUNCE_COUNT)
        {
            appData.switchHasBeenPressed = true;
        }
    }
        
    /* If we're in a blinking state, blink the LED. */
    if ( appData.state ==  APP_STATE_LED_BLINKING ||
         appData.state == APP_STATE_DEBOUNCE_STOP )
    {
        BSP_LEDToggle(APP_BSP_LED);
    }    
}

// *****************************************************************************
/* void APP_SwitchChangeNoticed ( void )

 Summary:
    Called back by the Change Notice (CN) interrupt.  Clears CN condition and 
    flag and sets internal flag to identify the .

 Remarks:
    context and tickCount parameters ignored.
*/

void APP_SwitchChangeNoticed ( void )
{
    uint32_t __attribute__ ((unused)) temp;
        
    /* Read port to clear mismatch on change notice pins */
    temp = PLIB_PORTS_Read(PORTS_ID_0, APP_CN_PORT_CHANNEL);

    /* Clear the interrupt flag */
    SYS_INT_SourceStatusClear(APP_CN_PORT_INTERRUPT);

    /* Set flag indicating that a changed has been noticed. */
    appData.changeNoticed = true;
}


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
    /* Place the application state machine in its initial state. */
    appData.state                   = APP_STATE_INIT;
    appData.changeNoticed           = false;
    appData.switchHasBeenPressed    = false;
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    /* Check the application's current state. */
    switch (appData.state)
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            //this flag & state change ensures that LED blinks at the start
			appData.switchHasBeenPressed = true;
            appData.state = APP_STATE_TIMER_OBJECT_CREATE;;
            break;
        }

        /* State to create the timer object for periodic alarm */
        case APP_STATE_TIMER_OBJECT_CREATE:
        {
            appData.tmrObj = SYS_TMR_ObjectCreate(APP_LED_BLINK_DELAY, 1, APP_TimerCallBack, SYS_TMR_FLAG_PERIODIC);
            if(SYS_TMR_HANDLE_INVALID != appData.tmrObj)
            {
                appData.state = APP_STATE_DEBOUNCE_START;
            }
            break;
        }
        
        case APP_STATE_DEBOUNCE_START:
        {
            if (appData.timeoutCounter < APP_DEBOUNCE_TIMEOUT)
            {
                if (appData.switchHasBeenPressed)
                {
                    appData.changeNoticed = false;
                    appData.state = APP_STATE_LED_BLINKING;
                }
            }
            else
            {
                SYS_TMR_ObjectDelete(appData.tmrObj);            
                appData.state = APP_STATE_IDLE;
            }
            break;
        }
        
        case APP_STATE_LED_BLINKING:
        {
            if (appData.changeNoticed)
            {
                appData.changeNoticed           = false;
                appData.switchHasBeenPressed    = false;
                appData.debounceCounter         = 0;
                appData.timeoutCounter          = 0;
                appData.state = APP_STATE_DEBOUNCE_STOP;
            }
            break;
        }

        case APP_STATE_DEBOUNCE_STOP:
        {
            if (appData.timeoutCounter < APP_DEBOUNCE_TIMEOUT)
            {
                if (appData.switchHasBeenPressed)
                {
                    appData.changeNoticed = false;
                    BSP_LEDOff(APP_BSP_LED);
                    SYS_TMR_ObjectDelete(appData.tmrObj);            
                    appData.state = APP_STATE_IDLE;
                }
            }
            else
            {
                appData.changeNoticed           = false;
                appData.switchHasBeenPressed    = false;
                appData.state = APP_STATE_LED_BLINKING;
            }
            break;
        }
        
        case APP_STATE_IDLE:
        default:
        {
            if (appData.changeNoticed)
            {
                /* Start timer and prepare to detect switch press. */
                appData.changeNoticed           = false;
                appData.switchHasBeenPressed    = false;
                appData.debounceCounter         = 0;
                appData.timeoutCounter          = 0;
                appData.state   = APP_STATE_TIMER_OBJECT_CREATE;
            }
            break;
        }
    }
}
 

/*******************************************************************************
 End of File
 */

