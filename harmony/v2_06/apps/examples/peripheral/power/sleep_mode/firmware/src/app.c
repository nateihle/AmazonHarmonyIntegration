/*******************************************************************************
  MPLAB Harmony Application 
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    Application Template

  Description:
    This file contains the application logic.
 *******************************************************************************/


// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "peripheral/devcon/plib_devcon.h"
#include "peripheral/ports/plib_ports.h"
#include "peripheral/wdt/plib_wdt.h"
#include "peripheral/power/plib_power.h"
#include "peripheral/osc/plib_osc.h"

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


unsigned int i = 0;

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine
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
    /* Put the application into its initial state */
    appData.state = APP_STATE_START_SLEEP_MODE;
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    switch(appData.state)
    {
        case APP_STATE_START_SLEEP_MODE:
        {
            // Enable the watchdog timer, turn on LED to indicate the device will be in sleep
            // mode, then service the watchdog timer
            PLIB_WDT_Enable(WDT_ID_0);
            BSP_LEDOn(BSP_LED_1);
            PLIB_WDT_TimerClear(WDT_ID_0);
            
            // Put device into power saving sleep mode - the device will resume normal
            // operation when the WDT fails to be serviced and triggers a WDT timeout
            // reset
            PLIB_DEVCON_SystemUnlock(DEVCON_ID_0);
            PLIB_OSC_OnWaitActionSet(OSC_ID_0, OSC_ON_WAIT_SLEEP);
            PLIB_DEVCON_SystemLock(DEVCON_ID_0);
            asm("WAIT");

            appData.state = APP_STATE_COUNT;
            break;
        }

        case APP_STATE_COUNT:
        {

            /* Keep incrementing the count if it's less than the blink delay */
            if (i < APP_LED_BLINK_DELAY)
            {
                /* Increment count. */
                i++;
            }

            else
            {
                /* If count is reached, switch states */
                appData.state = APP_STATE_BLINK;
            }

            /* Keep servicing WDT to keep system from resetting */
            PLIB_WDT_TimerClear(WDT_ID_0);
            break;
        }

        case APP_STATE_BLINK:
        {
            if(PLIB_POWER_DeviceWasInSleepMode(POWER_ID_0))
            {
                // Clear the Sleep status
                PLIB_POWER_ClearSleepStatus (POWER_ID_0);
                // Device is now out of sleep mode - Turn off the LED
                BSP_LEDToggle(BSP_LED_1);
            }
            
            /* Toggle LED */
            BSP_LEDToggle(BSP_LED_3);

            /* Put the application back to the timeout state*/
            appData.state = APP_STATE_COUNT;

            /* Restart count. */
            i = 0;

            break;
        }

        /* Should not come here during normal operation */
        default:
        {
            PLIB_ASSERT(false , "unknown application state");

            break;
        }
    }
} 

/*******************************************************************************
 End of File
 */

