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
#include "system_config.h"
#include "peripheral/osc/plib_osc.h"
#include "peripheral/devcon/plib_devcon.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************


/*****************************************************
 * Initialize the application data structure. All
 * application related variables are stored in this
 * data structure.
 *****************************************************/

APP_DATA appData = 
{
    //TODO - Initialize appData structure. 

};
// *****************************************************************************
/* Driver objects.

  Summary:
    Contains driver objects.

  Description:
    This structure contains driver objects returned by the driver init routines
    to the application. These objects are passed to the driver tasks routines.
*/


APP_DRV_OBJECTS appDrvObject;

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

/******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */

    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
}

/********************************************************
 * Application switch press routine
 ********************************************************/



/**********************************************************
 * Application tasks routine. This function implements the
 * application state machine.
 ***********************************************************/
void APP_Tasks ( void )
{
static uint8_t state = 0;

    switch(state)
    {

      case 0:
     /* Switch clock source from the primary oscillator to the FRC
       -PLL values should not be modified in run-time until the current clock
        source is switched to a non-PLL source */

      /* Unlock the Oscillator Registers */
    PLIB_DEVCON_SystemUnlock(DEVCON_ID_0);

    PLIB_OSC_SysClockSelect(OSC_ID_0, OSC_FRC);
    state = 1;

     case 1:
    if(!PLIB_OSC_ClockSwitchingIsComplete(OSC_ID_0))
        return;

    /* Change PLL values */
    PLIB_OSC_SysPLLMultiplierSelect(OSC_ID_0, 24);
    PLIB_OSC_SysPLLOutputDivisorSet(OSC_ID_0, OSC_SYSPLL_OUT_DIV_VALUE); //256 for MX

    /* Switch clock source back to the primary oscillator, now using new PLL values
       -SYSCLK was orignally 80MHz, now it is 375KHz */
    PLIB_OSC_SysClockSelect(OSC_ID_0, OSC_PRIMARY_WITH_PLL);
    state = 2;

        case 2:
    if (!PLIB_OSC_ClockSwitchingIsComplete(OSC_ID_0))
        return;

    /* Switch clock source to the Internal Low-Power RC Oscillator (LPRC)
       Now SYSCLK is running at ~31.25KHz */
    PLIB_OSC_SysClockSelect(OSC_ID_0, OSC_LPRC);
    state = 3;

    case 3:
    if(!PLIB_OSC_ClockSwitchingIsComplete(OSC_ID_0))
        return;

    /* Lock the Oscillator Registers again */
    PLIB_DEVCON_SystemLock(DEVCON_ID_0);
    state = 4;
        case 4:
            /* Turn on an LED to indicate success */
            BSP_LEDOn(BSP_LED_3);
    /* Stuck in this loop */
    return;
    }
} 

/*******************************************************************************
 End of File
 */

