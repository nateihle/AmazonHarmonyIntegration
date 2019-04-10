/*******************************************************************************
  MPLAB Harmony Reset Handler Example

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    MPLAB Harmony reset_handler main function

  Description:
    This example does checks on various reset flags, assigning
    each one to an Explorer 16 LED. If the flag is set, the LED
    is turned on and the reset flag is cleared.

  Tested with:
    -PIC32MX795F512L on the Explorer-16 Demo Board
    -XC32 compiler, MPLAB X IDE
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
    /* Place the App state machine in its initial state. */
    appData.state = RESET_INIT;
}

/**********************************************************
 * Application tasks routine. This function implements the
 * application state machine.
 ***********************************************************/
void APP_Tasks ( void )
{
    switch ( appData.state )
    {
        case RESET_INIT:

            /* This condition will be true when only POR status is ON */
            if (PLIB_RESET_ReasonGet(RESET_ID_0) == RESET_REASON_POWERON)
            {
                /* Glow an LED to indicate the state */
                BSP_LEDOn(LED_POR);
            }
            /* This condition will be true when only MCLR status is ON */
            else if (PLIB_RESET_ReasonGet(RESET_ID_0) == RESET_REASON_MCLR)
            {
                /* Glow an LED to indicate the state */
                BSP_LEDOn(LED_MCLR);
            }
            /* This condition will be true when POR and MCLR both status is ON */
            else if (PLIB_RESET_ReasonGet(RESET_ID_0) == (RESET_REASON_POWERON | RESET_REASON_MCLR))
            {
                /* Glow an LED to indicate the state */
                BSP_LEDOn(LED_MCLR);
                BSP_LEDOn(LED_POR);
            }
            /* This condition will be true when WDT status is ON */
            else if (PLIB_RESET_ReasonGet(RESET_ID_0) == (RESET_REASON_WDT_TIMEOUT))
            {
                /* Glow an LED to indicate the state */
                BSP_LEDOn(LED_WDT);
            }
            /* This condition will be true for any other reset conditions */
            else
            {
                PLIB_RESET_ReasonClear(RESET_ID_0, RESET_REASON_ALL);
                BSP_LEDOff(LED_POR);
                BSP_LEDOff(LED_MCLR);
                BSP_LEDOff(LED_WDT);
            }

            /* press the switch to clear all the RESET status */
            if(BSP_SwitchStateGet(SWITCH_RESET_REASON_CLEAR) == BSP_SWITCH_STATE_PRESSED)
            {
                PLIB_RESET_ReasonClear(RESET_ID_0, RESET_REASON_ALL);
                BSP_LEDOff(LED_POR);
                BSP_LEDOff(LED_MCLR);
                BSP_LEDOff(LED_WDT);
            }

            /* press the switch to enable Watch Dog Timer */
            if(BSP_SwitchStateGet(SWITCH_ENABLE_WDT) == BSP_SWITCH_STATE_PRESSED)
            {
                PLIB_WDT_Enable(WDT_ID_0);
            }
    }
}

/*******************************************************************************
 End of File
 */

