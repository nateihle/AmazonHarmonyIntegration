/*******************************************************************************
  MPLAB Harmony Timer3 Interrupt Example

  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    MPLAB Harmony Timer3 Interrupt application logic

  Description:
    This file contains the MPLAB Harmony Timer3 Interrupt application logic.
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
#include "system_definitions.h"

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
    .state = TMR_START,
    .sysDevconObject = (SYS_MODULE_OBJ)NULL
};

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

void APP_TimerCallback( uintptr_t context, uint32_t alarmCount )
{
    appData.tmrIntTriggered = true;
}

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
    /* Put the application into its initial state */
    appData.state = TMR_START;
    
    appData.tmrHandle = DRV_TMR_Open(DRV_TMR_INDEX_0, 0);

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
    /* check the application state*/
    switch ( appData.state )
    {
        case TMR_START:
            appData.tmrIntTriggered = 0;
            
            DRV_TMR_AlarmRegister(appData.tmrHandle, 16000, false, 0, APP_TimerCallback);
            
            DRV_TMR_AlarmEnable(appData.tmrHandle, true);

            /* Start Timer 3 */
            DRV_TMR_Start(appData.tmrHandle);

            appData.state =  TMR_INT_CHECK;

            break;

        case TMR_INT_CHECK:
            if (appData.tmrIntTriggered)
            {
                appData.state = TMR_DONE;
            }
            else
                appData.state = TMR_INT_CHECK;

            break;

        case TMR_DONE:
            BSP_LEDOn(LED_SUCCESS);

            break;
        default:
            SYS_ASSERT(false,"ERROR! Invalid state\r\n");
            break;
    }   
} 

/*******************************************************************************
 End of File
 */

