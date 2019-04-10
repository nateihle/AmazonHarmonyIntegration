/*******************************************************************************
  MPLAB Harmony Application

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

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
#include "peripheral/ports/plib_ports.h"
#include "peripheral/adc/plib_adc.h"
#include "system/system.h"
#include "system/devcon/sys_devcon.h"
#include "framework/driver/adc/drv_adc_static.h"
#include "system/ports/sys_ports.h"

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

/******************************************************************************
  Function:
    void Set_LED_Status ( void )

   Description:
        Set LEDs to display the ADC average result.
*/

void Set_LED_Status(void)
{
    int i;

    appData.ledMask = 0;
    
    /* Creates a mask for the LEDs, corresponding to the value read from
       the potentiometer */
    appData.potValue >>= 7; /* 10-bit value to 3-bit value */
    for (i = 0; i <= appData.potValue; i++)
    {
        appData.ledMask |=  1<<(i);
    }
   /* Write the mask to the LEDs */
    SYS_PORTS_Write( PORTS_ID_0, PORT_CHANNEL_A, (PORTS_DATA_MASK)appData.ledMask );    
}  


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
    appData.state = APP_STATE_INIT;
    appData.dataReady = false;

}




/**********************************************************
 * Application tasks routine. This function implements the
 * application state machine.
 ***********************************************************/
void APP_Tasks ( void )
{
    /* check the application state*/
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
            /* Enable ADC */
            DRV_ADC_Open();
            appData.state = APP_STATE_SPIN;
            
            break;
        
        case APP_STATE_UPDATE_ADC_AVERAGE:
        {
            if(true == appData.dataReady)
            {
                Set_LED_Status();
            }

			appData.dataReady = false;
            appData.state = APP_STATE_SPIN;
        }
        break;

        case APP_STATE_SPIN:
        {
           if (true == appData.dataReady)
            {
                appData.state = APP_STATE_UPDATE_ADC_AVERAGE;
            }
        }
        break;
            
        /* The default state should never be executed. */
        default:
            break;
	}
}


/*******************************************************************************
 End of File
 */

