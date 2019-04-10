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

/* Array in the RAM to store the data */
uint32_t databuff[APP_DEVICE_ROW_SIZE_DIVIDED_BY_4] ALIGN(16);

APP_DATA appData;

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

//*******************************************************************************



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

/**********************************************************
 * Application tasks routine. This function implements the
 * application state machine.
 ***********************************************************/
void APP_Tasks ( void )
{
   unsigned int x;
   switch(appData.state)
   {
      case APP_STATE_INIT:
        appData.flashHandle = DRV_FLASH_Open(DRV_FLASH_INDEX_0, intent);
        appData.state = APP_STATE_NVM_FILL_DATABUF_AND_ERASE_STATE;
        break;

      case APP_STATE_NVM_FILL_DATABUF_AND_ERASE_STATE:
        for (x = 0; x < APP_DATABUFF_SIZE; x++)
        {
            databuff[x] = x;
        }
        BSP_LEDOff(USERLED_SUCCESS);
        BSP_LEDOff(USERLED_ERROR);

        /* Erase the page which consist of the row to be written */
        DRV_FLASH_ErasePage(appData.flashHandle, APP_PROGRAM_FLASH_BASE_ADDRESS);
        appData.state = APP_STATE_NVM_ERASE_COMPLETION_CHECK;
        break;

      case APP_STATE_NVM_ERASE_COMPLETION_CHECK:
        if(!DRV_FLASH_IsBusy(appData.flashHandle))
        {
            appData.state = APP_STATE_NVM_WRITE_START;
        }
        break;

      case APP_STATE_NVM_WRITE_START:
        /* Erase Success */
        /* Write a row of data to PROGRAM_FLASH_BASE_ADDRESS, using databuff array as the source */
        DRV_FLASH_WriteRow(appData.flashHandle, APP_PROGRAM_FLASH_BASE_ADDRESS, databuff);
        appData.state = APP_STATE_NVM_WRITE_COMPLETION_CHECK_AND_VERIFY_CHECK;
        break;

      case APP_STATE_NVM_WRITE_COMPLETION_CHECK_AND_VERIFY_CHECK:
        if(!DRV_FLASH_IsBusy(appData.flashHandle))
        {
            /* Verify that data written to flash memory is valid (databuff array read from kseg1) */
            if (!memcmp(databuff, (void *)APP_PROGRAM_FLASH_BASE_ADDRESS_VALUE, sizeof(databuff)))
            {
                appData.state = APP_STATE_NVM_SUCCESS_STATE;
            }
            else
            {
                appData.state = APP_STATE_NVM_ERROR_STATE;
            }
        }
        break;

         
      case APP_STATE_NVM_ERROR_STATE:
        /*stay here, nvm had a failure*/
        BSP_LEDOn(USERLED_ERROR);
        BSP_LEDOff(USERLED_SUCCESS);
        break;

      case APP_STATE_NVM_SUCCESS_STATE:
        BSP_LEDOn(USERLED_SUCCESS);
        BSP_LEDOff(USERLED_ERROR);
        break;
   }
} 

/*******************************************************************************
 End of File
 */

