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

#include <string.h>
#include "app.h"
#include "app_sqi_static.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************
#define FLASH_PAGE_ADDR             0x010000
#define FLASH_READ_SIZE             256
uint8_t __attribute__((coherent))   writeBuffer[FLASH_READ_SIZE];
uint8_t __attribute__((coherent))   readBuffer[FLASH_READ_SIZE];

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
    APP_SQI_Initialize();
    
    /* Place the App state machine in it's initial state. */
    appData.state = APP_STATE_INIT_FLASH;
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
    SQI_STATUS sqiStatus;
    uint32_t   writeReadLoop;
            
    /* Check the application state*/
    switch ( appData.state )
    {
        /* Initialize the Flash for writes and reads. */
        case APP_STATE_INIT_FLASH:
        {
            APP_SQI_Flash_Initialize();

            /* Update the state */
            appData.state = APP_STATE_FLASH_ID_READ;

            break;
        }

        /* Read flash ID until successful */
        case APP_STATE_FLASH_ID_READ:
        {
            /* Get the ID read status */
            sqiStatus = APP_SQI_Flash_ID_Check();
            if ( sqiStatus !=  SQI_STATUS_SUCCESS)
                /* Update the state */
                appData.state = APP_STATE_FLASH_ID_READ;
            else
                /* Update the state */
                appData.state = APP_STATE_ERASE_FLASH;
            break;

        }

        /* Erase flash*/
        case APP_STATE_ERASE_FLASH:
        {
            APP_SQI_Flash_Erase();

            /* Update the state */
            appData.state = APP_STATE_WRITE_FLASH;
        }
            
        /* Write flash*/
        case APP_STATE_WRITE_FLASH:
        {
            for (writeReadLoop = 0; writeReadLoop < FLASH_READ_SIZE; writeReadLoop ++)
                writeBuffer[writeReadLoop] = writeReadLoop;
            
            APP_SQI_PIO_PageWrite(FLASH_PAGE_ADDR, writeBuffer);

            /* Update the state */
            appData.state = APP_STATE_READ_FLASH_DMA_MODE;
        }

        /* Read flash ID until successful */
        case APP_STATE_READ_FLASH_DMA_MODE:
        {
            /* Get the ID read status */
            APP_SQI_XIP_Read(FLASH_PAGE_ADDR, FLASH_READ_SIZE, readBuffer);
            
            if (memcmp(writeBuffer, readBuffer, FLASH_READ_SIZE) == 0)
            {
                appData.state = APP_STATE_DONE;
            }
            else
            {
                appData.state = APP_STATE_ERROR;
            }
            
            break;
        }

        /* Idle state (do nothing) */
        case APP_STATE_DONE:
            BSP_LEDStateSet(BSP_LED_3, BSP_LED_STATE_ON);
        case APP_STATE_ERROR:
        default:
            break;
    }
}

/*******************************************************************************
 End of File
 */

