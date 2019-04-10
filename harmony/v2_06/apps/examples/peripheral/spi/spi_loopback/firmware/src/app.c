/*******************************************************************************
  MPLAB Harmony SPI Looback Example

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    MPLAB Harmony SPI loopback application logic

  Description:
    This file contains the MPLAB Harmony SPI loopback application logic.
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
#include "system/system.h"
#include "system/debug/sys_debug.h"
#include "framework/driver/spi/static/drv_spi_static.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************
uint8_t __attribute__ ((aligned (16))) txData[]  = "Testing 8-bit SPI!";
uint8_t __attribute__ ((aligned (16))) rxData[sizeof(txData)];
uint8_t txDataSize = sizeof(txData);


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
/* Application Data

  Summary:
    Contains application data

  Description:
    This structure contains application data.
*/

APP_DATA appObject;


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
    appObject.state = SPI_ENABLE;

}

/********************************************************
 * Application switch press routine
 ********************************************************/



/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void)
{

    switch (appObject.state)
    {
        case SPI_ENABLE:
            appObject.state = SPI_XFER_CHECK;

            break;

        case SPI_XFER_CHECK:
            memset(&rxData, 0, sizeof(txData));

            DRV_SPI0_BufferAddWriteRead(&txData, &rxData, txDataSize);

            if (memcmp(txData, rxData, txDataSize) != 0)
            {
                appObject.state = SPI_XFER_ERROR;
            }
            else
            {
                appObject.state = SPI_XFER_DONE;
            }

            break;


        case SPI_XFER_DONE:
            BSP_LEDOn((BSP_LED)LED_SUCCESS);
            BSP_LEDOff((BSP_LED)LED_FAILURE);
            break;

        case SPI_XFER_ERROR:
            BSP_LEDOn((BSP_LED)LED_FAILURE);
            BSP_LEDOff((BSP_LED)LED_SUCCESS);
            break;

        default:
            SYS_DEBUG (SYS_ERROR_FATAL,"ERROR! Invalid state\r\n");
            while (1);
    }
}

/*******************************************************************************
 End of File
 */

