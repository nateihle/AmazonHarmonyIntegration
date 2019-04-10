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
uint8_t __attribute__ ((aligned (32))) txData[]  = "SELF LOOPBACK DEMO FOR SPI!";
uint8_t __attribute__ ((aligned (32))) rxData[sizeof(txData)];

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void SPIEventHandler (DRV_SPI_BUFFER_EVENT event,
        DRV_SPI_BUFFER_HANDLE bufferHandle, void * context )
{
    if (event == DRV_SPI_BUFFER_EVENT_COMPLETE )
    {
        appData.state = APP_STATE_SPI_DATA_COMPARISON;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


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
    appData.state = APP_STATE_SPI_OPEN;
    appData.drvSPIHandle = DRV_HANDLE_INVALID;
    
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

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_SPI_OPEN:
        {
            /* Open the SPI Driver */
            if(appData.drvSPIHandle == DRV_HANDLE_INVALID)
            {
                appData.drvSPIHandle = DRV_SPI_Open( DRV_SPI_INDEX_0, DRV_IO_INTENT_READWRITE );
            }
            if(appData.drvSPIHandle != DRV_HANDLE_INVALID)
            {
                appData.state = APP_STATE_SPI_TRANSFER_START;
            }
            break;
        }

        case APP_STATE_SPI_TRANSFER_START:
        {
            /* Clear the receive data array */
            memset(&rxData, 0, sizeof(txData));
            
            appData.state = APP_STATE_IDLE;
            appData.bufferHandle = DRV_SPI_BufferAddWriteRead2(appData.drvSPIHandle, &txData, sizeof(txData), &rxData, sizeof(rxData), SPIEventHandler, NULL, &appData.bufferHandle2 );
            break;
        }
        case APP_STATE_IDLE:
        {
            break;
        }
        case APP_STATE_SPI_DATA_COMPARISON:
        {
            /* BufferStatus polling is not needed here as we have already checked the buffer status in the SPIEventHandler,
             *  but status polling is still done just for the demonstration purpose. */
            /* Also note that bufferHandle2 is used for the status polling, not bufferHandle */
            if (DRV_SPI_BufferStatus(appData.bufferHandle2) == DRV_SPI_BUFFER_EVENT_COMPLETE)
            {
                if (memcmp(txData, rxData, sizeof(txData)) != 0)
                {
                    appData.state = APP_STATE_SPI_XFER_ERROR;
                }
                else
                {
                    appData.state = APP_STATE_SPI_XFER_DONE;
                }
            }
            break;
        }
        
        case APP_STATE_SPI_XFER_DONE:
            BSP_LEDOn((BSP_LED)LED_SUCCESS);
            BSP_LEDOff((BSP_LED)LED_FAILURE);
            break;

        case APP_STATE_SPI_XFER_ERROR:
            BSP_LEDOn((BSP_LED)LED_FAILURE);
            BSP_LEDOff((BSP_LED)LED_SUCCESS);
            break;

        default:
            while (1);
    }
}

 

/*******************************************************************************
 End of File
 */
