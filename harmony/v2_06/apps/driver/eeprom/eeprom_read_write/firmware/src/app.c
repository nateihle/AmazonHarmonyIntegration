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

#define APP_DATA_ARRAY_SIZE  (16)
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

const uint8_t writeData[APP_DATA_ARRAY_SIZE] __attribute__((aligned(16))) = {
        0x00, 0x01, 0x02, 0x03,
        0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B,
        0x0C, 0x0D, 0x0E, 0x0F};  

uint8_t readData[APP_DATA_ARRAY_SIZE] __attribute__((aligned(16)));

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/


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
    appData.state = APP_STATE_INIT;
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    DRV_EEPROM_COMMAND_STATUS commandStatus = DRV_EEPROM_COMMAND_ERROR_UNKNOWN;

    switch(appData.state)
    {
        case APP_STATE_INIT:
            {
                appData.eepromHandle = DRV_EEPROM_Open(0, DRV_IO_INTENT_READWRITE);
                if (DRV_HANDLE_INVALID == appData.eepromHandle)
                {
                    /* Try again until a valid handle is received. */
                }
                else
                {
                    /* Get the geometry of the media. */
                    appData.eepromGeometry = DRV_EEPROM_GeometryGet(appData.eepromHandle);
                    appData.readBlockSize = appData.eepromGeometry->geometryTable->blockSize;
                    appData.writeBlockSize = (appData.eepromGeometry->geometryTable + 1)->blockSize;

                    appData.state = APP_WRITE_DATA;
                }
                break;
            }

        case APP_WRITE_DATA:
            {
                DRV_EEPROM_Write (appData.eepromHandle, &appData.commandHandle, (void*)writeData, 0, APP_DATA_ARRAY_SIZE / appData.writeBlockSize);
                if (appData.commandHandle != DRV_EEPROM_COMMAND_HANDLE_INVALID)
                {
                    appData.state = APP_WRITE_DATA_STATUS;
                }
                else
                {
                    appData.state = APP_ERROR;
                }
                break;
            }

        case APP_WRITE_DATA_STATUS:
            {
                commandStatus = DRV_EEPROM_CommandStatus (appData.eepromHandle, appData.commandHandle);

                if (commandStatus == DRV_EEPROM_COMMAND_COMPLETED)
                {
                    appData.state = APP_READ_DATA;
                }
                else if (commandStatus == DRV_EEPROM_COMMAND_ERROR_UNKNOWN)
                {
                    appData.state = APP_ERROR;
                }
                else
                {
                    /* Continue to remain in the same state. */
                }

                break;
            }

        case APP_READ_DATA:
            {
                DRV_EEPROM_Read (appData.eepromHandle, &appData.commandHandle, readData, 0, APP_DATA_ARRAY_SIZE/ appData.readBlockSize);
                if (appData.commandHandle != DRV_EEPROM_COMMAND_HANDLE_INVALID)
                {
                    appData.state = APP_READ_DATA_STATUS;
                }
                else
                {
                    appData.state = APP_ERROR;
                }
                break;
            }

        case APP_READ_DATA_STATUS:
            {
                commandStatus = DRV_EEPROM_CommandStatus (appData.eepromHandle, appData.commandHandle);

                if (commandStatus == DRV_EEPROM_COMMAND_COMPLETED)
                {
                    appData.state = APP_VERIFY_DATA;
                }
                else if (commandStatus == DRV_EEPROM_COMMAND_ERROR_UNKNOWN)
                {
                    appData.state = APP_ERROR;
                }
                else
                {
                    /* Continue to remain in the same state. */
                }

                break;
            }

        case APP_VERIFY_DATA:
            {
                uint8_t i = 0;

                appData.state = APP_IDLE;
                for (i = 0; i < APP_DATA_ARRAY_SIZE; i ++)
                {
                    if (readData[i] != writeData[i])
                    {
                        appData.state = APP_ERROR;
                        break;
                    }
                }
                break;
            }

        case APP_IDLE:
            {
                /* App demo completed successfully. */
                BSP_LEDStateSet(APP_LED_3, BSP_LED_STATE_ON);
                break;
            }

        case APP_ERROR:
        default:
            {
                /* App demo failed. */
                BSP_LEDStateSet(APP_LED_1, BSP_LED_STATE_ON);
                break;
            }
    }
}
 

/*******************************************************************************
 End of File
 */
