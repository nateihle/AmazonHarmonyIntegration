/*******************************************************************************
  USB DEVICE CDC Basic Demo Application

  File Name:
    app.c

  Summary:
    USB DEVICE CDC Basic Demo application

  Description:
    This file contains the USB DEVICE CDC Basic Demo application logic.
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
#include "system_config.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************

extern SYS_MODULE_OBJ sysConsoleObject;

static char rdBuf[APP_READ_BUFFER_SIZE] SYS_DEBUG_BUFFER_DMA_READY;
static char wrBuf[APP_WRITE_BUFFER_SIZE] SYS_DEBUG_BUFFER_DMA_READY;

/*****************************************************
 * Initialize the application data structure. All
 * application related variables are stored in this
 * data structure.
 *****************************************************/

APP_DATA appData =
{
    .state = APP_STATE_INIT,
    .switchPressed = false,
    .bytesRead = 0,
    .wrComplete = true,
    .rdComplete = true
};

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Routines
// *****************************************************************************
// *****************************************************************************

void APP_TestConsoleWrite(void)
{
    ssize_t bytesWritten = 0;
    int i;

    SYS_MESSAGE("\n\r");

    for (i = 0; i < APP_WRITE_BUFFER_SIZE; i++)
    {
        wrBuf[i] = '*';
    }

    bytesWritten = SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, wrBuf, APP_WRITE_BUFFER_SIZE);
    SYS_PRINT("\n\rWrote %d bytes to Console\n\r", bytesWritten);
}

void APP_TestDebugAPI(void)
{
    int var = 100;

    SYS_MESSAGE("\n\rTest Message!");
    SYS_DEBUG_MESSAGE(SYS_ERROR_FATAL, "\n\rTest Debug Message!");
    SYS_DEBUG_PRINT(SYS_ERROR_FATAL, "\n\rTest Error Message");
    SYS_DEBUG_PRINT(SYS_ERROR_FATAL, "\n\rTest Error Message %d", 1);
    SYS_DEBUG_PRINT(SYS_ERROR_FATAL, "\n\rTest Error Message %d, %d, %s", 2, var, "dbg_str");
    SYS_PRINT("\n\rSys Print test %d, %s", 1, "str1");
    SYS_PRINT("\n\rSys Print test %d, %s", 2, "str2");
    SYS_PRINT("\n\rSys Print test %d, %s", 3, "str3");
    SYS_PRINT("\n\rSys Print test %d, %s", 4, "str4");
    SYS_PRINT("\n\rSys Print test %d, %s", 5, "str5");
    SYS_PRINT("\n\rSys Print test %d, %s", 6, "str6");
    SYS_PRINT("\n\rSys Print test %d, %s", 7, "str7");
    SYS_PRINT("\n\rSys Print test %d, %s", 8, "str8\n\r");
    SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, wrBuf, 10);
    appData.wrComplete = false;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************

void APP_ReadComplete (void *handle)
{
    appData.rdComplete = true;
}

void APP_WriteComplete (void *handle)
{
    if ((handle != wrBuf) && (appData.state == APP_STATE_WRITE_TEST_2_WFC))
    {
        return;
    }
    else
    {
        appData.wrComplete = true;
    }
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

void APP_Initialize (void)
{
    
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
    switch (appData.state)
    {
        case APP_STATE_INIT:
            if (SYS_CONSOLE_Status(sysObj.sysConsole0) == SYS_STATUS_READY)
            {
                SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0, APP_ReadComplete, SYS_CONSOLE_EVENT_READ_COMPLETE);
                SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0, APP_WriteComplete, SYS_CONSOLE_EVENT_WRITE_COMPLETE);
                appData.state = APP_STATE_WRITE_TEST_1;
            }
            break;
        case APP_STATE_WRITE_TEST_1:
            /* Demonstrates polling write method */
            APP_TestConsoleWrite();
            appData.state = APP_STATE_WRITE_TEST_1_WFC;
            break;
        case APP_STATE_WRITE_TEST_1_WFC:
            if (SYS_CONSOLE_Status(sysObj.sysConsole0) == SYS_STATUS_READY)
            {
                appData.state = APP_STATE_WRITE_TEST_2;
                SYS_MESSAGE("Polling Write Test completed.\n\r");
            }
            break;
        case APP_STATE_WRITE_TEST_2:
            /* Demonstrates callback write method */
            APP_TestDebugAPI();
            appData.state = APP_STATE_WRITE_TEST_2_WFC;
            break;
        case APP_STATE_WRITE_TEST_2_WFC:
            if (appData.wrComplete)
            {
                appData.state = APP_STATE_READ_TEST_1;
                SYS_MESSAGE("\n\rCallback Write Test completed.\n\r");
                appData.wrComplete = false;
            }
            break;
        case APP_STATE_READ_TEST_1:
            /* Demonstrates polling read method */
            SYS_MESSAGE("\n\rEnter 10 characters: ");
            appData.bytesRead = SYS_CONSOLE_Read(SYS_CONSOLE_INDEX_0, STDIN_FILENO, rdBuf, 10);
            appData.state = APP_STATE_READ_TEST_1_WFC;
            break;
        case APP_STATE_READ_TEST_1_WFC:
            if (SYS_CONSOLE_Status(sysObj.sysConsole0) == SYS_STATUS_READY)
            {
                SYS_PRINT("\n\rPolling Read %d bytes: ", appData.bytesRead);
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, rdBuf, 10);
                SYS_MESSAGE("\n\r");
                appData.wrComplete = false;
                appData.state = APP_STATE_READ_TEST_2;
            }
            break;
        case APP_STATE_READ_TEST_2:
            /* Demonstrates callback read method */
            SYS_MESSAGE("\n\rEnter 10 characters: ");
            appData.bytesRead = SYS_CONSOLE_Read(SYS_CONSOLE_INDEX_0, STDIN_FILENO, rdBuf, 10);
            appData.rdComplete = false;
            appData.state = APP_STATE_READ_TEST_2_WFC;
            break;
        case APP_STATE_READ_TEST_2_WFC:
            if (appData.rdComplete)
            {
                SYS_PRINT("\n\rCallback Read %d bytes: ", appData.bytesRead);
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, rdBuf, 10);
                SYS_MESSAGE("\n\r");
                appData.state = APP_STATE_ECHO_TEST;
            }
            break;
        case APP_STATE_ECHO_TEST:
            SYS_MESSAGE("\n\rEcho Test. Type 'q' to quit.\n\r");
            appData.state = APP_STATE_ECHO_TEST_RD;
            break;
        case APP_STATE_ECHO_TEST_RD:
            if (appData.wrComplete)
            {
                SYS_CONSOLE_Read(SYS_CONSOLE_INDEX_0, STDIN_FILENO, rdBuf, 1);
                appData.state = APP_STATE_ECHO_TEST_WR;
                appData.rdComplete = false;
            }
            break;
        case APP_STATE_ECHO_TEST_WR:
            if (appData.rdComplete)
            {
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, rdBuf, 1);

                if (rdBuf[0] == 'q')
                {
                    appData.state = APP_STATE_DONE;
                    SYS_MESSAGE("\n\n\rAll Tests Completed\n\r");
                }
                else
                {
                    appData.state = APP_STATE_ECHO_TEST_RD;
                }

                appData.wrComplete = false;
            }
        case APP_STATE_DONE:
        case APP_STATE_ERROR:
        default:
            break;
    }
} 

/*******************************************************************************
 End of File
 */

