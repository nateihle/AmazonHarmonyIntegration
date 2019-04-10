/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    UART DEBUG CONSOLE  Basic Demo application

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
#include "system/debug/sys_debug.h"
#include "system_definitions.h"

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

uint32_t testVar = 0;               //DEBCON1207
SYS_STATUS testconsoleStatus;       //DEBCON1207

char rdBuf[APP_READ_BUFFER_SIZE];
char wrBuf[APP_WRITE_BUFFER_SIZE];

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
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void APP_ReadComplete (void *handle)
{
    size_t *readSize = handle;
    appData.bytesRead = *readSize;
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
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


static void APP_TestConsoleWrite(void)
{
    ssize_t bytesWritten = 0;
    int i;
    
    for (i = 0; i < APP_WRITE_BUFFER_SIZE; i++)
    {
        wrBuf[i] = '*';
    }

    bytesWritten = SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, wrBuf, APP_WRITE_BUFFER_SIZE);
    SYS_PRINT("\n\rWrote %d bytes to Console\n\r", bytesWritten);
}

static void APP_TestDebugAPI(void)
{
        SYS_DEBUG_ErrorLevelSet(SYS_ERROR_ERROR);
        SYS_MESSAGE("\n\rTest Message!");
        SYS_DEBUG_MESSAGE(SYS_ERROR_DEBUG, "\n\rTest Debug Message!");
        SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "\n\rTest Error Message %d", 1);
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

void APP_Reset ()
{
    appData.bytesRead = 0;
    appData.rdComplete = true;
    appData.wrComplete = true;
}

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
    appData.state = APP_STATE_INIT;
    
    
/**************************************************************************
 Note: This code below is a safe-guard when using UART-2 on PIC32MZ-EF SK
 paired with MEB-II board. The USART-2 lines are shared with Bluetooth module.
 So if Bluetooth module is not disable, then UART-RX functionality will fail. 
 RB2 connects to the enable line of Bluetooth module, hence writing a LOW to
 RB2 to disable the module. 
 If PIC32MZ-EF SK is used stand-alone then this is not needed
 ******************************************************************************/    
    
#if defined  __PIC32MZ &&  (__PIC32_FEATURE_SET0 == 'E') && (__PIC32_FEATURE_SET1 == 'F')
    
    // make pin RB2 as output
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_2);
    //write RB2 to 0 to disable Bluetooth module
    PLIB_PORTS_PinClear ( PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_2 );
    
#endif
    
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

void APP_Tasks (void )
{
    int i;
    SYS_STATUS consoleStatus;

    consoleStatus = SYS_CONSOLE_Status(sysObj.sysConsole0);
    
    testconsoleStatus = consoleStatus;

    //Do not proceed in the current app state unless the console is ready
    if (consoleStatus != SYS_STATUS_READY)
    {
        if (consoleStatus == SYS_STATUS_ERROR)
        {            
            if (appData.state == APP_STATE_WRITE_TEST_3)       
            {
                APP_Reset();                                       
                SYS_CONSOLE_Flush(SYS_CONSOLE_INDEX_0);            
                SYS_MESSAGE("\n\r\n\rWrite Queue Overflowed! Flushed console.\n\r\n\r");
                appData.state = APP_STATE_READ_TEST_1;
            }
        }
        
        return;
    }

    switch (appData.state)
    {
        case APP_STATE_INIT:
            SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0, APP_ReadComplete, SYS_CONSOLE_EVENT_READ_COMPLETE);
            SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0, APP_WriteComplete, SYS_CONSOLE_EVENT_WRITE_COMPLETE);
            appData.state = APP_STATE_WRITE_TEST_1;
            break;
        case APP_STATE_WRITE_TEST_1:
            /* Demonstrates polling write method */
            APP_TestConsoleWrite();
            appData.state = APP_STATE_WRITE_TEST_1_WFC;
            break;
        case APP_STATE_WRITE_TEST_1_WFC:          
            appData.state = APP_STATE_WRITE_TEST_2;
            SYS_MESSAGE("Polling Write Test completed.\n\r");
            break;
        case APP_STATE_WRITE_TEST_2:
            /* Demonstrates callback write method */
            APP_TestDebugAPI();
            appData.state = APP_STATE_WRITE_TEST_2_WFC;
            break;
        case APP_STATE_WRITE_TEST_2_WFC:
            if (appData.wrComplete)
            {
                appData.state = APP_STATE_WRITE_TEST_3;
                SYS_MESSAGE("\n\rCallback Write Test completed.\n\r");
                appData.wrComplete = false;
                /* introducing a delay so that queued up messages can be transmitted */
                for (appData.qEmptyCnt = 0; appData.qEmptyCnt < 4000000; appData.qEmptyCnt++);
            }
            break;
        case APP_STATE_WRITE_TEST_3:
         
            /* Demonstrates write queue overflow error and flush recovery */
            
            for (i = 0; i < 2 * APP_WRITE_BUFFER_SIZE; i++)
            {
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, "$", 1);
                testVar++;
            }
            break;
        case APP_STATE_READ_TEST_1:
            /* Demonstrates polling read method */
            SYS_MESSAGE("\n\rEnter 10 characters: ");
            appData.bytesRead = SYS_CONSOLE_Read(SYS_CONSOLE_INDEX_0, STDIN_FILENO, rdBuf, 10);
            appData.state = APP_STATE_READ_TEST_1_WFC;
            break;
        case APP_STATE_READ_TEST_1_WFC:
            SYS_PRINT("\n\rPolling Read %d bytes: ", appData.bytesRead);
            SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, rdBuf, 10);
            SYS_MESSAGE("\n\r");
            appData.wrComplete = false;
            appData.state = APP_STATE_READ_TEST_2;
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

            //Flush console here to prevent over read characters from previous app state
            SYS_CONSOLE_Flush(SYS_CONSOLE_INDEX_0);

            SYS_MESSAGE("\n\rEcho Test. Type 'q' to quit.\n\r");
            appData.state = APP_STATE_ECHO_TEST_RD;
            break;
        case APP_STATE_ECHO_TEST_RD:
//            if (appData.wrComplete)
//            {
                SYS_CONSOLE_Read(SYS_CONSOLE_INDEX_0, STDIN_FILENO, rdBuf, 1);
                appData.state = APP_STATE_ECHO_TEST_WR;
                appData.rdComplete = false;
//            }
            break;
        case APP_STATE_ECHO_TEST_WR:
            if (appData.rdComplete)
            {
                if (appData.bytesRead > 0)
                {
                    SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, rdBuf, appData.bytesRead);
                    appData.wrComplete = false;
                }

                if (rdBuf[0] == 'q')
                {
                    appData.state = APP_STATE_DONE;
                    SYS_MESSAGE("\n\n\rAll Tests Completed\n\r");
                }
                else
                {
                    appData.state = APP_STATE_ECHO_TEST_RD;
                }
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

