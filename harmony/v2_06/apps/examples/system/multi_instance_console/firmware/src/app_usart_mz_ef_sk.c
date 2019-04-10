/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_usart_mz_ef_sk.c

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

#include "app_usart_mz_ef_sk.h"

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

APP_USART_MZ_EF_SK_DATA app_usart_mz_ef_skData;

static const char uart_msg0[] = "\n\rDemo Polling on UART Console - Enter 10 characters \n\r";
static const char uart_msg1[] = "\n\n\rUART Console Input String is : ";
static const char uart_msg2[] = "\n\n\rDemo Callback on UART Console - Enter 10 characters \n\r";
static const char uart_msg3[] = "\n\n\r**** UART Console Demo Complete *******";

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void APP_USART_MZ_EF_SK_WriteComplete (void *handle);
void APP_USART_MZ_EF_SK_ReadComplete (void *handle);

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


void APP_USART_MZ_EF_SK_ClearFlags(void);


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_USART_MZ_EF_SK_Initialize ( void )

  Remarks:
    See prototype in app_usart_mz_ef_sk.h.
 */

void APP_USART_MZ_EF_SK_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_usart_mz_ef_skData.state = APP_USART_MZ_EF_SK_STATE_INIT;

    APP_USART_MZ_EF_SK_ClearFlags();

}


/******************************************************************************
  Function:
    void APP_USART_MZ_EF_SK_Tasks ( void )

  Remarks:
    See prototype in app_usart_mz_ef_sk.h.
 */

void APP_USART_MZ_EF_SK_Tasks ( void )
{

    uint8_t loopcount;
    
    /* Check the application's current state. */
    switch ( app_usart_mz_ef_skData.state )
    {
        /* Application's initial state. */
        case APP_USART_MZ_EF_SK_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                app_usart_mz_ef_skData.state = APP_USART_MZ_EF_SK_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_USART_MZ_EF_SK_STATE_SERVICE_TASKS:
        {
                if (SYS_CONSOLE_Status(sysObj.sysConsole0) == SYS_STATUS_READY)
                {
                    SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0, APP_USART_MZ_EF_SK_ReadComplete, SYS_CONSOLE_EVENT_READ_COMPLETE);
                    SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0, APP_USART_MZ_EF_SK_WriteComplete, SYS_CONSOLE_EVENT_WRITE_COMPLETE);

                    app_usart_mz_ef_skData.state = APP_USART_MZ_EF_SK_DISPLAY_HEADER_STRING;
                }

                break;
        }
        
        case APP_USART_MZ_EF_SK_DISPLAY_HEADER_STRING:
        {
            for (loopcount = 0; loopcount < 25; loopcount++)
            {
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, "*", 1);
            }
            
            app_usart_mz_ef_skData.state = APP_USART_MZ_EF_SK_READ_STRING_1;
            
            break;
        }
        
        case APP_USART_MZ_EF_SK_READ_STRING_1:
        {
            /* demonstrates read by polling method */
            
            SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, uart_msg0, (sizeof(uart_msg0)-1)); 
            
            app_usart_mz_ef_skData.bytesReadUART = SYS_CONSOLE_Read(SYS_CONSOLE_INDEX_0, STDIN_FILENO, app_usart_mz_ef_skData.rdBufferUART, 10);
            
            app_usart_mz_ef_skData.state = APP_USART_MZ_EF_SK_DISPLAY_STRING_1;
            
            break;
        }
        
        case APP_USART_MZ_EF_SK_DISPLAY_STRING_1:
        {
            
            if (SYS_CONSOLE_Status(sysObj.sysConsole0) == SYS_STATUS_READY)
            {
                
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, uart_msg1, (sizeof(uart_msg1)-1));
                
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, app_usart_mz_ef_skData.rdBufferUART, app_usart_mz_ef_skData.bytesReadUART);
                
                APP_USART_MZ_EF_SK_ClearFlags();
                
                app_usart_mz_ef_skData.state = APP_USART_MZ_EF_SK_READ_STRING_2;               
            
            }
            break;
        }
        
        case APP_USART_MZ_EF_SK_READ_STRING_2:
        {
            /* demonstrates read over callback method */
            
            SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, uart_msg2, (sizeof(uart_msg2)-1)); 
            
            app_usart_mz_ef_skData.bytesReadUART = SYS_CONSOLE_Read(SYS_CONSOLE_INDEX_0, STDIN_FILENO, app_usart_mz_ef_skData.rdBufferUART, 10);
            
            app_usart_mz_ef_skData.state = APP_USART_MZ_EF_SK_DISPLAY_STRING_2;
            
            break;
        }
        
        case APP_USART_MZ_EF_SK_DISPLAY_STRING_2:
        {
            if (app_usart_mz_ef_skData.rdCompleteUART)
            {
                
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, uart_msg1, (sizeof(uart_msg1)-1));
                
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, app_usart_mz_ef_skData.rdBufferUART, app_usart_mz_ef_skData.bytesReadUART);
                
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, uart_msg3, (sizeof(uart_msg3)-1));
                
                APP_USART_MZ_EF_SK_ClearFlags();
                
                app_usart_mz_ef_skData.state = APP_USART_MZ_EF_SK_RESULT_COMPLETE;               
            
            }
            
            break;
        }
        
        case APP_USART_MZ_EF_SK_RESULT_COMPLETE:
        {           
            break;
        }
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

/*******************************************************************************
  Function:
    void APP_USART_MZ_EF_SK_WriteComplete (void *handle)

  Remarks:
    Callback function after read of specified number of bytes is completed
 */

void APP_USART_MZ_EF_SK_WriteComplete (void *handle)
{
    app_usart_mz_ef_skData.wrCompleteUART = true;
}

/*******************************************************************************
  Function:
    void APP_USART_MZ_EF_SK_ClearFlags (void *handle)

  Remarks:
    Callback function after write of specified number of bytes is completed
 */

void APP_USART_MZ_EF_SK_ReadComplete (void *handle)
{
    size_t *readSize = handle;
    app_usart_mz_ef_skData.bytesReadUART = *readSize;
    app_usart_mz_ef_skData.rdCompleteUART = true;
}

/*******************************************************************************
  Function:
    void APP_USART_MZ_EF_SK_ClearFlags(void)

  Remarks:
    Function to clear the flags that were set when callback were obtained
 */

void APP_USART_MZ_EF_SK_ClearFlags(void)
{
    app_usart_mz_ef_skData.rdCompleteUART = app_usart_mz_ef_skData.wrCompleteUART = false;
}


 

/*******************************************************************************
 End of File
 */
