/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_usb_usb_sk2.c

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

#include "app_usb_usb_sk2.h"

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

APP_USB_USB_SK2_DATA app_usb_usb_sk2Data;

static char rdBuf[BUFFER_USB_SIZE] __attribute__((coherent)) __attribute__((aligned(16)));

static const char usb_msg0[] = "\n\rDemo Polling on USB Console - Enter 10 characters \n\r";
static const char usb_msg1[] = "\n\n\rUSB Console Input String is : ";
static const char usb_msg2[] = "\n\n\rDemo Callback on USB Console - Enter 10 characters \n\r";
static const char usb_msg3[] = "\n\n\r**** USB Console Demo Complete *******";

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void APP_USB_USB_SK2_WriteCompleteUSB (void *handle);
void APP_USB_USB_SK2_ReadCompleteUSB (void *handle);

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


void APP_USB_USB_SK2_ClearUSBFlags(void);


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_USB_USB_SK2_Initialize ( void )

  Remarks:
    See prototype in app_usb_mx_b.h.
 */

void APP_USB_USB_SK2_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_usb_usb_sk2Data.state = APP_USB_USB_SK2_STATE_INIT;

    app_usb_usb_sk2Data.rdCompleteUSB  = false;
    app_usb_usb_sk2Data.wrCompleteUSB = false;
}


/******************************************************************************
  Function:
    void APP_USB_USB_SK2_Tasks ( void )

  Remarks:
    See prototype in app_usb_mx_b.h.
 */

void APP_USB_USB_SK2_Tasks ( void )
{
    uint8_t loopcount;
    
    /* Check the application's current state. */
    switch ( app_usb_usb_sk2Data.state )
    {
        /* Application's initial state. */
        case APP_USB_USB_SK2_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                app_usb_usb_sk2Data.state = APP_USB_USB_SK2_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_USB_USB_SK2_STATE_SERVICE_TASKS:
        {
            if (SYS_CONSOLE_Status(sysObj.sysConsole0) == SYS_STATUS_READY)
            {
                SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0, APP_USB_USB_SK2_ReadCompleteUSB, SYS_CONSOLE_EVENT_READ_COMPLETE);
                SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0, APP_USB_USB_SK2_WriteCompleteUSB, SYS_CONSOLE_EVENT_WRITE_COMPLETE);
                
                app_usb_usb_sk2Data.state = APP_USB_USB_SK2_DISPLAY_HEADER_STRING;
            }
            
            
            break;
        }
        
        case APP_USB_USB_SK2_DISPLAY_HEADER_STRING:
        {
         
            for (loopcount = 0; loopcount < 25; loopcount++)
            {
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, "*", 1);
            }

            app_usb_usb_sk2Data.state = APP_USB_USB_SK2_READ_STRING_1;
            
            break;
        }
        
        case APP_USB_USB_SK2_READ_STRING_1:
        {
            if (SYS_CONSOLE_Status(sysObj.sysConsole0) == SYS_STATUS_READY)
            {            
                /* demonstrates read by polling method */

                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, usb_msg0, (sizeof(usb_msg0)-1)); 

                app_usb_usb_sk2Data.bytesReadUSB = SYS_CONSOLE_Read(SYS_CONSOLE_INDEX_0, STDIN_FILENO, rdBuf, 10);

                app_usb_usb_sk2Data.state = APP_USB_USB_SK2_DISPLAY_STRING_1;
                
            }
            
            break;
        }
        
        case APP_USB_USB_SK2_DISPLAY_STRING_1:
        {
            /* checks the status of string read by polling method */
            
            if (SYS_CONSOLE_Status(sysObj.sysConsole0) == SYS_STATUS_READY)
            {            
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, usb_msg1, (sizeof(usb_msg1)-1));

                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, rdBuf, app_usb_usb_sk2Data.bytesReadUSB);

                APP_USB_USB_SK2_ClearUSBFlags();

                app_usb_usb_sk2Data.state = APP_USB_USB_SK2_READ_STRING_2;  
            }
            
            break;
        }
        
        case APP_USB_USB_SK2_READ_STRING_2:
        {
            /* demonstrates read over callback method */
            
            SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, usb_msg2, (sizeof(usb_msg2)-1)); 
            
            app_usb_usb_sk2Data.bytesReadUSB = SYS_CONSOLE_Read(SYS_CONSOLE_INDEX_0, STDIN_FILENO, rdBuf, 10);
            
            app_usb_usb_sk2Data.state = APP_USB_USB_SK2_DISPLAY_STRING_2;
            
            break;
        }
        
        case APP_USB_USB_SK2_DISPLAY_STRING_2:
        {
            /* checks status of character read by callback method */
            
            if (app_usb_usb_sk2Data.rdCompleteUSB)
            {
                
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, usb_msg1, (sizeof(usb_msg1)-1));
                
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, rdBuf, app_usb_usb_sk2Data.bytesReadUSB);
                
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, usb_msg3, (sizeof(usb_msg3)-1));
                
                APP_USB_USB_SK2_ClearUSBFlags();
                
                app_usb_usb_sk2Data.state = APP_USB_USB_SK2_RESULT_COMPLETE;               
            
            }
            
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
    void APP_USB_USB_SK2_ReadCompleteUSB (void *handle)

  Remarks:
    Callback function after read of specified number of bytes is completed
 */

void APP_USB_USB_SK2_ReadCompleteUSB (void *handle)
{
    size_t *readSize = handle;
    app_usb_usb_sk2Data.bytesReadUSB = *readSize;
    app_usb_usb_sk2Data.rdCompleteUSB = true;
}

/*******************************************************************************
  Function:
    void APP_USB_USB_SK2_WriteCompleteUSB (void *handle)

  Remarks:
    Callback function after write of specified number of bytes is completed
 */

void APP_USB_USB_SK2_WriteCompleteUSB (void *handle)
{
    app_usb_usb_sk2Data.wrCompleteUSB = true;
}

/*******************************************************************************
  Function:
    void APP_USB_USB_SK2_ClearUSBFlags(void)

  Remarks:
    Function to clear the flags that were set when callback were obtained
 */

void APP_USB_USB_SK2_ClearUSBFlags(void)
{
    app_usb_usb_sk2Data.rdCompleteUSB = app_usb_usb_sk2Data.wrCompleteUSB = false;
}

 

/*******************************************************************************
 End of File
 */
