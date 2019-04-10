/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app_appio_795_pim_e16.c

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

#include "app_appio_795_pim_e16.h"

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

APP_APPIO_795_PIM_E16_DATA app_appio_795_pim_e16Data;

static const char appio_msg0[] = "Demo APPIO Console";
static const char appio_msg1[] = "\nDemo 1st Read over APPIO - Enter 10 characters -> Press RETURN when done";
static const char appio_msg2[] = "\nAPPIO Input String is :";
static const char appio_msg3[] = "\nDemo 2nd Read over APPIO - Enter 10 characters -> Press RETURN when done";
static const char appio_msg4[] = "\n--------------- APPIO Demo Complete ------------------";

static char rdBufappio[30];

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void APP_APPIO_795_PIM_E16_ReadCompleteAPPIO (void *handle);
void APP_APPIO_795_PIM_E16_WriteCompleteAPPIO (void *handle);

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

void APP_APPIO_795_PIM_E16_ClearAPPIOFlags();



// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_APPIO_795_PIM_E16_Initialize ( void )

  Remarks:
    See prototype in app_appio_795_pim_e16.h.
 */

void APP_APPIO_795_PIM_E16_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_appio_795_pim_e16Data.state = APP_APPIO_795_PIM_E16_STATE_INIT;
    
    APP_APPIO_795_PIM_E16_ClearAPPIOFlags();

}


/******************************************************************************
  Function:
    void APP_APPIO_795_PIM_E16_Tasks ( void )

  Remarks:
    See prototype in app_appio_795_pim_e16.h.
 */

void APP_APPIO_795_PIM_E16_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( app_appio_795_pim_e16Data.state )
    {
        /* Application's initial state. */
        case APP_APPIO_795_PIM_E16_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                app_appio_795_pim_e16Data.state = APP_APPIO_795_PIM_E16_SET_CALLBACKS;
            }
            break;
        }

        case APP_APPIO_795_PIM_E16_SET_CALLBACKS:             
        {
            
            if (SYS_CONSOLE_Status(sysObj.sysConsole0) == SYS_STATUS_READY)
            {
                SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0, APP_APPIO_795_PIM_E16_ReadCompleteAPPIO, SYS_CONSOLE_EVENT_READ_COMPLETE);
                SYS_CONSOLE_RegisterCallback(SYS_CONSOLE_INDEX_0, APP_APPIO_795_PIM_E16_WriteCompleteAPPIO, SYS_CONSOLE_EVENT_WRITE_COMPLETE);
                
                app_appio_795_pim_e16Data.state = APP_APPIO_795_PIM_E16_STATE_SERVICE_TASKS;
            }
            
            break;
        }
        case APP_APPIO_795_PIM_E16_STATE_SERVICE_TASKS:
        {
        
            SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, appio_msg0, (sizeof(appio_msg0)-1) );
            
            app_appio_795_pim_e16Data.state = APP_APPIO_795_PIM_E16_READ_STRING_1;
            break;
        }
        
        case APP_APPIO_795_PIM_E16_READ_STRING_1:
        {
            SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, appio_msg1, (sizeof(appio_msg1)-1) );
             
            app_appio_795_pim_e16Data.bytesReadAPPIO = SYS_CONSOLE_Read(SYS_CONSOLE_INDEX_0, STDIN_FILENO, rdBufappio, 1);
             
            app_appio_795_pim_e16Data.state = APP_APPIO_795_PIM_E16_DISPLAY_STRING_1;
            
            break;
        }

        case APP_APPIO_795_PIM_E16_DISPLAY_STRING_1:
        {
            if (app_appio_795_pim_e16Data.rdCompleteAPPIO)
            {
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, appio_msg2, (sizeof(appio_msg2)-1) );

                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, rdBufappio, app_appio_795_pim_e16Data.bytesReadAPPIO);
                
                APP_APPIO_795_PIM_E16_ClearAPPIOFlags();

                app_appio_795_pim_e16Data.state = APP_APPIO_795_PIM_E16_READ_STRING_2;
            }
            break;
        }
        
        case APP_APPIO_795_PIM_E16_READ_STRING_2:
        {
            SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, appio_msg3, (sizeof(appio_msg3)-1) );
             
            app_appio_795_pim_e16Data.bytesReadAPPIO = SYS_CONSOLE_Read(SYS_CONSOLE_INDEX_0, STDIN_FILENO, rdBufappio, 1);
             
            app_appio_795_pim_e16Data.state = APP_APPIO_795_PIM_E16_DISPLAY_STRING_2;
            
            break;
        }

        case APP_APPIO_795_PIM_E16_DISPLAY_STRING_2:
        {
            if (app_appio_795_pim_e16Data.rdCompleteAPPIO)
            {
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, appio_msg2, (sizeof(appio_msg2)-1) );

                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, rdBufappio, app_appio_795_pim_e16Data.bytesReadAPPIO);
                
                SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, appio_msg4, (sizeof(appio_msg4)-1));
                
                APP_APPIO_795_PIM_E16_ClearAPPIOFlags();

                app_appio_795_pim_e16Data.state = APP_APPIO_795_PIM_E16_RESULT_COMPLETE;
            }
            break;
        }
        
        case APP_APPIO_795_PIM_E16_RESULT_COMPLETE:
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
    void APP_APPIO_795_PIM_E16_WriteCompleteAPPIO (void *handle)

  Remarks:
    Callback function after read of specified number of bytes is completed
 */

void APP_APPIO_795_PIM_E16_WriteCompleteAPPIO (void *handle)
{
    app_appio_795_pim_e16Data.wrCompleteAPPIO = true;
}

/*******************************************************************************
  Function:
    void APP_APPIO_795_PIM_E16_ReadCompleteAPPIO (void *handle)

  Remarks:
    Callback function after write of specified number of bytes is completed
 */

void APP_APPIO_795_PIM_E16_ReadCompleteAPPIO (void *handle)
{
    size_t *readSize = handle;
    app_appio_795_pim_e16Data.bytesReadAPPIO = *readSize;
    app_appio_795_pim_e16Data.rdCompleteAPPIO = true;
}

/*******************************************************************************
  Function:
    void APP_APPIO_795_PIM_E16_ClearAPPIOFlags(void)

  Remarks:
    Function to clear the flags that were set when callback were obtained
 */

void APP_APPIO_795_PIM_E16_ClearAPPIOFlags(void)
{
    app_appio_795_pim_e16Data.rdCompleteAPPIO = app_appio_795_pim_e16Data.wrCompleteAPPIO = false;
}
 

/*******************************************************************************
 End of File
 */
