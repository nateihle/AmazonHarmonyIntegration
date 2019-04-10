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

#include <string.h>
#include "app.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
/* This is the string that will written to the file */
const uint8_t writeData[13] = "Hello World";
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


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_SysFSEventHandler ( SYS_FS_EVENT event,void* eventData,uintptr_t context )

  Remarks:
    See prototype in app.h.
 */
void APP_SysFSEventHandler(SYS_FS_EVENT event,void* eventData,uintptr_t context)
{
    switch(event)
    {
        /* If the event is mount then check which media has been mounted */
        case SYS_FS_EVENT_MOUNT:
            if(0 == strcmp((const char *)eventData,"/mnt/myDrive1"))
            {
                appData.sdCardMountFlag = true;
            }
            else if(0 == strcmp((const char *)eventData,"/mnt/myDrive2"))
            {
                appData.nvmMountFlag = true;
            }
            break;
        /* If the event is unmount then check which media has been unmount */
        case SYS_FS_EVENT_UNMOUNT:
            if(0 == strcmp((const char *)eventData,"/mnt/myDrive1"))
            {
                appData.sdCardMountFlag = false;
            }
            else if(0 == strcmp((const char *)eventData,"/mnt/myDrive2"))
            {
                appData.nvmMountFlag = false;
            }

            if (appData.state != APP_IDLE)
            {
                appData.state = APP_ERROR;
            }
            break;

        case SYS_FS_EVENT_ERROR:
            break;      
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
    /* Intialize the app state to wait for
     * media attach. */
    appData.state = APP_MOUNT_WAIT;
    appData.nvmMountFlag = false;
    appData.sdCardMountFlag = false;
    /* register the event handler with media manager */
    SYS_FS_EventHandlerSet(APP_SysFSEventHandler,(uintptr_t)NULL);
    
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
    /* The application task state machine */

    switch(appData.state)
    {
        case APP_MOUNT_WAIT:
            /* wait for the volume to get auto mounted */
            if(appData.sdCardMountFlag && appData.nvmMountFlag)
            {
                appData.state = APP_OPEN_FILE;
            }
            break;             
        case APP_OPEN_FILE:

            appData.fileHandle1 = SYS_FS_FileOpen("/mnt/myDrive2/abc.txt", (SYS_FS_FILE_OPEN_READ));
            if(appData.fileHandle1 == SYS_FS_HANDLE_INVALID)
            {
                /* Could not open the file. Error out*/
                appData.state = APP_ERROR;
                break;
            }

            appData.fileHandle2 = SYS_FS_FileOpen("/mnt/myDrive1/FILE.TXT", (SYS_FS_FILE_OPEN_WRITE));
            if(appData.fileHandle2 == SYS_FS_HANDLE_INVALID)
            {
                /* Could not open the file. Error out*/
                appData.state = APP_ERROR;
                break;
            }

            /* Try reading from NVM file.*/
            appData.state = APP_READ_FILE_FROM_NVM;

            break;

        case APP_READ_FILE_FROM_NVM:
            if(SYS_FS_FileRead(appData.fileHandle1, (void *)appData.data, 13) == -1)
            {
                /* Read was not successful. Close the file
                 * and error out.*/
                SYS_FS_FileClose(appData.fileHandle1);
                appData.state = APP_ERROR;
            }
            else
            {
                /* Read was successful. Close the file and
                 * open SDCARD file for write. */
                SYS_FS_FileClose(appData.fileHandle1);
                appData.state = APP_WRITE_TO_FILE_ON_SDCARD;
            }
            break;

        case APP_WRITE_TO_FILE_ON_SDCARD:

            if(SYS_FS_FileWrite(appData.fileHandle2, (const void *)appData.data, 13) == -1)
            {
                /* There was an error while reading the file.
                 * Close the file and error out. */

                SYS_FS_FileClose(appData.fileHandle2);
                appData.state = APP_ERROR;
                break;
            }
            else
            {
                /* The test was successful. Lets idle. */
                appData.state = APP_IDLE;
                SYS_FS_FileClose(appData.fileHandle2);

                break;
            }
        case APP_IDLE:
            /* The application comes here when the demo
             * has completed successfully. Switch on
             * green LED. */
            BSP_LEDOn(APP_SUCCESS_LED);
            break;
        case APP_ERROR:
            /* The application comes here when the demo
             * has failed. Switch on the red LED.*/
            BSP_LEDOn(APP_FAILURE_LED);
            break;
        default:
            break;



    }

} //End of APP_Tasks
 

/*******************************************************************************
 End of File
 */

