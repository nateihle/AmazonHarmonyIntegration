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

/* MSD Application Data Structure */
APP_DATA_MSD    appDataMsd;
/* SD Card Application Data Structure */
APP_DATA_SD     appDataSd;

/* This is the string that will written to the file */
const uint8_t writeData[12]  __attribute__((aligned(16))) = "Hello World ";
uint8_t data[512] __attribute__((coherent, aligned(16)));
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/
USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler (USB_HOST_EVENT event, void * eventData, uintptr_t context)
{
    switch (event)
    {
        case USB_HOST_EVENT_DEVICE_UNSUPPORTED:
            break;
        default:
            break;
                    
    }
    
    return(USB_HOST_EVENT_RESPONSE_NONE);
}

void APP_SYSFSEventHandler(SYS_FS_EVENT event, void * eventData, uintptr_t context)
{
    switch(event)
    {
        /* If the event is mount then check which media has been mounted */
        case SYS_FS_EVENT_MOUNT:
            if(0 == strcmp((const char *)eventData,"/mnt/sdDrive"))
            {
                appDataSd.deviceIsConnected = true;
            }
            else if(0 == strcmp((const char *)eventData,"/mnt/msdDrive/"))
            {
                appDataMsd.deviceIsConnected = true;
            }
            break;
        /* If the event is unmount then check which media has been unmount */
        case SYS_FS_EVENT_UNMOUNT:
            if(0 == strcmp((const char *)eventData,"/mnt/sdDrive"))
            {
                appDataSd.deviceIsConnected = false;
            }
            else if(0 == strcmp((const char *)eventData,"/mnt/msdDrive/"))
            {
                appDataMsd.deviceIsConnected = false;
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
    appDataMsd.dirHandle = SYS_FS_HANDLE_INVALID;
    appDataMsd.fileHandle = SYS_FS_HANDLE_INVALID;
    appDataSd.fileHandle = SYS_FS_HANDLE_INVALID;
    /* Place the App state machine in its initial state. */
    appDataMsd.state = APP_STATE_BUS_ENABLE;
    appDataSd.state = APP_OPEN_FIRST_FILE;
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

void APP_Tasks( void )
{
    APP_USB_MSDTasks();
    APP_SDCardTasks();    
}

/******************************************************************************
  Function:
    void APP_USB_MSDTasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_USB_MSDTasks (void)
{
    SYS_FS_RESULT result = SYS_FS_RES_FAILURE;
    /* The application task state machine */

    switch(appDataMsd.state)
    {
        case APP_STATE_BUS_ENABLE:
            /* Set the event handler and enable the bus */
            SYS_FS_EventHandlerSet(APP_SYSFSEventHandler, (uintptr_t)NULL);
            USB_HOST_EventHandlerSet(APP_USBHostEventHandler, 0);
            USB_HOST_BusEnable(0);                      
            appDataMsd.state = APP_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE;
            break;

        case APP_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE:
            if(USB_HOST_BusIsEnabled(0))
            {
                appDataMsd.state = APP_STATE_WAIT_FOR_DEVICE_ATTACH;
            }
            break;

        case APP_STATE_WAIT_FOR_DEVICE_ATTACH:

            /* Wait for device attach. The state machine will move
             * to the next state when the attach event
             * is received.  */
            if(appDataMsd.deviceIsConnected)
            {
                BSP_LEDOn( BSP_LED_2 );
                BSP_LEDOff( BSP_LED_3 );
                appDataMsd.state = APP_STATE_DEVICE_CONNECTED;
                appDataSd.state = APP_OPEN_FIRST_FILE;
            }
            break;

        case APP_STATE_DEVICE_CONNECTED:

            /* Device was connected. We can try mounting the disk */
            appDataMsd.state = APP_OPEN_DIRECTORY;
            break;
			case APP_OPEN_DIRECTORY:
			 /* Open the root directory of MSD */
            appDataMsd.dirHandle = SYS_FS_DirOpen("/mnt/msdDrive/");
            if(SYS_FS_HANDLE_INVALID == appDataMsd.dirHandle)
            {
                /* Could not open the directory. Error out*/
                appDataMsd.state = APP_STATE_ERROR;
                break;
            }
            /* Search Directory for the file */
            appDataMsd.state = APP_SEARCH_FILE;
            break;
        case APP_SEARCH_FILE:
           
            /* Search for the file "mchpLogo.bmp" with wild characters */
            /* Since, we are using LFN, initialize the structure accordingly */
            appDataMsd.dirStatus.lfname = (char *) appDataMsd.lfndata;
            appDataMsd.dirStatus.lfsize = 64;
            /* LAB 4 - Step 1*/
            result = SYS_FS_DirSearch(appDataMsd.dirHandle, "mch*.*",
                                        SYS_FS_ATTR_ARC, &appDataMsd.dirStatus);
            if(result == SYS_FS_RES_FAILURE)
            {
                /* Could not search the directory. Error out*/
                appDataMsd.state = APP_STATE_ERROR;
                break;
            }
            /* File opened successfully. Write to file */
            appDataMsd.state = APP_STATE_IDLE;
            break;
        case APP_STATE_IDLE:
            /* Wait for device attach. The state machine will move
             * to the next state when the attach event
             * is received.  */
            if(!appDataMsd.deviceIsConnected)
            {
                BSP_LEDOff( BSP_LED_2 );
                appDataMsd.state = APP_STATE_WAIT_FOR_DEVICE_ATTACH;
            }

            
            break;
        case APP_STATE_UNMOUNT_DISK:

            /* The drive was detached. Switch off LED. Unmount the disk */
            BSP_LEDOff(BSP_LED_2);

            if(SYS_FS_Unmount("/mnt/myDrive") != 0)
            {
                /* The disk could not be un mounted. Try
                 * un mounting again untill success. */

                appDataMsd.state = APP_STATE_UNMOUNT_DISK;
            }
            else
            {
                /* UnMount was successful. Wait for device attach */
                appDataMsd.state =  APP_STATE_WAIT_FOR_DEVICE_ATTACH;

            }
            break;
        case APP_STATE_ERROR:

            /* The application comes here when the demo
             * has failed. Provide LED indication .*/

            BSP_LEDOn(BSP_LED_1);
            break;

        default:
            break;

    }

} //End of APP_Tasks
/******************************************************************************
  Function:
    void APP_SDCardTasks ( void )

  Remarks:
    See prototype in app.h.
*/

void APP_SDCardTasks ( void )
{
    SYS_FS_RESULT result = SYS_FS_RES_FAILURE;
    /* The application task state machine */
    switch(appDataSd.state)
    {
        case APP_OPEN_FIRST_FILE:
            /* Wait until MSD is mounted and file search is sucessful */
            if((appDataMsd.state == APP_STATE_IDLE) && appDataSd.deviceIsConnected)
            {
                SYS_FS_CurrentDriveSet("/mnt/msdDrive");
                /* open the file found on MSD in Read mode */
                appDataMsd.fileHandle = SYS_FS_FileOpen(appDataMsd.dirStatus.fname,
                        (SYS_FS_FILE_OPEN_READ));
                BSP_LEDOff(BSP_LED_3);
                if(appDataMsd.fileHandle != SYS_FS_HANDLE_INVALID)
                {
                    /* Could not open the file. Error out*/
                    appDataSd.state = APP_OPEN_SECOND_FILE;
                    break;
                }
                /* Open a File on SD Card */
                appDataSd.state = APP_ERROR;
            }
            break;
        case APP_OPEN_SECOND_FILE:
            /* LAB 4 - Step 2*/
            /* Set the current directory to sdDrive*/
            result = SYS_FS_CurrentDriveSet("/mnt/sdDrive");
            if(result == SYS_FS_RES_FAILURE)
            {
                /* Error while setting current drive */
                appDataSd.state = APP_ERROR;
                break;
            }
            /* Open a second file on SD Card in Write mode */
            appDataSd.fileHandle = SYS_FS_FileOpen(appDataMsd.dirStatus.fname,
                    (SYS_FS_FILE_OPEN_WRITE));

            if(appDataSd.fileHandle != SYS_FS_HANDLE_INVALID)
            {
                /* Could not open the file. Error out*/
                appDataSd.state = APP_READ_WRITE_TO_FILE;
                break;
            }
             /* Read from one file and write to another file */
                appDataSd.state = APP_ERROR;
                break;

        case APP_READ_WRITE_TO_FILE:
                     
            if(-1 == SYS_FS_FileRead(appDataMsd.fileHandle, (void *)data, 512))
            {
                /* There was an error while reading the file.
                * Close the file and error out. */

                SYS_FS_FileClose(appDataSd.fileHandle);
                appDataSd.state = APP_ERROR;
                break;
            }
             /* If read was success, try writing to the new file */
             if((SYS_FS_FileWrite(appDataSd.fileHandle, (const void *)data, 512) == -1))
             {
                    /* Write was not successful. Close the file
                    * and error out.*/
                    SYS_FS_FileClose(appDataSd.fileHandle);
                    appDataSd.state = APP_ERROR;
                    break;
             }
             if(true == SYS_FS_FileEOF(appDataMsd.fileHandle))    /* Test for end of file */
             {
                    /* Continue the read and write process, untill the end of file is reached */
                    appDataSd.state = APP_CLOSE_FILE;
             }
             break;

        case APP_CLOSE_FILE:
            /* Close both files */
            SYS_FS_FileClose(appDataMsd.fileHandle);
            SYS_FS_FileClose(appDataSd.fileHandle);
             /* The test was successful. Lets idle. */
            appDataSd.state = APP_IDLE;
            break;

        case APP_IDLE:
            /* The appliction comes here when the demo
             * has completed successfully. Switch on
             * green LED. */
            BSP_LEDOn(BSP_LED_3);
            break;
        case APP_ERROR:
            /* The application comes here when the demo
             * has failed. Switch on the red LED.*/
            BSP_LEDOn(BSP_LED_1);
            break;
        default:
            break;

    }
}
 

/*******************************************************************************
 End of File
 */
