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
    /* Intialize the app state to wait for
     * media attach. */
    appData.state = APP_MOUNT_DISK_MEDIA_NVM;
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
         case APP_MOUNT_DISK_MEDIA_NVM:
            if(SYS_FS_Mount("/dev/nvma1", "/mnt/myDrive1", FAT, 0, NULL) != 0)
            {
                /* The disk could not be mounted. Try
                 * untill success. */

                appData.state = APP_MOUNT_DISK_MEDIA_NVM;
                break;
            }
            else
            {
                appData.state = APP_MOUNT_DISK_MEDIA_SD;
                break;
            }
         case APP_MOUNT_DISK_MEDIA_SD:
            if(SYS_FS_Mount("/dev/mmcblka1", "/mnt/myDrive2", FAT, 0, NULL) != 0)
            {
                /* The disk could not be mounted. Try
                 * untill success. */

                appData.state = APP_MOUNT_DISK_MEDIA_SD;
                break;
            }
            else
            {
                /* Mount was successful. Search for file form NVM */

                appData.state = APP_OPEN_DIRECTORY;
                break;
            }
        case APP_OPEN_DIRECTORY:
            /* Open the root directory of NVM media and search for the file "FILE.TXT*/
            appData.dirHandle = SYS_FS_DirOpen("/mnt/myDrive1/");
            if(appData.dirHandle == SYS_FS_HANDLE_INVALID)
            {
                /* Could not open the directory. Error out*/
                appData.state = APP_ERROR;
            }
            else
            {
                /* Search Directory for the file */
                appData.state = APP_SEARCH_DIRECTORY;
            }
            break;
        case APP_SEARCH_DIRECTORY:
            /* Search for the file "FILE.TXT" with wild characters */
            /* Since, we are using LFN, initialize the structure accordingly */
            appData.dirStatus.lfname = (char *) appData.data;
            appData.dirStatus.lfsize = 64;

            if(SYS_FS_DirSearch(appData.dirHandle, "FIL*.*", SYS_FS_ATTR_ARC, &appData.dirStatus) == SYS_FS_RES_FAILURE)
            {
                /* Could not search the directory. Error out*/
                appData.state = APP_ERROR;
            }
            else
            {
                if(SYS_FS_DirClose(appData.dirHandle) == SYS_FS_RES_SUCCESS)
                {
                    /* Though, LFN is enabled, the file name "FILE.TXT" that we are searching, it fits in 8.3 format.
                     * Hence, the file name would be stored in "appData.dirStatus.fname" and not inside
                     * "appData.dirStatus.lfname" (or appData.data). In a real world case, the decision can be made
                     * by checking the contents of both the buffer. If any one of the buffer is NULL, the data will
                     * be in the other one. That check is not done here, since, this is a demo and we know the file
                     * name, that we are searching for.  */
                    /* Verify the searched file. Since there is only 1 file in the NVM, it should be the one,
                       we are looking for */
                    if((appData.dirStatus.fname[0] == 'F') && (appData.dirStatus.fname[1] == 'I') && (appData.dirStatus.fname[2] == 'L') &&
                            (appData.dirStatus.fname[3] == 'E') && (appData.dirStatus.fname[4] == '.') && (appData.dirStatus.fname[5] == 'T') &&
                            (appData.dirStatus.fname[6] == 'X') && (appData.dirStatus.fname[7] == 'T'))
                    {
                        /* Open the file */
                        appData.state = APP_OPEN_FILE;
                    }
                    else
                    {
                        /* File name does not match. Error out*/
                        appData.state = APP_ERROR;
                    }
                }
                else
                {
                    /* Directory close did not work. Error out*/
                    appData.state = APP_ERROR;
                }
            }
            break;
        case APP_OPEN_FILE:

            appData.fileHandle1 = SYS_FS_FileOpen("/mnt/myDrive1/FILE.TXT", (SYS_FS_FILE_OPEN_READ));
            if(appData.fileHandle1 == SYS_FS_HANDLE_INVALID)
            {
                /* Could not open the file. Error out*/
                appData.state = APP_ERROR;
                break;
            }

            appData.fileHandle2 = SYS_FS_FileOpen("/mnt/myDrive2/FILE.TXT", (SYS_FS_FILE_OPEN_WRITE));
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
            if(SYS_FS_FileRead(appData.fileHandle1, (void *)appData.data, 27) == -1)
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

            if(SYS_FS_FileWrite(appData.fileHandle2, (const void *)appData.data, 27) == -1)
            {
                /* There was an error while writing the file.
                 * Close the file and error out. */

                SYS_FS_FileClose(appData.fileHandle2);
                appData.state = APP_ERROR;
            }
            else
            {
                /* The test was successful. Write a character to file. */
                appData.state = APP_WRITE_CHAR_TO_FILE_ON_SDCARD;

                
            }
            break;
        case APP_WRITE_CHAR_TO_FILE_ON_SDCARD:
            if(SYS_FS_FileCharacterPut(appData.fileHandle2, '\n') == SYS_FS_RES_FAILURE)
            {
                /* There was an error while writing the file.
                 * Close the file and error out. */
                SYS_FS_FileClose(appData.fileHandle2);
                appData.state = APP_ERROR;
            }
            else
            {
                /* The test was successful. Write a string to file. */
                appData.state = APP_WRITE_STRING_TO_FILE_ON_SDCARD;
            }
            break;

        case APP_WRITE_STRING_TO_FILE_ON_SDCARD:
            if(SYS_FS_FileStringPut(appData.fileHandle2, "Test is successful.") == SYS_FS_RES_FAILURE)
            {
                /* There was an error while writing the file.
                 * Close the file and error out. */
                SYS_FS_FileClose(appData.fileHandle2);
                appData.state = APP_ERROR;
            }
            else
            {
                /* The test was successful. Go to idle loop. */
                SYS_FS_FileClose(appData.fileHandle2);
                appData.state = APP_IDLE;
            }
            break;

        case APP_IDLE:
            /* The appliction comes here when the demo
             * has completed successfully. Switch on
             * green LED. */
            BSP_LEDOn(APP_SUCCESS_LED);
            break;
        case APP_ERROR:
            /* The appliction comes here when the demo
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

