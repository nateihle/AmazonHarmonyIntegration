/*******************************************************************************
  MPLAB Harmony Application 
  NVM MPFS Single Disk Demo Application
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
   NVM MPFS Single Disk Demo

  Description:
   This file contains the NVM MPFS Single Disk Demo application logic.
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

/* This demonstration shows an example of implementing a MPFS disk in device
 * Flash memory. The demonstration contains a MPFS disk image in the internal
 * Flash memory. The disk image contains two files named:
 * FILE.txt, Size = 11 bytes. The content of the file is: "Hello World". 
 * TEST.txt, Size = 72 bytes. The content of the file is: "This file contains a
 * test string and it is meant for testing. 1234567890".
 * 
 * The demonstration application logic is implemented as a state machine in the
 * APP_Tasks function in the file app.c.
 *
 * The application does the following:
 * 1. Mount the file system image present on the internal flash. The volume is
 *    mounted against a MPFS2 type file system and mounted at /mnt/myDrive/.
 * 2. After the mount is successful open a file named "FILE.txt" in read mode.
 * 3. Open a second file named "TEST.txt" also in read mode.
 * 4. Find the size of the file "FILE.txt" and check that size matches the
 *    known value of 11 bytes.
 * 5. Move the file pointer of the file "TEST.txt" 10 bytes from the end of the
 *    file.
 * 6. Read 10 bytes of data from the file and compare it against the known
 *    string 1234567890 using the strncmp function.
 * 7. If the string comparison is successful, then check for the EOF of the
 *    file.
 * 8. If there is no error in any of the above steps then the application will
 *    go into Idle state.
 * 9. If there is an error then the application will go into Error state.
 * */

// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include <string.h>
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************
/* This string is already present in a file in the disk image */
const uint8_t compareString[] = "1234567890";

/*******************************************************************************
 * Initialize the application data structure. All application related variables
 * are stored in this data structure.
 *****************************************************************************/

APP_DATA appData;
// *****************************************************************************
/* Driver objects.

  Summary:
    Contains driver objects.

  Description:
    This structure contains driver objects returned by the driver init routines
    to the application. These objects are passed to the driver tasks routines.
*/



// *****************************************************************************
// *****************************************************************************
// Section: Application Local Routines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************



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

void APP_Initialize ( void )
{
    /* Initialize the app state to wait for media attach. */
    appData.state = APP_MOUNT_DISK;
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
    /* The application task state machine */

    switch(appData.state)
    {
        case APP_MOUNT_DISK:
            {
                if(SYS_FS_Mount("/dev/nvma1", "/mnt/myDrive", MPFS2, 0, NULL) != 0)
                {
                    /* The disk could not be mounted. Keep trying until the
                     * mount operation is successful. */
                    appData.state = APP_MOUNT_DISK;
                }
                else
                {
                    /* Mount was successful. Open file. */
                    appData.state = APP_OPEN_FILE_1;
                }
                break;
            }

        case APP_OPEN_FILE_1:
            {
                appData.fileHandle_1 = SYS_FS_FileOpen("/mnt/myDrive/FILE.txt", SYS_FS_FILE_OPEN_READ);
                if(appData.fileHandle_1 == SYS_FS_HANDLE_INVALID)
                {
                    /* Could not open the file. Error out. */
                    appData.state = APP_ERROR;
                }
                else
                {
                    /* First file open was successful. Now open the second
                     * file. */
                    appData.state = APP_OPEN_FILE_2;
                }
                break;
            }

        case APP_OPEN_FILE_2:
            {
                appData.fileHandle_2 = SYS_FS_FileOpen("/mnt/myDrive/TEST.txt", SYS_FS_FILE_OPEN_READ);
                if(appData.fileHandle_2 == SYS_FS_HANDLE_INVALID)
                {
                    /* Could not open the file. Error out. */
                    appData.state = APP_ERROR;
                }
                else
                {
                    /* Second file open was successful. */
                    appData.state = APP_DO_FILE_SIZE_CHECK;
                }
                break;
            }

        case APP_DO_FILE_SIZE_CHECK:
            {
                if(SYS_FS_FileSize(appData.fileHandle_1) != 11)
                {
                    /* Incorrect file size. */
                    appData.state = APP_ERROR;
                }
                else
                {
                    appData.state = APP_DO_FILE_SEEK;
                }

                /* We are done with this file, hence close it */
                SYS_FS_FileClose(appData.fileHandle_1);
                break;
            }

        case APP_DO_FILE_SEEK:
            {
                if(SYS_FS_FileSeek(appData.fileHandle_2, -10, SYS_FS_SEEK_END) != -10)
                {
                    /* File seek went wrong somewhere  */
                    appData.state = APP_ERROR;
                }
                else
                {
                    /* Compare the remaining file content with a known string */
                    appData.state = APP_READ_VERIFY_CONTENT;
                }
                break;
            }

        case APP_READ_VERIFY_CONTENT:
            {
                if(SYS_FS_FileRead(appData.fileHandle_2, (void *)appData.data, 10) == -1)
                {
                    /* There was an error while reading the file. Close the
                     * file and error out. */
                    SYS_FS_FileClose(appData.fileHandle_2);
                    appData.state = APP_ERROR;
                }
                else
                {
                    if(strncmp((const char *)appData.data, (const char *)compareString, 10) != 0)
                    {
                        /* The written and the read data don't match. */
                        appData.state = APP_ERROR;
                    }
                    else
                    {
                        /* The test was successful. Lets idle. */
                        appData.state = APP_CHECK_EOF;
                    }
                }
                break;
            }

        case APP_CHECK_EOF:
            {
                /* By now, we should have reached end of file */
                if(SYS_FS_FileEOF(appData.fileHandle_2) != true)
                {
                    /* Error */
                    appData.state = APP_ERROR;
                }
                else
                {
                    /* We have completed all tests. Hence go to idle state */
                    appData.state = APP_IDLE;
                }
                break;
            }

        case APP_IDLE:
            {
                /* The application comes here when the demo has completed
                 * successfully. */ 
                SYS_FS_FileClose(appData.fileHandle_2);
                BSP_LEDOn(APP_SUCCESS_LED);
                break;
            }

        case APP_ERROR:
            {
                /* The application comes here when the demo has failed. */
                BSP_LEDOn(APP_FAILURE_LED);
                break;
            }
        default:
            {
                break;
            }
    }

} //End of APP_Tasks


/*******************************************************************************
 End of File
 */

