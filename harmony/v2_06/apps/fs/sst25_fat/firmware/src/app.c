/*******************************************************************************
  MPLAB Harmony Application

  Company:
    Microchip Technology Inc.

  File Name:
    app.c

  Summary:
    Application Template

  Description:
    This file contains the application logic.
 *******************************************************************************/


// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 - 2017 released Microchip Technology Inc. All rights reserved.

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

#include <string.h>
#include "system_definitions.h"
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************
/* This is the string that will written to the file */
uint8_t __attribute__((coherent)) writeData[12] = "Hello World";
uint8_t __attribute__((coherent)) readData[16];

/*****************************************************
 * Initialize the application data structure. All application related variables
 * are stored in this data structure.
 *****************************************************/
APP_DATA appData;

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
    /* Intialize the app state to wait for media attach. */
    appData.state = APP_MOUNT_DISK;
    appData.data = &readData[0];
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
        case APP_MOUNT_DISK:
            {
                /* Mount the disk */
                if(SYS_FS_Mount("/dev/mtda1", "/mnt/myDrive", FAT, 0, NULL) != 0)
                {
                    /* The disk could not be mounted. Try mounting again until
                     * the operation succeeds. */
                    appData.state = APP_MOUNT_DISK;
                }
                else
                {
                    /* Mount was successful. Format the disk. */
                    appData.state = APP_FORMAT_DISK;
                }
                break;
            }

        case APP_FORMAT_DISK:
            {
                if (SYS_FS_DriveFormat ("/mnt/myDrive", SYS_FS_FORMAT_SFD, 0) != 0)
                {
                    /* Format of the disk failed. */
                    appData.state = APP_ERROR;
                }
                else
                {
                    /* Format succeeded. Open a file. */
                    appData.state = APP_OPEN_FILE;
                }
                break;
            }

        case APP_OPEN_FILE:
            {
                appData.fileHandle = SYS_FS_FileOpen("newFile.txt", (SYS_FS_FILE_OPEN_APPEND_PLUS));
                if(appData.fileHandle == SYS_FS_HANDLE_INVALID)
                {
                    /* File open unsuccessful */
                    appData.state = APP_ERROR;
                }
                else
                {
                    /* File open was successful. Write to the file. */
                    appData.state = APP_WRITE_TO_FILE;
                }
                break;
            }

        case APP_WRITE_TO_FILE:
            {
                if(SYS_FS_FileWrite (appData.fileHandle, (void *)writeData, 12) == -1)
                {
                    /* Failed to write to the file. */
                    appData.state = APP_ERROR;
                }
                else
                {
                    /* File write was successful. */
                    appData.state = APP_FLUSH_FILE;
                }
                break;
            }

        case APP_FLUSH_FILE:
            {
                if (SYS_FS_FileSync(appData.fileHandle) != 0)
                {
                    /* Could not flush the contents of the file. Error out. */
                    appData.state = APP_ERROR;
                }
                else
                {
                    /* Check the file status */
                    appData.state = APP_READ_FILE_STAT;
                }
                break;
            }

        case APP_READ_FILE_STAT:
            {
                if(SYS_FS_FileStat("/mnt/myDrive/newFile.txt", &appData.fileStatus) == SYS_FS_RES_FAILURE)
                {
                    /* Reading file status was a failure */
                    appData.state = APP_ERROR;
                }
                else
                {
                    /* Read file size */
                    appData.state = APP_READ_FILE_SIZE;
                }
                break;
            }

        case APP_READ_FILE_SIZE:
            {
                appData.fileSize = SYS_FS_FileSize(appData.fileHandle);
                if(appData.fileSize == -1)
                {
                    /* Reading file size was a failure */
                    appData.state = APP_ERROR;
                }
                else
                {
                    if(appData.fileSize == appData.fileStatus.fsize)
                    {
                        appData.state = APP_DO_FILE_SEEK;
                    }
                    else
                    {
                        appData.state = APP_ERROR;
                    }
                }
                break;
            }

        case APP_DO_FILE_SEEK:
            {
                if(SYS_FS_FileSeek( appData.fileHandle, appData.fileSize, SYS_FS_SEEK_SET ) == -1)
                {
                    /* File seek caused an error */
                    appData.state = APP_ERROR;
                }
                else
                {
                    /* Check for End of file */
                    appData.state = APP_CHECK_EOF;
                }
                break;
            }

        case APP_CHECK_EOF:
            {
                if(SYS_FS_FileEOF(appData.fileHandle) == false )
                {
                    /* Either, EOF is not reached or there was an error
                       In any case, for the application, its an error condition
                       */
                    appData.state = APP_ERROR;
                }
                else
                {
                    appData.state = APP_DO_ANOTHER_FILE_SEEK;
                }
                break;
            }

        case APP_DO_ANOTHER_FILE_SEEK:
            {
                /* Move file pointer to beginning of file */
                if(SYS_FS_FileSeek(appData.fileHandle, 0, SYS_FS_SEEK_SET) == -1)
                {
                    /* File seek caused an error */
                    appData.state = APP_ERROR;
                }
                else
                {
                    /* Check for original file content */
                    appData.state = APP_READ_FILE_CONTENT;
                }
                break;
            }

        case APP_READ_FILE_CONTENT:
            {
                if(SYS_FS_FileRead(appData.fileHandle, (void *)appData.data, appData.fileSize) == -1)
                {
                    /* There was an error while reading the file. Close the file
                     * and error out. */
                    SYS_FS_FileClose(appData.fileHandle);
                    appData.state = APP_ERROR;
                }
                else
                {
                    if ((appData.fileSize != 12) || (memcmp(appData.data, writeData, 12) != 0))
                    {
                        /* The written and the read data don't match. */
                        appData.state = APP_ERROR;
                    }
                    else
                    {
                        /* The test was successful. */
                        appData.state = APP_CLOSE_FILE;
                    }
                }
                break;
            }

        case APP_CLOSE_FILE:
            {
                /* Close the file */
                if (SYS_FS_FileClose(appData.fileHandle) != 0)
                {
                    appData.state = APP_ERROR;
                }
                else
                {
                    appData.state = APP_IDLE;
                }
                break;
            }

        case APP_IDLE:
            {
                /* The application comes here when the demo has completed
                 * successfully. */
                BSP_LEDOn(BSP_LED_9);
                break;
            }

        case APP_ERROR:
            {
                /* The application comes here when the demo has failed. */
                BSP_LEDOn(BSP_LED_8);
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


