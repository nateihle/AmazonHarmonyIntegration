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

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************

/* The demo app makes use of 32KB of NVM memory area starting at address
   DRV_NVM_MEDIA_START_ADDRESS. The app does the following:
   1. Erase the entire 32 KB of memory and verify the erase operation by reading
      back the data.
   2. Perform sequential writes within a page by queuing the write operations.
      Read back the data and verify.
   3. Repeat step 1 to erase all data of the previous operation.
   4. Perform random writes to addresses spread across the available memory area.
      This operation demonstrates the queuing of the write operations at the
      driver layer. It also demonstrates the usage of the driver event handler
      to track the completion of the queued operations. Read back the data and
      verify.
   5. Repeat step 1 to erase all data of the previous operation.
   6. Perform EraseWrite operation. This operation demonstrates the usage of the
      EraseWrite feature.
   */

/* NVM Media Layout:

   On MX devices:
      read block size  = 1 byte
      write block size = DRV_NVM_ROW_SIZE  = 512 bytes
      erase block size = DRV_NVM_PAGE_SIZE = 4096 bytes

  The following table illustrates the address and number of blocks for the read,
  write and erase regions for NVM media of size 32 KB on MX devices:
  ------------------------------------------------------------------------------
  |               | Block Size  | Number of Blocks             | Address Range |
  ------------------------------------------------------------------------------
  | Read region   | 1 Byte      | 32KB/Read Block Size = 32768 | 0 - 32767     |
  ------------------------------------------------------------------------------ 
  | Write Region  | 512 Bytes   | 32KB/Write Block Size = 64   | 0 - 63        |
  ------------------------------------------------------------------------------
  | Erase Region  | 4096 Bytes  | 32KB/Erase Block Size = 8    | 0 - 7         |
  ------------------------------------------------------------------------------ 

   On MZ devices:
      read block size  = 1 byte
      write block size = DRV_NVM_ROW_SIZE  = 2048 bytes
      erase block size = DRV_NVM_PAGE_SIZE = 16384 bytes

  The following table illustrates the address and number of blocks for the read,
  write and erase regions for NVM media of size 32 KB on MZ devices:
  ------------------------------------------------------------------------------
  |               | Block Size  | Number of Blocks             | Address Range |
  ------------------------------------------------------------------------------
  | Read region   | 1 Byte      | 32KB/Read Block Size = 32768 | 0 - 32767     |
  ------------------------------------------------------------------------------ 
  | Write Region  | 2048 Bytes  | 32KB/Write Block Size = 16   | 0 - 15        |
  ------------------------------------------------------------------------------
  | Erase Region  | 16384 Bytes | 32KB/Erase Block Size = 2    | 0 - 1         |
  ------------------------------------------------------------------------------ 
   
*/

/* gAppFlashReserveArea refers to the 32KB of memory starting from address 
   DRV_NVM_MEDIA_START_ADDRESS. This memory area is reserved for NVM read/write
   operations. The attribute keep instructs the linker not to remove the section
   even if it is not refered anywhere in the code.
 */
const uint8_t gAppFlashReserveArea[APP_NVM_MEMORY_AREA_SIZE] KEEP = {0};

/* Buffer used for reading data from the NVM */
uint8_t gAppReadBuffer [DRV_NVM_PAGE_SIZE];

/* Buffer used for writing data onto the NVM */
uint8_t gAppWriteBuffer[DRV_NVM_PAGE_SIZE];

/* Used to store addresses for random read write
 * operations.
 */
uint32_t gAppRandomAddress [8];

/* Data pattern used to fill up the gAppWriteBuffer.
 * This is used in the sequential, random read write
 * operations as well as the EraseWrite operations. 
 */
uint8_t gAppDataPattern [8] = {
	0x12,
	0x34,
	0x56,
	0x78,
	0x90,
	0xAB,
	0xCD,
	0xEF
};

/* Pointer to the NVM Media Geometry */
SYS_FS_MEDIA_GEOMETRY *gAppNVMMediaGeometry;

/*****************************************************
 * Initialize the application data structure. All
 * application related variables are stored in this
 * data structure.
 *****************************************************/

APP_DATA appData = 
{
    //Initialize appData structure. 

};
// *****************************************************************************
/* Driver objects.

  Summary:
    Contains driver objects.

  Description:
    This structure contains driver objects returned by the driver init routines
    to the application. These objects are passed to the driver tasks routines.
*/

APP_DRV_OBJECTS appDrvObject;

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Routines
// *****************************************************************************
// *****************************************************************************

/**********************************************************
 * This function is used to populate the gAppRandomAddress 
 * with addresses spread across the memory area. The random
 * read and write operations will be performed on these address
 * locations.
 ***********************************************************/
void APP_PopulateRandomAddresses 
(
    uint32_t numWriteBlocks
)
{
    /* Populate the array with addresses spread across the memory area,
       ensuring that no address is repeated.
    */
    gAppRandomAddress [0] = 0;
    gAppRandomAddress [1] = numWriteBlocks / 3;
    gAppRandomAddress [2] = numWriteBlocks / 2;
    gAppRandomAddress [3] = 1;
    gAppRandomAddress [4] = numWriteBlocks / 5;
    gAppRandomAddress [5] = numWriteBlocks / 4;
    gAppRandomAddress [6] = numWriteBlocks - 1;
    gAppRandomAddress [7] = numWriteBlocks - 2;
}

/**********************************************************
 * This function is used to fill up the write buffer with the
 * data pattern.
 ***********************************************************/
void APP_FillWriteBuffer ()
{
    uint32_t i = 0;

    /* There are eight rows in a page. Populate each row
       with a unique data pattern.
    */
    for(i = 0; i < DRV_NVM_ROW_SIZE; i++)
    {
        gAppWriteBuffer[i] = gAppDataPattern[0];
        gAppWriteBuffer[i + (1 * DRV_NVM_ROW_SIZE)] = gAppDataPattern[1];
        gAppWriteBuffer[i + (2 * DRV_NVM_ROW_SIZE)] = gAppDataPattern[2];
        gAppWriteBuffer[i + (3 * DRV_NVM_ROW_SIZE)] = gAppDataPattern[3];

        gAppWriteBuffer[i + (4 * DRV_NVM_ROW_SIZE)] = gAppDataPattern[4];
        gAppWriteBuffer[i + (5 * DRV_NVM_ROW_SIZE)] = gAppDataPattern[5];
        gAppWriteBuffer[i + (6 * DRV_NVM_ROW_SIZE)] = gAppDataPattern[6];
        gAppWriteBuffer[i + (7 * DRV_NVM_ROW_SIZE)] = gAppDataPattern[7];
    }
}

/**********************************************************
 * This function is used to verify the data written as part
 * of the sequential read write operations. 
 ***********************************************************/
bool APP_VerifySeqRWData ()
{
    uint32_t i = 0;

    for (i = 0; i < DRV_NVM_ROW_SIZE; i++)
    {
        if (gAppReadBuffer[i] != gAppDataPattern[0])
            return false;
        if (gAppReadBuffer[i + (1 * DRV_NVM_ROW_SIZE)] != gAppDataPattern[1])
            return false;
        if (gAppReadBuffer[i + (2 * DRV_NVM_ROW_SIZE)] != gAppDataPattern[2])
            return false;
        if (gAppReadBuffer[i + (3 * DRV_NVM_ROW_SIZE)] != gAppDataPattern[3])
            return false;
        if (gAppReadBuffer[i + (4 * DRV_NVM_ROW_SIZE)] != gAppDataPattern[4])
            return false;
        if (gAppReadBuffer[i + (5 * DRV_NVM_ROW_SIZE)] != gAppDataPattern[5])
            return false;
        if (gAppReadBuffer[i + (6 * DRV_NVM_ROW_SIZE)] != gAppDataPattern[6])
            return false;
        if (gAppReadBuffer[i + (7 * DRV_NVM_ROW_SIZE)] != gAppDataPattern[7])
            return false;
    }

    return true;
}

/**********************************************************
 * This function is used to verify the data written as part
 * of the random read write operations. 
 ***********************************************************/
bool APP_VerifyRandomData (uint8_t pattern)
{
    uint32_t i = 0;

    for (i = 0; i < DRV_NVM_ROW_SIZE; i++)
    {
        if (gAppReadBuffer [i] != pattern)
            return false;
    }

    return true;
}

/* This function verifies that the data in the first page is as 
 * expected.
 */
bool APP_VerifyPageOneData ()
{
    uint32_t i = 0;

    for(i = 0; i < DRV_NVM_ROW_SIZE; i ++)
    {
        /* 0 - 5 blocks should be empty*/
        if(gAppReadBuffer[i] != 0xFF)
            return false;
        if(gAppReadBuffer[i + DRV_NVM_ROW_SIZE] != 0xFF)
            return false;
        if(gAppReadBuffer[i + (2 * DRV_NVM_ROW_SIZE)] != 0xFF)
            return false;
        if(gAppReadBuffer[i + (3 * DRV_NVM_ROW_SIZE)] != 0xFF)
            return false;
        if(gAppReadBuffer[i + (4 * DRV_NVM_ROW_SIZE)] != 0xFF)
            return false;
        if(gAppReadBuffer[i + (5 * DRV_NVM_ROW_SIZE)] != 0xFF)
            return false;

        /* 6th block should have pattern gAppDataPattern[0] */
        if(gAppReadBuffer[i + (6 * DRV_NVM_ROW_SIZE)] != gAppDataPattern[0])
            return false;
        /* 7th block should have pattern gAppDataPattern[1] */
        if(gAppReadBuffer[i + (7 * DRV_NVM_ROW_SIZE)] != gAppDataPattern[1])
            return false;
    }

    return true;
}

/* This function verifies that the data in the second page is as 
 * expected.
 */
bool APP_VerifyPageTwoData ()
{
    uint32_t i = 0;

    for(i = 0; i < DRV_NVM_ROW_SIZE; i ++)
    {
        /* 8th block should have gAppDataPattern[2] */
        if(gAppReadBuffer[i] != gAppDataPattern[2])
            return false;
        /* 9th block should have gAppDataPattern[3] */
        if(gAppReadBuffer[i + DRV_NVM_ROW_SIZE] != gAppDataPattern[3])
            return false;
        /* 10th block should have gAppDataPattern[4] */
        if(gAppReadBuffer[i + (2 * DRV_NVM_ROW_SIZE)] != gAppDataPattern[4])
            return false;
        /* Rest of the blocks should be empty */
        if(gAppReadBuffer[i + (3 * DRV_NVM_ROW_SIZE)] != 0xFF)
            return false;
        if(gAppReadBuffer[i + (4 * DRV_NVM_ROW_SIZE)] != 0xFF)
            return false;
        if(gAppReadBuffer[i + (5 * DRV_NVM_ROW_SIZE)] != 0xFF)
            return false;
        if(gAppReadBuffer[i + (6 * DRV_NVM_ROW_SIZE)] != 0xFF)
            return false;
        if(gAppReadBuffer[i + (7 * DRV_NVM_ROW_SIZE)] != 0xFF)
            return false;
    }

    return true;
}

/**********************************************************
 * This function issues command to erase the entire memory 
 * area and then reads back the memory to ensure that the 
 * erase operation has set the bits to '1'
 ***********************************************************/
void APP_EraseMemoryAndVerify ( void )
{
    uint32_t i = 0;
    DRV_NVM_COMMAND_STATUS commandStatus;

    switch (appData.eraseState)
    {
        case APP_ERASE_STATE_INIT:
            {
                appData.eraseState = APP_ERASE_STATE_ERASE_CMD;
                appData.readBlockAddr    = 0;
                appData.numReadBlocks    = DRV_NVM_PAGE_SIZE;

                /* Intentional fall through */
            }

        case APP_ERASE_STATE_ERASE_CMD:
            {
                /* Erase the entire 32 KB of memory area */
                DRV_NVM_Erase(appData.nvmHandle, 
                        &appData.nvmCommandHandle[0],
                        0, 
                        gAppNVMMediaGeometry->geometryTable[APP_NVM_ERASE_REGION_INDEX].numBlocks);

                if(appData.nvmCommandHandle[0] != DRV_NVM_COMMAND_HANDLE_INVALID)
                {
                    appData.eraseState = APP_ERASE_STATE_ERASE_CMD_STATUS;
                }
                else
                {
                    appData.eraseState = APP_ERASE_STATE_ERROR;
                }
                break;
            }

        case APP_ERASE_STATE_ERASE_CMD_STATUS:
            {
                /* Check if the Erase operation has been completed successfully. */
                commandStatus = DRV_NVM_CommandStatus(appData.nvmHandle, appData.nvmCommandHandle[0]);
                if(DRV_NVM_COMMAND_COMPLETED == commandStatus)
                {
                    appData.eraseState = APP_ERASE_STATE_READ_CMD;
                }
                else if (DRV_NVM_COMMAND_ERROR_UNKNOWN == commandStatus)
                {
                    appData.eraseState = APP_ERASE_STATE_ERROR;
                }
                break;
            }

        case APP_ERASE_STATE_READ_CMD:
            {
                DRV_NVM_Read(appData.nvmHandle, &appData.nvmCommandHandle[0], 
                        gAppReadBuffer, appData.readBlockAddr, appData.numReadBlocks);

                if (appData.nvmCommandHandle[0] == DRV_NVM_COMMAND_HANDLE_INVALID)
                {
                    /* Failed to read data from the NVM */
                    appData.eraseState = APP_ERASE_STATE_ERROR; 
                }
                else
                {
                    appData.eraseState = APP_ERASE_STATE_READ_CMD_STATUS; 
                }

                break;
            }

        case APP_ERASE_STATE_READ_CMD_STATUS:
            {
                commandStatus = DRV_NVM_CommandStatus(appData.nvmHandle, appData.nvmCommandHandle[0]);
                if(DRV_NVM_COMMAND_COMPLETED == commandStatus)
                {
                    /* Update the read block address */
                    appData.readBlockAddr += appData.numReadBlocks;

                    appData.eraseState = APP_ERASE_STATE_VERIFY_DATA;
                }
                else if (DRV_NVM_COMMAND_ERROR_UNKNOWN == commandStatus)
                {
                    appData.eraseState = APP_ERASE_STATE_ERROR;
                }

                break;
            }

        case APP_ERASE_STATE_VERIFY_DATA:
            {
                for (i = 0; i < appData.numReadBlocks; i++)
                {
                    if (gAppReadBuffer[i] != 0xFF)
                    {
                        appData.eraseState = APP_ERASE_STATE_ERROR;
                        break;
                    }
                }

                if (appData.eraseState == APP_ERASE_STATE_VERIFY_DATA)
                {
                    if (appData.readBlockAddr == gAppNVMMediaGeometry->geometryTable[APP_NVM_READ_REGION_INDEX].numBlocks)
                    {
                        /* Completed verifying the erased memory. */
                        appData.eraseState = APP_ERASE_STATE_IDLE;
                    }
                    else
                    {
                        appData.eraseState = APP_ERASE_STATE_READ_CMD;
                    }
                }
                break;
            }

        case APP_ERASE_STATE_IDLE:
            {
                appData.eraseState = APP_ERASE_STATE_INIT;
                break;
            }

        case APP_ERASE_STATE_ERROR:
        default:
            {
                break;
            }
    }
}

/**********************************************************
 * This function performs a series of writes to sequential 
 * rows located in a single page, followed by read operations
 * to verify the data written during the write phase.
 ***********************************************************/
void APP_SequentialReadWrite (void)
{
    DRV_NVM_COMMAND_STATUS commandStatus;
            
    switch(appData.seqState)
    {
        case APP_SEQ_RW_INIT:
            {
                /* Clear the event counts */
                appData.eventCount = 0;
                appData.errorEventCount = 0;
                appData.seqState = APP_SEQ_RW_WRITE;
                
                /* Intentional Fallthrough */
            }

        case APP_SEQ_RW_WRITE:
            {
                /* Queue write operation at the driver layer */
                DRV_NVM_Write(appData.nvmHandle, &appData.nvmCommandHandle[0], gAppWriteBuffer, 0, 3);
                if (appData.nvmCommandHandle[0] == DRV_NVM_COMMAND_HANDLE_INVALID)
                {
                    appData.seqState = APP_SEQ_RW_ERROR;
                    break;
                }

                DRV_NVM_Write(appData.nvmHandle, &appData.nvmCommandHandle[1], &gAppWriteBuffer[3 * DRV_NVM_ROW_SIZE], 3, 1);
                if (appData.nvmCommandHandle[1] == DRV_NVM_COMMAND_HANDLE_INVALID)
                {
                    appData.seqState = APP_SEQ_RW_ERROR;
                    break;
                }

                DRV_NVM_Write(appData.nvmHandle, &appData.nvmCommandHandle[2], &gAppWriteBuffer[4 * DRV_NVM_ROW_SIZE], 4, 2);
                if (appData.nvmCommandHandle[2] == DRV_NVM_COMMAND_HANDLE_INVALID)
                {
                    appData.seqState = APP_SEQ_RW_ERROR;
                    break;
                }

                DRV_NVM_Write(appData.nvmHandle, &appData.nvmCommandHandle[3], &gAppWriteBuffer[6 * DRV_NVM_ROW_SIZE], 6, 2);
                if (appData.nvmCommandHandle[3] == DRV_NVM_COMMAND_HANDLE_INVALID)
                {
                    appData.seqState = APP_SEQ_RW_ERROR;
                    break;
                }

                appData.seqState = APP_SEQ_RW_WRITE_STATUS;
                break;
            }

        case APP_SEQ_RW_WRITE_STATUS:
            {
                /* wait for the driver to complete the write operations */
                if (appData.eventCount == 4)
                {
                    appData.seqState = APP_SEQ_RW_READ;
                }

                if (appData.errorEventCount > 0)
                {
                    appData.seqState = APP_SEQ_RW_ERROR;
                }

                break;
            }

        case APP_SEQ_RW_READ:
            {
                /* Read data back and verify */
                DRV_NVM_Read(appData.nvmHandle, &appData.nvmCommandHandle[0], gAppReadBuffer, 0, DRV_NVM_PAGE_SIZE);

                if (appData.nvmCommandHandle[0] == DRV_NVM_COMMAND_HANDLE_INVALID)
                {
                    appData.seqState = APP_SEQ_RW_ERROR;
                }
                else
                {
                    appData.seqState = APP_SEQ_RW_READ_STATUS;
                }

                break;
            }

        case APP_SEQ_RW_READ_STATUS:
            {
                commandStatus = DRV_NVM_CommandStatus(appData.nvmHandle, appData.nvmCommandHandle[0]);

                if(DRV_NVM_COMMAND_COMPLETED == commandStatus)
                {
                    if (APP_VerifySeqRWData ())
                    {
                        /* Sequential read write operation is complete */
                        appData.seqState = APP_SEQ_RW_IDLE;
                    }
                    else
                    {
                        appData.seqState = APP_SEQ_RW_ERROR;
                    }
                }
                else if (DRV_NVM_COMMAND_ERROR_UNKNOWN == commandStatus)
                {
                    appData.seqState = APP_SEQ_RW_ERROR;
                }
                break;
            }

        case APP_SEQ_RW_IDLE:
            {
                /* Sequential read write operation is completed successfully */
                appData.seqState = APP_SEQ_RW_INIT;
                break;
            }

        case APP_SEQ_RW_ERROR:
        default:
            {
                /* Sequential read write operation failed */
                break;
            }
    }
}

/**********************************************************
 * This function performs a series of writes to random 
 * addresses spread across the entire available media, 
 * followed by read operations to verify the data written 
 * during the write phase.
 ***********************************************************/
void APP_RandomReadWrite ( void )
{
    uint32_t i = 0;
    DRV_NVM_COMMAND_STATUS commandStatus;

    switch (appData.randomState)
    {
        case APP_RANDOM_RW_INIT:
            {
                /* Clear the event counts */
                appData.eventCount = 0;
                appData.errorEventCount = 0;
                appData.randomRWCount = 0;

                appData.randomState = APP_RANDOM_RW_WRITE;
                /* Intentional Fallthrough */
            }

        case APP_RANDOM_RW_WRITE:
            {
                /* Queue write operations at the driver layer. */
                for (i = 0; i < 8; i++)
                {
                    DRV_NVM_Write(appData.nvmHandle, &appData.nvmCommandHandle[i],
                            &gAppWriteBuffer[i * DRV_NVM_ROW_SIZE], gAppRandomAddress[i], 1);

                    if (appData.nvmCommandHandle[i] == DRV_NVM_COMMAND_HANDLE_INVALID)
                    {
                        appData.randomState = APP_RANDOM_RW_ERROR;
                        break;
                    }
                }

                if (appData.randomState != APP_RANDOM_RW_ERROR)
                {
                    appData.randomState = APP_RANDOM_RW_WRITE_STATUS;
                }

                break;
            }

        case APP_RANDOM_RW_WRITE_STATUS:
            {
                /* Wait for the driver to complete the write operations */
                if (appData.eventCount == 8)
                {
                    appData.randomState = APP_RANDOM_RW_READ;
                }

                if (appData.errorEventCount > 0)
                {
                    appData.randomState = APP_RANDOM_RW_ERROR;
                }

                break;
            }

        case APP_RANDOM_RW_READ:
            {
                /* Start reading the data */
                DRV_NVM_Read(appData.nvmHandle, &appData.nvmCommandHandle[0], 
                        gAppReadBuffer, (gAppRandomAddress[appData.randomRWCount] * DRV_NVM_ROW_SIZE), 
                        DRV_NVM_ROW_SIZE);
                if (appData.nvmCommandHandle[0] == DRV_NVM_COMMAND_HANDLE_INVALID)
                {
                    appData.randomState = APP_RANDOM_RW_ERROR;
                }
                else
                {
                    appData.randomState = APP_RANDOM_RW_READ_STATUS;
                }
                break;
            }

        case APP_RANDOM_RW_READ_STATUS:
            {
                commandStatus = DRV_NVM_CommandStatus(appData.nvmHandle, appData.nvmCommandHandle[0]);
                if(DRV_NVM_COMMAND_COMPLETED == commandStatus)
                {
                    appData.randomState = APP_RANDOM_RW_VERIFY_DATA;
                }
                else if (DRV_NVM_COMMAND_ERROR_UNKNOWN == commandStatus)
                {
                    appData.randomState = APP_RANDOM_RW_ERROR;
                }
                break;
            }

        case APP_RANDOM_RW_VERIFY_DATA:
            {
                /* Verify the data */
                if (APP_VerifyRandomData(gAppDataPattern[appData.randomRWCount]))
                {
                    /* Update the counter */
                    appData.randomRWCount ++;
                    if (appData.randomRWCount == 8)
                    {
                        /* Random read write operation is complete. */
                        appData.randomState = APP_RANDOM_RW_IDLE;
                    }
                    else
                    {
                        appData.randomState = APP_RANDOM_RW_READ;
                    }
                }
                else
                {
                    appData.randomState = APP_RANDOM_RW_ERROR;
                }
                break;
            }

        case APP_RANDOM_RW_IDLE:
            {
                /* Random read write operation is completed successfully */
                appData.randomState = APP_RANDOM_RW_INIT;
                break;
            }

        case APP_RANDOM_RW_ERROR:
        default:
            {
                /* Random read write operation failed */
                break;
            }
    }
}

/**********************************************************
 * This function makes use of the EraseWrite feature. This
 * feature combines the erase step along with that of the
 * write operation. The function writes different data pattern
 * to pages 1 and 2 and verifies the EraseWrite functionality.
 ***********************************************************/
void APP_EraseWriteOperations ( void )
{
    DRV_NVM_COMMAND_STATUS commandStatus;

    switch (appData.eraseWriteState)
    {
        case APP_ERASEWRITE_INIT:
            {
                /* Clear the event counts */
                appData.eventCount = 0;
                appData.errorEventCount = 0;
                appData.eraseWriteState = APP_ERASEWRITE_ERASEWIRTE;
                /* Intentional Fallthrough */
            }

        case APP_ERASEWRITE_ERASEWIRTE:
            {
                /* Erase write combines the erase and write operation. Even if 
                   write is spilled over to the next page, the EraseWrite 
                   operation handles it. 
                   0 - 5 rows of page 1 are left untouched
                   6th row - data pattern 0 is programmed
                   7th row - data pattern 1 is programmed
                   0th row of page 2 - data pattern 2 is programmed
                   1st row of page 2 - data pattern 3 is programmed
                   2nd row of page 2 - data pattern 4 is programmed
                   */
                DRV_NVM_EraseWrite(appData.nvmHandle, &appData.nvmCommandHandle[0],
                        gAppWriteBuffer, 6, 2);
                
				DRV_NVM_EraseWrite(appData.nvmHandle, &appData.nvmCommandHandle[0],
				        &gAppWriteBuffer[2 * DRV_NVM_ROW_SIZE], (DRV_NVM_PAGE_SIZE/DRV_NVM_ROW_SIZE) + 0, 1);

                if (appData.nvmCommandHandle[0] == DRV_NVM_COMMAND_HANDLE_INVALID)
                {
                    appData.eraseWriteState = APP_ERASEWRITE_ERROR;
                    break;
                }

                DRV_NVM_EraseWrite(appData.nvmHandle, &appData.nvmCommandHandle[1],
                        &gAppWriteBuffer[3 * DRV_NVM_ROW_SIZE],(DRV_NVM_PAGE_SIZE/DRV_NVM_ROW_SIZE) + 1, 1);
                if (appData.nvmCommandHandle[1] == DRV_NVM_COMMAND_HANDLE_INVALID)
                {
                    appData.eraseWriteState = APP_ERASEWRITE_ERROR;
                    break;
                }

                DRV_NVM_EraseWrite(appData.nvmHandle, &appData.nvmCommandHandle[2],
                        &gAppWriteBuffer[4 * DRV_NVM_ROW_SIZE], (DRV_NVM_PAGE_SIZE/DRV_NVM_ROW_SIZE) + 2, 1);
                if (appData.nvmCommandHandle[2] == DRV_NVM_COMMAND_HANDLE_INVALID)
                {
                    appData.eraseWriteState = APP_ERASEWRITE_ERROR;
                    break;
                }

                appData.eraseWriteState = APP_ERASEWRITE_ERASEWIRTE_STATUS;
                break;
            }

        case APP_ERASEWRITE_ERASEWIRTE_STATUS:
            {
                /* Wait for the EraseWrite complete events */
                if (appData.eventCount == 4)
                {
                    appData.eraseWriteState = APP_ERASEWRITE_READ_PAGE_ONE;
                }

                if (appData.errorEventCount > 0)
                {
                    appData.eraseWriteState = APP_ERASEWRITE_ERROR;
                }

                break;
            }

        case APP_ERASEWRITE_READ_PAGE_ONE:
            {
                /* Read page one data and verify the data */
                DRV_NVM_Read(appData.nvmHandle, &appData.nvmCommandHandle[0], 
                        gAppReadBuffer, 0, DRV_NVM_PAGE_SIZE);
                if (appData.nvmCommandHandle[0] == DRV_NVM_COMMAND_HANDLE_INVALID)
                {
                    appData.eraseWriteState = APP_ERASEWRITE_ERROR;
                }
                else
                {
                    appData.eraseWriteState = APP_ERASEWRITE_READ_PAGE_ONE_STATUS;
                }
                break;
            }

        case APP_ERASEWRITE_READ_PAGE_ONE_STATUS:
            {
                commandStatus = DRV_NVM_CommandStatus(appData.nvmHandle, appData.nvmCommandHandle[0]);
                if(DRV_NVM_COMMAND_COMPLETED == commandStatus)
                {
                    appData.eraseWriteState = APP_ERASEWRITE_VERIFY_PAGE_ONE_DATA;
                }
                else if (DRV_NVM_COMMAND_ERROR_UNKNOWN == commandStatus)
                {
                    appData.eraseWriteState = APP_ERASEWRITE_ERROR;
                }
                break;
            }

        case APP_ERASEWRITE_VERIFY_PAGE_ONE_DATA:
            {
                /* Verify the page 1 data */
                if (APP_VerifyPageOneData())
                {
                    appData.eraseWriteState = APP_ERASEWRITE_READ_PAGE_TWO;
                }
                else
                {
                    appData.eraseWriteState = APP_ERASEWRITE_ERROR;
                }
                break;
            }

        case APP_ERASEWRITE_READ_PAGE_TWO:
            {
                /* Read the page 2 data */
                DRV_NVM_Read(appData.nvmHandle, &appData.nvmCommandHandle[0], 
                        gAppReadBuffer, DRV_NVM_PAGE_SIZE, DRV_NVM_PAGE_SIZE);
                if (appData.nvmCommandHandle[0] == DRV_NVM_COMMAND_HANDLE_INVALID)
                {
                    appData.eraseWriteState = APP_ERASEWRITE_ERROR;
                }
                else
                {
                    appData.eraseWriteState = APP_ERASEWRITE_READ_PAGE_TWO_STATUS;
                }
                break;
            }

        case APP_ERASEWRITE_READ_PAGE_TWO_STATUS:
            {
                commandStatus = DRV_NVM_CommandStatus(appData.nvmHandle, appData.nvmCommandHandle[0]);
                if(DRV_NVM_COMMAND_COMPLETED == commandStatus)
                {
                    appData.eraseWriteState = APP_ERASEWRITE_VERIFY_PAGE_TWO_DATA;
                }
                else if (DRV_NVM_COMMAND_ERROR_UNKNOWN == commandStatus)
                {
                    appData.eraseWriteState = APP_ERASEWRITE_ERROR;
                }
                break;
            }

        case APP_ERASEWRITE_VERIFY_PAGE_TWO_DATA:
            {
                /* Verify the page 2 data */
                if (APP_VerifyPageTwoData())
                {
                    appData.eraseWriteState = APP_ERASEWRITE_IDLE;
                }
                else
                {
                    appData.eraseWriteState = APP_ERASEWRITE_ERROR;
                }
                break;
            }

        case APP_ERASEWRITE_IDLE:
            {
                /* EraseWrite completed successfully */
                appData.eraseWriteState = APP_ERASEWRITE_INIT;
                break;
            }

        case APP_ERASEWRITE_ERROR:
        default:
            {
                /* EraseWrite failed */
                break;
            }
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************

/********************************************************
 * NVM Driver Events handler
 ********************************************************/

void APP_EventHandler
(
    DRV_NVM_EVENT event,
    DRV_NVM_COMMAND_HANDLE commandHandle,
    uintptr_t context
)
{
    switch (event)
    {
        case DRV_NVM_EVENT_COMMAND_COMPLETE:
            {
                appData.eventCount ++;
                break;
            }

        case DRV_NVM_EVENT_COMMAND_ERROR:
            {
                appData.errorEventCount ++;
                break;
            }

        default:
            {
                break;
            }
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

void APP_Initialize ( void )
{
    /* Initialize the state variables to default values. */
    appData.eraseState      = APP_ERASE_STATE_INIT;
    appData.seqState        = APP_SEQ_RW_INIT;
    appData.randomState     = APP_RANDOM_RW_INIT;
    appData.eraseWriteState = APP_ERASEWRITE_INIT;
    appData.state           = APP_STATE_INIT;

    appData.eventCount      = 0;
    appData.errorEventCount = 0;
    appData.randomRWCount   = 0;
    appData.readBlockAddr   = 0;
    appData.numReadBlocks   = 0;

    APP_FillWriteBuffer ();
}

/**********************************************************
 * Application tasks routine. This function implements the
 * application state machine.
 ***********************************************************/
void APP_Tasks ( void )
{
    switch(appData.state)
    {
        case APP_STATE_INIT:
            appData.nvmHandle = DRV_NVM_Open(0, DRV_IO_INTENT_READWRITE);
            if(DRV_HANDLE_INVALID == appData.nvmHandle)
            {
                appData.state = APP_STATE_ERROR;
                break;
            }

            /* Register for NVM driver events */
            DRV_NVM_EventHandlerSet (appData.nvmHandle, APP_EventHandler, 1);

            /* Read the NVM Media Geometry. */
            gAppNVMMediaGeometry = DRV_NVM_GeometryGet(appData.nvmHandle);
            if(NULL == gAppNVMMediaGeometry)
            {
                appData.state = APP_STATE_ERROR;
                break;
            }

            /* Select addresses spread across the available memory area */
            APP_PopulateRandomAddresses (gAppNVMMediaGeometry->geometryTable[APP_NVM_WRITE_REGION_INDEX].numBlocks);

            /* After the erase operation is completed start the sequential 
               read write operation.
               */
            appData.nextState = APP_STATE_SEQ_RW;
            appData.state = APP_STATE_ERASE_ALL;
            break;

        case APP_STATE_ERASE_ALL:
            /* The erase operation sets all bits to 1.  Erase the memory area 
               and verify that it has indeed been erased by reading back the
               memory and checking that all bits are set to 1.
               */
            APP_EraseMemoryAndVerify ();
            if (appData.eraseState == APP_ERASE_STATE_IDLE)
            {
                /* Erase and verify operation is complete. Move on
                to the next state. */
                appData.state = appData.nextState;
            }
            else if (appData.eraseState == APP_ERASE_STATE_ERROR)
            {
                /* Erase and verify operation failed. */
                appData.state = APP_STATE_ERROR;
            }
            break;

        case APP_STATE_SEQ_RW:
            APP_SequentialReadWrite ();
            if (appData.seqState == APP_SEQ_RW_IDLE)
            {
                /* Sequential Read Write Operation is complete. Move on
                to the next state. */
                appData.nextState = APP_STATE_RANDOM_RW;
                appData.state = APP_STATE_ERASE_ALL;
            }
            else if (appData.seqState == APP_SEQ_RW_ERROR)
            {
                /* Sequential Read Write Operation failed. */
                appData.state = APP_STATE_ERROR;
            }
            break;

        case APP_STATE_RANDOM_RW:
            /* Perform random writes and reads */
            APP_RandomReadWrite ();
            if (appData.randomState == APP_RANDOM_RW_IDLE)
            {
                /* Random Read Write Operation is complete. Move on
                to the next state. */
                appData.nextState = APP_STATE_ERASEWRITE_RW;
                appData.state = APP_STATE_ERASE_ALL;
            }
            else if (appData.randomState == APP_RANDOM_RW_ERROR)
            {
                /* Random Read Write Operation failed. */
                appData.state = APP_STATE_ERROR;
            }
            break;

        case APP_STATE_ERASEWRITE_RW:
            /* Perfrom EraseWrite operations */
            APP_EraseWriteOperations ();
            if (appData.eraseWriteState == APP_ERASEWRITE_IDLE)
            {
                /* Erase write Operation is complete. Move on
                to the next state. */
                appData.state = APP_STATE_CLOSE;
            }
            else if (appData.eraseWriteState == APP_ERASEWRITE_ERROR)
            {
                /* Erase Write Operation failed. */
                appData.state = APP_STATE_ERROR;
            }
            break;

        case APP_STATE_CLOSE:
        {
            /* Close the driver */
            DRV_NVM_Close (appData.nvmHandle);
            appData.state = APP_STATE_IDLE;

            /* Intentional fallthrough */
        }
        case APP_STATE_IDLE:
        {
            /* App demo completed successfully. */
            BSP_LEDStateSet(APP_SUCCESS_LED, BSP_LED_STATE_ON);
            break;
        }

        case APP_STATE_ERROR:
        default:
        {
            /* App demo failed. */
            BSP_LEDStateSet(APP_FAILURE_LED, BSP_LED_STATE_ON);
            break;
        }
    }
}

/*******************************************************************************
 End of File
 */

