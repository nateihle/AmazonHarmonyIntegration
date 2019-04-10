/*******************************************************************************
  SST25VF020B SPI Flash Driver Dynamic implemention.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sst25vf020b_erasewrite.c

  Summary:
    Source code for the SST25VF020B Erase write feature implementation.

  Description:
    This file implements the SST25VF020B Driver Erase Write function.
    This file should be included in the project if SST25VF020B driver Erase Write
    functionality is needed.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "driver/spi_flash/sst25vf020b/src/drv_sst25vf020b_local.h"

// *****************************************************************************
/* Function:
    void DRV_SST25VF020B_BlockEraseWrite
    (
        const DRV_HANDLE handle,
        DRV_SST25VF020B_BLOCK_COMMAND_HANDLE * commandHandle,
        void * sourceBuffer,
        uint32_t writeBlockStart,
        uint32_t nWriteBlock
    )

  Summary:
    Erase and Write blocks of data starting from a specified address in SST flash
    memory.

  Description:
    This function combines the step of erasing blocks of SST Flash and then writing 
    the data. The application can use this function if it wants to avoid having to
    explicitly delete a block in order to update the bytes contained in the block. 

    This function schedules a non-blocking operation to erase and write blocks
    of data into SST flash. The function returns with a valid buffer handle
    in the commandHandle argument if the write request was scheduled successfully.
    The function adds the request to the hardware instance queue and returns 
    immediately. While the request is in the queue, the application buffer is 
    owned by the driver and should not be modified. The function returns 
    DRV_SST25VF020B_BLOCK_COMMAND_HANDLE_INVALID in the commandHandle argument under the 
    following circumstances:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the client opened the driver for read only
    - if the buffer size is 0 
    - if the queue size is full or queue depth is insufficient
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_SST25VF020B_EVENT_BLOCK_COMMAND_COMPLETE event if the buffer
    was processed successfully or DRV_SST25VF020B_EVENT_ERASE_ERROR event if 
    the buffer was not processed successfully.

  Remarks:
    Refer to drv_sst25vf020b.h for usage information.
*/

void DRV_SST25VF020B_BlockEraseWrite
(
    DRV_HANDLE hClient,
    DRV_SST25VF020B_BLOCK_COMMAND_HANDLE * commandHandle,
    uint8_t *sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SST25VF020B_CLIENT_OBJ * clientObj = NULL;
    DRV_SST25VF020B_OBJ * hDriver = NULL;
    DRV_SST25VF020B_BUFFER_OBJ * bufferObj = NULL, * iterator = NULL;
    unsigned int iEntry = 0;

    /* This function adds a buffer to the queue */

    /* We first check the arguments and initialize the
       command handle to INVALID */

    if(commandHandle != NULL)
    {
        *commandHandle = DRV_SST25VF020B_BLOCK_COMMAND_HANDLE_INVALID;
    }

    /* Validate the driver handle */
    clientObj = _DRV_SST25VF020B_DriverHandleValidate(hClient);
    if(clientObj == NULL)
    {
        /* We got an invalid client handle */
        SYS_DEBUG(0, "DRV_SST25VF020B: Invalid Driver Handle \n");
        return;
    }

    /* Check if Driver was open for Wtiting or not */
    if(!(clientObj->ioIntent & DRV_IO_INTENT_WRITE))
    {
        /* Driver is not open in Write mode */
        SYS_DEBUG(0, "DRV_SST25VF020B: Driver is not open in Write mode \n");
        return;
    }
    
    if((nBlock == 0) || (sourceBuffer == NULL ))
    {
        /* We either got an invalid block size or
         * invalid source buffer */
        SYS_DEBUG(0, "DRV_SST25VF020B: Invalid parameters \n");
        return;
    }

    hDriver = clientObj->hDriver;

    if(hDriver->queueOccupancy >= hDriver->queueSize)
    {
        /* This means the queue is full. We cannot add
           this request */

        SYS_DEBUG(0, "DRV_SST25VF020B: Queue is full \n");
        return;
    }

    /* Search the buffer pool for a free buffer object */
    for(iEntry = 0 ; iEntry < DRV_SST25VF020B_QUEUE_DEPTH_COMBINED; iEntry ++)
    {
        if(!gDrvSST25VF020BBufferObj[iEntry].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj = &gDrvSST25VF020BBufferObj[iEntry];
            bufferObj->hClient = clientObj;
            bufferObj->inUse = true;
            bufferObj->nCurrentBlocks = 0;
            bufferObj->size = nBlock;
            bufferObj->commandHandle    = _DRV_SST25VF020B_MAKE_HANDLE(gDrvSST25VF020BBufferToken, iEntry);
            bufferObj->commandStatus = DRV_SST25VF020B_COMMAND_QUEUED;
            bufferObj->address = blockStart;
            bufferObj->next = NULL;
            bufferObj->previous = NULL;
            bufferObj->buffer = sourceBuffer;
            bufferObj->operation = DRV_SST25VF020B_BLOCK_ERASE_WRITE;

            if(commandHandle != NULL)
            {
                /* Assign the return handle of this buffer */
                *commandHandle = bufferObj->commandHandle;
                break;
            }

            /* Update the token number. */
            _DRV_SST25VF020B_UPDATE_BUF_TOKEN(gDrvSST25VF020BBufferToken);
        }
    }

    if(iEntry == DRV_SST25VF020B_QUEUE_DEPTH_COMBINED)
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_SST25VF020B_QUEUE_DEPTH_COMBINED
           parameter is configured to be less */

        SYS_DEBUG(0, "DRV_SST25VF020B: Insufficient Queue Depth \n");
        return;
    }

    /* Check if the queue is empty */
    if(hDriver->queueTail == NULL)
    {
        /* This is the first buffer in the queue */
        hDriver->queueTail = bufferObj;
    }
    else
    {
        /* This means the write queue is not empty. We must add
         * the buffer object to the end of the queue */

        iterator = hDriver->queueTail;
        while(iterator->next != NULL)
        {
            /* Get the next buffer object */
            iterator = iterator->next;
        }

        /* At this point, iterator will point to the
           last object in the queue. We add the buffer
           object to the linked list. Note that we
           need to set up the previous pointer as well
           because buffer should be deleted when the
           client closes the driver */

        iterator->next = bufferObj;
        bufferObj->previous = iterator;
    }

    /* increment the current queue size */
    hDriver->queueOccupancy++;
    
    return;
}

