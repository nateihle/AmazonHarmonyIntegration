/*******************************************************************************
  SST25 SPI Flash Driver Dynamic implemention.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sst25.c

  Summary:
    Source code for the SPI flash driver dynamic implementation.

  Description:
    This file contains the source code for the dynamic implementation of the 
    SST25 SPI Flash driver.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute Software
only when embedded on a Microchip microcontroller or digital  signal  controller
that is integrated into your product or third party  product  (pursuant  to  the
sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
//DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "driver/spi_flash/sst25/src/drv_sst25_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/* This is the driver instance object array. */
DRV_SST25_OBJ gDrvSst25Obj[DRV_SST25_INSTANCES_NUMBER] ;

/* This is the client object array. */
DRV_SST25_CLIENT_OBJ gDrvSst25ClientObj[DRV_SST25_CLIENTS_NUMBER];

/* This is the array of SST25 SPI Flash Driver Buffer objects. */
DRV_SST25_BUFFER_OBJ gDrvSst25BufferObj[DRV_SST25_BUFFER_OBJ_NUMBER];

/************************************************
 * This token is incremented for every request
 * added to the queue and is used to generate
 * a different buffer handle for every request.
 ***********************************************/
uint16_t gDrvSst25BufferToken = 0;
uint8_t gDrvSst25InitCount = 0;
uint8_t __attribute__((coherent)) gDrvSst25StatusReg = 0; 
uint8_t gDrvSst25FlashId[2] __attribute__((coherent));

uint8_t gDrvSst25SectorBuffer[DRV_SST25_INSTANCES_NUMBER][DRV_SST25_ERASE_SECTOR_SIZE] __attribute__((coherent, aligned(16)));

uint8_t gDrvSst25CmdParams[DRV_SST25_INSTANCES_NUMBER][260] __attribute__((coherent, aligned(16)));
/* Table mapping the Flash ID's to their sizes. */
const uint32_t gSstFlashIdSizeTable [DRV_SST25_NUM_DEVICE_SUPPORTED][2] = {
    {0x49, 0x20000},  /* SST25VF010A - 1 MBit */
    {0x8C, 0x40000},  /* SST25VF020B - 2 MBit */
    {0x8D, 0x80000},  /* SST25VF040B - 4 MBit */
    {0x8E, 0x100000}, /* SST25VF080B - 8 MBit */
    {0x41, 0x200000}, /* SST25VF016B - 16 MBit */
    {0x4B, 0x800000}  /* SST25VF064C - 64 MBit */
};

OSAL_MUTEX_DECLARE(gSst25ClientObjMutex);

const SYS_FS_MEDIA_FUNCTIONS sst25MediaFunctions =
{
    .mediaStatusGet     = DRV_SST25_MediaIsAttached,
    .mediaGeometryGet   = DRV_SST25_GeometryGet,
    .sectorRead         = DRV_SST25_BlockRead,
    .sectorWrite        = DRV_SST25_BlockEraseWrite,
    .eventHandlerset    = DRV_SST25_BlockEventHandlerSet,
    .commandStatusGet   = (void *)DRV_SST25_CommandStatus,
    .Read               = DRV_SST25_BlockRead,
    .erase              = DRV_SST25_BlockErase,
    .addressGet         = DRV_SST25_AddressGet,
    .open               = DRV_SST25_Open,
    .close              = DRV_SST25_Close,
    .tasks              = DRV_SST25_Tasks,
};

static DRV_SPI_BUFFER_EVENT DRV_SST25_HandleRead
(
    void *driverObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
);

static DRV_SPI_BUFFER_EVENT DRV_SST25_WritePageProgram
(
    void *driverObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
);

static DRV_SPI_BUFFER_EVENT DRV_SST25_WriteAutoAddressIncrement
(
    void *driverObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
);

static DRV_SPI_BUFFER_EVENT DRV_SST25_HandleErase
(
    void *driverObj,
    uint32_t blockAddress,
    uint32_t nBlocks
);

static DRV_SPI_BUFFER_EVENT DRV_SST25_HandleEraseWrite
(
    void *driverObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
);

// *****************************************************************************
// *****************************************************************************
// Section: SST25 SPI Flash Driver Interface Implementations
// *****************************************************************************
// *****************************************************************************

/* This function finds a free buffer object and populates it with the transfer
 * parameters. It also generates a new command handle for the request. */
static DRV_SST25_BUFFER_OBJ* DRV_SST25_AllocateBufferObject
(
    DRV_SST25_CLIENT_OBJ *clientObj,
    void *buffer,
    uint32_t blockStart,
    uint32_t nBlocks,
    DRV_SST25_OPERATION_TYPE opType
)
{
    uint8_t iEntry = 0;
    DRV_SST25_BUFFER_OBJ *bufferObj = NULL;

    for (iEntry = 0; iEntry < DRV_SST25_BUFFER_OBJ_NUMBER; iEntry++)
    {
        /* Search for a free buffer object to use */
        if (gDrvSst25BufferObj[iEntry].inUse == false)
        {
            /* Found a free buffer object. */
            bufferObj = &gDrvSst25BufferObj[iEntry];

            bufferObj->inUse         = true;
            bufferObj->commandHandle = DRV_SST25_MAKE_HANDLE(gDrvSst25BufferToken, iEntry);
            bufferObj->hClient       = clientObj;
            bufferObj->buffer        = buffer;
            bufferObj->blockStart    = blockStart;
            bufferObj->nBlocks       = nBlocks;
            bufferObj->opType        = opType;
            bufferObj->status        = DRV_SST25_COMMAND_QUEUED;
            bufferObj->next          = NULL;
            bufferObj->previous      = NULL;

            /* Update the token number. */
            DRV_SST25_UPDATE_BUF_TOKEN(gDrvSst25BufferToken);
            break;
        }
    }

    return bufferObj;
}

/* This function returns the flash size in bytes for the specified deviceId. A
 * zero is returned if the device id is not supported. */
static uint32_t DRV_SST25_GetFlashSize
(
    uint8_t deviceId
)
{
    uint8_t i = 0;

    for (i = 0; i < DRV_SST25_NUM_DEVICE_SUPPORTED; i++)
    {
        if (deviceId == gSstFlashIdSizeTable[i][0])
        {
            return gSstFlashIdSizeTable[i][1];
        }
    }

    return 0;
}

/* This function updates the driver object's geometry information for the flash
 * device. */
static bool DRV_SST25_UpdateGeometry
(
    DRV_SST25_OBJ *dObj,
    uint8_t deviceId
)
{
    uint32_t flashSize = DRV_SST25_GetFlashSize (deviceId);
    if (flashSize == 0)
    {
        return false;
    }

    /* Read block size and number of blocks */
    dObj->mediaGeometryTable[0].blockSize = 1;
    dObj->mediaGeometryTable[0].numBlocks = flashSize;

    if (deviceId == 0x4B)
    {
        /* SST25VF064C */
        /* Write block size and number of blocks */
        dObj->mediaGeometryTable[1].blockSize = DRV_SST25_PAGE_SIZE;
        dObj->mediaGeometryTable[1].numBlocks = flashSize >> 8;
        dObj->flashFunctions.write = DRV_SST25_WritePageProgram;
    }
    else
    {
        /* Write block size and number of blocks */
        dObj->mediaGeometryTable[1].blockSize = 2;
        dObj->mediaGeometryTable[1].numBlocks = flashSize >> 1;

        dObj->flashFunctions.write = DRV_SST25_WriteAutoAddressIncrement;
        if (deviceId == 0x49)
        {
            /* SST25VF010A */
            dObj->opCodes.write = DRV_SST25_CMD_AAI_PROGRAM1;
        }
        else
        {
            dObj->opCodes.write = DRV_SST25_CMD_AAI_PROGRAM;
        }
    }

    /* Erase block size and number of blocks */
    dObj->mediaGeometryTable[2].blockSize = DRV_SST25_ERASE_SECTOR_SIZE;
    dObj->mediaGeometryTable[2].numBlocks = flashSize >> 12;

    /* Update the Media Geometry Main Structure */
    dObj->mediaGeometryObj.mediaProperty = (SYS_FS_MEDIA_READ_IS_BLOCKING | SYS_FS_MEDIA_WRITE_IS_BLOCKING),

    /* Number of read, write and erase entries in the table */
    dObj->mediaGeometryObj.numReadRegions = 1,
    dObj->mediaGeometryObj.numWriteRegions = 1,
    dObj->mediaGeometryObj.numEraseRegions = 1,
    dObj->mediaGeometryObj.geometryTable = (SYS_FS_MEDIA_REGION_GEOMETRY *)&dObj->mediaGeometryTable;

    return true;
}

/* This function adds the buffer object to the tail of the queue. */
static void DRV_SST25_AddToQueue
(
    DRV_SST25_OBJ *dObj,
    DRV_SST25_BUFFER_OBJ *bufferObj
)
{
    if (dObj->queue == NULL)
    {
        dObj->queue = bufferObj;    
    }
    else
    {
        if (dObj->queue->previous != NULL)
        {
            dObj->queue->previous->next = bufferObj;
            bufferObj->previous = dObj->queue->previous;
            dObj->queue->previous = bufferObj;
        }
        else
        {
            dObj->queue->previous = bufferObj;
            dObj->queue->next = bufferObj;
            bufferObj->previous = dObj->queue;
        }
    }
}

/* This function updates the head of the queue. */
static void DRV_SST25_UpdateQueue
(
    DRV_SST25_OBJ *dObj
)
{
    DRV_SST25_BUFFER_OBJ * bufferObj = dObj->queue;

    if (dObj->queue != NULL)
    {
        bufferObj->inUse = false;
        if (dObj->queue->next != NULL)
        {
            dObj->queue = dObj->queue->next;
            if (dObj->queue == bufferObj->previous)
            {
                dObj->queue->previous = NULL;
            }
            else
            {
                dObj->queue->previous = bufferObj->previous;
            }
            bufferObj->previous = NULL;
        }
        else
        {
            dObj->queue->previous = NULL;
            dObj->queue = NULL;
        }
    }

    return;
}

static void DRV_SST25_RemoveClientBufferObjects
(
    DRV_SST25_CLIENT_OBJ *clientObj,
    DRV_SST25_OBJ *dObj
)
{
    DRV_SST25_BUFFER_OBJ *bufferObject = NULL;
    DRV_SST25_BUFFER_OBJ *lastObject = NULL;
    DRV_SST25_BUFFER_OBJ *head = NULL;
    DRV_SST25_BUFFER_OBJ *temp = NULL;

    bufferObject = dObj->queue;

    if (dObj->queue != NULL)
    {
        dObj->queue->previous = NULL;
    }

    while (bufferObject != NULL)
    {
        temp = bufferObject->next;
        if (bufferObject->hClient == clientObj)
        {
            bufferObject->inUse = false;

            if(bufferObject->previous != NULL)
            {
                bufferObject->previous->next = bufferObject->next;
            }

            if(bufferObject->next != NULL)
            {
                bufferObject->next->previous = bufferObject->previous;
            }

            bufferObject->previous = NULL;
            bufferObject->next = NULL;
        }
        else
        {
            if (head == NULL)
            {
                head = bufferObject;
            }

            lastObject = bufferObject;
        }

        bufferObject = temp;
    }

    dObj->queue = head;

    if ((head != NULL) && (head != lastObject))
    {
        dObj->queue->previous = lastObject;
    }
}

/* This function validates the driver handle and returns the client object
    pointer associated with the driver handle if the handle is valid. If the
    driver handle is not valid or if the driver is in a not ready state then
    NULL is returned.
*/
static DRV_SST25_CLIENT_OBJ * _DRV_SST25_ValidateDriverHandle
(
    DRV_HANDLE handle
)
{
    DRV_SST25_CLIENT_OBJ *clientObj = NULL;
    DRV_SST25_OBJ *dObj = NULL;

    if ((handle == DRV_HANDLE_INVALID) || (handle == 0))
    {
        return NULL;
    }

    clientObj = (DRV_SST25_CLIENT_OBJ *)handle;
    if (clientObj->inUse == false)
    {
        return NULL;
    }

    /* Check if the driver is ready for operation */
    dObj = (DRV_SST25_OBJ *)clientObj->driverObj;
    if (dObj->status != SYS_STATUS_READY)
    {
        return NULL;
    }

    return clientObj;
}

/* This function finds and allocates a client object. */
static DRV_SST25_CLIENT_OBJ* DRV_SST25_AllocateClientObject
(
    void
)
{
    uint8_t count = 0;
    DRV_SST25_CLIENT_OBJ *clientObj = &gDrvSst25ClientObj[0];

    for (count = 0; count < DRV_SST25_CLIENTS_NUMBER; count++)
    {
        if (!clientObj->inUse)
        {
            return clientObj;
        }

        clientObj++;
    }

    return NULL;
}

static void DRV_SST25_SetupHardware
(
    DRV_SST25_INIT *sst25Init,
    DRV_SST25_OBJ *dObj
)
{
    if(_DRV_SST25_IsHoldEnabled())
    {
        /* If hardware HOLD is enabled, then user must provide a port pin
           in the initilization structure corresponding to HOLD pin on the
           flash */

        /* Make the HOLD pin direction as output */
        SYS_PORTS_PinDirectionSelect (PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, sst25Init->holdPort, sst25Init->holdPin);

        /* Set the HOLD high. */
        SYS_PORTS_PinSet (PORTS_ID_0, sst25Init->holdPort, sst25Init->holdPin);
    }

    if(_DRV_SST25_IsWriteProtectEnabled())
    {
        /* If hardware write protection is enabled, then user must
           provide a port pin in the initilization structure corresponding to
           WP pin on the flash */

        /* Make the WP pin direction as output */
        SYS_PORTS_PinDirectionSelect (PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, sst25Init->wpPort, sst25Init->wpPin);

        /* Keep the WP pin HIGH so that status register will be writeble always */
        SYS_PORTS_PinSet (PORTS_ID_0, sst25Init->wpPort, sst25Init->wpPin);
    }

    /* Make the chip slect pin direction as output */
    SYS_PORTS_PinDirectionSelect (PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, dObj->csPort, dObj->csPin);

    /* Configure the CS to HIGH initially. */
    SYS_PORTS_PinSet (PORTS_ID_0, dObj->csPort, dObj->csPin);
}


//******************************************************************************
/* Function:

   void _DRV_SST25_SpiBufferEventHandler 
   (
       DRV_SPI_BUFFER_EVENT event,
       DRV_SPI_BUFFER_HANDLE bufferHandle, 
       void * context
   )

  Summary:
    SPI Buffer Event Handler

  Description:
    SPI Buffer event handler. This handler controls the selection and 
    de-selection of the Chip Select line of the SD Card.

  Remarks:
    None
*/

void _DRV_SST25_SpiBufferEventHandler
(
    DRV_SPI_BUFFER_EVENT event,
    DRV_SPI_BUFFER_HANDLE bufferHandle, 
    void * context
)
{
    DRV_SST25_OBJ *dObj;
    
    if (context == NULL)
    {
        return;
    }

    dObj = (DRV_SST25_OBJ *)context;

    if (bufferHandle != dObj->spiBufferHandle)
    {
        return;
    }

    switch (event)
    {
        case DRV_SPI_BUFFER_EVENT_PROCESSING:
            {
                /* Select the chip */
                _DRV_SST25_CHIP_SELECT (dObj->csPort, dObj->csPin);
                break;
            }

        case DRV_SPI_BUFFER_EVENT_COMPLETE:
            {
                if (dObj->disableCs == true)
                {
                    /* De select the chip */
                    _DRV_SST25_CHIP_DESELECT (dObj->csPort, dObj->csPin);
                }

                break;
            }

        case DRV_SPI_BUFFER_EVENT_ERROR:
        default:
            {
                /* De select the chip */
                _DRV_SST25_CHIP_DESELECT (dObj->csPort, dObj->csPin);
                break;
            }
    }
}

static DRV_SPI_BUFFER_EVENT DRV_SST25_WriteEnable
(
    DRV_SST25_OBJ *dObj
)
{
    DRV_SPI_BUFFER_EVENT event = DRV_SPI_BUFFER_EVENT_ERROR;

    switch (dObj->subState)
    {
        case DRV_SST25_WRITE_ENABLE:
        default:
            {
                dObj->cmdParams[0] = dObj->opCodes.writeEnable;
                dObj->disableCs = true;
                DRV_SPI_BufferAddWrite2 (dObj->spiDriverHandle, &dObj->cmdParams[0], 1, 0, dObj, &dObj->spiBufferHandle);
                if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    dObj->subState = DRV_SST25_WRITE_ENABLE_STATUS;
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                else
                {
                    /* Failed to queue the write transfer request. */
                }

                break;
            }

        case DRV_SST25_WRITE_ENABLE_STATUS:
            {
                event = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                break;
            }
    }

    return event;
}

static DRV_SPI_BUFFER_EVENT DRV_SST25_ReadStatus
(
    DRV_SST25_OBJ *dObj
)
{
    DRV_SPI_BUFFER_EVENT event = DRV_SPI_BUFFER_EVENT_PENDING;

    switch (dObj->subState)
    {
        case DRV_SST25_STATUS_REG_WRITE_CMD:
        default:
            {
                dObj->cmdParams[0] = dObj->opCodes.readStatus;
                dObj->disableCs = false;
                DRV_SPI_BufferAddWrite2 (dObj->spiDriverHandle, &dObj->cmdParams[0], 1, 0, dObj, &dObj->spiBufferHandle);
                if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    dObj->subState = DRV_SST25_STATUS_REG_CHECK_WRITE_STATUS;
                }
                else
                {
                    event = DRV_SPI_BUFFER_EVENT_ERROR;
                }
                break;
            }

        case DRV_SST25_STATUS_REG_CHECK_WRITE_STATUS:
            {
                event = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    dObj->disableCs = true;
                    DRV_SPI_BufferAddRead2 (dObj->spiDriverHandle, &dObj->statusReg[0], 1, 0, dObj, &dObj->spiBufferHandle);
                    if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                    {
                        dObj->subState = DRV_SST25_STATUS_REG_CHECK_READ_STATUS;
                        event = DRV_SPI_BUFFER_EVENT_PENDING;
                    }
                    else
                    {
                        event = DRV_SPI_BUFFER_EVENT_ERROR;
                    }
                }
                break;
            }

        case DRV_SST25_STATUS_REG_CHECK_READ_STATUS:
            {
                event = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                break;
            }
    }

    return event;
}

static DRV_SPI_BUFFER_EVENT DRV_SST25_ReadFlashId
(
    DRV_SST25_OBJ *dObj
)
{
    DRV_SPI_BUFFER_EVENT event = DRV_SPI_BUFFER_EVENT_ERROR;

    switch (dObj->subState)
    {
        case DRV_SST25_READ_ID:
        default:
            {
                uint8_t *cmdParams = &dObj->cmdParams[0];
                cmdParams[0] = DRV_SST25_CMD_READ_ID;
                cmdParams[1] = 0x00;
                cmdParams[2] = 0x00;
                cmdParams[3] = 0x00;

                dObj->disableCs = false;
                DRV_SPI_BufferAddWrite2 (dObj->spiDriverHandle, &dObj->cmdParams[0], 4, 0, dObj, &dObj->spiBufferHandle);
                if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    dObj->subState = DRV_SST25_READ_ID_STATUS;
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                else
                {
                    /* Failed to queue the write transfer request. */
                }

                break;
            }

        case DRV_SST25_READ_ID_STATUS:
            {
                event = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    dObj->disableCs = true;
                    DRV_SPI_BufferAddRead2 (dObj->spiDriverHandle, &dObj->flashId[0], 2, 0, dObj, &dObj->spiBufferHandle);
                    if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                    {
                        dObj->subState = DRV_SST25_READ_ID_DATA_STATUS;
                        event = DRV_SPI_BUFFER_EVENT_PENDING;
                    }
                    else
                    {
                        /* Failed to queue the read transfer. */
                        event = DRV_SPI_BUFFER_EVENT_ERROR;
                    }
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* The caller will handle the error. */
                }
                else
                {
                    /* Continue to be in the same state. */
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }

                break;
            }

        case DRV_SST25_READ_ID_DATA_STATUS:
            {
                event = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                break;
            }
    }

    return event;
}

static DRV_SPI_BUFFER_EVENT DRV_SST25_UnlockFlash
(
    void *driverObj
)
{
    DRV_SPI_BUFFER_EVENT event = DRV_SPI_BUFFER_EVENT_ERROR;
    DRV_SST25_OBJ *dObj = (DRV_SST25_OBJ *)driverObj;

    switch (dObj->subState)
    {
        case DRV_SST25_UNLOCK_ENABLE_WRSR_REG:
        default:
            {
                dObj->cmdParams[0] = dObj->opCodes.enableWriteStatus;
                dObj->disableCs = true;
                DRV_SPI_BufferAddWrite2(dObj->spiDriverHandle, &dObj->cmdParams[0], 1, 0, dObj, &dObj->spiBufferHandle);
                if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    dObj->subState = DRV_SST25_UNLOCK_WRSR_CMD;
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                else
                {
                    /* Failed to queue the write transfer request. */
                }
                break;
            }

        case DRV_SST25_UNLOCK_WRSR_CMD:
            {
                event = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    dObj->cmdParams[0] = dObj->opCodes.writeStatus;
                    /* Reset BP0 and BP1 */
                    dObj->cmdParams[1] = 0x0;

                    dObj->disableCs = true;
                    DRV_SPI_BufferAddWrite2(dObj->spiDriverHandle, &dObj->cmdParams[0], 2, 0, dObj, &dObj->spiBufferHandle);
                    if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                    {
                        dObj->subState = DRV_SST25_UNLOCK_WRSR_CMD_STATUS;
                        event = DRV_SPI_BUFFER_EVENT_PENDING;
                    }
                    else
                    {
                        /* Failed to queue the write transfer request. */
                        event = DRV_SPI_BUFFER_EVENT_ERROR;
                    }
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* Error will be handled in the caller function. */
                }
                else
                {
                    /* Continue to remain in the same state. */
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }

                break;
            }

        case DRV_SST25_UNLOCK_WRSR_CMD_STATUS:
            {
                event = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                break;
            }
    }

    return event;
}

/* This function is responsible for handling the read operation. */
static DRV_SPI_BUFFER_EVENT DRV_SST25_HandleRead
(
    void *driverObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
)
{
    DRV_SST25_OBJ *dObj = (DRV_SST25_OBJ *)driverObj;
    DRV_SPI_BUFFER_EVENT event = DRV_SPI_BUFFER_EVENT_ERROR;

    switch (dObj->subState)
    {
        case DRV_SST25_READ_INIT:
        default:
            {
                uint8_t *cmdParams = dObj->cmdParams;

                dObj->data = data;
                dObj->nBlocks = nBlocks;

                cmdParams[0] = dObj->opCodes.read;
                cmdParams[1] = (blockAddress >> 16) & 0xFF;
                cmdParams[2] = (blockAddress >> 8) & 0xFF;
                cmdParams[3] = blockAddress & 0xFF;

                /* Chip Select should be deselected only after having read the
                 * required number of blocks of data. */
                dObj->disableCs = false;
                DRV_SPI_BufferAddWrite2 (dObj->spiDriverHandle, &dObj->cmdParams[0], 4, 0, dObj, &dObj->spiBufferHandle);
                if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    dObj->subState = DRV_SST25_READ_TRANSFER_DATA;
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                else
                {
                    /* Failed to queue the write transfer request. */
                }

                break;
            }

        case DRV_SST25_READ_TRANSFER_DATA:
            {
                event = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* After the required number of blocks of data have been
                     * read, deselect the CS line. */
                    dObj->disableCs = true;
                    DRV_SPI_BufferAddRead2(dObj->spiDriverHandle, dObj->data, dObj->nBlocks, 0, dObj, &dObj->spiBufferHandle);
                    if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                    {
                        dObj->subState = DRV_SST25_READ_TRANSFER_DATA_STATUS;
                        event = DRV_SPI_BUFFER_EVENT_PENDING;
                    }
                    else
                    {
                        event = DRV_SPI_BUFFER_EVENT_ERROR;
                    }
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* Do nothing. Error is handled in the caller function. */
                }
                else
                {
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }

                break;
            }

        case DRV_SST25_READ_TRANSFER_DATA_STATUS:
            {
                event = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                break;
            }
    }

    return event;
}

/* This variant of the write function uses the Page Program command to write to
 * the SPI Flash. */
static DRV_SPI_BUFFER_EVENT DRV_SST25_WritePageProgram
(
    void *driverObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
)
{
    uint16_t mediaBlockSize = 0;
    DRV_SPI_BUFFER_EVENT event = DRV_SPI_BUFFER_EVENT_PENDING;
    DRV_SST25_OBJ *dObj = (DRV_SST25_OBJ *)driverObj;
    uint8_t *cmdParams = &dObj->cmdParams[0];

    switch (dObj->writeState)
    {
        case DRV_SST25_WRITE_INIT:
        default:
            {
                dObj->data = data;
                dObj->nBlocks = nBlocks;
                /* Convert from block to byte address. */
                dObj->blockAddress = blockAddress * dObj->mediaGeometryTable[1].blockSize;
                dObj->subState = DRV_SST25_WRITE_ENABLE;
                dObj->writeState = DRV_SST25_WRITE_ENABLE_WRITE;
                /* Fall through. */
            }

        case DRV_SST25_WRITE_ENABLE_WRITE:
            {
                event = DRV_SST25_WriteEnable (dObj);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    mediaBlockSize = dObj->mediaGeometryTable[1].blockSize;
                    cmdParams[0] = dObj->opCodes.write;
                    cmdParams[1] = (dObj->blockAddress >> 16) & 0xFF;
                    cmdParams[2] = (dObj->blockAddress >> 8) & 0xFF;
                    cmdParams[3] = dObj->blockAddress & 0xFF;
                    dObj->cmdParamsLen = 4 + mediaBlockSize;

                    if (mediaBlockSize != 1)
                    {
                        memcpy ((void *)&dObj->cmdParams[4], (const void *)dObj->data, mediaBlockSize);
                    }
                    else
                    {
                        cmdParams[4] = *dObj->data;
                    }

                    dObj->data += mediaBlockSize;
                    dObj->blockAddress += mediaBlockSize;
                    dObj->nBlocks --;

                    dObj->writeState = DRV_SST25_WRITE_PROGRAM_DATA;
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* Do nothing. The caller function will handle the error. */
                }
                else
                {
                    /* Continue to remain in the same state. */
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                break;
            }

        case DRV_SST25_WRITE_PROGRAM_DATA:
            {
                dObj->disableCs = true;
                DRV_SPI_BufferAddWrite2 (dObj->spiDriverHandle, &dObj->cmdParams[0], dObj->cmdParamsLen, 0, dObj, &dObj->spiBufferHandle);
                if (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    event = DRV_SPI_BUFFER_EVENT_ERROR;
                    break;
                }
                else
                {
                    dObj->writeState = DRV_SST25_WRITE_PROGRAM_DATA_STATUS;
                    /* Fall through */
                }
            }

        case DRV_SST25_WRITE_PROGRAM_DATA_STATUS:
            {
                event = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* Read the status register. */
                    dObj->writeState = DRV_SST25_WRITE_READ_STATUS_REGISTER;
                    dObj->subState = DRV_SST25_STATUS_REG_WRITE_CMD;
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* Do nothing. The caller function will handle the error. */
                }
                else
                {
                    /* Continue to remain in the same state. */
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                break;
            }

        case DRV_SST25_WRITE_READ_STATUS_REGISTER:
            {
                event = DRV_SST25_ReadStatus (dObj);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    if (!(*dObj->statusReg & 0x01))
                    {
                        if (dObj->nBlocks != 0)
                        {
                            dObj->subState = DRV_SST25_WRITE_ENABLE;
                            dObj->writeState = DRV_SST25_WRITE_ENABLE_WRITE;
                            event = DRV_SPI_BUFFER_EVENT_PENDING;
                        }
                        else
                        {
                            /* Operation completed. */
                        }
                    }
                    else
                    {
                        /* Flash is busy with internal write operation. Poll
                         * until the busy bit is cleared. */
                        dObj->subState = DRV_SST25_STATUS_REG_WRITE_CMD;
                        event = DRV_SPI_BUFFER_EVENT_PENDING;
                    }
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* Do nothing. The caller function will handle the error. */
                }
                else
                {
                    /* Continue to remain in the same state. */
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                break;
            }
    }

    return event;
}

/* This variant of the write function uses the Auto Address Increment feature
 * to write to the SPI Flash. */
static DRV_SPI_BUFFER_EVENT DRV_SST25_WriteAutoAddressIncrement
(
    void *driverObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
)
{
    DRV_SPI_BUFFER_EVENT event = DRV_SPI_BUFFER_EVENT_PENDING;
    DRV_SST25_OBJ *dObj = (DRV_SST25_OBJ *)driverObj;
    uint8_t *cmdParams = &dObj->cmdParams[0];

    switch (dObj->writeState)
    {
        case DRV_SST25_WRITE_INIT:
        default:
            {
                dObj->data = data;
                dObj->nBlocks = nBlocks;
                dObj->blockAddress = blockAddress * dObj->mediaGeometryTable[1].blockSize;

                dObj->subState = DRV_SST25_WRITE_ENABLE;
                dObj->writeState = DRV_SST25_WRITE_ENABLE_WRITE;
                /* Fall through. */
            }

        case DRV_SST25_WRITE_ENABLE_WRITE:
            {
                event = DRV_SST25_WriteEnable (dObj);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    cmdParams[0] = dObj->opCodes.write;

                    /* Transition to the next state. */
                    cmdParams[1] = (dObj->blockAddress >> 16) & 0xFF;
                    cmdParams[2] = (dObj->blockAddress >> 8) & 0xFF;
                    cmdParams[3] = dObj->blockAddress & 0xFF;
                    cmdParams[4] = *dObj->data;
                    cmdParams[5] = *(dObj->data + 1);
                    dObj->cmdParamsLen = 6;

                    dObj->data += 2;
                    dObj->nBlocks -= 1;

                    dObj->writeState = DRV_SST25_WRITE_PROGRAM_DATA;
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* Do nothing. The caller function will handle the error. */
                }
                else
                {
                    /* Continue to remain in the same state. */
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                break;
            }

        case DRV_SST25_WRITE_PROGRAM_DATA:
            {
                dObj->disableCs = true;
                DRV_SPI_BufferAddWrite2 (dObj->spiDriverHandle, &dObj->cmdParams[0], dObj->cmdParamsLen, 0, dObj, &dObj->spiBufferHandle);
                if (dObj->spiBufferHandle == DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    event = DRV_SPI_BUFFER_EVENT_ERROR;
                    break;
                }
                else
                {
                    dObj->writeState = DRV_SST25_WRITE_PROGRAM_DATA_STATUS;
                    /* Fall through */
                }
            }

        case DRV_SST25_WRITE_PROGRAM_DATA_STATUS:
            {
                event = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* Read the status register. */
                    dObj->writeState = DRV_SST25_WRITE_READ_STATUS_REGISTER;
                    dObj->subState = DRV_SST25_STATUS_REG_WRITE_CMD;
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* Do nothing. The caller function will handle the error. */
                }
                else
                {
                    /* Continue to remain in the same state. */
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                break;
            }

        case DRV_SST25_WRITE_READ_STATUS_REGISTER:
            {
                event = DRV_SST25_ReadStatus (dObj);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    if (!(*dObj->statusReg & 0x01))
                    {
                        if (dObj->nBlocks == 0)
                        {
                            /* Disable the AAI Operation using the WRDI command. */
                            dObj->writeState = DRV_SST25_WRITE_DISABLE_WRITE;
                            event = DRV_SPI_BUFFER_EVENT_PENDING;
                        }
                        else
                        {
                            cmdParams[0] = dObj->opCodes.write;
                            cmdParams[1] = *dObj->data;
                            cmdParams[2] = *(dObj->data + 1);
                            dObj->cmdParamsLen = 3;

                            dObj->data += 2;
                            dObj->nBlocks -= 1;

                            dObj->writeState = DRV_SST25_WRITE_PROGRAM_DATA;

                            event = DRV_SPI_BUFFER_EVENT_PENDING;
                        }
                    }
                    else
                    {
                        /* Flash is busy with internal write operation. Poll
                         * until the busy bit is cleared. */
                        dObj->subState = DRV_SST25_STATUS_REG_WRITE_CMD;
                        event = DRV_SPI_BUFFER_EVENT_PENDING;
                    }
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* Do nothing. The caller function will handle the error. */
                }
                else
                {
                    /* Continue to remain in the same state. */
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                break;
            }

        case DRV_SST25_WRITE_DISABLE_WRITE:
            {
                dObj->cmdParams[0] = dObj->opCodes.writeDisable;
                dObj->disableCs = true;
                DRV_SPI_BufferAddWrite2 (dObj->spiDriverHandle, &dObj->cmdParams[0], 1, 0, dObj, &dObj->spiBufferHandle);
                if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    dObj->writeState = DRV_SST25_WRITE_DISABLE_WRITE_STATUS;
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                else
                {
                    event = DRV_SPI_BUFFER_EVENT_ERROR;
                }
                break;
            }

        case DRV_SST25_WRITE_DISABLE_WRITE_STATUS:
            {
                event = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* Operation complete. */
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* Do nothing. The caller function will handle the error. */
                }
                else
                {
                    /* Continue to remain in the same state. */
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                break;
            }
    }

    return event;
}

static DRV_SPI_BUFFER_EVENT DRV_SST25_HandleErase
(
    void *driverObj,
    uint32_t blockAddress,
    uint32_t nBlocks
)
{
    DRV_SPI_BUFFER_EVENT event = DRV_SPI_BUFFER_EVENT_ERROR;
    DRV_SST25_OBJ *dObj = (DRV_SST25_OBJ *)driverObj;

    switch (dObj->eraseState)
    {
        case DRV_SST25_ERASE_INIT:
        default:
            {
                dObj->nBlocks = nBlocks;
                dObj->blockAddress = blockAddress;

                dObj->subState = DRV_SST25_WRITE_ENABLE;
                dObj->eraseState = DRV_SST25_ERASE_ENABLE_WRITE;
                /* Fall through. */
            }

        case DRV_SST25_ERASE_ENABLE_WRITE:
            {
                event = DRV_SST25_WriteEnable (dObj);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    if ((dObj->nBlocks == 0) && (dObj->blockAddress == 0))
                    {
                        dObj->cmdParams[0] = dObj->opCodes.chipErase;
                        dObj->cmdParamsLen = 1;
                    }
                    else
                    {
                        dObj->cmdParams[0] = dObj->opCodes.erase;
                        dObj->cmdParams[1] = (dObj->blockAddress >> 16) & 0xFF;
                        dObj->cmdParams[2] = (dObj->blockAddress >> 8) & 0xFF;
                        dObj->cmdParams[3] = dObj->blockAddress & 0xFF;
                        dObj->cmdParamsLen = 4;

                        dObj->nBlocks --;
                    }

                    dObj->eraseState = DRV_SST25_ERASE_ERASE_CMD;
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* Do nothing. The caller function will handle the error. */
                }
                else
                {
                    /* Continue to remain in the same state. */
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                break;
            }

        case DRV_SST25_ERASE_ERASE_CMD:
            {
                dObj->disableCs = true;
                DRV_SPI_BufferAddWrite2 (dObj->spiDriverHandle, &dObj->cmdParams[0], dObj->cmdParamsLen, 0, dObj, &dObj->spiBufferHandle);
                if (dObj->spiBufferHandle != DRV_SPI_BUFFER_HANDLE_INVALID)
                {
                    dObj->eraseState = DRV_SST25_ERASE_ERASE_CMD_STATUS;
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                else
                {
                    event = DRV_SPI_BUFFER_EVENT_ERROR;
                }

                break;
            }

        case DRV_SST25_ERASE_ERASE_CMD_STATUS:
            {
                event = DRV_SPI_BufferStatus(dObj->spiBufferHandle);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* Transition to the next state. */
                    dObj->eraseState = DRV_SST25_ERASE_READ_STATUS_REGISTER;
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* Do nothing. The caller function will handle the error. */
                }
                else
                {
                    /* Continue to remain in the same state. */
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }

                break;
            }

        case DRV_SST25_ERASE_READ_STATUS_REGISTER:
            {
                event = DRV_SST25_ReadStatus (dObj);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    if (!(*dObj->statusReg & 0x01))
                    {
                        if (dObj->nBlocks != 0)
                        {
                            /* Increment the block address. */
                            dObj->blockAddress += DRV_SST25_ERASE_SECTOR_SIZE;

                            /* There is still blocks to be erased. */
                            event = DRV_SPI_BUFFER_EVENT_PENDING;
                            dObj->subState = DRV_SST25_WRITE_ENABLE;
                            dObj->eraseState = DRV_SST25_ERASE_ENABLE_WRITE;
                        }
                    }
                    else
                    {
                        /* Flash is busy with internal write operation. Poll
                         * until the busy bit is cleared. */
                        dObj->subState = DRV_SST25_STATUS_REG_WRITE_CMD;
                        event = DRV_SPI_BUFFER_EVENT_PENDING;
                    }
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* Do nothing. The caller function will handle the error. */
                }
                else
                {
                    /* Continue to remain in the same state. */
                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                break;
            }
    }

    return event;
}

static DRV_SPI_BUFFER_EVENT DRV_SST25_HandleChipErase
(
    void *dObj
)
{
    return DRV_SST25_HandleErase (dObj, 0, 0);
}

static DRV_SPI_BUFFER_EVENT DRV_SST25_HandleEraseWrite
(
    void *driverObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
)
{
    DRV_SST25_OBJ *dObj = (DRV_SST25_OBJ *)driverObj;
    DRV_SST25_BUFFER_OBJ *bufferObj = dObj->currentBufObj;

    DRV_SPI_BUFFER_EVENT event = DRV_SPI_BUFFER_EVENT_ERROR;
    uint32_t temp1 = 0;
    uint32_t temp2 = 0;

    switch (dObj->ewState)
    {
        case DRV_SST25_EW_INIT:
        default:
            {
                dObj->eraseState = DRV_SST25_ERASE_INIT;
                dObj->writeState = DRV_SST25_WRITE_INIT;

                temp1 = dObj->mediaGeometryTable[1].blockSize;
                temp2 = dObj->mediaGeometryTable[2].blockSize;
                /* Find the start sector address */
                dObj->sectorAddress = (bufferObj->blockStart * temp1)/temp2;

                /* Find the number of pages to be updated in this sector. */
                dObj->blockOffsetInSector = (bufferObj->blockStart * temp1) % temp2;
                dObj->nBlocksToWrite = (temp2 - dObj->blockOffsetInSector) / temp1;

                if (bufferObj->nBlocks < dObj->nBlocksToWrite)
                {
                    dObj->nBlocksToWrite = bufferObj->nBlocks;
                }

                if (dObj->nBlocksToWrite != (temp2/temp1))
                {
                    dObj->data = dObj->ewBuffer;

                    dObj->subState = DRV_SST25_READ_INIT;
                    dObj->ewState = DRV_SST25_EW_READ_SECTOR;
                }
                else
                {
                    dObj->data = bufferObj->buffer;
                    dObj->ewState = DRV_SST25_EW_ERASE_SECTOR;

                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                    break;
                }

                /* Fall through for read operation. */
            }

        case DRV_SST25_EW_READ_SECTOR:
            {
                event = dObj->flashFunctions.read (dObj, dObj->ewBuffer, dObj->sectorAddress << 12, dObj->mediaGeometryTable[2].blockSize);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* Find the offset from which the data is to be overlaid. */
                    memcpy ((void *)&dObj->ewBuffer[dObj->blockOffsetInSector], (const void *)bufferObj->buffer, dObj->nBlocksToWrite * dObj->mediaGeometryTable[1].blockSize);

                    dObj->eraseState = DRV_SST25_ERASE_INIT;
                    dObj->ewState = DRV_SST25_EW_ERASE_SECTOR;

                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                break;
            }

        case DRV_SST25_EW_ERASE_SECTOR:
            {
                event = dObj->flashFunctions.erase (dObj, dObj->sectorAddress << 12, 1);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    dObj->writeState = DRV_SST25_WRITE_INIT;
                    dObj->ewState = DRV_SST25_EW_WRITE_SECTOR;

                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }
                break;
            }

        case DRV_SST25_EW_WRITE_SECTOR:
            {
                temp1 = dObj->mediaGeometryTable[2].blockSize / dObj->mediaGeometryTable[1].blockSize;
                temp2 = (dObj->sectorAddress << 12) / dObj->mediaGeometryTable[1].blockSize;
                event = dObj->flashFunctions.write (dObj, dObj->data, temp2, temp1);

                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    if ((bufferObj->nBlocks - dObj->nBlocksToWrite) == 0)
                    {
                        /* This is the last write operation. */
                        break;
                    }

                    /* Update the number of block still to be written, sector address
                     * and the buffer pointer */
                    bufferObj->nBlocks -= dObj->nBlocksToWrite;
                    bufferObj->blockStart += dObj->nBlocksToWrite;
                    bufferObj->buffer += dObj->nBlocksToWrite;
                    dObj->ewState = DRV_SST25_EW_INIT;

                    event = DRV_SPI_BUFFER_EVENT_PENDING;
                }

                break;
            }
    }

    return event;
}

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SST25_Initialize
    (
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init
    );

  Summary:
    Dynamic impementation of DRV_SST25_Initialize system interface function.

  Description:
    This is the dynamic impementation of DRV_SST25_Initialize system
    interface function.
  
  Remarks:
    See drv_sst25.h for usage information.
*/

SYS_MODULE_OBJ DRV_SST25_Initialize
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT *const init
)
{
    DRV_SST25_OBJ *dObj = NULL;
    DRV_SST25_INIT *sst25Init = NULL ;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;
    DRV_SST25_FLASH_OPCODES *opCodes = NULL;

    if (drvIndex >= DRV_SST25_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "DRV_SST25: Invalid driver index \n");
        return SYS_MODULE_OBJ_INVALID;
    }

    dObj = &gDrvSst25Obj[drvIndex];
    if (dObj->inUse == true)
    {
        SYS_DEBUG(0, "DRV_SST25: Instance already in use \n");
        return SYS_MODULE_OBJ_INVALID;
    }
    
    sst25Init = (DRV_SST25_INIT *)init;

    /* Initialize the driver object members. */
    dObj->inUse = true;
    dObj->status = SYS_STATUS_BUSY;
    dObj->numClients = 0;
    dObj->isExclusive = false;
    dObj->state = DRV_SST25_TASK_OPEN_SPI_DRIVER;

    dObj->data = NULL;
    dObj->ewBuffer = &gDrvSst25SectorBuffer[drvIndex][0];
    dObj->cmdParams = &gDrvSst25CmdParams[drvIndex][0];
    dObj->nBlocks = 0;
    dObj->blockAddress = 0;
    dObj->sectorAddress = 0;
    dObj->blockOffsetInSector = 0;
    dObj->nBlocksToWrite = 0;

    dObj->csPort = sst25Init->csPort;
    dObj->csPin = sst25Init->csPin;
    dObj->spiDriverIndex = sst25Init->spiDriverIndex;

    dObj->statusReg = &gDrvSst25StatusReg;
    dObj->flashId = &gDrvSst25FlashId[2];

    /* Initialize the SPI Client Data Structure */
    dObj->spiClientData.baudRate = 0;
    dObj->spiClientData.operationStarting = _DRV_SST25_SpiBufferEventHandler;
    dObj->spiClientData.operationEnded = _DRV_SST25_SpiBufferEventHandler;

    DRV_SST25_SetupHardware (sst25Init, dObj);

    /* Initialize the default opcodes for the flash. */
    opCodes = &dObj->opCodes;
    opCodes->read = DRV_SST25_CMD_READ;
    opCodes->write = DRV_SST25_CMD_PAGE_PROGRAM;
    opCodes->erase = DRV_SST25_CMD_BLOCK_ERASE;
    opCodes->chipErase = DRV_SST25_CMD_CHIP_ERASE;

    opCodes->writeEnable = DRV_SST25_CMD_WRITE_ENABLE;
    opCodes->writeDisable = DRV_SST25_CMD_WRITE_DISABLE;

    opCodes->readStatus = DRV_SST25_CMD_READ_STATUS_REG;
    opCodes->writeStatus = DRV_SST25_CMD_WRITE_STATUS_REG;
    opCodes->enableWriteStatus = DRV_SST25_CMD_ENABLE_WRITE_STATUS_REG;

    /* Default implementation of the SPI Flash functions. */
    dObj->flashFunctions.unlock = DRV_SST25_UnlockFlash;
    dObj->flashFunctions.read = DRV_SST25_HandleRead;
    dObj->flashFunctions.write = DRV_SST25_WritePageProgram;
    dObj->flashFunctions.erase = DRV_SST25_HandleErase;
    dObj->flashFunctions.chipErase = DRV_SST25_HandleChipErase;
    dObj->flashFunctions.eraseWrite = DRV_SST25_HandleEraseWrite;

    if (gDrvSst25InitCount == 0)
    {
        retVal = OSAL_MUTEX_Create(&gSst25ClientObjMutex);
        if (retVal != OSAL_RESULT_TRUE)
        {
            return SYS_MODULE_OBJ_INVALID;
        }

        gDrvSst25InitCount ++;
    }

    if (OSAL_MUTEX_Create(&dObj->mutex) != OSAL_RESULT_TRUE)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    DRV_SST25_RegisterWithSysFs (drvIndex, drvIndex, sst25MediaFunctions);
    
    return drvIndex;
}

//******************************************************************************
/* Function:
    void DRV_SST25_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the SPI Flash driver module

  Description:
    Deinitializes the specified instance of the SPI Flash driver module,
    disabling its operation (and any hardware). Invalidates all the
    internal data.

  Remarks:
    See drv_sst25.h for usage information.
*/

void DRV_SST25_Deinitialize
(
    SYS_MODULE_OBJ object
)
{
    DRV_SST25_OBJ *dObj = NULL;

    if(object == SYS_MODULE_OBJ_INVALID)
    {
        SYS_DEBUG(0, "DRV_SST25: Invalid system module object \n" );
        return;
    }
    
    if(object >= DRV_SST25_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "DRV_SST25: Invalid system module object \n" );
        return;
    }

    dObj = (DRV_SST25_OBJ*)&gDrvSst25Obj[object];
    if (dObj->inUse == false)
    {
        SYS_DEBUG(0, "DRV_SST25: Invalid system module object \n");
        return;
    }

    dObj->numClients = 0;
    dObj->isExclusive = false;
    dObj->queue = NULL;
    dObj->status = SYS_STATUS_UNINITIALIZED;
    dObj->state = DRV_SST25_TASK_IDLE;
    dObj->inUse = false;

    /* Close the spi driver */
    DRV_SPI_Close (dObj->spiDriverHandle);

    gDrvSst25InitCount --;
    if (gDrvSst25InitCount == 0)
    {
        OSAL_MUTEX_Delete(&gSst25ClientObjMutex);
    }

    OSAL_MUTEX_Delete(&dObj->mutex);
}

//*************************************************************************
/* Function:
    SYS_STATUS DRV_SST25_Status( SYS_MODULE_OBJ object )

  Summary:
    Gets the current status of the SPI Flash driver module.

  Description:
    This routine provides the current status of the SPI Flash driver module.
  
  Remarks:
    See drv_sst25.h for usage information.
*/

SYS_STATUS DRV_SST25_Status( SYS_MODULE_OBJ object)
{
    /* Check if we have a valid object */
    if(object == SYS_MODULE_OBJ_INVALID)
    {
        SYS_DEBUG(0, "DRV_SST25: Invalid system object handle \n");
        return(SYS_STATUS_UNINITIALIZED);
    }
    
    if(object > DRV_SST25_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "DRV_SST25: Invalid system object handle \n");
        return(SYS_STATUS_UNINITIALIZED);
    }

    /* Return the system status of the hardware instance object */
    return (gDrvSst25Obj[object].status);
}

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_SST25_Open
    (
        const SYS_MODULE_INDEX drvIndex,
        const DRV_IO_INTENT ioIntent
    )

  Summary:
    Opens the specified SPI Flash driver instance and returns a handle to it

  Description:
    This routine opens the specified SPI Flash driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver.

  Remarks:
    See drv_sst25.h for usage information.
*/

DRV_HANDLE DRV_SST25_Open
( 
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
    DRV_SST25_CLIENT_OBJ *clientObj = NULL;
    DRV_SST25_OBJ *dObj = NULL;

    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    if (drvIndex >= DRV_SST25_INSTANCES_NUMBER)
    {
        /* Invalid driver index */
        SYS_DEBUG(0, "DRV_SST25: Invalid Driver Instance \n");
        return DRV_HANDLE_INVALID;
    }

    dObj = &gDrvSst25Obj[drvIndex];
    if ((dObj->status != SYS_STATUS_READY) || (dObj->inUse == false)) 
    {
        /* The SST25 module should be ready */
        SYS_DEBUG(0, "DRV_SST25: Was the driver initialized? \n");
        return DRV_HANDLE_INVALID;
    }

    if (dObj->isExclusive)
    {
        /* This means the another client has opened the driver in exclusive
           mode. The driver cannot be opened again */
        SYS_DEBUG(0, "DRV_SST25: Driver already opened exclusively \n"); 
        return DRV_HANDLE_INVALID;
    }

    if ((dObj->numClients > 0) && (ioIntent & DRV_IO_INTENT_EXCLUSIVE))
    {
        /* This means the driver was already opened and another driver was 
           trying to open it exclusively.  We cannot give exclusive access in 
           this case */

        SYS_DEBUG(0, "DRV_SST25: Driver already opened. Cannot be opened exclusively \n");
        return DRV_HANDLE_INVALID;
    }

    /* Obtain the Client object mutex */
    retVal = OSAL_MUTEX_Lock(&gSst25ClientObjMutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_Open(): Failed to acquire the client object mutex.\n");
        return DRV_HANDLE_INVALID;
    }

    clientObj = DRV_SST25_AllocateClientObject ();
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_Open(): Failed to allocate a Client Object.\n");
    }
    else
    {
        /* Found a client object that can be used */
        clientObj->inUse = true;
        clientObj->driverObj = dObj;
        clientObj->intent = ioIntent;
        clientObj->eventHandler = NULL;

        clientObj->context      = (uintptr_t)NULL;

        if (ioIntent & DRV_IO_INTENT_EXCLUSIVE)
        {
            /* Driver was opened in exclusive mode */
            dObj->isExclusive = true;
        }

        dObj->numClients ++;
        
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_Open(): Open successful.\n");
    }

    OSAL_MUTEX_Unlock(&gSst25ClientObjMutex);

    return clientObj ? ((DRV_HANDLE)clientObj) : DRV_HANDLE_INVALID;
}

// *****************************************************************************
/* Function:
    void DRV_SST25_Close( DRV_Handle handle );

  Summary:
    Closes an opened-instance of the SPI Flash driver

  Description:
    This routine closes an opened-instance of the SPI Flash driver, invalidating
    the handle.

  Remarks:
    See drv_sst25.h for usage information.
*/

void DRV_SST25_Close
(
    const DRV_HANDLE handle
)
{
    DRV_SST25_CLIENT_OBJ *clientObj = NULL;
    DRV_SST25_OBJ *dObj = NULL;

    clientObj = _DRV_SST25_ValidateDriverHandle(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_SST25_Close(): Invalid handle.\n");
        return;
    }

    dObj = clientObj->driverObj;
    DRV_SST25_RemoveClientBufferObjects (clientObj, dObj);

    /* Update the client count */
    dObj->numClients --;
    dObj->isExclusive = false;

    /* Free the Client Instance */
    clientObj->inUse = false;

    SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_SST25_Close(): Close successful.\n");
    return;
}

void DRV_SST25_BlockEventHandlerSet
(
    const DRV_HANDLE handle,
    const void *eventHandler,
    const uintptr_t context
)
{
    DRV_SST25_CLIENT_OBJ *clientObj = NULL;

    /* Validate the driver handle */
    clientObj = _DRV_SST25_ValidateDriverHandle(handle);
    if(clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockEventHandlerSet: Invalid driver handle.\n");
    }
    else
    {
        clientObj->context = context;
        clientObj->eventHandler = eventHandler;
    }
}

SYS_FS_MEDIA_GEOMETRY * DRV_SST25_GeometryGet( DRV_HANDLE handle )
{
    
    DRV_SST25_CLIENT_OBJ * clientObj;
    DRV_SST25_OBJ *dObj;

    /* Validate the driver handle */
    clientObj = _DRV_SST25_ValidateDriverHandle(handle);

    if(clientObj == NULL)
    {
        /* Driver handle is not valid */

        SYS_DEBUG(0, "DRV_SST25: Invalid driver handle \n");
        return NULL;
    }

    dObj = clientObj->driverObj;
    return &dObj->mediaGeometryObj;
}

bool DRV_SST25_MediaIsAttached
(
    DRV_HANDLE handle
)
{
    DRV_SST25_CLIENT_OBJ *clientObj = NULL;

    clientObj = _DRV_SST25_ValidateDriverHandle(handle);
    if(clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_MediaIsAttached: Invalid driver handle.\n");
        return false;
    }
    
    return true;
}

bool DRV_SST25_MediaIsWriteProtected
(
    const DRV_HANDLE handle
)
{
    /* This function always returns false */
    return false;
}

uintptr_t DRV_SST25_AddressGet
(
    const DRV_HANDLE handle
)
{
    DRV_SST25_CLIENT_OBJ *clientObj = NULL;

    clientObj = _DRV_SST25_ValidateDriverHandle(handle);
    if(clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_AddressGet: Invalid driver handle.\n");
        return (uintptr_t)NULL;
    }
    
    return 0;
}

DRV_SST25_COMMAND_STATUS DRV_SST25_CommandStatus
(
    const DRV_HANDLE hClient,
    const DRV_SST25_BLOCK_COMMAND_HANDLE commandHandle
)
{
    uint16_t iEntry = 0;

    if(_DRV_SST25_ValidateDriverHandle(hClient) == NULL)
    {
        return DRV_SST25_COMMAND_ERROR_UNKNOWN;
    }

    if (commandHandle == DRV_SST25_BLOCK_COMMAND_HANDLE_INVALID)
    {
        return DRV_SST25_COMMAND_ERROR_UNKNOWN;
    }

    /* The upper 16 bits of the buffer handle are the token and the lower 16
     * bits of the are buffer index into the gDrvSST25BufferObject array */
    iEntry = commandHandle & 0xFFFF;

    /* Compare the buffer handle with buffer handle in the object */
    if(gDrvSst25BufferObj[iEntry].commandHandle != commandHandle)
    {
        /* This means that object has been re-used by another request. Indicate
         * that the operation is completed.  */
        return (DRV_SST25_COMMAND_COMPLETED);
    }

    /* Return the last known buffer object status */
    return (gDrvSst25BufferObj[iEntry].status);
}

void DRV_SST25_BlockRead
(
    const DRV_HANDLE handle,
    DRV_SST25_BLOCK_COMMAND_HANDLE *commandHandle,
    void *targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SST25_BLOCK_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SST25_CLIENT_OBJ *clientObj = NULL;
    DRV_SST25_OBJ *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    DRV_SST25_BUFFER_OBJ *bufferObj = NULL;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SST25_BLOCK_COMMAND_HANDLE_INVALID;

    /* Validate the driver handle */
    clientObj = _DRV_SST25_ValidateDriverHandle(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockRead(): Invalid driver handle.\n");
        return;
    }

    /* Check if the driver was opened with read intent */
    if (!(clientObj->intent & DRV_IO_INTENT_READ))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockRead(): Opened with non-read intent.\n");
        return;
    }

    dObj = clientObj->driverObj;

    if ((targetBuffer == NULL) || (nBlock == 0) || ((blockStart + nBlock) > dObj->mediaGeometryTable[DRV_SST25_GEOMETRY_TABLE_READ_ENTRY].numBlocks))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockRead(): Invalid parameters.\n");
        return;
    }

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockRead(): Failed to acquire the driver object mutex.\n");
        return;
    }

    bufferObj = DRV_SST25_AllocateBufferObject (clientObj, targetBuffer, blockStart, nBlock, DRV_SST25_OPERATION_TYPE_READ);
    if (bufferObj != NULL)
    {
        *tempHandle1 = bufferObj->commandHandle;

        /* Add the request to the queue. */
        DRV_SST25_AddToQueue (dObj, bufferObj);
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);

    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockRead(): Read request has been queued.\n");
    return;
}

void DRV_SST25_BlockWrite
(
    DRV_HANDLE handle,
    DRV_SST25_BLOCK_COMMAND_HANDLE *commandHandle,
    uint8_t *sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SST25_BLOCK_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SST25_CLIENT_OBJ *clientObj = NULL;
    DRV_SST25_OBJ *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    DRV_SST25_BUFFER_OBJ *bufferObj = NULL;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SST25_BLOCK_COMMAND_HANDLE_INVALID;

    /* Validate the driver handle */
    clientObj = _DRV_SST25_ValidateDriverHandle(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockWrite(): Invalid driver handle.\n");
        return;
    }

    /* Check if the driver was opened with write intent */
    if (!(clientObj->intent & DRV_IO_INTENT_WRITE))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockWrite(): Opened with non-write intent.\n");
        return;
    }

    dObj = clientObj->driverObj;
    if ((sourceBuffer == NULL) || (nBlock == 0) || ((blockStart + nBlock) > dObj->mediaGeometryTable[DRV_SST25_GEOMETRY_TABLE_WRITE_ENTRY].numBlocks))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockWrite(): Invalid parameters.\n");
        return;
    }

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockWrite(): Failed to acquire the driver object mutex.\n");
        return;
    }

    bufferObj = DRV_SST25_AllocateBufferObject (clientObj, sourceBuffer, blockStart, nBlock, DRV_SST25_OPERATION_TYPE_WRITE);
    if (bufferObj != NULL)
    {
        *tempHandle1 = bufferObj->commandHandle;

        /* Add the request to the queue. */
        DRV_SST25_AddToQueue (dObj, bufferObj);
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);

    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockWrite(): Write request has been queued.\n");
    return;
}

void DRV_SST25_BlockErase
(
    const DRV_HANDLE handle,
    DRV_SST25_BLOCK_COMMAND_HANDLE *commandHandle,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SST25_BLOCK_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SST25_CLIENT_OBJ *clientObj = NULL;
    DRV_SST25_OBJ *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    DRV_SST25_BUFFER_OBJ *bufferObj = NULL;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SST25_BLOCK_COMMAND_HANDLE_INVALID;

    /* Validate the driver handle */
    clientObj = _DRV_SST25_ValidateDriverHandle(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockErase(): Invalid driver handle.\n");
        return;
    }

    /* Check if the driver was opened with write intent */
    if (!(clientObj->intent & DRV_IO_INTENT_WRITE))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockErase(): Opened with non-write intent.\n");
        return;
    }

    dObj = clientObj->driverObj;
    if ((nBlock == 0) || ((blockStart + nBlock) > dObj->mediaGeometryTable[DRV_SST25_GEOMETRY_TABLE_ERASE_ENTRY].numBlocks))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockErase(): Invalid parameters.\n");
        return;
    }

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockErase(): Failed to acquire the driver object mutex.\n");
        return;
    }

    bufferObj = DRV_SST25_AllocateBufferObject (clientObj, NULL, blockStart, nBlock, DRV_SST25_OPERATION_TYPE_ERASE);
    if (bufferObj != NULL)
    {
        *tempHandle1 = bufferObj->commandHandle;

        /* Add the request to the queue. */
        DRV_SST25_AddToQueue (dObj, bufferObj);
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);

    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockErase(): Erase request has been queued.\n");

    return;
}

void DRV_SST25_ChipErase
(
    const DRV_HANDLE handle,
    DRV_SST25_BLOCK_COMMAND_HANDLE *commandHandle
)
{
    DRV_SST25_BLOCK_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SST25_CLIENT_OBJ *clientObj = NULL;
    DRV_SST25_OBJ *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    DRV_SST25_BUFFER_OBJ *bufferObj = NULL;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SST25_BLOCK_COMMAND_HANDLE_INVALID;

    /* Validate the driver handle */
    clientObj = _DRV_SST25_ValidateDriverHandle(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_ChipErase(): Invalid driver handle.\n");
        return;
    }

    /* Check if the driver was opened with write intent */
    if (!(clientObj->intent & DRV_IO_INTENT_WRITE))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_ChipErase(): Opened with non-write intent.\n");
        return;
    }

    dObj = clientObj->driverObj;
    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_ChipErase(): Failed to acquire the driver object mutex.\n");
        return;
    }

    bufferObj = DRV_SST25_AllocateBufferObject (clientObj, NULL, 0, 0, DRV_SST25_OPERATION_TYPE_CHIP_ERASE);
    if (bufferObj != NULL)
    {
        *tempHandle1 = bufferObj->commandHandle;

        /* Add the request to the queue. */
        DRV_SST25_AddToQueue (dObj, bufferObj);
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);

    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_ChipErase(): Erase request has been queued.\n");

    return;
}

void DRV_SST25_BlockEraseWrite
(
    const DRV_HANDLE handle,
    DRV_SST25_BLOCK_COMMAND_HANDLE *commandHandle,
    void *sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SST25_BLOCK_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SST25_CLIENT_OBJ *clientObj = NULL;
    DRV_SST25_OBJ *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    DRV_SST25_BUFFER_OBJ *bufferObj = NULL;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SST25_BLOCK_COMMAND_HANDLE_INVALID;

    /* Validate the driver handle */
    clientObj = _DRV_SST25_ValidateDriverHandle(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockEraseWrite(): Invalid driver handle.\n");
        return;
    }

    /* Check if the driver was opened with write intent */
    if (!(clientObj->intent & DRV_IO_INTENT_WRITE))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockEraseWrite(): Opened with non-write intent.\n");
        return;
    }

    dObj = clientObj->driverObj;
    if ((sourceBuffer == NULL) || (nBlock == 0) || ((blockStart + nBlock) > dObj->mediaGeometryTable[DRV_SST25_GEOMETRY_TABLE_WRITE_ENTRY].numBlocks))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockEraseWrite(): Invalid parameters.\n");
        return;
    }

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockEraseWrite(): Failed to acquire the driver object mutex.\n");
        return;
    }

    bufferObj = DRV_SST25_AllocateBufferObject (clientObj, sourceBuffer, blockStart, nBlock, DRV_SST25_OPERATION_TYPE_ERASE_WRITE);
    if (bufferObj != NULL)
    {
        *tempHandle1 = bufferObj->commandHandle;

        /* Add the request to the queue. */
        DRV_SST25_AddToQueue (dObj, bufferObj);
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);

    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST25_BlockEraseWrite(): EraseWrite request has been queued.\n");
    return;
}

void DRV_SST25_Tasks
(
    SYS_MODULE_OBJ object
)
{
    DRV_SST25_BUFFER_OBJ *bufferObj = NULL;
    DRV_SST25_CLIENT_OBJ *clientObj = NULL;
    DRV_SST25_OBJ *dObj = NULL;
    bool done = false;
    DRV_SPI_BUFFER_EVENT event = DRV_SPI_BUFFER_EVENT_COMPLETE;

    /* Check if the specified module object is in valid range */
    if(object >= DRV_SST25_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "DRV_SST25: Invalid module object \n");
        return;
    }

    dObj = (DRV_SST25_OBJ*)&gDrvSst25Obj[object];
    
    if(!dObj->inUse)
    {
        /* This intance of the driver is not initialized. Don't
         * do anything */
        return;
    }
    
    if (OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
    {
        return;
    }

    switch(dObj->state)
    {
        case DRV_SST25_TASK_OPEN_SPI_DRIVER:
            {
                dObj->spiDriverHandle = DRV_SPI_Open (dObj->spiDriverIndex, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING);
                if(dObj->spiDriverHandle == DRV_HANDLE_INVALID)
                {
                    break;
                }
                else
                {
                    /* Register the callbacks with the driver. */
                    DRV_SPI_ClientConfigure (dObj->spiDriverHandle, &dObj->spiClientData);

                    dObj->subState = DRV_SST25_UNLOCK_ENABLE_WRSR_REG;
                    dObj->state = DRV_SST25_TASK_UNLOCK_FLASH;
                    /* Fall through to the next case. */
                }
            }

        case DRV_SST25_TASK_UNLOCK_FLASH:
            {
                event = dObj->flashFunctions.unlock (dObj);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    /* Transition to the next state. */
                    dObj->subState = DRV_SST25_READ_ID;
                    dObj->state = DRV_SST25_TASK_READ_FLASH_ID;
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* Handle error. */
                    dObj->state = DRV_SST25_TASK_ERROR;
                }
                else
                {
                    /* Continue to remain in the same state. */
                }
                break;
            }

        case DRV_SST25_TASK_READ_FLASH_ID:
            {
                event = DRV_SST25_ReadFlashId (dObj);
                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    if (DRV_SST25_UpdateGeometry (dObj, dObj->flashId[1]))
                    {
                        /* Transition to the next state. */
                        dObj->state = DRV_SST25_TASK_PROCESS_QUEUE;
                        dObj->status = SYS_STATUS_READY;
                    }
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* Handle error. */
                    dObj->state = DRV_SST25_TASK_ERROR;
                }
                break;
            }

        case DRV_SST25_TASK_PROCESS_QUEUE:
            {
                /* Process the queued requests. */
                dObj->currentBufObj = dObj->queue;
                if (dObj->currentBufObj == NULL)
                {
                    /* Queue is empty. Continue to remain in the same state. */
                    break;
                }
                else
                {
                    /* Init the various sub state machines. */
                    dObj->eraseState = DRV_SST25_ERASE_INIT;
                    dObj->writeState = DRV_SST25_WRITE_INIT;
                    dObj->subState = DRV_SST25_READ_INIT;
                    dObj->ewState = DRV_SST25_EW_INIT;

                    dObj->state = DRV_SST25_TASK_TRANSFER;
                    /* Fall through. */
                }
            }

        case DRV_SST25_TASK_TRANSFER:
            {
                bufferObj = dObj->currentBufObj;
                switch (bufferObj->opType)
                {
                    case DRV_SST25_OPERATION_TYPE_READ:
                    default:
                        {
                            event = dObj->flashFunctions.read(dObj, &bufferObj->buffer[0], bufferObj->blockStart, bufferObj->nBlocks);
                            break;
                        }
                    case DRV_SST25_OPERATION_TYPE_WRITE:
                        {
                            event = dObj->flashFunctions.write(dObj, &bufferObj->buffer[0], bufferObj->blockStart, bufferObj->nBlocks);
                            break;
                        }
                    case DRV_SST25_OPERATION_TYPE_ERASE:
                        {
                            event = dObj->flashFunctions.erase(dObj, bufferObj->blockStart << 12, bufferObj->nBlocks);
                            break;
                        }
                    case DRV_SST25_OPERATION_TYPE_CHIP_ERASE:
                        {
                            event = dObj->flashFunctions.chipErase(dObj);
                            break;
                        }
                    case DRV_SST25_OPERATION_TYPE_ERASE_WRITE:
                        {
                            event = dObj->flashFunctions.eraseWrite(dObj, &bufferObj->buffer[0], bufferObj->blockStart, bufferObj->nBlocks);
                            break;
                        }
                }

                if (event == DRV_SPI_BUFFER_EVENT_COMPLETE)
                {
                    bufferObj->status = DRV_SST25_COMMAND_COMPLETED;
                    /* The operation has completed. */
                    done = true;
                    event = DRV_SST25_EVENT_BLOCK_COMMAND_COMPLETE;
                }
                else if (event == DRV_SPI_BUFFER_EVENT_ERROR)
                {
                    /* The operation has failed. */
                    bufferObj->status = DRV_SST25_COMMAND_ERROR_UNKNOWN;
                    done = true;
                    event = DRV_SST25_EVENT_BLOCK_COMMAND_ERROR;
                }
                else
                {
                    /* Continue to remain in the same state. */
                    break;
                }

                if (done)
                {
                    dObj->state = DRV_SST25_TASK_PROCESS_QUEUE;
                    bufferObj->inUse = false;
                    DRV_SST25_UpdateQueue (dObj);

                    clientObj = (DRV_SST25_CLIENT_OBJ *)bufferObj->hClient;
                    if(clientObj->eventHandler != NULL)
                    {
                        /* Call the event handler */
                        clientObj->eventHandler(event, bufferObj->commandHandle, clientObj->context);
                    }
                }
                break;
            }

           case DRV_SST25_TASK_IDLE:
            {
                break;
            }

           case DRV_SST25_TASK_ERROR:
           default:
            {
                break;
            }
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);

    return;
}

/*******************************************************************************
 End of File
*/
