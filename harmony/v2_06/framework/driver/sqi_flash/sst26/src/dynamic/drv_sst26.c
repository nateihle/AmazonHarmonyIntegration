/******************************************************************************
  SST26 Driver Interface Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sst26.c

  Summary:
    SST26 Driver Interface Definition

  Description:
    The SST26 Driver provides a interface to access the SST26 on the PIC32
    microcontroller. This file implements the SST26 Driver interface. This file
    should be included in the project if SST26 driver functionality is needed.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "driver/sqi_flash/sst26/src/drv_sst26_local.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global objects
// *****************************************************************************
// *****************************************************************************

/*************************************************
 * Hardware instance objects
 *************************************************/

DRV_SST26_OBJECT gDrvSST26Obj[DRV_SST26_INSTANCES_NUMBER];

/*************************************************
 * Driver Client Objects
 *************************************************/

DRV_SST26_CLIENT_OBJECT gDrvSst26ClientObj[DRV_SST26_CLIENTS_NUMBER];

/*************************************************
 * Driver Buffer Objects.
 *************************************************/

DRV_SST26_BUFFER_OBJECT gDrvSst26BufferObject[DRV_SST26_BUFFER_OBJECT_NUMBER];

DRV_SQI_TransferElement gSst26XferElement[4];
uint8_t __attribute__((coherent)) gSst26CmdParams[7];
uint8_t __attribute__((coherent)) gSst26FlashId[3];
uint8_t __attribute__((coherent)) gSst26StatusReg;
DRV_SQI_TransferFrame gSst26XferFrame[4];

/************************************************
 * This token is incremented for every request added to the queue and is used
 * to generate a different buffer handle for every request.
 ***********************************************/

uint16_t gDrvSst26BufferToken = 0;

/**************************************************
 * Erase buffer size in case the erase write feature is enabled
 **************************************************/
#define DRV_SST26_ERASE_BUFFER_SIZE (4096)
uint8_t gDrvSST26EraseBuffer[DRV_SST26_INSTANCES_NUMBER][DRV_SST26_ERASE_BUFFER_SIZE] __attribute__((coherent, aligned(16)));

/*************************************************
 * OSAL Declarations
 *************************************************/
/* SST26 Client Object Mutex */
OSAL_MUTEX_DECLARE(sst26ClientObjMutex);

/* FS Function registration table. */
const SYS_FS_MEDIA_FUNCTIONS sst26MediaFunctions =
{
    .mediaStatusGet     = DRV_SST26_IsAttached,
    .mediaGeometryGet   = DRV_SST26_GeometryGet,
    .sectorRead         = DRV_SST26_Read,
    .sectorWrite        = DRV_SST26_EraseWrite,
    .eventHandlerset    = DRV_SST26_EventHandlerSet,
    .commandStatusGet   = (void *)DRV_SST26_CommandStatus,
    .Read               = DRV_SST26_Read,
    .erase              = DRV_SST26_Erase,
    .addressGet         = DRV_SST26_AddressGet,
    .open               = DRV_SST26_Open,
    .close              = DRV_SST26_Close,
    .tasks              = DRV_SST26_Tasks,
};

/* Table mapping the Flash ID's to their sizes. */
uint32_t gSstFlashIdSizeTable [5][2] = {
    {0x01, 0x200000}, /* 16 MBit */
    {0x41, 0x200000}, /* 16 MBit */
    {0x02, 0x400000}, /* 32 MBit */
    {0x42, 0x400000}, /* 32 MBit */
    {0x43, 0x800000}  /* 64 MBit */
};

static DRV_SQI_COMMAND_STATUS DRV_SST26_HandleRead
(
    DRV_SST26_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
);

static DRV_SQI_COMMAND_STATUS DRV_SST26_HandleWrite
(
    DRV_SST26_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
);

static DRV_SQI_COMMAND_STATUS DRV_SST26_HandleErase
(
    DRV_SST26_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
);
static DRV_SQI_COMMAND_STATUS DRV_SST26_HandleEraseWrite
(
    DRV_SST26_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
);

DRV_SST26_TransferOperation gSst26XferFuncPtr[4] =
{
    DRV_SST26_HandleRead,
    DRV_SST26_HandleWrite,
    DRV_SST26_HandleErase,
    DRV_SST26_HandleEraseWrite,
};

// *****************************************************************************
// *****************************************************************************
// Section: SST26 Driver Local Functions
// *****************************************************************************
// *****************************************************************************

/* This function finds a free buffer object and populates it with the transfer
 * parameters. It also generates a new command handle for the request. */
static DRV_SST26_BUFFER_OBJECT* DRV_SST26_AllocateBufferObject
(
    DRV_SST26_CLIENT_OBJECT *clientObj,
    void *buffer,
    uint32_t blockStart,
    uint32_t nBlocks,
    DRV_SST26_OPERATION_TYPE opType
)
{
    uint8_t iEntry = 0;
    DRV_SST26_BUFFER_OBJECT *bufferObj = NULL;

    for (iEntry = 0; iEntry < DRV_SST26_BUFFER_OBJECT_NUMBER; iEntry++)
    {
        /* Search for a free buffer object to use */
        if (gDrvSst26BufferObject[iEntry].inUse == false)
        {
            /* Found a free buffer object. */
            bufferObj = &gDrvSst26BufferObject[iEntry];

            bufferObj->inUse         = true;
            bufferObj->commandHandle = DRV_SST26_MAKE_HANDLE(gDrvSst26BufferToken, iEntry);
            bufferObj->hClient       = clientObj;
            bufferObj->buffer        = buffer;
            bufferObj->blockStart    = blockStart;
            bufferObj->nBlocks       = nBlocks;
            bufferObj->opType        = opType;
            bufferObj->status        = DRV_SQI_COMMAND_QUEUED;
            bufferObj->next          = NULL;
            bufferObj->previous      = NULL;

            /* Update the token number. */
            DRV_SST26_UPDATE_BUF_TOKEN(gDrvSst26BufferToken);
            break;
        }
    }

    return bufferObj;
}

/* This function finds and allocates a client object. */
static DRV_SST26_CLIENT_OBJECT* DRV_SST26_AllocateClientObject
(
    void
)
{
    uint8_t iClient = 0;
    DRV_SST26_CLIENT_OBJECT *object = &gDrvSst26ClientObj[0];

    /* Find available slot in array of client objects */
    for (iClient = 0; iClient < DRV_SST26_CLIENTS_NUMBER ; iClient++)
    {
        if (!object->inUse)
        {
            return object;
        }

        object ++;
    }

    return NULL;
}

/* This function validates the driver handle and returns the client object
 * pointer associated with the driver handle if the handle is valid. If the
 * driver handle is not valid or if the driver is in a not ready state then
 * NULL is returned. */
static DRV_SST26_CLIENT_OBJECT * DRV_SST26_ClientHandleValidate
(
    DRV_HANDLE handle
)
{
    DRV_SST26_CLIENT_OBJECT *clientObj = NULL;
    DRV_SST26_OBJECT *dObj = NULL;

    /* Validate the handle */
    if (handle == DRV_HANDLE_INVALID)
    {
        return NULL;
    }

    /* See if the client has been opened */
    clientObj = (DRV_SST26_CLIENT_OBJECT *)handle;
    if (!clientObj->inUse)
    {
        return NULL;
    }

    /* Check if the driver is ready for operation */
    dObj = (DRV_SST26_OBJECT *)clientObj->driverObj;
    if (dObj->status != SYS_STATUS_READY)
    {
        return NULL;
    }

    return clientObj;
}

/* This function returns the flash size in bytes for the specified deviceId. A
 * zero is returned if the device id is not supported. */
static uint32_t DRV_SST26_GetFlashSize
(
    uint8_t deviceId
)
{
    uint8_t i = 0;

    for (i = 0; i < 5; i++)
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
static bool DRV_SST26_UpdateGeometry
(
    DRV_SST26_OBJECT *dObj,
    uint8_t deviceId
)
{
    uint32_t flashSize = DRV_SST26_GetFlashSize (deviceId);
    if (flashSize == 0)
    {
        return false;
    }

    /* Read block size and number of blocks */
    dObj->mediaGeometryTable[0].blockSize = 1;
    dObj->mediaGeometryTable[0].numBlocks = flashSize;

    /* Write block size and number of blocks */
    dObj->mediaGeometryTable[1].blockSize = 256;
    dObj->mediaGeometryTable[1].numBlocks = flashSize >> 8;

    /* Erase block size and number of blocks */
    dObj->mediaGeometryTable[2].blockSize = 4096;
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
static void DRV_SST26_AddToQueue
(
    DRV_SST26_OBJECT *dObj,
    DRV_SST26_BUFFER_OBJECT *bufferObj
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
static void DRV_SST26_UpdateQueue
(
    DRV_SST26_OBJECT *dObj
)
{
    DRV_SST26_BUFFER_OBJECT * bufferObj = dObj->queue;

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

static void DRV_SST26_RemoveClientBufferObjects
(
    DRV_SST26_CLIENT_OBJECT *clientObj,
    DRV_SST26_OBJECT *dObj
)
{
    DRV_SST26_BUFFER_OBJECT *bufferObject = NULL;
    DRV_SST26_BUFFER_OBJECT *lastObject = NULL;
    DRV_SST26_BUFFER_OBJECT *head = NULL;
    DRV_SST26_BUFFER_OBJECT *temp = NULL;

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

/* This function resets the flash by sending down the reset enable command
 * followed by the reset command. */
static DRV_SQI_COMMAND_STATUS DRV_SST26_ResetFlash
(
    DRV_SST26_OBJECT *dObj
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->subState)
    {
        case DRV_SST26_SUBTASK_RESET_ENABLE_CMD:
        default:
            { 
                /* Reset Enable Command */
                DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                frame->instruction = DRV_SST26_CMD_FLASH_RESET_ENABLE;                
                frame->data = NULL;
                frame->length = 0;
                frame->laneCfg = DRV_SQI_LANE_SINGLE;
                frame->numDummyBytes = 0;
                frame->flags = (DRV_SQI_FLAG_INSTR_ENABLE_MASK);
                frame->flags |= DRV_SQI_FLAG_SQI_CS_NUMBER(dObj->sqiDevice);
                DRV_SQI_TransferFrames (dObj->sqiHandle, &dObj->cmdHandle, frame, 1);
                if (dObj->cmdHandle == DRV_SQI_COMMAND_HANDLE_INVALID)
                {
                    /* Failed to queue the request. */
                    cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;
                }
                else
                {
                    dObj->subState = DRV_SST26_SUBTASK_RESET_ENABLE_STATUS;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_SST26_SUBTASK_RESET_ENABLE_STATUS:
            {
                cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    dObj->tmrHandle = SYS_TMR_DelayMS (25);
                    dObj->subState = DRV_SST26_SUBTASK_RESET_ENABLE_DELAY;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }
                break;
            }

        case DRV_SST26_SUBTASK_RESET_ENABLE_DELAY:
            {
                if (SYS_TMR_DelayStatusGet(dObj->tmrHandle))
                {
                    /* Reset Command */
                    DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                    frame->instruction = DRV_SST26_CMD_FLASH_RESET;                
                    frame->data = NULL;
                    frame->length = 0;
                    frame->laneCfg = DRV_SQI_LANE_SINGLE;
                    frame->numDummyBytes = 0;
                    frame->flags = (DRV_SQI_FLAG_INSTR_ENABLE_MASK);
                    frame->flags |= DRV_SQI_FLAG_SQI_CS_NUMBER(dObj->sqiDevice);
                    DRV_SQI_TransferFrames (dObj->sqiHandle, &dObj->cmdHandle, frame, 1);

                    if (dObj->cmdHandle == DRV_SQI_COMMAND_HANDLE_INVALID)
                    {
                        /* Failed to queue the request. */
                        cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;
                    }
                    else
                    {
                        dObj->subState = DRV_SST26_SUBTASK_RESET_FLASH_STATUS;
                        cmdStatus = DRV_SQI_COMMAND_QUEUED;
                    }
                }
                else
                {
                    /* Wait for the timer to expire. */
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_SST26_SUBTASK_RESET_FLASH_STATUS:
            {
                cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
                break;
            }
    }

    return cmdStatus;
}

/* Enables the QUAD IO on the flash */
static DRV_SQI_COMMAND_STATUS DRV_SST26_EnableQuadIO
(
    DRV_SST26_OBJECT *dObj
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->subState)
    {
        case DRV_SST26_SUBTASK_ENABLE_QUAD_IO:
        default:
            {
                /* Enable Quad IO Command. */
                DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                frame->instruction = DRV_SST26_CMD_ENABLE_QUAD_IO;                
                frame->data = NULL;
                frame->length = 0;
                frame->laneCfg = DRV_SQI_LANE_SINGLE;
                frame->numDummyBytes = 0;
                frame->flags = (DRV_SQI_FLAG_INSTR_ENABLE_MASK);
                frame->flags |= DRV_SQI_FLAG_SQI_CS_NUMBER(dObj->sqiDevice);
                DRV_SQI_TransferFrames (dObj->sqiHandle, &dObj->cmdHandle, frame, 1);
                    
                if (dObj->cmdHandle == DRV_SQI_COMMAND_HANDLE_INVALID)
                {
                    /* Failed to queue the request. */
                    cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;
                }
                else
                {
                    dObj->subState = DRV_SST26_SUBTASK_ENABLE_QUAD_IO_STATUS;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_SST26_SUBTASK_ENABLE_QUAD_IO_STATUS:
            {
                cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
                break;
            }
    }

    return cmdStatus;
}

// *****************************************************************************
/* Function:
    static DRV_SQI_COMMAND_STATUS DRV_SST26_WriteEnable
    (
        DRV_S25FL_OBJECT *dObj
    );

  Summary:
    Frames and sends the write enable command to the flash.

  Description:
    This function frames and sends the write enable command to the flash and
    also tracks the status of the command.

  Remarks:
    None.
*/
static DRV_SQI_COMMAND_STATUS DRV_SST26_WriteEnable
(
    DRV_SST26_OBJECT *dObj
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->writeEnableState)
    {
        case DRV_SST26_WRITE_ENABLE_CMD:
        default:
            {
                DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                frame->instruction = DRV_SST26_CMD_WRITE_ENABLE;
                frame->laneCfg = DRV_SQI_LANE_QUAD_ALL;
                frame->numDummyBytes = 0;
                frame->flags = (DRV_SQI_FLAG_INSTR_ENABLE_MASK);
                frame->flags |= DRV_SQI_FLAG_SQI_CS_NUMBER(dObj->sqiDevice);
                DRV_SQI_TransferFrames (dObj->sqiHandle, &dObj->cmdHandle, frame, 1);
                if (dObj->cmdHandle == DRV_SQI_COMMAND_HANDLE_INVALID)
                {
                    cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;
                }
                else
                {
                    dObj->writeEnableState = DRV_SST26_WRITE_ENABLE_CMD_STATUS;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_SST26_WRITE_ENABLE_CMD_STATUS:
            {
                cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
                break;
            }
    }

    return cmdStatus;
}

/* This function sends down command to perform a global unprotect of the flash.
 * */
static DRV_SQI_COMMAND_STATUS DRV_SST26_UnlockFlash
(
    DRV_SST26_OBJECT *dObj
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->subState)
    {
        case DRV_SST26_SUBTASK_UNLOCK_FLASH_WRITE_ENABLE:
        default:
        {
            cmdStatus = DRV_SST26_WriteEnable(dObj);
            if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
            {
                dObj->subState = DRV_SST26_SUBTASK_UNLOCK_FLASH_CMD;
                cmdStatus = DRV_SQI_COMMAND_QUEUED;
            }

            break;
        }
        
        case DRV_SST26_SUBTASK_UNLOCK_FLASH_CMD:
        {
            /* Flash unlock Command */
            DRV_SQI_TransferFrame *frame = dObj->xferFrame;
            frame->instruction = DRV_SST26_CMD_UNPROTECT_GLOBAL;
            frame->laneCfg = DRV_SQI_LANE_QUAD_ALL;
            frame->numDummyBytes = 0;
            frame->flags = (DRV_SQI_FLAG_INSTR_ENABLE_MASK);
            frame->flags |= DRV_SQI_FLAG_SQI_CS_NUMBER(dObj->sqiDevice);
            DRV_SQI_TransferFrames(dObj->sqiHandle, &dObj->cmdHandle, frame, 1);

            if (dObj->cmdHandle == DRV_SST26_COMMAND_HANDLE_INVALID)
            {
                /* Failed to queue the request. */
                cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;
            }
            else
            {
                dObj->subState = DRV_SST26_SUBTASK_UNLOCK_FLASH_CMD_STATUS;
                cmdStatus = DRV_SQI_COMMAND_QUEUED;
            }
            break;
        }

        case DRV_SST26_SUBTASK_UNLOCK_FLASH_CMD_STATUS:
        {
            cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
            break;
        }
    }

    return cmdStatus;
}

/* This function reads and stores the flash id. */
static DRV_SQI_COMMAND_STATUS DRV_SST26_ReadFlashId
(
    DRV_SST26_OBJECT *dObj
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->subState)
    {
        case DRV_SST26_SUBTASK_JEDEC_ID_READ_CMD:
        default:
            {
                /* JEDEC-ID Read Command */
                DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                frame->instruction = DRV_SST26_CMD_QUAD_JEDEC_ID_READ;                
                frame->data = &dObj->flashId[0];
                frame->length = 3;
                frame->laneCfg = DRV_SQI_LANE_QUAD_ALL;
                frame->numDummyBytes = 1;
                frame->flags = (DRV_SQI_FLAG_INSTR_ENABLE_MASK | DRV_SQI_FLAG_DATA_ENABLE_MASK | 
                        DRV_SQI_FLAG_DATA_TARGET_REGISTER | DRV_SQI_FLAG_DATA_DIRECTION_READ);
                frame->flags |= DRV_SQI_FLAG_SQI_CS_NUMBER(dObj->sqiDevice);
                DRV_SQI_TransferFrames (dObj->sqiHandle, &dObj->cmdHandle, frame, 1);
                
                if (dObj->cmdHandle == DRV_SQI_COMMAND_HANDLE_INVALID)
                {
                    /* Failed to queue the request. */
                    cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;
                }
                else
                {
                    dObj->subState = DRV_SST26_SUBTASK_JEDEC_ID_READ_STATUS;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }
                break;
            }

        case DRV_SST26_SUBTASK_JEDEC_ID_READ_STATUS:
            {
                cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
                break;
            }
    }

    return cmdStatus;
}

/* This function is responsible for handling the read operation. */
static DRV_SQI_COMMAND_STATUS DRV_SST26_HandleRead
(
    DRV_SST26_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->subState)
    {
        case DRV_SST26_SUBTASK_CMD:
        default:
            {
                /* Read Command Transfer Element */
                DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                frame->instruction = DRV_SST26_CMD_HIGH_SPEED_READ;
                frame->address = blockAddress;
                frame->option = 0;
                frame->data = data;
                frame->length = nBlocks;
                frame->laneCfg = DRV_SQI_LANE_QUAD_ALL;
                frame->numDummyBytes = 2;
                frame->flags = (DRV_SQI_FLAG_INSTR_ENABLE_MASK | DRV_SQI_FLAG_DATA_ENABLE_MASK |
                        DRV_SQI_FLAG_OPT_ENABLE_MASK | DRV_SQI_FLAG_ADDR_ENABLE_MASK | 
                        DRV_SQI_FLAG_DATA_TARGET_MEMORY | DRV_SQI_FLAG_DATA_DIRECTION_READ);
                frame->flags |= DRV_SQI_FLAG_SQI_CS_NUMBER(dObj->sqiDevice);
                DRV_SQI_TransferFrames (dObj->sqiHandle, &dObj->cmdHandle, frame, 1);
                if (dObj->cmdHandle == DRV_SQI_COMMAND_HANDLE_INVALID)
                {
                    cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;
                }
                else
                {
                    dObj->subState = DRV_SST26_SUBTASK_CMD_STATUS;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_SST26_SUBTASK_CMD_STATUS:
            {
                cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
                break;
            }
    }

    return cmdStatus;
}


/* Function to read the status register of the flash. */
static DRV_SQI_COMMAND_STATUS DRV_SST26_ReadStatus
(
    DRV_SST26_OBJECT *dObj
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->subState)
    {
        case DRV_SST26_SUBTASK_CMD:
        default:
            {
                /* Read Status Register command */
                DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                frame->instruction = DRV_SST26_CMD_READ_STATUS_REG;
                frame->data = dObj->statusReg;
                frame->length = 1;
                frame->laneCfg = DRV_SQI_LANE_QUAD_ALL;
                frame->numDummyBytes = 1;
                frame->flags = (DRV_SQI_FLAG_INSTR_ENABLE_MASK | DRV_SQI_FLAG_DATA_ENABLE_MASK |
                        DRV_SQI_FLAG_DATA_TARGET_REGISTER | DRV_SQI_FLAG_DATA_DIRECTION_READ);
                frame->flags |= DRV_SQI_FLAG_SQI_CS_NUMBER(dObj->sqiDevice);
                DRV_SQI_TransferFrames (dObj->sqiHandle, &dObj->cmdHandle, frame, 1);
                
                if (dObj->cmdHandle == DRV_SQI_COMMAND_HANDLE_INVALID)
                {
                    cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;
                }
                else
                {
                    dObj->subState = DRV_SST26_SUBTASK_CMD_STATUS;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_SST26_SUBTASK_CMD_STATUS:
            {
                cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
                break;
            }
    }

    return cmdStatus;
}

/* This function writes one page of data. */
static DRV_SQI_COMMAND_STATUS DRV_SST26_WriteOnePage
(
    DRV_SST26_OBJECT *dObj,
    uint32_t address,
    uint8_t *data
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->subState)
    {
        case DRV_SST26_SUBTASK_CMD:
        default:
            {
                DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                frame->instruction = DRV_SST26_CMD_PAGE_PROGRAM;
                frame->address = address << 0x08;
                frame->data = data;
                frame->length = 256;
                frame->laneCfg = DRV_SQI_LANE_QUAD_ALL;
                frame->numDummyBytes = 0;
                frame->flags = (DRV_SQI_FLAG_INSTR_ENABLE_MASK | DRV_SQI_FLAG_DATA_ENABLE_MASK |
                        DRV_SQI_FLAG_ADDR_ENABLE_MASK | DRV_SQI_FLAG_DATA_TARGET_MEMORY |
                        DRV_SQI_FLAG_DATA_DIRECTION_WRITE);
                frame->flags |= DRV_SQI_FLAG_SQI_CS_NUMBER(dObj->sqiDevice);
                DRV_SQI_TransferFrames (dObj->sqiHandle, &dObj->cmdHandle, frame, 1);
                if (dObj->cmdHandle == DRV_SQI_COMMAND_HANDLE_INVALID)
                {
                    cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;
                }
                else
                {
                    dObj->subState = DRV_SST26_SUBTASK_CMD_STATUS;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_SST26_SUBTASK_CMD_STATUS:
            {
                cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
                break;
            }
    }

    return cmdStatus;
}

static DRV_SQI_COMMAND_STATUS DRV_SST26_HandleWrite
(
    DRV_SST26_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->writeState)
    {
        case DRV_SST26_WRITE_INIT:
        default:
            {
                dObj->bufferOffset = 0;
                dObj->blockAddress = blockAddress;
                dObj->nBlocks = nBlocks;
                dObj->writePtr = data;

                dObj->subState = DRV_SST26_SUBTASK_CMD;
                dObj->writeState = DRV_SST26_WRITE_WRITE_ENABLE;
                dObj->writeEnableState = DRV_SST26_WRITE_ENABLE_CMD;
                
                /* Fall through */
            }

        case DRV_SST26_WRITE_WRITE_ENABLE:
        {
            cmdStatus = DRV_SST26_WriteEnable(dObj);
            if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
            {
                dObj->writeState = DRV_SST26_WRITE_CMD;
                cmdStatus = DRV_SQI_COMMAND_QUEUED;
            }

            break;
        }
        
        case DRV_SST26_WRITE_CMD:
            {
                cmdStatus = DRV_SST26_WriteOnePage (dObj, dObj->blockAddress, &dObj->writePtr[dObj->bufferOffset]);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    dObj->subState = DRV_SST26_SUBTASK_CMD;
                    dObj->writeState = DRV_SST26_WRITE_CMD_STATUS;

                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_SST26_WRITE_CMD_STATUS:
            {
                cmdStatus = DRV_SST26_ReadStatus(dObj);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    dObj->subState = DRV_SST26_SUBTASK_CMD;

                    if (!(*dObj->statusReg & 0x80))
                    {
                        dObj->nBlocks --;
                        if (dObj->nBlocks != 0)
                        {
                            /* There is still data to be programmed. */
                            dObj->bufferOffset += 256;
                            /* Update the block address and the write pointer */
                            //dObj->writePtr += 256;
                            dObj->blockAddress += 1;

                            dObj->subState = DRV_SST26_SUBTASK_CMD;
                            dObj->writeState = DRV_SST26_WRITE_WRITE_ENABLE;
                            dObj->writeEnableState = DRV_SST26_WRITE_ENABLE_CMD;
    
                            cmdStatus = DRV_SQI_COMMAND_QUEUED;
                        }
                    }
                    else
                    {
                        /* Read the status register again */
                        cmdStatus = DRV_SQI_COMMAND_QUEUED;
                    }
                }

                break;
            }
    }

    return cmdStatus;
}

static DRV_SQI_COMMAND_STATUS DRV_SST26_SectorErase
(
    DRV_SST26_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->subState)
    {
        case DRV_SST26_SUBTASK_CMD:
        default:
            {
                DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                frame->instruction = DRV_SST26_CMD_SECTOR_ERASE;
                frame->address = blockAddress;
                frame->data = NULL;
                frame->length = 0;
                frame->laneCfg = DRV_SQI_LANE_QUAD_ALL;
                frame->numDummyBytes = 0;
                frame->flags = (DRV_SQI_FLAG_INSTR_ENABLE_MASK | DRV_SQI_FLAG_ADDR_ENABLE_MASK);
                frame->flags |= DRV_SQI_FLAG_SQI_CS_NUMBER(dObj->sqiDevice);
                DRV_SQI_TransferFrames (dObj->sqiHandle, &dObj->cmdHandle, frame, 1);
                if (dObj->cmdHandle == DRV_SQI_COMMAND_HANDLE_INVALID)
                {
                    cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;
                }
                else
                {
                    dObj->subState = DRV_SST26_SUBTASK_CMD_STATUS;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_SST26_SUBTASK_CMD_STATUS:
            {
                cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
                break;
            }
    }

    return cmdStatus;
}

static DRV_SQI_COMMAND_STATUS DRV_SST26_HandleErase
(
    DRV_SST26_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->eraseState)
    {
        case DRV_SST26_ERASE_INIT:
        default:
            {
                dObj->blockAddress = blockAddress * 4096;
                dObj->nBlocks = nBlocks;
                dObj->eraseState = DRV_SST26_ERASE_WRITE_ENABLE;
                dObj->writeEnableState = DRV_SST26_WRITE_ENABLE_CMD;
                dObj->subState = DRV_SST26_SUBTASK_CMD;
                
                /* Fall through */
            }
        
        case DRV_SST26_ERASE_WRITE_ENABLE:
        {
            cmdStatus = DRV_SST26_WriteEnable(dObj);
            if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
            {
                dObj->eraseState = DRV_SST26_ERASE_CMD;
                cmdStatus = DRV_SQI_COMMAND_QUEUED;
            }

            break;
        }
        
        case DRV_SST26_ERASE_CMD:
            {
                cmdStatus = DRV_SST26_SectorErase (dObj, NULL, dObj->blockAddress, 0);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    dObj->subState = DRV_SST26_SUBTASK_CMD;
                    dObj->eraseState = DRV_SST26_ERASE_CMD_STATUS;

                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }
                break;
            }

        case DRV_SST26_ERASE_CMD_STATUS:
            {
                cmdStatus = DRV_SST26_ReadStatus(dObj);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    dObj->subState = DRV_SST26_SUBTASK_CMD;

                    if (!(*dObj->statusReg & 0x80))
                    {
                        dObj->nBlocks --;
                        if (dObj->nBlocks != 0)
                        {
                            /* There are still sectors to be erased. */
                            dObj->blockAddress += 4096;

                            dObj->eraseState = DRV_SST26_ERASE_WRITE_ENABLE;
                            dObj->writeEnableState = DRV_SST26_WRITE_ENABLE_CMD;
                            dObj->subState = DRV_SST26_SUBTASK_CMD;
                            cmdStatus = DRV_SQI_COMMAND_QUEUED;
                        }
                    }
                    else
                    {
                        /* Read the status register again */
                        cmdStatus = DRV_SQI_COMMAND_QUEUED;
                    }
                }

                break;
            }
    }

    return cmdStatus;
}

static DRV_SQI_COMMAND_STATUS DRV_SST26_HandleEraseWrite
(
    DRV_SST26_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
)
{
    DRV_SST26_BUFFER_OBJECT *bufferObj = dObj->currentBufObj;
    uint8_t pagesPerSector = 16;

    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;  

    switch (dObj->ewState)
    {
        case DRV_SST26_EW_INIT:
        default:
            {
                dObj->subState = DRV_SST26_SUBTASK_CMD;
                dObj->eraseState = DRV_SST26_ERASE_INIT;
                dObj->writeState = DRV_SST26_WRITE_INIT;

                /* Find the sector for the starting page */
                dObj->sectorAddress = bufferObj->blockStart / pagesPerSector;

                /* Find the number of sectors to be updated in this block. */
                dObj->blockOffsetInSector = (bufferObj->blockStart % pagesPerSector);
                dObj->nBlocksToWrite = (pagesPerSector - dObj->blockOffsetInSector);

                if (bufferObj->nBlocks < dObj->nBlocksToWrite)
                {
                    dObj->nBlocksToWrite = bufferObj->nBlocks;
                }

                if (dObj->nBlocksToWrite != pagesPerSector)
                {
                    dObj->writePtr = dObj->ewBuffer;
                    dObj->ewState = DRV_SST26_EW_READ_SECTOR;
                }
                else
                {
                    dObj->writePtr = bufferObj->buffer;
                    dObj->ewState = DRV_SST26_EW_ERASE_SECTOR;

                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                    break;
                }

                /* Fall through for read operation. */
            }

        case DRV_SST26_EW_READ_SECTOR:
            {
                cmdStatus = DRV_SST26_HandleRead (dObj, dObj->ewBuffer, dObj->sectorAddress * 4096, 4096);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    /* Find the offset from which the data is to be overlaid. */
                    dObj->blockOffsetInSector <<= 8;
                    memcpy ((void *)&dObj->ewBuffer[dObj->blockOffsetInSector], (const void *)bufferObj->buffer, dObj->nBlocksToWrite << 8);
                    dObj->ewState = DRV_SST26_EW_ERASE_SECTOR;

                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }
                break;
            }

        case DRV_SST26_EW_ERASE_SECTOR:
            {
                cmdStatus = DRV_SST26_HandleErase(dObj, NULL, dObj->sectorAddress, 1);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    dObj->ewState = DRV_SST26_EW_WRITE_SECTOR;

                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }
                break;
            }

        case DRV_SST26_EW_WRITE_SECTOR:
            {
                cmdStatus = DRV_SST26_HandleWrite (dObj, dObj->writePtr, dObj->sectorAddress * 16, 16);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
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
                    bufferObj->buffer += (dObj->nBlocksToWrite << 8);
                    dObj->ewState = DRV_SST26_EW_INIT;

                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }
    }

    return cmdStatus;
}

// *****************************************************************************
// *****************************************************************************
// Section: SST26 Driver System Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SST26_Initialize
    (
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init
    )

  Summary:
    Initializes the SST26 instance for the specified driver index

  Description:
    This routine initializes the SST26 driver instance for the specified
    driver index, making it ready for clients to open and use it.

  Remarks:
    Refer to drv_sst26.h for usage information.
*/

SYS_MODULE_OBJ DRV_SST26_Initialize
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT *const init
)
{
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;
    DRV_SST26_OBJECT *dObj = (DRV_SST26_OBJECT*) NULL;
    DRV_SST26_INIT *sst26Init = NULL;

    /* Validate the driver index */
    if (drvIndex > DRV_SST26_INSTANCES_NUMBER)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Check if the instance has already been initialized. */
    if (gDrvSST26Obj[drvIndex].inUse)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Assign to the local pointer the init data passed */
    sst26Init = (DRV_SST26_INIT *)init;

    retVal = OSAL_MUTEX_Create(&sst26ClientObjMutex);
    if (retVal != OSAL_RESULT_TRUE)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    dObj = &gDrvSST26Obj[drvIndex];
    if (OSAL_MUTEX_Create(&dObj->mutex) != OSAL_RESULT_TRUE)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Indicate tha this object is in use */
    dObj->inUse = true;

    /* Initialize number of clients */
    dObj->numClients = 0;

    /* Set the driver state as busy as the Flash needs to be initialized and
     * queried for the SFDP data. */
    dObj->status = SYS_STATUS_BUSY;
    dObj->sqiDevice = sst26Init->sqiDevice;

    /* Set the erase buffer */
    dObj->ewBuffer = &gDrvSST26EraseBuffer[drvIndex][0];

    dObj->subState = DRV_SST26_SUBTASK_CMD;
    dObj->state = DRV_SST26_OPEN_SQI;

    dObj->transferElement = &gSst26XferElement[0];
    dObj->cmdParams = &gSst26CmdParams[0];
    dObj->flashId = &gSst26FlashId[0];
    dObj->statusReg = &gSst26StatusReg;
    dObj->xferFrame = &gSst26XferFrame[0];
    dObj->blockStartAddress = 0;

    DRV_SST26_RegisterWithSysFs (drvIndex, drvIndex, sst26MediaFunctions);

    /* Return the driver index */
    return drvIndex;
}

// ****************************************************************************
/* Function:
    void DRV_SST26_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the SST26 driver module

  Description:
    Deinitializes the specified instance of the SST26 driver module,
    disabling its operation (and any hardware). Invalidates all the
    internal data.

  Remarks:
    Refer to drv_sst26.h for usage information.
*/

void DRV_SST26_Deinitialize
(
    SYS_MODULE_OBJ object
)
{
    DRV_SST26_OBJECT * dObj = (DRV_SST26_OBJECT*)NULL;

    /* Validate the object */
    if ((object == SYS_MODULE_OBJ_INVALID) || (object >= DRV_SST26_INSTANCES_NUMBER))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO,"DRV_SST26_Deinitialize(): Invalid parameter.\n");
        return;
    }

    dObj = (DRV_SST26_OBJECT*)&gDrvSST26Obj[object];

    /* Reset the client count and the exclusive flag */
    dObj->numClients = 0;
    dObj->isExclusive = false;

    /* Reset the queue */
    dObj->queue = NULL;

    /* Set the Hardware instance object status an un-initialized */
    dObj->status = SYS_STATUS_UNINITIALIZED;
    dObj->state = DRV_SST26_IDLE;

    /* Hardware instance object is no more in use */
    dObj->inUse = false;

    OSAL_MUTEX_Delete(&sst26ClientObjMutex);
    OSAL_MUTEX_Delete(&dObj->mutex);

    SYS_DEBUG_PRINT(SYS_ERROR_INFO,"DRV_SST26_Deinitialize(): Completed.\n");
}

// ****************************************************************************
/* Function:
    SYS_STATUS DRV_SST26_Status( SYS_MODULE_OBJ object )

  Summary:
    Gets the current status of the SST26 driver module.

  Description:
    This routine provides the current status of the SST26 driver module.

  Remarks:
    Refer to drv_sst26.h for usage information.
*/

SYS_STATUS DRV_SST26_Status
(
    SYS_MODULE_OBJ object
)
{
    /* Validate the object */
    if ((object == SYS_MODULE_OBJ_INVALID) || (object >= DRV_SST26_INSTANCES_NUMBER))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO,"DRV_SST26_Status(): Invalid parameter.\n");
        return SYS_STATUS_UNINITIALIZED;
    }

    /* Return the driver status */
    return (gDrvSST26Obj[object].status);
}

// *****************************************************************************
// *****************************************************************************
// Section: SST26 Driver Client Routines
// *****************************************************************************
// *****************************************************************************

// ****************************************************************************
/* Function:
    DRV_HANDLE DRV_SST26_Open
    ( 
        const SYS_MODULE_INDEX index,
        const DRV_IO_INTENT ioIntent
    )
    
  Summary:
    Opens the specified SST26 driver instance and returns a handle to it
  
  Description:
    This routine opens the specified SST26 driver instance and provides a handle. 
    This handle must be provided to all other client-level operations to identify
    the caller and the instance of the driver.
  
  Remarks:
    Refer to drv_sst26.h for usage information.
*/

DRV_HANDLE DRV_SST26_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
    DRV_SST26_CLIENT_OBJECT *clientObj = NULL;
    DRV_SST26_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;;

    /* Validate the driver index */
    if (drvIndex >= DRV_SST26_INSTANCES_NUMBER)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Open(): Invalid driver index.\n");
        return DRV_HANDLE_INVALID;
    }

    dObj = &gDrvSST26Obj[drvIndex];
    /* Check if the driver is ready to be opened */
    if (dObj->status != SYS_STATUS_READY)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Open(): Driver is not ready.\n");
        return DRV_HANDLE_INVALID;
    }

    /* Check if the driver has already been opened in exclusive mode */
    if (dObj->isExclusive)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Open(): Driver is already open in exclusive mode.\n");
        return DRV_HANDLE_INVALID;
    }

    /* Driver has already been opened and cannot be opened exclusively */
    if ((dObj->numClients > 0) && (ioIntent & DRV_IO_INTENT_EXCLUSIVE))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Open(): Driver is already open. Can't be opened in exclusive mode.\n");
        return DRV_HANDLE_INVALID;
    }

    /* Obtain the Client object mutex */
    retVal = OSAL_MUTEX_Lock(&sst26ClientObjMutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Open(): Failed to acquire the client object mutex.\n");
        return DRV_HANDLE_INVALID;
    }

    clientObj = DRV_SST26_AllocateClientObject ();

    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Open(): Failed to allocate a Client Object.\n");
    }
    else
    {
        /* Found a client object that can be used */
        clientObj->inUse = true;
        clientObj->driverObj = dObj;
        clientObj->intent = ioIntent;
        clientObj->eventHandler = NULL;

        if (ioIntent & DRV_IO_INTENT_EXCLUSIVE)
        {
            /* Driver was opened in exclusive mode */
            dObj->isExclusive = true;
        }

        dObj->numClients ++;
        
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Open(): Open successful.\n");
    }

    OSAL_MUTEX_Unlock(&sst26ClientObjMutex);

    return clientObj ? ((DRV_HANDLE)clientObj) : DRV_HANDLE_INVALID;
}

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_SST26_Close( DRV_Handle handle )

  Summary:
    Closes an opened-instance of the SST26 driver

  Description:
    This routine closes an opened-instance of the SST26 driver, invalidating the
    handle.

  Remarks:
    Refer to drv_sst26.h for usage infomration.
*/

void DRV_SST26_Close
(
    const DRV_HANDLE handle
)
{
    DRV_SST26_CLIENT_OBJECT *clientObj = NULL;
    DRV_SST26_OBJECT *dObj = NULL;

    /* Get the Client object from the handle passed */
    clientObj = DRV_SST26_ClientHandleValidate(handle);
    /* Check if the driver handle is valid */
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_SST26_Close(): Invalid handle.\n");
        return;
    }

    dObj = clientObj->driverObj;
    DRV_SST26_RemoveClientBufferObjects (clientObj, dObj);

    /* Update the client count */
    dObj->numClients --;
    dObj->isExclusive = false;

    /* Free the Client Instance */
    clientObj->inUse = false;

    SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_SST26_Close(): Close successful.\n");
    return;
}

// *****************************************************************************
/* Function:
    void DRV_SST26_Read
    (
        const DRV_HANDLE handle,
        DRV_SST26_COMMAND_HANDLE * commandHandle,
        void * targetBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    )

  Summary:
    Reads blocks of data from the specified address in memory.

  Description:
    This routine reads a block of data from the specified address in memory.
    This operation is non blocking and returns with the required data in the
    target buffer. This function should not be used to read areas of memory 
    which are queued to be programmed or erased. If required, the program or
    erase operations should be allowed to complete. The function returns
    DRV_SST26_COMMAND_HANDLE_INVALID in the commandHandle argument under the 
    following circumstances:
    - if the driver handle is invalid
    - if the target buffer pointer is NULL
    - if the number of blocks to be read is zero or more than the actual number
      of blocks available
    - if a buffer object could not be allocated to the request
    - if the client opened the driver in write only mode

  Remarks:
    Refer to drv_sst26.h for usage information.
*/

void DRV_SST26_Read
(
    const DRV_HANDLE handle,
    DRV_SST26_COMMAND_HANDLE *commandHandle,
    void *targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SST26_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SST26_CLIENT_OBJECT *clientObj = NULL;
    DRV_SST26_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    DRV_SST26_BUFFER_OBJECT *bufferObj = NULL;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SST26_COMMAND_HANDLE_INVALID;

    /* Validate the driver handle */
    clientObj = DRV_SST26_ClientHandleValidate(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Read(): Invalid driver handle.\n");
        return;
    }

    /* Check if the driver was opened with read intent */
    if (!(clientObj->intent & DRV_IO_INTENT_READ))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Read(): Opened with non-read intent.\n");
        return;
    }

    dObj = clientObj->driverObj;

    if ((targetBuffer == NULL) || (nBlock == 0) || ((blockStart + nBlock) > dObj->mediaGeometryTable[DRV_SST26_GEOMETRY_TABLE_READ_ENTRY].numBlocks))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Read(): Invalid parameters.\n");
        return;
    }

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Read(): Failed to acquire the driver object mutex.\n");
        return;
    }

    bufferObj = DRV_SST26_AllocateBufferObject (clientObj, targetBuffer, blockStart, nBlock, DRV_SST26_OPERATION_TYPE_READ);
    if (bufferObj != NULL)
    {
        *tempHandle1 = bufferObj->commandHandle;

        /* Add the request to the queue. */
        DRV_SST26_AddToQueue (dObj, bufferObj);
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);

    return;
}

// *****************************************************************************
/* Function:
    void DRV_SST26_Write
    (
        const DRV_HANDLE handle,
        DRV_SST26_COMMAND_HANDLE * commandHandle,
        void * sourceBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    )

  Summary:
    Writes blocks of data starting from the specified address in flash memory.

  Description:
    This function schedules a non-blocking write operation for writing blocks
    of data into flash memory. The function returns with a valid buffer handle
    in the commandHandle argument if the write request was scheduled successfully.
    The function adds the request to the hardware instance queue and returns
    immediately. While the request is in the queue, the application buffer is
    owned by the driver and should not be modified. The function returns 
    DRV_SST26_COMMAND_HANDLE_INVALID in the commandHandle argument under the 
    following circumstances:
    - if a buffer object could not be allocated to the request
    - if the source buffer pointer is NULL
    - if the client opened the driver for read only
    - if the number of blocks to be written is either zero or more than the number
      of blocks actually available
    - if the write queue size is full or queue depth is insufficient
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_SST26_EVENT_COMMAND_COMPLETE event if the
    buffer was processed successfully or DRV_SST26_EVENT_COMMAND_ERROR
    event if the buffer was not processed successfully.

  Remarks:
    Refer to drv_sst26.h for usage information.
*/

void DRV_SST26_Write
(
    const DRV_HANDLE handle,
    DRV_SST26_COMMAND_HANDLE *commandHandle,
    void *sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SST26_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SST26_CLIENT_OBJECT *clientObj = NULL;
    DRV_SST26_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    DRV_SST26_BUFFER_OBJECT *bufferObj = NULL;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SST26_COMMAND_HANDLE_INVALID;

    /* Validate the driver handle */
    clientObj = DRV_SST26_ClientHandleValidate(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Write(): Invalid driver handle.\n");
        return;
    }

    /* Check if the driver was opened with write intent */
    if (!(clientObj->intent & DRV_IO_INTENT_WRITE))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Write(): Opened with non-write intent.\n");
        return;
    }

    dObj = clientObj->driverObj;
    if ((sourceBuffer == NULL) || (nBlock == 0) || ((blockStart + nBlock) > dObj->mediaGeometryTable[DRV_SST26_GEOMETRY_TABLE_WRITE_ENTRY].numBlocks))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Write(): Invalid parameters.\n");
        return;
    }

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Write(): Failed to acquire the driver object mutex.\n");
        return;
    }

    bufferObj = DRV_SST26_AllocateBufferObject (clientObj, sourceBuffer, blockStart, nBlock, DRV_SST26_OPERATION_TYPE_WRITE);
    if (bufferObj != NULL)
    {
        *tempHandle1 = bufferObj->commandHandle;

        /* Add the request to the queue. */
        DRV_SST26_AddToQueue (dObj, bufferObj);
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);
    return;
}

// **************************************************************************
/* Function:
    void DRV_SST26_Erase
    (
        const DRV_HANDLE handle,
        DRV_SST26_COMMAND_HANDLE * commandHandle,
        uint32_t blockStart,
        uint32_t nBlock
    )
    
  Summary:
    Erase the specified number of blocks of the Flash memory.
  
  Description:
    This function schedules a non-blocking erase operation of flash memory. The
    function returns with a valid erase handle in the commandHandle argument if
    the erase request was scheduled successfully. The function adds the request
    to the hardware instance queue and returns immediately. The function returns
    DRV_SST26_COMMAND_HANDLE_INVALID in the commandHandle argument under the
    following circumstances:
    - if a buffer object could not be allocated to the request
    - if the client opened the driver for read only
    - if the number of blocks to be erased is either zero or more than the number
      of blocks actually available
    - if the erase queue size is full or queue depth is insufficient
    - if the driver handle is invalid 
    
    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_SST26_EVENT_COMMAND_COMPLETE event if the
    erase operation was successful or DRV_SST26_EVENT_COMMAND_ERROR
    event if the erase operation was not successful.

  Remarks:
    Refer to drv_sst26.h for usage information.
*/

void DRV_SST26_Erase
(
    const DRV_HANDLE handle,
    DRV_SST26_COMMAND_HANDLE *commandHandle,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SST26_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SST26_CLIENT_OBJECT *clientObj = NULL;
    DRV_SST26_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    DRV_SST26_BUFFER_OBJECT *bufferObj = NULL;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SST26_COMMAND_HANDLE_INVALID;

    /* Validate the driver handle */
    clientObj = DRV_SST26_ClientHandleValidate(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Erase(): Invalid driver handle.\n");
        return;
    }

    /* Check if the driver was opened with write intent */
    if (!(clientObj->intent & DRV_IO_INTENT_WRITE))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Erase(): Opened with non-write intent.\n");
        return;
    }

    dObj = clientObj->driverObj;
    if ((nBlock == 0) || ((blockStart + nBlock) > dObj->mediaGeometryTable[DRV_SST26_GEOMETRY_TABLE_ERASE_ENTRY].numBlocks))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Erase(): Invalid parameters.\n");
        return;
    }

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Erase(): Failed to acquire the driver object mutex.\n");
        return;
    }

    bufferObj = DRV_SST26_AllocateBufferObject (clientObj, NULL, blockStart, nBlock, DRV_SST26_OPERATION_TYPE_ERASE);
    if (bufferObj != NULL)
    {
        *tempHandle1 = bufferObj->commandHandle;

        /* Add the request to the queue. */
        DRV_SST26_AddToQueue (dObj, bufferObj);
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);

    return;
}

void DRV_SST26_EraseWrite
(
    const DRV_HANDLE handle,
    DRV_SST26_COMMAND_HANDLE *commandHandle,
    void *sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SST26_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SST26_CLIENT_OBJECT *clientObj = NULL;
    DRV_SST26_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    DRV_SST26_BUFFER_OBJECT *bufferObj = NULL;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SST26_COMMAND_HANDLE_INVALID;

    /* Validate the driver handle */
    clientObj = DRV_SST26_ClientHandleValidate(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_EraseWrite(): Invalid driver handle.\n");
        return;
    }

    /* Check if the driver was opened with write intent */
    if (!(clientObj->intent & DRV_IO_INTENT_WRITE))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_EraseWrite(): Opened with non-write intent.\n");
        return;
    }

    dObj = clientObj->driverObj;
    if ((sourceBuffer == NULL) || (nBlock == 0) || ((blockStart + nBlock) > dObj->mediaGeometryTable[DRV_SST26_GEOMETRY_TABLE_WRITE_ENTRY].numBlocks))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_EraseWrite(): Invalid parameters.\n");
        return;
    }

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_EraseWrite(): Failed to acquire the driver object mutex.\n");
        return;
    }

    bufferObj = DRV_SST26_AllocateBufferObject (clientObj, sourceBuffer, blockStart, nBlock, DRV_SST26_OPERATION_TYPE_ERASE_WRITE);
    if (bufferObj != NULL)
    {
        *tempHandle1 = bufferObj->commandHandle;

        /* Add the request to the queue. */
        DRV_SST26_AddToQueue (dObj, bufferObj);
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);

    return;
}

// *****************************************************************************
/* Function:
    DRV_SST26_COMMAND_STATUS DRV_SST26_CommandStatus
    (
        const DRV_HANDLE handle, 
        const DRV_SST26_COMMAND_HANDLE commandHandle
    );

  Summary:
    Gets the current status of the command.

  Description:
    This routine gets the current status of the buffer. The application must use
    this routine where the status of a scheduled buffer needs to polled on. The
    function may return DRV_SST26_COMMAND_HANDLE_INVALID in a case where the buffer
    handle has expired. A buffer handle expires when the internal buffer object
    is re-assigned to another erase or write request. It is recommended that this
    function be called regularly in order to track the buffer status correctly.

    The application can alternatively register an event handler to receive write
    or erase operation completion events.

  Remarks:
    Refer to drv_sst26.h for usage information.
*/

DRV_SST26_COMMAND_STATUS DRV_SST26_CommandStatus
(
    const DRV_HANDLE handle,
    const DRV_SST26_COMMAND_HANDLE commandHandle
)
{
    uint16_t iEntry;

    /* Validate the client handle */
    if (DRV_SST26_ClientHandleValidate(handle) == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_CommandStatus(): Invalid driver handle.\n");
        return DRV_SST26_COMMAND_ERROR_UNKNOWN;
    }

    /* The upper 16 bits of the buffer handle are the token and the lower 16
     * bits of the are buffer index into the gDrvSst26BufferObject array */
    iEntry = commandHandle & 0xFFFF;

    /* Compare the buffer handle with buffer handle in the object */
    if(gDrvSst26BufferObject[iEntry].commandHandle != commandHandle)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_CommandStatus(): Handle has been reused.\n");
        /* This means that object has been re-used by another request. Indicate
         * that the operation is completed.  */
        return (DRV_SST26_COMMAND_COMPLETED);
    }

    /* Return the last known buffer object status */
    return (gDrvSst26BufferObject[iEntry].status);
}

// ****************************************************************************
/* Function:
    void DRV_SST26_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's erase and write state machine and implements its
    ISR.

  Description:
    This routine is used to maintain the driver's internal write and erase state
    machine and implement its ISR for interrupt-driven implementations.

  Remarks:
    Refer to drv_sst26.h for usage information.
*/

void DRV_SST26_Tasks
(
    SYS_MODULE_OBJ object
)
{
    DRV_SST26_OBJECT *dObj = NULL;
    DRV_SST26_CLIENT_OBJECT *clientObj = NULL;
    DRV_SST26_BUFFER_OBJECT *bufferObj = NULL;
    DRV_SST26_EVENT event = DRV_SST26_EVENT_COMMAND_ERROR;
    bool done = false;
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    if(SYS_MODULE_OBJ_INVALID == object)
    {
        /* Invalid system object */
        return;
    }

    dObj = &gDrvSST26Obj[object];

    if (OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
    {
        return;
    }

    switch (dObj->state)
    {
        case DRV_SST26_OPEN_SQI:
            {
                dObj->sqiHandle = DRV_SQI_Open (0, DRV_IO_INTENT_READWRITE);
                if (dObj->sqiHandle != DRV_HANDLE_INVALID)
                {
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_Tasks(): Opened the SQI driver successfully.\n");
                    dObj->state = DRV_SST26_RESET_FLASH;
                    dObj->subState = DRV_SST26_SUBTASK_RESET_ENABLE_CMD;
                }
                break;
            }

        case DRV_SST26_RESET_FLASH:
            {
                cmdStatus = DRV_SST26_ResetFlash(dObj);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    /* Flash Reset done. */
                    dObj->state = DRV_SST26_ENABLE_QUAD_IO;
                    dObj->subState = DRV_SST26_SUBTASK_ENABLE_QUAD_IO;
                }
                else if (cmdStatus == DRV_SQI_COMMAND_ERROR_UNKNOWN)
                {
                    dObj->state = DRV_SST26_ERROR;
                }
                else
                {
                    /* Continue to remain in the same state. */
                }
                break;
            }

        case DRV_SST26_ENABLE_QUAD_IO:
            {
                cmdStatus = DRV_SST26_EnableQuadIO(dObj);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    /* Enable QUAD IO completed. */
                    dObj->state = DRV_SST26_READ_FLASH_ID;
                    dObj->subState = DRV_SST26_SUBTASK_JEDEC_ID_READ_CMD;
                }
                else if (cmdStatus == DRV_SQI_COMMAND_ERROR_UNKNOWN)
                {
                    dObj->state = DRV_SST26_ERROR;
                }
                else
                {
                    /* Continue to remain in the same state. */
                }
                break;
            }
        case DRV_SST26_READ_FLASH_ID:
            {
                cmdStatus = DRV_SST26_ReadFlashId (dObj);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    if (DRV_SST26_UpdateGeometry (dObj, dObj->flashId[2]))
                    {
                        dObj->state = DRV_SST26_UNLOCK_FLASH;
                        dObj->subState = DRV_SST26_SUBTASK_UNLOCK_FLASH_WRITE_ENABLE;
                        dObj->writeEnableState = DRV_SST26_WRITE_ENABLE_CMD;
                    }
                    else
                    {
                        /* Unsupported device Id */
                        dObj->state = DRV_SST26_ERROR;
                    }
                }
                else if (cmdStatus == DRV_SQI_COMMAND_ERROR_UNKNOWN)
                {
                    dObj->state = DRV_SST26_ERROR;
                }
                else
                {
                    /* Continue to remain in the same state. */
                }
                break;
            }

        case DRV_SST26_UNLOCK_FLASH:
            {
                cmdStatus = DRV_SST26_UnlockFlash(dObj);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    /* Unlock of flash completed. */
                    dObj->state = DRV_SST26_PROCESS_QUEUE;
                    dObj->status = SYS_STATUS_READY;
                }
                else if (cmdStatus == DRV_SQI_COMMAND_ERROR_UNKNOWN)
                {
                    dObj->state = DRV_SST26_ERROR;
                }
                else
                {
                    /* Continue to remain in the same state. */
                }
                break;
            }

        case DRV_SST26_PROCESS_QUEUE:
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
                    dObj->eraseState = DRV_SST26_ERASE_INIT;
                    dObj->writeState = DRV_SST26_WRITE_INIT;
                    dObj->ewState = DRV_SST26_EW_INIT;
                    dObj->subState = DRV_SST26_SUBTASK_CMD;

                    dObj->state = DRV_SST26_TRANSFER;
                }
            }

        case DRV_SST26_TRANSFER:
            {
                bufferObj = dObj->currentBufObj;
                cmdStatus = gSst26XferFuncPtr[bufferObj->opType](dObj, &bufferObj->buffer[0], bufferObj->blockStart, bufferObj->nBlocks);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    bufferObj->status = DRV_SST26_COMMAND_COMPLETED;
                    /* The operation has completed. */
                    dObj->state = DRV_SST26_PROCESS_QUEUE;
                    done = true;
                    event = DRV_SQI_EVENT_COMMAND_COMPLETE;
                }
                else if (cmdStatus == DRV_SQI_COMMAND_ERROR_UNKNOWN)
                {
                    /* The operation has failed. */
                    bufferObj->status = DRV_SST26_COMMAND_ERROR_UNKNOWN;
                    dObj->state = DRV_SST26_ERROR;
                    done = true;
                    event = DRV_SQI_EVENT_COMMAND_ERROR;
                }
                else
                {
                    /* Continue to remain in the same state. */
                }

                if (done)
                {
                    bufferObj->inUse = false;
                    DRV_SST26_UpdateQueue (dObj);

                    clientObj = (DRV_SST26_CLIENT_OBJECT *)bufferObj->hClient;
                    if(clientObj->eventHandler != NULL)
                    {
                        /* Call the event handler */
                        clientObj->eventHandler(event, bufferObj->commandHandle, clientObj->context);
                    }
                }
                break;
            }

           case DRV_SST26_IDLE:
            {
                break;
            }

           case DRV_SST26_ERROR:
           default:
            {
                break;
            }
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);
}

// *****************************************************************************
/* Function:
    void DRV_SST26_EventHandlerSet
    (
        const DRV_HANDLE handle,
        const void * eventHandler,
        const uintptr_t context
    );

  Summary:
    Allows a client to identify an event handling function for the driver to
    call back when queued operation has completed.

  Description:
    This function allows a client to identify an event handling function for
    the driver to call back when queued operation has completed. When a client
    calls a write or erase function, it is provided with a handle identifying
    the buffer that was added to the driver's buffer queue. The driver will 
    pass this handle back to the client by calling "eventHandler" function when
    the queued operation has completed.
    
    The event handler should be set before the client performs any write or erase
    operations that could generate events. The event handler once set, persists
    until the client closes the driver or sets another event handler (which could
    be a "NULL" pointer to indicate no callback).

  Remarks:
    Refer to drv_sst26.h for usage information.
*/

void DRV_SST26_EventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
)
{
    DRV_SST26_CLIENT_OBJECT *clientObj = NULL;

    clientObj = DRV_SST26_ClientHandleValidate(handle);
    /* Check if the client handle is valid */
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_EventHandlerSet(): Invalid driver handle.\n");
        return;
    }

    /* Set the event handler */
    clientObj->eventHandler = eventHandler;
    clientObj->context = context;
}

// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_GEOMETRY * DRV_SST26_GeometryGet( const DRV_HANDLE handle );

  Summary:
    Returns the geometry of the device.

  Description:
    This API gives the following geometrical details of the SST26 Flash:
    - Media Property
    - Number of Read/Write/Erase regions in the flash device
    - Number of Blocks and their size in each region of the device

  Remarks:
    Refer to drv_sst26.h for usage information.
*/

SYS_FS_MEDIA_GEOMETRY * DRV_SST26_GeometryGet
(
    const DRV_HANDLE handle
)
{
    DRV_SST26_CLIENT_OBJECT *clientObj = NULL;
    DRV_SST26_OBJECT *dObj = NULL;

    /* Get the Client object from the handle passed */
    clientObj = DRV_SST26_ClientHandleValidate(handle);
    /* Check if the driver handle is valid */
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_GeometryGet(): Invalid driver handle.\n");
        return NULL;
    }

    dObj = clientObj->driverObj;
    return &dObj->mediaGeometryObj;
}

// *****************************************************************************
/* Function:
    bool DRV_SST26_isAttached( const DRV_HANDLE handle );

  Summary:
    Returns the physical attach status of the SST26.

  Description:
    This function returns the physical attach status of the SST26. This
    function returns false if the driver handle is invalid otherwise returns
    true.

  Remarks:
    Refer to drv_sst26.h for usage information.
*/

bool DRV_SST26_IsAttached
(
    const DRV_HANDLE handle
)
{
    /* Validate the driver handle */
    if (DRV_SST26_ClientHandleValidate(handle) == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_SST26_IsAttached(): Invalid driver handle.\n");
        return false;
    }

   return true;
}

// *****************************************************************************
/* Function:
    bool DRV_SST26_isWriteProtected( const DRV_HANDLE handle );

  Summary:
    Returns the write protect status of SST26.

  Description:
    This function returns the write protect status of the SST26. Always returns
    false.

  Remarks:
    Refer to drv_sst26.h for usage information.
*/

bool DRV_SST26_IsWriteProtected
(
    const DRV_HANDLE handle
)
{
    /* This function always returns false */
    return false;
}

// *****************************************************************************
/* Function:
    uintptr_t DRV_SST26_AddressGet
    (
        const DRV_HANDLE handle
    );

  Summary:
    Returns the SST26 media start address

  Description:
    This function returns the SST26 Media start address.

  Remarks:
    None.
*/

uintptr_t DRV_SST26_AddressGet
(
    const DRV_HANDLE handle
)
{
    DRV_SST26_CLIENT_OBJECT *clientObj = NULL;
    DRV_SST26_OBJECT *dObj = NULL;

    /* Validate the handle */
    if (handle == DRV_HANDLE_INVALID)
    {
        SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_SST26_AddressGet(): Invalid driver handle.\n");
        return (uintptr_t)NULL;
    }

    /* See if the client has been opened */
    clientObj = (DRV_SST26_CLIENT_OBJECT *)handle;
    if (clientObj->inUse == false)
    {
        SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_SST26_AddressGet(): Invalid client.\n");
        return (uintptr_t)NULL;
    }

    /* Check if the driver is ready for operation */
    dObj = (DRV_SST26_OBJECT *)clientObj->driverObj;
    if (dObj->status != SYS_STATUS_READY)
    {
        SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_SST26_AddressGet(): Driver is not ready.\n");
        return (uintptr_t)NULL;
    }

    return dObj->blockStartAddress;
}

