/******************************************************************************
  S25FL Driver Interface Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_s25fl.c

  Summary:
    S25FL Driver Interface Definition

  Description:
    The S25FL Driver provides a set of interfaces to access the S25FL SQI Flash 
    connected to the PIC32 microcontroller. This file implements the S25FL 
    Driver interface. This file should be included in the project if S25FL 
    driver functionality is needed.
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

#include "driver/sqi_flash/s25fl/src/drv_s25fl_local.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global objects
// *****************************************************************************
// *****************************************************************************

/*************************************************
 * Hardware instance objects
 *************************************************/

DRV_S25FL_OBJECT gDrvS25FLObj[DRV_S25FL_INSTANCES_NUMBER];

/*************************************************
 * Driver Client Objects
 *************************************************/

DRV_S25FL_CLIENT_OBJECT gDrvS25flClientObj[DRV_S25FL_CLIENTS_NUMBER];

/*************************************************
 * Driver Buffer Objects.
 *************************************************/

DRV_S25FL_BUFFER_OBJECT gDrvS25flBufferObject[DRV_S25FL_BUFFER_OBJECT_NUMBER];

DRV_SQI_TransferFrame gS25flXferFrame[4];
uint8_t gS25flFlashId[4];
uint8_t gS25flStatusReg[3];

/************************************************
 * This token is incremented for every request added to the queue and is used
 * to generate a different buffer handle for every request.
 ***********************************************/

uint16_t gDrvS25flBufferToken = 0;

/**************************************************
 * Erase buffer size in case the erase write feature is enabled
 **************************************************/
#define DRV_S25FL_ERASE_BUFFER_SIZE (4096)
uint8_t gDrvS25FLEraseBuffer[DRV_S25FL_INSTANCES_NUMBER][DRV_S25FL_ERASE_BUFFER_SIZE];

/*************************************************
 * OSAL Declarations
 *************************************************/
/* S25FL Client Object Mutex */
OSAL_MUTEX_DECLARE(s25flClientObjMutex);

/* FS Function registration table. */
const SYS_FS_MEDIA_FUNCTIONS s25flMediaFunctions =
{
    .mediaStatusGet     = DRV_S25FL_IsAttached,
    .mediaGeometryGet   = DRV_S25FL_GeometryGet,
    .sectorRead         = DRV_S25FL_Read,
    .sectorWrite        = DRV_S25FL_EraseWrite,
    .eventHandlerset    = DRV_S25FL_EventHandlerSet,
    .commandStatusGet   = (void *)DRV_S25FL_CommandStatus,
    .Read               = DRV_S25FL_Read,
    .erase              = DRV_S25FL_Erase,
    .addressGet         = DRV_S25FL_AddressGet,
    .open               = DRV_S25FL_Open,
    .close              = DRV_S25FL_Close,
    .tasks              = DRV_S25FL_Tasks,
};

/* Table mapping the Flash ID's to their sizes. */
uint32_t gSstFlashIdSizeTable [3][2] = {
    {0x15, 0x200000}, /* 16 MBit */    
    {0x16, 0x400000}, /* 32 MBit */    
    {0x17, 0x800000}  /* 64 MBit */
};

static DRV_SQI_COMMAND_STATUS DRV_S25FL_HandleRead
(
    DRV_S25FL_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
);

static DRV_SQI_COMMAND_STATUS DRV_S25FL_HandleWrite
(
    DRV_S25FL_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
);

static DRV_SQI_COMMAND_STATUS DRV_S25FL_HandleErase
(
    DRV_S25FL_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
);
static DRV_SQI_COMMAND_STATUS DRV_S25FL_HandleEraseWrite
(
    DRV_S25FL_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
);

DRV_S25FL_TransferOperation gS25flXferFuncPtr[4] =
{
    DRV_S25FL_HandleRead,
    DRV_S25FL_HandleWrite,
    DRV_S25FL_HandleErase,
    DRV_S25FL_HandleEraseWrite,
};

// *****************************************************************************
// *****************************************************************************
// Section: S25FL Driver Local Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    static DRV_S25FL_BUFFER_OBJECT* DRV_S25FL_AllocateBufferObject
    (
        DRV_S25FL_CLIENT_OBJECT *clientObj,
        void *buffer,
        uint32_t blockStart,
        uint32_t nBlocks,
        DRV_S25FL_OPERATION_TYPE opType
    );
    
  Summary:
    Allocate a free buffer object for the transfer request.

  Description:
    This function finds a free buffer object and populates it with the transfer
    parameters. It also generates a new command handle for the request.

  Remarks:
    None.
*/
static DRV_S25FL_BUFFER_OBJECT* DRV_S25FL_AllocateBufferObject
(
    DRV_S25FL_CLIENT_OBJECT *clientObj,
    void *buffer,
    uint32_t blockStart,
    uint32_t nBlocks,
    DRV_S25FL_OPERATION_TYPE opType
)
{
    uint8_t iEntry = 0;
    DRV_S25FL_BUFFER_OBJECT *bufferObj = NULL;

    for (iEntry = 0; iEntry < DRV_S25FL_BUFFER_OBJECT_NUMBER; iEntry++)
    {
        /* Search for a free buffer object to use */
        if (gDrvS25flBufferObject[iEntry].inUse == false)
        {
            /* Found a free buffer object. */
            bufferObj = &gDrvS25flBufferObject[iEntry];

            bufferObj->inUse         = true;
            bufferObj->commandHandle = DRV_S25FL_MAKE_HANDLE(gDrvS25flBufferToken, iEntry);
            bufferObj->hClient       = clientObj;
            bufferObj->buffer        = buffer;
            bufferObj->blockStart    = blockStart;
            bufferObj->nBlocks       = nBlocks;
            bufferObj->opType        = opType;
            bufferObj->status        = DRV_SQI_COMMAND_QUEUED;
            bufferObj->next          = NULL;
            bufferObj->previous      = NULL;

            /* Update the token number. */
            DRV_S25FL_UPDATE_BUF_TOKEN(gDrvS25flBufferToken);
            break;
        }
    }

    return bufferObj;
}

// *****************************************************************************
/* Function:
    static DRV_S25FL_CLIENT_OBJECT* DRV_S25FL_AllocateClientObject
    (
        void
    );
    
  Summary:
    Allocate a free client object.

  Description:
    This function finds a free client object and populates it with the client
    parameters.

  Remarks:
    None.
*/
static DRV_S25FL_CLIENT_OBJECT* DRV_S25FL_AllocateClientObject
(
    void
)
{
    uint8_t iClient = 0;
    DRV_S25FL_CLIENT_OBJECT *object = &gDrvS25flClientObj[0];

    /* Find available slot in array of client objects */
    for (iClient = 0; iClient < DRV_S25FL_CLIENTS_NUMBER ; iClient++)
    {
        if (!object->inUse)
        {
            return object;
        }

        object ++;
    }

    return NULL;
}

// *****************************************************************************
/* Function:
   static DRV_S25FL_CLIENT_OBJECT * DRV_S25FL_ClientHandleValidate
    (
        DRV_HANDLE handle
    );
    
  Summary:
    Validate the client handle.

  Description:
    This function validates the driver handle and returns the client object
    pointer associated with the driver handle if the handle is valid. If the
    driver handle is not valid or if the driver is in a not ready state then
    NULL is returned.

  Remarks:
    None.
*/
static DRV_S25FL_CLIENT_OBJECT * DRV_S25FL_ClientHandleValidate
(
    DRV_HANDLE handle
)
{
    DRV_S25FL_CLIENT_OBJECT *clientObj = NULL;
    DRV_S25FL_OBJECT *dObj = NULL;

    /* Validate the handle */
    if (handle == DRV_HANDLE_INVALID)
    {
        return NULL;
    }

    /* See if the client has been opened */
    clientObj = (DRV_S25FL_CLIENT_OBJECT *)handle;
    if (!clientObj->inUse)
    {
        return NULL;
    }

    /* Check if the driver is ready for operation */
    dObj = (DRV_S25FL_OBJECT *)clientObj->driverObj;
    if (dObj->status != SYS_STATUS_READY)
    {
        return NULL;
    }

    return clientObj;
}

// *****************************************************************************
/* Function:
    static uint32_t DRV_S25FL_GetFlashSize
    (
        uint8_t deviceId
    );
    
  Summary:
    Validate the Flash device id with the device id map table.

  Description:
    This function returns the flash size in bytes for the specified deviceId. A
    zero is returned if the device id is not supported.

  Remarks:
    None.
*/
static uint32_t DRV_S25FL_GetFlashSize
(
    uint8_t deviceId
)
{
    uint8_t i = 0;

    for (i = 0; i < 3; i++)
    {
        if (deviceId == gSstFlashIdSizeTable[i][0])
        {
            return gSstFlashIdSizeTable[i][1];
        }
    }

    return 0;
}

// *****************************************************************************
/* Function:
    static bool DRV_S25FL_UpdateGeometry
    (
        DRV_S25FL_OBJECT *dObj,
        uint8_t deviceId
    );

  Summary:
    Update the driver object's geometry information.

  Description:
    This function updates the driver object's geometry information for the flash
    device.

  Remarks:
    None.
*/
static bool DRV_S25FL_UpdateGeometry
(
    DRV_S25FL_OBJECT *dObj,
    uint8_t deviceId
)
{
    uint32_t flashSize = DRV_S25FL_GetFlashSize (deviceId);
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

// *****************************************************************************
/* Function:
    static void DRV_S25FL_AddToQueue
    (
        DRV_S25FL_OBJECT *dObj,
        DRV_S25FL_BUFFER_OBJECT *bufferObj
    );

  Summary:
    Add the transfer request to the driver queue.

  Description:
    This function adds the buffer object to the tail of the queue.

  Remarks:
    None.
*/
static void DRV_S25FL_AddToQueue
(
    DRV_S25FL_OBJECT *dObj,
    DRV_S25FL_BUFFER_OBJECT *bufferObj
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

// *****************************************************************************
/* Function:
    static void DRV_S25FL_UpdateQueue
    (
        DRV_S25FL_OBJECT *dObj
    );

  Summary:
    Update the queue.

  Description:
    This function updates the head of the queue.

  Remarks:
    None.
*/
static void DRV_S25FL_UpdateQueue
(
    DRV_S25FL_OBJECT *dObj
)
{
    DRV_S25FL_BUFFER_OBJECT * bufferObj = dObj->queue;

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

// *****************************************************************************
/* Function:
    static void DRV_S25FL_RemoveClientBufferObjects
    (
        DRV_S25FL_CLIENT_OBJECT *clientObj,
        DRV_S25FL_OBJECT *dObj
    );

  Summary:
    Remove buffer objects associated with a particular client.

  Description:
    This function removes the buffer objects associated with a particular
    client.

  Remarks:
    None.
*/
static void DRV_S25FL_RemoveClientBufferObjects
(
    DRV_S25FL_CLIENT_OBJECT *clientObj,
    DRV_S25FL_OBJECT *dObj
)
{
    DRV_S25FL_BUFFER_OBJECT *bufferObject = NULL;
    DRV_S25FL_BUFFER_OBJECT *lastObject = NULL;
    DRV_S25FL_BUFFER_OBJECT *head = NULL;
    DRV_S25FL_BUFFER_OBJECT *temp = NULL;

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

// *****************************************************************************
/* Function:
    static DRV_SQI_COMMAND_STATUS DRV_S25FL_WriteEnable
    (
        DRV_S25FL_OBJECT *dObj
    );

  Summary:
    Frames and sends the write enable command to the flash.

  Description:
    This function frames and sends the write enable command to the flash and
    also tracks the status of the commmand.

  Remarks:
    None.
*/
static DRV_SQI_COMMAND_STATUS DRV_S25FL_WriteEnable
(
    DRV_S25FL_OBJECT *dObj
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->subState)
    {
        case DRV_S25FL_SUBTASK_CMD:
        default:
            {
                DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                frame->instruction = DRV_S25FL_CMD_WRITE_ENABLE;
                frame->laneCfg = DRV_SQI_LANE_SINGLE;
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
                    dObj->subState = DRV_S25FL_SUBTASK_CMD_STATUS;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_S25FL_SUBTASK_CMD_STATUS:
            {
                cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
                break;
            }
    }

    return cmdStatus;
}

// *****************************************************************************
/* Function:
    static DRV_SQI_COMMAND_STATUS DRV_S25FL_ReadStatus
    (
        DRV_S25FL_OBJECT *dObj,
        uint8_t command,
        uint8_t *statusReg
    );

  Summary:
    Fetches the specified status register of the flash.

  Description:
    This function is used to fetch the contents of the specified status register
    of the flash.

  Remarks:
    None.
*/
static DRV_SQI_COMMAND_STATUS DRV_S25FL_ReadStatus
(
    DRV_S25FL_OBJECT *dObj,
    uint8_t command,
    uint8_t *statusReg
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->subState)
    {
        case DRV_S25FL_SUBTASK_CMD:
        default:
            {
                DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                frame->instruction = command;                
                frame->data = statusReg;
                frame->length = 1;
                frame->laneCfg = DRV_SQI_LANE_SINGLE;
                frame->numDummyBytes = 0;
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
                    dObj->subState = DRV_S25FL_SUBTASK_CMD_STATUS;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_S25FL_SUBTASK_CMD_STATUS:
            {
                cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
                break;
            }
    }

    return cmdStatus;
}

// *****************************************************************************
/* Function:
    static DRV_SQI_COMMAND_STATUS DRV_S25FL_ConfigureFlash
    (
        DRV_S25FL_OBJECT *dObj
    );

  Summary:
    Configure the flash parameters.

  Description:
    This function is used to configure the flash parameters such as block
    protection levels and the quad enable operation mode.

  Remarks:
    None.
*/
static DRV_SQI_COMMAND_STATUS DRV_S25FL_ConfigureFlash
(
    DRV_S25FL_OBJECT *dObj
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->cfgFlashState)
    {
        case DRV_S25FL_CFG_FLASH_READ_STATUS_REG_1:
        default:
            {
                cmdStatus = DRV_S25FL_ReadStatus(dObj, DRV_S25FL_CMD_READ_STATUS_REG, &dObj->statusReg[0]);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    dObj->subState = DRV_S25FL_SUBTASK_CMD;
                    dObj->cfgFlashState = DRV_S25FL_CFG_FLASH_READ_STATUS_REG_2;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }
                break;
        }
        
        case DRV_S25FL_CFG_FLASH_READ_STATUS_REG_2:
            {
                cmdStatus = DRV_S25FL_ReadStatus(dObj, DRV_S25FL_CMD_READ_STATUS_REG_2, &dObj->statusReg[1]);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    dObj->subState = DRV_S25FL_SUBTASK_CMD;
                    dObj->cfgFlashState = DRV_S25FL_CFG_FLASH_READ_STATUS_REG_3;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }
                break;
        }
        
        case DRV_S25FL_CFG_FLASH_READ_STATUS_REG_3:        
            {
                cmdStatus = DRV_S25FL_ReadStatus(dObj, DRV_S25FL_CMD_READ_STATUS_REG_3, &dObj->statusReg[2]);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    dObj->subState = DRV_S25FL_SUBTASK_CMD;
                    dObj->cfgFlashState = DRV_S25FL_CFG_FLASH_WRITE_ENABLE;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }
                break;
        }
        
        case DRV_S25FL_CFG_FLASH_WRITE_ENABLE:
        {
            cmdStatus = DRV_S25FL_WriteEnable(dObj);
            if (cmdStatus == DRV_SQI_COMMAND_COMPLETED) {
                dObj->cfgFlashState = DRV_S25FL_CFG_FLASH_WRITE_STATUS_REG;
                
                /* Set BP2, BP1 and BP0 to zero. */
                dObj->statusReg[0] &= ~(0x07 << 2);
                /* Set Quad Enable to one. */
                dObj->statusReg[1] |=  (1 << 1);
                
                cmdStatus = DRV_SQI_COMMAND_QUEUED;
            }
            break;
        }
        
        case DRV_S25FL_CFG_FLASH_WRITE_STATUS_REG:
        {
                DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                frame->instruction = DRV_S25FL_CMD_WRITE_STATUS_REG;
                frame->data = &dObj->statusReg[0];
                frame->length = 3;
                frame->laneCfg = DRV_SQI_LANE_SINGLE;
                frame->numDummyBytes = 0;
                frame->flags = (DRV_SQI_FLAG_INSTR_ENABLE_MASK | DRV_SQI_FLAG_DATA_ENABLE_MASK |
                        DRV_SQI_FLAG_DATA_TARGET_REGISTER | DRV_SQI_FLAG_DATA_DIRECTION_WRITE);
                frame->flags |= DRV_SQI_FLAG_SQI_CS_NUMBER(dObj->sqiDevice);
                DRV_SQI_TransferFrames (dObj->sqiHandle, &dObj->cmdHandle, frame, 1);
                if (dObj->cmdHandle == DRV_S25FL_COMMAND_HANDLE_INVALID)
                {
                    /* Failed to queue the request. */
                    cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;
                }
                else
                {
                    dObj->cfgFlashState = DRV_S25FL_CFG_FLASH_WRITE_STATUS_REG_STATUS;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_S25FL_CFG_FLASH_WRITE_STATUS_REG_STATUS:
        {
            cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);

            if (cmdStatus == DRV_SQI_COMMAND_COMPLETED) {
                dObj->subState = DRV_S25FL_SUBTASK_CMD;
                dObj->cfgFlashState = DRV_S25FL_CFG_FLASH_STATUS;

                cmdStatus = DRV_SQI_COMMAND_QUEUED;
            }

            break;
        }

        case DRV_S25FL_CFG_FLASH_STATUS:
            {
                cmdStatus = DRV_S25FL_ReadStatus(dObj, DRV_S25FL_CMD_READ_STATUS_REG, &dObj->statusReg[0]);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    dObj->subState = DRV_S25FL_SUBTASK_CMD;

                    if (!(*dObj->statusReg & 0x01))
                    {
                        /* Completed. */
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

// *****************************************************************************
/* Function:
    static DRV_SQI_COMMAND_STATUS DRV_S25FL_ReadFlashId
    (
        DRV_S25FL_OBJECT *dObj
    );

  Summary:
    Reads the Flash Device Id.

  Description:
    This function is used to read the Flash Device Id.

  Remarks:
    None.
*/
static DRV_SQI_COMMAND_STATUS DRV_S25FL_ReadFlashId
(
    DRV_S25FL_OBJECT *dObj
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->subState)
    {
        case DRV_S25FL_SUBTASK_JEDEC_ID_READ_CMD:
        default:
            {
                DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                
                /* JEDEC-ID Read Command */
                frame->instruction = DRV_S25FL_CMD_JEDEC_ID_READ;
                frame->data = &dObj->flashId[0];
                frame->length = 4;
                frame->laneCfg = DRV_SQI_LANE_SINGLE;
                frame->numDummyBytes = 0;
                frame->flags = (DRV_SQI_FLAG_INSTR_ENABLE_MASK | DRV_SQI_FLAG_DATA_ENABLE_MASK |
                        DRV_SQI_FLAG_DATA_TARGET_REGISTER | DRV_SQI_FLAG_DATA_DIRECTION_READ);
                frame->flags |= DRV_SQI_FLAG_SQI_CS_NUMBER(dObj->sqiDevice);
                DRV_SQI_TransferFrames (dObj->sqiHandle, &dObj->cmdHandle, frame, 1);
                if (dObj->cmdHandle == DRV_S25FL_COMMAND_HANDLE_INVALID)
                {
                    /* Failed to queue the request. */
                    cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;
                }
                else
                {
                    dObj->subState = DRV_S25FL_SUBTASK_JEDEC_ID_READ_STATUS;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }
                break;
            }

        case DRV_S25FL_SUBTASK_JEDEC_ID_READ_STATUS:
            {
                cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
                break;
            }
    }

    return cmdStatus;
}

// *****************************************************************************
/* Function:
    static DRV_SQI_COMMAND_STATUS DRV_S25FL_HandleRead
    (
        DRV_S25FL_OBJECT *dObj,
        uint8_t *data,
        uint32_t blockAddress,
        uint32_t nBlocks
    );

  Summary:
    Read data from the flash.

  Description:
    This function is responsible for sending the appropriate instruction opcode
    to the flash to read data from the flash and monitoring the status of the
    transfer request.

  Remarks:
    None.
*/
static DRV_SQI_COMMAND_STATUS DRV_S25FL_HandleRead
(
    DRV_S25FL_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->subState)
    {
        case DRV_S25FL_SUBTASK_CMD:
        default:
            {
                DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                frame->instruction = 0x6B;
                frame->address = blockAddress;
                frame->data = data;
                frame->length = nBlocks;                
                frame->laneCfg = DRV_SQI_LANE_QUAD_DATA;
                frame->numDummyBytes = 8;
                frame->flags = (DRV_SQI_FLAG_INSTR_ENABLE_MASK | DRV_SQI_FLAG_DATA_ENABLE_MASK |
                        DRV_SQI_FLAG_ADDR_ENABLE_MASK | DRV_SQI_FLAG_DATA_TARGET_MEMORY | 
                        DRV_SQI_FLAG_DATA_DIRECTION_READ);
                frame->flags |= DRV_SQI_FLAG_SQI_CS_NUMBER(dObj->sqiDevice);
                DRV_SQI_TransferFrames (dObj->sqiHandle, &dObj->cmdHandle, frame, 1);
                if (dObj->cmdHandle == DRV_SQI_COMMAND_HANDLE_INVALID)
                {
                    cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;
                }
                else
                {
                    dObj->subState = DRV_S25FL_SUBTASK_CMD_STATUS;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_S25FL_SUBTASK_CMD_STATUS:
            {
                cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
                break;
            }
    }

    return cmdStatus;
}

// *****************************************************************************
/* Function:
    static DRV_SQI_COMMAND_STATUS DRV_S25FL_WriteOnePage
    (
        DRV_S25FL_OBJECT *dObj,
        uint32_t address,
        uint8_t *data
    );

  Summary:
    Write one page of to the flash.

  Description:
    This function is responsible for sending the appropriate instruction opcode
    to the flash to perform a write operation of one page.

  Remarks:
    None.
*/
static DRV_SQI_COMMAND_STATUS DRV_S25FL_WriteOnePage
(
    DRV_S25FL_OBJECT *dObj,
    uint32_t address,
    uint8_t *data
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->subState)
    {
        case DRV_S25FL_SUBTASK_CMD:
        default:
            {
                DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                frame->instruction = DRV_S25FL_CMD_PAGE_PROGRAM;
                frame->address = address << 0x08;
                frame->data = data;
                frame->length = 256;
                frame->laneCfg = DRV_SQI_LANE_SINGLE;
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
                    dObj->subState = DRV_S25FL_SUBTASK_CMD_STATUS;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_S25FL_SUBTASK_CMD_STATUS:
            {
                cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
                break;
            }
    }

    return cmdStatus;
}

// *****************************************************************************
/* Function:
    static DRV_SQI_COMMAND_STATUS DRV_S25FL_HandleWrite
    (
        DRV_S25FL_OBJECT *dObj,
        uint8_t *data,
        uint32_t blockAddress,
        uint32_t nBlocks
    );

  Summary:
    Writes the requested data onto the flash.

  Description:
    This function is responsible preparing the flash device for the write
    operation by sending the write enable command, followed by sending opcode
    for writing one page of data and then reading the status register to
    check if the flash's internal programming operation is complete. This
    sequence is performed for each page of data being written.

  Remarks:
    None.
*/
static DRV_SQI_COMMAND_STATUS DRV_S25FL_HandleWrite
(
    DRV_S25FL_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->writeState)
    {
        case DRV_S25FL_WRITE_INIT:
        default:
            {
                dObj->bufferOffset = 0;
                dObj->blockAddress = blockAddress;
                dObj->nBlocks = nBlocks;
                dObj->writePtr = data;

                dObj->subState = DRV_S25FL_SUBTASK_CMD;
                dObj->writeState = DRV_S25FL_WRITE_ENABLE;
                
                /* Fall through to the next state. */
            }

        case DRV_S25FL_WRITE_ENABLE:
        {
            cmdStatus = DRV_S25FL_WriteEnable(dObj);
            if (cmdStatus == DRV_SQI_COMMAND_COMPLETED) {
                dObj->subState = DRV_S25FL_SUBTASK_CMD;
                dObj->writeState = DRV_S25FL_WRITE_CMD;

                cmdStatus = DRV_SQI_COMMAND_QUEUED;
            }

            break;
        }
        case DRV_S25FL_WRITE_CMD:
            {
                cmdStatus = DRV_S25FL_WriteOnePage (dObj, dObj->blockAddress, &dObj->writePtr[dObj->bufferOffset]);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    dObj->subState = DRV_S25FL_SUBTASK_CMD;
                    dObj->writeState = DRV_S25FL_WRITE_CMD_STATUS;

                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_S25FL_WRITE_CMD_STATUS:
            {
                cmdStatus = DRV_S25FL_ReadStatus(dObj, DRV_S25FL_CMD_READ_STATUS_REG, &dObj->statusReg[0]);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    dObj->subState = DRV_S25FL_SUBTASK_CMD;

                    if (!(*dObj->statusReg & 0x01))
                    {
                        dObj->nBlocks --;
                        if (dObj->nBlocks != 0)
                        {
                            /* There is still data to be programmed. */
                            dObj->bufferOffset += 256;
                            /* Update the block address and the write pointer */
                            dObj->blockAddress += 1;
                            dObj->writeState = DRV_S25FL_WRITE_ENABLE;
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

// *****************************************************************************
/* Function:
    static DRV_SQI_COMMAND_STATUS DRV_S25FL_SectorErase
    (
        DRV_S25FL_OBJECT *dObj,
        uint8_t *data,
        uint32_t blockAddress,
        uint32_t nBlocks
    );

  Summary:
    Erases one sector of the flash.

  Description:
    This function is responsible for sending the appropriate instruction opcode
    to the flash to perform a write operation of one page.

  Remarks:
    None.
*/
static DRV_SQI_COMMAND_STATUS DRV_S25FL_SectorErase
(
    DRV_S25FL_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->subState)
    {
        case DRV_S25FL_SUBTASK_CMD:
        default:
            {
                DRV_SQI_TransferFrame *frame = dObj->xferFrame;
                frame->instruction = DRV_S25FL_CMD_SECTOR_ERASE;
                frame->address = blockAddress;
                frame->laneCfg = DRV_SQI_LANE_SINGLE;
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
                    dObj->subState = DRV_S25FL_SUBTASK_CMD_STATUS;
                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }

        case DRV_S25FL_SUBTASK_CMD_STATUS:
            {
                cmdStatus = DRV_SQI_CommandStatus(dObj->sqiHandle, dObj->cmdHandle);
                break;
            }
    }

    return cmdStatus;
}

// *****************************************************************************
/* Function:
    static DRV_SQI_COMMAND_STATUS DRV_S25FL_HandleErase
    (
        DRV_S25FL_OBJECT *dObj,
        uint8_t *data,
        uint32_t blockAddress,
        uint32_t nBlocks
    );

  Summary:
    Erases the requested number of blocks of the flash.

  Description:
    This function is responsible preparing the flash device for the erase
    operation by sending the write enable command, followed by sending opcode
    for erasing one sector and then reading the status register to check if the
    flash's internal erase operation is complete.

  Remarks:
    None.
*/
static DRV_SQI_COMMAND_STATUS DRV_S25FL_HandleErase
(
    DRV_S25FL_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
)
{
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    switch (dObj->eraseState)
    {
        case DRV_S25FL_ERASE_INIT:
        default:
            {
                dObj->blockAddress = blockAddress * 4096;
                dObj->nBlocks = nBlocks;
                dObj->eraseState = DRV_S25FL_ERASE_WRITE_ENABLE;
                dObj->subState = DRV_S25FL_SUBTASK_CMD;
                /* Fall through to the next state. */
            }
        
        case DRV_S25FL_ERASE_WRITE_ENABLE:
        {
            cmdStatus = DRV_S25FL_WriteEnable(dObj);
            if (cmdStatus == DRV_SQI_COMMAND_COMPLETED) {
                dObj->subState = DRV_S25FL_SUBTASK_CMD;
                dObj->eraseState = DRV_S25FL_ERASE_CMD;

                cmdStatus = DRV_SQI_COMMAND_QUEUED;
            }

            break;
        }

        case DRV_S25FL_ERASE_CMD:
            {
                cmdStatus = DRV_S25FL_SectorErase (dObj, NULL, dObj->blockAddress, 0);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    dObj->subState = DRV_S25FL_SUBTASK_CMD;
                    dObj->eraseState = DRV_S25FL_ERASE_CMD_STATUS;

                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }
                break;
            }

        case DRV_S25FL_ERASE_CMD_STATUS:
            {
                cmdStatus = DRV_S25FL_ReadStatus(dObj, DRV_S25FL_CMD_READ_STATUS_REG, &dObj->statusReg[0]);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    dObj->subState = DRV_S25FL_SUBTASK_CMD;

                    if (!(*dObj->statusReg & 0x01))
                    {
                        dObj->nBlocks --;
                        if (dObj->nBlocks != 0)
                        {
                            /* There are still sectors to be erased. */
                            dObj->blockAddress += 4096;

                            dObj->eraseState = DRV_S25FL_ERASE_WRITE_ENABLE;
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

// *****************************************************************************
/* Function:
    static DRV_SQI_COMMAND_STATUS DRV_S25FL_HandleEraseWrite
    (
        DRV_S25FL_OBJECT *dObj,
        uint8_t *data,
        uint32_t blockAddress,
        uint32_t nBlocks
    );

  Summary:
    Write the requested number of blocks from the flash address specified.

  Description:
    This function is responsible performs the read-modify-write cycle by first
    reading out the contents of the area of the flash that needs to be written
    , erasing the required flash area and then writing the modified pages onto
    the flash.

  Remarks:
    None.
*/
static DRV_SQI_COMMAND_STATUS DRV_S25FL_HandleEraseWrite
(
    DRV_S25FL_OBJECT *dObj,
    uint8_t *data,
    uint32_t blockAddress,
    uint32_t nBlocks
)
{
    DRV_S25FL_BUFFER_OBJECT *bufferObj = dObj->currentBufObj;
    uint8_t pagesPerSector = 16;

    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;  

    switch (dObj->ewState)
    {
        case DRV_S25FL_EW_INIT:
        default:
            {
                dObj->subState = DRV_S25FL_SUBTASK_CMD;
                dObj->eraseState = DRV_S25FL_ERASE_INIT;
                dObj->writeState = DRV_S25FL_WRITE_INIT;

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
                    dObj->ewState = DRV_S25FL_EW_READ_SECTOR;
                }
                else
                {
                    dObj->writePtr = bufferObj->buffer;
                    dObj->ewState = DRV_S25FL_EW_ERASE_SECTOR;

                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                    break;
                }

                /* Fall through for read operation. */
            }

        case DRV_S25FL_EW_READ_SECTOR:
            {
                cmdStatus = DRV_S25FL_HandleRead (dObj, dObj->ewBuffer, dObj->sectorAddress * 4096, 4096);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    /* Find the offset from which the data is to be overlaid. */
                    dObj->blockOffsetInSector <<= 8;
                    memcpy ((void *)&dObj->ewBuffer[dObj->blockOffsetInSector], (const void *)bufferObj->buffer, dObj->nBlocksToWrite << 8);
                    dObj->ewState = DRV_S25FL_EW_ERASE_SECTOR;

                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }
                break;
            }

        case DRV_S25FL_EW_ERASE_SECTOR:
            {
                cmdStatus = DRV_S25FL_HandleErase(dObj, NULL, dObj->sectorAddress, 1);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    dObj->ewState = DRV_S25FL_EW_WRITE_SECTOR;

                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }
                break;
            }

        case DRV_S25FL_EW_WRITE_SECTOR:
            {
                cmdStatus = DRV_S25FL_HandleWrite (dObj, dObj->writePtr, dObj->sectorAddress * 16, 16);
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
                    dObj->ewState = DRV_S25FL_EW_INIT;

                    cmdStatus = DRV_SQI_COMMAND_QUEUED;
                }

                break;
            }
    }

    return cmdStatus;
}

// *****************************************************************************
// *****************************************************************************
// Section: S25FL Driver System Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_S25FL_Initialize
    ( 
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init 
    );
    
  Summary:
    Initializes the S25FL instance for the specified driver index.

  Description:
    This routine initializes the S25FL driver instance for the specified driver
    index, making it ready for clients to open and use it.

  Remarks:
    Refer to drv_s25fl.h for usage information.
*/

SYS_MODULE_OBJ DRV_S25FL_Initialize
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT *const init
)
{
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;
    DRV_S25FL_OBJECT *dObj = (DRV_S25FL_OBJECT*) NULL;
    DRV_S25FL_INIT *s25flInit = NULL;

    /* Validate the driver index */
    if (drvIndex > DRV_S25FL_INSTANCES_NUMBER)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Check if the instance has already been initialized. */
    if (gDrvS25FLObj[drvIndex].inUse)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Assign to the local pointer the init data passed */
    s25flInit = (DRV_S25FL_INIT *)init;

    retVal = OSAL_MUTEX_Create(&s25flClientObjMutex);
    if (retVal != OSAL_RESULT_TRUE)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    dObj = &gDrvS25FLObj[drvIndex];
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
    dObj->sqiDevice = s25flInit->sqiDevice;

    /* Set the erase buffer */
    dObj->ewBuffer = &gDrvS25FLEraseBuffer[drvIndex][0];

    dObj->subState = DRV_S25FL_SUBTASK_CMD;
    dObj->state = DRV_S25FL_OPEN_SQI;

    dObj->xferFrame = &gS25flXferFrame[0];
    dObj->flashId = &gS25flFlashId[0];
    dObj->statusReg = &gS25flStatusReg[0];
    dObj->blockStartAddress = 0;

    DRV_S25FL_RegisterWithSysFs (drvIndex, drvIndex, s25flMediaFunctions);

    /* Return the driver index */
    return drvIndex;
}

// ****************************************************************************
/* Function:
    void DRV_S25FL_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the S25FL driver module

  Description:
    Deinitializes the specified instance of the S25FL driver module,
    disabling its operation (and any hardware). Invalidates all the
    internal data.

  Remarks:
    Refer to drv_s25fl.h for usage information.
*/

void DRV_S25FL_Deinitialize
(
    SYS_MODULE_OBJ object
)
{
    DRV_S25FL_OBJECT * dObj = (DRV_S25FL_OBJECT*)NULL;

    /* Validate the object */
    if ((object == SYS_MODULE_OBJ_INVALID) || (object >= DRV_S25FL_INSTANCES_NUMBER))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO,"DRV_S25FL_Deinitialize(): Invalid parameter.\n");
        return;
    }

    dObj = (DRV_S25FL_OBJECT*)&gDrvS25FLObj[object];

    /* Reset the client count and the exclusive flag */
    dObj->numClients = 0;
    dObj->isExclusive = false;

    /* Reset the queue */
    dObj->queue = NULL;

    /* Set the Hardware instance object status an un-initialized */
    dObj->status = SYS_STATUS_UNINITIALIZED;
    dObj->state = DRV_S25FL_IDLE;

    /* Hardware instance object is no more in use */
    dObj->inUse = false;

    OSAL_MUTEX_Delete(&s25flClientObjMutex);
    OSAL_MUTEX_Delete(&dObj->mutex);

    SYS_DEBUG_PRINT(SYS_ERROR_INFO,"DRV_S25FL_Deinitialize(): Completed.\n");
}

// ****************************************************************************
/* Function:
    SYS_STATUS DRV_S25FL_Status( SYS_MODULE_OBJ object )

  Summary:
    Gets the current status of the S25FL driver module.

  Description:
    This routine provides the current status of the S25FL driver module.

  Remarks:
    Refer to drv_s25fl.h for usage information.
*/

SYS_STATUS DRV_S25FL_Status
(
    SYS_MODULE_OBJ object
)
{
    /* Validate the object */
    if ((object == SYS_MODULE_OBJ_INVALID) || (object >= DRV_S25FL_INSTANCES_NUMBER))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO,"DRV_S25FL_Status(): Invalid parameter.\n");
        return SYS_STATUS_UNINITIALIZED;
    }

    /* Return the driver status */
    return (gDrvS25FLObj[object].status);
}

// *****************************************************************************
// *****************************************************************************
// Section: S25FL Driver Client Routines
// *****************************************************************************
// *****************************************************************************

// ****************************************************************************
/* Function:
    DRV_HANDLE DRV_S25FL_Open
    ( 
        const SYS_MODULE_INDEX index,
        const DRV_IO_INTENT ioIntent
    )
    
  Summary:
    Opens the specified S25FL driver instance and returns a handle to it
  
  Description:
    This routine opens the specified S25FL driver instance and provides a
    handle. This handle must be provided to all other client-level operations
    to identify the caller and the instance of the driver.
  
  Remarks:
    Refer to drv_s25fl.h for usage information.
*/

DRV_HANDLE DRV_S25FL_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
    DRV_S25FL_CLIENT_OBJECT *clientObj = NULL;
    DRV_S25FL_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;;

    /* Validate the driver index */
    if (drvIndex >= DRV_S25FL_INSTANCES_NUMBER)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Open(): Invalid driver index.\n");
        return DRV_HANDLE_INVALID;
    }

    dObj = &gDrvS25FLObj[drvIndex];
    /* Check if the driver is ready to be opened */
    if (dObj->status != SYS_STATUS_READY)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Open(): Driver is not ready.\n");
        return DRV_HANDLE_INVALID;
    }

    /* Check if the driver has already been opened in exclusive mode */
    if (dObj->isExclusive)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Open(): Driver is already open in exclusive mode.\n");
        return DRV_HANDLE_INVALID;
    }

    /* Driver has already been opened and cannot be opened exclusively */
    if ((dObj->numClients > 0) && (ioIntent & DRV_IO_INTENT_EXCLUSIVE))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Open(): Driver is already open. Can't be opened in exclusive mode.\n");
        return DRV_HANDLE_INVALID;
    }

    /* Obtain the Client object mutex */
    retVal = OSAL_MUTEX_Lock(&s25flClientObjMutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Open(): Failed to acquire the client object mutex.\n");
        return DRV_HANDLE_INVALID;
    }

    clientObj = DRV_S25FL_AllocateClientObject ();

    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Open(): Failed to allocate a Client Object.\n");
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
        
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Open(): Open successful.\n");
    }

    OSAL_MUTEX_Unlock(&s25flClientObjMutex);

    return clientObj ? ((DRV_HANDLE)clientObj) : DRV_HANDLE_INVALID;
}

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_S25FL_Close( DRV_Handle handle )

  Summary:
    Closes an opened-instance of the S25FL driver

  Description:
    This routine closes an opened-instance of the S25FL driver, invalidating
    the handle.

  Remarks:
    Refer to drv_s25fl.h for usage infomration.
*/

void DRV_S25FL_Close
(
    const DRV_HANDLE handle
)
{
    DRV_S25FL_CLIENT_OBJECT *clientObj = NULL;
    DRV_S25FL_OBJECT *dObj = NULL;

    /* Get the Client object from the handle passed */
    clientObj = DRV_S25FL_ClientHandleValidate(handle);
    /* Check if the driver handle is valid */
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_S25FL_Close(): Invalid handle.\n");
        return;
    }

    dObj = clientObj->driverObj;
    DRV_S25FL_RemoveClientBufferObjects (clientObj, dObj);

    /* Update the client count */
    dObj->numClients --;
    dObj->isExclusive = false;

    /* Free the Client Instance */
    clientObj->inUse = false;

    SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_S25FL_Close(): Close successful.\n");
    return;
}

// *****************************************************************************
/* Function:
    void DRV_S25FL_Read
    (
        const DRV_HANDLE handle,
        DRV_S25FL_COMMAND_HANDLE * commandHandle,
        void * targetBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    )

  Summary:
    Reads blocks of data from the specified address in memory.

  Description:
    This routine reads a block of data from the specified address in flash.
    This operation is non blocking and returns with the required data in the
    target buffer. This function should not be used to read areas of memory
    which are queued to be programmed or erased. If required, the program or
    erase operations should be allowed to complete. The function returns
    DRV_S25FL_COMMAND_HANDLE_INVALID in the commandHandle argument under the
    following circumstances:
    - if the driver handle is invalid
    - if the target buffer pointer is NULL
    - if the number of blocks to be read is zero or more than the actual number
      of blocks available
    - if a buffer object could not be allocated to the request
    - if the client opened the driver in write only mode

  Remarks:
    Refer to drv_s25fl.h for usage information.
*/

void DRV_S25FL_Read
(
    const DRV_HANDLE handle,
    DRV_S25FL_COMMAND_HANDLE *commandHandle,
    void *targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_S25FL_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_S25FL_CLIENT_OBJECT *clientObj = NULL;
    DRV_S25FL_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    DRV_S25FL_BUFFER_OBJECT *bufferObj = NULL;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_S25FL_COMMAND_HANDLE_INVALID;

    /* Validate the driver handle */
    clientObj = DRV_S25FL_ClientHandleValidate(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Read(): Invalid driver handle.\n");
        return;
    }

    /* Check if the driver was opened with read intent */
    if (!(clientObj->intent & DRV_IO_INTENT_READ))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Read(): Opened with non-read intent.\n");
        return;
    }

    dObj = clientObj->driverObj;

    if ((targetBuffer == NULL) || (nBlock == 0) || ((blockStart + nBlock) > dObj->mediaGeometryTable[DRV_S25FL_GEOMETRY_TABLE_READ_ENTRY].numBlocks))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Read(): Invalid parameters.\n");
        return;
    }

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Read(): Failed to acquire the driver object mutex.\n");
        return;
    }

    bufferObj = DRV_S25FL_AllocateBufferObject (clientObj, targetBuffer, blockStart, nBlock, DRV_S25FL_OPERATION_TYPE_READ);
    if (bufferObj != NULL)
    {
        *tempHandle1 = bufferObj->commandHandle;

        /* Add the request to the queue. */
        DRV_S25FL_AddToQueue (dObj, bufferObj);
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);

    return;
}

// *****************************************************************************
/* Function:
    void DRV_S25FL_Write
    (
        const DRV_HANDLE handle,
        DRV_S25FL_COMMAND_HANDLE * commandHandle,
        void * sourceBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    )

  Summary:
    Writes blocks of data starting from the specified address in flash memory.

  Description:
    This function schedules a non-blocking write operation for writing blocks
    of data into flash memory. The function returns with a valid buffer handle
    in the commandHandle argument if the write request was scheduled
    successfully. The function adds the request to the hardware instance queue
    and returns immediately. While the request is in the queue, the application
    buffer is owned by the driver and should not be modified. The function
    returns DRV_S25FL_COMMAND_HANDLE_INVALID in the commandHandle argument
    under the following circumstances:
    - if a buffer object could not be allocated to the request
    - if the source buffer pointer is NULL
    - if the client opened the driver for read only
    - if the number of blocks to be written is either zero or more than the
      number of blocks actually available
    - if the write queue size is full or queue depth is insufficient
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_S25FL_EVENT_COMMAND_COMPLETE event if the
    buffer was processed successfully or DRV_S25FL_EVENT_COMMAND_ERROR
    event if the buffer was not processed successfully.

  Remarks:
    Refer to drv_s25fl.h for usage information.
*/

void DRV_S25FL_Write
(
    const DRV_HANDLE handle,
    DRV_S25FL_COMMAND_HANDLE *commandHandle,
    void *sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_S25FL_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_S25FL_CLIENT_OBJECT *clientObj = NULL;
    DRV_S25FL_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    DRV_S25FL_BUFFER_OBJECT *bufferObj = NULL;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_S25FL_COMMAND_HANDLE_INVALID;

    /* Validate the driver handle */
    clientObj = DRV_S25FL_ClientHandleValidate(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Write(): Invalid driver handle.\n");
        return;
    }

    /* Check if the driver was opened with write intent */
    if (!(clientObj->intent & DRV_IO_INTENT_WRITE))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Write(): Opened with non-write intent.\n");
        return;
    }

    dObj = clientObj->driverObj;
    if ((sourceBuffer == NULL) || (nBlock == 0) || ((blockStart + nBlock) > dObj->mediaGeometryTable[DRV_S25FL_GEOMETRY_TABLE_WRITE_ENTRY].numBlocks))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Write(): Invalid parameters.\n");
        return;
    }

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Write(): Failed to acquire the driver object mutex.\n");
        return;
    }

    bufferObj = DRV_S25FL_AllocateBufferObject (clientObj, sourceBuffer, blockStart, nBlock, DRV_S25FL_OPERATION_TYPE_WRITE);
    if (bufferObj != NULL)
    {
        *tempHandle1 = bufferObj->commandHandle;

        /* Add the request to the queue. */
        DRV_S25FL_AddToQueue (dObj, bufferObj);
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);
    return;
}

// **************************************************************************
/* Function:
    void DRV_S25FL_Erase
    (
        const DRV_HANDLE handle,
        DRV_S25FL_COMMAND_HANDLE * commandHandle,
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
    DRV_S25FL_COMMAND_HANDLE_INVALID in the commandHandle argument under the
    following circumstances:
    - if a buffer object could not be allocated to the request
    - if the client opened the driver for read only
    - if the number of blocks to be erased is either zero or more than the
      number of blocks actually available
    - if the erase queue size is full or queue depth is insufficient
    - if the driver handle is invalid 
    
    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_S25FL_EVENT_COMMAND_COMPLETE event if the
    erase operation was successful or DRV_S25FL_EVENT_COMMAND_ERROR
    event if the erase operation was not successful.

  Remarks:
    Refer to drv_s25fl.h for usage information.
*/

void DRV_S25FL_Erase
(
    const DRV_HANDLE handle,
    DRV_S25FL_COMMAND_HANDLE *commandHandle,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_S25FL_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_S25FL_CLIENT_OBJECT *clientObj = NULL;
    DRV_S25FL_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    DRV_S25FL_BUFFER_OBJECT *bufferObj = NULL;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_S25FL_COMMAND_HANDLE_INVALID;

    /* Validate the driver handle */
    clientObj = DRV_S25FL_ClientHandleValidate(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Erase(): Invalid driver handle.\n");
        return;
    }

    /* Check if the driver was opened with write intent */
    if (!(clientObj->intent & DRV_IO_INTENT_WRITE))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Erase(): Opened with non-write intent.\n");
        return;
    }

    dObj = clientObj->driverObj;
    if ((nBlock == 0) || ((blockStart + nBlock) > dObj->mediaGeometryTable[DRV_S25FL_GEOMETRY_TABLE_ERASE_ENTRY].numBlocks))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Erase(): Invalid parameters.\n");
        return;
    }

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Erase(): Failed to acquire the driver object mutex.\n");
        return;
    }

    bufferObj = DRV_S25FL_AllocateBufferObject (clientObj, NULL, blockStart, nBlock, DRV_S25FL_OPERATION_TYPE_ERASE);
    if (bufferObj != NULL)
    {
        *tempHandle1 = bufferObj->commandHandle;

        /* Add the request to the queue. */
        DRV_S25FL_AddToQueue (dObj, bufferObj);
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);

    return;
}

// *****************************************************************************
/* Function:
    void DRV_S25FL_EraseWrite
    (
        const DRV_HANDLE handle,
        DRV_S25FL_COMMAND_HANDLE * commandHandle,
        void * sourceBuffer,
        uint32_t writeBlockStart,
        uint32_t nWriteBlock
    );

  Summary:
    Erase and Write blocks of data starting from a specified block start
    address.

  Description:
    This function combines the step of erasing a sector and then writing the
    page. The application can use this function if it wants to avoid having to
    explicitly delete a sector in order to update the pages contained in the
    sector. 

    This function schedules a non-blocking operation to erase and write blocks
    of data into flash memory. The function returns with a valid command handle
    in the commandHandle argument if the write request was scheduled
    successfully. The function adds the request to the hardware instance queue
    and returns immediately. While the request is in the queue, the application
    buffer is owned by the driver and should not be modified. The function
    returns DRV_S25FL_COMMAND_HANDLE_INVALID in the commandHandle argument
    under the following circumstances:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the client opened the driver for read only
    - if the number of blocks to be written is either zero or more than the
      number of blocks actually available
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_S25FL_EVENT_COMMAND_COMPLETE event if the buffer
    was processed successfully or DRV_S25FL_EVENT_COMMAND_ERROR event if the
    buffer was not processed successfully.

  Precondition:
    The DRV_S25FL_Initialize() routine must have been called for the specified
    S25FL driver instance.

    The DRV_S25FL_Open() must have been called with DRV_IO_INTENT_WRITE or
    DRV_IO_INTENT_READWRITE as a parameter to obtain a valid opened device
    handle.

  Remarks:
    See drv_s25fl.h for usage information.
*/
void DRV_S25FL_EraseWrite
(
    const DRV_HANDLE handle,
    DRV_S25FL_COMMAND_HANDLE *commandHandle,
    void *sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_S25FL_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_S25FL_CLIENT_OBJECT *clientObj = NULL;
    DRV_S25FL_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    DRV_S25FL_BUFFER_OBJECT *bufferObj = NULL;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_S25FL_COMMAND_HANDLE_INVALID;

    /* Validate the driver handle */
    clientObj = DRV_S25FL_ClientHandleValidate(handle);
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_EraseWrite(): Invalid driver handle.\n");
        return;
    }

    /* Check if the driver was opened with write intent */
    if (!(clientObj->intent & DRV_IO_INTENT_WRITE))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_EraseWrite(): Opened with non-write intent.\n");
        return;
    }

    dObj = clientObj->driverObj;
    if ((sourceBuffer == NULL) || (nBlock == 0) || ((blockStart + nBlock) > dObj->mediaGeometryTable[DRV_S25FL_GEOMETRY_TABLE_WRITE_ENTRY].numBlocks))
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_EraseWrite(): Invalid parameters.\n");
        return;
    }

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_EraseWrite(): Failed to acquire the driver object mutex.\n");
        return;
    }

    bufferObj = DRV_S25FL_AllocateBufferObject (clientObj, sourceBuffer, blockStart, nBlock, DRV_S25FL_OPERATION_TYPE_ERASE_WRITE);
    if (bufferObj != NULL)
    {
        *tempHandle1 = bufferObj->commandHandle;

        /* Add the request to the queue. */
        DRV_S25FL_AddToQueue (dObj, bufferObj);
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);

    return;
}

// *****************************************************************************
/* Function:
    DRV_S25FL_COMMAND_STATUS DRV_S25FL_CommandStatus
    (
        const DRV_HANDLE handle, 
        const DRV_S25FL_COMMAND_HANDLE commandHandle
    );

  Summary:
    Gets the current status of the command.

  Description:
    This routine gets the current status of the buffer. The application must
    use this routine where the status of a scheduled buffer needs to polled on.
    The function may return DRV_S25FL_COMMAND_HANDLE_INVALID in a case where
    the buffer handle has expired. A buffer handle expires when the internal
    buffer object is re-assigned to another erase or write request. It is
    recommended that this function be called regularly in order to track the
    buffer status correctly.

    The application can alternatively register an event handler to receive
    write or erase operation completion events.

  Remarks:
    Refer to drv_s25fl.h for usage information.
*/

DRV_S25FL_COMMAND_STATUS DRV_S25FL_CommandStatus
(
    const DRV_HANDLE handle,
    const DRV_S25FL_COMMAND_HANDLE commandHandle
)
{
    uint16_t iEntry;

    /* Validate the client handle */
    if (DRV_S25FL_ClientHandleValidate(handle) == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_CommandStatus(): Invalid driver handle.\n");
        return DRV_S25FL_COMMAND_ERROR_UNKNOWN;
    }

    /* The upper 16 bits of the buffer handle are the token and the lower 16
     * bits of the are buffer index into the gDrvS25flBufferObject array */
    iEntry = commandHandle & 0xFFFF;

    /* Compare the buffer handle with buffer handle in the object */
    if(gDrvS25flBufferObject[iEntry].commandHandle != commandHandle)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_CommandStatus(): Handle has been reused.\n");
        /* This means that object has been re-used by another request. Indicate
         * that the operation is completed.  */
        return (DRV_S25FL_COMMAND_COMPLETED);
    }

    /* Return the last known buffer object status */
    return (gDrvS25flBufferObject[iEntry].status);
}

// ****************************************************************************
/* Function:
    void DRV_S25FL_Tasks
    (
        SYS_MODULE_OBJ object
    );

  Summary:
    Maintains the driver's state machine.

  Description:
    This routine is used to maintain the driver's internal state machine.

  Remarks:
    Refer to drv_s25fl.h for usage information.
*/

void DRV_S25FL_Tasks
(
    SYS_MODULE_OBJ object
)
{
    DRV_S25FL_OBJECT *dObj = NULL;
    DRV_S25FL_CLIENT_OBJECT *clientObj = NULL;
    DRV_S25FL_BUFFER_OBJECT *bufferObj = NULL;
    DRV_S25FL_EVENT event = DRV_S25FL_EVENT_COMMAND_ERROR;
    bool done = false;
    DRV_SQI_COMMAND_STATUS cmdStatus = DRV_SQI_COMMAND_ERROR_UNKNOWN;

    if(SYS_MODULE_OBJ_INVALID == object)
    {
        /* Invalid system object */
        return;
    }

    dObj = &gDrvS25FLObj[object];

    if (OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER) != OSAL_RESULT_TRUE)
    {
        return;
    }

    switch (dObj->state)
    {
        case DRV_S25FL_OPEN_SQI:
            {
                dObj->sqiHandle = DRV_SQI_Open (0, DRV_IO_INTENT_READWRITE);
                if (dObj->sqiHandle != DRV_HANDLE_INVALID)
                {
                    SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_Tasks(): Opened the SQI driver successfully.\n");
                    dObj->state = DRV_S25FL_READ_FLASH_ID;
                    dObj->subState = DRV_S25FL_SUBTASK_JEDEC_ID_READ_CMD;
                }
                break;
            }

        case DRV_S25FL_READ_FLASH_ID:
            {
                cmdStatus = DRV_S25FL_ReadFlashId (dObj);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    if (DRV_S25FL_UpdateGeometry (dObj, dObj->flashId[2]))
                    {
                        dObj->state = DRV_S25FL_CFG_FLASH;
                        dObj->cfgFlashState = DRV_S25FL_CFG_FLASH_READ_STATUS_REG_1;
                        dObj->subState = DRV_S25FL_SUBTASK_CMD;
                    }
                    else
                    {
                        /* Unsupported device Id */
                        dObj->state = DRV_S25FL_ERROR;
                    }
                }
                else if (cmdStatus == DRV_SQI_COMMAND_ERROR_UNKNOWN)
                {
                    dObj->state = DRV_S25FL_ERROR;
                }
                else
                {
                    /* Continue to remain in the same state. */
                }
                break;
            }

        case DRV_S25FL_CFG_FLASH:
            {
                cmdStatus = DRV_S25FL_ConfigureFlash(dObj);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    /* Enable QUAD IO completed. */
                    dObj->state = DRV_S25FL_PROCESS_QUEUE;
                    dObj->status = SYS_STATUS_READY;
                }
                else if (cmdStatus == DRV_SQI_COMMAND_ERROR_UNKNOWN)
                {
                    dObj->state = DRV_S25FL_ERROR;
                }
                else
                {
                    /* Continue to remain in the same state. */
                }
                break;
            }

        case DRV_S25FL_PROCESS_QUEUE:
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
                    dObj->eraseState = DRV_S25FL_ERASE_INIT;
                    dObj->writeState = DRV_S25FL_WRITE_INIT;
                    dObj->ewState = DRV_S25FL_EW_INIT;
                    dObj->subState = DRV_S25FL_SUBTASK_CMD;

                    dObj->state = DRV_S25FL_TRANSFER;
                }
            }

        case DRV_S25FL_TRANSFER:
            {
                bufferObj = dObj->currentBufObj;
                cmdStatus = gS25flXferFuncPtr[bufferObj->opType](dObj, &bufferObj->buffer[0], bufferObj->blockStart, bufferObj->nBlocks);
                if (cmdStatus == DRV_SQI_COMMAND_COMPLETED)
                {
                    bufferObj->status = DRV_S25FL_COMMAND_COMPLETED;
                    /* The operation has completed. */
                    dObj->state = DRV_S25FL_PROCESS_QUEUE;
                    done = true;
                    event = DRV_SQI_EVENT_COMMAND_COMPLETE;
                }
                else if (cmdStatus == DRV_SQI_COMMAND_ERROR_UNKNOWN)
                {
                    /* The operation has failed. */
                    bufferObj->status = DRV_S25FL_COMMAND_ERROR_UNKNOWN;
                    dObj->state = DRV_S25FL_ERROR;
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
                    DRV_S25FL_UpdateQueue (dObj);

                    clientObj = (DRV_S25FL_CLIENT_OBJECT *)bufferObj->hClient;
                    if(clientObj->eventHandler != NULL)
                    {
                        /* Call the event handler */
                        clientObj->eventHandler(event, bufferObj->commandHandle, clientObj->context);
                    }
                }
                break;
            }

           case DRV_S25FL_IDLE:
            {
                break;
            }

           case DRV_S25FL_ERROR:
           default:
            {
                break;
            }
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);
}

// *****************************************************************************
/* Function:
    void DRV_S25FL_EventHandlerSet
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
    
    The event handler should be set before the client performs any write or
    erase operations that could generate events. The event handler once set,
    persists until the client closes the driver or sets another event handler
    (which could be a "NULL" pointer to indicate no callback).

  Remarks:
    Refer to drv_s25fl.h for usage information.
*/

void DRV_S25FL_EventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
)
{
    DRV_S25FL_CLIENT_OBJECT *clientObj = NULL;

    clientObj = DRV_S25FL_ClientHandleValidate(handle);
    /* Check if the client handle is valid */
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_EventHandlerSet(): Invalid driver handle.\n");
        return;
    }

    /* Set the event handler */
    clientObj->eventHandler = eventHandler;
    clientObj->context = context;
}

// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_GEOMETRY * DRV_S25FL_GeometryGet( const DRV_HANDLE handle );

  Summary:
    Returns the geometry of the device.

  Description:
    This API gives the following geometrical details of the S25FL Flash:
    - Media Property
    - Number of Read/Write/Erase regions in the flash device
    - Number of Blocks and their size in each region of the device

  Remarks:
    Refer to drv_s25fl.h for usage information.
*/

SYS_FS_MEDIA_GEOMETRY * DRV_S25FL_GeometryGet
(
    const DRV_HANDLE handle
)
{
    DRV_S25FL_CLIENT_OBJECT *clientObj = NULL;
    DRV_S25FL_OBJECT *dObj = NULL;

    /* Get the Client object from the handle passed */
    clientObj = DRV_S25FL_ClientHandleValidate(handle);
    /* Check if the driver handle is valid */
    if (clientObj == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_GeometryGet(): Invalid driver handle.\n");
        return NULL;
    }

    dObj = clientObj->driverObj;
    return &dObj->mediaGeometryObj;
}

// *****************************************************************************
/* Function:
    bool DRV_S25FL_isAttached( const DRV_HANDLE handle );

  Summary:
    Returns the physical attach status of the S25FL.

  Description:
    This function returns the physical attach status of the S25FL. This
    function returns false if the driver handle is invalid otherwise returns
    true.

  Remarks:
    Refer to drv_s25fl.h for usage information.
*/

bool DRV_S25FL_IsAttached
(
    const DRV_HANDLE handle
)
{
    /* Validate the driver handle */
    if (DRV_S25FL_ClientHandleValidate(handle) == NULL)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_INFO, "DRV_S25FL_IsAttached(): Invalid driver handle.\n");
        return false;
    }

   return true;
}

// *****************************************************************************
/* Function:
    bool DRV_S25FL_isWriteProtected( const DRV_HANDLE handle );

  Summary:
    Returns the write protect status of S25FL.

  Description:
    This function returns the write protect status of the S25FL. Always returns
    false.

  Remarks:
    Refer to drv_s25fl.h for usage information.
*/

bool DRV_S25FL_IsWriteProtected
(
    const DRV_HANDLE handle
)
{
    /* This function always returns false */
    return false;
}

// *****************************************************************************
/* Function:
    uintptr_t DRV_S25FL_AddressGet
    (
        const DRV_HANDLE handle
    );

  Summary:
    Returns the S25FL media start address

  Description:
    This function returns the S25FL Media start address.

  Remarks:
    None.
*/

uintptr_t DRV_S25FL_AddressGet
(
    const DRV_HANDLE handle
)
{
    DRV_S25FL_CLIENT_OBJECT *clientObj = NULL;
    DRV_S25FL_OBJECT *dObj = NULL;

    /* Validate the handle */
    if (handle == DRV_HANDLE_INVALID)
    {
        SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_S25FL_AddressGet(): Invalid driver handle.\n");
        return (uintptr_t)NULL;
    }

    /* See if the client has been opened */
    clientObj = (DRV_S25FL_CLIENT_OBJECT *)handle;
    if (clientObj->inUse == false)
    {
        SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_S25FL_AddressGet(): Invalid client.\n");
        return (uintptr_t)NULL;
    }

    /* Check if the driver is ready for operation */
    dObj = (DRV_S25FL_OBJECT *)clientObj->driverObj;
    if (dObj->status != SYS_STATUS_READY)
    {
        SYS_DEBUG_PRINT (SYS_ERROR_INFO, "DRV_S25FL_AddressGet(): Driver is not ready.\n");
        return (uintptr_t)NULL;
    }

    return dObj->blockStartAddress;
}

