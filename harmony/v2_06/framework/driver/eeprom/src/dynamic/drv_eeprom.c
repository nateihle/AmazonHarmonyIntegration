/*******************************************************************************
  EEPROM Driver Interface Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_eeprom.c

  Summary:
    EEPROM Driver Interface Definition

  Description:
    The EEPROM Driver provides a interface to access the EEPROM on the PIC32
    microcontroller. This file implements the EEPROM Driver interface. This file
    should be included in the project if EEPROM driver functionality is needed.
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

#include "driver/eeprom/src/drv_eeprom_local.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global objects
// *****************************************************************************
// *****************************************************************************

/*************************************************
 * Hardware instance objects
 *************************************************/

DRV_EEPROM_OBJECT gDrvEEPROMObj[DRV_EEPROM_INSTANCES_NUMBER];

/*************************************************
 * Driver Client Objects
 *************************************************/

DRV_EEPROM_CLIENT_OBJECT gDrvEEPROMClientObj[DRV_EEPROM_CLIENTS_NUMBER];

/*************************************************
 * Driver Buffer Objects. These transport the
 * read, write and erase requests.
 *************************************************/

DRV_EEPROM_BUFFER_OBJECT gDrvEEPROMBufferObject[DRV_EEPROM_BUFFER_OBJECT_NUMBER];

/************************************************
 * This token is incremented for every request
 * added to the queue and is used to generate
 * a different buffer handle for every request.
 ***********************************************/

uint16_t gDrvEEPROMBufferToken = 0;

uint16_t gDrvEepromCalibrationData[8] = {};

/*************************************************
 * OSAL Declarations
 *************************************************/
/* EEPROM Client Object Mutex */
OSAL_MUTEX_DECLARE(eepromClientObjMutex);

const SYS_FS_MEDIA_FUNCTIONS eepromMediaFunctions =
{
    .mediaStatusGet     = DRV_EEPROM_IsAttached,
    .mediaGeometryGet   = DRV_EEPROM_GeometryGet,
    .sectorRead         = DRV_EEPROM_Read,
    .sectorWrite        = DRV_EEPROM_Write,
    .eventHandlerset    = DRV_EEPROM_EventHandlerSet,
    .commandStatusGet   = (void *)DRV_EEPROM_CommandStatus,
    .Read               = DRV_EEPROM_Read,
    .erase              = DRV_EEPROM_Erase,
    .addressGet         = DRV_EEPROM_AddressGet,
    .open               = DRV_EEPROM_Open,
    .close              = DRV_EEPROM_Close,
    .tasks              = DRV_EEPROM_Tasks,
};

// *****************************************************************************
/* Function:
    DRV_EEPROM_CLIENT * DRV_EEPROM_ClientHandleValidate(DRV_HANDLE handle);

  Summary:
    Validate the driver handle.

  Description:
    This function validates the driver handle and returns the client object pointer
    associated with the driver handle if the handle is valid. If the driver handle
    is not valid or if the driver is in a not ready state then NULL is returned.

  Remarks:
    None
*/

static DRV_EEPROM_CLIENT_OBJECT * DRV_EEPROM_ClientHandleValidate
(
    DRV_HANDLE handle
)
{
    DRV_EEPROM_CLIENT_OBJECT *clientObj = NULL;
    DRV_EEPROM_OBJECT *dObj = NULL;

    /* Validate the handle */
    if ((handle == 0) || (handle == DRV_HANDLE_INVALID))
    {
        return NULL;
    }

    /* See if the client has been opened */
    clientObj = (DRV_EEPROM_CLIENT_OBJECT *)handle;
    if (!clientObj->inUse)
    {
        return NULL;
    }

    /* Check if the driver is ready for operation */
    dObj = (DRV_EEPROM_OBJECT *)clientObj->driverObj;
    if (dObj->status != SYS_STATUS_READY)
    {
        return NULL;
    }

    return clientObj;
}

static void DRV_EEPROM_UnlockAndWrite
(
    DRV_EEPROM_OBJECT *dObj
)
{
    /* Disable the global interrupts */
    dObj->intStatus = SYS_INT_Disable();

    PLIB_NVM_EEPROMKeySequenceWrite (dObj->moduleId, DRV_EEPROM_UNLOCK_KEY_1);
    PLIB_NVM_EEPROMKeySequenceWrite (dObj->moduleId, DRV_EEPROM_UNLOCK_KEY_2);
    PLIB_NVM_EEPROMWriteStart (dObj->moduleId);

    /* Enable the global interrupts if they were enabled */
    if(dObj->intStatus)
    {
        SYS_INT_Enable();
    }
}

static void DRV_EEPROM_ConfigureWaitStates
(
    DRV_EEPROM_OBJECT *dObj
)
{
    uint32_t sysClkHz = 0;
    uint8_t eews = 0;

    /* Configure the CFGCON2 <7:0> EEPROM Access Time field based
     * on the sys clock frequency being used. */
    sysClkHz = SYS_CLK_SystemFrequencyGet ();

    if (sysClkHz < 25000000)
    {
        /* Less than 25 MHz. */
        eews = 2;
    }
    else if (sysClkHz < 50000000)
    {
        /* Less than 50 MHz. */
        eews = 4;
    }
    else if (sysClkHz < 75000000)
    {
        /* Less than 75 MHz. */
        eews = 7;
    }
    else if (sysClkHz < 100000000)
    {
        /* Less than 100 MHz. */
        eews = 9;
    }
    else if (sysClkHz < 125000000)
    {
        /* Less than 125 MHz. */
        eews = 12;
    }
    else if (sysClkHz < 150000000)
    {
        /* Less than 150 MHz. */
        eews = 14;
    }
    else if (sysClkHz < 175000000)
    {
        /* Less than 175 MHz. */
        eews = 17;
    }
    else// if (sysClkHz < 200000000)
    {
        /* Less than 200 MHz. */
        eews = 19;
    }

    PLIB_NVM_EEPROMWaitStatesSet (dObj->moduleId, eews);
}

static bool DRV_EEPROM_Configure
(
    DRV_EEPROM_OBJECT *dObj
)
{
    uint32_t data = 0;
    bool done = false;

    switch (dObj->initState)
    {
        case DRV_EEPROM_INIT_START:
        default:
            {
                dObj->address = 0;
                dObj->initState = DRV_EEPROM_INIT_CALIBRATION_DATA;

                data = PLIB_NVM_EEPROMReadCalibrationData(dObj->moduleId, CALIBRATION_REG_0);
                gDrvEepromCalibrationData[0] = data & 0xFFFF;
                gDrvEepromCalibrationData[1] = (data >> 16) & 0xFFFF;

                data = PLIB_NVM_EEPROMReadCalibrationData(dObj->moduleId, CALIBRATION_REG_1);
                gDrvEepromCalibrationData[2] = data & 0xFFFF;
                gDrvEepromCalibrationData[3] = (data >> 16) & 0xFFFF;

                data = PLIB_NVM_EEPROMReadCalibrationData(dObj->moduleId, CALIBRATION_REG_2);
                gDrvEepromCalibrationData[4] = data & 0xFFFF;
                gDrvEepromCalibrationData[5] = (data >> 16) & 0xFFFF;

                data = PLIB_NVM_EEPROMReadCalibrationData(dObj->moduleId, CALIBRATION_REG_3);
                gDrvEepromCalibrationData[6] = data & 0xFFFF;
                gDrvEepromCalibrationData[7] = (data >> 16) & 0xFFFF;

                /* Fall through. */
            }

        case DRV_EEPROM_INIT_CALIBRATION_DATA:
            {
                /* Set the configuration values in the EEPROM Controller. */

                /* Enable writing to the EEPROM. */
                PLIB_NVM_EEPROMAddress (dObj->moduleId, dObj->address << 2);
                PLIB_NVM_EEPROMDataToWrite (dObj->moduleId, gDrvEepromCalibrationData[dObj->address]);
                PLIB_NVM_EEPROMWriteEnable (dObj->moduleId);
                PLIB_NVM_EEPROMOperationSelect(dObj->moduleId, EEPROM_CONFIG_WRITE_OPERATION);

                DRV_EEPROM_UnlockAndWrite(dObj);

                dObj->initState = DRV_EEPROM_INIT_CALIBRATION_STATUS_CHECK;
                break;
            }

        case DRV_EEPROM_INIT_CALIBRATION_STATUS_CHECK:
            {
                if (PLIB_NVM_EEPROMOperationHasCompleted(dObj->moduleId))
                {
                    dObj->address ++;
                    /* Check if writing of calibration data is complete. */
                    if (dObj->address != 8)
                    {
                        dObj->initState = DRV_EEPROM_INIT_CALIBRATION_DATA;
                    }
                    else
                    {
                        dObj->address = 0;
                        dObj->initState = DRV_EEPROM_INIT_DONE;
                    }
                }

                break;
            }

        case DRV_EEPROM_INIT_DONE:
            {
                /* Disable writing to the EEPROM. */
                PLIB_NVM_EEPROMReadEnable(dObj->moduleId);

                done = true;
                break;
            }
    }

    return done;
}

static void DRV_EEPROM_AddToQueue
(
    DRV_EEPROM_OBJECT *dObj,
    DRV_EEPROM_BUFFER_OBJECT *bufferObject
)
{
    if (dObj->queue == NULL)
    {
        dObj->queue = bufferObject;
    }
    else
    {
        if (dObj->queue->previous != NULL)
        {
            dObj->queue->previous->next = bufferObject;
            bufferObject->previous = dObj->queue->previous;
            dObj->queue->previous = bufferObject;
        }
        else
        {
            dObj->queue->previous = bufferObject;
            dObj->queue->next = bufferObject;
            bufferObject->previous = dObj->queue;
        }
    }
}

/* This function updates the head of the queue. */
static void DRV_EEPROM_UpdateQueue
(
    DRV_EEPROM_OBJECT *dObj
)
{
    DRV_EEPROM_BUFFER_OBJECT * bufferObj = dObj->queue;

    if (bufferObj != NULL)
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

/* This function removes the buffer objects that were queued by a client. */
static void DRV_EEPROM_RemoveClientBufferObjects
(
    const DRV_HANDLE handle,
    DRV_EEPROM_OBJECT *dObj
)
{
    DRV_EEPROM_BUFFER_OBJECT *bufferObject = NULL;
    DRV_EEPROM_BUFFER_OBJECT *lastObject = NULL;
    DRV_EEPROM_BUFFER_OBJECT *head = NULL;
    DRV_EEPROM_BUFFER_OBJECT *temp = NULL;

    bufferObject = dObj->queue;

    if (dObj->queue != NULL)
    {
        dObj->queue->previous = NULL;
    }

    while (bufferObject != NULL)
    {
        temp = bufferObject->next;
        if (bufferObject->hClient == handle)
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

static DRV_EEPROM_BUFFER_OBJECT * DRV_EEPROM_GetFreeBufferObject
(
    uint16_t *position
)
{
    uint16_t iEntry = 0;
    DRV_EEPROM_BUFFER_OBJECT *bufferObj = NULL;

    *position = 0;

    for (iEntry = 0; iEntry < DRV_EEPROM_BUFFER_OBJECT_NUMBER; iEntry++)
    {
        /* Search for a free buffer object to use */
        if (gDrvEEPROMBufferObject[iEntry].inUse == false)
        {
            /* Found a free buffer object. */
            bufferObj = &gDrvEEPROMBufferObject[iEntry];
            *position = iEntry;
            break;
        }
    }

    return bufferObj;
}

static void DRV_EEPROM_AddRequest
(
    const DRV_HANDLE handle,
    DRV_EEPROM_COMMAND_HANDLE *commandHandle,
    uint8_t *buffer, 
    uint32_t blockStart, 
    uint32_t nBlock, 
    DRV_EEPROM_BUFFER_OPERATION operation
)
{
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;
    DRV_EEPROM_OBJECT *dObj = NULL; 
    DRV_EEPROM_CLIENT_OBJECT *clientObj = NULL;
    DRV_EEPROM_BUFFER_OBJECT *bufferObj = NULL;
    uint32_t blockSize = 0;
    uint16_t iEntry = 0;

    DRV_EEPROM_COMMAND_HANDLE *tempHandle1, tempHandle2;

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_EEPROM_COMMAND_HANDLE_INVALID;

    clientObj = DRV_EEPROM_ClientHandleValidate(handle);
    if (clientObj == NULL)
    {
        return;
    }

    if (operation == DRV_EEPROM_BUFFER_OPERATION_READ)
    {
        if (!(clientObj->intent & DRV_IO_INTENT_READ))
        {
            return;
        }
    }
    else
    {
        if (!(clientObj->intent & DRV_IO_INTENT_WRITE))
        {
            return;
        }
    }

    dObj = (DRV_EEPROM_OBJECT *)clientObj->driverObj;
    
    if (operation != DRV_EEPROM_BUFFER_OPERATION_BULK_ERASE)
    {
        blockSize = dObj->mediaGeometryObj->geometryTable[operation].blockSize;

        if ((blockStart + nBlock) > dObj->mediaGeometryObj->geometryTable[operation].numBlocks)
        {
            return;
        }

        if (nBlock == 0)
        {
            return;
        }

        if (operation != DRV_EEPROM_BUFFER_OPERATION_ERASE)
        {
            if (buffer == NULL)
            {
                return;
            }
        }
    }

    /* Acquire Buffer Object Mutex */
    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        return;
    }

    bufferObj = DRV_EEPROM_GetFreeBufferObject (&iEntry);
    if (bufferObj != NULL)
    {
        bufferObj->inUse         = true;
        bufferObj->commandHandle = DRV_EEPROM_MAKE_HANDLE(gDrvEEPROMBufferToken, iEntry);
        bufferObj->hClient       = handle;
        bufferObj->buffer        = buffer;
        bufferObj->address       = dObj->blockStartAddress + (blockStart * blockSize);
        bufferObj->nBlocks       = nBlock;
        bufferObj->operation     = operation;
        bufferObj->status        = DRV_EEPROM_COMMAND_QUEUED;

        /* Update the token number. */
        DRV_EEPROM_UPDATE_BUF_TOKEN(gDrvEEPROMBufferToken);

        /* Queue the request */
        bufferObj->next = NULL;
        bufferObj->previous = NULL;

        DRV_EEPROM_AddToQueue (dObj, bufferObj);
        *tempHandle1 = (bufferObj->commandHandle);
    }

    /* Release Buffer Object Mutex */
    OSAL_MUTEX_Unlock(&dObj->mutex);

    return;
}

// *****************************************************************************
// *****************************************************************************
// Section: EEPROM Driver System Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_EEPROM_Initialize
    (
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init
    )

  Summary:
    Initializes the EEPROM instance for the specified driver index

  Description:
    This routine initializes the EEPROM driver instance for the specified
    driver index, making it ready for clients to open and use it.

  Remarks:
    Refer to drv_eeprom.h for usage information.
*/

SYS_MODULE_OBJ DRV_EEPROM_Initialize
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT *const init
)
{
    OSAL_RESULT retVal;
    DRV_EEPROM_OBJECT *dObj = (DRV_EEPROM_OBJECT *)NULL;
    DRV_EEPROM_INIT *eepromInit = NULL;

    if ((drvIndex > DRV_EEPROM_INSTANCES_NUMBER) || (init == NULL))
    {
        /* Validate the parameters */
        return SYS_MODULE_OBJ_INVALID;
    }

    if (gDrvEEPROMObj[drvIndex].inUse)
    {
        /* Instance has already been initialized */
        return SYS_MODULE_OBJ_INVALID;
    }

    eepromInit = (DRV_EEPROM_INIT *)init;

    retVal = OSAL_MUTEX_Create(&eepromClientObjMutex);
    if (retVal != OSAL_RESULT_TRUE)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    dObj = &gDrvEEPROMObj[drvIndex];

    retVal = OSAL_MUTEX_Create(&dObj->mutex);
    if (retVal != OSAL_RESULT_TRUE)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    dObj->inUse = true;
    dObj->numClients = 0;
    dObj->blockStartAddress = 0;
    dObj->mediaGeometryObj = (SYS_FS_MEDIA_GEOMETRY *)eepromInit->eepromMediaGeometry;
    dObj->moduleId = eepromInit->eepromId;
    dObj->drvIndex = drvIndex;

    /* Configure the wait states */
    DRV_EEPROM_ConfigureWaitStates(dObj);
    /* Turn ON the EEPROM. */
    PLIB_NVM_EEPROMEnable(dObj->moduleId);

    /* TODO: Disable the EEPROM global interrupt. */

    dObj->status = SYS_STATUS_BUSY;

    /* Return the driver index as the System Module Object */
    return (SYS_MODULE_OBJ)drvIndex;
}

// ****************************************************************************
/* Function:
    void DRV_EEPROM_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the EEPROM driver module

  Description:
    Deinitializes the specified instance of the EEPROM driver module,
    disabling its operation (and any hardware). Invalidates all the
    internal data.

  Remarks:
    Refer to drv_eeprom.h for usage information.
*/

void DRV_EEPROM_Deinitialize
(
    SYS_MODULE_OBJ object
)
{
    DRV_EEPROM_OBJECT *dObj = (DRV_EEPROM_OBJECT*)NULL;
    if ((object == SYS_MODULE_OBJ_INVALID) || (object >= DRV_EEPROM_INSTANCES_NUMBER))
    {
        return;
    }

    dObj = (DRV_EEPROM_OBJECT*)&gDrvEEPROMObj[object];

    dObj->numClients = 0;
    dObj->isExclusive = false;
    dObj->queue = NULL;
    dObj->status = SYS_STATUS_UNINITIALIZED;
    dObj->taskState = DRV_EEPROM_TASK_IDLE;

    OSAL_MUTEX_Delete(&eepromClientObjMutex);
    OSAL_MUTEX_Delete(&dObj->mutex);

    dObj->inUse = false;
}

// ****************************************************************************
/* Function:
    SYS_STATUS DRV_EEPROM_Status( SYS_MODULE_OBJ object )

  Summary:
    Gets the current status of the EEPROM driver module.

  Description:
    This routine provides the current status of the EEPROM driver module.

  Remarks:
    Refer to drv_eeprom.h for usage information.
*/

SYS_STATUS DRV_EEPROM_Status
(
    SYS_MODULE_OBJ object
)
{
    if ((object == SYS_MODULE_OBJ_INVALID) || (object >= DRV_EEPROM_INSTANCES_NUMBER))
    {
        return SYS_STATUS_UNINITIALIZED;
    }

    return (gDrvEEPROMObj[object].status);
}

// *****************************************************************************
// *****************************************************************************
// Section: EEPROM Driver Client Routines
// *****************************************************************************
// *****************************************************************************

// ****************************************************************************
/* Function:
    DRV_HANDLE DRV_EEPROM_Open
    ( 
        const SYS_MODULE_INDEX drvIndex,
        const DRV_IO_INTENT ioIntent
    );
    
  Summary:
    Opens the specified EEPROM driver instance and returns a handle to it
  
  Description:
    This routine opens the specified EEPROM driver instance and provides a
    handle. This handle must be provided to all other client-level operations
    to identify the caller and the instance of the driver.
  
  Remarks:
    Refer to drv_eeprom.h for usage information.
*/

DRV_HANDLE DRV_EEPROM_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
    DRV_EEPROM_CLIENT_OBJECT *clientObj = (DRV_EEPROM_CLIENT_OBJECT *)gDrvEEPROMClientObj;
    DRV_EEPROM_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;
    DRV_HANDLE drvHandle = DRV_HANDLE_INVALID;
    uint8_t iClient = 0;

    if (drvIndex >= DRV_EEPROM_INSTANCES_NUMBER)
    {
        /* Invalid driver index. */
        return DRV_HANDLE_INVALID;
    }

    dObj = &gDrvEEPROMObj[drvIndex];

    if (dObj->status != SYS_STATUS_READY)
    {
        /* The driver is not ready to be opened. */
        return DRV_HANDLE_INVALID;
    }

    if (dObj->isExclusive)
    {
        /* The driver has already been opened in exclusive mode. */
        return DRV_HANDLE_INVALID;
    }

    if ((dObj->numClients > 0) && (ioIntent & DRV_IO_INTENT_EXCLUSIVE))
    {
        /* Driver has already been opened and cannot be opened exclusively */
        return DRV_HANDLE_INVALID;
    }

    retVal = OSAL_MUTEX_Lock(&eepromClientObjMutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        /* Could not obtain the Client object mutex */
        return DRV_HANDLE_INVALID;
    }

    /* Find available slot in array of client objects */
    for (iClient = 0; iClient < DRV_EEPROM_CLIENTS_NUMBER; iClient++)
    {
        if (!clientObj->inUse)
        {
            /* Found a client object that can be used */
            clientObj->inUse = true;
            clientObj->driverObj = dObj;
            clientObj->intent = ioIntent;
            clientObj->eventHandler = NULL;

            if (ioIntent & DRV_IO_INTENT_EXCLUSIVE)
            {
                /* Exclusive mode of access */
                dObj->isExclusive = true;
            }

            dObj->numClients ++;
            drvHandle = (DRV_HANDLE)clientObj;

            break;
        }

        clientObj += 1;
    }

    OSAL_MUTEX_Unlock(&eepromClientObjMutex);
    return drvHandle;
}

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_EEPROM_Close( DRV_Handle handle )

  Summary:
    Closes an opened-instance of the EEPROM driver

  Description:
    This routine closes an opened-instance of the EEPROM driver, invalidating the
    handle.

  Remarks:
    Refer to drv_eeprom.h for usage infomration.
*/

void DRV_EEPROM_Close
(
    const DRV_HANDLE handle
)
{
    DRV_EEPROM_CLIENT_OBJECT *clientObj = NULL;
    DRV_EEPROM_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    clientObj = DRV_EEPROM_ClientHandleValidate(handle);
    if (clientObj == NULL)
    {
        /* Invalid driver handle. */
        return;
    }

    dObj = clientObj->driverObj;

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        return;
    }

    DRV_EEPROM_RemoveClientBufferObjects (handle, dObj);

    /* Update the client count */
    dObj->numClients --;
    dObj->isExclusive = false;
    OSAL_MUTEX_Unlock(&dObj->mutex);

    retVal = OSAL_MUTEX_Lock(&eepromClientObjMutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        /* Could not obtain the Client object mutex */
        return;
    }

    /* Free the Client Instance */
    clientObj->inUse = false;

    OSAL_MUTEX_Unlock(&eepromClientObjMutex);
    return;
}

// *****************************************************************************
/* Function:
    void DRV_EEPROM_Read
    (
        const DRV_HANDLE handle,
        DRV_EEPROM_COMMAND_HANDLE * commandHandle,
        void * buffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Reads blocks of data from the specified address in EEPROM memory.

  Description:
    This function schedules a non-blocking read operation for reading blocks of
    data from the EEPROM memory. The function returns with a valid handle in
    the commandHandle argument if the read request was scheduled successfully.
    The function adds the request to the driver instance queue and returns
    immediately. While the request is in the queue, the application buffer is
    owned by the driver and should not be modified. The function returns
    DRV_EEPROM_COMMAND_HANDLE_INVALID in the commandHandle argument under the
    following circumstances:
    - if a buffer object could not be allocated to the request
    - if the buffer pointer is NULL
    - if the queue size is full or queue depth is insufficient
    - if the driver handle is invalid
    - if the number of blocks to be read is zero or more than the actual number
      of blocks available
    - if the client opened the driver in write only mode

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_EEPROM_EVENT_COMMAND_COMPLETE event if the command
    was processed successfully or DRV_EEPROM_EVENT_COMMAND_ERROR event if the
    command was not processed successfully.

  Remarks:
    Refer to drv_eeprom.h for usage information.
*/

void DRV_EEPROM_Read
(
    const DRV_HANDLE handle,
    DRV_EEPROM_COMMAND_HANDLE *commandHandle,
    void    *targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_EEPROM_AddRequest (handle, commandHandle, targetBuffer, blockStart, nBlock, DRV_EEPROM_BUFFER_OPERATION_READ);
}

// *****************************************************************************
/* Function:
    void DRV_EEPROM_Write
    (
        const DRV_HANDLE handle,
        DRV_EEPROM_COMMAND_HANDLE * commandHandle,
        void * buffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Writes blocks of data starting from the specified address in EEPROM memory.

  Description:
    This function schedules a non-blocking write operation for writing blocks
    of data into memory. The function returns with a valid handle in the
    commandHandle argument if the write request was scheduled successfully. The
    function adds the request to the hardware instance queue and returns
    immediately. While the request is in the queue, the application buffer is
    owned by the driver and should not be modified. The function returns
    DRV_EEPROM_COMMAND_HANDLE_INVALID in the commandHandle argument under the
    following circumstances:
    - if a buffer object could not be allocated to the request
    - if the buffer pointer is NULL
    - if the client opened the driver for read only
    - if the number of blocks to be written is either zero or more than the
      number of blocks actually available
    - if the write queue size is full or queue depth is insufficient
    - if the driver handle is invalid

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_EEPROM_EVENT_COMMAND_COMPLETE event if the command
    was processed successfully or DRV_EEPROM_EVENT_COMMAND_ERROR event if the
    command was not processed successfully.

  Remarks:
    Refer to drv_eeprom.h for usage information.
*/

void DRV_EEPROM_Write
(
    const DRV_HANDLE handle,
    DRV_EEPROM_COMMAND_HANDLE *commandHandle,
    void *sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_EEPROM_AddRequest (handle, commandHandle, sourceBuffer, blockStart, nBlock, DRV_EEPROM_BUFFER_OPERATION_WRITE);
}

// *****************************************************************************
/* Function:
    void DRV_EEPROM_Erase
    (
        const DRV_HANDLE handle,
        DRV_EEPROM_COMMAND_HANDLE * commandHandle,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Erases blocks of data starting from the specified block address.

  Description:
    This function schedules a non-blocking erase operation for erasing blocks
    of memory. The function returns with a valid handle in the commandHandle
    argument if the erase request was scheduled successfully. The function adds
    the request to the hardware instance queue and returns immediately. The
    function returns DRV_EEPROM_COMMAND_HANDLE_INVALID in the commandHandle
    argument under the following circumstances:
    - if a buffer object could not be allocated to the request
    - if the client opened the driver for read only
    - if the number of blocks to be erased is either zero or more than the
      number of blocks actually available
    - if the driver handle is invalid

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_EEPROM_EVENT_COMMAND_COMPLETE event if the command
    was processed successfully or DRV_EEPROM_EVENT_COMMAND_ERROR event if the
    command was not processed successfully.

  Remarks:
    Refer to drv_eeprom.h for usage information.
*/

void DRV_EEPROM_Erase
(
    const DRV_HANDLE handle,
    DRV_EEPROM_COMMAND_HANDLE *commandHandle,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_EEPROM_AddRequest (handle, commandHandle, NULL, blockStart, nBlock, DRV_EEPROM_BUFFER_OPERATION_ERASE);
}

// *****************************************************************************
/* Function:
    void DRV_EEPROM_BulkErase
    (
        const DRV_HANDLE handle,
        DRV_EEPROM_COMMAND_HANDLE * commandHandle
    );

  Summary:
    Performs a bulk erase of the entire Data EEPROM.

  Description:
    This function schedules a non-blocking bulk erase operation of the entire
    Data EEPROM.  The function returns with a valid handle in the commandHandle
    argument if the erase request was scheduled successfully. The function adds
    the request to the hardware instance queue and returns immediately. The
    function returns DRV_EEPROM_COMMAND_HANDLE_INVALID in the commandHandle
    argument under the following circumstances:
    - if a buffer object could not be allocated to the request
    - if the client opened the driver for read only
    - if the driver handle is invalid

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_EEPROM_EVENT_COMMAND_COMPLETE event if the command
    was processed successfully or DRV_EEPROM_EVENT_COMMAND_ERROR event if the
    command was not processed successfully.

  Remarks:
    Refer to drv_eeprom.h for usage information.
*/

void DRV_EEPROM_BulkErase
(
    const DRV_HANDLE handle,
    DRV_EEPROM_COMMAND_HANDLE *commandHandle
)
{
    DRV_EEPROM_AddRequest (handle, commandHandle, NULL, 0, 0, DRV_EEPROM_BUFFER_OPERATION_BULK_ERASE);
}

void DRV_EEPROM_ProcessRequest
(
    DRV_EEPROM_OBJECT *dObj
)
{
    DRV_EEPROM_BUFFER_OBJECT *bufferObj = NULL;
    DRV_EEPROM_CLIENT_OBJECT *clientObj = NULL;
    DRV_EEPROM_EVENT event = DRV_EEPROM_EVENT_COMMAND_COMPLETE;
    uint32_t data = 0;
    
    bufferObj = dObj->queue;
    if (bufferObj == NULL)
    {
        dObj->reqState = DRV_EEPROM_PROCESS_REQ_START;
        return;
    }

    if ((dObj->status != SYS_STATUS_READY) && (dObj->reqState != DRV_EEPROM_PROCESS_BOR_RESET))
    {
        /* The driver is not ready to process any requests. Wait
         * until the driver becomes ready. */
        return;
    }

    switch (dObj->reqState)
    {
        case DRV_EEPROM_PROCESS_REQ_START:
        default:
            {
                bufferObj->status = DRV_EEPROM_COMMAND_IN_PROGRESS;
                if (bufferObj->operation == DRV_EEPROM_BUFFER_OPERATION_READ)
                {
                    dObj->reqState = DRV_EEPROM_PROCESS_READ;
                }
                else if (bufferObj->operation == DRV_EEPROM_BUFFER_OPERATION_WRITE)
                {
                    dObj->reqState = DRV_EEPROM_PROCESS_WRITE;
                }
                else if (bufferObj->operation == DRV_EEPROM_BUFFER_OPERATION_ERASE)
                {
                    dObj->reqState = DRV_EEPROM_PROCESS_ERASE;
                }
                else
                {
                    dObj->reqState = DRV_EEPROM_PROCESS_BULK_ERASE;
                }
                break;
            }

        case DRV_EEPROM_PROCESS_READ:
            {
                PLIB_NVM_EEPROMReadEnable(dObj->moduleId);
                PLIB_NVM_EEPROMAddress(dObj->moduleId, bufferObj->address << 2);
                PLIB_NVM_EEPROMOperationSelect(dObj->moduleId, EEPROM_WORD_READ_OPERATION);
                PLIB_NVM_EEPROMReadStart(dObj->moduleId);
                dObj->reqState = DRV_EEPROM_PROCESS_READ_STATUS;

                break;
            }

        case DRV_EEPROM_PROCESS_READ_STATUS:
            {
                if (PLIB_NVM_EEPROMOperationHasCompleted(dObj->moduleId))
                {
                    dObj->error = PLIB_NVM_EEPROMErrorGet(dObj->moduleId);
                    if (dObj->error == NO_ERROR)
                    {
                        uint8_t *ptr = bufferObj->buffer;
                        bufferObj->address ++;
                        bufferObj->nBlocks --;
                        data = PLIB_NVM_EEPROMRead(dObj->moduleId);
                        ptr[0] = data;
                        ptr[1] = data >> 8;
                        ptr[2] = data >> 16;
                        ptr[3] = data >> 24;
                        bufferObj->buffer += 4;
                        if (bufferObj->nBlocks != 0)
                        {
                            dObj->reqState = DRV_EEPROM_PROCESS_READ;
                        }
                    }

                    if ((bufferObj->nBlocks == 0) || (dObj->error != NO_ERROR))
                    {
                        dObj->reqState = DRV_EEPROM_PROCESS_XFER_COMPLETION;
                    }
                }

                break;
            }

        case DRV_EEPROM_PROCESS_WRITE:
            {
                uint8_t *ptr = bufferObj->buffer;

                PLIB_NVM_EEPROMAddress (dObj->moduleId, bufferObj->address << 2);
                data = DRV_EEPROM_MAKE_WORD(ptr[0], ptr[1], ptr[2], ptr[3]);
                PLIB_NVM_EEPROMDataToWrite (dObj->moduleId, data);
                PLIB_NVM_EEPROMWriteEnable(dObj->moduleId);
                PLIB_NVM_EEPROMOperationSelect(dObj->moduleId, EEPROM_WORD_WRITE_OPERATION);
                DRV_EEPROM_UnlockAndWrite(dObj);
                dObj->reqState = DRV_EEPROM_PROCESS_WRITE_STATUS;

                break;
            }

        case DRV_EEPROM_PROCESS_WRITE_STATUS:
            {
                if (PLIB_NVM_EEPROMOperationHasCompleted(dObj->moduleId))
                {
                    dObj->error = PLIB_NVM_EEPROMErrorGet(dObj->moduleId);
                    if (dObj->error == NO_ERROR)
                    {
                        bufferObj->address ++;
                        bufferObj->nBlocks --;
                        bufferObj->buffer += 4;
                        if (bufferObj->nBlocks != 0)
                        {
                            dObj->reqState = DRV_EEPROM_PROCESS_WRITE;
                        }
                    }

                    if ((bufferObj->nBlocks == 0) || (dObj->error != NO_ERROR))
                    {
                        dObj->reqState = DRV_EEPROM_PROCESS_XFER_COMPLETION;
                    }
                }

                break;
            }

        case DRV_EEPROM_PROCESS_ERASE:
            {
                PLIB_NVM_EEPROMAddress (dObj->moduleId, bufferObj->address << 2);
                PLIB_NVM_EEPROMWriteEnable(dObj->moduleId);
                PLIB_NVM_EEPROMOperationSelect(dObj->moduleId, EEPROM_FORCED_WORD_ERASE_OPERATION);
                DRV_EEPROM_UnlockAndWrite(dObj);
                dObj->reqState = DRV_EEPROM_PROCESS_ERASE_STATUS;

                break;
            }

        case DRV_EEPROM_PROCESS_ERASE_STATUS:
            {
                if (PLIB_NVM_EEPROMOperationHasCompleted(dObj->moduleId))
                {
                    dObj->error = PLIB_NVM_EEPROMErrorGet(dObj->moduleId);
                    if (dObj->error == NO_ERROR)
                    {
                        bufferObj->address ++;
                        bufferObj->nBlocks --;
                        if (bufferObj->nBlocks != 0)
                        {
                            dObj->reqState = DRV_EEPROM_PROCESS_ERASE;
                        }
                    }

                    if ((bufferObj->nBlocks == 0) || (dObj->error != NO_ERROR))
                    {
                        dObj->reqState = DRV_EEPROM_PROCESS_XFER_COMPLETION;
                    }
                }
                break;
            }

        case DRV_EEPROM_PROCESS_BULK_ERASE:
            {
                PLIB_NVM_EEPROMWriteEnable(dObj->moduleId);
                PLIB_NVM_EEPROMOperationSelect(dObj->moduleId, EEPROM_ERASE_ALL_OPERATION);
                DRV_EEPROM_UnlockAndWrite(dObj);
                dObj->reqState = DRV_EEPROM_PROCESS_BULK_ERASE_STATUS;
                break;
            }

        case DRV_EEPROM_PROCESS_BULK_ERASE_STATUS:
            {
                if (PLIB_NVM_EEPROMOperationHasCompleted(dObj->moduleId))
                {
                    dObj->error = PLIB_NVM_EEPROMErrorGet(dObj->moduleId);
                    dObj->reqState = DRV_EEPROM_PROCESS_XFER_COMPLETION;
                }
                break;
            }

        case DRV_EEPROM_PROCESS_XFER_COMPLETION:
            {
                if (dObj->error != NO_ERROR)
                {
                    PLIB_NVM_EEPROMErrorClear (dObj->moduleId);
                    bufferObj->status = DRV_EEPROM_COMMAND_ERROR_UNKNOWN;
                    event = DRV_EEPROM_EVENT_COMMAND_ERROR;
                }
                else
                {
                    bufferObj->status = DRV_EEPROM_COMMAND_COMPLETED;
                    event = DRV_EEPROM_EVENT_COMMAND_COMPLETE;
                }

                clientObj = (DRV_EEPROM_CLIENT_OBJECT *)bufferObj->hClient;
                if (clientObj->eventHandler != NULL)
                {
                    clientObj->eventHandler(event, (DRV_EEPROM_COMMAND_HANDLE)bufferObj->commandHandle, clientObj->context);
                }

                DRV_EEPROM_UpdateQueue (dObj);

                if (dObj->error == BOR_ERROR)
                {
                    /* BOR has happened. Handle the reset. */
                    dObj->status = SYS_STATUS_BUSY;
                    dObj->reqState = DRV_EEPROM_PROCESS_BOR_RESET;
                }
                else
                {
                    dObj->reqState = DRV_EEPROM_PROCESS_REQ_START;
                }

                /* Reset the error. */
                dObj->error = NO_ERROR;
                break;
            }

        case DRV_EEPROM_PROCESS_BOR_RESET:
            {
                /* The errors have already been cleared. As part of the BOR
                 * event handling, wait for the EEPROM RDY bit to be set. */
                if (PLIB_NVM_EEPROMIsReady(dObj->moduleId) == true)
                {
                    dObj->status = SYS_STATUS_READY;
                    dObj->reqState = DRV_EEPROM_PROCESS_REQ_START;
                }
                break;
            }
    }
}

// ****************************************************************************
/* Function:
    void DRV_EEPROM_Tasks 
    (
        SYS_MODULE_OBJ object
    );
    
  Summary:
    Handles the read, write or erase requests queued to the driver.
  
  Description:
    This routine is used to handle the read, write or erase requests queued to
    the driver.

  Remarks:
    Refer to drv_eeprom.h for usage information.
*/

void DRV_EEPROM_Tasks
(
    SYS_MODULE_OBJ object
)
{
    DRV_EEPROM_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;
    
    if(SYS_MODULE_OBJ_INVALID == object)
    {
        return;
    }

    dObj = &gDrvEEPROMObj[object];

    retVal = OSAL_MUTEX_Lock(&dObj->mutex, OSAL_WAIT_FOREVER);
    if (retVal != OSAL_RESULT_TRUE)
    {
        return;
    }

    switch (dObj->taskState)
    {
        case DRV_EEPROM_TASK_EEPROM_READY:
            {
                /* Check if the EEPROM is ready. Approx. 125 us for the EEPROM
                 * to become ready. */
                if (PLIB_NVM_EEPROMIsReady(dObj->moduleId) == true)
                {
                    dObj->taskState = DRV_EEPROM_TASK_INIT;
                }
                else
                {
                    /* Continue to stay in the same state until EEPROM becomes
                     * ready for operation. */
                }
                break;
            }

        case DRV_EEPROM_TASK_INIT:
            {
                if (DRV_EEPROM_Configure(dObj))
                {
                    dObj->status = SYS_STATUS_READY;
                    DRV_EEPROM_RegisterWithSysFs (dObj->drvIndex, dObj->drvIndex, eepromMediaFunctions);
                    dObj->taskState = DRV_EEPROM_TASK_PROCESS_QUEUE;
                }
                else
                {
                    /* Continue to stay in the same state until the calibration
                     * is not completed. */
                }
                break;
            }

        case DRV_EEPROM_TASK_PROCESS_QUEUE:
            {
                DRV_EEPROM_ProcessRequest (dObj);
                break;
            }

        case DRV_EEPROM_TASK_IDLE:
            {
                break;
            }
    }

    OSAL_MUTEX_Unlock(&dObj->mutex);

    return;
}

// *****************************************************************************
/* Function:
    DRV_EEPROM_COMMAND_STATUS DRV_EEPROM_CommandStatus
    (
        const DRV_HANDLE handle, 
        const DRV_EEPROM_COMMAND_HANDLE commandHandle
    );

  Summary:
    Gets the current status of the command.

  Description:
    This routine gets the current status of the command. The application must
    use this routine where the status of a scheduled command needs to be polled
    on. The function may return DRV_EEPROM_COMMAND_HANDLE_INVALID in a case
    where the command handle has expired. A command handle expires when the
    internal buffer object is re-assigned to another read, write or erase
    request. It is recommended that this function be called regularly in order
    to track the command status correctly.

    The application can alternatively register an event handler to receive
    read, write or erase operation completion events.

  Remarks:
    Refer to drv_eeprom.h for usage information.
*/

DRV_EEPROM_COMMAND_STATUS DRV_EEPROM_CommandStatus
(
    const DRV_HANDLE handle,
    const DRV_EEPROM_COMMAND_HANDLE commandHandle
)
{
    uint16_t iEntry;

    /* Validate the client handle */
    if (DRV_EEPROM_ClientHandleValidate(handle) == NULL)
    {
        return DRV_EEPROM_COMMAND_HANDLE_INVALID;
    }

    /* The upper 16 bits of the buffer handle are the token and the lower 16
     * bits of the are buffer index into the gDrvEEPROMBufferObject array */
    iEntry = commandHandle & 0xFFFF;

    /* Compare the buffer handle with buffer handle in the object */
    if (gDrvEEPROMBufferObject[iEntry].commandHandle != commandHandle)
    {
        /* This means that object has been re-used by another request. Indicate
         * that the operation is completed.  */
        return (DRV_EEPROM_COMMAND_COMPLETED);
    }

    /* Return the last known buffer object status */
    return (gDrvEEPROMBufferObject[iEntry].status);
}

// *****************************************************************************
/* Function:
    void DRV_EEPROM_EventHandlerSet
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
    calls a read, write or a erase function, it is provided with a handle
    identifying the command that was added to the driver's command queue. The
    driver will pass this handle back to the client by calling "eventHandler"
    function when the queued operation has completed.
    
    The event handler should be set before the client performs any read, write
    or erase operations that could generate events. The event handler once set,
    persists until the client closes the driver or sets another event handler
    (which could be a "NULL" pointer to indicate no callback).

  Remarks:
    Refer to drv_eeprom.h for usage information.
*/

void DRV_EEPROM_EventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
)
{
    DRV_EEPROM_CLIENT_OBJECT * clientObj;

    clientObj = DRV_EEPROM_ClientHandleValidate(handle);
    /* Check if the client handle is valid */
    if (clientObj != NULL)
    {
        /* Set the event handler */
        clientObj->eventHandler = eventHandler;
        clientObj->context = context;
    }
}

// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_GEOMETRY * DRV_EEPROM_GeometryGet
    (
        const DRV_HANDLE handle
    );

  Summary:
    Returns the geometry of the device.

  Description:
    This API gives the following geometrical details of the EEPROM memory:
    - Media Property
    - Number of Read/Write/Erase regions
    - Number of Blocks and their size in each region of the device

  Remarks:
    Refer to drv_eeprom.h for usage information.
*/

SYS_FS_MEDIA_GEOMETRY * DRV_EEPROM_GeometryGet
(
    const DRV_HANDLE handle
)
{
    DRV_EEPROM_CLIENT_OBJECT *clientObj = NULL;
    DRV_EEPROM_OBJECT *dObj = NULL;

    /* Validate the handle */
    clientObj = DRV_EEPROM_ClientHandleValidate(handle);
    if (clientObj == NULL)
    {
        return NULL;
    }

    dObj = (DRV_EEPROM_OBJECT *)clientObj->driverObj;
    return (dObj->mediaGeometryObj);
}

// *****************************************************************************
/* Function:
    bool DRV_EEPROM_IsAttached
    ( 
        const DRV_HANDLE handle 
    );

  Summary:
    Returns the physical attach status of the EEPROM.

  Description:
    This function returns the physical attach status of the EEPROM.

  Remarks:
    Refer to drv_eeprom.h for usage information.
*/

bool DRV_EEPROM_IsAttached
(
    const DRV_HANDLE handle
)
{
    DRV_EEPROM_CLIENT_OBJECT *clientObj = NULL;

    /* Validate the handle */
    clientObj = DRV_EEPROM_ClientHandleValidate(handle);
    if (clientObj == NULL)
    {
        return false;
    }

    return true;
}

// *****************************************************************************
/* Function:
    bool DRV_EEPROM_IsWriteProtected
    ( 
        const DRV_HANDLE handle 
    );

  Summary:
    Returns the write protect status of the EEPROM.

  Description:
    This function returns the physical attach status of the EEPROM. This function
    always returns false.

  Remarks:
    Refer to drv_eeprom.h for usage information.
*/

bool DRV_EEPROM_IsWriteProtected
(
    const DRV_HANDLE handle
)
{
    return false;
}

// *****************************************************************************
/* Function:
    uintptr_t DRV_EEPROM_AddressGet
    (
        const DRV_HANDLE handle
    );

  Summary:
    Returns the EEPROM media start address

  Description:
    This function returns the EEPROM Media start address.

  Remarks:
    Refer drv_eeprom.h for usage information.
*/

uintptr_t DRV_EEPROM_AddressGet
(
    const DRV_HANDLE handle
)
{
    DRV_EEPROM_CLIENT_OBJECT *clientObj = NULL;
    DRV_EEPROM_OBJECT *dObj = NULL;

    /* Validate the handle */
    clientObj = DRV_EEPROM_ClientHandleValidate(handle);
    if (clientObj == NULL)
    {
        return NULL;
    }

    dObj = (DRV_EEPROM_OBJECT *)clientObj->driverObj;
    return (dObj->blockStartAddress);
}

