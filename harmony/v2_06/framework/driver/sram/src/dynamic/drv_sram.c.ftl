/*******************************************************************************
  SRAM Driver Interface Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sram.c

  Summary:
    SRAM Driver Interface Definition

  Description:
    The SRAM Driver provides a interface to access the SRAM on the PIC32
    microcontroller. This file implements the SRAM Driver interface. This file
    should be included in the project if SRAM driver functionality is needed.
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

#include "driver/sram/src/drv_sram_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global objects
// *****************************************************************************
// *****************************************************************************

/*************************************************
 * Hardware instance objects
 *************************************************/

DRV_SRAM_OBJECT gDrvSRAMObj[DRV_SRAM_INSTANCES_NUMBER];

/*************************************************
 * Driver Client Objects
 *************************************************/

DRV_SRAM_CLIENT_OBJECT gDrvSRAMClientObj[DRV_SRAM_CLIENTS_NUMBER];

/*************************************************
 * Driver Buffer Objects. These transport the
 * read, write and erase requests.
 *************************************************/

DRV_SRAM_BUFFER_OBJECT gDrvSRAMBufferObject;

/************************************************
 * This token is incremented for every request
 * added to the queue and is used to generate
 * a different buffer handle for every request.
 ***********************************************/

uint16_t gDrvSRAMBufferToken = 0;

uint8_t gDrvSRAMInitCount = 0;
/*************************************************
 * OSAL Declarations
 *************************************************/
/* SRAM Client Object Mutex */
OSAL_MUTEX_DECLARE(gDrvSRAMClientObjMutex);

/* SRAM Buffer Object Mutex */
OSAL_MUTEX_DECLARE(gDrvSRAMBufObjMutex);

const SYS_FS_MEDIA_FUNCTIONS sramMediaFunctions =
{
    .mediaStatusGet     = DRV_SRAM_IsAttached,
    .mediaGeometryGet   = DRV_SRAM_GeometryGet,
    .sectorRead         = DRV_SRAM_Read,
    .sectorWrite        = DRV_SRAM_Write,
    .eventHandlerset    = DRV_SRAM_EventHandlerSet,
    .commandStatusGet   = (void *)DRV_SRAM_CommandStatus,
    .Read               = DRV_SRAM_Read,
    .erase              = NULL,
    .addressGet         = DRV_SRAM_AddressGet,
    .open               = DRV_SRAM_Open,
    .close              = DRV_SRAM_Close,
    .tasks              = NULL,
};

// *****************************************************************************
/* Function:
    static DRV_SRAM_CLIENT_OBJECT * DRV_SRAM_ValidateClientHandle
    (
        DRV_HANDLE handle
    )

  Summary:
    This function validates the driver handle.

  Description:
    This function validates the driver handle and returns the client object
    pointer associated with the driver handle if the handle is valid. If the
    driver handle is not valid or if the driver state is not ready then NULL is
    returned.

  Remarks:
    None
*/

static DRV_SRAM_CLIENT_OBJECT * DRV_SRAM_ValidateClientHandle
(
    DRV_HANDLE handle
)
{
    DRV_SRAM_CLIENT_OBJECT *clientObj = NULL;
    DRV_SRAM_OBJECT *dObj = NULL;

    /* Validate the handle */
    _DRV_SRAM_VALIDATE_EXPR((0 == handle), NULL);
    _DRV_SRAM_VALIDATE_EXPR((DRV_HANDLE_INVALID == handle), NULL);

    /* See if the client has been opened */
    clientObj = (DRV_SRAM_CLIENT_OBJECT *)handle;
    _DRV_SRAM_VALIDATE_EXPR((!clientObj->inUse), NULL);

    /* Check if the driver is ready for operation */
    dObj = (DRV_SRAM_OBJECT *)clientObj->driverObj;
    _DRV_SRAM_VALIDATE_EXPR((dObj->status != SYS_STATUS_READY), NULL);
    (void)dObj;

    return clientObj;
}

static void DRV_SRAM_XferData
(
    const DRV_HANDLE handle,
    DRV_SRAM_COMMAND_HANDLE *commandHandle,
    bool isRead,
    void *buffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SRAM_OBJECT *dObj = NULL;
    DRV_SRAM_COMMAND_HANDLE *tempHandle1, tempHandle2;
    DRV_SRAM_CLIENT_OBJECT *clientObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    DRV_IO_INTENT ioIntent = DRV_IO_INTENT_READ;
    uint8_t entry = GEOMETRY_TABLE_READ_ENTRY;

    uint8_t *source = NULL;
    uint8_t *destination = NULL;

    uint32_t temp = 0;
    uint32_t numBytes = 0;

    if (!isRead)
    {
        ioIntent = DRV_IO_INTENT_WRITE;
        entry = GEOMETRY_TABLE_WRITE_ENTRY;
    }

    tempHandle1 = (commandHandle == NULL) ? &tempHandle2 : commandHandle;
    *tempHandle1 = DRV_SRAM_COMMAND_HANDLE_INVALID;

    clientObj = DRV_SRAM_ValidateClientHandle(handle);
    dObj = (DRV_SRAM_OBJECT *)clientObj->driverObj;

    _DRV_SRAM_VALIDATE_EXPR_VOID((clientObj == NULL));
    _DRV_SRAM_VALIDATE_EXPR_VOID(((clientObj->intent & ioIntent) == 0));
    _DRV_SRAM_VALIDATE_EXPR_VOID((buffer == NULL));
    _DRV_SRAM_VALIDATE_EXPR_VOID((nBlock == 0));
    _DRV_SRAM_VALIDATE_EXPR_VOID(((blockStart + nBlock) > dObj->sramMediaGeometry->geometryTable[entry].numBlocks));
    (void)ioIntent;

    /* Acquire Buffer Object Mutex */
    retVal = OSAL_MUTEX_Lock(&gDrvSRAMBufObjMutex, OSAL_WAIT_FOREVER);
    _DRV_SRAM_VALIDATE_EXPR_VOID((retVal != OSAL_RESULT_TRUE));
    (void)retVal;

    *tempHandle1 = _DRV_SRAM_MAKE_HANDLE(gDrvSRAMBufferToken, blockStart & 0xFFFF);
    gDrvSRAMBufferObject.commandHandle = *tempHandle1;
    /* Update the token number. */
    _DRV_SRAM_UPDATE_BUF_TOKEN(gDrvSRAMBufferToken);

    temp = dObj->sramMediaGeometry->geometryTable[entry].blockSize;

    if (isRead)
    {
        source = (uint8_t *)dObj->blockStartAddress + (blockStart * temp);
        destination = buffer;
    }
    else
    {
        source = buffer;
        destination = (uint8_t *)dObj->blockStartAddress + (blockStart * temp);
    }

    /* Find the number of bytes to be read/written. */
    numBytes = nBlock * temp;

    for (temp = 0; temp < numBytes; temp++)
    {
        *destination++ = *source++;
    }

    if (!isRead)
    {
        destination = (uint8_t *)dObj->blockStartAddress + (blockStart * temp);
        /* Invalidate the cache for this address range. */
        DRV_SRAM_INVALIDATE_CACHE((uint32_t)destination, numBytes);
    }

    gDrvSRAMBufferObject.status = DRV_SRAM_COMMAND_COMPLETED;
    if(clientObj->eventHandler != NULL)
    {
        clientObj->eventHandler(DRV_SRAM_EVENT_COMMAND_COMPLETE, (DRV_SRAM_COMMAND_HANDLE)gDrvSRAMBufferObject.commandHandle, clientObj->context);
    }

    /* Release Buffer Object Mutex */
    OSAL_MUTEX_Unlock(&gDrvSRAMBufObjMutex);

    return;
}

// *****************************************************************************
// *****************************************************************************
// Section: SRAM Driver System Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SRAM_Initialize
    (
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init
    )

  Summary:
    Initializes the SRAM instance for the specified driver index

  Description:
    This routine initializes the SRAM driver instance for the specified
    driver index, making it ready for clients to open and use it.

  Remarks:
    Refer to drv_sram.h for usage information.
*/

SYS_MODULE_OBJ DRV_SRAM_Initialize
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT *const init
)
{
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;
    DRV_SRAM_OBJECT *dObj = NULL;
    DRV_SRAM_INIT *sramInit = NULL;

    /* Validate the driver index */
    _DRV_SRAM_VALIDATE_EXPR((drvIndex > DRV_SRAM_INSTANCES_NUMBER), SYS_MODULE_OBJ_INVALID);

    /* Validate the init parameter */
    _DRV_SRAM_VALIDATE_EXPR((init == NULL), SYS_MODULE_OBJ_INVALID);

    /* Instance has already been initialized */
    _DRV_SRAM_VALIDATE_EXPR((gDrvSRAMObj[drvIndex].inUse), SYS_MODULE_OBJ_INVALID);

    /* Assign to the local pointer the init data passed */
    sramInit = (DRV_SRAM_INIT *)init;

    if (gDrvSRAMInitCount == 0)
    {
        retVal = OSAL_MUTEX_Create(&gDrvSRAMClientObjMutex);
        _DRV_SRAM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), SYS_MODULE_OBJ_INVALID);
        (void)retVal;

        retVal = OSAL_MUTEX_Create(&gDrvSRAMBufObjMutex);
        _DRV_SRAM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), SYS_MODULE_OBJ_INVALID);

        (void)retVal;
        gDrvSRAMInitCount++;
    }

    dObj = &gDrvSRAMObj[drvIndex];

    /* Indicate that this object is in use */
    dObj->inUse = true;

    /* Initialize number of clients */
    dObj->numClients = 0;

    dObj->blockStartAddress = (uintptr_t)sramInit->mediaStartAddress;
    dObj->sramMediaGeometry = (SYS_FS_MEDIA_GEOMETRY *)sramInit->sramMediaGeometry;
<#if CONFIG_USE_DRV_SRAM_SYS_FS_REGISTER_IDX0 == true>
    if (sramInit->registerWithFs)
    {
        /* Register the SRAM media driver with the file system. */
        if (SYS_FS_MEDIA_HANDLE_INVALID == SYS_FS_MEDIA_MANAGER_Register(drvIndex, drvIndex, &sramMediaFunctions, SYS_FS_MEDIA_TYPE_RAM))
        {
            return SYS_MODULE_OBJ_INVALID;
        }
    }
</#if>
    /* Set the current driver state */
    dObj->status = SYS_STATUS_READY;

    /* Return the driver index as the System Module Object */
    return (SYS_MODULE_OBJ)drvIndex;
}

// ****************************************************************************
/* Function:
    void DRV_SRAM_Deinitialize
    (
        SYS_MODULE_OBJ object
    )

  Summary:
    Deinitializes the specified instance of the SRAM driver module.

  Description:
    Deinitializes the specified instance of the SRAM driver module, disabling
    its operation (and any hardware). Invalidates all the internal data.

  Remarks:
    Refer to drv_sram.h for usage information.
*/

void DRV_SRAM_Deinitialize
(
    SYS_MODULE_OBJ object
)
{
    DRV_SRAM_OBJECT *dObj = (DRV_SRAM_OBJECT*)NULL;

    /* Validate the object */
    _DRV_SRAM_VALIDATE_EXPR_VOID((object == SYS_MODULE_OBJ_INVALID));
    _DRV_SRAM_VALIDATE_EXPR_VOID((object > DRV_SRAM_INSTANCES_NUMBER));

    dObj = (DRV_SRAM_OBJECT*)&gDrvSRAMObj[object];

    /* Reset the client count and the exclusive flag */
    dObj->numClients = 0;
    dObj->isExclusive = false;

    /* Set the Hardware instance object status an un-initialized */
    dObj->status = SYS_STATUS_UNINITIALIZED;

    gDrvSRAMInitCount--;
    if (gDrvSRAMInitCount == 0)
    {
        OSAL_MUTEX_Delete(&gDrvSRAMClientObjMutex);
        OSAL_MUTEX_Delete(&gDrvSRAMBufObjMutex);
    }

    /* Hardware instance object is no more in use */
    dObj->inUse = false;

    return;
}

// ****************************************************************************
/* Function:
    SYS_STATUS DRV_SRAM_Status( SYS_MODULE_OBJ object )

  Summary:
    Gets the current status of the SRAM driver module.

  Description:
    This routine provides the current status of the SRAM driver module.

  Remarks:
    Refer to drv_sram.h for usage information.
*/

SYS_STATUS DRV_SRAM_Status
(
    SYS_MODULE_OBJ object
)
{
    /* Validate the object */
    _DRV_SRAM_VALIDATE_EXPR((SYS_MODULE_OBJ_INVALID == object), SYS_STATUS_UNINITIALIZED);
    _DRV_SRAM_VALIDATE_EXPR((object > DRV_SRAM_INSTANCES_NUMBER), SYS_STATUS_UNINITIALIZED);

    /* Return the driver status */
    return (gDrvSRAMObj[object].status);
}

// *****************************************************************************
// *****************************************************************************
// Section: SRAM Driver Client Routines
// *****************************************************************************
// *****************************************************************************

// ****************************************************************************
/* Function:
    DRV_HANDLE DRV_SRAM_Open
    (
        const SYS_MODULE_INDEX index,
        const DRV_IO_INTENT ioIntent
    )

  Summary:
    Opens the specified SRAM driver instance and returns a handle to it

  Description:
    This routine opens the specified SRAM driver instance and provides a handle.
    This handle must be provided to all other client-level operations to identify
    the caller and the instance of the driver.

  Remarks:
    Refer to drv_sram.h for usage information.
*/

DRV_HANDLE DRV_SRAM_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
    DRV_SRAM_CLIENT_OBJECT *clientObj = (DRV_SRAM_CLIENT_OBJECT *)gDrvSRAMClientObj;
    DRV_SRAM_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    uint8_t iClient = 0;
    DRV_HANDLE drvHandle = DRV_HANDLE_INVALID;

    /* Validate the driver index */
    _DRV_SRAM_VALIDATE_EXPR((drvIndex >= DRV_SRAM_INSTANCES_NUMBER), DRV_HANDLE_INVALID);

    dObj = &gDrvSRAMObj[drvIndex];

    /* Check if the driver is ready to be opened */
    _DRV_SRAM_VALIDATE_EXPR((dObj->status != SYS_STATUS_READY), DRV_HANDLE_INVALID);

    /* Check if the driver has already been opened in exclusive mode */
    _DRV_SRAM_VALIDATE_EXPR((dObj->isExclusive), DRV_HANDLE_INVALID);

    /* Driver has already been opened and cannot be opened exclusively */
    _DRV_SRAM_VALIDATE_EXPR(((dObj->numClients > 0) && (ioIntent & DRV_IO_INTENT_EXCLUSIVE)), DRV_HANDLE_INVALID);

    /* Obtain the Client object mutex */
    retVal = OSAL_MUTEX_Lock(&gDrvSRAMClientObjMutex,OSAL_WAIT_FOREVER);
    _DRV_SRAM_VALIDATE_EXPR((retVal != OSAL_RESULT_TRUE), DRV_HANDLE_INVALID);

     (void)retVal;

    /* Find available slot in array of client objects */
    for (iClient = 0; iClient < DRV_SRAM_CLIENTS_NUMBER; iClient++)
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

    OSAL_MUTEX_Unlock(&gDrvSRAMClientObjMutex);

    return drvHandle;
}

// *****************************************************************************
/* Function:
    void DRV_SRAM_Close
    (
        const DRV_HANDLE handle
    )

  Summary:
    Closes an opened-instance of the SRAM driver

  Description:
    This routine closes an opened-instance of the SRAM driver, invalidating the
    handle.

  Remarks:
    Refer to drv_sram.h for usage infomration.
*/

void DRV_SRAM_Close
(
    const DRV_HANDLE handle
)
{
    DRV_SRAM_CLIENT_OBJECT *clientObj = NULL;
    DRV_SRAM_OBJECT *dObj = NULL;
    OSAL_RESULT retVal = OSAL_RESULT_FALSE;

    /* Get the Client object from the handle passed */
    clientObj = DRV_SRAM_ValidateClientHandle(handle);
    /* Check if the driver handle is valid */
    _DRV_SRAM_VALIDATE_EXPR_VOID((NULL == clientObj));

    dObj = clientObj->driverObj;

    /* Obtain the Client object mutex */
    retVal = OSAL_MUTEX_Lock(&gDrvSRAMClientObjMutex,OSAL_WAIT_FOREVER);
    _DRV_SRAM_VALIDATE_EXPR_VOID((retVal != OSAL_RESULT_TRUE));
    (void)retVal;
    /* Update the client count */
    dObj->numClients --;
    dObj->isExclusive = false;

    /* Free the Client Instance */
    clientObj->inUse = false;

    OSAL_MUTEX_Unlock(&gDrvSRAMClientObjMutex);

    return;
}

// *****************************************************************************
/* Function:
    void DRV_SRAM_Read
    (
        const DRV_HANDLE handle,
        DRV_SRAM_COMMAND_HANDLE * commandHandle,
        void * targetBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    )

  Summary:
    Reads blocks of data from the specified block address.

  Description:
    This routine reads blocks of data from the specified block start address.
    This operation is blocking and returns with the required data in the target
    buffer. If a event handler has been registered to receive the driver events
    then the event handler will be called from within this function. The
    function returns DRV_SRAM_COMMAND_HANDLE_INVALID in the commandHandle
    argument under the following circumstances:
    - if the driver handle is invalid
    - if the driver state is not ready
    - if the target buffer pointer is NULL
    - if the number of blocks to be read is zero or more than the actual number
    of blocks available
    - if the client opened the driver in write only mode

  Remarks:
    Refer to drv_sram.h for usage information.
*/

void DRV_SRAM_Read
(
    const DRV_HANDLE handle,
    DRV_SRAM_COMMAND_HANDLE * commandHandle,
    void    *targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SRAM_XferData (handle, commandHandle, true, targetBuffer, blockStart, nBlock);
}


// *****************************************************************************
/* Function:
    void DRV_SRAM_Write
    (
        const DRV_HANDLE handle,
        DRV_SRAM_COMMAND_HANDLE * commandHandle,
        void * sourceBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    )

  Summary:
    Writes blocks of data starting at the specified block address.

  Description:
    This routine writes blocks of data starting at the specified block start
    address.  This operation is blocking and returns after having written the
    data. If a event handler has been registered to receive the driver events
    then the event handler will be called from within this function. The
    function returns DRV_SRAM_COMMAND_HANDLE_INVALID in the commandHandle
    argument under the following circumstances:
    - if the driver handle is invalid
    - if the driver state is not ready
    - if the source buffer pointer is NULL
    - if the number of blocks to be written is zero or more than the actual number
    of blocks available
    - if the client opened the driver in read only mode

  Remarks:
    Refer to drv_sram.h for usage information.
*/

void DRV_SRAM_Write
(
    const DRV_HANDLE handle,
    DRV_SRAM_COMMAND_HANDLE * commandHandle,
    void    *sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_SRAM_XferData (handle, commandHandle, false, sourceBuffer, blockStart, nBlock);
}

// *****************************************************************************
/* Function:
    DRV_SRAM_COMMAND_STATUS DRV_SRAM_CommandStatus
    (
        const DRV_HANDLE handle,
        const DRV_SRAM_COMMAND_HANDLE commandHandle
    );

  Summary:
    Gets the current status of the command.

  Description:
    This routine gets the current status of the buffer. The application must use
    this routine where the status of a scheduled buffer needs to polled on. The
    function may return DRV_SRAM_COMMAND_HANDLE_INVALID in a case where the buffer
    handle has expired. A buffer handle expires when the internal buffer object
    is re-assigned to another erase or write request. It is recommended that this
    function be called regularly in order to track the buffer status correctly.

    The application can alternatively register an event handler to receive write
    or erase operation completion events.

  Remarks:
    Refer to drv_sram.h for usage information.
*/

DRV_SRAM_COMMAND_STATUS DRV_SRAM_CommandStatus
(
    const DRV_HANDLE handle,
    const DRV_SRAM_COMMAND_HANDLE commandHandle
)
{
    /* Validate the client handle */
    _DRV_SRAM_VALIDATE_EXPR((NULL == DRV_SRAM_ValidateClientHandle(handle)),
            (DRV_SRAM_COMMAND_STATUS)DRV_SRAM_COMMAND_HANDLE_INVALID);

    /* Compare the buffer handle with buffer handle in the object */
    if (gDrvSRAMBufferObject.commandHandle != commandHandle)
    {
        /* This means that object has been re-used by another request. Indicate
         * that the operation is completed.  */
        return (DRV_SRAM_COMMAND_COMPLETED);
    }

    /* Return the last known buffer object status */
    return (gDrvSRAMBufferObject.status);
}

// *****************************************************************************
/* Function:
    void DRV_SRAM_EventHandlerSet
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
    Refer to drv_sram.h for usage information.
*/

void DRV_SRAM_EventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
)
{
    DRV_SRAM_CLIENT_OBJECT * clientObj;

    clientObj = DRV_SRAM_ValidateClientHandle(handle);
    /* Check if the client handle is valid */
    _DRV_SRAM_VALIDATE_EXPR_VOID((NULL == clientObj));

    /* Set the event handler */
    clientObj->eventHandler = eventHandler;
    clientObj->context = context;

    return;
}

// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_GEOMETRY * DRV_SRAM_GeometryGet( const DRV_HANDLE handle );

  Summary:
    Returns the geometry of the device.

  Description:
    This API gives the following geometrical details of the SRAM Media:
    - Media Property
    - Number of Read/Write/Erase regions in the SRAM Media
    - Number of Blocks and their size in each region of the device

  Remarks:
    Refer to drv_sram.h for usage information.
*/

SYS_FS_MEDIA_GEOMETRY * DRV_SRAM_GeometryGet
(
    const DRV_HANDLE handle
)
{
    DRV_SRAM_CLIENT_OBJECT *clientObj = NULL;
    DRV_SRAM_OBJECT *dObj = NULL;

    /* Validate the handle */
    clientObj = DRV_SRAM_ValidateClientHandle(handle);
    _DRV_SRAM_VALIDATE_EXPR((NULL == clientObj), NULL);

    dObj = (DRV_SRAM_OBJECT *)clientObj->driverObj;

    return (dObj->sramMediaGeometry);
}

// *****************************************************************************
/* Function:
    bool DRV_SRAM_isAttached( const DRV_HANDLE handle );

  Summary:
    Returns the physical attach status of the SRAM.

  Description:
    This function returns the physical attach status of the SRAM. This
    function returns false if the driver handle is invalid otherwise returns
    true.

  Remarks:
    Refer to drv_sram.h for usage information.
*/

bool DRV_SRAM_IsAttached
(
    const DRV_HANDLE handle
)
{
    /* Validate the driver handle */
    _DRV_SRAM_VALIDATE_EXPR((NULL == DRV_SRAM_ValidateClientHandle(handle)), false);

   return true;
}

// *****************************************************************************
/* Function:
    bool DRV_SRAM_isWriteProtected( const DRV_HANDLE handle );

  Summary:
    Returns the write protect status of SRAM.

  Description:
    This function returns the write protect status of the SRAM. Always returns
    false.

  Remarks:
    Refer to drv_sram.h for usage information.
*/

bool DRV_SRAM_IsWriteProtected
(
    const DRV_HANDLE handle
)
{
    return false;
}

// *****************************************************************************
/* Function:
    uintptr_t DRV_SRAM_AddressGet
    (
        const DRV_HANDLE handle
    );

  Summary:
    Returns the SRAM media start address

  Description:
    This function returns the SRAM Media start address.

  Example:
    <code>

    uintptr_t startAddress;
    startAddress = DRV_SRAM_AddressGet(drvSRAMHandle);

    </code>

  Remarks:
    None.
*/

uintptr_t DRV_SRAM_AddressGet
(
    const DRV_HANDLE handle
)
{
    DRV_SRAM_CLIENT_OBJECT *clientObj = NULL;
    DRV_SRAM_OBJECT *dObj = NULL;

    /* Validate the handle */
    clientObj = DRV_SRAM_ValidateClientHandle(handle);
    _DRV_SRAM_VALIDATE_EXPR((NULL == clientObj), (uintptr_t)NULL);

    dObj = (DRV_SRAM_OBJECT *)clientObj->driverObj;
    return (dObj->blockStartAddress);
}
