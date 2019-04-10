/*******************************************************************************
  IPF SPI Flash Driver Dynamic implemention.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ipf.c

  Summary:
    Source code for the SPI flash driver dynamic implementation.

  Description:
    This file contains the source code for the dynamic implementation of the 
    IPF SPI Flash driver.
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
#include "driver/spi_flash/pic32wk_ipf/src/drv_ipf_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/* This is the driver instance object array. */
DRV_IPF_OBJ gDrvIPFObj[DRV_IPF_INSTANCES_NUMBER] ;

/* This is the client object array. */
DRV_IPF_CLIENT_OBJ gDrvIPFClientObj[DRV_IPF_CLIENTS_NUMBER];

/* This is the array of IPF SPI Flash Driver Buffer objects. */
DRV_IPF_BUFFER_OBJ gDrvIPFBufferObj[DRV_IPF_QUEUE_SIZE];

/* This object maintains data that is required by all IPF
   driver instances. */
DRV_IPF_COMMON_DATA_OBJ gDrvIPFCommonDataObj;

/* Array to maintain the block protect register status */
uint8_t gDrvBlockProtReg[DRV_IPF_BLOCK_PROT_BYTES] = {0,};

#if defined(DRV_IPF_REGISTER_MEDIA)
const SYS_FS_MEDIA_FUNCTIONS ipfMediaFunctions =
{
    .mediaStatusGet     = DRV_IPF_MediaIsAttached,
    .mediaGeometryGet   = DRV_IPF_GeometryGet,
    .sectorRead         = (void *) DRV_IPF_FS_BlockRead,
    .sectorWrite        = NULL,
    .eventHandlerset    = (void *) DRV_IPF_BlockEventHandlerSet,
    .commandStatusGet   = (void *) DRV_IPF_CommandStatus,
    .Read               = (void *) DRV_IPF_FS_BlockRead,
    .erase              = NULL,
    .addressGet         = DRV_IPF_AddressGet,
    .open               = DRV_IPF_Open,
    .close              = DRV_IPF_Close,
    .tasks              = DRV_IPF_Tasks,
};
#endif

// *****************************************************************************
// *****************************************************************************
// Section: IPF SPI Flash Driver Interface Implementations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    DRV_IPF_CLIENT_OBJ * _DRV_IPF_DriverHandleValidate(DRV_HANDLE handle)

  Summary:
    Dynamic implementation of the _DRV_IPF_DriverHandleValidate() function.

  Description:
    Dynamic implementation of the _DRV_IPF_DriverHandleValidate() function.
    This function returns NULL if the handle is invalid else it return a pointer
    to the IPF Driver Client Object associated with this handle.

  Remarks:
    This is a private function and should not be called directly by an
    application.
*/

DRV_IPF_CLIENT_OBJ * _DRV_IPF_DriverHandleValidate(DRV_HANDLE handle)
{
    /* This function returns the pointer to the client object that is
       associated with this handle if the handle is valid. Returns NULL
       otherwise. */

    DRV_IPF_CLIENT_OBJ * client;

    if((DRV_HANDLE_INVALID == handle) || (0 == handle))
    {
        return(NULL);
    }

    client = (DRV_IPF_CLIENT_OBJ *)handle;

    if(!client->inUse)
    {
        return(NULL);
    }

    return(client);
}

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_IPF_Initialize
    (
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init
    );

  Summary:
    Dynamic impementation of DRV_IPF_Initialize system interface function.

  Description:
    This is the dynamic impementation of DRV_IPF_Initialize system
    interface function.
  
  Remarks:
    See drv_ipf.h for usage information.
*/

SYS_MODULE_OBJ DRV_IPF_Initialize
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT * const init
)
{
    DRV_IPF_OBJ *dObj = (DRV_IPF_OBJ*)NULL;
    DRV_IPF_INIT *ipfInit = NULL ;

    /* Check if the specified driver index is in valid range */
    if(drvIndex >= DRV_IPF_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "DRV_IPF: Invalid driver index \n");
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Check if this hardware instance was already initialized */
    if(gDrvIPFObj[drvIndex].inUse != false)
    {
        SYS_DEBUG(0, "DRV_IPF: Instance already in use \n");
        return SYS_MODULE_OBJ_INVALID;
    }
    
    /* Assign to the local pointer the init data passed */
    ipfInit = ( DRV_IPF_INIT * ) init ;

    /* Allocate the driver object and set the operation flag to be in use */
    dObj = &gDrvIPFObj[drvIndex];
    dObj->inUse = true;
    
    /* Set the driver status as busy */
    dObj->status =  SYS_STATUS_BUSY ;

    /* Save the SPI driver module index in SST hardware instance object */
    dObj->spiDriverModuleIndex = ipfInit->spiDriverModuleIndex;
    
    dObj->nClients = 0;
    dObj->isExclusive = false;

    /* Set the port remapping pins as per init structure */
    dObj->chipSelectPortChannel = ipfInit->chipSelectPortChannel;
    dObj->chipSelectBitPosition = ipfInit->chipSelectBitPosition;

    /* Make the HOLD pin direction as output */
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_OUTPUT, ipfInit->holdPortChannel, ipfInit->holdBitPosition);
    /* Keep the HOLD pin always HIGH, it should never have high to low transition */
    SYS_PORTS_PinSet(PORTS_ID_0, ipfInit->holdPortChannel, ipfInit->holdBitPosition);

    /* Make the WP pin direction as output */
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_OUTPUT, ipfInit->writeProtectPortChannel, ipfInit->writeProtectBitPosition);
    /* Keep the WP pin always HIGH so that status register will be always writable */
    SYS_PORTS_PinSet(PORTS_ID_0, ipfInit->writeProtectPortChannel, ipfInit->writeProtectBitPosition);

    /* Make the chip select pin direction as output */
    SYS_PORTS_PinDirectionSelect(PORTS_ID_0,SYS_PORTS_DIRECTION_OUTPUT, dObj->chipSelectPortChannel, dObj->chipSelectBitPosition);
    /* Make the chip select pin HIGH initially */
    SYS_PORTS_PinSet(PORTS_ID_0, dObj->chipSelectPortChannel, dObj->chipSelectBitPosition);
	
    
    /* Initialize Queue parameters */
    dObj->queueTail = NULL;
    dObj->queueSize = ipfInit->queueSize;
    dObj->queueOccupancy = 0;

    /* Set driver task state to opening spi driver */
    dObj->state = DRV_IPF_TASK_OPEN_SPI_DRIVER;
    
    /* Set internal operations state */
    dObj->bufferProcessState = DRV_IPF_BUFFER_PROCESS_INIT;

//    /* Create the hardware instance mutex. */
//     OSAL_ASSERT((OSAL_MUTEX_Create(&(dObj->mutexDriverInstance)) == OSAL_RESULT_TRUE),
//                 "Unable to create hardware instance mutex");
//
//
//    /* Check if the global mutexes have been created. If not
//       then create these. */
//
//     if(!gDrvIPFCommonDataObj.membersAreInitialized)
//     {
//         /* This means that mutexes where not created. Create them. */
//         OSAL_ASSERT((OSAL_MUTEX_Create(&(gDrvIPFCommonDataObj.mutexClientObjects)) == OSAL_RESULT_TRUE),
//                     "Unable to create client instance mutex");
//         OSAL_ASSERT((OSAL_MUTEX_Create(&(gDrvIPFCommonDataObj.mutexBufferQueueObjects)) == OSAL_RESULT_TRUE),
//                     "Unable to create buffer queue objects mutex");
//
//         /* Set this flag so that global mutexes get allocated only once */
//         gDrvIPFCommonDataObj.membersAreInitialized = true;
//     }

#if defined(DRV_IPF_REGISTER_MEDIA)
     SYS_FS_MEDIA_MANAGER_Register (drvIndex, drvIndex, &ipfMediaFunctions, SYS_FS_MEDIA_TYPE_SPIFLASH);
#endif
     
    /* Driver status will be made ready once SPI Driver is openend
       successfully and Write Protection is disabled in Task API */

    return drvIndex;
}


//******************************************************************************
/* Function:
    void DRV_IPF_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the SPI Flash driver module

  Description:
    Deinitializes the specified instance of the SPI Flash driver module,
    disabling its operation (and any hardware). Invalidates all the
    internal data.

  Remarks:
    See drv_ipf.h for usage information.
*/

void DRV_IPF_Deinitialize( SYS_MODULE_OBJ object)
{
    DRV_IPF_OBJ * dObj;
    DRV_IPF_BUFFER_OBJ * iterator;


    /* Check that the object is valid */
    if(object == SYS_MODULE_OBJ_INVALID)
    {
        SYS_DEBUG(0, "DRV_IPF: Invalid system module object \n" );
        return;
    }
    
    if(object >= DRV_IPF_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "DRV_IPF: Invalid system module object \n" );
        return;
    }

    dObj = (DRV_IPF_OBJ*) &gDrvIPFObj[object];

    if(!dObj->inUse)
    {
        SYS_DEBUG(0, "DRV_IPF: Invalid system module object \n");
        return;
    }

    /* The driver will not have clients when it is
       being de-initialized. So the order in which
       we do the following steps is not that important */

    /* Indicate that this object is not is use */
    dObj->inUse = false;

    /* Deinitialize the IPF status */
    dObj->status =  SYS_STATUS_UNINITIALIZED ;


    // /* Deallocate all mutexes */
    // OSAL_ASSERT( (OSAL_MUTEX_Delete(&(dObj->mutexDriverInstance)) == OSAL_RESULT_TRUE),
            // "Unable to delete client handle mutex" );

            
    // Close the spi driver
    DRV_SPI_Close (dObj->spiDriverOpenHandle);

    /* Remove all objects from the queue */
    iterator = dObj->queueTail;
    while(iterator != NULL)
    {
        /* Return the buffer object to the pool */
        iterator->inUse = false;
        iterator = iterator->next;
    }
    
}

//*************************************************************************
/* Function:
    SYS_STATUS DRV_IPF_Status( SYS_MODULE_OBJ object )

  Summary:
    Gets the current status of the SPI Flash driver module.

  Description:
    This routine provides the current status of the SPI Flash driver module.
  
  Remarks:
    See drv_ipf.h for usage information.
*/

SYS_STATUS DRV_IPF_Status( SYS_MODULE_OBJ object)
{
    /* Check if we have a valid object */
    if(object == SYS_MODULE_OBJ_INVALID)
    {
        SYS_DEBUG(0, "DRV_IPF: Invalid system object handle \n");
        return(SYS_STATUS_UNINITIALIZED);
    }
    
    if(object > DRV_IPF_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "DRV_IPF: Invalid system object handle \n");
        return(SYS_STATUS_UNINITIALIZED);
    }

    /* Return the system status of the hardware instance object */
    return (gDrvIPFObj[object].status);
}

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_IPF_Open
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
    See drv_ipf.h for usage information.
*/

DRV_HANDLE DRV_IPF_Open
( 
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
    DRV_IPF_CLIENT_OBJ *clientObj;
    DRV_IPF_OBJ *dObj;
    unsigned int iClient;

    if (drvIndex >= DRV_IPF_INSTANCES_NUMBER)
    {
        /* Invalid driver index */
        SYS_DEBUG(0, "DRV_IPF: Invalid Driver Instance \n");
        return (DRV_HANDLE_INVALID);
    }

    dObj = &gDrvIPFObj[drvIndex];
    
    if((dObj->status != SYS_STATUS_READY) || (dObj->inUse == false)) 
    {
        /* The IPF module should be ready */

        SYS_DEBUG(0, "DRV_IPF: Was the driver initialized? \n");
        return DRV_HANDLE_INVALID;
    }

    if(dObj->isExclusive)
    {
        /* This means the another client has opened the driver in exclusive
           mode. The driver cannot be opened again */

        SYS_DEBUG(0, "DRV_IPF: Driver already opened exclusively \n"); 
        return ( DRV_HANDLE_INVALID ) ;
    }

    if((dObj->nClients > 0) && (ioIntent & DRV_IO_INTENT_EXCLUSIVE))
    {
        /* This means the driver was already opened and another driver was 
           trying to open it exclusively.  We cannot give exclusive access in 
           this case */

        SYS_DEBUG(0, "DRV_IPF: Driver already opened. Cannot be opened exclusively \n");
        return(DRV_HANDLE_INVALID);
    }

    /* Grab client object mutex here */

//    if(OSAL_MUTEX_Lock(&(gDrvIPFCommonDataObj.mutexClientObjects), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        /* Enter here only if the lock was obtained (appplicable in 
           RTOS only). If the mutex lock fails due to time out then
           this code does not get executed */

        for(iClient = 0; iClient != DRV_IPF_CLIENTS_NUMBER; iClient ++)
        {
            if(!gDrvIPFClientObj[iClient].inUse)
            {
                /* This means we have a free client object to use */
                clientObj = &gDrvIPFClientObj[iClient];
                clientObj->inUse        = true;
                
                /* We have found a client object. Release the mutex */

//                OSAL_ASSERT(OSAL_MUTEX_Unlock(&(gDrvIPFCommonDataObj.mutexClientObjects)),
//                        "Unable to unlock clients objects routine mutex");
                
                clientObj->hDriver      = dObj;                
                clientObj->ioIntent     = ioIntent;
                clientObj->eventHandler = (DRV_IPF_EVENT_HANDLER) NULL;
                clientObj->context      = (uintptr_t) NULL;

                if(ioIntent & DRV_IO_INTENT_EXCLUSIVE)
                {
                    /* Set the driver exclusive flag */
                    dObj->isExclusive = true;
                }

                dObj->nClients ++;

                // /* Create the semaphores */
                // OSAL_ASSERT(((OSAL_SEM_Create(&(clientObj->semReadDone), OSAL_SEM_TYPE_COUNTING, 1, 0)) == OSAL_RESULT_TRUE),
                        // "Unable to create client read done semaphore");
                // OSAL_ASSERT(((OSAL_SEM_Create(&(clientObj->semWriteDone), OSAL_SEM_TYPE_COUNTING, 1, 0)) == OSAL_RESULT_TRUE),
                        // "Unable to create client write done semaphore");

                /* Update the client status */
                clientObj->clientStatus = DRV_IPF_CLIENT_STATUS_READY;
                return ((DRV_HANDLE) clientObj );
            }
        }

        /* Could not find a client object. Release the mutex and 
           return with an invalid handle. */
//        OSAL_ASSERT((OSAL_MUTEX_Unlock(&(gDrvIPFCommonDataObj.mutexClientObjects))),
//                    "Unable to unlock clients objects routine mutex");
    }

    /* If we have reached here, it means either we could not find a spare
       client object or the mutex timed out in a RTOS environment. */
    
    return DRV_HANDLE_INVALID;
}

// *****************************************************************************
/* Function:
    void DRV_IPF_Close( DRV_Handle handle );

  Summary:
    Closes an opened-instance of the SPI Flash driver

  Description:
    This routine closes an opened-instance of the SPI Flash driver, invalidating
    the handle.

  Remarks:
    See drv_ipf.h for usage information.
*/

void DRV_IPF_Close( const DRV_HANDLE handle)
{
    /* This function closes the client, The client
       object is de-allocated and returned to the
       pool. */

    DRV_IPF_CLIENT_OBJ  * clientObj;
    DRV_IPF_OBJ * dObj;
    DRV_IPF_BUFFER_OBJ * iterator = NULL;

    /* Validate the handle */
    clientObj = _DRV_IPF_DriverHandleValidate(handle);

    if(clientObj == NULL)
    {
        /* Driver handle is not valid */
        SYS_DEBUG(0, "DRV_IPF: Invalid Driver Handle \n");
        return;
    }

    dObj = (DRV_IPF_OBJ *)clientObj->hDriver;

    
    /* Remove all buffers that this client owns from the driver queue. */
//    if(OSAL_MUTEX_Lock(&(dObj->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        iterator = dObj->queueTail;
        while(iterator != NULL)
        {
            if(clientObj == (DRV_IPF_CLIENT_OBJ *)iterator->hClient)
            {
                /* That means this buffer object is owned
                   by this client. This buffer object should
                   be removed. The following code removes
                   the object from a doubly linked list queue. */

                iterator->inUse = false;
                if(iterator->previous != NULL)
                {
                    iterator->previous->next = iterator->next;
                }
                if(iterator->next != NULL)
                {
                    iterator->next->previous = iterator->previous;
                }
                /* Decrementing Current queue occupancy */
                dObj->queueOccupancy --;
            }
            
            iterator = iterator->next;
        }

        /* updating the queue tail once the buffer objects associated
           with the client is removed from the queue */

        
        if(dObj->queueOccupancy == 0)
        {
            /* If there are no buffers in the queue.
            * Make the tail pointer point to NULL */
            dObj->queueTail = NULL;
        }
        else
        {
            iterator = dObj->queueTail;
            while(iterator != NULL)
            {
                if (iterator->inUse == true)
                {
                    dObj->queueTail = iterator;
                    break;
                }
                iterator = iterator->next;
            }
        }
        
        /* Unlock the mutex */
//        OSAL_ASSERT((OSAL_MUTEX_Unlock(&(dObj->mutexDriverInstance))),
//                "Unable to unlock Driver instance mutex");
    }
//    else
//    {
//        /* The function could fail if the mutex time out occurred */
//        SYS_DEBUG(0, "DRV_IPF: Could not remove client buffer objects \n");
//        clientObj->clientStatus = DRV_IPF_CLIENT_STATUS_ERROR;
//        return;
//    }
    
    // /* Deallocate all semaphores */
    // OSAL_ASSERT((OSAL_SEM_Delete(&(clientObj->semWriteDone)) == OSAL_RESULT_TRUE),
            // "Unable to delete client write done semaphore");
    // OSAL_ASSERT((OSAL_SEM_Delete(&(clientObj->semReadDone)) == OSAL_RESULT_TRUE),
            // "Unable to delete client read done semaphore");

    if(dObj->isExclusive)
    {
        /* clear the driver exclusive flag if it was set */
        dObj->isExclusive = false;
    }

    /* Reduce the number of clients */
    dObj->nClients --;

    /* De-allocate the object */
    clientObj->clientStatus = DRV_IPF_CLIENT_STATUS_CLOSED;
    clientObj->inUse = false;

    return;
}
 
// *****************************************************************************
/* Function:
    DRV_IPF_CLIENT_STATUS DRV_IPF_ClientStatus (DRV_HANDLE handle )

  Summary:
    Dynamic impementation of DRV_IPF_ClientStatus client interface function.

  Description:
    This is the dynamic impementation of DRV_IPF_ClientStatus client interface 
    function.
  
  Remarks:
    See drv_IPF.h for usage information.
*/

DRV_IPF_CLIENT_STATUS DRV_IPF_ClientStatus(DRV_HANDLE handle)
{
    DRV_IPF_CLIENT_OBJ * client;

    /* Validate the driver handle */
    client = _DRV_IPF_DriverHandleValidate(handle);

    if(client == NULL)
    {
        /* Driver handle is not valid */
        
        SYS_DEBUG(0, "DRV_IPF: Invalid driver handle \n");
        return DRV_IPF_CLIENT_STATUS_CLOSED;
    }

    /* Return the client status */
    return(client->clientStatus);
}

// *****************************************************************************
/* Function:
    void DRV_IPF_BlockEventHandlerSet
    (
        const DRV_HANDLE handle,
        const DRV_IPF_EVENT_HANDLER eventHandler,
        const uintptr_t context
    )

  Summary:
    Allows a client to identify an event handling function for the driver to
    call back when queued operation has completed.

  Description:
    This function allows a client to identify an event handling function
    for the driver to call back when queued operation has completed.
    When a client calls any read, write or erase function, it is provided with a
    handle identifying the buffer that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling
    "eventHandler" function when the queued operation has completed.

    The event handler should be set before the client performs any
    read/write/erase operations that could generate events. The event handler
    once set, persists until the client closes the driver or sets another event
    handler (which could be a "NULL" pointer to indicate no callback).

  Remarks:
    See drv_ipf.h for usage information.
*/

void DRV_IPF_BlockEventHandlerSet
(
    const DRV_HANDLE handle,
    const DRV_IPF_EVENT_HANDLER eventHandler,
    const uintptr_t context
)
{
    DRV_IPF_CLIENT_OBJ *client;

    /* Validate the driver handle */
    client = _DRV_IPF_DriverHandleValidate(handle);

    if(client == NULL)
    {
        /* Driver handle is not valid */

        SYS_DEBUG(0, "DRV_IPF: Invalid driver handle \n");
        return;
    }

    /* save the context and the eventHandler in the Client Object */
    client->context = context;
    client->eventHandler = eventHandler;

}

// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_GEOMETRY DRV_IPF_GeometryGet( DRV_HANDLE handle );

  Summary:
    Returns the geometry of the device.

  Description:
    This API gives the following geometrical details of the IPF Flash:
    - Media Property
    - Number of Read/Write/Erase regions in the flash device
    - Number of Blocks and their size in each region of the device

  Remarks:
    This API will be usually used by File System Media Manager.
*/

SYS_FS_MEDIA_GEOMETRY * DRV_IPF_GeometryGet( DRV_HANDLE handle )
{
    
    DRV_IPF_CLIENT_OBJ * hClient;
    DRV_IPF_OBJ *hDriver;

    /* Validate the driver handle */
    hClient = _DRV_IPF_DriverHandleValidate(handle);

    if(hClient == NULL)
    {
        /* Driver handle is not valid */

        SYS_DEBUG(0, "DRV_IPF: Invalid driver handle \n");
        return NULL;
    }

    hDriver = hClient->hDriver;

    /* 2 MByte is SST Flash size */
    /* 1 Byte is the SST Flash Read Block size */
    hDriver->memoryRegions[0].blockSize = 1;          //  1 Byte
    hDriver->memoryRegions[0].numBlocks = IPF_RW_NUMBLOCKS;   //  defined as per controller

    /* 2 MByte is SST Flash size */
    /* 1 Byte is the SST Flash Write Block size */
    hDriver->memoryRegions[1].blockSize = 1;          //  1 Byte
    hDriver->memoryRegions[1].numBlocks = IPF_RW_NUMBLOCKS;	//  defined as per controller

    /* 2 MByte is SST Flash size */
    /* 4 KByte is the SST Flash Erase Block size */
    hDriver->memoryRegions[2].blockSize = 4096;	//  4 KByte
    hDriver->memoryRegions[2].numBlocks = IPF_ERASE_NUMBLOCKS;		//  defined as per controller

    hDriver->ipfDeviceGeometry.mediaProperty = SYS_FS_MEDIA_SUPPORTS_BYTE_WRITES;
    hDriver->ipfDeviceGeometry.numReadRegions = 1; 	// 1 Read region
    hDriver->ipfDeviceGeometry.numWriteRegions = 1;	// 1 Write region
    hDriver->ipfDeviceGeometry.numEraseRegions = 1; 	// 1 Erase region
    hDriver->ipfDeviceGeometry.geometryTable = &hDriver->memoryRegions[0];

    return (&hDriver->ipfDeviceGeometry);
}

// *****************************************************************************
/* Function:
    bool DRV_IPF_MediaIsAttached(DRV_HANDLE handle);

  Summary:
    Returns the status of the media

  Description:
    This API tells if the media is attached or not.

  Remarks:
    This API will be usually used by File System Media Manager.
*/

bool DRV_IPF_MediaIsAttached(DRV_HANDLE handle)
{

    DRV_IPF_CLIENT_OBJ * client;

    /* Validate the driver handle */
    client = _DRV_IPF_DriverHandleValidate(handle);

    if(client == NULL)
    {
        /* Driver handle is not valid */

        SYS_DEBUG(0, "DRV_IPF: Invalid driver handle \n");
        return false;
    }
    
    return true;

}

// *****************************************************************************
/* Function:
    void _DRV_IPF_ProcessBufferTask
    (
        DRV_IPF_OBJ * hDriver,
        DRV_IPF_BUFFER_OBJ * bufferObj
    )

  Summary:
    Internal Buffer processing task function

  Description:
    Dynamic implementation of the _DRV_IPF_ProcessBufferTask() function.
    This function process the IPF buffer task using SPI.

  Remarks:
    This is a private function and should not be called directly by an
    application.
*/

void _DRV_IPF_ProcessBufferTask
(
    DRV_IPF_OBJ * hDriver,
    DRV_IPF_BUFFER_OBJ * bufferObj
)
{
    static uint32_t writeSize = 0;
    switch(hDriver->bufferProcessState)
    {
        case DRV_IPF_BUFFER_PROCESS_INIT:
        {
            /* Enable CS Line */
            SYS_PORTS_PinClear(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);

            if(bufferObj->operation == DRV_IPF_HW_BLOCK_PROT)
            {
                DRV_IPF_BPRBitSet(bufferObj->protBitPos, bufferObj->protMode);
            }
            else if(bufferObj->operation == DRV_IPF_HW_BLOCK_UNPROT)
            {
                DRV_IPF_BPRBitClear(bufferObj->protBitPos, bufferObj->protMode);
            }

            if (bufferObj->operation == DRV_IPF_BLOCK_READ)
            {
                /* If request is for read, then load the read command and address */
                /* Add READ op code */
                hDriver->commandAddressData[0] = IPF_NORMAL_READ_OP_CODE;
                /* Add READ address */
                hDriver->commandAddressData[1] = (uint8_t)(bufferObj->address >> 16);
                hDriver->commandAddressData[2] = (uint8_t)(bufferObj->address >> 8);
                hDriver->commandAddressData[3] = (uint8_t)(bufferObj->address);

                hDriver->bufferProcessState = DRV_IPF_SEND_READ_CMD_AND_ADDRESS;
            }
            else if (bufferObj->operation == DRV_IPF_READ_HW_BLOCK_PROT)
            {
                /* If request is for read BPR status, then load the RBPR command */
                /* Add READ op code */
                hDriver->commandAddressData[0] = IPF_READ_BLOCK_PROT_STATUS;

                hDriver->bufferProcessState = DRV_IPF_SEND_READ_HW_BLOCK_PROT_STATUS;   
                break;
            }
            else
            {
                /* If request is for Erase/Write, then load Write Enable command first */
                hDriver->commandAddressData[0] = IPF_WREN_OP_CODE;

                hDriver->bufferProcessState = DRV_IPF_SEND_WREN_CMD;
                break;
            }       
        }
         case DRV_IPF_SEND_READ_CMD_AND_ADDRESS:
         {
            /* Add the buffer in SPI Queue */
            if ((DRV_SPI_BufferAddWrite(hDriver->spiDriverOpenHandle,
                     (uint8_t *)&hDriver->commandAddressData[0], 4, NULL, NULL)) != (DRV_SPI_BUFFER_HANDLE)NULL)
            {
                /* This means command has been allocated in the SPI queue,
                 * now go to the nexr state */
                hDriver->bufferProcessState = DRV_IPF_START_READING;
            }
            else
            {
                /* This means command has not been allocated in the SPI queue.
                 * Stay in the same state untill it gets queued */
                break;
            }
         }
        case DRV_IPF_START_READING:
        {
            hDriver->spiBufferHandle = DRV_SPI_BufferAddRead(hDriver->spiDriverOpenHandle,
                    (uint8_t *)bufferObj->buffer, bufferObj->size, NULL, NULL );
            if(hDriver->spiBufferHandle != (DRV_SPI_BUFFER_HANDLE) NULL)
            {
                hDriver->bufferProcessState = DRV_IPF_READ_COMPLETED;
            }
            else
            {
                break;
            }       
        }
        case DRV_IPF_READ_COMPLETED:
        {
            /* Check if the transfer status is success or not */
            if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(hDriver->spiBufferHandle))
            {
                /* This means Read is done, now disable the CS Line */
                SYS_PORTS_PinSet(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);

                /* Update the block size */
                bufferObj->nCurrentBlocks = bufferObj->size;

                /* Re-initialize the internal buffer processing state for the new buffer object */
                hDriver->bufferProcessState = DRV_IPF_BUFFER_PROCESS_INIT;
            }
            break;
        }
        case DRV_IPF_SEND_READ_HW_BLOCK_PROT_STATUS:
        {
            /* Add the buffer in SPI Queue */
            if ((DRV_SPI_BufferAddWrite(hDriver->spiDriverOpenHandle,
                     (uint8_t *)&hDriver->commandAddressData[0], 1, NULL, NULL)) != (DRV_SPI_BUFFER_HANDLE)NULL)
            {
                /* This means command has been allocated in the SPI queue,
                 * now go to the nexr state */
                hDriver->bufferProcessState = DRV_IPF_HW_PROT_STATUS_READING;
            }
            else
            {
                /* This means command has not been allocated in the SPI queue.
                 * Stay in the same state untill it gets queued */
                break;
            }            
        }
        case DRV_IPF_HW_PROT_STATUS_READING:
        {
            hDriver->spiBufferHandle = DRV_SPI_BufferAddRead(hDriver->spiDriverOpenHandle,
                    (uint8_t *)bufferObj->buffer, bufferObj->size, NULL, NULL );
            if(hDriver->spiBufferHandle != (DRV_SPI_BUFFER_HANDLE) NULL)
            {
                hDriver->bufferProcessState = DRV_IPF_HW_PROT_READ_COMPLETED;
            }
            else
            {
                break;
            }       
        }
        case DRV_IPF_HW_PROT_READ_COMPLETED:
        {
            /* Check if the transfer status is success or not */
            if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(hDriver->spiBufferHandle))
            {
                /* This means Read is done, now disable the CS Line */
                SYS_PORTS_PinSet(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);

                /* Update the block size */
                bufferObj->nCurrentBlocks = bufferObj->size;

                /* Re-initialize the internal buffer processing state for the new buffer object */
                hDriver->bufferProcessState = DRV_IPF_BUFFER_PROCESS_INIT;
            }
            break;            
        }
        case DRV_IPF_SEND_WREN_CMD:
        {
            /* Add the buffer in SPI Queue */
            hDriver->spiBufferHandle = (DRV_SPI_BufferAddWrite(hDriver->spiDriverOpenHandle,
                     (uint8_t *)&hDriver->commandAddressData[0], 1, NULL, NULL));
            if (hDriver->spiBufferHandle != (DRV_SPI_BUFFER_HANDLE) NULL)
            {
                /* If the command has been queued,
                 * then check if it has been processed in the next state */
                hDriver->bufferProcessState = DRV_IPF_WREN_EXECUTION_STATUS_CHECK;
            }
            else
            {
                /* This means command has not been allocated in the SPI queue,
                 * so stay in the same state */
                break;
            }
        }
        case DRV_IPF_WREN_EXECUTION_STATUS_CHECK:
        {
            /* Check if the transfer status is success or not */
            if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(hDriver->spiBufferHandle))
            {
                /* This means WREN command has been send, now disable the CS Line */
                SYS_PORTS_PinSet(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);

                if (bufferObj->operation == DRV_IPF_BLOCK_WRITE)
                {
                    /* It is a WRITE request */
                    hDriver->bufferProcessState = DRV_IPF_SEND_WRITE_CMD_ADDRESS_AND_DATA;
                }
                else if(bufferObj->operation == DRV_IPF_BLOCK_ERASE)
                {
                    /* This means it is an Erase request */
                    hDriver->bufferProcessState = DRV_IPF_SEND_ERASE_CMD_AND_ADDRESS;
                    break;
                }
                else if((DRV_IPF_HW_BLOCK_PROT == bufferObj->operation) || (DRV_IPF_HW_BLOCK_UNPROT == bufferObj->operation))
                {
                    /* This means it is an Memory protection request */
                    hDriver->bufferProcessState = DRV_IPF_SEND_MEM_PROT_WORD;
                    break;    
                }
                else
                {
                    
                }
            }
            else
            {
                break;
            }
        }
        case DRV_IPF_SEND_WRITE_CMD_ADDRESS_AND_DATA:
        {
            /* Enable CS Line */
            SYS_PORTS_PinClear(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);

            /* Add Write Byte op code */
            hDriver->commandAddressData[0] = IPF_WRITE_BYTE_OP_CODE;
            /* Add Write address and data pointer */
            hDriver->commandAddressData[1] = (uint8_t)(bufferObj->address >> 16);
            hDriver->commandAddressData[2] = (uint8_t)(bufferObj->address >> 8);
            hDriver->commandAddressData[3] = (uint8_t)(bufferObj->address);
			
			if((bufferObj->size - bufferObj->nCurrentBlocks) > 256)
				writeSize = 256;
			else
				writeSize = bufferObj->size - bufferObj->nCurrentBlocks;
			
			memcpy(&hDriver->commandAddressData[4],(uint8_t *)((uint32_t)bufferObj->buffer + bufferObj->nCurrentBlocks),writeSize);

            /* Add the buffer in SPI Queue */
            hDriver->spiBufferHandle = (DRV_SPI_BufferAddWrite(hDriver->spiDriverOpenHandle,
                     (uint8_t *)&hDriver->commandAddressData[0], writeSize + 4, NULL, NULL));
            if (hDriver->spiBufferHandle != (DRV_SPI_BUFFER_HANDLE) NULL)
            {
                /* this means command has been allocated in the SPI queue,
                 *  now wait for it to be executed in the next state */
                hDriver->bufferProcessState = DRV_IPF_WAIT_FOR_WRITE_OR_ERASE_BUFFER_COMPLETE;
            }
            break;
        }
        case DRV_IPF_SEND_MEM_PROT_WORD:
        {
            /* Enable CS Line */
            SYS_PORTS_PinClear(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);

            /* Add Write Block protection register op code */
            hDriver->commandAddressData[0] = IPF_WRITE_BLOCK_PROT_REG;
            /* Add Block protection Words */
            hDriver->commandAddressData[1] = gDrvBlockProtReg[5];
            hDriver->commandAddressData[2] = gDrvBlockProtReg[4];
            hDriver->commandAddressData[3] = gDrvBlockProtReg[3];
            hDriver->commandAddressData[4] = gDrvBlockProtReg[2];
            hDriver->commandAddressData[5] = gDrvBlockProtReg[1];
            hDriver->commandAddressData[6] = gDrvBlockProtReg[0];

            /* Add the buffer in SPI Queue */
            hDriver->spiBufferHandle = (DRV_SPI_BufferAddWrite(hDriver->spiDriverOpenHandle,
                     (uint8_t *)&hDriver->commandAddressData[0], 7, NULL, NULL));
            if (hDriver->spiBufferHandle != (DRV_SPI_BUFFER_HANDLE) NULL)
            {
                /* this means command has been allocated in the SPI queue,
                 *  now wait for it to be executed in the next state */
                hDriver->bufferProcessState = DRV_IPF_WAIT_FOR_WRITE_OR_ERASE_BUFFER_COMPLETE;
            }
            else
            {
                /* This means command has not been allocated in the SPI queue,
                 * wait for it to be queued */
                break;
            }
        }
        case DRV_IPF_SEND_ERASE_CMD_AND_ADDRESS:
        {
            /* Enable CS Line */
            SYS_PORTS_PinClear(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);

            /* Add Erase op code */
            hDriver->commandAddressData[0] = IPF_ERASE_OP_CODE;
            /* Add erase address */
            hDriver->commandAddressData[1] = (uint8_t)(bufferObj->address >> 16);
            hDriver->commandAddressData[2] = (uint8_t)(bufferObj->address >> 8);
            hDriver->commandAddressData[3] = (uint8_t)(bufferObj->address);

            /* Add the buffer in SPI Queue */
            hDriver->spiBufferHandle = (DRV_SPI_BufferAddWrite(hDriver->spiDriverOpenHandle,
                     (uint8_t *)&hDriver->commandAddressData[0], 4, NULL, NULL));
            if (hDriver->spiBufferHandle != (DRV_SPI_BUFFER_HANDLE) NULL)
            {
                /* this means command has been allocated in the SPI queue,
                 *  now wait for it to be executed in the next state */
                hDriver->bufferProcessState = DRV_IPF_WAIT_FOR_WRITE_OR_ERASE_BUFFER_COMPLETE;
            }
            else
            {
                /* This means command has not been allocated in the SPI queue,
                 * wait for it to be queued */
                break;
            }
        }        
        case DRV_IPF_WAIT_FOR_WRITE_OR_ERASE_BUFFER_COMPLETE:
        {
            /* Check if the transfer status is success or not */
            if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(hDriver->spiBufferHandle))
            {
                /* This means Erase Command and address has been send, now disable the CS Line */
                SYS_PORTS_PinSet(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);
                
                hDriver->bufferProcessState = DRV_IPF_SEND_COMMAND_FOR_BUSY_STATUS;
            }
            else
            {
                break;
            }
        }
        case DRV_IPF_SEND_COMMAND_FOR_BUSY_STATUS:
        {
            /* Enable CS Line */
            SYS_PORTS_PinClear(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);

            /* Add RDSR op code to read the status register */
            hDriver->commandAddressData[0] = IPF_RDSR_OP_CODE;

            hDriver->spiBufferHandle = (DRV_SPI_BufferAddWrite(hDriver->spiDriverOpenHandle,
                     (uint8_t *)&hDriver->commandAddressData[0], 1, NULL, NULL));
                    
            if (hDriver->spiBufferHandle != (DRV_SPI_BUFFER_HANDLE) NULL)
            {
                /* This means command has been allocated in the SPI queue,
                 *  now wait for it to be executed in the next state */
                hDriver->bufferProcessState = DRV_IPF_READ_BUSY_STATUS;
            }
            else
            {
                /* This means command has not been allocated in the SPI queue,
                 * wait for it to be queued */
                hDriver->bufferProcessState = DRV_IPF_SEND_COMMAND_FOR_BUSY_STATUS;
            }
            break;
        }
        case DRV_IPF_READ_BUSY_STATUS:
        {
            /* Read the Status Register value */
            hDriver->spiBufferHandle = (DRV_SPI_BufferAddRead(hDriver->spiDriverOpenHandle,
                     (uint8_t *)&hDriver->commandAddressData[3], 1, NULL, NULL));

            if (hDriver->spiBufferHandle != (DRV_SPI_BUFFER_HANDLE) NULL)
            {
                /* This means command has been allocated in the SPI queue,
                 *  now wait for it to be executed in the next state */
                hDriver->bufferProcessState = DRV_IPF_WAIT_FOR_BUSY_CLEAR;
            }
            else
            {
                /* This means command has not been allocated in the SPI queue,
                 * wait for it to be queued */
                hDriver->bufferProcessState = DRV_IPF_READ_BUSY_STATUS;
            }
            break;
        }
        case DRV_IPF_WAIT_FOR_BUSY_CLEAR:
        {
            if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(hDriver->spiBufferHandle))
            {
              if (!(hDriver->commandAddressData[3] & 0x01))
              {
                    /* This means BUSY bit is clear and operation is completed */

                    /* Disable CS */
                    SYS_PORTS_PinSet(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);

                    if (DRV_IPF_BLOCK_WRITE == bufferObj->operation)
                    {
						/* Increase the block count as write/erase/protect is completed */
						bufferObj->nCurrentBlocks = bufferObj->nCurrentBlocks + writeSize;
                        /* It was a WRITE request */
                        /* Increment the write address to point to next block,
                        *  1 write block = 1 byte */
                        bufferObj->address = bufferObj->address + writeSize;
                    }
                    else if (DRV_IPF_BLOCK_ERASE == bufferObj->operation)
                    {
						/* Increase the block count as write/erase/protect is completed */
						bufferObj->nCurrentBlocks++;
                        /* This means it was an Erase request */
                        /* Increment the erase address to point to next block,
                        * 1 erase block = 4kbyte = 0x1000 */
                        bufferObj->address = bufferObj->address + 0x1000;
                    }
                    else
                    {
                        bufferObj->nCurrentBlocks++;
                    }
                    
                    /* re-initialize the internal buffer processing state for the new buffer object */
                    hDriver->bufferProcessState = DRV_IPF_BUFFER_PROCESS_INIT;
              }
              else
              {
                  SYS_PORTS_PinSet(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);
                  /* This means BUSY bit is not yet clear, read it again */
                  hDriver->bufferProcessState = DRV_IPF_SEND_COMMAND_FOR_BUSY_STATUS; //DRV_IPF_READ_BUSY_STATUS;
              }
           }
        }
        default:
            break;
    }
    return;
}

// ***************************************************************************
/* Function:
    void DRV_IPF_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's read, erase and write state machine and implements
    its ISR.

  Description:
    This routine is used to maintain the driver's internal state machine
    and should be called from the system tasks routine.

  Remarks:
    See drv_ipf.h for usage information.
*/

void DRV_IPF_Tasks ( SYS_MODULE_OBJ object )
{
    DRV_IPF_BUFFER_OBJ * bufferObj;
    DRV_IPF_CLIENT_OBJ * client;
    
    DRV_IPF_OBJ * hDriver;
	
    /* Check if the specified module object is in valid range */
    if(object >= DRV_IPF_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "DRV_IPF: Invalid module object \n");
        return;
    }

    hDriver = (DRV_IPF_OBJ*) &gDrvIPFObj[object];
    
    if(!hDriver->inUse)
    {
        /* This intance of the driver is not initialized. Don't
         * do anything */
        return;
    }
    
    switch(hDriver->state)
    {
        case DRV_IPF_TASK_OPEN_SPI_DRIVER:
        {
            /* Open SPI Driver to read/write in NON Blocking mode with Exclusive privilage */
            hDriver->spiDriverOpenHandle =
            DRV_SPI_Open (hDriver->spiDriverModuleIndex, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING | DRV_IO_INTENT_EXCLUSIVE);

            if(hDriver->spiDriverOpenHandle != DRV_HANDLE_INVALID)
            {
                /* If SPI was open successfully, then now Disable the
                * WRITE Protection of SST Flash in the next state */
                hDriver->state = DRV_IPF_SEND_WREN_CMD_ULBPR;
            }
            else
            {
                /* If SPI could not be open, then stay in the same state and
                   keep trying to Open it */
                break;
            }
        }
        case DRV_IPF_SEND_WREN_CMD_ULBPR:
        {
            /* This state is to send the WREN command to be able to modify
             * SST Status register */

            /* Enable CS line */
            SYS_PORTS_PinClear(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);
            
            /* Load Write Enable command */
            hDriver->commandAddressData[0] = IPF_WREN_OP_CODE;
            
            /* Add the buffer in SPI Queue */
            hDriver->spiBufferHandle = (DRV_SPI_BufferAddWrite(hDriver->spiDriverOpenHandle,
                     (uint8_t *)&hDriver->commandAddressData[0], 1, NULL, NULL));

            if (hDriver->spiBufferHandle != (DRV_SPI_BUFFER_HANDLE) NULL)
            {
                /* If the command has been queued,
                 * then check if it has been processed in the next state */
                hDriver->state = DRV_IPF_WREN_EXECUTION_STATUS_CHECK_ULBPR;
            }
            else
            {
                /* this means command has not been allocated in the SPI queue,
                 * so wait in the same state for it to be queued */
                break;
            }
        }
        case DRV_IPF_WREN_EXECUTION_STATUS_CHECK_ULBPR:
        {
            /* Check if the transfer status is success or not */
            if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(hDriver->spiBufferHandle))
            {
                /* this means WREN command has been send,
                 * now disable the CS Line and go to the next state */
                SYS_PORTS_PinSet(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);
                hDriver->state = DRV_IPF_SEND_ULBPR_CMD;
            }
            else
            {
                /* this means WREN command is not yet send,
                 * so wait in the same state for it to be sent */
                break;
            }
        }
        case DRV_IPF_SEND_ULBPR_CMD:
        {
            SYS_PORTS_PinClear(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);

            /* Add WRSR op code */
            hDriver->commandAddressData[0] = IPF_ULBPR_OP_CODE;
            
            /* Add the buffer in SPI Queue */
            hDriver->spiBufferHandle = (DRV_SPI_BufferAddWrite(hDriver->spiDriverOpenHandle,
                     (uint8_t *)&hDriver->commandAddressData[0], 1, NULL, NULL));

            if (hDriver->spiBufferHandle != (DRV_SPI_BUFFER_HANDLE) NULL)
            {
                /* If the command has been queued,
                 * then check if it has been processed in the next state */
                hDriver->state = DRV_IPF_ULBPR_EXECUTION_STATUS_CHECK;
            }
            else
            {
                /* this means command was not queued,
                 * so wait in the same state for it to be queued */
                break;
            }            
            
        }
        case DRV_IPF_ULBPR_EXECUTION_STATUS_CHECK:
        {
            /* Check if the transfer status is success or not */
            if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(hDriver->spiBufferHandle))
            {
                /* This means WRSR command and value has been send, now disable the CS Line */
                SYS_PORTS_PinSet(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);

                hDriver->state = DRV_IPF_SEND_WREN_CMD_WRSR;
            }
            else
            {
                /* this means command and value are not yet sent,
                 * so wait in the same state for it to be sent */
                break;
            }
        }     
        case DRV_IPF_SEND_WREN_CMD_WRSR:
        {
            /* This state is to send the WREN command to be able to modify
             * SST Status register */

            /* Enable CS line */
            SYS_PORTS_PinClear(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);
            
            /* Load Write Enable command */
            hDriver->commandAddressData[0] = IPF_WREN_OP_CODE;
            
            /* Add the buffer in SPI Queue */
            hDriver->spiBufferHandle = (DRV_SPI_BufferAddWrite(hDriver->spiDriverOpenHandle,
                     (uint8_t *)&hDriver->commandAddressData[0], 1, NULL, NULL));

            if (hDriver->spiBufferHandle != (DRV_SPI_BUFFER_HANDLE) NULL)
            {
                /* If the command has been queued,
                 * then check if it has been processed in the next state */
                hDriver->state = DRV_IPF_WREN_EXECUTION_STATUS_CHECK_WRSR;
            }
            else
            {
                /* this means command has not been allocated in the SPI queue,
                 * so wait in the same state for it to be queued */
                break;
            }
        }        
        case DRV_IPF_WREN_EXECUTION_STATUS_CHECK_WRSR:
        {
            /* Check if the transfer status is success or not */
            if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(hDriver->spiBufferHandle))
            {
                /* this means WREN command has been send,
                 * now disable the CS Line and go to the next state */
                SYS_PORTS_PinSet(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);
                hDriver->state = DRV_IPF_SEND_WRSR_CMD_AND_VALUE;
            }
            else
            {
                /* this means WREN command is not yet send,
                 * so wait in the same state for it to be sent */
                break;
            }
        }		
        case DRV_IPF_SEND_WRSR_CMD_AND_VALUE:
        {
            /* This state is to send the WRSR command and Status Register Value */

            /* Enable CS Line */
            SYS_PORTS_PinClear(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);

            /* Add WRSR op code */
            hDriver->commandAddressData[0] = IPF_WRSR_OP_CODE;
            /* Add WRSR value, make BP0, BP1 0 i.e. no write protection */
            hDriver->commandAddressData[1] = 0x0;
            hDriver->commandAddressData[2] = 0x0;

            /* Add the buffer in SPI Queue */
            hDriver->spiBufferHandle = (DRV_SPI_BufferAddWrite(hDriver->spiDriverOpenHandle,
                     (uint8_t *)&hDriver->commandAddressData[0], 3, NULL, NULL));

            if (hDriver->spiBufferHandle != (DRV_SPI_BUFFER_HANDLE) NULL)
            {
                /* If the command has been queued,
                  then check if it has been processed in the next state */
                hDriver->state = DRV_IPF_WRSR_EXECUTION_STATUS_CHECK;
            }
            else
            {
                /* this means command was not queued,
                 * so wait in the same state for it to be queued */
                break;
            }
        }
        case DRV_IPF_WRSR_EXECUTION_STATUS_CHECK:
        {
            /* Check if the transfer status is success or not */
            if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(hDriver->spiBufferHandle))
            {
                /* This means WRSR command and value has been send, now disable the CS Line */
                SYS_PORTS_PinSet(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);

                hDriver->state = DRV_IPF_READ_BPR_STATUS_CMD_SEND;
            }
            else
            {
                /* this means command and value are not yet sent,
                 * so wait in the same state for it to be sent */
                break;
            }
        }
		case DRV_IPF_READ_BPR_STATUS_CMD_SEND:
		{
            /* Enable CS Line */
            SYS_PORTS_PinClear(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);

            /* Add WRSR op code */
            hDriver->commandAddressData[0] = IPF_READ_BLOCK_PROT_STATUS;
            
            /* Add the buffer in SPI Queue */
            hDriver->spiBufferHandle = (DRV_SPI_BufferAddWrite(hDriver->spiDriverOpenHandle,
                     (uint8_t *)&hDriver->commandAddressData[0], 1, NULL, NULL));

            if (hDriver->spiBufferHandle != (DRV_SPI_BUFFER_HANDLE) NULL)
            {
                /* If the command has been queued,
                 * then check if it has been processed in the next state */
                hDriver->state = DRV_IPF_READ_BPR_STATUS;
            }
            else
            {
                /* this means command was not queued,
                 * so wait in the same state for it to be queued */
                break;
            }			
		}	
        case DRV_IPF_READ_BPR_STATUS:
        {
            /* Read the Status Register value */
            hDriver->spiBufferHandle = (DRV_SPI_BufferAddRead(hDriver->spiDriverOpenHandle,
                     (uint8_t *)&gDrvBlockProtReg[0], 6, NULL, NULL));

            if (hDriver->spiBufferHandle != (DRV_SPI_BUFFER_HANDLE) NULL)
            {
                /* This means command has been allocated in the SPI queue,
                 *  now wait for it to be executed in the next state */
                hDriver->state = DRV_IPF_READ_BPR_COMPLETE_CHECK;
            }
            else
            {
                /* This means command has not been allocated in the SPI queue,
                 * wait for it to be queued */
                break;
            }
        }		
        case DRV_IPF_READ_BPR_COMPLETE_CHECK:
        {
            if(DRV_SPI_BUFFER_EVENT_COMPLETE & DRV_SPI_BufferStatus(hDriver->spiBufferHandle))
            {
				/* Disable CS */
				SYS_PORTS_PinSet(PORTS_ID_0, hDriver->chipSelectPortChannel, hDriver->chipSelectBitPosition);

				/* Now Set the SST Driver status as ready and process the queue */
				hDriver->status =  SYS_STATUS_READY ;
				hDriver->state = DRV_IPF_TASK_PROCESS_QUEUE;
			}
            else
            {
                break;
            }
        }		
        case DRV_IPF_TASK_PROCESS_QUEUE:
        {
            /* get the first object of the queue */
            bufferObj = hDriver->queueTail;
            
            if(bufferObj != NULL)
            {
                /* This means the queue is not empty. Check if this buffer is done */
                if(bufferObj->nCurrentBlocks >= bufferObj->size)
                {
                    /* This means the buffer is completed. If there
                    is a callback registered with client, then
                    call it */

                    client = (DRV_IPF_CLIENT_OBJ *)bufferObj->hClient;
                    if(client->eventHandler != NULL)
                    {
                        client->eventHandler(DRV_IPF_EVENT_BLOCK_COMMAND_COMPLETE,
                            (DRV_IPF_BLOCK_COMMAND_HANDLE)bufferObj,
                            client->context);
                    }

                    /* Get the next buffer in the queue and deallocate this buffer */
                    hDriver->queueTail = bufferObj->next;
                    bufferObj->inUse = false;
                    hDriver->queueOccupancy --;
                }
            }

            /* Check if the queue is still not empty, if so process the buffer task */
            if(hDriver->queueTail != NULL)
            {
                _DRV_IPF_ProcessBufferTask(hDriver, hDriver->queueTail);
            }
            break;
        }
        default:
            break;
    }
}

// **************************************************************************
/* Function:
    void DRV_IPF_BlockErase
    (
        const DRV_HANDLE hClient,
        DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle,
        uint32_t blockStart,
        uint32_t nBlock
    )

  Summary:
    Erase the specified number of blocks in flash memory.

  Description:
    This function schedules a non-blocking erase operation in flash memory.
    The function returns with a valid erase handle in the commandHandle argument
    if the erase request was scheduled successfully. The function adds the
    request to the hardware instance queue and returns immediately.
    The function returns DRV_IPF_EVENT_BLOCK_HANDLE_INVALID in the
    commandHandle argument
    - if the client opened the driver for read only
    - if nBlock is 0.
    - if the queue size is full or queue depth is insufficient.
    - if the driver handle is invalid
    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_IPF_EVENT_ERASE_COMPLETE event if the
    erase operation was successfull or DRV_IPF_EVENT_ERASE_ERROR
    event if the erase operation was not successfull.

    The block start address should be aligned on a
    DRV_IPF_ERASE_BLOCK_SIZE byte boundary.
  
  Remarks:
    See drv_ipf.h for usage information.
*/

void DRV_IPF_BlockErase
(
    const DRV_HANDLE hClient,
    DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_IPF_CLIENT_OBJ * clientObj = NULL;
    DRV_IPF_OBJ * hDriver = NULL;
    DRV_IPF_BUFFER_OBJ * bufferObj = NULL, * iterator = NULL;
    unsigned int i = 0;

    /* This function adds a buffer to the queue */

    /* We first check the arguments and initialize the
       command handle to INVALID */
    if(commandHandle != NULL)
    {
        *commandHandle = DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID;
    }

    /* Validate the driver handle */
    clientObj = _DRV_IPF_DriverHandleValidate(hClient);
    if(clientObj == NULL)
    {
        /* We got an invalid client handle */
        SYS_DEBUG(0, "DRV_IPF: Invalid Driver Handle \n");
        return;
    }

    /* Check if Driver was open for Wtiting or not */
    if(!(clientObj->ioIntent & DRV_IO_INTENT_WRITE))
    {
        /* Driver is not open in Write mode */
        SYS_DEBUG(0, "DRV_IPF: Driver is not open in Write mode \n");
        return;
    }

    if((nBlock == 0))
    {
        /* We got an invalid block size */
        SYS_DEBUG(0, "DRV_IPF: Invalid block size parameter \n");
        return;    
    }
    
    hDriver = clientObj->hDriver;

    if(hDriver->queueOccupancy >= hDriver->queueSize)
    {
        /* This means the queue is full for this driver instance. We cannot add
           this request */

        SYS_DEBUG(0, "DRV_IPF: Queue is full for this driver instance \n");
        return;
    }

    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i < DRV_IPF_QUEUE_SIZE; i ++)
    {
        if(!gDrvIPFBufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj = &gDrvIPFBufferObj[i];
            bufferObj->hClient = clientObj;
            bufferObj->inUse = true;
            bufferObj->nCurrentBlocks = 0;
            bufferObj->size = nBlock;
            bufferObj->address = blockStart;
            bufferObj->next = NULL;
            bufferObj->previous = NULL;
            /* Erase API doesn't need buffer pointer */
            bufferObj->buffer = NULL;
            bufferObj->operation = DRV_IPF_BLOCK_ERASE;

            if(commandHandle != NULL)
            {
                /* Assign a handle to this buffer */
                *commandHandle = (DRV_IPF_BLOCK_COMMAND_HANDLE)bufferObj;
            }
            break;
        }
    }

    if(i == DRV_IPF_QUEUE_SIZE)
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_IPF_QUEUE_SIZE
           parameter is configured to be less */

        SYS_DEBUG(0, "DRV_IPF: Insufficient Queue Depth");
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
        /* This means the queue is not empty. We must add
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

// *****************************************************************************
/* Function:
    void DRV_IPF_BlockRead
    (
        const DRV_HANDLE handle,
        DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle,
        uint8_t *targetBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Reads blocks of data starting from the specified address in flash memory.

  Description:
    This function schedules a non-blocking read operation for reading blocks of
    data from flash memory. The function returns with a valid handle in
    the commandHandle argument if the read request was scheduled successfully.
    The function adds the request to the hardware instance queue and
    returns immediately. While the request is in the queue, the application
    buffer is owned by the driver and should not be modified.  The function
    returns DRV_IPF_EVENT_BLOCK_HANDLE_INVALID in the commandHandle
    argument under the following circumstances:
    - if a buffer could not be allocated to the request
    - if the target buffer pointer is NULL
    - if the client opened the driver for write only
    - if the buffer size is 0
    - if the read queue size is full or queue depth is insufficient
    - if the driver handle is invalid
    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_IPF_EVENT_BLOCK_COMPLETE event if the
    buffer was processed successfully of DRV_IPF_EVENT_BLOCK_ERROR
    event if the buffer was not processed successfully.

    The block start address should be aligned on a
    DRV_IPF_READ_BLOCK_SIZE byte boundary.

  Remarks:
    See drv_ipf.h for usage information.
*/

void DRV_IPF_BlockRead
(
    const DRV_HANDLE hClient,
    DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle,
    uint8_t *targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_IPF_CLIENT_OBJ * clientObj = NULL;
    DRV_IPF_OBJ * hDriver = NULL;
    DRV_IPF_BUFFER_OBJ * bufferObj = NULL, * iterator = NULL;
    unsigned int i = 0;

    /* This function adds a buffer to the queue */

    /* We first check the arguments and initialize the
       command handle to INVALID */

    if(commandHandle != NULL)
    {
       * commandHandle = DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID;
    }

    /* Validate the driver handle */
    clientObj = _DRV_IPF_DriverHandleValidate(hClient);
    if(clientObj == NULL)
    {
        /* We got an invalid client handle */
        SYS_DEBUG(0, "DRV_IPF: Invalid Driver Handle \n");
        return;
    }

    /* Check if Driver was open for Reading or not */
    if(!(clientObj->ioIntent & DRV_IO_INTENT_READ))
    {
        /* Driver is not open in Read mode */
        SYS_DEBUG(0, "DRV_IPF: Driver is not open in Read mode \n");
        return;
    }

    if((nBlock == 0) || (targetBuffer == NULL ))
    {
        /* We either got an invalid block size or target buffer */
        SYS_DEBUG(0, "DRV_IPF: Invalid parameters \n");
        return;
    }
    
    hDriver = clientObj->hDriver;

    if(hDriver->queueOccupancy >= hDriver->queueSize)
    {
        /* This means the queue is full. We cannot add
           this request */

        SYS_DEBUG(0, "DRV_IPF: Queue is full for this driver instance \n");
        return;
    }

    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i < DRV_IPF_QUEUE_SIZE; i ++)
    {
        if(!gDrvIPFBufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj = &gDrvIPFBufferObj[i];
            bufferObj->hClient = clientObj;
            bufferObj->inUse = true;
            bufferObj->nCurrentBlocks = 0;
            bufferObj->size = nBlock;
            bufferObj->address = blockStart;
            bufferObj->next = NULL;
            bufferObj->previous = NULL;
            bufferObj->buffer = targetBuffer;
            bufferObj->operation = DRV_IPF_BLOCK_READ;

            if(commandHandle != NULL)
            {
                /* Assign a handle to this buffer */
                *commandHandle = (DRV_IPF_BLOCK_COMMAND_HANDLE)bufferObj;
            }
            break;
        }
    }

    if(i == DRV_IPF_QUEUE_SIZE)
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_IPF_QUEUE_SIZE
           parameter is configured to be less */

        SYS_DEBUG(0, "DRV_IPF: Insufficient Queue Depth \n");
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

// *****************************************************************************
/* Function:
    void DRV_IPF_BlockWrite
    (
        DRV_HANDLE handle,
        DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle,
        uint8_t *sourceBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Write blocks of data starting from a specified address in flash memory.

  Description:
    This function schedules a non-blocking write operation for writing blocks of
    data into flash memory. The function returns with a valid buffer handle in
    the commandHandle argument if the write request was scheduled successfully.
    The function adds the request to the hardware instance queue and
    returns immediately. While the request is in the queue, the application
    buffer is owned by the driver and should not be modified.  The function
    returns DRV_IPF_EVENT_BLOCK_HANDLE_INVALID in the commandHandle
    argument under the following circumstances:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the client opened the driver for read only
    - if the buffer size is 0
    - if the write queue size is full or queue depth is insufficient
    - if the driver handle is invalid
    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_IPF_EVENT_BLOCK_COMPLETE event if the
    buffer was processed successfully or DRV_IPF_EVENT_BLOCK_ERROR
    event if the buffer was not processed successfully.

    The block start address should be aligned on a
    DRV_IPF_WRITE_BLOCK_SIZE byte boundary.

  Remarks:
    See drv_ipf.h for usage information.
*/

void DRV_IPF_BlockWrite
(
    DRV_HANDLE hClient,
    DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle,
    uint8_t *sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
)
{
    DRV_IPF_CLIENT_OBJ * clientObj = NULL;
    DRV_IPF_OBJ * hDriver = NULL;
    DRV_IPF_BUFFER_OBJ * bufferObj = NULL, * iterator = NULL;
    unsigned int i = 0;

    /* This function adds a buffer to the queue */

    /* We first check the arguments and initialize the
       command handle to INVALID */

    if(commandHandle != NULL)
    {
        *commandHandle = DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID;
    }

    /* Validate the driver handle */
    clientObj = _DRV_IPF_DriverHandleValidate(hClient);
    if(clientObj == NULL)
    {
        /* We got an invalid client handle */
        SYS_DEBUG(0, "DRV_IPF: Invalid Driver Handle \n");
        return;
    }

    /* Check if Driver was open for Wtiting or not */
    if(!(clientObj->ioIntent & DRV_IO_INTENT_WRITE))
    {
        /* Driver is not open in Write mode */
        SYS_DEBUG(0, "DRV_IPF: Driver is not open in Write mode \n");
        return;
    }
    
    if((nBlock == 0) || (sourceBuffer == NULL ))
    {
        /* We either got an invalid block size or
         * invalid source buffer */
        SYS_DEBUG(0, "DRV_IPF: Invalid parameters \n");
        return;
    }
    
    hDriver = clientObj->hDriver;

    if(hDriver->queueOccupancy >= hDriver->queueSize)
    {
        /* This means the queue is full. We cannot add
           this request */

        SYS_DEBUG(0, "DRV_IPF: Queue is full \n");
        return;
    }

    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i < DRV_IPF_QUEUE_SIZE; i ++)
    {
        if(!gDrvIPFBufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj = &gDrvIPFBufferObj[i];
            bufferObj->hClient = clientObj;
            bufferObj->inUse = true;
            bufferObj->nCurrentBlocks = 0;
            bufferObj->size = nBlock;
            bufferObj->address = blockStart;
            bufferObj->next = NULL;
            bufferObj->previous = NULL;
            bufferObj->buffer = sourceBuffer;
            bufferObj->operation = DRV_IPF_BLOCK_WRITE;

            if(commandHandle != NULL)
            {
                /* Assign a handle to this buffer */
                *commandHandle = (DRV_IPF_BLOCK_COMMAND_HANDLE)bufferObj;
            }
            break;
        }
    }

    if(i == DRV_IPF_QUEUE_SIZE)
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_IPF_QUEUE_SIZE
           parameter is configured to be less */

        SYS_DEBUG(0, "DRV_IPF: Insufficient Queue Depth \n");
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

void DRV_IPF_ProtectMemoryVolatile
(
    DRV_HANDLE clientHandle, 
    DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle, 
    uintptr_t memAddress, 
    DRV_IPF_PROT_MODE protMode
)
{
	DRV_IPF_CLIENT_OBJ * clientObj = NULL;
    DRV_IPF_OBJ * hDriver = NULL;
    DRV_IPF_BUFFER_OBJ * bufferObj = NULL, * iterator = NULL;    
	uint8_t bitPosition = 0;
    uint8_t i = 0;
    
	/* Validate the driver handle */
	clientObj = _DRV_IPF_DriverHandleValidate(clientHandle);
	if(clientObj == NULL)
	{
		/* We got an invalid client handle */
		SYS_DEBUG(0, "DRV_IPF: Invalid Driver Handle \n");
		return;
	}

    /* Check if Driver was open for Writing or not */
    if(!(clientObj->ioIntent & DRV_IO_INTENT_WRITE))
    {
        /* Driver is not open in Write mode */
        SYS_DEBUG(0, "DRV_IPF: Driver is not open in Write mode \n");
        return;
    }
    
	bitPosition = DRV_IPF_GetBlockProtectBitPosition(memAddress, protMode);

	if(bitPosition == 0xFF)
	{
		SYS_DEBUG(0, "DRV_IPF: Invalid Protection type for the addressed block \n");
		return;
	}

//  As Multiple Prot and Unprot is supported in a active task queue, 
//  it cannot be determined whether a block is already protected as there can be
//  unexecuted unprotect call waiting in the queue before this protect command    
//	if(DRV_IPF_CheckBitInArray(gDrvBlockProtReg, bitPosition) == true)
//	{
//		SYS_DEBUG(0, "DRV_IPF: Block Already protected \n");
//		return;
//	}
    
    hDriver = clientObj->hDriver;
    
    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i < DRV_IPF_QUEUE_SIZE; i ++)
    {
        if(!gDrvIPFBufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj = &gDrvIPFBufferObj[i];
            bufferObj->hClient = clientObj;
            bufferObj->inUse = true;
            bufferObj->nCurrentBlocks = 0;
            bufferObj->size = 1;
            bufferObj->address = memAddress;
            bufferObj->next = NULL;
            bufferObj->previous = NULL;
            bufferObj->buffer = NULL;
            bufferObj->operation = DRV_IPF_HW_BLOCK_PROT;
			bufferObj->protBitPos = bitPosition;
            bufferObj->protMode = protMode;

            if(commandHandle != NULL)
            {
                /* Assign a handle to this buffer */
                *commandHandle = (DRV_IPF_BLOCK_COMMAND_HANDLE)bufferObj;
            }
            break;
        }
    }

    if(i == DRV_IPF_QUEUE_SIZE)
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_IPF_QUEUE_SIZE
           parameter is configured to be less */

        SYS_DEBUG(0, "DRV_IPF: Insufficient Queue Depth \n");
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

void DRV_IPF_UnProtectMemoryVolatile
(
    DRV_HANDLE clientHandle, 
    DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle, 
    uintptr_t memAddress, 
    DRV_IPF_PROT_MODE protMode
)
{
	DRV_IPF_CLIENT_OBJ * clientObj = NULL;
    DRV_IPF_OBJ * hDriver = NULL;
    DRV_IPF_BUFFER_OBJ * bufferObj = NULL, * iterator = NULL;    
	uint8_t bitPosition = 0;
    uint8_t i = 0;
    
	/* Validate the driver handle */
	clientObj = _DRV_IPF_DriverHandleValidate(clientHandle);
	if(clientObj == NULL)
	{
		/* We got an invalid client handle */
		SYS_DEBUG(0, "DRV_IPF: Invalid Driver Handle \n");
		return;
	}

    /* Check if Driver was open for Writing or not */
    if(!(clientObj->ioIntent & DRV_IO_INTENT_WRITE))
    {
        /* Driver is not open in Write mode */
        SYS_DEBUG(0, "DRV_IPF: Driver is not open in Write mode \n");
        return;
    }
    
	bitPosition = DRV_IPF_GetBlockProtectBitPosition(memAddress, protMode);

	if(bitPosition == 0xFF)
	{
		SYS_DEBUG(0, "DRV_IPF: Invalid Protection type for the addressed block \n");
		return;
	}
    
//  As Multiple Prot and Unprot is supported in a active task queue, 
//  it cannot be determined whether a block is already unprotected    
//    if(DRV_IPF_CheckBitInArray(gDrvBlockProtReg, bitPosition) == false)
//    {
//        SYS_DEBUG(0, "DRV_IPF: The block is currently un-protected \n");
//		return;        
//    }

/*  [MH-4618] [DRV_IPF] Memory unprotect is not possible from different client  */    
//	if(DRV_IPF_CheckBitInArray(clientObj->blockProtStatus, bitPosition) == false)
//	{
//		SYS_DEBUG(0, "DRV_IPF: Cannot un-protect this block, as it is protected by other clients \n");
//		return;
//	}

    hDriver = clientObj->hDriver;
    
    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i < DRV_IPF_QUEUE_SIZE; i ++)
    {
        if(!gDrvIPFBufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj = &gDrvIPFBufferObj[i];
            bufferObj->hClient = clientObj;
            bufferObj->inUse = true;
            bufferObj->nCurrentBlocks = 0;
            bufferObj->size = 1;
            bufferObj->address = memAddress;
            bufferObj->next = NULL;
            bufferObj->previous = NULL;
            bufferObj->buffer = NULL;
            bufferObj->operation = DRV_IPF_HW_BLOCK_UNPROT;
            bufferObj->protBitPos = bitPosition;
            bufferObj->protMode = protMode;
            
            if(commandHandle != NULL)
            {
                /* Assign a handle to this buffer */
                *commandHandle = (DRV_IPF_BLOCK_COMMAND_HANDLE)bufferObj;
            }
            break;
        }
    }

    if(i == DRV_IPF_QUEUE_SIZE)
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_IPF_QUEUE_SIZE
           parameter is configured to be less */

        SYS_DEBUG(0, "DRV_IPF: Insufficient Queue Depth \n");
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

void DRV_IPF_ReadBlockProtectionStatus
(
    DRV_HANDLE clientHandle, 
    DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle, 
    uint8_t * buffer
)
{
	DRV_IPF_CLIENT_OBJ * clientObj = NULL;
    DRV_IPF_OBJ * hDriver = NULL;
    DRV_IPF_BUFFER_OBJ * bufferObj = NULL, * iterator = NULL;    
    uint8_t i = 0;
    
	/* Validate the driver handle */
	clientObj = _DRV_IPF_DriverHandleValidate(clientHandle);
	if(clientObj == NULL)
	{
		/* We got an invalid client handle */
		SYS_DEBUG(0, "DRV_IPF: Invalid Driver Handle \n");
		return;
	}

    /* Check if Driver was open for Writing or not */
    if(!(clientObj->ioIntent & DRV_IO_INTENT_READ))
    {
        /* Driver is not open in Write mode */
        SYS_DEBUG(0, "DRV_IPF: Driver is not open in Read mode \n");
        return;
    }
       
    hDriver = clientObj->hDriver;
    
    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i < DRV_IPF_QUEUE_SIZE; i ++)
    {
        if(!gDrvIPFBufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj = &gDrvIPFBufferObj[i];
            bufferObj->hClient = clientObj;
            bufferObj->inUse = true;
            bufferObj->nCurrentBlocks = 0;
            bufferObj->size = DRV_IPF_BLOCK_PROT_BYTES;
            bufferObj->address = (uint32_t) NULL;
            bufferObj->next = NULL;
            bufferObj->previous = NULL;
            bufferObj->buffer = buffer;
            bufferObj->operation = DRV_IPF_READ_HW_BLOCK_PROT;

            if(commandHandle != NULL)
            {
                /* Assign a handle to this buffer */
                *commandHandle = (DRV_IPF_BLOCK_COMMAND_HANDLE)bufferObj;
            }
            break;
        }
    }

    if(i == DRV_IPF_QUEUE_SIZE)
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_IPF_QUEUE_SIZE
           parameter is configured to be less */

        SYS_DEBUG(0, "DRV_IPF: Insufficient Queue Depth \n");
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

/* APIs to control WP and Hold Pins */

void DRV_IPF_WPAssert()
{
	_DRV_IPF_WPAssert();
}

void DRV_IPF_WPDeAssert()
{
    _DRV_IPF_WPDeAssert();
}

void DRV_IPF_HoldAssert()
{
   _DRV_IPF_HoldAssert();
}

void DRV_IPF_HoldDeAssert()
{
	_DRV_IPF_HoldDeAssert();
}

/*******************************************************************************
 End of File
*/
