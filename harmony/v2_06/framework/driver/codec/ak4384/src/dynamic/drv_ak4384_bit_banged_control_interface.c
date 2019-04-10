/*******************************************************************************
  AK4384 CODEC Driver Dynamic implementation.

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ak4384.c

  Summary:
    AK4384 CODEC Driver Dynamic implementation.

  Description:
    This file contains the Dynamic mode implementation of the AK4384 driver.
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

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
// DOM-IGNORE-END

/*************************************************************
 * Include files.
 ************************************************************/
#include "driver/codec/ak4384/src/drv_ak4384_local.h"


// *****************************************************************************
/* Driver Hardware instance objects.

  Summary:
    Defines the hardware instances objects for the AK4384 CODEC

  Description:
    This data type defines the hardware instance objects that are available for
    AK4384 CODEC, so as to capture the hardware state of the instance.

  Remarks:
    Not all modes are available on all micro-controllers.
 */
DRV_AK4384_OBJ gDrvak4384Obj[DRV_AK4384_INSTANCES_NUMBER];


// *****************************************************************************
/* Driver Client instance objects.

  Summary:
    Defines the client instances objects

  Description:
    This data type defines the client instance objects that are available on
    AK4384, so as to capture the client state of the instance.
    It uses the configuration of maximum number of clients which can get
    registered per hardware instance.

  Remarks:
    Not all modes are available on all micro-controllers.
 */
DRV_AK4384_CLIENT_OBJ gDrvak4384ClientObj[DRV_AK4384_CLIENTS_NUMBER];


// *****************************************************************************
/* Driver common data object

  Summary:
    Defines the common data object

  Description:
    This object maintains data that is required by all AK4384
   driver instances

  Remarks:
    None
 */
DRV_AK4384_COMMON_DATA_OBJ gDrvak4384CommonDataObj;

// *****************************************************************************
// *****************************************************************************
// Section: AK4384 CODEC Driver System Routine Implementations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
	SYS_MODULE_OBJ  DRV_AK4384_Initialize
	(
		const SYS_MODULE_INDEX drvIndex,
        const SYS_MODULE_INIT *const init
	);

  Summary:
    Initializes hardware and data for the instance of the AK4384 DAC module

  Description:
    This routine initializes the AK4384 driver instance for the specified driver
    index, making it ready for clients to open and use it. The initialization
    data is specified by the init parameter. The initialization may fail if the
    number of driver objects allocated are insufficient or if the specified
    driver instance is already initialized.

  Remarks:
    This routine must be called before any other AK4384 routine is called.

    This routine should only be called once during system initialization
    unless DRV_AK4384_Deinitialize is called to de-initialize the driver
    instance. This routine will NEVER block for hardware access.

 */
SYS_MODULE_OBJ  DRV_AK4384_Initialize
(
	const SYS_MODULE_INDEX drvIndex,
	const SYS_MODULE_INIT *const init
)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_INIT *ak4384Init;
    uint8_t index;

    /* Validate the driver index */
    if (drvIndex >= DRV_AK4384_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "Invalid driver index \r\n");
        return SYS_MODULE_OBJ_INVALID;
    }

    if (true == gDrvak4384Obj[drvIndex].inUse)
    {
        /* Cannot initialize an object that is already in use. */
        SYS_DEBUG(0, "Instance already in use \r\n");
        return SYS_MODULE_OBJ_INVALID;
    }

    ak4384Init = (DRV_AK4384_INIT *) init;
    drvObj = (DRV_AK4384_OBJ *)&gDrvak4384Obj[drvIndex];

    /* Populate the driver object with the required data */
    drvObj->inUse                           = true;
    drvObj->status                          = SYS_STATUS_UNINITIALIZED;
    drvObj->numClients                      = 0;
    drvObj->i2sDriverModuleIndex            = ak4384Init->i2sDriverModuleIndex;
    drvObj->samplingRate                    = DRV_AK4384_AUDIO_SAMPLING_RATE;
    drvObj->mclkMode                        = ak4384Init->mclkMode;
    drvObj->audioDataFormat                 = DRV_AK4384_AUDIO_DATA_FORMAT_MACRO;
    drvObj->tmrDriverModuleIndex            = DRV_AK4384_TIMER_DRIVER_MODULE_INDEX;
    drvObj->isInInterruptContext            = false;

    /*Assigning the init volume to all supported audio channels*/
    for(index=0; index < DRV_AK4384_NUMBER_OF_CHANNELS; index++)
    {
        drvObj->volume[index] = ak4384Init->volume;
    }

    /* Default values of CODEC control registers */
    drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_1] = 0x8B;
    drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2] = 0x2;
    drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_3] = 0x0;
    drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_LATT] = 0xFF;
    drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_RATT] = 0xFF;

    /* Initialize */
    drvObj->commandCompleteCallback = (DRV_AK4384_COMMAND_EVENT_HANDLER)0;
    drvObj->commandContextData = -1;
    drvObj->mclk_multiplier = DRV_AK4384_MCLK_SAMPLE_FREQ_MULTPLIER;
    drvObj->bclk_divider = DRV_AK4384_BCLK_BIT_CLK_DIVISOR;
	drvObj->delayDriverInitialization = ak4384Init->delayDriverInitialization;

    /* Create the hardware instance mutex. */
     if(OSAL_MUTEX_Create(&(drvObj->mutexDriverInstance)) != OSAL_RESULT_TRUE)
     {
        return SYS_MODULE_OBJ_INVALID;
     }

    /* Check if the global mutexes have been created. If not
       then create these. */
     if(!gDrvak4384CommonDataObj.membersAreInitialized)
     {
         /* This means that mutexes where not created. Create them. */
         if((OSAL_MUTEX_Create(&(gDrvak4384CommonDataObj.mutexClientObjects)) != OSAL_RESULT_TRUE))
         {
            return SYS_MODULE_OBJ_INVALID;
         }
         /* Set this flag so that global mutexes get allocated only once */
         gDrvak4384CommonDataObj.membersAreInitialized = true;
     }

    drvObj->status = SYS_STATUS_BUSY;
    drvObj->command = DRV_AK4384_COMMAND_INIT_OPEN_TIMER;

    /* Return the object structure */
    return ((SYS_MODULE_OBJ) drvObj);

} /* DRV_AK4384_Initialize */


// *****************************************************************************
/* Function:
    void DRV_AK4384_Deinitialize( SYS_MODULE_OBJ object)

  Summary:
    Deinitializes the specified instance of the AK4384 driver module

  Description:
    Deinitializes the specified instance of the AK4384 driver module, disabling
    its operation (and any hardware).  Invalidates all the internal data.

  Remarks:
    Once the Initialize operation has been called, the De-initialize operation
    must be called before the Initialize operation can be called again. This
    routine will NEVER block waiting for hardware.
*/
void DRV_AK4384_Deinitialize( SYS_MODULE_OBJ object)
{
    DRV_AK4384_OBJ *drvObj;

    if (object == SYS_MODULE_OBJ_INVALID || object == (SYS_MODULE_OBJ)NULL)
    {
        /* Invalid object */
        SYS_DEBUG(0, "Invalid object \r\n");
        return;
    }

    drvObj = (DRV_AK4384_OBJ *) object;
    if (false == drvObj->inUse)
    {
        /* Cannot deinitialize an object that is
         * not already in use. */
        SYS_DEBUG(0, "Instance not in use \r\n");
        return;
    }

    /* Deallocate all the mutexes */
     if((OSAL_MUTEX_Delete(&(drvObj->mutexDriverInstance)) != OSAL_RESULT_TRUE))
     {
        SYS_DEBUG(0, "Unable to delete client handle mutex \r\n");
        return;
     }

    DRV_I2S_Close (drvObj->i2sDriverHandle);
    
    /* Indicate that this object is not is use */
    drvObj->inUse = false;
    /* Set number of clients to zero */
    drvObj->numClients = 0;
    drvObj->status = SYS_STATUS_UNINITIALIZED;

    return;
}


// *****************************************************************************
/* Function:
    SYS_STATUS DRV_AK4384_Status( SYS_MODULE_OBJ object)

  Summary:
    Gets the current status of the AK4384 driver module.

  Description:
    This routine provides the current status of the AK4384 driver module.

  Remarks:
    A driver can opened only when its status is SYS_STATUS_READY.
*/
SYS_STATUS DRV_AK4384_Status( SYS_MODULE_OBJ object)
{
    DRV_AK4384_OBJ *drvObj;

    if (object == SYS_MODULE_OBJ_INVALID ||
        object < DRV_AK4384_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "System Module Object is invalid \r\n");
        return SYS_STATUS_ERROR;
    }
    drvObj = (DRV_AK4384_OBJ *)object;

    /* Return the status of the driver object */
    return drvObj->status;
} /* DRV_AK4384_Status */

// *****************************************************************************
/* Function:
    void DRV_AK4384_EnableInitialization(SYS_MODULE_OBJ object);

  Summary:
   Enable delayed initialization of the driver.

  Description:
   If the AK4384 codec is sharing a RESET line with another peripheral, such as
   a Bluetooth module with its own driver, then the codec driver initialization
   has to be delayed until after the Bluetooth module has toggled its RESET pin.
   Once this has been accomplished, this function should be called to kick-start
   the codec driver initialization.

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_AK4384_Initialize)
  Returns:
    None.

  Remarks:
    This is not needed for audio-only applications without a Bluetooth module.
*/
void DRV_AK4384_EnableInitialization(SYS_MODULE_OBJ object)
{
    DRV_AK4384_OBJ *drvObj;
    
    drvObj = (DRV_AK4384_OBJ *)object;

    if((false == drvObj->inUse))
    {
        /* This instance of the driver is not initialized. Don't
         * do anything */
        return;
    }

    if ((true == drvObj->delayDriverInitialization) &&
        (DRV_AK4384_COMMAND_NONE == drvObj->command))
    {
        drvObj->command = DRV_AK4384_COMMAND_INIT_CLK_PDN_SET;
    }
}

// *****************************************************************************
/* Function:
    bool DRV_AK4384_IsInitializationDelayed(SYS_MODULE_OBJ object);

  Summary:
   Checks if delayed initialization of the driver has been requested.

  Description:
   If the AK4384 codec is sharing a RESET line with another peripheral, such as
   a Bluetooth module with its own driver, then the codec driver initialization
   has to be delayed until after the Bluetooth module has toggled its RESET pin.
   This function returns true if that option has been selected in MHC in the
   checkbox: "Delay driver initialization (due to shared RESET pin)"

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_AK4384_Initialize)
  Returns:
    true if the delayed initilization option has been enabled

  Remarks:
    This is not needed for audio-only applications without a Bluetooth module.
*/

bool DRV_AK4384_IsInitializationDelayed(SYS_MODULE_OBJ object)
{
    DRV_AK4384_OBJ *drvObj;
    
    drvObj = (DRV_AK4384_OBJ *)object;

    if((false == drvObj->inUse))
    {
        /* This instance of the driver is not initialized. Don't
         * do anything */
        return false;
    }

    return drvObj->delayDriverInitialization;   // return true or false
}


// *****************************************************************************
/* Function:
    void  DRV_AK4384_Tasks(SYS_MODULE_OBJ object);

  Summary:
    Maintains the driver's control and data interface state machine.

  Description:
    This routine is used to maintain the driver's internal control and data
    interface state machine and implement its control and data interface
    implementations.
    This function should be called from the SYS_Tasks() function.

  Remarks:
    This routine is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks).

*/
void DRV_AK4384_Tasks(SYS_MODULE_OBJ object)
{
    DRV_AK4384_OBJ *drvObj;

    drvObj = (DRV_AK4384_OBJ *)object;

    if((false == drvObj->inUse))
    {
        /* This intance of the driver is not initialized. Dont do anything */
        return;
    }

    _DRV_AK4384_ControlTasks(drvObj);    
    return;
}


// *****************************************************************************
// *****************************************************************************
// Section: AK4384 CODEC Driver Client Routines
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_AK4384_Open
    (
		const SYS_MODULE_INDEX drvIndex,
		const DRV_IO_INTENT    ioIntent
	)

  Summary:
    Opens the specified AK4384 driver instance and returns a handle to it

  Description:
    This routine opens the specified AK4384 driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    The DRV_IO_INTENT_BLOCKING and DRV_IO_INTENT_NONBLOCKING ioIntent
    options are not relevant to this driver. All the data transfer functions
    of this driver are non blocking.

    Only DRV_IO_INTENT_WRITE is a valid ioIntent option as AK4384 is DAC only.

    Specifying a DRV_IO_INTENT_EXCLUSIVE will cause the driver to provide
    exclusive access to this client. The driver cannot be opened by any
    other client.

  Remarks:
    The handle returned is valid until the DRV_AK4384_Close routine is called.
    This routine will NEVER block waiting for hardware.If the requested intent
    flags are not supported, the routine will return DRV_HANDLE_INVALID.  This
    function is thread safe in a RTOS application. It should not be called in an
    ISR.
*/
DRV_HANDLE DRV_AK4384_Open
(
	const SYS_MODULE_INDEX iDriver,
	const DRV_IO_INTENT ioIntent
)
{
    DRV_AK4384_CLIENT_OBJ *hClient;
    DRV_AK4384_OBJ *drvObj;
    uint32_t iClient;

    /* The iDriver value should be valid. It should be
     * less the number of driver object instances.
     */
    if (iDriver >= DRV_AK4384_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "Bad Driver Index \r\n");
        return DRV_HANDLE_INVALID;
    }
    
    drvObj = (DRV_AK4384_OBJ *)&gDrvak4384Obj[iDriver];
    if (drvObj->status == SYS_STATUS_BUSY)
    {
        return DRV_HANDLE_INVALID;
    }
    
    if (drvObj->status != SYS_STATUS_READY)
    {
        /* The ak4384  module should be ready */
        SYS_DEBUG(0, "Was the driver initialized? \r\n");
        return DRV_HANDLE_INVALID;
    }

    if ((drvObj->numClients > 0) && (true == drvObj->isExclusive))
    {
        /* Driver already opened in exclusive mode. Cannot open a new client. */
        SYS_DEBUG(0, "Cannot open a new client in exclusive mode \r\n");
        return DRV_HANDLE_INVALID;
    }

    if ((drvObj->numClients > 0) &&
        (DRV_IO_INTENT_EXCLUSIVE == (ioIntent & DRV_IO_INTENT_EXCLUSIVE)))
    {
        /*  A client Instance of driver is open.
            Cannot open the new client in exclusive mode */
            SYS_DEBUG(0, "Cannot open a new client in exclusive mode \r\n");
            return DRV_HANDLE_INVALID;
    }

    iClient = 0;
    hClient = (DRV_AK4384_CLIENT_OBJ *)&gDrvak4384ClientObj[iClient];

    /* Grab client object mutex here */
    if(OSAL_MUTEX_Lock(&(gDrvak4384CommonDataObj.mutexClientObjects), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        /* Setup client operations */
        /* Find available slot in array of client objects */
        for (; iClient < DRV_AK4384_CLIENTS_NUMBER; iClient++)
        {
            if (false == hClient->inUse)
            {
                /* Set the exlusive mode for the driver instance */
                if (DRV_IO_INTENT_EXCLUSIVE == (ioIntent & DRV_IO_INTENT_EXCLUSIVE))
                {
                    drvObj->isExclusive = true;
                }

                hClient->ioIntent = DRV_IO_INTENT_WRITE;
                hClient->ioIntent |= DRV_IO_INTENT_NONBLOCKING;

                /* Remember which ak4384 driver instance owns me */
                hClient->inUse  = true;
                hClient->hDriver = drvObj;
                hClient->pEventCallBack = NULL;
                drvObj->numClients++;
                /* We have found a client object
                 * Release the mutex and return with
                 * the driver handle */
                /* An operation mode is needed */
                if((OSAL_MUTEX_Unlock(&(gDrvak4384CommonDataObj.mutexClientObjects))) != OSAL_RESULT_TRUE)
                {
                    SYS_DEBUG(0, "Unable to unlock open routine mutex \r\n");
                    return DRV_HANDLE_INVALID;
                }
                /* Return the client object */
                return (DRV_HANDLE) hClient;
            }
            hClient++;
        }
        /* Could not find a client object. Release the mutex and
         * return with an invalid handle. */
        if((OSAL_MUTEX_Unlock(&(gDrvak4384CommonDataObj.mutexClientObjects))) != OSAL_RESULT_TRUE)
        {
            SYS_DEBUG(0, "Unable to unlock open routine mutex \r\n");
        }
    }
    return DRV_HANDLE_INVALID;
}

// *****************************************************************************
/* Function:
    void DRV_AK4384_Close( DRV_Handle handle )

  Summary:
    Closes an opened-instance of the AK4384 driver

  Description:
    This routine closes an opened-instance of the AK4384 driver, invalidating the
    handle. Any buffers in the driver queue that were submitted by this client
    will be removed.  After calling this routine, the handle passed in "handle"
    must not be used with any of the remaining driver routines.  A new handle must
    be obtained by calling DRV_AK4384_Open before the caller may use the driver
    again

  Remarks:
    Usually there is no need for the driver client to verify that the Close
    operation has completed.  The driver will abort any ongoing operations
    when this routine is called.
*/
void DRV_AK4384_Close( const DRV_HANDLE handle)
{
    DRV_AK4384_CLIENT_OBJ *clientObj;
    DRV_AK4384_OBJ *drvObj;

    if(handle == DRV_HANDLE_INVALID || (DRV_HANDLE)NULL == handle)
    {
        SYS_DEBUG(0, "Invalid Driver Handle \r\n");
        return;
    }

    clientObj = (DRV_AK4384_CLIENT_OBJ *) handle;
    if (false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid Driver Handle \r\n");
        return;
    }

    drvObj = (DRV_AK4384_OBJ *) clientObj->hDriver;

    /* De-allocate the object */
    clientObj->inUse = false;
    /* Reduce the number of clients */
    drvObj->numClients--;
    return;
} /* DRV_AK4384_Close */


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_BufferQueueFlush(DRV_HANDLE handle)

  Summary:
    This function flushes off the buffers associated with the client object.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function flushes off the buffers associated with the client object and
    disables the DMA channel used for transmission.

  Remarks:
    None.
*/
void DRV_AK4384_BufferQueueFlush( const DRV_HANDLE handle)
{
    DRV_AK4384_CLIENT_OBJ *clientObj;
    DRV_AK4384_OBJ *drvObj;

    if((DRV_HANDLE_INVALID == handle) || (0 == handle))
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "Handle is invalid \r\n");
        return;
    }

    clientObj = (DRV_AK4384_CLIENT_OBJ *) handle;
    if(false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid driver handle \r\n");
        return;
    }
    drvObj = clientObj->hDriver;   
    
    DRV_I2S_BufferQueueFlush(drvObj->i2sDriverHandle);
}

// *****************************************************************************
/*
Function:
	void DRV_AK4384_BufferAddWrite
	(
		const DRV_HANDLE handle,
		DRV_AK4384_BUFFER_HANDLE *bufferHandle,
		void *buffer, size_t size
	)

  Summary:
    Schedule a non-blocking driver write operation.

  Description:
    This function schedules a non-blocking write operation. The function returns
    with a valid buffer handle in the bufferHandle argument if the write request
    was scheduled successfully. The function adds the request to the hardware
    instance transmit queue and returns immediately. While the request is in the
    queue, the application buffer is owned by the driver and should not be
    modified.  The function returns DRV_AK4384_BUFFER_HANDLE_INVALID
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the buffer size is 0.
    - if the queue is full or the queue depth is insufficient
    If the requesting client registered an event callback with the driver,
    the driver will issue a DRV_AK4384_BUFFER_EVENT_COMPLETE event if the buffer
    was processed successfully of DRV_AK4384_BUFFER_EVENT_ERROR event if the
    buffer was not processed successfully.

  Returns:
    The bufferHandle parameter will contain the return buffer handle. This will be
    DRV_AK4384_BUFFER_HANDLE_INVALID if the function was not successful.

   Remarks:
    This function is thread safe in a RTOS application. It can be called from
    within the AK4384 Driver Buffer Event Handler that is registered by this
    client. It should not be called in the event handler associated with another
    AK4384 driver instance. It should not otherwise be called directly in an ISR.

*/

void DRV_AK4384_BufferAddWrite
(
	const DRV_HANDLE handle,
	DRV_AK4384_BUFFER_HANDLE *bufferHandle,
	void *buffer, size_t size
)
{
    DRV_AK4384_CLIENT_OBJ *clientObj;
    DRV_AK4384_OBJ *drvObj;

    /* The Client and driver objects from the handle */
    clientObj = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj = (DRV_AK4384_OBJ *) clientObj->hDriver;

    /* We first check the arguments and initialize the
     * buffer handle */
    if(bufferHandle != NULL)
    {
        *bufferHandle = DRV_AK4384_BUFFER_HANDLE_INVALID;
    }

    /* See if the handle is still valid */
    if(false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid Driver Handle \r\n");
        return;
    }

    /* Grab a mutex. */
    if (OSAL_MUTEX_Lock(&(drvObj->mutexDriverInstance), OSAL_WAIT_FOREVER) == OSAL_RESULT_TRUE)
    {
        ;
    }
    else
    {
        /* The mutex acquisition timed out. Return with an
         * invalid handle. This code will not execute
         * if there is no RTOS. */
        return;
    }
    {
        DRV_I2S_BUFFER_HANDLE i2sBufferHandle = DRV_I2S_BUFFER_HANDLE_INVALID;
        DRV_I2S_BufferAddWrite( drvObj->i2sDriverHandle, &i2sBufferHandle,
                                (uint8_t *) buffer, size);

        if(i2sBufferHandle != DRV_I2S_BUFFER_HANDLE_INVALID)
        {
            *bufferHandle = (DRV_AK4384_BUFFER_HANDLE)i2sBufferHandle;
        }
        else
        {
            *bufferHandle = DRV_AK4384_BUFFER_HANDLE_INVALID;
        }
    }

    /* Release mutex */
    if((OSAL_MUTEX_Unlock(&(drvObj->mutexDriverInstance))) != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG(0, "Unable to DriverInstance mutex \r\n");
    }

    return;
} /* DRV_AK4384_BufferAddWrite */

// *****************************************************************************
/*
  Function:
    size_t DRV_AK4384_BufferProcessedSizeGet(DRV_HANDLE handle)

  Summary:
    This function returns number of bytes that have been processed for the
    specified buffer.

  Description:
    This function returns number of bytes that have been processed for the
    specified buffer. The client can use this function, in a case where the
    buffer has terminated due to an error, to obtain the number of bytes that
    have been processed.
    If this function is called on a invalid buffer handle, or if the buffer
    handle has expired, the function returns 0.

  Remarks:
    None.
*/
size_t DRV_AK4384_BufferProcessedSizeGet(DRV_HANDLE handle)
{
    DRV_AK4384_CLIENT_OBJ *clientObj;
    DRV_AK4384_OBJ *drvObj;
	
	if((DRV_HANDLE_INVALID == handle) || (0 == handle))
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "Handle is invalid \r\n");
        return 0;
    }

    /* The Client and driver objects from the handle */
    clientObj = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj = (DRV_AK4384_OBJ *) clientObj->hDriver;

    /* See if the handle is still valid */
    if(false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid Driver Handle \r\n");
        return 0;
    }

    return DRV_I2S_BufferProcessedSizeGet( drvObj->i2sDriverHandle );
}

// *****************************************************************************
/*
  Function:
    size_t DRV_AK4384_BufferCombinedQueueSizeGet(DRV_HANDLE handle)

  Summary:
    This function returns the number of bytes queued (to be processed) in the
    buffer queue.

  Description:
    This function returns the number of bytes queued (to be processed) in the
    buffer queue. The client can use this function to know number of bytes
    that is in the queue to be transmitted.

    If this function is called on a invalid client handle, or if the
    handle has expired, then the function returns 0.

  Remarks:
    None.
*/
size_t DRV_AK4384_BufferCombinedQueueSizeGet(DRV_HANDLE handle)
{
    DRV_AK4384_CLIENT_OBJ *clientObj;
    DRV_AK4384_OBJ *drvObj;

    if((DRV_HANDLE_INVALID == handle) || (0 == handle))
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "Handle is invalid \r\n");
        return 0;
    }

    clientObj = (DRV_AK4384_CLIENT_OBJ *) handle;
    if(false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid driver handle \r\n");
        return 0;
    }
    drvObj = clientObj->hDriver;

    return DRV_I2S_BufferCombinedQueueSizeGet(drvObj->i2sDriverHandle);
}

// *****************************************************************************
/*
  Function:
	void DRV_AK4384_BufferEventHandlerSet
	(
		DRV_HANDLE handle,
		const DRV_AK4384_BUFFER_EVENT_HANDLER eventHandler,
		const uintptr_t contextHandle
	)

  Summary:
    This function allows a client to identify a buffer event handling function
    for the driver to call back when queued buffer transfers have finished.

  Description:
    This function allows a client to identify a buffer event handling function
    for the driver to call back when queued buffer transfers have finished.
    When a client calls DRV_AK4384_BufferAddWrite function, it is provided with
    a handle identifying  the buffer that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling "eventHandler"
    function when the buffer transfer has completed.

    The event handler should be set before the client performs any "buffer add"
    operations that could generate events. The event handler once set, persists
    until the client closes the driver or sets another event handler (which
    could be a "NULL" pointer to indicate no callback).

  Remarks:
    If the client does not want to be notified when the queued buffer transfer
    has completed, it does not need to register a callback.
*/
void DRV_AK4384_BufferEventHandlerSet
(
	DRV_HANDLE handle,
	const DRV_AK4384_BUFFER_EVENT_HANDLER eventHandler,
	const uintptr_t contextHandle
)
{
    DRV_AK4384_CLIENT_OBJ *clientObj;
    DRV_AK4384_OBJ *drvObj;

    if((DRV_HANDLE_INVALID == handle) || (0 == handle))
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "Handle is invalid \r\n");
        return;
    }

    /* Assing the event handler and the context */
    clientObj = (DRV_AK4384_CLIENT_OBJ *) handle;
    if(false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid driver handle \r\n");
        return;
    }
    drvObj = clientObj->hDriver;
    /* Set the Event Handler and context */
    clientObj->pEventCallBack = eventHandler;
    clientObj->hClientArg = contextHandle;

    DRV_I2S_BufferEventHandlerSet(drvObj->i2sDriverHandle,
        (DRV_I2S_BUFFER_EVENT_HANDLER) _DRV_AK4384_I2SBufferEventHandler,
        (uintptr_t)(clientObj));

    return;
} /* DRV_AK4384_BufferEventHandlerSet */

// *****************************************************************************
/*
  Function:
    void DRV_AK4384_SetAudioCommunicationMode
(
    DRV_HANDLE handle, 
    const DATA_LENGTH dl, 
    const SAMPLE_LENGTH sl
)

  Summary:
    This function provides a run time audio format configuration

  Description:
    This function sets up audio mode in I2S protocol

  Precondition:
    The DRV_AK4384_Initialize routine must have been called for the specified
    AK4384 driver instance.

    DRV_AK4384_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine
    dl           - Data length for I2S audio interface
    sl           - Left/Right Sample Length for I2S audio interface
  Returns:
    None

  Remarks:
    None.
*/
void DRV_AK4384_SetAudioCommunicationMode
(
    DRV_HANDLE handle, 
    const DATA_LENGTH dl, 
    const SAMPLE_LENGTH sl)
{
    
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;

    if((DRV_HANDLE_INVALID == handle) || (0 == handle))
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "Handle is invalid \r\n");
        return;
    }
    
    clientObj = (DRV_AK4384_CLIENT_OBJ *) handle;
    
    if(false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid driver handle \r\n");
        return;
    }
    
    drvObj = (DRV_AK4384_OBJ *)clientObj->hDriver;

    // initialize with a mostly used one
    SPI_AUDIO_COMMUNICATION_WIDTH spi_audio_mode = SPI_AUDIO_COMMUNICATION_16DATA_16FIFO_32CHANNEL;
    if(sl == SAMPLE_LENGTH_32)
    {
        switch(dl)
        {
            case DATA_LENGTH_16:
                spi_audio_mode = SPI_AUDIO_COMMUNICATION_16DATA_16FIFO_32CHANNEL;
                break;
            case DATA_LENGTH_24:
                spi_audio_mode = SPI_AUDIO_COMMUNICATION_24DATA_32FIFO_32CHANNEL;
                break;
            case DATA_LENGTH_32:
                spi_audio_mode = SPI_AUDIO_COMMUNICATION_32DATA_32FIFO_32CHANNEL;
                break;
            default:
                 // should never reach this branch
                break;
        };
    }else
    {
        // no mater what dl is, the mode can only be
        spi_audio_mode = SPI_AUDIO_COMMUNICATION_16DATA_16FIFO_16CHANNEL;
    }
    
    DRV_I2S_SetAudioCommunicationMode(drvObj->i2sDriverHandle, spi_audio_mode);
}

// *****************************************************************************
// *****************************************************************************
// Section: AK4384 CODEC Specific Client Routines
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/*
  Function:
    void DRV_AK4384_SamplingRateSet(DRV_HANDLE handle, uint32_t samplingRate)

  Summary:
    This function sets the sampling rate of the media stream.

  Description:
    This function sets the media sampling rate for the client handle.

  Remarks:
    None.
*/
void DRV_AK4384_SamplingRateSet(DRV_HANDLE handle, uint32_t samplingRate)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj = (DRV_AK4384_OBJ *)clientObj->hDriver;
    drvObj->command = DRV_AK4384_COMMAND_SAMPLING_RATE_SET;
    
 /* If MCLK multiplier specified during initialization
       is not supported for the sampling rate range
       Normal Speed Mode 8kHz~48kHz, Program a suitable value */
    if((samplingRate <= DRV_AK4384_SAMPLERATE_48000HZ &&
        samplingRate >= DRV_AK4384_SAMPLERATE_8000HZ) 
        && 
       ( drvObj->mclk_multiplier != DRV_AK4384_MCLK_MULTIPLIER_256FS ||
         drvObj->mclk_multiplier != DRV_AK4384_MCLK_MULTIPLIER_384FS ||
         drvObj->mclk_multiplier != DRV_AK4384_MCLK_MULTIPLIER_512FS ||
         drvObj->mclk_multiplier != DRV_AK4384_MCLK_MULTIPLIER_768FS ||
         drvObj->mclk_multiplier != DRV_AK4384_MCLK_MULTIPLIER_1152FS))
    {
        drvObj->mclk_multiplier = DRV_AK4384_MCLK_MULTIPLIER_256FS;
        /* Bit Clock Divider is 256/64 = 4 */               
        drvObj->bclk_divider = 4;   
    }
    /* If MCLK multiplier specified during initialization
       is not supported for the sampling rate range
       Double Speed Mode 60kHz~96kHz, Program a suitable value */    
    else if((samplingRate <= DRV_AK4384_SAMPLERATE_96000HZ &&
             samplingRate >= DRV_AK4384_SAMPLERATE_60000HZ) 
        && 
       ( drvObj->mclk_multiplier != DRV_AK4384_MCLK_MULTIPLIER_128FS ||
         drvObj->mclk_multiplier != DRV_AK4384_MCLK_MULTIPLIER_192FS ||            
         drvObj->mclk_multiplier != DRV_AK4384_MCLK_MULTIPLIER_256FS ||
         drvObj->mclk_multiplier != DRV_AK4384_MCLK_MULTIPLIER_384FS))
    {           
        drvObj->mclk_multiplier = DRV_AK4384_MCLK_MULTIPLIER_128FS;
        /* Bit Clock Divider is 128/64 = 2 */               
        drvObj->bclk_divider = 2;        
    }
    /* If MCLK multiplier specified during initialization
       is not supported for the sampling rate range
       Quad Speed Mode 120kHz~192kHz, Program a suitable value */        
    else if((samplingRate <= DRV_AK4384_SAMPLERATE_192000HZ &&
             samplingRate >= DRV_AK4384_SAMPLERATE_120000HZ) 
        && 
       ( drvObj->mclk_multiplier != DRV_AK4384_MCLK_MULTIPLIER_128FS ||
         drvObj->mclk_multiplier != DRV_AK4384_MCLK_MULTIPLIER_192FS))
    {
        drvObj->mclk_multiplier = DRV_AK4384_MCLK_MULTIPLIER_128FS;        
        /* Bit Clock Divider is 128/64 = 2 */               
        drvObj->bclk_divider = 2;        
    }
    else
    {
        ; 
    }    
            
    _DRV_AK4384_MasterClockSet(samplingRate, drvObj->mclk_multiplier);
    if( samplingRate >= DRV_AK4384_SAMPLERATE_8000HZ &&
        samplingRate <= DRV_AK4384_SAMPLERATE_48000HZ)
    {
        regValue = DRV_AK4384_CONTROL_REG_FIELD_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                        DRV_AK4384_CTRL2_DFS_MASK,
                        DRV_AK4384_CTRL2_DFS_NORM_POS,
                        DRV_AK4384_CTRL2_DFS_NORM_VAL);
        _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_2,regValue);
    }
    else if(samplingRate >= DRV_AK4384_SAMPLERATE_60000HZ &&
            samplingRate <= DRV_AK4384_SAMPLERATE_96000HZ)
    {
        regValue = DRV_AK4384_CONTROL_REG_FIELD_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                        DRV_AK4384_CTRL2_DFS_MASK,
                        DRV_AK4384_CTRL2_DFS_DOUBLE_POS,
                        DRV_AK4384_CTRL2_DFS_DOUBLE_VAL);
        _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_2,regValue);
    }
    else if(samplingRate >= DRV_AK4384_SAMPLERATE_120000HZ &&
            samplingRate <= DRV_AK4384_SAMPLERATE_192000HZ)
    {
        regValue = DRV_AK4384_CONTROL_REG_FIELD_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                        DRV_AK4384_CTRL2_DFS_MASK,
                        DRV_AK4384_CTRL2_DFS_QUAD_POS,
                        DRV_AK4384_CTRL2_DFS_QUAD_VAL);
        _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_2,regValue);
    }
    else
    {
        ;
    }

    DRV_I2S_BaudSet(drvObj->i2sDriverHandle, (samplingRate*drvObj->bclk_divider), samplingRate);
    
    drvObj->samplingRate = samplingRate;
    return;
}


// *****************************************************************************
/*
  Function:
    uint32_t DRV_AK4384_SamplingRateGet(DRV_HANDLE handle)

  Summary:
    This function gets the sampling rate set on the DAC AK4384

  Description:
    This function gets the sampling rate set on the DAC AK4384.

  Remarks:
    None.
 */
uint32_t DRV_AK4384_SamplingRateGet(DRV_HANDLE handle)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;

    clientObj = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj = (DRV_AK4384_OBJ *)clientObj->hDriver;

    /* Return the sampling rate */
    return drvObj->samplingRate;
}

// *****************************************************************************
/*
  Function:
    void DRV_AK4384_VolumeSet(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan, uint8_t volume)

  Summary:
    This function sets the volume for AK4384 CODEC.

  Description:
    This functions sets the volume value from 0-255 which can attenuate
    from 0dB to –48dB and mute

  Remarks:
    None.
*/
void DRV_AK4384_VolumeSet(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan, uint8_t volume)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;
    clientObj = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj = (DRV_AK4384_OBJ *)clientObj->hDriver;
    
    if(DRV_AK4384_CHANNEL_LEFT == chan)
    {
        drvObj->volume[DRV_AK4384_CHANNEL_LEFT] = volume;
        _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_LATT,volume);
    }
    else if(DRV_AK4384_CHANNEL_RIGHT == chan)
    {
        drvObj->volume[DRV_AK4384_CHANNEL_RIGHT] = volume;
        _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_RATT,volume);
    }
    else
    {
        // if there was no command processing
        if(drvObj->command == DRV_AK4384_COMMAND_NONE)
        {
            drvObj->volume[DRV_AK4384_CHANNEL_LEFT] = volume;
            drvObj->volume[DRV_AK4384_CHANNEL_RIGHT] = volume;
            drvObj->volume[DRV_AK4384_CHANNEL_LEFT_RIGHT] = volume;
            _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_LATT,volume);
            drvObj->command = DRV_AK4384_COMMAND_VOLUME_SET_CHANNEL_RIGHT;
        }
    }
    return;
}


// *****************************************************************************
/*
  Function:
    uint8_t DRV_AK4384_VolumeGet(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan)

  Summary:
    This function gets the volume for AK4384 CODEC.

  Description:
    This functions gets the current volume programmed to the DAC AK4384.

  Remarks:
    None.
 */
uint8_t DRV_AK4384_VolumeGet(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;

    clientObj = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj = (DRV_AK4384_OBJ *)clientObj->hDriver;

    /* Return the volume */
    return drvObj->volume[chan];
}

// *****************************************************************************
/*
  Function:
    void DRV_AK4384_MuteOn(DRV_HANDLE handle);

  Summary:
    This function allows AK4384 output for soft mute on.

  Description:
    This function Enables AK4384 output for soft mute.

  Remarks:
    None.
*/
void DRV_AK4384_MuteOn(DRV_HANDLE handle)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj = (DRV_AK4384_OBJ *)clientObj->hDriver;
    drvObj->command = DRV_AK4384_COMMAND_MUTE_ON;

    regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
               DRV_AK4384_CTRL2_SMUTE_MUTE_POS, DRV_AK4384_CTRL2_SMUTE_MUTE_VAL);
    _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_2,regValue);
    return;
}


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_MuteOff(DRV_HANDLE handle)

  Summary:
    This function disables AK4384 output for soft mute.

  Description:
    This function disables AK4384 output for soft mute.

  Remarks:
    None.
*/
void DRV_AK4384_MuteOff(DRV_HANDLE handle)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj       = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj          = (DRV_AK4384_OBJ *)clientObj->hDriver;
    drvObj->command = DRV_AK4384_COMMAND_MUTE_OFF;

    regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                    drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                    DRV_AK4384_CTRL2_SMUTE_NORM_POS,
                    DRV_AK4384_CTRL2_SMUTE_NORM_VAL);
    _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_2,regValue);
    return;
}


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_ZeroDetectEnable(DRV_HANDLE handle)

  Summary:
    This function enables AK4384 channel-independent zeros detect function.

  Description:
    This function enables AK4384 channel-independent zeros detect function.

  Remarks:
    None.
*/
void DRV_AK4384_ZeroDetectEnable(DRV_HANDLE handle)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj       = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj          = (DRV_AK4384_OBJ *)clientObj->hDriver;
    drvObj->command = DRV_AK4384_COMMAND_ZERO_DETECT_ENABLE;

    regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                    drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                    DRV_AK4384_CTRL2_DZFE_ENABLE_POS,
                    DRV_AK4384_CTRL2_DZFE_ENABLE_VAL);
    _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_2,regValue);
    return;
}


// *****************************************************************************
/*
  Function:
	void DRV_AK4384_ZeroDetectDisable(DRV_HANDLE handle)

  Summary:
    This function disables AK4384 channel-independent zeros detect function.

  Description:
	This function disables AK4384 channel-independent zeros detect function.

  Remarks:
    None.
*/
void DRV_AK4384_ZeroDetectDisable(DRV_HANDLE handle)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj       = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj          = (DRV_AK4384_OBJ *)clientObj->hDriver;
    drvObj->command = DRV_AK4384_COMMAND_ZERO_DETECT_DISABLE;

    regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                    drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                    DRV_AK4384_CTRL2_DZFE_DISABLE_POS,
                    DRV_AK4384_CTRL2_DZFE_DISABLE_VAL);
    _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_2,regValue);
    return;
}


// *****************************************************************************
/*
  Function:
	void DRV_AK4384_ZeroDetectModeSet
	(
		DRV_HANDLE handle,
		DRV_AK4384_ZERO_DETECT_MODE zdMode
	)

  Summary:
    This function sets mode of AK4384 channel-independent zeros detect function

  Description:
    This function sets mode of AK4384 channel-independent zeros detect function

  Remarks:
    None.
*/
void DRV_AK4384_ZeroDetectModeSet
(
	DRV_HANDLE handle,
	DRV_AK4384_ZERO_DETECT_MODE zdMode
)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj       = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj          = (DRV_AK4384_OBJ *)clientObj->hDriver;
    drvObj->command = DRV_AK4384_COMMAND_ZERO_DETECT_MODE_SET;
    regValue        = 0;
    if(DRV_AK4384_ZERO_DETECT_MODE_CHANNEL_SEPARATED == zdMode)
    {
        regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                        DRV_AK4384_CTRL2_DZFM_SEPA_POS,
                        DRV_AK4384_CTRL2_DZFM_SEPA_VAL);
    }
    else if(DRV_AK4384_ZERO_DETECT_MODE_ANDED == zdMode)
    {
        regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                        DRV_AK4384_CTRL2_DZFM_ANDED_POS,
                        DRV_AK4384_CTRL2_DZFM_ANDED_VAL);
    }
    else
    {
        ;
    }
    regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                    regValue, DRV_AK4384_CTRL2_DZFE_ENABLE_POS,
                    DRV_AK4384_CTRL2_DZFE_ENABLE_VAL);
    _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_2,regValue);
    return;
}


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_ZeroDetectInvertEnable(DRV_HANDLE handle)

  Summary:
    This function enables inversion of polarity for zero detect function.

  Description:
    This function enables inversion of polarity for zero detect function.
	DZF goes “L” at Zero Detection

  Remarks:
    None.
*/
void DRV_AK4384_ZeroDetectInvertEnable(DRV_HANDLE handle)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj       = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj          = (DRV_AK4384_OBJ *)clientObj->hDriver;
    drvObj->command = DRV_AK4384_COMMAND_ZERO_DETECT_INVERT_ENABLE;

    regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                    drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_3],
                    DRV_AK4384_CTRL3_DZFB_LOW_POS,
                    DRV_AK4384_CTRL3_DZFB_LOW_VAL);
    _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_3,regValue);
    return;
}

// *****************************************************************************
/*
  Function:
    void DRV_AK4384_ZeroDetectInvertDisable(DRV_HANDLE handle)

  Summary:
    This function disables inversion of polarity for zero detect function.

  Description:
    This function disables inversion of polarity for zero detect function.
	DZF goes “H” at Zero Detection.

  Remarks:
    None.
*/
void DRV_AK4384_ZeroDetectInvertDisable(DRV_HANDLE handle)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj       = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj          = (DRV_AK4384_OBJ *)clientObj->hDriver;
    drvObj->command = DRV_AK4384_COMMAND_ZERO_DETECT_INVERT_DISABLE;

    regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                    drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_3],
                    DRV_AK4384_CTRL3_DZFB_HIGH_POS,
                    DRV_AK4384_CTRL3_DZFB_HIGH_VAL);
    _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_3,regValue);
    return;
}


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_ChannelOutputInvertEnable(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan)

  Summary:
    This function enables output polarity of the selected Channel.

  Description:
    This function enables output polarity of the selected Channel.

  Returns:
    None

  Remarks:
    None.
*/
void DRV_AK4384_ChannelOutputInvertEnable(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj       = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj          = (DRV_AK4384_OBJ *)clientObj->hDriver;

    if(chan == DRV_AK4384_CHANNEL_LEFT)
    {
        drvObj->command = DRV_AK4384_COMMAND_LEFT_CHANNEL_ONLY_INVERT_ENABLE;
        regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_3],
                        DRV_AK4384_CTRL3_INVL_INV_POS,
                        DRV_AK4384_CTRL3_INVL_INV_VAL);
    }
    else if(chan == DRV_AK4384_CHANNEL_RIGHT)
    {
        drvObj->command = DRV_AK4384_COMMAND_RIGHT_CHANNEL_ONLY_INVERT_ENABLE;
        regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_3],
                        DRV_AK4384_CTRL3_INVR_INV_POS,
                        DRV_AK4384_CTRL3_INVR_INV_VAL);
    }
    else
    {
        drvObj->command = DRV_AK4384_COMMAND_LEFT_CHANNEL_INVERT_ENABLE;
        regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_3],
                        DRV_AK4384_CTRL3_INVL_INV_POS,
                        DRV_AK4384_CTRL3_INVL_INV_VAL);
    }
    _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_3,regValue);
    return;
}


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_ChannelOutputInvertDisable(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan)

  Summary:
    This function disables output polarity of the selected Channel.

  Description:
    This function disables output polarity of the selected Channel.

  Remarks:
    None.
*/
void DRV_AK4384_ChannelOutputInvertDisable(DRV_HANDLE handle, DRV_AK4384_CHANNEL chan)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj       = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj          = (DRV_AK4384_OBJ *)clientObj->hDriver;

    if(chan == DRV_AK4384_CHANNEL_LEFT)
    {
        drvObj->command = DRV_AK4384_COMMAND_LEFT_CHANNEL_ONLY_INVERT_DISABLE;
        regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_3],
                        DRV_AK4384_CTRL3_INVL_NORM_POS,
                        DRV_AK4384_CTRL3_INVL_NORM_VAL);
    }
    else if(chan == DRV_AK4384_CHANNEL_RIGHT)
    {
        drvObj->command = DRV_AK4384_COMMAND_RIGHT_CHANNEL_ONLY_INVERT_DISABLE;
        regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_3],
                        DRV_AK4384_CTRL3_INVR_NORM_POS,
                        DRV_AK4384_CTRL3_INVR_NORM_VAL);
    }

    else
    {
        drvObj->command = DRV_AK4384_COMMAND_LEFT_CHANNEL_INVERT_DISABLE;
        regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_3],
                        DRV_AK4384_CTRL3_INVL_NORM_POS,
                        DRV_AK4384_CTRL3_INVL_NORM_VAL);
    }
    _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_3,regValue);
    return;
}

// *****************************************************************************
/*
  Function:
    void DRV_AK4384_SlowRollOffFilterEnable(DRV_HANDLE handle)

  Summary:
    This function enables Slow Roll-off filter function.

  Description:
    This function enables Slow Roll-off filter function.

  Remarks:
    None.
*/
void DRV_AK4384_SlowRollOffFilterEnable(DRV_HANDLE handle)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj       = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj          = (DRV_AK4384_OBJ *)clientObj->hDriver;
    drvObj->command = DRV_AK4384_COMMAND_SLOW_ROLL_OFF_FILTER_ENABLE;

    regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                    drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                    DRV_AK4384_CTRL2_SLOW_SLOW_POS,
                    DRV_AK4384_CTRL2_SLOW_SLOW_VAL);
    _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_2,regValue);
    return;
}

// *****************************************************************************
/*
  Function:
    void DRV_AK4384_SlowRollOffFilterDisable(DRV_HANDLE handle)

  Summary:
    This function disables Slow Roll-off filter function.

  Description:
    This function disables Slow Roll-off filter function. Sharp Roll-off filter
    function gets enabled.

  Remarks:
    None.
*/
void DRV_AK4384_SlowRollOffFilterDisable(DRV_HANDLE handle)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj       = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj          = (DRV_AK4384_OBJ *)clientObj->hDriver;
    drvObj->command = DRV_AK4384_COMMAND_SLOW_ROLL_OFF_FILTER_DISABLE;

    regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                    drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                    DRV_AK4384_CTRL2_SLOW_SHARP_POS,
                    DRV_AK4384_CTRL2_SLOW_SHARP_VAL);
    _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_2,regValue);
    return;
}


// *****************************************************************************
/*
  Function:
    void DRV_AK4384_DeEmphasisFilterSet
    (
        DRV_HANDLE handle,
        DRV_AK4384_DEEMPHASIS_FILTER filter
    )

  Summary:
    This function allows specifies enabling of digital de-emphasis filter.

  Description:
    This function allows specifies enabling of digital de-emphasis for 32, 44.1 or
    48kHz sampling rates (tc = 50/15µs)

  Remarks:
    None.
*/
void DRV_AK4384_DeEmphasisFilterSet
(
    DRV_HANDLE handle,
    DRV_AK4384_DEEMPHASIS_FILTER filter
)
{
    DRV_AK4384_OBJ *drvObj;
    DRV_AK4384_CLIENT_OBJ *clientObj;
    uint8_t regValue;

    clientObj       = (DRV_AK4384_CLIENT_OBJ *) handle;
    drvObj          = (DRV_AK4384_OBJ *)clientObj->hDriver;
    drvObj->command = DRV_AK4384_COMMAND_DEEMPHASIS_FILTER_SET;
    regValue        = 0;
    if(DRV_AK4384_DEEMPHASIS_FILTER_44_1KHZ == filter)
    {
        regValue = DRV_AK4384_CONTROL_REG_FIELD_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                        DRV_AK4384_CTRL2_DEM_MASK,
                        DRV_AK4384_CTRL2_DEM_44_1KHZ_POS,
                        DRV_AK4384_CTRL2_DEM_44_1KHZ_VAL);
    }
    else if(DRV_AK4384_DEEMPHASIS_FILTER_OFF == filter)
    {
        regValue = DRV_AK4384_CONTROL_REG_FIELD_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                        DRV_AK4384_CTRL2_DEM_MASK,
                        DRV_AK4384_CTRL2_DEM_OFF_POS,
                        DRV_AK4384_CTRL2_DEM_OFF_VAL);
    }
    else if(DRV_AK4384_DEEMPHASIS_FILTER_48KHZ == filter)
    {
        regValue = DRV_AK4384_CONTROL_REG_FIELD_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                        DRV_AK4384_CTRL2_DEM_MASK,
                        DRV_AK4384_CTRL2_DEM_48KHZ_POS,
                        DRV_AK4384_CTRL2_DEM_48KHZ_VAL);
    }
    else if(DRV_AK4384_DEEMPHASIS_FILTER_32KHZ == filter)
    {
        regValue = DRV_AK4384_CONTROL_REG_FIELD_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                        DRV_AK4384_CTRL2_DEM_MASK,
                        DRV_AK4384_CTRL2_DEM_32KHZ_POS,
                        DRV_AK4384_CTRL2_DEM_32KHZ_VAL);
    }
    else
    {
        ;
    }
    _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_2,regValue);
    return;
}


// *****************************************************************************
/*
  Function:
	void DRV_AK4384_CommandEventHandlerSet
	(
		DRV_HANDLE handle,
		const DRV_AK4384_COMMAND_EVENT_HANDLER eventHandler,
		const uintptr_t contextHandle
	)

  Summary:
    This function allows a client to identify a command event handling function
    for the driver to call back when the last submitted command have finished.

  Description:
    This function allows a client to identify a command event handling function
    for the driver to call back when the last submitted command have finished.

    When a client calls DRV_AK4384_BufferAddWrite function, it is provided with
    a handle identifying  the buffer that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling "eventHandler"
    function when the buffer transfer has completed.

    The event handler should be set before the client performs any "AK4384 CODEC
    Specific Client Routines" operations that could generate events.
    The event handler once set, persists until the client closes the driver or
    sets another event handler (which could be a "NULL" pointer to indicate no callback).

  Remarks:
    If the client does not want to be notified when the command
    has completed, it does not need to register a callback.
*/
void DRV_AK4384_CommandEventHandlerSet
(
	DRV_HANDLE handle,
	const DRV_AK4384_COMMAND_EVENT_HANDLER eventHandler,
	const uintptr_t contextHandle
)
{
    DRV_AK4384_CLIENT_OBJ *clientObj;
    DRV_AK4384_OBJ *drvObj;

    if((DRV_HANDLE_INVALID == handle) || (0 == handle))
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "Handle is invalid \r\n");
        return;
    }

    clientObj = (DRV_AK4384_CLIENT_OBJ *) handle;
    /* Assing the event handler and the context */
    if(false == clientObj->inUse)
    {
        SYS_DEBUG(0, "Invalid driver handle \r\n");
        return;
    }

    drvObj = (DRV_AK4384_OBJ *)clientObj->hDriver;
    drvObj->commandCompleteCallback = eventHandler;
    drvObj->commandContextData = contextHandle;
    return;
}


// *****************************************************************************
// *****************************************************************************
// Section: AK4384 CODEC Version Information Routines
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/*
  Function:
    int8_t* DRV_AK4384_VersionStrGet(void)

  Summary:
    This function returns the version of AK4384 driver in string format.

  Description:
    The DRV_AK4384_VersionStrGet function returns a string in the format:
    "<major>.<minor>[.<patch>][<type>]"
    Where:
        <major> is the AK4384 driver's version number.
        <minor> is the AK4384 driver's version number.
        <patch> is an optional "patch" or "dot" release number (which is not
        included in the string if it equals "00").
        <type> is an optional release type ("a" for alpha, "b" for beta ?
        not the entire word spelled out) that is not included if the release
        is a production version (I.e. Not an alpha or beta).

        The String does not contain any spaces.

        Example:
        "0.03a"
        "1.00"

  Remarks:
    None
 */
int8_t* DRV_AK4384_VersionStrGet(void)
{
    return (int8_t*) _DRV_AK4384_VERSION_STR;
}


// *****************************************************************************
/*
  Function:
    uint32_t DRV_AK4384_VersionGet( void )

  Summary:
    This function returns the version of AK4384 driver

  Description:
    The version number returned from the DRV_AK4384_VersionGet function is an
    unsigned integer in the following decimal format.
    <major> * 10000 + <minor> * 100 + <patch>

    Where the numbers are represented in decimal and the meaning is the same as
    above.  Note that there is no numerical representation of release type.

    Example:
    For version "0.03a", return:  0 * 10000 + 3 * 100 + 0
    For version "1.00", return:  1 * 100000 + 0 * 100 + 0

  Remarks:
    None
 */
uint32_t DRV_AK4384_VersionGet(void)
{
    return (_DRV_AK4384_VERSION_MAJOR * 10000 +  \
            _DRV_AK4384_VERSION_MINOR * 100 + \
            _DRV_AK4384_VERSION_PATCH);
}

// *****************************************************************************
// *****************************************************************************
// Section: File scope functions
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
 /*
  Function:
    static void _DRV_AK4384_MasterClockSet(uint32_t samplingRate, uint16_t mclk_multiplier)

  Summary:
    Generates the master clock(to AK4384) from REFCLOCK  for the given
    sampling rate.

  Description:
    Generates the master clock(to AK4384) from REFCLOCK  for the given
    sampling rate.

  Remarks:
    None
*/
static void _DRV_AK4384_MasterClockSet(uint32_t samplingRate, uint16_t mclk_multiplier)
{
    uint32_t mclkInHertz, achievedFrequencyHz;

    mclkInHertz = mclk_multiplier*samplingRate;
    achievedFrequencyHz = SYS_CLK_ReferenceFrequencySet(
                                CLK_BUS_REFERENCE_1,
                                DRV_AK4384_INPUT_REFCLOCK,
                                mclkInHertz, true );
    if (achievedFrequencyHz == 0)
    {
        SYS_DEBUG(0, "Frequency not set properly. check what is the problem \r\n");
    }
    return;

}


// *****************************************************************************
 /*
  Function:
        static void _DRV_AK4384_ConrolRegisterSet
        (
            DRV_AK4384_OBJ *drvObj,
            DRV_AK4384_CONTROL_REGISTER contRegister,
            uint8_t value
        )

  Summary:
    Prepares the control command to be sent to AK4384. Also starts the
    timer to initiate the control command transfer.

  Description:
    Prepares the control command to be sent to AK4384. Also starts the
    timer to initiate the control command transfer.

  Remarks:
    None
*/
static void _DRV_AK4384_ConrolRegisterSet
(
    DRV_AK4384_OBJ *drvObj,
    DRV_AK4384_CONTROL_REGISTER contRegister,
    uint8_t value
)
{
    uint32_t controlCommand;

    controlCommand = (  DRV_AK4384_CHIP_ADDRESS |
                        DRV_AK4384_CONTROL_WRITE |
                        DRV_AK4384_REG_ADDRESS(contRegister) |
                        DRV_AK4384_REG_DATA(value) );
    drvObj->controlCommand = controlCommand;
    drvObj->controlCommandStatus = true;
    drvObj->countBit = 0;

    /* Clear Control Chip Select, Clear Control Clock, Clear Control Data */
    BSP_AK4384_CONTROL_CSOff();
    BSP_AK4384_CONTROL_CLKOff();
    BSP_AK4384_CONTROL_DOOff();

    
    if(true == DRV_AK4384_COMMAND_SHIFT_BIT(0))
    {
        BSP_AK4384_CONTROL_DOOn();
    }
    else
    {
        BSP_AK4384_CONTROL_DOOff();
    }

    /* Start Timer */
    DRV_TMR_Start (drvObj->tmrDriverHandle);
    return;
}


// *****************************************************************************
 /*
  Function:
    static void _DRV_AK4384_ControlTasks(DRV_AK4384_OBJ *drvObj)

  Summary:
    Implements the state maching for the Audio control interface of AK4384

  Description:
    Implements the state maching for the Audio control interface of AK4384

  Remarks:
    None
*/
static void _DRV_AK4384_ControlTasks(DRV_AK4384_OBJ *drvObj)
{
    volatile uint8_t regValue;

    switch (drvObj->command)
    {
        case DRV_AK4384_COMMAND_NONE:
        {   
            /* Do nothing. No Control Command executed */
            ;
        }
        break;

        case DRV_AK4384_COMMAND_INIT_OPEN_TIMER:
        {
            /* Open the bit bang timer driver */
            drvObj->tmrDriverHandle = DRV_TMR_Open (drvObj->tmrDriverModuleIndex,
                                            DRV_IO_INTENT_EXCLUSIVE);
            if( DRV_HANDLE_INVALID == drvObj->tmrDriverHandle ||
            (DRV_HANDLE) NULL == drvObj->tmrDriverHandle)
            {
                // Do nothing. Iterate till the timer is initialized and
                // successfully opened
                ;
            }
            else
            {
                //DRV_TMR_Alarm16BitRegister( drvObj->tmrDriverHandle, //Deprecated
                DRV_TMR_AlarmRegister( drvObj->tmrDriverHandle,
                                    DRV_AK4384_TIMER_PERIOD, true,
                                    (uintptr_t)drvObj,
                                    (DRV_TMR_CALLBACK)&_DRV_AK4384_TimerCallbackHandler);
			
				// if delayed initialization is enabled, just go into a wait state
				// until application calls EnableInitialization routine
                 drvObj->command = (drvObj->delayDriverInitialization) ? DRV_AK4384_COMMAND_NONE : DRV_AK4384_COMMAND_INIT_CLK_PDN_SET;
            }
        }
        break;

        case DRV_AK4384_COMMAND_INIT_CLK_PDN_SET:
        {
            /* Generate master clocks from REFCLOCK for the given sampling rate */
            _DRV_AK4384_MasterClockSet(drvObj->samplingRate, drvObj->mclk_multiplier);

            /* Reset Values for Control inteface  and power down pins */
            BSP_AK4384_CONTROL_CSOn();
            BSP_AK4384_CONTROL_CLKOn();
            BSP_AK4384_CONTROL_DOOff();

            /* If the delayDriverInitialization option is enabled, we will skip
               toggling the chip's reset pin, since we assume it is tied to a
               common reset line shared with other peripherals such as a
               Bluetooth module, that has already been toggled (which would
               have earlier reset the codec). */

            if (drvObj->delayDriverInitialization==false)
            {
               /* Reset Values for Control interface  and power down pins */
               BSP_AK4384_PDNOff();
		    }
			BSP_AK4201_AMPLIFIER_PDNOff();
            if (drvObj->delayDriverInitialization==false)
            {
               /* Bring power down out of reset */
               BSP_AK4384_PDNOn();
            }
            BSP_AK4201_AMPLIFIER_PDNOn();

            /* Initiate AK4384 Command */
            drvObj->command = DRV_AK4384_COMMAND_INIT_START;
            regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                            drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_1],
                            DRV_AK4384_CTRL1_RSTN_RST_POS,
                            DRV_AK4384_CTRL1_RSTN_RST_VAL);
            regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                            regValue, DRV_AK4384_CTRL1_PW_PDN_POS,
                            DRV_AK4384_CTRL1_PW_PDN_VAL);
            regValue = DRV_AK4384_CONTROL_REG_FIELD_WRITE(
                            regValue, DRV_AK4384_CTRL1_DIF_MASK,
                            DRV_AK4384_CTRL1_DIF_POS,
                            drvObj->audioDataFormat);
            regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                            regValue, DRV_AK4384_CTRL1_ACKS_MANUAL_POS,
                            drvObj->mclkMode);
            _DRV_AK4384_ConrolRegisterSet(drvObj, DRV_AK4384_CONTROL_REGISTER_1,regValue);
        }
        break;

        case DRV_AK4384_COMMAND_INIT_START:
        {
            if (false == drvObj->controlCommandStatus)
            {
                drvObj->command = DRV_AK4384_COMMAND_INIT_SAMPLING_RATE;
                if (drvObj->samplingRate >= DRV_AK4384_SAMPLERATE_8000HZ &&
                    drvObj->samplingRate <= DRV_AK4384_SAMPLERATE_48000HZ)
                {
                    regValue = DRV_AK4384_CONTROL_REG_FIELD_WRITE(
                                    drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                                    DRV_AK4384_CTRL2_DFS_MASK,
                                    DRV_AK4384_CTRL2_DFS_NORM_POS,
                                    DRV_AK4384_CTRL2_DFS_NORM_VAL);
                    regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                                    regValue, DRV_AK4384_CTRL2_DZFM_ANDED_POS,
                                    DRV_AK4384_CTRL2_DZFM_ANDED_VAL);
                    regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                                    regValue, DRV_AK4384_CTRL2_DZFE_ENABLE_POS,
                                    DRV_AK4384_CTRL2_DZFE_ENABLE_VAL);
                    _DRV_AK4384_ConrolRegisterSet(drvObj, DRV_AK4384_CONTROL_REGISTER_2, regValue);
                }
                else if (drvObj->samplingRate >= DRV_AK4384_SAMPLERATE_60000HZ &&
                         drvObj->samplingRate <= DRV_AK4384_SAMPLERATE_96000HZ)
                {
                    regValue = DRV_AK4384_CONTROL_REG_FIELD_WRITE(
                                    drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                                    DRV_AK4384_CTRL2_DFS_MASK,
                                    DRV_AK4384_CTRL2_DFS_DOUBLE_POS,
                                    DRV_AK4384_CTRL2_DFS_DOUBLE_VAL);
                    regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                                    regValue, DRV_AK4384_CTRL2_DZFM_ANDED_POS,
                                    DRV_AK4384_CTRL2_DZFM_ANDED_VAL);
                    regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                                    regValue, DRV_AK4384_CTRL2_DZFE_ENABLE_POS,
                                    DRV_AK4384_CTRL2_DZFE_ENABLE_VAL);
                    _DRV_AK4384_ConrolRegisterSet(drvObj, DRV_AK4384_CONTROL_REGISTER_2, regValue);
                }
                else if (drvObj->samplingRate >= DRV_AK4384_SAMPLERATE_120000HZ &&
                         drvObj->samplingRate <= DRV_AK4384_SAMPLERATE_192000HZ)
                {
                    regValue = DRV_AK4384_CONTROL_REG_FIELD_WRITE(
                                    drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_2],
                                    DRV_AK4384_CTRL2_DFS_MASK,
                                    DRV_AK4384_CTRL2_DFS_QUAD_POS,
                                    DRV_AK4384_CTRL2_DFS_QUAD_VAL);
                    regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                                    regValue, DRV_AK4384_CTRL2_DZFM_ANDED_POS,
                                    DRV_AK4384_CTRL2_DZFM_ANDED_VAL);
                    regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                                    regValue, DRV_AK4384_CTRL2_DZFE_ENABLE_POS,
                                    DRV_AK4384_CTRL2_DZFE_ENABLE_VAL);
                    _DRV_AK4384_ConrolRegisterSet(drvObj, DRV_AK4384_CONTROL_REGISTER_2, regValue);
                }
            }
            else
            {
                /* Do Nothing. Remain in this state untill
                 * the INIT_START command is transferred successfully */
                ;
            }

        }
        break;

        case DRV_AK4384_COMMAND_INIT_SAMPLING_RATE:
        {
            if (false == drvObj->controlCommandStatus)
            {
                /* Open the i2s driver Handle */
                drvObj->i2sDriverHandle = DRV_I2S_Open(drvObj->i2sDriverModuleIndex,
                        (DRV_IO_INTENT_WRITE | DRV_IO_INTENT_NONBLOCKING));
                if (drvObj->i2sDriverHandle != DRV_HANDLE_INVALID)
                {
                    DRV_I2S_TransmitErrorIgnore(drvObj->i2sDriverHandle, true);
                    DRV_I2S_ReceiveErrorIgnore(drvObj->i2sDriverHandle, true);
                    DRV_I2S_BaudSet(drvObj->i2sDriverHandle,
                            (drvObj->samplingRate * drvObj->bclk_divider),
                            drvObj->samplingRate);
                }
                else
                {
                    SYS_DEBUG(0, "i2s DRV_I2S_Open Error");
                }
                drvObj->command = DRV_AK4384_COMMAND_VOLUME_SET_CHANNEL_LEFT_INIT;
                
            }
            else
            {
                /* Do Nothing. Remain in this state untill
                 * the INIT_SAMPLING_RATE command is transferred successfully */
                ;
            }
        }
        break;
        
        case DRV_AK4384_COMMAND_VOLUME_SET_CHANNEL_LEFT_INIT:
        {
            if(false == drvObj->controlCommandStatus)
            {
                drvObj->command = DRV_AK4384_COMMAND_VOLUME_SET_CHANNEL_RIGHT_INIT;
                _DRV_AK4384_ConrolRegisterSet(drvObj,
                        DRV_AK4384_CONTROL_REGISTER_LATT,
                        drvObj->volume[DRV_AK4384_CHANNEL_LEFT]);
            }else
            {
                /* Do Nothing. Remain in this state until
                 * the INIT_END command is transferred successfully */
                ;
            }
        }
        break;
        case DRV_AK4384_COMMAND_VOLUME_SET_CHANNEL_RIGHT_INIT:
        {
            if (false == drvObj->controlCommandStatus)
            {
                drvObj->command = DRV_AK4384_COMMAND_SET_CONTROL1_INIT;
                _DRV_AK4384_ConrolRegisterSet(drvObj,
                        DRV_AK4384_CONTROL_REGISTER_RATT,
                        drvObj->volume[DRV_AK4384_CHANNEL_RIGHT]);
            }
            else
            {
                /* Do Nothing. Remain in this state untill
                 * the INIT_END command is transferred successfully */
                ;
            }
        }
        break;
        case DRV_AK4384_COMMAND_SET_CONTROL1_INIT:
        {
            if (false == drvObj->controlCommandStatus)
            {
                drvObj->command = DRV_AK4384_COMMAND_INIT_END;
                regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                                drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_1],
                                DRV_AK4384_CTRL1_RSTN_NORM_POS,
                                DRV_AK4384_CTRL1_RSTN_NORM_VAL);
                regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                                regValue, DRV_AK4384_CTRL1_PW_NORM_POS,
                                DRV_AK4384_CTRL1_PW_NORM_VAL);
                regValue = DRV_AK4384_CONTROL_REG_FIELD_WRITE(
                                regValue, DRV_AK4384_CTRL1_DIF_MASK,
                                DRV_AK4384_CTRL1_DIF_POS,
                                drvObj->audioDataFormat);
                regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                                regValue, DRV_AK4384_CTRL1_ACKS_MANUAL_POS,
                                drvObj->mclkMode);
                _DRV_AK4384_ConrolRegisterSet(drvObj, DRV_AK4384_CONTROL_REGISTER_1, regValue);
            }
            else
            {
                /* Do Nothing. Remain in this state untill
                 * the command is transferred successfully */
                ;
            }
        }
        break;
        case DRV_AK4384_COMMAND_VOLUME_SET_CHANNEL_RIGHT:
        {
            if (false == drvObj->controlCommandStatus)
            {
                drvObj->command = DRV_AK4384_COMMAND_INIT_END;
               
                _DRV_AK4384_ConrolRegisterSet(drvObj,
                    DRV_AK4384_CONTROL_REGISTER_RATT,
                    drvObj->volume[DRV_AK4384_CHANNEL_RIGHT]);
                
            }
        }
        break;
        case DRV_AK4384_COMMAND_VOLUME_SET_CHANNEL_LEFT_ONLY:
        case DRV_AK4384_COMMAND_VOLUME_SET_CHANNEL_RIGHT_ONLY:
        case DRV_AK4384_COMMAND_VOLUME_SET_CHANNEL_LEFT:
        case DRV_AK4384_COMMAND_INIT_END:
        {
            if (false == drvObj->controlCommandStatus)
            {
                drvObj->command = DRV_AK4384_COMMAND_NONE;
                drvObj->status = SYS_STATUS_READY;
            }
            else
            {
                /* Do Nothing. Remain in this state untill
                 * the INIT_END command is transferred successfully */
                ;
            }
        }
        break;
        case DRV_AK4384_COMMAND_LEFT_CHANNEL_INVERT_ENABLE:
        {
            if (false == drvObj->controlCommandStatus)
            {
                drvObj->command = DRV_AK4384_COMMAND_RIGHT_CHANNEL_INVERT_ENABLE;
                regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_3],
                        DRV_AK4384_CTRL3_INVR_INV_POS,
                        DRV_AK4384_CTRL3_INVR_INV_VAL);
                _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_3,regValue);
            }
            else
            {
                /* Do Nothing. Remain in this state untill
                 * the INIT_END command is transferred successfully */
                ;
            }
        }
        break;


        case DRV_AK4384_COMMAND_LEFT_CHANNEL_INVERT_DISABLE:
        {
            if (false == drvObj->controlCommandStatus)
            {
                drvObj->command = DRV_AK4384_COMMAND_RIGHT_CHANNEL_INVERT_DISABLE;
                regValue = DRV_AK4384_CONTROL_REG_BIT_WRITE(
                        drvObj->lastRegValue[DRV_AK4384_CONTROL_REGISTER_3],
                        DRV_AK4384_CTRL3_INVR_NORM_POS,
                        DRV_AK4384_CTRL3_INVR_NORM_VAL);
                _DRV_AK4384_ConrolRegisterSet(drvObj,DRV_AK4384_CONTROL_REGISTER_3,regValue);
            }
            else
            {
                /* Do Nothing. Remain in this state untill
                 * the INIT_END command is transferred successfully */
                ;
            }
        }
        break;

        case DRV_AK4384_COMMAND_SAMPLING_RATE_SET:
        case DRV_AK4384_COMMAND_MUTE_ON:
        case DRV_AK4384_COMMAND_MUTE_OFF:
        case DRV_AK4384_COMMAND_ZERO_DETECT_ENABLE:
        case DRV_AK4384_COMMAND_ZERO_DETECT_DISABLE:
        case DRV_AK4384_COMMAND_ZERO_DETECT_MODE_SET:
        case DRV_AK4384_COMMAND_ZERO_DETECT_INVERT_ENABLE:
        case DRV_AK4384_COMMAND_ZERO_DETECT_INVERT_DISABLE:
        case DRV_AK4384_COMMAND_LEFT_CHANNEL_ONLY_INVERT_ENABLE:
        case DRV_AK4384_COMMAND_RIGHT_CHANNEL_ONLY_INVERT_ENABLE:
        case DRV_AK4384_COMMAND_RIGHT_CHANNEL_INVERT_ENABLE: // Coming here after enabling left channel
        case DRV_AK4384_COMMAND_LEFT_CHANNEL_ONLY_INVERT_DISABLE:
        case DRV_AK4384_COMMAND_RIGHT_CHANNEL_ONLY_INVERT_DISABLE:
        case DRV_AK4384_COMMAND_RIGHT_CHANNEL_INVERT_DISABLE: // Coming here after disabling left channel
        case DRV_AK4384_COMMAND_SLOW_ROLL_OFF_FILTER_ENABLE:
        case DRV_AK4384_COMMAND_SLOW_ROLL_OFF_FILTER_DISABLE:
        case DRV_AK4384_COMMAND_DEEMPHASIS_FILTER_SET:
        {
            if (false == drvObj->controlCommandStatus)
            {
//                drvObj->controlCommandStatus = true;
                drvObj->command = DRV_AK4384_COMMAND_NONE;
                if (drvObj->commandCompleteCallback != (DRV_AK4384_COMMAND_EVENT_HANDLER) 0)
                {
                    drvObj->commandCompleteCallback(drvObj->commandContextData);
                }
            }
            else
            {
                /* Do Nothing. Remain in this state untill
                 * the command is transferred successfully */
                ;
            }
        }
        break;
    }
    return;
}

// *****************************************************************************
 /*
  Function:
    static void _DRV_AK4384_TimerCallbackHandler(uintptr_t context,  uint32_t currTick)

  Summary:
    Implements the bit banging SPI implementation

  Description:
    Implements the bit banging SPI implementation for the control interface
    commands

  Remarks:
    None
*/
static void _DRV_AK4384_TimerCallbackHandler(uintptr_t context,  uint32_t currTick)
{
    DRV_AK4384_OBJ *drvObj;

    drvObj = (DRV_AK4384_OBJ *) context;
    drvObj->countBit++;

    if(drvObj->countBit >= 32)
    {
        /* Set Control Chip Select, Control Clock, Control Data */

        BSP_AK4384_CONTROL_CSOn();
        BSP_AK4384_CONTROL_CLKOn();
        BSP_AK4384_CONTROL_DOOn();

        /* Stop Timer */
        DRV_TMR_Stop (drvObj->tmrDriverHandle);

        /* Taking a backup of last successfull command submitted */
        drvObj->lastRegValue[((drvObj->controlCommand & 0x1F00)>>8)] = (drvObj->controlCommand & 0xFF);
        /* Reset Values */
        drvObj->countBit = 0;
        drvObj->controlCommandStatus = false;
    }
    else if((drvObj->countBit%2) == 0)
    {
        BSP_AK4384_CONTROL_CLKOff();
        if(DRV_AK4384_COMMAND_SHIFT_BIT((drvObj->countBit >> 1)))
        {
            BSP_AK4384_CONTROL_DOOn();
        }
        else
        {
            BSP_AK4384_CONTROL_DOOff();
        }
    }
    else
    {
        BSP_AK4384_CONTROL_CLKOn();
    }
    return;
}



// *****************************************************************************
 /*
  Function:
        static void _DRV_AK4384_I2SBufferEventHandler
        (
            DRV_I2S_BUFFER_EVENT event,
            DRV_I2S_BUFFER_HANDLE bufferHandle,
            uintptr_t contextHandle
        )

  Summary:
    Implements the handler for i2s buffer request completion.

  Description:
    Implements the handler for i2s buffer request completion.

  Remarks:
    None
*/
static void _DRV_AK4384_I2SBufferEventHandler
(
    DRV_I2S_BUFFER_EVENT event,
    DRV_I2S_BUFFER_HANDLE bufferHandle,
    uintptr_t contextHandle
)
{
    DRV_AK4384_CLIENT_OBJ *clientObj;

    if(DRV_I2S_BUFFER_HANDLE_INVALID == bufferHandle || 0 == contextHandle )
    {
        /* This means the handle is invalid */
        SYS_DEBUG(0, "Handle is invalid \r\n");
        return;
    }

    clientObj = (DRV_AK4384_CLIENT_OBJ *)contextHandle;
    if(DRV_I2S_BUFFER_EVENT_COMPLETE == event)
    {
        clientObj->pEventCallBack(DRV_AK4384_BUFFER_EVENT_COMPLETE,
            (DRV_AK4384_BUFFER_HANDLE) bufferHandle, clientObj->hClientArg);
    }
    else if(DRV_I2S_BUFFER_EVENT_ABORT == event)
    {
        clientObj->pEventCallBack(DRV_AK4384_BUFFER_EVENT_ABORT,
            (DRV_AK4384_BUFFER_HANDLE) bufferHandle, clientObj->hClientArg);
    }
    else if(DRV_I2S_BUFFER_EVENT_ERROR == event)
    {
        clientObj->pEventCallBack(DRV_AK4384_BUFFER_EVENT_ERROR,
            (DRV_AK4384_BUFFER_HANDLE) bufferHandle, clientObj->hClientArg);
    }
    else
    {
        ;
    }
    return;
}


/*******************************************************************************
 End of File
*/

