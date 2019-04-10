/*******************************************************************************
 Touch controller MXT336T driver file

  File Name:
    drv_MXT336T.c

  Summary:
    Touch controller MXT336T driver interface file.

  Description:
    This file consist of touch controller MXT336T driver interfaces. It
    implements the driver interfaces which read the touch input data from
    MXT336T through I2C bus.
 ******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2016 released Microchip Technology Inc.  All rights reserved.

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
 ******************************************************************************/
// DOM-IGNORE-END

#include <sys/attribs.h>
#include <sys/kmem.h>
#include "system/int/sys_int.h"
#include "system/touch/sys_touch.h"
#include "system/ports/sys_ports.h"
#include "driver/i2c/drv_i2c.h"
#include "driver/touch/mxt336t/src/drv_mxt336t_local.h"

/* MXT336T Driver instance object */
static struct DRV_MXT336T_DEVICE_OBJECT            
                  sMXT336TDriverInstances[DRV_MXT336T_INSTANCES_NUMBER];

/* MXT336T Driver client object pool, this is shared between all of the
 * driver instances */
static struct DRV_MXT336T_DEVICE_CLIENT_OBJECT     
                  sMXT336TClientInstances[DRV_MXT336T_CLIENTS_NUMBER];

/* flag to indicate if the shared client pool has been initialized */
static bool sClientsInitalized = false;

/* MXT336T Driver task queue */
static DRV_MXT336T_TASK_QUEUE
                  sMXT336TQueue[DRV_MXT336T_NUM_QUEUE];

DRV_I2C_BUFFER_EVENT operationStatus;


// *****************************************************************************
/* Find a specific MXT336T client object in the device object table 
 * Internally used by the driver to find the MXT336T object data  */
DRV_MXT336T_OBJECT_T* DRV_MXT336T_DEVICE_ClientObjectFind(
                    const struct DRV_MXT336T_DEVICE_OBJECT *pDrvObject,
                    const uint8_t objType, 
                    const uint8_t objInstance);


// *****************************************************************************
/* Find the report ID base number for a given MXT336T object, this is calculated
 * by summing all of the device instances and number of reports up to the specified
 * object in the table */
uint8_t DRV_MXT336T_DEVICE_ClientObjectFindReportIDBase(const struct DRV_MXT336T_DEVICE_OBJECT *pDrvObject,
            const uint8_t objType, const uint8_t objInstance);

// *****************************************************************************
// *****************************************************************************
// Section: Initialization
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* Function:
      SYS_MODULE_OBJ DRV_MXT336T_Initialize(const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the MXT336T instance for the specified driver index

  Description:
    This routine initializes the MXT336T driver instance for the specified
    driver index, making it ready for clients to open and use it. The
    initialization data is specified by the 'init' parameter. The initialization
    may fail if the number of driver objects allocated are insufficient or if
    the specified driver instance is already initialized. The driver instance
    index is independent of the MXT336T module ID. For example, driver instance
    0 can be assigned to MXT336T2.  If the driver is built statically, then
    some of the initialization parameters are overridden by configuration
    macros. Refer to the description of the DRV_MXT336T_INIT data
    structure for more details on which members on this data structure are
    overridden.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the instance to be initialized.  Please note this
             is not the MXT336T ID.  The hardware MXT336T ID is set in the
             initialization structure. This is the index of the driver index to
             use.

    init   - Pointer to a data structure containing any data necessary to
             initialize the driver. If this pointer is NULL, the driver
             uses the static initialization override macros for each
             member of the initialization data structure.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, returns SYS_MODULE_OBJ_INVALID.

  Example:
    <code>
    DRV_MXT336T_INIT        init;
    SYS_MODULE_OBJ      objectHandle;

    // Populate the MXT336T initialization structure
    // Touch Module Id
    init.moduleInit                  = {0},
    init.touchId                     = DRV_TOUCH_INDEX_0,
    init.drvInitialize               = NULL,
    init.drvOpen                     = DRV_I2C_Open,
    init.interruptSource             = INT_SOURCE_EXTERNAL_1,
    init.interruptChannel            = PORT_CHANNEL_D,
    init.interruptPin                = PORTS_BIT_POS_1,

    objectHandle = DRV_MXT336T_Initialize(DRV_TOUCH_INDEX_0,
                                              (SYS_MODULE_INIT*)init);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This routine must be called before any other MXT336T routine is called.

    This routine should only be called once during system initialization
    unless DRV_MXT336T_Deinitialize is called to deinitialize the driver
    instance. This routine will NEVER block for hardware access.
*/

SYS_MODULE_OBJ DRV_MXT336T_Initialize( const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init )
{
    const DRV_MXT336T_INIT *pInit = NULL;

    if ( index >= DRV_MXT336T_INDEX_COUNT )
    {
        SYS_ASSERT(false, "MXT336T Driver: Attempting to initialize an instance number greater than the max");
        return SYS_MODULE_OBJ_INVALID;
    }

    struct DRV_MXT336T_DEVICE_OBJECT * pDrvInstance =
                ( struct DRV_MXT336T_DEVICE_OBJECT *)&sMXT336TDriverInstances[index];

    if ( pDrvInstance->inUse == true )
    {
        SYS_ASSERT(false, "MXT336T Driver: Attempting to reinitialize a driver instance that is already in use");
        return SYS_MODULE_OBJ_INVALID;
    }

    pDrvInstance->inUse = true;

    pInit = (const DRV_MXT336T_INIT * const)init;

    /* */
    pDrvInstance->touchId               = pInit->touchId;
    pDrvInstance->drvOpen               = pInit->drvOpen;
    pDrvInstance->taskQueue             = (DRV_MXT336T_TASK_QUEUE*)&sMXT336TQueue;
    pDrvInstance->drvI2CHandle          = DRV_HANDLE_INVALID;
    pDrvInstance->pObjectTable          = NULL;
    pDrvInstance->messageClientObject.deviceObject = NULL;
    pDrvInstance->clientObject.deviceObject = NULL;
    /* */

    /* initialize the client pool if not already done */
    if (sClientsInitalized == false)
    {
        memset(sMXT336TClientInstances, 0, sizeof(sMXT336TClientInstances));
        sClientsInitalized = true;
    }
    
    pDrvInstance->status = SYS_STATUS_READY;
    pDrvInstance->deviceState = DRV_MXT336T_DEVICE_STATE_INIT;

    return (SYS_MODULE_OBJ)pDrvInstance;

}

/*************************************************************************
  Function:
       void DRV_MXT336T_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the MXT336T driver module.

  Description:
    Deinitializes the specified instance of the MXT336T driver module,
    disabling its operation (and any hardware) and invalidates all of the
    internal data.

  Preconditions:
    Function DRV_MXT336T_Initialize must have been called before calling
    this routine and a valid SYS_MODULE_OBJ must have been returned.

  Parameter:
    object -  Driver object handle, returned from DRV_MXT336T_Initialize

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;    //Returned from DRV_MXT336T_Initialize
    SYS_STATUS          status;

    DRV_MXT336T_Deinitialize ( object );

    status = DRV_MXT336T_Status( object );
    if( SYS_MODULE_UNINITIALIZED == status )
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the De-initialize
    operation must be called before the Initialize operation can be called
    again.

    This function will NEVER block waiting for hardware. If the operation
    requires time to allow the hardware to complete, this will be reported
    by the DRV_MXT336T_Status operation. The system has to use
    DRV_MXT336T_Status to determine when the module is in the ready state.
*/

void DRV_MXT336T_Deinitialize ( SYS_MODULE_OBJ object )
{
    struct DRV_MXT336T_DEVICE_OBJECT * pDrvInstance =
                                        (struct DRV_MXT336T_DEVICE_OBJECT *)object;
    struct DRV_MXT336T_DEVICE_CLIENT_OBJECT *pClient = &sMXT336TClientInstances[0];

    if( pDrvInstance == NULL )
    {
        SYS_ASSERT(false, "MXT336T Driver: Attempting to deinitialize a NULL object");
        return;
    }

    if ( pDrvInstance->inUse == false )
    {
        SYS_ASSERT(false, "MXT336T Driver: Attempting to deinitialize a driver instance that is not in use");
        return;
    }

    SYS_INT_SourceDisable(pDrvInstance->interruptSource);

    if( pClient->deviceObject == (struct DRV_MXT336T_DEVICE_OBJECT * )pDrvInstance)
    {
        pClient->deviceObject = NULL;
    }

    pDrvInstance->touchId               = 0xFF;
    pDrvInstance->inUse                 = false;
    pDrvInstance->status                = SYS_STATUS_UNINITIALIZED;

    return;
}

/**************************************************************************
  Function:
       SYS_STATUS DRV_MXT336T_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the MXT336T driver module.

  Description:
    This function provides the current status of the MXT336T driver module.

  Precondition:
    The DRV_MXT336T_Initialize function must have been called before
    calling this function.

  Parameters:
    object -  Driver object handle, returned from DRV_MXT336T_Initialize

  Returns:
    SYS_STATUS_READY - Indicates that the driver is busy with a previous
    system-level operation and cannot start another

  Example:
    <code>
    SYS_MODULE_OBJ      object;  // Returned from DRV_MXT336T_Initialize
    SYS_STATUS          status;

    status = DRV_MXT336T_Status( object );
    if( SYS_STATUS_READY != status )
    {
        // Handle error
    }
    </code>

  Remarks:
    Any value greater than SYS_STATUS_READY is also a normal running state
    in which the driver is ready to accept new operations.

    SYS_MODULE_UNINITIALIZED - Indicates that the driver has been
    deinitialized

    This value is less than SYS_STATUS_ERROR.

    This function can be used to determine when any of the driver's module
    level operations has completed.

    If the status operation returns SYS_STATUS_BUSY, the previous operation
    has not yet completed. Once the status operation returns
    SYS_STATUS_READY, any previous operations have completed.

    The value of SYS_STATUS_ERROR is negative (-1). Any value less than
    that is also an error state.

    This function will NEVER block waiting for hardware.

    If the Status operation returns an error value, the error may be
    cleared by calling the reinitialize operation. If that fails, the
    deinitialize operation will need to be called, followed by the
    initialize operation to return to normal operations.
*/

SYS_STATUS DRV_MXT336T_Status ( SYS_MODULE_OBJ object )
{
    struct DRV_MXT336T_DEVICE_OBJECT * pDrvInstance =
                                        ( struct DRV_MXT336T_DEVICE_OBJECT *)object;

    if ( object == SYS_MODULE_OBJ_INVALID )
    {
        //SYS_ASSERT( " Handle is invalid " );
        return SYS_STATUS_ERROR;
    }
    
    return pDrvInstance->status;
}

/**************************************************************************
  Function:
       DRV_HANDLE DRV_MXT336T_Open ( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified MXT336T driver instance and returns a handle to it.

  Description:
    This routine opens the specified MXT336T driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    Specifying a DRV_IO_INTENT_EXCLUSIVE will cause the driver to provide
    exclusive access to this client. The driver cannot be opened by any
    other client.

  Precondition:
    The DRV_MXT336T_Initialize function must have been called before
    calling this function.

  Parameters:
    drvIndex -  Index of the driver initialized with
                DRV_MXT336T_Initialize().

    intent -    Zero or more of the values from the enumeration
                DRV_IO_INTENT ORed together to indicate the intended use of
                the driver

  Returns:
    If successful, the routine returns a valid open-instance handle (a
    number identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID. An error
    can occur when the following is true:
      * if the number of client objects allocated via
        DRV_MXT336T_CLIENTS_NUMBER is insufficient
      * if the client is trying to open the driver but driver has been
        opened exclusively by another client
      * if the driver hardware instance being opened is not initialized or
        is invalid

  Example:
    <code>
    DRV_HANDLE  handle;

    handle = DRV_MXT336T_Open( DRV_MXT336T_INDEX_0,
                                      DRV_IO_INTENT_EXCLUSIVE );

    if( DRV_HANDLE_INVALID == handle )
    {
        // Unable to open the driver
    }
    </code>

  Remarks:
    The handle returned is valid until the DRV_MXT336T_Close routine is
    called. This routine will NEVER block waiting for hardware. If the
    requested intent flags are not supported, the routine will return
    DRV_HANDLE_INVALID. This function is thread safe in a RTOS application.
    It should not be called in an ISR.
*/

DRV_HANDLE DRV_MXT336T_Open ( const SYS_MODULE_INDEX index,
                                                    const DRV_IO_INTENT intent )
{ 
    if (index >= DRV_MXT336T_INDEX_COUNT)
    {
        SYS_ASSERT(false, "MXT336T Driver: Attempting to open an instance" \
                          "number greater than the max");
        return DRV_HANDLE_INVALID;
    }
    
    struct DRV_MXT336T_DEVICE_OBJECT * pDrvInstance =
                 ( struct DRV_MXT336T_DEVICE_OBJECT *)&sMXT336TDriverInstances[index];

    /* Open the bus driver */
    if(pDrvInstance->drvOpen == NULL)
    {
        SYS_ASSERT(false, "MXT336T Driver: Bus driver init parameter missing");
        return DRV_HANDLE_INVALID;
    }
    
    if(pDrvInstance->drvI2CHandle == DRV_HANDLE_INVALID)
    {
    pDrvInstance->drvI2CHandle = pDrvInstance->drvOpen( DRV_MXT336T_I2C_MODULE_INDEX,
                                                      DRV_IO_INTENT_READWRITE);
    }
    
    if(pDrvInstance->drvI2CHandle == DRV_HANDLE_INVALID)
    {
        SYS_ASSERT(false, "MXT336T Driver: Bus driver initialization failed");
        return DRV_HANDLE_INVALID;
    }

    if ((intent & DRV_IO_INTENT_EXCLUSIVE) == DRV_IO_INTENT_EXCLUSIVE)
    {
        pDrvInstance->isExclusive = true;
    }
    
       
    pDrvInstance->status = SYS_STATUS_BUSY;
    
    /* Start process to read information block from the device */
    pDrvInstance->taskQueue[0].drvI2CFrameData[0] = 0;
    pDrvInstance->taskQueue[0].drvI2CFrameData[1] = 0;
    
    return (DRV_HANDLE)pDrvInstance;
}

// *****************************************************************************
/* Function:
    void DRV_MXT336T_Close ( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the MXT336T driver

  Description:
    This function closes an opened instance of the MXT336T driver, invalidating
    the handle.

  Precondition:
    The DRV_MXT336T_Initialize routine must have been called for the
    specified MXT336T driver instance.

    DRV_MXT336T_Open must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_MXT336T_Open

    DRV_MXT336T_Close ( handle );
    </code>

  Remarks:
	After calling this routine, the handle passed in "handle" must not be
    used with any of the remaining driver routines.  A new handle must be
    obtained by calling DRV_MXT336T_Open before the caller may use the
    driver again. This function is thread safe in a RTOS application.

    Note: Usually, there is no need for the driver client to verify that the
          Close operation has completed.
*/

void DRV_MXT336T_Close ( DRV_HANDLE handle )
{
    uint32_t    instance;
    struct DRV_MXT336T_DEVICE_CLIENT_OBJECT *pClientObject;
    struct DRV_MXT336T_DEVICE_OBJECT * pDrvObject =
                                (struct DRV_MXT336T_DEVICE_OBJECT *)handle;

    if( pDrvObject == NULL )
    {
        SYS_ASSERT(false, "MXT336T Driver: Trying to close a client with invalid driver object");
         return;
    }
    
    /* move driver to the idle state to stop any processes */
    pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_INIT;    
    
    /* Close any clients first */
    for(instance = 0; instance < DRV_MXT336T_CLIENTS_NUMBER; instance++)
    {
        pClientObject = &sMXT336TClientInstances[instance];
        /* if the client has this driver has a parent then close it */
        if (pDrvObject == pClientObject->deviceObject)
        {
            /* remove the client object */
            pClientObject->context = 0;
            pClientObject->deviceObject = NULL;
            pClientObject->pMXT336TObject = NULL;
            pClientObject->pNext = NULL;
            pClientObject->report_id_base = 0;   
        }
    }
    
    /* free up the memory used */
    OSAL_Free(pDrvObject->pObjectTable);
    pDrvObject->pObjectTable = NULL;
    
    return;
}

// *****************************************************************************
/* Function:
    void DRV_MXT336T_ReadRequest( SYS_MODULE_OBJ object )

  Summary:
    Sends a read request to I2C bus driver and adds the read task to queue.

  Description:
	This routine is used to send a touch input read request to the I2C bus
        driver. It is always called from MXT336T interrupt ISR routine.

  Precondition:
    The DRV_MXT336T_Initialize routine must have been called for the
    specified MXT336T driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_MXT336T_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;   // Returned from DRV_MXT336T_Initialize

    void __ISR(_EXTERNAL_INT_VECTOR, ipl5) _IntHandlerDrvMXT(void)
    {
        DRV_MXT336T_ReadRequest ( object );

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the MXT336T ISR routine.

*/

void DRV_MXT336T_ReadRequest( SYS_MODULE_OBJ object )
{
    struct DRV_MXT336T_DEVICE_OBJECT *pDrvObject = (struct DRV_MXT336T_DEVICE_OBJECT *)object;

    if ( object == SYS_MODULE_OBJ_INVALID )
    {
        return;
    }    
    
    /* Check we are not processing any existing transaction, if a transaction is still
     * in process then the CHG line will still be asserted and the read will be started
     * in the main tasks routine */
    if ( DRV_MXT336T_DEVICE_STATE_READY != pDrvObject->deviceState)
    {
        return;
    }
    
    _DRV_MXT336T_DEVICE_MessageObjectRead(pDrvObject);
    pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_READ_MESSAGE_OBJECT;        

    return;
}

// *****************************************************************************
/* Function:
    void DRV_MXT336T_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's state machine and implements its task queue
    processing.

  Description:
	This routine is used to maintain the driver's internal state
	machine and implement its command queue processing. It is always called
        from SYS_Tasks() function. 

  Precondition:
    The DRV_MXT336T_Initialize routine must have been called for the
    specified MXT336T driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_MXT336T_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;   // Returned from DRV_MXT336T_Initialize

    void SYS_Tasks( void )
    {
        DRV_MXT336T_Tasks ( object );

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks)

*/

bool MXT_INTERRUPT_PIN_VALUE_GET(void)
{
    return(PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_E, PORTS_BIT_POS_8 ));
}


void DRV_MXT336T_Tasks ( SYS_MODULE_OBJ object )
{
    uint8_t                                     numObjects;
    uint8_t                                     reportID;
    uint32_t                                    objTableSize;
    uint32_t                                    clientIndex;
    uint16_t                                    xRange;
    uint16_t                                    yRange;
    struct DRV_MXT336T_DEVICE_CLIENT_OBJECT    *pClientObject;
    DRV_MXT336T_OBJECT_T                       *pMXT336TObject;
    static DRV_MXT336T_OBJECT_CLIENT_EVENT_DATA       MXT336TEvent;
    
    struct DRV_MXT336T_DEVICE_OBJECT * pDrvObject =
            (struct DRV_MXT336T_DEVICE_OBJECT *)object;

    if ( object == SYS_MODULE_OBJ_INVALID )
    {
        return;
    }
    
    /* MXT336T Driver state machine */
    switch (pDrvObject->deviceState)
    {
        case DRV_MXT336T_DEVICE_STATE_INIT:
            
            pDrvObject->sysTmrMXT336T = SYS_TMR_DelayMS(500);
            
            if(pDrvObject->sysTmrMXT336T != SYS_TMR_HANDLE_INVALID)
            {
                pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_DELAYED_OPEN;
            }
            
            break;
            
        case DRV_MXT336T_DEVICE_STATE_DELAYED_OPEN: /* Driver Initialize state */
            if(SYS_TMR_DelayStatusGet (pDrvObject->sysTmrMXT336T))
            {
                pDrvObject->taskQueue[0].drvI2CBufferHandle = 
                DRV_I2C_Transmit(pDrvObject->drvI2CHandle, DRV_MXT336T_I2C_MASTER_WRITE_ID, 
                                &pDrvObject->taskQueue[0].drvI2CFrameData[0], 2, NULL);  

                if (pDrvObject->taskQueue[0].drvI2CBufferHandle)
                {
                    pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_INIT_RESET;
                }

            }
    
            break;      
         
        case DRV_MXT336T_DEVICE_STATE_INIT_RESET: /* Device reset address */
            if (DRV_I2C_BUFFER_EVENT_COMPLETE != DRV_I2C_TransferStatusGet(pDrvObject->drvI2CHandle,pDrvObject->taskQueue[0].drvI2CBufferHandle))
            {
                return;
            }
            
            /* read the information block */
            pDrvObject->taskQueue[0].drvI2CBufferHandle = 
                    DRV_I2C_Receive(pDrvObject->drvI2CHandle, DRV_MXT336T_I2C_MASTER_READ_ID,
                                    &pDrvObject->taskQueue[0].drvI2CFrameData[0], 7, NULL);
            if (!pDrvObject->taskQueue[0].drvI2CBufferHandle)
            {
                pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_ERROR;
                break;
            }
            pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_READ_IB;
            break;          

        case DRV_MXT336T_DEVICE_STATE_READ_IB: /* Read information block */
            if (DRV_I2C_BUFFER_EVENT_COMPLETE != DRV_I2C_TransferStatusGet(pDrvObject->drvI2CHandle,pDrvObject->taskQueue[0].drvI2CBufferHandle))
            {
                return;
            }
            
            /* copy the information block */
            numObjects = pDrvObject->taskQueue[0].drvI2CFrameData[6];
            
            /* read the object table */
            objTableSize = numObjects * sizeof(DRV_MXT336T_OBJECT_T);
            objTableSize += sizeof(DRV_MXT336T_INFO_ID_T);            
            pDrvObject->pObjectTable = OSAL_Malloc(objTableSize); 
            
            pDrvObject->taskQueue[0].drvI2CBufferHandle =
                    DRV_I2C_Receive(pDrvObject->drvI2CHandle, DRV_MXT336T_I2C_MASTER_READ_ID,
                                    pDrvObject->pObjectTable, objTableSize, NULL);
            
            if (!pDrvObject->taskQueue[0].drvI2CBufferHandle)
            {
                pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_ERROR;
                break;
            }
            
            pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_READ_OBJECT_TABLE;
            break;            

        case DRV_MXT336T_DEVICE_STATE_READ_OBJECT_TABLE: /* Wait for object table to be read */
            if (DRV_I2C_BUFFER_EVENT_COMPLETE != DRV_I2C_TransferStatusGet(pDrvObject->drvI2CHandle,pDrvObject->taskQueue[0].drvI2CBufferHandle))
            {
                return;
            }
            
            /* we assume that all MXT336T devices have a message processor object in order to signal 
             * events back to the host. Search for and hold a pointer to this object */
            pDrvObject->messageClientObject.pMXT336TObject = 
                    //DRV_MXT336T_DEVICE_ClientObjectFind(pDrvObject, DRV_MXT336T_OBJECT_GEN_MESSAGEPROCESSOR_T5, 1);
                    DRV_MXT336T_DEVICE_ClientObjectFind(pDrvObject, DRV_MXT336T_OBJECT_GEN_MESSAGEPROCESSOR_T5, 0);
            /* if no message object then an error */       
            if (pDrvObject->messageClientObject.pMXT336TObject == NULL)
            {
                pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_ERROR;
                break;
            }
                           
            pDrvObject->messageClientObject.clientCallback = NULL;       /* T5 does not callback into the driver */
            pDrvObject->messageClientObject.deviceObject = pDrvObject;
            pDrvObject->messageClientObject.pNext = NULL;               /* no other copies of T5 kept */
            pDrvObject->messageClientObject.report_id_base = 
                    //DRV_MXT336T_DEVICE_ClientObjectFindReportIDBase(pDrvObject, DRV_MXT336T_OBJECT_GEN_MESSAGEPROCESSOR_T5, 1);
                    DRV_MXT336T_DEVICE_ClientObjectFindReportIDBase(pDrvObject, DRV_MXT336T_OBJECT_GEN_MESSAGEPROCESSOR_T5, 0);

            pDrvObject->clientObject.pMXT336TObject = (DRV_MXT336T_OBJECT_T*)
                    //DRV_MXT336T_DEVICE_ClientObjectFind(pDrvObject, DRV_MXT336T_OBJECT_TOUCH_MULTITOUCHSCREEN_T100, 1);
                    DRV_MXT336T_DEVICE_ClientObjectFind(pDrvObject, DRV_MXT336T_OBJECT_TOUCH_MULTITOUCHSCREEN_T100, 0);
            /* if no message object then an error */       
            if (pDrvObject->clientObject.pMXT336TObject == NULL)
            {
                pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_ERROR;
                break;
            }
            
            pDrvObject->clientObject.clientCallback = NULL;       /* T5 does not callback into the driver */
            pDrvObject->clientObject.deviceObject = pDrvObject;
            pDrvObject->clientObject.pNext = NULL;               /* no other copies of T5 kept */
            pDrvObject->clientObject.report_id_base = 
                    //DRV_MXT336T_DEVICE_ClientObjectFindReportIDBase(pDrvObject, DRV_MXT336T_OBJECT_TOUCH_MULTITOUCHSCREEN_T100, 1);
                    DRV_MXT336T_DEVICE_ClientObjectFindReportIDBase(pDrvObject, DRV_MXT336T_OBJECT_TOUCH_MULTITOUCHSCREEN_T100, 0);

            _DRV_MXT336T_DEVICE_RegRead(pDrvObject,pDrvObject->clientObject.pMXT336TObject, DRV_MXT336T_T100_XRANGE);
            
            //pDrvObject->status = SYS_STATUS_READY;
            pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_READ_T100_XRANGE;
            break;
   
        case DRV_MXT336T_DEVICE_STATE_READ_T100_XRANGE:
            if (DRV_I2C_BUFFER_EVENT_COMPLETE != DRV_I2C_TransferStatusGet(pDrvObject->drvI2CHandle,pDrvObject->taskQueue[1].drvI2CBufferHandle))
            {
                return;
            }
            
            xRange = pDrvObject->taskQueue[1].drvI2CFrameData[0];
            xRange |= (pDrvObject->taskQueue[1].drvI2CFrameData[1]) << 8;
            MXT336TEvent.xRange = xRange;
                            
            _DRV_MXT336T_DEVICE_RegRead(pDrvObject, pDrvObject->clientObject.pMXT336TObject, DRV_MXT336T_T100_YRANGE);
            

            pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_READ_T100_YRANGE;
            break;
            
        case DRV_MXT336T_DEVICE_STATE_READ_T100_YRANGE:
            if (DRV_I2C_BUFFER_EVENT_COMPLETE != DRV_I2C_TransferStatusGet(pDrvObject->drvI2CHandle,pDrvObject->taskQueue[1].drvI2CBufferHandle))
            {
                return;
            }
            
            yRange = pDrvObject->taskQueue[1].drvI2CFrameData[0];
            yRange |= (pDrvObject->taskQueue[1].drvI2CFrameData[1]) << 8;
            MXT336TEvent.yRange = yRange;
             

            pDrvObject->status = SYS_STATUS_READY;
            pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_READY;
            break;

        case DRV_MXT336T_DEVICE_STATE_READY: /* Driver ready state */
            /* device can now be accessed by clients */
            
			/* Check for ~CHG line asserted in case interrupt line did not come back up high */
            /* send a read request to the message processor object T5 */ 
            if(MXT_INTERRUPT_PIN_VALUE_GET() == 0)
            {
                _DRV_MXT336T_DEVICE_MessageObjectRead(pDrvObject);
                
                pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_READ_MESSAGE_OBJECT;                   
            }
            break;    
        case DRV_MXT336T_DEVICE_STATE_READ_MESSAGE_OBJECT: /* read a specified object from the device */
            if (DRV_I2C_BUFFER_EVENT_COMPLETE != DRV_I2C_TransferStatusGet(pDrvObject->drvI2CHandle,pDrvObject->taskQueue[1].drvI2CBufferHandle))
            {
                return;
            }
            
            /* completed reading the object (in our case DRV_MXT336T_OBJECT_TOUCH_MULTITOUCHSCREEN_T100)that generated the message in T5, parse the report ID*/   
            reportID = pDrvObject->taskQueue[1].drvI2CFrameData[0];
            if ((reportID == 0x00) || (reportID == 0xFF))
            {
                /* invalid message */
                pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_READY;   
                return;
            }                       
            
            /* process the report by examining frame data for a report ID */
            /* find if any clients are listening to this object and pass the message back to them if so */
            for(clientIndex = 0; clientIndex < DRV_MXT336T_CLIENTS_NUMBER; clientIndex++)
            {
                pClientObject = (struct DRV_MXT336T_DEVICE_CLIENT_OBJECT*) &sMXT336TClientInstances[clientIndex];
                
                /* check the client belongs to this device */
                if (pClientObject->deviceObject == pDrvObject)
                {
                    pMXT336TObject = pClientObject->pMXT336TObject;
                    /* check the report ID belongs to this client */
                    if ((reportID >= pClientObject->report_id_base) && 
                        (reportID < (pClientObject->report_id_base + pMXT336TObject->num_report_ids)))
                    {
                        /* at this point there is a qualified report that matches a client */
                        if (pClientObject->clientCallback)
                        {
                            /* prepare the callback event data */
                            MXT336TEvent.reportID = reportID - pClientObject->report_id_base;
                            MXT336TEvent.dataSize = pDrvObject->messageClientObject.pMXT336TObject->size - 1;
                            MXT336TEvent.pData = &pDrvObject->taskQueue[1].drvI2CFrameData[1];
                            
                            pClientObject->clientCallback((DRV_HANDLE) pClientObject, &MXT336TEvent, pClientObject->context);
                        }   
                    }
                }
            }
            
            pDrvObject->deviceState = DRV_MXT336T_DEVICE_STATE_READY;    

            break;
            
        case DRV_MXT336T_DEVICE_STATE_ERROR: /* In error state */
            pDrvObject->status = SYS_STATUS_ERROR;
            break;

        default:
            break;
    }
    
    return;
}

/**************************************************************************
  Function:
       DRV_HANDLE DRV_MXT336T_OpenObject ( const DRV_HANDLE deviceHandle, const uint8_t objType,
               const uint8_t objInstance )

  Summary:
    Opens the specified MXT336T object driver instance and returns a handle to it.
	<p><b>Implementation:</b> Dynamic</p>
	
  Description:
    This routine opens the specified MXT336T object driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. 
	
  Precondition:
    The DRV_MXT336T_Initialize function must have been called before 
    calling this function. The driver must have been opened.
	
  Parameters:
    deviceHandle -  Handle of the MXT336T device
    objType      -  Object type being requested
    objInstance  -  Instance of the object of this type
    
				
  Returns:
    If successful, the routine returns a valid object-instance handle (
		
  Example:
    <code>
    DRV_HANDLE  handle;


    </code>
	
  Remarks:

*/

DRV_HANDLE DRV_MXT336T_OpenObject ( const DRV_HANDLE deviceHandle, const uint8_t objType,
               const uint8_t objInstance )
{
    uint32_t                                    instance;
    DRV_MXT336T_OBJECT_T                       *pObject;
    struct DRV_MXT336T_DEVICE_CLIENT_OBJECT    *pClientObject;
    uint8_t                                     reportIDBase;
    
    struct DRV_MXT336T_DEVICE_OBJECT * pDrvObject =
            (struct DRV_MXT336T_DEVICE_OBJECT *)deviceHandle;

    if ( pDrvObject == NULL )
    {
        return DRV_HANDLE_INVALID;
    }
    
    if (pDrvObject->pObjectTable == NULL)
    {
        return DRV_HANDLE_INVALID;
    } 
    
    /* find the matching object in the object table */
    pObject = (DRV_MXT336T_OBJECT_T*) DRV_MXT336T_DEVICE_ClientObjectFind(pDrvObject, objType, objInstance);
    if (pObject == NULL)
    {
        return DRV_HANDLE_INVALID;
    }
    
    /* find the matching report ID base for the given object */
    reportIDBase = DRV_MXT336T_DEVICE_ClientObjectFindReportIDBase(pDrvObject, objType, objInstance);
    
    /* find an unused client instance, we use the deviceObject handle as an indication that an
     * entry has been assigned and is therefore already used */
    instance = 0;
    while ((instance < DRV_MXT336T_CLIENTS_NUMBER) && (sMXT336TClientInstances[instance].deviceObject != NULL))
    {
        instance++;
    }
        
    /* no spare instances */
    if (instance == DRV_MXT336T_CLIENTS_NUMBER)
    {
        return DRV_HANDLE_INVALID;
    }
        
    pClientObject = &sMXT336TClientInstances[instance];
    pClientObject->clientCallback = NULL;
    pClientObject->deviceObject = pDrvObject;
    pClientObject->pMXT336TObject = pObject;
    pClientObject->pNext = NULL;
    pClientObject->report_id_base = reportIDBase;
    
    
    
    
    return (DRV_HANDLE) pClientObject;    
}

// *****************************************************************************
/* Function:
    void DRV_MXT336T_CloseObject ( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the MXT336T client object

  Description:
    This function closes an opened instance of the MXT336T client object, invalidating
    the handle.

  Precondition:
    The DRV_MXT336T_Initialize routine must have been called for the
    specified MXT336T driver instance.

    DRV_MXT336T_OpenObject must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_MXT336T_Open

    DRV_MXT336T_CloseObject ( handle );
    </code>

  Remarks:
	After calling this routine, the handle passed in "handle" must not be
    used with any of the remaining driver routines.  A new handle must be
    obtained by calling DRV_MXT336T_OpenObject before the caller may use the
    driver again. This function is thread safe in a RTOS application.

    Note: Usually, there is no need for the driver client to verify that the
          Close operation has completed.
*/

void DRV_MXT336T_CloseObject ( DRV_HANDLE handle )
{
    uint32_t                                instance;
    struct DRV_MXT336T_DEVICE_CLIENT_OBJECT *pClientObject =
                                (struct DRV_MXT336T_DEVICE_CLIENT_OBJECT *)handle;

    if( pClientObject == NULL )
    {
        return;
    }
    
    /* locate the client object instance ID */
    instance = 0;
    while (pClientObject != &sMXT336TClientInstances[instance])
    {
        instance++;
    }
    
    /* remove the client object */
    pClientObject->context = 0;
    pClientObject->deviceObject = NULL;
    pClientObject->pMXT336TObject = NULL;
    pClientObject->pNext = NULL;
    pClientObject->report_id_base = 0;
    
    return;
}


// *****************************************************************************
/* Function:
    bool DRV_MXT336T_DEVICE_ClientObjectEventHandlerSet(const DRV_HANDLE clientHandle,
        const DRV_MXT336T_CLIENT_CALLBACK callback, uintptr_t context)

  Summary:
    Sets the event handler for a MXT336T client object

  Description:
    This function sets the event handler used to handle report messages
    from a MXT336T object.
  Precondition:
    The DRV_MXT336T_OpenObject routine must have been called for the
    specified MXT336T driver instance.

    DRV_MXT336T_OpenObject must have been called to obtain a valid opened
    device handle.

  Parameters:
    clientHandle        - A valid open-instance handle, returned from the driver's
                            openobject routine
    
    callback            - A callback function to handle report messages
 
    context             - The context for the call
  Returns:
    bool                - true if the handler was successfully set
                        - false if the handler could not be set

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_MXT336T_OpenObject

    DRV_MXT336T_DEVICE_ClientObjectEventHandlerSet(handle, objectCallback, NULL);
    </code>

  Remarks:

*/

bool DRV_MXT336T_DEVICE_ClientObjectEventHandlerSet(const DRV_HANDLE clientHandle,
        const DRV_MXT336T_CLIENT_CALLBACK callback, uintptr_t context)
{
    struct DRV_MXT336T_DEVICE_CLIENT_OBJECT   *pClientObject =
        (struct DRV_MXT336T_DEVICE_CLIENT_OBJECT*) clientHandle;
    
    /* validate the handle */
    if ((clientHandle == DRV_HANDLE_INVALID))
    {
        return false;       
    }   
    
    /* set up the callback function */
    pClientObject->context = context;
    pClientObject->clientCallback = callback;
    
    return true;
}


// *****************************************************************************
// *****************************************************************************
// Section: Internal functions used by this module only
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
/* Send request to read message processor object */
static void _DRV_MXT336T_DEVICE_MessageObjectRead(struct DRV_MXT336T_DEVICE_OBJECT *pDrvObject)
{
    size_t                                      msgSize;
    struct DRV_MXT336T_DEVICE_CLIENT_OBJECT    *pMessageClient;
    DRV_MXT336T_OBJECT_T                       *pMXT336TObject;

    if (pDrvObject == NULL)
    {
        return;
    }
    
    /* ensure that we have a message object assigned for this device */
    pMessageClient = &pDrvObject->messageClientObject;
    
    if (pMessageClient->deviceObject == NULL)
    {
        return;
    }

    pMXT336TObject = pMessageClient->pMXT336TObject;
    if (pMXT336TObject == NULL)
    {
        return;
    }
    
    /* write the address of the message processor object to the device */     
    pDrvObject->taskQueue[0].drvI2CFrameData[0] = pMXT336TObject->i2c_address & 0xFF;  
    pDrvObject->taskQueue[0].drvI2CFrameData[1] = pMXT336TObject->i2c_address >> 8;
    pDrvObject->taskQueue[0].drvI2CBufferHandle = 
                DRV_I2C_Transmit(pDrvObject->drvI2CHandle, DRV_MXT336T_I2C_MASTER_WRITE_ID, 
                    &pDrvObject->taskQueue[0].drvI2CFrameData[0], 2, NULL); 

    /* schedule a read of the message processor object */
    msgSize = (pMXT336TObject->size + 1) - 2 + 1; /* + 1 for size offset, -2 for spec, +1 for reportID field */
    
    if (msgSize >= DRV_MXT336T_I2C_FRAME_SIZE)
    {
        return;
    }
    
    pDrvObject->taskQueue[1].drvI2CBufferHandle = 
            DRV_I2C_Receive(pDrvObject->drvI2CHandle, DRV_MXT336T_I2C_MASTER_READ_ID,
                    &pDrvObject->taskQueue[1].drvI2CFrameData[0], msgSize, NULL);
}

/* Send request to read message processor object */
static void _DRV_MXT336T_DEVICE_RegRead(struct DRV_MXT336T_DEVICE_OBJECT *pDrvObject, DRV_MXT336T_OBJECT_T  *pMXT336TObject, uint8_t reg)
{
    uint16_t pReg ;
    //void* addr = NULL;
    if (pDrvObject == NULL)
    {
        return;
    }
      

    
    if (pMXT336TObject == NULL)
    {
        return;
    }
    
    
    /* write the address of the object register to the device */     
    pReg = pMXT336TObject->i2c_address + reg;
    
    pDrvObject->taskQueue[0].drvI2CFrameData[0] = pReg & 0xFF;  
    pDrvObject->taskQueue[0].drvI2CFrameData[1] = pReg >> 8;
            
            
    //addr = pDrvObject->taskQueue[0].drvI2CFrameData[0]+pReg;
    pDrvObject->taskQueue[0].drvI2CBufferHandle = 
                DRV_I2C_Transmit(pDrvObject->drvI2CHandle, DRV_MXT336T_I2C_MASTER_WRITE_ID, 
                    &pDrvObject->taskQueue[0].drvI2CFrameData[0], 2, NULL); 

    
    /* schedule a read of the specific register in the object */
    //msgSize = (pMXT336TObject->size + 1) - 2 + 1; /* + 1 for size offset, -2 for spec, +1 for reportID field */
    pDrvObject->taskQueue[1].drvI2CBufferHandle = 
            DRV_I2C_Receive(pDrvObject->drvI2CHandle, DRV_MXT336T_I2C_MASTER_READ_ID,
                    &pDrvObject->taskQueue[1].drvI2CFrameData[0], 2, NULL);

}
// *****************************************************************************
/* Find the report ID base number for a given MXT336T object, this is calculated
 * by summing all of the device instances and number of reports up to the specified
 * object in the table */
uint8_t DRV_MXT336T_DEVICE_ClientObjectFindReportIDBase(const struct DRV_MXT336T_DEVICE_OBJECT *pDrvObject,
            const uint8_t objType, const uint8_t objInstance)
{
    DRV_MXT336T_INFO_ID_T  *pInfoBlock;
    DRV_MXT336T_OBJECT_T   *pObject;
    uint8_t                 objectIndex = 0;
    uint8_t                 reportIDBase = 1;
    
    if ( pDrvObject == NULL )
    {
        return 0;
    }
    
    if (pDrvObject->pObjectTable == NULL)
    {
        return 0;
    }
    
    /* get the info block and first object in the object table */
    pInfoBlock = (DRV_MXT336T_INFO_ID_T*) pDrvObject->pObjectTable;
    pObject = (DRV_MXT336T_OBJECT_T*) (pDrvObject->pObjectTable + sizeof(DRV_MXT336T_INFO_ID_T));  
    
    /* iterate through the object table to find the required object and instance */
    do {
        /* check the object type */
        if (objType == pObject->object_type)
        {
            /* found a match for the object type */
            /* check the instance matches the requested one */
            //if (objInstance == (pObject->instances + 1))
            if (objInstance <= (pObject->instances)  )
            {
                return reportIDBase;                       
            }                   
        }
        
        reportIDBase += (pObject->num_report_ids * (pObject->instances + 1));
        //reportIDBase += (pObject->num_report_ids * (pObject->instances));
        pObject++;
        objectIndex++;    
    } while (objectIndex != pInfoBlock->num_declared_objects);
    
    return 0;    
}


// *****************************************************************************
/* Find a specific MXT336T client object in the device object table 
 * Internally used by the driver to find the MXT336T object data  */
DRV_MXT336T_OBJECT_T* DRV_MXT336T_DEVICE_ClientObjectFind(
                    const struct DRV_MXT336T_DEVICE_OBJECT *pDrvObject,
                    const uint8_t objType, 
                    const uint8_t objInstance)
{
    DRV_MXT336T_INFO_ID_T  *pInfoBlock;
    DRV_MXT336T_OBJECT_T   *pObject;
    uint8_t                 objectIndex = 0;
    
    if ( pDrvObject == NULL )
    {
        return NULL;
    }
    
    if (pDrvObject->pObjectTable == NULL)
    {
        return NULL;
    }
    
    /* get the info block and first object in the object table */
    pInfoBlock = (DRV_MXT336T_INFO_ID_T*) pDrvObject->pObjectTable;
    pObject = (DRV_MXT336T_OBJECT_T*) (pDrvObject->pObjectTable + sizeof(DRV_MXT336T_INFO_ID_T));  
    
    /* iterate through the object table to find the required object and instance */
    do {
        /* check the object type */
        if (objType == pObject->object_type)
        {
            /* found a match for the object type */
            
            /* check the instance matches the requested one - this is questionable for now */
            //if (objInstance == (pObject->instances + 1))
            if (objInstance <= (pObject->instances))    
            {
                return pObject;                       
            }                   
        }
        
        pObject++;
        objectIndex++;    
    } while (objectIndex != pInfoBlock->num_declared_objects);
    
    return NULL;    
}


