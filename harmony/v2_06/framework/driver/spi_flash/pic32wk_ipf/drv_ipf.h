/*******************************************************************************
  SPI Flash Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_IPF.h

  Summary:
    SPI Flash Driver Interface Definition

  Description:
    The SPI Flash device driver provides a simple interface to manage the SPI
    Flash modules which are external to Microchip Controllers.
    This file defines the interface definition for the SPI Flash Driver.
******************************************************************************/

//DOM-IGNORE-BEGIN
/******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#ifndef _DRV_IPF_H
#define _DRV_IPF_H


// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
/* Note:  A file that maps the interface definitions above to appropriate static
          implementations (depending on build mode) is included at the end of
          this file.
*/

#include "driver/driver_common.h"

#include "system/system.h"

#include "system/int/sys_int.h"
#include "system/ports/sys_ports.h"
#include "driver/spi/drv_spi.h"
#include "system/fs/sys_fs_media_manager.h"

#include "osal/osal.h"



// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver SPI Flash Module Index reference

  Summary:
    SPI Flash driver index definitions

  Description:
    These constants provide IPF SPI Flash driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_IPF_Initialize and
    DRV_IPF_Open routines to identify the driver instance in use.
*/

#define      DRV_IPF_INDEX_0      0


// *****************************************************************************
/* SPI Flash Driver Block Command Handle

  Summary:
    Handle identifying block commands of the driver.

  Description:
    A block command handle is returned by a call to the Read, Write, or Erase
    functions. This handle allows the application to track the completion of
    the operation. The handle is returned back to the client by the "event
    handler callback" function registered with the driver.

    The handle assigned to a client request expires when the client has
    been notified of the completion of the operation (after event handler
    function that notifies the client returns) or after the buffer has been
    retired by the driver if no event handler callback was set.

  Remarks:
    None.
*/

typedef SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE  DRV_IPF_BLOCK_COMMAND_HANDLE;

// *****************************************************************************
/* SPI Flash Driver Block Event Invalid Handle

  Summary:
    This value defines the SPI Flash Driver Block Command Invalid handle.

  Description:
    This value defines the SPI Flash Driver Block Command Invalid handle. It is
    returned by read/write/erase routines when the request could not be taken.

  Remarks:
    None.
*/

#define DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID   /*DOM-IGNORE-BEGIN*/ SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID /*DOM-IGNORE-END*/

// *****************************************************************************
/* IPF SPI Flash Driver Events

   Summary
    Identifies the possible events that can result from a request.

   Description
    This enumeration identifies the possible events that can result from a
    Read, Write, or Erase request caused by the client.

   Remarks:
    One of these values is passed in the "event" parameter of the event
    handling callback function that client registered with the driver by
    calling the DRV_IPF_BlockEventHandlerSet function when a block
    request is completed.
*/

typedef enum
{
    /* Block operation has been completed successfully. */
    /* Read/Write/Erase Complete */
    DRV_IPF_EVENT_BLOCK_COMMAND_COMPLETE
       /*DOM-IGNORE-BEGIN*/ = SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_COMPLETE /*DOM-IGNORE-END*/,

    /* There was an error during the block operation */
    /* Read/Write/Erase Error */
    DRV_IPF_EVENT_BLOCK_COMMAND_ERROR
       /*DOM-IGNORE-BEGIN*/ = SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_ERROR /*DOM-IGNORE-END*/

} DRV_IPF_BLOCK_EVENT;

// ***********************************************************************
/* IPF Driver Command Status

  Summary:
    Specifies the status of the command for the read, write and erase
    operations.
	
  Description:
    IPF Driver command Status
    
    This type specifies the status of the command for the read, write and
    erase operations.
	
  Remarks:
    None.                                                               
*/  
typedef enum
{
    /*Done OK and ready */
    DRV_IPF_COMMAND_COMPLETED          = SYS_FS_MEDIA_COMMAND_COMPLETED,

    /*Scheduled but not started */
    DRV_IPF_COMMAND_QUEUED             = SYS_FS_MEDIA_COMMAND_QUEUED,

    /*Currently being in transfer */
    DRV_IPF_COMMAND_IN_PROGRESS        = SYS_FS_MEDIA_COMMAND_IN_PROGRESS,

    /*Unknown Command */
    DRV_IPF_COMMAND_ERROR_UNKNOWN      = SYS_FS_MEDIA_COMMAND_UNKNOWN,

} DRV_IPF_COMMAND_STATUS;

// *****************************************************************************
/* IPF SPI Flash Driver Event Handler Function Pointer

   Summary:
    Pointer to a IPF SPI Flash Driver Event handler function.
	<p><b>Implementation:</b> Dynamic</p>

   Description:
    This data type defines the required function signature for the IPF
    SPI Flash driver event handling callback function. A client must register
    a pointer to an event handling function whose function signature (parameter
    and return value types) match the types specified by this function pointer
    in order to receive event calls back from the driver.

    The parameters and return values and return value are described here and
    a partial example implementation is provided.

  Parameters:
    event           - Identifies the type of event

    commandHandle   - Handle returned from the Read/Write/Erase requests

    context         - Value identifying the context of the application that
                      registered the event handling function

  Returns:
    None.

  Example:
    <code>
    void APP_MyBufferEventHandler
    (
        DRV_IPF_BLOCK_EVENT event,
        DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle,
        uintptr_t context
    )
    {
        MY_APP_DATA_STRUCT pAppData = (MY_APP_DATA_STRUCT) context;

        switch(event)
        {
            case DRV_IPF_EVENT_BLOCK_COMMAND_COMPLETE:

                // Handle the completed buffer.
                break;

            case DRV_IPF_EVENT_BLOCK_COMMAND_ERROR:
            default:

                // Handle error.
                break;
        }
    }
    </code>

  Remarks:
    If the event is DRV_IPF_EVENT_BLOCK_COMMAND_COMPLETE, it means that the
    data was transferred successfully.

    If the event is DRV_IPF_EVENT_BLOCK_COMMAND_ERROR, it means that the data
    was not transferred successfully.

    The context parameter contains the a handle to the client context,
    provided at the time the event handling function was  registered using the
    DRV_IPF_BlockEventHandlerSet function. This context handle value is
    passed back to the client as the "context" parameter.  It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) instance of the client that made the read/write/erase
    request.

    The event handler function executes in the driver peripheral's interrupt
    context when the driver is configured for interrupt mode operation. It is
    recommended of the application to not perform process intensive or blocking
    operations with in this function.

    The Read, Write, and Erase functions can be called in the event handler to
    add a buffer to the driver queue. These functions can only be called to add
    buffers to the driver whose event handler is running.
*/

typedef void ( *DRV_IPF_EVENT_HANDLER )
(
    DRV_IPF_BLOCK_EVENT event,
    DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle,
    uintptr_t context
);

// *****************************************************************************
/* SPI Flash Client Status

  Summary:
    Defines the client status.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    Defines the various client status codes.

  Remarks:
    None.
*/

typedef enum
{
    /* Up and running, ready to start new operations */
    DRV_IPF_CLIENT_STATUS_READY = DRV_CLIENT_STATUS_READY + 0,

    /* Operation in progress, unable to start a new one */
    DRV_IPF_CLIENT_STATUS_BUSY = DRV_CLIENT_STATUS_BUSY,

    /* Client is closed */
    DRV_IPF_CLIENT_STATUS_CLOSED = DRV_CLIENT_STATUS_CLOSED,

    /* Client Error */
    DRV_IPF_CLIENT_STATUS_ERROR = DRV_CLIENT_STATUS_ERROR

} DRV_IPF_CLIENT_STATUS;

// *****************************************************************************
/* SST SPI Flash Driver Initialization Data

  Summary:
    Contains all the data necessary to initialize the SPI Flash device.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This structure contains all of the data necessary to initialize the SPI
    Flash device.

  Remarks:
    A pointer to a structure of this format containing the desired
    initialization data must be passed into the DRV_IPF_Initialize
    function.
*/

typedef struct
{
    /* System module initialization */
    SYS_MODULE_INIT                     moduleInit;

    /* Identifies the SPI driver to be used */
    SYS_MODULE_INDEX                    spiDriverModuleIndex;

    /* HOLD pin port channel */
    PORTS_CHANNEL                       holdPortChannel;

    /* HOLD pin port position*/
    PORTS_BIT_POS                       holdBitPosition;

    /* Write protect pin port channel */
    PORTS_CHANNEL                       writeProtectPortChannel;

    /* Write Protect Bit pin position */
    PORTS_BIT_POS                       writeProtectBitPosition;

    /* Chip select pin port channel */
    PORTS_CHANNEL                       chipSelectPortChannel;

    /* Chip Select Bit pin position */
    PORTS_BIT_POS                       chipSelectBitPosition;

    /* This is the buffer queue size. This is the maximum
       number of requests that this instance of the driver will
	   queue. For a static build of the driver, this is overridden by the
       DRV_IPF_QUEUE_SIZE macro in system_config.h */
    uint32_t queueSize;

} DRV_IPF_INIT;

// *****************************************************************************
/* IPF Driver Operations

  Summary
    Lists the different operations that IPF driver can do.

  Description
    This enumeration lists the different operations that IPF driver can
    do.

  Remarks:
    None.
*/

typedef enum
{
    /* Block Read */
    DRV_IPF_BLOCK_READ,

    /* Block Write */
    DRV_IPF_BLOCK_WRITE,

    /* Block Erase */
    DRV_IPF_BLOCK_ERASE,
	
	/* Hardware Block Protection */
	DRV_IPF_HW_BLOCK_PROT,
    
    /* Hardware Block Un-Protection */
    DRV_IPF_HW_BLOCK_UNPROT,
    
   /* Read HW Block Protection Status*/
    DRV_IPF_READ_HW_BLOCK_PROT

} DRV_IPF_BLOCK_OPERATION;

// *****************************************************************************
/* IPF Driver memory protection modes

  Summary
    Lists the different memory protection modes.

  Description
    This enumeration lists the different memory protection modes.

  Remarks:
    None.
*/

typedef enum
{
	/* Write Protect */
	DRV_IPF_WRITE_PROTECT = 1,
	
	/* Read Protect */
	DRV_IPF_READ_PROTECT
	
} DRV_IPF_PROT_MODE;


// *****************************************************************************
// *****************************************************************************
// Section: SPI Flash Driver Module Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_IPF_Initialize
    (
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init
    );

  Summary:
    Initializes the IPF SPI Flash Driver instance for the specified
    driver index.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function initializes the SPI Flash driver instance for the specified
    driver index, making it ready for clients to open and use it.

  Precondition:
    None.

  Parameters :
    index -  Identifier for the instance to be initialized

    init -   Pointer to a data structure containing data necessary to
             initialize the driver.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.

  Example:
    <code>
    // This code snippet shows an example of initializing the IPF SPI
    // Flash Driver. SPI driver index 0 is used for the purpose. Pin numbers 1, 2
	// and 3 of port channel B are configured for hold pin, write protection pin
	// and chip select pin respectively. Maximum buffer queue size is set 5.

    DRV_IPF_INIT   IPFInitData;
    SYS_MODULE_OBJ      objectHandle;

    IPFInitData.moduleInit.value      = SYS_MODULE_POWER_RUN_FULL;
    IPFInitData.spiDriverModuleIndex  = DRV_SPI_INDEX_0;
    IPFInitData.holdPortChannel       = PORT_CHANNEL_B;
    IPFInitData.holdBitPosition       = PORTS_BIT_POS_1;
    IPFInitData.writeProtectPortChannel = PORT_CHANNEL_B;
    IPFInitData.writeProtectBitPosition = PORTS_BIT_POS_2;
    IPFInitData.chipSelectPortChannel = PORT_CHANNEL_F;
    IPFInitData.chipSelectBitPosition = PORTS_BIT_POS_2;
    IPFInitData.queueSize = 5;

    objectHandle = DRV_IPF_Initialize(DRV_IPF_INDEX_0,
                                    (SYS_MODULE_INIT*)IPFInitData);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This function must be called before any other SPI Flash function is called.

    This function should only be called once during system initialization
    unless DRV_IPF_Deinitialize is called to deinitialize the driver
    instance.

    Build configuration options may be used to statically override options
    in the "init" structure and will take precedence over initialization
    data passed using this function.
*/

SYS_MODULE_OBJ DRV_IPF_Initialize
(
    const SYS_MODULE_INDEX index,
    const SYS_MODULE_INIT * const init
);

//******************************************************************************
/* Function:
    void DRV_IPF_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the SPI Flash driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    Deinitializes the specified instance of the SPI Flash Driver module,
    disabling its operation (and any hardware) and invalidates all of the
    internal data.

  Precondition:
    Function DRV_IPF_Initialize should have been called before calling
    this function.

  Parameters:
    object -  Driver object handle, returned from the DRV_IPF_Initialize
              function
  Returns:
    None.

  Example:
    <code>
    // This code snippet shows an example of deinitializing the driver.

    SYS_MODULE_OBJ      object;     //  Returned from DRV_IPF_Initialize
    SYS_STATUS          status;


    DRV_IPF_Deinitialize(object);

    status = DRV_IPF_Status(object);
    if (SYS_MODULE_DEINITIALIZED != status)
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize
    operation must be called before the Initialize operation can be called
    again. This function will NEVER block waiting for hardware.
*/

void DRV_IPF_Deinitialize( SYS_MODULE_OBJ object);

//*************************************************************************
/* Function:
    SYS_STATUS DRV_IPF_Status( SYS_MODULE_OBJ object )

  Summary:
    Gets the current status of the SPI Flash Driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function provides the current status of the SPI Flash Driver module.

  Precondition:
    Function DRV_IPF_Initialize should have been called before calling
    this function.

  Parameters:
    object -  Driver object handle, returned from the DRV_IPF_Initialize
              function
  Returns:
    SYS_STATUS_READY - Indicates that the driver is ready and accept requests
                       for new operations

    SYS_STATUS_UNINITIALIZED - Indicates that the driver is not initialized

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_IPF_Initialize
    SYS_STATUS          IPFStatus;

    IPFStatus = DRV_IPF_Status(object);
    else if (SYS_STATUS_ERROR >= IPFStatus)
    {
        // Handle error
    }
    </code>

  Remarks:
    A driver can only be opened when its status is SYS_STATUS_READY.

*/

SYS_STATUS DRV_IPF_Status( SYS_MODULE_OBJ object);

// ***************************************************************************
/* Function:
    void DRV_IPF_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's read, erase, and write state machine and implements
    its ISR.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function is used to maintain the driver's internal state machine
    and should be called from the system's Tasks function.

  Precondition:
    The DRV_IPF_Initialize function must have been called for the
    specified SPI Flash driver instance.

  Parameters:
    object -  Object handle for the specified driver instance (returned from
              DRV_IPF_Initialize)
  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_IPF_Initialize

    while (true)
    {
        DRV_IPF_Tasks (object);

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application. It is
    called by the system's Tasks function (SYS_Tasks).
*/

void DRV_IPF_Tasks ( SYS_MODULE_OBJ object );

// *****************************************************************************
// *****************************************************************************
// Section: SPI Flash Driver Client Routines
// *****************************************************************************
// *****************************************************************************

/* Function:
    DRV_HANDLE DRV_IPF_Open
    (
        const SYS_MODULE_INDEX drvIndex,
        const DRV_IO_INTENT ioIntent
    );

  Summary:
    Opens the specified SPI Flash driver instance and returns a handle to it.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function opens the specified SPI Flash driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver.

  Precondition:
    Function DRV_IPF_Initialize must have been called before calling
    this function.

  Parameters:
    drvIndex -  Identifier for the object instance to be opened
    ioIntent -  Zero or more of the values from the enumeration
                DRV_IO_INTENT "ORed" together to indicate the intended use
                of the driver

  Returns:
    If successful, the function returns a valid open-instance handle (a
    number identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID. Errors can occur
	under the following circumstances:
    - if the number of client objects allocated via
         DRV_IPF_CLIENTS_NUMBER is insufficient
    - if the client is trying to open the driver but driver has been opened
         exclusively by another client
    - if the driver hardware instance being opened is not initialized or is
         invalid
    - if the client is trying to open the driver exclusively, but has already
         been opened in a non exclusive mode by another client.
    - if the driver status is not ready.

    The driver status becomes ready inside "DRV_IPF_Tasks" function. To
    make the SST Driver status ready and hence successfully "Open" the driver,
    "Task" routine need to be called periodically.

  Example:
    <code>
    DRV_HANDLE handle;

    handle = DRV_IPF_Open(DRV_IPF_INDEX_0,
                                                    DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
    }
    </code>

  Remarks:
    The driver will always work in Non-Blocking mode even if IO-intent is
    selected as blocking.

    The handle returned is valid until the DRV_IPF_Close function is
    called.

    This function will NEVER block waiting for hardware.
*/

DRV_HANDLE DRV_IPF_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
);

// *****************************************************************************
/* Function:
    void DRV_IPF_Close( DRV_Handle handle );

  Summary:
    Closes an opened-instance of the SPI Flash driver.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function closes an opened-instance of the SPI Flash driver, invalidating
    the handle.

  Precondition:
    The DRV_IPF_Initialize function must have been called for the
    specified SPI Flash driver instance.

    DRV_IPF_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_IPF_Open

    DRV_IPF_Close(handle);
    </code>

  Remarks:
    After calling this function, the handle passed in "handle" must not be used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling DRV_IPF_Open before the caller may use the driver again.

    Note: Usually, there is no need for the driver client to verify that the
    Close operation has completed.
*/

void DRV_IPF_Close( const DRV_HANDLE handle);

// ****************************************************************************
/* Function:
    DRV_IPF_CLIENT_STATUS DRV_IPF_ClientStatus(DRV_HANDLE handle);

  Summary:
    Gets current client-specific status of the SPI Flash driver.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function gets the client-specific status of the SPI Flash driver
    associated with the given handle.

  Precondition:
    The DRV_IPF_Initialize function must have been called.

    DRV_IPF_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle -  A valid open instance handle, returned from the driver's open
              function

  Returns:
    A DRV_IPF_CLIENT_STATUS value describing the current status of the
    driver.

  Example:
    <code>
    DRV_HANDLE      handle;         // Returned from DRV_IPF_Open
    DRV_IPF_CLIENT_STATUS     clientStatus;

    clientStatus = DRV_IPF_ClientStatus(handle);
    if(DRV_IPF_CLIENT_STATUS_READY == clientStatus)
    {
        // do the tasks
    }
    </code>

  Remarks:
    This function will not block for hardware access and will immediately
    return the current status.
*/

DRV_IPF_CLIENT_STATUS   DRV_IPF_ClientStatus( const DRV_HANDLE handle );

// *****************************************************************************
/* Function:
    void DRV_IPF_BlockEventHandlerSet
    (
        const DRV_HANDLE handle,
        const DRV_IPF_EVENT_HANDLER eventHandler,
        const uintptr_t context
    );

  Summary:
    Allows a client to identify an event handling function for the driver to
    call back when queued operation has completed.
	<p><b>Implementation:</b> Dynamic</p>

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

  Precondition:
    The DRV_IPF_Initialize function must have been called for the
    specified SPI FLash driver instance.

    DRV_IPF_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

    eventHandler - Pointer to the event handler function implemented by the user

    context      - The value of parameter will be passed back to the client
                   unchanged, when the eventHandler function is called. It can
                   be used to identify any client specific data object that
                   identifies the instance of the client module (for example,
                   it may be a pointer to the client module's state structure).

  Returns:
    None.

  Example:
    <code>
    // myAppObj is an application specific state data object.
    MY_APP_OBJ myAppObj;

    uint8_t myBuffer[MY_BUFFER_SIZE];
    uint32_t blockStart, nBlock;
    DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle;

    // myIPFHandle is the handle returned
    // by the DRV_IPF_Open function.

    // Client registers an event handler with driver. This is done once.

    DRV_IPF_BlockEventHandlerSet( myIPFHandle,
                    APP_IPFEventHandler, (uintptr_t)&myAppObj );

    DRV_IPF_BlockRead( myIPFHandle, commandHandle,
                                            &myBuffer, blockStart, nBlock );

    if(DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event Processing Technique. Event is received when operation is done.

    void APP_IPFEventHandler(DRV_IPF_BLOCK_EVENT event,
            DRV_IPF_BLOCK_COMMAND_HANDLE handle, uintptr_t context)
    {
        // The context handle was set to an application specific
        // object. It is now retrievable easily in the event handler.
        MY_APP_OBJ myAppObj = (MY_APP_OBJ *) context;

        switch(event)
        {
            case DRV_IPF_EVENT_BLOCK_COMMAND_COMPLETE:

                // This means the data was transferred.
                break;

            case DRV_IPF_EVENT_BLOCK_COMMAND_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }
    </code>

  Remarks:
    If the client does not want to be notified when the queued operation
    has completed, it does not need to register a callback.
*/

void DRV_IPF_BlockEventHandlerSet
(
    const DRV_HANDLE handle,
    const DRV_IPF_EVENT_HANDLER eventHandler,
    const uintptr_t context
);

// **************************************************************************
/* Function:
    void DRV_IPF_BlockErase
    (
        const DRV_HANDLE handle,
        DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Erase the specified number of blocks in Flash memory.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function schedules a non-blocking erase operation in flash memory.
    The function returns with a valid erase handle in the commandHandle argument
    if the erase request was scheduled successfully. The function adds the
    request to the hardware instance queue and returns immediately.
    The function returns DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID in the
    commandHandle argument under the following circumstances:
    - if the client opened the driver for read only
    - if nBlock is 0
    - if the queue size is full or queue depth is insufficient
    - if the driver handle is invalid
    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_IPF_EVENT_ERASE_COMPLETE event if the
    erase operation was successful or DRV_IPF_EVENT_ERASE_ERROR
    event if the erase operation was not successful.

  Precondition:
    The DRV_IPF_Initialize function must have been called for the
    specified SPI Flash driver instance.

    DRV_IPF_Open must have been called to obtain a valid opened device
    handle.

    DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified
    in the DRV_IPF_Open call.

  Parameters:
    handle -       A valid open-instance handle, returned from the
                   driver's open function
    commandHandle -  Pointer to an argument that will contain the return buffer
                   handle
    blockStart -   Start block address in IPF memory from where the
                   erase should begin. LSBs (A0-A11) of block start address
                   will be ignored to align it with Erase block size boundary.

    nBlock -       Total number of blocks to be erased. Each Erase block is of
                   size 4 KByte.

  Returns:
    The buffer handle is returned in the commandHandle argument. It Will be
    DRV_BUFFER_HANDLE_INVALID if the request was not queued.

  Example:
    <code>

    // Destination address should be block aligned.
    uint32_t blockStart;
    uint32_t nBlock;
    DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;

    // myIPFHandle is the handle returned
    // by the DRV_IPF_Open function.

    // Client registers an event handler with driver

    DRV_IPF_BlockEventHandlerSet(myIPFHandle,
                    APP_IPFEventHandler, (uintptr_t)&myAppObj);

    DRV_IPF_BlockErase( myIPFHandle, commandHandle,
                                                        blockStart, nBlock );

    if(DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer queue is processed.

    void APP_IPFEventHandler(DRV_IPF_BLOCK_EVENT event,
            DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_IPF_EVENT_ERASE_COMPLETE:

                // This means the data was transferred.
                break;

            case DRV_IPF_EVENT_ERASE_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    Write Protection will be disabled for the complete flash memory region
    in the beginning by default.
*/

void DRV_IPF_BlockErase
(
    const DRV_HANDLE handle,
    DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle,
    uint32_t blockStart,
    uint32_t nBlock
);

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
    Reads blocks of data starting from the specified address in Flash memory.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function schedules a non-blocking read operation for reading blocks of
    data from flash memory. The function returns with a valid handle in
    the commandHandle argument if the read request was scheduled successfully.
    The function adds the request to the hardware instance queue and
    returns immediately. While the request is in the queue, the application
    buffer is owned by the driver and should not be modified.  The function
    returns DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID in the commandHandle
    argument under the following circumstances:
    - if a buffer could not be allocated to the request
    - if the target buffer pointer is NULL
    - if the client opened the driver for write only
    - if the buffer size is 0
    - if the read queue size is full or queue depth is insufficient
    - if the driver handle is invalid
    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_IPF_EVENT_BLOCK_COMMAND_COMPLETE event if the
    buffer was processed successfully of DRV_IPF_EVENT_BLOCK_COMMAND_ERROR
    event if the buffer was not processed successfully.

  Precondition:
    The DRV_IPF_Initialize function must have been called for the
    specified SPI Flash driver instance.

    DRV_IPF_Open must have been called to obtain a valid opened device
    handle.

    DRV_IO_INTENT_READ or DRV_IO_INTENT_READWRITE must have been specified in
    the DRV_IPF_Open call.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle

    *targetBuffer - Buffer into which the data read from the SPI Flash instance
                    will be placed

    blockStart    -  Start block address in IPF memory from where the
                     read should begin. It can be any address of the flash.

    nBlock        -  Total number of blocks to be read. Each Read block is of 1
                     byte.

  Returns:
    The buffer handle is returned in the commandHandle argument. It will be
    DRV_BUFFER_HANDLE_INVALID if the request was not successful.

  Example:
    <code>

    uint8_t myBuffer[MY_BUFFER_SIZE];

    // address should be block aligned.
    uint32_t blockStart = IPF_BASE_ADDRESS_TO_READ_FROM;
    uint32_t    nBlock = 2;
    DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;

    // myIPFHandle is the handle returned
    // by the DRV_IPF_Open function.

    // Client registers an event handler with driver

    DRV_IPF_BlockEventHandlerSet(myIPFHandle,
                    APP_IPFEventHandler, (uintptr_t)&myAppObj);

    DRV_IPF_BlockRead( myIPFHandle, commandHandle,
                                            &myBuffer, blockStart, nBlock );

    if(DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when the buffer is processed.

    void APP_IPFEventHandler(DRV_IPF_BLOCK_EVENT event,
            DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_IPF_EVENT_BLOCK_COMMAND_COMPLETE:

                // This means the data was transferred.
                break;

            case DRV_IPF_EVENT_BLOCK_COMMAND_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    The maximum read speed is 33 MHz.
*/

void DRV_IPF_BlockRead
(
    const DRV_HANDLE handle,
    DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle,
    uint8_t *targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
);

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
    Write blocks of data starting from a specified address in Flash memory.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function schedules a non-blocking write operation for writing blocks of
    data into flash memory. The function returns with a valid buffer handle in
    the commandHandle argument if the write request was scheduled successfully.
    The function adds the request to the hardware instance queue and
    returns immediately. While the request is in the queue, the application
    buffer is owned by the driver and should not be modified.  The function
    returns DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID in the commandHandle
    argument under the following circumstances:
    - if a buffer could not be allocated to the request
    - if the input buffer pointer is NULL
    - if the client opened the driver for read only
    - if the buffer size is 0
    - if the write queue size is full or queue depth is insufficient
    - if the driver handle is invalid
    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_IPF_EVENT_BLOCK_COMMAND_COMPLETE event if the
    buffer was processed successfully or DRV_IPF_EVENT_BLOCK_COMMAND_ERROR
    event if the buffer was not processed successfully.

  Precondition:
    The DRV_IPF_Initialize function must have been called for the
    specified SPI Flash driver instance.

    DRV_IPF_Open must have been called to obtain a valid opened
    device handle.

    DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have been specified in
    the DRV_IPF_Open call.

    The flash address location which has to be written, must be erased before
    using the API DRV_IPF_BlockErase().

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

    commandHandle -Pointer to an argument that will contain the return buffer
                   handle

    sourceBuffer - The source buffer containing data to be programmed into SPI
                   Flash

    blockStart -   Start block address of IPF Flash where the write
                   should begin. It can be any address of the flash.
    nBlock -       Total number of blocks to be written. Each write block is of
                   1 byte.

  Returns:
    The buffer handle is returned in the commandHandle argument. It will be
    DRV_BUFFER_HANDLE_INVALID if the request was not successful.

  Example:
    <code>

    uint8_t myBuffer[MY_BUFFER_SIZE];

    // address should be block aligned.
    uint32_t blockStart = IPF_BASE_ADDRESS_TO_WRITE_TO;
    uint32_t    nBlock = 2;
    DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;

    // myIPFHandle is the handle returned
    // by the DRV_IPF_Open function.

    // Client registers an event handler with driver

    DRV_IPF_BlockEventHandlerSet(myIPFHandle,
                    APP_IPFEventHandler, (uintptr_t)&myAppObj);

    DRV_IPF_BlockWrite( myIPFHandle, commandHandle,
                                            &myBuffer, blockStart, nBlock );

    if(DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_IPFEventHandler(DRV_IPF_BLOCK_EVENT event,
            DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_IPF_EVENT_BLOCK_COMMAND_COMPLETE:

                // This means the data was transferred.
                break;

            case DRV_IPF_EVENT_BLOCK_COMMAND_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    In the case of multi bytes write operation, byte by byte writing will happen
    instead of Address auto Increment writing.

    Write Protection will be disabled for the complete flash memory region
    in the beginning by default.
*/

void DRV_IPF_BlockWrite
(
    DRV_HANDLE handle,
    DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle,
    uint8_t *sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
);

// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_GEOMETRY DRV_IPF_GeometryGet( DRV_HANDLE handle );

  Summary:
    Returns the geometry of the device.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This API gives the following geometrical details of the IPF Flash:
    - Media Property
    - Number of Read/Write/Erase regions in the flash device
    - Number of Blocks and their size in each region of the device

  Precondition:
    None.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    SYS_FS_MEDIA_GEOMETRY - Structure which holds the media geometry information.

  Example:
    <code>

    SYS_FS_MEDIA_GEOMETRY * sstFlashGeometry;
    uint32_t readBlockSize, writeBlockSize, eraseBlockSize;
    uint32_t nReadBlocks, nReadRegions, totalFlashSize;

    sstFlashGeometry = DRV_IPF_GeometryGet(sstOpenHandle1);

    // read block size should be 1 byte
    readBlockSize  = sstFlashGeometry->geometryTable->blockSize;
    nReadBlocks = sstFlashGeometry->geometryTable->numBlocks;
    nReadRegions = sstFlashGeometry->numReadRegions;

    // write block size should be 1 byte
    writeBlockSize  = (sstFlashGeometry->geometryTable +1)->blockSize;
    // erase block size should be 4k byte
    eraseBlockSize  = (sstFlashGeometry->geometryTable +2)->blockSize;

    // total flash size should be 256k byte
    totalFlashSize = readBlockSize * nReadBlocks * nReadRegions;

    </code>

  Remarks:
    This function is typically used by File System Media Manager.
*/

SYS_FS_MEDIA_GEOMETRY * DRV_IPF_GeometryGet( DRV_HANDLE handle );

// *****************************************************************************
/* Function:
    bool DRV_IPF_MediaIsAttached(DRV_HANDLE handle);

  Summary:
    Returns the status of the media.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This API tells if the media is attached or not.

  Precondition:
    None.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    - True         - Media is attached
    - False        - Media is not attached

  Example:
    <code>

    if (DRV_IPF_MediaIsAttached(handle))
    {
    // Do Something
    }

    </code>

  Remarks:
    This function is typically used by File System Media Manager.
*/

bool DRV_IPF_MediaIsAttached(DRV_HANDLE handle);

// *****************************************************************************
/* Function:
    void DRV_IPF_WPAssert();

  Summary:
    Asserts the WP pin for flash.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This API is used to assert the Write Protect (WP) pin of the in-package flash.

  Precondition:
    None.

  Parameters:
	None.

  Returns:
	None.

  Example:
    <code>
	DRV_IPF_WPAssert();
    </code>

  Remarks:
	The Write Protection GPIO is fixed in case of PIC32WK devices.
*/

void DRV_IPF_WPAssert();

// *****************************************************************************
/* Function:
    void DRV_IPF_WPAssert();

  Summary:
    Deasserts the WP pin for flash.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This API is used to deassert the Write Protect (WP) pin of the in-package flash.

  Precondition:
    None.

  Parameters:
	None.

  Returns:
	None.

  Example:
    <code>
	DRV_IPF_WPDeAssert();
    </code>

  Remarks:
	The Write Protection GPIO is fixed in case of PIC32WK devices.
*/

void DRV_IPF_WPDeAssert();

// *****************************************************************************
/* Function:
    void DRV_IPF_HoldAssert();

  Summary:
    Asserts the Hold pin for flash.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This API is used to assert the Hold pin of the in-package flash.

  Precondition:
    None.

  Parameters:
	None.

  Returns:
	None.

  Example:
    <code>
	DRV_IPF_HoldAssert();
    </code>

  Remarks:
	The Hold GPIO is fixed in case of PIC32WK devices.
*/

void DRV_IPF_HoldAssert();

// *****************************************************************************
/* Function:
    void DRV_IPF_HoldDeAssert();

  Summary:
    Deasserts the Hold pin for flash.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This API is used to deassert the Hold pin of the in-package flash.

  Precondition:
    None.

  Parameters:
	None.

  Returns:
	None.

  Example:
    <code>
	DRV_IPF_HoldDeAssert();
    </code>

  Remarks:
	The Hold GPIO is fixed in case of PIC32WK devices.
*/

void DRV_IPF_HoldDeAssert();

// *****************************************************************************
/* Function:
	void DRV_IPF_ProtectMemoryVolatile
	(
		DRV_HANDLE clientHandle, 
		DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle, 
		uintptr_t memAddress, 
		DRV_IPF_PROT_MODE protMode
	);
  Summary:
    Protects the memory block to which the given memory address belongs
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This API is used to protect the memory block to which a given memory address
	belongs. Both read and write protection mode is supported. The memory will be
	protected until the next power cycle.

  Precondition:
    In-package flash driver open function must be called and a valid client handle
	must be available.

  Parameters:
	clientHandle 	- A valid open-instance handle, returned from the driver's
                   open function
	commandHandle 	- Pointer to an argument that will contain the return buffer
                   handle
	memAddress 		- Memory address which belongs to the memory block which needs 
					to be protected
	protMode 		- Read or write protect mode. If a block needs to be protected
					for both read and write, then both enum values can be ORed and
					passed to the function.
	
  Returns:
	None.

  Example:
    <code>
    uintptr_t memAddr = IPF_ADDRESS_PROTECT;
    DRV_IPF_PROT_MODE    protMode = DRV_IPF_WRITE_PROTECT;
    DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;

    // myIPFHandle is the handle returned
    // by the DRV_IPF_Open function.

    // Client registers an event handler with driver

    DRV_IPF_BlockEventHandlerSet(myIPFHandle,
                    APP_IPFEventHandler, (uintptr_t)&myAppObj);

    DRV_IPF_ProtectMemoryVolatile( myIPFHandle, commandHandle,
                                            memAddr, protMode );

    if(DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_IPFEventHandler(DRV_IPF_BLOCK_EVENT event,
            DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_IPF_EVENT_BLOCK_COMMAND_COMPLETE:

                // This means the memory protection is complete.
                break;

            case DRV_IPF_EVENT_BLOCK_COMMAND_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }
    </code>

  Remarks:
	Only the selected blocks can be read protected, which is as per the 
	in-package flash specification.
*/

void DRV_IPF_ProtectMemoryVolatile
(
    DRV_HANDLE clientHandle, 
    DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle, 
    uintptr_t memAddress, 
    DRV_IPF_PROT_MODE protMode
);

// *****************************************************************************
/* Function:
	void DRV_IPF_UnProtectMemoryVolatile
	(
		DRV_HANDLE clientHandle, 
		DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle, 
		uintptr_t memAddress, 
		DRV_IPF_PROT_MODE protMode
	);
  Summary:
    Un-protects the memory block to which the given memory address belongs
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This API is used to un-protect the memory block to which a given memory address
	belongs. Both read and write protection mode is supported. The memory will be
	protected until the next power cycle.

  Precondition:
    In-package flash driver open function must be called and a valid client handle
	must be available.

  Parameters:
	clientHandle 	- A valid open-instance handle, returned from the driver's
                   open function
	commandHandle 	- Pointer to an argument that will contain the return buffer
                   handle
	memAddress 		- Memory address which belongs to the memory block which needs 
					to be un-protected
	protMode 		- Read or write protect mode. If a block needs to be un-protected
					for both read and write, then both enum values can be ORed and
					passed to the function.
	
  Returns:
	None.

  Example:
    <code>
    uintptr_t memAddr = IPF_ADDRESS_UNPROTECT;
    DRV_IPF_PROT_MODE    protMode = DRV_IPF_WRITE_PROTECT;
    DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;

    // myIPFHandle is the handle returned
    // by the DRV_IPF_Open function.

    // Client registers an event handler with driver

    DRV_IPF_BlockEventHandlerSet(myIPFHandle,
                    APP_IPFEventHandler, (uintptr_t)&myAppObj);

    DRV_IPF_UnProtectMemoryVolatile( myIPFHandle, commandHandle,
                                            memAddr, protMode );

    if(DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_IPFEventHandler(DRV_IPF_BLOCK_EVENT event,
            DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_IPF_EVENT_BLOCK_COMMAND_COMPLETE:

                // This means the memory unprotection is complete.
                break;

            case DRV_IPF_EVENT_BLOCK_COMMAND_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }
    </code>

  Remarks:
	If the memory block a client is trying to unprotect, is protected by some other
	client, then memory unprotection will not executed. The function will return 
	without	unprotecting.
*/

void DRV_IPF_UnProtectMemoryVolatile
(
    DRV_HANDLE clientHandle, 
    DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle, 
    uintptr_t memAddress, 
    DRV_IPF_PROT_MODE protMode
);

// *****************************************************************************
/* Function:
	void DRV_IPF_ReadBlockProtectionStatus
	(
		DRV_HANDLE clientHandle, 
		DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle, 
		uint8_t * buffer
	);
  Summary:
    Reads the content of Block Protection Register which belongs to In-Package 
	flash.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This API is read the current contents of the block protection register in 
	in-package flash and fills the buffer passed by the client. 

  Precondition:
    In-package flash driver open function must be called and a valid client handle
	must be available.

  Parameters:
	clientHandle 	- A valid open-instance handle, returned from the driver's
                   open function
	commandHandle 	- Pointer to an argument that will contain the return buffer
                   handle
	buffer  		- pointer to a buffer to which the block protection status
					has to be updated
	
  Returns:
	None.

  Example:
    <code>
    uint8_t buf[6] = {0,};
    DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;

    // myIPFHandle is the handle returned
    // by the DRV_IPF_Open function.

    // Client registers an event handler with driver

    DRV_IPF_BlockEventHandlerSet(myIPFHandle,
                    APP_IPFEventHandler, (uintptr_t)&myAppObj);

    DRV_IPF_ReadBlockProtectionStatus( myIPFHandle, commandHandle,
                                            buf );

    if(DRV_IPF_BLOCK_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when
    // the buffer is processed.

    void APP_IPFEventHandler(DRV_IPF_BLOCK_EVENT event,
            DRV_IPF_BLOCK_COMMAND_HANDLE commandHandle, uintptr_t contextHandle)
    {
        // contextHandle points to myAppObj.

        switch(event)
        {
            case DRV_IPF_EVENT_BLOCK_COMMAND_COMPLETE:

                // This means the BPR read is complete.
                break;

            case DRV_IPF_EVENT_BLOCK_COMMAND_ERROR:

                // Error handling here.

                break;

            default:
                break;
        }
    }
    </code>

  Remarks:
	The block protection word is 6-bytes wide.
*/

void DRV_IPF_ReadBlockProtectionStatus
(
    DRV_HANDLE clientHandle, 
    DRV_IPF_BLOCK_COMMAND_HANDLE * commandHandle, 
    uint8_t * buffer
);


// ****************************************************************************
// ****************************************************************************
// Section: Included Files (continued)
// ****************************************************************************
// ****************************************************************************
/*  The files included below map the interface definitions above to appropriate
    static implementations, depending on build mode.
*/



#endif // #ifndef _DRV_IPF_H
/*******************************************************************************
 End of File
*/

