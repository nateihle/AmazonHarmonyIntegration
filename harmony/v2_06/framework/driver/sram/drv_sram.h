/*******************************************************************************
  SRAM Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sram.h

  Summary:
    SRAM Driver Interface Definition

  Description:
    The SRAM driver provides a simple interface to manage the SRAM Memory on
    Microchip microcontrollers. This file defines the interface definition for
    the SRAM driver.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc. All rights reserved.

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
#ifndef _DRV_SRAM_H
#define _DRV_SRAM_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
#include "system/common/sys_common.h"
#include "driver/driver_common.h"
#include "system/common/sys_module.h"
#include "osal/osal.h"
#include "system/fs/sys_fs_media_manager.h"
#include "system/devcon/sys_devcon.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
/* SRAM Driver command handle.

  Summary:
    Handle identifying commands queued in the driver.

  Description:
    A command handle is returned by a call to the Read or Write functions. This
    handle allows the application to track the completion of the operation.
    This command handle is also returned to the client along with the event
    that has occurred with respect to the command. This allows the application
    to connect the event to a specific command in case where multiple commands
    are queued.

    The command handle associated with the command request expires when the
    client has been notified of the completion of the command (after event
    handler function that notifies the client returns) or after the command has
    been retired by the driver if no event handler callback was set. 

  Remarks:
    None.
*/

typedef SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE DRV_SRAM_COMMAND_HANDLE;


// *****************************************************************************
/* SRAM Driver Invalid Command Handle.

  Summary:
    This value defines the SRAM Driver's Invalid Command Handle.

  Description:
    This value defines the SRAM Driver Invalid Command Handle. This value is
    returned by read/write routines when the command request is not accepted.

  Remarks:
    None.
*/

#define DRV_SRAM_COMMAND_HANDLE_INVALID SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID


// *****************************************************************************
/* SRAM Driver Events

  Summary
    Identifies the possible events that can result from a request.

  Description
    This enumeration identifies the possible events that can result from a 
    read or a write request caused by the client.

  Remarks:
    One of these values is passed in the "event" parameter of the event
    handling callback function that client registered with the driver by
    calling the DRV_SRAM_EventHandlerSet function when a request is completed.
*/

typedef enum
{
    /* Operation has been completed successfully. */
    DRV_SRAM_EVENT_COMMAND_COMPLETE = SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_COMPLETE,

    /* There was an error during the operation */
    DRV_SRAM_EVENT_COMMAND_ERROR = SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_ERROR 

} DRV_SRAM_EVENT;


// ***********************************************************************
/* SRAM Driver Command Status

  Summary:
    Specifies the status of the command for the read and write operations.
	
  Description:
    SRAM Driver command Status
    
    This type specifies the status of the command for the read and write
    operations.
	
  Remarks:
    None.                                                               
*/

typedef enum
{
    /* Done OK and ready */
    DRV_SRAM_COMMAND_COMPLETED     = SYS_FS_MEDIA_COMMAND_COMPLETED,

    /* Scheduled but not started */
    DRV_SRAM_COMMAND_QUEUED        = SYS_FS_MEDIA_COMMAND_QUEUED,

    /* Currently being in transfer */
    DRV_SRAM_COMMAND_IN_PROGRESS   = SYS_FS_MEDIA_COMMAND_IN_PROGRESS,

    /* Unknown Command */
    DRV_SRAM_COMMAND_ERROR_UNKNOWN = SYS_FS_MEDIA_COMMAND_UNKNOWN,

} DRV_SRAM_COMMAND_STATUS;


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver SRAM Module Index reference

  Summary:
    SRAM driver index definitions

  Description:
    These constants provide SRAM driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
    These values should be passed into the DRV_SRAM_Initialize and
    DRV_SRAM_Open routines to identify the driver instance in use.
*/

#define  DRV_SRAM_INDEX_0      0
#define  DRV_SRAM_INDEX_1      1


// *****************************************************************************
/* SRAM Driver Initialization Data

  Summary:
    Defines the data required to initialize the SRAM driver

  Description:
    This data type defines the data required to initialize the SRAM driver.

  Remarks:
    None.
*/

typedef struct
{
    /* Flag to indicate if the driver is to be registered with the file system.
     * */
    bool registerWithFs;
    /* SRAM Media start address. The driver treats this address as block 0
     * address for read and write operations.  */
    uint8_t* mediaStartAddress;

    /* SRAM Media geometry object. */
    const SYS_FS_MEDIA_GEOMETRY *sramMediaGeometry;

} DRV_SRAM_INIT;


// *****************************************************************************
/* SRAM Driver Event Handler Function Pointer

  Summary
    Pointer to a SRAM Driver Event handler function

  Description
    This data type defines the required function signature for the SRAM event
    handling callback function. A client must register a pointer to an event
    handling function whose function signature (parameter and return value 
    types) match the types specified by this function pointer in order to 
    receive event callbacks from the driver.
    
    The parameters and return values are described here and a partial example
    implementation is provided.

  Parameters:
    event           - Identifies the type of event
    
    commandHandle   - Handle returned from the Read/Write requests
    
    context         - Value identifying the context of the application that
                      registered the event handling function

  Returns:
    None.

  Example:
    <code>
    void APP_MySramEventHandler
    (
        DRV_SRAM_EVENT event,
        DRV_SRAM_COMMAND_HANDLE commandHandle,
        uintptr_t context
    )
    {
        MY_APP_DATA_STRUCT pAppData = (MY_APP_DATA_STRUCT) context;
        
        switch(event)
        {
            case DRV_SRAM_EVENT_COMMAND_COMPLETE:

                // Handle the completed buffer. 
                break;
            
            case DRV_SRAM_EVENT_COMMAND_ERROR:
            default:

                // Handle error.
                break;
        }
    }
    </code>

  Remarks:
    If the event is DRV_SRAM_EVENT_COMMAND_COMPLETE, it means that the read or
    write operation was completed successfully. 
    
    If the event is DRV_SRAM_EVENT_COMMAND_ERROR, it means that the scheduled
    operation was not completed successfully.
     
    The context parameter contains the handle to the client context, provided
    at the time the event handling function was  registered using the
    DRV_SRAM_EventHandlerSet function. This context handle value is
    passed back to the client as the "context" parameter.  It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) instance of the client that made the read/write
    request.

    The event handler function executes in the driver's context. It is
    recommended of the application to not perform process intensive or blocking
    operations within this function.
*/

typedef SYS_FS_MEDIA_EVENT_HANDLER DRV_SRAM_EVENT_HANDLER;


// *****************************************************************************
// *****************************************************************************
// Section: SRAM Driver Module Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SRAM_Initialize
    ( 
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init 
    );
    
  Summary:
    Initializes the SRAM instance for the specified driver index.

  Description:
    This routine initializes the SRAM driver instance for the specified driver
    index, making it ready for clients to open and use it.

  Precondition:
    None.
  
  Parameters:
    index - Identifier for the instance to be initialized.
    init  - Pointer to a data structure containing any data necessary to
            initialize the driver.
  
  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise it returns SYS_MODULE_OBJ_INVALID.
  
  Example:
    <code>
    // This code snippet shows an example of initializing the SRAM Driver.
    
    SYS_MODULE_OBJ  objectHandle;

    SYS_FS_MEDIA_REGION_GEOMETRY gSramGeometryTable[3] = 
    {
        {
            // Read Region Geometry
            .blockSize = 512,
            .numBlocks = (DRV_SRAM_MEDIA_SIZE * (1024/512)),
        },
        {
            // Write Region Geometry
            .blockSize = 512,
            .numBlocks = ((DRV_SRAM_MEDIA_SIZE * (1024/512))
        },
        {
            // Erase Region Geometry
            .blockSize = 512,
            .numBlocks = ((DRV_SRAM_MEDIA_SIZE * (1024/512))
        }
    };

    const SYS_FS_MEDIA_GEOMETRY gSramGeometry = 
    {
        .mediaProperty = SYS_FS_MEDIA_WRITE_IS_BLOCKING,

        // Number of read, write and erase entries in the table
        .numReadRegions = 1,
        .numWriteRegions = 1,
        .numEraseRegions = 1,
        .geometryTable = &gSramGeometryTable
    };

    // SRAM Driver Initialization Data
    const DRV_SRAM_INIT drvSramInit =
    {
        .mediaStartAddress = DRV_SRAM_MEDIA_START_ADDRESS,
        .sramMediaGeometry = &gSramGeometry
    };

    objectHandle = DRV_SRAM_Initialize(DRV_SRAM_INDEX_0, (SYS_MODULE_INIT*)&drvSRAMInit);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This routine must be called before any other SRAM routine is called.
    
    This routine should only be called once during system initialization unless
    DRV_SRAM_Deinitialize is called to deinitialize the driver instance.
    
    This routine will NEVER block for hardware access. The system must use
    DRV_SRAM_Status to find out when the driver is in the ready state.
*/

SYS_MODULE_OBJ DRV_SRAM_Initialize
(
    const SYS_MODULE_INDEX index,
    const SYS_MODULE_INIT * const init
);


// ****************************************************************************
/* Function:
    void DRV_SRAM_Deinitialize
    (
        SYS_MODULE_OBJ object 
    );
    
  Summary:
    Deinitializes the specified instance of the SRAM driver module

  Description:
    Deinitializes the specified instance of the SRAM driver module, disabling
    its operation. Invalidates all the internal data.
  
  Preconditions:
    Function DRV_SRAM_Initialize should have been called before calling
    this function.
  
  Parameter:
    object -  Driver object handle, returned from the DRV_SRAM_Initialize
              routine

  Returns:
    None.

  Example:
    <code>
    // This code snippet shows an example of deinitializing the driver.
    
    SYS_MODULE_OBJ      object;     //  Returned from DRV_SRAM_Initialize
    SYS_STATUS          status;
    
    DRV_SRAM_Deinitialize(object);
    
    status = DRV_SRAM_Status(object);
    if (SYS_MODULE_DEINITIALIZED != status)
    {
        // Check again later if you need to know when the driver is
        // deinitialized.
    }
    </code>
  
  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again.
*/

void DRV_SRAM_Deinitialize
(
    SYS_MODULE_OBJ object
);


// *************************************************************************
/* Function:
    SYS_STATUS DRV_SRAM_Status
    (
        SYS_MODULE_OBJ object
    );
    
  Summary:
    Gets the current status of the SRAM driver module.
  
  Description:
    This routine provides the current status of the SRAM driver module.
  
  Preconditions:
    Function DRV_SRAM_Initialize should have been called before calling this
    function.
  
  Parameters:
    object - Driver object handle, returned from the DRV_SRAM_Initialize
             routine
  
  Returns:
    SYS_STATUS_READY - Indicates that the driver is ready and accept requests
    for new operations.
    
    SYS_STATUS_UNINITIALIZED - Indicates the driver is not initialized.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_SRAM_Initialize
    SYS_STATUS          SRAMStatus;
    
    SRAMStatus = DRV_SRAM_Status(object);
    if (SRAMStatus == SYS_STATUS_READY)
    {
        // Driver is ready to process read/write operations.
    }
    else
    {
        // Driver is not ready.
    }
    </code>
  
  Remarks:
    None.
*/

SYS_STATUS DRV_SRAM_Status
(
    SYS_MODULE_OBJ object
);

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
    );
    
  Summary:
    Opens the specified SRAM driver instance and returns a handle to it
  
  Description:
    This routine opens the specified SRAM driver instance and provides a
    handle. This handle must be provided to all other client-level operations
    to identify the caller and the instance of the driver.
  
  Preconditions:
    DRV_SRAM_Initialize must have been called before calling this function.
  
  Parameters:
    index  - Identifier for the object instance to be opened
    intent - Zero or more of the values from the enumeration DRV_IO_INTENT
             "ORed" together to indicate the intended use of the driver
  
  Returns:
    If successful, the routine returns a valid open-instance handle (a
    number identifying both the caller and the module instance).
    
    If an error occurs, DRV_HANDLE_INVALID is returned. Errors can occur under
    the following circumstances:
        - if the number of client objects allocated via DRV_SRAM_CLIENTS_NUMBER
        is insufficient
        - if the client is trying to open the driver but driver has been opened
        exclusively by another client
        - if the client is trying to open the driver exclusively, but has
        already been opened in a non exclusive mode by another client.
        - if the driver hardware instance being opened is invalid
  
  Example:
    <code>
    DRV_HANDLE handle;
    
    handle = DRV_SRAM_Open(DRV_SRAM_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
    }
    </code>
  
  Remarks:
    The handle returned is valid until the DRV_SRAM_Close routine is called.
    This routine will NEVER block waiting for hardware. If the driver has has
    already been opened, it cannot be opened exclusively.
*/

DRV_HANDLE DRV_SRAM_Open
(
    const SYS_MODULE_INDEX index, 
    const DRV_IO_INTENT ioIntent
);


// *****************************************************************************
/* Function:
    void DRV_SRAM_Close
    (
        const DRV_HANDLE handle
    );

  Summary:
    Closes an opened-instance of the SRAM driver

  Description:
    This routine closes an opened-instance of the SRAM driver, invalidating the
    handle.

  Precondition:
    The DRV_SRAM_Initialize routine must have been called for the specified
    SRAM driver instance.

    DRV_SRAM_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle - A valid open-instance handle, returned from the driver's open
             routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_SRAM_Open

    DRV_SRAM_Close(handle);
    </code>

  Remarks:
    After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines. A new handle must be obtained by
    calling DRV_SRAM_Open before the caller may use the driver again. Usually
    there is no need for the driver client to verify that the Close operation
    has completed.
*/

void DRV_SRAM_Close
(
    const DRV_HANDLE handle
);


// *****************************************************************************
/* Function:
    void DRV_SRAM_Read
    (
        const DRV_HANDLE handle,
        DRV_SRAM_COMMAND_HANDLE * commandHandle,
        void * targetBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Reads blocks of data from the specified block start address.

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

  Precondition:
    The DRV_SRAM_Initialize routine must have been called for the specified
    SRAM driver instance.

    DRV_SRAM_Open must have been called with DRV_IO_INTENT_READ or
    DRV_IO_INTENT_READWRITE as the ioIntent to obtain a valid opened device
    handle.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle
                   
    targetBuffer  - Buffer into which the data read from the SRAM memory will
                    be placed

    blockStart    - SRAM media's start block address from where the read should
                    begin.

    nBlock        - Total number of blocks to be read.

  Returns:
    The buffer handle is returned in the commandHandle argument. It will be
    DRV_SRAM_COMMAND_HANDLE_INVALID if the request was not successful.

  Example:
    <code>

    uint8_t myBuffer[MY_BUFFER_SIZE];
    uint32_t blockStart = 0;
    uint32_t nBlock = 2;
    DRV_SRAM_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;    

    // mySRAMHandle is the handle returned by the DRV_SRAM_Open function.
    
    DRV_SRAM_EventHandlerSet(mySRAMHandle, APP_SRAMEventHandler, (uintptr_t)&myAppObj);
    DRV_SRAM_Read(mySRAMHandle, &commandHandle, &myBuffer, blockStart, nBlock);
    if(DRV_SRAM_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }
    else
    {
        // Read operation completed successfully.
    }

    // Event is invoked from within the DRV_SRAM_Read function when the read
    // operation processing is complete.

    void APP_SRAMEventHandler
    (
        DRV_SRAM_EVENT event, 
        DRV_SRAM_COMMAND_HANDLE commandHandle, 
        uintptr_t contextHandle
    )
    {
        // contextHandle points to myAppObj.
        switch(event)
        {
            case DRV_SRAM_EVENT_COMMAND_COMPLETE:
                // This means the data was transferred. 
                break;
            
            case DRV_SRAM_EVENT_COMMAND_ERROR:
                // Error handling here.
                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    None.
*/

void DRV_SRAM_Read
(
    const DRV_HANDLE handle,
    DRV_SRAM_COMMAND_HANDLE * commandHandle,
    void * targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
);


// *****************************************************************************
/* Function:
    void DRV_SRAM_Write
    (
        const DRV_HANDLE handle,
        DRV_SRAM_COMMAND_HANDLE * commandHandle,
        void * sourceBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Writes blocks of data starting from the specified block start address of
    the SRAM media.

  Description:
    This routine writes blocks of data starting at the specified block start
    address. This operation is blocking and returns after having written the
    data. If a event handler has been registered to receive the driver events
    then the event handler will be called from within this function. The
    function returns DRV_SRAM_COMMAND_HANDLE_INVALID in the commandHandle
    argument under the following circumstances:
    - if the driver handle is invalid
    - if the driver state is not ready
    - if the source buffer pointer is NULL
    - if the number of blocks to be written is zero or more than the actual
    number of blocks available
    - if the client opened the driver in read only mode

  Precondition:
    The DRV_SRAM_Initialize() routine must have been called for the specified
    SRAM driver instance.

    DRV_SRAM_Open() routine must have been called to obtain a valid opened
    device handle. DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have
    been specified as a parameter to this routine.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle
                   
    sourceBuffer  - The source buffer containing data to be programmed into
                    SRAM memory

    blockStart    - Start block address of SRAM media from where the write
                    should begin.

    nBlock        - Total number of blocks to be written. 

  Returns:
    The buffer handle is returned in the commandHandle argument. It will be
    DRV_SRAM_COMMAND_HANDLE_INVALID if the request was not successful.

  Example:
    <code>
    
    uint8_t myBuffer[MY_BUFFER_SIZE];
    uint32_t blockStart = 2;
    uint32_t nBlock = 2;
    DRV_SRAM_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;    

    // mySRAMHandle is the handle returned by the DRV_SRAM_Open function.
    // Client registers an event handler with driver

    DRV_SRAM_EventHandlerSet(mySRAMHandle, APP_SRAMEventHandler, (uintptr_t)&myAppObj);
    DRV_SRAM_Write(mySRAMHandle, &commandHandle, &myBuffer, blockStart, nBlock);

    if(DRV_SRAM_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }
    else
    {
        // Write completed successfully.
    }

    // Event is received from within the DRV_SRAM_Write function when the
    // buffer is processed.

    void APP_SRAMEventHandler
    (
        DRV_SRAM_EVENT event, 
        DRV_SRAM_COMMAND_HANDLE commandHandle, 
        uintptr_t contextHandle
    )
    {
        // contextHandle points to myAppObj.
        switch(event)
        {
            case DRV_SRAM_EVENT_COMMAND_COMPLETE:
                // This means the data was transferred. 
                break;
            
            case DRV_SRAM_EVENT_COMMAND_ERROR:
                // Error handling here.
                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    None
*/

void DRV_SRAM_Write
(
    const DRV_HANDLE handle,
    DRV_SRAM_COMMAND_HANDLE * commandHandle,
    void * sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
);


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
    This routine gets the current status of the command. The application must
    use this routine where the status of a scheduled command needs to be polled
    on. The function may return DRV_SRAM_COMMAND_COMPLETED in a case where the
    command handle has expired. A command handle expires when the internal
    buffer object is re-assigned to another read or write request. It is
    recommended that this function be called regularly in order to track the
    command status correctly.

    The application can alternatively register an event handler to receive read
    or write operation completion events.

  Preconditions:
    The DRV_SRAM_Initialize() routine must have been called.

    The DRV_SRAM_Open() must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    A DRV_SRAM_COMMAND_STATUS value describing the current status of the
    command.  Returns DRV_SRAM_COMMAND_COMPLETED if the client handle or the
    command handle is not valid.

  Example:
    <code>
    DRV_HANDLE                  handle;         // Returned from DRV_SRAM_Open
    DRV_SRAM_COMMAND_HANDLE     commandHandle;
    DRV_SRAM_COMMAND_STATUS     status;
 
    status = DRV_SRAM_CommandStatus(handle, commandHandle);
    if(status == DRV_SRAM_COMMAND_COMPLETED)
    {
        // Operation Done
    }
    </code>

  Remarks:
    This routine will not block for hardware access and will immediately return
    the current status.
*/

DRV_SRAM_COMMAND_STATUS DRV_SRAM_CommandStatus
(
    const DRV_HANDLE handle, 
    const DRV_SRAM_COMMAND_HANDLE commandHandle
);

// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_GEOMETRY * DRV_SRAM_GeometryGet
    (
        const DRV_HANDLE handle
    );

  Summary:
    Returns the geometry of the device.

  Description:
    This API gives the following geometrical details of the SRAM memory:
    - Media Property
    - Number of Read/Write/Erase regions
    - Number of Blocks and their size in each region of the device

  Precondition:
    The DRV_SRAM_Initialize() routine must have been called for the specified
    SRAM driver instance.

    The DRV_SRAM_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    SYS_FS_MEDIA_GEOMETRY - Pointer to structure which holds the media geometry
    information.

  Example:
    <code> 
    
    SYS_FS_MEDIA_GEOMETRY * sramGeometry;
    uint32_t readBlockSize, writeBlockSize, eraseBlockSize;
    uint32_t nReadBlocks, nReadRegions, totalSize;

    sramGeometry = DRV_SRAM_GeometryGet(sramOpenHandle1);

    readBlockSize  = sramGeometry->geometryTable->blockSize;
    nReadBlocks = sramGeometry->geometryTable->numBlocks;
    nReadRegions = sramGeometry->numReadRegions;

    writeBlockSize  = (sramGeometry->geometryTable +1)->blockSize;
    eraseBlockSize  = (sramGeometry->geometryTable +2)->blockSize;

    totalSize = readBlockSize * nReadBlocks * nReadRegions;

    </code>

  Remarks:
    None.
*/

SYS_FS_MEDIA_GEOMETRY * DRV_SRAM_GeometryGet
(
    const DRV_HANDLE handle
);

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
    call back when an operation has completed.

  Description:
    This function allows a client to identify an event handling function for
    the driver to call back when an operation has completed. When a client
    calls a read or a write function, it is provided with a handle identifying
    the read/write request. The driver will pass this handle back to the client
    by calling "eventHandler" function when the operation has completed.
    
    The event handler should be set before the client performs any read or
    write operations that could generate events. The event handler once set,
    persists until the client closes the driver or sets another event handler
    (which could be a "NULL" pointer to indicate no callback).

  Precondition:
    The DRV_SRAM_Initialize() routine must have been called for the specified
    SRAM driver instance.

    The DRV_SRAM_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

    eventHandler - Pointer to the event handler function implemented by the
                   user
    
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
    DRV_SRAM_COMMAND_HANDLE commandHandle;

    // drvSRAMHandle is the handle returned by the DRV_SRAM_Open function.
    // Client registers an event handler with driver. This is done once.

    DRV_SRAM_EventHandlerSet(drvSRAMHandle, APP_SRAMEventHandler, (uintptr_t)&myAppObj);

    DRV_SRAM_Read(drvSRAMHandle, &commandHandle, &myBuffer, blockStart, nBlock);

    if(DRV_SRAM_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event Processing Technique. Event is received when operation is done.
    void APP_SRAMEventHandler
    (
        DRV_SRAM_EVENT event, 
        DRV_SRAM_COMMAND_HANDLE handle,
        uintptr_t context
    )
    {
        // The context handle was set to an application specific object. It is
        // now retrievable easily in the event handler.
        MY_APP_OBJ myAppObj = (MY_APP_OBJ *) context;

        switch(event)
        {
            case DRV_SRAM_EVENT_COMMAND_COMPLETE:
                // This means the data was transferred. 
                break;
            
            case DRV_SRAM_EVENT_COMMAND_ERROR:
                // Error handling here.
                break;

            default:
                break;
        }
    }
    </code>

  Remarks:
    If the client does not want to be notified when the queued operation has
    completed, it does not need to register a callback.
*/

void DRV_SRAM_EventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
);

// *****************************************************************************
/* Function:
    bool DRV_SRAM_IsAttached
    ( 
        const DRV_HANDLE handle 
    );

  Summary:
    Returns the physical attach status of the SRAM.

  Description:
    This function returns the physical attach status of the SRAM.

  Precondition:
    The DRV_SRAM_Initialize() routine must have been called for the specified
    SRAM driver instance.

    The DRV_SRAM_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle - A valid open-instance handle, returned from the driver's open
             function

  Returns:
    Returns false if the handle is invalid otherwise returns true.

  Example:
    <code> 

    // The SRAM media is always attached and so the below always returns true.
    bool isSRAMAttached;
    isSRAMAttached = DRV_SRAM_isAttached(drvSRAMHandle);

    </code>

  Remarks:
    None.
*/

bool DRV_SRAM_IsAttached
(
    const DRV_HANDLE handle
);

// *****************************************************************************
/* Function:
    bool DRV_SRAM_IsWriteProtected
    ( 
        const DRV_HANDLE handle 
    );

  Summary:
    Returns the write protect status of the SRAM.

  Description:
    This function returns the physical attach status of the SRAM. This function
    always returns false.

  Precondition:
    The DRV_SRAM_Initialize() routine must have been called for the specified 
    SRAM driver instance.

    The DRV_SRAM_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    Always returns false.

  Example:
    <code>

    // The SRAM media is treated as always writeable.
    bool isWriteProtected;
    isWriteProtected = DRV_SRAM_IsWriteProtected(drvSRAMHandle);

    </code>

  Remarks:
    None.
*/

bool DRV_SRAM_IsWriteProtected
(
    const DRV_HANDLE handle
);

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

  Precondition:
    The DRV_SRAM_Initialize() routine must have been called for the specified
    SRAM driver instance.

    The DRV_SRAM_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle - A valid open-instance handle, returned from the driver's open
             function

  Returns:
    Start address of the SRAM Media if the handle is valid otherwise NULL.

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
);

#ifdef __cplusplus
}
#endif

#endif // #ifndef _DRV_SRAM_H
/*******************************************************************************
 End of File
*/

