/*******************************************************************************
  S25FL Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_s25fl.h

  Summary:
    S25FL Driver Interface Definition

  Description:
    The S25FL driver provides a simple interface to manage the S25FLVF series
    of SQI Flash Memory connected to Microchip microcontrollers. This file
    defines the interface definition for the S25FL driver.
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
#ifndef _DRV_S25FL_H
#define _DRV_S25FL_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************

#include <stdbool.h>
#include "driver/sqi/drv_sqi.h"
#include "system/common/sys_common.h"
#include "driver/driver_common.h"
#include "system/common/sys_module.h"
#include "system/tmr/sys_tmr.h"
#include "osal/osal.h"
#include "system/fs/sys_fs_media_manager.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
/* S25FL Driver command handle.

  Summary:
    Handle identifying commands queued in the driver.

  Description:
    A command handle is returned by a call to the Read, Write, Erase or
    EraseWrite functions. This handle allows the application to track the
    completion of the operation. This command handle is also returned to the
    client along with the event that has occurred with respect to the command.
    This allows the application to connect the event to a specific command in
    case where multiple commands are queued.

    The command handle associated with the command request expires when the
    client has been notified of the completion of the command (after event
    handler function that notifies the client returns) or after the command has
    been retired by the driver if no event handler callback was set. 

  Remarks:
    None.
*/

typedef SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE DRV_S25FL_COMMAND_HANDLE;

// *****************************************************************************
/* S25FL Driver Invalid Command Handle.

  Summary:
    This value defines the S25FL Driver's Invalid Command Handle.

  Description:
    This value defines the S25FL Driver's Invalid Command Handle. This value is
    returned by read/write/erase/erasewrite routines when the command request
    was not accepted.

  Remarks:
    None.
*/

#define DRV_S25FL_COMMAND_HANDLE_INVALID SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID

// *****************************************************************************
/* S25FL Driver Events

   Summary
    Identifies the possible events that can result from a request.

   Description
    This enumeration identifies the possible events that can result from a
    read, write, erase or erasewrite request caused by the client.

   Remarks:
    One of these values is passed in the "event" parameter of the event
    handling callback function that client registered with the driver by
    calling the DRV_S25FL_EventHandlerSet function when a request is completed.
*/

typedef enum
{
    /* Operation has been completed successfully. */
    DRV_S25FL_EVENT_COMMAND_COMPLETE = SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_COMPLETE,

    /* There was an error during the operation */
    DRV_S25FL_EVENT_COMMAND_ERROR = SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_ERROR 

} DRV_S25FL_EVENT;

// ***********************************************************************
/* S25FL Driver Command Status

  Summary:
    S25FL Driver command Status
    
  Description:
    Specifies the status of the command for the read, write, erase and
    erasewrite operations.
    
  Remarks:
    None.                                                               
*/  
typedef enum
{
    /* Done OK and ready */
    DRV_S25FL_COMMAND_COMPLETED          = SYS_FS_MEDIA_COMMAND_COMPLETED,

    /* Scheduled but not started */
    DRV_S25FL_COMMAND_QUEUED             = SYS_FS_MEDIA_COMMAND_QUEUED,

    /* Currently being in transfer */
    DRV_S25FL_COMMAND_IN_PROGRESS        = SYS_FS_MEDIA_COMMAND_IN_PROGRESS,

    /* Unknown Command */
    DRV_S25FL_COMMAND_ERROR_UNKNOWN      = SYS_FS_MEDIA_COMMAND_UNKNOWN,

} DRV_S25FL_COMMAND_STATUS;

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver S25FL Module Index reference

  Summary:
    S25FL driver index definitions

  Description:
    These constants provide S25FL driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
    These values should be passed into the DRV_S25FL_Initialize and
    DRV_S25FL_Open routines to identify the driver instance in use.
*/

#define  DRV_S25FL_INDEX_0      0
#define  DRV_S25FL_INDEX_1      1

// *****************************************************************************
/* S25FL Driver Initialization Data

  Summary:
    Defines the data required to initialize or reinitialize the S25FL driver

  Description:
    This data type defines the data required to initialize or reinitialize the
    S25FL driver.

  Remarks:
    Not all initialization features are available for all devices. Please
    refer to the specific device data sheet to determine availability.
*/

typedef struct
{
    /* SQI Device Index. */
    uint8_t sqiDevice;

} DRV_S25FL_INIT;

// *****************************************************************************
/* S25FL Driver Event Handler Function Pointer

   Summary
    Pointer to a S25FL Driver Event handler function

   Description
    This data type defines the required function signature for the S25FL event
    handling callback function. A client must register a pointer to an event
    handling function whose function signature (parameter and return value
    types) match the types specified by this function pointer in order to
    receive event calls back from the driver.
    
    The parameters and return values are described here and a partial example
    implementation is provided.

  Parameters:
    event           - Identifies the type of event
    
    commandHandle   - Handle returned from the Read/Write/Erase/EraseWrite
                      requests
    
    context         - Value identifying the context of the application that
                      registered the event handling function

  Returns:
    None.

  Example:
    <code>
    void APP_MyS25flEventHandler
    (
        DRV_S25FL_EVENT event,
        DRV_S25FL_COMMAND_HANDLE commandHandle,
        uintptr_t context
    )
    {
        MY_APP_DATA_STRUCT pAppData = (MY_APP_DATA_STRUCT) context;
        
        switch(event)
        {
            case DRV_S25FL_EVENT_COMMAND_COMPLETE:
                // Handle the completed buffer. 
                break;
            
            case DRV_S25FL_EVENT_COMMAND_ERROR:
            default:
                // Handle error.
                break;
        }
    }
    </code>

  Remarks:
    If the event is DRV_S25FL_EVENT_COMMAND_COMPLETE, it means that the
    requested operation was completed successfully. 
    
    If the event is DRV_S25FL_EVENT_COMMAND_ERROR, it means that the scheduled
    operation was not completed successfully.
     
    The context parameter contains the handle to the client context, provided
    at the time the event handling function was registered using the
    DRV_S25FL_EventHandlerSet function. This context handle value is passed
    back to the client as the "context" parameter. It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) instance of the client that made the read/write/erase
    request.

    The event handler function executes in the driver peripheral's interrupt
    context when the driver is configured for interrupt mode operation. It is
    recommended of the application to not perform process intensive or blocking
    operations within this function.
*/
typedef SYS_FS_MEDIA_EVENT_HANDLER DRV_S25FL_EVENT_HANDLER;

// *****************************************************************************
// *****************************************************************************
// Section: S25FL Driver Module Interface Routines
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

  Precondition:
    None.
  
  Parameters:
    index -  Identifier for the instance to be initialized

    init -   Pointer to a data structure containing any data necessary to
             initialize the driver.
  
  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise it returns SYS_MODULE_OBJ_INVALID.
  
  Example:
    <code>
    // This code snippet shows an example of initializing the S25FL Driver.
    
    SYS_MODULE_OBJ  objectHandle;

    const DRV_S25FL_INIT drvS25flInitData0 =
    {
        .sqiDevice = 1,
    };

    //usage of DRV_S25FL_INDEX_0 indicates usage of Flash-related APIs
    objectHandle = DRV_S25FL_Initialize(DRV_S25FL_INDEX_0, (SYS_MODULE_INIT*)&drvS25flInitData0);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This routine must be called before any other S25FL routine is called.
    
    This routine should only be called once during system initialization unless
    DRV_S25FL_Deinitialize is called to deinitialize the driver instance.
    
    This routine will NEVER block for hardware access. If the operation
    requires time to allow the hardware to initialize, it will be reported by
    the DRV_S25FL_Status operation. The system must use DRV_S25FL_Status to
    find out when the driver is in the ready state.
    
*/

SYS_MODULE_OBJ DRV_S25FL_Initialize
(
    const SYS_MODULE_INDEX index,
    const SYS_MODULE_INIT * const init
);

// ****************************************************************************
/* Function:
    void DRV_S25FL_Deinitialize
    (
        SYS_MODULE_OBJ object 
    );
    
  Summary:
    Deinitializes the specified instance of the S25FL driver module.

  Description:
    Deinitializes the specified instance of the S25FL driver module, disabling
    its operation (and any hardware). Invalidates all the internal data.
  
  Preconditions:
    Function DRV_S25FL_Initialize should have been called before calling
    this function.
  
  Parameter:
    object -  Driver object handle, returned from the DRV_S25FL_Initialize
              routine

  Returns:
    None.

  Example:
    <code>
    // This code snippet shows an example of deinitializing the driver.
    
    SYS_MODULE_OBJ      object;     //  Returned from DRV_S25FL_Initialize
    SYS_STATUS          status;
    
    DRV_S25FL_Deinitialize(object);
    
    status = DRV_S25FL_Status(object);
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

void DRV_S25FL_Deinitialize
(
    SYS_MODULE_OBJ object
);

// *************************************************************************
/* Function:
    SYS_STATUS DRV_S25FL_Status
    (
        SYS_MODULE_OBJ object
    );
    
  Summary:
    Gets the current status of the S25FL driver module.
  
  Description:
    This routine provides the current status of the S25FL driver module.
  
  Preconditions:
    Function DRV_S25FL_Initialize should have been called before calling this
    function.
  
  Parameters:
    object -  Driver object handle, returned from the DRV_S25FL_Initialize
              routine
  
  Returns:
    SYS_STATUS_READY - Indicates that the driver is ready and accept requests
                       for new operations.
    
    SYS_STATUS_UNINITIALIZED - Indicates the driver is not initialized.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_S25FL_Initialize
    SYS_STATUS          S25FLStatus;
    
    S25FLStatus = DRV_S25FL_Status(object);
    else if (SYS_STATUS_ERROR >= S25FLStatus)
    {
        // Handle error
    }
    </code>
  
  Remarks:
    This routine will NEVER block waiting for hardware.
*/

SYS_STATUS DRV_S25FL_Status
(
    SYS_MODULE_OBJ object
);

// ****************************************************************************
/* Function:
    void DRV_S25FL_Tasks 
    (
        SYS_MODULE_OBJ object
    );
    
  Summary:
    Maintains the S25FL driver's internal state machine.
  
  Description:
    This routine maintains the driver's internal state machine. Part of the
    driver initialization is done in this routine. This routine is responsible
    for processing the read, write, erase or erasewrite requests queued for the
    S25FL driver.
  
  Preconditions:
    The DRV_S25FL_Initialize routine must have been called for the specified
    S25FL driver instance.
  
  Parameters:
    object -  Object handle for the specified driver instance (returned from
              DRV_S25FL_Initialize)
  Returns:
    None.
  
  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_S25FL_Initialize
    
    while (true)
    {
        DRV_S25FL_Tasks (object);
        // Do other tasks
    }
    </code>

  Remarks:
    This routine is normally not called directly by an application. It is
    called by the system's Tasks routine (SYS_Tasks).
    
    This routine may execute in an ISR context and will never block or
    access any resources that may cause it to block.                        
*/

void DRV_S25FL_Tasks 
(
    SYS_MODULE_OBJ object
);

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
    );
    
  Summary:
    Opens the specified S25FL driver instance and returns a handle to it
  
  Description:
    This routine opens the specified S25FL driver instance and provides a
    handle. This handle must be provided to all other client-level operations
    to identify the caller and the instance of the driver.
  
  Preconditions:
    Function DRV_S25FL_Initialize must have been called before calling this
    function.
  
  Parameters:
    index  - Identifier for the object instance to be opened

    intent - Zero or more of the values from the enumeration DRV_IO_INTENT
             "ORed" together to indicate the intended use of the driver
  
  Returns:
    If successful, the routine returns a valid open-instance handle (a
    number identifying both the caller and the module instance).
    
    If an error occurs, DRV_HANDLE_INVALID is returned. Errors can occur under
    the following circumstances:
        - if the number of client objects allocated via
          DRV_S25FL_CLIENTS_NUMBER is insufficient
        - if the client is trying to open the driver but driver has been opened
          exclusively by another client
        - if the client is trying to open the driver exclusively, but has
          already been opened in a non exclusive mode by another client.
        - if the driver hardware instance being opened is not initialized or is
          invalid
  
  Example:
    <code>
    DRV_HANDLE handle;
    
    handle = DRV_S25FL_Open(DRV_S25FL_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
    }
    </code>
  
  Remarks:
    The handle returned is valid until the DRV_S25FL_Close routine is called.
    This routine will NEVER block waiting for hardware. If the driver has has
    already been opened, it cannot be opened exclusively.
*/

DRV_HANDLE DRV_S25FL_Open
(
    const SYS_MODULE_INDEX index, 
    const DRV_IO_INTENT ioIntent
);

// *****************************************************************************
/* Function:
    void DRV_S25FL_Close
    (
        const DRV_HANDLE handle
    );

  Summary:
    Closes an opened-instance of the S25FL driver

  Description:
    This routine closes an opened-instance of the S25FL driver, invalidating
    the handle.

  Precondition:
    The DRV_S25FL_Initialize routine must have been called for the specified
    S25FL driver instance.

    DRV_S25FL_Open must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_S25FL_Open

    DRV_S25FL_Close(handle);
    </code>

  Remarks:
    After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines. A new handle must be obtained by
    calling DRV_S25FL_Open before the caller may use the driver again. Usually
    there is no need for the driver client to verify that the Close operation
    has completed.
*/

void DRV_S25FL_Close
(
    const DRV_HANDLE handle
);

// *****************************************************************************
/* Function:
    void DRV_S25FL_Read
    (
        const DRV_HANDLE handle,
        DRV_S25FL_COMMAND_HANDLE * commandHandle,
        void * targetBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Reads blocks of data from the specified block start address.

  Description:
    This function schedules a non-blocking read operation for reading blocks of
    data from the flash memory. The function returns with a valid command
    handle in the commandHandle argument if the request was scheduled
    successfully. The function adds the request to the hardware instance queue
    and returns immediately. While the request is in the queue, the application
    buffer is owned by the driver and should not be modified. The function
    returns DRV_S25FL_COMMAND_HANDLE_INVALID in the commandHandle argument
    under the following circumstances:
    - if a buffer object could not be allocated to the request
    - if the target buffer pointer is NULL
    - if the client opened the driver for write only
    - if the number of blocks to be read is either zero or more than the number
      of blocks actually available
    - if the driver handle is invalid 

  Precondition:
    The DRV_S25FL_Initialize routine must have been called for the specified
    S25FL driver instance.

    DRV_S25FL_Open must have been called with DRV_IO_INTENT_READ or
    DRV_IO_INTENT_READWRITE as the ioIntent to obtain a valid opened device
    handle.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the command handle
                   
    targetBuffer  - Buffer into which the data read from the S25FL Flash memory
                    will be placed

    blockStart    - Read block start address from where the data should be
                    read.

    nBlock        - Total number of blocks to be read.

  Returns:
    The command handle is returned in the commandHandle argument. It will be
    DRV_S25FL_COMMAND_HANDLE_INVALID if the request was not successful.

  Example:
    <code>

    uint8_t myBuffer[MY_BUFFER_SIZE];
    
    // Use DRV_S25FL_GeometryGet () to find the read region geometry.
    // Find the block address from which to read data.
    uint32_t blockStart = S25FL_BLOCK_ADDRESS_TO_READ_FROM;
    uint32_t nBlock = 2;
    DRV_S25FL_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;    

    // myS25FLHandle is the handle returned by the DRV_S25FL_Open function.
    // Client registers an event handler with driver

    DRV_S25FL_EventHandlerSet(myS25FLHandle, APP_S25FLEventHandler, (uintptr_t)&myAppObj);

    DRV_S25FL_Read(myS25FLHandle, &commandHandle, &myBuffer, blockStart, nBlock);
    if(DRV_S25FL_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }
    else
    {
        // Read queued successfully.
    }

    // Event is received when the command request is processed.

    void APP_S25FLEventHandler
    (
        DRV_S25FL_EVENT event, 
        DRV_S25FL_COMMAND_HANDLE commandHandle,
        uintptr_t contextHandle
    )
    {
        // contextHandle points to myAppObj.
        switch(event)
        {
            case DRV_S25FL_EVENT_COMMAND_COMPLETE:
                // This means the data was transferred. 
                break;
            
            case DRV_S25FL_EVENT_COMMAND_ERROR:
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

void DRV_S25FL_Read
(
    const DRV_HANDLE handle,
    DRV_S25FL_COMMAND_HANDLE * commandHandle,
    void * targetBuffer,
    uint32_t blockStart,
    uint32_t nBlock
);

// *****************************************************************************
/* Function:
    void DRV_S25FL_Write
    (
        const DRV_HANDLE handle,
        DRV_S25FL_COMMAND_HANDLE * commandHandle,
        void * sourceBuffer,
        uint32_t blockStart,
        uint32_t nBlock
    );

  Summary:
    Writes blocks of data starting at the specified block start address.

  Description:
    This function schedules a non-blocking write operation for writing blocks
    of data into flash memory. The function returns with a valid command handle
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
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_S25FL_EVENT_COMMAND_COMPLETE event if the buffer
    was processed successfully or DRV_S25FL_EVENT_COMMAND_ERROR event if the
    buffer was not processed successfully.

  Precondition:
    The DRV_S25FL_Initialize() routine must have been called for the specified
    S25FL driver instance.

    DRV_S25FL_Open() routine must have been called to obtain a valid opened
    device handle. DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have
    been specified as a parameter to this routine.

    The flash address location which has to be written, must have be erased
    before using the DRV_S25FL_Erase() routine.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle
                   
    sourceBuffer  - The source buffer containing data to be programmed into
                    S25FL Flash

    blockStart    - Write block start address from where the data should be
                    written to.

    nBlock        - Total number of blocks to be written. 

  Returns:
    The command handle is returned in the commandHandle argument. It will be
    DRV_S25FL_COMMAND_HANDLE_INVALID if the request was not successful.

  Example:
    <code>
    
    uint8_t myBuffer[MY_BUFFER_SIZE];
    
    // Use DRV_S25FL_GeometryGet () to find the write region geometry.
    // Find the block address to which data is to be written.
    uint32_t blockStart = S25FL_BLOCK_ADDRESS_TO_WRITE_TO;
    uint32_t nBlock = 2;
    DRV_S25FL_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;    

    // myS25FLHandle is the handle returned by the DRV_S25FL_Open function.
    // Client registers an event handler with driver

    DRV_S25FL_EventHandlerSet(myS25FLHandle, APP_S25FLEventHandler, (uintptr_t)&myAppObj);

    DRV_S25FL_Write(myS25FLHandle, &commandHandle, &myBuffer, blockStart, nBlock);

    if(DRV_S25FL_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when the buffer is processed.

    void APP_S25FLEventHandler
    (
        DRV_S25FL_EVENT event, 
        DRV_S25FL_COMMAND_HANDLE commandHandle,
        uintptr_t contextHandle
    )
    {
        // contextHandle points to myAppObj.
        switch(event)
        {
            case DRV_S25FL_EVENT_COMMAND_COMPLETE:
                // This means the data was transferred. 
                break;
            
            case DRV_S25FL_EVENT_COMMAND_ERROR:
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

void DRV_S25FL_Write
(
    const DRV_HANDLE handle,
    DRV_S25FL_COMMAND_HANDLE * commandHandle,
    void * sourceBuffer,
    uint32_t blockStart,
    uint32_t nBlock
);

// **************************************************************************
/* Function:
    void DRV_S25FL_Erase
    (
        const DRV_HANDLE handle,
        DRV_S25FL_COMMAND_HANDLE * commandHandle,
        uint32_t blockStart,
        uint32_t nBlock
    );
    
  Summary:
    Erase the specified number of flash blocks from the specified block start
    address.
  
  Description:
    This function schedules a non-blocking erase operation of flash memory. The
    function returns with a valid erase handle in the commandHandle argument if
    the erase request was scheduled successfully. The function adds the request
    to the hardware instance queue and returns immediately. The function
    returns DRV_S25FL_COMMAND_HANDLE_INVALID in the commandHandle argument
    under the following circumstances:
    - if a buffer object could not be allocated to the request
    - if the client opened the driver for read only
    - if the number of blocks to be erased is either zero or more than the
      number of blocks actually available
    - if the driver handle is invalid 

    If the requesting client registered an event callback with the driver, the
    driver will issue a DRV_S25FL_EVENT_COMMAND_COMPLETE event if the erase
    operation was successful or DRV_S25FL_EVENT_COMMAND_ERROR event if the
    erase operation was not successful.
  
  Preconditions:
    The DRV_S25FL_Initialize() routine must have been called for the specified
    S25FL driver instance.
    
    The DRV_S25FL_Open() routine must have been called with DRV_IO_INTENT_WRITE
    or DRV_IO_INTENT_READWRITE to obtain a valid opened device handle.
  
  Parameters:
    handle        - A valid open-instance handle, returned from the
                    driver's open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle

    blockStart    - Erase block start address from where the blocks should be
                    erased.

    nBlock        - Total number of blocks to be erased. 
  
  Returns:
    The command handle is returned in the commandHandle argument. It Will be
    DRV_S25FL_COMMAND_HANDLE_INVALID if the request was not queued.
  
  Example:
    <code>
   
    // Use DRV_S25FL_GeometryGet () to find the read region geometry.
    // Find the erase block start address from where the number of blocks
    // should be erased.
    uint32_t blockStart = 0;
    uint32_t nBlock = 4; 
    DRV_S25FL_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;    

    // myS25FLHandle is the handle returned by the DRV_S25FL_Open function.
    
    // Client registers an event handler with driver
    DRV_S25FL_EventHandlerSet(myS25FLHandle, APP_S25FLEventHandler, (uintptr_t)&myAppObj);

    DRV_S25FL_Erase( myS25FLHandle, &commandHandle, blockStart, nBlock );

    if(DRV_S25FL_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when the buffer queue is processed.

    void APP_S25FLEventHandler
    (
        DRV_S25FL_EVENT event, 
        DRV_S25FL_COMMAND_HANDLE commandHandle,
        uintptr_t contextHandle
    )
    {
        // contextHandle points to myAppObj.
        switch(event)
        {
            case DRV_S25FL_EVENT_COMMAND_COMPLETE:
                // Erase operation completled successfully.
                break;
            
            case DRV_S25FL_EVENT_COMMAND_ERROR:
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

void DRV_S25FL_Erase
(
    const DRV_HANDLE handle,
    DRV_S25FL_COMMAND_HANDLE * commandHandle,
    uint32_t blockStart,
    uint32_t nBlock
);

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

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return command
                    handle. If NULL, then command handle is not returned.
                   
    sourceBuffer  - The source buffer containing data to be programmed into
                    S25FL Flash

    writeBlockStart - Write block start address where the write should begin.

    nWriteBlock   - Total number of blocks to be written. 

  Returns:
    The command handle is returned in the commandHandle argument. It Will be
    DRV_S25FL_COMMAND_HANDLE_INVALID if the request was not queued.

  Example:
    <code>
    
    uint8_t myBuffer[MY_BUFFER_SIZE];
    
    // Use DRV_S25FL_GeometryGet () to find the write region geometry.
    // Find the block address to which data is to be written.
    uint32_t blockStart = S25FL_BLOCK_ADDRESS_TO_WRITE_TO;
    uint32_t nBlock = 2;
    DRV_S25FL_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;    

    // myS25FLHandle is the handle returned by the DRV_S25FL_Open function.
    // Client registers an event handler with driver

    DRV_S25FL_EventHandlerSet(myS25FLHandle, APP_S25FLEventHandler, (uintptr_t)&myAppObj);

    DRV_S25FL_EraseWrite(myS25FLHandle, &commandHandle, &myBuffer, blockStart, nBlock);

    if(DRV_S25FL_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when the buffer is processed.

    void APP_S25FLEventHandler
    (
        DRV_S25FL_EVENT event, 
        DRV_S25FL_COMMAND_HANDLE commandHandle,
        uintptr_t contextHandle
    )
    {
        // contextHandle points to myAppObj.
        switch(event)
        {
            case DRV_S25FL_EVENT_COMMAND_COMPLETE:
                // Operation completled successfully.
                break;
            
            case DRV_S25FL_EVENT_COMMAND_ERROR:
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

void DRV_S25FL_EraseWrite
(
    const DRV_HANDLE handle,
    DRV_S25FL_COMMAND_HANDLE * commandHandle,
    void * sourceBuffer,
    uint32_t writeBlockStart,
    uint32_t nWriteBlock
);

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
    This routine gets the current status of the command. The application must
    use this routine where the status of a scheduled command needs to be polled
    on. The function may return DRV_S25FL_COMMAND_COMPLETED in a case where the
    command handle has expired. A command handle expires when the internal
    buffer object is re-assigned to another request. It is recommended that
    this function be called regularly in order to track the command status
    correctly.

    The application can alternatively register an event handler to receive the
    command completion events.

  Preconditions:
    The DRV_S25FL_Initialize() routine must have been called.

    The DRV_S25FL_Open() must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    A DRV_S25FL_COMMAND_STATUS value describing the current status of the
    command.  Returns DRV_S25FL_COMMAND_HANDLE_INVALID if the client handle or
    the command handle is not valid.

  Example:
    <code>
    DRV_HANDLE                  handle;         // Returned from DRV_S25FL_Open
    DRV_S25FL_COMMAND_HANDLE      commandHandle;
    DRV_S25FL_COMMAND_STATUS      status;
 
    status = DRV_S25FL_CommandStatus(handle, commandHandle);
    if(status == DRV_S25FL_COMMAND_COMPLETED)
    {
        // Operation Done
    }
    </code>

  Remarks:
    This routine will not block for hardware access and will immediately return
    the current status.
*/

DRV_S25FL_COMMAND_STATUS DRV_S25FL_CommandStatus
(
    const DRV_HANDLE handle, 
    const DRV_S25FL_COMMAND_HANDLE commandHandle
);

// *****************************************************************************
/* Function:
    SYS_FS_MEDIA_GEOMETRY * DRV_S25FL_GeometryGet
    (
        const DRV_HANDLE handle
    );

  Summary:
    Returns the geometry of the device.

  Description:
    This API gives the following geometrical details of the S25FL Flash:
    - Media Property
    - Number of Read/Write/Erase regions in the flash device
    - Number of Blocks and their size in each region of the device

  Precondition:
    The DRV_S25FL_Initialize() routine must have been called for the
    specified S25FL driver instance.

    The DRV_S25FL_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    SYS_FS_MEDIA_GEOMETRY - Pointer to structure which holds the media geometry
    information.

  Example:
    <code> 
    
    SYS_FS_MEDIA_GEOMETRY * s25flFlashGeometry;
    uint32_t readBlockSize, writeBlockSize, eraseBlockSize;
    uint32_t nReadBlocks, nReadRegions, totalFlashSize;

    s25flFlashGeometry = DRV_S25FL_GeometryGet(s25flOpenHandle1);

    readBlockSize  = s25flFlashGeometry->geometryTable->blockSize;
    nReadBlocks = s25flFlashGeometry->geometryTable->numBlocks;
    nReadRegions = s25flFlashGeometry->numReadRegions;

    writeBlockSize  = (s25flFlashGeometry->geometryTable +1)->blockSize;
    eraseBlockSize  = (s25flFlashGeometry->geometryTable +2)->blockSize;

    totalFlashSize = readBlockSize * nReadBlocks * nReadRegions;

    </code>

  Remarks:
    None.
*/

SYS_FS_MEDIA_GEOMETRY * DRV_S25FL_GeometryGet
(
    const DRV_HANDLE handle
);

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
    the driver to call back when queued operation has completed.  When a client
    calls a read, write, erase or a erasewrite function, it is provided with a
    handle identifying the command that was added to the driver's buffer queue.
    The driver will pass this handle back to the client by calling
    "eventHandler" function when the queued operation has completed.
    
    The event handler should be set before the client performs any operations
    that could generate events. The event handler once set, persists until the
    client closes the driver or sets another event handler (which could be a
    "NULL" pointer to indicate no callback).

  Precondition:
    The DRV_S25FL_Initialize() routine must have been called for the
    specified S25FL driver instance.

    The DRV_S25FL_Open() routine must have been called to obtain a valid opened
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
    DRV_S25FL_COMMAND_HANDLE commandHandle;

    // drvS25FLHandle is the handle returned by the DRV_S25FL_Open function.
    // Client registers an event handler with driver. This is done once.

    DRV_S25FL_EventHandlerSet(drvS25FLHandle, APP_S25FLEventHandler, (uintptr_t)&myAppObj);

    DRV_S25FL_Read(drvS25FLHandle, &commandHandle, &myBuffer, blockStart, nBlock);

    if(DRV_S25FL_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event Processing Technique. Event is received when operation is done.

    void APP_S25FLEventHandler
    (
        DRV_S25FL_EVENT event, 
        DRV_S25FL_COMMAND_HANDLE commandHandle,
        uintptr_t contextHandle
    )
    {
        // The context handle was set to an application specific
        // object. It is now retrievable easily in the event handler.
        MY_APP_OBJ myAppObj = (MY_APP_OBJ *) context;

        switch(event)
        {
            case DRV_S25FL_EVENT_COMMAND_COMPLETE:
                // Operation completled successfully.
                break;
            
            case DRV_S25FL_EVENT_COMMAND_ERROR:
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

void DRV_S25FL_EventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
);

// *****************************************************************************
/* Function:
    bool DRV_S25FL_IsAttached
    ( 
        const DRV_HANDLE handle 
    );

  Summary:
    Returns the physical attach status of the S25FL.

  Description:
    This function returns the physical attach status of the S25FL.

  Precondition:
    The DRV_S25FL_Initialize() routine must have been called for the specified
    S25FL driver instance.

    The DRV_S25FL_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    Returns false if the handle is invalid otherwise returns true.

  Example:
    <code> 

    bool isS25FLAttached;
    isS25FLAttached = DRV_S25FL_isAttached(drvS25FLHandle);

    </code>

  Remarks:
    None.
*/

bool DRV_S25FL_IsAttached
(
    const DRV_HANDLE handle
);

// *****************************************************************************
/* Function:
    bool DRV_S25FL_IsWriteProtected
    ( 
        const DRV_HANDLE handle 
    );

  Summary:
    Returns the write protect status of the S25FL.

  Description:
    This function returns the write protect status of the S25FL.

  Precondition:
    The DRV_S25FL_Initialize() routine must have been called for the specified
    S25FL driver instance.

    The DRV_S25FL_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    True - If the flash is write protected.
    False - If the flash is not write protected.

  Example:
    <code>

    bool isWriteProtected;
    isWriteProtected = DRV_S25FL_IsWriteProtected(drvS25FLHandle);

    </code>

  Remarks:
    None.
*/

bool DRV_S25FL_IsWriteProtected
(
    const DRV_HANDLE handle
);

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

  Precondition:
    The DRV_S25FL_Initialize() routine must have been called for the specified
    S25FL driver instance.

    The DRV_S25FL_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    Start address of the S25FL Media if the handle is valid otherwise NULL.

  Example:
    <code>

    uintptr_t startAddress;
    startAddress = DRV_S25FL_AddressGet(drvS25FLHandle);

    </code>

  Remarks:
    None.
*/

uintptr_t DRV_S25FL_AddressGet
(
    const DRV_HANDLE handle
);

#ifdef __cplusplus
}
#endif

#endif // #ifndef _DRV_S25FL_H
/*******************************************************************************
 End of File
*/

