/*******************************************************************************
  EEPROM Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_eeprom.h

  Summary:
    EEPROM Driver Interface Definition

  Description:
    The EEPROM driver provides a simple interface to manage the EEPROM Memory on
    Microchip microcontrollers. This file defines the interface definition for
    the EEPROM driver.
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
#ifndef _DRV_EEPROM_H
#define _DRV_EEPROM_H

// *****************************************************************************
// *****************************************************************************
// Section: File includes
// *****************************************************************************
// *****************************************************************************
#include "system/common/sys_common.h"
#include "driver/driver_common.h"
#include "system/common/sys_module.h"
#include "osal/osal.h"
#include "peripheral/nvm/plib_nvm.h"
#include "system/int/sys_int.h"
#include "system/clk/sys_clk.h"
#include "system/fs/sys_fs_media_manager.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
/* EEPROM Driver command handle.

  Summary:
    Handle identifying commands queued in the driver.

  Description:
    A command handle is returned by a call to the read or write functions.
    This handle allows the application to track the completion of the
    operation. This command handle is also returned to the client along with
    the event that has occurred with respect to the command. This allows the
    application to connect the event to a specific command in case where
    multiple commands are queued.

    The command handle associated with the command request expires when the
    client has been notified of the completion of the command (after event
    handler function that notifies the client returns) or after the command has
    been retired by the driver if no event handler callback was set. 

  Remarks:
    None.
*/

typedef SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE DRV_EEPROM_COMMAND_HANDLE;

// *****************************************************************************
/* EEPROM Driver Invalid Command Handle.

  Summary:
    This value defines the EEPROM Driver's Invalid Command Handle.

  Description:
    This value defines the EEPROM Driver Invalid Command Handle. This value is
    returned by read or write routines when the command request was not
    accepted.

  Remarks:
    None.
*/

#define DRV_EEPROM_COMMAND_HANDLE_INVALID SYS_FS_MEDIA_BLOCK_COMMAND_HANDLE_INVALID

// *****************************************************************************
/* EEPROM Driver Events

  Summary
    Identifies the possible events that can result from a request.

  Description
    This enumeration identifies the possible events that can result from a
    read or write request caused by the client.

  Remarks:
    One of these values is passed in the "event" parameter of the event
    handling callback function that client registered with the driver by
    calling the DRV_EEPROM_EventHandlerSet function when a request is completed.
*/

typedef enum
{
    /* Operation has been completed successfully. */
    DRV_EEPROM_EVENT_COMMAND_COMPLETE = SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_COMPLETE,

    /* There was an error during the operation */
    DRV_EEPROM_EVENT_COMMAND_ERROR = SYS_FS_MEDIA_EVENT_BLOCK_COMMAND_ERROR 

} DRV_EEPROM_EVENT;

// ***********************************************************************
/* EEPROM Driver Command Status

  Summary:
    Specifies the status of the command for read or write requests.
	
  Description:
    EEPROM Driver command Status
    
    This type specifies the status of the command for the read or write
    requests.
	
  Remarks:
    None.                                                               
*/

typedef enum
{
    /* Done OK and ready */
    DRV_EEPROM_COMMAND_COMPLETED     = SYS_FS_MEDIA_COMMAND_COMPLETED,

    /* Scheduled but not started */
    DRV_EEPROM_COMMAND_QUEUED        = SYS_FS_MEDIA_COMMAND_QUEUED,

    /* Currently being in transfer */
    DRV_EEPROM_COMMAND_IN_PROGRESS   = SYS_FS_MEDIA_COMMAND_IN_PROGRESS,

    /* Unknown Command */
    DRV_EEPROM_COMMAND_ERROR_UNKNOWN = SYS_FS_MEDIA_COMMAND_UNKNOWN,

} DRV_EEPROM_COMMAND_STATUS;

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver EEPROM Module Index reference

  Summary:
    EEPROM driver index definition

  Description:
    This constant provides EEPROM driver index definition.

  Remarks:
    This constant should be used in place of hard-coded numeric literals. This
    value should be passed into the DRV_EEPROM_Initialize and DRV_EEPROM_Open
    routines to identify the driver instance in use.
*/

#define  DRV_EEPROM_INDEX_0      0

// *****************************************************************************
/* EEPROM Driver Initialization Data

  Summary:
    Defines the data required to initialize the EEPROM driver

  Description:
    This data type defines the data required to initialize the EEPROM driver.

  Remarks:
    None.
*/

typedef struct
{
    /* System module initialization */
    SYS_MODULE_INIT  moduleInit;

    /* Identifies hardware module (PLIB-level) ID */
    NVM_MODULE_ID    eepromId;

    /* EEPROM Media geometry object. */
    const SYS_FS_MEDIA_GEOMETRY *eepromMediaGeometry;

} DRV_EEPROM_INIT;

// *****************************************************************************
/* EEPROM Driver Event Handler Function Pointer

  Summary
    Pointer to a EEPROM Driver Event handler function

  Description
    This data type defines the required function signature for the EEPROM event
    handling callback function. A client must register a pointer to an event
    handling function whose function signature (parameter and return value 
    types) match the types specified by this function pointer in order to 
    receive event callbacks from the driver.
    
    The parameters and return values are described here and a partial example
    implementation is provided.

  Parameters:
    event           - Identifies the type of event
    
    commandHandle   - Handle returned from the Read or Write requests
    
    context         - Value identifying the context of the application that
                      registered the event handling function

  Returns:
    None.

  Example:
    <code>
    void APP_MyEepromEventHandler
    (
        DRV_EEPROM_EVENT event,
        DRV_EEPROM_COMMAND_HANDLE commandHandle,
        uintptr_t context
    )
    {
        MY_APP_DATA_STRUCT pAppData = (MY_APP_DATA_STRUCT) context;
        
        switch(event)
        {
            case DRV_EEPROM_EVENT_COMMAND_COMPLETE:

                // Handle the completed buffer. 
                break;
            
            case DRV_EEPROM_EVENT_COMMAND_ERROR:
            default:

                // Handle error.
                break;
        }
    }
    </code>

  Remarks:
    If the event is DRV_EEPROM_EVENT_COMMAND_COMPLETE, it means that the
    scheduled operation was completed successfully. 
    
    If the event is DRV_EEPROM_EVENT_COMMAND_ERROR, it means that the scheduled
    operation was not completed successfully.
     
    The context parameter contains the handle to the client context, provided
    at the time the event handling function was registered using the
    DRV_EEPROM_EventHandlerSet function. This context handle value is passed
    back to the client as the "context" parameter. It can be any value
    necessary to identify the client context or instance (such as a pointer to
    the client's data) instance of the client that scheduled the request.

    The event handler function executes in the driver's context. It is
    recommended of the application to not perform process intensive or blocking
    operations within this function.
*/

typedef SYS_FS_MEDIA_EVENT_HANDLER DRV_EEPROM_EVENT_HANDLER;

// *****************************************************************************
// *****************************************************************************
// Section: EEPROM Driver Module Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_EEPROM_Initialize
    ( 
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init 
    );
    
  Summary:
    Initializes the EEPROM instance for the specified driver index.

  Description:
    This routine initializes the EEPROM driver instance for the specified
    driver index, making it ready for clients to open and use it.

  Precondition:
    None.
  
  Parameters:
    index -  Identifier for the instance to be initialized.
    init  -  Pointer to a data structure containing any data necessary to
             initialize the driver.
  
  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise it returns SYS_MODULE_OBJ_INVALID.
  
  Example:
    <code>
    // This code snippet shows an example of initializing the EEPROM Driver.
    
    SYS_MODULE_OBJ  objectHandle;

    SYS_FS_MEDIA_REGION_GEOMETRY EEPROMGeometryTable[3] = 
    {
        {
            .blockSize = 4,
            .numBlocks = (DRV_EEPROM_MEDIA_SIZE * 1024),
        },
        {
            .blockSize = 4,
            .numBlocks = ((DRV_EEPROM_MEDIA_SIZE * 1024)/4)
        },
        {
            .blockSize = 4,
            .numBlocks = ((DRV_EEPROM_MEDIA_SIZE * 1024)/4)
        }
    };

    const SYS_FS_MEDIA_GEOMETRY EEPROMGeometry = 
    {
        .mediaProperty = SYS_FS_MEDIA_WRITE_IS_BLOCKING,
        .numReadRegions = 1,
        .numWriteRegions = 1,
        .numEraseRegions = 1,
        .geometryTable = (SYS_FS_MEDIA_REGION_GEOMETRY *)&EEPROMGeometryTable
    };

    // EEPROM Driver Initialization Data
    const DRV_EEPROM_INIT drvEepromInit =
    {
        .moduleInit.sys.powerState = SYS_MODULE_POWER_RUN_FULL,
        .eepromId = NVM_ID_0,
        .eepromMediaGeometry = (SYS_FS_MEDIA_GEOMETRY *)&EEPROMGeometry
    };

    objectHandle = DRV_EEPROM_Initialize(DRV_EEPROM_INDEX_0, (SYS_MODULE_INIT*)&drvEepromInit);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This routine must be called before any other EEPROM routine is called.
    
    This routine should only be called once during system initialization unless
    DRV_EEPROM_Deinitialize is called to deinitialize the driver instance.
    
    This routine will NEVER block for hardware access. The system must use
    DRV_EEPROM_Status to find out when the driver is in the ready state.
    
    Build configuration options may be used to statically override options in
    the "init" structure and will take precedence over initialization data
    passed using this routine.                                                   
*/

SYS_MODULE_OBJ DRV_EEPROM_Initialize
(
    const SYS_MODULE_INDEX index,
    const SYS_MODULE_INIT * const init
);

// ****************************************************************************
/* Function:
    void DRV_EEPROM_Deinitialize
    (
        SYS_MODULE_OBJ object 
    );
    
  Summary:
    Deinitializes the specified instance of the EEPROM driver module

  Description:
    Deinitializes the specified instance of the EEPROM driver module, disabling
    its operation. Invalidates all the internal data.
  
  Preconditions:
    Function DRV_EEPROM_Initialize should have been called before calling
    this function.
  
  Parameter:
    object -  Driver object handle, returned from the DRV_EEPROM_Initialize
              routine

  Returns:
    None.

  Example:
    <code>
    // This code snippet shows an example of deinitializing the driver.
    
    SYS_MODULE_OBJ      object;     //  Returned from DRV_EEPROM_Initialize
    SYS_STATUS          status;
    
    DRV_EEPROM_Deinitialize(object);
    
    status = DRV_EEPROM_Status(object);
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

void DRV_EEPROM_Deinitialize
(
    SYS_MODULE_OBJ object
);

// *************************************************************************
/* Function:
    SYS_STATUS DRV_EEPROM_Status
    (
        SYS_MODULE_OBJ object
    );
    
  Summary:
    Gets the current status of the EEPROM driver module.
  
  Description:
    This routine provides the current status of the EEPROM driver module.
  
  Preconditions:
    Function DRV_EEPROM_Initialize should have been called before calling this
    function.
  
  Parameters:
    object -  Driver object handle, returned from the DRV_EEPROM_Initialize
              routine
  
  Returns:
    SYS_STATUS_READY - Indicates that the driver is ready and accept requests
                       for new operations.
    
    SYS_STATUS_UNINITIALIZED - Indicates the driver is not initialized.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_EEPROM_Initialize
    SYS_STATUS          EEPROMStatus;
    
    EEPROMStatus = DRV_EEPROM_Status(object);
    if (EEPROMStatus == SYS_STATUS_READY)
    {
        // Driver is ready to perform operations.
    }
    else
    {
        // Driver is not ready.
    }
    </code>
  
  Remarks:
    None.
*/

SYS_STATUS DRV_EEPROM_Status
(
    SYS_MODULE_OBJ object
);

// ****************************************************************************
/* Function:
    void DRV_EEPROM_Tasks 
    (
        SYS_MODULE_OBJ object
    );
    
  Summary:
    Handles the read or write requests queued to the driver.
  
  Description:
    This routine is used to handle the read or write requests queued to
    the driver.
  
  Preconditions:
    The DRV_EEPROM_Initialize routine must have been called for the specified
    EEPROM driver instance.
  
  Parameters:
    object -  Object handle for the specified driver instance (returned from
              DRV_EEPROM_Initialize)
  Returns:
    None.
  
  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_EEPROM_Initialize
    
    while (true)
    {
        DRV_EEPROM_Tasks (object);
        // Do other tasks
    }
    </code>

  Remarks:
    This routine is normally not called directly by an application. It is
    called by the system's Tasks routine (SYS_Tasks).
*/

void DRV_EEPROM_Tasks
(
    SYS_MODULE_OBJ object
);

// *****************************************************************************
// *****************************************************************************
// Section: EEPROM Driver Client Routines
// *****************************************************************************
// *****************************************************************************

// ****************************************************************************
/* Function:
    DRV_HANDLE DRV_EEPROM_Open
    ( 
        const SYS_MODULE_INDEX index,
        const DRV_IO_INTENT ioIntent
    );
    
  Summary:
    Opens the specified EEPROM driver instance and returns a handle to it
  
  Description:
    This routine opens the specified EEPROM driver instance and provides a
    handle. This handle must be provided to all other client-level operations
    to identify the caller and the instance of the driver.
  
  Preconditions:
    DRV_EEPROM_Initialize must have been called before calling this function.
  
  Parameters:
    index  - Identifier for the object instance to be opened
    intent - Zero or more of the values from the enumeration DRV_IO_INTENT
             "ORed" together to indicate the intended use of the driver
  
  Returns:
    If successful, the routine returns a valid open-instance handle (a
    number identifying both the caller and the module instance).
    
    If an error occurs, DRV_HANDLE_INVALID is returned. Errors can occur
    under the following circumstances:
    	- if the number of client objects allocated via DRV_EEPROM_CLIENTS_NUMBER 
          is insufficient
    	- if the client is trying to open the driver but driver has been opened
    	  exclusively by another client
        - if the client is trying to open the driver exclusively, but has
          already been opened in a non exclusive mode by another client.
        - if the driver hardware instance being opened is invalid
  
  Example:
    <code>
    DRV_HANDLE handle;
    
    handle = DRV_EEPROM_Open(DRV_EEPROM_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
    }
    </code>
  
  Remarks:
    The handle returned is valid until the DRV_EEPROM_Close routine is called.
    This routine will NEVER block waiting for hardware. If the driver has
    already been opened, it cannot be opened exclusively.
*/

DRV_HANDLE DRV_EEPROM_Open
(
    const SYS_MODULE_INDEX index, 
    const DRV_IO_INTENT ioIntent
);

// *****************************************************************************
/* Function:
    void DRV_EEPROM_Close
    (
        const DRV_HANDLE handle
    );

  Summary:
    Closes an opened-instance of the EEPROM driver

  Description:
    This routine closes an opened-instance of the EEPROM driver, invalidating the
    handle.

  Precondition:
    The DRV_EEPROM_Initialize routine must have been called for the specified
    EEPROM driver instance.

    DRV_EEPROM_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_EEPROM_Open

    DRV_EEPROM_Close(handle);
    </code>

  Remarks:
    After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines. A new handle must be obtained by
    calling DRV_EEPROM_Open before the caller may use the driver again. Usually
    there is no need for the driver client to verify that the Close operation
    has completed.
*/

void DRV_EEPROM_Close
(
    const DRV_HANDLE handle
);

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

  Precondition:
    The DRV_EEPROM_Initialize routine must have been called for the specified
    EEPROM driver instance.

    DRV_EEPROM_Open must have been called with DRV_IO_INTENT_READ or
    DRV_IO_INTENT_READWRITE as the ioIntent to obtain a valid opened device
    handle.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle
                   
    buffer        - Buffer into which the data read from the EEPROM memory will
                    be placed

    blockStart    - Start block address in EEPROM memory from where the read
                    should begin.

    nBlock        - Total number of blocks to be read.

  Returns:
    If the request was queued successfully then a valid command handle is
    returned in the commandHandle argument. Otherwise
    DRV_EEPROM_COMMAND_HANDLE_INVALID is returned if the request was not
    successful.

  Example:
    <code>

    uint8_t myBuffer[MY_BUFFER_SIZE];
    // address should be block aligned.
    uint32_t blockStart = EEPROM_BASE_ADDRESS_TO_READ_FROM;
    uint32_t nBlock = 2;
    DRV_EEPROM_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;    

    // myEEPROMHandle is the handle returned by the DRV_EEPROM_Open function.
    DRV_EEPROM_EventHandlerSet(myEEPROMHandle, APP_EEPROMEventHandler, (uintptr_t)&myAppObj);
    DRV_EEPROM_Read(myEEPROMHandle, &commandHandle, &myBuffer, blockStart, nBlock);

    if(DRV_EEPROM_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }
    else
    {
        // Read queued successfully.
    }

    // Event is received when the buffer is processed.

    void APP_EEPROMEventHandler
    (
        DRV_EEPROM_EVENT event, 
        DRV_EEPROM_COMMAND_HANDLE commandHandle, 
        uintptr_t context
    )
    {
        // context points to myAppObj.

        switch(event)
        {
            case DRV_EEPROM_EVENT_COMMAND_COMPLETE:

                // This means the data was transferred. 
                break;
            
            case DRV_EEPROM_EVENT_COMMAND_ERROR:

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

void DRV_EEPROM_Read
(
    const DRV_HANDLE handle,
    DRV_EEPROM_COMMAND_HANDLE * commandHandle,
    void * buffer,
    uint32_t blockStart,
    uint32_t nBlock
);

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

  Precondition:
    The DRV_EEPROM_Initialize() routine must have been called for the specified
    EEPROM driver instance.

    DRV_EEPROM_Open() routine must have been called to obtain a valid opened
    device handle. DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have
    been specified as a parameter to this routine.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle
                   
    buffer        - The buffer containing data to be programmed into EEPROM
                    memory

    blockStart    - Start block address of EEPROM memory where the write should
                    begin.

    nBlock        - Total number of blocks to be written. 

  Returns:
    If the request was queued successfully then a valid command handle is
    returned in the commandHandle argument. Otherwise
    DRV_EEPROM_COMMAND_HANDLE_INVALID is returned if the request was not
    successful.

  Example:
    <code>
    
    uint8_t myBuffer[MY_BUFFER_SIZE];
    
    uint32_t blockStart = EEPROM_BASE_ADDRESS_TO_WRITE_TO;
    uint32_t nBlock = 2;
    DRV_EEPROM_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;    

    // myEEPROMHandle is the handle returned by the DRV_EEPROM_Open function.
    // Client registers an event handler with driver

    DRV_EEPROM_EventHandlerSet(myEEPROMHandle, APP_EEPROMEventHandler, (uintptr_t)&myAppObj);
    DRV_EEPROM_Write(myEEPROMHandle, &commandHandle, &myBuffer, blockStart, nBlock);

    if(DRV_EEPROM_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when the buffer is processed.

    void APP_EEPROMEventHandler
    (
        DRV_EEPROM_EVENT event, 
        DRV_EEPROM_COMMAND_HANDLE commandHandle, 
        uintptr_t context
    )
    {
        // context points to myAppObj.
        switch(event)
        {
            case DRV_EEPROM_EVENT_COMMAND_COMPLETE:

                // This means the data was transferred. 
                break;
            
            case DRV_EEPROM_EVENT_COMMAND_ERROR:

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

void DRV_EEPROM_Write
(
    const DRV_HANDLE handle,
    DRV_EEPROM_COMMAND_HANDLE * commandHandle,
    void * buffer,
    uint32_t blockStart,
    uint32_t nBlock
);

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

  Precondition:
    The DRV_EEPROM_Initialize() routine must have been called for the specified
    EEPROM driver instance.

    DRV_EEPROM_Open() routine must have been called to obtain a valid opened
    device handle. DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have
    been specified as a parameter to this routine.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle
                   
    blockStart    - block start addess for the erase operation.

    nBlock        - Total number of blocks to be erased. 

  Returns:
    If the request was queued successfully then a valid command handle is
    returned in the commandHandle argument. Otherwise
    DRV_EEPROM_COMMAND_HANDLE_INVALID is returned if the request was not
    successful.
  Example:
    <code>
    
    uint32_t blockStart = 0;
    uint32_t nBlock = 2;
    DRV_EEPROM_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;    

    // myEEPROMHandle is the handle returned by the DRV_EEPROM_Open function.
    // Client registers an event handler with driver

    DRV_EEPROM_EventHandlerSet(myEEPROMHandle, APP_EEPROMEventHandler, (uintptr_t)&myAppObj);
    DRV_EEPROM_Erase(myEEPROMHandle, &commandHandle, blockStart, nBlock);

    if(DRV_EEPROM_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when the buffer is processed.

    void APP_EEPROMEventHandler
    (
        DRV_EEPROM_EVENT event, 
        DRV_EEPROM_COMMAND_HANDLE commandHandle, 
        uintptr_t context
    )
    {
        // context points to myAppObj.
        switch(event)
        {
            case DRV_EEPROM_EVENT_COMMAND_COMPLETE:

                // Erase operation is complete.
                break;
            
            case DRV_EEPROM_EVENT_COMMAND_ERROR:

                // Erase operation failed.
                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    None
*/

void DRV_EEPROM_Erase
(
    const DRV_HANDLE handle,
    DRV_EEPROM_COMMAND_HANDLE *commandHandle,
    uint32_t blockStart,
    uint32_t nBlock
);


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
    Data EEPROM. The function returns with a valid handle in the commandHandle
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

  Precondition:
    The DRV_EEPROM_Initialize() routine must have been called for the specified
    EEPROM driver instance.

    DRV_EEPROM_Open() routine must have been called to obtain a valid opened
    device handle. DRV_IO_INTENT_WRITE or DRV_IO_INTENT_READWRITE must have
    been specified as a parameter to this routine.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open function

    commandHandle - Pointer to an argument that will contain the return buffer
                    handle
                   
  Returns:
    If the request was queued successfully then a valid command handle is
    returned in the commandHandle argument. Otherwise
    DRV_EEPROM_COMMAND_HANDLE_INVALID is returned if the request was not
    successful.
  Example:
    <code>
    
    DRV_EEPROM_COMMAND_HANDLE commandHandle;
    MY_APP_OBJ myAppObj;    

    // myEEPROMHandle is the handle returned by the DRV_EEPROM_Open function.
    // Client registers an event handler with driver

    DRV_EEPROM_EventHandlerSet(myEEPROMHandle, APP_EEPROMEventHandler, (uintptr_t)&myAppObj);
    DRV_EEPROM_BulkErase(myEEPROMHandle, &commandHandle);

    if(DRV_EEPROM_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event is received when the buffer is processed.

    void APP_EEPROMEventHandler
    (
        DRV_EEPROM_EVENT event, 
        DRV_EEPROM_COMMAND_HANDLE commandHandle, 
        uintptr_t context
    )
    {
        // context points to myAppObj.
        switch(event)
        {
            case DRV_EEPROM_EVENT_COMMAND_COMPLETE:

                // Bulk Erase operation is complete.
                break;
            
            case DRV_EEPROM_EVENT_COMMAND_ERROR:

                // Bulk Erase operation failed.
                break;

            default:
                break;
        }
    }

    </code>

  Remarks:
    None
  Remarks:
    Refer to drv_eeprom.h for usage information.
*/

void DRV_EEPROM_BulkErase
(
    const DRV_HANDLE handle,
    DRV_EEPROM_COMMAND_HANDLE *commandHandle
);

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

  Preconditions:
    The DRV_EEPROM_Initialize() routine must have been called.

    The DRV_EEPROM_Open() must have been called to obtain a valid opened device
    handle.

  Parameters:
    handle        - A valid open-instance handle, returned from the driver's
                    open routine

    commandHandle - A valid command handle returned from read, write or erase
                    request.

  Returns:
    A DRV_EEPROM_COMMAND_STATUS value describing the current status of the command.

  Example:
    <code>
    DRV_HANDLE                handle;         // Returned from DRV_EEPROM_Open
    DRV_EEPROM_COMMAND_HANDLE commandHandle;
    DRV_EEPROM_COMMAND_STATUS status;
 
    status = DRV_EEPROM_CommandStatus(handle, commandHandle);
    if(status == DRV_EEPROM_COMMAND_COMPLETED)
    {
        // Operation Done
    }
    </code>

  Remarks:
    This routine will not block for hardware access and will immediately return
    the current status.
*/

DRV_EEPROM_COMMAND_STATUS DRV_EEPROM_CommandStatus
(
    const DRV_HANDLE handle, 
    const DRV_EEPROM_COMMAND_HANDLE commandHandle
);

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

  Precondition:
    The DRV_EEPROM_Initialize() routine must have been called for the
    specified EEPROM driver instance.

    The DRV_EEPROM_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    SYS_FS_MEDIA_GEOMETRY - Pointer to structure which holds the media geometry
    information.

  Example:
    <code> 
    
    SYS_FS_MEDIA_GEOMETRY * eepromGeometry;
    uint32_t readBlockSize, writeBlockSize, eraseBlockSize;
    uint32_t nReadBlocks, nReadRegions, totalSize;

    eepromGeometry = DRV_EEPROM_GeometryGet(eepromOpenHandle1);

    readBlockSize  = eepromGeometry->geometryTable->blockSize;
    nReadBlocks = eepromGeometry->geometryTable->numBlocks;
    nReadRegions = eepromGeometry->numReadRegions;

    writeBlockSize  = (eepromGeometry->geometryTable +1)->blockSize;
    eraseBlockSize  = (eepromGeometry->geometryTable +2)->blockSize;

    totalSize = readBlockSize * nReadBlocks * nReadRegions;

    </code>

  Remarks:
    None.
*/

SYS_FS_MEDIA_GEOMETRY * DRV_EEPROM_GeometryGet
(
    const DRV_HANDLE handle
);

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

  Precondition:
    The DRV_EEPROM_Initialize() routine must have been called for the specified
    EEPROM driver instance.

    The DRV_EEPROM_Open() routine must have been called to obtain a valid opened
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
    DRV_EEPROM_COMMAND_HANDLE commandHandle;

    // drvEEPROMHandle is the handle returned by the DRV_EEPROM_Open function.
    // Client registers an event handler with driver. This is done once.

    DRV_EEPROM_EventHandlerSet(drvEEPROMHandle, APP_EEPROMEventHandler, (uintptr_t)&myAppObj);

    DRV_EEPROM_Read(drvEEPROMHandle, &commandHandle, &myBuffer, blockStart, nBlock);

    if(DRV_EEPROM_COMMAND_HANDLE_INVALID == commandHandle)
    {
        // Error handling here
    }

    // Event Processing Technique. Event is received when operation is done.

    void APP_EEPROMEventHandler
    (
        DRV_EEPROM_EVENT event, 
        DRV_EEPROM_COMMAND_HANDLE commandHandle, 
        uintptr_t context
    )
    {
        // The context handle was set to an application specific
        // object. It is now retrievable easily in the event handler.
        MY_APP_OBJ myAppObj = (MY_APP_OBJ *) context;

        switch(event)
        {
            case DRV_EEPROM_EVENT_COMMAND_COMPLETE:

                // This means the data was transferred. 
                break;
            
            case DRV_EEPROM_EVENT_COMMAND_ERROR:

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

void DRV_EEPROM_EventHandlerSet
(
    const DRV_HANDLE handle,
    const void * eventHandler,
    const uintptr_t context
);

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

  Precondition:
    The DRV_EEPROM_Initialize() routine must have been called for the specified 
    EEPROM driver instance.

    The DRV_EEPROM_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    Returns true always

  Example:
    <code> 

    // The EEPROM media is always attached and so the below always returns
    // true.
    
    bool isEEPROMAttached;
    isEEPROMAttached = DRV_EEPROM_isAttached(drvEEPROMHandle);

    </code>

  Remarks:
    None.
*/

bool DRV_EEPROM_IsAttached
(
    const DRV_HANDLE handle
);

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

  Precondition:
    The DRV_EEPROM_Initialize() routine must have been called for the specified 
    EEPROM driver instance.

    The DRV_EEPROM_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    Always returns false.

  Example:
    <code>

    // The EEPROM media is treated as always writeable.
    bool isWriteProtected;
    isWriteProtected = DRV_EEPROM_IsWriteProtected(drvEEPROMHandle);

    </code>

  Remarks:
    None.
*/

bool DRV_EEPROM_IsWriteProtected
(
    const DRV_HANDLE handle
);

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

  Precondition:
    The DRV_EEPROM_Initialize() routine must have been called for the specified 
    EEPROM driver instance.

    The DRV_EEPROM_Open() routine must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    Start address of the EEPROM Media if the handle is valid otherwise NULL.

  Example:
    <code>

    uintptr_t startAddress;
    startAddress = DRV_EEPROM_AddressGet(drvEEPROMHandle);

    </code>

  Remarks:
    None.
*/

uintptr_t DRV_EEPROM_AddressGet
(
    const DRV_HANDLE handle
);

#ifdef __cplusplus
}
#endif

#endif // #ifndef _DRV_EEPROM_H
/*******************************************************************************
 End of File
*/

