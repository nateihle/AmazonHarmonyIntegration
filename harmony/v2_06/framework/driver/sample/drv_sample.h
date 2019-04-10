/*******************************************************************************
  Sample Device Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sample.h

  Summary:
    Sample Device Driver Interface File

  Description:
    The Sample device driver provides a simple interface to manage the "Sample"
    peripheral.  This file defines the interface definitions and prototypes for
    the Sample driver.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012-2015 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_SAMPLE_H
#define _DRV_SAMPLE_H


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>

#include "system/system.h"
#include "driver/driver.h"


// *****************************************************************************
/* Sample Driver Module Index Numbers

  Summary:
    Sample driver index definitions.

  Description:
    These constants provide Sample Driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_SAMPLE_Initialize and
    DRV_SAMPLE_Open routines to identify the driver instance in use.
*/

#define DRV_SAMPLE_INDEX_0         0
#define DRV_SAMPLE_INDEX_1         1


// *****************************************************************************
/* Sample Driver Client Status

  Summary:
    Identifies the client-specific status of the Sample Driver.

  Description:
    This enumeration identifies the client-specific status of the Sample Driver.

  Remarks:
    None.
*/

typedef enum
{
    /* Client in an invalid state */
    DRV_SAMPLE_CLIENT_STATUS_INVALID
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_ERROR_EXTENDED - 0 /*DOM-IGNORE-END*/,

    /* Unspecified error condition */
    DRV_SAMPLE_CLIENT_STATUS_ERROR
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_ERROR - 0 /*DOM-IGNORE-END*/,

   /* Client is not open */
    DRV_SAMPLE_CLIENT_STATUS_CLOSED
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_CLOSED + 0 /*DOM-IGNORE-END*/,

    /* An operation is currently in progress */
    DRV_SAMPLE_CLIENT_STATUS_BUSY
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_BUSY      /*DOM-IGNORE-END*/,

    /* Up and running, no operations running */
    DRV_SAMPLE_CLIENT_STATUS_READY
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_READY + 0 /*DOM-IGNORE-END*/,

    /* Sample stopped (not running), but ready to accept commands */
    DRV_SAMPLE_CLIENT_STATUS_STOPPED
        /*DOM-IGNORE-BEGIN*/  = DRV_CLIENT_STATUS_READY_EXTENDED + 0 /*DOM-IGNORE-END*/,

} DRV_SAMPLE_CLIENT_STATUS;


// *****************************************************************************
/* Sample Device Driver Initialization Data

  Summary:
    Contains all the data necessary to initialize the sample device.

  Description:
    This structure contains all the data necessary to initialize the sample
    device.

  Remarks:
    A pointer to a structure of this format containing the desired
    initialization data must be passed into the DRV_SAMPLE_Initialize function.
*/

typedef struct _DRV_SAMPLE_INIT
{
    /* System module initialization */
    SYS_MODULE_INIT         moduleInit;

    /* Identifies peripheral (PLIB-level) ID */
    // TODO: emulate - SAMPLE_MODULE_ID        sampleId;

    /* Interrupt Source for Sample module */
    // TODO:  Emulate - INT_SOURCE              interruptSource;

} DRV_SAMPLE_INIT;


// *****************************************************************************
// *****************************************************************************
// Section: System Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SAMPLE_Initialize( const SYS_MODULE_INDEX        index,
                                          const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the Sample Driver.

  Description:
    This function initializes the Sample Driver, making it ready for clients to
    open and use it.

  Precondition:
    None.

  Parameters:
    drvIndex        - Index for the driver instance to be initialized

    init            - Pointer to a data structure containing any data necessary
                      to initialize the driver. This pointer may be null if no
                      data is required because static overrides have been
                      provided.

  Returns:
    If successful, returns a valid handle to a driver object.  Otherwise, it
    returns SYS_MODULE_OBJ_INVALID. The returned object must be passed as
    argument to DRV_SAMPLE_Reinitialize, DRV_SAMPLE_Deinitialize, DRV_SAMPLE_Tasks and
    DRV_SAMPLE_Status functions.

  Example:
    <code>
    DRV_SAMPLE_INIT     init;
    SYS_MODULE_OBJ      objectHandle;

    // Populate the sample initialization structure
    init.moduleInit.value   = SYS_MODULE_POWER_RUN_FULL;
    init.sampleId           = SAMPLE_ID_2;
    init.interruptSource    = INT_SOURCE_SAMPLE_2;

    // Do something

    objectHandle = DRV_SAMPLE_Initialize(DRV_SAMPLE_INDEX_0, (SYS_MODULE_INIT*)&init);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This function must be called before any other sample function is called.

    This function should only be called once during system initialization
    unless DRV_SAMPLE_Deinitialize is called to deinitialize the driver instance.

    This function will NEVER block for hardware access. If the operation requires
    time to allow the hardware to re-initialize, it will be reported by the
    DRV_SAMPLE_Status operation. The system must use DRV_SAMPLE_Status to find out
    when the driver is in the ready state.

    Build configuration options may be used to statically override options in the
    "init" structure and will take precedence over initialization data passed
    using this function.
*/

SYS_MODULE_OBJ DRV_SAMPLE_Initialize ( const SYS_MODULE_INDEX        index,
                                       const SYS_MODULE_INIT * const init );


// *****************************************************************************
/* Function:
    void DRV_SAMPLE_Reinitialize( SYS_MODULE_OBJ                object,
                                  const SYS_MODULE_INIT * const init )

  Summary:
    Reinitializes the driver and refreshes any associated hardware settings.

  Description:
    This function reinitializes the driver and refreshes any associated hardware
    settings using the initialization data given, but it will not interrupt any
    ongoing operations.

  Precondition:
    Function DRV_SAMPLE_Initialize must have been called before calling this
    function and a valid SYS_MODULE_OBJ must have been returned.

  Parameters:
    object          - Driver object handle, returned from the DRV_SAMPLE_Initialize
                      function
    init            - Pointer to the initialization data structure

  Returns:
    None.

  Example:
    <code>
    DRV_SAMPLE_INIT init;
    SYS_MODULE_OBJ  objectHandle;   // Returned from DRV_SAMPLE_Initialize
    SYS_STATUS      sampleStatus;

    // Populate the sample initialization structure
    init.moduleInit.value   = SYS_MODULE_POWER_RUN_FULL;
    init.sampleId           = SAMPLE_ID_2;
    init.interruptSource    = INT_SOURCE_SAMPLE_3;

    DRV_SAMPLE_Reinitialize(objectHandle, (SYS_MODULE_INIT*)&init);

    sampleStatus = DRV_SAMPLE_Status(objectHandle);
    if (SYS_STATUS_BUSY == sampleStatus)
    {
        // Check again later to ensure the driver is ready
    }
    else if (SYS_STATUS_ERROR >= sampleStatus)
    {
        // Handle error
    }
    </code>

  Remarks:
    This function can be called multiple times to reinitialize the module.

    This operation can be used to refresh any supported hardware registers as
    specified by the initialization data or to change the power state of the
    module.

    This function will NEVER block for hardware access. If the operation requires
    time to allow the hardware to re-initialize, it will be reported by the
    DRV_SAMPLE_Status operation. The system must use DRV_SAMPLE_Status to find out
    when the driver is in the ready state.

    Build configuration options may be used to statically override options in the
    "init" structure and will take precedence over initialization data passed
    using this function.
*/

void DRV_SAMPLE_Reinitialize ( SYS_MODULE_OBJ                object,
                               const SYS_MODULE_INIT * const init );


// *****************************************************************************
/* Function:
    void DRV_SAMPLE_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the Sample Driver module.

  Description:
    This function deinitializes the specified instance of the Sample Driver module, 
	disabling its operation (and any hardware), and invalidates all of the internal 
	data.

  Precondition:
    Function DRV_SAMPLE_Initialize must have been called before calling this
    function and a valid SYS_MODULE_OBJ must have been returned.

  Parameters:
    object          - Driver object handle, returned from the DRV_SAMPLE_Initialize
                      function

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     //  Returned from DRV_SAMPLE_Initialize
    SYS_STATUS          status;

    DRV_SAMPLE_Deinitialize(object);

    status = DRV_SAMPLE_Status(object);
    if (SYS_MODULE_DEINITIALIZED == status)
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again.

    This function will NEVER block waiting for hardware. If the operation
    requires time to allow the hardware to complete, this will be reported by
    the DRV_SAMPLE_Status operation.  The system has to use DRV_SAMPLE_Status to find
    out when the module is in the ready state.
*/

void DRV_SAMPLE_Deinitialize ( SYS_MODULE_OBJ object );


// *****************************************************************************
/* Function:
    SYS_STATUS DRV_SAMPLE_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the Sample Driver module

  Description:
    This function provides the current status of the Sample Driver module.

  Precondition:
    DRV_SAMPLE_Initialize must have been called before calling this function.

  Parameters:
    object          - Driver object handle, returned from the
                      DRV_SAMPLE_Initialize function

  Returns:
    - SYS_STATUS_READY          - Indicates that the driver is busy with a
                                  previous system level operation and cannot start
                                  another
                                  Note: Any value greater than SYS_STATUS_READY is
                                  also a normal running state in which the driver
                                  is ready to accept new operations.
    - SYS_STATUS_BUSY           - Indicates that the driver is busy with a
                                  previous system level operation and cannot start
                                  another
    - SYS_STATUS_ERROR          - Indicates that the driver is in an error state
                                  Note:  Any value less than SYS_STATUS_ERROR is
                                  also an error state.

    - SYS_MODULE_DEINITIALIZED  - Indicates that the driver has been deinitialized
                                  Note:  This value is less than SYS_STATUS_ERROR

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_SAMPLE_Initialize
    SYS_STATUS          status;

    status = DRV_SAMPLE_Status(object);
    else if (SYS_STATUS_ERROR >= status)
    {
        // Handle error
    }
    </code>

  Remarks:
    The this operation can be used to determine when any of the driver's module
    level operations has completed.

    If the status operation returns SYS_STATUS_BUSY, the a previous operation
    has not yet completed.  Once the status operation returns SYS_STATUS_READY,
    any previous operations have completed.

    The value of SYS_STATUS_ERROR is negative (-1).  Any value less than that is
    also an error state.

    This function will NEVER block waiting for hardware.

    If the Status operation returns an error value, the error may be cleared by
    calling the reinitialize operation.  If that fails, the deinitialize
    operation will need to be called, followed by the initialize operation to
    return to normal operations.
*/

SYS_STATUS DRV_SAMPLE_Status ( SYS_MODULE_OBJ object );


// *****************************************************************************
/* Function:
    void DRV_SAMPLE_Tasks( SYS_MODULE_OBJ object )

  Summary:
    Maintains the driver's state machine and implements its ISR.

  Description:
    This function is used to maintain the driver's internal state machine and
    implement its ISR for interrupt-driven implementations.

  Precondition:
    The DRV_SAMPLE_Initialize function must have been called for the specified
    Sample Driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_SAMPLE_Initialize)

  Returns:
    None

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_SAMPLE_Initialize

    while (true)
    {
        DRV_SAMPLE_Tasks (object);

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the system's Tasks function (SYS_Tasks) or by the appropriate raw
    ISR.

    This function may execute in an ISR context and will never block or access any
    resources that may cause it to block.
*/

void DRV_SAMPLE_Tasks( SYS_MODULE_OBJ object );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client Level
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_SAMPLE_Open( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified Sample Driver instance and returns a handle to it.

  Description:
    This function opens the specified Sample Driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver.

  Precondition:
    DRV_SAMPLE_Initialize must have been called before calling this function.

  Parameters:
    drvIndex    - Identifier for the object instance to be opened
    intent      - Zero or more of the values from the enumeration
                  DRV_IO_INTENT "ORed" together to indicate the intended use
                  of the driver

  Returns:
    If successful, the function returns a valid open-instance handle (a number
    identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID.

  Example:
    <code>
    DRV_HANDLE  handle;

    handle = DRV_SAMPLE_Open(DRV_SAMPLE_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
    }
    </code>

  Remarks:
    The handle returned is valid until the DRV_SAMPLE_Close function is called.

    This function will NEVER block waiting for hardware.

    If the DRV_IO_INTENT_BLOCKING is requested and the driver was built
    appropriately to support blocking behavior, then other client-level
    operations may block waiting on hardware until they are complete.

    If DRV_IO_INTENT_NON_BLOCKING is requested the driver client can call the
    DRV_SAMPLE_ClientStatus operation to find out when the module is in the ready
    state.

    If the requested intent flags are not supported, the function will return
    DRV_HANDLE_INVALID.
*/

DRV_HANDLE DRV_SAMPLE_Open( const SYS_MODULE_INDEX drvIndex,
                            const DRV_IO_INTENT    intent );


// *****************************************************************************
/* Function:
    void DRV_SAMPLE_Close( DRV_HANDLE handle )

  Summary:
    Closes an opened-instance of the Sample Driver.

  Description:
    This function closes an opened-instance of the Sample Driver, invalidating the
    handle.

  Precondition:
    The DRV_SAMPLE_Initialize function must have been called for the specified
    Sample Driver instance.

    DRV_SAMPLE_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_SAMPLE_Open

    DRV_SAMPLE_Close(handle);
    </code>

  Remarks:
    After calling this function, the handle passed in "handle" must not be used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling DRV_SAMPLE_Open before the caller may use the driver again.

    If DRV_IO_INTENT_BLOCKING was requested and the driver was built
    appropriately to support blocking behavior call may block until the
    operation is complete.

    If DRV_IO_INTENT_NON_BLOCKING request the driver client can call the
    DRV_SAMPLE_Status operation to find out when the module is in
    the ready state (the handle is no longer valid).

    Usually there is no need for the driver client to verify that the Close
    operation has completed.
*/

void DRV_SAMPLE_Close( DRV_HANDLE handle );


// *****************************************************************************
/* Function:
    DRV_SAMPLE_CLIENT_STATUS DRV_SAMPLE_ClientStatus( DRV_HANDLE handle )

  Summary:
    Gets current client-specific status the Sample Driver.

  Description:
    This function gets the client-specific status of the Sample Driver associated
    with the given handle.

  Precondition:
    The DRV_SAMPLE_Initialize function must have been called.

    DRV_SAMPLE_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    A DRV_SAMPLE_CLIENT_STATUS value describing the current status of the driver.

  Example:
    <code>
    DRV_HANDLE sampleHandle;  // Returned from DRV_SAMPLE_Open
    DRV_SAMPLE_CLIENT_STATUS sampleClientStatus;

    sampleClientStatus = DRV_SAMPLE_ClientStatus(sampleHandle);
    if(DRV_SAMPLE_CLIENT_STATUS_ERROR >= sampleClientStatus)
    {
        // Handle the error
    }
    </code>

  Remarks:
    This function will not block for hardware access and will immediately return
    the current status.
*/

DRV_SAMPLE_CLIENT_STATUS DRV_SAMPLE_ClientStatus( DRV_HANDLE handle );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client & Module Level
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Included Files (continued)
// *****************************************************************************
// *****************************************************************************
/*  The file included below maps the interface definitions above to appropriate
    static implementations, depending on build mode.
*/

// TODO:  #include "sample/drv_sample_mapping.h"


#endif // #ifndef _DRV_SAMPLE_H

/*******************************************************************************
 End of File
*/