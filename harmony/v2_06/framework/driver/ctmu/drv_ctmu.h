/*******************************************************************************
  CTMU Device Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ctmu.h

  Summary:
    CTMU Device Driver Interface File

  Description:
    The CTMU device driver provides a simple interface to manage the "CTMU"
    peripheral.  This file defines the interface definitions and prototypes for
    the CTMU driver.
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

#ifndef _DRV_CTMU_H
#define _DRV_CTMU_H


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "system_config.h"
#include "peripheral/ctmu/plib_ctmu.h"

#include "system/system.h"
#include "driver/driver.h"


// *****************************************************************************
/* CTMU Driver Module Index Numbers

  Summary:
    CTMU driver index definitions.

  Description:
    These constants provide CTMU Driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_CTMU_Initialize and
    DRV_CTMU_Open routines to identify the driver instance in use.
*/

#define DRV_CTMU_INDEX_0         0



// *****************************************************************************
// *****************************************************************************
// Section: System Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_CTMU_Initialize( const SYS_MODULE_INDEX        index,
                                          const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the CTMU Driver.

  Description:
    This function initializes the CTMU Driver, making it ready for clients to
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
    argument to DRV_CTMU_Reinitialize, DRV_CTMU_Deinitialize, DRV_CTMU_Tasks and
    DRV_CTMU_Status functions.

  Example:
    <code>
    DRV_CTMU_INIT     init;
    SYS_MODULE_OBJ      objectHandle;

    // Populate the CTMU initialization structure
    init.moduleInit.value   = SYS_MODULE_POWER_RUN_FULL;
    init.CTMUId           = CTMU_ID_2;
    init.interruptSource    = INT_SOURCE_CTMU_2;

    // Do something

    objectHandle = DRV_CTMU_Initialize(DRV_CTMU_INDEX_0, (SYS_MODULE_INIT*)&init);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This function must be called before any other CTMU function is called.

    This function should only be called once during system initialization
    unless DRV_CTMU_Deinitialize is called to deinitialize the driver instance.

    This function will NEVER block for hardware access. If the operation requires
    time to allow the hardware to re-initialize, it will be reported by the
    DRV_CTMU_Status operation. The system must use DRV_CTMU_Status to find out
    when the driver is in the ready state.

    Build configuration options may be used to statically override options in the
    "init" structure and will take precedence over initialization data passed
    using this function.
*/

SYS_MODULE_OBJ DRV_CTMU_Initialize ( const SYS_MODULE_INDEX        index,
                                     const SYS_MODULE_INIT * const init );


// *****************************************************************************
/* Function:
    void DRV_CTMU_Reinitialize( SYS_MODULE_OBJ                object,
                                  const SYS_MODULE_INIT * const init )

  Summary:
    Reinitializes the driver and refreshes any associated hardware settings.

  Description:
    This function reinitializes the driver and refreshes any associated hardware
    settings using the initialization data given, but it will not interrupt any
    ongoing operations.

  Precondition:
    Function DRV_CTMU_Initialize must have been called before calling this
    function and a valid SYS_MODULE_OBJ must have been returned.

  Parameters:
    object          - Driver object handle, returned from the DRV_CTMU_Initialize
                      function
    init            - Pointer to the initialization data structure

  Returns:
    None.

  Example:
    <code>
    DRV_CTMU_INIT init;
    SYS_MODULE_OBJ  objectHandle;   // Returned from DRV_CTMU_Initialize
    SYS_STATUS      CTMUStatus;

    // Populate the CTMU initialization structure
    init.moduleInit.value   = SYS_MODULE_POWER_RUN_FULL;
    init.CTMUId           = CTMU_ID_2;
    init.interruptSource    = INT_SOURCE_CTMU_3;

    DRV_CTMU_Reinitialize(objectHandle, (SYS_MODULE_INIT*)&init);

    CTMUStatus = DRV_CTMU_Status(objectHandle);
    if (SYS_STATUS_BUSY == CTMUStatus)
    {
        // Check again later to ensure the driver is ready
    }
    else if (SYS_STATUS_ERROR >= CTMUStatus)
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
    DRV_CTMU_Status operation. The system must use DRV_CTMU_Status to find out
    when the driver is in the ready state.

    Build configuration options may be used to statically override options in the
    "init" structure and will take precedence over initialization data passed
    using this function.
*/

void DRV_CTMU_Reinitialize ( SYS_MODULE_OBJ                object,
                               const SYS_MODULE_INIT * const init );


// *****************************************************************************
/* Function:
    void DRV_CTMU_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the CTMU Driver module.

  Description:
    This function deinitializes the specified instance of the CTMU Driver module, 
	disabling its operation (and any hardware), and invalidates all of the internal 
	data.

  Precondition:
    Function DRV_CTMU_Initialize must have been called before calling this
    function and a valid SYS_MODULE_OBJ must have been returned.

  Parameters:
    object          - Driver object handle, returned from the DRV_CTMU_Initialize
                      function

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     //  Returned from DRV_CTMU_Initialize
    SYS_STATUS          status;

    DRV_CTMU_Deinitialize(object);

    status = DRV_CTMU_Status(object);
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
    the DRV_CTMU_Status operation.  The system has to use DRV_CTMU_Status to find
    out when the module is in the ready state.
*/

void DRV_CTMU_Deinitialize ( SYS_MODULE_OBJ object );
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client Level
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_CTMU_Open( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified CTMU Driver instance and returns a handle to it.

  Description:
    This function opens the specified CTMU Driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver.

  Precondition:
    DRV_CTMU_Initialize must have been called before calling this function.

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

    handle = DRV_CTMU_Open(DRV_CTMU_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
    }
    </code>

  Remarks:
    The handle returned is valid until the DRV_CTMU_Close function is called.

    This function will NEVER block waiting for hardware.

    If the DRV_IO_INTENT_BLOCKING is requested and the driver was built
    appropriately to support blocking behavior, then other client-level
    operations may block waiting on hardware until they are complete.

    If DRV_IO_INTENT_NON_BLOCKING is requested the driver client can call the
    DRV_CTMU_ClientStatus operation to find out when the module is in the ready
    state.

    If the requested intent flags are not supported, the function will return
    DRV_HANDLE_INVALID.
*/

DRV_HANDLE DRV_CTMU_Open( const SYS_MODULE_INDEX drvIndex,
                            const DRV_IO_INTENT    intent );


// *****************************************************************************
/* Function:
    void DRV_CTMU_Close( DRV_HANDLE handle )

  Summary:
    Closes an opened-instance of the CTMU Driver.

  Description:
    This function closes an opened-instance of the CTMU Driver, invalidating the
    handle.

  Precondition:
    The DRV_CTMU_Initialize function must have been called for the specified
    CTMU Driver instance.

    DRV_CTMU_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open function

  Returns:
    None.

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_CTMU_Open

    DRV_CTMU_Close(handle);
    </code>

  Remarks:
    After calling this function, the handle passed in "handle" must not be used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling DRV_CTMU_Open before the caller may use the driver again.

    If DRV_IO_INTENT_BLOCKING was requested and the driver was built
    appropriately to support blocking behavior call may block until the
    operation is complete.

    If DRV_IO_INTENT_NON_BLOCKING request the driver client can call the
    DRV_CTMU_Status operation to find out when the module is in
    the ready state (the handle is no longer valid).

    Usually there is no need for the driver client to verify that the Close
    operation has completed.
*/
void DRV_CTMU_Close( DRV_HANDLE handle );

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client & Module Level
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Function:
    void DRV_CTMU_CurrentRangeSet(DRV_HANDLE handle, CTMU_CURRENT_RANGE range);

  Summary:
    This function selects the current source range.

  Description:
    At the heart of the CTMU is a precision current source, designed to provide a 
    constant reference for measurements. The level of current is user-selectable a
    cross four ranges, or a total of two orders of magnitude, with the ability to 
    trim the output in ±2% increments (nominal). 

  Precondition:
    The DRV_CTMU_Initialize function must have been called for the specified
    CTMU Driver instance.

    DRV_CTMU_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                    open function
    range       Charge pump current range selected, one of the possible 
                    enumeration values from CTMU_CURRENT_RANGE enum.  

  Returns:
    None.

  Remarks:
    None.
*/
void DRV_CTMU_CurrentRangeSet(DRV_HANDLE handle, CTMU_CURRENT_RANGE range);

// *****************************************************************************
/* Function:
    void DRV_CTMU_CurrentTrimSet(DRV_HANDLE handle, uint16_t trim);

  Summary:
    This function trims current source off of the nominal value.

  Description:
    At the heart of the CTMU is a precision current source, designed to provide a 
    constant reference for measurements. The level of current is user-selectable a
    cross four ranges, or a total of two orders of magnitude, with the ability to 
    trim the output in ±2% increments (nominal). 

  Precondition:
    The DRV_CTMU_Initialize function must have been called for the specified
    CTMU Driver instance.

    DRV_CTMU_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                    open function
    trim        Current trim index, from -31 to 31 
  Returns:
    None.
        
  Remarks:
    None.
*/
void DRV_CTMU_CurrentTrimSet(DRV_HANDLE handle, uint16_t trim);

// *****************************************************************************
/* Function:
    void DRV_CTMU_CurrentSourceGround(DRV_HANDLE handle, bool onOff);

  Summary:
    This function enables or disables the Current Discharge by connecting or not 
    connecting the current source output to ground.

  Description:

  Precondition:
    The DRV_CTMU_Initialize function must have been called for the specified
    CTMU Driver instance.

    DRV_CTMU_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle      - A valid open-instance handle, returned from the driver's
                    open function
    onOff       If true, the Current Discharge is disabled and the source output 
                    is NOT connected to ground.
                if false, the Current Discharge is enabled and the source output 
                    is connected to ground.

  Returns:
    None.
    
  Remarks:
    None.
*/
void DRV_CTMU_CurrentSourceGround(DRV_HANDLE handle, bool onOff);

// *****************************************************************************
// *****************************************************************************
// Section: Included Files (continued)
// *****************************************************************************
// *****************************************************************************

#ifdef DRV_CTMU_DRIVER_MODE_STATIC
#include "framework/driver/ctmu/drv_ctmu_static.h"
#endif

#endif // #ifndef _DRV_CTMU_H

/*******************************************************************************
 End of File
*/