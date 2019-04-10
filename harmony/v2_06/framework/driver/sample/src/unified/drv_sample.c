/*******************************************************************************
  SAMPLE Device Driver Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sample.c

  Summary:
    SAMPLE Device Driver Implementation

  Description:
    The SAMPLE device driver provides a simple interface to manage the SAMPLE
    modules on Microchip microcontrollers.  This file Implements the core
    interface routines for the SAMPLE driver.

    While building the driver from source, ALWAYS use this file in the build.
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

#include "sample/src/drv_sample_local.h"


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    static void _DRV_SAMPLE_SetupHardware ( const SAMPLE_MODULE_ID   plibId,
                                            DRV_SAMPLE_OBJ_HANDLE    hObj,
                                            DRV_SAMPLE_INIT          *sampleInit )

  Summary:
    Sets up the hardware from the initialization structure

  Description:
    This routine sets up the hardware from the initialization structure.

  Remarks:
    Called
*/

static void _DRV_SAMPLE_SetupHardware
(
    /* Function Parameters: (Dynamic arguments are removed from static builds.) */
    _DRV_SAMPLE_IF_DYN_COMMA( const SAMPLE_MODULE_ID plibId )
    _DRV_SAMPLE_IF_DYN_COMMA( DRV_SAMPLE_OBJ_HANDLE hObj )
    DRV_SAMPLE_INIT * sampleInit
)
{
    /* Initialize the Interrupt Source */
    _DRV_SAMPLE_STATIC_INT_SRC( _DRV_SAMPLE_OBJ(hObj, interruptSource) = _DRV_SAMPLE_INT_SRC_GET(sampleInit->interruptSource) );

    /* Power state initialization */
    _DRV_SAMPLE_PowerState( _DRV_SAMPLE_PERIPHERAL_ID_GET(plibId) , sampleInit);

    /* TODO: Call to other Module specific Initialization APIs */

} /* _DRV_SAMPLE_SetupHardware */


// *****************************************************************************
// *****************************************************************************
// Section: Driver Interface Function Definitions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SAMPLE_Initialize( const SYS_MODULE_INDEX index,
                                          const SYS_MODULE_INIT * const init )

  Summary:
    Initializes hardware and data for the given instance of the SAMPLE module

  Description:
    This routine initializes hardware for the instance of the SAMPLE module,
    using the hardware initialization given data.  It also initializes all
    necessary internal data.

  Parameters:
    index           - Identifies the driver instance to be initialized

    init            - Pointer to the data structure containing all data
                      necessary to initialize the hardware. This pointer may
                      be null if no data is required and static initialization
                      values are to be used.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.
*/

_DRV_SAMPLE_IF_DYN_RETURN_TYPE(SYS_MODULE_OBJ) _DRV_SAMPLE_MAKE_NAME(Initialize)
(
    /* Function Parameters: (Dynamic arguments are removed from static builds.) */
    _DRV_SAMPLE_IF_DYN( const SYS_MODULE_INDEX        drvIndex )
                        const SYS_MODULE_INIT * const init
)
{
    _DRV_SAMPLE_IF_DYN( DRV_SAMPLE_OBJ_HANDLE hObj = (DRV_SAMPLE_OBJ_HANDLE) 0 );
    DRV_SAMPLE_INIT * sampleInit = NULL;

    /* Validate the driver index */
    if ( _DRV_SAMPLE_INDEX_GET(drvIndex) >= DRV_SAMPLE_INDEX_COUNT )
    {
        _DRV_SAMPLE_IF_DYN_RETURN( SYS_MODULE_OBJ_INVALID );
    }

    /* Assign to the local pointer the init data passed */
    sampleInit = ( DRV_SAMPLE_INIT * ) init;

    /* Allocate the driver object and set the operation flag to be in use */
    _DRV_SAMPLE_IF_DYN( hObj = _DRV_SAMPLE_OBJ_ALLOCATE(drvIndex) );

    /* TODO : Initialize any other SAMPLE specific members */

    /* Check that the object is valid */
    SYS_ASSERT( _DRV_SAMPLE_ObjectIsValid(hObj), "Hardware Object is invalid" );

    /* Object is valid, set it in use */
    _DRV_SAMPLE_OBJ( hObj, inUse ) = true;

    /* Update the SAMPLE Module Index */
    _DRV_SAMPLE_IF_DYN(_DRV_SAMPLE_OBJ( hObj , sampleId ) = _DRV_SAMPLE_PERIPHERAL_ID_GET( sampleInit->sampleId ));

    /* Setup the Hardware */
    _DRV_SAMPLE_SetupHardware( _DRV_SAMPLE_IF_DYN_COMMA( _DRV_SAMPLE_PERIPHERAL_ID_GET(sampleInit->sampleId) )
                               _DRV_SAMPLE_IF_DYN_COMMA( hObj )
                               sampleInit );

    /* Interrupt flag cleared on the safer side */
    _DRV_SAMPLE_InterruptSourceClear( _DRV_SAMPLE_INT_SRC_GET( _DRV_SAMPLE_OBJ(hObj, interruptSource) ) );

    /* Enable the interrupt source in case of interrupt mode */
    _DRV_SAMPLE_InterruptSourceEnable( _DRV_SAMPLE_INT_SRC_GET( _DRV_SAMPLE_OBJ(hObj, interruptSource) ) );

    /* Set the current driver state */
    _DRV_SAMPLE_OBJ( hObj , status ) = SYS_STATUS_READY;

    /* Return the driver handle */
    _DRV_SAMPLE_IF_DYN_RETURN( (SYS_MODULE_OBJ)hObj );
} /* DRV_SAMPLE_Initialize */


//******************************************************************************
/* Function:
    void DRV_SAMPLE_Reinitialize( SYS_MODULE_OBJ object,
                                  const SYS_MODULE_INIT * const init )

  Summary:
    Reinitializes and refreshes the hardware for the instance of the SAMPLE
    module

  Description:
    This routine reinitializes and refreshes the hardware for the instance
    of the SAMPLE module using the hardware initialization given data.
    It does not clear or reinitialize internal data structures

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface
    init            - Pointer to the data structure containing any data
                      necessary to initialize the hardware.

  Returns:
    None
*/

void _DRV_SAMPLE_MAKE_NAME( Reinitialize )
(
    /* Function Parameters: (Dynamic arguments are removed from static builds.) */
    _DRV_SAMPLE_IF_DYN_COMMA( SYS_MODULE_OBJ object )
    const SYS_MODULE_INIT * const init
)
{
    _DRV_SAMPLE_IF_DYN( DRV_SAMPLE_OBJ_HANDLE hObj = (DRV_SAMPLE_OBJ_HANDLE) object );
    DRV_SAMPLE_INIT * sampleInit = NULL;

    /* Check for the valid driver object passed */
    SYS_ASSERT( _DRV_SAMPLE_ObjectIsValid(hObj), "Driver Object is invalid" );

    /* Valid init structure is present */
    sampleInit = ( DRV_SAMPLE_INIT * ) init;

    /* TODO: Stop/Disable the device if needed */

    /* Set the current driver state */
    _DRV_SAMPLE_OBJ( hObj , status ) = SYS_STATUS_UNINITIALIZED ;

    /* Setup the Hardware */
    _DRV_SAMPLE_SetupHardware( _DRV_SAMPLE_IF_DYN_COMMA( sampleInit->sampleId )
                               _DRV_SAMPLE_IF_DYN_COMMA( hObj )
                               sampleInit );

    /* TODO: Start/Enable the device if it was stop/disabled */

    /* Set the curent driver state */
    _DRV_SAMPLE_OBJ( hObj , status ) = SYS_STATUS_READY;

} /* DRV_SAMPLE_Reinitialize */


//******************************************************************************
/* Function:
    void DRV_SAMPLE_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specific module instance of the SAMPLE module

  Description:
    Deinitializes the specific module instancedisabling its operation (and any
    hardware for driver modules).  Resets all the internal data structures and
    fields for the specified instance to the default settings.

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface

  Returns:
    None
*/

void _DRV_SAMPLE_MAKE_NAME( Deinitialize )
(
    /* Function Parameter: (Dynamic arguments are removed from static builds.) */
    _DRV_SAMPLE_IF_DYN( SYS_MODULE_OBJ object )
)
{
    _DRV_SAMPLE_IF_DYN( DRV_SAMPLE_OBJ_HANDLE hObj = (DRV_SAMPLE_OBJ_HANDLE) object );

    /* Check for the valid driver object passed */
    SYS_ASSERT( _DRV_SAMPLE_ObjectIsValid(hObj), "Driver Object is invalid" );

    /* Interrupt De-Registration */
    _DRV_SAMPLE_InterruptSourceDisable( _DRV_SAMPLE_INT_SRC_GET( _DRV_SAMPLE_OBJ(hObj, interruptSource) ) );

    /* Set the Device Status */
    _DRV_SAMPLE_OBJ( hObj , status ) = SYS_MODULE_DEINITIALIZED;

    /* Remove the driver usage */
    _DRV_SAMPLE_OBJ( hObj , inUse ) = false;

} /* DRV_SAMPLE_Deinitialize */


//******************************************************************************
/* Function:
    SYS_STATUS DRV_SAMPLE_Status( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the hardware instance of the SAMPLE module

  Description:
    This routine Provides the current status of the hardware instance of the
    SAMPLE module.

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface

  Returns:
    SYS_STATUS_READY    Indicates that any previous module operation for the
                        specified module has completed

    SYS_STATUS_BUSY     Indicates that a previous module operation for the
                        specified module has not yet completed

    SYS_STATUS_ERROR    Indicates that the specified module is in an error state
*/

SYS_STATUS _DRV_SAMPLE_MAKE_NAME( Status )
(
    /* Function Parameter: (Dynamic arguments are removed from static builds.) */
    _DRV_SAMPLE_IF_DYN( SYS_MODULE_OBJ object )
)
{
    _DRV_SAMPLE_IF_DYN( DRV_SAMPLE_OBJ_HANDLE hObj = (DRV_SAMPLE_OBJ_HANDLE) object );

    /* Check for the valid driver object passed */
    SYS_ASSERT( _DRV_SAMPLE_ObjectIsValid(hObj), "Driver Object is invalid" );

    /* Return the status associated with the driver handle */
    return (_DRV_SAMPLE_OBJ( hObj , status ) );

} /* DRV_SAMPLE_Status */


//******************************************************************************
/* Function:
    void DRV_SAMPLE_Tasks( SYS_MODULE_OBJ object)

  Summary:
    Used to maintain the driver's state machine and implement its ISR

  Description:
    This routine is used to maintain the driver's internal state machine and
    implement its ISR for interrupt-driven implementations.

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface

  Returns:
    None
*/

void _DRV_SAMPLE_MAKE_NAME( Tasks )
(
    /* Function Parameter: (Dynamic arguments are removed from static builds.) */
    _DRV_SAMPLE_IF_DYN( SYS_MODULE_OBJ object )
)
{
    _DRV_SAMPLE_IF_DYN( DRV_SAMPLE_OBJ_HANDLE hObj = (DRV_SAMPLE_OBJ_HANDLE) object );

    /* Check for the valid driver object passed */
    SYS_ASSERT( _DRV_SAMPLE_ObjectIsValid(hObj), "Driver Object is invalid" );

    /* Check if the Interrupt/Status is set */
    if (  true == _DRV_SAMPLE_InterruptSourceStatusGet( _DRV_SAMPLE_INT_SRC_GET(_DRV_SAMPLE_OBJ(hObj, interruptSource)) )  )
    {

        // TODO : Perform driver specific tasks

        /* Clear Interrupt/Status Flag */
        _DRV_SAMPLE_InterruptSourceClear( _DRV_SAMPLE_INT_SRC_GET( _DRV_SAMPLE_OBJ(hObj, interruptSource) ) );
    }

} /* DRV_SAMPLE_Tasks */


//******************************************************************************
/* Function:
    DRV_HANDLE DRV_SAMPLE_Open( const SYS_MODULE_INDEX index,
                                const DRV_IO_INTENT intent )

  Summary:
    Opens the specific module instance and returns a handle

  Description:
    This routine opens a driver for use by any client module and provides a
    handle that must be provided to any of the other driver operations to
    identify the caller and the instance of the driver/hardware module.

  Parameters:
    index           - Identifier for the instance to be initialized
    ioIntent        - Possible values from the enumeration DRV_IO_INTENT

  Returns:
    If successful, the routine returns a valid open-instance handle (a number
    identifying both the caller and the module instance)
    If an error occurs, the return value is DRV_HANDLE_INVALID
*/

_DRV_SAMPLE_IF_MC_RETURN_TYPE( DRV_HANDLE ) _DRV_SAMPLE_MAKE_NAME( Open )
(
    /* Function Parameters: (Dynamic arguments are removed from static builds.) */
    _DRV_SAMPLE_IF_DYN_COMMA( const SYS_MODULE_INDEX drvIndex )
    const DRV_IO_INTENT ioIntent
)
{
    /* Multi client variables are removed from single client builds. */
    _DRV_SAMPLE_IF_MC( DRV_SAMPLE_CLIENT_OBJ_HANDLE hClientObj = (DRV_SAMPLE_CLIENT_OBJ_HANDLE) 0 );
    _DRV_SAMPLE_IF_MC( DRV_SAMPLE_OBJ_HANDLE hObj = 0 );

    /* Validate the driver index */
    /* If there is anything specific to the module & needs to be checked, should
       be handled in this section with an || condition.
       May be something like ioIntent test for Exclusive access */
    if ( _DRV_SAMPLE_INDEX_GET(drvIndex) >= DRV_SAMPLE_INDEX_COUNT )
    {
        _DRV_SAMPLE_IF_MC_RETURN( DRV_HANDLE_INVALID );
    }

    /* Setup client operations */

    /* To Do: OSAL - Lock Mutex */

    /* Allocate the client object and set the flag as in use */
    _DRV_SAMPLE_IF_MC(hClientObj = _DRV_SAMPLE_ClientObjectAllocate( _DRV_SAMPLE_INDEX_GET( drvIndex ) )) ;
    _DRV_SAMPLE_CLIENT_OBJ( hClientObj , inUse ) = true;
    _DRV_SAMPLE_CLIENT_OBJ( hClientObj , driverObject ) = _DRV_SAMPLE_INDEX_GET( drvIndex );

    /* Increment the client in case of Multi client support, otherwise remove
       the below statement */
    _DRV_SAMPLE_IF_MC(hObj = _DRV_SAMPLE_CLIENT_OBJ(hClientObj, driverObject));
    _DRV_SAMPLE_IF_MC( _DRV_SAMPLE_OBJ(hObj, numClients)++ ) ;

    /* Check for the client object */
    SYS_ASSERT( _DRV_SAMPLE_MC_Test( hClientObj < DRV_SAMPLE_CLIENTS_NUMBER) , "Invalid Client Object" );

    /* To Do: OSAL - Unlock Mutex */

    /* Update the Client Status */
    _DRV_SAMPLE_CLIENT_OBJ( hClientObj , status ) = DRV_SAMPLE_CLIENT_STATUS_READY;

    /* Return the client object */
    _DRV_SAMPLE_IF_MC_RETURN( ( DRV_HANDLE ) hClientObj );

} /* DRV_SAMPLE_Open */


//******************************************************************************
/* Function:
    void DRV_SAMPLE_Close( DRV_HANDLE handle )

  Summary:
    Closes an opened-instance of a driver

  Description:
    This routine closes an opened-instance of a driver, invalidating the given
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None
*/

void _DRV_SAMPLE_MAKE_NAME( Close )
(
    /* Function Parameters: (Multi client arguments are removed from single client builds.) */
    _DRV_SAMPLE_IF_MC( DRV_HANDLE handle )
)
{
    /* Multi client variables are removed from single client builds. */
    _DRV_SAMPLE_IF_MC( DRV_SAMPLE_CLIENT_OBJ_HANDLE hClientObj );

    /* Get the Client object from the handle passed */
    _DRV_SAMPLE_IF_MC( hClientObj = handle );

    /* Check for the Client validity */
    SYS_ASSERT( _DRV_SAMPLE_MC_Test( hClientObj < DRV_SAMPLE_CLIENTS_NUMBER), "Invalid Client Object" ) ;

    /* To Do: OSAL - lock Mutex */

    /* Free the Client Instance */
    _DRV_SAMPLE_CLIENT_OBJ( hClientObj , inUse ) = false ;

    /* To Do: OSAL - unlock Mutex */

    /* Update the Client Status */
    _DRV_SAMPLE_CLIENT_OBJ( hClientObj , status ) = DRV_SAMPLE_CLIENT_STATUS_INVALID;

} /* DRV_SAMPLE_Close */


//******************************************************************************
/* Function:
    DRV_SAMPLE_CLIENT_STATUS DRV_SAMPLE_ClientStatus(DRV_HANDLE handle)

  Summary:
    Gets the status of the module instance associated with the handle

  Description:
    This routine gets the status of the module instance associated with the
    handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    DRV_SAMPLE_CLIENT_STATUS value describing the current status of the driver
*/

DRV_SAMPLE_CLIENT_STATUS _DRV_SAMPLE_MAKE_NAME( ClientStatus )
(
    /* Function Parameters: (Multi client arguments are removed from single client builds.) */
    _DRV_SAMPLE_IF_MC( DRV_HANDLE handle )
)
{
    /* Multi client variables are removed from single client builds. */
    _DRV_SAMPLE_IF_MC( DRV_SAMPLE_CLIENT_OBJ_HANDLE hClientObj = (DRV_SAMPLE_CLIENT_OBJ_HANDLE) 0 );

    /* Get the Client object from the handle passed */
    _DRV_SAMPLE_IF_MC( hClientObj = handle );

    /* Check for the Client validity */
    SYS_ASSERT( _DRV_SAMPLE_MC_Test( hClientObj < DRV_SAMPLE_CLIENTS_NUMBER), "Invalid Client Object" ) ;

    /* Return the client status associated with the handle passed */
    return (_DRV_SAMPLE_CLIENT_OBJ( hClientObj , status ) );

} /* DRV_SAMPLE_ClientStatus */


/*******************************************************************************
End of File
*/



