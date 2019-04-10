/*******************************************************************************
  Sample Device Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_sample.c

  Summary:
    The Sample device driver provides an example of an MPLAB Harmony driver.

  Description:
    The Sample device driver provides an example of an MPLAB Harmony driver.  It
    can be used as a starting point for developing a new driver.
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


// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "driver/sample/drv_sample.h"
#include "driver/sample/src/drv_sample_local.h"


// *****************************************************************************
// *****************************************************************************
// Section: Default Configuration Option Values
// *****************************************************************************
// *****************************************************************************

/* Default to polled mode. */
#ifndef DRV_SAMPLE_INTERRUPT_MODE
    #define DRV_SAMPLE_INTERRUPT_MODE    false
    #warning "Sample driver defaulting to polled mode."
#endif

/* Default number of sample driver instances. */
#ifndef DRV_SAMPLE_INSTANCES_NUMBER
    #define DRV_SAMPLE_INSTANCES_NUMBER  1
    #warning "Sample driver defaulting to 1 instance."
#endif

/* Default number of sample driver clients (shared by all instances). */
#ifndef DRV_SAMPLE_CLIENTS_NUMBER
    #define DRV_SAMPLE_CLIENTS_NUMBER  2
    #warning "Sample driver defaulting to 2 clients."
#endif


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Global Variables
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Driver Hardware instance objects.

  Summary:
    Defines the hardware instances objects that are available on the part

  Description:
    This data type defines the hardware instance objects that are available on
    the part, so as to capture the hardware state of the instance.

  Remarks:
    Not all modes are available on all micro-controllers.
*/

static DRV_SAMPLE_OBJ           gObjDrvSample[DRV_SAMPLE_INSTANCES_NUMBER];


// *****************************************************************************
/* Driver Client instance objects.

  Summary:
    Defines the Client instances objects that are available on the part

  Description:
    This data type defines the Client instance objects that are available on
    the part, so as to capture the Client state of the instance.

  Remarks:
    None
*/

static DRV_SAMPLE_CLIENT_OBJ    gObjDrvSampleClient[DRV_SAMPLE_CLIENTS_NUMBER];


// *****************************************************************************
// *****************************************************************************
// Section: File Scope Functions
// *****************************************************************************
// *****************************************************************************

static DRV_SAMPLE_CLIENT_OBJ_HANDLE ClientObjectAllocate ( void )
{
    int i;
    DRV_SAMPLE_CLIENT_OBJ_HANDLE handle = (DRV_SAMPLE_CLIENT_OBJ_HANDLE)DRV_HANDLE_INVALID;

    // TODO:  Guard with mutex

    for (i = 0; i < DRV_SAMPLE_CLIENTS_NUMBER ; i++)
    {
        // Return the matching index associated the hardware instance.
        if (gObjDrvSampleClient[i].inUse == false)
        {
            gObjDrvSampleClient[i].inUse = true;
            handle = &gObjDrvSampleClient[i];
            break;
        }
    }

    // End - guard with mutex

    return handle;
}


// *****************************************************************************
// *****************************************************************************
// Section: Client Interface Functions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    DRV_HANDLE DRV_SAMPLE_Open( const SYS_MODULE_INDEX  drvIndex,
                                const DRV_IO_INTENT     ioIntent )

  Summary:
    Opens the specific module instance and returns a handle

  Description:
    This routine opens a driver for use by any client module and provides a
    handle that must be provided to any of the other driver operations to
    identify the caller and the instance of the driver/hardware module.

  Parameters:
    drvIndex        - Identifier for the instance to be initialized
    ioIntent        - Possible values from the enumeration DRV_IO_INTENT

  Returns:
    If successful, the routine returns a valid open-instance handle (a number
    identifying both the caller and the module instance)
    If an error occurs, the return value is DRV_HANDLE_INVALID
*/

DRV_HANDLE  DRV_SAMPLE_Open ( const SYS_MODULE_INDEX    index,
                              const DRV_IO_INTENT       ioIntent )
{
    DRV_SAMPLE_OBJ         *pObj;
    DRV_SAMPLE_CLIENT_OBJ  *pClient;

    /* Validate the driver index */
    if ( index >= DRV_SAMPLE_INSTANCES_NUMBER || gObjDrvSample[index].inUse == false)
    {
        return DRV_HANDLE_INVALID;
    }
    pObj = &gObjDrvSample[index];

    /* Allocate and initialize the client object. */
    pClient = ClientObjectAllocate();
    if (pClient == (DRV_SAMPLE_CLIENT_OBJ_HANDLE)DRV_HANDLE_INVALID)
    {
        return DRV_HANDLE_INVALID;
    }

    /* Initialize the client object. */
    pClient->inUse  = true;
    pClient->driver = pObj;
    pClient->status = DRV_SAMPLE_CLIENT_STATUS_READY;

    /* Return the client object pointer as the driver handle. */
    return (DRV_HANDLE)pClient;

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

void DRV_SAMPLE_Close( DRV_HANDLE handle )
{
    DRV_SAMPLE_OBJ         *pObj;
    DRV_SAMPLE_CLIENT_OBJ  *pClient = (DRV_SAMPLE_CLIENT_OBJ *)handle;

    /* Validate the driver index */
    if ( pClient->inUse == false)
    {
        return;
    }
    pObj = pClient->driver;

    /* Free the Client Instance */
    pClient->inUse = false ;

} /* DRV_SAMPLE_Close */


// *****************************************************************************
// *****************************************************************************
// Section: System Interface Functions
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_SAMPLE_Initialize( const SYS_MODULE_INDEX    drvIndex,
                                          const SYS_MODULE_INIT    *const init )

  Summary:
    Initializes hardware and data for the given instance of the Sample module

  Description:
    This routine initializes hardware for the instance of the Sample module,
    using the hardware initialization given data.  It also initializes all
    necessary internal data.

  Parameters:
    drvIndex        - Identifies the driver instance to be initialized
    init            - Pointer to the data structure containing all data
                      necessary to initialize the hardware. This pointer may
                      be null if no data is required and static initialization
                      values are to be used.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.
*/

SYS_MODULE_OBJ DRV_SAMPLE_Initialize( const SYS_MODULE_INDEX        index,
                                      const SYS_MODULE_INIT * const init )
{
    DRV_SAMPLE_OBJ  *pObj   = NULL;
    DRV_SAMPLE_INIT *pInit  = NULL;

    /* Validate the driver index */
    if ( index >= DRV_SAMPLE_INSTANCES_NUMBER || gObjDrvSample[index].inUse)
    {
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Assign to the local pointer the init data passed in. */
    pInit = (DRV_SAMPLE_INIT *)init;

    /* Allocate the driver object. */
    pObj = &gObjDrvSample[index];
    pObj->inUse = true;

    /* TODO: Initialize any other Sample specific members */

    /* TODO: Check that the object is valid */

    /* TODO: Setup the Hardware */

    /* TODO: Set the current driver state */
    pObj->status = SYS_STATUS_READY;

    /* Return the driver handle */
    return (SYS_MODULE_OBJ)pObj;

} /* DRV_SAMPLE_Initialize */


//******************************************************************************
/* Function:
    void DRV_SAMPLE_Reinitialize( SYS_MODULE_OBJ        object,
                                  const SYS_MODULE_INIT * const init )

  Summary:
    Reinitializes and refreshes the hardware for the instance of the Sample
    module

  Description:
    This routine reinitializes and refreshes the hardware for the instance
    of the Sample module using the hardware initialization given data.
    It does not clear or reinitialize internal data structures

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface
    init            - Pointer to the data structure containing any data
                      necessary to initialize the hardware.

  Returns:
    None
*/

void DRV_SAMPLE_Reinitialize( SYS_MODULE_OBJ                object,
                              const SYS_MODULE_INIT * const init )
{
    //DRV_SAMPLE_OBJ  *pObj   = (DRV_SAMPLE_OBJ *)object;
    DRV_SAMPLE_INIT *pInit  = NULL;

    /* TODO: Check for the valid driver object passed */

    /* Valid init structure is present */
    pInit = (DRV_SAMPLE_INIT *)init;

    /* TODO: Stop/Disable the device if needed */

    /* TODO: Set the current driver state */

    /* TODO: Setup the Hardware */

    /* TODO: Start/Enable the device if it was stop/disabled */

    /* TODO: Set the current driver state */

} /* DRV_SAMPLE_Reinitialize */


//******************************************************************************
/* Function:
    void DRV_SAMPLE_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specific module instance of the Sample module

  Description:
    Deinitializes the specific module instance disabling its operation (and any
    hardware for driver modules).  Resets all the internal data structures and
    fields for the specified instance to the default settings.

  Parameters:
    object          - Identifies the Driver Object returned by the Initialize
                      interface

  Returns:
    None
*/

void DRV_SAMPLE_Deinitialize( SYS_MODULE_OBJ object )
{
    DRV_SAMPLE_OBJ  *pObj   = (DRV_SAMPLE_OBJ *)object;

    /* TODO: Check for the valid driver object passed */

    /* Set the Device Status */
    pObj->status    = SYS_STATUS_UNINITIALIZED;

    /* Remove the driver usage */
    pObj->inUse     = false;

} /* DRV_SAMPLE_Deinitialize */


//******************************************************************************
/* Function:
    SYS_STATUS DRV_SAMPLE_Status( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the hardware instance of the Sample module

  Description:
    This routine Provides the current status of the hardware instance of the
    Sample module.

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

SYS_STATUS DRV_SAMPLE_Status( SYS_MODULE_OBJ object )
{
    DRV_SAMPLE_OBJ  *pObj   = (DRV_SAMPLE_OBJ *)object;

    /* TODO: Check for the valid driver object passed */

    /* Return the status associated with the driver handle */
    return( pObj->status ) ;

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

void DRV_SAMPLE_Tasks( SYS_MODULE_OBJ object )
{
    //DRV_SAMPLE_OBJ  *pObj   = (DRV_SAMPLE_OBJ *)object;

    /* TODO: Check for the valid driver object passed */

    /* TODO: Check if the Interrupt/Status is set */
    if ( false )  
    {

        // TODO : Perform driver specific tasks

        /* TODO: Clear Interrupt/Status Flag */
    }

} /* DRV_SAMPLE_Tasks */


/*******************************************************************************
End of File
*/