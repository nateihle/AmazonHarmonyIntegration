/*******************************************************************************
 Touch Controller MXT336T Driver Interface File

  File Name:
    drv_MXT336T.c

  Summary:
    Touch controller MXT336T Driver interface header file.

  Description:
    This header file describes the macros, data structure and prototypes of the 
    touch controller MXT336T driver interface.
 ******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_MXT336T_H
#define _DRV_MXT336T_H

#include "system_definitions.h"

#ifdef __cplusplus
    extern "C" {
#endif

// *****************************************************************************
/* MXT336T Driver Module Index Numbers

  Summary:
    MXT336T driver index definitions.

  Description:
    These constants provide the MXT336T driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
    These values should be passed into the DRV_MXT336T_Initialize and
    DRV_MXT336T_Open functions to identify the driver instance in use.
*/

#define DRV_MXT336T_INDEX_0         0
#define DRV_MXT336T_INDEX_1         1

// *****************************************************************************
/* MXT336T Driver Module Index Count

  Summary:
    Number of valid Touch controller MXT336T driver indices.

  Description:
    This constant identifies the number of valid Touch Controller MXT336T
    driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.
    This value is derived from device-specific header files defined as part of 
    the peripheral libraries.
*/

#define DRV_MXT336T_INDEX_COUNT     2

// *****************************************************************************
/*Structure
	DRV_MXT336T_INIT

  Summary:
    Defines the data required to initialize or reinitialize the MXT336T driver

  Description:
    This data type defines the data required to initialize or reinitialize the
    MXT336T driver. If the driver is built statically, the members of this data
    structure are statically over-ridden by static override definitions in the
    system_config.h file.

  Remarks:
    None.
*/
typedef struct
{
    /* System module initialization */
    SYS_MODULE_INIT         moduleInit;

    /* ID */
    int                     touchId;

    /* initialize function for module (normally called statically */
    SYS_MODULE_OBJ          (*drvInitialize) (const SYS_MODULE_INDEX index,
                                              const SYS_MODULE_INIT* const init);

    /* open function for I2C driver */
    DRV_HANDLE              (*drvOpen) ( const SYS_MODULE_INDEX index, const DRV_IO_INTENT intent );
    
    /* interrupt source for driver instance */
    INT_SOURCE              interruptSource;
    
    /* */
    uint16_t	            orientation;          // Orientation of the display (given in degrees of 0,90,180,270)

    /* */
    uint16_t                horizontalResolution; // Horizontal Resolution of the displayed orientation in Pixels

    /* */
    uint16_t                verticalResolution;
             
} DRV_MXT336T_INIT;

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - System Level
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
    init.resetChannel                = PORT_CHANNEL_A,
    init.resetPin                    = PORTS_BIT_POS_14,

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

SYS_MODULE_OBJ DRV_MXT336T_Initialize(const SYS_MODULE_INDEX index,
                                      const SYS_MODULE_INIT* const init);

/*************************************************************************
  Function:
       void DRV_MXT336T_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the MXT336T driver module.
	<p><b>Implementation:</b> Dynamic</p>

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

void DRV_MXT336T_Deinitialize(SYS_MODULE_OBJ object);


/**************************************************************************
  Function:
       SYS_STATUS DRV_MXT336T_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the MXT336T driver module.
	<p><b>Implementation:</b> Dynamic</p>

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

SYS_STATUS DRV_MXT336T_Status(SYS_MODULE_OBJ object);


// *****************************************************************************
/* Function:
    void DRV_MXT336T_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's state machine and implements its task queue
    processing.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
	This routine is used to maintain the driver's internal state
	machine and implement its command queue processing. It is always called
        from SYS_Tasks() function. This routine decodes the touch input data
        available in drvI2CReadFrameData.

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

void DRV_MXT336T_Tasks (SYS_MODULE_OBJ object);

// *****************************************************************************
/* Function:
    void DRV_MXT336T_ReadRequest( SYS_MODULE_OBJ object )

  Summary:
    Sends a read request to I2C bus driver and adds the read task to queue.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
	This routine is used to send a touch input read request to the I2C bus
        driver and adding the input read decode task to the queue. It is always
        called from MXT336T interrupt ISR routine.

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
void DRV_MXT336T_ReadRequest(SYS_MODULE_OBJ object);

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client Level
// *****************************************************************************
// *****************************************************************************

/**************************************************************************
  Function:
       DRV_HANDLE DRV_MXT336T_Open ( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified MXT336T driver instance and returns a handle to it.
	<p><b>Implementation:</b> Dynamic</p>
	
  Description:
    This routine opens the specified MXT336T driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    The current version of driver does not support the DRV_IO_INTENT feature.
    The driver is by default non-blocking. The driver can perform both read
    and write to the MXT336T device. The driver supports single client only.	
	
  Precondition:
    The DRV_MXT336T_Initialize function must have been called before 
    calling this function.
	
  Parameters:
    drvIndex -  Index of the driver initialized with
                DRV_MXT336T_Initialize().
                
    intent -    Zero or more of the values from the enumeration
                DRV_IO_INTENT ORed together to indicate the intended use of
                the driver. The current version of driver does not support
				the selective IO intent feature.
				
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

DRV_HANDLE DRV_MXT336T_Open(const SYS_MODULE_INDEX drvIndex,
                            const DRV_IO_INTENT    intent );

// *****************************************************************************
/* Function:
    void DRV_MXT336T_Close ( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the MXT336T driver.
	<p><b>Implementation:</b> Dynamic</p>

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

void DRV_MXT336T_Close ( DRV_HANDLE handle );



#ifdef __cplusplus
    }
#endif
    
#endif //_DRV_MXT336T_H