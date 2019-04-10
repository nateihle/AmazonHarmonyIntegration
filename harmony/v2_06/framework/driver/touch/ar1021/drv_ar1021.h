/*******************************************************************************
 Touch controller AR1021 driver file

  File Name:
    drv_ar1021.c

  Summary:
    Touch controller AR1021 driver implementation.

  Description:
    This file consist of touch controller AR1021 driver interfaces. It
    implements the driver interfaces which read the touch input data from
    AR1021 through SPI bus.
 ******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_TOUCH_AR1021_H
#define _DRV_TOUCH_AR1021_H

#include "driver/touch/drv_touch.h"
#include "driver/spi/drv_spi.h"
#include "peripheral/spi/plib_spi.h"
#include "system/tmr/drv_tmr.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
// *****************************************************************************
/* AR1021 Driver Handle

  Summary:
    Touch screen controller AR1021 driver handle.

  Description:
    Touch controller AR1021 driver handle is a handle for the driver
    client object. Each driver with successful open call will return a new handle
    to the client object.

  Remarks:
    None.
*/

typedef uintptr_t DRV_TOUCH_AR1021_HANDLE;

// *****************************************************************************
/* AR1021 Driver Invalid Handle

  Summary:
    Definition of an invalid handle.

  Description:
    This is the definition of an invalid handle. An invalid handle is 
    is returned by DRV_TOUCH_AR1021_Open() and DRV_AR1021_Close()
    functions if the request was not successful.

  Remarks:
    None.
*/

#define DRV_TOUCH_AR1021_HANDLE_INVALID ((DRV_TOUCH_AR1021_HANDLE)(-1))

// *****************************************************************************
/* AR1021 Driver Module Index Numbers

  Summary:
    AR1021 driver index definitions.

  Description:
    These constants provide the AR1021 driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
    These values should be passed into the DRV_AR1021_Initialize and
    DRV_AR1021_Open functions to identify the driver instance in use.
*/

#define DRV_TOUCH_AR1021_INDEX_0         0

// *****************************************************************************
/* AR1021 Driver Module Index Count

  Summary:
    Number of valid AR1021 driver indices.

  Description:
    This constant identifies the number of valid AR1021 driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is derived from device-specific header files defined as part of 
    the peripheral libraries.
*/

#define DRV_TOUCH_AR1021_INDEX_COUNT     1

typedef enum {

     AR1021_ID_1 = 0,
     AR1021_NUMBER_OF_MODULES
} DRV_TOUCH_AR1021_MODULE_ID;

// *****************************************************************************
/* AR1021 Touch Controller Driver Task State

  Summary:
    Enumeration defining AR1021 touch controller driver task state.

  Description:
    This enumeration defines the AR1021 touch controller driver task state.
    The task state helps to synchronize the operations of initialization the
    the task, adding the read input task to the task queue once the touch
    controller notifies the available touch input and a decoding the touch input
    received.

  Remarks:
    None.
*/

typedef enum
{
    /* Task initialize state */
    DRV_TOUCH_AR1021_TASK_STATE_INIT = 0,
            
    /* Task complete state */
    DRV_TOUCH_AR1021_TASK_STATE_DONE,

} DRV_TOUCH_AR1021_TASK_STATE;


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - System Level
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* TOUCH Driver Calibration Initialization Data

  Summary:
    Defines the callback functions required to inform the user of touch and release
    targets.

  Description:
    This data type defines the callback function pointers required to inform
    of touch and release targets. The driver will invoke each callback in sequential
    order. The host code can display graphic and/or textual content to direct the
    user when a where on the LCD display to touch and release.

  Remarks:
    None.
*/
typedef struct
{
    /* first calibration target */
    void         (*firstPromptCallback) ( void );

    /* second calibration target */
    void         (*secondPromptCallback) ( void );

    /* third calibration target */
    void         (*thirdPromptCallback) ( void );

    /* fourth calibration target */
    void         (*fourthPromptCallback)( void );
    
    /* complete calibration */
    void         (*completeCallback)(void);

} DRV_TOUCH_AR1021_CALIBRATION_PROMPT_CALLBACK;

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_TOUCH_AR1021_Initialize( const SYS_MODULE_INDEX index,
                                                const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the  AR1021 instance for the specified driver index

  Description:
    This routine initializes the  AR1021 driver instance for the specified driver
    index, making it ready for clients to open and use it. The initialization
    data is specified by the 'init' parameter. The initialization may fail if the
    number of driver objects allocated are insufficient or if the specified
    driver instance is already initialized. The driver instance index is
    independent of the  AR1021 module ID. For example, driver instance 0 can be
    assigned to  AR10212.  If the driver is built statically, then some of the
    initialization parameters are overridden by configuration macros. Refer to
    the description of the DRV_TOUCH_AR1021_INIT data structure for more details on
    which members on this data structure are overridden.

  Precondition:
    None.

  Input:
    index  - Identifier for the instance to be initialized.  Please note this
             is not the  AR1021 id.  The hardware  AR1021 id is set in the initialization
             structure.  This is the index of the driver index to use.

    init   - Pointer to a data structure containing any data necessary to
             initialize the driver. If this pointer is NULL, the driver
             uses the static initialization override macros for each
             member of the initialization data structure.

  Return:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, returns SYS_MODULE_OBJ_INVALID.

  Example:
    <code>
    DRV_TOUCH_INIT      drvAr1021InitData;
    SYS_MODULE_OBJ      objectHandle;

    objectHandle = DRV_TOUCH_AR1021_Initialize(DRV_TOUCH_AR1021_INDEX_1, (SYS_MODULE_INIT*)drvAr1021InitData);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>
  Remarks:
    This routine must be called before any other AR1021 routine is called.

    This routine should only be called once during system initialization
    unless DRV_TOUCH_AR1021_Deinitialize is called to deinitialize the driver
    instance. This routine will NEVER block for hardware access.
*/

SYS_MODULE_OBJ DRV_TOUCH_AR1021_Initialize( const SYS_MODULE_INDEX index,
                                            const SYS_MODULE_INIT * const init );

/*************************************************************************
  Function:
    void DRV_TOUCH_AR1021_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    De-initializes the specified instance of the  AR1021 driver module.

  Description:
    De-initializes the specified instance of the  AR1021 driver module,
    disabling its operation (and any hardware) and invalidates all of the
    internal data.

  Conditions:
    Function DRV_TOUCH_AR1021_Initialize must have been called before calling this
    routine and a valid SYS_MODULE_OBJ must have been returned.

  Input:
    object -  Driver object handle, returned from DRV_TOUCH_AR1021_Initialize

  Return:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     //  Returned from DRV_TOUCH_AR1021_Initialize
    SYS_STATUS          status;

    DRV_TOUCH_AR1021_Deinitialize ( object );

    status = DRV_TOUCH_AR1021_Status( object );
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
    by the DRV_TOUCH_AR1021_Status operation. The system has to use DRV_TOUCH_AR1021_Status
    to find out when the module is in the ready state.
*/

void DRV_TOUCH_AR1021_Deinitialize ( SYS_MODULE_OBJ object );

/**************************************************************************
  Function:
    SYS_STATUS DRV_TOUCH_AR1021_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the  AR1021 driver module.

  Description:
    This function provides the current status of the  AR1021 driver module.

  Conditions:
    The DRV_TOUCH_AR1021_Initialize function must have been called before calling
    this function.

  Input:
    object -  Driver object handle, returned from DRV_TOUCH_AR1021_Initialize

  Return:
    SYS_STATUS_READY - Indicates that the driver is busy with a previous
    system level operation and cannot start another

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_TOUCH_AR1021_Initialize
    SYS_STATUS          status;

    status = DRV_TOUCH_AR1021_Status( object );
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

SYS_STATUS DRV_TOUCH_AR1021_Status ( SYS_MODULE_OBJ object );


// *****************************************************************************
/* Function:
    void DRV_TOUCH_AR1021_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's state machine and implements its task queue
    processing.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
	This routine is used to maintain the driver's internal state
	machine and implement its command queue processing. It is always called
    from SYS_Tasks() function.

  Precondition:
    The DRV_TOUCH_AR1021_Initialize routine must have been called for the specified
     AR1021 driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_TOUCH_AR1021_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_TOUCH_AR1021_Initialize

    while( true )
    {
        DRV_TOUCH_AR1021_Tasks ( object );

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks)
*/

void DRV_TOUCH_AR1021_Tasks ( SYS_MODULE_OBJ object );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client Level
// *****************************************************************************
// *****************************************************************************

/**************************************************************************
  Function:
    DRV_HANDLE DRV_TOUCH_AR1021_Open ( const SYS_MODULE_INDEX drvIndex,
                                       const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified AR1021 driver instance and returns a handle to it.
	<p><b>Implementation:</b> Dynamic</p>
 
  Description:
    This routine opens the specified AR1021 driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    The current version of driver does not support the DRV_IO_INTENT feature.
    The driver is by default non-blocking. The driver can perform both read
    and write to the AR1021 device. The driver supports single client only.
  
  Precondition:
    The DRV_TOUCH_AR1021_Initialize function must have been called before 
    calling this function.

  Parameters:
    drvIndex -  Index of the driver initialized with
                DRV_TOUCH_AR1021_Initialize().
                
    intent -    Zero or more of the values from the enumeration
                DRV_IO_INTENT ORed together to indicate the intended use of
                the driver. The current version of driver does not support
				the selective IO intent feature.
 
  Return:
    If successful, the routine returns a valid open-instance handle (a
    number identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID. An error
    can occur when the following is true:
      * if the number of client objects allocated via
        DRV_TOUCH_AR1021_CLIENTS_NUMBER is insufficient
      * if the client is trying to open the driver but driver has been
        opened exclusively by another client
      * if the driver hardware instance being opened is not initialized or
        is invalid

  Example:
    <code>
    DRV_HANDLE  handle;

    handle = DRV_TOUCH_AR1021_Open( DRV_TOUCH_AR1021_INDEX_0, DRV_IO_INTENT_EXCLUSIVE );

    if( DRV_HANDLE_INVALID == handle )
    {
        // Unable to open the driver
    }
    </code>

  Remarks:
    The handle returned is valid until the DRV_TOUCH_AR1021_Close routine is
    called. This routine will NEVER block waiting for hardware. If the
    requested intent flags are not supported, the routine will return
    DRV_HANDLE_INVALID. This function is thread safe in a RTOS application.
    It should not be called in an ISR.
*/

DRV_HANDLE DRV_TOUCH_AR1021_Open ( const SYS_MODULE_INDEX drvIndex,
                                   const DRV_IO_INTENT    intent );

// *****************************************************************************
/* Function:
    void DRV_TOUCH_AR1021_Close ( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the  AR1021 driver

  Description:
    This function closes an opened instance of the  AR1021 driver, invalidating the
    handle.

  Precondition:
    The DRV_TOUCH_AR1021_Initialize routine must have been called for the specified
     AR1021 driver instance.

    DRV_TOUCH_AR1021_Open must have been called to obtain a valid opened device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TOUCH_AR1021_Open

    DRV_TOUCH_AR1021_Close ( handle );
    </code>

  Remarks:
	After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling DRV_TOUCH_AR1021_Open before the caller may use the driver again.  This
    function is thread safe in a RTOS application.

    Note:
    Usually there is no need for the driver client to verify that the Close
    operation has completed.
*/

void DRV_TOUCH_AR1021_Close ( DRV_HANDLE handle );

/*********************************************************************
  Function:
    DRV_TOUCH_POSITION_SINGLE DRV_TOUCH_AR1021_TouchStatus( const SYS_MODULE_INDEX index )

  Summary:
    Returns the status of the current touch input.

  Description:
    It returns the status of the current touch input.

  Parameters
    None.

  Returns
    It returns the status of the current touch input.

*/
DRV_TOUCH_POSITION_STATUS DRV_TOUCH_AR1021_TouchStatus( const SYS_MODULE_INDEX index );

/*********************************************************************
  Function:
    void DRV_TOUCH_AR1021_TouchDataRead( const SYS_MODULE_INDEX index )

  Summary:
    Notifies the driver that the current touch data has been read

  Description:
    Notifies the driver that the current touch data has been read

  Parameters
    None.

  Returns
    None.

*/
void DRV_TOUCH_AR1021_TouchDataRead( const SYS_MODULE_INDEX index );

/*********************************************************************
  Function:
    short DRV_TOUCH_AR1021_TouchGetX( uint8 touchNumber )

  Summary:
    Returns the x coordinate of touch input.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    It returns the x coordinate in form of number of pixels for a touch input
  denoted by touchNumber.

  Parameters
   touchNumber - index to the touch input.

  Returns
    It returns the x coordinate of the touch input in terms of number of pixels.

*/
short DRV_TOUCH_AR1021_TouchGetX(uint8_t touchNumber);

/*********************************************************************
  Function:
    short DRV_TOUCH_AR1021_TouchGetY( uint8 touchNumber )

  Summary:
    Returns the y coordinate of touch input.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    It returns the y coordinate in form of number of pixes for a touch input
  denoted by touchNumber.

  Parameters
   touchNumber - index to the touch input.

  Returns
    It returns the y coordinate of the touch input in terms of number of pixels.

*/
short DRV_TOUCH_AR1021_TouchGetY(uint8_t touchNumber);

/*********************************************************************
  Function:
    DRV_TOUCH_PEN_STATE DRV_TOUCH_AR1021_TouchPenGet(uint8_t touchNumber)

  Summary:
    Returns the PEN state of the touch event.

  Description:
    It returns the PEN state of the last touch event corresponding to the x and y
    position.

  Parameters
   touchNumber - index to the touch input.

  Returns
    It returns DRV_TOUCH_PEN_STATE

*/
DRV_TOUCH_PEN_STATE DRV_TOUCH_AR1021_TouchPenGet(uint8_t touchNumber);

// *****************************************************************************
/* Function:
    void DRV_TOUCH_AR1021_Calibrate ( ( const DRV_TOUCH_AR1021_CALIBRATION_PROMPT_CALLBACK * prompt ) )

  Summary:
    Calibrate the touch screen

  Description:
    This function display calibration points on the display to enable calibration.

  Precondition:
    The DRV_TOUCH_AR1021_Initialize routine must have been called for the specified
    AR1021 driver instance.

  Parameters:
     none

  Returns:
    None

  Example:
    <code>
    DRV_TOUCH_AR1021_Calibrate ( handle );
    </code>

  Remarks:
    None
*/
void DRV_TOUCH_AR1021_Calibrate( const DRV_TOUCH_AR1021_CALIBRATION_PROMPT_CALLBACK * prompt );

// *****************************************************************************
/* Function:
    void DRV_TOUCH_AR1021_CalibrationSet(void)

  Summary:
    Set calibration with pre-defined points..

  Description:
    This function allows for the setting of pre-loaded calibration points.

  Precondition:
    The DRV_TOUCH_AR1021_Open routine must have been called for the specified
    AR1021 driver instance.

  Parameters:
     none

  Returns:
    None

  Example:
    <code>
    DRV_TOUCH_AR1021_CalibrationSet ( void );
    </code>

  Remarks:
    None
*/
void DRV_TOUCH_AR1021_CalibrationSet(DRV_TOUCH_SAMPLE_POINTS * samplePoints);


// *****************************************************************************
/* Function:
    void DRV_TOUCH_AR1021_FactoryDefaultSet(void)

  Summary:
    Set AR1021 controller to factory default configuration settings.

  Description:
    This function returns the AR1021 to operate on factory default configuration settings.

  Precondition:
    The DRV_TOUCH_AR1021_Open routine must have been called for the specified
    AR1021 driver instance.

  Parameters:
     none

  Returns:
    None

  Example:
    <code>
    DRV_TOUCH_AR1021_FactoryDefaultSet ( void );
    </code>

  Remarks:
    A power cycle is required to run on the default settings.
*/
void DRV_TOUCH_AR1021_FactoryDefaultSet(void);

// *****************************************************************************
/* Function:
    void DRV_TOUCH_AR1021_RegisterConfigWrite(uint16_t regOffset, uint8_t Value)

  Summary:
    Write a value to the given AR1021 configuration register.

  Description:
    This function set a value to the given AR1021 configuration register.

  Precondition:
    The DRV_TOUCH_AR1021_Open routine must have been called for the specified
    AR1021 driver instance.

  Parameters:
     none

  Returns:
    None

  Example:
    <code>
    DRV_TOUCH_AR1021_RegisterConfigWrite(uint16_t regOffset, uint8_t Value);
    </code>

  Remarks:
    none
*/
void DRV_TOUCH_AR1021_RegisterConfigWrite(uint16_t regOffset, uint8_t Value);

#ifdef __cplusplus
    }
#endif
    
#endif //_DRV_TOUCH_AR1021_H
