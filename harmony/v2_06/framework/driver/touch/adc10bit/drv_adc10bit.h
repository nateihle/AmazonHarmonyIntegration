/*******************************************************************************
  10-bit ADC Touch Driver Interface Definition

  Company:
    Microchip Technology Inc.

  File Name:
    drv_adc10bit.h

  Summary:
    10-bit ADC Touch Driver interface definitions

  Description:
    This is a resistive touch screen driver that is using the Microchip Graphics 
	Library. The calibration values are automatically checked (by reading a specific 
	memory location on the non-volatile memory) when initializing the module if the 
	function pointers to the read and write callback functions are initialized. If 
	the read value is invalid calibration will automatically be executed. Otherwise, 
	the calibration values will be loaded and used. The driver assumes that the 
	application side provides the read and write routines to a non-volatile memory. 
	If the callback functions are not initialized, the calibration routine will always 
	be called at start-up to initialize the global calibration values. 
	This driver assumes that the Graphics Library is initialized and will be using 
	the default font of the library.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014-2015 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_TOUCH_ADC10BIT_H
#define _DRV_TOUCH_ADC10BIT_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "driver/touch/drv_touch.h"

// DOM-IGNORE-BEGIN 
#ifdef __cplusplus
    extern "C" {
#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Macros
// *****************************************************************************
// *****************************************************************************

// Use this macro to debug the touch screen panel 
// this will enable the use of debugging functions in the C file.
// It assumes that the graphics portion is working.
// #define ENABLE_DEBUG_TOUCHSCREEN

// *****************************************************************************
/* Macro:
    ADC10BIT Driver Handle

  Summary:
    Driver handle.

  Description:
    Touch screen controller interfacing with the 10-bit Analog-to-Digital (ADC)
    converter device.

  Remarks:
    None
*/

typedef uintptr_t DRV_TOUCH_ADC10BIT_HANDLE;



// *****************************************************************************
/* Macro: 
    ADC10BIT Driver Invalid Handle

  Summary:
    Definition of an invalid handle.

  Description:
    This is the definition of an invalid handle. An invalid handle
    is returned by DRV_ADC10BIT_RawRead and DRV_ADC10BIT_RawRead
    functions if the request was not successful.

  Remarks:
    None.
*/

#define DRV_TOUCH_ADC10BIT_HANDLE_INVALID ((DRV_TOUCH_ADC10BIT_HANDLE)(-1))


// *****************************************************************************
/* Macro:
    ADC10BIT Driver Module Index Numbers

  Summary:
    ADC10BIT driver index definitions.

  Description:
    These constants provide the 10-bit ADC Driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_ADC10BIT_Initialize and
    DRV_ADC10BIT_Open functions to identify the driver instance in use.
*/

#define DRV_TOUCH_ADC10BIT_INDEX_0         0
#define DRV_TOUCH_ADC10BIT_INDEX_1         1

// *****************************************************************************
/* Macro:
    ADC10BIT Driver Module Index Count

  Summary:
    Number of valid ADC10BIT driver indices.

  Description:
    This constant identifies the number of valid 10-bit ADC Driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is derived from device-specific header files defined as part of the
    peripheral libraries.
*/

#define DRV_TOUCH_ADC10BIT_INDEX_COUNT     2

typedef enum {

    ADC10BIT_ID_1 = 0,
    ADC10BIT_NUMBER_OF_MODULES
} DRV_ADC10BIT_MODULE_ID;

// *****************************************************************************
/* Macro:
    ADC10BIT Driver Initialization Data

  Summary:
    Defines the data required to initialize or reinitialize the 10-bit ADC Driver.

  Description:
    This data type defines the data required to initialize or reinitialize the
    10-bit ADC Driver. If the driver is built statically, the members of this data
    structure are statically over-ridden by static override definitions in the
    system_config.h file.

  Remarks:
    None.
*/

typedef struct _DRV_TOUCH_ADC10BIT_INIT
{
    /* System module initialization */
    SYS_MODULE_INIT                 		moduleInit;

    /* Identifies peripheral (PLIB-level) ID */
    DRV_ADC10BIT_MODULE_ID                   	adc10bitId;

} DRV_TOUCH_ADC10BIT_INIT;

// *****************************************************************************
/* Macro:
    ADC10BIT Driver Client Specific Configuration

  Summary:
    Defines the data that can be changed per client.

  Description:
    This data type defines the data can be configured per client.  This data can
    be per client, and overrides the configuration data contained inside of 
	DRV_TOUCH_ADC10BIT_INIT.
	
  Remarks:
    None.
*/

typedef struct _DRV_TOUCH_ADC10BIT_CLIENT_DATA
{

} DRV_TOUCH_ADC10BIT_CLIENT_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - System Level
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_TOUCH_ADC10BIT_Initialize( const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the 10-bit ADC Driver instance for the specified driver index

  Description:
    This function initializes the 10-bit ADC Driver instance for the specified driver
    index, making it ready for clients to open and use it. The initialization
    data is specified by the 'init' parameter. The initialization may fail if the
    number of driver objects allocated are insufficient or if the specified
    driver instance is already initialized. The driver instance index is
    independent of the 10-bit ADC Driver module ID. For example, driver instance 0 can be
    assigned to ADC10BIT2.  If the driver is built statically, then some of the
    initialization parameters are overridden by configuration macros. Refer to
    the description of the DRV_TOUCH_ADC10BIT_INIT data structure for more details on
    which members on this data structure are overridden.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the instance to be initialized.  Please note this
             is not the 10-bit ADC Driver ID.  The hardware 10-bit ADC Driver ID 
			 is set in the initialization structure.  This is the index of the 
			 driver index to use.
    init   - Pointer to a data structure containing any data necessary to
             initialize the driver. If this pointer is NULL, the driver
             uses the static initialization override macros for each
             member of the initialization data structure.

  Returns:
    If successful, returns a valid handle to a driver instance object.
    Otherwise, returns SYS_MODULE_OBJ_INVALID.

  Example:
    <code>
    DRV_TOUCH_ADC10BIT_INIT        init;
    SYS_MODULE_OBJ      objectHandle;

    // Populate the ADC10BIT initialization structure
    init.spiId              = ADC10BIT_ID_1;

    objectHandle = DRV_TOUCH_ADC10BIT_Initialize(DRV_TOUCH_ADC10BIT_INDEX_1, (SYS_MODULE_INIT*)usartInitData);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>
  Remarks:
    This routine must be called before any other ADC10BIT routine is called.

    This routine should only be called once during system initialization
    unless DRV_TOUCH_ADC10BIT_Deinitialize is called to deinitialize the driver
    instance. This routine will NEVER block for hardware access.
*/

SYS_MODULE_OBJ DRV_TOUCH_ADC10BIT_Initialize( const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init );

//*************************************************************************
/*
  Function:
    void DRV_TOUCH_ADC10BIT_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the ADC10BIT driver module.

  Description:
    This function deinitializes the specified instance of the 10-bit ADC Driver 
	module, disabling its operation (and any hardware) and invalidates all of the
    internal data.

  Preconditions:
    DRV_TOUCH_ADC10BIT_Initialize must have been called before calling this
    routine and a valid SYS_MODULE_OBJ must have been returned.

  Parameters:
    object -  Driver object handle, returned from DRV_TOUCH_ADC10BIT_Initialize

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     //  Returned from DRV_TOUCH_ADC10BIT_Initialize
    SYS_STATUS          status;

    DRV_TOUCH_ADC10BIT_Deinitialize ( object );

    status = DRV_TOUCH_ADC10BIT_Status( object );
    if( SYS_MODULE_UNINITIALIZED == status )
    {
        // Check again later if you need to know
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize
    operation must be called before the Initialize operation can be called
    again.

    This function will NEVER block waiting for hardware. If the operation
    requires time to allow the hardware to complete, this will be reported
    by the DRV_TOUCH_ADC10BIT_Status operation. The system has to use 
	DRV_TOUCH_ADC10BIT_Status to determine when the module is in the ready state.
*/

void DRV_TOUCH_ADC10BIT_Deinitialize ( SYS_MODULE_OBJ object );


//**************************************************************************
/*
  Function:
    SYS_STATUS DRV_TOUCH_ADC10BIT_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the ADC10BIT driver module.

  Description:
    This function provides the current status of the ADC10BIT driver module.

  Preconditions:
    DRV_TOUCH_ADC10BIT_Initialize must have been called before calling
    this function.

  Parameters:
    object -  Driver object handle, returned from DRV_TOUCH_ADC10BIT_Initialize

  Returns:
    SYS_STATUS_READY - Indicates that the driver is busy with a previous
    system level operation and cannot start another

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_TOUCH_ADC10BIT_Initialize
    SYS_STATUS          status;

    status = DRV_TOUCH_ADC10BIT_Status( object );
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

SYS_STATUS DRV_TOUCH_ADC10BIT_Status ( SYS_MODULE_OBJ object );


// *****************************************************************************
/* Function:
    void DRV_TOUCH_ADC10BIT_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's state machine and implements its ISR.

  Description:
	This routine is used to maintain the driver's internal state
	machine and implement its transmit ISR for interrupt-driven implementations.
	In polling mode, this function should be called from the SYS_Tasks
	function. In Interrupt mode, this function should be called in the transmit
	interrupt service routine of the USART that is associated with this USART
	driver hardware instance.

  Precondition:
    DRV_TOUCH_ADC10BIT_Initialize must have been called for the specified
    10-bit ADC Driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_TOUCH_ADC10BIT_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;     // Returned from DRV_TOUCH_ADC10BIT_Initialize

    while( true )
    {
        DRV_TOUCH_ADC10BIT_Tasks ( object );

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks) or by the appropriate raw
    ISR.

    This function may execute in an ISR context and will never block or access any
    resources that may cause it to block.
*/

void DRV_TOUCH_ADC10BIT_Tasks ( SYS_MODULE_OBJ object );


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client Level
// *****************************************************************************
// *****************************************************************************

//**************************************************************************
/*
  Function:
    DRV_HANDLE DRV_TOUCH_ADC10BIT_Open ( const SYS_MODULE_INDEX drvIndex,
                                         const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified ADC10BIT driver instance and returns a handle to it.
	
  Description:
    This function opens the specified USART driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    The DRV_IO_INTENT_BLOCKING and DRV_IO_INTENT_NONBLOCKING ioIntent
    options additionally affect the behavior of the DRV_USART_Read() and
    DRV_USART_Write() functions. If the ioIntent is
    DRV_IO_INTENT_NONBLOCKING, then these function will not block even if
    the required amount of data could not be processed. If the ioIntent is
    DRV_IO_INTENT_BLOCKING, these functions will block until the required
    amount of data is processed.

    If ioIntent is DRV_IO_INTENT_READ, the client will only be read from
    the driver. If ioIntent is DRV_IO_INTENT_WRITE, the client will only be
    able to write to the driver. If the ioIntent in
    DRV_IO_INTENT_READWRITE, the client will be able to do both, read and
    write.

    Specifying a DRV_IO_INTENT_EXCLUSIVE will cause the driver to provide
    exclusive access to this client. The driver cannot be opened by any
    other client.
	
  Preconditions:
    DRV_TOUCH_ADC10BIT_Initialize must have been called before calling
    this function.
	
  Parameters:
    drvIndex -  Index of the driver initialized with DRV_TOUCH_ADC10BIT_Initialize.
                Please note this is not the SPI id.
    intent -    Zero or more of the values from the enumeration
                DRV_IO_INTENT ORed together to indicate the intended use of
                the driver
  Returns:
    If successful, the routine returns a valid open-instance handle (a
    number identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID. An error
    can occur when the following is true:
      * if the number of client objects allocated via
        DRV_TOUCH_ADC10BIT_CLIENTS_NUMBER is insufficient
      * if the client is trying to open the driver but driver has been
        opened exclusively by another client
      * if the driver hardware instance being opened is not initialized or
        is invalid
		
  Example:
    <code>
    DRV_HANDLE  handle;

    handle = DRV_TOUCH_ADC10BIT_Open( DRV_TOUCH_ADC10BIT_INDEX_0, DRV_IO_INTENT_EXCLUSIVE );

    if( DRV_HANDLE_INVALID == handle )
    {
        // Unable to open the driver
    }
    </code>
	
  Remarks:
    The handle returned is valid until the DRV_TOUCH_ADC10BIT_Close routine is
    called. This routine will NEVER block waiting for hardware. If the
    requested intent flags are not supported, the routine will return
    DRV_HANDLE_INVALID. This function is thread safe in a RTOS application.
    It should not be called in an ISR.
*/

DRV_HANDLE DRV_TOUCH_ADC10BIT_Open ( const SYS_MODULE_INDEX drvIndex,
                         const DRV_IO_INTENT    intent );

// *****************************************************************************
/* Function:
    void DRV_TOUCH_ADC10BIT_Close ( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the 10-bit ADC Driver.

  Description:
    This function closes an opened instance of the 10-bit ADC Driver, invalidating 
	the handle.

  Precondition:
    DRV_TOUCH_ADC10BIT_Initialize must have been called for the specified
    ADC10BIT driver instance.

    DRV_TOUCH_ADC10BIT_Open must have been called to obtain a valid opened device 
	handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TOUCH_ADC10BIT_Open

    DRV_TOUCH_ADC10BIT_Close ( handle );
    </code>

  Remarks:
	After calling this routine, the handle passed in "handle" must not be used
    with any of the remaining driver routines.  A new handle must be obtained by
    calling DRV_TOUCH_ADC10BIT_Open before the caller may use the driver again.  This
    function is thread safe in a RTOS application.

    Usually there is no need for the driver client to verify that the Close
    operation has completed.
*/
void DRV_TOUCH_ADC10BIT_Close ( DRV_HANDLE handle );


/*********************************************************************
  Function:
    DRV_TOUCH_POSITION_SINGLE DRV_TOUCH_ADC10BIT_TouchStatus( const SYS_MODULE_INDEX index )

  Summary:
    Returns the status of the current touch input.

  Description:
    It returns the status of the current touch input.

  Parameters
    None.

  Returns
    It returns the status of the current touch input.

*/
DRV_TOUCH_POSITION_STATUS DRV_TOUCH_ADC10BIT_TouchStatus( const SYS_MODULE_INDEX index );


/*********************************************************************
  Function:
    void DRV_TOUCH_ADC10BIT_TouchDataRead( const SYS_MODULE_INDEX index )

  Summary:
    Notifies the driver that the current touch data has been read

  Description:
    Notifies the driver that the current touch data has been read

  Parameters
    None.

  Returns
    None.

*/
void DRV_TOUCH_ADC10BIT_TouchDataRead( const SYS_MODULE_INDEX index );


//*********************************************************************
/* Function:
    short DRV_TOUCH_ADC10BIT_TouchGetX( uint8_t touchNumber )
	
   Summary:
     Returns x coordinate status when the touch screen is pressed.
   
   Description:
     This function returns the x coordinate status when the touch screen
	 is pressed.
   
   Preconditions:
     None.
   
   Parameters:
	 touchNumber - touch input index.
   
   Returns:
     * x coordinate - Indicates the touch screen was pressed
	 * -1           - Indicates the touch screen was not pressed
   
   Remarks:
     None.

*/
short DRV_TOUCH_ADC10BIT_TouchGetX( uint8_t touchNumber );


//*********************************************************************
/* Function:
    short DRV_TOUCH_ADC10BIT_TouchGetRawX()
	
   Summary:
     Returns raw x coordinate status when the touch screen is pressed.
   
   Description:
     This function returns the raw x coordinate status when the touch screen
	 is pressed.
   
   Preconditions:
     None.
   
   Parameters:
     None.
   
   Returns:
     * raw x coordinate - Indicates the touch screen was pressed
	 * -1               - Indicates the touch screen was not pressed
   
   Remarks:
     None.
 
*/

short DRV_TOUCH_ADC10BIT_TouchGetRawX(void);

//*********************************************************************
/* Function:
    short DRV_TOUCH_ADC10BIT_TouchGetY( DRV_HANDLE handle, uint8_t touchNumber )
	
   Summary:
     Returns y coordinate status when the touch screen is pressed.
   
   Description:
     This function returns the y coordinate status when the touch screen
	 is pressed.
   
   Preconditions:
     None.
   
   Parameters:
     handle - driver client handle.
	 touchNumber - touch input index.
   
   Returns:
     * y coordinate - Indicates the touch screen was pressed
	 * -1           - Indicates the touch screen was not pressed
   
   Remarks:
     None.

*/

short DRV_TOUCH_ADC10BIT_TouchGetY( uint8_t touchNumber );

//*********************************************************************
/* Function:
    short DRV_TOUCH_ADC10BIT_TouchGetRawY()
	
   Summary:
     Returns raw y coordinate status when the touch screen is pressed.
   
   Description:
     This function returns the raw y coordinate status when the touch screen
	 is pressed.
   
   Preconditions:
     None.
   
   Parameters:
     None.
   
   Returns:
     * raw y coordinate - Indicates the touch screen was pressed
	 * -1           - Indicates the touch screen was not pressed
   
   Remarks:
     None.

*/

short DRV_TOUCH_ADC10BIT_TouchGetRawY( void );

//*********************************************************************
/* Function:
    void DRV_TOUCH_ADC10BIT_TouchStoreCalibration(void)
	
   Summary:
     Stores calibration parameters into Non-volatile Memory.
   
   Description:
     This function stores calibration parameters into Non-volatile Memory.
   
   Preconditions:
     The NVM initialization function must be called before calling this function.
   
   Parameters:
     None.
   
   Returns:
     None.

   Remarks:
     This API is deprecated and its funcationality is handled via SYSTEM_INITIALIZATION
 
*/

void DRV_TOUCH_ADC10BIT_TouchStoreCalibration( void );

//*********************************************************************
/* Function:
     void DRV_TOUCH_ADC10BIT_TouchLoadCalibration(void)
  	
   Summary:
     Loads calibration parameters from Non-volatile Memory.
   
   Description:
     This function loads calibration parameters from Non-volatile Memory.
   
   Preconditions:
     The NVM initialization function must be called before calling this function.
   
   Parameters:
     None.
   
   Returns:
     None.

*/

void DRV_TOUCH_ADC10BIT_CalibrationSet(DRV_TOUCH_SAMPLE_POINTS * samplePoints);

//*********************************************************************
/* Function:
     void DRV_TOUCH_ADC10BIT_TouchLoadCalibration(void)
  	
   Summary:
     None.

   Description:
     None.

   Preconditions:
     None.
   
   Parameters:
     None.
   
   Returns:
     None.

*/

short DRV_TOUCH_ADC10BIT_PositionDetect(void);

#ifdef __cplusplus
    }
#endif
    
#endif //_DRV_TOUCH_ADC10BIT_H
