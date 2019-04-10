/*******************************************************************************
 Touch Controller TOUCH_GENERIC Driver Interface File

  File Name:
    drv_touch_generic.c

  Summary:
    Touch controller TOUCH_GENERIC Driver interface header file.

  Description:
    This header file describes the macros, data structure and prototypes of the 
    touch controller TOUCH_GENERIC driver interface.
*******************************************************************************/

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

#ifndef _DRV_TOUCH_GENERIC_H
#define _DRV_TOUCH_GENERIC_H

#include "driver/touch/drv_touch.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
// *****************************************************************************
// *****************************************************************************
// Section: Data Types and Constants
// *****************************************************************************
// *****************************************************************************



// *****************************************************************************
/* Driver Handle

  Summary:
    Touch screen controller driver handle.

  Description:
    Touch controller driver handle is a handle for the driver
    client object. Each driver with succesful open call will return a new handle
    to the client object.

  Remarks:
    None.
*/

typedef uintptr_t DRV_TOUCH_GENERIC_HANDLE;



// *****************************************************************************
/* Driver Invalid Handle

  Summary:
    Definition of an invalid handle.

  Description:
    This is the definition of an invalid handle. An invalid handle is 
    is returned by DRV_TOUCH_GENERIC_Open() and DRV_TOUCH_GENERIC_Close()
    functions if the request was not successful.

  Remarks:
    None.
*/

#define DRV_TOUCH_GENERIC_HANDLE_INVALID ((DRV_TOUCH_GENERIC_HANDLE)(-1))


// *****************************************************************************
/* Driver Module Index Numbers

  Summary:
    driver index definitions.

  Description:
    These constants provide the driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.
    These values should be passed into the DRV_TOUCH_GENERIC_Initialize and
    DRV_TOUCH_GENERIC_Open functions to identify the driver instance in use.
*/

#define DRV_TOUCH_GENERIC_INDEX_0         0
#define DRV_TOUCH_GENERIC_INDEX_1         1

// *****************************************************************************
/* Driver Module Index Count

  Summary:
    Number of valid Touch controller driver indices.

  Description:
    This constant identifies the number of valid Touch Controller 
    driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.
    This value is derived from device-specific header files defined as part of 
    the peripheral libraries.
*/

#define DRV_TOUCH_GENERIC_INDEX_COUNT     2

//// *****************************************************************************
///* Driver Module I2C Frame Size
//
//  Summary:
//    I2C Frame size for reading touch input.
//
//  Description:
//    This constant identifies the size of I2C frame required to read from
//    touch controller. Driver notifies the availability of input data
//    through interrupt pin.
//
//  Remarks:
//    This constant should be used in place of hard-coded numeric literals.
//    This value is derived from device-specific data sheets.
//*/
//#define DRV_TOUCH_GENERIC_I2C_READ_FRAME_SIZE          7
//
//// *****************************************************************************
///* Driver Module Master Command Write I2C Address
//
//  Summary:
//    Touch command register write, I2C address where master sends the
//    commands.
//
//  Description:
//    This constant defines the command register I2C write address. This
//    address is used as I2C address to write commands into Touch
//    controller register.
//
//  Remarks:
//    This constant should be used in place of hard-coded numeric literals.
//    This value is derived from device-specific data sheets.
//*/
//#define DRV_TOUCH_GENERIC_I2C_MASTER_WRITE_ID                0x4A
//
//// *****************************************************************************
///* Driver Module Master Input Read I2C address
//
//  Summary:
//    input read, I2C address from where master reads touch input data.
//
//  Description:
//    This constant defines the touch input read I2C address. This
//    address is used as I2C address to read Touch input from Touch
//    controller.
//
//  Remarks:
//    This constant should be used in place of hard-coded numeric literals.
//
//    This value is derived from device-specific data sheets.
//*/
//#define DRV_TOUCH_GENERIC_I2C_MASTER_READ_ID                0x4B


// *****************************************************************************
/* Enumeration: 
	DRV_TOUCH_GENERIC_MODULE_ID

  Summary:
    Number of valid driver indices.

  Description:
    This constant identifies the number of valid driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is derived from device-specific header files defined as part of 
    the peripheral libraries.
*/

typedef enum {

    TOUCH_GENERIC_ID_1 = 0,
    TOUCH_GENERIC_NUMBER_OF_MODULES
            
} DRV_TOUCH_GENERIC_MODULE_ID;

// *****************************************************************************
/* Enumeration 
	DRV_TOUCH_GENERIC_TASK_STATE

  Summary:
    Enumeration defining touch controller driver task state.

  Description:
    This enumeration defines the touch controller driver task state.
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
    DRV_TOUCH_GENERIC_TASK_STATE_INIT = 0,

    /* Task read touch input request state */
    DRV_TOUCH_GENERIC_TASK_STATE_READ_INPUT,

    /* Task touch input decode state */
    DRV_TOUCH_GENERIC_TASK_STATE_DECODE_INPUT,
            
    /* Task complete state */
    DRV_TOUCH_GENERIC_TASK_STATE_DONE,

} DRV_TOUCH_GENERIC_TASK_STATE;

//// *****************************************************************************
///*Structure
//	DRV_TOUCH_GENERIC_TASK_QUEUE
//
//  Summary:
//    Defines the Touch Controller driver task data structure.
//
//  Description:
//    This data type defines the data structure maintaing task context in the task
//    queue. The inUse flag denotes the task context allocation for a task.
//    The enum variable taskState maintains the current task state. The I2C
//    buffer handle drvI2CReadBufferHandle maintains the I2C driver buffer handle
//    returned by the I2C driver read request. The byte array variable
//    drvI2CReadFrameData maintains the I2C frame data sent by TOUCH after a
//    successful read request.
//
//  Remarks:
//    None.
//*/
//typedef struct
//{
//    /* Flag denoting the allocation of task */
//    bool                            inUse;
//
//    /* Enum maintaining the task state */
//    DRV_TOUCH_GENERIC_TASK_STATE   taskState;
//
//    /* I2C Buffer handle */
//    DRV_I2C_BUFFER_HANDLE           drvI2CReadBufferHandle;
//
//
//    /* Response to Read Touch Input Command
//     * Response = { TOUCH_GENERIC Read Address,
//     *              Input Data Size,
//     *              Touch Id, Pen status,
//     *              Touch X coordinate (0 to 6),
//     *              Touch X coordinate (7 to 11),
//     *              Touch Y coordinate (0 to 6),
//     *              Touch Y coordinate (7 to 11) } */
//    uint8_t                         drvI2CReadFrameData
//                                    [DRV_TOUCH_GENERIC_I2C_READ_FRAME_SIZE];
//
//} DRV_TOUCH_GENERIC_TASK_QUEUE;

// *****************************************************************************
/*Structure
	DRV_TOUCH_GENERIC_OBJECT

  Summary:
    Defines the data structure maintaining driver instance object.

  Description:
    This data structure maintains the driver instance object. The
    object exists once per hardware instance.

  Remarks:
    None.
*/

typedef struct
{
    /* The status of the driver */
    SYS_STATUS                      status;

//    /* The peripheral Id associated with the object */
    int                             touchId;

    /* Save the index of the driver. Important to know this
    as we are using reference based accessing */
    SYS_MODULE_INDEX                drvIndex;

    /* Flag to indicate instance in use  */
    bool                            inUse;

    /* Flag to indicate module used in exclusive access mode */
    bool                            isExclusive;

    /* Number of clients possible with the hardware instance */
    uint8_t                         numClients;

    /* Orientation of the display (given in degrees of 0,90,180,270) */
    uint16_t                        orientation;

    /* Horizontal Resolution of the displayed orientation in Pixels */
    uint16_t                        horizontalResolution;

    /* Vertical Resolution of the displayed orientaion in Pixels */
    uint16_t                        verticalResolution;

    
    /* Touch status */
    DRV_TOUCH_POSITION_STATUS       touchStatus;
    

} DRV_TOUCH_GENERIC_OBJECT;

// *****************************************************************************
/*Structure 
	DRV_TOUCH_GENERIC_CLIENT_OBJECT

  Summary:
    Driver client object maintaining client data.

  Description:
    This defines the object required for the maintenance of the software
    clients instance. This object exists once per client instance.

  Remarks:
    None.
*/

typedef struct _DRV_TOUCH_GENERIC_CLIENT_OBJECT
{
    /* Driver Object associated with the client */
    DRV_TOUCH_GENERIC_OBJECT*                      driverObject;

    /* The intent with which the client was opened */
    DRV_IO_INTENT                        intent;
    

} DRV_TOUCH_GENERIC_CLIENT_OBJECT;

// *****************************************************************************
/*Structure 
	DRV_TOUCH_GENERIC_INIT

  Summary:
    Defines the data required to initialize or reinitialize the driver

  Description:
    This data type defines the data required to initialize or reinitialize the
    driver. If the driver is built statically, the members of this data
    structure are statically over-ridden by static override definitions in the
    system_config.h file.

  Remarks:
    None.
*/
typedef struct
{
    /* System module initialization */
    SYS_MODULE_INIT         moduleInit;

    /* initialize function for module (normally called statically */
    SYS_MODULE_OBJ          (*drvInitialize) (const SYS_MODULE_INDEX   index,
                                                const SYS_MODULE_INIT    * const init);

    /* index for the maxtouch driver instance used by this driver */
    int       touchID;

        /* */
    uint16_t	           orientation;          // Orientation of the display (given in degrees of 0,90,180,270)

    /* */
    uint16_t               horizontalResolution; // Horizontal Resolution of the displayed orientation in Pixels

    /* */
    uint16_t               verticalResolution;

} DRV_TOUCH_GENERIC_INIT;

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - System Level
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
      SYS_MODULE_OBJ DRV_TOUCH_GENERIC_Initialize(const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the TOUCH_GENERIC instance for the specified driver index.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This routine initializes the TOUCH_GENERIC driver instance for the specified 
    driver index, making it ready for clients to open and use it. The
    initialization data is specified by the 'init' parameter. The initialization
    may fail if the number of driver objects allocated are insufficient or if
    the specified driver instance is already initialized. The driver instance
    index is independent of the TOUCH_GENERIC module ID. For example, driver instance
    0 can be assigned to TOUCH_GENERIC.  If the driver is built statically, then
    some of the initialization parameters are overridden by configuration
    macros. Refer to the description of the DRV_TOUCH_GENERIC_INIT data
    structure for more details on which members on this data structure are
    overridden.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the instance to be initialized.  Please note this
             is not the TOUCH_GENERIC ID.  The hardware TOUCH_GENERIC ID is set in the 
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
    DRV_TOUCH_GENERIC_INIT        init;
    SYS_MODULE_OBJ      objectHandle;

    // Populate the TOUCH_GENERIC initialization structure
    // Touch Module Id
    init.touchId                      = DRV_TOUCH_GENERIC_INDEX_0;

    // I2C Bus driver open
    init.drvOpen                      = DRV_I2C_Open;

    // Interrupt Source for Touch
    init.interruptSource              = INT_SOURCE_EXTERNAL_1;

    // Interrupt Pin function mapping
    init.interruptPort.inputFunction  = INPUT_FUNC_INT1;

    // Pin to be mapped as interrupt pin
    init.interruptPort.inputPin       = INPUT_PIN_RPE8;

    // Analog pin number
    init.interruptPort.analogPin      = PORTS_ANALOG_PIN_25;

    // Pin Mode of analog pin
    init.interruptPort.pinMode        = PORTS_PIN_MODE_DIGITAL;

    // Interrupt pin port
    init.interruptPort.channel        = PORT_CHANNEL_E;

    // Interrupt pin port maskl
    init.interruptPort.dataMask       = 0x8;

    // Touch screen orientation
    init.orientation                  = DISP_ORIENTATION;

    // Touch screen horizontal resolution
    init.horizontalResolution         = DISP_HOR_RESOLUTION;

    // Touch screen vertical resolution
    init.verticalResolution           = DISP_VER_RESOLUTION;


    objectHandle = DRV_TOUCH_GENERIC_Initialize(DRV_TOUCH_GENERIC_INDEX_0,
                                              (SYS_MODULE_INIT*)init);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>
	
  Remarks:
    This routine must be called before any other TOUCH_GENERIC routine is called.

    This routine should only be called once during system initialization
    unless DRV_TOUCH_GENERIC_Deinitialize is called to deinitialize the driver
    instance. This routine will NEVER block for hardware access.
*/

SYS_MODULE_OBJ DRV_TOUCH_GENERIC_Initialize( const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init );

/*************************************************************************
  Function:
       void DRV_TOUCH_GENERIC_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the TOUCH_GENERIC driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    Deinitializes the specified instance of the TOUCH_GENERIC driver module,
    disabling its operation (and any hardware) and invalidates all of the
    internal data.

  Preconditions:
    Function DRV_TOUCH_GENERIC_Initialize must have been called before calling 
    this routine and a valid SYS_MODULE_OBJ must have been returned.

  Parameter:
    object -  Driver object handle, returned from DRV_TOUCH_GENERIC_Initialize

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;    //Returned from DRV_TOUCH_GENERIC_Initialize
    SYS_STATUS          status;

    DRV_TOUCH_GENERIC_Deinitialize ( object );

    status = DRV_TOUCH_GENERIC_Status( object );
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
    by the DRV_TOUCH_GENERIC_Status operation. The system has to use
    DRV_TOUCH_GENERIC_Status to determine when the module is in the ready state.
*/

void DRV_TOUCH_GENERIC_Deinitialize ( SYS_MODULE_OBJ object );


/**************************************************************************
  Function:
       SYS_STATUS DRV_TOUCH_GENERIC_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the TOUCH_GENERIC driver module.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function provides the current status of the TOUCH_GENERIC driver module.

  Precondition:
    The DRV_TOUCH_GENERIC_Initialize function must have been called before 
    calling this function.

  Parameters:
    object -  Driver object handle, returned from DRV_TOUCH_GENERIC_Initialize

  Returns:
    SYS_STATUS_READY - Indicates that the driver is busy with a previous
    system-level operation and cannot start another

  Example:
    <code>
    SYS_MODULE_OBJ      object;  // Returned from DRV_TOUCH_GENERIC_Initialize
    SYS_STATUS          status;

    status = DRV_TOUCH_GENERIC_Status( object );
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

SYS_STATUS DRV_TOUCH_GENERIC_Status ( SYS_MODULE_OBJ object );


// *****************************************************************************
/* Function:
    void DRV_TOUCH_GENERIC_Tasks ( SYS_MODULE_OBJ object );

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
    The DRV_TOUCH_GENERIC_Initialize routine must have been called for the 
    specified TOUCH_GENERIC driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_TOUCH_GENERIC_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;   // Returned from DRV_TOUCH_GENERIC_Initialize

    void SYS_Tasks( void )
    {
        DRV_TOUCH_GENERIC_Tasks ( object );

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks)

*/

void DRV_TOUCH_GENERIC_Tasks ( SYS_MODULE_OBJ object );

// *****************************************************************************
/* Function:
    void DRV_TOUCH_GENERIC_ReadRequest( SYS_MODULE_OBJ object )

  Summary:
    Sends a read request to I2C bus driver and adds the read task to queue.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
	This routine is used to send a touch input read request to the I2C bus
        driver and adding the input read decode task to the queue. It is always
        called from TOUCH_GENERIC interrupt ISR routine.

  Precondition:
    The DRV_TOUCH_GENERIC_Initialize routine must have been called for the 
    specified TOUCH_GENERIC driver instance. 

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_TOUCH_GENERIC_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;   // Returned from DRV_TOUCH_GENERIC_Initialize

    void __ISR(_EXTERNAL_INT_VECTOR, ipl5) _IntHandlerDrvTOUCH_GENERIC(void)
    {
        DRV_TOUCH_GENERIC_ReadRequest ( object );

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the TOUCH_GENERIC ISR routine.

*/
void DRV_TOUCH_GENERIC_ReadRequest( SYS_MODULE_OBJ object );

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines - Client Level
// *****************************************************************************
// *****************************************************************************

/**************************************************************************
  Function:
       DRV_HANDLE DRV_TOUCH_GENERIC_Open ( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified TOUCH_GENERIC driver instance and returns a handle to it.
	<p><b>Implementation:</b> Dynamic</p>
	
  Description:
    This routine opens the specified TOUCH_GENERIC driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    The current version of driver does not support the DRV_IO_INTENT feature.
    The driver is by default non-blocking. The driver can perform both read
    and write to the TOUCH_GENERIC device. The driver supports single client only.	
	
  Precondition:
    The DRV_TOUCH_GENERIC_Initialize function must have been called before 
    calling this function.
	
  Parameters:
    drvIndex -  Index of the driver initialized with
                DRV_TOUCH_GENERIC_Initialize().
                
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
        DRV_TOUCH_GENERIC_CLIENTS_NUMBER is insufficient
      * if the client is trying to open the driver but driver has been
        opened exclusively by another client
      * if the driver hardware instance being opened is not initialized or
        is invalid
		
  Example:
    <code>
    DRV_HANDLE  handle;

    handle = DRV_TOUCH_GENERIC_Open( DRV_TOUCH_GENERIC_INDEX_0,
                                      DRV_IO_INTENT_EXCLUSIVE );

    if( DRV_HANDLE_INVALID == handle )
    {
        // Unable to open the driver
    }
    </code>
	
  Remarks:
    The handle returned is valid until the DRV_TOUCH_GENERIC_Close routine is
    called. This routine will NEVER block waiting for hardware. If the
    requested intent flags are not supported, the routine will return
    DRV_HANDLE_INVALID. This function is thread safe in a RTOS application.
    It should not be called in an ISR.
*/

DRV_HANDLE DRV_TOUCH_GENERIC_Open ( const SYS_MODULE_INDEX drvIndex,
                         const DRV_IO_INTENT    intent );

// *****************************************************************************
/* Function:
    void DRV_TOUCH_GENERIC_Close ( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the TOUCH_GENERIC driver.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    This function closes an opened instance of the TOUCH_GENERIC driver, invalidating
    the handle.

  Precondition:
    The DRV_TOUCH_GENERIC_Initialize routine must have been called for the 
    specified TOUCH_GENERIC driver instance.

    DRV_TOUCH_GENERIC_Open must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TOUCH_GENERIC_Open

    DRV_TOUCH_GENERIC_Close ( handle );
    </code>

  Remarks:
	After calling this routine, the handle passed in "handle" must not be 
    used with any of the remaining driver routines.  A new handle must be
    obtained by calling DRV_TOUCH_GENERIC_Open before the caller may use the
    driver again. This function is thread safe in a RTOS application.

    Note: Usually, there is no need for the driver client to verify that the 
          Close operation has completed.
*/

void DRV_TOUCH_GENERIC_Close ( DRV_HANDLE handle );

//*********************************************************************
/*  Function:
    DRV_TOUCH_GENERIC_POSITION_SINGLE DRV_TOUCH_GENERIC_TouchStatus( const SYS_MODULE_INDEX index )

  Summary:
    Returns the status of the current touch input.

  Description:
    It returns the status of the current touch input.

  Parameters
    None.

  Returns
    It returns the status of the current touch input.

*/
DRV_TOUCH_POSITION_STATUS DRV_TOUCH_GENERIC_TouchStatus( const SYS_MODULE_INDEX index );


//*********************************************************************
/*  Function:
    void DRV_TOUCH_GENERIC_TouchDataRead( const SYS_MODULE_INDEX index )

  Summary:
    Notifies the driver that the current touch data has been read

  Description:
    Notifies the driver that the current touch data has been read

  Parameters
    None.

  Returns
    None.

*/
void DRV_TOUCH_GENERIC_TouchDataRead( const SYS_MODULE_INDEX index );


//*********************************************************************
/*  Function:
    short DRV_TOUCH_GENERIC_TouchGetX( uint8 touchNumber )

  Summary:
    Returns the x coordinate of touch input.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    It returns the x coordinate in form of number of pixes for a touch input
  denoted by touchNumber.

  Parameters
   touchNumber - index to the touch input.

  Returns
    It returns the x coordinate of the touch input in terms of number of pixels.

*/
short DRV_TOUCH_GENERIC_TouchGetX( uint8_t touchNumber );


//*********************************************************************
/*  Function:
    short DRV_TOUCH_GENERIC_TouchGetY( uint8 touchNumber )

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

short DRV_TOUCH_GENERIC_TouchGetY( uint8_t touchNumber );




#ifdef __cplusplus
    }
#endif
    
#endif //_DRV_TOUCH_GENERIC_H