/*******************************************************************************
 Touch controller MTCH6301 driver file

  File Name:
    drv_mtch6301.c

  Summary:
    Touch controller MTCH6301 driver interface file.

  Description:
    This file consist of touch controller MTCH6301 driver interfaces. It
    implements the driver interfaces which read the touch input data from
    MTCH6301 through I2C bus.
 ******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2016 released Microchip Technology Inc.  All rights reserved.

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

#include <sys/attribs.h>
#include <sys/kmem.h>
#include "system/int/sys_int.h"
#include "system/touch/sys_touch.h"
#include "system/ports/sys_ports.h"
#include "driver/i2c/drv_i2c.h"
#include "driver/touch/mtch6301/drv_mtch6301.h"

#define DRV_MTCH_BITMASK_PEN_STATE                   0x01
#define DRV_MTCH_TOUCH_ID_MASK              0x78
#define DRV_MTCH_XY_AXIS_HIGHER_BITS_MASK   0x1F
#define DRV_MTCH_XY_AXIS_LOWER_BITS_MASK    0x7F
#define DRV_MTCH_TOUCH_ID_BIT_3             0x40
#define SHIFT_BY_3                          0x03
#define SHIFT_BY_7                          0x07
#define SHIFT_BY_10                         0x0A


/* Touch input data */
static int16_t PCapX[5]= {-1,-1,-1,-1,-1};
static int16_t PCapY[5] = { -1, -1, -1, -1, -1 };

// Default Calibration Inset Value (percentage of vertical or horizontal resolution)
// Calibration Inset = ( CALIBRATIONINSET / 2 ) % , Range of 0?20% with 0.5% resolution
// Example with CALIBRATIONINSET == 20, the calibration points are measured
// 10% from the corners.
#ifndef CALIBRATIONINSET
    #define CALIBRATIONINSET   6       // range 0 <= CALIBRATIONINSET <= 40
#endif

#define CAL_X_INSET    (((480)*(CALIBRATIONINSET>>1))/100)
#define CAL_Y_INSET    (((272)*(CALIBRATIONINSET>>1))/100)

/* MTCH6301 Driver instance object */
static DRV_TOUCH_MTCH6301_OBJECT            
                  sMTCH6301DriverInstances[DRV_TOUCH_MTCH6301_INSTANCES_NUMBER];

/* MTCH6301 Driver client object */
static DRV_TOUCH_MTCH6301_CLIENT_OBJECT     
                  sMTCH6301ClientInstances[DRV_TOUCH_MTCH6301_CLIENTS_NUMBER];

/* MTCH6301 Driver task queue */
static DRV_TOUCH_MTCH6301_TASK_QUEUE
                  sMTCH6301Queue          [DRV_TOUCH_MTCH6301_NUM_QUEUE];

DRV_I2C_BUFFER_EVENT operationStatus;

// *****************************************************************************
// *****************************************************************************
// Section: Initialization
// *****************************************************************************
// *****************************************************************************




// *****************************************************************************
/* Function:
      SYS_MODULE_OBJ DRV_TOUCH_MTCH6301_Initialize(const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init )

  Summary:
    Initializes the MTCH6301 instance for the specified driver index

  Description:
    This routine initializes the MTCH6301 driver instance for the specified
    driver index, making it ready for clients to open and use it. The
    initialization data is specified by the 'init' parameter. The initialization
    may fail if the number of driver objects allocated are insufficient or if
    the specified driver instance is already initialized. The driver instance
    index is independent of the MTCH6301 module ID. For example, driver instance
    0 can be assigned to MTCH63012.  If the driver is built statically, then
    some of the initialization parameters are overridden by configuration
    macros. Refer to the description of the DRV_TOUCH_MTCH6301_INIT data
    structure for more details on which members on this data structure are
    overridden.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the instance to be initialized.  Please note this
             is not the MTCH6301 ID.  The hardware MTCH6301 ID is set in the
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
    DRV_TOUCH_MTCH6301_INIT        init;
    SYS_MODULE_OBJ      objectHandle;

    // Populate the MTCH6301 initialization structure
    // Touch Module Id
    init.touchId                      = DRV_TOUCH_INDEX_0;

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


    objectHandle = DRV_TOUCH_MTCH6301_Initialize(DRV_TOUCH_INDEX_0,
                                              (SYS_MODULE_INIT*)init);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This routine must be called before any other MTCH6301 routine is called.

    This routine should only be called once during system initialization
    unless DRV_TOUCH_MTCH6301_Deinitialize is called to deinitialize the driver
    instance. This routine will NEVER block for hardware access.
*/

SYS_MODULE_OBJ DRV_TOUCH_MTCH6301_Initialize( const SYS_MODULE_INDEX index,
                                           const SYS_MODULE_INIT * const init )
{
    const DRV_TOUCH_INIT *pInit = NULL;

    if ( index >= DRV_TOUCH_MTCH6301_INDEX_COUNT )
    {
        SYS_ASSERT(false, "MTCH6301 Driver: Attempting to initialize an instance number greater than the max");
        return SYS_MODULE_OBJ_INVALID;
    }

    DRV_TOUCH_MTCH6301_OBJECT * pDrvInstance =
                ( DRV_TOUCH_MTCH6301_OBJECT *)&sMTCH6301DriverInstances[index];

    if ( pDrvInstance->inUse == true )
    {
        SYS_ASSERT(false, "MTCH6301 Driver: Attempting to reinitialize a driver instance that is already in use");
        return SYS_MODULE_OBJ_INVALID;
    }

    pDrvInstance->inUse = true;

    pInit = (const DRV_TOUCH_INIT * const)init;

    /* */
    pDrvInstance->touchId               = pInit->touchId;
    pDrvInstance->drvOpen               = pInit->drvOpen;
    pDrvInstance->orientation           = pInit->orientation;
    pDrvInstance->verticalResolution    = pInit->verticalResolution;
    pDrvInstance->horizontalResolution  = pInit->horizontalResolution;
    pDrvInstance->numClients            = 0;
    pDrvInstance->readRequest           = 0;
    pDrvInstance->taskQueue             = (DRV_TOUCH_MTCH6301_TASK_QUEUE*)&sMTCH6301Queue;
    pDrvInstance->drvI2CHandle          = DRV_HANDLE_INVALID;
    pDrvInstance->touchStatus           = DRV_TOUCH_POSITION_NONE;

    /* */

    pDrvInstance->status = SYS_STATUS_READY;

    return (SYS_MODULE_OBJ)pDrvInstance;

}

/*************************************************************************
  Function:
       void DRV_TOUCH_MTCH6301_Deinitialize ( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the MTCH6301 driver module.

  Description:
    Deinitializes the specified instance of the MTCH6301 driver module,
    disabling its operation (and any hardware) and invalidates all of the
    internal data.

  Preconditions:
    Function DRV_TOUCH_MTCH6301_Initialize must have been called before calling
    this routine and a valid SYS_MODULE_OBJ must have been returned.

  Parameter:
    object -  Driver object handle, returned from DRV_TOUCH_MTCH6301_Initialize

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;    //Returned from DRV_TOUCH_MTCH6301_Initialize
    SYS_STATUS          status;

    DRV_TOUCH_MTCH6301_Deinitialize ( object );

    status = DRV_TOUCH_MTCH6301_Status( object );
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
    by the DRV_TOUCH_MTCH6301_Status operation. The system has to use
    DRV_TOUCH_MTCH6301_Status to determine when the module is in the ready state.
*/

void DRV_TOUCH_MTCH6301_Deinitialize ( SYS_MODULE_OBJ object )
{
    DRV_TOUCH_MTCH6301_OBJECT * pDrvInstance =
                                        (DRV_TOUCH_MTCH6301_OBJECT *)object;
    DRV_TOUCH_MTCH6301_CLIENT_OBJECT *pClient = &sMTCH6301ClientInstances[0];

    if( pDrvInstance == NULL )
    {
        SYS_ASSERT(false, "MTCH6301 Driver: Attempting to deinitialize a NULL object");
        return;
    }

    if ( pDrvInstance->inUse == false )
    {
        SYS_ASSERT(false, "MTCH6301 Driver: Attempting to deinitialize a driver instance that is not in use");
        return;
    }

    SYS_INT_SourceDisable(pDrvInstance->interruptSource);

    if( pClient->driverObject == (DRV_TOUCH_MTCH6301_OBJECT * )pDrvInstance)
    {
        pClient->driverObject = NULL;
    }

    pDrvInstance->touchId               = 0xFF;
    pDrvInstance->verticalResolution    = 0;
    pDrvInstance->horizontalResolution  = 0;
    pDrvInstance->inUse                 = false;
    pDrvInstance->status                = SYS_STATUS_UNINITIALIZED;
    pDrvInstance->readRequest           = 0;

    return;
}

/**************************************************************************
  Function:
       SYS_STATUS DRV_TOUCH_MTCH6301_Status ( SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the MTCH6301 driver module.

  Description:
    This function provides the current status of the MTCH6301 driver module.

  Precondition:
    The DRV_TOUCH_MTCH6301_Initialize function must have been called before
    calling this function.

  Parameters:
    object -  Driver object handle, returned from DRV_TOUCH_MTCH6301_Initialize

  Returns:
    SYS_STATUS_READY - Indicates that the driver is busy with a previous
    system-level operation and cannot start another

  Example:
    <code>
    SYS_MODULE_OBJ      object;  // Returned from DRV_TOUCH_MTCH6301_Initialize
    SYS_STATUS          status;

    status = DRV_TOUCH_MTCH6301_Status( object );
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

SYS_STATUS DRV_TOUCH_MTCH6301_Status ( SYS_MODULE_OBJ object )
{
    DRV_TOUCH_MTCH6301_OBJECT * pDrvInstance =
                                        ( DRV_TOUCH_MTCH6301_OBJECT *)object;

    if ( object == SYS_MODULE_OBJ_INVALID )
    {
        //SYS_ASSERT( " Handle is invalid " );
        return SYS_MODULE_OBJ_INVALID;
    }
    
    return pDrvInstance->status;
}

/**************************************************************************
  Function:
       DRV_HANDLE DRV_TOUCH_MTCH6301_Open ( const SYS_MODULE_INDEX drvIndex,
                                const DRV_IO_INTENT    intent )

  Summary:
    Opens the specified MTCH6301 driver instance and returns a handle to it.

  Description:
    This routine opens the specified MTCH6301 driver instance and provides a
    handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent
    parameter defines how the client interacts with this driver instance.

    Specifying a DRV_IO_INTENT_EXCLUSIVE will cause the driver to provide
    exclusive access to this client. The driver cannot be opened by any
    other client.

  Precondition:
    The DRV_TOUCH_MTCH6301_Initialize function must have been called before
    calling this function.

  Parameters:
    drvIndex -  Index of the driver initialized with
                DRV_TOUCH_MTCH6301_Initialize().

    intent -    Zero or more of the values from the enumeration
                DRV_IO_INTENT ORed together to indicate the intended use of
                the driver

  Returns:
    If successful, the routine returns a valid open-instance handle (a
    number identifying both the caller and the module instance).

    If an error occurs, the return value is DRV_HANDLE_INVALID. An error
    can occur when the following is true:
      * if the number of client objects allocated via
        DRV_TOUCH_MTCH6301_CLIENTS_NUMBER is insufficient
      * if the client is trying to open the driver but driver has been
        opened exclusively by another client
      * if the driver hardware instance being opened is not initialized or
        is invalid

  Example:
    <code>
    DRV_HANDLE  handle;

    handle = DRV_TOUCH_MTCH6301_Open( DRV_TOUCH_MTCH6301_INDEX_0,
                                      DRV_IO_INTENT_EXCLUSIVE );

    if( DRV_HANDLE_INVALID == handle )
    {
        // Unable to open the driver
    }
    </code>

  Remarks:
    The handle returned is valid until the DRV_TOUCH_MTCH6301_Close routine is
    called. This routine will NEVER block waiting for hardware. If the
    requested intent flags are not supported, the routine will return
    DRV_HANDLE_INVALID. This function is thread safe in a RTOS application.
    It should not be called in an ISR.
*/

DRV_HANDLE DRV_TOUCH_MTCH6301_Open ( const SYS_MODULE_INDEX index,
                                                    const DRV_IO_INTENT intent )
{
    if (index >= DRV_TOUCH_MTCH6301_INDEX_COUNT)
    {
        SYS_ASSERT(false, "MTCH6301 Driver: Attempting to open an instance" \
                          "number greater than the max");
        return DRV_HANDLE_INVALID;
    }
    
    DRV_TOUCH_MTCH6301_OBJECT * pDrvInstance =
                 ( DRV_TOUCH_MTCH6301_OBJECT *)&sMTCH6301DriverInstances[index];

    DRV_TOUCH_MTCH6301_CLIENT_OBJECT *pClient = &sMTCH6301ClientInstances[0];
    if (pClient == NULL)
    {
        SYS_ASSERT(false, "MTCH6301 Driver: Couldn't find a free client to open");
        return DRV_HANDLE_INVALID;
    }

    pClient->driverObject = pDrvInstance;

    /* Open the bus driver */
    if(pDrvInstance->drvOpen == NULL)
    {
        SYS_ASSERT(false, "MTCH6301 Driver: Bus driver init parameter missing");
        return DRV_HANDLE_INVALID;
    }

    pDrvInstance->drvI2CHandle = pDrvInstance->drvOpen( DRV_TOUCH_MTCH6301_I2C_MODULE_INDEX,
                                                      DRV_IO_INTENT_READWRITE);
    if(pDrvInstance->drvI2CHandle == DRV_HANDLE_INVALID)
    {
        SYS_ASSERT(false, "MTCH6301 Driver: Bus driver initialization failed");
        return DRV_HANDLE_INVALID;
    }

    pClient->intent       = intent;
    pClient->pNext        = NULL;
    if ((intent & DRV_IO_INTENT_EXCLUSIVE) == DRV_IO_INTENT_EXCLUSIVE)
    {
        pDrvInstance->isExclusive = true;
    }
    
    pDrvInstance->numClients++;

    return (DRV_HANDLE)pClient;
}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_MTCH6301_Close ( DRV_HANDLE handle )

  Summary:
    Closes an opened instance of the MTCH6301 driver

  Description:
    This function closes an opened instance of the MTCH6301 driver, invalidating
    the handle.

  Precondition:
    The DRV_TOUCH_MTCH6301_Initialize routine must have been called for the
    specified MTCH6301 driver instance.

    DRV_TOUCH_MTCH6301_Open must have been called to obtain a valid opened
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None

  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_TOUCH_MTCH6301_Open

    DRV_TOUCH_MTCH6301_Close ( handle );
    </code>

  Remarks:
	After calling this routine, the handle passed in "handle" must not be
    used with any of the remaining driver routines.  A new handle must be
    obtained by calling DRV_TOUCH_MTCH6301_Open before the caller may use the
    driver again. This function is thread safe in a RTOS application.

    Note: Usually, there is no need for the driver client to verify that the
          Close operation has completed.
*/

void DRV_TOUCH_MTCH6301_Close ( DRV_HANDLE handle )
{
    DRV_TOUCH_MTCH6301_CLIENT_OBJECT * pClient =
                                (DRV_TOUCH_MTCH6301_CLIENT_OBJECT *)handle;
    DRV_TOUCH_MTCH6301_OBJECT * pDrvObject = pClient->driverObject;

    if( pDrvObject == NULL )
    {
        SYS_ASSERT(false, "MTCH6301 Driver: Trying to close a client with invalid driver object");
         return;
    }

    if (pDrvObject->numClients == 0)
    {
         SYS_ASSERT(false, "MTCH6301 Driver: Trying to close a client which does not exist");
         return;
    }

    pDrvObject->numClients--;
    
    return;
}


/*********************************************************************
  Function:
    DRV_TOUCH_POSITION_SINGLE DRV_TOUCH_MTCH6301_TouchStatus( )

  Summary:
    Returns the status of the current touch input.

*/
DRV_TOUCH_POSITION_STATUS DRV_TOUCH_MTCH6301_TouchStatus( const SYS_MODULE_INDEX index )
{
    DRV_TOUCH_MTCH6301_OBJECT * pDrvInstance =
        (DRV_TOUCH_MTCH6301_OBJECT *)&sMTCH6301DriverInstances[index];
    return (pDrvInstance->touchStatus);
}


/*********************************************************************
  Function:
    void DRV_TOUCH_MTCH6301_TouchDataRead( )

  Summary:
    Notify the driver that the current touch data has been read

*/
void DRV_TOUCH_MTCH6301_TouchDataRead( const SYS_MODULE_INDEX index )
{
    DRV_TOUCH_MTCH6301_OBJECT * pDrvInstance =
        (DRV_TOUCH_MTCH6301_OBJECT *)&sMTCH6301DriverInstances[index];
    pDrvInstance->touchStatus = DRV_TOUCH_POSITION_NONE;
}


/*********************************************************************
  Function:
    short DRV_TOUCH_MTCH6301_TouchGetX( uint8 touchNumber )

  Summary:
    Returns the x coordinate of touch input.

  Description:
    It returns the x coordinate in form of number of pixes for a touch input
  denoted by touchNumber.

  Parameters
   touchNumber - index to the touch input.

  Returns
    It returns the x coordinate of the touch input in terms of number of pixels.

*/

short DRV_TOUCH_MTCH6301_TouchGetX( uint8_t touchNumber )
{
    return (PCapX[touchNumber]);
}

/*********************************************************************
  Function:
    short DRV_TOUCH_MTCH6301_TouchGetY( uint8 touchNumber )

  Summary:
    Returns the y coordinate of touch input.

  Description:
    It returns the y coordinate in form of number of pixes for a touch input
  denoted by touchNumber.

  Parameters
   touchNumber - index to the touch input.

  Returns
    It returns the y coordinate of the touch input in terms of number of pixels.

*/

short DRV_TOUCH_MTCH6301_TouchGetY( uint8_t touchNumber )
{
    return PCapY[touchNumber];
}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_MTCH6301_ReadRequest( SYS_MODULE_OBJ object )

  Summary:
    Sends a read request to I2C bus driver and adds the read task to queue.

  Description:
	This routine is used to send a touch input read request to the I2C bus
        driver and adding the input read decode task to the queue. It is always
        called from MTCH6301 interrupt ISR routine.

  Precondition:
    The DRV_TOUCH_MTCH6301_Initialize routine must have been called for the
    specified MTCH6301 driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_TOUCH_MTCH6301_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;   // Returned from DRV_TOUCH_MTCH6301_Initialize

    void __ISR(_EXTERNAL_INT_VECTOR, ipl5) _IntHandlerDrvMtch6301(void)
    {
        DRV_TOUCH_MTCH6301_ReadRequest ( object );

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the MTCH6301 ISR routine.

*/

void DRV_TOUCH_MTCH6301_ReadRequest( SYS_MODULE_OBJ object )
{
    DRV_TOUCH_MTCH6301_OBJECT * pDrvObject =
                                        (DRV_TOUCH_MTCH6301_OBJECT *)object;

    if ( object == SYS_MODULE_OBJ_INVALID )
    {
        return;
    }
    
    // Verifying if queue is full
    if(pDrvObject->readRequest >= DRV_TOUCH_MTCH6301_NUM_QUEUE -1)
    {
        return;
    }

    // Verifying if I2C driver handle is valid
    if(pDrvObject->drvI2CHandle == DRV_HANDLE_INVALID)
    {
        return;
    }

    pDrvObject->readRequest++;


    if( pDrvObject->taskQueue[pDrvObject->readRequest - 1].inUse == true)
    {
        return;
    }

    pDrvObject->taskQueue[pDrvObject->readRequest - 1].drvI2CReadFrameData[0] =
                        DRV_TOUCH_MTCH6301_I2C_MASTER_READ_ID;

    pDrvObject->taskQueue[pDrvObject->readRequest - 1].drvI2CReadBufferHandle =
            DRV_I2C_Receive(
                            pDrvObject->drvI2CHandle,                            
     pDrvObject->taskQueue[pDrvObject->readRequest - 1].drvI2CReadFrameData[0],
                            (I2C_DATA_TYPE *)
     &pDrvObject->taskQueue[pDrvObject->readRequest - 1].drvI2CReadFrameData[1],
                            DRV_TOUCH_MTCH6301_I2C_READ_FRAME_SIZE - 1,
                            NULL );

    if( !pDrvObject->taskQueue[pDrvObject->readRequest - 1].drvI2CReadBufferHandle )
    {
       return;
    }

    pDrvObject->taskQueue[pDrvObject->readRequest - 1].inUse     = true;
    pDrvObject->taskQueue[pDrvObject->readRequest - 1].taskState =
                                    DRV_TOUCH_MTCH6301_TASK_STATE_DECODE_INPUT;
    return;
}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_MTCH6301_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's state machine and implements its task queue
    processing.

  Description:
	This routine is used to maintain the driver's internal state
	machine and implement its command queue processing. It is always called
        from SYS_Tasks() function. This routine decodes the touch input data
        available in drvI2CReadFrameData.

  Precondition:
    The DRV_TOUCH_MTCH6301_Initialize routine must have been called for the
    specified MTCH6301 driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_TOUCH_MTCH6301_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;   // Returned from DRV_TOUCH_MTCH6301_Initialize

    void SYS_Tasks( void )
    {
        DRV_TOUCH_MTCH6301_Tasks ( object );

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks)

*/

void DRV_TOUCH_MTCH6301_Tasks ( SYS_MODULE_OBJ object )
{
    static int32_t  taskIndex = 0;
    static uint8_t  touchpoint = 0;
    int16_t lastX;
    int16_t lastY;
     
    DRV_TOUCH_MTCH6301_OBJECT * pDrvObject =
            (DRV_TOUCH_MTCH6301_OBJECT *)object;

    if ( object == SYS_MODULE_OBJ_INVALID )
    {
        return;
    }

    if( pDrvObject->readRequest == 0)
    {     
        return;
    }

    while(taskIndex < pDrvObject->readRequest)
    {
        if( pDrvObject->taskQueue[taskIndex].inUse == false )
        {
            return;
        }

        if( pDrvObject->taskQueue[taskIndex].taskState ==
                DRV_TOUCH_MTCH6301_TASK_STATE_INIT ||
            pDrvObject->taskQueue[taskIndex].taskState ==
                DRV_TOUCH_MTCH6301_TASK_STATE_DONE )
        {
            return;
        }

        if( pDrvObject->taskQueue[taskIndex].taskState
                == DRV_TOUCH_MTCH6301_TASK_STATE_DECODE_INPUT )
        {
            if( DRV_I2C_BUFFER_EVENT_COMPLETE != DRV_I2C_BufferStatus(pDrvObject->taskQueue[taskIndex].drvI2CReadBufferHandle))
            {
                return;
            }

            if( (pDrvObject->taskQueue[taskIndex].drvI2CReadFrameData[2] & DRV_MTCH_BITMASK_PEN_STATE)
                 &&
             !(pDrvObject->taskQueue[taskIndex].drvI2CReadFrameData[2] & DRV_MTCH_TOUCH_ID_BIT_3) )
            {              
                lastX  = pDrvObject->taskQueue[taskIndex].drvI2CReadFrameData[3]
                        & DRV_MTCH_XY_AXIS_LOWER_BITS_MASK;
                lastX |= 
         (uint16_t)(( pDrvObject->taskQueue[taskIndex].drvI2CReadFrameData[4] &
                        DRV_MTCH_XY_AXIS_HIGHER_BITS_MASK )<<SHIFT_BY_7);                   

                lastY = pDrvObject->taskQueue[taskIndex].drvI2CReadFrameData[5]
                        & DRV_MTCH_XY_AXIS_LOWER_BITS_MASK;
                lastY |= 
         (uint16_t)(( pDrvObject->taskQueue[taskIndex].drvI2CReadFrameData[6] &
                        DRV_MTCH_XY_AXIS_HIGHER_BITS_MASK )<<SHIFT_BY_7);

                if ( pDrvObject->orientation == 180 )
                {
                    PCapX[touchpoint] =
                            pDrvObject->horizontalResolution - (( lastX * pDrvObject->horizontalResolution ) >> SHIFT_BY_10);
                    PCapY[touchpoint] =
                            pDrvObject->verticalResolution  - (( lastY * pDrvObject->verticalResolution ) >> SHIFT_BY_10);
                } else if ( pDrvObject->orientation == 90 )
                {
                    PCapX[touchpoint] =
                            pDrvObject->verticalResolution  - (( lastY * pDrvObject->verticalResolution ) >> SHIFT_BY_10);
                    PCapY[touchpoint] =
                            (( lastX * pDrvObject->horizontalResolution ) >> SHIFT_BY_10);
                } else if ( pDrvObject->orientation == 270 )
                {
                    PCapX[touchpoint] =
                            (( lastY * pDrvObject->verticalResolution ) >> SHIFT_BY_10);
                    PCapY[touchpoint] =
                            pDrvObject->horizontalResolution - (( lastX * pDrvObject->horizontalResolution ) >> SHIFT_BY_10);
                } else
                {
                    PCapX[touchpoint] =
                            (( lastX * pDrvObject->horizontalResolution ) >> SHIFT_BY_10)  -  CAL_X_INSET;
                    PCapY[touchpoint] =
                            ( lastY * pDrvObject->verticalResolution ) >> SHIFT_BY_10;
                }
                pDrvObject->touchStatus = DRV_TOUCH_POSITION_SINGLE;
            }

            if(!(pDrvObject->taskQueue[taskIndex].drvI2CReadFrameData[2] & DRV_MTCH_BITMASK_PEN_STATE)
                 &&
             !(pDrvObject->taskQueue[taskIndex].drvI2CReadFrameData[2] & DRV_MTCH_TOUCH_ID_BIT_3) )
            {       
                PCapX[touchpoint] = -1;
                PCapY[touchpoint] = -1;
                pDrvObject->touchStatus = DRV_TOUCH_POSITION_SINGLE;

            }

            pDrvObject->taskQueue[taskIndex].taskState
                            = DRV_TOUCH_MTCH6301_TASK_STATE_DONE;
            pDrvObject->taskQueue[taskIndex].inUse = false;
                
            taskIndex++;
             
        }
    }

    pDrvObject->readRequest = 0;
    taskIndex = 0;
    
    return;
}
