/*******************************************************************************
  Touch Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_touch.h

  Summary:
    Touch device driver interface file.

  Description:
    The Touch driver provides a abstraction to all touch drivers.

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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


#ifndef _DRV_TOUCH_H
#define _DRV_TOUCH_H


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "system_config.h"
#include "driver/driver_common.h"			// Common Driver Definitions
#include "system/common/sys_common.h"      	// Common System Service Definitions
#include "system/common/sys_module.h"      	// Module/Driver Definitions
#include "system/int/sys_int.h"

#ifdef __cplusplus
    extern "C" {
#endif
        
// *****************************************************************************
/* Touch Driver Module Index Numbers

  Summary:
    Touch driver index definitions.

  Description:
    These constants provide the Touch driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_TOUCH_Initialize and
    DRV_TOUCH_Open functions to identify the driver instance in use.
*/

#define DRV_TOUCH_INDEX_0         0
#define DRV_TOUCH_INDEX_1         1


// *****************************************************************************
/* TOUCH Controller Driver Touch status

  Summary:
    Identifies the current status of the current touch point.

  Description:
    Identifies the current status of the current touch point.

  Remarks:
    This enumeration is the return type for the status routine
    for the current touch point
*/
typedef enum
{
    /*An unspecified error has occurred.*/
    DRV_TOUCH_POSITION_ERROR  = -1,

    // The module position is not avaliable
    DRV_TOUCH_POSITION_NONE = 0,

    // The module has a single touch point
    DRV_TOUCH_POSITION_SINGLE = 2

} DRV_TOUCH_POSITION_STATUS;

// *****************************************************************************
/* TOUCH Controller Driver Pen State

  Summary:
    Identifies the current state of the pen.

  Description:
    Identifies the current state of the pen reported from a touch event.

  Remarks:
    This enumeration is the return type for the TouchGetPen routine.
*/
typedef enum
{
    /* Pen up state */
    DRV_TOUCH_PEN_UNKNOWN  = -1,
            
    /* Pen up state */
    DRV_TOUCH_PEN_UP  = 0,

    /* Pen down state */
    DRV_TOUCH_PEN_DOWN = 1,

} DRV_TOUCH_PEN_STATE;

// *****************************************************************************
/* TOUCH Driver Module Index Count

  Summary:
    Number of valid TOUCH driver indices.

  Description:
    This constant identifies the number of valid TOUCH driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is derived from device-specific header files defined as part of the
    peripheral libraries.
*/

#define DRV_TOUCH_INDEX_COUNT     1

typedef struct
{
    short touchCalUlx;
    short touchCalUly;
    short touchCalUrx;
    short touchCalUry;
    short touchCalLrx;
    short touchCalLry;
    short touchCalLlx;
    short touchCalLly;
} DRV_TOUCH_SAMPLE_POINTS;

// *****************************************************************************
/* TOUCH Driver Initialization Data

  Summary:
    Defines the data required to initialize or reinitialize the TOUCH driver

  Description:
    This data type defines the data required to initialize or reinitialize the
    TOUCH driver. If the driver is built statically, the members of this data
    structure are statically over-ridden by static override definitions in the
    system_config.h file.

  Remarks:
    None.
*/
typedef struct
{
    /* System module initialization */
    SYS_MODULE_INIT        moduleInit;

    /* ID */
    int                    touchId;

    /* */
    SYS_MODULE_OBJ         (*drvInitialize) (const SYS_MODULE_INDEX   index,
                                                const SYS_MODULE_INIT    * const init);

    /* */
    DRV_HANDLE             (*drvOpen) ( const SYS_MODULE_INDEX index, const DRV_IO_INTENT intent );

    /* */
    void                   (*drvCalibrationSet) (  DRV_TOUCH_SAMPLE_POINTS * samplePoints );

    /* */
    short                  (*drvTouchGetX)( uint8_t touchNumber);

    /* */
    short                  (*drvTouchGetY)( uint8_t touchNumber);

    /* */
    DRV_TOUCH_POSITION_STATUS  (*drvTouchStatus)( const SYS_MODULE_INDEX index );

    /* */
    void                   (*drvTouchDataRead)( const SYS_MODULE_INDEX index );

    /* */
    DRV_TOUCH_PEN_STATE    (*drvTouchPenGet)( uint8_t touchNumber );
    
    /* */
    INT_SOURCE             interruptSource;

    /* */
    uint16_t	           orientation;          // Orientation of the display (given in degrees of 0,90,180,270)

    /* */
    uint16_t               horizontalResolution; // Horizontal Resolution of the displayed orientation in Pixels

    /* */
    uint16_t               verticalResolution;

    /* */
    uint16_t              (*pReadFunc)(uint32_t);           // typedef for read function pointer

    /* */
    void                  (*pWriteFunc)(uint16_t, uint32_t);    // typedef for write function pointer

    /* */
    void                  (*pSectorErase)(uint32_t);
    
    int32_t                 minTouchDetectDelta;

} DRV_TOUCH_INIT;


// *****************************************************************************
// *****************************************************************************
// Section: Touch Driver Module Initialization Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void DRV_TOUCH_Initialize ( const TOUCH_MODULE_ID    index,
                              const SYS_MODULE_INIT *const data )

  Summary:
    Initializes hardware and data for the index instance of the TOUCH module.

  Description:
    This function initializes hardware for the index instance of the TOUCH module,
    using the hardware initialization given data.  It also initializes any
    internal driver data structures making the driver ready to be opened.

  Precondition:
    None.

  Parameters:
    index       - Index, identifying the instance of the TOUCH module to be
                  initialized

    data        - Pointer to the data structure containing any data necessary to
                  initialize the hardware.  This pointer may be null if no data
                  is required and the default initialization is to be used.

  Returns:
    None

  Example:
    <code>
    DRV_TOUCH_INIT_DATA         touchInitData;
    SYS_STATUS                  touchStatus;

    // Populate the touchInitData structure
    touchInitData.moduleInit.powerState = SYS_MODULE_POWER_RUN_FULL;
    touchInitData.moduleInit.moduleCode = (DRV_TOUCH_INIT_DATA_MASTER | DRV_TOUCH_INIT_DATA_SLAVE);

    DRV_TOUCH_Initialize(DRV_TOUCH_ID_1, (SYS_MODULE_INIT*)&touchInitData);
    touchStatus = DRV_TOUCH_Status(DRV_TOUCH_ID_1);
    </code>

  Remarks:

*/
SYS_MODULE_OBJ DRV_TOUCH_Initialize ( const SYS_MODULE_INDEX        index,
                                   const SYS_MODULE_INIT * const init );


/*******************************************************************************
  Function:
    void DRV_TOUCH_Reinitialize( const SYS_MODULE_ID    index,
                               const SYS_MODULE_INIT *const data )

  Summary:


  Description:


  Precondition:
    The DRV_TOUCH_Initialize function should have been called before calling this
    function.

  Parameters:
    index       - Index, identifying the instance of the TOUCH module to be
                  reinitialized

    data        - Pointer to the data structure containing any data necessary to
                  reinitialize the hardware.  This pointer may be null if no
                  data is required and default configuration is to be used.

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_INIT touchInit;
    SYS_STATUS      touchStatus;

    DRV_TOUCH_Reinitialize(DRV_TOUCH_ID_1, &touchStatus);

    </code>

  Remarks:

*/

void DRV_TOUCH_Reinitialize ( const SYS_MODULE_INDEX index, const SYS_MODULE_INIT *const data );


/*******************************************************************************
  Function:
    void DRV_TOUCH_Deinitialize ( const SYS_MODULE_ID index )

  Summary:
    Deinitializes the index instance of the TOUCH module.

  Description:
    This function deinitializes the index instance of the TOUCH module, disabling
    its operation (and any hardware for driver modules).  It deinitializes only
    the specified module instance.  It also resets all the internal data
    structures and fields for the specified instance to the default settings.

  Precondition:
    The DRV_TOUCH_Initialize function should have been called before calling this
    function.

  Parameters:
    index       - Index, identifying the instance of the TOUCH module to be
                  deinitialized

  Returns:
    None.

  Example:
    <code>
    SYS_STATUS   touchstatus;

    DRV_TOUCH_Deinitialize(DRV_TOUCH_ID_1);

    touchstatus = DRV_TOUCH_Status(DRV_TOUCH_ID_1);

    </code>

  Remarks:
*/

void DRV_TOUCH_Deinitialize ( const SYS_MODULE_INDEX index );


// *****************************************************************************
// *****************************************************************************
// Section: TOUCH Driver Module Status Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    SYS_STATUS DRV_TOUCH_Status ( const TOUCH_MODULE_ID index )

  Summary:
    Provides the current status of the index instance of the TOUCH module.

  Description:
    This function provides the current status of the index instance of the TOUCH
    module.

  Precondition:
    The DRV_TOUCH_Initialize function should have been called before calling this
    function.

  Parameters:

  Example:

  Remarks:

*/

SYS_STATUS DRV_TOUCH_Status ( const SYS_MODULE_INDEX index );


// *****************************************************************************
// *****************************************************************************
// Section: Touch Driver Client Setup Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    DRV_HANDLE DRV_TOUCH_Open ( const SYS_MODULE_INDEX index,
                              const DRV_IO_INTENT intent )

  Summary:
    Opens the specified instance of the Touch driver for use and provides an
    "open-instance" handle.

  Description:
    This function opens the specified instance of the Touch module for use and
    provides a handle that is required to use the remaining driver routines.

    This function opens a specified instance of the Touch module driver for use by
    any client module and provides an "open-instance" handle that must be
    provided to any of the other Touch driver operations to identify the caller
    and the instance of the Touch driver/hardware module.

  Precondition:
    The DRV_TOUCH_Initialize routine must have been called for the specified TOUCH
    device instance and the DRV_TOUCH_Status must have returned SYS_STATUS_READY.

  Parameters:
    index       - Index, identifying the instance of the TOUCH module to be
                  opened.

    intent      - Flags parameter identifying the intended usage and behavior
                  of the driver.  Multiple flags may be ORed together to
                  specify the intended usage of the device.
                  See the DRV_IO_INTENT definition.


  Returns:
    If successful, the routine returns a valid open-instance handle (a value
    identifying both the caller and the module instance).  If an error occurs,
    the returned value is DRV_HANDLE_INVALID.

  Example:
    <code>
    DRV_HANDLE                touchHandle;
    DRV_TOUCH_CLIENT_STATUS   touchClientStatus;

    touchHandle = DRV_TOUCH_Open(DRV_TOUCH_ID_1, DRV_IO_INTENT_NONBLOCKING|DRV_IO_INTENT_READWRITE);
    if (DRV_HANDLE_INVALID == touchHandle)
    {
        // Handle open error
    }

    touchClientStatus = DRV_TOUCH_ClientStatus(touchHandle);

    // Close the device when it is no longer needed.
    DRV_TOUCH_Close(touchHandle);
    </code>

  Remarks:

*/

DRV_HANDLE DRV_TOUCH_Open ( const SYS_MODULE_INDEX index, const DRV_IO_INTENT intent );


/*******************************************************************************
  Function:
    void DRV_TOUCH_Close ( const DRV_HANDLE drvHandle )

  Summary:
    Closes an opened instance of an TOUCH module driver.

  Description:
    This function closes an opened instance of an TOUCH module driver, making the
    specified handle invalid.

  Precondition:
    The DRV_TOUCH_Initialize routine must have been called for the specified TOUCH
    device instance and the DRV_TOUCH_Status must have returned SYS_STATUS_READY.

    DRV_TOUCH_Open must have been called to obtain a valid opened device handle.

  Parameters:
    drvHandle   - A valid open-instance handle, returned from the driver's open
                  routine

  Returns:
    None.

  Example:
    <code>
    myTouchHandle = DRV_TOUCH_Open(DRV_TOUCH_ID_1, DRV_IO_INTENT_NONBLOCKING|DRV_IO_INTENT_READWRITE);

    DRV_TOUCH_Close(myTouchHandle);
    </code>

  Remarks:

  */

void DRV_TOUCH_Close ( DRV_HANDLE handle );


// *****************************************************************************
// *****************************************************************************
// Section: Touch Driver Client Status Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    size_t DRV_TOUCH_Read ( DRV_HANDLE drvHandle, void *buffer, size_t size )

  Summary:
    Notifies the driver that there is current touch data to read

  Description:
    Notifies the driver that there is current touch data to read
	
  Precondition:

  Parameters:

  Returns:

  Example:
    <code>

    </code>

  Remarks:
*/

size_t DRV_TOUCH_Read ( DRV_HANDLE drvHandle, void *buffer, size_t size );



// *****************************************************************************
/* Function:
    void DRV_TOUCH_Tasks ( SYS_MODULE_OBJ object );

  Summary:
    Maintains the driver's state machine and implements its task queue
    processing.
	<p><b>Implementation:</b> Dynamic</p>

  Description:
	This routine is used to maintain the driver's internal state
	machine and implement its command queue processing. It is always called
        from SYS_Tasks() function. This routine decodes the touch input data
        available.

  Precondition:
    The DRV_TOUCH_Initialize routine must have been called for the 
    specified MTCH6301 driver instance.

  Parameters:
    object      - Object handle for the specified driver instance (returned from
                  DRV_TOUCH_Initialize)

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object;   // Returned from DRV_TOUCH_MTCH6301_Initialize

    void SYS_Tasks( void )
    {
        DRV_TOUCH_Tasks ( object );

        // Do other tasks
    }
    </code>

  Remarks:
    This function is normally not called directly by an application.  It is
    called by the system's Tasks routine (SYS_Tasks)

*/
void DRV_TOUCH_Tasks ( SYS_MODULE_OBJ object );

#ifdef __cplusplus
    }
#endif
    
#endif //_DRV_TOUCH_H
