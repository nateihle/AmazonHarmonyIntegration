/*******************************************************************************
  Camera Driver Interface

  Company:
    Microchip Technology Inc.

  File Name:
    drv_camera.h

  Summary:
    Camera device driver interface file.

  Description:
    The Camera driver provides a abstraction to all camera drivers.

*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED 'AS IS' WITHOUT WARRANTY OF ANY KIND,
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


#ifndef _DRV_CAMERA_H
#define _DRV_CAMERA_H


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
/* Camera Driver Module Index Numbers

  Summary:
    Camera driver index definitions.

  Description:
    These constants provide the Camera driver index definitions.

  Remarks:
    These constants should be used in place of hard-coded numeric literals.

    These values should be passed into the DRV_CAMERA_Initialize and
    DRV_CAMERA_Open functions to identify the driver instance in use.
*/

#define DRV_CAMERA_INDEX_0         0
#define DRV_CAMERA_INDEX_1         1


// *****************************************************************************
/* CAMERA Driver Module Index Count

  Summary:
    Number of valid CAMERA driver indices.

  Description:
    This constant identifies the number of valid CAMERA driver indices.

  Remarks:
    This constant should be used in place of hard-coded numeric literals.

    This value is derived from device-specific header files defined as part of the
    peripheral libraries.
*/

#define DRV_CAMERA_INDEX_COUNT     1

typedef enum
{
    /* */
    CAMERA_MODULE_OVM7690 /* DOM-IGNORE-BEGIN */ = 0, /* DOM-IGNORE-END */

} CAMERA_MODULE_ID;

// *****************************************************************************
/* CAMERA Driver Interrupt Port Remap Initialization Data

  Summary:
    Defines the data required to initialize the CAMERA driver interrupt port
    remap.

  Description:
    This data type defines the data required to initialize the CAMERA driver
    interrupt port remap.

  Remarks:
    None.
*/
typedef struct
{
    /* */
    PORTS_REMAP_INPUT_FUNCTION inputFunction;

    /* */
    PORTS_REMAP_INPUT_PIN      inputPin;

    /* */
    PORTS_ANALOG_PIN           analogPin;

    /* */
    PORTS_PIN_MODE             pinMode;

    /* */
    PORTS_CHANNEL              channel;

    /* */
    PORTS_DATA_MASK            dataMask;

} DRV_CAMERA_INTERRUPT_PORT_REMAP;


// *****************************************************************************
/* CAMERA Driver Initialization Data

  Summary:
    Defines the data required to initialize or reinitialize the CAMERA driver.

  Description:
    This data type defines the data required to initialize or reinitialize the
    CAMERA driver. If the driver is built statically, the members of this data
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
    int                    cameraId;

    /* */
    SYS_MODULE_OBJ         (*drvInitialize) (const SYS_MODULE_INDEX   index,
                                                const SYS_MODULE_INIT    * const init);

    /* */
    DRV_HANDLE             (*drvOpen) ( const SYS_MODULE_INDEX index, const DRV_IO_INTENT intent );

    /* */
    INT_SOURCE             interruptSource;

    /* */
    DRV_CAMERA_INTERRUPT_PORT_REMAP interruptPort;

    /* */
    uint16_t	           orientation;          // Orientation of the display (given in degrees of 0,90,180,270)

    /* */
    uint16_t               horizontalResolution; // Horizontal Resolution of the displayed orientation in Pixels

    /* */
    uint16_t               verticalResolution;


} DRV_CAMERA_INIT;


// *****************************************************************************
// *****************************************************************************
// Section: Camera Driver Module Initialization Functions
// *****************************************************************************
// *****************************************************************************

//*******************************************************************************
/*
  Function:
    void DRV_CAMERA_Initialize ( const CAMERA_MODULE_ID    index,
                                 const SYS_MODULE_INIT *const data )

  Summary:
    Initializes hardware and data for the index instance of the CAMERA module.

  Description:
    This function initializes hardware for the index instance of the CAMERA module,
    using the hardware initialization given data. It also initializes any
    internal driver data structures making the driver ready to be opened.

  Precondition:
    None.

  Parameters:
    index       - Index, identifying the instance of the CAMERA module to be
                  initialized

    data        - Pointer to the data structure containing any data necessary to
                  initialize the hardware.  This pointer may be null if no data
                  is required and the default initialization is to be used.

  Returns:
    None.

  Example:
    <code>
    DRV_CAMERA_INIT_DATA        cameraInitData;
    SYS_STATUS                  cameraStatus;

    // Populate the cameraInitData structure
    cameraInitData.moduleInit.powerState = SYS_MODULE_POWER_RUN_FULL;
    cameraInitData.moduleInit.moduleCode = (DRV_CAMERA_INIT_DATA_MASTER | DRV_CAMERA_INIT_DATA_SLAVE);

    DRV_CAMERA_Initialize(DRV_CAMERA_ID_1, (SYS_MODULE_INIT*)&cameraInitData);
    cameraStatus = DRV_CAMERA_Status(DRV_CAMERA_ID_1);
    </code>

  Remarks:
    None.

*/
SYS_MODULE_OBJ DRV_CAMERA_Initialize ( const SYS_MODULE_INDEX        index,
                                       const SYS_MODULE_INIT * const init );


//*******************************************************************************
/*
  Function:
    void DRV_CAMERA_Reinitialize( const SYS_MODULE_ID    index,
                                  const SYS_MODULE_INIT *const data )

  Summary:
    Reinitializes hardware and data for the index instance of the CAMERA module.

  Description:
    This function reinitializes hardware for the index instance of the CAMERA module,
    using the hardware initialization given data. It also reinitializes any
    internal driver data structures making the driver ready to be opened.


  Precondition:
    The DRV_CAMERA_Initialize function should have been called before calling this
    function.

  Parameters:
    index       - Index, identifying the instance of the CAMERA module to be
                  reinitialized

    data        - Pointer to the data structure containing any data necessary to
                  reinitialize the hardware.  This pointer may be null if no
                  data is required and default configuration is to be used.

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_INIT cameraInit;
    SYS_STATUS      cameraStatus;

    DRV_CAMERA_Reinitialize(DRV_CAMERA_ID_1, &cameraStatus);
    </code>

  Remarks:
    None.

*/

void DRV_CAMERA_Reinitialize ( const SYS_MODULE_INDEX index, const SYS_MODULE_INIT *const data );


//*******************************************************************************
/*
  Function:
    void DRV_CAMERA_Deinitialize ( const SYS_MODULE_ID index )

  Summary:
    Deinitializes the index instance of the CAMERA module.

  Description:
    This function deinitializes the index instance of the CAMERA module, disabling
    its operation (and any hardware for driver modules).  It deinitializes only
    the specified module instance.  It also resets all the internal data
    structures and fields for the specified instance to the default settings.

  Precondition:
    The DRV_CAMERA_Initialize function should have been called before calling this
    function.

  Parameters:
    index       - Index, identifying the instance of the CAMERA module to be
                  deinitialized

  Returns:
    None.

  Example:
    <code>
    SYS_STATUS   cameraStatus;

    DRV_CAMERA_Deinitialize(DRV_CAMERA_ID_1);

    cameraStatus = DRV_CAMERA_Status(DRV_CAMERA_ID_1);
    </code>

  Remarks:
    None.
*/

void DRV_CAMERA_Deinitialize ( const SYS_MODULE_INDEX index );


// *****************************************************************************
// *****************************************************************************
// Section: CAMERA Driver Module Status Functions
// *****************************************************************************
// *****************************************************************************

//*******************************************************************************
/*
  Function:
    SYS_STATUS DRV_CAMERA_Status ( const CAMERA_MODULE_ID index )

  Summary:
    Provides the current status of the index instance of the CAMERA module.

  Description:
    This function provides the current status of the index instance of the CAMERA
    module.

  Precondition:
    The DRV_CAMERA_Initialize function should have been called before calling this
    function.

  Parameters:
    None.
	
  Returns:
    The current status of the index instance.

  Remarks:
    None.

*/

SYS_STATUS DRV_CAMERA_Status ( const SYS_MODULE_INDEX index );


// *****************************************************************************
// *****************************************************************************
// Section: Camera Driver Client Setup Functions
// *****************************************************************************
// *****************************************************************************

//*******************************************************************************
/*
  Function:
    DRV_HANDLE DRV_CAMERA_Open ( const SYS_MODULE_INDEX index,
                                 const DRV_IO_INTENT intent )

  Summary:
    Opens the specified instance of the Camera driver for use and provides an
    "open instance" handle.

  Description:
    This function opens the specified instance of the Camera module for use and
    provides a handle that is required to use the remaining driver routines.

    This function opens a specified instance of the Camera module driver for use by
    any client module and provides an "open instance" handle that must be
    provided to any of the other Camera driver operations to identify the caller
    and the instance of the Camera driver/hardware module.

  Precondition:
    The DRV_CAMERA_Initialize routine must have been called for the specified CAMERA
    device instance and the DRV_CAMERA_Status must have returned SYS_STATUS_READY.

  Parameters:
    index       - Index, identifying the instance of the CAMERA module to be
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
    DRV_HANDLE                cameraHandle;
    DRV_CAMERA_CLIENT_STATUS   cameraClientStatus;

    cameraHandle = DRV_CAMERA_Open(DRV_CAMERA_ID_1, DRV_IO_INTENT_NONBLOCKING|DRV_IO_INTENT_READWRITE);
    if (DRV_HANDLE_INVALID == cameraHandle)
    {
        // Handle open error
    }

    cameraClientStatus = DRV_CAMERA_ClientStatus(cameraHandle);

    // Close the device when it is no longer needed.
    DRV_CAMERA_Close(cameraHandle);
    </code>

  Remarks:

*/

DRV_HANDLE DRV_CAMERA_Open ( const SYS_MODULE_INDEX index, const DRV_IO_INTENT intent );


//*******************************************************************************
/*
  Function:
    void DRV_CAMERA_Close ( const DRV_HANDLE drvHandle )

  Summary:
    Closes an opened instance of an CAMERA module driver.

  Description:
    This function closes an opened instance of an CAMERA module driver, making the
    specified handle invalid.

  Precondition:
    The DRV_CAMERA_Initialize routine must have been called for the specified CAMERA
    device instance and the DRV_CAMERA_Status must have returned SYS_STATUS_READY.

    DRV_CAMERA_Open must have been called to obtain a valid opened device handle.

  Parameters:
    drvHandle   - A valid open-instance handle, returned from the driver's open
                  routine

  Returns:
    None.

  Example:
    <code>
    myCameraHandle = DRV_CAMERA_Open(DRV_CAMERA_ID_1, DRV_IO_INTENT_NONBLOCKING|DRV_IO_INTENT_READWRITE);

    DRV_CAMERA_Close(myCameraHandle);
    </code>

  Remarks:
    None.

  */

void DRV_CAMERA_Close ( DRV_HANDLE handle );


// *****************************************************************************
// *****************************************************************************
// Section: Camera Driver Client Status Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************

void DRV_CAMERA_Tasks ( SYS_MODULE_OBJ object );

#ifdef __cplusplus
    }
#endif
    
#endif //_DRV_CAMERA_H
