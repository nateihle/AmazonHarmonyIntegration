/*******************************************************************************
 OVM7690 Camera Driver Implementation.

  File Name:
    drv_camera_ovm7690.c

  Summary:
    OVM7690 camera driver interface declarations for the
    static single instance driver.

  Description:
    The OVM7690 camera device driver provides a simple interface to manage
    the OVM7690 camera cube interfacing to Microchip microcontrollers. This
    file defines the implementation for the OVM7690 driver.
 ******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "driver/camera/ovm7690/drv_camera_ovm7690.h"
#include "framework/driver/i2c/drv_i2c_static.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/* This is the driver instance object array. */
DRV_CAMERA_OVM7690_OBJ        gDrvCameraOVM7690Obj[DRV_CAMERA_OVM7690_INSTANCES_NUMBER] ;

/* This is the client object array. */
DRV_CAMERA_OVM7690_CLIENT_OBJ gDrvCameraOVM7690ClientObj[DRV_CAMERA_OVM7690_CLIENTS_NUMBER];

// *****************************************************************************
// *****************************************************************************
// Section: Local prototypes
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Camera OVM7690 Driver Interface Implementations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
     SYS_MODULE_OBJ DRV_CAMERA_OVM7690_Initialize
     (
        const SYS_MODULE_INDEX index,
        const SYS_MODULE_INIT * const init 
     )

  Summary:
    Initializes the Camera OVM7690 instance for the specified driver index.

  Description:
    This routine initializes the Camera OVM7690 driver instance for the 
    specified driver index, making it ready for clients to open and use it. The 
    initialization data is specified by the init parameter. The initialization 
    may fail if the number of driver objects allocated are insufficient or if 
    the specified driver instance is already initialized. The driver instance 
    index is independent of the Camera OVM7690 module ID. Refer to
    the description of the DRV_CAMERA_OVM7690_INIT data structure for more 
    details on which members on this data structure are overridden.

  Precondition:
    None.

  Parameters:
    index  - Identifier for the instance to be initialized
    
    init   - Pointer to a data structure containing any data necessary to
             initialize the driver.

  Returns:
    If successful, returns a valid handle to a driver instance object.  
    Otherwise, returns SYS_MODULE_OBJ_INVALID.

  Example:
    <code>
    // The following code snippet shows an example OVM7690 driver initialization.
    
    DRV_CAMERA_OVM7690_INIT     cameraInit;
    SYS_MODULE_OBJ              objectHandle;

    cameraInit.cameraID                = CAMERA_MODULE_OVM7690;
    cameraInit.sourcePort              = (void *)&PORTK,
    cameraInit.hsyncInterruptSource    = INT_SOURCE_CHANGE_NOTICE_A,
    cameraInit.vsyncInterruptSource    = INT_SOURCE_CHANGE_NOTICE_J,
    cameraInit.dmaChannel              = DRV_CAMERA_OVM7690_DMA_CHANNEL_INDEX,
    cameraInit.dmaTriggerSource        = DMA_TRIGGER_EXTERNAL_2,
    cameraInit.bpp                     = GFX_CONFIG_COLOR_DEPTH,

    objectHandle = DRV_CAMERA_OVM7690_Initialize( DRV_CAMERA_OVM7690_INDEX_0, 
                                                (SYS_MODULE_INIT*)&cameraInit);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }
    </code>

  Remarks:
    This routine must be called before any other OVM7690 routine is called.

    This routine should only be called once during system initialization
    unless DRV_CAMERA_OVM7690_Deinitialize is called to deinitialize the driver 
    instance. This routine will NEVER block for hardware access.
 */

SYS_MODULE_OBJ DRV_CAMERA_OVM7690_Initialize
(
    const SYS_MODULE_INDEX drvIndex,
    const SYS_MODULE_INIT * const init
)
{
    DRV_CAMERA_OVM7690_OBJ  *dObj       = (DRV_CAMERA_OVM7690_OBJ*)NULL;
    DRV_CAMERA_OVM7690_INIT *cameraInit = NULL ;

    /* Check if the specified driver index is in valid range */
    if(drvIndex >= DRV_CAMERA_OVM7690_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "Invalid driver index");
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Check if this hardware instance was already initialized */
    if(gDrvCameraOVM7690Obj[drvIndex].inUse != false)
    {
        SYS_DEBUG(0, "Instance already in use");
        return SYS_MODULE_OBJ_INVALID;
    }

    /* Assign to the local pointer the init data passed */
    cameraInit = ( DRV_CAMERA_OVM7690_INIT * ) init ;
    
    /* Allocate the driver object and set the operation flag to be in use */
    dObj        = &gDrvCameraOVM7690Obj[drvIndex];
    dObj->inUse = true;
    
    dObj->nClients              = 0;
    dObj->frameBufferAddress    = NULL;
    dObj->frameLineAddress      = NULL;
    dObj->isExclusive           = false;
    dObj->dmaHandle             = SYS_DMA_CHANNEL_HANDLE_INVALID;
    dObj->dmaTransferComplete   = false;
    dObj->frameLineCount        = 0;
    dObj->frameLineSize         = 0;
    dObj->moduleId              = cameraInit->cameraID;
    dObj->dmaChannel            = cameraInit->dmaChannel;
    dObj->hsyncInterruptSource  = cameraInit->hsyncInterruptSource;
    dObj->vsyncInterruptSource  = cameraInit->vsyncInterruptSource;
    dObj->dmaTriggerSource      = cameraInit->dmaTriggerSource;
    dObj->sourcePort            = cameraInit->sourcePort;
    dObj->hsyncChannel          = cameraInit->hsyncChannel;
    dObj->hsyncPosition         = cameraInit->hsyncPosition;
    dObj->vsyncChannel          = cameraInit->vsyncChannel;
    dObj->vsyncPosition         = cameraInit->vsyncPosition;
    dObj->bpp                   = cameraInit->bpp;
    
    /* Setup the Hardware */
    _DRV_CAMERA_OVM7690_HardwareSetup( dObj ) ;
    
    /* Clear the interrupts */
    SYS_INT_SourceStatusClear(dObj->hsyncInterruptSource);
    SYS_INT_SourceStatusClear(dObj->vsyncInterruptSource);
    
    /* Update the status */
    dObj->status = SYS_STATUS_READY_EXTENDED;

    /* Return the object structure */
    return ( (SYS_MODULE_OBJ)drvIndex );
}

// *****************************************************************************
/* Function:
    void DRV_CAMERA_OVM7690_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Deinitializes the specified instance of the Camera OVM7690 driver module.

  Description:
    Deinitializes the specified instance of the Camera OVM7690 driver module, 
    disabling its operation (and any hardware). Invalidates all the internal 
    data.

  Precondition:
    Function DRV_CAMERA_OVM7690_Initialize should have been called before 
    calling this function.

  Parameters:
    object          - Driver object handle, returned from the 
                      DRV_CAMERA_OVM7690_Initialize routine

  Returns:
    None.

  Example:
    <code>
    SYS_MODULE_OBJ      object; //  Returned from DRV_CAMERA_OVM7690_Initialize
    SYS_STATUS          status;

    DRV_CAMERA_OVM7690_Deinitialize(object);

    status = DRV_CAMERA_OVM7690_Status(object);
    if (SYS_MODULE_DEINITIALIZED != status)
    {
        // Check again later if you need to know 
        // when the driver is deinitialized.
    }
    </code>

  Remarks:
    Once the Initialize operation has been called, the Deinitialize operation
    must be called before the Initialize operation can be called again. This 
    routine will NEVER block waiting for hardware.
 */
    
void DRV_CAMERA_OVM7690_DeInitialize(SYS_MODULE_OBJ object)
{
    DRV_CAMERA_OVM7690_OBJ * dObj;

    /* Check that the object is valid */

    if(object == SYS_MODULE_OBJ_INVALID)
    {
        SYS_DEBUG(0, "Invalid system object handle" );
        return;
    }

    if(object >= DRV_CAMERA_OVM7690_INSTANCES_NUMBER)
    {
        SYS_DEBUG(0, "Invalid system object handle" );
        return;
    }

    dObj = (DRV_CAMERA_OVM7690_OBJ*) &gDrvCameraOVM7690Obj[object];

    if(!dObj->inUse)
    {
        SYS_DEBUG(0, "Invalid system object handle");
        return;
    }

    /* The driver will not have clients when it is
       being deinitialized. So the order in which
       we do the following steps is not that important */

    /* Indicate that this object is not is use */
    dObj->inUse = false;

    /* Deinitialize the Camera status */
    dObj->status =  SYS_STATUS_UNINITIALIZED ;

    /* Disable the interrupt */
    SYS_INT_SourceDisable(dObj->hsyncInterruptSource) ;
    SYS_INT_SourceDisable(dObj->vsyncInterruptSource) ;

}

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_CAMERA_OVM7690_Open
    (
        const SYS_MODULE_INDEX index,
        const DRV_IO_INTENT ioIntent
    )

  Summary:
    Opens the specified Camera OVM7690 driver instance and returns a handle to 
    it.

  Description:
    This routine opens the specified Camera OVM7690 driver instance and provides
    a handle that must be provided to all other client-level operations to
    identify the caller and the instance of the driver. The ioIntent 
    parameter defines how the client interacts with this driver instance.

  Precondition:
    Function DRV_CAMERA_OVM7690_Initialize must have been called before calling 
    this function.

  Parameters:
    index   - Identifier for the object instance to be opened
    
    intent  - Zero or more of the values from the enumeration
              DRV_IO_INTENT "ORed" together to indicate the intended use
              of the driver. See function description for details.

  Returns:
    If successful, the routine returns a valid open-instance handle (a number
    identifying both the caller and the module instance).
    
    If an error occurs, the return value is DRV_HANDLE_INVALID. Error can occur
    - if the number of client objects allocated via DRV_CAMERA_OVM7690_CLIENTS_NUMBER 
      is insufficient.
    - if the client is trying to open the driver but driver has been opened
      exclusively by another client.
    - if the driver hardware instance being opened is not initialized or is
      invalid.
    - if the client is trying to open the driver exclusively, but has already
      been opened in a non exclusive mode by another client.
    - if the driver is not ready to be opened, typically when the initialize
      routine has not completed execution.

  Example:
    <code>
    DRV_HANDLE handle;

    handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        // Unable to open the driver
        // May be the driver is not initialized or the initialization
        // is not complete.
    }
    </code>

  Remarks:
    The handle returned is valid until the DRV_CAMERA_OVM7690_Close routine is 
    called. This routine will NEVER block waiting for hardware.If the requested 
    intent flags are not supported, the routine will return DRV_HANDLE_INVALID. 
    This function is thread safe in a RTOS application.

 */

DRV_HANDLE DRV_CAMERA_OVM7690_Open
(
    const SYS_MODULE_INDEX drvIndex,
    const DRV_IO_INTENT ioIntent
)
{
    DRV_CAMERA_OVM7690_CLIENT_OBJ *clientObj;
    DRV_CAMERA_OVM7690_OBJ *dObj;
    unsigned int iClient = 0;

    if (drvIndex >= DRV_CAMERA_OVM7690_INSTANCES_NUMBER)
    {
        /* Invalid driver index */
        SYS_DEBUG(0, "Invalid Driver Instance");
        return (DRV_HANDLE_INVALID);
    }

    dObj = &gDrvCameraOVM7690Obj[drvIndex];

    if((dObj->status != SYS_STATUS_READY) || (dObj->inUse == false))
    {
        /* The Camera module should be ready */

        SYS_DEBUG(0, "Was the driver initialized?");
        return DRV_HANDLE_INVALID;
    }

    if(dObj->isExclusive)
    {
        /* This means the another client has opened the driver in exclusive
           mode. The driver cannot be opened again */

        SYS_DEBUG(0, "Driver already opened exclusively");
        return ( DRV_HANDLE_INVALID ) ;
    }

    if((dObj->nClients > 0) && (ioIntent & DRV_IO_INTENT_EXCLUSIVE))
    {
        /* This means the driver was already opened and another driver was
           trying to open it exclusively.  We cannot give exclusive access in
           this case */

        SYS_DEBUG(0, "Driver already opened. Cannot be opened exclusively");
        return(DRV_HANDLE_INVALID);
    }
    
    clientObj               = &gDrvCameraOVM7690ClientObj[iClient];
    clientObj->inUse        = true;
    clientObj->hDriver      = dObj;
    dObj->nClients ++;
    
    /* Update the client status */
    clientObj->status = DRV_CAMERA_OVM7690_CLIENT_STATUS_READY;
    return ((DRV_HANDLE) clientObj );
}

// *****************************************************************************
/* Function:
    void DRV_CAMERA_OVM7690_Close( DRV_Handle handle )

  Summary:
    Closes an opened-instance of the Camera OVM7690 driver.

  Description:
    This routine closes an opened-instance of the Camera OVM7690 driver, 
    invalidating the handle. Any buffers in the driver queue that were submitted
    by this client will be removed. After calling this routine, the handle 
    passed in "handle" must not be used with any of the remaining driver 
    routines (with one possible exception described in the "Remarks" section).
    A new handle must be obtained by calling DRV_CAMERA_OVM7690_Open before the 
    caller may use the driver again

  Precondition:
    The DRV_CAMERA_OVM7690_Initialize routine must have been called for the 
    specified Camera OVM7690 driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

  Parameters:
    handle       - A valid open-instance handle, returned from the driver's
                   open routine

  Returns:
    None.
                
  Example:
    <code>
    DRV_HANDLE handle;  // Returned from DRV_USART_Open

    DRV_CAMERA_OVM7690_Close(handle);
    
    </code>

  Remarks:
    Usually there is no need for the client to verify that the Close operation
    has completed.  The driver will abort any ongoing operations when this
    routine is called.
 */

void DRV_CAMERA_OVM7690_close ( DRV_HANDLE handle )
{
    /* This function closes the client, The client
       object is deallocated and returned to the
       pool. */

    DRV_CAMERA_OVM7690_CLIENT_OBJ * clientObj;
    DRV_CAMERA_OVM7690_OBJ * dObj;

    clientObj = (DRV_CAMERA_OVM7690_CLIENT_OBJ *) handle;
    
    if(clientObj == NULL)
    {
        /* Driver handle is not valid */
        SYS_DEBUG(0, "Invalid Driver Handle");
        return;
    }

    dObj = (DRV_CAMERA_OVM7690_OBJ *)clientObj->hDriver;

    /* Reduce the number of clients */
    dObj->nClients --;

    /* De-allocate the object */
    clientObj->status = DRV_CAMERA_OVM7690_CLIENT_STATUS_CLOSED;
    clientObj->inUse = false;

    return;
}

// *****************************************************************************
/* Function:
    DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_FrameBufferAddressSet
    ( 
     DRV_HANDLE handle,
     void * frameBuffer
    )

  Summary:
    Sets Frame buffer address.

  Description:
    This routine will set the Frame Buffer Address. This frame buffer address 
  will point to the location at which frame data is to be rendered. This buffer
  is shared with the display controller to display the frame on the display.
 
  Precondition:
    The DRV_CAMERA_OVM7690_Initialize routine must have been called for the 
    specified Camera OVM7690 driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

  Parameters:
    handle  - A valid open-instance handle, returned from the driver's
              open routine

  Returns:
    DRV_CAMERA_OVM7690_ERROR_INVALID_HANDLE - Invalid driver Handle.
    DRV_CAMERA_OVM7690_ERROR_NONE - No error.

  Example:
    <code>
      
        DRV_HANDLE handle;
        uint16_t frameBuffer[DISP_VER_RESOLUTION][DISP_HOR_RESOLUTION];

        handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
        if (DRV_HANDLE_INVALID == handle)
        {
            //error
            return;    
        }

        if ( DRV_CAMERA_OVM7690_FrameBufferAddressSet( handle, (void *) frameBuffer ) != 
                                            DRV_CAMERA_OVM7690_ERROR_NONE )
        {
            //error
            return;
        }
    
    </code>

  Remarks:
    This routine is mandatory. A valid frame buffer address need to be set to 
    display camera data.
 */

DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_FrameBufferAddressSet
( 
    DRV_HANDLE handle,
    void * frameBuffer
)
{
    DRV_CAMERA_OVM7690_CLIENT_OBJ * clientObj;
    DRV_CAMERA_OVM7690_OBJ * dObj;

    clientObj = (DRV_CAMERA_OVM7690_CLIENT_OBJ *) handle;
    
    if(clientObj == NULL)
    {
        /* Driver handle is not valid */
        SYS_DEBUG(0, "Invalid Driver Handle");
        return DRV_CAMERA_OVM7690_ERROR_INVALID_HANDLE;
    }

    dObj = (DRV_CAMERA_OVM7690_OBJ *)clientObj->hDriver;

    dObj->frameBufferAddress = frameBuffer;
    dObj->frameLineAddress = frameBuffer;
    
    return (DRV_CAMERA_OVM7690_ERROR_NONE);
}

// *****************************************************************************
/* Function:
    DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_FrameRectSet
    ( 
        DRV_HANDLE handle,
        uint32_t   left,
        uint32_t   top,
        uint32_t   right,
        uint32_t   bottom
    )

  Summary:
    Sets the Frame Rectangle Set.

  Description:
    This routine sets the frame rectangle coordinates. The Frame within the 
    rectangle is copied to the frame buffer. The left and top values are 
    expected to be less than right and bottom respectively. Left, top, right and 
    bottom values are also expected to be within range of screen coordinates.
    Internally it calls DRV_CAMERA_OVM7690_RegisterSet routine to set respective
    registers. The rectangle coordinates are also maintained in the driver
    object.

  Precondition:
    The DRV_CAMERA_OVM7690_Initialize routine must have been called for the 
    specified Camera OVM7690 driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

    The SCCB interface also must have been initialized to configure the Camera
    OVM7690.

  Parameters:
    handle  - A valid open-instance handle, returned from the driver's
            open routine

    left   - left frame coordinate
    top    - top frame coordinate
    right  - right frame coordinate
    bottom - bottom frame coordinate 

  Returns:
    DRV_CAMERA_OVM7690_ERROR_INVALID_HANDLE - Invalid driver Handle.
    DRV_CAMERA_OVM7690_ERROR_NONE - No error.

  Example:
    <code>
        DRV_HANDLE handle;
        uint32_t left   = 0x69;
        uint32_t top    = 0x0E;
        uint32_t right  = DISP_HOR_RESOLUTION + 0x69;
        uint32_t bottom = DISP_VER_RESOLUTION + 0x69;

        handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
        if (DRV_HANDLE_INVALID == handle)
        {
            //error
            return;    
        }

        if ( DRV_CAMERA_OVM7690_FrameRectSet( handle, left, top, right, bottom ) != 
                                             DRV_CAMERA_OVM7690_ERROR_NONE )
        {
            //error
            return;
        }

    </code>

  Remarks:
    This function is mandatory. It also calculates the frame line size in bytes
    used in initiation of dma transfer.
 */

DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_FrameRectSet
( 
    DRV_HANDLE handle,
    uint32_t   left,
    uint32_t   top,
    uint32_t   right,
    uint32_t   bottom
)
{
    DRV_CAMERA_OVM7690_CLIENT_OBJ * clientObj;
    DRV_CAMERA_OVM7690_OBJ * dObj;
    
    uint32_t cameraVerticalSize   = 0;
    uint32_t cameraHorizontalSize = 0;

    clientObj = (DRV_CAMERA_OVM7690_CLIENT_OBJ *) handle;
    
    if(clientObj == NULL)
    {
        /* Driver handle is not valid */
        SYS_DEBUG(0, "Invalid Driver Handle");
        return DRV_CAMERA_OVM7690_ERROR_INVALID_HANDLE;
    }

    dObj = (DRV_CAMERA_OVM7690_OBJ *)clientObj->hDriver;

    dObj->rect.left   = left;
    dObj->rect.top    = top;
    dObj->rect.right  = right;
    dObj->rect.bottom = bottom;
    dObj->frameLineSize = (dObj->rect.right - dObj->rect.left) * (dObj->bpp / 8);
    
    cameraHorizontalSize = ((dObj->rect.right - dObj->rect.left) & 0x3FF) >> 1; 
    cameraVerticalSize   = ((dObj->rect.bottom - dObj->rect.top) & 0x3FF) >> 1;
    
    //Horizontal Size
    while(DRV_CAMERA_OVM7690_RegisterSet( DRV_CAMERA_OVM7690_REG16_REG_ADDR, 
                                        (cameraHorizontalSize  & 0x001) << 6) 
                                        == false );
    _DRV_CAMERA_OVM7690_delayMS(100);
    
    while(DRV_CAMERA_OVM7690_RegisterSet( DRV_CAMERA_OVM7690_HSIZE_REG_ADDR, 
                                        (cameraHorizontalSize & 0x1FE) >> 1 ) 
                                        == false );
    _DRV_CAMERA_OVM7690_delayMS(100);
    
    //Vertical Size
    while(DRV_CAMERA_OVM7690_RegisterSet( DRV_CAMERA_OVM7690_VSIZE_REG_ADDR, 
                                        cameraVerticalSize & 0xFF ) 
                                        == false);
    _DRV_CAMERA_OVM7690_delayMS(100);
    
    //Horizontal Start
    while(DRV_CAMERA_OVM7690_RegisterSet( DRV_CAMERA_OVM7690_HSTART_REG_ADDR, 
                                        dObj->rect.left & 0xFF ) == false);
    _DRV_CAMERA_OVM7690_delayMS(100);

    //Vertical Start
    while(DRV_CAMERA_OVM7690_RegisterSet( DRV_CAMERA_OVM7690_VSTART_REG_ADDR, 
                                        dObj->rect.top & 0xFF ) == false);
    _DRV_CAMERA_OVM7690_delayMS(100);
            
    return (DRV_CAMERA_OVM7690_ERROR_NONE);
}

// *****************************************************************************
/* Function:
    DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_Start
    ( 
        DRV_HANDLE handle
    );

  Summary:
    Starts camera rendering to the display.

  Description:
    This routine starts the camera rendering to the display by writing the pixel
    data to the frame buffer. Frame buffer is shared between camera OVM7690 and
    display controller. 

  Precondition:
    The DRV_CAMERA_OVM7690_Initialize routine must have been called for the 
    specified Camera OVM7690 driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

    DRV_CAMERA_OVM7690_FrameBufferAddressSet must have been called to set a valid
    frame buffer address.

  Parameters:
    handle  - A valid open-instance handle, returned from the driver's
              open routine

  Returns:
    DRV_CAMERA_OVM7690_ERROR_INVALID_HANDLE - Invalid driver Handle.
    DRV_CAMERA_OVM7690_ERROR_NONE - No error.
 
  Example:
    <code>
    
        DRV_HANDLE handle;
        uint16_t frameBuffer[DISP_VER_RESOLUTION][DISP_HOR_RESOLUTION];

        handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
        if (DRV_HANDLE_INVALID == handle)
        {
            //error
            return;    
        }

        if ( DRV_CAMERA_OVM7690_FrameBufferAddressSet( handle, (void *) frameBuffer ) != 
                                            DRV_CAMERA_OVM7690_ERROR_NONE )
        {
            //error
            return;
        }

        if ( DRV_CAMERA_OVM7690_Start( handle ) != 
                                            DRV_CAMERA_OVM7690_ERROR_NONE )
        {
            //error
            return;
        }
 
    </code>

  Remarks:
    This routine is mandatory. Camera module will not update the framebuffer
    without calling this routine.
 */

DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_Start
( 
    DRV_HANDLE handle
)
{
    DRV_CAMERA_OVM7690_CLIENT_OBJ * clientObj;
    DRV_CAMERA_OVM7690_OBJ * dObj;
    DRV_CAMERA_OVM7690_ERROR returnValue = DRV_CAMERA_OVM7690_ERROR_NONE;

    clientObj = (DRV_CAMERA_OVM7690_CLIENT_OBJ *) handle;
    
    if(clientObj == NULL)
    {
        /* Driver handle is not valid */
        SYS_DEBUG(0, "Invalid Driver Handle");
        returnValue = DRV_CAMERA_OVM7690_ERROR_INVALID_HANDLE;
    }
    else
    {
        dObj = (DRV_CAMERA_OVM7690_OBJ *)clientObj->hDriver;    
        dObj->frameLineCount = 0;
        dObj->dmaTransferComplete = false;
    
        SYS_INT_SourceEnable(dObj->vsyncInterruptSource);
        SYS_INT_SourceEnable(dObj->hsyncInterruptSource) ;
    }

    return returnValue;
    
}

// *****************************************************************************
/* Function:
    DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_Stop
    ( 
        DRV_HANDLE handle
    );
 
  Summary:
    Stops rendering the camera Pixel data.

  Description:
    This routine starts the camera rendering to the display by writing the pixel
    data to the frame buffer. Frame buffer is shared between camera OVM7690 and
    display controller.

  Precondition:
    The DRV_CAMERA_OVM7690_Initialize routine must have been called for the 
    specified Camera OVM7690 driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

  Parameters:
    handle  - A valid open-instance handle, returned from the driver's
            open routine.

  Returns:
    DRV_CAMERA_OVM7690_ERROR_INVALID_HANDLE - Invalid driver Handle.
    DRV_CAMERA_OVM7690_ERROR_NONE - No error.
 
  Example:
    <code>

        DRV_HANDLE handle;

        handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
        if (DRV_HANDLE_INVALID == handle)
        {
            //error
            return;    
        }

        if ( DRV_CAMERA_OVM7690_Stop( handle ) != 
                                            DRV_CAMERA_OVM7690_ERROR_NONE )
        {
            //error
            return;
        }

    </code>

  Remarks:
    This routine only disables the interrupt for hsync and vsync. To stop the 
    camera the power down pin need to be toggled to active high value. This will
    stop the camera internal clock and will maintain the register values.
 */

DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_Stop
( 
    DRV_HANDLE handle
)
{
    DRV_CAMERA_OVM7690_CLIENT_OBJ * clientObj;
    DRV_CAMERA_OVM7690_OBJ * dObj;
    DRV_CAMERA_OVM7690_ERROR returnValue = DRV_CAMERA_OVM7690_ERROR_NONE;

    clientObj = (DRV_CAMERA_OVM7690_CLIENT_OBJ *) handle;
    
    if(clientObj == NULL)
    {
        /* Driver handle is not valid */
        SYS_DEBUG(0, "Invalid Driver Handle");
        returnValue = DRV_CAMERA_OVM7690_ERROR_INVALID_HANDLE;
    }
    else
    {
        dObj = (DRV_CAMERA_OVM7690_OBJ *)clientObj->hDriver;    
        dObj->frameLineCount = 0;
        dObj->dmaTransferComplete = false;
    
        SYS_INT_SourceDisable(dObj->vsyncInterruptSource);
        SYS_INT_SourceDisable(dObj->hsyncInterruptSource);
    
        /* Clear the interrupts */
        SYS_INT_SourceStatusClear(dObj->hsyncInterruptSource);
        SYS_INT_SourceStatusClear(dObj->vsyncInterruptSource);
    }

    return returnValue;
}

// *****************************************************************************
/* Function:
    DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_RegisterSet 
    ( 
        DRV_CAMERA_OVM7690_REGISTER_ADDRESS regIndex, 
        uint8_t regValue 
    )

  Summary:
    Sets the camera OVM7690 configuration registers

  Description:
    This routine sets the Camera OVM7690 configuration registers using SCCB
    interface. 

  Precondition:
    The DRV_CAMERA_OVM7690_Initialize routine must have been called for the 
    specified Camera OVM7690 driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

    The SCCB interface also must have been initialized to configure the Camera
    OVM7690.

  Parameters:
    regIndex - Defines the configuration register addresses for OVM7690.
    regValue - Defines the register value to be set.

  Returns:
    DRV_CAMERA_OVM7690_ERROR_INVALID_HANDLE - Invalid driver Handle.
    DRV_CAMERA_OVM7690_ERROR_NONE - No error.

  Example:
    <code>

       DRV_HANDLE handle;
       uint8_t reg12 = DRV_CAMERA_OVM7690_REG12_SOFT_RESET;

        handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
        if (DRV_HANDLE_INVALID == handle)
        {
            //error
            return;    
        }

        if ( DRV_CAMERA_OVM7690_RegisterSet( DRV_CAMERA_OVM7690_REG12_REG_ADDR,
                                             reg12 ) != 
                                             DRV_CAMERA_OVM7690_ERROR_NONE )
        {
            //error
            return;
        }

    </code>

  Remarks:
    This routine can be used separately or within a interface.
 */

DRV_CAMERA_OVM7690_ERROR DRV_CAMERA_OVM7690_RegisterSet 
( 
    DRV_CAMERA_OVM7690_REGISTER_ADDRESS regIndex, 
    uint8_t regValue 
)
{
    uint8_t frame[3] = {DRV_CAMERA_OVM7690_SCCB_WRITE_ID, 0, 0};
    uint32_t frameIndex = 0;
    
    frame[1] = regIndex;
    frame[2] = regValue;
    
        
    while(DRV_I2C0_MasterBusIdle() == false);
    
    DRV_I2C0_MasterStart();
            
    while ( DRV_I2C0_WaitForStartComplete() == false ) 
        ;
        
    for( frameIndex = 0; frameIndex < 3; frameIndex++ )
    {
        DRV_I2C0_ByteWrite(frame[frameIndex]);
        while ( DRV_I2C0_WaitForByteWriteToComplete() == false )
            ;
        
        if(DRV_I2C0_WriteByteAcknowledged() == false)
        {
            DRV_I2C0_MasterStop();
            return (false);
        }        
    }
       
   DRV_I2C0_MasterStop();
   while ( DRV_I2C0_WaitForStopComplete() == false ) 
        ;
       
    return (true);
}

// *****************************************************************************
/* Function:
    void DRV_CAMERA_OVM7690_HsyncEventHandler(SYS_MODULE_OBJ object)
 
  Summary:
    Horizontal Synchronization Event Handler 

  Description:
    This routine is called when Camera OVM7690 sends Horizontal Sync Pulse on
    HSync line. It sets the next line address in the DMA module.

  Precondition:
    The DRV_CAMERA_OVM7690_Initialize routine must have been called for the 
    specified Camera OVM7690 driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

  Parameters:
    object - Driver object handle, returned from the 
             DRV_CAMERA_VOM7690_Initialize routine

  Returns:
    None.

  Example:
    <code>

    DRV_CAMERA_OVM7690_INIT     cameraInit;
    SYS_MODULE_OBJ              objectHandle;

    cameraInit.cameraID                = CAMERA_MODULE_OVM7690;
    cameraInit.sourcePort              = (void *)&PORTK,
    cameraInit.hsyncInterruptSource    = INT_SOURCE_CHANGE_NOTICE_A,
    cameraInit.vsyncInterruptSource    = INT_SOURCE_CHANGE_NOTICE_J,
    cameraInit.dmaChannel              = DRV_CAMERA_OVM7690_DMA_CHANNEL_INDEX,
    cameraInit.dmaTriggerSource        = DMA_TRIGGER_EXTERNAL_2,
    cameraInit.bpp                     = GFX_CONFIG_COLOR_DEPTH,

    objectHandle = DRV_CAMERA_OVM7690_Initialize( DRV_CAMERA_OVM7690_INDEX_0, 
                                                (SYS_MODULE_INIT*)&cameraInit);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }

    handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        //error
        return;    
    }

    void __ISR( HSYNC_ISR_VECTOR) _Ovm7690HSyncHandler(void)
    {
        DRV_CAMERA_OVM7690_HsyncEventHandler(objectHandle);

        SYS_INT_SourceStatusClear(HSYNC_INTERRUPT_SOURCE);
    }
 
    </code>

  Remarks:
    This routine is mandatory.

 */

void DRV_CAMERA_OVM7690_HsyncEventHandler(SYS_MODULE_OBJ object)
{
    DRV_CAMERA_OVM7690_OBJ *dObj = &gDrvCameraOVM7690Obj[object];
    
    if( SYS_PORTS_PinRead( PORTS_ID_0, dObj->hsyncChannel, dObj->hsyncPosition ) 
            == false )
    {
        return;
    }
    
    if((!dObj->inUse) || (dObj->status != SYS_STATUS_READY))
    {
        /* This instance of the driver is not initialized. Don't
         * do anything */
        return;
    }
    
    if(dObj->dmaTransferComplete == false)
    {
        return;
    }
    
    SYS_DMA_ChannelTransferAdd( dObj->dmaHandle, 
                                (void *)KVA_TO_PA(dObj->sourcePort), 
                                1, 
                                (void *)KVA_TO_PA((void *)dObj->frameLineAddress),
                                dObj->frameLineSize, 
                                1);
    
    dObj->dmaTransferComplete = false;
}

// *****************************************************************************
/* Function:
    void DRV_CAMERA_OVM7690_VsyncEventHandler(SYS_MODULE_OBJ object)
 
  Summary:
    Vertical Synchronization Event Handler 

  Description:
    This routine is called when Camera OVM7690 sends Vertical Sync Pulse on
    VSync line. It clears the number of lines drawn variable.

  Precondition:
    The DRV_CAMERA_OVM7690_Initialize routine must have been called for the 
    specified Camera OVM7690 driver instance.

    DRV_CAMERA_OVM7690_Open must have been called to obtain a valid opened 
    device handle.

  Parameters:
    object - Driver object handle, returned from the 
             DRV_CAMERA_OVM7690_Initialize routine

  Returns:
    None.

  Example:
    <code>

    DRV_CAMERA_OVM7690_INIT     cameraInit;
    SYS_MODULE_OBJ              objectHandle;

    cameraInit.cameraID                = CAMERA_MODULE_OVM7690;
    cameraInit.sourcePort              = (void *)&PORTK,
    cameraInit.hsyncInterruptSource    = INT_SOURCE_CHANGE_NOTICE_A,
    cameraInit.vsyncInterruptSource    = INT_SOURCE_CHANGE_NOTICE_J,
    cameraInit.dmaChannel              = DRV_CAMERA_OVM7690_DMA_CHANNEL_INDEX,
    cameraInit.dmaTriggerSource        = DMA_TRIGGER_EXTERNAL_2,
    cameraInit.bpp                     = GFX_CONFIG_COLOR_DEPTH,

    objectHandle = DRV_CAMERA_OVM7690_Initialize( DRV_CAMERA_OVM7690_INDEX_0, 
                                                (SYS_MODULE_INIT*)&cameraInit);
    if (SYS_MODULE_OBJ_INVALID == objectHandle)
    {
        // Handle error
    }

    handle = DRV_CAMERA_OVM7690_Open(DRV_CAMERA_OVM7690_INDEX_0, DRV_IO_INTENT_EXCLUSIVE);
    if (DRV_HANDLE_INVALID == handle)
    {
        //error
        return;    
    }

    void __ISR( VSYNC_ISR_VECTOR) _Ovm7690VSyncHandler(void)
    {
        DRV_CAMERA_OVM7690_VsyncEventHandler(objectHandle);

        SYS_INT_SourceStatusClear(VSYNC_INTERRUPT_SOURCE);
    }
 
    </code>

  Remarks:
    This routine is mandatory.

 */

void DRV_CAMERA_OVM7690_VsyncEventHandler(SYS_MODULE_OBJ object)
{
    DRV_CAMERA_OVM7690_OBJ *dObj = &gDrvCameraOVM7690Obj[object];
    
    if( SYS_PORTS_PinRead( PORTS_ID_0, dObj->vsyncChannel, dObj->vsyncPosition ) 
            == false )
    {
        return;
    }
   
    if((!dObj->inUse) || (dObj->status != SYS_STATUS_READY))
    {
        /* This instance of the driver is not initialized. Don't
         * do anything */
        return;
    }
    
    dObj->frameLineCount = 0;
    dObj->dmaTransferComplete = true;
    
}

// *****************************************************************************
// *****************************************************************************
// Section: File scope functions
// *****************************************************************************
// *****************************************************************************

void _DRV_CAMERA_OVM7690_HardwareSetup( DRV_CAMERA_OVM7690_OBJ * dObj )
{
    if( dObj == NULL )
    {
        /* Driver object is not valid */
        SYS_DEBUG(0, "Invalid Driver Handle");
        return;
    }
    
    SYS_INT_SourceDisable(dObj->hsyncInterruptSource) ;
    SYS_INT_SourceDisable(dObj->vsyncInterruptSource) ;
    
    /* Initialize DRV_CAMERA_OVM7690  */
    dObj->dmaHandle = SYS_DMA_ChannelAllocate( dObj->dmaChannel );
    
    SYS_DMA_ChannelSetup( dObj->dmaHandle, 
                          SYS_DMA_CHANNEL_OP_MODE_BASIC, 
                          dObj->dmaTriggerSource );
    
    SYS_DMA_ChannelTransferEventHandlerSet( dObj->dmaHandle,
                                            _DRV_CAMERA_OVM7690_DMAEventHandler,
                                            (uintptr_t)dObj);
    
    /* Clear Power Down*/
    SYS_PORTS_PinDirectionSelect( PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, 
                                  PORT_CHANNEL_J, PORTS_BIT_POS_7);
    SYS_PORTS_PinClear( PORTS_ID_0, PORT_CHANNEL_J, PORTS_BIT_POS_7 );
    
//    //Turn on Camera Clock source
//    DRV_TMR0_Start();
//    DRV_OC0_Enable();
    
}

void _DRV_CAMERA_OVM7690_DMAEventHandler( SYS_DMA_TRANSFER_EVENT event,
                                          SYS_DMA_CHANNEL_HANDLE handle, 
                                          uintptr_t contextHandle )
{
    DRV_CAMERA_OVM7690_OBJ *dObj = NULL;
    uint32_t byteOffset = 0;
            
    if ( event != SYS_DMA_TRANSFER_EVENT_COMPLETE )
    {
        return;
    }
    
    dObj = (DRV_CAMERA_OVM7690_OBJ *)contextHandle;
    
    dObj->dmaTransferComplete = true;
    dObj->frameLineCount++;
    
    if(dObj->frameLineCount >= (dObj->rect.bottom - dObj->rect.top))
    {
        dObj->frameLineCount = 0;
        return;
    }
    
    byteOffset = dObj->frameLineCount * dObj->frameLineSize;
    dObj->frameLineAddress = (uint8_t *) dObj->frameBufferAddress + byteOffset;
    
    return;
}

/*******************************************************************************
  Function:
    void DRV_CAMERA_OVM7690_Tasks(SYS_MODULE_OBJ object );

  Summary:
    Maintains the OVM7690 state machine.

  Description:
    This routine is used to maintain the OVM7690 internal state machine.
    This function is specifically designed to perform establish services
    required for steady-state operation.

  Remarks:
    This routine is normally not called directly by an application.  It is
    called by the system's Ta
 */

void DRV_CAMERA_OVM7690_Tasks(SYS_MODULE_OBJ object)
{
    
    DRV_CAMERA_OVM7690_OBJ *dObj = &gDrvCameraOVM7690Obj[object];
    
    switch ( dObj->status )
    {
        case SYS_STATUS_READY_EXTENDED:
            //Turn on Camera Clock source
            DRV_TMR0_Start();
            DRV_OC0_Enable();
            dObj->status = SYS_STATUS_READY;
            break;
            
        default:
            break;
    }
}

 void _DRV_CAMERA_OVM7690_delayMS ( unsigned int delayMs )
{
    if(delayMs)
    {
        uint32_t sysClk = SYS_CLK_FREQ;
        uint32_t t0;
        t0 = _CP0_GET_COUNT();
        while (_CP0_GET_COUNT() - t0 < (sysClk/2000)*delayMs);
    }
}

/*******************************************************************************
 End of File
*/
