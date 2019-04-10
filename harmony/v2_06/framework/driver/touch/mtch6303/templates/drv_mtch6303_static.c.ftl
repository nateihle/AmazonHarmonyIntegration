/*******************************************************************************
  MTCH6303 Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_mtch6303_static.c

  Summary:
    MTCH6303 driver impementation for the static single instance driver.

  Description:
    The MTCH6303 device driver provides a simple interface to manage the MTCH6303
    modules. This file contains implemenation for the MTCH6303 driver.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    
    Static single-open interfaces also eliminate the need for the open handle.
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
#include "system_definitions.h"
#include "driver/touch/mtch6303/src/drv_mtch6303_static_local.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data
// *****************************************************************************
// *****************************************************************************

/* This is the driver instance object array. */
DRV_TOUCH_MTCH6303_STATIC_OBJ             gDrvMTCH6303StaticObj;

// *****************************************************************************
// *****************************************************************************
// Section: MTCH6303 Driver Interface Implementations
// *****************************************************************************
// *****************************************************************************

//******************************************************************************
/* Function:
    SYS_MODULE_OBJ DRV_TOUCH_MTCH6303_Initialize( const SYS_MODULE_INDEX   index, 
                                        const SYS_MODULE_INIT    * const init )

  Summary:
    Initializes hardware and data for the given instance of the MTCH6303 module.

  Description:
    This function initializes hardware for the instance of the MTCH6303 module,
    using the hardware initialization given data.  It also initializes all 
    necessary internal data.

  Parameters:

  Returns:
    If successful, it returns a valid handle to a driver instance object.
    Otherwise, it returns SYS_MODULE_OBJ_INVALID.
*/
 SYS_MODULE_OBJ DRV_TOUCH_MTCH6303_Initialize( const SYS_MODULE_INDEX   index, 
                                        const SYS_MODULE_INIT * const init )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ *dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*)NULL;

    /* Allocate the driver object and set the operation flag to be in use */
    dObj = &gDrvMTCH6303StaticObj;

    /* Update driver object */
    dObj->inUse                 = true;
    dObj->queueSizeCurrent      = 0;
    dObj->touchQueueSizeCurrent = 0;
    dObj->queue                 = NULL;
    dObj->touchQueue            = NULL;
    dObj->deviceAddress         = DRV_TOUCH_MTCH6303_I2C_DEVICE_ADDRESS;
<#if CONFIG_DRV_TOUCH_MTCH6303_QUEUE_DEPTH?has_content>
    dObj->queueSize             = ${CONFIG_DRV_TOUCH_MTCH6303_QUEUE_DEPTH};
</#if>
<#if CONFIG_DRV_TOUCH_MTCH6303_MESSAGE_QUEUE_DEPTH?has_content>
    dObj->touchQueueSize        = ${CONFIG_DRV_TOUCH_MTCH6303_MESSAGE_QUEUE_DEPTH};
</#if>
<#if CONFIG_DRV_TOUCH_MTCH6303_INTERRUPT_SOURCE?has_content>
    SYS_INT_SourceStatusClear(${CONFIG_DRV_TOUCH_MTCH6303_INTERRUPT_SOURCE});
</#if>

    /* Update the status */
    dObj->status = SYS_STATUS_READY;

    /* Return the driver handle */
    return( (SYS_MODULE_OBJ)dObj );

}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_MTCH6303_Deinitialize( SYS_MODULE_OBJ object )

  Summary:
    Dynamic implementation of DRV_TOUCH_MTCH6303_Deinitialize system interface function.

  Description:
    This is the dynamic implementation of DRV_TOUCH_MTCH6303_Deinitialize system interface
    function.

  Remarks:
    See drv_mtch6303.h for usage information.
*/

void  DRV_TOUCH_MTCH6303_Deinitialize( SYS_MODULE_OBJ object )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ *dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*)NULL;

    dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*) &gDrvMTCH6303StaticObj;

    if(!dObj->inUse)
    {
        SYS_DEBUG(0, "Invalid system object handle");
        return;
    }

    /* Indicate that this object is not is use */
    dObj->inUse = false;

    /* Deinitialize the MTCH6303 status */
    dObj->status =  SYS_STATUS_UNINITIALIZED ;

    /* Disable the interrupt */
<#if CONFIG_DRV_TOUCH_MTCH6303_INTERRUPT_SOURCE?has_content>
    SYS_INT_SourceDisable(${CONFIG_DRV_TOUCH_MTCH6303_INTERRUPT_SOURCE});
</#if>

}

// *****************************************************************************
/* Function:
    SYS_STATUS DRV_TOUCH_MTCH6303_Status( SYS_MODULE_OBJ object )

  Summary:
    Dynamic implementation of DRV_TOUCH_MTCH6303_Status system interface function.

  Description:
    This is the dynamic implementation of DRV_TOUCH_MTCH6303_Status system interface
    function.

  Remarks:
    See drv_mtch6303.h for usage information.
*/

SYS_STATUS DRV_TOUCH_MTCH6303_Status( SYS_MODULE_OBJ object )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ *dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*)NULL;
    
    dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*) &gDrvMTCH6303StaticObj;
    
    /* Return the system status of the hardware instance object */
    return (dObj->status);
}

// *****************************************************************************
/* Function:
    DRV_HANDLE DRV_TOUCH_MTCH6303_Open( const SYS_MODULE_INDEX drvIndex, 
                                const DRV_IO_INTENT intent )

  Summary:
    Dynamic implementation of DRV_TOUCH_MTCH6303_Open client interface function.

  Description:
    This is the dynamic implementation of DRV_TOUCH_MTCH6303_Open client interface
    function.

  Remarks:
    See drv_mtch6303.h for usage information.
*/

DRV_HANDLE DRV_TOUCH_MTCH6303_Open( const SYS_MODULE_INDEX drvIndex, 
                              const DRV_IO_INTENT intent )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ *dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*)NULL;
    
    dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*) &gDrvMTCH6303StaticObj;

    if((dObj->status != SYS_STATUS_READY) || (dObj->inUse == false))
    {
        /* The MTCH6303 module should be ready */
        SYS_DEBUG(0, "Was the driver initialized?");
        return DRV_HANDLE_INVALID;
    }

<#if CONFIG_DRV_TOUCH_MTCH6303_BUS_SELECT?has_content>
<#if CONFIG_DRV_TOUCH_MTCH6303_BUS_SELECT = "DRV_TOUCH_MTCH6303_BUS_I2C">
    dObj->drvBusHandle = DRV_I2C_Open( ${CONFIG_DRV_TOUCH_MTCH6303_I2C_MODULE_INDEX}, 
                                       DRV_IO_INTENT_READWRITE );
</#if>
</#if>
    if( DRV_HANDLE_INVALID == dObj->drvBusHandle )
    {
        return (DRV_HANDLE_INVALID);
    }

    DRV_I2C_BufferEventHandlerSet( dObj->drvBusHandle,
        (DRV_I2C_BUFFER_EVENT_HANDLER)_DRV_TOUCH_MTCH6303_I2C_EventHandler,
        (uintptr_t)dObj );

    if(!dObj->clientInUse)
    {
        dObj->clientInUse = true;
        
        dObj->touchEventHandler = NULL;
        dObj->context      = (uintptr_t)NULL;				
        dObj->error        = DRV_TOUCH_MTCH6303_ERROR_NONE;
        
        /* Update the client status */
        dObj->clientStatus = DRV_TOUCH_MTCH6303_CLIENT_STATUS_READY;
        return ((DRV_HANDLE)0 );
        
    }

    return (DRV_HANDLE_INVALID);
}

// *****************************************************************************
/* Function:
    DRV_TOUCH_MTCH6303_CLIENT_STATUS DRV_TOUCH_MTCH6303_Close ( void )

  Summary:
    Dynamic implementation of DRV_TOUCH_MTCH6303_Close client interface function.

  Description:
    This is the dynamic implementation of DRV_TOUCH_MTCH6303_Close client interface
    function.

  Remarks:
    See drv_mtch6303.h for usage information.
*/

DRV_TOUCH_MTCH6303_CLIENT_STATUS DRV_TOUCH_MTCH6303_Close ( void )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ *dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*)NULL;
    
    dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*) &gDrvMTCH6303StaticObj;

    /* Remove all buffers that this client owns from the driver queue. This
       function will map to _DRV_TOUCH_MTCH6303_ClientBufferQueueObjectsRemove() if the
       driver was built for buffer queue support. Else this condition always
       maps to true. */
    if(!_DRV_TOUCH_MTCH6303_ClientBufferQueueObjectsRemove( ))
    {
        /* The function could fail if the mutex time out occurred */
        SYS_DEBUG(0, "Could not remove client buffer objects");
        dObj->clientStatus = DRV_TOUCH_MTCH6303_CLIENT_STATUS_ERROR;
        return (DRV_TOUCH_MTCH6303_CLIENT_STATUS_ERROR);
    }

    /* De-allocate the object */
    dObj->clientStatus = DRV_TOUCH_MTCH6303_CLIENT_STATUS_CLOSED;
    dObj->clientInUse = false;

    return (DRV_TOUCH_MTCH6303_CLIENT_STATUS_CLOSED);
}

// *****************************************************************************
/* Function:
    DRV_TOUCH_MTCH6303_ERROR DRV_TOUCH_MTCH6303_ErrorGet( void )

  Summary:
    Dynamic implementation of DRV_TOUCH_MTCH6303_ErrorGet client interface function.

  Description:
    This is the dynamic implementation of DRV_TOUCH_MTCH6303_ErrorGet client interface
    function.

  Remarks:
    See drv_mtch6303.h for usage information.
*/

DRV_TOUCH_MTCH6303_ERROR DRV_TOUCH_MTCH6303_ErrorGet( void )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ *dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*)NULL;
    DRV_TOUCH_MTCH6303_ERROR error;

    dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*) &gDrvMTCH6303StaticObj;
    
    /* Return the error. Clear the error before
       returning. */
    error = dObj->error;
    dObj->error = DRV_TOUCH_MTCH6303_ERROR_NONE;
    return(error);
}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_MTCH6303_Tasks ( SYS_MODULE_OBJ object )

  Summary:
    Dynamic implementation of DRV_TOUCH_MTCH6303_Tasks system interface function.

  Description:
    This is the dynamic implementation of DRV_TOUCH_MTCH6303_Tasks system interface
    function.

  Remarks:
    See drv_mtch6303.h for usage information.
*/
void DRV_TOUCH_MTCH6303_Tasks( SYS_MODULE_OBJ object )
{
    /* This is the MTCH6303 Driver Write tasks routine.
       In this function, the driver checks if a transmit
       interrupt is active and if there are any buffers in
       queue. If so the buffer is serviced. A buffer that
       is serviced completely is removed from the queue.
     */

    DRV_TOUCH_MTCH6303_STATIC_OBJ * hDriver = &gDrvMTCH6303StaticObj;

    if((!hDriver->inUse) || (hDriver->status != SYS_STATUS_READY))
    {
        /* This instance of the driver is not initialized. Don't
         * do anything */
        return;
    }

    _DRV_TOUCH_MTCH6303_BufferQueueTasks(hDriver);

}

/*********************************************************************
  Function:
    DRV_TOUCH_POSITION_SINGLE DRV_TOUCH_MTCH6303_TouchStatus( )

  Summary:
    Returns the status of the current touch input.

*/
DRV_TOUCH_POSITION_STATUS DRV_TOUCH_MTCH6303_TouchStatus( const SYS_MODULE_INDEX index )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ *dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*) &gDrvMTCH6303StaticObj;
    return (dObj->touchStatus);
}


/*********************************************************************
  Function:
    void DRV_TOUCH_MTCH6303_TouchDataRead( )

  Summary:
    Notify the driver that the current touch data has been read

*/
void DRV_TOUCH_MTCH6303_TouchDataRead( const SYS_MODULE_INDEX index )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ *dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*) &gDrvMTCH6303StaticObj;
    dObj->touchStatus = DRV_TOUCH_POSITION_NONE;
}


/*********************************************************************
  Function:
    short DRV_TOUCH_MTCH6303_TouchGetX( uint8 touchNumber )

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

short DRV_TOUCH_MTCH6303_TouchGetX( uint8_t touchNumber )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ *dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*) &gDrvMTCH6303StaticObj;
    return dObj->mostRecentTouchX;
}

/*********************************************************************
  Function:
    short DRV_TOUCH_MTCH6303_TouchGetY( uint8 touchNumber )

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

short DRV_TOUCH_MTCH6303_TouchGetY( uint8_t touchNumber )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ *dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*) &gDrvMTCH6303StaticObj;
    return dObj->mostRecentTouchY;
}

// *****************************************************************************
// *****************************************************************************
// Section: File scope functions
// *****************************************************************************
// *****************************************************************************

bool _DRV_TOUCH_MTCH6303_ClientBufferQueueObjectsRemove( void )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ * dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ *) NULL;
    bool interruptWasEnabled = false;
    DRV_TOUCH_MTCH6303_BUFFER_OBJ * iterator = NULL;

    dObj = &gDrvMTCH6303StaticObj;

<#if CONFIG_DRV_TOUCH_MTCH6303_INTERRUPT_SOURCE?has_content>
    /* Disable the transmit interrupt */
    interruptWasEnabled = SYS_INT_SourceDisable(${CONFIG_DRV_TOUCH_MTCH6303_INTERRUPT_SOURCE});
</#if>
    
    iterator = dObj->queue;
    while(iterator != NULL)
    {
        iterator->inUse = false;
        if(iterator->previous != NULL)
        {
            iterator->previous->next = iterator->next;
        }
        if(iterator->next != NULL)
        {
            iterator->next->previous = iterator->previous;
        }
        /* Decrementing Current queue size */
        dObj->queueSizeCurrent --;

        iterator = iterator->next;
    }

    /* If there are no buffers in the write queue.
     * Make the head pointer point to NULL */
    if(dObj->queueSizeCurrent == 0)
    {
        dObj->queue = NULL;
    }
    else
    {
        /* Iterate to update the head pointer to point
         * the first valid buffer object in the queue */
        iterator = dObj->queue;
        while(iterator != NULL)
        {
            if(iterator->inUse == true)
            {
                dObj->queue = iterator;
                break;
            }
            iterator = iterator->next;
        }
    }

    /* Re-enable the interrupt if it was enabled */
    if(interruptWasEnabled)
    {
<#if CONFIG_DRV_TOUCH_MTCH6303_INTERRUPT_SOURCE?has_content>
        SYS_INT_SourceEnable(${CONFIG_DRV_TOUCH_MTCH6303_INTERRUPT_SOURCE});
</#if>
    }

    return true;

}

void _DRV_TOUCH_MTCH6303_BufferQueueTasks(DRV_TOUCH_MTCH6303_STATIC_OBJ * hDriver)
{
    DRV_TOUCH_MTCH6303_BUFFER_OBJ * bufferObj = ( DRV_TOUCH_MTCH6303_BUFFER_OBJ * ) NULL;

    bufferObj = hDriver->queue;

    if( DRV_HANDLE_INVALID != hDriver->drvBusHandle &&
        NULL               != bufferObj )
    {
        if( 0 == bufferObj->size )
        {
            hDriver->queue   = bufferObj->next;
            bufferObj->inUse = false;
            hDriver->queueSizeCurrent--;

            if( hDriver->touchData.status.nTouch  != 0 &&
                hDriver->touchData.touch[0].nibble_0.inRange != 0 &&
                hDriver->touchData.touch[0].x != 0 &&
                hDriver->touchData.touch[0].y != 0 )
            {
                //Translate the touch data to X/Y coordinates
                hDriver->mostRecentTouchX = (int16_t)DRV_TOUCH_MTCH6303_TouchInputMap(hDriver->touchData.touch[0].x, ${CONFIG_DRV_GFX_DISPLAY_WIDTH});
                hDriver->mostRecentTouchY = (int16_t)DRV_TOUCH_MTCH6303_TouchInputMap(hDriver->touchData.touch[0].y, ${CONFIG_DRV_GFX_DISPLAY_HEIGHT});
            }
            else
            {
                hDriver->mostRecentTouchX = -1; 
                hDriver->mostRecentTouchY = -1; 
            }

            hDriver->touchStatus = DRV_TOUCH_POSITION_SINGLE;

            if( NULL != hDriver->queue )
            {
                bufferObj = hDriver->queue;

                switch( bufferObj->flags )
                {
                    case DRV_TOUCH_MTCH6303_BUFFER_OBJ_FLAG_REG_READ:
                    {
                        bufferObj->hBusBuffer = 
                                DRV_I2C_BufferAddWriteRead( hDriver->drvBusHandle,
                                                            (uint8_t *)&hDriver->deviceAddress,
                                                            ( void *)&bufferObj->regAddress,
                                                            1, 
                                                            ( void * )bufferObj->readBuffer,
                                                            bufferObj->size,
                                                            (void *)NULL );

                        break;
                    }

                    case DRV_TOUCH_MTCH6303_BUFFER_OBJ_FLAG_REG_WRITE:
                    {
                        bufferObj->hBusBuffer = 
                                    DRV_I2C_BufferAddWrite( hDriver->drvBusHandle,
                                                            (uint8_t *)&hDriver->deviceAddress, 
                                                            (uint8_t *)bufferObj->writeBuffer, 
                                                            bufferObj->size, 
                                                            (void *) NULL);
                        break;
                    }

                }        
            }
        }
    }

    return;
}

/*******************************************************************************
 End of File
*/