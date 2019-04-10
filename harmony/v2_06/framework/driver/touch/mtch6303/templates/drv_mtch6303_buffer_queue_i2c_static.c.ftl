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
extern DRV_TOUCH_MTCH6303_STATIC_OBJ             gDrvMTCH6303StaticObj;

/**************************************************************
 * This is the array of MTCH6303 Driver Buffet object.
 **************************************************************/
DRV_TOUCH_MTCH6303_BUFFER_OBJ gDrvMTCH6303BufferObj[${CONFIG_DRV_TOUCH_MTCH6303_QUEUE_DEPTH} ];

// *****************************************************************************
// *****************************************************************************
// Section: MTCH6303 Driver Interface Implementations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: MTCH6303 Driver Buffer Queue Interface Implementations
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/*

  Summary:


  Description:


  Remarks:


*/
inline uint16_t DRV_TOUCH_MTCH6303_TouchInputMap( uint16_t touchValue, uint16_t dispResolution )
{
    return ( ( touchValue * dispResolution ) / DRV_TOUCH_MTCH6303_MAX_POSITION_OUTPUT );
}

// *****************************************************************************
/*

  Summary:


  Description:


  Remarks:


*/
void DRV_TOUCH_MTCH6303_TouchInputRead( SYS_MODULE_OBJ object )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ* dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*)object;

    DRV_TOUCH_MTCH6303_AddRegisterRead( &dObj->bufferHandle,
                                  DRV_TOUCH_MTCH6303_REG_TOUCH_STATUS,
                                  DRV_TOUCH_MTCH6303_REG_TOUCH_MAX,
                                  (uint8_t *)&dObj->touchData );
}

// *****************************************************************************
/*

  Summary:


  Description:


  Remarks:


*/

void DRV_TOUCH_MTCH6303_AddRegisterRead( DRV_TOUCH_MTCH6303_BUFFER_HANDLE * bufferHandle,
                                   uint8_t source,
                                   size_t  nBytes,
                                   uint8_t * destination )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ * hDriver = (DRV_TOUCH_MTCH6303_STATIC_OBJ *) NULL;
    DRV_TOUCH_MTCH6303_BUFFER_OBJ * bufferObj, * iterator;
    unsigned int i;

    hDriver = &gDrvMTCH6303StaticObj;

    if((nBytes == 0) ||
       (NULL == destination) || (bufferHandle == NULL))
    {
        /* We either got an invalid client handle,
           invalid destination pointer or 0 bytes to
           transfer */

        SYS_ASSERT(false, "Invalid parameters");
        return;
    }

    *bufferHandle = DRV_TOUCH_MTCH6303_BUFFER_HANDLE_INVALID;

    if(hDriver->queueSizeCurrent >= hDriver->queueSize)
    {
        /* This means the queue is full. We cannot add this request */
        SYS_DEBUG_PRINT(SYS_ERROR_WARNING, "In function DRV_TOUCH_MTCH6303_AddRegisterRead @ line %d, Queue is full!!\r\n",__LINE__);

        return;
    }

    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i <${CONFIG_DRV_TOUCH_MTCH6303_QUEUE_DEPTH}; i ++)
    {
        if(!gDrvMTCH6303BufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj                  = &gDrvMTCH6303BufferObj[i];
            bufferObj->size            = nBytes;
            bufferObj->inUse           = true;
            bufferObj->regAddress      = source;
            bufferObj->readBuffer      = destination;
            bufferObj->next            = NULL;
            bufferObj->previous        = NULL;
            bufferObj->nCurrentBytes   = 0;
            bufferObj->flags           = (0 | DRV_TOUCH_MTCH6303_BUFFER_OBJ_FLAG_REG_READ );
            bufferObj->hBusBuffer      = 0;

            /* Assign a handle to this buffer */
            *bufferHandle = (DRV_TOUCH_MTCH6303_BUFFER_HANDLE)bufferObj;
            break;
        }
    }

    if(i ==${CONFIG_DRV_TOUCH_MTCH6303_QUEUE_DEPTH})
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_PMP_QUEUE_DEPTH_COMBINED
           parameter is configured to be less */

        SYS_ASSERT(false, "Insufficient Combined Queue Depth");

<#if CONFIG_DRV_TOUCH_MTCH6303_INTERRUPT_SOURCE?has_content>
        SYS_INT_SourceDisable(${CONFIG_DRV_TOUCH_MTCH6303_INTERRUPT_SOURCE});
</#if>

        return;
    }

    /* Check if the queue is empty */
    if(hDriver->queue == NULL)
    {

<#if CONFIG_DRV_TOUCH_MTCH6303_BUS_SELECT?has_content>
<#if CONFIG_DRV_TOUCH_MTCH6303_BUS_SELECT = "DRV_TOUCH_MTCH6303_BUS_I2C">
        bufferObj->hBusBuffer = DRV_I2C_BufferAddWriteRead( hDriver->drvBusHandle,
                                                            (uint8_t *)&hDriver->deviceAddress,
                                                            ( void *)&bufferObj->regAddress,
                                                            1,
                                                            ( void * )bufferObj->readBuffer,
                                                            bufferObj->size,
                                                            (void *)NULL );
</#if>
</#if>

        if( 0 == bufferObj->hBusBuffer )
        {
            bufferObj->inUse = false;
            *bufferHandle = DRV_TOUCH_MTCH6303_BUFFER_HANDLE_INVALID;
            return;
        }

        /* This is the first buffer in the
           queue */
        hDriver->queue = bufferObj;

    }
    else
    {
        /* This means the read queue is not empty. We must add
           the buffer object to the end of the queue */
        iterator = hDriver->queue;
        while(iterator->next != NULL)
        {
            /* Get the next buffer object */
            iterator = iterator->next;
        }

        /* At this point, iterator will point to the
           last object in the queue. We add the buffer
           object to the linked list. Note that we
           need to set up the previous pointer as well
           because buffer should be deleted when the
           client closes the driver */

        iterator->next = bufferObj;
        bufferObj->previous = iterator;

    }

    /* Increment the current queue size*/
    hDriver->queueSizeCurrent ++;

}

// *****************************************************************************
/*

  Summary:


  Description:


  Remarks:


*/

void DRV_TOUCH_MTCH6303_AddRegisterWrite( DRV_TOUCH_MTCH6303_BUFFER_HANDLE * bufferHandle,
                                    uint8_t destination,
                                    size_t  nBytes,
                                    uint8_t * source )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ * hDriver = (DRV_TOUCH_MTCH6303_STATIC_OBJ *) NULL;
    DRV_TOUCH_MTCH6303_BUFFER_OBJ * bufferObj, * iterator;
    unsigned int i, j;

    hDriver = &gDrvMTCH6303StaticObj;

    if((nBytes == 0) ||
       /* work around to add register address in transfer */
       /***************************************************/
       ( nBytes > 64 ) ||
       /***************************************************/
       (NULL == source) || (bufferHandle == NULL))
    {
        /* We either got an invalid client handle,
           invalid destination pointer or 0 bytes to
           transfer */

        SYS_ASSERT(false, "Invalid parameters");
        return;
    }

    *bufferHandle = DRV_TOUCH_MTCH6303_BUFFER_HANDLE_INVALID;

    if(hDriver->queueSizeCurrent >= hDriver->queueSize)
    {
        /* This means the queue is full. We cannot add
         * this request */

        SYS_ASSERT(false, "Queue is full");
        return;
    }

    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i <${CONFIG_DRV_TOUCH_MTCH6303_QUEUE_DEPTH}; i ++)
    {
        if(!gDrvMTCH6303BufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj                  = &gDrvMTCH6303BufferObj[i];
            bufferObj->inUse           = true;
            bufferObj->regAddress      = destination;
            bufferObj->next            = NULL;
            bufferObj->previous        = NULL;
            bufferObj->nCurrentBytes   = 0;
            bufferObj->flags           = (0 | DRV_TOUCH_MTCH6303_BUFFER_OBJ_FLAG_REG_WRITE );
            bufferObj->hBusBuffer      = 0;

            /* work around to add register address in transfer */
            /***************************************************/
            bufferObj->size           = nBytes + 1;
            bufferObj->writeBuffer[0] = destination;
            for( j = 0; j < nBytes; j++ )
            {
                bufferObj->writeBuffer[j + 1] = source[j];
            }
            /***************************************************/

            /* Assign a handle to this buffer */
            *bufferHandle = (DRV_TOUCH_MTCH6303_BUFFER_HANDLE)bufferObj;
            break;
        }
    }

    if(i ==${CONFIG_DRV_TOUCH_MTCH6303_QUEUE_DEPTH})
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_PMP_QUEUE_DEPTH_COMBINED
           parameter is configured to be less */

        SYS_ASSERT(false, "Insufficient Combined Queue Depth");

        return;
    }

    /* Check if the queue is empty */
    if(hDriver->queue == NULL)
    {
<#if CONFIG_DRV_TOUCH_MTCH6303_BUS_SELECT?has_content>
<#if CONFIG_DRV_TOUCH_MTCH6303_BUS_SELECT = "DRV_TOUCH_MTCH6303_BUS_I2C">

        bufferObj->hBusBuffer = DRV_I2C_BufferAddWrite( hDriver->drvBusHandle,
                                                        (uint8_t *)&hDriver->deviceAddress,
                                                        (uint8_t *)bufferObj->writeBuffer,
                                                        bufferObj->size,
                                                        (void *) NULL);
</#if>
</#if>
        if( 0 == bufferObj->hBusBuffer )
        {
            bufferObj->inUse = false;
            *bufferHandle = DRV_TOUCH_MTCH6303_BUFFER_HANDLE_INVALID;
            return;
        }

        /* This is the first buffer in the
           queue */
        hDriver->queue = bufferObj;

    }
    else
    {
        /* This means the read queue is not empty. We must add
           the buffer object to the end of the queue */
        iterator = hDriver->queue;
        while(iterator->next != NULL)
        {
            /* Get the next buffer object */
            iterator = iterator->next;
        }

        /* At this point, iterator will point to the
           last object in the queue. We add the buffer
           object to the linked list. Note that we
           need to set up the previous pointer as well
           because buffer should be deleted when the
           client closes the driver */

        iterator->next = bufferObj;
        bufferObj->previous = iterator;

    }

    /* Increment the current queue size*/
    hDriver->queueSizeCurrent ++;

}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_MTCH6303_TouchEventHandlerSet
    (
        const DRV_TOUCH_MTCH6303_TOUCH_EVENT_HANDLER eventHandler,
        const uintptr_t context
    )

  Summary:
    Dynamic implementation of DRV_TOUCH_MTCH6303_TouchEventHandlerSet client interface
    function.

  Description:
    This is the dynamic implementation of DRV_TOUCH_MTCH6303_TouchEventHandlerSet
    client interface function.

  Remarks:
    See drv_pmp_dma.h for usage information.
*/

void DRV_TOUCH_MTCH6303_TOUCH_TouchEventHandlerSet
(
    const DRV_TOUCH_MTCH6303_TOUCH_EVENT_HANDLER eventHandler,
    const uintptr_t context
)
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ *dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*)NULL;
    dObj                     = &gDrvMTCH6303StaticObj;

    /* Register the event handler with the client */
    dObj->touchEventHandler = eventHandler;
    dObj->context      = context;

    /* Setup the Event handler for the Bus interrupts */
    if(DRV_HANDLE_INVALID != dObj->drvBusHandle )
    {
<#if CONFIG_DRV_TOUCH_MTCH6303_BUS_SELECT?has_content>
<#if CONFIG_DRV_TOUCH_MTCH6303_BUS_SELECT = "DRV_TOUCH_MTCH6303_BUS_I2C">
        DRV_I2C_BufferEventHandlerSet( dObj->drvBusHandle,
            (DRV_I2C_BUFFER_EVENT_HANDLER)_DRV_TOUCH_MTCH6303_I2C_EventHandler,
            (uintptr_t)dObj );
</#if>
</#if>
    }

}

// *****************************************************************************
// *****************************************************************************
// Section: File scope functions
// *****************************************************************************
// *****************************************************************************

<#if CONFIG_DRV_TOUCH_MTCH6303_BUS_SELECT?has_content>
<#if CONFIG_DRV_TOUCH_MTCH6303_BUS_SELECT = "DRV_TOUCH_MTCH6303_BUS_I2C">

/*******************************************************************************
  Function:
    void _DRV_TOUCH_MTCH6303_I2C_EventHandler( DRV_I2C_BUFFER_EVENT event,
                                         DRV_I2C_BUFFER_HANDLE  bufferHandle,
                                         uintptr_t contextHandle )

  Summary:

  Description:

  Remarks:
    None
 */
void _DRV_TOUCH_MTCH6303_I2C_EventHandler( DRV_I2C_BUFFER_EVENT event,
                                     DRV_I2C_BUFFER_HANDLE  bufferHandle,
                                     uintptr_t contextHandle )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ * dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ *) NULL;
    DRV_TOUCH_MTCH6303_BUFFER_OBJ * bufObject = (DRV_TOUCH_MTCH6303_BUFFER_OBJ *) NULL;

    if( 0 == bufferHandle )
    {
        /* This means the handle is invalid */
        SYS_DEBUG(SYS_ERROR_DEBUG, "Handle is invalid \r\n");
        return;
    }

    dObj      = ( DRV_TOUCH_MTCH6303_STATIC_OBJ * ) &gDrvMTCH6303StaticObj;
    bufObject = dObj->queue;

    if( bufObject == NULL )
    {
        return;
    }

    switch( event )
    {
        case DRV_I2C_SEND_STOP_EVENT:
        {
            break;
        }

        case DRV_I2C_SEND_RESTART_EVENT:
        {
            break;
        }

        case DRV_I2C_BUFFER_EVENT_COMPLETE:
        {
            bufObject->nCurrentBytes = bufObject->size;
            bufObject->size          = 0;
            dObj->event              = DRV_TOUCH_MTCH6303_BUFFER_EVENT_COMPLETE;

            break;
        }

        default:
        {
            break;
        }

    }

    return;
}

</#if>
</#if>

/*******************************************************************************
 End of File
*/
