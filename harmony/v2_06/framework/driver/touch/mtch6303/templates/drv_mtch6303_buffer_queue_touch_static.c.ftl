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
DRV_TOUCH_MTCH6303_TOUCH_BUFFER_OBJ gDrvMTCH6303TouchBufferObj[ ${CONFIG_DRV_TOUCH_MTCH6303_MESSAGE_QUEUE_DEPTH} ];

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

void DRV_TOUCH_MTCH6303_TOUCH_AddTouchInputRead
( 
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE * bufferHandle,
    DRV_TOUCH_MTCH6303_TOUCH_DATA          * touchData
)
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ * hDriver = (DRV_TOUCH_MTCH6303_STATIC_OBJ *) NULL;
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_OBJ * bufferObj, * iterator;
    unsigned int i;

    if( NULL == bufferHandle || NULL == touchData )
    {
        SYS_ASSERT(false, "Invalid parameters");
        return;
    }
    
    hDriver = &gDrvMTCH6303StaticObj;

    *bufferHandle = DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE_INVALID;

    if(hDriver->touchQueueSizeCurrent >= hDriver->touchQueueSize)
    {
        /* This means the queue is full. We cannot add
         * this request */

        SYS_ASSERT(false, "Queue is full");
        return;
    }

    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i < ${CONFIG_DRV_TOUCH_MTCH6303_MESSAGE_QUEUE_DEPTH}; i ++)
    {
        if(!gDrvMTCH6303TouchBufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj                  = &gDrvMTCH6303TouchBufferObj[i];
            bufferObj->inUse           = true;
            bufferObj->size            = 0xFFFFFFFF; 
            bufferObj->touchData       = touchData;
            bufferObj->next            = NULL;
            bufferObj->previous        = NULL;
            bufferObj->flags           = (0 | DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_STATUS_READ );
            bufferObj->hRegBuffer      = NULL;

            /* Assign a handle to this buffer */
            *bufferHandle = (DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE)bufferObj;
            break;
        }
    }
    
    if(i == ${CONFIG_DRV_TOUCH_MTCH6303_MESSAGE_QUEUE_DEPTH})
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_PMP_QUEUE_DEPTH_COMBINED
           parameter is configured to be less */

        SYS_ASSERT(false, "Insufficient Combined Queue Depth");

        return;
    }

    /* Check if the queue is empty */
    if(hDriver->touchQueue == NULL)
    {
        DRV_TOUCH_MTCH6303_AddRegisterRead( &bufferObj->hRegBuffer,
                                      DRV_TOUCH_MTCH6303_REG_TOUCH_STATUS,
                                      1, 
                                      (uint8_t *)&bufferObj->touchData->status );

        if( DRV_TOUCH_MTCH6303_BUFFER_HANDLE_INVALID == bufferObj->hRegBuffer )
        {
            bufferObj->inUse = false;
            *bufferHandle = DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE_INVALID;
            return;
        }

        /* This is the first buffer in the
           queue */
        hDriver->touchQueue = bufferObj;
    }
    else
    {
        /* This means the read queue is not empty. We must add
           the buffer object to the end of the queue */
        iterator = hDriver->touchQueue;
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
    hDriver->touchQueueSizeCurrent ++;
}

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

void DRV_TOUCH_MTCH6303_TOUCH_AddMessageReportRead
( 
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE * bufferHandle,
    DRV_TOUCH_MTCH6303_TOUCH_MESSAGE       * messageRep,
    size_t                             messageSize
)
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ * hDriver = (DRV_TOUCH_MTCH6303_STATIC_OBJ *) NULL;
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_OBJ * bufferObj, * iterator;
    unsigned int i;
    
    hDriver = &gDrvMTCH6303StaticObj;

    *bufferHandle = DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE_INVALID;

    if(hDriver->touchQueueSizeCurrent >= hDriver->touchQueueSize)
    {
        /* This means the queue is full. We cannot add
         * this request */

        SYS_ASSERT(false, "Queue is full");
        return;
    }

    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i < ${CONFIG_DRV_TOUCH_MTCH6303_MESSAGE_QUEUE_DEPTH}; i ++)
    {
        if(!gDrvMTCH6303TouchBufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj                = &gDrvMTCH6303TouchBufferObj[i];
            bufferObj->inUse         = true;
            bufferObj->size          = messageSize;
            bufferObj->nCurrentBytes = 0;
            bufferObj->numHWBytes    = 0;
            bufferObj->next          = NULL;
            bufferObj->previous      = NULL;
            bufferObj->flags         = (0 | DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_TXRDY_READ );
            bufferObj->hRegBuffer    = NULL;
            bufferObj->message       = messageRep;

            /* Assign a handle to this buffer */
            *bufferHandle = (DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE)bufferObj;
            break;
        }
    }
    
    if(i == ${CONFIG_DRV_TOUCH_MTCH6303_MESSAGE_QUEUE_DEPTH})
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_PMP_QUEUE_DEPTH_COMBINED
           parameter is configured to be less */

        SYS_ASSERT(false, "Insufficient Combined Queue Depth");

        return;
    }

    /* Check if the queue is empty */
    if(hDriver->touchQueue == NULL)
    {
        DRV_TOUCH_MTCH6303_AddRegisterRead( &bufferObj->hRegBuffer,
                                      DRV_TOUCH_MTCH6303_REG_TX_BYTES_READY,
                                      1, 
                                      (uint8_t *)&bufferObj->numHWBytes );

        if( DRV_TOUCH_MTCH6303_BUFFER_HANDLE_INVALID == bufferObj->hRegBuffer )
        {
            bufferObj->inUse = false;
            *bufferHandle = DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE_INVALID;
            return;
        }

        /* This is the first buffer in the
           queue */
        hDriver->touchQueue = bufferObj;
    }
    else
    {
        /* This means the read queue is not empty. We must add
           the buffer object to the end of the queue */
        iterator = hDriver->touchQueue;
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
    hDriver->touchQueueSizeCurrent ++;

}

// *****************************************************************************
/* 

  Summary:
    

  Description:
    

  Remarks:
    

*/

void DRV_TOUCH_MTCH6303_TOUCH_AddMessageCommandWrite
( 
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE   * bufferHandle,
    DRV_TOUCH_MTCH6303_TOUCH_MESSAGE         * messageCmd,
    size_t                               messageSize
)
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ * hDriver = (DRV_TOUCH_MTCH6303_STATIC_OBJ *) NULL;
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_OBJ * bufferObj, * iterator;
    unsigned int i;
    
    hDriver = &gDrvMTCH6303StaticObj;

    *bufferHandle = DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE_INVALID;

    if( NULL == messageCmd || 0 == messageSize )
    {
        return;
    }

    if(hDriver->touchQueueSizeCurrent >= hDriver->touchQueueSize)
    {
        /* This means the queue is full. We cannot add
         * this request */

        SYS_ASSERT(false, "Queue is full");
        return;
    }

    /* Search the buffer pool for a free buffer object */
    for(i = 0 ; i < ${CONFIG_DRV_TOUCH_MTCH6303_MESSAGE_QUEUE_DEPTH}; i ++)
    {
        if(!gDrvMTCH6303TouchBufferObj[i].inUse)
        {
            /* This means this object is free.
             * Configure the object and then
             * break */
            bufferObj                = &gDrvMTCH6303TouchBufferObj[i];
            bufferObj->inUse         = true;
            bufferObj->size          = messageSize;
            bufferObj->nCurrentBytes = 0;
            bufferObj->numHWBytes    = 0;
            bufferObj->next          = NULL;
            bufferObj->previous      = NULL;
            bufferObj->flags         = (0 | DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_RXRDY_READ );
            bufferObj->hRegBuffer    = NULL;
            bufferObj->message       = messageCmd;

            /* Assign a handle to this buffer */
            *bufferHandle = (DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE)bufferObj;
            break;
        }
    }
    
    if(i == ${CONFIG_DRV_TOUCH_MTCH6303_MESSAGE_QUEUE_DEPTH})
    {
        /* This means we could not find a buffer. This
           will happen if the the DRV_PMP_QUEUE_DEPTH_COMBINED
           parameter is configured to be less */

        SYS_ASSERT(false, "Insufficient Combined Queue Depth");

        return;
    }

    /* Check if the queue is empty */
    if(hDriver->touchQueue == NULL)
    {
        DRV_TOUCH_MTCH6303_AddRegisterRead( &bufferObj->hRegBuffer,
                                      DRV_TOUCH_MTCH6303_REG_RX_BYTES_READY,
                                      1, 
                                      (uint8_t *)&bufferObj->numHWBytes );

        if( DRV_TOUCH_MTCH6303_BUFFER_HANDLE_INVALID == bufferObj->hRegBuffer )
        {
            bufferObj->inUse = false;
            *bufferHandle = DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE_INVALID;
            return;
        }

        /* This is the first buffer in the
           queue */
        hDriver->touchQueue = bufferObj;
    }
    else
    {
        /* This means the read queue is not empty. We must add
           the buffer object to the end of the queue */
        iterator = hDriver->touchQueue;
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
    hDriver->touchQueueSizeCurrent ++;
}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_MTCH6303_TOUCH_BufferEventHandlerSet
    (
        const DRV_TOUCH_MTCH6303_BUFFER_EVENT_HANDLER eventHandler,
        const uintptr_t context
    )

  Summary:
    Dynamic implementation of DRV_TOUCH_MTCH6303_BufferEventHandlerSet client interface
    function.

  Description:
    This is the dynamic implementation of DRV_TOUCH_MTCH6303_BufferEventHandlerSet
    client interface function.

  Remarks:
    See drv_pmp_dma.h for usage information.
*/

void DRV_TOUCH_MTCH6303_TOUCH_BufferEventHandlerSet
(
    const DRV_TOUCH_MTCH6303_TOUCH_BUFFER_EVENT_HANDLER eventHandler,
    const uintptr_t context
)
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ *dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ*)NULL;
    dObj                     = &gDrvMTCH6303StaticObj;
    
    /* Register the event handler with the client */
    dObj->touchEventHandler = eventHandler;
    dObj->touchContext      = context;

    DRV_TOUCH_MTCH6303_BufferEventHandlerSet ( 
                      (DRV_TOUCH_MTCH6303_BUFFER_EVENT_HANDLER) _DRV_TOUCH_MTCH6303_Buffer_EventHandler,
                      (uintptr_t) dObj );
   
}

// *****************************************************************************
/* Function:
    void DRV_TOUCH_MTCH6303_TOUCH_Tasks ( void )

  Summary:
    Dynamic implementation of DRV_TOUCH_MTCH6303_TouchTasks system interface function.

  Description:
    This is the dynamic implementation of DRV_TOUCH_MTCH6303_TouchTasks system interface
    function.

  Remarks:
    See drv_mtch6303.h for usage information.
*/
void DRV_TOUCH_MTCH6303_TOUCH_Tasks( void )
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

    _DRV_TOUCH_MTCH6303_TouchBufferQueueTasks(hDriver);

}

// *****************************************************************************
// *****************************************************************************
// Section: File scope functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void _DRV_TOUCH_MTCH6303_Buffer_EventHandler( DRV_TOUCH_MTCH6303_BUFFER_EVENT event,
                                            DRV_TOUCH_MTCH6303_BUFFER_HANDLE  bufferHandle, 
                                            uintptr_t contextHandle )

  Summary:

  Description:

  Remarks:
    None
 */
void _DRV_TOUCH_MTCH6303_Buffer_EventHandler( DRV_TOUCH_MTCH6303_BUFFER_EVENT  event,
                                        DRV_TOUCH_MTCH6303_BUFFER_HANDLE bufferHandle, 
                                        uintptr_t contextHandle )
{
    DRV_TOUCH_MTCH6303_STATIC_OBJ * dObj = (DRV_TOUCH_MTCH6303_STATIC_OBJ *) NULL;
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_OBJ * bufObject = 
                             (DRV_TOUCH_MTCH6303_TOUCH_BUFFER_OBJ *) NULL;
    
    if( 0 == bufferHandle )
    {
        /* This means the handle is invalid */
        SYS_DEBUG(SYS_ERROR_DEBUG, "Handle is invalid \r\n");
        return;
    }

    dObj      = ( DRV_TOUCH_MTCH6303_STATIC_OBJ * ) &gDrvMTCH6303StaticObj;
    bufObject = dObj->touchQueue;

    if( bufObject->hRegBuffer != bufferHandle )
    {
       return;
    }

    if ( DRV_TOUCH_MTCH6303_BUFFER_EVENT_COMPLETE == event )
    {
        switch( bufObject->flags )
        { 
            case DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_STATUS_READ:
            {
                bufObject->size = 6 * bufObject->touchData->status.nTouch;

                if( 0 != bufObject->size )
                {
                    bufObject->flags = DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_INPUT_READ;
                }
                else
                {
                    bufObject->nCurrentBytes = bufObject->size;
                    bufObject->size          = 0;
                    dObj->event              = DRV_TOUCH_MTCH6303_TOUCH_BUFFER_EVENT_COMPLETE;
                }

                break;
            }

            case DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_INPUT_READ:
            {
                bufObject->nCurrentBytes = bufObject->size;
                bufObject->size          = 0;
                dObj->event              = DRV_TOUCH_MTCH6303_TOUCH_BUFFER_EVENT_COMPLETE;
                break;
            }

            case DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_TXRDY_READ:
            {
                if( bufObject->numHWBytes != 0 )
                {
                    if( ( bufObject->size - bufObject->nCurrentBytes ) <= 
                          bufObject->numHWBytes )
                    {
                        bufObject->numHWBytes = bufObject->size - 
                                                bufObject->nCurrentBytes; 
                    }

                    bufObject->flags = DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_READ;
                }

                break;
            }

            case DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_READ:
            {
                bufObject->nCurrentBytes += bufObject->numHWBytes;

                if( bufObject->nCurrentBytes == bufObject->size )
                {
                    bufObject->size = 0;
                    dObj->event     = DRV_TOUCH_MTCH6303_TOUCH_BUFFER_EVENT_COMPLETE;
                }
                else
                {
                    bufObject->flags = DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_TXRDY_READ;
                }

                break; 
            }

            case DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_RXRDY_READ:
            {
                if( bufObject->numHWBytes != 0 )
                {
                    if( ( bufObject->size - bufObject->nCurrentBytes ) <= 
                        bufObject->numHWBytes )
                    {
                        bufObject->numHWBytes = bufObject->size - 
                                                bufObject->nCurrentBytes; 
                    }

                    bufObject->flags = DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_WRITE;
                }

                break;
            }

            case DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_WRITE:
            {
                bufObject->nCurrentBytes += bufObject->numHWBytes;

                if( bufObject->nCurrentBytes == bufObject->size )
                {
                    bufObject->size = 0;
                    dObj->event     = DRV_TOUCH_MTCH6303_TOUCH_BUFFER_EVENT_COMPLETE;
                }
                else
                {
                    bufObject->flags = DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_RXRDY_READ;
                }

                break;
            }
        }

        DRV_TOUCH_MTCH6303_TOUCH_Tasks( );

    }

    return;
}

void _DRV_TOUCH_MTCH6303_TouchBufferQueueTasks(DRV_TOUCH_MTCH6303_STATIC_OBJ * hDriver)
{
    DRV_TOUCH_MTCH6303_TOUCH_BUFFER_OBJ * bufferObj = 
                           ( DRV_TOUCH_MTCH6303_TOUCH_BUFFER_OBJ * ) NULL;

    bufferObj = hDriver->touchQueue;

    if( NULL != bufferObj )
    {
        if( 0 == bufferObj->size )
        {
            if( ( NULL != hDriver->touchEventHandler ) &&
                ( DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_INPUT_READ & bufferObj->flags ||
                  DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_READ   & bufferObj->flags ||
                  DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_WRITE  & bufferObj->flags ) )
            {
                  hDriver->touchEventHandler( DRV_TOUCH_MTCH6303_TOUCH_BUFFER_EVENT_COMPLETE,
                                              (DRV_TOUCH_MTCH6303_TOUCH_BUFFER_HANDLE) bufferObj,
                                              (uintptr_t) hDriver->touchContext );
            }
                    
            hDriver->touchQueue   = bufferObj->next;
            bufferObj->inUse = false;
            hDriver->touchQueueSizeCurrent--;

        }
 
        if( NULL != hDriver->touchQueue )
        {
            bufferObj = hDriver->touchQueue;

            switch( bufferObj->flags )
            {
                case DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_STATUS_READ:
                {
                    DRV_TOUCH_MTCH6303_AddRegisterRead( &bufferObj->hRegBuffer,
                                                  DRV_TOUCH_MTCH6303_REG_TOUCH_STATUS,
                                                  1, 
                                                  (uint8_t *)&bufferObj->touchData->status );
                    break;                 
                }

                case DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_INPUT_READ:
                {
                    DRV_TOUCH_MTCH6303_AddRegisterRead( &bufferObj->hRegBuffer,
                                                  DRV_TOUCH_MTCH6303_REG_TOUCH_0_NIBBLE_0,
                                                  bufferObj->size, 
                                                  (uint8_t *)&bufferObj->touchData->touch[0] );
                    break; 
                }

                case DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_TXRDY_READ:
                {
                    DRV_TOUCH_MTCH6303_AddRegisterRead( &bufferObj->hRegBuffer,
                                                  DRV_TOUCH_MTCH6303_REG_TX_BYTES_READY,
                                                  1, 
                                                  (uint8_t *)&bufferObj->numHWBytes );
                    break;
                }

                case DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_READ:
                {
                    DRV_TOUCH_MTCH6303_AddRegisterRead( &bufferObj->hRegBuffer,
                                                  DRV_TOUCH_MTCH6303_REG_TX_BUFFER_POINTER,
                                                  bufferObj->numHWBytes, 
                                                  (uint8_t *)&bufferObj->message +
                                                  bufferObj->nCurrentBytes );
                    break;
                }

                case DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_RXRDY_READ:
                {
                    DRV_TOUCH_MTCH6303_AddRegisterRead( &bufferObj->hRegBuffer,
                                                  DRV_TOUCH_MTCH6303_REG_RX_BYTES_READY,
                                                  1, 
                                                  (uint8_t *)&bufferObj->numHWBytes );
                    break;
                }

                case DRV_TOUCH_MTCH6303_TOUCH_BUFFER_FLAG_MSG_WRITE:
                {   
                    DRV_TOUCH_MTCH6303_AddRegisterWrite( &bufferObj->hRegBuffer,
                                                   DRV_TOUCH_MTCH6303_REG_RX_BUFFER_POINTER,
                                                   bufferObj->numHWBytes, 
                                                   (uint8_t *)&bufferObj->message +
                                                   bufferObj->nCurrentBytes );

                    break;
                }
            }

        }
  
    }

    return;
}

/*******************************************************************************
 End of File
*/