//******************************************************************************
// AudioPlayBufferQueue.c file
//
// Description:
//
//   Audio play buffer queue implementation
//
//   NOTE: APP_DATA_TYPE in system_config.h determines data size. 
// 
//******************************************************************************

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.
Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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
// DOM-IGNORE-END

#include "AudioPlayBufferQueue.h"

//******************************************************************************
//Initialize the queue empty.
//******************************************************************************
void PlayBufferQueueInit(PlayBufferQueue * queue, AUDIO_PLAY_BUFFER * audioPlayBuffer)
{
    int j;
    int32_t numPlayBuffers = NUM_PLAY_BUFFERS;
    

    memset(audioPlayBuffer,0,sizeof(AUDIO_PLAY_BUFFER));

    for(j=0; j<numPlayBuffers; j++)
    {
        queue->bufferQueue[j] = &audioPlayBuffer[j];
        audioPlayBuffer[j].dataLenMax = AUDIO_PLAY_BUFFER_SIZE;
    }

    queue->queueHead = 0;
    queue->queueTail = 0;
    queue->queueLen  = 0;

}

//******************************************************************************
// Get the tail buffer (Next buffer to fill)
//******************************************************************************
AUDIO_PLAY_BUFFER * PlayBufferQueueGetTail(PlayBufferQueue * queue)
{
    if (queue->queueLen < NUM_PLAY_BUFFERS)
    {
        return queue->bufferQueue[queue->queueTail];
    }
    else
    {
        return NULL;
    }
}

AUDIO_PLAY_BUFFER * PlayBufferQueueGetHead(PlayBufferQueue * queue)
{
    if ( queue->queueLen > 0)
    {
        return queue->bufferQueue[queue->queueHead];
    }
    else
    {
        return NULL;
    }
}


//Add to current tail position
BOOL PlayBufferQueueAdd(PlayBufferQueue * queue)
{
    if (queue->queueLen < NUM_PLAY_BUFFERS)
    {
        queue->queueTail++;
        if (queue->queueTail >= NUM_PLAY_BUFFERS)
        {
            queue->queueTail = 0; //Circular queue
        }

        //Disable interrupts
        queue->queueLen++;
        //Enable interrups


        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

//Remove the current head position
BOOL PlayBufferQueueRemove(PlayBufferQueue * queue)
{
    queue->queueHead++;
    if (queue->queueHead >= NUM_PLAY_BUFFERS)
    {
        queue->queueHead = 0; //Circular queue
    }

    //Disable interrupts
    queue->queueLen--;
    //Enable interrups

    return FALSE;
}

void PlayBufferQueueClear(PlayBufferQueue * queue)
{
    queue->queueLen  = 0;
    queue->queueHead = 0;
    queue->queueTail = 0;
}
