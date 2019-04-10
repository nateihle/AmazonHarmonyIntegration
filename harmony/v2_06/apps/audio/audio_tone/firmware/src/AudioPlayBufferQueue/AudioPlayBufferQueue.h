//******************************************************************************
// AudioPlayBufferQueue.h
// 
// file name: 
// 
// Description:
//
//   Audio play buffer queue for 16 bit data
//
//   NOTE: APP_DATA_TYPE in system_config.h determines data size. 
//
//******************************************************************************

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _AUDIOPLAYBUFFERQUEUE_H
#define _AUDIOPLAYBUFFERQUEUE_H

#include "GenericTypeDefs.h"
#include "app_config.h"       //LED definitions added here.

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END


//******************************************************************************
//AUDIO_PLAY_BUFFER
//******************************************************************************
#define AUDIO_PLAY_BUFFER_SIZE (1280*2)
typedef struct _AUDIO_PLAY_BUFFER
{
    uint32_t   dataLen;    //#actually generated data
    uint32_t   dataLenMax; //Size of allocated data buffer
    APP_DATA_TYPE data[AUDIO_PLAY_BUFFER_SIZE];  //A single buffer in the playback queu
} AUDIO_PLAY_BUFFER;

//******************************************************************************
// AUDIO_PLAY_BUFFER Queue
//******************************************************************************
#define NUM_PLAY_BUFFERS 3
typedef struct _PlayBufferQueue
{
    AUDIO_PLAY_BUFFER * bufferQueue[NUM_PLAY_BUFFERS]; 
    uint8_t             queueHead;
    uint8_t             queueTail;
    volatile uint8_t    queueLen;
} PlayBufferQueue;

void PlayBufferQueueInit(PlayBufferQueue * queue, AUDIO_PLAY_BUFFER * audioPlayBuffer);
AUDIO_PLAY_BUFFER * PlayBufferQueueGetTail(PlayBufferQueue * queue);
AUDIO_PLAY_BUFFER * PlayBufferQueueGetHead(PlayBufferQueue * queue);
BOOL PlayBufferQueueAdd(PlayBufferQueue * queue);
BOOL PlayBufferQueueRemove(PlayBufferQueue * queue);
void PlayBufferQueueClear(PlayBufferQueue * queue);

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif  //_AUDIOPLAYBUFFERQUEUE_H
