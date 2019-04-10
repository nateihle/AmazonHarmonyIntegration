/*******************************************************************************
  Audio Microphone store

  Company:
    Microchip Technology Inc.

  File Name:
    app_microphone_task.c

  Summary:
    Contains the functional implementation of the Microphone application.

  Description:
    This file contains the functional implementation of Microphone application.
    Once sufficient audio samples are available, it notifies the frequency 
    spectrum task to calculate frequency spectrum of the audio samples.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "app_microphone_task.h"
#include "app_freq_spectrum_task.h"
#include "app_tone_generator.h"   
    
#define APP_MICROPHONE_SAMPLES_PER_OPERATION                     4096
#define APP_MICROPHONE_SAMPLE_SIZE_IN_BYTES                      DRV_I2S_DATA16      
    
// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************

static APP_MICROPHONE_MAKE_BUFFER_DMA_READY APP_MICROPHONE_SAMPLE_SIZE_IN_BYTES 
    readBuffer[APP_MICROPHONE_READ_QUEUE_SIZE][APP_MICROPHONE_SAMPLES_PER_OPERATION];

static APP_MICROPHONE_TASK_DATA appMicrophoneData;
static uint32_t someVar;
static void APP_MICROPHONE_BufferEventHandler(
    DRV_CODEC_BUFFER_EVENT event,
    DRV_CODEC_BUFFER_HANDLE handle, 
    uintptr_t context 
);

/******************************************************
 * Application Initialize. It is
 * called from the SYS_Initialized() function.
 ******************************************************/
void APP_MICROPHONE_TaskInitialize (void)
{            
    appMicrophoneData.state = APP_MICROPHONE_TASK_STATES_INIT;        
    appMicrophoneData.codecClient.context = (uintptr_t)&someVar;
    appMicrophoneData.codecClient.handle = DRV_HANDLE_INVALID;    
    appMicrophoneData.codecClient.bufferEventHandler = (DRV_CODEC_BUFFER_EVENT_HANDLER) APP_MICROPHONE_BufferEventHandler;    
    appMicrophoneData.codecClient.bufHandle = DRV_CODEC_BUFFER_HANDLE_INVALID;
    appMicrophoneData.codecClient.bufferSize = APP_MICROPHONE_SAMPLES_PER_OPERATION*sizeof(APP_MICROPHONE_SAMPLE_SIZE_IN_BYTES);    
    appMicrophoneData.codecInIndex = 0;
    appMicrophoneData.codecOutIndex = 0;
    appMicrophoneData.nEventListners = 0;
    appMicrophoneData.isTaskRun = false;
    appMicrophoneData.queueHandle = APP_QUEUE_Open(
        appMicrophoneData.cmdQueue, 
        sizeof(appMicrophoneData.cmdQueue)/APP_MICROPHONE_REQ_QUEUE_SIZE,
        APP_MICROPHONE_REQ_QUEUE_SIZE
    );
    
    if (NULL == appMicrophoneData.queueHandle)
    {
        //printf("Unable to get mic Queue handle");
    }                    
}

static bool APP_MICROPHONE_QueueCodecReadBuffers(uint8_t numReadBuffers)
{
    bool isSuccess = true;
    uint8_t i;
    
    if (true == appMicrophoneData.isTaskRun)
    {
        for(i = 0; i < numReadBuffers; i++)            
        {
            DRV_CODEC_BufferAddRead(
                appMicrophoneData.codecClient.handle, 
                &appMicrophoneData.codecClient.bufHandle,
                readBuffer[appMicrophoneData.codecInIndex], 
                appMicrophoneData.codecClient.bufferSize
            );

            if(appMicrophoneData.codecClient.bufHandle == DRV_CODEC_BUFFER_HANDLE_INVALID)
            {
                isSuccess = false;
            }
            else
            {                                    
                appMicrophoneData.codecInIndex++;                                
                if(appMicrophoneData.codecInIndex >= APP_MICROPHONE_READ_QUEUE_SIZE)
                {
                    appMicrophoneData.codecInIndex = 0;
                }            
            }
        }
    }
    
    return isSuccess;
}

bool APP_MICROPHONE_AddCommand(APP_MICROPHONE_REQ cmd)
{
    bool isSuccess = false;
    
    if (cmd < APP_MICROPHONE_REQ_MAX)
    {
        if (true == APP_QUEUE_Push(appMicrophoneData.queueHandle, &cmd))
        {
            isSuccess = true;
        }
    }
    
    return isSuccess;
}

static void APP_MICROPHONE_DiscardEmptyChannel(
    DRV_I2S_DATA16* pInput, 
    int16_t* pOutput, 
    int numSamples
)
{
    int i;   
    volatile int16_t temp;

    for(i = 0; i < numSamples; i++)
    {        
        // Only one channel has valid data, the other one is zero, but we don't know which one is the 
        // "REAL" left channel and right channel, by adding this two together, we can get 
        // that one channel data, then send this output buffer to USB.
        temp =  pInput[i].leftData + pInput[i].rightData;                
        pOutput[i] = temp;
    }
    
}

static void APP_MICROPHONE_NotifyEventListners(
    APP_MICROPHONE_EVENT event, 
    const void* const pEventData
)
{
    uint8_t i = 0;
    
    for (i = 0; i < appMicrophoneData.nEventListners; i++)
    {
        appMicrophoneData.eventsCallback[i](event, pEventData);
    }
}

static void APP_MICROPHONE_ServiceRequests(void)
{
    uint8_t cmd;
    
    if (APP_QUEUE_Pull(appMicrophoneData.queueHandle, &cmd))
    {
        switch(cmd)
        {            
            case APP_MICROPHONE_REQ_START:
                appMicrophoneData.isTaskRun = true;
                APP_MICROPHONE_QueueCodecReadBuffers(APP_MICROPHONE_READ_QUEUE_SIZE-1);
                break;
                
            case APP_MICROPHONE_REQ_STOP:
                appMicrophoneData.isTaskRun = false;
                break;
                                        
            case APP_MICROPHONE_REQ_MICROPHONE_DATA_AVAIL:
                
                APP_MICROPHONE_DiscardEmptyChannel(
                    readBuffer[appMicrophoneData.codecOutIndex],
                    (int16_t*)readBuffer[appMicrophoneData.codecOutIndex],
                    APP_MICROPHONE_SAMPLES_PER_OPERATION
                );
                
                APP_MICROPHONE_NotifyEventListners(
                    APP_MICROPHONE_EVENT_DATA_AVAIL,
                    readBuffer[appMicrophoneData.codecOutIndex]
                );
    
                appMicrophoneData.codecOutIndex++;

                if (appMicrophoneData.codecOutIndex >= APP_MICROPHONE_READ_QUEUE_SIZE)
                {
                    appMicrophoneData.codecOutIndex = 0;
                }

                if (appMicrophoneData.codecInIndex == appMicrophoneData.codecOutIndex)
                {
                    APP_MICROPHONE_AddCommand(APP_MICROPHONE_REQ_NO_BUFFER_TO_PLAY);        
                }
                break;
                
            case APP_MICROPHONE_REQ_NO_BUFFER_TO_PLAY:
                APP_MICROPHONE_NotifyEventListners(
                    APP_MICROPHONE_EVENT_STOPPED,
                    NULL
                );                
                break;
                
            default:
                break;
        }
    }
}

/**********************************************************
 * Application tasks routine. This function implements the
 * application state machine.
 ***********************************************************/
void APP_MICROPHONE_Tasks (void )
{    
    switch(appMicrophoneData.state)
    {            
        case APP_MICROPHONE_TASK_STATES_INIT:
            
            APP_FREQ_SPECTRUM_ConfigInputAudio(AUDIO_TYPE_MONO, SAMPLING_FREQ_48_KHZ);      
            appMicrophoneData.state = APP_MICROPHONE_TASK_STATES_CODEC_OPEN;
            
            break;
        case APP_MICROPHONE_TASK_STATES_CODEC_OPEN:
        
            /* A client opens the driver object to get an Handle */
            appMicrophoneData.codecClient.handle = 
                DRV_CODEC_Open(
                    DRV_CODEC_INDEX_0,
                    DRV_IO_INTENT_READ
                );
                                    
            if(appMicrophoneData.codecClient.handle != DRV_HANDLE_INVALID)
            {
                DRV_CODEC_SamplingRateSet(
                    appMicrophoneData.codecClient.handle, 
                    APP_CODEC_SAMPLING_RATE
                );                                 

                appMicrophoneData.state = APP_MICROPHONE_TASK_STATES_CODEC_SET_BUFFER_HANDLER;
            }
            else
            {
                /* Got an Invalid Handle.  Wait for CODEC to Initialize */
                ;
            }        
            break;

        /* Set a handler for the audio buffer completion event */
        case APP_MICROPHONE_TASK_STATES_CODEC_SET_BUFFER_HANDLER:
            {                
                DRV_CODEC_BufferEventHandlerSet(
                    appMicrophoneData.codecClient.handle,
                    appMicrophoneData.codecClient.bufferEventHandler,
                    appMicrophoneData.codecClient.context
                );
                                                
                appMicrophoneData.state = APP_MICROPHONE_TASK_STATES_IDLE;   
            }
            break;            
        
        case APP_MICROPHONE_TASK_STATES_IDLE:        
            APP_MICROPHONE_ServiceRequests();
            break;
            
        case APP_MICROPHONE_TASK_STATES_ERROR:            
            break;
    } 
}

/**********************************************************
 * Application CODEC buffer Event handler.
 * This function is called back by the CODEC driver when
 * a CODEC data buffer RX or TX completes.
 ***********************************************************/
static void APP_MICROPHONE_BufferEventHandler(
    DRV_CODEC_BUFFER_EVENT event,
    DRV_CODEC_BUFFER_HANDLE handle, 
    uintptr_t context 
)
{
    switch(event)
    {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:               
            APP_MICROPHONE_QueueCodecReadBuffers(1);
            APP_MICROPHONE_AddCommand(APP_MICROPHONE_REQ_MICROPHONE_DATA_AVAIL);            
            break;
            
        case DRV_CODEC_BUFFER_EVENT_ERROR:        
            break;

        case DRV_CODEC_BUFFER_EVENT_ABORT:        
            break;
    }
}

///////////////////////////////////////////////////////////////////////////////
__attribute__((unused)) static void APP_MICROPHONE_CodecClose(void)
{
    if(appMicrophoneData.codecClient.handle != DRV_HANDLE_INVALID)
    {
        DRV_CODEC_Close(appMicrophoneData.codecClient.handle);
    }   
}

bool APP_MICROPHONE_RegisterCallback(APP_MICROPHONE_EVENT_CALLBACK evHandler)
{
    bool isSuccess = false;
    
    if (appMicrophoneData.nEventListners < APP_MICROPHONE_MAX_EVENT_LISTNERS)
    {
        if (evHandler)
        {
            appMicrophoneData.eventsCallback[appMicrophoneData.nEventListners] = evHandler;
            appMicrophoneData.nEventListners++;
            isSuccess = true;
        }
    }
    
    return isSuccess;
}

#ifdef __cplusplus
}
#endif

/*******************************************************************************
 End of File
 */


