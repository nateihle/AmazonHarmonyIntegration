
/*******************************************************************************
  Audio Microphone store

  Company:
    Microchip Technology Inc.

  File Name:
    app_speaker_task.c

  Summary:
    Contains the functional implementation of the speaker application task.

  Description:
    This file contains the functional implementation of the speaker application.
    The speaker application requests the tone generator application to generate
    audio tone for playing on the speaker. Once the audio tone is played out,
    the speaker application notifies the frequency spectrum task to calculate 
    the frequency spectrum of the audio tone.
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
#include "app_speaker_task.h"
#include "app_freq_spectrum_task.h"
#include "app_tone_generator.h"   
#include "app_display_task.h"
    
#define APP_SPEAKER_SAMPLES_PER_OPERATION                       4096
#define APP_SPEAKER_SAMPLE_SIZE_IN_BYTES                        int16_t      

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************

static APP_SPEAKER_MAKE_BUFFER_DMA_READY APP_SPEAKER_SAMPLE_SIZE_IN_BYTES 
    writeBuffer[APP_SPEAKER_WRITE_QUEUE_SIZE][APP_SPEAKER_SAMPLES_PER_OPERATION];

static APP_SPEAKER_TASK_DATA appSpeakerTaskData;
static uint32_t someVar;

static void APP_SPEAKER_BufferEventHandler(
    DRV_CODEC_BUFFER_EVENT event,
    DRV_CODEC_BUFFER_HANDLE handle, 
    uintptr_t context 
);

static void APP_SPEAKER_VolumeSet(uint8_t volume);

inline static uint8_t APP_SPEAKER_ConvertPerVolumeToCodecValue(
    uint8_t perVolume
)
{
    return ((uint32_t)perVolume * DRV_CODEC_VOLUME_MAX )/(uint32_t)100;
}

static void APP_SPEAKER_ToneGeneratorEvHandler (
    APP_TONE_GENERATOR_EVENT_TYPE eventType, 
    const void* const pEventData
)
{
    uint8_t cmd;
    
    switch(eventType)
    {
        case APP_TONE_GENERATOR_EVENT_TYPE_SIG_READY:
            {
                int16_t* pSigData = (int16_t*)pEventData;
                memcpy( writeBuffer[appSpeakerTaskData.codecInIndex], 
                        pSigData, 
                        (APP_SPEAKER_SAMPLES_PER_OPERATION*sizeof(APP_SPEAKER_SAMPLE_SIZE_IN_BYTES))
                );
                                
                cmd = APP_SPEAKER_REQ_SUBMIT_WRITE_BUFFER;
                APP_QUEUE_Push(appSpeakerTaskData.queueHandle, &cmd);                
            }
            break;
        case APP_TONE_GENERATOR_EVENT_TYPE_SIG_DURATION_ELAPSED:
            
            appSpeakerTaskData.isTaskRun = false;
            
            break;
        default:
            break;            
    }
}

static void APP_SPEAKER_DisplayEvHandler (
    APP_DISPLAY_EVENT_TYPE eventType, 
    const void* const pEventData
)
{
    switch(eventType)
    {
        case APP_DISPLAY_EVENT_TYPE_VOL_CHANGE_REQ:
        {
            uint8_t codecVolume;
            codecVolume = APP_SPEAKER_ConvertPerVolumeToCodecValue(*((uint8_t*)pEventData));
            APP_SPEAKER_VolumeSet(codecVolume);
        }
            break;
        default:
            break;
    }
}

static void APP_SPEAKER_InitEvListners(void)
{
    uint8_t i;
    
    for (i = 0; i < APP_SPEAKER_MAX_EVENT_LISTNERS; i++)
    {
        appSpeakerTaskData.eventsCallback[i] = NULL;
    }
}

bool APP_SPEAKER_RegisterCallback(APP_SPEAKER_EVENT_CALLBACK evHandler)
{
    bool isSuccess = false;
    uint8_t i;
    
    if (evHandler)
    {
        for (i = 0; i < APP_SPEAKER_MAX_EVENT_LISTNERS; i++)
        {
            if (NULL == appSpeakerTaskData.eventsCallback[i])
            {
                appSpeakerTaskData.eventsCallback[i] = evHandler;
                isSuccess = true;
                break;
            }
        }
    }                   
    
    return isSuccess;    
}

bool APP_SPEAKER_UnRegisterCallback(APP_SPEAKER_EVENT_CALLBACK evHandler)
{
    bool isSuccess = false;
    uint8_t i;
    
    if (evHandler)
    {
        for (i = 0; i < APP_SPEAKER_MAX_EVENT_LISTNERS; i++)
        {
            if (appSpeakerTaskData.eventsCallback[i] == evHandler)
            {
                appSpeakerTaskData.eventsCallback[i] = NULL;
                isSuccess = true;
                break;
            }
        }
    }
    
    return isSuccess;
}

static void APP_SPEAKER_NotifyListners(
    APP_SPEAKER_EVENT event,
    const void* const pEventData
)
{
    uint8_t i;
    
    for (i = 0; i < APP_SPEAKER_MAX_EVENT_LISTNERS; i++)
    {
        if (appSpeakerTaskData.eventsCallback[i])
        {
            appSpeakerTaskData.eventsCallback[i](event, pEventData);
        }
    }    
}
/******************************************************
 * Application Initialize. It is
 * called from the SYS_Initialized() function.
 ******************************************************/
void APP_SPEAKER_TaskInitialize (void)
{            
    appSpeakerTaskData.state = APP_SPEAKER_TASK_STATES_INIT;        
    appSpeakerTaskData.codecClient.context = (uintptr_t)&someVar;
    appSpeakerTaskData.codecClient.handle = DRV_HANDLE_INVALID;   
    appSpeakerTaskData.codecClient.bufHandle = DRV_CODEC_BUFFER_HANDLE_INVALID;
    appSpeakerTaskData.codecClient.bufferEventHandler = (DRV_CODEC_BUFFER_EVENT_HANDLER) APP_SPEAKER_BufferEventHandler;        
    appSpeakerTaskData.codecClient.bufferSize = APP_SPEAKER_SAMPLES_PER_OPERATION*sizeof(APP_SPEAKER_SAMPLE_SIZE_IN_BYTES);        
    appSpeakerTaskData.codecInIndex = 0;
    appSpeakerTaskData.codecOutIndex = 0;        
    APP_SPEAKER_InitEvListners();
    appSpeakerTaskData.queueHandle = APP_QUEUE_Open(
        appSpeakerTaskData.cmdQueue, 
        sizeof(appSpeakerTaskData.cmdQueue)/APP_SPEAKER_REQ_QUEUE_SIZE,
        APP_SPEAKER_REQ_QUEUE_SIZE
    );
    
    if (NULL == appSpeakerTaskData.queueHandle)
    {
        //printf("Unable to get Speaker Queue handle");
    }        
        

}

static bool APP_SPEAKER_QueueCodecWriteBuffers(uint8_t numWriteBuffers)
{
    bool isSuccess = true;
    uint8_t i;
            
    if (true == appSpeakerTaskData.isTaskRun)
    {
        for(i = 0; i < numWriteBuffers; i++)            
        {
            DRV_CODEC_BufferAddWrite(
                appSpeakerTaskData.codecClient.handle, 
                &appSpeakerTaskData.codecClient.bufHandle,
                writeBuffer[appSpeakerTaskData.codecInIndex], 
                appSpeakerTaskData.codecClient.bufferSize
            );
                        
            if(appSpeakerTaskData.codecClient.bufHandle == DRV_CODEC_BUFFER_HANDLE_INVALID)
            {
                isSuccess = false;
            }
            else
            {                                    
                appSpeakerTaskData.codecInIndex++;                                
                if(appSpeakerTaskData.codecInIndex >= APP_SPEAKER_WRITE_QUEUE_SIZE)
                {
                    appSpeakerTaskData.codecInIndex = 0;
                }            
            }
        }
    }
    
    return isSuccess;
}

bool APP_SPEAKER_AddCommand(APP_SPEAKER_REQ cmd)
{
    bool isSuccess = false;
    
    if (cmd < APP_SPEAKER_REQ_MAX)
    {
        if (true == APP_QUEUE_Push(appSpeakerTaskData.queueHandle, &cmd))
        {
            isSuccess = true;
        }
    }
    
    return isSuccess;
}

static void APP_SPEAKER_ServiceRequests(void)
{
    uint8_t cmd;
    
    if (APP_QUEUE_Pull(appSpeakerTaskData.queueHandle, &cmd))
    {
        switch(cmd)
        {                     
            case APP_SPEAKER_REQ_START:
                appSpeakerTaskData.isTaskRun = true;
                memset(writeBuffer, 0, sizeof(writeBuffer));
                APP_SPEAKER_QueueCodecWriteBuffers(APP_SPEAKER_WRITE_QUEUE_SIZE-1);
                APP_TONE_StartSignalGeneration(MAX_DURATION_IN_SECS);
                break;
                
            case APP_SPEAKER_REQ_STOP:
                appSpeakerTaskData.isTaskRun = false;
                break;
                            
            case APP_SPEAKER_REQ_SPEAKER_DATA_AVAIL:
                APP_SPEAKER_NotifyListners(
                    APP_SPEAKER_EVENT_DATA_AVAIL,
                    writeBuffer[appSpeakerTaskData.codecOutIndex]
                );
    
                appSpeakerTaskData.codecOutIndex++;

                if (appSpeakerTaskData.codecOutIndex >= APP_SPEAKER_WRITE_QUEUE_SIZE)
                {
                    appSpeakerTaskData.codecOutIndex = 0;
                }
                
                if (appSpeakerTaskData.codecInIndex == appSpeakerTaskData.codecOutIndex)
                {
                    APP_SPEAKER_AddCommand(APP_SPEAKER_REQ_NO_BUFFER_TO_PLAY);    
                }                
                break;
                
            case APP_SPEAKER_REQ_SUBMIT_SIG_DATA_REQUEST:                
                APP_TONE_GenSignalData();                
                break;            
                
            case APP_SPEAKER_REQ_SUBMIT_WRITE_BUFFER:
                APP_SPEAKER_QueueCodecWriteBuffers(1);                
                break;             
                
            case APP_SPEAKER_REQ_NO_BUFFER_TO_PLAY:
                APP_SPEAKER_NotifyListners(
                    APP_SPEAKER_EVENT_STOPPED,
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
void APP_SPEAKER_Tasks (void )
{    
    switch(appSpeakerTaskData.state)
    {            
        case APP_SPEAKER_TASK_STATES_INIT:
            
            APP_FREQ_SPECTRUM_ConfigInputAudio(
                AUDIO_TYPE_MONO, 
                SAMPLING_FREQ_48_KHZ
            );  
            APP_TONE_RegisterCallback(APP_SPEAKER_ToneGeneratorEvHandler);
            APP_DISPLAY_RegisterCallback(APP_SPEAKER_DisplayEvHandler);
            appSpeakerTaskData.state = APP_SPEAKER_TASK_STATES_CODEC_OPEN;
            
            break;
        case APP_SPEAKER_TASK_STATES_CODEC_OPEN:
        
            /* A client opens the driver object to get an Handle */            
            appSpeakerTaskData.codecClient.handle = 
                DRV_CODEC_Open(
                    DRV_CODEC_INDEX_0,
                    DRV_IO_INTENT_WRITE
                );
            
            if(appSpeakerTaskData.codecClient.handle != DRV_HANDLE_INVALID)
            {
                DRV_CODEC_SamplingRateSet(
                    appSpeakerTaskData.codecClient.handle, 
                    APP_CODEC_SAMPLING_RATE
                );                                 

                appSpeakerTaskData.state = APP_SPEAKER_TASK_STATES_CODEC_SET_BUFFER_HANDLER;
            }
            else
            {
                /* Got an Invalid Handle.  Wait for CODEC to Initialize */
                ;
            }        
            break;

        /* Set a handler for the audio buffer completion event */
        case APP_SPEAKER_TASK_STATES_CODEC_SET_BUFFER_HANDLER:
            {                                
                DRV_CODEC_BufferEventHandlerSet(appSpeakerTaskData.codecClient.handle,
                        appSpeakerTaskData.codecClient.bufferEventHandler,
                        appSpeakerTaskData.codecClient.context);
                
                appSpeakerTaskData.state = APP_SPEAKER_TASK_STATES_IDLE;   
            }
            break;            
        
        case APP_SPEAKER_TASK_STATES_IDLE:        
            APP_SPEAKER_ServiceRequests();
            break;
            
        case APP_SPEAKER_TASK_STATES_ERROR:            
            break;
    } 
}

/**********************************************************
 * Application CODEC buffer Event handler.
 * This function is called back by the CODEC driver when
 * a CODEC data buffer RX or TX completes.
 ***********************************************************/
static void APP_SPEAKER_BufferEventHandler(
    DRV_CODEC_BUFFER_EVENT event,
    DRV_CODEC_BUFFER_HANDLE handle, 
    uintptr_t context 
)
{
    switch(event)
    {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:                              
            APP_SPEAKER_AddCommand(APP_SPEAKER_REQ_SPEAKER_DATA_AVAIL);                    
            APP_SPEAKER_AddCommand(APP_SPEAKER_REQ_SUBMIT_SIG_DATA_REQUEST);              
            break;
            
        case DRV_CODEC_BUFFER_EVENT_ERROR:        
            break;

        case DRV_CODEC_BUFFER_EVENT_ABORT:        
            break;
    }
}

///////////////////////////////////////////////////////////////////////////////
__attribute__((unused)) static void APP_SPEAKER_CodecClose(void)
{
    if(appSpeakerTaskData.codecClient.handle!=DRV_HANDLE_INVALID)
    {
        DRV_CODEC_Close(appSpeakerTaskData.codecClient.handle);
    }   
}

///////////////////////////////////////////////////////////////////////////////
__attribute__((unused)) static void APP_SPEAKER_MuteOn(void)
{
    DRV_CODEC_MuteOn(appSpeakerTaskData.codecClient.handle);
}

///////////////////////////////////////////////////////////////////////////////
__attribute__((unused)) static void APP_SPEAKER_MuteOff(void)
{
    DRV_CODEC_MuteOff(appSpeakerTaskData.codecClient.handle);
}

///////////////////////////////////////////////////////////////////////////////
static void APP_SPEAKER_VolumeSet(uint8_t volume)
{
    if (appSpeakerTaskData.codecClient.handle)
    {
        DRV_CODEC_VolumeSet(appSpeakerTaskData.codecClient.handle, DRV_CODEC_CHANNEL_LEFT_RIGHT, volume);
    }
}

#ifdef __cplusplus
}
#endif

/*******************************************************************************
 End of File
 */


