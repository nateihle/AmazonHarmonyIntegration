<#-- drv_codec_gencode_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_drv_codec_app_c_includes>
#include <string.h>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data
*/
-->
<#macro macro_drv_codec_app_c_global_data>
#define ${APP_NAME?upper_case}_NUM_PLAY_BUFFERS 3
#define ${APP_NAME?upper_case}_AUDIO_PLAY_BUFFER_SIZE (48*2)


typedef struct _AUDIO_PLAY_BUFFER
{
    uint32_t dataLen;    //#actually generated data
    uint16_t data[${APP_NAME?upper_case}_AUDIO_PLAY_BUFFER_SIZE];
} ${APP_NAME?upper_case}_AUDIO_PLAY_BUFFER;

typedef struct _PlayBufferQueue
{
    ${APP_NAME?upper_case}_AUDIO_PLAY_BUFFER * bufferQueue[${APP_NAME?upper_case}_NUM_PLAY_BUFFERS]; 
    uint8_t             queueHead;
    uint8_t             queueTail;
    volatile uint8_t    queueLen;
} ${APP_NAME?lower_case}PlayBufferQueue;

${APP_NAME?upper_case}_AUDIO_PLAY_BUFFER audioPlayBuffer[${APP_NAME?upper_case}_NUM_PLAY_BUFFERS];
${APP_NAME?upper_case}_AUDIO_PLAY_BUFFER *currAudioPlayBuffer = NULL;

${APP_NAME?lower_case}PlayBufferQueue     queue;

 uint16_t ${APP_NAME?lower_case}ToneData16[${APP_NAME?upper_case}_AUDIO_PLAY_BUFFER_SIZE] =
 {0x0000, 0x0000, 0x10B0, 0x10B0, 0x2120, 0x2120, 0x3100, 0x3100,
  0x4000, 0x4000, 0x4DE8, 0x4DE8, 0x5A80, 0x5A80, 0x6588, 0x6588,
  0x6ED8, 0x6ED8, 0x7640, 0x7640, 0x7BA8, 0x7BA8, 0x7EE8, 0x7EE8,
  0x7FFF, 0x7FFF, 0x7EE8, 0x7EE8, 0x7BA8, 0x7BA8, 0x7640, 0x7640,
  0x6ED8, 0x6ED8, 0x6588, 0x6588, 0x5A80, 0x5A80, 0x4DE8, 0x4DE8,
  0x4000, 0x4000, 0x3100, 0x3100, 0x2120, 0x2120, 0x10B0, 0x10B0,
  0x0000, 0x0000, 0xEF50, 0xEF50, 0xDEE0, 0xDEE0, 0xCF00, 0xCF00,
  0xC000, 0xC000, 0xB218, 0xB218, 0xA580, 0xA580, 0x9A78, 0x9A78,
  0x9128, 0x9128, 0x89C0, 0x89C0, 0x8458, 0x8458, 0x8118, 0x8118,
  0x8001, 0x8001, 0x8118, 0x8118, 0x8458, 0x8458, 0x89C0, 0x89C0,
  0x9128, 0x9128, 0x9A78, 0x9A78, 0xA580, 0xA580, 0xB218, 0xB218,
  0xC000, 0xC000, 0xCF00, 0xCF00, 0xDEE0, 0xDEE0, 0xEF50, 0xEF50};


</#macro>

<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_drv_codec_app_c_callback_functions>
void ${APP_NAME?upper_case}_CODEC_BufferEventHandler(DRV_CODEC_BUFFER_EVENT event)
{
    switch(event)
    {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        {
            ${APP_NAME?lower_case}Data.AudioToneTaskState = ${APP_NAME?upper_case}_AUDIO_TONE_TASK_STATE_CODEC_BUFFER_COMPLETE;
        }      
        case DRV_CODEC_BUFFER_EVENT_ERROR:
        case DRV_CODEC_BUFFER_EVENT_ABORT:
        break;
    }
}
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_drv_codec_app_c_local_functions>
void AUDIO_CODEC_Initialize (${APP_NAME?upper_case}_AUDIO_CODEC_CLIENT * codecClient)
{
    codecClient->handle = DRV_HANDLE_INVALID;
    codecClient->context = 0;
    codecClient->bufferHandler = 
           (DRV_CODEC_BUFFER_EVENT_HANDLER) ${APP_NAME?upper_case}_CODEC_BufferEventHandler;
    codecClient->commandHandler = 
           (DRV_CODEC_COMMAND_EVENT_HANDLER) ${APP_NAME?upper_case}_CodecCommandClear;
    codecClient->txbufferObject = NULL;
    codecClient->bufferSize = 0;
    codecClient->currentCommand = CODEC_COMMAND_NONE;
}

void PlayBufferQueueInit(${APP_NAME?lower_case}PlayBufferQueue * queue, ${APP_NAME?upper_case}_AUDIO_PLAY_BUFFER * audioPlayBuffer)
{
    int j;

    for(j = 0; j < ${APP_NAME?upper_case}_NUM_PLAY_BUFFERS; j++)
    {
        queue->bufferQueue[j] = &audioPlayBuffer[j];
    }

    queue->queueHead = 0;
    queue->queueTail = 0;
    queue->queueLen  = 0;
}

void PlayBufferQueueRemove(${APP_NAME?lower_case}PlayBufferQueue * queue)
{
    queue->queueHead++;
    if (queue->queueHead >= ${APP_NAME?upper_case}_NUM_PLAY_BUFFERS)
    {
        queue->queueHead = 0; //Circular queue
    }
    queue->queueLen--;
}

void PlayBufferQueueAdd(${APP_NAME?lower_case}PlayBufferQueue * queue)
{
    if (queue->queueLen < ${APP_NAME?upper_case}_NUM_PLAY_BUFFERS)
    {
        queue->queueTail++;
        if (queue->queueTail >= ${APP_NAME?upper_case}_NUM_PLAY_BUFFERS)
        {
            queue->queueTail = 0; //Circular queue
        }
        queue->queueLen++;
    }
}

${APP_NAME?upper_case}_AUDIO_PLAY_BUFFER * PlayBufferQueueGetHead(${APP_NAME?lower_case}PlayBufferQueue * queue)
{
    return queue->queueLen > 0 ? queue->bufferQueue[queue->queueHead] : NULL;
}

${APP_NAME?upper_case}_AUDIO_PLAY_BUFFER * PlayBufferQueueGetTail(${APP_NAME?lower_case}PlayBufferQueue * queue)
{
    return queue->queueLen < ${APP_NAME?upper_case}_NUM_PLAY_BUFFERS ? queue->bufferQueue[queue->queueTail] : NULL;
}

void ${APP_NAME?upper_case}_CodecCommandClear()
{
    if(${APP_NAME?lower_case}Data.codecClient.currentCommand == CODEC_COMMAND_SAMPLING_RATE_SET)
    {
        ${APP_NAME?lower_case}Data.codecClient.currentCommand = CODEC_COMMAND_NONE;
    }
}


static void AudioToneTask(void)
{
    switch (${APP_NAME?lower_case}Data.AudioToneTaskState)
    {
        case ${APP_NAME?upper_case}_AUDIO_TONE_TASK_STATE_CODEC_ADD_BUFFER:
        {
            //Get the next output play buffer
            currAudioPlayBuffer = PlayBufferQueueGetHead(&queue);

            ${APP_NAME?lower_case}Data.codecClient.txbufferObject = (uint8_t*) currAudioPlayBuffer->data;
            ${APP_NAME?lower_case}Data.codecClient.bufferSize = currAudioPlayBuffer->dataLen*sizeof(uint16_t);

            DRV_CODEC_BufferAddWrite(${APP_NAME?lower_case}Data.codecClient.handle, 
                             &(${APP_NAME?lower_case}Data.codecClient.writeBufHandle),
                             (int8_t*)currAudioPlayBuffer->data, 
                             (size_t)${APP_NAME?lower_case}Data.codecClient.bufferSize);

            if(${APP_NAME?lower_case}Data.codecClient.writeBufHandle != DRV_CODEC_BUFFER_HANDLE_INVALID)
            {
                ${APP_NAME?lower_case}Data.AudioToneTaskState = ${APP_NAME?upper_case}_AUDIO_TONE_TASK_STATE_CODEC_WAIT_FOR_BUFFER_COMPLETE;
            }
        }
        break;
        
        case ${APP_NAME?upper_case}_AUDIO_TONE_TASK_STATE_CODEC_WAIT_FOR_BUFFER_COMPLETE:
        {
            currAudioPlayBuffer = PlayBufferQueueGetTail(&queue);
            if (currAudioPlayBuffer != NULL)
            {
                if (currAudioPlayBuffer->dataLen > 0) PlayBufferQueueAdd(&queue); 
            }
        }
        break;

        case ${APP_NAME?upper_case}_AUDIO_TONE_TASK_STATE_CODEC_BUFFER_COMPLETE:
        {
            PlayBufferQueueRemove(&queue);
            currAudioPlayBuffer = PlayBufferQueueGetHead(&queue); 

            if (currAudioPlayBuffer == NULL) 
            {
                ${APP_NAME?lower_case}Data.AudioToneTaskState = ${APP_NAME?upper_case}_AUDIO_TONE_TASK_STATE_CODEC_ADD_BUFFER;
            }
            else
            {
                ${APP_NAME?lower_case}Data.codecClient.txbufferObject = 
                        (uint8_t *) currAudioPlayBuffer->data;
                ${APP_NAME?lower_case}Data.codecClient.bufferSize =
                (size_t) currAudioPlayBuffer->dataLen*sizeof(uint16_t);
        
                DRV_CODEC_BufferAddWrite(${APP_NAME?lower_case}Data.codecClient.handle, 
                             &(${APP_NAME?lower_case}Data.codecClient.writeBufHandle),
                             ${APP_NAME?lower_case}Data.codecClient.txbufferObject, 
                             ${APP_NAME?lower_case}Data.codecClient.bufferSize);

                if(${APP_NAME?lower_case}Data.codecClient.writeBufHandle != DRV_CODEC_BUFFER_HANDLE_INVALID)
                {
                    ${APP_NAME?lower_case}Data.AudioToneTaskState = ${APP_NAME?upper_case}_AUDIO_TONE_TASK_STATE_CODEC_WAIT_FOR_BUFFER_COMPLETE;
                }
            }
        }
        break;
    } 
}
</#macro>

<#--
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Initialize ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_INIT;
-->
<#macro macro_drv_codec_app_c_initialize>
    AUDIO_CODEC_Initialize(&(${APP_NAME?lower_case}Data.codecClient));
    
    PlayBufferQueueInit(&queue, audioPlayBuffer);
    
    do
    {
        currAudioPlayBuffer = PlayBufferQueueGetTail(&queue);
        if (currAudioPlayBuffer != NULL)
        {
            currAudioPlayBuffer->dataLen = ${APP_NAME?upper_case}_AUDIO_PLAY_BUFFER_SIZE;
            memcpy(currAudioPlayBuffer->data, ${APP_NAME?lower_case}ToneData16, sizeof(${APP_NAME?lower_case}ToneData16));
            PlayBufferQueueAdd(&queue); //Add to tail
            currAudioPlayBuffer = PlayBufferQueueGetTail(&queue);
        }

    } while (currAudioPlayBuffer != NULL);

	${APP_NAME?lower_case}Data.AudioToneTaskState = ${APP_NAME?upper_case}_AUDIO_TONE_TASK_STATE_CODEC_ADD_BUFFER;
</#macro>

<#--
}


/******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Tasks ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Tasks ( void )
{
-->
<#macro macro_drv_codec_app_c_tasks_data>
</#macro>

<#--
    /* Check the application's current state. */
    switch ( ${APP_NAME?lower_case}Data.state )
    {
        /* Application's initial state. */
        case ${APP_NAME?upper_case}_STATE_INIT:
        {
            bool appInitialized = true;
-->   
<#macro macro_drv_codec_app_c_tasks_state_init>
			if (SYS_STATUS_READY == DRV_CODEC_Status(sysObjdrvCodec0))
            {
                ${APP_NAME?lower_case}Data.codecClient.handle = DRV_CODEC_Open(DRV_CODEC_INDEX_0,
                        DRV_IO_INTENT_WRITE | DRV_IO_INTENT_EXCLUSIVE);
                if(${APP_NAME?lower_case}Data.codecClient.handle != DRV_HANDLE_INVALID)
                {
                    DRV_CODEC_BufferEventHandlerSet(${APP_NAME?lower_case}Data.codecClient.handle, 
                                     ${APP_NAME?lower_case}Data.codecClient.bufferHandler, 
                                     ${APP_NAME?lower_case}Data.codecClient.context);
                }
                else
                {
                    appInitialized = false;
                }
            }
            else
            {
                appInitialized = false;
            }
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_drv_codec_app_c_tasks_calls_after_init>
</#macro>

<#--            /* Advance to the next state */
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_drv_codec_app_c_tasks_state_service_tasks>
			AudioToneTask();
</#macro>

<#--        
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
-->

<#macro macro_drv_codec_app_c_tasks_states>
</#macro>
