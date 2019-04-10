

//******************************************************************************
//AUDIO_PLAY_BUFFER
//******************************************************************************
#define AUDIO_PLAY_BUFFER_SIZE (1280*2)
typedef struct _AUDIO_PLAY_BUFFER
{
    uint32_t   dataLen;    //#actually generated data
    uint32_t   dataLenMax; //Size of allocated data buffer
    int16_t    data[AUDIO_PLAY_BUFFER_SIZE];  //A single buffer in the playback queu
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

