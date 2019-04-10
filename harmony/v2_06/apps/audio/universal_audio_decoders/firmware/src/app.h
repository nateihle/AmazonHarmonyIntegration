/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

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

#include "system_config.h"
#include "system_definitions.h"    
#include "user_config.h"   


typedef enum{
    APP_SUCCESS = 1,
    APP_READ_ERROR,
    APP_STREAM_ERROR,
    APP_BUFF_ERROR,
    APP_STREAM_END,
    APP_PLAYBACK_ERROR,
    APP_OUT_OF_MEM_ERROR,
    APP_DISK_ERROR,
    APP_GENERAL_ERROR
}APP_ERROR_MSG;

typedef enum
{
    APP_STATE_OPEN_HOST_LAYER,
    APP_STATE_WAIT_FOR_HOST_ENABLE,
    APP_STATE_DEVICE_CONNECTED,
    APP_STATE_WAIT_FOR_DEVICE_ATTACH,
    APP_STATE_UNMOUNT_DISK
  } APP_USB_STATE;

typedef enum
{
    APP_STATE_RUNNING,
    APP_STATE_HALT,
    APP_STATE_NO_MEDIA,
    APP_STATE_NO_FILE,
    APP_STATE_WAITING,
    APP_STATE_CODEC_OPEN,
    APP_STATE_SCANNING,
    APP_SEND_AUDIO_DATA,
    APP_DECODE_DATA,
    APP_STATE_TRACK_CHANGE,
    APP_WAIT_BUFFERS_COMPLETED,
} APP_PLAYER_STATE;

typedef enum
{
    PLAYER_CMD_NONE,
    PLAYER_CMD_STOP,
    PLAYER_CMD_PLAY,
    PLAYER_CMD_PAUSE,
    PLAYER_CMD_PLAY_PAUSE,
    PLAYER_CMD_NEXT_FILE,
    PLAYER_CMD_PREV_FILE,
    PLAYER_CMD_FAST_FORWARD,
    PLAYER_CMD_REWIND,
}PLAYER_COMMAND;

typedef enum
{
    PLAYER_EVENT_DISK_REMOVED,
    PLAYER_EVENT_DISK_INSERTED,
    PLAYER_EVENT_SCANNING_STARTED,
    PLAYER_EVENT_SCANNING_FOUND,
    PLAYER_EVENT_SCANNING_FINISHED,
    PLAYER_EVENT_NO_FILES_FOUND,
    PLAYER_EVENT_READY,
    PLAYER_EVENT_TAG_ARTIST,
    PLAYER_EVENT_TAG_ALBUM,
    PLAYER_EVENT_TAG_TITLE,
    PLAYER_EVENT_TRACK_CHANGED,
    PLAYER_EVENT_TIME_CHANGED,
    PLAYER_EVENT_SAMPLERATE,
    PLAYER_EVENT_BITRATE,
    PLAYER_EVENT_TRACK_TOTAL_TIME,
    PLAYER_EVENT_TRACK_PLAY_TIME,
    PLAYER_EVENT_VOLUME_CHANGE,
    PLAYER_EVENT_ERROR_MESSAGE,
    PLAYER_EVENT_GOTO_EXPLORER_WINDOW,
    PLAYER_EVENT_GOTO_PLAYER_WINDOW,
}
PLAYER_EVENT;

typedef enum{
    APP_STREAM_MP3,
    APP_STREAM_WMA,
    APP_STREAM_WAV,
    APP_STREAM_AAC,
    APP_STREAM_SPEEX,
    APP_STREAM_ADPCM,
    APP_STREAM_OPUS,
    APP_STREAM_FLAC,
}PLAYER_STREAM_TYPE;

typedef enum{
    APP_FILE_TYPE_WAV,
    APP_FILE_TYPE_MP3,
    APP_FILE_TYPE_WMA,
    APP_FILE_TYPE_AAC,
    APP_FILE_TYPE_MP4,
    APP_FILE_TYPE_UNKNOWN,
}APP_FILE_TYPE;

typedef enum{
    APP_DECODER_MP3,
    APP_DECODER_AAC,
    APP_DECODER_ADPCM,
    APP_DECODER_WMA,
    APP_DECODER_WAV,
    APP_DECODER_SPEEX,
    APP_DECODER_OPUS,
    APP_DECODER_FLAC,
    APP_DECODER_UNKNOWN,
}APP_DECODER_TYPE;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

//every 100ms
#define APP_REPEAT_TIMER_PERIOD                  (DRV_TMR_PRESCALE_IDX2 == 0x7)? \
                                                (SYS_CLK_BUS_PERIPHERAL_1/(1<<(DRV_TMR_PRESCALE_IDX2+1))/10): \
                                                (SYS_CLK_BUS_PERIPHERAL_1/(1<<DRV_TMR_PRESCALE_IDX2)/10)

#define AUDIO_QUEUEBUFFER_NUMBER 2
#define APP_MAKE_BUFFER_DMA_READY   __attribute__((aligned(16)))//__attribute__((coherent))

typedef uint32_t (*UpdatePlaytimeFuncPtr)();


typedef struct{
    int8_t buffer[DECODER_MAX_OUTPUT_BUFFER_SIZE];
    bool inUse;
    bool decoded;
    uint32_t offset;
    DRV_CODEC_BUFFER_HANDLE writeHandler;
    size_t bufferSize;
}AUDIO_QUEUEBUFFER;



typedef struct {
    /* SYS_FS File handle for 1st file */
    SYS_FS_HANDLE fileHandle;
    SYS_FS_FSTAT fileStatus;

    /* Application's current state */
    APP_PLAYER_STATE playerState;
    APP_USB_STATE state;

    /*Unfolding audio files*/
    uint16_t currentSongIdx;
    uint16_t nextSongIdx;
    uint16_t previousSongIdx;
    uint16_t totalAudioFiles;

    /* Number of bytes read */
    uint32_t nBytesRead;
    uint32_t diskCurrentFile;
    int32_t  fileSize;
    int32_t readBytes;
    uint32_t current_filesize;
    
    int8_t readIdx;
    uint8_t writeIdx;
    uint16_t playerDecodeDataSize;
    
    bool nextTrack;
    bool isDiskMounted;
    bool readbyte_flag;

    UpdatePlaytimeFuncPtr updatePlaytimeFunc;
    
    PLAYER_COMMAND pressCMD;
    PLAYER_STREAM_TYPE currentStreamType;
    /* repeat timer handle */
    DRV_HANDLE repeatTmrHandle;
    AUDIO_QUEUEBUFFER  audioBuffer[AUDIO_QUEUEBUFFER_NUMBER];
    uint8_t audioInput[DECODER_MAX_INPUT_BUFFER_SIZE];
}APP_AUDIOPLAYER ;

typedef struct{
    bool bStereoMode;
}APP_RUNDCPT; // application layer descriptor 


#define INVALID_BUTTON -1
int8_t  mRepeatButton;
int32_t mRepeatCount;

//extern APP_AUDIOPLAYER appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_PlayerInitialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_PlayerInitialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_PlayerInitialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */
void APP_Tasks();
void APP_Initialize();
void APP_InitCodec(void);
void APP_PlayerTasks(void);
void APP_PlayerReset(void);
void APP_PlayerReinit(void);
bool APP_PlayerDecode(uint8_t *ptr, int16_t* out);
bool APP_IsSupportedAudioFile(char *name);
void USB_Connection_Tasks (void);
void APP_ButtonInit(void);

void    APP_ButtonTask(void);
int8_t  APP_ButtonGetState(int8_t  button);

void APP_PlayerSendAudioData(void);
bool APP_PlayerFillDiskBuffer( uint8_t *ptr);
bool APP_PlayerCommand ( PLAYER_COMMAND cmd );
bool APP_PlayerEventHandler ( PLAYER_EVENT event, uint32_t data );
void APP_SetPlayerPlay(bool b);
void APP_SetPlayerTrackTotalTime(uint16_t totalTimeParam);
void APP_SetPlayerPlay(bool playerPlayParam);
void APP_ResetPlayerFileAdvanceStep();
 APP_DECODER_TYPE APP_PlayerGetCurrentFileType ( char *ext );
USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler (USB_HOST_EVENT event, void * eventData, uintptr_t context);
void APP_SYSFSEventHandler(SYS_FS_EVENT event, void * eventData, uintptr_t context);
void Player_SetSamplingRate(int data);
void Player_MuteOff(void);
void Player_MuteOn(void);
void APP_OnButtonEvent(uint8_t button, bool bButtonClosed, int32_t repeatCount);
void APP_handlePeriodicTimerSignal(uintptr_t context, uint32_t alarmCount);
void buttons_handleInterrupt(unsigned int newButtonState);
uint8_t APP_GetNextIdxInOutputBuffer(uint8_t idx);

// DISPLAY functions
//void GFX_MENU_DRAW(void);
//void APP_DisplayTasks(PLAYER_EVENT event, uint32_t data);

bool                APP_GetSpiAudioMode();
APP_AUDIOPLAYER*    APP_GetAppDataInstance();
int32_t             APP_GetReadBytesInAppData();
void                APP_SetSpiAudioMode(bool mode);
void                APP_SetReadBytesInAppData(int32_t val);
void                APP_SetReadFlagInAppData(bool b);
void                APP_SetReadBytesReadFlag(int32_t val, bool b);


#ifdef __cplusplus
}
#endif

#endif /* _APP_H */
/*******************************************************************************
 End of File
 */

