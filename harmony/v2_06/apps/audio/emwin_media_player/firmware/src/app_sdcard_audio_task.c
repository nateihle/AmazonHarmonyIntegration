/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_sdcard_audio_task.c

  Summary:
    This file contains the source code for managing the SD card and flash drive
    audio task.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016-2017 released Microchip Technology Inc.  All rights reserved.

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

#include "app_sdcard_audio_task.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************

#define APP_SDCARD_AUDIO_DEFAULT_VOLUME     170


uint32_t AppSdCardAudioContext;
int8_t AppSdCardAudioCardCurrentLFN[255];
static uint32_t AppDataAudioCardTotalBytesRead = 0;
APP_SDCARD_AUDIO_DATA AppSdCardAudioData;
APP_SDCARD_AUDIO_PLAYER *AppDataAudioPlayerPtr;
APP_SDCARD_AUDIO_CARD_TASK_DATA AppDataAudioCardData;
static APP_SDCARD_AUDIO_CARD_FILE_PATH AppSdCardAudioCardFilesTable[APP_SDCARD_AUDIO_CARD_MAX_FILES];
static APP_SDCARD_AUDIO_CARD_FILE_NODE AppDataAudioCardRootNode;
uint16_t playerDiskDataSize=0;

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler (USB_HOST_EVENT event, void * eventData, uintptr_t context)
{
    switch (event)
    {
        case USB_HOST_EVENT_DEVICE_UNSUPPORTED:            
            break;
        default:
            break;

    }
    return(USB_HOST_EVENT_RESPONSE_NONE);
}

void APP_SYSFSEventHandler(SYS_FS_EVENT event, void * eventData, uintptr_t context)
{    
    switch(event)
    {
        case SYS_FS_EVENT_MOUNT:
            if(0 == strcmp((const char *)eventData,SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0))
            {
                AppSdCardAudioData.mountedDeviceType = APP_STREAMING_SOURCE_SDCARD;                
            }
            else if(0 == strcmp((const char *)eventData,SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX0))
            {
                AppSdCardAudioData.isThumbDriveConnected = true;
                AppSdCardAudioData.isFSMountedOnThumbDrive = true;
                AppSdCardAudioData.mountedDeviceType = APP_STREAMING_SOURCE_USB;
            }

            AppSdCardAudioData.isDeviceConnected = true;
            break;

        case SYS_FS_EVENT_UNMOUNT:
                                   
            if(0 == strcmp((const char *)eventData,SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX0))
            {
                AppSdCardAudioData.isThumbDriveConnected = false;
                AppSdCardAudioData.isFSMountedOnThumbDrive = false;
                if (APP_STREAMING_SOURCE_USB == APP_StreamSourceGet())
                {
                    AppSdCardAudioData.state = APP_SDCARD_AUDIO_THUMBDRIVE_FS_UNMOUNT;
                }
            }         
            
            AppSdCardAudioData.isDeviceConnected = false;
            
            break;

        default:
            break;
    }
}

void APP_SDCARD_AUDIO_RegisterEventHandler(APP_SDCARD_AUDIO_EVENT_NOTIFY evHandler)
{
    if (evHandler)
    {
        AppSdCardAudioData.player.eventNotify = evHandler;
    }
}

void APP_SDCARD_AUDIO_Initialize(void)
{
    SYS_FS_EventHandlerSet(APP_SYSFSEventHandler, (uintptr_t)NULL);
    
    AppSdCardAudioData.state = APP_SDCARD_AUDIO_USB_BUS_ENABLE;            
    AppSdCardAudioData.codec.handle = DRV_HANDLE_INVALID;
    AppSdCardAudioData.codec.context = (uintptr_t)&AppSdCardAudioContext;
    AppSdCardAudioData.codec.bufferHandler = 
            (DRV_CODEC_BUFFER_EVENT_HANDLER) APP_SDCARD_AUDIO_BufferEventHandler;          
    APP_SDCARD_AUDIO_Player_Initialize();     
    
}

///////////////////////////////////////////////////////////////////////////////
void APP_SDCARD_AUDIO_Player_Initialize(void)
{    
    AppSdCardAudioData.player.state = APP_SDCARD_AUDIO_PLAYER_STATE_SEND_AUDIO_DATA;
    AppSdCardAudioData.player.fileHandle = SYS_FS_HANDLE_INVALID;
    AppSdCardAudioData.player.fileStatus.lfname = (char*)AppSdCardAudioCardCurrentLFN;
    AppSdCardAudioData.player.fileStatus.lfsize = FAT_FS_MAX_LFN;
    AppSdCardAudioData.player.nextTrack = false;
    AppSdCardAudioData.player.readBytes = 0;
    AppSdCardAudioData.player.currentFilesize = 0;
    AppSdCardAudioData.player.isPlayerRunning = false;
    AppSdCardAudioData.player.nTotalBytesPlayed = 0;
    AppSdCardAudioData.player.playerTrackTotalTime = 0;    
    AppSdCardAudioData.player.pressCMD = APP_SDCARD_AUDIO_PLAYER_COMMAND_NONE;        
    APP_SDCARD_AUDIO_ResetBuffers();      
    AppSdCardAudioData.player.WAV_decoder_enabled  = isWAVdecoder_enabled();    
    APP_SDCARD_AUDIO_Player_ReInitialize();     
    AppDataAudioPlayerPtr = APP_SDCARD_AUDIO_Player_GetInstance();        
    APP_SDCARD_AUDIO_ResetTrackPlayTime();                  //3-
    AppSdCardAudioData.player.eventNotify(DISP_CMD_PLAYER_PAUSE_REQ);        //1
    AppSdCardAudioData.player.eventNotify(DISP_CMD_CLEAR_TRACKLIST_REQ);     //2
    AppSdCardAudioData.player.eventNotify(DISP_CMD_UPDATE_TOTAL_TRACK_TIME_REQ);//4
}

///////////////////////////////////////////////////////////////////////////////
void APP_SDCARD_AUDIO_Player_ReInitialize(void)
{
    AppSdCardAudioData.player.fileSize = 0;
    AppSdCardAudioData.player.decodeDataSize = 0;    
    
    AppSdCardAudioData.player.playerTrackTotalTime = 0;    
    AppSdCardAudioData.player.playerBitrate = 0;    
    APP_SDCARD_AUDIO_ResetBuffers();    
}

///////////////////////////////////////////////////////////////////////////////
void APP_SDCARD_AUDIO_Player_Reset(void)
{
    uint8_t i=0;    
    uint8_t decoderType;
    AppSdCardAudioData.player.decodeDataSize = 0;    
    AppSdCardAudioData.player.playerTrackTotalTime = 0;    
    AppSdCardAudioData.player.playerBitrate = 0;    
    APP_SDCARD_AUDIO_ResetBuffers();    
    do
    {
       i++;
    } while(AppSdCardAudioData.player.fileStatus.fname[i]!= '.');    
    decoderType = APP_SDCARD_AUDIO_Player_GetCurrentFileType 
            ((int8_t*)(&(AppSdCardAudioData.player.fileStatus.fname[i+1])));
    DECODER_Initialize (decoderType);
}


///////////////////////////////////////////////////////////////////////////////
void APP_SDCARD_AUDIO_Tasks(void)
{    
    switch(AppSdCardAudioData.state)
    {
        case APP_SDCARD_AUDIO_USB_BUS_ENABLE:

            USB_HOST_BusEnable(0);
            AppSdCardAudioData.state = APP_SDCARD_AUDIO_WAIT_FOR_USB_BUS_ENABLE_COMPLETE;
            break;

        case APP_SDCARD_AUDIO_WAIT_FOR_USB_BUS_ENABLE_COMPLETE:
            if(USB_HOST_BusIsEnabled(0))
            {
                AppSdCardAudioData.state = APP_SDCARD_AUDIO_CARD_MOUNT;
            }
            break;

        case APP_SDCARD_AUDIO_CARD_MOUNT:

            if ((true == AppSdCardAudioData.isDeviceConnected) && 
                (APP_StreamSourceGet() == AppSdCardAudioData.mountedDeviceType)                    
            )            
            {
                AppSdCardAudioData.state = APP_SDCARD_AUDIO_CARD_CURRENT_DRIVE_SET;
            }                
            break;  

        case APP_SDCARD_AUDIO_CARD_CURRENT_DRIVE_SET:

            APP_SDCARD_AUDIO_Card_Initialize();                              
            /* Open a file for reading. */
            AppSdCardAudioData.state = APP_SDCARD_AUDIO_STATE_CODEC_OPEN;  

            break;

        case APP_SDCARD_AUDIO_STATE_CODEC_OPEN:            
            /* A client opens the driver object to get an Handle */
            AppSdCardAudioData.codec.handle = DRV_CODEC_Open(DRV_CODEC_INDEX_0, 
                    DRV_IO_INTENT_WRITE);
            if(AppSdCardAudioData.codec.handle != DRV_HANDLE_INVALID)
            {
                AppSdCardAudioData.state = 
                        APP_SDCARD_AUDIO_STATE_CODEC_SET_BUFFER_HANDLER;
            }
            else
            {
                /* Got an Invalid Handle.  Wait for AK4384 to Initialize */
                ;
            }           
            break;

        /* Set a handler for the audio buffer completion event */
        case APP_SDCARD_AUDIO_STATE_CODEC_SET_BUFFER_HANDLER:

            DRV_CODEC_BufferEventHandlerSet(AppSdCardAudioData.codec.handle,
                    AppSdCardAudioData.codec.bufferHandler,
                    AppSdCardAudioData.codec.context);

            APP_SDCARD_AUDIO_VolumeSet(APP_SDCARD_AUDIO_DEFAULT_VOLUME);
            AppSdCardAudioData.player.eventNotify(DISP_CMD_VOLUME_CHANGE_REQ);

            AppSdCardAudioData.state = APP_SDCARD_AUDIO_STATE_RUNNING;            

            break;

        case APP_SDCARD_AUDIO_STATE_RUNNING: 

            APP_SDCARD_AUDIO_Card_Tasks();
            switch(AppSdCardAudioData.player.state)
            {
                case APP_SDCARD_AUDIO_PLAYER_STATE_RUNNING:

                    APP_SDCARD_AUDIO_FillOutputBuffer();                                                            
                    break;

                case APP_SDCARD_AUDIO_PLAYER_STATE_TRACK_CHANGE:

                    APP_SDCARD_AUDIO_ResetTrackPlayTime();
                    // Goto next state, wait for all previous buffers complete.
                    AppSdCardAudioData.player.state = 
                            APP_SDCARD_AUDIO_PLAYER_STATE_WAIT_FOR_BUFFER_COMPLETION;

                    break;            


                case APP_SDCARD_AUDIO_PLAYER_STATE_WAIT_FOR_BUFFER_COMPLETION:
                    // wait the buffers in I2S are all played
                    if(APP_SDCARD_AUDIO_hasAllBuffersCompleted())
                    {
                        if(AppSdCardAudioData.player.nextTrack)
                        {                            
                            APP_PlayerCommand(APP_SDCARD_AUDIO_PLAYER_COMMAND_NEXT_FILE);                            
                            AppSdCardAudioData.player.eventNotify(DISP_CMD_TRACK_CHANGE_REQ);                            
                            AppSdCardAudioData.player.nextTrack = false;
                        }                        
                        else if (AppSdCardAudioData.player.isPlayerRunning == false)
                        {
                            AppSdCardAudioData.player.state = APP_SDCARD_AUDIO_PLAYER_STATE_STOP;
                        }                                                        
                    }                    
                    break;

                case APP_SDCARD_AUDIO_PLAYER_STATE_STOP:
                    if (AppSdCardAudioData.player.isPlayerRunning == true)
                    {
                        AppSdCardAudioData.player.state = APP_SDCARD_AUDIO_PLAYER_STATE_RUNNING;
                    }                    
                    break;

                default:
                    break;
            }              
            break;
        case APP_SDCARD_AUDIO_THUMBDRIVE_FS_UNMOUNT:
            if (APP_STREAMING_SOURCE_USB == APP_StreamSourceGet())
            {
                APP_SDCARD_AUDIO_SuspendStreaming();  
                APP_StreamSourceSet(APP_STREAMING_SOURCE_SDCARD);                    
                APP_SDCARD_AUDIO_Initialize();                
                //change the UI to SD card mode
                AppSdCardAudioData.player.eventNotify(DISP_CMD_SDCARD_MODE_ON_REQ);                
            }
            
            break;

        case APP_SDCARD_AUDIO_ERROR:

            //Handle error condition..

            break;

        default:
            break;
    }        
}

void APP_SDCARD_AUDIO_MountFileSysOnSDCard(void)
{
    AppSdCardAudioData.isDeviceConnected = false;
    
    if (SYS_FS_RES_SUCCESS == SYS_FS_Mount(SYS_FS_MEDIA_IDX0_DEVICE_NAME_VOLUME_IDX0, SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0, FAT, 0, NULL))
    {
        AppSdCardAudioData.isDeviceConnected = true;
        AppSdCardAudioData.mountedDeviceType = APP_STREAMING_SOURCE_SDCARD;
    }
}

void APP_SDCARD_AUDIO_MountFileSysOnUSB(void)
{        
    AppSdCardAudioData.isDeviceConnected = false;
    
    if (true == AppSdCardAudioData.isThumbDriveConnected)
    {
        /*Mount FS only if the thumb-drive is still connected and no FS is currently mounted on thumb-drive*/
        if (false == AppSdCardAudioData.isFSMountedOnThumbDrive)
        {
            if (SYS_FS_RES_SUCCESS == SYS_FS_Mount(SYS_FS_MEDIA_IDX1_DEVICE_NAME_VOLUME_IDX0, SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX0, FAT, 0, NULL))
            {
                AppSdCardAudioData.isDeviceConnected = true;
                AppSdCardAudioData.isFSMountedOnThumbDrive = true;
                AppSdCardAudioData.mountedDeviceType = APP_STREAMING_SOURCE_USB;
            }
        }
        else
        {
            /*Thumb-drive is connected and FS is mounted*/
            AppSdCardAudioData.isDeviceConnected = true;            
            AppSdCardAudioData.mountedDeviceType = APP_STREAMING_SOURCE_USB;
        }
    }
    else
    {
        //Thumb-drive is not connected. 
        //The auto-mount feature will mount the file system once the thumb-drive is connected.
    }
}

void APP_SDCARD_AUDIO_SuspendStreaming(void)
{                  
    DRV_CODEC_Close(AppSdCardAudioData.codec.handle);   
    AppSdCardAudioData.codec.handle = DRV_HANDLE_INVALID;        
                
    APP_SDCARD_AUDIO_Card_CloseFile(AppSdCardAudioData.player.fileHandle);
            
    if (APP_STREAMING_SOURCE_SDCARD == APP_StreamSourceGet())
    {
        SYS_FS_Unmount(SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0);        
        APP_SDCARD_AUDIO_MountFileSysOnUSB();                        
    }
    else
    {
        SYS_FS_Unmount(SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX0);  
        AppSdCardAudioData.isFSMountedOnThumbDrive = false;
        APP_SDCARD_AUDIO_MountFileSysOnSDCard();            
    }  
}

bool APP_SDCARD_AUDIO_FillOutputBuffer(void)
{
    uint8_t audioInput[DECODER_MAX_INPUT_BUFFER_SIZE];
    int8_t *outputptr = NULL;

    while( !APP_SDCARD_AUDIO_isOutputBufferFull() )
    {
        outputptr = AppSdCardAudioData.player.buffer
                    [AppSdCardAudioData.player.writeIdx].buffer;
        if(APP_SDCARD_AUDIO_Card_FillBuffer(audioInput)==true)
        {
            if(APP_SDCARD_AUDIO_Player_Decode (audioInput, (int16_t*) outputptr)==true)//
            {
                if(AppSdCardAudioData.player.decodeDataSize != 0)
                {
                    AppSdCardAudioData.player.buffer
                            [AppSdCardAudioData.player.writeIdx].decoded = true;
                    AppSdCardAudioData.player.buffer
                            [AppSdCardAudioData.player.writeIdx].bufferSize = 
                            AppSdCardAudioData.player.decodeDataSize;                    
                    AppSdCardAudioData.player.writeIdx = 
                            APP_SDCARD_AUDIO_GetNextIdxInOutputBuffer(AppSdCardAudioData.player.writeIdx);
                    AppSdCardAudioData.player.decodeDataSize = 0;
                }
                else
                {                    
                    return false;
                }
                if (AppSdCardAudioData.player.isPlayerRunning == true)
                {
                    AppSdCardAudioData.player.state = APP_SDCARD_AUDIO_PLAYER_STATE_RUNNING;
                }
                APP_SDCARD_AUDIO_UpdateElapsedTrackTimeInBytes(WAV_INPUT_BUFFER_SIZE);
            }
            else
            {
                AppSdCardAudioData.player.nextTrack = true;
                AppSdCardAudioData.player.state = 
                        APP_SDCARD_AUDIO_PLAYER_STATE_TRACK_CHANGE;

                return false;
            }
        }
        else
        {
            AppSdCardAudioData.player.nextTrack = true;
            AppSdCardAudioData.player.state = APP_SDCARD_AUDIO_PLAYER_STATE_TRACK_CHANGE;
            return false;
        }
    }        
    if(APP_SDCARD_AUDIO_isNoOutputBufferInUse())
    {
        int8_t readIdx = AppSdCardAudioData.player.readIdx; 
        
        DRV_CODEC_BufferAddWrite(
                AppSdCardAudioData.codec.handle, 
                &AppSdCardAudioData.codec.writeBufHandle,
                AppSdCardAudioData.player.buffer[readIdx].buffer,
                AppSdCardAudioData.player.buffer[readIdx].bufferSize
        );
        if(AppSdCardAudioData.codec.writeBufHandle != DRV_CODEC_BUFFER_HANDLE_INVALID)
        {
            AppSdCardAudioData.player.buffer[readIdx].writeHandler = AppSdCardAudioData.codec.writeBufHandle;
            AppSdCardAudioData.player.buffer[readIdx].inUse = true;
            AppSdCardAudioData.player.readIdx = APP_SDCARD_AUDIO_GetNextIdxInOutputBuffer(readIdx);             
        }
    }    
    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool APP_SDCARD_AUDIO_isOutputBufferFull(void)
{
    int i;
    
    for(i = 0; i < APP_SDCARD_AUDIO_PLAYER_QUEUE_BUFFER_SIZE; i++)
    {
        if(!AppSdCardAudioData.player.buffer[i].decoded)
        {
            return false;
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool APP_SDCARD_AUDIO_isNoOutputBufferInUse(void)
{
    int i;
    
    for(i = 0; i < APP_SDCARD_AUDIO_PLAYER_QUEUE_BUFFER_SIZE;i++)
    {
        if(AppSdCardAudioData.player.buffer[i].inUse)
        {
            return false;
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool APP_SDCARD_AUDIO_hasAllBuffersCompleted(void)
{
    int i;
    
    for(i = 0; i < APP_SDCARD_AUDIO_PLAYER_QUEUE_BUFFER_SIZE; i++)
    {
        if(AppSdCardAudioData.player.buffer[i].writeHandler != DRV_CODEC_BUFFER_HANDLE_INVALID)
        {
            return false;
        }
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////
void APP_SDCARD_AUDIO_ResetBuffers(void)
{
    int i = 0;
    
    for(i=0; i < APP_SDCARD_AUDIO_PLAYER_QUEUE_BUFFER_SIZE; i++)
    {
        AppSdCardAudioData.player.buffer[i].inUse = false;
        AppSdCardAudioData.player.buffer[i].decoded = false;
        AppSdCardAudioData.player.buffer[i].writeHandler = DRV_CODEC_BUFFER_HANDLE_INVALID;
        AppSdCardAudioData.player.readIdx = 0;
        AppSdCardAudioData.player.writeIdx = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////
uint8_t APP_SDCARD_AUDIO_GetNextIdxInOutputBuffer(uint8_t idx)
{
    return (uint8_t)((idx+1) % APP_SDCARD_AUDIO_PLAYER_QUEUE_BUFFER_SIZE);
}

////////////////////////////////////////////////////////////////////////////////
void APP_SDCARD_AUDIO_ClearBufferFlags(DRV_CODEC_BUFFER_HANDLE handle)
{
    int8_t releaseIdx = APP_SDCARD_AUDIO_GetCompletedBufferIdx(handle);
    if(releaseIdx == -1)
    {
        return;
    }
    APP_SDCARD_AUDIO_PLAYER *AppDataAudioPlayerPtr = APP_SDCARD_AUDIO_Player_GetInstance();
    AppDataAudioPlayerPtr->buffer[releaseIdx].inUse = false;
    AppDataAudioPlayerPtr->buffer[releaseIdx].bufferSize = 0;
    AppDataAudioPlayerPtr->buffer[releaseIdx].decoded = false;
    AppDataAudioPlayerPtr->buffer[releaseIdx].writeHandler = DRV_CODEC_BUFFER_HANDLE_INVALID;
}

////////////////////////////////////////////////////////////////////////////////
int8_t APP_SDCARD_AUDIO_GetCompletedBufferIdx(DRV_CODEC_BUFFER_HANDLE handle)
{
    int8_t i;
    APP_SDCARD_AUDIO_PLAYER *AppDataAudioPlayerPtr = APP_SDCARD_AUDIO_Player_GetInstance();
    for(i = 0; i < APP_SDCARD_AUDIO_PLAYER_QUEUE_BUFFER_SIZE; i++)
    {
        if(AppDataAudioPlayerPtr->buffer[i].writeHandler == handle)
        {
            return i;
        }
    }
    // something wrong
    return -1;
}

////////////////////////////////////////////////////////////////////////////////
APP_SDCARD_AUDIO_PLAYER* APP_SDCARD_AUDIO_Player_GetInstance(void)
{
    return &AppSdCardAudioData.player;
}

////////////////////////////////////////////////////////////////////////////////
bool APP_SDCARD_AUDIO_Player_Decode(uint8_t *ptr, int16_t* outputptr) 
{
    uint16_t read;
    if (AppSdCardAudioData.player.decodeDataSize == 0) 
    {
        if (DECODER_Decode(ptr, playerDiskDataSize, &read, outputptr, 
                &AppSdCardAudioData.player.decodeDataSize) == true) 
        {
            if (read != 0) 
            {
                playerDiskDataSize -= read;
            }
            return true;
        } 
        else 
        {
            return false;
        }
    } 
    else 
    {
        AppSdCardAudioData.player.nextTrack = true;
        return false;
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////
APP_SDCARD_AUDIO_DECODER APP_SDCARD_AUDIO_Player_GetCurrentFileType(int8_t *ext)
{      
    if (( ext[0] == 'M' && ext[1] == 'P' && ext[2] == '3' )||
        ( ext[0] == 'm' && ext[1] == 'p' && ext[2] == '3' ))
    {
        return APP_SDCARD_AUDIO_DECODER_MP3;
    }
    if (( ext[0] == 'W' && ext[1] == 'A' && ext[2] == 'V' )||
        ( ext[0] == 'w' && ext[1] == 'a' && ext[2] == 'v' ))
    {
        return APP_SDCARD_AUDIO_DECODER_WAV;
    } 
    return APP_SDCARD_AUDIO_DECODER_UNKNOWN;
}

////////////////////////////////////////////////////////////////////////////////
bool DECODER_EventHandler ( DECODER_EVENT event, uint32_t data )
{
    switch ( event )
    {
        case DECODER_EVENT_TAG_ARTIST:          
            return ( true );

        case DECODER_EVENT_TAG_ALBUM:           
            return ( true );

        case DECODER_EVENT_TAG_TITLE:          
            return ( true );

        case DECODER_EVENT_STREAM_START:                             
            return ( true );

        case DECODER_EVENT_SAMPLERATE:
            if(AppSdCardAudioData.codec.handle != DRV_HANDLE_INVALID)
            {
                DRV_CODEC_SamplingRateSet(AppSdCardAudioData.codec.handle, data);
            }                      
            return ( true );
        
        case DECODER_EVENT_BITRATE:
            AppSdCardAudioData.player.playerBitrate = data;            
            return ( true );

        case DECODER_EVENT_TRACK_TIME:
            AppSdCardAudioData.player.playerTrackTotalTime = data; 
            AppSdCardAudioData.player.eventNotify(DISP_CMD_UPDATE_TOTAL_TRACK_TIME_REQ);            
            return ( true );

        case DECODER_INITIALIZATION_ERROR:            
            APP_PlayerCommand(APP_SDCARD_AUDIO_PLAYER_COMMAND_NEXT_FILE);
            break;
    }
    return ( false );
}

////////////////////////////////////////////////////////////////////////////////
bool APP_PlayerCommand ( APP_SDCARD_AUDIO_PLAYER_COMMAND cmd )
{       
    // If there is no audio files, return
    if(AppSdCardAudioData.player.state == APP_SDCARD_AUDIO_PLAYER_STATE_NO_FILE)
    {
        return true;
    }

    switch ( cmd )
    {
        case APP_SDCARD_AUDIO_PLAYER_COMMAND_NEXT_FILE:
            
            APP_SDCARD_AUDIO_Card_CloseFile(AppSdCardAudioData.player.fileHandle);
            if (AppSdCardAudioData.player.isPlayerRunning == true)
            {
                AppSdCardAudioData.player.state = APP_SDCARD_AUDIO_PLAYER_STATE_RUNNING;
            }
            
            return APP_SDCARD_AUDIO_Card_NextTrack ();
            
            break;
                
        default:
            break;
    }
    return ( true );
}


/**********************************************************
 * Application CODEC buffer Event handler.
 ***********************************************************/
void APP_SDCARD_AUDIO_BufferEventHandler(
    DRV_CODEC_BUFFER_EVENT event,
    DRV_CODEC_BUFFER_HANDLE handle, 
    uintptr_t context 
)
{              
    switch(event)
    {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        {                
            if(AppDataAudioPlayerPtr->state == APP_SDCARD_AUDIO_PLAYER_STATE_RUNNING)
            {
                int8_t readIdx = AppDataAudioPlayerPtr->readIdx;
                if(AppDataAudioPlayerPtr->buffer[readIdx].decoded && !AppDataAudioPlayerPtr->buffer[readIdx].inUse)
                {            
                    if (AppSdCardAudioData.codec.handle != DRV_HANDLE_INVALID)
                    {
                        DRV_CODEC_BufferAddWrite(AppSdCardAudioData.codec.handle, 
                            &AppSdCardAudioData.codec.writeBufHandle,
                            AppDataAudioPlayerPtr->buffer[readIdx].buffer,
                            AppDataAudioPlayerPtr->buffer[readIdx].bufferSize);
                    }
                    
                    if(AppSdCardAudioData.codec.writeBufHandle != DRV_CODEC_BUFFER_HANDLE_INVALID)
                    {                        
                        AppDataAudioPlayerPtr->buffer[readIdx].writeHandler = AppSdCardAudioData.codec.writeBufHandle;
                        AppDataAudioPlayerPtr->buffer[readIdx].inUse = true;
                        AppDataAudioPlayerPtr->readIdx = APP_SDCARD_AUDIO_GetNextIdxInOutputBuffer(readIdx);            
                    }                    
                }
                else
                {
                    // underrun
                    Nop();
                }
            }
            APP_SDCARD_AUDIO_ClearBufferFlags(handle);                                        
        }
        break;
        case DRV_CODEC_BUFFER_EVENT_ERROR:
        {
        } 
        break;

        case DRV_CODEC_BUFFER_EVENT_ABORT:
        {
        } 
        break;
    }
}

/*******************************************************************************
 * Application SD CARD Functions
 ******************************************************************************/

void APP_SDCARD_AUDIO_Card_Initialize ( void )
{
    AppDataAudioCardRootNode.fstat.fattrib = SYS_FS_ATTR_DIR;
    if (APP_STREAMING_SOURCE_SDCARD == AppSdCardAudioData.mountedDeviceType)
    {
        strcpy(AppDataAudioCardRootNode.path, SYS_FS_MEDIA_IDX0_MOUNT_NAME_VOLUME_IDX0);
    }
    else if (APP_STREAMING_SOURCE_USB == AppSdCardAudioData.mountedDeviceType)
    {
        strcpy(AppDataAudioCardRootNode.path, SYS_FS_MEDIA_IDX1_MOUNT_NAME_VOLUME_IDX0);
    }
    AppDataAudioCardData.state = APP_SDCARD_AUDIO_CARD_STATE_INIT;                                        
}

////////////////////////////////////////////////////////////////////////////////
int32_t APP_SDCARD_AUDIO_Card_GetCurrentFilePosition(void)
{
    return SYS_FS_FileTell(AppDataAudioPlayerPtr->fileHandle);
}

////////////////////////////////////////////////////////////////////////////////
uint32_t APP_SDCARD_AUDIO_Card_ReadCurrentFile(uint8_t *ptr, size_t readSize)
{
    uint32_t ret = 0;
    AppDataAudioPlayerPtr->nBytesRead = 
            SYS_FS_FileRead(AppDataAudioPlayerPtr->fileHandle,ptr,readSize);
    if ((AppDataAudioPlayerPtr->nBytesRead != readSize))
    {
        ret = 0;
        AppDataAudioCardTotalBytesRead = 0;
    }
    else
    {
        AppDataAudioPlayerPtr->nBytesRead =0;
        return readSize;
    }    
    return ret;
}

////////////////////////////////////////////////////////////////////////////////
int32_t APP_SDCARD_AUDIO_Card_GetFilePosition (SYS_FS_HANDLE fileHandle )
{
    int32_t tell;
    tell = SYS_FS_FileTell(fileHandle);
    return ( tell );
}

////////////////////////////////////////////////////////////////////////////////
uint32_t APP_SDCARD_AUDIO_Card_GetFileSize(SYS_FS_HANDLE fileHandle)
{
    AppDataAudioPlayerPtr->fileSize = SYS_FS_FileSize( fileHandle );

    if(AppDataAudioPlayerPtr->fileSize == -1)
    {
        /* Reading file size was a failure */
        return (uint32_t) SYS_FS_HANDLE_INVALID;
    }
    else
    {
        return (uint32_t)AppDataAudioPlayerPtr->fileSize;
    }
}

////////////////////////////////////////////////////////////////////////////////
bool APP_SDCARD_AUDIO_Card_SetFilePosition (SYS_FS_HANDLE fileHandle, int32_t pos )
{
    int status;
    status = SYS_FS_FileSeek(fileHandle, pos, SYS_FS_SEEK_SET);
    if( status == pos )
    {
        return true;
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////
bool APP_SDCARD_AUDIO_Card_FillBuffer(uint8_t *ptr)
{
    if ((AppDataAudioCardTotalBytesRead < AppDataAudioPlayerPtr->currentFilesize)) 
    {
        if ((AppDataAudioPlayerPtr->readbyte_flag)) 
        {
            //bool FileEnd = false;            
            AppDataAudioPlayerPtr->nBytesRead = 
                    SYS_FS_FileRead(AppDataAudioPlayerPtr->fileHandle, ptr, 
                    AppDataAudioPlayerPtr->readBytes);
            //FileEnd = APP_SDCARD_AUDIO_Card_EndOfFile();
            if ((AppDataAudioPlayerPtr->nBytesRead == -1) /* || ( FileEnd == true) */) 
            {
                AppDataAudioCardTotalBytesRead = 0;
                SYS_FS_FileClose(AppDataAudioPlayerPtr->fileHandle);
                return false;
            }
            else if (APP_SDCARD_AUDIO_Card_EndOfFile() == true)
            {
                if(AppDataAudioPlayerPtr->nBytesRead > 0 && 
                        AppDataAudioPlayerPtr->nBytesRead < AppDataAudioPlayerPtr->readBytes)
                {
                    AppDataAudioCardTotalBytesRead += AppDataAudioPlayerPtr->nBytesRead;
                    playerDiskDataSize += AppDataAudioPlayerPtr->nBytesRead;
                    AppDataAudioPlayerPtr->nBytesRead = 0;                    
                }
                else
                {
                    AppDataAudioCardTotalBytesRead = 0;
                    SYS_FS_FileClose(AppDataAudioPlayerPtr->fileHandle);
                    return false;                    
                }
            }
            else 
            {
                AppDataAudioCardTotalBytesRead += AppDataAudioPlayerPtr->nBytesRead;
                playerDiskDataSize += AppDataAudioPlayerPtr->nBytesRead;
                AppDataAudioPlayerPtr->nBytesRead = 0;
            }
        }
    } 
    else 
    {
        AppDataAudioCardTotalBytesRead = 0;
        return false;
    }
    return true;
}


////////////////////////////////////////////////////////////////////////////////
bool APP_SDCARD_AUDIO_Card_OpenTrack ( const int8_t *fname )
{
    AppDataAudioPlayerPtr->fileHandle = APP_SDCARD_AUDIO_Card_OpenFile(fname);         

    if ( AppDataAudioPlayerPtr->fileHandle != SYS_FS_HANDLE_INVALID )
    {        
        if(APP_SDCARD_AUDIO_Card_FileStatus(fname) == true)
        {
            AppDataAudioPlayerPtr->currentFilesize = 
                APP_SDCARD_AUDIO_Card_GetFileSize(AppDataAudioPlayerPtr->fileHandle);
            APP_SDCARD_AUDIO_Card_EventHandler ( APP_SDCARD_AUDIO_CARD_EVENT_TRACK_CHANGED, 
                    AppDataAudioPlayerPtr->diskCurrentFile, AppDataAudioPlayerPtr->fileHandle );            
            AppDataAudioCardTotalBytesRead = 
                    APP_SDCARD_AUDIO_Card_GetFilePosition(AppDataAudioPlayerPtr->fileHandle);
            return true;
        }else
        {
              return false;
        }        
    }
    else
    {
        APP_SDCARD_AUDIO_Card_EventHandler ( APP_SDCARD_AUDIO_CARD_EVENT_FILE_OPEN_ERROR, 
                AppDataAudioPlayerPtr->diskCurrentFile,AppDataAudioPlayerPtr->fileHandle );        
        return ( false );
    }  
        
    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool APP_SDCARD_AUDIO_Card_FileStatus(const int8_t* fname)
{
    if( SYS_FS_FileStat((const char*)fname, &(AppDataAudioPlayerPtr->fileStatus)) == SYS_FS_RES_FAILURE )
    {
            return false;
    }
    else
    {
        // Read name
        bool stat = APP_SDCARD_AUDIO_Card_FileNameGet(AppDataAudioPlayerPtr->fileHandle, 
                (int8_t*)AppDataAudioPlayerPtr->fileStatus.lfname);
       /* Read file size */
        return stat;
    }
    return true;
}

////////////////////////////////////////////////////////////////////////////////
bool APP_SDCARD_AUDIO_Card_FileNameGet(SYS_FS_HANDLE handle, int8_t* cName)
{
    bool stat = SYS_FS_FileNameGet(handle, (uint8_t*) cName, 255);
    return stat;
}

bool APP_SDCARD_AUDIO_Card_EndOfFile(void)
{    
    if(SYS_FS_FileEOF( AppDataAudioPlayerPtr->fileHandle ) == false )
    {
        return false;
    }
    else
    {
        return true;
    }
}

////////////////////////////////////////////////////////////////////////////////
SYS_FS_HANDLE APP_SDCARD_AUDIO_Card_OpenFile ( const int8_t *fname )
{
    SYS_FS_HANDLE fileHandle;
    
    if ( AppDataAudioCardData.state != APP_SDCARD_AUDIO_CARD_STATE_RUNNING && 
            AppDataAudioCardData.state != APP_SDCARD_AUDIO_CARD_STATE_NO_AUDIO_FILES 
            && AppDataAudioCardData.state != APP_SDCARD_AUDIO_CARD_STATE_SCANNING)
    {
        return ( SYS_FS_HANDLE_INVALID );
    }

    AppDataAudioCardTotalBytesRead = 0;
    playerDiskDataSize=0;

    fileHandle = SYS_FS_FileOpen((const char*)fname, (SYS_FS_FILE_OPEN_READ_PLUS));
    if(fileHandle == SYS_FS_HANDLE_INVALID)     
    {
        return SYS_FS_HANDLE_INVALID; 
    }
    else
    {
        return fileHandle;
    }
}

////////////////////////////////////////////////////////////////////////////////
void APP_SDCARD_AUDIO_Card_Tasks(void)
{        
    switch ( AppDataAudioCardData.state )
    {        
        case APP_SDCARD_AUDIO_CARD_STATE_INIT:
            {
                AppDataAudioCardData.state = APP_SDCARD_AUDIO_CARD_STATE_SCANNING;

                AppDataAudioPlayerPtr->totalAudioFiles = 0;
                AppDataAudioPlayerPtr->currentSongIdx  = 0;
                AppDataAudioPlayerPtr->nextSongIdx     = 0;                
                
                AppDataAudioPlayerPtr->isDiskMounted = true;
                APP_SDCARD_AUDIO_Card_EventHandler ( APP_SDCARD_AUDIO_CARD_EVENT_SCANNING_STARTED, 
                        0, AppDataAudioPlayerPtr->fileHandle);                
                
                APP_SDCARD_AUDIO_Card_TraverseAllFiles(AppDataAudioCardRootNode, true);
                if(AppDataAudioPlayerPtr->totalAudioFiles == 0)
                {
                    // No Audio File
                    AppDataAudioCardData.state = APP_SDCARD_AUDIO_CARD_STATE_NO_AUDIO_FILES;
                    AppDataAudioPlayerPtr->isSongListAvailable = false;
                }
                else
                {
                    AppDataAudioPlayerPtr->isSongListAvailable = true;
                    AppDataAudioPlayerPtr->currentSongIdx = 0;
                    //AppDataAudioPlayerPtr->state = APP_SDCARD_AUDIO_PLAYER_STATE_RUNNING;
                    AppDataAudioPlayerPtr->state = APP_SDCARD_AUDIO_PLAYER_STATE_STOP;                    
                    
                    if(APP_SDCARD_AUDIO_Card_OpenTrack(AppSdCardAudioCardFilesTable[0].path))
                    {                                                
                        DRV_CODEC_MuteOff(AppSdCardAudioData.codec.handle);   
                        AppSdCardAudioData.player.eventNotify(DISP_CMD_MUTE_OFF_REQ);
                    }
                }
                AppSdCardAudioData.player.eventNotify(DISP_CMD_POPULATE_TRACKLIST_REQ);
            }
            break;

        case APP_SDCARD_AUDIO_CARD_STATE_SCANNING:
            break;
           
        case APP_SDCARD_AUDIO_CARD_STATE_RUNNING:
            break;

        case APP_SDCARD_AUDIO_CARD_STATE_NO_AUDIO_FILES:
            AppDataAudioPlayerPtr->noAudioFiles = true;
            AppDataAudioPlayerPtr->state = APP_SDCARD_AUDIO_PLAYER_STATE_NO_FILE;            
            //APP_PlayerEventHandler(PLAYER_EVENT_TAG_TITLE, (uint32_t)"No Available Audio Files" );
            break;
        default:
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////
bool APP_SDCARD_AUDIO_Card_EventHandler 
( 
    APP_SDCARD_AUDIO_CARD_EVENT event, 
    uint32_t variable,
    SYS_FS_HANDLE fileHandle
)
{
    switch ( event )
    {
        case APP_SDCARD_AUDIO_CARD_EVENT_REMOVED:
        
            APP_SDCARD_AUDIO_Player_ReInitialize();
            AppDataAudioPlayerPtr->state = APP_SDCARD_AUDIO_PLAYER_STATE_NO_MEDIA;       
            return ( true );

        case APP_SDCARD_AUDIO_CARD_EVENT_INSERTED:
        
            AppDataAudioPlayerPtr->state = APP_SDCARD_AUDIO_PLAYER_STATE_WAITING;        
            return ( true );
        

        case APP_SDCARD_AUDIO_CARD_EVENT_SCANNING_STARTED:
        
            AppDataAudioPlayerPtr->state = APP_SDCARD_AUDIO_PLAYER_STATE_SCANNING;            
            return ( true );
        

        case APP_SDCARD_AUDIO_CARD_EVENT_SCANNING_FINISHED:                   
            return ( true );        

        case APP_SDCARD_AUDIO_CARD_EVENT_TRACK_CHANGED:
                
            APP_SDCARD_AUDIO_Player_Reset();
            if (AppSdCardAudioData.player.isPlayerRunning == true)
            {
                AppDataAudioPlayerPtr->state = APP_SDCARD_AUDIO_PLAYER_STATE_RUNNING;            
            }
            return ( true );        

        case APP_SDCARD_AUDIO_CARD_EVENT_FILE_OPEN_ERROR:
        case APP_SDCARD_AUDIO_CARD_EVENT_END_OF_FILE:
            
            APP_SDCARD_AUDIO_Card_CloseFile(AppDataAudioPlayerPtr->fileHandle);            
            APP_SDCARD_AUDIO_Card_NextTrack();
            return ( true );
        
    }
    return ( false );
}

uint32_t APP_SDCARD_AUDIO_GenreateRandomTrackNum(uint32_t maxRandNum)
{    
    volatile uint32_t randNumLSBs = 1;   
    volatile uint32_t generatedRandNum = SYS_RANDOM_PseudoGet();
    //Calculate the number of LSBs to consider from the generated random number.
    while(randNumLSBs < maxRandNum)  
    {
        //Round to the nearest power of 2
        randNumLSBs <<= 1;
    }
    uint32_t randNum = (generatedRandNum & (randNumLSBs-1)) + 1;
    randNum = (randNum * maxRandNum)/randNumLSBs;
    
    return randNum;
}

////////////////////////////////////////////////////////////////////////////////
 bool APP_SDCARD_AUDIO_Card_CloseFile (SYS_FS_HANDLE fileHandle )
{
    SYS_FS_RESULT ret;
    if ( fileHandle != SYS_FS_HANDLE_INVALID )
    {
        ret = SYS_FS_FileClose ( fileHandle );
        if(ret == SYS_FS_RES_SUCCESS)
        {
            return true;
        }
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////// 
bool APP_SDCARD_AUDIO_Card_NextTrack(void)
{
    bool isTrackListRewind = false;
        
    if(AppDataAudioPlayerPtr->isRandomSongSet == true)
    {
        AppDataAudioPlayerPtr->isRandomSongSet = false;
        AppDataAudioPlayerPtr->nextSongIdx = AppDataAudioPlayerPtr->randomSongIdx;                         
    }
    else if (AppDataAudioPlayerPtr->isLoopSingleTrack == true)
    {
        AppDataAudioPlayerPtr->nextSongIdx = AppDataAudioPlayerPtr->nextSongIdx;
    }
    else if (AppDataAudioPlayerPtr->isShuffleOn == true)
    {
        uint32_t numTracks = APP_SDCARD_AUDIO_FileCountGet();        
        AppDataAudioPlayerPtr->nextSongIdx = APP_SDCARD_AUDIO_GenreateRandomTrackNum(numTracks);        
    }
    else
    {            
        AppDataAudioPlayerPtr->nextSongIdx = AppDataAudioPlayerPtr->currentSongIdx + 1;   
        if (AppDataAudioPlayerPtr->nextSongIdx >= AppDataAudioPlayerPtr->totalAudioFiles)
        {
            AppDataAudioPlayerPtr->nextSongIdx = 0;
            isTrackListRewind = true;                
        }
    }
        
    AppDataAudioPlayerPtr->currentSongIdx = AppDataAudioPlayerPtr->nextSongIdx;

    APP_SDCARD_AUDIO_Card_OpenTrack(AppSdCardAudioCardFilesTable
            [AppDataAudioPlayerPtr->currentSongIdx].path);
    
    if (true == isTrackListRewind && false == AppDataAudioPlayerPtr->isLoopTrackList
            && false == AppDataAudioPlayerPtr->isShuffleOn)
    {
        AppDataAudioPlayerPtr->state = APP_SDCARD_AUDIO_PLAYER_STATE_STOP;  
        AppSdCardAudioData.player.isPlayerRunning = false;
        AppSdCardAudioData.player.eventNotify(DISP_CMD_PLAYER_PAUSE_REQ);
        return false;
    }
    else
    {
        return true;
    }
    
} 
 
void APP_SDCARD_AUDIO_Card_TraverseAllFiles
(
    APP_SDCARD_AUDIO_CARD_FILE_NODE node, 
    bool isRoot
)
{
    int i;
    SYS_FS_RESULT ret;
    uint8_t totalDir = 0;
    SYS_FS_FSTAT dirTable[APP_SDCARD_AUDIO_CARD_MAX_DIRS];

    ret = APP_SDCARD_AUDIO_Card_FS_ReadDirFlat((const int8_t*)node.path, &totalDir, dirTable, isRoot);

    if(ret == SYS_FS_RES_FAILURE)
    {
        return;
    }

    APP_SDCARD_AUDIO_CARD_FILE_NODE child_node; 

    for(i = 0; i < totalDir; i++)
    {
         
        child_node.fstat = dirTable[i];         
        strcpy(child_node.path, node.path);
        if(!isRoot)
        {
            strcat(child_node.path, "/");
        }
        strcat(child_node.path, child_node.fstat.fname);
        APP_SDCARD_AUDIO_Card_TraverseAllFiles(child_node, false);         
    }     
    return;
 }

////////////////////////////////////////////////////////////////////////////////
SYS_FS_RESULT APP_SDCARD_AUDIO_Card_FS_ReadDirFlat
(
    const int8_t *fname, 
    uint8_t *dir_count, 
    SYS_FS_FSTAT * dir_table, 
    bool isRoot
)
{
    SYS_FS_RESULT ret;
    AppDataAudioCardData.dirHandle = SYS_FS_DirOpen((const char*)fname);
    if(AppDataAudioCardData.dirHandle == SYS_FS_HANDLE_INVALID)
    {
        return SYS_FS_RES_FAILURE;
    }
    *dir_count = 0;
    
    do
    {
        if(AppDataAudioPlayerPtr->totalAudioFiles < APP_SDCARD_AUDIO_CARD_MAX_FILES)
        {            
            ret = SYS_FS_DirRead(AppDataAudioCardData.dirHandle,&AppDataAudioCardData.dirstat);             
            // End of this directory
            if(AppDataAudioCardData.dirstat.fname[0] == '\0')
            {
                break;
            }
            if(ret!= SYS_FS_RES_FAILURE)
            {
                if(AppDataAudioCardData.dirstat.fattrib != SYS_FS_ATTR_DIR)  
                {                     
                    if(APP_SDCARD_AUDIO_Card_isSupportedAudioFile((int8_t*)AppDataAudioCardData.dirstat.fname))
                    {
                        strcpy((char*)AppSdCardAudioCardFilesTable
                               [AppDataAudioPlayerPtr->totalAudioFiles].path, (const char*)fname);
                        if(!isRoot)
                        {
                            strcat((char*)AppSdCardAudioCardFilesTable
                                   [AppDataAudioPlayerPtr->totalAudioFiles].path, "/");
                        }
                        strcat((char*)AppSdCardAudioCardFilesTable
                               [AppDataAudioPlayerPtr->totalAudioFiles].path,
                               AppDataAudioCardData.dirstat.fname);

                        (AppDataAudioPlayerPtr->totalAudioFiles)++;
                    }
                }
                else if(AppDataAudioCardData.dirstat.fattrib == SYS_FS_ATTR_DIR 
                        && AppDataAudioCardData.dirstat.fname[0] != '.') // Skip ".\" and "..\" directories
                {
                    if(*dir_count < APP_SDCARD_AUDIO_CARD_MAX_DIRS)
                    {
                        dir_table[*dir_count]=AppDataAudioCardData.dirstat;
                        (*dir_count)++;
                    }
                }
            }
        }
        else
        {
            ret = SYS_FS_RES_FAILURE;
            break;
        }
    }while(ret==SYS_FS_RES_SUCCESS);

    SYS_FS_DirClose(AppDataAudioCardData.dirHandle);
    return ret;
}

////////////////////////////////////////////////////////////////////////////////
bool APP_SDCARD_AUDIO_Card_isSupportedAudioFile(int8_t *name)
{
    int i = 0;
    
    do
    {
       i++;
    }while(name[i]!= '.' && name[i] != '\0');
    
    if(name[i] == '\0')
        return false;
    
    if(AppSdCardAudioData.player.MP3_decoder_enabled)
    {
        if((name[i+1] == 'm' && name[i+2] == 'p' && name[i+3] == '3') || 
          (name[i+1] == 'M' && name[i+2] == 'P' && name[i+3] == '3'))
            return true;
    }
    
    if(AppSdCardAudioData.player.WAV_decoder_enabled){
        if((name[i+1] == 'w' && name[i+2] == 'a' && name[i+3] == 'v') || 
           (name[i+1] == 'W' && name[i+2] == 'A' && name[i+3] == 'V'))
            return true;
    }

    return false;
}

APP_SDCARD_AUDIO_CARD_FILE_PATH *APP_SDCARD_AUDIO_GetFileTablePointer(void)
{
    return &AppSdCardAudioCardFilesTable[0];
}

void APP_SDCARD_AUDIO_UpdatePlayerRunningStatus(bool isPlayerRunning)
{
    if (AppSdCardAudioData.player.isPlayerRunning == true)
    {
        AppSdCardAudioData.player.state = APP_SDCARD_AUDIO_PLAYER_STATE_WAIT_FOR_BUFFER_COMPLETION; 
    }
    AppSdCardAudioData.player.isPlayerRunning = isPlayerRunning;      
}

void APP_SDCARD_AUDIO_RandomTrackSet(uint16_t trackId)
{
    if(AppSdCardAudioData.player.currentSongIdx != trackId)
    {
        AppSdCardAudioData.player.isRandomSongSet = true;
        AppSdCardAudioData.player.randomSongIdx = trackId;
        AppSdCardAudioData.player.nextTrack = true;
        AppSdCardAudioData.player.state = APP_SDCARD_AUDIO_PLAYER_STATE_TRACK_CHANGE;        
    }
}

bool APP_SDCARD_AUDIO_isSonglistAvailable(void)
{
    return AppSdCardAudioData.player.isSongListAvailable;
}

void APP_SDCARD_AUDIO_MuteOn(void)
{
    if (AppSdCardAudioData.codec.handle != DRV_HANDLE_INVALID)
    {
        DRV_CODEC_MuteOn(AppSdCardAudioData.codec.handle);    
    }
}

void APP_SDCARD_AUDIO_MuteOff(void)
{
    if (AppSdCardAudioData.codec.handle != DRV_HANDLE_INVALID)
    {
        DRV_CODEC_MuteOff(AppSdCardAudioData.codec.handle);        
    }
}

bool APP_SDCARD_AUDIO_VolumeGet(uint8_t* const pVolLevel)
{
    bool isSuccess = false;
    if (AppSdCardAudioData.codec.handle != DRV_HANDLE_INVALID)
    {
        *pVolLevel = DRV_CODEC_VolumeGet(AppSdCardAudioData.codec.handle, DRV_CODEC_CHANNEL_LEFT_RIGHT);    
        isSuccess = true;
    }
    return isSuccess;
}

void APP_SDCARD_AUDIO_VolumeSet(uint8_t volume)
{
    if (AppSdCardAudioData.codec.handle != DRV_HANDLE_INVALID)
    {
        DRV_CODEC_VolumeSet(AppSdCardAudioData.codec.handle, DRV_CODEC_CHANNEL_LEFT_RIGHT, volume);           
    }
}

uint16_t APP_SDCARD_AUDIO_FileCountGet(void)
{
    return AppDataAudioPlayerPtr->totalAudioFiles;
}

uint16_t APP_SDCARD_AUDIO_CurrentTrackIdGet(void)
{
    return AppSdCardAudioData.player.currentSongIdx;
}

void APP_SDCARD_AUDIO_LoopSingleTrackEvent(bool isRepeatSingleTrack)
{
    AppSdCardAudioData.player.isLoopSingleTrack = isRepeatSingleTrack;
    AppSdCardAudioData.player.isLoopTrackList = false;
}

void APP_SDCARD_AUDIO_LoopTrackListEvent(bool isLoopOn)
{
    AppSdCardAudioData.player.isLoopTrackList = isLoopOn;
    AppSdCardAudioData.player.isLoopSingleTrack = false;
}

void APP_SDCARD_AUDIO_ShuffleTracksEvent(bool isOn)
{
    AppSdCardAudioData.player.isShuffleOn = isOn;
}

void APP_SDCARD_AUDIO_ResetTrackPlayTime(void)
{    
    AppSdCardAudioData.player.nTotalBytesPlayed = 0;
    AppSdCardAudioData.player.eventNotify(DISP_CMD_UPDATE_PROGBAR_IN_PER_REQ);
}

uint32_t APP_SDCARD_AUDIO_GetElapsedTrackTimeInBytes(void)
{
    return AppSdCardAudioData.player.nTotalBytesPlayed;
}

uint32_t APP_SDCARD_AUDIO_GetTotalTrackTimeInBytes(void)
{
    return WAV_GetAudioSize();
}
void APP_SDCARD_AUDIO_UpdateElapsedTrackTimeInBytes(uint32_t nBytesPlayed)
{                
    AppSdCardAudioData.player.nTotalBytesPlayed += nBytesPlayed;    

    AppSdCardAudioData.player.eventNotify(DISP_CMD_UPDATE_PROGBAR_IN_PER_REQ);                     
}

uint8_t APP_SDCARD_AUDIO_GetTrackPlayTimeInPer(void)
{    
    uint32_t audioSize = WAV_GetAudioSize();
    
    if (audioSize > 0)
    {
        return ((uint64_t)AppSdCardAudioData.player.nTotalBytesPlayed*100)/(uint64_t)audioSize;
    }
    else
    {
        return 0;
    }
}

void APP_SDCARD_AUDIO_SeekTrackPosition(int32_t seekPos)
{  
    seekPos = seekPos & 0xFFFFFFFE;
    AppSdCardAudioData.player.nTotalBytesPlayed = seekPos;    
    AppDataAudioCardTotalBytesRead = seekPos;
        
    APP_SDCARD_AUDIO_ResetBuffers();
    
    SYS_FS_FileSeek(AppSdCardAudioData.player.fileHandle, seekPos, SYS_FS_SEEK_SET);
                                
    AppSdCardAudioData.player.eventNotify(DISP_CMD_UPDATE_PROGBAR_IN_PER_REQ);   
}

uint32_t APP_SDCARD_AUDIO_GetTrackPlayTime(void)
{
    return AppSdCardAudioData.player.playerTrackTotalTime;
}

uint32_t APP_SDCARD_AUDIO_GetCurrentFileSizeBytes(void)
{
    return AppSdCardAudioData.player.currentFilesize;
}
