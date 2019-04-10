/*******************************************************************************
  USB Host MSD Demo .

  Company:
    Microchip Technology Inc.

  File Name:
    audio_codec.c

  Summary:
   Contains the functional implementation of CODEC.

  Description:
   This file contains the functional implementation of CODEC.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "audio_codec.h"


AUDIO_CODEC_DATA CodecData;
APP_AUDIOPLAYER *appDataPtr;
static int8_t getCompletedBufferIdx(DRV_CODEC_BUFFER_HANDLE handle);
static void clearBufferFlags(DRV_CODEC_BUFFER_HANDLE handle);

//int8_t  __attribute__((coherent)) sine_tone2[sizeof(App_sine_tone)];
/******************************************************
 * Codec Initialize. It is
 * called from the APP_Initialize() function.
 ******************************************************/
void AUDIO_CODEC_Initialize (void)
{
    CodecData.dl = DATA_LENGTH_16;
    CodecData.sl = SAMPLE_LENGTH_32;
    CodecData.state = AUDIO_CODEC_BUFFER_COMPLETE;
    CodecData.codecClient.handle = DRV_HANDLE_INVALID;
    CodecData.codecClient.context = 0;
    CodecData.codecClient.bufferHandler = (DRV_CODEC_BUFFER_EVENT_HANDLER) APP_CODECBufferEventHandler;
    CodecData.codecClient.commandHandler = (DRV_CODEC_COMMAND_EVENT_HANDLER) APP_CODECCommandEventHandler;
    CodecData.codecClient.currentCommand = CODEC_COMMAND_NONE;
    appDataPtr = APP_GetAppDataInstance();
}

/**********************************************************
 * Application CODEC buffer Event handler.
 * This function is called back by the CODEC driver when
 * a CODEC data buffer TX completes.
 ***********************************************************/
static int int_num = 0;
//static int underrun = 0;
void APP_CODECBufferEventHandler(DRV_CODEC_BUFFER_EVENT event,
        DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context )
{
    /* Transmission has completed. OFF the LEDs */    
   
    switch(event)
    {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        {
            CodecData.state = AUDIO_CODEC_BUFFER_COMPLETE;

            if(appDataPtr->playerState == APP_STATE_RUNNING){
                int8_t readIdx = appDataPtr->readIdx;
                if(appDataPtr->audioBuffer[readIdx].decoded && !appDataPtr->audioBuffer[readIdx].inUse)
                {
                    if( Audio_Codec_Addbuffer(appDataPtr->audioBuffer[readIdx].buffer+appDataPtr->audioBuffer[readIdx].offset, 
                            appDataPtr->audioBuffer[readIdx].bufferSize) == true)
                    {
                        appDataPtr->audioBuffer[readIdx].writeHandler = CodecData.codecClient.writeBufHandle;
                        appDataPtr->audioBuffer[readIdx].inUse = true;
                        appDataPtr->readIdx = APP_GetNextIdxInOutputBuffer(readIdx);
                        int_num++;
                    }else{
                        Nop();
                    }
                }else{
                    // underrun
//                    underrun++;
                    Nop();
                    
                }
            }

            clearBufferFlags(handle);
        }
        
        break;
        case DRV_CODEC_BUFFER_EVENT_ERROR:
        {
            Nop();
        } break;

        case DRV_CODEC_BUFFER_EVENT_ABORT:
        {
            Nop();
        } break;

    }
}


void APP_CODECCommandEventHandler( uintptr_t context )
{
    // command executed successfully
    if(CodecData.codecClient.currentCommand == CODEC_COMMAND_SAMPLING_RATE_SET)
    {
        APP_PlayerCommand ( PLAYER_CMD_PLAY );
        CodecData.codecClient.currentCommand = CODEC_COMMAND_NONE;
    }
    
}

bool Audio_Codec_Open()
{
    SYS_STATUS status;
    status = DRV_CODEC_Status(sysObjdrvCodec0);
    if (SYS_STATUS_READY == status)
    {
        // This means the driver can now be be opened.
        /* A client opens the driver object to get an Handle */
        CodecData.codecClient.handle = DRV_CODEC_Open(DRV_CODEC_INDEX_0, DRV_CODEC_IO_INTENT);
        
        if(CodecData.codecClient.handle != DRV_HANDLE_INVALID)
        {
            return true;
        }
        else
        {
          return false;
        }
    }
   return false;
}

void Audio_Codec_Close(void)
{
    DRV_CODEC_Close(CodecData.codecClient.handle);
}

void Audio_Codec_SetBufferHandler(void)
{
     DRV_CODEC_BufferEventHandlerSet(CodecData.codecClient.handle, CodecData.codecClient.bufferHandler, CodecData.codecClient.context);
}

void Audio_Codec_SetCommandCallback(void)
{
    DRV_CODEC_CommandEventHandlerSet(CodecData.codecClient.handle, CodecData.codecClient.commandHandler, CodecData.codecClient.context);
}

void Audio_Codec_SetAudioFormat(DATA_LENGTH dl, SAMPLE_LENGTH sl)
{
    if(dl != CodecData.dl || sl != CodecData.sl)
    {
        DRV_CODEC_SetAudioCommunicationMode((CodecData.codecClient.handle),dl, sl);
        CodecData.dl = dl;
        CodecData.sl = sl;
    }
}

bool Audio_Codec_Addbuffer(int8_t* buffer, size_t bufferSize)
{
    DRV_CODEC_BufferAddWrite(CodecData.codecClient.handle, &CodecData.codecClient.writeBufHandle,
                                buffer, bufferSize);
            
    if(CodecData.codecClient.writeBufHandle != DRV_CODEC_BUFFER_HANDLE_INVALID)
    {
        Nop();
        return true;
    }
    else
    {
        Nop();
        return false;
    }
}

static int8_t getCompletedBufferIdx(DRV_CODEC_BUFFER_HANDLE handle)
{
    int8_t i;
    APP_AUDIOPLAYER *appDataPtr = APP_GetAppDataInstance();
    for(i = 0; i < AUDIO_QUEUEBUFFER_NUMBER; i++)
    {
        if(appDataPtr->audioBuffer[i].writeHandler == handle)
        {
            return i;
        }
    }
    // something wrong
    return -1;
}

static void clearBufferFlags(DRV_CODEC_BUFFER_HANDLE handle)
{
    int8_t releaseIdx = getCompletedBufferIdx(handle);
    if(releaseIdx == -1)
    {
        return;
    }
    APP_AUDIOPLAYER *appDataPtr = APP_GetAppDataInstance();
    appDataPtr->audioBuffer[releaseIdx].inUse = false;
    appDataPtr->audioBuffer[releaseIdx].bufferSize = 0;
    appDataPtr->audioBuffer[releaseIdx].offset = 0;
    appDataPtr->audioBuffer[releaseIdx].decoded = false;
    appDataPtr->audioBuffer[releaseIdx].writeHandler = DRV_CODEC_BUFFER_HANDLE_INVALID;
}
