/*******************************************************************************
  Universal Audio Decoders Demo

  Company:
    Microchip Technology Inc.

  File Name:
    decoder.c

  Summary:
   Contains the functional implementation of decoder functions.

  Description:
   This file contains the functional implementation of decoder functions.
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
OTHER LEGAL EQUITABLE THEORY ANY DIR-*9ECT OR INDIRECT DAMAGES OR EXPENSES
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

#define DECODER_C

#include "app.h"
#include "../disk/disk.h"
#include "decoder.h"
#include "../audio_codec/audio_codec.h"

////////////////////////////////////////////////////////////////////////////////
static DECODER_TYPE decoderType;
// wav meta header is 44 bytes, aac frameheader size is 7 bytes.
// using 48 bytes here.
uint8_t metadata[48];
// internal use in decoder library for holding runtime decoder state info
// size of decoderState depends on which decoder you are using,
// in this demo, we are giving the size of AAC decoder state,
// which is 11032 bytes.
uint8_t decoderState[11032];
uint32_t decoderBitrate;
uint32_t decoderSamplerate;
int sampling_frequency;
const static char __attribute__((unused)) *unknown = "UNKNOWN";
uint8_t outbuf_offset;


/*********************STATIC FUNCTIONS DECLARATION*****************************/
static void DECODE_ConfigureAudioTransmitMode(bool stereo);


////////////////////////////////////////////////////////////////////////////////
void DECODER_Initialize ( uint8_t type )
{
    outbuf_offset = 0;
    decoderType = DECODER_TYPE_UNKNOWN;
    APP_AUDIOPLAYER __attribute__((unused)) *appDataPtr = APP_GetAppDataInstance();
    DECODER_EventHandler(DECODER_EVENT_DISPLAY_CLEAN, 0);
    switch ( type )
    {
#ifdef MP3_DECODER_ENABLED
        case APP_DECODER_MP3:
            appDataPtr->currentStreamType = APP_STREAM_MP3;
            decoderType = DECODER_TYPE_MP3;
            APP_LED_1Off();
            APP_LED_2Off();
            APP_LED_3On();
            appDataPtr->updatePlaytimeFunc = MP3_UpdatePlaytime;
            if ( MP3_Initialize ( decoderState, MP3_DECODER_HEAP_SIZE, appDataPtr->fileHandle) != true )
            {
               DECODER_EventHandler ( DECODER_INITIALIZATION_ERROR, 0 );
            }
            Audio_Codec_SetAudioFormat(DATA_LENGTH_16, SAMPLE_LENGTH_32);
            DECODE_ConfigureAudioTransmitMode((MP3_GetChannels()==2));
            // add back later
            break;
#endif
            
#ifdef AAC_DECODER_ENABLED
        case APP_DECODER_AAC:
            appDataPtr->currentStreamType = APP_STREAM_AAC;
            decoderType = DECODER_TYPE_AAC;
            
            APP_LED_1Off();
            APP_LED_2On();
            APP_LED_3Off();

            appDataPtr->readBytes = AAC_FRAME_HEADER_SIZE;
            appDataPtr->readbyte_flag = true;
            AAC_Initialize ( decoderState, AACDECODER_STATE_SIZE, metadata, appDataPtr->fileHandle);
            sampling_frequency = AAC_GetSamplingFrequency(metadata);
            DECODER_EventHandler ( DECODER_EVENT_SAMPLERATE, sampling_frequency );
            DECODER_EventHandler ( DECODER_EVENT_TAG_TITLE, (uint32_t)appDataPtr->fileStatus.lfname );
            DECODER_EventHandler ( DECODER_EVENT_TRACK_TIME, 0);
            Audio_Codec_SetAudioFormat(DATA_LENGTH_16, SAMPLE_LENGTH_32);
            if(SYS_FS_FileSeek( appDataPtr->fileHandle, 0, SYS_FS_SEEK_SET ) == -1)
            {
                Nop();
            }
            else
            {
                /* Check for End of file */
                 Nop();
            }
            DECODE_ConfigureAudioTransmitMode((AAC_GetChannels()==2));
            break;
#endif
            

#ifdef WMA_DECODER_ENABLED
        case APP_DECODER_WMA:
            appDataPtr->currentStreamType = APP_STREAM_WMA;
            decoderType = DECODER_TYPE_WMA;
            
            APP_LED_1Off();
            APP_LED_2On();
            APP_LED_3On();

            WMA_Initialize (appDataPtr->fileHandle, DECODER_MAX_INPUT_BUFFER_SIZE);

            DECODE_ConfigureAudioTransmitMode((WMA_GetChannels()==2));
            sampling_frequency = WMA_SamplingFrequency_Get();
            decoderBitrate = WMA_BitRate_Get()/1000;
            DECODER_EventHandler ( DECODER_EVENT_SAMPLERATE, sampling_frequency );
            DECODER_EventHandler ( DECODER_EVENT_BITRATE, decoderBitrate );
            DECODER_EventHandler ( DECODER_EVENT_TAG_TITLE, (uint32_t)appDataPtr->fileStatus.lfname );
            DECODER_EventHandler ( DECODER_EVENT_TRACK_TIME, 0);
            Audio_Codec_SetAudioFormat(DATA_LENGTH_16, SAMPLE_LENGTH_32);
            if(SYS_FS_FileSeek( appDataPtr->fileHandle, WMA_GetHeaderPacketOffset(), SYS_FS_SEEK_SET ) == -1)
            {
                Nop();
            }
            else
            {
               /* Check for End of file */
                Nop();
            }
            break;
#endif
            
            
#ifdef WAV_STREAMING_ENABLED
        case APP_DECODER_WAV:
            appDataPtr->currentStreamType = APP_STREAM_WAV;
            appDataPtr->updatePlaytimeFunc = WAV_UpdatePlaytime;
            decoderType = DECODER_TYPE_WAV;
            
            APP_LED_1On();
            APP_LED_2Off();
            APP_LED_3Off();
            
            DISK_ReadCurrentFile(metadata, WAV_HEADER_SIZE);
            
            appDataPtr->readBytes = WAV_INPUT_BUFFER_SIZE;
            appDataPtr->readbyte_flag = true;
            WAV_Initialize_N (metadata, appDataPtr->fileHandle);
//            sampling_frequency = 
            DECODER_EventHandler ( DECODER_EVENT_SAMPLERATE, WAV_GetSampleRate());
            DECODER_EventHandler ( DECODER_EVENT_BITRATE, WAV_GetBitRate());
            DECODER_EventHandler ( DECODER_EVENT_TRACK_TIME, WAV_GetDuration());

            outbuf_offset = 0;
            
            DECODER_EventHandler ( DECODER_EVENT_TAG_TITLE, (uint32_t)appDataPtr->fileStatus.lfname );
            DECODER_EventHandler ( DECODER_EVENT_TAG_ARTIST, (uint32_t)unknown );
            DECODER_EventHandler ( DECODER_EVENT_TAG_ALBUM, (uint32_t)unknown );
            // for RW function, move to end of previous song
            DECODER_EventHandler ( DECODER_EVENT_STREAM_START, 0 );
            Audio_Codec_SetAudioFormat(DATA_LENGTH_16, SAMPLE_LENGTH_32);
            DECODE_ConfigureAudioTransmitMode((WAV_GetChannels()==2));
            break;
#endif
            
#ifdef OGG_SPEEX_DECODER_ENABLED
        case APP_DECODER_SPEEX:
            appDataPtr->currentStreamType = APP_STREAM_SPEEX;
            APP_ERROR_MSG res = SPEEX_Initialize(appDataPtr->fileHandle);
            if(res == APP_SUCCESS)
            {
                sampling_frequency = SPEEX_GetSamplingRate();
                DECODER_EventHandler ( DECODER_EVENT_SAMPLERATE, sampling_frequency );
                decoderBitrate = SPEEX_GetBitrate()/1000;
                DECODER_EventHandler ( DECODER_EVENT_BITRATE, decoderBitrate );
                
                DECODER_EventHandler ( DECODER_EVENT_TAG_TITLE, (uint32_t)appDataPtr->fileStatus.lfname);
                DECODER_EventHandler ( DECODER_EVENT_TRACK_TIME, 0);
                
                Audio_Codec_SetAudioFormat(DATA_LENGTH_16, SAMPLE_LENGTH_32);
                DECODE_ConfigureAudioTransmitMode((SPEEX_GetChannels()==2));
            }
            decoderType = DECODER_TYPE_SPEEX;
            break;
#endif
            
#ifdef ADPCM_STREAMING_ENABLED
        case APP_DECODER_ADPCM:
            appDataPtr->currentStreamType = APP_STREAM_ADPCM;
            DISK_ReadCurrentFile(metadata, ADPCM_HEADER_SIZE);
            
            appDataPtr->readbyte_flag = true;
            // read 512 bytes as one ADPCM frame, this number could be any number
            // less than DECODER_MAX_INPUT_BUFFER_SIZE.
            appDataPtr->readBytes = 512;
            ADPCM_Initialize(metadata);
            
            sampling_frequency = ADPCM_HdrGetSamplesPerSec();
            
            DECODER_EventHandler ( DECODER_EVENT_SAMPLERATE, sampling_frequency );
            DECODER_EventHandler ( DECODER_EVENT_TAG_TITLE, (uint32_t)appDataPtr->fileStatus.lfname );
            DECODER_EventHandler ( DECODER_EVENT_TRACK_TIME, 0);
            
            Audio_Codec_SetAudioFormat(DATA_LENGTH_16, SAMPLE_LENGTH_32);
            DECODE_ConfigureAudioTransmitMode((ADPCM_GetChannels()==2));
            decoderType = DECODER_TYPE_ADPCM;
            break;
#endif
            
#ifdef OGG_OPUS_DECODER_ENABLED
        case APP_DECODER_OPUS:
            appDataPtr->currentStreamType = APP_STREAM_OPUS;
            APP_ERROR_MSG opus_ret = OPUS_Initialize(appDataPtr->fileHandle);
            if(opus_ret == APP_SUCCESS)
            {
                sampling_frequency = OPUS_GetSamplingRate();
                DECODER_EventHandler ( DECODER_EVENT_SAMPLERATE, sampling_frequency );
                DECODER_EventHandler ( DECODER_EVENT_TAG_TITLE, (uint32_t)appDataPtr->fileStatus.lfname );
                DECODER_EventHandler ( DECODER_EVENT_TRACK_TIME, 0);
                Audio_Codec_SetAudioFormat(DATA_LENGTH_16, SAMPLE_LENGTH_32);
            }
            DECODE_ConfigureAudioTransmitMode((OPUS_GetChannels()==2));
            decoderType = DECODER_TYPE_OPUS;
            break;
#endif
            
#ifdef FLAC_DECODER_ENABLED
        case APP_DECODER_FLAC:
            appDataPtr->currentStreamType = APP_STREAM_FLAC;
            bool flac_ret = FLAC_Initialize(appDataPtr->fileHandle, appDataPtr->audioInput);
            if(flac_ret)
            {
                sampling_frequency = FLAC_GetSamplingRate();
                DECODER_EventHandler ( DECODER_EVENT_SAMPLERATE, sampling_frequency );
                DECODER_EventHandler ( DECODER_EVENT_TRACK_TIME, FLAC_GetDuration());
                DECODER_EventHandler ( DECODER_EVENT_TAG_TITLE, (uint32_t)appDataPtr->fileStatus.lfname );
                
                if(FLAC_GetBitdepth() == 24)
                {
                    outbuf_offset = 32;
                    Audio_Codec_SetAudioFormat(DATA_LENGTH_24, SAMPLE_LENGTH_32);
                }else
                {
                    Audio_Codec_SetAudioFormat(DATA_LENGTH_16, SAMPLE_LENGTH_32);
                }
            }
            decoderType = DECODER_TYPE_FLAC;
            
            break;
#endif
            
        case APP_DECODER_UNKNOWN:
            break;
        default:
            break;
       
    }
}

bool DECODER_Decode ( uint8_t *input, uint16_t inSize, uint16_t *read, int16_t *output, uint16_t *written )
{
    *read = 0;
    *written = 0;
    
    switch ( decoderType )
    {
        case DECODER_TYPE_MP3:
#ifdef MP3_DECODER_ENABLED
           if(MP3_Decode ( input, inSize, read, (uint8_t *)output, written ))
           {
               return true;
           }
           else
           {
               APP_LED_1On();
               APP_LED_2On();
               APP_LED_3On();
               return false;
           }
#endif
           break;

        case DECODER_TYPE_AAC:
#ifdef AAC_DECODER_ENABLED
            if (AAC_Decoder(input, inSize,read,output,written)==true)
            {
               return true;
            }
            else
            {
               APP_LED_1On();
               APP_LED_2On();
               APP_LED_3On();
               return false;
            }
#endif
            break;

            

        case DECODER_TYPE_WMA:
#ifdef WMA_DECODER_ENABLED
            if (WMA_Decoder(input, inSize,read,output,written)==true)
            {
               return true;
            }
            else
            {
                APP_LED_1On();
                APP_LED_2On();
                APP_LED_3On();
                return false;
            }
#endif
            break;

            

        case DECODER_TYPE_WAV:
#ifdef WAV_STREAMING_ENABLED
            if( WAV_Decoder (input,inSize,read,output,written)==true)
            {
                return true;
            }
            else
            {
                return false;
            }
#endif
            break;

            

        case DECODER_TYPE_SPEEX: // ogg-speex
#ifdef OGG_SPEEX_DECODER_ENABLED
        {
            APP_ERROR_MSG res;
            res = SPEEX_Decoder(input,inSize,read,output,written);
            if(res == APP_SUCCESS){
                return true;
            }else if(res == APP_STREAM_END){
                return true;
            }
            else{
                return false;
            }
        }
#endif
            break;

            

        case DECODER_TYPE_ADPCM:
#ifdef ADPCM_STREAMING_ENABLED
            if(ADPCM_Decoder(input,inSize,read,output,written)){
                return true;
            }else{
                return false;
            }
#endif
            break;

        case DECODER_TYPE_OPUS:
#ifdef OGG_OPUS_DECODER_ENABLED
            if(OPUS_Decoder(input, inSize, read, output, written, DECODER_MAX_OUTPUT_BUFFER_SIZE)==1){
                return true;
            }else{
                return false;
            }
#endif
            break;

        case DECODER_TYPE_FLAC:
#ifdef FLAC_DECODER_ENABLED             
            if(FLAC_Decoder(input, inSize, read, (uint8_t *)output, written))
            {
                return true;
            }else
            {
                APP_LED_1On();
                APP_LED_2On();
                APP_LED_3On();
                return false;
            }
#endif
            break;
                  
        case DECODER_TYPE_UNKNOWN:
            break;
    }

    return ( true );
}

// This is the function returns the audio size in file, 
// which means it doesn't contain the header size or meta data.
uint32_t DECODER_GetCurrentValidAudioSize(){
    // add other decoder cases later
    uint32_t ret = 0;
    switch(decoderType){
        
        case DECODER_TYPE_WAV:
#ifdef WAV_STREAMING_ENABLED
            ret = WAV_GetAudioSize();
#endif
            break;
        case DECODER_TYPE_MP3:
#ifdef MP3_DECODER_ENABLED
            ret = MP3_GetAudioSize();
#endif
            break;
        case DECODER_TYPE_AAC:
            break;
        case DECODER_TYPE_WMA:
            break;
        /* Tobe added
        case DECODER_TYPE_FLAC:
            break;
        */
         default:
            ret = 0;
            break;
    }
    return ret;
}

void DECODER_Cleanup()
{
    switch ( decoderType )
    {
        case DECODER_TYPE_WMA:
#ifdef WMA_DECODER_ENABLED
            WMA_FreeMemory();
#endif
            break;
        case DECODER_TYPE_SPEEX:
#ifdef OGG_SPEEX_DECODER_ENABLED
            SPEEX_Cleanup();
#endif
            break;
        case DECODER_TYPE_OPUS:
#ifdef OGG_OPUS_DECODER_ENABLED
            OPUS_Cleanup();
#endif
            break;
        case DECODER_TYPE_FLAC:
#ifdef FLAC_DECODER_ENABLED
            FLAC_Cleanup();
#endif
            break;

       default:
            break;
    }
}

/////////////////////////////////////////////////////////////
uint32_t GetDecodeBitRate(void)
{
    return ( (uint32_t) decoderBitrate );
}

/////////////////////////////////////////////////////////////
uint32_t GetDecodeSamplingRate ( void )
{
    return ( (uint32_t) decoderSamplerate );
}


static void __attribute__((unused)) DECODE_ConfigureAudioTransmitMode(bool stereo)
{
    if(stereo){
        if(!APP_GetSpiAudioMode()){
            PLIB_SPI_AudioTransmitModeSelect(DRV_I2S_PERIPHERAL_ID_IDX0, 
                                                SPI_AUDIO_TRANSMIT_STEREO);
            APP_SetSpiAudioMode(stereo);
        }// if current SPI audio mode is mono, reconfigure SPI
    }else{
        if(APP_GetSpiAudioMode()){
            PLIB_SPI_AudioTransmitModeSelect(DRV_I2S_PERIPHERAL_ID_IDX0, 
                                                SPI_AUDIO_TRANSMIT_MONO);
            APP_SetSpiAudioMode(stereo);
        }// if current SPI audio is stereo
    }
    
}
