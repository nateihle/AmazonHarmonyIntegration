/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************
#include <stdarg.h>
#include "app.h"

#include "display/display.h"
#include "system/debug/sys_debug.h"

#include "wav_format_container.h"

#ifdef MEB2_CONFIG
#include "ogg_format_container.h"
#endif

#include "encoder.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define AUDIO_FILE_METADATA_HEADER_SIZE 1024 //1Kb contains header+comments
#define NUM_PACKETS_TO_ONE_PAGE  10
#define AUDIO_ENCODE_SAMPLE_RATE 16000

typedef struct {
    uint8_t *buffer;
    uint32_t len;
} BUFFER;
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

APP_DATA appData;
const HAR_ENCODER *runtimeEncoderInst;

const char *extension_str[4] = {"wav", "wav", "opus", "spx"};
static uint8_t pheader[AUDIO_FILE_METADATA_HEADER_SIZE];
static uint32_t _audio_frame_count = 0;
static StreamInfo si;

// 20ms for 16Khz 16bits, stereo

DRV_I2S_DATA16 APP_MAKE_BUFFER_DMA_READY rxBuffer[2][INPUT_SAMPLES];
DRV_I2S_DATA16 APP_MAKE_BUFFER_DMA_READY outBuffer[2][OUTPUT_BUFFER_SIZE];
//DRV_I2S_DATA16 APP_MAKE_BUFFER_DMA_READY dummy[1024];

#define SPEEX_8K_STEREO_FRAMESIZE INPUT_SAMPLES/2
#define SPEEX_8K_MONO_FRAMESIZE SPEEX_8K_STEREO_FRAMESIZE/2
static uint32_t encoded_data_size;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

static int32_t _construct_audio_file_header(EncoderType et,
        StreamInfo *si,
        uint32_t encodedAudioSize,
        void *pheader,
        uint32_t insize);

/* TODO:  Add any necessary local functions.
 */
static void APP_CODECBufferReadEventHandler(DRV_CODEC_BUFFER_EVENT event,
        DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context);

static USB_HOST_EVENT_RESPONSE _USBHostEventHandler(USB_HOST_EVENT event,
        void * eventData, uintptr_t context) {
    switch (event) {
        case USB_HOST_EVENT_DEVICE_UNSUPPORTED:
            break;
        default:
            break;

    }

    return (USB_HOST_EVENT_RESPONSE_NONE);
}

static void _SYSFSEventHandler(SYS_FS_EVENT event, void * eventData, uintptr_t context) {
    switch (event) {
        case SYS_FS_EVENT_MOUNT:
            appData.deviceIsConnected = true;
            break;

        case SYS_FS_EVENT_UNMOUNT:
            appData.deviceIsConnected = false;

            break;

        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */
void APP_Initialize(void) {
    encoded_data_size = 0;
    
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_BUS_ENABLE;
    appData.deviceIsConnected = false;
    appData.record = false;
    appData.encoderSelectIdx = 0; // select PCM by default
    appData.codecClientRead.handle = DRV_HANDLE_INVALID;



    appData.codecClientRead.bufferHandler = (DRV_CODEC_BUFFER_EVENT_HANDLER) APP_CODECBufferReadEventHandler;
    appData.codecClientRead.bufferObject1 = (uint8_t *) rxBuffer[0];
    appData.codecClientRead.bufferObject2 = (uint8_t *) rxBuffer[1];
//    memset(dummy, 0xfe, 4*1024);
    // One Frame size in bytes
    // Speex two frames
    appData.codecClientRead.bufferSize = INPUT_SAMPLES * sizeof (DRV_I2S_DATA16);
    appData.codecClientRead.isCodecReadComplete1 = false;
    appData.codecClientRead.isCodecReadComplete2 = false;
    APP_ButtonInit();

}

bool APP_Writeback_To_File(SYS_FS_HANDLE fileHandle, int vanum, ...) {
    int i;
    va_list valist;
    /* initialize valist for num number of arguments */
    va_start(valist, vanum);

    /* access all the arguments assigned to valist */
    for (i = 0; i < vanum; i++) {
        BUFFER towrite = va_arg(valist, BUFFER);
        if (towrite.len <= 0) continue;
        if (SYS_FS_FileWrite(fileHandle, (const void *) towrite.buffer, towrite.len) == -1) {
            /* Write was not successful. Close the file
             * and error out.*/
            SYS_FS_FileClose(appData.fileHandle);
            appData.state = APP_STATE_ERROR;
        }
    }
    return true;
}

/**
 * Use Model is different based which output format is
 * 
 * WAV header must be written to file when encoding ends,
 * Ogg header must be written at the beginning of encoding.
 * 
 * @param aff
 * @param pheader
 * @param insize
 * @return 
 */

int32_t _construct_audio_file_header(EncoderType encoderType,
        StreamInfo *si,
        uint32_t encodedAudioSize,
        void *pheader,
        uint32_t insize) {
    
//    int ret = 0;
    switch (encoderType) {
#if defined (PCM_ENCODER) || defined(ADPCM_ENCODER)
        // wav(RIFF) file header
        case PCM:
            return wav_riff_fill_header(pheader, PCM, si, encodedAudioSize);
            break;
        case ADPCM:
            return wav_riff_fill_header(pheader, ADPCM, si, encodedAudioSize);
            break;
#endif
#ifdef MEB2_CONFIG
        case OPUS:
            return ogg_opus_fill_header(
                    si->channel,
                    si->sample_rate,
                    si->bps,
                    pheader,
                    insize);

            break;
        case SPEEX:
            return ogg_speex_fill_header(
                    si->channel,
                    si->sample_rate,
                    si->bps,
                    pheader,
                    insize
                    );

            break;
#endif
        default:
            break;
    }
    return -1;
}

/**
 * return -1 means it's last packet handling 
 * @param encoded
 * @param size
 */
#ifdef OGG_CONTAINER
void __attribute__((unused)) APP_Ogg_Container_PageOut(ogg_page *og) {
    BUFFER oggPageHeader;
    BUFFER oggPageBody;

    oggPageHeader.buffer = og->header;
    oggPageHeader.len = og->header_len;
    oggPageBody.buffer = og->body;
    oggPageBody.len = og->body_len;

    if (APP_Writeback_To_File(appData.fileHandle, 2, oggPageHeader, oggPageBody)) {
        // success
        Nop();
    } else {
        // handle file system write ERROR
    }
}
#endif

static __attribute__((unused)) void _convert_mono_data_to_stereo(DRV_I2S_DATA16 *micData, int size) {
    // basically, it copies non-zero channel data to another channel
    int i = 0;
    for (i = 0; i < size; i++) {
        micData[i].leftData = micData[i].leftData + micData[i].rightData;
        micData[i].rightData = micData[i].leftData;
    }
}

inline static void _init_runtime_encoder()
{
    switch(appData.encoderSelectIdx)
    {
        case 0: // PCM
            runtimeEncoderInst = &pcmEncoderInst;
            appData.runTimeAudioType = WAV_RIFF_PCM_FILE;
            break;
        case 1: //ADPCM
            runtimeEncoderInst = &adpcmEncoderInst;
            appData.runTimeAudioType = WAV_RIFF_ADPCM_FILE;
            break;
#ifdef MEB2_CONFIG
        case 2: // Opus
            runtimeEncoderInst = &opusEncoderInst;
            appData.runTimeAudioType = OGG_OPUS_FILE;
            break;
        case 3: // Speex
            runtimeEncoderInst = &speexEncoderInst;
            appData.runTimeAudioType = OGG_SPEEX_FILE;
            break;
#endif
        default:
            break;
    }   
    
    appData.codecClientRead.isCodecReadComplete1 = false;
    appData.codecClientRead.isCodecReadComplete2 = false;
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {
    SYS_STATUS codecStatus;
    uint32_t size;
    uint32_t outsize = 0;
    
    APP_ButtonTask();
    APP_DisplayTask();
    
//    uint32_t outsize2 = 0;
    switch (appData.state) {
        case APP_STATE_BUS_ENABLE:
            /* Set the event handler and enable the bus */
            SYS_FS_EventHandlerSet(_SYSFSEventHandler, (uintptr_t) NULL);
            USB_HOST_EventHandlerSet(_USBHostEventHandler, 0);
            USB_HOST_BusEnable(0);
            appData.state = APP_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE;
            break;

        case APP_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE:
            if (USB_HOST_BusIsEnabled(0)) {
                appData.state = APP_STATE_WAIT_FOR_DEVICE_ATTACH;
            }
            break;
        case APP_STATE_WAIT_FOR_DEVICE_ATTACH:

            /* Wait for device attach. The state machine will move
             * to the next state when the attach event
             * is received.  */
            if (appData.deviceIsConnected) {
                BSP_LEDOn(APP_LED3);
                BSP_LEDOff(APP_LED2);

                appData.state = APP_STATE_AUDIO_CODEC_OPEN;
            }
            break;
        case APP_STATE_AUDIO_CODEC_OPEN:

            codecStatus = DRV_CODEC_Status(sysObjdrvCodec0);

            if (SYS_STATUS_READY == codecStatus) 
            {
                appData.codecClientRead.handle =
                    DRV_CODEC_Open(DRV_CODEC_INDEX_0,
                    DRV_IO_INTENT_READ);
                if (appData.codecClientRead.handle != DRV_HANDLE_INVALID) {
                    appData.state = APP_STATE_OPEN_FILE;
                    DRV_CODEC_MicSet(appData.codecClientRead.handle, MIC_SELECT);
//                    DRV_CODEC_SamplingRateSet(appData.codecClientRead.handle, AUDIO_ENCODE_SAMPLE_RATE);

//                    DRV_CODEC_MicrophoneSoundSet(appData.codecClientRead.handle, MONO_LEFT_CHANNEL);
                    // change prompt text string to start recording
                    APP_DisplayStartRecord();
                } else {
                    SYS_DEBUG(0, "Find out whats wrong \r\n");
                }
            }
            break;
        case APP_STATE_OPEN_FILE:
        {
            _init_runtime_encoder();
            /* Try opening the file for append */
            // construct output file name
            char name[128] = "/mnt/myDrive1/encoder.";

            strcat(name, extension_str[appData.runTimeAudioType]);

            appData.fileHandle = SYS_FS_FileOpen(name, (SYS_FS_FILE_OPEN_WRITE));
            if (appData.fileHandle == SYS_FS_HANDLE_INVALID) {
                /* Could not open the file. Error out*/
                appData.state = APP_STATE_ERROR;
            } else {
                /* File opened successfully. Write to file */
                appData.state = APP_STATE_CODEC_SET_BUFFER_HANDLER;

            }
        }
            break;

        case APP_STATE_CODEC_SET_BUFFER_HANDLER:
            DRV_CODEC_BufferEventHandlerSet(appData.codecClientRead.handle,
                    appData.codecClientRead.bufferHandler,
                    appData.codecClientRead.context);
            appData.state = APP_STATE_START_RECORD;
            break;

        case APP_STATE_START_RECORD:
            if (appData.record) {
                // get runtime encoder type from input system,
                // use macro for now
                encoded_data_size = 0;
                
                appData.state = APP_STATE_PREPARE_ENCODING;
                memset(pheader, 0, AUDIO_FILE_METADATA_HEADER_SIZE);
                // change prompt text string to how stop recording
                APP_DisplayStopRecord();
            }
            break;

        case APP_STATE_PREPARE_ENCODING:
            
            
            switch (appData.runTimeAudioType) {
                case WAV_RIFF_PCM_FILE:
#ifdef PCM_ENCODER
                    si.sample_rate = AUDIO_ENCODE_SAMPLE_RATE;
                    si.channel = 2; // stereo
                    si.bit_depth = 16;
                    si.bps = si.sample_rate * si.channel * si.bit_depth;
                    if (runtimeEncoderInst->enc_init(si.channel, si.sample_rate)) {
                        appData.state = APP_SUBMIT_INITIAL_CODEC_READ_REQUEST;
                    }
#endif
                    break;
                case WAV_RIFF_ADPCM_FILE:
#ifdef ADPCM_ENCODER
                    si.sample_rate = AUDIO_ENCODE_SAMPLE_RATE;
                    si.channel = 2; // stereo
                    si.bit_depth = 16;
                    si.bps = si.sample_rate * si.channel * si.bit_depth;
                    if (runtimeEncoderInst->enc_init(si.channel, si.sample_rate)) {
                        appData.state = APP_SUBMIT_INITIAL_CODEC_READ_REQUEST;
                    }
#endif
                    break;

                case OGG_OPUS_FILE:
#ifdef OPUS_ENCODER 
                    // For Ogg Container, first initialize container
                    // then initialize Encoder

                    // for this demonstration, a single logical stream is enough
                    if (ogg_container_stream_init(0x04030201,
                            (OGG_CONTAINER_PAGEOUT_CALLBACK) APP_Ogg_Container_PageOut,
                            NUM_PACKETS_TO_ONE_PAGE)) {
                        
                        si.sample_rate = AUDIO_ENCODE_SAMPLE_RATE;
                        si.channel = 2; // stereo
                        si.bit_depth = 16;
                        si.bps = si.sample_rate * si.channel * si.bit_depth;

//                        encoder_set_stream_info(si);
//                        encoder_init(OPUS);
                        runtimeEncoderInst->enc_init(si.channel, si.sample_rate);
                        size = _construct_audio_file_header(
                                runtimeEncoderInst->encoderType, 
                                &si, 
                                0, 
                                pheader,
                                AUDIO_FILE_METADATA_HEADER_SIZE);
                        // write Ogg_Opus header
                        if (SYS_FS_FileWrite(appData.fileHandle,
                                (const void *) pheader, size) == -1) {
                            /* Write was not successful. Close the file
                             * and error out.*/
                            SYS_FS_FileClose(appData.fileHandle);
                            appData.state = APP_STATE_ERROR;

                        } else {
                            appData.state = APP_SUBMIT_INITIAL_CODEC_READ_REQUEST;
                        }
                    } else {
                        appData.state = APP_STATE_ERROR;
                    }
#endif
                    break;
                case OGG_SPEEX_FILE:
#ifdef SPEEX_ENCODER 
                    // For Ogg Container, first initialize container
                    // then initialize Encoder

                    // for this demonstration, a single logical stream is enough
                    if (ogg_container_stream_init(0x04030201,
                            (OGG_CONTAINER_PAGEOUT_CALLBACK) APP_Ogg_Container_PageOut,
                            NUM_PACKETS_TO_ONE_PAGE)) {
                        
                        si.sample_rate = AUDIO_ENCODE_SAMPLE_RATE; // support narrow band only
                        si.channel = 2; // stereo
                        si.bit_depth = 16;
                        si.bps = si.sample_rate * si.channel * si.bit_depth;

//                        encoder_set_stream_info(si);
//                        encoder_init(SPEEX);
                        runtimeEncoderInst->enc_init(si.channel, si.sample_rate);
                        size = _construct_audio_file_header(
                                runtimeEncoderInst->encoderType, 
                                &si, 
                                0, 
                                pheader,
                                AUDIO_FILE_METADATA_HEADER_SIZE);
                        // write Ogg_Speex header
                        if (SYS_FS_FileWrite(appData.fileHandle,
                                (const void *) pheader, size) == -1) {
                            /* Write was not successful. Close the file
                             * and error out.*/
                            SYS_FS_FileClose(appData.fileHandle);
                            appData.state = APP_STATE_ERROR;

                        } else {
                            appData.state = APP_SUBMIT_INITIAL_CODEC_READ_REQUEST;
                        }
                    } else {
                        appData.state = APP_STATE_ERROR;
                    }
#endif
                    break;

            }
            break;
        case APP_SUBMIT_INITIAL_CODEC_READ_REQUEST:

            DRV_CODEC_BufferAddRead(appData.codecClientRead.handle, &appData.codecClientRead.bufHandle1,
                    appData.codecClientRead.bufferObject1, appData.codecClientRead.bufferSize);

            DRV_CODEC_BufferAddRead(appData.codecClientRead.handle, &appData.codecClientRead.bufHandle2,
                    appData.codecClientRead.bufferObject2, appData.codecClientRead.bufferSize);

            if (appData.codecClientRead.bufHandle1 != DRV_CODEC_BUFFER_HANDLE_INVALID
                    && appData.codecClientRead.bufHandle2 != DRV_CODEC_BUFFER_HANDLE_INVALID) {
                appData.state = APP_PROCESS_DATA;
            }
            break;
        case APP_PROCESS_DATA:
            // user stops recording
            if (appData.record == false) {
                appData.state = APP_STATE_CLOSE_ENCODER;
            }
            // read1 is ready
            if (appData.codecClientRead.isCodecReadComplete1) {
                // encode process happens here
                outsize = 0;
//                memset(appData.codecClientRead.bufferObject1, 0x89, appData.codecClientRead.bufferSize);
                runtimeEncoderInst->enc_one_frame(appData.codecClientRead.bufferObject1, appData.codecClientRead.bufferSize, outBuffer[0], &outsize);
//                encode_one_frame(appData.codecClientRead.bufferObject1, appData.codecClientRead.bufferSize, outBuffer[0], &outsize);
//                encode_one_frame((appData.codecClientRead.bufferObject1+appData.codecClientRead.bufferSize/2), appData.codecClientRead.bufferSize/2, (outBuffer[0]+outsize), &outsize2);
//                outsize += outsize2;
                
//                _audio_frame_count++;
                _audio_frame_count++;

                switch (appData.runTimeAudioType) {
                    case WAV_RIFF_PCM_FILE:
                    case WAV_RIFF_ADPCM_FILE:
                        
                        // There is NO Encapsulation for WAV format file
                        if (SYS_FS_FileWrite(appData.fileHandle, (const void *) (outBuffer[0]), outsize) == -1) {
                            /* Write was not successful. Close the file
                             * and error out.*/
                            
                            SYS_FS_FileClose(appData.fileHandle);
                            appData.state = APP_STATE_ERROR;

                        }else
                        {
                            BSP_LEDToggle(APP_LED1);
                        }
                        break;
                        
                     

                    case OGG_OPUS_FILE:
#ifdef OPUS_ENCODER
                        // one frame per packet
                        ogg_container_encapsulate_packet(runtimeEncoderInst->encoderType, 
                                outBuffer[0],
                                outsize,
                                _audio_frame_count,
                                INPUT_SAMPLES,
                                AUDIO_ENCODE_SAMPLE_RATE,
                                appData.record == false);
#endif
                        break;
                    case OGG_SPEEX_FILE:
#ifdef SPEEX_ENCODER
                        ogg_container_encapsulate_packet(runtimeEncoderInst->encoderType, 
                                outBuffer[0],
                                outsize,
                                _audio_frame_count,
                                SPEEX_8K_STEREO_FRAMESIZE,
                                AUDIO_ENCODE_SAMPLE_RATE,
                                appData.record == false);
#endif
                        break;

                }

                encoded_data_size += outsize; //appData.codecClientRead.bufferSize;

                /* put the bufferObject1 in Audio Codec read queue. */
                DRV_CODEC_BufferAddRead(appData.codecClientRead.handle,
                        &appData.codecClientRead.bufHandle1,
                        appData.codecClientRead.bufferObject1,
                        appData.codecClientRead.bufferSize);
                appData.codecClientRead.isCodecReadComplete1 = false;
                if (appData.codecClientRead.bufHandle1 == DRV_CODEC_BUFFER_HANDLE_INVALID) {
                    appData.state = APP_STATE_ERROR;
                }


            }
            if (appData.state == APP_STATE_CLOSE_ENCODER)
                break;
            // read2 is ready
            if (appData.codecClientRead.isCodecReadComplete2) {
//                memset(appData.codecClientRead.bufferObject2, 0x58, appData.codecClientRead.bufferSize);
                runtimeEncoderInst->enc_one_frame(appData.codecClientRead.bufferObject2, appData.codecClientRead.bufferSize, outBuffer[1], &outsize);
                _audio_frame_count++;

                switch (appData.runTimeAudioType) {
                    case WAV_RIFF_PCM_FILE:
                    case WAV_RIFF_ADPCM_FILE:
                        
                        // There is NO Encapsulation for WAV format file
                        if (SYS_FS_FileWrite(appData.fileHandle, (const void *) (outBuffer[1]), outsize) == -1) {
                            /* Write was not successful. Close the file
                             * and error out.*/
                            
                            SYS_FS_FileClose(appData.fileHandle);
                            appData.state = APP_STATE_ERROR;
                        }else
                        {
                            BSP_LEDToggle(APP_LED1);
                        }
                        break;

                    case OGG_OPUS_FILE:
#ifdef OPUS_ENCODER

                        ogg_container_encapsulate_packet(OPUS, outBuffer[1],
                                outsize,
                                _audio_frame_count,
                                INPUT_SAMPLES,
                                si.sample_rate,
                                appData.record == false);
#endif
                        break;
                    case OGG_SPEEX_FILE:
#ifdef SPEEX_ENCODER
                        ogg_container_encapsulate_packet(SPEEX, outBuffer[1],
                                outsize,
                                _audio_frame_count,
                                INPUT_SAMPLES,//SPEEX_8K_STEREO_FRAMESIZE,
                                si.sample_rate,
                                appData.record == false);
#endif
                        break;

                }

                encoded_data_size += outsize; //appData.codecClientRead.bufferSize;

                /* put the bufferObject1 in Audio Codec read queue. */
                DRV_CODEC_BufferAddRead(appData.codecClientRead.handle,
                        &appData.codecClientRead.bufHandle2,
                        appData.codecClientRead.bufferObject2,
                        appData.codecClientRead.bufferSize);
                appData.codecClientRead.isCodecReadComplete2 = false;
                if (appData.codecClientRead.bufHandle2 == DRV_CODEC_BUFFER_HANDLE_INVALID) {
                    appData.state = APP_STATE_ERROR;
                }

            }
            break;
        
        case APP_STATE_CLOSE_ENCODER:
            if(runtimeEncoderInst->encoderType == PCM || runtimeEncoderInst->encoderType == ADPCM)
            {
                appData.state = APP_STATE_CONSTRUCT_AUDIO_FILE_HEADER;
            }
#ifdef MEB2_CONFIG
            else
            {
                
                // free Ogg Container structure
                ogg_container_stream_destroy();
                appData.state = APP_STATE_CLOSE_FILE;
            }
#endif
            break;
        case APP_STATE_CONSTRUCT_AUDIO_FILE_HEADER: // wrap in wav container
            // for WAV RIFF PCM FILE
            /** general stream properties **/
//            encoder_set_audio_data_size(encoded_data_size);
            size = _construct_audio_file_header(runtimeEncoderInst->encoderType,
                    &si, //encoder_get_stream_info(),
                    encoded_data_size,
                    pheader, AUDIO_FILE_METADATA_HEADER_SIZE);

            if (SYS_FS_FileSeek(appData.fileHandle, 0, SYS_FS_SEEK_SET) != -1) {
                // write wav header to file
                if (SYS_FS_FileWrite(appData.fileHandle, (const void *) pheader, size) == -1) {
                    SYS_FS_FileClose(appData.fileHandle);
                    appData.state = APP_STATE_ERROR;
                } else {

                    appData.state = APP_STATE_CLOSE_FILE;
                }
            } else {
                SYS_FS_FileClose(appData.fileHandle);
                appData.state = APP_STATE_ERROR;
            }
            break;
        case APP_STATE_CLOSE_FILE:
            /* Close the file */
            SYS_FS_FileClose(appData.fileHandle);
            APP_DisplaySavedFile();
            DRV_CODEC_Close(appData.codecClientRead.handle);
            if (runtimeEncoderInst->enc_free() == false) {
                appData.state = APP_STATE_ERROR;
            } else
                /* The test was successful. Lets idle. */
                appData.state = APP_STATE_IDLE;
            break;
        case APP_STATE_IDLE:

            /* The application comes here when the demo has completed
             * successfully. Provide LED indication. Wait for device detach
             * and if detached, wait for attach. */

            BSP_LEDOff(APP_LED3);
            BSP_LEDOn(APP_LED2);
            if (appData.deviceIsConnected == false) {
                APP_DisplayInsertUSB();
                appData.state = APP_STATE_WAIT_FOR_DEVICE_ATTACH;
                BSP_LEDOff(APP_LED2);
            }
            break;

        case APP_STATE_ERROR:

            /* The application comes here when the demo
             * has failed. Provide LED indication .*/

            APP_LEDS_ALL_ON();
            if (SYS_FS_Unmount("/mnt/myDrive") != 0 || runtimeEncoderInst->enc_free() == false) {
                /* The disk could not be un mounted. Try
                 * un mounting again untill success. */

                appData.state = APP_STATE_ERROR;
            } else {
                /* UnMount was successful. Wait for device attach */
                appData.state = APP_STATE_WAIT_FOR_DEVICE_ATTACH;
            }
            break;

        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Interface Functions for communicating with 
//          other components
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_StopRecord ( void )

  Remarks:
    See prototype in app.h.
 */
void APP_StopRecord() {
    appData.record = false;
}

/*******************************************************************************
  Function:
    void APP_StartRecord ( void )

  Remarks:
    See prototype in app.h.
 */
void APP_StartRecord() {
    appData.record = true;
}

void APP_DecrementEncoderSelectionIndex()
{
    (appData.encoderSelectIdx--);
    appData.encoderSelectIdx = (appData.encoderSelectIdx+SUPPORT_ENCODER_COUNT)%SUPPORT_ENCODER_COUNT;   
}

void APP_IncrementEncoderSelectionIndex()
{
    appData.encoderSelectIdx++;
    appData.encoderSelectIdx = (appData.encoderSelectIdx)%SUPPORT_ENCODER_COUNT;
}

void APP_ListWheelItemChanged(uint8_t idx)
{
    appData.encoderSelectIdx = idx;
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Static Functions implementation
// *****************************************************************************
// *****************************************************************************
static uint32_t _d_receive_complete = 0;

static void APP_CODECBufferReadEventHandler(DRV_CODEC_BUFFER_EVENT event,
        DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context) {
    switch (event) {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        {
            if (handle == appData.codecClientRead.bufHandle1) {
                appData.codecClientRead.isCodecReadComplete1 = true;
                _d_receive_complete++;
            } else if (handle == appData.codecClientRead.bufHandle2) {
                appData.codecClientRead.isCodecReadComplete2 = true;
                _d_receive_complete++;
            }


        }
        break;

        case DRV_CODEC_BUFFER_EVENT_ERROR:
        {
            appData.state = APP_STATE_CLOSE_FILE;
        }
            break;

        case DRV_CODEC_BUFFER_EVENT_ABORT:
        {
            appData.state = APP_STATE_CLOSE_FILE;
        }
            break;

    }
}
/*******************************************************************************
 End of File
 */
