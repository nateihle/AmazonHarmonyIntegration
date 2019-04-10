/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    appWR.c for usb_headset application

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

    The USB playback interface is a multifrequency stream with 16 bit stereo 
    data.  The USB record interface should be at the same frequency as playback
    (as set from the HOST) and is mono 16 bit.


 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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


//*****************************************************************************
//*****************************************************************************
// Section: Included Files
//*****************************************************************************
//*****************************************************************************

#include "app.h"

//DEBUG
#undef DEBUG_AUDIO_PATH
#undef DEBUG_CODEC  //Sin Tone Output
#define DEBUG_SAMPLECOUNT     14000
uint32_t Debug_var;
//DEBUG variables
int reportCnt = 0;
int start = 0;


typedef enum _ERROR_NUM {ERR_CODEC, ERR_CODEC_EVENT, ERR_USB} ERROR_NUM;
ERROR_NUM errorNum;
char * ERROR_STR[] = 
    { "Codec Not Ready", "Codec Event Error", "USB Not Ready"};


//Playback Buffer QUEUE Initialization 
//N, where N gives the fraction of the QUEUE to be filled, i.e. 1/N
//and USB reads generated and completed and Codec AddWrites before
//transitioning to APP_PROCESS_DATA timing. 
#define QUEUE_USB_INIT_PART   2  

bool queueFull;
bool queueEmpty;

uint32_t someVar;  //Codec context variable

DtdConfig config;

//******************************************************************************
// Acoustic Echo Canceller (AEC)
// 
//NOTE: Cache coherency and 16 byte alignment required for MZ processor,
//      -->as the cache page size on PIC32MZ is 16 bytes.
//      You don?t want to run into an issue where linker allocates the data 
//      structure in the same cache page (line) as another data structure and 
//      then a line flush causes coherency issues.

//MICBUFFERIDX micBufferIdx = MICBUFFER_NONE;
//DRV_I2S_DATA16 APP_MAKE_BUFFER_DMA_READY 
//    txBuffer[2][APP_MAX_NO_OF_SAMPLES_IN_A_USB_FRAME];

//Acoustic Echo Canceller 
//int16_t speakerBufferNum; //Playback Buffer associated with current mic input buffer

//uint32_t timerStart, timerEnd;
//uint32_t cycles = 0;

//All USB Read buffers ready for playback
volatile bool usbReadCompleteFlag = false;

USB_DEVICE_AUDIO_TRANSFER_HANDLE writeTransferHandle =
                                     USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;

//NOTE: APP_PLAYBACK BUFFER is what is transferred using DMA--> it is padded
// and placed at the beginning of the struct and the struct attribute is the
// allocated with the COHERENT and aligned(16) attributes so that it is 
// placed at the correct page boundary.
static __attribute__((coherent)) __attribute__((aligned(16))) 
    APP_BUFFER_QUEUE appWRQueue;

#ifdef DEBUG_CODEC
static __attribute__((coherent)) __attribute__((aligned(16))) 
    DRV_I2S_DATA16 sinBuffer[8] =
{
{0x0000, 0x0000},
{0x2d41, 0x2d41},
{0x4000, 0x4000},
{0x2d41, 0x2d41},
{0x0000, 0x0000},
{0xd2bf, 0xd2bf},
{0xc000, 0xc000},
{0xd2bf, 0xd2bf},
};
#endif


//Mono Reference playback buffer (ping-pong) 
//Speaker-Out 
q15 APP_MAKE_BUFFER_DMA_READY 
    xOutQ1d15[APP_MAX_NO_OF_SAMPLES_IN_A_USB_FRAME];

//Mono Mic buffer (ping-pong)  
//Mic-In
q15 APP_MAKE_BUFFER_DMA_READY 
    yInQ1d15[APP_MAX_NO_OF_SAMPLES_IN_A_USB_FRAME];

//Mono Mic buffer (ping-pong) after echo cancelling for USB TX (record) stream
//Mic-In-EchoEst
q15 APP_MAKE_BUFFER_DMA_READY 
    yOutQ1d15[APP_MAX_NO_OF_SAMPLES_IN_A_USB_FRAME];

q15 APP_MAKE_BUFFER_DMA_READY
    echoQ1d15[AECFRAMELENGTH];  //Echo estimate

//AEC Data
//--NOTE:  padded to 16 (TODO: Not used yet.)
q15 APP_MAKE_BUFFER_DMA_READY 
    normErrQXdX16[AECNUMESSTEPS+8];

//FIR filter state in oldest(K-1 delay) to newest(0 delay) order
q15 APP_MAKE_BUFFER_DMA_READY 
       xFirQ1d15[AECMAXTAPS];   //Reference delay buffer input

//AEC coefficients in oldest to newest delay order corresponding
//to the delay buffer
q15 APP_MAKE_BUFFER_DMA_READY 
       hCoefQXdX16[AECMAXTAPS];    //FIR filter taps (scaled by aec.hExp value)

APP_RW_BUFFER  * currentQueue;
DRV_I2S_DATA16 * currentPlayback;
DRV_I2S_DATA16 * currentRecord;
int16_t        * currentAecBuffer;

//AEC Initialization 
//--AecInit Memory Table for x,h and normalize error.
AecMemRec APP_MAKE_BUFFER_DMA_READY memtab[3] =
{
    {  //
        .size      = AECMAXTAPS,    // X delay buffer size
        .alignment = 16,            // Alignment requirement(MZ EF)
        .base      = (void *) xFirQ1d15,        // base address of the allocated buffer
    },
    {
        .size      = AECMAXTAPS,    // h filter size
        .alignment = 16,            // Alignment requirement(MZ EF)
        .base      = hCoefQXdX16,       // base address of the allocated buffer
    },
    {
        .size      = AECNUMESSTEPS, // normErrQXdX16 array size
        .alignment = 16,            // Alignment requirement(MZ EF)
        .base      = (void *) normErrQXdX16,       // base address of the allocated buffer
    },
};

//--AecInit Parameters
AecParam aecParam =
{
    .maxFirLen    = AECMAXTAPS,         //Maximum FIR length for this instance
    .numEsSteps   = AECNUMESSTEPS,      //Number of Exp. decaying mu steps.
    .echoQ1d15    = (void *) 0,
};

//NOTE:  Not used at this time
AecParams aecParams =
{
    .size = sizeof(AecParams),  //Size of this structure
    .maxAecTail = AECMAXTAPS,   //Acoustic echo tail length in samples
    .maxTxBlkDly = 0            //TX (acoustic) bulk delay in number of samples
};

AecConfig aecConfig =
{
    //Initial frame count to adapt without error scale
    .initAdaptCnt = AECSCALEINITCNT, 

    //Number of mono samples per frame
    //--The USB write/read buffers should be stereo @ the current sampling rate,
    //  as selected by the USB_HOST (either 48Khz normal or 8Khz for testing)
    //--The stereo samples will be summed to a mono channel,
    //  since the echo path is the combined echo path for the two speakers 
    //  summed at the microphone.
    .frameLength  = AECFRAMELENGTH,      // 8Khz sampling stereo to mono 16 bit --> 32/4 samples
    .firLen       = AECMAXTAPS,          //Initial length of FIR filter
    .flatSamp     = AECFLATSAMPLES,      //Flat delay time in samples
    .mu0Q1d15     = AECMU0Q1D15,         //Step size value (mu0) in range [0;1)
    .reverbQ1d15  = AECREVERBDECAYQ1D15, //Reverberation decay for 1/Fs period
    .echoQ1d15    = (q15 *) echoQ1d15,
};


//==============================================================================
// Application Playback Buffer Queue
//==============================================================================
static void    _APP_SetUSBReadBufferReady(USB_DEVICE_AUDIO_TRANSFER_HANDLE handle);
static void    _APP_SetUSBWriteBufferReady(USB_DEVICE_AUDIO_TRANSFER_HANDLE handle);
//static void    _APP_SetAecBufferReady(int idx);
static void    _APP_SetCodecWriteReadComplete(DRV_CODEC_BUFFER_HANDLE handle);
static void    _APP_Init_RWBufferQueue();
//static void    _APP_ClearCodecReturnBuffer(DRV_CODEC_BUFFER_HANDLE handle);
static uint8_t _APP_GetNextIdx(uint8_t index);
//static bool _APP_CodecBufferInUse();
static bool    _APP_USBReadAllBufferReady();
#if 0
static void    _APP_DiscardTheEmptyChannelFromMicrophone2(DRV_I2S_DATA16 *input, 
                                                      DRV_I2S_DATA16 *output, 
                                                      int size);
#endif
static void    _aecSumStereoChannels(DRV_I2S_DATA16 * input, 
                           q15 * output, int numSamples);
static bool hpInterfaceChanged = false;
static bool mpInterfaceChanged = false;
static USB_DEVICE_AUDIO_RESULT audioErrRead;
static USB_DEVICE_AUDIO_RESULT audioErrWrite;
//static uint32_t usbWriteError=0;

inline extern q31 ExpAvgPower(q31 powerQ1d31, q15 xFirQ1d15, int16_t lambExp);

//Application Class Data
APP_DATA appData =
{
    //Device Layer Handle  
    .usbDevHandle = -1,

    //USB Audio Instance index for this app object 0
    .audioInstance = 0,

     //app state
    .state = APP_STATE_INIT,

    //Device configured statu */
    .isConfigured = false,

    //Initialize active interface setting to 0.
    .activeInterfaceAlternateSetting = APP_USB_SPEAKER_PLAYBACK_NONE,
    
    .activeMicInterfaceAlternateSetting = APP_USB_SPEAKER_PLAYBACK_NONE,

    // DAC is not muted initially 
    .dacMute = false,
    
    //<STUB> - change to 48000 Hz after downsampling to AEC
    //NOTE:  The record and playback sample rates must be the same to
    //       match the codec sampling rate.
    //USB Sampling Frequency
    .sampleFreq = SAMPLING_RATE_8000, //8000 Hz
    //.sampleFreq = SAMPLING_RATE_48000, //48000 Hz
    .sampleFreqMic = SAMPLING_RATE_8000,

    .USBReadBufSize = APP_MAX_PLAYBACK_BUFFER_BYTES,  //1ms at 48000Hz

    .codecClientWriteRead.context = (uintptr_t)&someVar,
    .codecClientWriteRead.bufferHandler = 
        (DRV_CODEC_BUFFER_EVENT_HANDLER) APP_CODECBufferEventHandlerWriteRead,
    .codecClientWriteRead.bufferSize = 32,   //1ms @ 8000Hz 16bit-Stereo
    
    // No Audio control in progress.
    .currentAudioControl = APP_USB_CONTROL_NONE,

    .playbackActive = 0,
    .recordActive   = 0,

    .gfxDisable = false,
};


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************


//******************************************************************************
// APP_USBDeviceEventHandler()
//******************************************************************************
void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, 
                               void * pEventData, 
                               uintptr_t context )
{
    volatile USB_DEVICE_EVENT_DATA_CONFIGURED* configuredEventData;

    switch( event )
    {
        case USB_DEVICE_EVENT_RESET:
            break;

        case USB_DEVICE_EVENT_DECONFIGURED:
            // USB device is reset or device is de-configured.
            // This means that USB device layer is about to de-initialize
            // all function drivers. So close handles to previously opened
            // function drivers.

            // Also turn ON LEDs to indicate reset/de-configured state.
            /* Switch on red and orange, switch off green */
            APP_LED1_ON();
            APP_LED2_ON();
            APP_LED3_ON();
            APP_LED4_ON();
            APP_LED5_OFF();

            break;

        case USB_DEVICE_EVENT_CONFIGURED:
            /* check the configuration */
            configuredEventData = 
                     (USB_DEVICE_EVENT_DATA_CONFIGURED *)pEventData;
            if(configuredEventData->configurationValue == 1)
            {
                /* the device is in configured state */
                /* Switch on green and switch off red and orange */
                APP_LED1_OFF();
                APP_LED2_OFF();
                APP_LED3_OFF();
                APP_LED4_OFF();
                APP_LED5_ON();

                USB_DEVICE_AUDIO_EventHandlerSet(0,
                                                 APP_USBDeviceAudioEventHandler ,
                                                 (uintptr_t)NULL);
                /* mark that set configuration is complete */
                appData.isConfigured = true;
                SYS_MESSAGE("APP: USB Device Configured\r\n");
            }
            break;

        case USB_DEVICE_EVENT_SUSPENDED:
            /* Switch on green and orange, switch off red */

            SYS_MESSAGE("APP: USB Device Configured\r\n");
            APP_LED1_OFF();
            APP_LED2_OFF();
            APP_LED3_OFF();
            APP_LED4_OFF();
            APP_LED5_ON();

            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_POWER_DETECTED:
            /* VBUS was detected. Notify USB stack about the event */
            USB_DEVICE_Attach (appData.usbDevHandle);
            SYS_MESSAGE("APP: USB Device Attached\r\n");
            displayStats.usbConnected = true;
            displayStats.displayUpdate = true;
            display_tasks(&displayStats);

            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS was removed. Notify USB stack about the event*/
            SYS_MESSAGE("APP: USB Device DEtached\r\n");
            USB_DEVICE_Detach (appData.usbDevHandle);
            displayStats.usbConnected = false;
            displayStats.displayUpdate = true;
            display_tasks(&displayStats);
            break;

        case USB_DEVICE_EVENT_ERROR:
            SYS_MESSAGE("APP: USB Device ERROR\r\n");
            //Code ERROR
            errorNum = ERR_USB; 
            appData.state = APP_STATE_ERROR;
            break;

        default:
            break;
    }
} //End APP_USBDeviceEventHandler()


//******************************************************************************
// APP_USBDeviceAudioEventHandler()
//******************************************************************************
void APP_USBDeviceAudioEventHandler(USB_DEVICE_AUDIO_INDEX iAudio,
                                    USB_DEVICE_AUDIO_EVENT event ,
                                    void * pData,
                                    uintptr_t context)
{
    volatile USB_DEVICE_AUDIO_EVENT_DATA_INTERFACE_SETTING_CHANGED *interfaceInfo;
    volatile USB_DEVICE_AUDIO_EVENT_DATA_READ_COMPLETE *readEventData;
    volatile USB_DEVICE_AUDIO_EVENT_DATA_WRITE_COMPLETE *writeEventData;
    uint8_t entityID;
    uint8_t controlSelector;

    if ( iAudio == 0 )
    {
        switch (event)
        {
            case USB_DEVICE_AUDIO_EVENT_INTERFACE_SETTING_CHANGED:
                /* We have received a request from USB host to change the Interface-
                   Alternate setting.*/
                interfaceInfo = 
                   (USB_DEVICE_AUDIO_EVENT_DATA_INTERFACE_SETTING_CHANGED *)
                   pData;
                if(interfaceInfo->interfaceNumber == APP_PLAYBACK_INTERFACE)
                {
                    if (appData.activeInterfaceAlternateSetting != 
                        interfaceInfo->interfaceAlternateSetting)
                    {

                        hpInterfaceChanged = true;
                        mpInterfaceChanged = false;
                        appData.state = APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD;
                        SYS_PRINT("\r\nAPP: USB Audio Playback Interface Setting: %d\r\n",
                                       interfaceInfo->interfaceAlternateSetting);
                        appData.activeInterfaceAlternateSetting = 
                                       interfaceInfo->interfaceAlternateSetting;
                    }
                    
                }
                if(interfaceInfo->interfaceNumber == APP_RECORDING_INTERFACE)
                {
                    if (appData.activeMicInterfaceAlternateSetting 
                        != interfaceInfo->interfaceAlternateSetting)
                    {

                        hpInterfaceChanged = false;
                        mpInterfaceChanged = true;
                        appData.state = APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD;
                        SYS_PRINT("\r\nAPP: USB Audio Record Interface Setting: %d\r\n",
                                       interfaceInfo->interfaceAlternateSetting);
                        appData.activeMicInterfaceAlternateSetting = 
                                       interfaceInfo->interfaceAlternateSetting;
                    }
                    
                }

            break;

            case USB_DEVICE_AUDIO_EVENT_READ_COMPLETE:
            {
                //We have received an audio frame from the Host.
                //Now send this audio frame to Audio Codec for Playback.
                readEventData = 
                   (USB_DEVICE_AUDIO_EVENT_DATA_READ_COMPLETE *)pData;

                //Make USB Read Buffer Queue Ready for Data (Clear data flags)
                _APP_SetUSBReadBufferReady(readEventData->handle);
                appWRQueue.usbCompleteBufferLevel++;
                queueEmpty = false;

                //Check if the this is the initial loading of the playback
                //queue with USB Rx Data 
                if(appData.state == APP_SUBMIT_INITIAL_CODEC_WRITEREAD_REQUEST)
                {
                    if(_APP_USBReadAllBufferReady())
                    {
                        usbReadCompleteFlag = true; //ALL Ready for Codec Playback
                        APP_LED1_OFF();  //QUEUE Not Empty
                        SYS_MESSAGE("APP: USB READ Complete Queue READY\r\n");
                    }
                }
            }
            break;

            case USB_DEVICE_AUDIO_EVENT_WRITE_COMPLETE:
            {
                writeEventData = (USB_DEVICE_AUDIO_EVENT_DATA_WRITE_COMPLETE *)pData;
                _APP_SetUSBWriteBufferReady(writeEventData->handle);
            }
            break;
            
            case USB_DEVICE_AUDIO_EVENT_CONTROL_SET_CUR:
            {
                if(((USB_SETUP_PACKET*)pData)->Recipient == 
                     USB_SETUP_REQUEST_RECIPIENT_INTERFACE)
                {
                    entityID = 
                       ((USB_AUDIO_CONTROL_INTERFACE_REQUEST*)pData)->entityID;
                    if ((entityID == APP_ID_FEATURE_UNIT) || 
                        (entityID == APP_ID_FEATURE_UNIT_MICROPHONE) || 
                        (entityID == APP_ID_FEATURE_UNIT_SIDE_TONING)) 
                    {
                       controlSelector = 
                          ((USB_AUDIO_FEATURE_UNIT_CONTROL_REQUEST*)
                              pData)->controlSelector;
                       if (controlSelector == USB_AUDIO_MUTE_CONTROL)
                       {
                           //A control write transfer received from Host. 
                           //Now receive data from Host.
                           USB_DEVICE_ControlReceive(appData.usbDevHandle, 
                                                    (void *) &(appData.dacMute), 
                                                    1);
                           appData.currentAudioControl = APP_USB_AUDIO_MUTE_CONTROL;
                       }
                       
                    }
                    else if (entityID == APP_ID_MIXER_UNIT)
                    {
                        USB_DEVICE_ControlStatus(appData.usbDevHandle, 
                                                 USB_DEVICE_CONTROL_STATUS_ERROR);
                    }
                }
                else if (((USB_SETUP_PACKET*)pData)->Recipient == 
                           USB_SETUP_REQUEST_RECIPIENT_ENDPOINT)
                {
                    controlSelector = 
                       ((USB_AUDIO_ENDPOINT_CONTROL_REQUEST*)
                          pData)->controlSelector;

                    if (controlSelector == USB_AUDIO_SAMPLING_FREQ_CONTROL)
                    {
                        if (((USB_AUDIO_ENDPOINT_CONTROL_REQUEST*)
                                pData)->endpointNumber == MICROPHONE_EP)
                        {
                           //A control write transfer received from Host. Now receive data from Host.
                           USB_DEVICE_ControlReceive(appData.usbDevHandle, 
                                                     (void *) &(appData.sampleFreqMic), 
                                                     3);
                           SYS_MESSAGE("APP: USB Audio Sampling Freq Control Rcvd: MP\r\n");
                           appData.currentAudioControl = 
                                   APP_USB_AUDIO_SAMPLING_FREQ_CONTROL_MP;
                        }
                        
                        else if (((USB_AUDIO_ENDPOINT_CONTROL_REQUEST*)
                                pData)->endpointNumber == HEADPHONE_EP)
                        {
                           //A control write transfer received from Host. Now receive data from Host.
                           SYS_MESSAGE("APP: USB Audio Sampling Freq Control Rcvd: HP\r\n");
                           USB_DEVICE_ControlReceive(appData.usbDevHandle, 
                                                     (void *) &(appData.sampleFreq), 
                                                     3);
                           appData.currentAudioControl = 
                                   APP_USB_AUDIO_SAMPLING_FREQ_CONTROL_HP;
                        }
                    }
                }
            }
            break;

            case USB_DEVICE_AUDIO_EVENT_CONTROL_GET_CUR:
            {
                if (((USB_SETUP_PACKET*)pData)->Recipient == 
                     USB_SETUP_REQUEST_RECIPIENT_INTERFACE)
                {
                    entityID = ((USB_AUDIO_CONTROL_INTERFACE_REQUEST*)
                                  pData)->entityID;
                    if ((entityID == APP_ID_FEATURE_UNIT) || 
                         (entityID == APP_ID_FEATURE_UNIT_MICROPHONE) || 
                         (entityID == APP_ID_FEATURE_UNIT_SIDE_TONING))
                    {
                       controlSelector = 
                              ((USB_AUDIO_FEATURE_UNIT_CONTROL_REQUEST*)
                                            pData)->controlSelector;
                       if (controlSelector == USB_AUDIO_MUTE_CONTROL)
                       {
                           /*Handle Get request*/
                           USB_DEVICE_ControlSend(appData.usbDevHandle, 
                                                 (void *)&(appData.dacMute), 
                                                 1);
                       }
                    }
                    
                    else if (entityID == APP_ID_MIXER_UNIT)
                    {
                        USB_DEVICE_ControlStatus(
                                appData.usbDevHandle, 
                                USB_DEVICE_CONTROL_STATUS_ERROR);
                    }
                }
                else if (((USB_SETUP_PACKET*)pData)->Recipient == 
                           USB_SETUP_REQUEST_RECIPIENT_ENDPOINT)
                {
                    controlSelector = ((USB_AUDIO_ENDPOINT_CONTROL_REQUEST*)
                                         pData)->controlSelector;
                    if (controlSelector == USB_AUDIO_SAMPLING_FREQ_CONTROL)
                    {
                        if (((USB_AUDIO_ENDPOINT_CONTROL_REQUEST*)
                                pData)->endpointNumber == MICROPHONE_EP)
                        {
                           //A control write transfer received from Host. 
                           //Now receive data from Host.
                           USB_DEVICE_ControlSend(appData.usbDevHandle, 
                                                  (void *)&(appData.sampleFreqMic), 
                                                  3);
                        }
                        
                        else if (((USB_AUDIO_ENDPOINT_CONTROL_REQUEST*)
                                       pData)->endpointNumber == HEADPHONE_EP)
                        {
                           //A control write transfer received from Host. Now receive data from Host.
                           USB_DEVICE_ControlSend(
                                   appData.usbDevHandle, 
                                   (void *)&(appData.sampleFreq), 3 );
                        }
                    }
                }
            }
            break;

            case USB_DEVICE_AUDIO_EVENT_CONTROL_SET_MIN:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_GET_MIN:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_SET_MAX:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_GET_MAX:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_SET_RES:
            case USB_DEVICE_AUDIO_EVENT_CONTROL_GET_RES:
            case USB_DEVICE_AUDIO_EVENT_ENTITY_GET_MEM:
                /* Stall request */
                USB_DEVICE_ControlStatus(
                        appData.usbDevHandle, 
                        USB_DEVICE_CONTROL_STATUS_ERROR);
            break;

            case USB_DEVICE_AUDIO_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:
            {
                USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK );

                if (appData.currentAudioControl == APP_USB_AUDIO_MUTE_CONTROL)
                {
                    SYS_PRINT("APP:  USB Audio MUTE Control\r\n");
                    appData.state = APP_MUTE_AUDIO_PLAYBACK;
                    appData.currentAudioControl = APP_USB_CONTROL_NONE;
                    //Handle Mute Control Here.
                }
                
                if (appData.currentAudioControl == 
                    APP_USB_AUDIO_SAMPLING_FREQ_CONTROL_HP)
                {
                    //DRV_CODEC_SamplingRateSet(appData.codecClientWriteRead.handle, 
                    //                          appData.sampleFreq);
                    if (appData.sampleFreq == SAMPLING_RATE_48000)
                    {
                        appData.codecClientWriteRead.bufferSize = 192;
                    }
                    else if (appData.sampleFreq == SAMPLING_RATE_32000)
                    {
                        appData.codecClientWriteRead.bufferSize = 128;
                    }
                    else if (appData.sampleFreq == SAMPLING_RATE_24000)
                    {
                        appData.codecClientWriteRead.bufferSize = 96;
                    }
                    else if (appData.sampleFreq == SAMPLING_RATE_16000)
                    {
                        appData.codecClientWriteRead.bufferSize = 64;
                    }
                    else if (appData.sampleFreq == SAMPLING_RATE_8000)
                    {
                        appData.codecClientWriteRead.bufferSize = 32;
                    }
                    
                    SYS_PRINT("APP: HP  Fs=%d (buffer=%d)\r\n",
                            appData.sampleFreq,
                            appData.codecClientWriteRead.bufferSize);
                    //NOTE:  Change Sampling Frequency then Reinit Playback Queue
                    //appData.state = APP_MUTE_AUDIO_PLAYBACK;
                    //appData.state = APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD;
                    appData.state = APP_SAMPLING_FREQUENCY_CHANGE; //RTOS Version
                    appData.currentAudioControl = APP_USB_CONTROL_NONE;
                }
                else if (appData.currentAudioControl == 
                         APP_USB_AUDIO_SAMPLING_FREQ_CONTROL_MP)
                {
                    //This not in RTOS version
                    //DRV_CODEC_SamplingRateSet(appData.codecClientWriteRead.handle, 
                    //                          appData.sampleFreqMic);

                    //NOTE:  buffersize is in stereo samples (/2) in RTOS version
                    if (appData.sampleFreqMic == SAMPLING_RATE_48000)
                    {
                        appData.codecClientWriteRead.bufferSize = 192; //int16 values
                    }
                    else if (appData.sampleFreqMic == SAMPLING_RATE_32000)
                    {
                        appData.codecClientWriteRead.bufferSize = 128;
                    }
                    else if (appData.sampleFreqMic == SAMPLING_RATE_24000)
                    {
                        appData.codecClientWriteRead.bufferSize = 96;
                    }
                    else if (appData.sampleFreqMic == SAMPLING_RATE_16000)
                    {
                         appData.codecClientWriteRead.bufferSize = 64;
                    }
                    else if (appData.sampleFreqMic == SAMPLING_RATE_8000)
                    {
                         appData.codecClientWriteRead.bufferSize = 32;
                    }
                    SYS_PRINT("APP: MP  Fs=%d (buffer=%d)\r\n",
                            appData.sampleFreqMic,
                            appData.codecClientWriteRead.bufferSize);
                    
                    //NOTE:  The value received here is not used.  
                    //       Playback sets the sampling 
                    //       frequency.  The Mic frequency if different will
                    //       cause problems.
                    
                    //NOTE:  Change Sampling Frequency then Reinit Playback Queue
                    //appData.state = APP_MUTE_AUDIO_PLAYBACK;
                    //appData.state = APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD;
                    appData.state = APP_SAMPLING_FREQUENCY_CHANGE;
                    appData.currentAudioControl = APP_USB_CONTROL_NONE;
                }
            }
            break;

            case  USB_DEVICE_AUDIO_EVENT_CONTROL_TRANSFER_DATA_SENT:
            break;

            default:
                SYS_ASSERT ( false , "Invalid callback" );
            break;

        } //End switch (event)
    }//end of if  if ( iAudio == 0 )

}//End APP_AudioEventCallback()


//******************************************************************************
// APP_Initialize() - USB Playback and Record
//
// Application Initialize. It is called from the SYS_Initialized() function.
//******************************************************************************
void APP_Initialize()
{
    {
        char *initMessage = 
            "\r\nApplication created " __DATE__ " " __TIME__ " initialized!\r\n";
        SYS_CONSOLE_Write(SYS_CONSOLE_INDEX_0, STDOUT_FILENO, initMessage, 
                strlen(initMessage));
    }
    SYS_MESSAGE("----------------------------------------\r\n");
    SYS_MESSAGE("- Starting: usb_smart_speaker\r\n");
    SYS_MESSAGE("----------------------------------------\r\n");

    //Initialize and set the configuratio of the the AEC
    //--Initializes AEC object (memory parameters and canceller configuration)
    SYS_PRINT("APP: Initialize AEC with FrameLength %d and %d Taps\r\n",
            aecConfig.frameLength, aecConfig.firLen);

    config.frameLength            = AECFRAMELENGTH; 
    config.pathLossQ15            = DTDPATHLOSSQ1D15;
    config.refThreshQ31           = THRESHREFQ31;
    config.echoThreshQ31          = THRESHECHOQ31;
    config.micThreshQ31           = THRESHMICQ31;
    config.refDetectHoldoffFrames = REFDETECTHOLDOFFFRAMES;
    config.dtDetectHoldoffFrames  = DTDETECTHOLDOFFFRAMES;
    config.dtdWindowSamples       = DTDWINDOWSAMPLES;
    dtdInit(&appData.dtd, config);

    //Default Config initialization
    aecInit(&appData.aec, memtab, &aecParam);

    if (aecSetConfig(&appData.aec, &aecConfig) == 1)
    {
        SYS_DEBUG(0, "APP: ERROR - AEC Set Config \n");
    }

    appData.playbackActive = 0;
    appData.recordActive   = 0;
    appData.gfxDisable = false;

    SYS_PRINT("APP AEC: Fr=%d Taps=%d iCnt=%d flat=%d mu0=%6.2f rev=%8.6f\r\n",
            (appData.aec.frameLength), 
            (appData.aec.firLen),  
            (appData.aec.initCount),
            (appData.aec.flatSamp),
            (appData.aec.mu0Q1d15),
            (appData.aec.reverbQ1d15));

    display_init(&displayStats);
    
} //End APP_Initialize()


//******************************************************************************
// APP_Tasks()
//
// Application tasks routine. This function implements the
// application state machine.
//******************************************************************************
static APP_STATES histState[16];
static int histCnt = 0;

void APP_Tasks()
{
    int i;

    histState[histCnt++] = appData.state;
    if (histCnt == 16) histCnt = 0;

    switch(appData.state)
    {
        //---------------------------------------------------------------------
        // Application's initial state. 
        //---------------------------------------------------------------------
        case APP_STATE_INIT:
        {    
            SYS_MESSAGE("APP: APP_STATE_INIT\r\n");
            _APP_Init_RWBufferQueue();
            queueFull = false;
            queueEmpty = true;

            /* Open the device layer */

            //Open USB for Read/Write
            appData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,
                    DRV_IO_INTENT_READWRITE );

            if(appData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get 
                 * event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.usbDevHandle, 
                                           APP_USBDeviceEventHandler, 0);
                SYS_MESSAGE("APP: USB Device Layer Open\r\n");

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                //The Device Layer is not ready to be opened. 
                //--We should try again later.
                SYS_MESSAGE("APP: ERROR - USB Device Layer Not Ready\r\n");
            }
        }
        //Fall Through

        //---------------------------------------------------------------------
        // Wait for USB Connection to be configured
        //---------------------------------------------------------------------
        case APP_STATE_WAIT_FOR_CONFIGURATION:
        {
            //Check if Host has configured the Device.
            if (appData.isConfigured == true)
            {
                SYS_MESSAGE("APP: U, }SB Device Configured\r\n");
                appData.state = APP_STATE_CODEC_OPEN;
            }
        }
        break;

        //---------------------------------------------------------------------
        // Configure CODEC
        //
        // Transition from: APP_STATE_WAIT_FOR_CONFIGURATION
        //
        // Wait until: SYS_STATUS_READY 
        //
        // Transition To: 
        //
        //---------------------------------------------------------------------
        case APP_STATE_CODEC_OPEN:
        {
            SYS_STATUS codecStatus;
            codecStatus = DRV_CODEC_Status(sysObjdrvCodec0);

            if (SYS_STATUS_READY == codecStatus)
            {
                SYS_MESSAGE("APP: CODEC Ready\r\n");

                // A client opens the driver object to get an Handle 
                appData.codecClientWriteRead.handle = 
                        DRV_CODEC_Open(DRV_CODEC_INDEX_0,
                        DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_EXCLUSIVE);
                       
                if (appData.codecClientWriteRead.handle != DRV_HANDLE_INVALID) 
                {
                    appData.state = APP_STATE_CODEC_SET_BUFFER_HANDLER;
                }
                else
                {
                    SYS_DEBUG(0, "APP: ERROR - Can't open CODEC\r\n");
            errorNum = ERR_CODEC;
                    appData.state = APP_STATE_ERROR;
                }
            }
            else
            {
                //Wait for CODEC to Initialize
                Nop();
            }
        }
        break;
        
        //---------------------------------------------------------------------
        // Set a handler for the audio buffer completion event 
        // --Then APP_IDLE
        //---------------------------------------------------------------------
        case APP_STATE_CODEC_SET_BUFFER_HANDLER:
        {
            DRV_CODEC_BufferEventHandlerSet(
                        appData.codecClientWriteRead.handle,
                        appData.codecClientWriteRead.bufferHandler,
                        appData.codecClientWriteRead.context);

            //Enable SPI Data In
            PLIB_SPI_PinEnable(SPI_ID_1, SPI_PIN_DATA_IN);      

            SYS_MESSAGE("APP: CODEC Event Handlers Set\r\n");
            appData.state = APP_IDLE;
        }
        break;

        //---------------------------------------------------------------------
        // Initial USB Read Request
        //
        // Transition From:
        //         APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD:
        //              --16Khz playback alternate setting
        //       
        //       
        //       USB Audio Control: APP_USB_AUDIO_MUTE_CONTROL
        //       to state  APP_MUTE_AUDIO_PLAYBACK 
        //
        // Wait for 16Khz alt playback 
        // -->Initiate USB Rx to all playback queue buffers
        //
        // Transition To: 
        //     APP_SUBMIT_INITIAL_CODEC_WRITE_REQUEST
        //---------------------------------------------------------------------
        case APP_SUBMIT_INITIAL_USB_READ_REQUEST:
        {
            if (appData.activeInterfaceAlternateSetting == 
                     APP_USB_SPEAKER_PLAYBACK_STEREO)
            {
                SYS_MESSAGE("APP: USB Initial Read Queue Requests\r\n");
                _APP_Init_RWBufferQueue();
                queueFull = false;
                queueEmpty = false;

                //To check alignment for MZ Architecture:
                //int value1 = sizeof(APP_RW_BUFFER)%16; 

                //Fill the queue with USB Read Data
                usbReadCompleteFlag = false;  //Actually USB Init Q Read Complete
                for(i = 0;i < APP_QUEUE_SIZE;i++)
                {
                    //USB Read to Head of Codec Playback Buffer
                    currentQueue     = &(appWRQueue.buffer[i]);
                    currentPlayback  = (DRV_I2S_DATA16 *)
                                      (currentQueue->playbackbuffer);
                    currentAecBuffer = (currentQueue->aecbuffer);

                    if (currentQueue != NULL && 
                        !currentQueue->codecInUse && 
                        !currentQueue->usbInUse)
                    {

                        currentQueue->usbReadComplete = false;
                        currentQueue->usbInUse = true;

                        //Read Stereo Playback buffer
                        audioErrRead = USB_DEVICE_AUDIO_Read(
                                USB_DEVICE_INDEX_0, 
                                &currentQueue->usbReadHandle, 
                                1, //Interface # 
                                currentPlayback,
                                appData.USBReadBufSize); //64

            //NOTE:  Initially wrie a ZERO buffer since MIC 
            //       has not been accessed yet.
                        memset(currentAecBuffer, 0, 
                                appData.codecClientWriteRead.bufferSize/2);

                        //Write Mono Record Buffer - AEC
                        audioErrWrite = USB_DEVICE_AUDIO_Write(
                                USB_DEVICE_INDEX_0, 
                                &currentQueue->usbWriteHandle, 
                                2,  //Interface #
                                currentAecBuffer,  //Zeros
                                appData.codecClientWriteRead.bufferSize/2);

                        if (audioErrRead != USB_DEVICE_AUDIO_RESULT_OK)
                        {
                            currentQueue->usbInUse = false;
                            break;
                        }
                        else
                        {
                            appWRQueue.headIdx = _APP_GetNextIdx(appWRQueue.headIdx);
                        }
                    }
                    else
                    {
                        Nop();
                    }
                }
                appData.state = APP_SUBMIT_INITIAL_CODEC_WRITEREAD_REQUEST;
            }
            else
            {
                SYS_MESSAGE("APP: USB Muted--Init Playback queue\r\n");
                _APP_Init_RWBufferQueue();
                APP_LED1_OFF();
                queueFull = false;
                queueEmpty = false;

            }
        }
        break;


        //---------------------------------------------------------------------
        // Initial Codec WriteRead Request
        //
        // Transition From:
        //     APP_SUBMIT_INITIAL_USB_WRITEREAD_REQUEST
        //     USB_DEVICE_AUDIO_EVENT_READ_COMPLETE
        //
        // Wait: until both interfaces are active and usbReadCompletFlag
        //
        // Transition To: APP_PROCESS_DATA 
        //----------------------------------------------------------------------
        case APP_SUBMIT_INITIAL_CODEC_WRITEREAD_REQUEST:
        {    
            if ((appData.activeInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_STEREO) && 
                usbReadCompleteFlag)
            {
                appData.playbackActive = 1; 

                if (appData.activeMicInterfaceAlternateSetting == 
                        APP_USB_SPEAKER_PLAYBACK_STEREO) 
                {
                    appData.recordActive = 1;    
                }

                SYS_MESSAGE("APP: CODEC Initial WRITE/READ\r\n");
                for (i = 0;
                     i < APP_QUEUE_SIZE/QUEUE_USB_INIT_PART; 
                     i++)
                {
                    //int8_t headIdx = appWRQueue.headIdx; //Queue HEAD
                    int8_t tailIdx  = appWRQueue.tailIdx;   //Queue TAIL
                    currentQueue    = &(appWRQueue.buffer[tailIdx]);
                    currentPlayback = (DRV_I2S_DATA16 *)
                                          (currentQueue->playbackbuffer);
                    currentRecord   = (DRV_I2S_DATA16 *)
                                          (currentQueue->recordbuffer);

                    //Initiate write and read
                    if (currentQueue->usbReadComplete && 
                        currentQueue->usbWriteComplete && 
                        !currentQueue->codecInUse)
                    {
                        //Initial CODEC Write
                        DRV_CODEC_BufferAddWriteRead(
                                appData.codecClientWriteRead.handle, 
                                &(currentQueue->codecWriteReadHandle),
#ifdef DEBUG_CODEC
                                sinBuffer,
#else
                                currentPlayback,
#endif
                                currentRecord, 
                                appData.codecClientWriteRead.bufferSize); 
                       if (currentQueue->codecWriteReadHandle != 
                           DRV_CODEC_BUFFER_HANDLE_INVALID)
                       {
                           //++TAIL
                           currentQueue->codecInUse = true;
                           currentQueue->usbReadComplete = false;
                           currentQueue->usbWriteComplete = false;

                           //TAIL++  
                           appWRQueue.tailIdx = _APP_GetNextIdx(tailIdx);
                       }
                       else
                       {
                           currentQueue->codecInUse = false;
                       }
                    }
                } //End Initial Codec WR Queue Request Loop

                appData.state = APP_PROCESS_DATA;
                usbReadCompleteFlag = false;

            } //End Alternate Settings block
            else
            {
                appData.state = APP_SUBMIT_INITIAL_CODEC_WRITEREAD_REQUEST;
            }
        }
        break;

        //---------------------------------------------------------------------
        // Process USB Read buffers to Codec Playback  and Codec read mic data to
        // USB Write buffers
        //
        // The codec mic data runs through the echo canceller and is then sent
        // to the USB record ping-pong buffer
        //
        // Transition From:
        //    APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD
        //    --When the alt settings are not really changed from NONE

        //    APP_SUBMIT_INITIAL_CODEC_WRITEREAD_REQUEST:
        //    --When both alt setting is APP_USB_SPEAKER_PLAYBACK_STEREO
        //       && usbReadCompleteFlag
        //---------------------------------------------------------------------
        case APP_PROCESS_DATA:
        {
            //USB Playback/Record 
            if ((appData.activeInterfaceAlternateSetting == 
                 APP_USB_SPEAKER_PLAYBACK_STEREO) // &&
                //appData.activeMicInterfaceAlternateSetting == 
                //APP_USB_SPEAKER_PLAYBACK_STEREO
                )
            {

                appData.playbackActive = 1; //Enable AEC

                //----------------------------
                // USB Read and Write Request to HEAD
                int8_t headIdx = appWRQueue.headIdx;  //Queue HEAD
                //int8_t tailIdx  = appWRQueue.tailIdx;  //Queue TAIL
                currentQueue    = &appWRQueue.buffer[headIdx];
                currentPlayback = (DRV_I2S_DATA16 *) 
                                  (currentQueue->playbackbuffer);
                //currentRecord   = (DRV_I2S_DATA16 *) 
                //                  (currentQueue->recordbuffer);
                currentAecBuffer  = currentQueue->aecbuffer;

                if (appWRQueue.usbCompleteBufferLevel  == 
                    APP_QUEUE_SIZE)
                {
                    //SYS_PRINT("APP: CODEC Playback Queue FULL(%d)\r\n", 
                    //                       APP_QUEUE_SIZE);
                    APP_LED2_ON();
                    //APP_LED3_TOGGLE();
                    //APP_LED4_TOGGLE();
                    queueFull = true;
                }

                //Check Playback/Record QUEUE Buffers Ready 
                //--Playback and Record data buffers ready
                if ( currentPlayback != NULL && 
                     currentRecord != NULL && 
                     currentQueue->codecAecComplete)
                {

                    //USB Read (Playback stream) 
                    //NOTE:  Buffer size always at the maximum value (1ms @ 48000Hz)
                    audioErrRead  = USB_DEVICE_AUDIO_Read(
                                        USB_DEVICE_INDEX_0, 
                                        &currentQueue->usbReadHandle, 
                                        1, currentPlayback,  
                                        appData.USBReadBufSize);

                    //USB Write (Record stream)
                    if (appData.recordActive == 1) 
                    {
                        audioErrWrite = USB_DEVICE_AUDIO_Write(
                                            USB_DEVICE_INDEX_0, 
                                            &currentQueue->usbWriteHandle, 
                                            2, currentAecBuffer,
                                            appData.codecClientWriteRead.bufferSize/
                                                    APP_INT16_MONO_SAMPLE_BYTES);
                    }
                    else
                    {
                        audioErrWrite = USB_DEVICE_AUDIO_RESULT_OK;
                    }

                    if ((audioErrRead == USB_DEVICE_AUDIO_RESULT_OK) &&
                        (audioErrWrite== USB_DEVICE_AUDIO_RESULT_OK))
                    {
                        currentQueue->usbInUse         = true;
                        currentQueue->usbReadComplete  = false;
                        currentQueue->codecWRComplete  = false;
                        currentQueue->codecAecComplete = false;

                        if (appData.recordActive == 0)
                        {
                            //Don't wait for Write Complete
                            currentQueue->usbWriteComplete = true;
                        }
                        else
                        {
                            //Wait for write complete
                            currentQueue->usbWriteComplete = false;
                        }

                        //++HEAD (Next USB RW)
                        appWRQueue.headIdx = _APP_GetNextIdx(appWRQueue.headIdx);
                    }
                    else
                    {
                        //ERROR
                        currentQueue->usbInUse = false;
                    }
                    
                    //Check for QUEUE Full
                    if (appWRQueue.tailIdx == appWRQueue.headIdx)
                    {
                        APP_LED2_ON();
                        queueFull = true;
                    }
                    else
                    {
                        APP_LED2_OFF();
                        queueFull = false;
                    }
                } //End codecAecComplete (USB Read/Write)
            } //End Record and Playback Alternate Setting
                
            //------------------------- 
            //CODEC ADD WRITE/READ
            //--USB Write and Read Completed, ready for 
            //  speaker Playback/Mic Record
            {
                //int8_t headIdx = appWRQueue.headIdx; //Queue HEAD
                int8_t tailIdx  = appWRQueue.tailIdx;  //Queue TAIL
                currentQueue    = &(appWRQueue.buffer[tailIdx]);
                currentPlayback = (DRV_I2S_DATA16 *)
                                      (currentQueue->playbackbuffer);
                currentRecord   = (DRV_I2S_DATA16 *) 
                                      (currentQueue->recordbuffer);

                if ((currentQueue->usbWriteComplete) &&
                    (currentQueue->usbReadComplete) &&
                    (!currentQueue->codecInUse) &&
                    (!currentQueue->codecWRComplete) &&
                    (!currentQueue->codecAecComplete))
                {
                    //CODEC WRITE/READ Request
                    DRV_CODEC_BufferAddWriteRead(
                            appData.codecClientWriteRead.handle, 
                            &currentQueue->codecWriteReadHandle,
#ifdef DEBUG_CODEC
                                sinBuffer,
#else
                                currentPlayback,
#endif
                            currentRecord, 
                            appData.codecClientWriteRead.bufferSize);


                    if (currentQueue->codecWriteReadHandle != 
                        DRV_CODEC_BUFFER_HANDLE_INVALID)
                    {
                        
                        currentQueue->codecInUse       = true;
                        currentQueue->usbReadComplete  = false;
                        currentQueue->usbWriteComplete = false;
                        currentQueue->codecWRComplete  = false;
                        currentQueue->codecAecComplete = false;

                        //++TAIL  (Next Codec WR)
                        appWRQueue.tailIdx = _APP_GetNextIdx(appWRQueue.tailIdx);

                        //Check for QUEUE Empty 
                        if (appWRQueue.usbCompleteBufferLevel == 0)
                        {
                           //QUEUE is empty do not execute USB Read command;
                           SYS_PRINT("APP: ERROR - Playback Queue EMPTY(%d)", 
                                          appWRQueue.
                                              usbCompleteBufferLevel);
                           queueEmpty = true;
                           APP_LED1_ON();
                           //APP_LED3_TOGGLE();
                           //APP_LED4_TOGGLE();
                        }
                        else
                        {
                           APP_LED1_OFF();
                           queueEmpty = false;
                        }
                    }
                    else
                    {
                        currentQueue->codecInUse = false;
                    }
                } //End Codec WRITE/READ
                else
                {
                    Nop();
                }
            } //End USB Codec Playback/Record Request Codec AddWriteRead 
            
            //------------------------------------- 
            //MIC CODEC to USB RECORD BUFFER - AEC 
            if ((appData.activeInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_STEREO) &&
                (appData.activeMicInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_STEREO))
            {
                //int8_t headIdx = appWRQueue.headIdx; //Queue HEAD
                //int8_t tailIdx  = appWRQueue.tailIdx;  //Queue TAIL
                int8_t aecIdx  = appWRQueue.aecIdx;  //Queue TAIL
                currentQueue    = &(appWRQueue.buffer[aecIdx]);
                currentPlayback = (DRV_I2S_DATA16 *) 
                                       (currentQueue->playbackbuffer);
                currentRecord   = (DRV_I2S_DATA16 *) 
                                       (currentQueue->recordbuffer);
                currentAecBuffer = (currentQueue->aecbuffer);

                if (currentQueue->codecWRComplete)
                {


                    #ifdef CPUCORETIMER
                    //static unsigned int loopTimerValue; 
                    //unsigned int coreTimerValue; 
                    //unsigned int coreTimerDiff;
                    //unsigned int cycleCnt; 
                    //unsigned int timeUs;
                    //unsigned int temp;

                    //coreTimerValue = _CP0_GET_COUNT(); 
                    if (reportCnt == 0)
                    {
                        //loopTimerValue = _CP0_GET_COUNT();
                        //SYS_PRINT("APP: AEC(0) Queue WIX(%d) RIX(%d)\r\n",
                        //           headIdx, tailIdx);
                    }
                    else if (reportCnt < 10)
                    {
                        //temp = _CP0_GET_COUNT();
                        //coreTimerDiff = temp - loopTimerValue; 
                        //loopTimerValue = temp;
                        //cycleCnt = coreTimerDiff * 2; 
                        //timeUs = cycleCnt*1e6/SYS_CLK_FREQ;
                        //SYS_PRINT("APP: AEC(%d) WIX(%d) RIX(%d) CYCLES %d (%d Us)\r\n",
                        //                 reportCnt, 
                        //                 headIdx, tailIdx,
                        //                 cycleCnt, timeUs);
                    }
                    else
                    {
                        start = 0;
                    }
                    #endif

                    //TODO:  Record without Echo Playback.
                    
                    //--------------------------
                    //Acoustic Echo Cancelling
                    //--Discard empty channel and convert to mono buffer
                    //--Acoustic Echo Cancellation (if enabled)
                    //Stereo Mic audio Input to Mono 
                    _aecSumStereoChannels((DRV_I2S_DATA16 *) currentRecord, 
                                          yInQ1d15,
                                          appData.codecClientWriteRead.bufferSize/
                                                APP_INT16_STEREO_SAMPLE_BYTES);
    
                    _aecSumStereoChannels((DRV_I2S_DATA16 *) currentPlayback,
                                          xOutQ1d15,
                                          appData.codecClientWriteRead.bufferSize/
                                                APP_INT16_STEREO_SAMPLE_BYTES);
#ifndef DEBUG_AUDIO_PATH 
                    //-------------------------------------
                    //Test for double talk and active echo
                    if (appData.playbackActive == 1)
                    {
                        //appData.gfxDisable = true;
                        dtdProc(&appData.dtd, 
                                xOutQ1d15, 
                                yInQ1d15); 
                        //appData.dtd.dtDetected = false;
                        //appData.dtd.echoDetected = true;

                        if (!appData.dtd.dtDetected && appData.dtd.echoDetected)
                        {
                            //SYS_PRINT("Echo Detected\r\n");
                            //Cancel Echo/Adapt Filter 
                            //--Only echo and low-level near noise present at the 
                            //  microphone
        
                            aecSetAdaptEn(&appData.aec, 1);
                            aecSetCancelEn(&appData.aec, 1);
                            displayStats.dt = false;
                            displayStats.echo = true;
                        }
                        else if (appData.dtd.dtDetected && appData.dtd.echoDetected)
                        {
                            //Cancel Echo/Do NOT Adapt Filter
                            //--Near speech and Echo present at the microphone
                            //SYS_PRINT("DBLTalk Detected\r\n");
                            aecSetAdaptEn(&appData.aec, 0);
                            aecSetCancelEn(&appData.aec, 1);
                            displayStats.dt = true;
                            displayStats.echo = true;
                        }
                        else if (!appData.dtd.echoDetected)
                        {
                            //Do NOT Cancel Echo/Do NOT Adapt Filter
                            //--Only noise or near speech present at the microphone
                            aecSetAdaptEn(&appData.aec, 0);
                            aecSetCancelEn(&appData.aec, 1);
                            displayStats.dt = false;
                            displayStats.echo = false;
                        }
                        displayStats.displayUpdate = true;
                        display_tasks(&displayStats);

                        //Acoustic Echo Canceller
                        aecProc(&appData.aec, 
                                xOutQ1d15,          //SpeakerOut
                                yInQ1d15,           //MicIn
                                currentAecBuffer);  //MicIn-EchoEst
#ifdef DEBUG_SAMPLECOUNT
            if (appData.aec.sampleCount >= DEBUG_SAMPLECOUNT) 
                    {
                    //Breakpoint here to see state at a particular sample
                    SYS_PRINT("Sample Count Break");
                    }
#endif
                    } //End playbackActive AEC block
                    else 
                    {
                        //NO AEC -- Only microphone input
                        //--Reset the canceller
                        //aecReset(&appData.aec);
                        memcpy(currentAecBuffer, yInQ1d15,
                               APP_MAX_NO_OF_SAMPLES_IN_A_USB_FRAME*sizeof(q15));
                    }
#else  //NO Echo Canceller
                    //NO ECHO CANCELLER
                    //Stereo Mic audio to Mono USB
                    //_APP_DiscardTheEmptyChannelFromMicrophone2(
                    //          (DRV_I2S_DATA16 *) yInQ1d15,
                    //          (DRV_I2S_DATA16 *) currentAecBuffer, 
                    //          appData.codecClientWriteRead.bufferSize/
                    //                            APP_INT16_STEREO_SAMPLE_BYTES);
                    memcpy(currentAecBuffer, yInQ1d15, 
                                appData.codecClientWriteRead.bufferSize/
                                                APP_INT16_STEREO_SAMPLE_BYTES);
#endif
                    //++AEC  (Next AEC buffer)
                    appWRQueue.aecIdx = _APP_GetNextIdx(aecIdx);
                    currentQueue->codecAecComplete       = true;
                    currentQueue->codecWRComplete        = false;
                    currentQueue->codecInUse             = false;

                } //End MIC CODEC to USB RECORD BUFFER - AEC 
            } //End Both Alternate Settings

        } //End case APP_PROCESS_DATA
        break;

        case APP_MUTE_AUDIO_PLAYBACK:
        {
            if ((appData.activeInterfaceAlternateSetting ==
                   APP_USB_SPEAKER_PLAYBACK_NONE) &&
                 (appData.activeMicInterfaceAlternateSetting == 
                   APP_USB_SPEAKER_PLAYBACK_NONE))
            {
                SYS_PRINT("APP: Mute Control ON\r\n");
                DRV_CODEC_MuteOn(appData.codecClientWriteRead.handle);
                reportCnt=0;
                appData.gfxDisable = false;
                appData.state = APP_IDLE;
            }
            else if (appData.activeInterfaceAlternateSetting ==
                   APP_USB_SPEAKER_PLAYBACK_STEREO) 
            {
                SYS_PRINT("APP: Mute Control OFF - INIT USB Reads\r\n");
                DRV_CODEC_MuteOff(appData.codecClientWriteRead.handle);
                appData.state = APP_SUBMIT_INITIAL_USB_READ_REQUEST;
                appData.playbackActive = 1;
                if (appData.activeMicInterfaceAlternateSetting == 
                   APP_USB_SPEAKER_PLAYBACK_STEREO)
                {
                    appData.recordActive   = 1;
                }
            }
            else if (appData.activeMicInterfaceAlternateSetting == 
                     APP_USB_SPEAKER_PLAYBACK_NONE)
            {
                appData.gfxDisable = false;
                appData.recordActive   = 0;
            }
            else if (appData.activeInterfaceAlternateSetting == 
                     APP_USB_SPEAKER_PLAYBACK_NONE)
            {
                appData.gfxDisable = false;
                appData.playbackActive = 0;
            }
        }
        break;

        //RTOS Version
        case APP_SAMPLING_FREQUENCY_CHANGE:
        {
            //Changes sampling rate for Record and Playback (both stereo)
            SYS_PRINT("APP: Change Sample Frequency(%d)\r\n",
                        appData.sampleFreq);
                    
            DRV_CODEC_SamplingRateSet(appData.codecClientWriteRead.handle, 
                                              appData.sampleFreq);
            //Reinit the queue
            appData.state = APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD;
        }
        break;
        
        case APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD:
        {
            SYS_STATUS codecStatus;

            if ((appData.activeInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_NONE)  ||
                (appData.activeMicInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_NONE))
            {
                SYS_PRINT("APP: AEC interface OFF - INIT PLAYBACK Queue\r\n");
                _APP_Init_RWBufferQueue();
                queueFull = false;
                queueEmpty = true;
                APP_LED1_ON();     //QUEUE Empty

                if (appData.activeInterfaceAlternateSetting == 
                        APP_USB_SPEAKER_PLAYBACK_NONE) 
        {
                    appData.playbackActive = 0;
        }
                if (appData.activeMicInterfaceAlternateSetting == 
                        APP_USB_SPEAKER_PLAYBACK_NONE) 
        {
                    appData.recordActive   = 0;
        }

                codecStatus = DRV_CODEC_Status(sysObjdrvCodec0);
                if (SYS_STATUS_READY == codecStatus)
                {
                if (appData.playbackActive == 0 && appData.recordActive == 0)
                {
                        DRV_CODEC_MuteOn(appData.codecClientWriteRead.handle);
                }    
                    reportCnt=0;

                    SYS_PRINT("APP: HP/MP Mute ON\r\n");
                    appData.state = APP_IDLE;
                }
            } //End !MP || !HP

            if (hpInterfaceChanged && 
                    appData.activeInterfaceAlternateSetting == 
                        APP_USB_SPEAKER_PLAYBACK_STEREO)//
            {
                codecStatus = DRV_CODEC_Status(sysObjdrvCodec0);
                if (SYS_STATUS_READY == codecStatus)
                {
                    appData.playbackActive = 1;

            //NOTE:  Playback without record allowed.
                    SYS_PRINT("APP: HP Interface ON - INIT USB Reads\r\n");
                    DRV_CODEC_MuteOff(appData.codecClientWriteRead.handle);
                    appData.state =  APP_SUBMIT_INITIAL_USB_READ_REQUEST;
                }
                else
                {
                    //Code ERROR
                    errorNum = ERR_CODEC;
                    appData.state = APP_STATE_ERROR;
                    break;
                }
            }

            if (mpInterfaceChanged && 
                     appData.activeMicInterfaceAlternateSetting == 
                        APP_USB_SPEAKER_PLAYBACK_STEREO)//
            {

                codecStatus = DRV_CODEC_Status(sysObjdrvCodec0);
                if (SYS_STATUS_READY == codecStatus)
                {
                    appData.recordActive   = 1;

                    SYS_PRINT("APP: MP Interface ON - INIT Codec Reads\r\n");
                    DRV_CODEC_MuteOff(appData.codecClientWriteRead.handle);
                    //appData.gfxDisable = true;
                    //appData.state =  APP_SUBMIT_INITIAL_CODEC_WRITEREAD_REQUEST;
                    if (appData.playbackActive == 1)
                    {
                        appData.state = APP_SUBMIT_INITIAL_USB_READ_REQUEST;
                        break;
                    }
                    else
                    {
                        //TODO:   Record without playback
                        //appData.state = APP_SUBMIT_INITIAL_CODEC_WRITEREAD_REQUEST;
                        appData.state = APP_IDLE;  //No Audio
                        break;
                    }
                }
            } //End Mic USB Interface Alt Setting
        } //End case APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD:
        break;

    //No Audio
        case APP_IDLE:
        {
            if ((appData.activeInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_NONE) &&
                (appData.activeMicInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_NONE))
            {
                //SYS_PRINT("APP: MP Mute ON\r\n");
                DRV_CODEC_MuteOn(appData.codecClientWriteRead.handle);
                appData.gfxDisable = false;
                reportCnt=0;
                appData.playbackActive = 0;
                appData.recordActive   = 0;
            }
        }
        break;

        case APP_STATE_ERROR:
        {
            /* The default state should never be executed. */
            static int printErr = 1;
            if (printErr == 1)
            {
                printErr = 0;
                SYS_PRINT(0,"APP ERROR:  %s", ERROR_STR[errorNum]);
                SYS_DEBUG(0,"APP ERROR:  HALT!!!");
            }
        }
        break;

        default:
        {
            SYS_DEBUG(0,"APP ERROR:  Invalid STATE -- HALT!!!");
            break;
        }
    }

    //Save state for debug of transitions
    appData.lastState = appData.state;

} //End APP_Tasks()


//******************************************************************************
// APP_CODECBufferEventHandlerWriteRead()
//
// Application CODEC buffer Event handler.
// This function is called back by the CODEC driver when
// a CODEC data buffer TX completes.
//******************************************************************************
void APP_CODECBufferEventHandlerWriteRead(DRV_CODEC_BUFFER_EVENT event,
        DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context )
{
    switch(event)
    {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        {
            if (appWRQueue.tailIdx != appWRQueue.headIdx)
            {
                //QUEUE is not full or empty
                APP_LED1_OFF();
                APP_LED2_OFF();
            }

            //This buffer is ready for USB Write/Read after AEC is performed
            //_APP_ClearCodecReturnBuffer(handle);
            _APP_SetCodecWriteReadComplete(handle);
            appWRQueue.usbCompleteBufferLevel--;
            if (appWRQueue.usbCompleteBufferLevel <= 0)
            {
                //USB Read needs to complete before next Codec Write.
                appWRQueue.usbCompleteBufferLevel = 0;
                queueEmpty = true;
                APP_LED1_ON();
            }
        }
        break;

        case DRV_CODEC_BUFFER_EVENT_ERROR:
        {
            errorNum = ERR_CODEC_EVENT;
            appData.state = APP_STATE_ERROR;
        } 
        break;

        case DRV_CODEC_BUFFER_EVENT_ABORT:
        {
            errorNum = ERR_CODEC_EVENT;
            appData.state = APP_STATE_ERROR;
        } 
        break;

    }
} //End APP_CODECBufferEventHandlerWriteRead()


//******************************************************************************
// _APP_SetUSBReadBufferReady()
//Set the USB Read Buffer Data Ready 
//******************************************************************************
static void _APP_SetUSBReadBufferReady(USB_DEVICE_AUDIO_TRANSFER_HANDLE handle)
{
    int i=0;
    for(i=0;i<APP_QUEUE_SIZE;i++)
    {
        if (appWRQueue.buffer[i].usbReadHandle == handle)
        {
            appWRQueue.buffer[i].usbReadComplete = true;
            if (appWRQueue.buffer[i].usbWriteComplete == true)
            {
                //Both USB Read and Write requests are complete
                appWRQueue.buffer[i].usbInUse = false;
            }
            break;
        }
    }
}
//******************************************************************************
// _APP_SetUSBWriteBufferReady()
//Set the USB Write Buffer Data Ready 
//******************************************************************************
static void _APP_SetUSBWriteBufferReady(USB_DEVICE_AUDIO_TRANSFER_HANDLE handle)
{
    int i=0;
    for(i=0;i<APP_QUEUE_SIZE;i++)
    {
        if (appWRQueue.buffer[i].usbWriteHandle == handle)
        {
            appWRQueue.buffer[i].usbWriteComplete = true;
            if (appWRQueue.buffer[i].usbReadComplete == true)
            {
                //Both USB Read and Write requests completed
                appWRQueue.buffer[i].usbInUse         = false;
            }
            break;
        }
    }
}

//******************************************************************************
// _APP_USBReadAllBufferReady()
//    Check if USB Read Buffer Queue is Ready for CODEC Writes 
//******************************************************************************
static bool _APP_USBReadAllBufferReady()
{
    int i = 0;

    for (i=0; i<APP_QUEUE_SIZE/QUEUE_USB_INIT_PART; i++)
    {
        if(appWRQueue.buffer[i].usbReadComplete != true)
        {
            return false;
        }
    }
    return true;
}

//******************************************************************************
// _APP_Init_RWBufferQueue()
//Initialize Codec Playback Buffer Queue
//******************************************************************************
static void _APP_Init_RWBufferQueue()
{
    int i=0;
    appWRQueue.tailIdx = 0;
    appWRQueue.headIdx = 0;
    appWRQueue.aecIdx = 0;

    for(i=0;i<APP_QUEUE_SIZE;i++)
    {
        appWRQueue.buffer[i].codecInUse = false;
        appWRQueue.buffer[i].usbInUse = false;
        appWRQueue.buffer[i].usbReadComplete = false;
        appWRQueue.buffer[i].usbWriteComplete = false;
        appWRQueue.buffer[i].codecWRComplete = false;
        appWRQueue.buffer[i].codecAecComplete = false;
    }
    appWRQueue.usbCompleteBufferLevel = 0;
}

//******************************************************************************
//_APP_GetNextIdx()
//Increment the Head or Tail Index to Codec Playback Buffer Queue
//******************************************************************************
static uint8_t _APP_GetNextIdx(uint8_t index)
{
    return (index+1)%APP_QUEUE_SIZE;
}

//******************************************************************************
// _APP_SetCodecWriteReadComplete()
//Set the Codec Read Buffer Complete 
//******************************************************************************
static void _APP_SetCodecWriteReadComplete(DRV_CODEC_BUFFER_HANDLE handle)
{
    int i = appWRQueue.lastBufferOutIdx;

    for(i=0; i<APP_QUEUE_SIZE;i++)
    {
        if(appWRQueue.buffer[i].codecWriteReadHandle == handle)
        {
            appWRQueue.buffer[i].codecWRComplete = true; }
    }
}

#if 0
//******************************************************************************
// _APP_SetCodecReadComplete()
//Set the Codec Read Buffer Complete 
//******************************************************************************
static void _APP_SetAecBufferReady(int idx)
{
    appWRQueue.buffer[idx].codecAecComplete = true;
}



//******************************************************************************
// _APP_ClearCodecReturnBuffer()
//Completely Clear the Codec Playback Buffer Queue
//******************************************************************************
static void _APP_ClearCodecReturnBuffer(DRV_CODEC_BUFFER_HANDLE handle)
{
    int i = appWRQueue.lastBufferOutIdx;

    //TODO:  Since the circular queue buffers are played in sequence the next
    //       buffer to complete write output would be the next buffer in the
    //       circular sequence.  This could be implemented faster.
    for(i=0; i<APP_QUEUE_SIZE;i++)
    {
        if(appWRQueue.buffer[i].codecWriteReadHandle == handle)
        {
            appWRQueue.buffer[i].codecWriteReadHandle = 
                                      DRV_CODEC_BUFFER_HANDLE_INVALID;
            appWRQueue.buffer[i].codecInUse = false;
            appWRQueue.buffer[i].usbInUse = false;
            appWRQueue.buffer[i].codecWRComplete = false;
            appWRQueue.buffer[i].codecAecComplete = false;
            appWRQueue.buffer[i].usbReadComplete = false;
            appWRQueue.buffer[i].usbWriteComplete = false;
            appWRQueue.buffer[i].usbReadHandle = 
                                    USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;
            appWRQueue.buffer[i].usbWriteHandle = 
                                    USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;
            appWRQueue.lastBufferOutIdx = i;
        }
    }
}
#endif

#if 0
//******************************************************************************
//_APP_DupTheEmptyChannelFromMicrophone
//Create Full Stereo Buffer from One Channel stereo buffer (one empty channel)
//******************************************************************************
static void _APP_DupTheEmptyChannelFromMicrophone(DRV_I2S_DATA16 *input, 
                                                  DRV_I2S_DATA16 *output, 
                                                  int numSamples)
{
    int i;
    int j = 0;

    for(i = 0;i<numSamples; i+=2)
    {
        // AK4642 daughter card microphone has only one valid channel (AK4953 too).
        // but stereo microphone mode is set for synchronizing playback and 
        // microphone. 
        // Only one channel has valid data, the other one is zero, but we don't 
        // know which one is the 
        // "REAL" left channel and right channel, by adding this two together, 
        // we can get 
        // that one channel data, then send this output buffer to USB.
        output[j].leftData =  input[i].leftData + input[i].rightData;
        output[j].rightData = input[i+1].leftData + input[i+1].rightData;
        j++;
    }
    
}
#endif

#if 0
//******************************************************************************
// _APP_DiscardTheEmptyChannelFromMicrophone2()
// 
// Create Mono from Stereo Channel buffer 
//******************************************************************************
static void _APP_DiscardTheEmptyChannelFromMicrophone2(DRV_I2S_DATA16 *input, 
                                                      DRV_I2S_DATA16 *output, 
                                                      int numSamples)
{
    int i;
    int j = 0;

    for(i = 0;i<numSamples; i+=2)
    {
        // AK4642 daughter card microphone has only one valid channel 
        // (AK4953 too) but stereo microphone mode is set for synchronizing 
        // playback and microphone. 
        // Only one channel has valid data, the other one is zero, but we don't 
        // know which one is the "REAL" left channel and right channel, 
        // by adding this two together, we can get that one channel data, 
        // then send this output buffer to USB.
        output[j].leftData =  input[i].leftData + input[i].rightData;
        output[j].rightData = input[i+1].leftData + input[i+1].rightData;
        j++;
    }
    
}
#endif

//******************************************************************************
// _aecSumStereoChannels
//
// Summary:
//   Adds up the stereo playback channels to make a mono playback 
//
//   NOTE:  Same as _APP_DiscardTheEmptyChannelFromMicrophone()
//
// Arguments:
//    DRV_I2S_DATA16 *input   - [in] Pointer to stereo input buffer
//    q15            *output  - [in] Pointer to mono output buffer
//    int          numSamples - [in] #Stereo Samples 
//
// Return Value: //   None
//
//****************************************************************************/
static void _aecSumStereoChannels(DRV_I2S_DATA16 * input, 
                                  q15 * output, 
                                  int numSamples)
{
    int i;

    for(i = 0;i<numSamples; i++)
    {
        //The reference channel echos sum at the single mic input.
        output[i] =  input[i].leftData + input[i].rightData;
    }
}

/*******************************************************************************
 End of File
 */

