/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c for usb_headset application

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
Copyright (c) 2013-2016 released Microchip Technology Inc.  All rights reserved.

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

#undef DEBUG_TONE_CODEC_TX
//#define DEBUG_TONE_USB_TX


uint32_t Debug_var;

//QUEUE Initialization 
//N, where N gives the fraction of the QUEUE to be filled, i.e. 1/N
//and USB reads generated and completed and Codec AddWrites before
//transitioning to APP_PROCESS_DATA timing. 
#define QUEUE_USB_INIT_PART   2  

bool queueFull;
bool queueEmpty;



//NOTE: Cache coherency and 16 byte alignment required for MZ processor,
//      -->as the cache page size on PIC32MZ is 16 bytes.
//      You don’t want to run into an issue where linker allocates the data 
//      structure in the same page as another data structure and then a line 
//      flush causes coherency issues.

//USB Tx Buffer
DRV_I2S_DATA16 APP_MAKE_BUFFER_DMA_READY 
    txBuffer[2][APP_MAX_NO_OF_SAMPLES_IN_A_USB_FRAME];

//USB TX (Write) buffer
DRV_I2S_DATA16 APP_MAKE_BUFFER_DMA_READY 
    mTxBuffer[2][APP_MAX_NO_OF_SAMPLES_IN_A_USB_FRAME]; //Stereo
//uint16_t mTxBuffer[2][APP_MAX_NO_OF_SAMPLES_IN_A_USB_FRAME];  //Mono

//Test buffers (sine tone)
//uint16_t txBuffer[APP_QUEUING_DEPTH][16];
uint16_t __attribute__((coherent)) __attribute__((aligned(16))) testBuffer1[16];
DRV_I2S_DATA16 __attribute__((coherent)) __attribute__((aligned(16))) testBuffer2[16];

//uint32_t timerStart, timerEnd;
//uint32_t cycles = 0;
volatile bool usbReadCompleteFlag = false;

//NOTE: APP_PLAYBACK BUFFER is what is transferred using DMA--> it is padded
// and placed at the beginning of the struct and the struct attribute is the
// allocated with the COHERENT and aligned(16) attributes so that it is 
// placed at the correct page boundary.
static __attribute__((coherent)) __attribute__((aligned(16))) 
    APP_PLAYBACK_BUFFER_QUEUE appPlaybackBuffer;

//==============================================================================
// Application Playback Buffer Queue
//==============================================================================
static void _APP_SetUSBReadBufferReady(USB_DEVICE_AUDIO_TRANSFER_HANDLE handle);
static void _APP_Init_PlaybackBufferQueue();
static void _APP_ClearCodecReturnBuffer(DRV_CODEC_BUFFER_HANDLE handle);
static uint8_t _APP_GetNextIdx(uint8_t index);
//static bool _APP_CodecBufferInUse();
static bool _APP_USBReadAllBufferReady();
static void _APP_DiscardTheEmptyChannelFromMicrophone(DRV_I2S_DATA16 *input, 
                                                      DRV_I2S_DATA16 *output, 
                                                      int size);
static bool hpInterfaceChanged = false;
static bool mpInterfaceChanged = false;
static USB_DEVICE_AUDIO_RESULT audioErr1, audioErr;
static uint32_t usbWriteError=0;

//Application Class Data
APP_DATA appData =
{
    /* Device Layer Handle  */
    .usbDevHandle = -1,

    /* USB Audio Instance index for this app object 0*/
    .audioInstance = 0,

     /* app state */
    .state = APP_STATE_INIT,

    /* device configured status */
    .isConfigured = false,

    /* Initialize active interface setting to 0. */
    .activeInterfaceAlternateSetting = APP_USB_SPEAKER_PLAYBACK_NONE,
    
    .activeMicInterfaceAlternateSetting = APP_USB_SPEAKER_PLAYBACK_NONE,

    /* DAC is not muted initially */
    .dacMute = false,
    
    .sampleFreq = 0xBB80, //48000 Hz
    //.sampleFreq = 0x3E80,    //16000 Hz
    //.sampleFreq = 0x7D00, //32000 Hz
    
    .USBReadBufSize = 192,
    
    .sampleFreqMic = 0x3E80,

    .codecClientWrite.context = (uintptr_t)0,
    .codecClientWrite.bufferHandler = 
        (DRV_CODEC_BUFFER_EVENT_HANDLER) APP_CODECBufferEventHandler,
    .codecClientWrite.bufferSize = 64,
    
    .codecClientRead.context = (uintptr_t)0,
    .codecClientRead.bufferHandler = 
        (DRV_CODEC_BUFFER_EVENT_HANDLER) APP_CODECBufferEventHandlerRead,
    .codecClientRead.bufferSize = 64,
    
    /* No Audio control in progress.*/
    .currentAudioControl = APP_USB_CONTROL_NONE
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
            }
            break;

        case USB_DEVICE_EVENT_SUSPENDED:
            /* Switch on green and orange, switch off red */

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
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:
            /* VBUS was removed. Notify USB stack about the event*/
            USB_DEVICE_Detach (appData.usbDevHandle);
        case USB_DEVICE_EVENT_ERROR:
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
                    if(appData.activeInterfaceAlternateSetting != interfaceInfo->interfaceAlternateSetting){
                        hpInterfaceChanged = true;
                        mpInterfaceChanged = false;
                        appData.state = APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD;
                        appData.activeInterfaceAlternateSetting = interfaceInfo->interfaceAlternateSetting;
                    }
                    
                }
                if(interfaceInfo->interfaceNumber == APP_RECORDING_INTERFACE)
                {
                    if(appData.activeMicInterfaceAlternateSetting != interfaceInfo->interfaceAlternateSetting){
                        mpInterfaceChanged = true;
                        hpInterfaceChanged = false;
                        appData.state = APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD;
                        appData.activeMicInterfaceAlternateSetting = interfaceInfo->interfaceAlternateSetting;
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
                appPlaybackBuffer.usbReadCompleteBufferLevel++;
                queueEmpty = false;

                //Check if the this is the initial loading of the playback
                //queue with USB Rx Data 
                if(appData.state == APP_SUBMIT_INITIAL_CODEC_WRITE_REQUEST)
                {
                    if(_APP_USBReadAllBufferReady()){
                        usbReadCompleteFlag = true;
                        APP_LED1_OFF();  //QUEUE Not Empty
                    }
                }
            }
            break;

            case USB_DEVICE_AUDIO_EVENT_WRITE_COMPLETE:
            {
                writeEventData = (USB_DEVICE_AUDIO_EVENT_DATA_WRITE_COMPLETE *)pData;
                //Audio frame was written to the Host.
                //Now get next frame from Audio Codec.
                if (writeEventData->handle == appData.writeTransferHandle1)
                {
                    appData.isUSBWriteComplete1 = true;
                }
                else if(writeEventData->handle == appData.writeTransferHandle2)
                {
                    appData.isUSBWriteComplete2 = true;
                }
                else
                {
                }
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
                           appData.currentAudioControl = APP_USB_AUDIO_SAMPLING_FREQ_CONTROL_MP;
                        }
                        
                        else if (((USB_AUDIO_ENDPOINT_CONTROL_REQUEST*)
                                pData)->endpointNumber == HEADPHONE_EP)
                        {
                           //A control write transfer received from Host. Now receive data from Host.
                           USB_DEVICE_ControlReceive(appData.usbDevHandle, 
                                                     (void *) &(appData.sampleFreq), 
                                                     3);
                           appData.currentAudioControl = APP_USB_AUDIO_SAMPLING_FREQ_CONTROL_HP;
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
                        USB_DEVICE_ControlStatus (appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
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
                           //A control write transfer received from Host. Now receive data from Host.
                           USB_DEVICE_ControlSend(appData.usbDevHandle, 
                                                  (void *)&(appData.sampleFreqMic), 
                                                  3);
                        }
                        
                        else if (((USB_AUDIO_ENDPOINT_CONTROL_REQUEST*)
                                       pData)->endpointNumber == HEADPHONE_EP)
                        {
                           //A control write transfer received from Host. Now receive data from Host.
                           USB_DEVICE_ControlSend(appData.usbDevHandle, (void *)&(appData.sampleFreq), 3 );
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
                USB_DEVICE_ControlStatus (appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            break;
            case USB_DEVICE_AUDIO_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:
            {
                USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK );

                if (appData.currentAudioControl == APP_USB_AUDIO_MUTE_CONTROL)
                {
                    appData.state = APP_MUTE_AUDIO_PLAYBACK;
                    appData.currentAudioControl = APP_USB_CONTROL_NONE;
                    //Handle Mute Control Here.
                }
                
                if (appData.currentAudioControl == APP_USB_AUDIO_SAMPLING_FREQ_CONTROL_HP)
                {
                    //DRV_CODEC_SamplingRateSet(appData.codecClientWrite.handle, 
                    //                          appData.sampleFreq);
                    if (appData.sampleFreq == SAMPLING_RATE_48000)
                    {
                        appData.codecClientWrite.bufferSize = 192;
                        appData.USBReadBufSize = 192;
                    }
                    else if (appData.sampleFreq == SAMPLING_RATE_32000)
                    {
                        appData.codecClientWrite.bufferSize = 128;
                        appData.USBReadBufSize = 192;
                    }
                    else if (appData.sampleFreq == SAMPLING_RATE_24000)
                    {
                        appData.codecClientWrite.bufferSize = 96;
                        appData.USBReadBufSize = 192;
                    }
                    else if (appData.sampleFreq == SAMPLING_RATE_16000)
                    {
                        appData.codecClientWrite.bufferSize = 64;
                        appData.USBReadBufSize = 192;
                    }
                    
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
                    //DRV_CODEC_SamplingRateSet(appData.codecClientWrite.handle, 
                    //                          appData.sampleFreqMic);

                    //NOTE:  buffersize is in stereo samples (/2) in RTOS version
                    if (appData.sampleFreqMic == SAMPLING_RATE_48000)
                    {
                        appData.codecClientRead.bufferSize = 192; //int16 values
                    }
                    else if (appData.sampleFreqMic == SAMPLING_RATE_32000)
                    {
                        appData.codecClientRead.bufferSize = 128;
                    }
                    else if (appData.sampleFreqMic == SAMPLING_RATE_24000)
                    {
                        appData.codecClientRead.bufferSize = 96;
                    }
                    else if (appData.sampleFreqMic == SAMPLING_RATE_16000)
                    {
                         appData.codecClientRead.bufferSize = 64;
                    }
                    
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
    //USB Read Completed (not used)
    appData.isReadComplete1 = false;
    appData.isReadComplete2 = false;
    
    //Codec Event Write Completed
    appData.isWriteComplete1 = false;
    appData.isWriteComplete2 = false;
    
    //USB Event Write Complete
    appData.isUSBWriteComplete1 = false;
    appData.isUSBWriteComplete2 = false;

    //USB Device Handle (not used)
    appData.readTransferHandle1 = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;
    appData.readTransferHandle2 = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;

    //USB Device Write Transfer Handle
    //appData.writeTransferHandle1 = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;
    //appData.writeTransferHandle2 = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;

    appData.codecClientRead.txbufferObject1 = (uint8_t *) txBuffer[0];
    appData.codecClientRead.txbufferObject2 = (uint8_t *) txBuffer[1];

    // (not used)
    appData.codecClientWrite.iswriteBufHandleAvailable1 = false;
    appData.codecClientWrite.iswriteBufHandleAvailable2 = false;
    
    //Sine Tone Test Buffer
    //Stereo
    testBuffer2[0].leftData = 0;
    testBuffer2[0].rightData = 0;
    testBuffer2[1].leftData  = 0x30FB;
    testBuffer2[1].rightData  = 0x30FB;
    testBuffer2[2].leftData  = 0x5A82;
    testBuffer2[2].rightData  = 0x5A82;
    testBuffer2[3].leftData  = 0x7641;
    testBuffer2[3].rightData  = 0x7641;
    testBuffer2[4].leftData  = 0x7FFF;
    testBuffer2[4].rightData  = 0x7FFF;
    testBuffer2[5].leftData  = 0x7641;
    testBuffer2[5].rightData  = 0x7641;
    testBuffer2[6].leftData  = 0x5A82;
    testBuffer2[6].rightData  = 0x5A82;
    testBuffer2[7].leftData  = 0x30FB;
    testBuffer2[7].rightData  = 0x30FB;
    testBuffer2[8].leftData  = 0x0;
    testBuffer2[8].rightData  = 0x0;
    testBuffer2[9].leftData  = 0xCF05;
    testBuffer2[9].rightData  = 0xCF05;
    testBuffer2[10].leftData  = 0xA57E;
    testBuffer2[10].rightData  = 0xA57E;
    testBuffer2[11].leftData  = 0x89BF;
    testBuffer2[11].rightData  = 0x89BF;
    testBuffer2[12].leftData  = 0x8001;
    testBuffer2[12].rightData  = 0x8001;
    testBuffer2[13].leftData  = 0x89BF;
    testBuffer2[13].rightData  = 0x89BF;
    testBuffer2[14].leftData  = 0xA5FE;
    testBuffer2[14].rightData  = 0xA5FE;
    testBuffer2[15].rightData  = 0xCF05;
    testBuffer2[15].leftData  = 0xCF05;
    
    //Sine Tone Test Buffer
    //Mono
    testBuffer1[0] = 0;
    testBuffer1[1] = 0x187E;
    testBuffer1[2] = 0x2D41;
    testBuffer1[3] = 0x3B20;
    testBuffer1[4] = 0x4000;
    testBuffer1[5] = 0x3B20;
    testBuffer1[6] = 0x2D41;
    testBuffer1[7] = 0x187E;
    testBuffer1[8] = 0x0;
    testBuffer1[9] = 0xE782;
    testBuffer1[10] = 0xD2BF;
    testBuffer1[11] = 0xC4E0;
    testBuffer1[12] = 0xC000;
    testBuffer1[13] = 0xC4E0;
    testBuffer1[14] = 0xD2BF;
    testBuffer1[15] = 0xE782;
    
} //End APP_Initialize()


//******************************************************************************
// APP_Tasks()
//
// Application tasks routine. This function implements the
// application state machine.
//******************************************************************************
void APP_Tasks()
{
    int i;
    switch(appData.state)
    {
        //---------------------------------------------------------------------
        // Application's initial state. 
        //---------------------------------------------------------------------
        case APP_STATE_INIT:
        {    
            _APP_Init_PlaybackBufferQueue();
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

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
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
                // This means the driver can now be be opened.
                /* A client opens the driver object to get an Handle */
                appData.codecClientWrite.handle = 
                        DRV_CODEC_Open(DRV_CODEC_INDEX_0, 
                                       DRV_IO_INTENT_WRITE);
                       
                appData.codecClientRead.handle = 
                        DRV_CODEC_Open(DRV_CODEC_INDEX_0, 
                                       DRV_IO_INTENT_READ );
                
                if (appData.codecClientWrite.handle != DRV_HANDLE_INVALID 
                    && appData.codecClientRead.handle != DRV_HANDLE_INVALID) 
                {
                    appData.state = APP_STATE_CODEC_SET_BUFFER_HANDLER;
                }
                else
                {
                    SYS_DEBUG(0, "Find out whats wrong \r\n");
                }
            }
            else
            {
                /* Wait for CODEC to Initialize */
                ;
            }
        }
        break;
        
        //---------------------------------------------------------------------
        // Set a handler for the audio buffer completion event 
        // --Then APP_IDLE
        //---------------------------------------------------------------------
        case APP_STATE_CODEC_SET_BUFFER_HANDLER:
        {
            DRV_CODEC_BufferEventHandlerSet(appData.codecClientWrite.handle,
                        appData.codecClientWrite.bufferHandler,
                        appData.codecClientWrite.context);

            DRV_CODEC_BufferEventHandlerSet(appData.codecClientRead.handle,
                        appData.codecClientRead.bufferHandler,
                        appData.codecClientRead.context);

            //Enable SPI Data In
            PLIB_SPI_PinEnable(SPI_ID_1, SPI_PIN_DATA_IN);      

            appData.state = APP_IDLE;
        }
        break;

        //---------------------------------------------------------------------
        // Initial USB Read Request
        //
        // Transition From:
        //         APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD:
        //              --16Khz playback alternate setting
        //         APP_MUTE_AUDIO_PLAYBACK 
        //              --16Khz playback alternate setting
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
                     APP_USB_SPEAKER_PLAYBACK_STEREO_16KHZ)
            {
                _APP_Init_PlaybackBufferQueue();
                queueFull = false;
                queueEmpty = false;

                //To check alignment
                //int value1 = sizeof(APP_PLAYBACK_BUFFER)%16; 

                //Fill the queue with USB Read Data
                usbReadCompleteFlag = false;  //Actually USB Init Q Read Complete
                //for(i = 0;i < (APP_PLAYBACK_QUEUE_BUFFER_SIZE/4 + 8);i++)
                for(i = 0;i < APP_PLAYBACK_QUEUE_BUFFER_SIZE;i++)
                {
                    //USB Read to Head of Codec Playback Buffer
                    APP_PLAYBACK_BUFFER* writeBuffer = 
                            &appPlaybackBuffer.playbackBuffer[i];

                    if (writeBuffer != NULL && 
                        !writeBuffer->codecInUse && 
                        !writeBuffer->usbInUse)
                    {

                        writeBuffer->usbReadCompleted = false;
                        writeBuffer->usbInUse = true;

                        audioErr1 = USB_DEVICE_AUDIO_Read(USB_DEVICE_INDEX_0, 
                                                          &writeBuffer->usbReadHandle, 
                                                          1, writeBuffer->buffer, 
                                                          appData.USBReadBufSize); //64

                        if(audioErr1 != USB_DEVICE_AUDIO_RESULT_OK)
                        {
                            writeBuffer->usbInUse = false;
                            break;
                        }
                        else
                        {
                            appPlaybackBuffer.writeIdx = _APP_GetNextIdx(appPlaybackBuffer.writeIdx);
                        }

                        //Increment the Playback Queue HEAD
                        
                    }
                    else
                    {
                        //One of the buffers is ready for Codec Write
                        //usbReadCompleteFlag = true;
                    }
                }
                appData.state = APP_SUBMIT_INITIAL_CODEC_WRITE_REQUEST;
            }
            else
            {
                _APP_Init_PlaybackBufferQueue();
                APP_LED1_OFF();
                queueFull = false;
                queueEmpty = false;

            }
        }
        break;


        //---------------------------------------------------------------------
        // Initial Codec Write Request
        //
        // Transition From:
        //     APP_SUBMIT_INITIAL_USB_READ_REQUEST 
        //       -- initiates USB reads to all playback queue buffers.
        //     USB_DEVICE_AUDIO_EVENT_READ_COMPLETE:
        //       -- USB read completion
        //
        // Transition To: 
        //----------------------------------------------------------------------
        case APP_SUBMIT_INITIAL_CODEC_WRITE_REQUEST:
        {    
            if (appData.activeInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_STEREO_16KHZ
                && usbReadCompleteFlag)
            {
                for (i = 0;
                     i < APP_PLAYBACK_QUEUE_BUFFER_SIZE/QUEUE_USB_INIT_PART; i++)
                {
                int8_t readIdx = appPlaybackBuffer.readIdx;
                APP_PLAYBACK_BUFFER* current = &appPlaybackBuffer.playbackBuffer[readIdx];

                if(current->usbReadCompleted && !current->codecInUse)
                {
                    //Initial CODEC Write
                    current->codecInUse = true;
                    DRV_CODEC_BufferAddWrite(appData.codecClientWrite.handle, 
                                            &current->writeHandle,
                                            current->buffer, 
                                            appData.codecClientWrite.bufferSize);//  usbReadBufferSize

                   if(current->writeHandle != DRV_CODEC_BUFFER_HANDLE_INVALID)
                   {
                        appPlaybackBuffer.readIdx = _APP_GetNextIdx(readIdx);

                   }
                   else
                   {
                       current->codecInUse = false;
                       // CODEC doesn't have enough write buffers
                       // should never happen
                       Nop();
                   }
                }
                }
                appData.state = APP_PROCESS_DATA;
                usbReadCompleteFlag = false;
            }
        }
        break;

        //---------------------------------------------------------------------
        //  Initial Codec Read Mic Data
        //---------------------------------------------------------------------
        case APP_SUBMIT_INITIAL_CODEC_READ_REQUEST:
        {
            if (appData.activeMicInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_STEREO_16KHZ)
            {
                /*Submit Codec reads*/
                appData.isWriteComplete1 = false;
                appData.isWriteComplete2 = false;
                appData.isUSBWriteComplete1 = false;
                appData.isUSBWriteComplete2 = false;

                //Ping-Pong Buffer for Mic Read/USB Write
                DRV_CODEC_BufferAddRead(appData.codecClientRead.handle, 
                                        &appData.codecClientRead.writeBufHandle1,
                                        appData.codecClientRead.txbufferObject1, 
                                        appData.codecClientRead.bufferSize); //int16 values
            
                DRV_CODEC_BufferAddRead(appData.codecClientRead.handle, 
                                        &appData.codecClientRead.writeBufHandle2,
                                        appData.codecClientRead.txbufferObject2, 
                                        appData.codecClientRead.bufferSize); //int16 values

                if (appData.codecClientRead.writeBufHandle1 != DRV_CODEC_BUFFER_HANDLE_INVALID 
                    && appData.codecClientRead.writeBufHandle2 != DRV_CODEC_BUFFER_HANDLE_INVALID)
                {
                    appData.state = APP_PROCESS_DATA;
                }
            }
            appData.state = APP_PROCESS_DATA;
        }
        break;

        //---------------------------------------------------------------------
        // Process USB Read buffers to Codec Playback  and Codec read mic data to
        // USB Write buffers
        //
        // Transition From:
        //    APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD
        //    --When the alt settings are not really changed from NONE

        //    APP_SUBMIT_INITIAL_CODEC_WRITE_REQUEST:
        //    --When alt setting is APP_USB_SPEAKER_PLAYBACK_STEREO_16KHZ
        //       && usbReadCompleteFlag
        //---------------------------------------------------------------------
        case APP_PROCESS_DATA:
        {
            if (appData.activeInterfaceAlternateSetting == APP_USB_SPEAKER_PLAYBACK_STEREO_16KHZ)
            {
                //----------------------------
                // USB Read Request 
                APP_PLAYBACK_BUFFER* writeBuffer = 
                     &appPlaybackBuffer.playbackBuffer[appPlaybackBuffer.writeIdx];

                if (appPlaybackBuffer.usbReadCompleteBufferLevel == 
                    APP_PLAYBACK_QUEUE_BUFFER_SIZE)
                {
                    APP_LED2_ON();
                    APP_LED3_TOGGLE();
                    APP_LED4_TOGGLE();
                    queueFull = true;
                }

                if ( writeBuffer != NULL && 
                     !writeBuffer->usbInUse && 
                     !writeBuffer->codecInUse)
                {
                    writeBuffer->usbInUse = true;
                    writeBuffer->usbReadCompleted = false;

                    audioErr1 = USB_DEVICE_AUDIO_Read(USB_DEVICE_INDEX_0, 
                                          &writeBuffer->usbReadHandle, 
                                          1, writeBuffer->buffer,  
                                          appData.USBReadBufSize);//

                    if(audioErr1 == USB_DEVICE_AUDIO_RESULT_OK)
                    {
                    appPlaybackBuffer.writeIdx = _APP_GetNextIdx(appPlaybackBuffer.writeIdx);
                    
                    //Check for QUEUE Full
                    if (appPlaybackBuffer.readIdx == appPlaybackBuffer.writeIdx)
                    {
                            //USB Reads have caught up with Codec Writes
                            //--> Wait for next read, which may cause a skip.
                       queueFull = true;
                            APP_LED2_ON();
                       APP_LED3_TOGGLE();
                       APP_LED4_TOGGLE();
                    }
                    else
                    {
                            //APP_LED2_OFF();
                       queueFull = false;
                    }
                }
                    else
                    {
                        //USB Read Fail on this buffer
                        //--> Try again
                        writeBuffer->usbInUse = false;
                    }
                } //End USB Read Request 
                
                //------------------------- 
                //CODEC ADD WRITE
                //if (_APP_BufferInCodec())
                {
                    //Playback Codec is Ready (not in use)

                    int8_t readIdx = appPlaybackBuffer.readIdx;
                    APP_PLAYBACK_BUFFER* current = &appPlaybackBuffer.playbackBuffer[readIdx];

                    if(current->usbReadCompleted && !current->codecInUse)
                    {
                        //Codec has not written this buffer

                        if (appPlaybackBuffer.readIdx != appPlaybackBuffer.writeIdx)
                        {
                            //QUEUE is not full or empty
                            //APP_LED1_OFF();
                            //APP_LED2_OFF();
                        }
                        else
                        {
                            //Codec Write have caught up with USB Read
                            queueEmpty = true;
                            APP_LED1_ON();
                            APP_LED3_TOGGLE();
                            APP_LED4_TOGGLE();
                        }

                        current->codecInUse = true;
                        #ifdef DEBUG_TONE_CODEC_TX
                        //Write stereo tone to output instead of USB data.
                        DRV_CODEC_BufferAddWrite(appData.codecClientWrite.handle, 
                                                 &current->writeHandle,
                                                 testBuffer2, sizeof(testBuffer2));
                        #else
                        DRV_CODEC_BufferAddWrite(appData.codecClientWrite.handle, 
                                                 &current->writeHandle,
                                                 current->buffer, 
                                                 appData.codecClientWrite.bufferSize);
                        #endif
                        if (current->writeHandle != DRV_CODEC_BUFFER_HANDLE_INVALID)
                        {
                            
                            appPlaybackBuffer.readIdx = _APP_GetNextIdx(readIdx);

                            //Check for QUEUE Empty 
                            if (appPlaybackBuffer.readIdx == appPlaybackBuffer.writeIdx)
                            {
                               //QUEUE is empty do not execute USB Read command;
                               queueEmpty = true;
                               APP_LED1_ON();
                               APP_LED3_TOGGLE();
                               APP_LED4_TOGGLE();
                            }
                            else
                            {
                               APP_LED1_OFF();
                               queueEmpty = false;
                            }
                        }
                        else
                        {
                            current->codecInUse = false;
                           // CODEC doesn't have enough write buffers
                           // should never happen
                           Nop();
                        }
                    }
                    else
                    {
                        Nop();
                    }
                }
            }
            
            if (appData.activeMicInterfaceAlternateSetting == APP_USB_SPEAKER_PLAYBACK_STEREO_16KHZ)
            {
                if (appData.isUSBWriteComplete1)
                {
                    appData.isUSBWriteComplete1 = false;

                    DRV_CODEC_BufferAddRead(appData.codecClientRead.handle, 
                                            &appData.codecClientRead.writeBufHandle1,
                                            appData.codecClientRead.txbufferObject1, 
                                            appData.codecClientRead.bufferSize); //int16 values

                    if (appData.codecClientRead.writeBufHandle1 == DRV_CODEC_BUFFER_HANDLE_INVALID)
                    {
                        Nop();
                    }
                }
                else if(appData.isUSBWriteComplete2)
                {
                    
                    appData.isUSBWriteComplete2 = false;

                    DRV_CODEC_BufferAddRead(appData.codecClientRead.handle, 
                                            &appData.codecClientRead.writeBufHandle2,
                                            appData.codecClientRead.txbufferObject2, 
                                            appData.codecClientRead.bufferSize); //int16 values

                    if (appData.codecClientRead.writeBufHandle2 == DRV_CODEC_BUFFER_HANDLE_INVALID)
                    {
                        Nop();
                    }
                }
                        
                /* Submit write to the CODEC driver */
                if(appData.isWriteComplete1 == true)
                {
                    appData.isWriteComplete1 = false;
                    
                    //audioErr = USB_DEVICE_AUDIO_Write ( USB_DEVICE_INDEX_0 , &appData.writeTransferHandle1, 2 , &testBuffer1, 32 );

                    _APP_DiscardTheEmptyChannelFromMicrophone(
                              txBuffer[0], 
                              mTxBuffer[0],
                              appData.codecClientRead.bufferSize); //#int16 values
                    audioErr = USB_DEVICE_AUDIO_Write(
                              USB_DEVICE_INDEX_0, 
                              &appData.writeTransferHandle1, 
                              2 , mTxBuffer[0], 
                              appData.codecClientRead.bufferSize/2);//#int16 stereo samples

                    
                    if (audioErr != USB_DEVICE_AUDIO_RESULT_OK)
                    {
                        usbWriteError++;
                    }
                    else
                    {

                    }
                }
                else if(appData.isWriteComplete2 == true)
                {
                    appData.isWriteComplete2 = false;
                    
                    _APP_DiscardTheEmptyChannelFromMicrophone(
                            txBuffer[1], 
                            mTxBuffer[1],
                            appData.codecClientRead.bufferSize/4); //#int16 values

                    audioErr = USB_DEVICE_AUDIO_Write(
                            USB_DEVICE_INDEX_0, 
                            &appData.writeTransferHandle2, 
                            2, mTxBuffer[1], 
                            appData.codecClientRead.bufferSize/2); //#int16 stereo samples

                    if (audioErr != USB_DEVICE_AUDIO_RESULT_OK)
                    {
                        usbWriteError++;
                    }
                }
                
                
            }
        }
        break;

        case APP_MUTE_AUDIO_PLAYBACK:
        {
            if (appData.activeInterfaceAlternateSetting == 0 && 
                appData.activeMicInterfaceAlternateSetting == 
                   APP_USB_SPEAKER_PLAYBACK_NONE)
            {
                DRV_CODEC_MuteOn(appData.codecClientWrite.handle);
                appData.state = APP_IDLE;
            }
            else if (appData.activeInterfaceAlternateSetting == 1)
            {
                DRV_CODEC_MuteOff(appData.codecClientWrite.handle);
                appData.state = APP_SUBMIT_INITIAL_USB_READ_REQUEST;
            }
            
            else if (appData.activeMicInterfaceAlternateSetting == 
                     APP_USB_SPEAKER_PLAYBACK_STEREO_16KHZ)
            {
                DRV_CODEC_MuteOff(appData.codecClientWrite.handle);
                appData.state =  APP_SUBMIT_INITIAL_CODEC_READ_REQUEST;
            }
        }
        break;

        //RTOS Version
        case APP_SAMPLING_FREQUENCY_CHANGE:
        {
            //Changes sampling rate for Record and Playback (both stereo)
            DRV_CODEC_SamplingRateSet(appData.codecClientWrite.handle, 
                                              appData.sampleFreq);
            //Reinit the queue
            appData.state = APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD;
        }
        break;
        
        case APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD:
        {
           SYS_STATUS codecStatus;

           if ((appData.activeInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_NONE))
           {
               _APP_Init_PlaybackBufferQueue();
                queueFull = false;
                queueEmpty = true;
                APP_LED1_ON();     //QUEUE Empty
           }
           if ((appData.activeInterfaceAlternateSetting == 
                   APP_USB_SPEAKER_PLAYBACK_NONE) &&
                   (appData.activeMicInterfaceAlternateSetting == 
                   APP_USB_SPEAKER_PLAYBACK_NONE))
            {
               
                codecStatus = DRV_CODEC_Status(sysObjdrvCodec0);
                if (SYS_STATUS_READY == codecStatus)
                {
                        DRV_CODEC_MuteOn(appData.codecClientWrite.handle);
                        appData.state = APP_IDLE;
                }
            }
           else if (hpInterfaceChanged && 
                    appData.activeInterfaceAlternateSetting == 
                        APP_USB_SPEAKER_PLAYBACK_STEREO_16KHZ)//
            {
                codecStatus = DRV_CODEC_Status(sysObjdrvCodec0);
                if (SYS_STATUS_READY == codecStatus)
                {
                    DRV_CODEC_MuteOff(appData.codecClientWrite.handle);
                    appData.state =  APP_SUBMIT_INITIAL_USB_READ_REQUEST;
                }
                else
                {
                    break;
                }
            }
           else if (mpInterfaceChanged && 
                   appData.activeMicInterfaceAlternateSetting == 
                        APP_USB_SPEAKER_PLAYBACK_STEREO_16KHZ)//
            {
                codecStatus = DRV_CODEC_Status(sysObjdrvCodec0);
                if (SYS_STATUS_READY == codecStatus)
                {
                    DRV_CODEC_MuteOff(appData.codecClientWrite.handle);
                    appData.state =  APP_SUBMIT_INITIAL_CODEC_READ_REQUEST;
                }
                else
                {
                    break;
                }
            }else
            {
               appData.state = APP_PROCESS_DATA;
            }
        }
        break;

        case APP_IDLE:
        {
            if ((appData.activeInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_NONE) &&
                (appData.activeMicInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_NONE))
            {
                DRV_CODEC_MuteOn(appData.codecClientWrite.handle);
            }
        }
        break;

        case APP_STATE_ERROR:
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }

    //Save state for debug of transitions
    appData.lastState = appData.state;

} //End APP_Tasks()


//******************************************************************************
// APP_CODECBufferEventHandler
//
// This function is called back by the CODEC driver when
// a CODEC data buffer RX completes.
//******************************************************************************
void APP_CODECBufferEventHandlerRead(DRV_CODEC_BUFFER_EVENT event,
        DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context )
{
     switch(event)
    {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        {
//            Nop();
            if(handle == appData.codecClientRead.writeBufHandle1)
            {
                appData.isWriteComplete1 = true;
            }
            else if(handle == appData.codecClientRead.writeBufHandle2){
                appData.isWriteComplete2 = true;
            }
//            appData.state = APP_PROCESS_DATA;
        }
        break;
        case DRV_CODEC_BUFFER_EVENT_ERROR:
        {
            
        } break;

        case DRV_CODEC_BUFFER_EVENT_ABORT:
        {
        } break;
    }
    
} //End APP_CODECBufferEventHandlerRead()


//******************************************************************************
// APP_CODECBufferEventHandler()
//
// Application CODEC buffer Event handler.
// This function is called back by the CODEC driver when
// a CODEC data buffer TX completes.
//******************************************************************************
void APP_CODECBufferEventHandler(DRV_CODEC_BUFFER_EVENT event,
        DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context )
{
    switch(event)
    {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        {
#if 0  //This used only for speeding up the CODEC response

            int8_t readIdx = appPlaybackBuffer.readIdx;
            APP_PLAYBACK_BUFFER* current = &appPlaybackBuffer.playbackBuffer[readIdx];
            if (  current->usbReadCompleted && 
                 !current->codecInUse)
            {
                current->codecInUse = true;
                DRV_CODEC_BufferAddWrite(appData.codecClientWrite.handle, 
                                        &current->writeHandle,
                                        current->buffer, 
                                        appData.codecClientWrite.bufferSize);
//                DRV_CODEC_BufferAddWrite(appData.codecClientWrite.handle, 
//                                         &current->writeHandle,
//                                         testBuffer1, 128);
               
               
               if(current->writeHandle != DRV_CODEC_BUFFER_HANDLE_INVALID)
               {
                    
                    //CODEC Write buffer complete
                    appPlaybackBuffer.readIdx = _APP_GetNextIdx(readIdx);

                    //Check for QUEUE Empty 
                    if (appPlaybackBuffer.readIdx == appPlaybackBuffer.writeIdx)
                    {
                       //Queue is empty do not execute USB Read command;
                       queueEmpty = true;
                       APP_LED1_TOGGLE();
                    }
                    else
                    {
                       queueEmpty = false;
                    }

               }else{
                   current->codecInUse = false;
                   // CODEC doesn't have enough write buffers
                   // should never happen
                   Nop();
               }
                
            }else{
                // underrun
//                    underrun++;
                    Nop();
            }
#endif
            if (appPlaybackBuffer.readIdx != appPlaybackBuffer.writeIdx)
            {
                //QUEUE is not full or empty
                APP_LED1_OFF();
                APP_LED2_OFF();
            }

            //This buffer is ready for USB Write
            _APP_ClearCodecReturnBuffer(handle);
            appPlaybackBuffer.usbReadCompleteBufferLevel--;
            if (appPlaybackBuffer.usbReadCompleteBufferLevel == 0)
            {
                //USB Read needs to complete before next Codec Write.
                queueEmpty = true;
                APP_LED1_ON();
            }
        }
        break;
        case DRV_CODEC_BUFFER_EVENT_ERROR:
        {
        } break;

        case DRV_CODEC_BUFFER_EVENT_ABORT:
        {
        } break;

    }
}


//******************************************************************************
// _APP_SetUSBReadBufferReady()
//Set the USB Read Buffer Data Ready 
//******************************************************************************
static void _APP_SetUSBReadBufferReady(USB_DEVICE_AUDIO_TRANSFER_HANDLE handle)
{
    int i=0;
    for(i=0;i<APP_PLAYBACK_QUEUE_BUFFER_SIZE;i++)
    {
        if (appPlaybackBuffer.playbackBuffer[i].usbReadHandle == handle)
        {
            appPlaybackBuffer.playbackBuffer[i].usbReadCompleted = true;
            appPlaybackBuffer.playbackBuffer[i].usbInUse = false;
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

    for (i=0; i<APP_PLAYBACK_QUEUE_BUFFER_SIZE/QUEUE_USB_INIT_PART; i++)
    {
        if(appPlaybackBuffer.playbackBuffer[i].usbReadCompleted != true)
        {
            return false;
        }
    }
    return true;
}

//******************************************************************************
// _APP_Init_PlaybackBufferQueue()
//Initialize Codec Playback Buffer Queue
//******************************************************************************
static void _APP_Init_PlaybackBufferQueue()
{
    int i=0;
    appPlaybackBuffer.readIdx = 0;
    appPlaybackBuffer.writeIdx = 0;

    for(i=0;i<APP_PLAYBACK_QUEUE_BUFFER_SIZE;i++)
    {
        appPlaybackBuffer.playbackBuffer[i].codecInUse = false;
        appPlaybackBuffer.playbackBuffer[i].usbInUse = false;
        appPlaybackBuffer.playbackBuffer[i].usbReadCompleted = false;
    }
    appPlaybackBuffer.usbReadCompleteBufferLevel = 0;
}

//******************************************************************************
//_APP_GetNextIdx()
//Increment the Head or Tail Index to Codec Playback Buffer Queue
//******************************************************************************
static uint8_t _APP_GetNextIdx(uint8_t index)
{
    return (index+1)%APP_PLAYBACK_QUEUE_BUFFER_SIZE;
}

//******************************************************************************
// _APP_ClearCodecReturnBuffer()
//Completely Clear the Codec Playback Buffer Queue
//******************************************************************************
static void _APP_ClearCodecReturnBuffer(DRV_CODEC_BUFFER_HANDLE handle)
{
    int i = 0;
    for(i=0;i<APP_PLAYBACK_QUEUE_BUFFER_SIZE;i++)
    {
        if(appPlaybackBuffer.playbackBuffer[i].writeHandle == handle)
        {
            appPlaybackBuffer.playbackBuffer[i].writeHandle = DRV_CODEC_BUFFER_HANDLE_INVALID;
            appPlaybackBuffer.playbackBuffer[i].codecInUse = false;
            appPlaybackBuffer.playbackBuffer[i].usbInUse = false;
            appPlaybackBuffer.playbackBuffer[i].usbReadCompleted = false;
            appPlaybackBuffer.playbackBuffer[i].usbReadHandle = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;
        }
    }
}

#if 0
//******************************************************************************
//_APP_CodecBufferInUse()
//Check if Playback Codec is Ready for Data
//******************************************************************************
static bool _APP_CodecBufferInUse()
{
    int i;
    for(i=0;i<APP_PLAYBACK_QUEUE_BUFFER_SIZE;i++)
    {
        if(appPlaybackBuffer.playbackBuffer[i].codecInUse == true)
        {
            return false;
        }
    }
    return true;
}
#endif


//******************************************************************************
//_APP_DiscardTheEmptyChannelFromMicrophone()
//Create Mono from Stereo Channel buffer 
//******************************************************************************
static void _APP_DiscardTheEmptyChannelFromMicrophone(DRV_I2S_DATA16 *input, 
	                                              DRV_I2S_DATA16 *output, 
	                                              int numSamples)
{
    int i;
    int j = 0;

    for(i = 0;i<numSamples; i+=2)
    {
        // AK4642 daughter card microphone has only one valid channel (AK4953 too).
        // but stereo microphone mode is set for synchronizing playback and microphone. 
        // Only one channel has valid data, the other one is zero, but we don't know which one is the 
        // "REAL" left channel and right channel, by adding this two together, we can get 
        // that one channel data, then send this output buffer to USB.
        output[j].leftData =  input[i].leftData + input[i].rightData;
        output[j].rightData = input[i+1].leftData + input[i+1].rightData;
        j++;
    }
    
}

/*******************************************************************************
 End of File
 */
