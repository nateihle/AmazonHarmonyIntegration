/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c for usb_host_headphone application

  Summary:
    This is an implementation of a USB Host Headphone Audio V1.0 Interface to an
    a Codec with Line-In and Line-Out.

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
Copyright (c) 2013-2017 released Microchip Technology Inc.  All rights reserved.

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

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

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

#define APP_LED_BLINK_COUNT 10000

#define APP_USB_SWITCH_DEBOUNCE_COUNT_FS 260

#undef  DEBUG_TONE_CODEC_TX  //Not used
#undef DEBUG_TONE_USB_TX
#undef DEBUG_USB_TX_ONLY

uint32_t samplingRate = 0;

//==============================================================================
//
//Playback QUEUE Initialization 
//N, where N gives the fraction of the QUEUE to be filled, i.e. 1/N
//and USB reads generated and completed and Codec AddWrites before
//transitioning to APP_PROCESS_DATA timing. 
#define QUEUE_USB_INIT_PART   2  

bool queueFull;
bool queueEmpty;
volatile bool usbReadCompleteFlag = false;
static USB_HOST_AUDIO_V1_RESULT audioErr1;

//NOTE: Cache coherency and 16 byte alignment required for MZ processor,
//      -->as the cache page size on PIC32MZ is 16 bytes.
// APP_PLAYBACK BUFFER is what is transferred using DMA--> it is padded
// and placed at the beginning of the struct and the struct attribute is the
// allocated with the COHERENT and aligned(16) attributes so that it is 
// placed at the correct page boundary.
//      You don?t want to run into an issue where linker allocates the data 
//      structure in the same page as another data structure and then a line 
//      flush causes coherency issues.
static __attribute__((coherent)) __attribute__((aligned(16))) 
    APP_PLAYBACK_BUFFER_QUEUE appPlaybackBuffer;

//------------------------------------------------------------------------------
// Application Playback Buffer Queue
//------------------------------------------------------------------------------
static void _APP_SetUSBReadBufferReady(
                  USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE handle);
static void _APP_Init_PlaybackBufferQueue();
static void _APP_ClearCodecReturnBuffer(DRV_CODEC_BUFFER_HANDLE handle);
static uint8_t _APP_GetNextIdx(uint8_t index);
static bool _APP_USBReadAllBufferReady();

//==============================================================================


//------------------------------------------------------------------------------
//USB Line-In Pin-Pong Buffers -- USB Tx Buffer
//------------------------------------------------------------------------------
DRV_I2S_DATA16 APP_MAKE_BUFFER_DMA_READY 
    txBuffer[2][APP_MAX_NO_OF_SAMPLES_IN_A_USB_FRAME];

//USB TX (Write) buffer
//DRV_I2S_DATA16 APP_MAKE_BUFFER_DMA_READY 
//    mTxBuffer[2][APP_MAX_NO_OF_SAMPLES_IN_A_USB_FRAME]; //Stereo

// The OUTPUT Audio Stream format for Line-In to Headphone
const APP_USB_HOST_AUDIO_STREAM_FORMAT audioSpeakerStreamFormat = 
{
    .streamDirection = USB_HOST_AUDIO_V1_DIRECTION_OUT,
    .format = USB_AUDIO_FORMAT_PCM,
    .nChannels = 2,
    .bitResolution = 16,
    .subFrameSize = 2,
    .samplingRate = AUDIO_SAMPLING_RATE
}; 


//==============================================================================
// The INPUT Audio Stream format for Microphone to Line-Out
const APP_USB_HOST_AUDIO_STREAM_FORMAT audioMicStreamFormat = 
{
    .streamDirection = USB_HOST_AUDIO_V1_DIRECTION_IN,
    .format = USB_AUDIO_FORMAT_PCM,
    .nChannels = 2,
    .bitResolution = 16,
    .subFrameSize = 2,
    .samplingRate = AUDIO_SAMPLING_RATE
}; 
//==============================================================================

//Application Class Data
APP_DATA appData __attribute__((coherent)) __attribute__((aligned(16))) =
{
     /* app state */
    .state = APP_STATE_INIT,

    .sampleFreq    = AUDIO_SAMPLING_RATE,
    .sampleFreqMic = AUDIO_SAMPLING_RATE,

    .codecClientWrite.context = (uintptr_t)0,
    .codecClientWrite.bufferHandler = 
        (DRV_CODEC_BUFFER_EVENT_HANDLER) APP_CODECBufferEventHandlerWrite,
    .codecClientWrite.bufferSize = APP_USB_FRAME_SIZE_BYTES,
    
    .codecClientRead.context = (uintptr_t)0,
    .codecClientRead.bufferHandler = 
        (DRV_CODEC_BUFFER_EVENT_HANDLER) APP_CODECBufferEventHandlerRead,
    .codecClientRead.bufferSize = APP_USB_FRAME_SIZE_BYTES,   //48*2*2 = 192 bytes
    //.currentAudioControl = APP_USB_CONTROL_NONE

};

/* PCM16 samples for 1Khz Sine Wave at 48Khz Sample Rate */
uint16_t audioSamples[96] =  
{
    0x0000, 0x0000,  //Sample 1
    0x10B4, 0x10B4,  //Sample 2
    0x2120, 0x2120,  //Sample 3
    0x30FB, 0x30FB,  //Sample 4
    0x3FFF, 0x3FFF,  //Sample 5
    0x4DEB, 0x4DEB,  //Sample 6
    0x5A81, 0x5A81,  //Sample 7
    0x658B, 0x658B,  //Sample 8
    0x6ED9, 0x6ED9,  //Sample 9
    0x7640, 0x7640,  //Sample 10
    0x7BA2, 0x7BA2,  //Sample 11
    0x7EE6, 0x7EE6,  //Sample 12
    0x7FFF, 0x7FFF,  //Sample 13
    0x7FE6, 0x7FE6,  //Sample 14
    0x7BA2, 0x7BA2,  //Sample 15
    0x7640, 0x7640,  //Sample 16
    0x6ED9, 0x6ED9,  //Sample 17
    0x658B, 0x658B,  //Sample 18
    0x5A81, 0x5A81,  //Sample 19
    0x4DEB, 0x4DEB,  //Sample 20
    0x3FFF, 0x3FFF,  //Sample 21
    0x30FB, 0x30FB,  //Sample 22
    0x2120, 0x2120,  //Sample 23
    0x10B4, 0x10B4,  //Sample 24
    0x0000, 0x0000,  //Sample 25
    0xEF4C, 0xEF4C,  //Sample 26
    0xDEE0, 0xDEE0,  //Sample 27
    0xCF05, 0xCF05,  //Sample 28
    0xC001, 0xC001,  //Sample 29
    0xB215, 0xB215,  //Sample 30
    0xA57F, 0xA57F,  //Sample 31
    0x9A75, 0x9A75,  //Sample 32
    0x9127, 0x9127,  //Sample 33
    0x89C0, 0x89C0,  //Sample 34
    0x845E, 0x845E,  //Sample 35
    0x811A, 0x811A,  //Sample 36
    0x8001, 0x8001,  //Sample 37
    0x811A, 0x811A,  //Sample 38
    0x845E, 0x845E,  //Sample 39
    0x89C0, 0x89C0,  //Sample 40
    0x9127, 0x9127,  //Sample 41
    0x9A75, 0x9A75,  //Sample 42
    0xA57F, 0xA57F,  //Sample 43
    0xB215, 0xB215,  //Sample 44
    0xC001, 0xC001,  //Sample 45
    0xCF05, 0xCF05,  //Sample 46
    0xDEE0, 0xDEE0,  //Sample 47
    0xFF4C, 0xFF4C,  //Sample 48
};
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

/* TODO:  Add any necessary local functions.
*/


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

void APP_Initialize()
{
    //appData.state =  APP_STATE_BUS_ENABLE;
    appData.state =  APP_STATE_INIT;  //APP_STATE_CODEC_OPEN
    
    //Initialize the USB Audio and Codec (NOTE:  ATTACH code runs this also)
    APP_AudioDataSetDefault();
    appData.muteStatus = 0;

    //Ping-Pong Codec Read Buffers
    appData.codecClientRead.txbufferObject0 = (uint8_t *) txBuffer[0];
    appData.codecClientRead.txbufferObject1 = (uint8_t *) txBuffer[1];
    appData.codecClientRead.readBufHandle0 = 
                        DRV_CODEC_BUFFER_HANDLE_INVALID;
    appData.codecClientRead.readBufHandle1 = 
                        DRV_CODEC_BUFFER_HANDLE_INVALID;
    appData.codecConfigured = false;

    //USB Event Write Complete
    appData.isUSBWriteComplete0 = false;
    appData.isUSBWriteComplete1 = false;

} //End APP_Initialize()

//******************************************************************************
// APP_AudioDataSetDefault
//   
// (Re)initializes appData elements required for USB Audio Data transfer
//******************************************************************************
void APP_AudioDataSetDefault()
{
    appData.isAudioDeviceAttached            = false;
    appData.isAudioOutStreamFound            = false; 

    appData.isAudioInStreamFound             = false; 

    appData.isSpeakerSampleRateSetComplete   = false;
    appData.isMicSampleRateSetComplete       = false;

    appData.isUSBWriteCompleted              = false;
    appData.isUSBWriteComplete0              = false;
    appData.isUSBWriteComplete1              = false;
    appData.usbWriteBufferIdx                = 0; //current buffer being written
    appData.transferHandleUSBWrite0 = 
              USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE_INVALID;
    appData.transferHandleUSBWrite1 = 
              USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE_INVALID;

    appData.isStreamEnabled                  = false;  //Not used
    appData.isStreamInterfaceSetZeroComplete = false;
    appData.isMicStreamInterfaceSetZeroComplete = false;
    appData.isStreamInterfaceSetOneComplete  = false;
    appData.isMicStreamInterfaceSetOneComplete  = false;
    appData.speakerRequestHandle = USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID;
    appData.micRequestHandle = USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID;
    appData.isMasterMuteSetComplete          = false; 
    appData.isMasterUnmuteSetComplete        = false; 
    appData.isMasterVolumeSetComplete        = false;
    appData.isLeftVolumeSetComplete          = false;
    appData.isRightVolumeSetComplete         = false;
    appData.isMuteSwitchPressed              = false; 
    appData.ignoreSwitchPress                = true;
    appData.sofEventHasOccurred              = false; 
    
    /* Set following to true until we start parsing descriptors */
    appData.isMasterMuteSupported            = true; 
    appData.isMasterVolumeSupported          = false;
    appData.isLeftChannelPresent             = true;
    appData.isRightChannelPresent            = true;
    appData.isLeftMuteSupported              = true;
    appData.isRightMuteSupported             = true;
    appData.isLeftVolumeSupported            = true;
    appData.isRightVolumeSupported           = true;

    //CODEC app data
    //USB Read Completed (not used)
    appData.isCodecReadComplete0 = false;
    appData.isCodecReadComplete1 = false;
    
    //Start Over
    APP_LED1_OFF();  //Attach  (Toggle is error state)
    APP_LED2_OFF();  //Underflow Playback
    APP_LED3_OFF();  //Overflow Playback
    APP_LED4_OFF();  //SpeakerStream Found
    APP_LED5_OFF();  //MicStream Found

} //End void APP_AudioDataSetDefault()


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks()
{
    USB_HOST_AUDIO_V1_RESULT audioResult;
    static uint32_t countLedBlink = 0; 
    uint32_t interruptStatus;
    bool status = false; 
  
    //APP_ProcessSwitchPress(); 
    
    // ATTACH CODE for all states >= APP_STATE_WAIT_FOR_DEVICE_ATTACH 
    // --If USB Device was detached, change application state accordingly
    if ((appData.state >= APP_STATE_WAIT_FOR_DEVICE_ATTACH )
       && (appData.isAudioDeviceAttached == false))
    {
        appData.state = APP_STATE_WAIT_FOR_DEVICE_ATTACH;
    }
    else if ((appData.isMuteSwitchPressed == true)
        && (appData.state > APP_STATE_WAIT_FOR_DEVICE_ATTACH )
            &&(appData.muteStatus == 0) )
    {
        appData.isMuteSwitchPressed = false; 
        /* User has pressed mute button. */
        appData.muteStatus = 1; 
        USB_HOST_AUDIO_V1_FeatureUnitChannelMuteSet
        (
            appData.audioDeviceObj,
            appData.speakerFeatureUnitObj,
            &appData.speakerRequestHandle,
            0,
            (bool *)&appData.muteStatus
        ); 
        
    }
    else if ((appData.isMuteSwitchPressed == true)
        && (appData.state > APP_STATE_WAIT_FOR_DEVICE_ATTACH )
            &&(appData.muteStatus == 1))
    {
        appData.isMuteSwitchPressed = false; 
        /* User has pressed unmute button */
        appData.muteStatus = 0; 
        USB_HOST_AUDIO_V1_FeatureUnitChannelMuteSet
        (
            appData.audioDeviceObj,
            appData.speakerFeatureUnitObj,
            &appData.speakerRequestHandle,
            0,
            (bool *)&appData.muteStatus
        );
    }

    /* Check the application's current state. */
    switch ( appData.state )
    {
        //---------------------------------------------------------------------
        // Configure and start CODEC Driver
        //
        // Transition: Initial State 
        //
        // Wait until: Codec SYS_STATUS_READY 
        //
        // Transition To: APP_STATE_CODEC_SET_BUFFER_HANDLER 
        //
        //---------------------------------------------------------------------
        case APP_STATE_INIT:
        case APP_STATE_CODEC_OPEN:
        {
            SYS_STATUS codecStatus;
            codecStatus = DRV_CODEC_Status(sysObjdrvCodec0);

            if (SYS_STATUS_READY == codecStatus)
            {

                //--USB HOST Headset playback
                // This means the driver can now be be opened.
                /* A client opens the driver object to get an Handle */
                appData.codecClientWrite.handle = 
                        DRV_CODEC_Open(DRV_CODEC_INDEX_0, 
                                       DRV_IO_INTENT_WRITE);

                       
                //MIC or LINE-IN data to USB -- Headphone or Headset
                appData.codecClientRead.handle = 
                        DRV_CODEC_Open(DRV_CODEC_INDEX_0, 
                                       DRV_IO_INTENT_READ );
                
                if (appData.codecClientWrite.handle != DRV_HANDLE_INVALID 
                    && appData.codecClientRead.handle != DRV_HANDLE_INVALID) 
                //if (appData.codecClientRead.handle != DRV_HANDLE_INVALID) 
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
        // Set a Codec Buffer handler for the read and write audio buffer 
        // DMA completion events. 
        //
        // --Transition to APP_STATE_BUS_ENABLE to start usb host interface
        //---------------------------------------------------------------------
        case APP_STATE_CODEC_SET_BUFFER_HANDLER:
        {
            //USB HOST Headset -- Playback
            DRV_CODEC_BufferEventHandlerSet(appData.codecClientWrite.handle,
                        appData.codecClientWrite.bufferHandler,
                        appData.codecClientWrite.context);

            DRV_CODEC_BufferEventHandlerSet(
                        appData.codecClientRead.handle,
                        appData.codecClientRead.bufferHandler,
                        appData.codecClientRead.context);

            //Enable SPI Data In
            PLIB_SPI_PinEnable(SPI_ID_1, SPI_PIN_DATA_IN);      

            //LINE-IN or EXT_MIC(<L,R>IN2 on the AK4642) on AK4642 DB - MIC1
            //LINE-IN on the AK4954 DB -- MIC3
            DRV_CODEC_MicSet(appData.codecClientRead.handle,
                    MIC_SELECT);

            appData.codecConfigured = true;

            appData.state = APP_STATE_BUS_ENABLE;
        }
        break;

        //---------------------------------------------------------------------
        // Open host layer (USB Bus Enable) 
        // --Set the Host AttachEventHandler
        //
        // --Transistion From: APP_STATE_CODEC_SET_BUFFER_HANDLER
        //
        // --Wait: None
        //
        // --Transition To:  APP_STATE_WAIT_FOR_BUFFER_ENABLE
        //
        //---------------------------------------------------------------------
        case APP_STATE_BUS_ENABLE:
        {
            
            /* Register a callback for Audio Device Attach. */
            audioResult = USB_HOST_AUDIO_V1_AttachEventHandlerSet(
                              &App_USBHostAudioV1AttachHandler,
                              0); 
            
            if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS )
            {
                /* Set Host Event Handler */
                USB_HOST_EventHandlerSet(APP_USBHostEventHandler, 0);
                USB_HOST_BusEnable(0);
                /* Advance application state */
                appData.state = APP_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE; 
            }
        }
        break; 
            
        //---------------------------------------------------------------------
        //  Wait for USB Host Interface to initialize
        //
        //  --Transition To: APP_STATE_WAIT_FOR_DEVICE_ATTACH, which also
        //    initiates attach interface code occurring each polling cycle
        //    (see above)
        //
        //---------------------------------------------------------------------
        case APP_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE:
        {
            if(USB_HOST_BusIsEnabled(0))
            {
                appData.state = APP_STATE_WAIT_FOR_DEVICE_ATTACH;
            }
        } //end case APP_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE
        break; 


        //---------------------------------------------------------------------
        //  Wait for Device to Attach
        //
        //  --Transistion From:  APP_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE 
        //                       AND
        //                       host interface attached via 
        //                       attach interface code 
        //                       occurring each polling cycle
        //                       (see above switch)
        //
        //  --Wait:  Wait for USB Device attach event 
        //
        //  --Transition To: APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET
        //  NOTE:  A suitable USB Audio V1.0 interface at the correct
        //         sample rate must be found for both the 
        //         IN (when headset device) and OUT direction(s).
        //---------------------------------------------------------------------
        case APP_STATE_WAIT_FOR_DEVICE_ATTACH:
        {
            /* Check if an Audio Device has been attached  */
            if(appData.isAudioDeviceAttached != true)
            {
                break; 
            }
            /* Register a Call back with Audio Object for Entity Control 
                 Transfer Receive Complete event */ 
            USB_HOST_AUDIO_V1_EntityRequestCallbackSet(
                    appData.audioDeviceObj,  
                    App_USBAudioControlRequestCallback, 
                    0); 
                
            /* Find an OUTPUT Audio Stream matching to our requirement */
            status = App_USBHostAudioSpeakerStreamFind(
                        appData.audioDeviceObj, 
                        audioSpeakerStreamFormat,
                        &appData.outStreamObj,
                        &appData.speakerStreamInterfaceSettingObj,
                        &appData.speakerStreamInterfaceSettingObjZeroBandwidth,
                        &appData.numberOfChannels);

            if (status == false)
            {
                //OUTPUT must be configured
                appData.state = APP_STATE_ERROR; 
                break;
            }

            if (appData.numberOfChannels < 2)
            {
                //Assuming stereo
                appData.state = APP_STATE_ERROR; 
                break;
            }

            appData.isAudioOutStreamFound = true;
                
            /* Now that we have found an OUT audio stream, Get the ID of the terminal 
             * to which this Audio Streaming Interface is connected */ 
            appData.speakerTerminalLink = 
                USB_HOST_AUDIO_V1_StreamingInterfaceTerminalLinkGet(
                       appData.audioDeviceObj,
                       appData.outStreamObj,
                       appData.speakerStreamInterfaceSettingObj);

            /* Get the Feature Unit Object */ 
            status = APP_SpeakerFeatureUnitObjectGet(
                        appData.speakerTerminalLink, 
                        &appData.speakerFeatureUnitObj); 
            if (status == false)
            {
                appData.state = APP_STATE_ERROR; 
            }
            else
            {
                APP_LED4_ON();
            }
               
#ifndef DEBUG_USB_TX_ONLY
            /* Find an INPUT Audio Stream matching to our requirement */
            //TODO: Should by APP_USBHostAudioStreamFind()
            status = App_USBHostAudioSpeakerStreamFind(
                                  appData.audioDeviceObj, 
                                  audioMicStreamFormat,
                                  &appData.inStreamObj,
                                  &appData.micStreamInterfaceSettingObj,
                                  &appData.micStreamInterfaceSettingObjZeroBandwidth,
                                  &appData.numberOfChannels);

            //status = false;  //DEBUG
            if (status == true ) //NOTE: IN Stream not essential for playback
            {
                //IN Stream available for headset Mic path. 
                appData.isAudioInStreamFound = true;

                // Now that we have found an IN audio stream, 
                // Get the ID of the terminal to which this Audio Streaming 
                // Interface is connected
                appData.micTerminalLink = 
                    USB_HOST_AUDIO_V1_StreamingInterfaceTerminalLinkGet(
                           appData.audioDeviceObj,
                           appData.inStreamObj,
                           appData.micStreamInterfaceSettingObj);

                /* Get the Feature Unit Object */ 
                status = APP_MicFeatureUnitObjectGet(
                            appData.micTerminalLink, 
                            &appData.micFeatureUnitObj); 
                if (status == false)
                {
                    appData.state = APP_STATE_ERROR; 
                }
                else
                {
                    APP_LED5_ON();
                }
            }
#endif //DEBUG_USB_TX_ONLY
                
            /* Check if Device supports Mute Control on different channels */ 
            appData.isMasterMuteSupported = 
                USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists(
                       appData.audioDeviceObj,
                       appData.speakerFeatureUnitObj,
                       0); 

            appData.isLeftMuteSupported = 
                USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists(
                       appData.audioDeviceObj,
                       appData.speakerFeatureUnitObj,
                       1); 

            appData.isRightMuteSupported = 
                USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists(
                       appData.audioDeviceObj,
                       appData.speakerFeatureUnitObj,
                       2);   

            /* Open Audio Stream */
            appData.outStreamHandle = USB_HOST_AUDIO_V1_StreamOpen(
                                          appData.outStreamObj);

            if (appData.outStreamHandle == USB_HOST_AUDIO_V1_STREAM_HANDLE_INVALID)
            {
                appData.state = APP_STATE_ERROR; 
                break;
            }
            
            /* Set Stream Event Handler  */
            audioResult = USB_HOST_AUDIO_V1_StreamEventHandlerSet(
                               appData.outStreamHandle, 
                               APP_USBHostAudioStreamEventHandler, 
                               (uintptr_t)appData.outStreamObj);

            if (audioResult != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
            {
                appData.state = APP_STATE_ERROR; 
                break;
            }

#ifndef DEBUG_USB_TX_ONLY
            if (appData.isAudioInStreamFound == true)
            {
#if 0
                /* Check if Device supports Mute Control on different channels */ 
                appData.isMasterMuteSupported = 
                    USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists(
                           appData.audioDeviceObj,
                           appData.micFeatureUnitObj,
                           0); 

                appData.isLeftMuteSupported = 
                    USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists(
                           appData.audioDeviceObj,
                           appData.micFeatureUnitObj,
                           1); 

                appData.isRightMuteSupported = 
                    USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists(
                           appData.audioDeviceObj,
                           appData.micFeatureUnitObj,
                           2);   
#endif

                /* Open IN Audio Stream */
                appData.inStreamHandle = USB_HOST_AUDIO_V1_StreamOpen(
                                              appData.inStreamObj);

                if (appData.inStreamHandle == USB_HOST_AUDIO_V1_STREAM_HANDLE_INVALID)
                {
                    appData.state = APP_STATE_ERROR; 
                    break;
                }
                
                /* Set Stream Event Handler  */
                audioResult = USB_HOST_AUDIO_V1_StreamEventHandlerSet(
                                   appData.inStreamHandle, 
                                   APP_USBHostAudioStreamEventHandler, 
                                   (uintptr_t)appData.inStreamObj);

                if (audioResult != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                {
                    appData.state = APP_STATE_ERROR; 
                    break;
                }

            } //End Audio IN Stream available
#endif //DEBUG_USB_TX_ONLY
            
            appData.state = APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET;

        } //End case APP_STATE_WAIT_FOR_DEVICE_ATTACH
        break; 
            
        //----------------------------------------------------------------------
        // OUT Zero Bandwidth (UnMute) Interface Set
        //
        // --Transition From: APP_STATE_WAIT_FOR_DEVICE_ATTACH
        //
        // --Wait for Audio Streaming interface NOT busy
        //
        // --Transistion To: 
        //       APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO
        //
        //----------------------------------------------------------------------
        case APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET:
        {
            appData.isStreamInterfaceSetComplete = false; 

            //Set AS #0 -- 0 Bandwidth
            audioResult = 
                USB_HOST_AUDIO_V1_StreamingInterfaceSet(
                     appData.outStreamHandle, 
                     &appData.speakerRequestHandle,
                     appData.speakerStreamInterfaceSettingObjZeroBandwidth);

            if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
            {
                appData.state = 
                        APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET; 
            }
            else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
            {
                /* Advance Application state */
                appData.state = APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO; 
            }
            else
            {
                appData.state = APP_STATE_ERROR; 
            }
        } //End case APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET
        break;

        //----------------------------------------------------------------------
        // OUT Wait for Audio Stream Zero Bandwidth Interface to Set
        // to send UnMute command to the device.
        //
        // --Transition From: APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET
        //
        // --Wait for Feature Unit Mute Set Channel NOT Busy
        //
        // --Transition To: APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE
        //
        //----------------------------------------------------------------------
        case APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO:
        {
            if (appData.isStreamInterfaceSetComplete == false)
            {
                break; 
            }
            appData.isStreamInterfaceSetZeroComplete = true;

            /* If  the attached speaker supports Mutes Control, then unmute it*/
            if (appData.isMasterMuteSupported == true)
            {
                /* Unmute the  Device */
                appData.isControlRequestCompleted = false; 

                audioResult = USB_HOST_AUDIO_V1_FeatureUnitChannelMuteSet(
                                  appData.audioDeviceObj,
                                  appData.speakerFeatureUnitObj,
                                  &appData.speakerRequestHandle,
                                  0, //Master
                                  (bool *)&appData.muteStatus);

                if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
                {
                    appData.isControlRequestCompleted = true;
                    appData.state = 
                        APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO; 
                }
                else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                {
                    appData.state = 
                        APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE;
                }
                else
                {
                    appData.state = APP_STATE_ERROR; 
                }
            }
            else
            {
                /* Device does not support mute control. */
                appData.isControlRequestCompleted = true;
                appData.state = APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE;
            }          
        } //case APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO
        break; 
            
        //----------------------------------------------------------------------
        // OUT Wait for UnMute to Complete before commanding volume control,
        // if available.
        //
        // --Transistion From: APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO
        //
        // --Wait for Feature Unit Channel Volume Set Interface NOT Busy
        //
        // --Transition To: 
        //  Master Volume Supported-->
        //     APP_STATE_AUDIO_WAIT_FOR_MASTER_VOL_CONTROL_COMPLETE
        //
        //      ELSE
        //
        //      Left && Right Volume Supported--> 
        //     APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE
        //
        //      OTHERWISE
        //
        //         
        //
        //----------------------------------------------------------------------
        case APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE:
        {
            if (appData.isControlRequestCompleted != true)
            {
                break; 
            }
            
            appData.currentVolume = 0x0500;
            if (appData.isMasterVolumeSupported == true)
            {
                appData.isControlRequestCompleted = false;

                //Send Master volume Control
                audioResult = USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeSet(
                                  appData.audioDeviceObj,
                                  appData.speakerFeatureUnitObj,
                                  &appData.speakerRequestHandle,
                                  0,
                                  &appData.currentVolume); 

                if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
                {
                    appData.isControlRequestCompleted = true;
                    appData.state = 
                        APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE; 
                }
                else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                {
                    appData.state =
                        APP_STATE_AUDIO_WAIT_FOR_MASTER_VOL_CONTROL_COMPLETE;
                }
                else
                {
                    appData.state = APP_STATE_ERROR; 
                }
            }
            else if ((appData.isLeftVolumeSupported) && (appData.isRightVolumeSupported))
            {
                //Send Left Volume Control
                appData.isControlRequestCompleted = false;

                audioResult = 
                    USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeSet(
                            appData.audioDeviceObj,
                            appData.speakerFeatureUnitObj,
                            &appData.speakerRequestHandle,
                            1, //LEFT
                            &appData.currentVolume); 

                if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
                {
                    appData.isControlRequestCompleted = true;
                    appData.state = 
                        APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE; 
                }
                else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                {
                    appData.state = 
                        APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE;
                }
                else
                {
                    appData.state = APP_STATE_ERROR; 
                }
            }
            else
            {
                //Volume Control not Supported
                appData.state = APP_STATE_ENABLE_AUDIO_STREAM;
            }
        } //End case APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE
        break;

            
        //----------------------------------------------------------------------
        //  OUT Master volume control complete
        //
        //  --Transition To: 
        //
        //----------------------------------------------------------------------
        case APP_STATE_AUDIO_WAIT_FOR_MASTER_VOL_CONTROL_COMPLETE:
        {
            if (appData.isControlRequestCompleted == true)
            {
                appData.state = APP_STATE_ENABLE_AUDIO_STREAM; 
                //appData.state = APP_STATE_SET_AUDIO_SAMPLING_RATE;
            }
        } //End case APP_STATE_AUDIO_WAIT_FOR_MASTER_VOL_CONTROL_COMPLETE
        break; 
            
        //----------------------------------------------------------------------
        // OUT APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE
        //  
        // --Transition To: 
        //     APP_STATE_AUDIO_WAIT_FOR_RIGHT_VOL_CONTROL_COMPLETE
        //
        //----------------------------------------------------------------------
        case APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE:
        {
            if (appData.isControlRequestCompleted != true)
            {
                break; 
            }
            appData.isControlRequestCompleted = false;

            audioResult = USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeSet(
                              appData.audioDeviceObj,
                              appData.speakerFeatureUnitObj,
                              &appData.speakerRequestHandle,
                               2, //RIGHT
                               &appData.currentVolume); 

            if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
            {
                appData.isControlRequestCompleted = true;
                appData.state = APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE; 
            }
            else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
            {
                appData.state = APP_STATE_AUDIO_WAIT_FOR_RIGHT_VOL_CONTROL_COMPLETE;
            }
            else
            {
                appData.state = APP_STATE_ERROR; 
            }           
        } //End case APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE
        break;
            
        //----------------------------------------------------------------------
        // OUT Wait for Right Volume Control Complete
        //
        // --Transistion To: APP_STATE_SET_AUDIO_SAMPLING_RATE
        //
        //----------------------------------------------------------------------
        case APP_STATE_AUDIO_WAIT_FOR_RIGHT_VOL_CONTROL_COMPLETE:
        {
            if (appData.isControlRequestCompleted == true)
            {
                /* Advance Application state */
                appData.state = APP_STATE_ENABLE_AUDIO_STREAM; 
            }
        } //End case APP_STATE_AUDIO_WAIT_FOR_RIGHT_VOL_CONTROL_COMPLETE
        break;
            
        //----------------------------------------------------------------------
        // Enable Audio Stream -- Set the Interface to AS #1 FULL BW
        //
        // --Transition To: APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ONE
        //----------------------------------------------------------------------
        case  APP_STATE_ENABLE_AUDIO_STREAM:
        {
            appData.isStreamInterfaceSetComplete = false;
            
            /* Set default interface setting of the streaming interface */
            audioResult = USB_HOST_AUDIO_V1_StreamingInterfaceSet(
                               appData.outStreamHandle, 
                               &appData.speakerRequestHandle,
                               appData.speakerStreamInterfaceSettingObj);

            if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
            {
                appData.isControlRequestCompleted = true;
                appData.state = APP_STATE_ENABLE_AUDIO_STREAM; 
            }
            else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
            {
                appData.state = APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ONE;
            }
            else
            {
                appData.state = APP_STATE_ERROR; 
            }     
        } //End APP_STATE_ENABLE_AUDIO_STREAM
        break; 
            
        //----------------------------------------------------------------------
        //  Wait for Audio Stream Interface to Set
        //
        //  --Transition To: APP_STATE_START_STREAM_DATA
        //
        //----------------------------------------------------------------------
        case  APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ONE:
        {
            if (appData.isStreamInterfaceSetComplete == true)
            {
        appData.isStreamInterfaceSetOneComplete = true;
                //appData.state = APP_STATE_SET_AUDIO_SAMPLING_RATE;
                appData.state = APP_STATE_SET_AUDIO_SAMPLING_RATE;
            }
        } //End case APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ONE
        break; 

        //----------------------------------------------------------------------
        // OUT Set the Sampling Rate
        //
        // --Transition From:  APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ONE:
        // --Wait For: Sampling Rate to Set.
        // --Transistion To: APP_STATE_WAIT_FOR_SET_SAMPLING_RATE
        //----------------------------------------------------------------------
        case APP_STATE_SET_AUDIO_SAMPLING_RATE:
        {
            //appData.state = APP_STATE_WAIT_FOR_SET_AUDIO_SAMPLING_RATE;
            appData.state = APP_STATE_SET_AUDIO_SAMPLING_RATE;
            appData.isControlRequestCompleted = false;
            appData.isSpeakerSampleRateSetComplete = false;

            //Set the Device Sampling Rate, event:
            //  USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE_DATA
            samplingRate = audioSpeakerStreamFormat.samplingRate;
            audioResult = USB_HOST_AUDIO_V1_StreamSamplingFrequencySet(
                              appData.outStreamHandle,
                              &appData.speakerRequestHandle,
                              &samplingRate);

            if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
            {
                appData.isControlRequestCompleted = true;
                appData.state = APP_STATE_SET_AUDIO_SAMPLING_RATE;
            }
            else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
            {
                appData.state = APP_STATE_WAIT_FOR_SET_AUDIO_SAMPLING_RATE;
            }
            else
            {
                appData.state = APP_STATE_ERROR; 
            }           
        } //End case APP_STATE_SET_AUDIO_SAMPLING_RATE
        break;

        //------------------------------------------------------------------------------
        // OUT Wait for the Sample Rate Set control request
        //------------------------------------------------------------------------------
        case APP_STATE_WAIT_FOR_SET_AUDIO_SAMPLING_RATE:
        {
            //if (appData.isControlRequestCompleted = true)
            if (appData.isSpeakerSampleRateSetComplete == true)
            {
#ifdef DEBUG_USB_TX_ONLY
	        appData.state = APP_STATE_START_STREAM_DATA;	
#else
                if (appData.isAudioInStreamFound == false)
                {
                    //Start the playback stream without MIC
                appData.state = APP_STATE_START_STREAM_DATA;
                }
                else
                {
                    //Enable the MIC stream AS 1
                    //and set the audio and sample rate
                    appData.state = APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET_MIC;
                }
#endif
            }

        } //End APP_STATE_WAIT_FOR_SET_AUDIO_SAMPLING_RATE
        break;

        //----------------------------------------------------------------------
        // IN Zero Bandwidth (UnMute) Interface Set Mic Stream
        //
        // --Transition From: Speaker audio stream enable
        //
        // --Wait for Audio Streaming interface NOT busy
        //
        // --Transistion To: 
        //       APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO_MIC
        //
        //----------------------------------------------------------------------
        case APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET_MIC:
        {
            /* Disable Audio Stream */
            appData.isStreamInterfaceSetComplete = false; 

            audioResult = 
                USB_HOST_AUDIO_V1_StreamingInterfaceSet(
                     appData.inStreamHandle, 
                     &appData.micRequestHandle,
                     appData.micStreamInterfaceSettingObjZeroBandwidth);

            if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
            {
                appData.state = 
                        APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET_MIC; 
            }
            else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
            {
                /* Advance Application state */
                appData.state = 
            APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO_MIC; 
            }
            else
            {
                appData.state = APP_STATE_ERROR; 
            }
        } //End case APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET_MIC_MIC
        break;

        //----------------------------------------------------------------------
        // IN Wait for Audio Stream Zero Bandwidth Interface to Set
        // to send UnMute command to the device.
        //
        // --Transition From: APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET
        //
        // --Wait for Feature Unit Mute Set Channel NOT Busy
        //
        // --Transition To: APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE_MIC
        //
        //----------------------------------------------------------------------
        case APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO_MIC:
        {
            if (appData.isStreamInterfaceSetComplete == false) 
            {
        //Wait for USB AS to complete
                break; 
            }
            appData.isMicStreamInterfaceSetZeroComplete = false; 

            /* If  the attached speaker supports Mutes Control, then unmute it*/
            if (appData.isMicMasterMuteSupported == true)
            {
                /* Unmute the  Device */
                appData.isControlRequestCompleted = false; 

                audioResult = USB_HOST_AUDIO_V1_FeatureUnitChannelMuteSet(
                                  appData.audioDeviceObj,
                                  appData.micFeatureUnitObj,
                                  &appData.micRequestHandle,
                                  0,
                                  (bool *)&appData.muteStatus);

                if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
                {
                    appData.isControlRequestCompleted = true;
                    appData.state = 
                        APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO_MIC; 
                }
                else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                {
                    appData.state = 
                        APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE_MIC;
                }
                else
                {
                    appData.state = APP_STATE_ERROR; 
                }
            }
            else
            {
                /* Device does not support mute control. */
                appData.isControlRequestCompleted = true;
                appData.state = APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE_MIC;
            }          
        } //case APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO
        break; 
            
        //----------------------------------------------------------------------
        // IN Wait for UnMute to Complete before commanding mic volume control,
        // if available.
        //
        // --Transition From: 
    //      APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO_MIC
        //
        // --Wait for Feature Unit Channel Volume Set Interface NOT Busy
        //
        // --Transition To: 
        //  Master Volume Supported-->
        //     APP_STATE_AUDIO_WAIT_FOR_MASTER_VOL_CONTROL_COMPLETE_MIC
        //
        //      ELSE
        //
        //      Left && Right Volume Supported--> 
        //     APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE_MIC
        //
        //      OTHERWISE
        //----------------------------------------------------------------------
        case APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE_MIC:
        {
            if (appData.isControlRequestCompleted != true)
            {
                break; 
            }

        appData.isMicStreamInterfaceSetZeroComplete = true;
            
            appData.currentVolume = 0x0500;
            if (appData.isMicMasterVolumeSupported == true)
            {
                appData.isControlRequestCompleted = false;

                //Send Master volume Control
                audioResult = USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeSet(
                                  appData.audioDeviceObj,
                                  appData.micFeatureUnitObj,
                                  &appData.micRequestHandle,
                                  0,
                                  &appData.currentVolume); 

                if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
                {
                    appData.isControlRequestCompleted = true;
                    appData.state = 
                        APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE_MIC; 
                }
                else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                {
                    appData.state = 
                        APP_STATE_AUDIO_WAIT_FOR_MASTER_VOL_CONTROL_COMPLETE_MIC;
                }
                else
                {
                    appData.state = APP_STATE_ERROR; 
                }
            }
            else if ((appData.isMicLeftVolumeSupported) && (appData.isMicRightVolumeSupported))
            {
                //Send Left Volume Control
                appData.isControlRequestCompleted = false;

                audioResult = 
                    USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeSet(
                            appData.audioDeviceObj,
                            appData.micFeatureUnitObj,
                            &appData.micRequestHandle,
                            1, //LEFT
                            &appData.currentVolume); 

                if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
                {
                    appData.isControlRequestCompleted = true;
                    appData.state = 
                        APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE_MIC; 
                }
                else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                {
                    appData.state = 
                        APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE_MIC;
                }
                else
                {
                    appData.state = APP_STATE_ERROR; 
                }
            }
            else
            {
                //Volume Control not Supported
                appData.state = APP_STATE_ENABLE_AUDIO_STREAM_MIC;
            }
        } //End case APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE
        break;

            
        //----------------------------------------------------------------------
        //  IN Master volume control complete
        //
        //  --Transition To: APP_STATE_SET_AUDIO_SAMPLING_RATE_MIC
        //----------------------------------------------------------------------
        case APP_STATE_AUDIO_WAIT_FOR_MASTER_VOL_CONTROL_COMPLETE_MIC:
        {
            if (appData.isControlRequestCompleted == true)
            {
                appData.state = APP_STATE_SET_AUDIO_SAMPLING_RATE_MIC;
            }
        } //End case APP_STATE_AUDIO_WAIT_FOR_MASTER_VOL_CONTROL_COMPLETE
        break; 

        //----------------------------------------------------------------------
        // IN APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE_MIC
        //
        // --Transition To: 
        //     APP_STATE_AUDIO_WAIT_FOR_RIGHT_VOL_CONTROL_COMPLETE_MIC
        //----------------------------------------------------------------------
        case APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE_MIC:
        {
            if (appData.isControlRequestCompleted != true)
            {
                break; 
            }
            appData.isControlRequestCompleted = false;

            audioResult = USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeSet(
                                   appData.audioDeviceObj,
                                   appData.micFeatureUnitObj,
                              &appData.micRequestHandle,
                                   2, //RIGHT
                                   &appData.currentVolume); 

            if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
            {
                appData.isControlRequestCompleted = true;
                appData.state = 
            APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE_MIC; 
            }
            else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
            {
                appData.state = 
            APP_STATE_AUDIO_WAIT_FOR_RIGHT_VOL_CONTROL_COMPLETE_MIC;
            }
            else
            {
                appData.state = APP_STATE_ERROR; 
            }           
        } //End case APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE
        break;
            
        //----------------------------------------------------------------------
        // IN Wait for Right Volume Control Complete
        //----------------------------------------------------------------------
        case APP_STATE_AUDIO_WAIT_FOR_RIGHT_VOL_CONTROL_COMPLETE_MIC:
        {
            if (appData.isControlRequestCompleted == true)
            {
                /* Advance Application state */
                appData.state = APP_STATE_ENABLE_AUDIO_STREAM_MIC; 
                //appData.state = APP_STATE_SET_AUDIO_SAMPLING_RATE_MIC;
            }
        } //End case APP_STATE_AUDIO_WAIT_FOR_RIGHT_VOL_CONTROL_COMPLETE
        break;

        //----------------------------------------------------------------------
        // IN Enable Audio Stream
        //
        // --Transition From: APP_STATE_WAIT_FOR_SET_AUDIO_SAMPLING_RATE
        //
        // --Transition To: APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ONE_MIC
        //
        //----------------------------------------------------------------------
        case  APP_STATE_ENABLE_AUDIO_STREAM_MIC:
        {
            
            appData.isStreamInterfaceSetComplete = false;

            /* Set default interface setting of the streaming interface */
            audioResult = USB_HOST_AUDIO_V1_StreamingInterfaceSet(
                               appData.inStreamHandle, 
                               &appData.micRequestHandle,
                               appData.micStreamInterfaceSettingObj);

            if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
            {
                appData.isControlRequestCompleted = true;
                appData.state = APP_STATE_ENABLE_AUDIO_STREAM_MIC; 
            }
            else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                {
                appData.state = 
            APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ONE_MIC;
                }
                else
                {
                    appData.state = APP_STATE_ERROR; 
                }
        } //End APP_STATE_ENABLE_AUDIO_STREAM_MIC
        break; 
            
        //----------------------------------------------------------------------
        //  IN Wait for Audio Stream Interface to Set
        //
        //  --Transition To: APP_STATE_START_STREAM_DATA_
        //
        //----------------------------------------------------------------------
        case  APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ONE_MIC:
        {
            if (appData.isStreamInterfaceSetComplete == true)
            {
        appData.isMicStreamInterfaceSetZeroComplete = true;
                appData.state = APP_STATE_SET_AUDIO_SAMPLING_RATE_MIC;
            }
        } //End case APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ONE_MIC
        break;

        //----------------------------------------------------------------------
        // IN Set the MIC Sampling Rate
        // --Transition From: APP_STATE_SET_AUDIO_SAMPLING_RATE
        // --Wait For:
        // --Transition To
        // NOTE:  The MIC sampling rate may be different from the SPEAKER rate
        //----------------------------------------------------------------------
        case APP_STATE_SET_AUDIO_SAMPLING_RATE_MIC:
        {
            appData.state = APP_STATE_SET_AUDIO_SAMPLING_RATE_MIC;
            appData.isControlRequestCompleted = false;
            appData.isMicSampleRateSetComplete = false;

            //Set the Device Sampling Rate, event:
            //  USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE_DATA
            samplingRate = audioSpeakerStreamFormat.samplingRate;
            audioResult = USB_HOST_AUDIO_V1_StreamSamplingFrequencySet(
                            appData.inStreamHandle,
                            &appData.micRequestHandle,
                            &samplingRate);
            if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
            {
                appData.isControlRequestCompleted = true;
                appData.state = APP_STATE_SET_AUDIO_SAMPLING_RATE_MIC;
            }
            else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
            {
                appData.state = APP_STATE_WAIT_FOR_SET_AUDIO_SAMPLING_RATE_MIC;
            }
            else
            {
                appData.state = APP_STATE_ERROR; 
            }           
        } //End case APP_STATE_SET_AUDIO_SAMPLING_RATE_MIC

        //----------------------------------------------------------------------
        // IN Wait for the Sample Rate Set control request
        //----------------------------------------------------------------------
        case APP_STATE_WAIT_FOR_SET_AUDIO_SAMPLING_RATE_MIC:
        {
            //if (appData.isControlRequestCompleted = true)
            if (appData.isMicSampleRateSetComplete == true)
            {
                appData.state = APP_STATE_START_STREAM_DATA;
            }

        } //End APP_STATE_WAIT_FOR_SET_AUDIO_SAMPLING_RATE_MIC
            
        //----------------------------------------------------------------------
        //  Start Stream Data
        //  --Queue Initial Codec Reads to the Ping-Pong Buffers
        //
        //  --Transition To: APP_STATE_WAIT_FOR_WRITE_COMPLETE
        //----------------------------------------------------------------------
        case APP_STATE_START_STREAM_DATA:
        {
            interruptStatus = __builtin_disable_interrupts(); 
            appData.isUSBWriteCompleted = false;
            __builtin_mtc0(12,0,interruptStatus);

            if (appData.codecConfigured)
            {
#if defined(DEBUG_TONE_USB_TX)
                //Just write the tone data
                USB_HOST_AUDIO_V1_StreamWrite(appData.outStreamHandle,
                                              &appData.transferHandleUSBWrite0, 
                                              (void*)&audioSamples, 
                                              sizeof(audioSamples));
                appData.usbWriteBufferIdx = 0;
                appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE; 
#else
                //==============================================================
                //Start the OUT Stream Codec Reads
                //Queue Codec Rx to both ping-pong buffers
                /*Submit Codec reads*/
                appData.isUSBWriteComplete0  = false;
                appData.isUSBWriteComplete1  = false;
                appData.isCodecReadComplete0 = false;
                appData.isCodecReadComplete1 = false;

                appData.transferHandleUSBWrite0 = 
                               USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE_INVALID;
                appData.transferHandleUSBWrite1 = 
                           USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE_INVALID;

                appData.usbWriteBufferIdx    = 0;  //Initial USBbuffer to complete

                //Initial Ping-Pong Buffer for Mic Read/USB Write
                appData.codecClientRead.readBufHandle0 = 
                        DRV_CODEC_BUFFER_HANDLE_INVALID;
                DRV_CODEC_BufferAddRead(appData.codecClientRead.handle, 
                                        &appData.codecClientRead.readBufHandle0,
                                        appData.codecClientRead.txbufferObject0, 
                                        appData.codecClientRead.bufferSize); 
            
                appData.codecClientRead.readBufHandle1 = 
                        DRV_CODEC_BUFFER_HANDLE_INVALID;
                DRV_CODEC_BufferAddRead(appData.codecClientRead.handle, 
                                        &appData.codecClientRead.readBufHandle1,
                                        appData.codecClientRead.txbufferObject1, 
                                        appData.codecClientRead.bufferSize); 


                if (appData.codecClientRead.readBufHandle0 != 
                       DRV_CODEC_BUFFER_HANDLE_INVALID && 
                    appData.codecClientRead.readBufHandle1 != 
                       DRV_CODEC_BUFFER_HANDLE_INVALID)
                {
#ifdef DEBUG_USB_TX_ONLY
                    appData.state = APP_STATE_PROCESS_DATA;
#else
                    if (appData.isAudioInStreamFound == false)
                    {
                        appData.state = APP_STATE_PROCESS_DATA;
                    }
                    else
                    {
                        //Start IN Stream
                        appData.state = APP_STATE_INITIAL_USB_READ_REQUEST;
                    }
#endif
                }
                else
                {
                    appData.state = APP_STATE_ERROR;
                }
#endif

            } //End appData.codecConfigured
            
        } //End case APP_STATE_START_STREAM_DATA
        break;


        //---------------------------------------------------------------------
        // IN Stream:  Initial USB Read Request
        //
        // Transition From:
        // Transition To: 
        //---------------------------------------------------------------------
        case APP_STATE_INITIAL_USB_READ_REQUEST:
        {
	    int qIdx;

            //Initilizate USB Read Queue
            _APP_Init_PlaybackBufferQueue();
            queueFull = false;
            queueEmpty = false;
            usbReadCompleteFlag = false;  //Actually USB Init Q Read Complete

            //To check alignment
            //int value1 = sizeof(APP_PLAYBACK_BUFFER)%16; 

            //Fill the queue with USB Read Data
            for(qIdx= 0;qIdx < APP_PLAYBACK_QUEUE_BUFFER_SIZE;qIdx++)
            {
                //USB Read to Head of Codec Playback Buffer
                APP_PLAYBACK_BUFFER* 
                    writeBuffer = &appPlaybackBuffer.playbackBuffer[qIdx];

                if (writeBuffer != NULL && 
                    !writeBuffer->codecInUse && 
                    !writeBuffer->usbInUse)
                {
                    writeBuffer->usbReadCompleted = false;
                    writeBuffer->usbInUse = true;

                    audioErr1 = USB_HOST_AUDIO_V1_StreamRead(
                                        appData.inStreamHandle,
                                        &writeBuffer->usbReadHandle, 
                                        writeBuffer->buffer, 
                                        appData.codecClientWrite.bufferSize); //64

                    if(audioErr1 != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                    {
                        writeBuffer->usbInUse = false;
                        break;
                    }
                    else
                    {
                        //Increment the Queue HEAD
                        appPlaybackBuffer.writeIdx = _APP_GetNextIdx(appPlaybackBuffer.writeIdx);
                    }
                }
                else
                {
                    Nop();
                }
            } //End USB Read Queue Loop

            //appData.state = APP_STATE_PROCESS_DATA;
            appData.state = APP_STATE_INITIAL_CODEC_WRITE_REQUEST;

        } //End case APP_STATE_INITIAL_USB_READ_REQUEST:
        break;


        //---------------------------------------------------------------------
        // Initial Codec Write Requests Queued 
        // --After the USB Intitial Read Queue Requests
        //
        // Transition From:
        //     APP_SUBMIT_INITIAL_USB_READ_REQUEST 
        //       -- initiates USB reads to all playback queue buffers.
        //
        // Wait: The queue to fill to initial read level
        //       (see USB_DEVICE_AUDIO_EVENT_READ_COMPLETE)
        //
        // Transition To: APP_STATE_PROCESS_DATA 
        //----------------------------------------------------------------------
        case APP_STATE_INITIAL_CODEC_WRITE_REQUEST:
        {    
            //Wait for the USB Reads to completely fill the buffer
            if (usbReadCompleteFlag)
            {

                int qIdx;
                for (qIdx = 0;
                     qIdx < APP_PLAYBACK_QUEUE_BUFFER_SIZE/QUEUE_USB_INIT_PART; qIdx++)
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

                //usbReadCompleteFlag = false; //TODO:  For next time
                appData.state = APP_STATE_PROCESS_DATA;

            } //End usbReadCompleteFlag 

        }
        break;

#if defined(DEBUG_TONE_USB_TX)
        //----------------------------------------------------------------------
        // Wait for USB Host Write to Complete
        //
        // --No Transistion (Always Writes Stream)
        //----------------------------------------------------------------------
        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:
        {

            if (appData.isUSBWriteCompleted)
            {
                interruptStatus = __builtin_disable_interrupts(); 
                    appData.isUSBWriteCompleted = false;
                __builtin_mtc0(12,0,interruptStatus);

                USB_HOST_AUDIO_V1_StreamWrite(appData.outStreamHandle,
                                              &appData.transferHandleUSBWrite0,  
                                              (void*)&audioSamples, 
                                              sizeof(audioSamples));
            } //End isUSBWriteCompleted

            appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE; 

        } //End case APP_STATE_WAIT_FOR_WRITE_COMPLETE
        break;

#else //DEBUG_TONE_USB_TX

        //---------------------------------------------------------------------
        // Process Codec Read buffers to USB Writes
    // Process USB Reads to Codec Writes
        //
        // --Transition From:
        //
        // --Transition To:
        //---------------------------------------------------------------------
        case APP_STATE_PROCESS_DATA:
        {
            //CODEC Read to USB Writes=========================================
            //CODEC Buffer Ready for Read from line-in/mic-in
            if (appData.isUSBWriteComplete0)
            {
                interruptStatus = __builtin_disable_interrupts(); 
                    appData.isUSBWriteComplete0 = false;
                __builtin_mtc0(12,0,interruptStatus);

                appData.codecClientRead.readBufHandle0 = 
                        DRV_CODEC_BUFFER_HANDLE_INVALID;
                DRV_CODEC_BufferAddRead(
                        appData.codecClientRead.handle, 
                        &appData.codecClientRead.readBufHandle0,
                        appData.codecClientRead.txbufferObject0, 
                        appData.codecClientRead.bufferSize); //int16 values

                if (appData.codecClientRead.readBufHandle0 != 
                DRV_CODEC_BUFFER_HANDLE_INVALID)
                {
                    //interruptStatus = __builtin_disable_interrupts(); 
                        appData.isCodecReadComplete0 = false;
                    //__builtin_mtc0(12,0,interruptStatus);
                }
                else
                {
                    Nop();
                }
            }
            else if(appData.isUSBWriteComplete1)
            {
                //Make Ready for USB Write
                interruptStatus = __builtin_disable_interrupts(); 
                    appData.isUSBWriteComplete1 = false;
                __builtin_mtc0(12,0,interruptStatus);

                appData.codecClientRead.readBufHandle1 = 
                        DRV_CODEC_BUFFER_HANDLE_INVALID;
                DRV_CODEC_BufferAddRead(
                        appData.codecClientRead.handle, 
                        &appData.codecClientRead.readBufHandle1,
                        appData.codecClientRead.txbufferObject1, 
                        appData.codecClientRead.bufferSize); //int16 values

                if (appData.codecClientRead.readBufHandle1 != 
                DRV_CODEC_BUFFER_HANDLE_INVALID)
                {
                    //interruptStatus = __builtin_disable_interrupts(); 
                    appData.isCodecReadComplete1 = false;
                    //__builtin_mtc0(12,0,interruptStatus);
                }
                else
                {
                    Nop();
                }
            } //End CODEC AddRead Ping-Pong Buffer
                        
            //USB Write from Ping-Pong Buffer
            if (appData.isCodecReadComplete0)
            {
                //Make Ready for USB Write Complete
                interruptStatus = __builtin_disable_interrupts(); 
                    appData.isUSBWriteComplete0 = false;
                    appData.isCodecReadComplete0 = false;
                __builtin_mtc0(12,0,interruptStatus);

                //TODO:  The bufferSize should be obtained from the descriptor

                audioErr1 = USB_HOST_AUDIO_V1_StreamWrite(
                                              appData.outStreamHandle,
                                              &appData.transferHandleUSBWrite0,  
                                              txBuffer[0], 
                                              appData.codecClientRead.bufferSize);

                if (audioErr1 != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                {
                    appData.state = APP_STATE_ERROR; 
                    break;
                }

            }
            else if(appData.isCodecReadComplete1)
            {
                //Make Ready for USB Write Complete
                interruptStatus = __builtin_disable_interrupts(); 
                    appData.isUSBWriteComplete1 = false;
                    appData.isCodecReadComplete1 = false;
                __builtin_mtc0(12,0,interruptStatus);

                audioErr1 = USB_HOST_AUDIO_V1_StreamWrite(appData.outStreamHandle,
                                              &appData.transferHandleUSBWrite1,  
                                              txBuffer[1], 
                                              appData.codecClientRead.bufferSize);

                if (audioErr1 != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                {
                    appData.state = APP_STATE_ERROR; 
                    break;
                }

            } //End USB Stream Write from Ping-Pong Buffer

            //Check if IN Stream is available for USB Reads to Codec Write 
            //buffers 
            if (appData.isAudioInStreamFound == false) break;

        //USB Reads to CODEC Writes ========================================
            //----------------------------
            // USB Read Request 
            APP_PLAYBACK_BUFFER* writeBuffer = 
                 &appPlaybackBuffer.playbackBuffer[appPlaybackBuffer.writeIdx];

            //Check for Overflow
            if (appPlaybackBuffer.usbReadCompleteBufferLevel == 
                APP_PLAYBACK_QUEUE_BUFFER_SIZE)
            {
                APP_LED3_ON();
                queueFull = true;
            }

            //------------------------------------------------ 
            //USB Stream Reads to Queue
            if (writeBuffer != NULL && 
                !writeBuffer->usbInUse &&   //USB streamRead in progress 
                !writeBuffer->codecInUse)   //Codec AddWrite in progress
            {
                writeBuffer->usbInUse = true;
                writeBuffer->usbReadCompleted = false;

                audioErr1 = USB_HOST_AUDIO_V1_StreamRead(
                                            appData.inStreamHandle,
                                            &writeBuffer->usbReadHandle, 
                                            writeBuffer->buffer, 
                                            appData.codecClientWrite.bufferSize); //64

                if(audioErr1 != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                {
                    writeBuffer->usbInUse = false;
                    break;
                }
                else
                {
                    //Increment the Queue HEAD
                    appPlaybackBuffer.writeIdx = 
                            _APP_GetNextIdx(appPlaybackBuffer.writeIdx);
                }

                //Check for QUEUE Full
                if (appPlaybackBuffer.readIdx == appPlaybackBuffer.writeIdx)
                {
                    //USB Reads have caught up with Codec Writes
                    //--> Wait for next read, which may cause a skip.
                    queueFull = true;
                    APP_LED3_ON();
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
            } //End USB Read Request 
                
            //------------------------------------------------ 
            //CODEC ADD WRITES from USB Read Queue
            //if (_APP_BufferInCodec())
            //-->Playback Codec is Ready (not in use)

            int8_t readIdx = appPlaybackBuffer.readIdx;
            APP_PLAYBACK_BUFFER* current = 
                    &appPlaybackBuffer.playbackBuffer[readIdx];

            if(current->usbReadCompleted &&  //USB Read Completed
               !current->codecInUse)         //Read for CODEC Write
            {
                //Underflow Check
                //if (appPlaybackBuffer.readIdx == appPlaybackBuffer.writeIdx)
                //{
                    //Codec Write have caught up with USB Read
                //    queueEmpty = true;
                //    APP_LED2_ON();
                //}

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

                    //Undeflow Check
                    if (appPlaybackBuffer.readIdx == appPlaybackBuffer.writeIdx)
                    {
                       //QUEUE is empty do not execute USB Read command;
                       queueEmpty = true;
                       APP_LED2_ON();
                    }
                    else
                    {
                       APP_LED2_OFF();
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

        } //case APP_PROCESS_DATA
        break;
#endif //!DEBUG_TONE_USB_TX

        /* The default state should never be executed. */
        case APP_STATE_ERROR:
            countLedBlink++; 
            if (countLedBlink >= APP_LED_BLINK_COUNT)
            {
                //BSP_LEDToggle(APP_USB_LED_2); 
                APP_LED1_TOGGLE();
                countLedBlink = 0; 
            }
            break; 
        default:
        {
            Nop(); 
            break;
        }

    } //End switch(state)


} //End APP_Tasks())


void APP_FindAudioControls()
{
    USB_AUDIO_FEATURE_UNIT_BMA_CONTROLS * audioControlMaster;
    USB_AUDIO_FEATURE_UNIT_BMA_CONTROLS * audioControlLeft;
    USB_AUDIO_FEATURE_UNIT_BMA_CONTROLS * audioControlRight;
    uint8_t bControlSize;

    /* Read size of each control field */
    bControlSize = appData.featureUnitDescriptor->bControlSize;

    /* Read Number of channels */
    appData.numberOfChannels = appData.inputTerminaDescriptor->bNrChannels;

    if ((bControlSize > 0 ) && (appData.numberOfChannels > 0))
    {
        /* Find if Left channel is supported */
        appData.isLeftChannelPresent = 
            appData.inputTerminaDescriptor->wChannelConfig & 0x0001;

        /* Find if Right channel is supported */
        appData.isRightChannelPresent = 
            appData.inputTerminaDescriptor->wChannelConfig & 0x0002;

        /* Get pointer to Master audio control structure */
        audioControlMaster = (USB_AUDIO_FEATURE_UNIT_BMA_CONTROLS *)
            ((uint8_t*)(&appData.featureUnitDescriptor->bControlSize) +1);

        /* Get pointer to Channel1 audio control structure */
        audioControlLeft = (USB_AUDIO_FEATURE_UNIT_BMA_CONTROLS *)
            ((uint8_t*)audioControlMaster + bControlSize);

        /* Get pointer to Channel 2 audio control structure */
        audioControlRight = (USB_AUDIO_FEATURE_UNIT_BMA_CONTROLS *)
            ((uint8_t*)audioControlLeft + bControlSize);

        /* Check if the device supports Mute Control */
        if (audioControlMaster->mute)
        {
            /* This means the attached Device supprts Mute Control */
            appData.isMasterMuteSupported = true;

        }

        if (audioControlLeft->mute)
        {
            appData.isLeftMuteSupported = true;
        }

        if (audioControlRight->mute)
        {
            appData.isRightMuteSupported = true; 
        }

        if (audioControlMaster->volume)
        {
            appData.isMasterVolumeSupported = true;
        }

        if (audioControlLeft->volume == 1)
        {
            appData.isLeftVolumeSupported = true;
        }

        if (audioControlRight->volume == 1)
        {
            appData.isRightVolumeSupported = true; 
        }
    }
}


bool APP_SpeakerFeatureUnitObjectGet(uint8_t inputTerminalID, 
                                  USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ* obj )
{
    USB_HOST_AUDIO_V1_RESULT audioResult;
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObj, entityObjFeatureUnit; 
    USB_AUDIO_V1_ENTITY_TYPE entityType;
    bool status = false; 
    
    audioResult = USB_HOST_AUDIO_V1_EntityObjectGet
                  (
                      appData.audioDeviceObj,
                      inputTerminalID,
                      &entityObj
                  );
    if (audioResult != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
    {
        appData.state = APP_STATE_ERROR; 
        return false; 
    }
    
    /* Get the First Audio Control Entity Object */ 
    audioResult = USB_HOST_AUDIO_V1_ControlEntityGetNext(appData.audioDeviceObj,
                                                         entityObj, 
                                                         &entityObjFeatureUnit);
    if (audioResult != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
    {
        appData.state = APP_STATE_ERROR;
        return false; 
    }
    
    /* Get the Entity Type */ 
    entityType = USB_HOST_AUDIO_V1_EntityTypeGet(appData.audioDeviceObj,
                                                 entityObjFeatureUnit); 
    
    if (entityType  == USB_AUDIO_MIXER_UNIT ||
        entityType == USB_AUDIO_FEATURE_UNIT ||
        entityType  == USB_AUDIO_INPUT_TERMINAL)  //Added for usb_headset
    {
        *obj = entityObj; 
        status = true; 
    }
    
    return status; 
}


//******************************************************************************
// APP_MicFeatureUnitObjectGet())
//******************************************************************************
bool APP_MicFeatureUnitObjectGet(uint8_t terminalID, 
                USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ* obj)
{
    USB_HOST_AUDIO_V1_RESULT audioResult;
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObj, entityObjFeatureUnit; 
    USB_AUDIO_V1_ENTITY_TYPE entityType;
    bool status = false; 
    
    audioResult = USB_HOST_AUDIO_V1_EntityObjectGet
                  (
                      appData.audioDeviceObj,
                      terminalID,
                      &entityObj
                  );
    if (audioResult != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
    {
        appData.state = APP_STATE_ERROR; 
        return false; 
    }
    
    /* Get the First Audio Control Entity Object */ 
    audioResult = USB_HOST_AUDIO_V1_ControlEntityGetNext(appData.audioDeviceObj,
                                                         entityObj, 
                                                         &entityObjFeatureUnit);
    if (audioResult != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
    {
        appData.state = APP_STATE_ERROR;
        return false; 
    }
    
    /* Get the Entity Type */ 
    entityType = USB_HOST_AUDIO_V1_EntityTypeGet(appData.audioDeviceObj,
                                                 entityObjFeatureUnit); 
    
    if (entityType == USB_AUDIO_HEADER ||
        entityType == USB_AUDIO_INPUT_TERMINAL) //Added for Chanceaux Headset
    {
        *obj = entityObj; 
        status = true; 
    }
    
    return status; 

} //End APP_MicFeatureUnitObjectGet()


void APP_GetSpeakerTopology()
{
    USB_HOST_AUDIO_V1_RESULT audioResult; 
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObj, entityObjNext; 
    USB_AUDIO_V1_ENTITY_TYPE entityType; 
    USB_AUDIO_V1_TERMINAL_TYPE terminalType; 
    bool isSpeakerOutputTerminalFound = false; 
    uint8_t sourceId; 
    
    /* Get the First Audio Control Entity Object */ 
    audioResult = USB_HOST_AUDIO_V1_ControlEntityGetFirst(appData.audioDeviceObj, 
                                                          &entityObj);
    
    if (audioResult != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
    {
        appData.state = APP_STATE_ERROR;
        return; 
    }
    
    /* Get the Entity Type */ 
    entityType = USB_HOST_AUDIO_V1_EntityTypeGet(appData.audioDeviceObj,entityObj); 
    
    /* We are trying get topology for Speaker. Start from Output Terminal. */  
    if (entityType !=  USB_AUDIO_OUTPUT_TERMINAL)
    {
        /* If entity Type is not Output Terminal Continue searching until 
           Output Terminal is found. */ 
        do 
        {
            /* Get the next Audio Control Entity */ 
            audioResult = USB_HOST_AUDIO_V1_ControlEntityGetNext(
                              appData.audioDeviceObj, 
                              entityObj, &entityObjNext );
            if (audioResult != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
            {
                appData.state = APP_STATE_ERROR;
                return; 
            }
            entityObj = entityObjNext; 
            
            /* Get Entity Type */ 
            entityType = USB_HOST_AUDIO_V1_EntityTypeGet(
                              appData.audioDeviceObj,entityObj);
            
            if (entityType == USB_AUDIO_OUTPUT_TERMINAL)
            {
                terminalType = USB_HOST_AUDIO_V1_TerminalTypeGet(
                                   appData.audioDeviceObj,
                                   entityObj ); 
                if (terminalType == USB_AUDIO_TERMINAL_TYPE_OUTPUT_SPEAKER)
                {
                    isSpeakerOutputTerminalFound = true; 
                    break; 
                }
            }
        }while (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS); 
    }
    
    if (isSpeakerOutputTerminalFound == true)
    {
        sourceId = USB_HOST_AUDIO_V1_TerminalSourceIDGet(
                       appData.audioDeviceObj,
                       entityObj);
        
        audioResult = USB_HOST_AUDIO_V1_EntityObjectGet(appData.audioDeviceObj, 
                                                        sourceId,  
                                                        &entityObj );
        
        if (audioResult != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
        {
            appData.state = APP_STATE_ERROR; 
            return; 
        }
        
        entityType = USB_HOST_AUDIO_V1_EntityTypeGet(appData.audioDeviceObj, 
                                                     entityObj ); 
        
        if (entityType == USB_AUDIO_FEATURE_UNIT)
        {
           Nop(); 
        }
        else
        {
           Nop(); 
        }   
    }

} //End  APP_GetSpeakerTopology()


//******************************************************************************
//  App_USBHostAudioSpeakerStreamFind()
//  given audioStream parameters
//
//  Results:
//    streamingInterfaceSettingObj for the AS Interface Setting found
//    that matches the audioSteam format.  
//    the returned values are uint_ptr values (i.e. indices to the arrays)
//******************************************************************************
bool App_USBHostAudioSpeakerStreamFind(
            USB_HOST_AUDIO_V1_OBJ audioDeviceObj,
            APP_USB_HOST_AUDIO_STREAM_FORMAT audioStream, 
            USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ * 
                audioStreamingInterfaceObj,                   //Index
            USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *
                streamingInterfaceSettingObj,                 //Index
            USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *
                streamingInterfaceSettingObjZeroBandwidth,
            uint8_t * numChannels) 
{
    USB_HOST_AUDIO_V1_RESULT result;
    int freqCount;
    uint8_t numOfStreamGrps;
    uint8_t aSinterfaceCnt;
    bool status = false; 
    USB_AUDIO_V1_FORMAT_TAG formatTag; 
    USB_HOST_AUDIO_V1_STREAM_DIRECTION streamDirection; 
    uint8_t nChannels; 
    uint8_t bitResolution; 
    uint8_t subFrameSize; 
    uint8_t nSamplingFreq; 
    uint8_t *pSamplingFrequency; 
    uint32_t samplingFrequency; 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ
        audioStreamingInterfaceObjNext; 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ 
        streamingInterfaceSettingObjNext; 
    
    
    /* Get Number of Stream Groups */
    numOfStreamGrps = 
            USB_HOST_AUDIO_V1_0_NumberOfStreamGroupsGet(audioDeviceObj);

    if (numOfStreamGrps == 0)
    {
        status = false; 
        return status;
    }


    //Check all AS interfaces
    for (aSinterfaceCnt = 0; aSinterfaceCnt < numOfStreamGrps; aSinterfaceCnt++)
    {
    
        if (aSinterfaceCnt == 0)
        {
            //First Interface
            result = USB_HOST_AUDIO_V1_StreamingInterfaceGetFirst(
                            audioDeviceObj, 
                            audioStreamingInterfaceObj); //First interface (idx)
        }
        else
        {
            result = USB_HOST_AUDIO_V1_StreamingInterfaceGetNext(
                            audioDeviceObj, 
                            *audioStreamingInterfaceObj,  
                            &audioStreamingInterfaceObjNext);

            *audioStreamingInterfaceObj = audioStreamingInterfaceObjNext; 
        }

        if (result !=  USB_HOST_AUDIO_V1_RESULT_SUCCESS)
        {
          status = false;   
          return status; 
        }
    
        /* First Streaming Interface Setting */            
        result = USB_HOST_AUDIO_V1_StreamingInterfaceSettingGetFirst(
                    audioDeviceObj, 
                    *audioStreamingInterfaceObj, //AS Interface (idx) 
                    streamingInterfaceSettingObjZeroBandwidth);
                                        //First Interface Setting (idx)

        if (result !=  USB_HOST_AUDIO_V1_RESULT_SUCCESS)
        {
          status = false;   
          return status; 
        }
    
        *streamingInterfaceSettingObj = *streamingInterfaceSettingObjZeroBandwidth;
        
        //Start searching for a suitable Audio Stream 
        do 
        {
            result = USB_HOST_AUDIO_V1_StreamingInterfaceSettingGetNext(
                            audioDeviceObj, 
                            *audioStreamingInterfaceObj,
                                *streamingInterfaceSettingObj,
                            &streamingInterfaceSettingObjNext);

                if (result != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                {
                    break;
                }

            *streamingInterfaceSettingObj = streamingInterfaceSettingObjNext;
            
            formatTag = USB_HOST_AUDIO_V1_StreamingInterfaceFormatTagGet(
                                audioDeviceObj,
                            *audioStreamingInterfaceObj,
                            *streamingInterfaceSettingObj); 
        
            streamDirection = USB_HOST_AUDIO_V1_StreamingInterfaceDirectionGet (
                                      audioDeviceObj,
                                  *audioStreamingInterfaceObj,
                                  *streamingInterfaceSettingObj); 

            nChannels = USB_HOST_AUDIO_V1_StreamingInterfaceChannelNumbersGet(
                                audioDeviceObj,
                            *audioStreamingInterfaceObj,
                            *streamingInterfaceSettingObj); 

            bitResolution = USB_HOST_AUDIO_V1_StreamingInterfaceBitResolutionGet(
                                    audioDeviceObj,
                                *audioStreamingInterfaceObj,
                                *streamingInterfaceSettingObj); 

            subFrameSize = USB_HOST_AUDIO_V1_StreamingInterfaceSubFrameSizeGet(
                                    audioDeviceObj,
                                *audioStreamingInterfaceObj,
                                *streamingInterfaceSettingObj); 

            nSamplingFreq = 
                    USB_HOST_AUDIO_V1_StreamingInterfaceSamplingFrequencyTypeGet(
                                    audioDeviceObj,
                                *audioStreamingInterfaceObj,
                                *streamingInterfaceSettingObj); 

            /* Compare Audio Stream info */
            if ((formatTag == audioStream.format)
                && (streamDirection == audioStream.streamDirection)
                    //&& (nChannels == audioStream.nChannels)
                && (bitResolution == audioStream.bitResolution)
                && (subFrameSize == audioStream.subFrameSize))
            {    
                    
                if (nSamplingFreq != 0)
                {
                    pSamplingFrequency = 
                    USB_HOST_AUDIO_V1_StreamingInterfaceSamplingFrequenciesGet(
                                             audioDeviceObj,
                                     *audioStreamingInterfaceObj,
                                     *streamingInterfaceSettingObj);         

                    for (freqCount = 0; freqCount < nSamplingFreq; freqCount++)
                    {
                        uint32_t sa,sb,sc;
                        sa = (uint32_t)pSamplingFrequency[freqCount*3];
                        sb = (uint32_t)pSamplingFrequency[freqCount*3 + 1] << 8;
                        sc = (uint32_t)pSamplingFrequency[freqCount*3 + 2] << 16;

                        samplingFrequency = sa | sb | sc; 

                        if (samplingFrequency == audioStream.samplingRate )
                        {
                            /* We have found an interface setting that is matching our 
                               requirement */ 
                            *numChannels = nChannels;
                            status = true; 
                            return status; 
                        }
                    }         
                }
                else
                {//WORKAROUND:  For continuous sampling rate.
                    *numChannels = nChannels;
                    status = true;
                    return status;
                }

            } //End AS Interface Check
            
        } while (result == USB_HOST_AUDIO_V1_RESULT_SUCCESS); 
        //AS Interface Setting Loop
        
    }; //End AS Loop
    return status;  
} //End App_USBHostAudioSpeakerStreamFind()


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

#if 0
void APP_ProcessSwitchPress(void)
{
    /* This function checks if the switch is pressed and then
     * debounces the switch press*/
    if(BSP_SWITCH_STATE_PRESSED == (BSP_SwitchStateGet(APP_USB_SWITCH_1)))
    {
        if(appData.ignoreSwitchPress)
        {
            /* This means the key press is in progress */
            if(appData.sofEventHasOccurred)
            {
                /* A timer event has occurred. Update the debounce timer */
                appData.switchDebounceTimer++;
                appData.sofEventHasOccurred = false;

                if(appData.switchDebounceTimer == APP_USB_SWITCH_DEBOUNCE_COUNT_FS)
                {
                    /* Indicate that we have valid switch press. The switch is
                     * pressed flag will be cleared by the application tasks
                     * routine. We should be ready for the next key press.*/
                    appData.isMuteSwitchPressed = true;
                    appData.switchDebounceTimer = 0;
                    appData.ignoreSwitchPress = false;
                }
            }
        }
        else
        {
            /* We have a fresh key press */
            appData.ignoreSwitchPress = true;
            appData.switchDebounceTimer = 0;
        }
    }
    else
    {
        /* No key press. Reset all the indicators. */
        appData.ignoreSwitchPress = false;
        appData.switchDebounceTimer = 0;
        appData.sofEventHasOccurred = false;
    }

}
#endif //0


/*************************************************************
 * AUDIO Host Attach Event Handler function.
 ************************************************************/
void App_USBHostAudioV1AttachHandler(USB_HOST_AUDIO_V1_OBJ audioObj, 
    USB_HOST_AUDIO_V1_EVENT event,
    uintptr_t context)
{
    switch (event)
    {
        case USB_HOST_AUDIO_V1_EVENT_ATTACH: 
            if (appData.isAudioDeviceAttached == false)
            {
                appData.isAudioDeviceAttached = true; 
                appData.audioDeviceObj = audioObj; 
                //BSP_LEDOn(APP_USB_LED_1); 
                APP_LED1_ON();
            }
            else
            {
                /* This application supports only one Audio Device . Handle Error Here.*/ 
            }
        break; 
        case USB_HOST_AUDIO_V1_EVENT_DETACH: 
            if (appData.isAudioDeviceAttached == true)
            {
                /* This means the device was detached. There is no event data
                 * associated with this event.*/

                APP_AudioDataSetDefault();

                //TODO:
                //DRV_CODEC_EmptyBufferQueue(appData.codecClientRead.handle);

                APP_LED1_OFF();

                appData.state = APP_STATE_WAIT_FOR_DEVICE_ATTACH;
                break;
            }
        break; 
    }
    
} //End App_USBHostAudioV1AttachHandler()


//******************************************************************************
// AUDIO Host Application Event Handler function.
//*****************************************************************************
USB_HOST_AUDIO_V1_STREAM_EVENT_RESPONSE 
    APP_USBHostAudioStreamEventHandler(
        USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
        USB_HOST_AUDIO_V1_STREAM_EVENT event,
        void * eventData,
        uintptr_t context)
{

    switch(event)
    {


        case USB_HOST_AUDIO_V1_STREAM_EVENT_INTERFACE_SET_COMPLETE:
        {
            USB_HOST_AUDIO_V1_STREAM_EVENT_INTERFACE_SET_COMPLETE_DATA *
                    interfaceSetCompleteEventData;

            interfaceSetCompleteEventData =
                (USB_HOST_AUDIO_V1_STREAM_EVENT_INTERFACE_SET_COMPLETE_DATA *)
                    eventData;

            if (interfaceSetCompleteEventData->requestStatus ==     
                USB_HOST_AUDIO_V1_RESULT_SUCCESS)
            {
            appData.isStreamInterfaceSetComplete = true; 
            }
        }
            break;
            
        case USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE:
        {
            USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE_DATA * 
                    writeCompleteEventData;

            appData.sofEventHasOccurred = true;
            /* This means the Write request completed. We can
             * find out if the request was successful. */
            writeCompleteEventData =
                (USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE_DATA*)eventData;

#ifdef DEBUG_TONE_USB_TX
            if (appData.transferHandleUSBWrite0 == 
                writeCompleteEventData->transferHandle)
            {
                    appData.isUSBWriteCompleted = true;
            }
#else //DEBUG_TONE_USB_TX
            if (appData.usbWriteBufferIdx == 0)  //Ping
            {
                if (appData.transferHandleUSBWrite0 == 
                        writeCompleteEventData->transferHandle)
                {
                    appData.isUSBWriteCompleted = true;
                    appData.isUSBWriteComplete0 = true;
                    appData.usbWriteBufferIdx = 1; //Pong
                }
                else
                {
                    Nop();
                }
            }
            else                                  //Pong
            {
                if (appData.transferHandleUSBWrite1 == 
                        writeCompleteEventData->transferHandle)
                {
                    appData.isUSBWriteCompleted = true;
                    appData.isUSBWriteComplete1 = true;
                    appData.usbWriteBufferIdx = 0; //Ping
                }
                else
                {
                    Nop();
                }
            }
#endif //End DEBUG_TONE_USB_TX

        } //End case USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE:
        break;

//------------------------------------------------------------------------------
        case USB_HOST_AUDIO_V1_STREAM_EVENT_READ_COMPLETE:
        {
            USB_HOST_AUDIO_V1_STREAM_EVENT_READ_COMPLETE_DATA * 
                    readCompleteEventData;

            readCompleteEventData =
                (USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE_DATA*)eventData;

            //Make USB Read Buffer Queue Ready for Data (Clear data flags)
            _APP_SetUSBReadBufferReady(readCompleteEventData->transferHandle);
            appPlaybackBuffer.usbReadCompleteBufferLevel++;
            queueEmpty = false;

            //Check if the this is the initial loading of the playback
            //queue with USB Rx Data 
            if(appData.state == APP_STATE_INITIAL_CODEC_WRITE_REQUEST)
            {
                if(_APP_USBReadAllBufferReady())
                {
                    usbReadCompleteFlag = true;
                    APP_LED2_OFF();  //QUEUE Not Empty
                }
            }

        } //End case USB_HOST_AUDIO_V1_STREAM_EVENT_READ_COMPLETE:
        break;
//------------------------------------------------------------------------------

        case  USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_FREQUENCY_SET_COMPLETE:
        {
            USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE_DATA *
                    samplingRateSetCompleteEventData;

            samplingRateSetCompleteEventData =
              (USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_SET_COMPLETE_DATA *)
                    eventData;

            if (samplingRateSetCompleteEventData->requestStatus ==     
                USB_HOST_AUDIO_V1_RESULT_SUCCESS)
            {
                if (samplingRateSetCompleteEventData->requestHandle ==     
                     appData.speakerRequestHandle)
                {
            appData.isSpeakerSampleRateSetComplete    = true;
                }
                else if(samplingRateSetCompleteEventData->requestHandle ==     
                     appData.micRequestHandle)
                {
            appData.isMicSampleRateSetComplete        = true;
                }
            }


            //Buffer Size - IN/OUT Data to the USBDevice
            appData.USBReadBufSize  = APP_USB_FRAME_SIZE_BYTES;

            //Codec Read/Write Buffer Size (1ms of Stereo Audio)
            //TODO:  Get this from the AudioFormat descriptor.
            //       A Fixed 48000 sample rate for now.
            if (appData.sampleFreq == SAMPLING_RATE_48000)
            {
                appData.codecClientRead.bufferSize = 
                                        APP_USB_FRAME_SIZE_BYTES;
                appData.codecClientWrite.bufferSize = 
                                        APP_USB_FRAME_SIZE_BYTES;
            }
            else if (appData.sampleFreq == SAMPLING_RATE_32000)
            {
                appData.codecClientRead.bufferSize = 128;
                appData.codecClientWrite.bufferSize = 128;
            }
            else if (appData.sampleFreq == SAMPLING_RATE_24000)
            {
                appData.codecClientRead.bufferSize = 96;
                appData.codecClientWrite.bufferSize = 96;
            }
            else if (appData.sampleFreq == SAMPLING_RATE_16000)
            {
                appData.codecClientRead.bufferSize = 64;
                appData.codecClientWrite.bufferSize = 64;
            }

        }

        case  USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_FREQUENCY_GET_COMPLETE:
        {
            USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_GET_COMPLETE_DATA *
                    samplingRateGetCompleteEventData;

            samplingRateGetCompleteEventData =
              (USB_HOST_AUDIO_V1_STREAM_EVENT_SAMPLING_RATE_GET_COMPLETE_DATA *)
                    eventData;

            if (samplingRateGetCompleteEventData->requestStatus ==     
                USB_HOST_AUDIO_V1_RESULT_SUCCESS)
            {
                if (samplingRateGetCompleteEventData->requestHandle ==     
                     appData.speakerRequestHandle)
                {
                    //appData.isSpeakerSampleRateGetComplete    = true;
                    Nop();
                }
                else if(samplingRateGetCompleteEventData->requestHandle ==     
                     appData.micRequestHandle)
                {
                    //appData.isMicSampleRateGetComplete        = true;
                    Nop();
                }
            }

        }
        break;

        default:
            break;
            
    } //End switch(USB Audio Stream event)

    return USB_HOST_AUDIO_V1_STREAM_EVENT_RESPONSE_NONE; 

} //End APP_USBHostAudioStreamEventHandler()



/*************************************************************
 * USB Host Layer Application Event Handler function.
 ************************************************************/
USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler(USB_HOST_EVENT event, 
                                                void * eventData, 
                                                uintptr_t context)
{
    switch(event)
    {
        case USB_HOST_EVENT_DEVICE_REJECTED_INSUFFICIENT_POWER:
        case USB_HOST_EVENT_DEVICE_UNSUPPORTED:
        case USB_HOST_EVENT_HUB_TIER_LEVEL_EXCEEDED:
        case USB_HOST_EVENT_PORT_OVERCURRENT_DETECTED:
        default: 
            break; 
    }
    return USB_HOST_EVENT_RESPONSE_NONE;
} //End APP_USBHostEventHandler()


void App_USBAudioControlRequestCallback(
        USB_HOST_AUDIO_V1_OBJ audioObj, 
        USB_HOST_AUDIO_V1_REQUEST_HANDLE requestHandle,
        USB_HOST_AUDIO_V1_RESULT result,
        size_t size,
        uintptr_t context)
{
    if (requestHandle == appData.speakerRequestHandle)
    {
        appData.isControlRequestCompleted = true; 
    }
    else
    {
        appData.isControlRequestCompleted = true; 
    }

}


//******************************************************************************
// APP_CODECBufferEventHandler
//
// This function is called back by the CODEC driver when
// a CODEC data buffer RX completes.
//******************************************************************************
void APP_CODECBufferEventHandlerRead(DRV_CODEC_BUFFER_EVENT event,
                                     DRV_CODEC_BUFFER_HANDLE handle, 
                                     uintptr_t context )
{
     switch(event)
    {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        {
            if (appData.codecClientRead.readBufHandle0 == handle)
            {
                //Ready to TX USB buffer 0
                //--buffer 0 not available until USB TX completes
                appData.isCodecReadComplete0 = true;
            }
            else if(appData.codecClientRead.readBufHandle1 == handle){
                //Ready to TX USB buffer 1
                //--buffer 1 not available until USB TX completes
                appData.isCodecReadComplete1 = true;
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
    
} //End APP_CODECBufferEventHandlerRead()

//------------------------------------------------------------------------------
//******************************************************************************
// APP_CODECBufferEventHandler()
//
// Application CODEC buffer Event handler.
// This function is called back by the CODEC driver when
// a CODEC data buffer TX completes.
//******************************************************************************
void APP_CODECBufferEventHandlerWrite(DRV_CODEC_BUFFER_EVENT event,
                                      DRV_CODEC_BUFFER_HANDLE handle, 
                                      uintptr_t context )
{
    switch(event)
    {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        {
            if (appPlaybackBuffer.readIdx != appPlaybackBuffer.writeIdx)
            {
                //QUEUE is not full or empty
                APP_LED2_OFF();
                APP_LED3_OFF();
            }

            //This buffer is ready for USB Write
            _APP_ClearCodecReturnBuffer(handle);
            appPlaybackBuffer.usbReadCompleteBufferLevel--;
            if (appPlaybackBuffer.usbReadCompleteBufferLevel == 0)
            {
                //USB Read needs to complete before next Codec Write.
                queueEmpty = true;
                APP_LED2_ON();
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
//------------------------------------------------------------------------------



//******************************************************************************
// _APP_SetUSBReadBufferReady()
//Set the USB Read Buffer Data Ready 
//******************************************************************************
static void _APP_SetUSBReadBufferReady(
                  USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE handle)
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
            appPlaybackBuffer.playbackBuffer[i].writeHandle = 
                                               DRV_CODEC_BUFFER_HANDLE_INVALID;
            appPlaybackBuffer.playbackBuffer[i].codecInUse = false;
            appPlaybackBuffer.playbackBuffer[i].usbInUse = false;
            appPlaybackBuffer.playbackBuffer[i].usbReadCompleted = false;
            appPlaybackBuffer.playbackBuffer[i].usbReadHandle = 
                               USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE_INVALID;
        }
    }
}

/*******************************************************************************
 End of File
 */