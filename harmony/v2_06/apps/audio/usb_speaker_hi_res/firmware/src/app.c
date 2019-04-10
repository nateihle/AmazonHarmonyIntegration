/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app.c for usb_speaker_96Khz_24bit

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


//*****************************************************************************
//*****************************************************************************
// Section: Included Files
//*****************************************************************************
//*****************************************************************************

#include "app.h"

#ifdef USE_DISPLAY
#include "display.h"
#endif

//DEBUG  
#undef  DEBUG_TONE_CODEC_TX  //TX TONE or SAWTOOTH
#undef  DEBUG_TONE_SAWTOOTH  //SELECT Sawtooth, otherwise TONE
//#define DEBUG_REPORT_INTERVAL 96 
#define DEBUG_REPORT_INTERVAL 1
uint32_t Debug_var;

//QUEUE Initialization 
//N, where N gives the fraction of the QUEUE to be filled, i.e. 1/N
//and USB reads generated and completed and Codec AddWrites before
//transitioning to APP_PROCESS_DATA timing. 
#define QUEUE_USB_INIT_PART   4  

bool queueFull;
bool queueEmpty;

//24 to 32 bit Unpacking 
uint32_t sa;
uint32_t sb;
uint32_t sc;

//NOTE: Cache coherency and 16 byte alignment required for MZ processor,
//      -->as the cache page size on PIC32MZ is 16 bytes.
//      You don?t want to run into an issue where linker allocates the data 
//      structure in the same page as another data structure and then a line 
//      flush causes coherency issues.

//48Khz Test Data
//Test buffers (sine tone)
//-->48X4 = 192 bytes
//NOTE:  DRV_I2S_DATA24 is equivalent to DRV_I2S_DATA32
DRV_I2S_DATA32 __attribute__((coherent)) __attribute__((aligned(16)))
        testBuffer2[96] =
{
#include "tone_1000Hz_32bit_96Khz_p125.txt" 
};

typedef struct  _BITS24
{
    int8_t bytes[3]; 
} BITS24;
int32_t _Convert24to32bit(BITS24 val);

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
static int _APP_SetUSBReadBufferReady(USB_DEVICE_AUDIO_TRANSFER_HANDLE handle);
static void _APP_Init_PlaybackBufferQueue();
static int _APP_ClearCodecReturnBuffer(DRV_CODEC_BUFFER_HANDLE handle);
static uint8_t _APP_GetNextIdx(uint8_t index);
//static bool _APP_CodecBufferInUse();
static bool    _APP_USBReadAllBufferReady();
static bool hpInterfaceChanged = false;
static USB_DEVICE_AUDIO_RESULT audioErr1;


//Application Class Data
APP_DATA appData =
{
    /* Device Layer Handle  */
    .usbDevHandle = -1,

    /* USB Audio Instance index for this app object 0*/
    .audioInstance = 0,

     /* app state */
    .state = APP_STATE_CODEC_OPEN,

    /* device configured status */
    .isConfigured = false,

    .codecConfigured = false,

    // Initialize active USB interface setting 
    // --Holds active value of USB Audio Streaming Interface Alternate setting.
    // Every USB Audio Streaming should have 1 or more Alternate settings.
    // Alternate setting Zero (or Default Alternate setting) never has any
    // data payload. An audio streaming can have multiple number of alternate
    // settings each having it own sampling rate, bandwidth etc. 
    //--Set by the APP_USBDeviceAudioEventHandler() 
    //.activeInterfaceAlternateSetting    = APP_USB_SPEAKER_PLAYBACK_NONE;
    //.activeInterfaceAlternateSettingMic = APP_USB_MIC_RECORD_MONO_48KHZ;
    .activeInterfaceAlternateSetting = APP_USB_SPEAKER_PLAYBACK_NONE,
    
    /* DAC is not muted initially */
    .dacMute = false,
    
    .sampleFreq = 0x17700,    //96000 Hz
    
    //96000Hz @ 1ms = 96*8 = 768
    .USBReadBufSize = APP_MAX_NO_BYTES_IN_USB_BUFFER, //96000Hz 1ms (96*6) = 576  
    
    //CODEC Driver Write only Client
    .codecClientWrite.context = (uintptr_t)0,
    .codecClientWrite.bufferHandler = (DRV_CODEC_BUFFER_EVENT_HANDLER) 
                                         APP_CODECBufferEventHandler,

    .codecClientWrite.bufferSize = APP_MAX_NO_BYTES_IN_CODEC_BUFFER, // 96*8 = 768 
    
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

    uint8_t entityID;
    uint8_t controlSelector;

    if ( iAudio == 0 )
    {
        switch (event)
        {
            case USB_DEVICE_AUDIO_EVENT_INTERFACE_SETTING_CHANGED:
            {
                /* We have received a request from USB host to change the Interface-
                   Alternate setting.*/
                interfaceInfo = 
                   (USB_DEVICE_AUDIO_EVENT_DATA_INTERFACE_SETTING_CHANGED *)
                   pData;

                //if(interfaceInfo->interfaceNumber == APP_PLAYBACK_INTERFACE)
                //{
                //    if(appData.activeInterfaceAlternateSetting != interfaceInfo->interfaceAlternateSetting){
                        hpInterfaceChanged = true;
                        appData.activeInterfaceAlternateSetting = interfaceInfo->interfaceAlternateSetting;
                        appData.state = APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD;
                //    }
                //    
                //}
                /* We have received a request from USB host to change the Interface-
                   Alternate setting.*/
            }
            break;

            case USB_DEVICE_AUDIO_EVENT_READ_COMPLETE:
            {
                //We have received an audio frame from the Host.
                //Now send this audio frame to Audio Codec for Playback.
                readEventData = 
                   (USB_DEVICE_AUDIO_EVENT_DATA_READ_COMPLETE *)pData;

                //Make USB Read Buffer Queue Ready for Data (Clear data flags)
                int __attribute__((unused)) usbBufferIdx = 
                    _APP_SetUSBReadBufferReady(readEventData->handle);

                appPlaybackBuffer.usbReadCompleteBufferLevel++;
                appPlaybackBuffer.usbReadCompleteCnt++;
                appPlaybackBuffer.usbReadQueueCnt--;  //Not completed level

#if (DEBUG_LEVEL == 1)
                if (!(appPlaybackBuffer.usbReadCompleteCnt%DEBUG_REPORT_INTERVAL))
                {
                    SYS_PRINT("--RD COMPLETE RC %d: QL %d RQ %d WQ %d WC %d",
                             appPlaybackBuffer.usbReadCompleteCnt,
                             appPlaybackBuffer.usbReadCompleteBufferLevel,
                             appPlaybackBuffer.usbReadQueueCnt,
                             appPlaybackBuffer.codecWriteQueueCnt,
                             appPlaybackBuffer.codecWriteCompleteCnt);
                }
#endif //DEBUG_LEVEL>1
#if DEBUG_LEVEL == 3 
                    SYS_PRINT("RCOMP: QL %d  RQ %d  WQ %d WC %d", 
                             appPlaybackBuffer.usbReadCompleteBufferLevel,
                             appPlaybackBuffer.usbReadQueueCnt,
                             appPlaybackBuffer.codecWriteQueueCnt,
                             appPlaybackBuffer.codecWriteCompleteCnt);
#endif //DEBUG_LEVEL==3

                queueEmpty = false;
                APP_LED1_OFF();

                //Check if the this is the initial loading of the playback
                //queue with USB Rx Data 
                if(appData.state == APP_SUBMIT_INITIAL_CODEC_WRITE_REQUEST)
                {
                    //Check that the initial part of the playback queue is
                    //ready for Codec Writes
                    if(_APP_USBReadAllBufferReady())
                    {
                        usbReadCompleteFlag = true;
                        SYS_PRINT("USB INIT READ COMPLETE RC %d:  QL %d RQ %d WQ %d WC %d",
                            appPlaybackBuffer.usbReadCompleteCnt,
                            appPlaybackBuffer.usbReadCompleteBufferLevel,
                            appPlaybackBuffer.usbReadQueueCnt,
                            appPlaybackBuffer.codecWriteQueueCnt,
                            appPlaybackBuffer.codecWriteCompleteCnt);
                        usbReadCompleteFlag = true;
                    }
                }
            }
            break;

            case USB_DEVICE_AUDIO_EVENT_WRITE_COMPLETE:
            {
            }
            break;
            
            case USB_DEVICE_AUDIO_EVENT_CONTROL_SET_CUR:
            {
                //if(((USB_SETUP_PACKET*)pData)->Recipient == 
                //     USB_SETUP_REQUEST_RECIPIENT_INTERFACE)
                //{
                    entityID = 
                       ((USB_AUDIO_CONTROL_INTERFACE_REQUEST*)pData)->entityID;
                    if ((entityID == APP_ID_FEATURE_UNIT))
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
                    //else if (entityID == APP_ID_MIXER_UNIT)
                    //{
                    //    USB_DEVICE_ControlStatus(appData.usbDevHandle, 
                    //                             USB_DEVICE_CONTROL_STATUS_ERROR);
                    //}
                //}
            }
            break;

            case USB_DEVICE_AUDIO_EVENT_CONTROL_GET_CUR:
            {
                entityID = ((USB_AUDIO_CONTROL_INTERFACE_REQUEST*)
                           pData)->entityID;
                if (entityID == APP_ID_FEATURE_UNIT)
                {
                   controlSelector = ((USB_AUDIO_FEATURE_UNIT_CONTROL_REQUEST*)pData)->controlSelector;

                   if (controlSelector == USB_AUDIO_MUTE_CONTROL)
                   {
                       /*Handle Get request*/
                       USB_DEVICE_ControlSend(appData.usbDevHandle, 
                                              (void *)&(appData.dacMute), 
                                              1);
                   }
                }
                //if (((USB_SETUP_PACKET*)pData)->Recipient == 
                //     USB_SETUP_REQUEST_RECIPIENT_INTERFACE)
                //{
                //    if ((entityID == APP_ID_FEATURE_UNIT) || 
                //         (entityID == APP_ID_FEATURE_UNIT_SIDE_TONING))
                //    {
                //       controlSelector = 
                //              ((USB_AUDIO_FEATURE_UNIT_CONTROL_REQUEST*)
                //                            pData)->controlSelector;
                //       if (controlSelector == USB_AUDIO_MUTE_CONTROL)
                //       {
                //           /*Handle Get request*/
                //           USB_DEVICE_ControlSend(appData.usbDevHandle, 
                //                                 (void *)&(appData.dacMute), 
                //                                 1);
                //       }
                //    }
                    
                //    else if (entityID == APP_ID_MIXER_UNIT)
                //    {
                //        USB_DEVICE_ControlStatus (appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
                //    }
                //}
                //else if (((USB_SETUP_PACKET*)pData)->Recipient == 
                //           USB_SETUP_REQUEST_RECIPIENT_ENDPOINT)
                //{
                //    controlSelector = ((USB_AUDIO_ENDPOINT_CONTROL_REQUEST*)
                //                         pData)->controlSelector;
                //    if (controlSelector == USB_AUDIO_SAMPLING_FREQ_CONTROL)
                //    {
                //        if (((USB_AUDIO_ENDPOINT_CONTROL_REQUEST*)
                //                       pData)->endpointNumber == HEADPHONE_EP)
                //        {
                //           //A control write transfer received from Host. Now receive data from Host.
                //           USB_DEVICE_ControlSend(appData.usbDevHandle, (void *)&(appData.sampleFreq), 3 );
                //        }
                //    }
                //}
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
    #if defined( ENABLE_SYS_PRINT )    
    SYS_PRINT_Init();
    SYS_PRINT("----------------------------------------");
    SYS_PRINT("- Starting:");
    SYS_PRINT("----------------------------------------");
    #endif

#ifdef USE_DISPLAY
    //Update Version String
    display_init(&DISPLAY_STATS);
#endif

#if defined (DEBUG_TONE_CODEC_TX)
    int i;
    int     numSamples = sizeof(testBuffer2)/8;

    #if defined(DEBUG_TONE_SAWTOOTH)
      SYS_PRINT("DEBUG_SAWTOOTH TX: TONE %d samples(%d bytes)", 
            numSamples, sizeof(testBuffer2));
    #else
      SYS_PRINT("DEBUG_TONETX: TONE %d samples(%d bytes) - USB buffer %d bytes", 
            sizeof(testBuffer2)/8,sizeof(testBuffer2), APP_MAX_NO_BYTES_IN_USB_BUFFER);
    #endif

    #if defined(DEBUG_TONE_SAWTOOTH)
    int32_t qcode = 0;
    int32_t incr = 0x00010000; 
    int32_t amp  = incr * numSamples;

    SYS_PRINT("SAWTOOTH:  Amp 0x%0x Incr 0x%0x #Samples/Cycle %d (0x%0x) ", 
             amp, incr, numSamples, numSamples);

    #endif //DEBUG_TONE_SAWTOOTH

    for (i=0; i < numSamples; i++)
    {
        #if defined(DEBUG_TONE_SAWTOOTH)
        testBuffer2[i].leftData     = (qcode<<8) ;  //shift to 24bit boundary
        testBuffer2[i].rightDataPad = (qcode<<8) ;
        qcode += incr;

        #else  //Scale the Sin tone

        testBuffer2[i].leftData     &= 0xFFFFFF00;  //mask off LS byte  
        testBuffer2[i].rightDataPad &= 0xFFFFFF00;

        #endif //DEBUG_TONE_SAWTOOTH
    }
#else
    SYS_PRINT("AUDIO QUEUE: buffer queue size %d(%d bytes))",
           APP_PLAYBACK_QUEUE_BUFFER_SIZE, 
           sizeof(appPlaybackBuffer.playbackBuffer));

    SYS_PRINT("AUDIO BUFFERS: USB buffer %d(%d bytes), CODEC buffer %d(%d bytes)",
             APP_MAX_NO_BYTES_IN_USB_BUFFER/APP_USB_SAMPLE_SIZE_BYTES, 
             APP_MAX_NO_BYTES_IN_USB_BUFFER,
             APP_MAX_NO_OF_SAMPLES_IN_A_USB_FRAME*APP_CODEC_SAMPLE_SIZE_BYTES/
                 APP_CODEC_SAMPLE_SIZE_BYTES,
             APP_MAX_NO_OF_SAMPLES_IN_A_USB_FRAME*APP_CODEC_SAMPLE_SIZE_BYTES);
#endif


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
        // Configure CODEC
        // Transition from: APP_STATE_WAIT_FOR_CONFIGURATION
        // Wait until: SYS_STATUS_READY 
        // Transition To: 
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
                       
                if (appData.codecClientWrite.handle != DRV_HANDLE_INVALID) 
                {
                    SYS_PRINT("APP_STATE_CODEC_OPEN: status(%d)", codecStatus);
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
            DRV_CODEC_BufferEventHandlerSet(
                        appData.codecClientWrite.handle,
                        appData.codecClientWrite.bufferHandler,
                        appData.codecClientWrite.context);

            //Default Value -- Never changes for usb_speaker.
            DRV_CODEC_SamplingRateSet(appData.codecClientWrite.handle, 
                                      appData.sampleFreq);

            //Enable the I2S data pins 
            PLIB_SPI_PinEnable(SPI_ID_1, SPI_PIN_DATA_IN);      

            appData.codecConfigured = true;

            appData.state = APP_STATE_INIT;
            SYS_PRINT("APP_STATE_SET_BUFFER_HANDLER: Set SampleRate %d", 
                    appData.sampleFreq);
        }
        break;
        //---------------------------------------------------------------------
        // Application's initial state. 
        //---------------------------------------------------------------------
        case APP_STATE_INIT:
        {    
            _APP_Init_PlaybackBufferQueue();
            queueFull = false;
            queueEmpty = true;
            APP_LED1_ON();
            APP_LED2_OFF();

            /* Open the device layer */

            //Open USB for Read/Write //WRITE??? only
            appData.usbDevHandle = USB_DEVICE_Open(USB_DEVICE_INDEX_0,
                                                   DRV_IO_INTENT_READWRITE); 

            if (appData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
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
                appData.state = APP_STATE_INIT;
            }
            SYS_PRINT("APP_STATE_INIT: USB open R/W (Sample Rate %d Hz)",
                    appData.sampleFreq);

        } //End APP_STATE_INIT
        //Fall Through

        //---------------------------------------------------------------------
        // Wait for USB Connection to be configured
        //---------------------------------------------------------------------
        case APP_STATE_WAIT_FOR_CONFIGURATION:
        {
            //Check if Host has configured the Device.
            if (appData.isConfigured == true)
            {
                appData.state = APP_IDLE;
                SYS_PRINT("USB HOST CONFIGURED");
            }
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
                     APP_USB_SPEAKER_PLAYBACK_STEREO_96KHZ)
            {
                _APP_Init_PlaybackBufferQueue();

                queueFull = false;
                queueEmpty = false;
                APP_LED1_OFF();
                APP_LED2_OFF();

                //To check alignment
                //int value1 = sizeof(APP_PLAYBACK_BUFFER)%16; 

                //Fill the queue with USB Read Data
                usbReadCompleteFlag = false;  //Actually USB Init Q Read Complete

                //QUEUE up USB Reads for the full playback queue
                for (i = 0; 
                     //i < APP_PLAYBACK_QUEUE_BUFFER_SIZEi/QUEUE_USB_INIT_PART; 
                     i < APP_PLAYBACK_QUEUE_BUFFER_SIZE; 
                     i++)
                {
                    //USB Read to Head of Codec Playback Buffer
                    APP_PLAYBACK_BUFFER* usbReadBuffer = 
                            &appPlaybackBuffer.playbackBuffer[i];

                    if (usbReadBuffer != NULL && 
                        !usbReadBuffer->codecInUse && 
                        !usbReadBuffer->usbInUse)
                    {

                        usbReadBuffer->usbReadCompleted = false;
                        usbReadBuffer->usbInUse = true;

                        //Read to Queue HEAD
                        audioErr1 = USB_DEVICE_AUDIO_Read(
                                        USB_DEVICE_INDEX_0, 
                                        &usbReadBuffer->usbReadHandle, 
#ifdef DEBUG_BSUBFRAMESIZE_3
                                        1, usbReadBuffer->buffer,  //24 bit buffer 
#else
                                        1, usbReadBuffer->buffer32, 
#endif //DEBUG_BSUBFRAMESIZE_3
                                        appData.USBReadBufSize); //96 Samples

                        if(audioErr1 != USB_DEVICE_AUDIO_RESULT_OK)
                        {
                            //USB Read Queue Full and does not match the APP 
                            //read queue size--ERROR.
                            usbReadBuffer->usbInUse = false;
                            break;
                        }
                        else
                        {
                            //USB Queue Buffer OK 
                            appPlaybackBuffer.usbReadQueueCnt++;
                            appPlaybackBuffer.usbReadIdx = _APP_GetNextIdx(appPlaybackBuffer.usbReadIdx);
                        }

                        //Increment the Playback Queue HEAD
                        
                    }
                } //End USB Audio Read Queue loop

                SYS_PRINT("***INIT USB READ QUEUED RC%d: RQ %d",
                         appPlaybackBuffer.usbReadCompleteCnt,
                         appPlaybackBuffer.usbReadQueueCnt);
                appData.state = APP_SUBMIT_INITIAL_CODEC_WRITE_REQUEST;
            } //activeInterfaceAlternateSetting
            else
            {
                _APP_Init_PlaybackBufferQueue();

                queueFull = false;
                queueEmpty = false;
                APP_LED1_OFF();
                APP_LED2_OFF();

            }//No activeInterfaceAlternateSetting

        } //End APP_SUBMIT_INITIAL_USB_READ_REQUEST:
        break;


        //---------------------------------------------------------------------
        // Initial Codec Write Request
        //
        // Transition From:
        //     APP_SUBMIT_INITIAL_USB_READ_REQUEST 
        //       -- initiates USB reads to playback queue buffers.
        //          and waits for a number of them to complete
        //     USB_DEVICE_AUDIO_EVENT_READ_COMPLETE:
        //       -- USB read completion
        //
        // Transition To: 
        //----------------------------------------------------------------------
        case APP_SUBMIT_INITIAL_CODEC_WRITE_REQUEST:
        {    
            //Chekc that initial part of read queue is ready to write
            if (appData.activeInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_STEREO_96KHZ
                && usbReadCompleteFlag)
            {

                //QUEUE UP Codec Writes for the initial part of the read queue
                for (i = 0;
                     i < APP_PLAYBACK_QUEUE_BUFFER_SIZE/QUEUE_USB_INIT_PART; 
                     i++)
                {

                    //TX HEAD of Playback Queue
                    int8_t codecWriteIdx = appPlaybackBuffer.codecWriteIdx;
                        APP_PLAYBACK_BUFFER* current = 
                            &appPlaybackBuffer.playbackBuffer[codecWriteIdx];

                    if(current->usbReadCompleted && !current->codecInUse)
                    {

                        #ifdef DEBUG_TONE_CODEC_TX
                        //TONE Data

                        //Write stereo tone to output instead of USB data.
                        DRV_CODEC_BufferAddWrite(appData.codecClientWrite.handle, 
                                                 &current->codecWriteHandle,
                                                 testBuffer2, sizeof(testBuffer2));
                        #else
                        //USB Data

#ifdef DEBUG_BSUBFRAMESIZE_3
                        //Unpack the 24 bit buffer to the 32 bit buffer
                        // 4bytes x 4words = 12 bytes -> 4x24 bit samples
                        int i;
                        int j;
                        int numUsb24BitSamples = appData.USBReadBufSize/3;
                        //BITS24 *         in24Ptr  = (BITS24*) (current->buffer);
                        //DRV_I2S_DATA32 * out32Ptr = 
                        //                  (DRV_I2S_DATA32 *) (current->buffer32);

                        uint32_t *src = (uint32_t *) current->buffer;
                        uint32_t *dst = (uint32_t *) current->buffer32; 

                        for (i=0, j=0; j<numUsb24BitSamples; i+=3, j+=4) 
                        {
                            //Every 3 32 bit words converted to 4 32 bit samples
                            sa = src[i+0]; //LS
                            sb = src[i+1];
                            sc = src[i+2]; //MS

                            //Little ENDIAN - MS bytes first
                            //dst[j+0] = sa & 0xFFFFFF00;
                            dst[j+0] = sa<<8; 
                            //dst[j+1] = ((sa<<24) | (sb>>8)) & 0xFFFFFF00;
                            dst[j+1] = ((sb<<16) | (sa>>16)) & 0xFFFFFF00;
                            //dst[j+2] = ((sb<<16) | (sc>>16)) & 0xFFFFFF00;
                            dst[j+2] = ((sb>>8) | (sc<<24)) & 0xFFFFFF00;
                            //dst[j+3] = sc<<8;
                            dst[j+3] = sc & 0xFFFFFF00;
                        }

                        //Initial CODEC Write
                        current->codecInUse = true;
#endif //DEBUG_BSUBFRAMESIZE_3

                        //current->usbInUse   = false;     
                        DRV_CODEC_BufferAddWrite(
                                appData.codecClientWrite.handle, 
                                &current->codecWriteHandle,
                                current->buffer32, 
                                sizeof(current->buffer32)); //96*8=768 bytes

                       #endif //DEBUG_TONE_CODEC TX

                       if(current->codecWriteHandle != DRV_CODEC_BUFFER_HANDLE_INVALID)
                       {
                            appPlaybackBuffer.codecWriteQueueCnt++;
                            appPlaybackBuffer.codecWriteIdx = 
                                    _APP_GetNextIdx(codecWriteIdx);
                       }
                       else
                       {
                           current->codecInUse = false;
                           // CODEC doesn't have enough write buffers
                           // should never happen
                           Nop();
                       }
                    }
                } //End Initial Codec Write Queue Loop
                SYS_PRINT("CODEC INIT WRITES QUEUED RC %d: QL %d RQ %d WQ %d WC %d",
                         appPlaybackBuffer.usbReadCompleteCnt,
                         appPlaybackBuffer.usbReadCompleteBufferLevel,
                         appPlaybackBuffer.usbReadQueueCnt,
                         appPlaybackBuffer.codecWriteQueueCnt,
                         appPlaybackBuffer.codecWriteCompleteCnt);
                appData.state = APP_PROCESS_DATA;
                usbReadCompleteFlag = false;
            }

        } // End State APP_SUBMIT_INITIAL_CODEC_WRITE_REQUEST:
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
            if (appData.activeInterfaceAlternateSetting == 
                APP_USB_SPEAKER_PLAYBACK_STEREO_96KHZ)  //96Khz
            {
                //----------------------------
                //USB read
                //--Read to usbReadIdx (Next Queue HEAD index)
                //--Returns NULL pointer if FULL
                APP_PLAYBACK_BUFFER * usbReadBuffer = 
                     &appPlaybackBuffer.
                        playbackBuffer[appPlaybackBuffer.usbReadIdx];

                if (appPlaybackBuffer.usbReadCompleteBufferLevel == 
                    APP_PLAYBACK_QUEUE_BUFFER_SIZE)
                {
                    //APP_LED3_TOGGLE();
                    //APP_LED4_TOGGLE();
                    queueFull = true;
                    APP_LED2_ON();
                      SYS_PRINT("**QUEUE FULL: RC %d: QL %d RQ %d WQ %d WC %d\r\n",
                                  appPlaybackBuffer.usbReadCompleteCnt,
                                  appPlaybackBuffer.usbReadCompleteBufferLevel,
                                  appPlaybackBuffer.usbReadQueueCnt,
                                  appPlaybackBuffer.codecWriteQueueCnt,
                                  appPlaybackBuffer.codecWriteCompleteCnt);
                }

                if ( usbReadBuffer != NULL && 
                     !usbReadBuffer->usbInUse && 
                     !usbReadBuffer->codecInUse)
                {
                    usbReadBuffer->usbReadCompleted = false;

                    audioErr1 = USB_DEVICE_AUDIO_Read(USB_DEVICE_INDEX_0, 
                                          &usbReadBuffer->usbReadHandle, 
#ifdef DEBUG_BSUBFRAMESIZE_3
                                          1, usbReadBuffer->buffer,  
#else
                                          1, usbReadBuffer->buffer32,  
#endif //DEBUG_BSUBFRAMESIZE_3
                                          appData.USBReadBufSize);//96*3 bytes

                    if(audioErr1 == USB_DEVICE_AUDIO_RESULT_OK)
                    {
                        //USB Buffer Read OK
                        usbReadBuffer->usbInUse = true;

                        appPlaybackBuffer.usbReadQueueCnt++;
                        appPlaybackBuffer.usbReadIdx = 
                                _APP_GetNextIdx(appPlaybackBuffer.usbReadIdx);
                    }
                    else
                    {
                        //USB Read Fail to queue this buffer
                        //--> Wait for queue to empty
                        usbReadBuffer->usbInUse = false;
                    }
                } //End USB Queue Read
                

                //------------------------- 
                //CODEC ADD WRITE
                //TX codecWriteIdx to CODEC (Next Queue TAIL Index)
                int8_t codecWriteIdx = appPlaybackBuffer.codecWriteIdx;

                APP_PLAYBACK_BUFFER* current = 
                        &appPlaybackBuffer.playbackBuffer[codecWriteIdx];

                //CHECK for UNDERFLOW: Reads Queued/Read Complete--None/
                //                     --> No current Writes Queued, No read
                //                         data available.
                if ( (appPlaybackBuffer.usbReadCompleteBufferLevel == 0) &&
                     (appPlaybackBuffer.usbReadQueueCnt > 0) && 
                     (appPlaybackBuffer.codecWriteQueueCnt == 0) )
                {
                    APP_LED1_ON();
                    SYS_PRINT("*QUEUE UNDER: RC %d RL %d, RQ %d WQ %d WC %d",
                            appPlaybackBuffer.usbReadCompleteCnt,
                            appPlaybackBuffer.usbReadCompleteBufferLevel,
                            appPlaybackBuffer.usbReadQueueCnt,
                            appPlaybackBuffer.codecWriteQueueCnt,
                            appPlaybackBuffer.codecWriteCompleteCnt);
                }

                if(current->usbReadCompleted && !current->codecInUse)
                {
                    //Codec has not written this completed read buffer
                
                    #ifdef DEBUG_TONE_CODEC_TX
                    //TONE Data

                    //Write stereo tone to output instead of USB data.
                    DRV_CODEC_BufferAddWrite(appData.codecClientWrite.handle, 
                                             &current->codecWriteHandle,
                                             testBuffer2, sizeof(testBuffer2));
                    #else
                    //USB Data

#ifdef DEBUG_BSUBFRAMESIZE_3
                    //Unpack the 24 bit buffer to the 32 bit buffer
                    int i;
                    int j;
                    int numUsb24BitSamples = appData.USBReadBufSize/3;
                    //BITS24 *         in24Ptr  = (BITS24*) (current->buffer);
                    //DRV_I2S_DATA32 * out32Ptr = 
                    //                  (DRV_I2S_DATA32 *) (current->buffer32);

                    uint32_t *src = (uint32_t *) current->buffer;
                    uint32_t *dst = (uint32_t *) current->buffer32; 


                    for (i=0, j=0; j<numUsb24BitSamples; i+=3, j+=4) 
                    {
                        //Every 3 32 bit words converted to 4 32 bit samples
                        sa = src[i+0]; //LS
                        sb = src[i+1];
                        sc = src[i+2]; //MS

                        //Little ENDIAN - MS bytes first
                        //dst[j+0] = sa & 0xFFFFFF00;
                        dst[j+0] = sa<<8; 
                        //dst[j+1] = ((sa<<24) | (sb>>8)) & 0xFFFFFF00;
                        dst[j+1] = ((sb<<16) | (sa>>16)) & 0xFFFFFF00;
                        //dst[j+2] = ((sb<<16) | (sc>>16)) & 0xFFFFFF00;
                        dst[j+2] = ((sb>>8) | (sc<<24)) & 0xFFFFFF00;
                        //dst[j+3] = sc<<8;
                        dst[j+3] = sc & 0xFFFFFF00;
                    }
#endif //DEBUG_BSUBFRAMESIZE_3

                    DRV_CODEC_BufferAddWrite(
                            appData.codecClientWrite.handle, 
                            &current->codecWriteHandle,
                            current->buffer32, 
                            appData.codecClientWrite.bufferSize); //96*8=768

                    #endif
                    if (current->codecWriteHandle != DRV_CODEC_BUFFER_HANDLE_INVALID)
                    {
                        current->codecInUse = true;
                        
                        appPlaybackBuffer.codecWriteQueueCnt++;
                        appPlaybackBuffer.codecWriteIdx = _APP_GetNextIdx(codecWriteIdx);

                        //Check for QUEUE Empty 
                        if (appPlaybackBuffer.codecWriteIdx == appPlaybackBuffer.usbReadIdx)
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
                        // CODEC doesn't have enough write buffers
                        // --Wait till one becomes available
                        current->codecInUse = false;
                    }
                }
                else
                {
                    Nop();
                }

            } //End APP_USB_SPEAKER_PLAYBACK_STEREO_96KHZ
        }
        break; //End case APP_PROCESS_DATA:

        case APP_MUTE_AUDIO_PLAYBACK:
        {
            if (appData.activeInterfaceAlternateSetting == 0 && 
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
        }
        break;

        //RTOS Version
        //case APP_SAMPLING_FREQUENCY_CHANGE:
        //{
        //    //Changes sampling rate for Record and Playback (both stereo)
        //    DRV_CODEC_SamplingRateSet(appData.codecClientWrite.handle, 
        //                                      appData.sampleFreq);
        //    //Reinit the queue
        //    appData.state = APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD;
        //}
        //break;
        
        case APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD:
        {
            SYS_STATUS codecStatus;

            if (appData.activeInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_NONE)
            {
                _APP_Init_PlaybackBufferQueue();
                queueFull = false;
                queueEmpty = true;
                APP_LED1_ON();     //QUEUE Empty
                APP_LED2_OFF();     //QUEUE Empty

                codecStatus = DRV_CODEC_Status(sysObjdrvCodec0);
                if (SYS_STATUS_READY == codecStatus)
                {
                    DRV_CODEC_MuteOn(appData.codecClientWrite.handle);
                    appData.state = APP_IDLE;
                }
                appData.state = APP_IDLE;
            }
            else if(appData.activeInterfaceAlternateSetting == APP_USB_SPEAKER_PLAYBACK_STEREO_96KHZ)
            {
                codecStatus = DRV_CODEC_Status(sysObjdrvCodec0);
                if (SYS_STATUS_READY == codecStatus)
                {
                    DRV_CODEC_MuteOff(appData.codecClientWrite.handle);
                    _APP_Init_PlaybackBufferQueue();
                    appData.state =  APP_SUBMIT_INITIAL_USB_READ_REQUEST;
                }
            }
        }
        break;

        case APP_IDLE:
        {
            if ((appData.activeInterfaceAlternateSetting == 
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

    } //End switch(appData.state))

    //Save state for debug of transitions
    appData.lastState = appData.state;

} //End APP_Tasks()


//******************************************************************************
// APP_CODECBufferEventHandler
//
// This function is called back by the CODEC driver when
// a CODEC data buffer RX completes.
//******************************************************************************

//******************************************************************************
// APP_CODECBufferEventHandler()
//
// Application CODEC Write buffer Event handler.
// This function is called back by the CODEC driver when
// a CODEC data buffer TX completes.
//******************************************************************************
void APP_CODECBufferEventHandler(DRV_CODEC_BUFFER_EVENT event,
                                 DRV_CODEC_BUFFER_HANDLE handle, 
                                 uintptr_t context )
{
    switch(event)
    {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        {
            //CODEC Buffer Write Complete

            appPlaybackBuffer.codecWriteCompleteCnt++;

            //Make this buffer is ready for USB Write
            int codecCompleteIndex = _APP_ClearCodecReturnBuffer(handle);

            if (codecCompleteIndex < 0)
            {
                Nop();  //Buffer error
            }

            //Remove 1 buffer from USB complete read queue
            appPlaybackBuffer.codecWriteQueueCnt--;
            appPlaybackBuffer.usbReadCompleteBufferLevel--;

            //Underflow Check -- Reads queued/Read completes 0--All available
            //                   already written.
            if (appPlaybackBuffer.usbReadCompleteBufferLevel == 0 &&
                appPlaybackBuffer.usbReadQueueCnt > 0 &&
                appPlaybackBuffer.codecWriteQueueCnt == 0)
            {
                Nop();
            } //UNDERFLOW

            
#if DEBUG_LEVEL > 0
            if (!(appPlaybackBuffer.codecWriteCompleteCnt%DEBUG_REPORT_INTERVAL))
            {
#if DEBUG_LEVEL == 0
                SYS_PRINT("**WR COMPLETE RC %d: QL %d RQ %d WQ %d WC %d",
                         appPlaybackBuffer.usbReadCompleteCnt,
                         appPlaybackBuffer.usbReadCompleteBufferLevel,
                         appPlaybackBuffer.usbReadQueueCnt,
                         appPlaybackBuffer.codecWriteQueueCnt,
                         appPlaybackBuffer.codecWriteCompleteCnt);
#endif
#if DEBUG_LEVEL == 3 
                    SYS_PRINT("WCOMP: QL %d  RQ %d  WQ %d WC %d", 
                             appPlaybackBuffer.usbReadCompleteBufferLevel,
                             appPlaybackBuffer.usbReadQueueCnt,
                             appPlaybackBuffer.codecWriteQueueCnt,
                             appPlaybackBuffer.codecWriteCompleteCnt);
#endif //DEBUG_LEVEL==3
            }
#endif //DEBUG_LEVEL
        } //End case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        break;

        case DRV_CODEC_BUFFER_EVENT_ERROR:
        {
        } break;

        case DRV_CODEC_BUFFER_EVENT_ABORT:
        {
        } break;
    }; //End switch(event)
}


//******************************************************************************
// _APP_SetUSBReadBufferReady()
//--Set the USB Read Buffer Data Ready for CODEC Write 
//******************************************************************************
static int _APP_SetUSBReadBufferReady(USB_DEVICE_AUDIO_TRANSFER_HANDLE handle)
{
    int i=0;
    for(i=0;i<APP_PLAYBACK_QUEUE_BUFFER_SIZE;i++)
    {
        if (appPlaybackBuffer.playbackBuffer[i].usbReadHandle == handle)
        {
            appPlaybackBuffer.playbackBuffer[i].usbReadCompleted = true;
            //appPlaybackBuffer.playbackBuffer[i].usbInUse = false;
            return i;
        }
    }
    return -1;
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
    appPlaybackBuffer.codecWriteIdx = 0;
    appPlaybackBuffer.usbReadIdx = 0;

    for(i=0;i<APP_PLAYBACK_QUEUE_BUFFER_SIZE;i++)
    {
        appPlaybackBuffer.playbackBuffer[i].codecInUse = false;
        appPlaybackBuffer.playbackBuffer[i].usbInUse = false;
        appPlaybackBuffer.playbackBuffer[i].usbReadCompleted = false;
    }
    appPlaybackBuffer.usbReadCompleteBufferLevel = 0;
    appPlaybackBuffer.codecWriteCompleteCnt      = 0;
    appPlaybackBuffer.usbReadCompleteCnt         = 0;
    appPlaybackBuffer.usbReadQueueCnt            = 0;
    appPlaybackBuffer.codecWriteQueueCnt         = 0;
}

//******************************************************************************
//_APP_GetNextIdx()
//Increment the Head or Tail Index to Codec Playback Buffer Queue
//******************************************************************************
static uint8_t _APP_GetNextIdx(uint8_t index)
{
    if (index == 32)
    {
        Nop();
    }
    return (index+1)%APP_PLAYBACK_QUEUE_BUFFER_SIZE;
}

//******************************************************************************
// _APP_ClearCodecReturnBuffer()
//--Clear the Codec Playback Buffer to allow for  USB Reads 
//******************************************************************************
static int _APP_ClearCodecReturnBuffer(DRV_CODEC_BUFFER_HANDLE handle)
{
    int i = 0;
    for(i=0;i<APP_PLAYBACK_QUEUE_BUFFER_SIZE;i++)
    {
        if(appPlaybackBuffer.playbackBuffer[i].codecWriteHandle == handle)
        {
            appPlaybackBuffer.playbackBuffer[i].codecWriteHandle = 
                    DRV_CODEC_BUFFER_HANDLE_INVALID;
            appPlaybackBuffer.playbackBuffer[i].codecInUse = false;
            appPlaybackBuffer.playbackBuffer[i].usbInUse = false;
            appPlaybackBuffer.playbackBuffer[i].usbReadCompleted = false;
            appPlaybackBuffer.playbackBuffer[i].usbReadHandle = 
                    USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;

            return i;
        }
    }
    
    return -1; //ERROR
}

//******************************************************************************
// Convert the packed 24 bit byte array value to 32 bit word value
//******************************************************************************
int32_t _Convert24to32bit(BITS24 val)
{
        if ( val.bytes[2] & 0x80 ) // Is this a negative?  Then we need to siingn extend.
        {
            return ( 0xff << 24)         | 
                    (val.bytes[2] << 16) | 
                    (val.bytes[1] << 8)  | 
                    (val.bytes[0] << 0);
        }
        else
        {
            return ( val.bytes[2] << 16) |
                    (val.bytes[1] << 8)  | 
                    (val.bytes[0] << 0);
        }
}
/*******************************************************************************
 End of File
 */
