/*******************************************************************************
  USB DEVICE Audio Speaker Demo Application

  File Name:
    app.c

  Summary:
    USB DEVICE Audio Speaker Demo application

  Description:
    This file contains the USB DEVICE Audio Speaker Demo application logic.
 *******************************************************************************/


// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

// Note: 565 background color used in app schemes (3,11,7) is 24,44,56 in 
// Windows 888 space

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variable Definitions
// *****************************************************************************
// *****************************************************************************
uint8_t __attribute__((coherent)) __attribute__((aligned(16))) txBuffer[4];
uint32_t someVar;
volatile uint32_t usbSkip, usbReadComplete, codecSkip, codecWriteComplete;
volatile uint32_t queueCount, writeQueueCount;
volatile uint32_t codecNextBuffer, usbNextBuffer;
uint32_t codecWriteCount, bufferWriteError, usbReadError, usbReadCount;
size_t bufferCount;
bool justOnce;
USB_DEVICE_AUDIO_V2_RESULT audioErr =0;
uint32_t loopCount;
uint32_t usbOverLap, usbCount;

uint32_t LRClockSync;
#define LRCLOCKSYNC 2000        // # of times we let the DMA ISR call syncLRClock, to make sure it is sync'ed with incoming USB packets
// the number is somewhat arbitrary; 100 is too low and a much higher number will result in too long a delay before audio begins playing

uint32_t blinkTimer, checkTimer, checkUSBtimer;
uint8_t blinkSwitch, checkSwitch, checkUSBswitch;

uint8_t led1On, led2On, led3On, led4On, led5On;
uint32_t led1StuckOn, led2StuckOff, led2StuckOn, led3StuckOff, led3StuckOn, led5StuckOn, led5StuckOff;
uint32_t led2StuckOnMax, led2StuckOffMax, led3StuckOnMax, led3StuckOffMax, led5StuckOnMax, led5StuckOffMax;

#define LED5_TASK   1     // uncomment for LED5 to monitor APP_TASKS function
//#define LED5_USB   1     // uncomment for LED5 to monitor USB read activity

#define LEVEL1_ERROR 1      // mute, init, submit read request
#define LEVEL2_ERROR 2      // mute, init, detach/attach usb, wait for config

#define MAX_BAD_PACKETS 20

// *****************************************************************************
/* Application Data

  Summary:
    Contains application data

  Description:
    This structure contains application data.
*/

APP_DATA appData =
{
    /* Device Layer Handle  */
    .usbDevHandle = -1,

    /* USB Audio Instance index for this app object 0*/
    .audioInstance = 0,

     /* app state */
    .state = APP_STATE_INIT,

    .codecClient.context = (uintptr_t)&someVar,

    .codecClient.bufferHandler = (DRV_CODEC_BUFFER_EVENT_HANDLER) 
                                       APP_CodecBufferEventHandler,

    /* device configured status */
    .isConfigured = false,

    /* Read Transfer Handle */
    .isReadComplete = false,

    /* Write Transfer complete */
    .isWriteComplete = false,

    /* Read transfer handle */

    /* Write transfer handle */
    .writeTransferHandle = USB_DEVICE_AUDIO_V2_TRANSFER_HANDLE_INVALID,

    /* Initialize active interface setting to 0. */
    .activeInterfaceAlternateSetting = APP_USB_SPEAKER_PLAYBACK_NONE,

    /* DAC is not muted initially */
    .dacMute = false,
    .lastClockSource = APP_USB_AUDIO_SAMPLING_RATE_INITIAL,
    // Clock Source
    .clockSource = APP_USB_AUDIO_SAMPLING_RATE_INITIAL,
    .clockValid = 1,
 
#ifdef SUPPORT_ALL_SAMPLE_RATES
    .clockSourceRange.numSampleRate = 7,
    .clockSourceRange.sampleRate32.dMin = APP_USB_AUDIO_SAMPLING_RATE_32KHZ,
    .clockSourceRange.sampleRate32.dMax = APP_USB_AUDIO_SAMPLING_RATE_32KHZ,
    .clockSourceRange.sampleRate32.dRes = 0,
    .clockSourceRange.sampleRate44.dMin = APP_USB_AUDIO_SAMPLING_RATE_44_1KHZ,
    .clockSourceRange.sampleRate44.dMax = APP_USB_AUDIO_SAMPLING_RATE_44_1KHZ,
    .clockSourceRange.sampleRate44.dRes = 0,
    .clockSourceRange.sampleRate48.dMin = APP_USB_AUDIO_SAMPLING_RATE_48KHZ,
    .clockSourceRange.sampleRate48.dMax = APP_USB_AUDIO_SAMPLING_RATE_48KHZ,
    .clockSourceRange.sampleRate48.dRes = 0,
    .clockSourceRange.sampleRate88.dMin = APP_USB_AUDIO_SAMPLING_RATE_88_2KHZ,
    .clockSourceRange.sampleRate88.dMax = APP_USB_AUDIO_SAMPLING_RATE_88_2KHZ,
    .clockSourceRange.sampleRate88.dRes = 0,
    .clockSourceRange.sampleRate96.dMin = APP_USB_AUDIO_SAMPLING_RATE_96KHZ,
    .clockSourceRange.sampleRate96.dMax = APP_USB_AUDIO_SAMPLING_RATE_96KHZ,
    .clockSourceRange.sampleRate96.dRes = 0,
    .clockSourceRange.sampleRate176.dMin = APP_USB_AUDIO_SAMPLING_RATE_176_4KHZ,
    .clockSourceRange.sampleRate176.dMax = APP_USB_AUDIO_SAMPLING_RATE_176_4KHZ,
    .clockSourceRange.sampleRate176.dRes = 0,
    .clockSourceRange.sampleRate192.dMin = APP_USB_AUDIO_SAMPLING_RATE_192KHZ,
    .clockSourceRange.sampleRate192.dMax = APP_USB_AUDIO_SAMPLING_RATE_192KHZ,
    .clockSourceRange.sampleRate192.dRes = 0,
#else
    .clockSourceRange.numSampleRate = 4,        // limited to 44.1, 88.2, 176.4 and 192
    .clockSourceRange.sampleRate44.dMin = APP_USB_AUDIO_SAMPLING_RATE_44_1KHZ,
    .clockSourceRange.sampleRate44.dMax = APP_USB_AUDIO_SAMPLING_RATE_44_1KHZ,
    .clockSourceRange.sampleRate44.dRes = 0,
    .clockSourceRange.sampleRate88.dMin = APP_USB_AUDIO_SAMPLING_RATE_88_2KHZ,
    .clockSourceRange.sampleRate88.dMax = APP_USB_AUDIO_SAMPLING_RATE_88_2KHZ,
    .clockSourceRange.sampleRate88.dRes = 0,
    .clockSourceRange.sampleRate176.dMin = APP_USB_AUDIO_SAMPLING_RATE_176_4KHZ,
    .clockSourceRange.sampleRate176.dMax = APP_USB_AUDIO_SAMPLING_RATE_176_4KHZ,
    .clockSourceRange.sampleRate176.dRes = 0,
    .clockSourceRange.sampleRate192.dMin = APP_USB_AUDIO_SAMPLING_RATE_192KHZ,
    .clockSourceRange.sampleRate192.dMax = APP_USB_AUDIO_SAMPLING_RATE_192KHZ,
    .clockSourceRange.sampleRate192.dRes = 0,
#endif    
     
    // Clock Select
    .clockSelectIndex = 1,

    // currentAudioControl
    .currentAudioControl = APP_USB_CONTROL_NONE,
    .codecClient.isMute = false,
    .codecClient.isInitialized = false,
    .display.update = false,            // not used in Harmony version 2.02 (graphics disabled))
    .codecClient.samplingRate = APP_USB_AUDIO_SAMPLING_RATE_INITIAL    
};

void APP_USBDeviceAudioEventHandler
(
    USB_DEVICE_AUDIO_V2_INDEX iAudio ,
    USB_DEVICE_AUDIO_V2_EVENT event ,
    void * pData,
    uintptr_t context
)
{
    USB_DEVICE_AUDIO_V2_EVENT_DATA_SET_ALTERNATE_INTERFACE * interfaceInfo;
    USB_DEVICE_AUDIO_V2_EVENT_DATA_READ_COMPLETE * readEventData;
    USB_DEVICE_AUDIO_V2_EVENT_DATA_WRITE_COMPLETE * writeEventData;
    if ( iAudio == 0 )
    {
        switch (event)
        {
            case USB_DEVICE_AUDIO_V2_EVENT_READ_COMPLETE:
                readEventData = (USB_DEVICE_AUDIO_V2_EVENT_DATA_READ_COMPLETE *)pData;
                //We have received an audio frame from the Host.
                //Now send this audio frame to Audio Codec for Playback.

                if(readEventData->status == USB_DEVICE_AUDIO_V2_RESULT_OK)
                {
                    for (queueCount=0; queueCount < APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT; queueCount++)
                    {
                        if (readEventData->handle == appData.usbReadQueue.readTransferHandle[queueCount])
                        {
                            appData.usbReadQueue.dataAvailable[queueCount] = true;
#ifdef LED5_USB                            
                            APP_LED5_ON();
#endif
                            led5On = 1;                            
                            appData.codecClient.bufferSize[queueCount] = readEventData->length;
                            usbReadComplete++;
                        }
                    }
                }

            break;

            case USB_DEVICE_AUDIO_V2_EVENT_WRITE_COMPLETE:
                writeEventData = (USB_DEVICE_AUDIO_V2_EVENT_DATA_READ_COMPLETE *)pData;

                if (writeEventData->handle == appData.writeTransferHandle)
                {
                    appData.isWriteComplete = true;
                }
            break;

            case USB_DEVICE_AUDIO_V2_EVENT_INTERFACE_SETTING_CHANGED:
                //We have received a request from USB host to change the Interface-
                //Alternate setting.
                interfaceInfo = (USB_DEVICE_AUDIO_V2_EVENT_DATA_SET_ALTERNATE_INTERFACE *)pData;
                appData.activeInterfaceAlternateSetting = interfaceInfo->interfaceAlternateSetting;
                appData.state = APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD;

            break;
            case USB_DEVICE_AUDIO_V2_CUR_ENTITY_SETTINGS_RECEIVED:
                APP_AudioCurEntitySettingsHandler(iAudio, pData );
                break;

            case USB_DEVICE_AUDIO_V2_RANGE_ENTITY_SETTINGS_RECEIVED:
                APP_AudioRangeEntitySettingsHandler(iAudio, pData );
                break;
                
            case USB_DEVICE_AUDIO_V2_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:
                USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK );
                switch (appData.currentAudioControl)
                {
                    case APP_USB_AUDIO_MUTE_CONTROL:
                    {
                        appData.state = APP_MUTE_AUDIO_PLAYBACK;
                        appData.currentAudioControl = APP_USB_CONTROL_NONE;
                    }
                    break;
                    case APP_USB_AUDIO_CLOCKSOURCE_CONTROL:
                    {

                        appData.maxAudioSamples = appData.clockSource/1000;
                        appData.usbAudioFrameSize = appData.maxAudioSamples*4*2;        
                        appData.usbAudioFrameSizeUa2 = appData.usbAudioFrameSize/8;    
                                                                                
                        appData.usbFeedbackNormal = ((appData.maxAudioSamples/APP_USB_DATA_PACKET_SIZE) << 16);
                        appData.usbFeedbackMax = (((appData.maxAudioSamples/APP_USB_DATA_PACKET_SIZE) + 1) << 16);
                        appData.usbFeedbackMin = (((appData.maxAudioSamples/APP_USB_DATA_PACKET_SIZE) - 1) << 16);    
                                                
                        /*Generating Feedback normal data rate */
                        appData.usbFeedbackNormalValue[0] = appData.usbFeedbackNormal & 0xFF;
                        appData.usbFeedbackNormalValue[1] = (appData.usbFeedbackNormal >> 8) & 0xFF; 
                        appData.usbFeedbackNormalValue[2] = (appData.usbFeedbackNormal >> 16) & 0xFF; 
                        appData.usbFeedbackNormalValue[3] = (appData.usbFeedbackNormal >> 24) & 0xFF;                                  

                        /*Generating Feedback Slow down data rate */                        
                        appData.usbFeedbackMin = appData.usbFeedbackMin | 0x00008000;       
                        appData.usbFeedbackSlowDownValue[0] = appData.usbFeedbackMin & 0xFF;
                        appData.usbFeedbackSlowDownValue[1] = (appData.usbFeedbackMin >> 8) & 0xFF;
                        appData.usbFeedbackSlowDownValue[2] = (appData.usbFeedbackMin >> 16) & 0xFF; 
                        appData.usbFeedbackSlowDownValue[3] = (appData.usbFeedbackMin >> 24) & 0xFF;                  

                        /*Generating Feedback Speedup data rate */                           
                        if(APP_USB_AUDIO_SAMPLING_RATE_32KHZ == appData.clockSource)
                        {
                            appData.usbFeedbackNormal = appData.usbFeedbackNormal | APP_USB_FEEDBACK_32_SPEED_UP; 
                            appData.usbAudioBufferUpperLimit = APP_USB_AUDIO_BUFFER_QUEUE_TUNE_UPPER_LIMIT_32;
                            appData.usbAudioBufferLowerLimit = APP_USB_AUDIO_BUFFER_QUEUE_TUNE_LOWER_LIMIT_32;
                        }
                        else if(APP_USB_AUDIO_SAMPLING_RATE_44_1KHZ == appData.clockSource)
                        {
                            appData.usbFeedbackNormal = appData.usbFeedbackNormal | APP_USB_FEEDBACK_44_1_SPEED_UP;                                                                                                                
                            appData.usbAudioBufferUpperLimit = APP_USB_AUDIO_BUFFER_QUEUE_TUNE_UPPER_LIMIT_44_1;
                            appData.usbAudioBufferLowerLimit = APP_USB_AUDIO_BUFFER_QUEUE_TUNE_LOWER_LIMIT_44_1;                                                                
                        }
                        else if(APP_USB_AUDIO_SAMPLING_RATE_48KHZ == appData.clockSource)
                        {
                            appData.usbFeedbackNormal = appData.usbFeedbackNormal | APP_USB_FEEDBACK_48_SPEED_UP;
                            appData.usbAudioBufferUpperLimit = APP_USB_AUDIO_BUFFER_QUEUE_TUNE_UPPER_LIMIT_48;
                            appData.usbAudioBufferLowerLimit = APP_USB_AUDIO_BUFFER_QUEUE_TUNE_LOWER_LIMIT_48;
                        }                        
                        else if(APP_USB_AUDIO_SAMPLING_RATE_88_2KHZ == appData.clockSource)
                        {
                            appData.usbFeedbackNormal = appData.usbFeedbackNormal | APP_USB_FEEDBACK_88_2_SPEED_UP;
                            appData.usbAudioBufferUpperLimit = APP_USB_AUDIO_BUFFER_QUEUE_TUNE_UPPER_LIMIT_88_2;
                            appData.usbAudioBufferLowerLimit = APP_USB_AUDIO_BUFFER_QUEUE_TUNE_LOWER_LIMIT_88_2;
                        }   
                        else if(APP_USB_AUDIO_SAMPLING_RATE_96KHZ == appData.clockSource)
                        {
                            appData.usbFeedbackNormal = appData.usbFeedbackNormal | APP_USB_FEEDBACK_96_SPEED_UP;
                            appData.usbAudioBufferUpperLimit = APP_USB_AUDIO_BUFFER_QUEUE_TUNE_UPPER_LIMIT_96;
                            appData.usbAudioBufferLowerLimit = APP_USB_AUDIO_BUFFER_QUEUE_TUNE_LOWER_LIMIT_96;
                        }  
                        else if(APP_USB_AUDIO_SAMPLING_RATE_176_4KHZ == appData.clockSource)
                        {
                            appData.usbFeedbackNormal = appData.usbFeedbackNormal | APP_USB_FEEDBACK_176_4_SPEED_UP;
                            appData.usbAudioBufferUpperLimit = APP_USB_AUDIO_BUFFER_QUEUE_TUNE_UPPER_LIMIT_176_4;
                            appData.usbAudioBufferLowerLimit = APP_USB_AUDIO_BUFFER_QUEUE_TUNE_LOWER_LIMIT_176_4;
                        }                          
                        else if(APP_USB_AUDIO_SAMPLING_RATE_192KHZ == appData.clockSource)
                        {
                            appData.usbFeedbackNormal = appData.usbFeedbackNormal | APP_USB_FEEDBACK_192_SPEED_UP;
                            appData.usbAudioBufferUpperLimit = APP_USB_AUDIO_BUFFER_QUEUE_TUNE_UPPER_LIMIT_192;
                            appData.usbAudioBufferLowerLimit = APP_USB_AUDIO_BUFFER_QUEUE_TUNE_LOWER_LIMIT_192;                                                             
                        }
                        else
                        {
                            /* Unsupported Sampling rate tuning */
                            ;
                        }
                        appData.usbFeedbackSpeedUpValue[0] = appData.usbFeedbackNormal & 0xFF;
                        appData.usbFeedbackSpeedUpValue[1] = (appData.usbFeedbackNormal >> 8) & 0xFF; 
                        appData.usbFeedbackSpeedUpValue[2] = (appData.usbFeedbackNormal >> 16) & 0xFF; 
                        appData.usbFeedbackSpeedUpValue[3] = (appData.usbFeedbackNormal >> 24) & 0xFF;                                                                                           
                        
                        appData.state = APP_CLOCKSOURCE_SET;                        
                        appData.currentAudioControl = APP_USB_CONTROL_NONE;
                    }
                    break;
                    case APP_USB_AUDIO_CLOCKSELECT_CONTROL:
                    {
                        appData.currentAudioControl = APP_USB_CONTROL_NONE;
                        // Handle Clock Source Control here.
                    }
                    break;
                    
                    case APP_USB_CONTROL_NONE:
                    {
                        /* Do Nothing */
                        ;
                    }
                    break;                    
                }
            break;
            case  USB_DEVICE_AUDIO_V2_EVENT_CONTROL_TRANSFER_DATA_SENT:
            break;
            default:
                SYS_ASSERT ( false , "Invalid callback" );
            break;
        } //end of switch ( callback )
    }//end of if  if ( iAudio == 0 )
}//end of function APP_AudioEventCallback

int APP_ClockSelectRequestHandler
(
    USB_DEVICE_AUDIO_V2_INDEX iAudio ,
    USB_AUDIO_V2_CONTROL_INTERFACE_REQUEST* controlRequest
)
{
    USB_AUDIO_V2_CLOCKSELECT_CONTROL_REQUEST* clockSelectRequest;
    clockSelectRequest = (USB_AUDIO_V2_CLOCKSELECT_CONTROL_REQUEST*) controlRequest;

    int8_t err = 0;

    if (clockSelectRequest->bRequest == AUDIO_V2_CUR)
    {
        switch(clockSelectRequest->controlSelector)
        {
            case AUDIO_V2_CX_CLOCK_SELECTOR_CONTROL:
            {
                if ((controlRequest->bmRequestType & 0x80) == 0)
                {
                    //A control write transfer received from Host. Now receive data from Host.
                    USB_DEVICE_ControlReceive(appData.usbDevHandle, (void *) &(appData.clockSelectIndex), 1 );
                    appData.currentAudioControl = APP_USB_CONTROL_NONE;
                }
                else
                {
                    /*Handle Get request*/
                    USB_DEVICE_ControlSend(appData.usbDevHandle, (void *)&(appData.clockSelectIndex), 1 );
                }
            }
            break;

            case AUDIO_V2_CS_CLOCK_VALID_CONTROL:
            {
                if ((controlRequest->bmRequestType & 0x80) == 0x80)
                {
                    /*Handle Get request*/
                    USB_DEVICE_ControlSend(appData.usbDevHandle, (void *)&(appData.clockValid), 1 );
                }
                else
                {
                     USB_DEVICE_ControlStatus( appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);

                }
            }
            break;

            default:
                //This USB Audio Speaker application does not support any other feature unit request
                // from Host. So Stall if any other feature unit request received from Host.
                USB_DEVICE_ControlStatus (appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            break;

        } // end of switch(featureRequest->controlSelector)
    }
    else
        USB_DEVICE_ControlStatus (appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);

    return (int) err;
} //end of function APP_ClockSelectRequestHandler

int APP_ClockSourceRequestHandler
(
    USB_DEVICE_AUDIO_V2_INDEX iAudio ,
    USB_AUDIO_V2_CONTROL_INTERFACE_REQUEST* controlRequest
)
{
    USB_AUDIO_V2_CLOCKSOURCE_CONTROL_REQUEST* clockSourceRequest;
    clockSourceRequest = (USB_AUDIO_V2_CLOCKSOURCE_CONTROL_REQUEST*) controlRequest;

    int8_t err = 0;

    if (clockSourceRequest->bRequest == AUDIO_V2_CUR)
    {
        switch(clockSourceRequest->controlSelector)
        {
            case AUDIO_V2_CS_SAM_FREQ_CONTROL:
            {
                if ((controlRequest->bmRequestType & 0x80) == 0)
                {
                    //A control write transfer received from Host. Now receive data from Host.
                    USB_DEVICE_ControlReceive(appData.usbDevHandle, (void *) &(appData.clockSource), 4 );
                    appData.currentAudioControl = APP_USB_AUDIO_CLOCKSOURCE_CONTROL;
                }
                else
                {
                    /*Handle Get request*/
                    USB_DEVICE_ControlSend(appData.usbDevHandle, (void *)&(appData.clockSource), 4 );
                    appData.currentAudioControl = APP_USB_CONTROL_NONE;
                }
            }
            break;

            case AUDIO_V2_CS_CLOCK_VALID_CONTROL:
            {
                if ((controlRequest->bmRequestType & 0x80) == 0x80)
                {
                    /*Handle Get request*/
                    USB_DEVICE_ControlSend(appData.usbDevHandle, (void *)&(appData.clockValid), 1 );
                }
                else
                {
                     USB_DEVICE_ControlStatus( appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);

                }
            }
            break;

            default:
                //This USB Audio Speaker application does not support any other feature unit request
                // from Host. So Stall if any other feature unit request received from Host.
                USB_DEVICE_ControlStatus (appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            break;

        } // end of switch(featureRequest->controlSelector)
    }
    else if (clockSourceRequest->bRequest == AUDIO_V2_RANGE)
    {
        switch(clockSourceRequest->controlSelector)
        {
            case AUDIO_V2_CS_SAM_FREQ_CONTROL:
            {
                if ((controlRequest->bmRequestType & 0x80) == 0x80)
                {
                    //A control read transfer received from Host. Now send data to Host.
                    //USB_DEVICE_ControlSend(appData.usbDevHandle, (void *) &(appData.clockSourceRange), sizeof(appData.clockSourceRange));
                    USB_DEVICE_ControlSend(appData.usbDevHandle, (void *) &(appData.clockSourceRange), clockSourceRequest->wLength);
                    
                }
                else
                {
                    /*Handle Get request*/
                    USB_DEVICE_ControlStatus( appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
                }
            }
            break;

            default:
                //This USB Audio Speaker application does not support any other feature unit request
                // from Host. So Stall if any other feature unit request received from Host.
                USB_DEVICE_ControlStatus (appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            break;

        } // end of switch(featureRequest->controlSelector)
    }

    return (int) err;
} //end of function APP_lockSourceRequestHandler

/* Feature Unit Request */
int APP_FeatureUnitRequestHandler
(
    USB_DEVICE_AUDIO_V2_INDEX iAudio ,
    USB_AUDIO_V2_CONTROL_INTERFACE_REQUEST* controlRequest
)
{
    USB_AUDIO_V2_FEATURE_CONTROL_REQUEST* featureRequest;
    featureRequest = (USB_AUDIO_V2_FEATURE_CONTROL_REQUEST*) controlRequest;

    int8_t err = 0;

    switch(featureRequest->controlSelector)
    {
        case USB_AUDIO_V2_FCS_MUTE_CONTROL:
        {
            if ((controlRequest->bmRequestType & 0x80) == 0)
            {
                //A control write transfer received from Host. Now receive data from Host.
                USB_DEVICE_ControlReceive(appData.usbDevHandle, (void *) &(appData.dacMute), 1 );
                appData.currentAudioControl = APP_USB_AUDIO_MUTE_CONTROL;
            }
           else
            {
                /*Handle Get request*/
                USB_DEVICE_ControlSend(appData.usbDevHandle, (void *)&(appData.dacMute), 1 );
            }
        }
        break;

        case USB_AUDIO_V2_FCS_VOLUME_CONTROL:
        case USB_AUDIO_V2_FCS_BASS_CONTROL:
        case USB_AUDIO_V2_FCS_MID_CONTROL:
        case USB_AUDIO_V2_FCS_TREBLE_CONTROL:
        case USB_AUDIO_V2_FCS_GRAPHIC_EQUALIZER_CONTROL:
        case USB_AUDIO_V2_FCS_AUTOMATIC_GAIN_CONTROL:
        case USB_AUDIO_V2_FCS_DELAY_CONTROL:
        case USB_AUDIO_V2_FCS_BASS_BOOST_CONTROL:
        case USB_AUDIO_V2_FCS_LOUDNESS_CONTROL :
        case USB_AUDIO_V2_FCS_FU_CONTROL_UNDEFINED:
        default:
            //This USB Audio Speaker application does not support any other feature unit request
            // from Host. So Stall if any other feature unit request received from Host.
            USB_DEVICE_ControlStatus (appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
        break;

    } // end of switch(featureRequest->controlSelector)
    return err;
} //end of function APP_FeatureUnitRequestHandler

int APP_AudioCurEntitySettingsHandler
(
    USB_DEVICE_AUDIO_V2_INDEX iAudio ,
    USB_AUDIO_V2_CONTROL_INTERFACE_REQUEST *setupPkt
)
{
    int8_t err=-1;
    USB_AUDIO_V2_CONTROL_INTERFACE_REQUEST* controlRequest;
    controlRequest = (USB_AUDIO_V2_CONTROL_INTERFACE_REQUEST*) setupPkt;
    switch(controlRequest->entityID)
    {
        case APP_ID_CLOCK_SOURCE:
            err = APP_ClockSourceRequestHandler (iAudio, controlRequest);
            break;
//        case APP_ID_CLOCK_SELECT:
//            err = APP_ClockSelectRequestHandler (iAudio, controlRequest);
//            break;
        case APP_ID_INPUT_TERMINAL:
        case APP_ID_OUTPUT_TERMINAL:
            /* check if input or output terminal */
            if(controlRequest->entityID == APP_ID_INPUT_TERMINAL)
            {
                /* we need to support get CUR request */
            }
            else if(controlRequest->entityID == APP_ID_OUTPUT_TERMINAL)
            {
                /*we need to support set CUR request */
            }
        break;

        case APP_ID_FEATURE_UNIT:
            err = APP_FeatureUnitRequestHandler (iAudio, controlRequest);
        break;
        case APP_ID_CLOCK_SELECT:        
        case APP_ID_MIXER_UNIT:
        case APP_ID_SELECTOR_UNIT:
        case APP_ID_PROCESSING_UNIT:
        case APP_ID_EXTENSION_UNIT:
        default:
            //This USB Audio Speaker application does not support any other control request
            // received for other unit from Host. So Stall if any other request received from Host.
            USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR );
        break;
    }//end of switch(controlRequest->entityID)
    return (int) err;
}//end of function APP_AudioEntitySettingsCallback

int APP_AudioRangeEntitySettingsHandler
(
    USB_DEVICE_AUDIO_V2_INDEX iAudio ,
    USB_AUDIO_V2_CONTROL_INTERFACE_REQUEST *setupPkt
)
{
    int8_t err=-1;
    USB_AUDIO_V2_CONTROL_INTERFACE_REQUEST* controlRequest;
    controlRequest = (USB_AUDIO_V2_CONTROL_INTERFACE_REQUEST*) setupPkt;
    switch(controlRequest->entityID)
    {
        case APP_ID_CLOCK_SOURCE:
            err = APP_ClockSourceRequestHandler (iAudio, controlRequest);
            break;
//        case APP_ID_CLOCK_SELECT:
//            err = APP_ClockSelectRequestHandler (iAudio, controlRequest);
//            break;
        case APP_ID_INPUT_TERMINAL:
        case APP_ID_OUTPUT_TERMINAL:
            /* check if input or output terminal */
            if(controlRequest->entityID == APP_ID_INPUT_TERMINAL)
            {
                /* we need to support get CUR request */
            }
            else if(controlRequest->entityID == APP_ID_OUTPUT_TERMINAL)
            {
                /*we need to support set CUR request */
            }
        break;

        case APP_ID_FEATURE_UNIT:
            err = APP_FeatureUnitRequestHandler (iAudio, controlRequest);
        break;
        case APP_ID_CLOCK_SELECT:        
        case APP_ID_MIXER_UNIT:
        case APP_ID_SELECTOR_UNIT:
        case APP_ID_PROCESSING_UNIT:
        case APP_ID_EXTENSION_UNIT:
        default:
            //This USB Audio Speaker application does not support any other control request
            // received for other unit from Host. So Stall if any other request received from Host.
            USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR );
        break;
    }//end of switch(controlRequest->entityID)
    return (int) err;
}//end of function APP_AudioEntitySettingsCallback

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

/******************************************************************************
  Function:
    void APP_usbDeviceEventCallBack(USB_DEVICE_EVENTS events)

  Remarks:
    See prototype in app.h.
*/

void APP_UsbDeviceEventCallBack( USB_DEVICE_EVENT event, void * pEventData, uintptr_t context )
{
    uint8_t * configuredEventData;
    switch( event )
    {
        case USB_DEVICE_EVENT_RESET:            
            // called when USB cable is plugged  in
            APP_LED1_OFF();
            APP_LED2_ON();
            APP_LED3_OFF();
            APP_LED4_ON();            
            APP_LED5_OFF(); 
            led1On = 0; led2On = 1; led3On = 0; led4On = 1; led1StuckOn = 0;
            usbReadError = 0;
            
            muteAudio();        // mutes audio during initial plugging in of USB cable
            
            break;
        case USB_DEVICE_EVENT_DECONFIGURED:
            // USB device is reset or device is de-configured.
            // This means that USB device layer is about to de-initialize
            // all function drivers. So close handles to previously opened
            // function drivers.            
            break;

        case USB_DEVICE_EVENT_CONFIGURED:
            /* check the configuration */
             /* Initialize the Application */
            configuredEventData = pEventData;
            if(*configuredEventData == 1)
            {
                USB_DEVICE_AUDIO_V2_EventHandlerSet
                (
                    0,
                    APP_USBDeviceAudioEventHandler ,
                    (uintptr_t)NULL
                );
                /* mark that set configuration is complete */
                appData.isConfigured = true;
                appData.display.status = APP_DISP_STATUS_USB_CONNECTED; 
                appData.display.update = true;             
                DRV_CODEC_MuteOff(appData.codecClient.handle);
            }
            break;

        case USB_DEVICE_EVENT_SUSPENDED:
                // called when USB cable is unplugged
                if(appData.isConfigured == true)
                {
                    appData.isConfigured = false;                    
                    appData.display.status = APP_DISP_STATUS_USB_DISCONNECTED; 
                    appData.display.update = true; 
                    
                    muteAudio();
                    
                    appData.state = APP_REINITIALIZE;                   
                }            
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:
            /* Attach the device */
            USB_DEVICE_Attach (appData.usbDevHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:
                /* VBUS is not available. We can detach the device */
                USB_DEVICE_Detach(appData.usbDevHandle);
                appData.isConfigured = false;
                //appData.state = APP_REINITIALIZE;
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}
void doErrorRecovery(uint8_t level)
{
    muteAudio();
    initData();
    if (level > LEVEL1_ERROR)
    {
        USB_DEVICE_Detach(appData.usbDevHandle);                        
        USB_DEVICE_Attach(appData.usbDevHandle);                        
        appData.isConfigured = false;                        
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
    }
    else
    {
        appData.state = APP_SUBMIT_READ_REQUEST;
    }
}

void muteAudio ( void )
{
    //appData.codecClient.isMute = true;                       
    DRV_CODEC_MuteOn(appData.codecClient.handle);
    BSP_AK4201_AMPLIFIER_PDNOff();  // turn off amplifier
}
/*****************************************************
 * This function is called from the DMA Channel 0 interrupt handler in system_interrupt.c
 * to synchronize up the I2S LRCLK when the audio stream is restarted.  Otherwise, the left/right
 * channels may become swapped.
 *****************************************************/
/******************************************************************************
  Function:
    void syncLRClock( void )
 */

void syncLRClock( void )
{    
    if (LRClockSync>0)
    {       
        if (LRClockSync!=1)
        {
            APP_LED4_OFF();
            APP_LED4_ON();
            led4On = 1;
        }
        else    // = 1, therefore will be decremented to 0 before leaving function
        {
            APP_LED4_OFF();
            led4On = 0;
            
            PLIB_SPI_Disable(DRV_I2S_PERIPHERAL_ID_IDX0);       // turn off SPI interface
            if (appData.codecClient.samplingRate <= APP_USB_AUDIO_SAMPLING_RATE_48KHZ)
            {
                PLIB_SPI_FrameSyncPulsePolaritySelect(DRV_I2S_PERIPHERAL_ID_IDX0, 1);   // invert polarity, otherwise L/R are always backwards           
            }
            else
            {
                PLIB_SPI_FrameSyncPulsePolaritySelect(DRV_I2S_PERIPHERAL_ID_IDX0, 0);   // regular polarity for higher bitrates
            }
            PLIB_SPI_Enable(DRV_I2S_PERIPHERAL_ID_IDX0);        // turn SPI interface back on again (restarts LRCLK)           
            
            DRV_CODEC_MuteOff(appData.codecClient.handle);      // if muted, unmuted
            BSP_AK4201_AMPLIFIER_PDNOn();      // if amp was powred down, power it back up again
        }

        LRClockSync--;          // seems to work better if we do this more than once, # is defined as LRCLOCKSYNC
    }
}

/*****************************************************
* This function is called in from the SYS_Initialize function.
 *****************************************************/
/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{  
    return;
}

/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/
/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
void initData ( void )
{
    APP_LED1_OFF();
    APP_LED2_OFF();
    APP_LED3_OFF();
    APP_LED4_OFF();
    led1On = 0; led2On = 0; led3On = 0; led4On = 0;
    blinkSwitch = 0;
    
    someVar=0;
    usbSkip=0; 
    usbReadComplete=0;
    codecSkip=0; 
    codecWriteComplete=0;
    codecWriteCount = 0;
    bufferWriteError = 0;
    usbReadCount = 0;
    justOnce = false;
    bufferCount=0;
    appData.isWriteComplete = true;
    usbNextBuffer = 0; 
    codecNextBuffer=0;           
    //usbReadError = 0;
    usbOverLap = 0;
    
    LRClockSync = LRCLOCKSYNC;

    led2StuckOnMax = 0;
    led3StuckOffMax = 0;    
    led3StuckOnMax = 0;
    led3StuckOffMax = 0;
    led5StuckOnMax = 0;
    led5StuckOffMax = 0;
    
    /* Initializing appData members*/
    for (loopCount = 0; loopCount < APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT; loopCount++)
    {
        appData.codecClient.bufferSize[loopCount] = AUDIO_FRAME_SIZE_UAC2;               
        appData.codecClient.writeBufHandle[loopCount] = DRV_CODEC_BUFFER_HANDLE_INVALID;
        appData.usbReadQueue.dataAvailable[loopCount] = false;
        appData.usbReadQueue.readTransferHandle[loopCount] = USB_DEVICE_AUDIO_V2_TRANSFER_HANDLE_INVALID;
        appData.CodecWriteComplete[loopCount] = false;
        appData.usbReadComplete[loopCount] = false;
    }
}

void APP_TasksCheckStuck()
{
    // here we check for various conditions which represent potentials for the playback to hang
    // if met, we kickstart the audio audio to resubmit a read request via USB
    // normally during playback, LED 1 is off, and LEDs 2 and/or 3 are flashing indicating packets are being read
    // led4 on means we are paused
    if ((appData.codecClient.isMute==0) && (led1On!=0))               // stuck high is an error
    {
        led1StuckOn++;
        if (led1StuckOn > 20)
        {
            initData();
            muteAudio();
            appData.state = APP_SUBMIT_READ_REQUEST;
            
            led1StuckOn = 0;
        }
    }
    else if (led1On==0)
    {
        led1StuckOn = 0;
    }    

    if ((APP_USB_AUDIO_SAMPLING_RATE_48KHZ == appData.codecClient.samplingRate)||
        (APP_USB_AUDIO_SAMPLING_RATE_96KHZ == appData.codecClient.samplingRate)||
        (APP_USB_AUDIO_SAMPLING_RATE_192KHZ == appData.codecClient.samplingRate))            
    { 
        // for these three (48, 96 and 192 kHz), normal pattern is LED2 always off and LED3 always on
        // so error if LED2 on, or LED3 off for more than certain minimum
        if ((appData.codecClient.isMute==0) && (led2On!=0))           // stuck high is an error
        {
            led2StuckOn++;
            if (led2StuckOn > led2StuckOnMax)
            {
                led2StuckOnMax = led2StuckOn;    
            }             
            if (led2StuckOn > 15)
            {
                doErrorRecovery(LEVEL1_ERROR);

                led2StuckOn = 0;
                led2StuckOnMax = 0;
            }
        }
        else if (led2On==0)
        {
            led2StuckOn = 0;
        } 
        
        if ((appData.codecClient.isMute==0) && (led3On==0))          // stuck low is an error
        {
            led3StuckOff++;
            if (led3StuckOff > led3StuckOffMax)
            {
                led3StuckOffMax = led3StuckOff;    
            }               
            if (led3StuckOff > 15) 
            {
                doErrorRecovery(LEVEL1_ERROR);

                led3StuckOff = 0;      
                led3StuckOffMax = 0;                
            }
        }   
        else if (led3On!=0)
        {
            led3StuckOff = 0;
        }        
    }
    
    if ((APP_USB_AUDIO_SAMPLING_RATE_32KHZ == appData.codecClient.samplingRate)||
        (APP_USB_AUDIO_SAMPLING_RATE_44_1KHZ == appData.codecClient.samplingRate)||
        (APP_USB_AUDIO_SAMPLING_RATE_88_2KHZ == appData.codecClient.samplingRate)||            
        (APP_USB_AUDIO_SAMPLING_RATE_176_4KHZ == appData.codecClient.samplingRate))            
    {
        // for these four (32, 44.1, 88.2 and 176.4 kHz), normal pattern is LED2 to be normally high and pulsing low,
        // and LED3 to be normally low and pulsing high       
        if ((appData.codecClient.isMute==0) && (led2On==0))          // stuck low is an error
        {      
            led2StuckOff++;
            if (led2StuckOff > led2StuckOffMax)
            {
                led2StuckOffMax = led2StuckOff;    
            }             
            if (led2StuckOff > 20)      // highest measured was 1
            {
                doErrorRecovery(LEVEL1_ERROR);
                
                led2StuckOffMax = 0;
                led2StuckOff = 0;                
            }
        }  
        else if (led2On!=0)
        {
            led2StuckOff = 0;
        }
        
        if ((appData.codecClient.isMute==0) && (led3On!=0))          // stuck high is an error
        {      
            led3StuckOn++;
            if (led3StuckOn > led3StuckOnMax)
            {
                led3StuckOnMax = led3StuckOn;    
            }            
            if (led3StuckOn > 250)      // highest measured was 150
            {
                doErrorRecovery(LEVEL1_ERROR);

                led3StuckOn = 0;      
                led3StuckOnMax = 0;                   
            }
        }
        else if (led3On==0)
        {
            led3StuckOn = 0;
        }

       if ((appData.codecClient.isMute==0) && (led3On==0))          // stuck low is an error
        {
            led3StuckOff++;
            if (led3StuckOff > led3StuckOffMax)
            {
                led3StuckOffMax = led3StuckOff;    
            }               
            if (led3StuckOff > 20) 
            {
                doErrorRecovery(LEVEL1_ERROR);

                led3StuckOff = 0;      
                led3StuckOffMax = 0;                
            }
        }   
        else if (led3On!=0)
        {
            led3StuckOff = 0;
        }                
    }  
 }
void APP_TasksCheckUSBStuck()
{
   if ((appData.codecClient.isMute==0) && (led5On!=0))
    {
        led5StuckOff = 0;
        led5StuckOn++;
        if (led5StuckOn > led5StuckOnMax)
        {
            led5StuckOnMax = led5StuckOn;    
        }
        if (led5StuckOn>250)        // highest measured was 130
        {
            doErrorRecovery(LEVEL1_ERROR);

            led5StuckOn = 0;
            led5StuckOnMax = 0;
        }
    }
    else if ((appData.codecClient.isMute==0) && (led5On==0))
    {
        led5StuckOn = 0;
        led5StuckOff++;
        if (led5StuckOff > led5StuckOffMax)
        {
            led5StuckOffMax = led5StuckOff;    
        }        
        if (led5StuckOff>250)       // highest measured was 155
        {
            doErrorRecovery(LEVEL1_ERROR);

            led5StuckOff = 0;
            led5StuckOffMax = 0;
        }
    }  
    else if (appData.codecClient.isMute==1)
    {
        led5StuckOn = 0;
        led5StuckOff = 0;
    }
}

void APP_Tasks ( void )
{
    bool intStatus = false;
    uint32_t usbReadCompleteStatus;
    
    blinkTimer++;
    
    if (blinkTimer & 0x8000)
    {
#ifdef  LED5_TASK           // enable to blink LED5 every 32000 times this function is entered,
                            // blinks 30 times in 19 seconds (while playing music at 44.1 kHz)
                            // 60/19 = 3.15 * 32000 = 101052 calls/sec, or every 9.85 µS
        APP_LED5_ON();      
#endif        
        if (blinkSwitch==0)
        {
            blinkSwitch = 1;   //lockout
        }
    }
    else
    {
#ifdef  LED5_TASK
        APP_LED5_OFF();
#endif        
        if (blinkSwitch==1)
        {
            blinkSwitch = 0;   //lockout         
        }               
    }
    
    if (appData.isConfigured)
    {
        if (blinkTimer & 0x8000)                
        {                               // for now, this is executed every 317 ms        
            if (checkSwitch==0)
            {
                checkSwitch = 1;   //lockout
                //APP_TasksCheckStuck();           
            }
        }
        else
        {       
            if (checkSwitch==1)
            {
                checkSwitch = 0;   //lockout
                //APP_TasksCheckStuck();            
            }               
        }

        if (blinkTimer & 0x0080)
        {                              // for now, this is executed every 1.26 ms    
            if (checkUSBswitch==0)
            {
                checkUSBswitch = 1;   //lockout
                //APP_TasksCheckUSBStuck();
            }
        }
        else
        {       
            if (checkUSBswitch==1)
            {
                checkUSBswitch = 0;   //lockout
                //APP_TasksCheckUSBStuck();            
            }               
        }
    }
    
    switch(appData.state)
    {
        case APP_STATE_INIT:           

            initData();

            /* Open the device layer */
            appData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,
                    DRV_IO_INTENT_READWRITE );

            if(appData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.usbDevHandle, APP_UsbDeviceEventCallBack, 0);

                /* Attach the device */
                USB_DEVICE_Attach (appData.usbDevHandle);

                appData.state = INITIALIZE_AUDIO_CODEC;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:
            //Check if Host has configured the Device.
            if (appData.isConfigured == true)
            {
                ;
            }
            break;

        case INITIALIZE_AUDIO_CODEC:

            appData.codecClient.handle = DRV_CODEC_Open(
                                                  DRV_CODEC_INDEX_0, 
                                                  DRV_IO_INTENT_EXCLUSIVE);

            if(appData.codecClient.handle != DRV_HANDLE_INVALID)
            {
                appData.codecClient.volume = 
                        DRV_CODEC_VolumeGet(appData.codecClient.handle, 
                                            DRV_CODEC_CHANNEL_LEFT_RIGHT);

                appData.display.volumeP = 
                        (100*appData.codecClient.volume)/APP_VOLUME_MAX_VALUE;            

                sprintf(appData.display.volumePercent,
                        "%d%%",
                        appData.display.volumeP);                

                appData.state = APP_CODEC_BUFFER_HANDLER_SET;
            }
            else
            {
                /* Got an Invalid Handle.  Wait for AK4384 to Initialize */
                ;
            }
            
        break;

        case APP_CODEC_BUFFER_HANDLER_SET:
         
            DRV_CODEC_BufferEventHandlerSet(
                    appData.codecClient.handle,
                    appData.codecClient.bufferHandler,
                    appData.codecClient.context);

            appData.codecClient.isInitialized = true;        
            appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            //APP_GFX_MENU_DRAW();            
            break;

        case APP_SUBMIT_READ_REQUEST:
            if (justOnce == false)
            {       
                DRV_CODEC_MuteOn(appData.codecClient.handle);
                justOnce = true;
                for (usbNextBuffer=0; 
                     usbNextBuffer < (APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT-4); 
                     usbNextBuffer++)
                {
                    audioErr = USB_DEVICE_AUDIO_V2_Read(
                                        USB_DEVICE_INDEX_0, 
                                        &appData.usbReadQueue.
                                        readTransferHandle[usbNextBuffer], 
                                        1, 
                                        &appData.usbReadQueue.
                                             rxBuffer[usbNextBuffer], 
                                        1024);
#ifdef LED5_USB                            
                    APP_LED5_OFF();
#endif
                    led5On = 0;
                    if (audioErr != USB_DEVICE_AUDIO_V2_RESULT_OK)
                    {
                        usbReadError++;
                        if (usbReadError > MAX_BAD_PACKETS)
                        {
                            doErrorRecovery(LEVEL2_ERROR);
                            usbReadError = 0;
                            break;
                        }                      
                    }                    
                    //appData.usbReadQueue.dataAvailable[usbNextBuffer] = false;
                }                                     
                txBuffer[0] =  appData.usbFeedbackNormalValue[0];
                txBuffer[1] =  appData.usbFeedbackNormalValue[1]; 
                txBuffer[2] =  appData.usbFeedbackNormalValue[2]; 
                txBuffer[3] =  appData.usbFeedbackNormalValue[3];                                                                        
            }

            if (appData.isWriteComplete == true)
            {                      
                appData.isWriteComplete = false;
                audioErr = USB_DEVICE_AUDIO_V2_Write ( USB_DEVICE_INDEX_0 , &appData.writeTransferHandle, 1 , txBuffer, 4 );
            }

            intStatus = SYS_INT_Disable();
            {
                usbReadCompleteStatus  = usbReadComplete;
            }
            if(intStatus) SYS_INT_Enable();
            
            if(usbReadCompleteStatus >= (APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT-8))
            {
                DRV_CODEC_MuteOff(appData.codecClient.handle);
                for (loopCount=0; loopCount < (APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT-16); loopCount++)
                {
                    if(appData.usbReadQueue.dataAvailable[loopCount] == true)
                    {
                        DRV_CODEC_BufferAddWrite(appData.codecClient.handle, &appData.codecClient.writeBufHandle[loopCount],
                        &appData.usbReadQueue.rxBuffer[loopCount], appData.codecClient.bufferSize[loopCount]);
                        if(appData.codecClient.writeBufHandle[loopCount] == DRV_CODEC_BUFFER_HANDLE_INVALID)
                        {
                            bufferWriteError++;
                        }
                        else
                        {                                                   
                            codecNextBuffer++;
                        }
                    }
                }                
                appData.state = APP_PROCESS_DATA;
            }
         
        break;

        case APP_PROCESS_DATA:
            {
                bufferCount = DRV_CODEC_BufferCombinedQueueSizeGet(appData.codecClient.handle);
                if ( bufferCount >= (appData.usbAudioBufferUpperLimit*appData.maxAudioSamples))                
                {
                    APP_LED1_ON();                     
                    APP_LED2_OFF();
                    APP_LED3_OFF(); 
                    led1On = 1; led2On = 0; led3On = 0;
                    led1StuckOn = 1;
                    /*Slow down the host data rate*/                    
                    txBuffer[0] = appData.usbFeedbackSlowDownValue[0]; 
                    txBuffer[1] = appData.usbFeedbackSlowDownValue[1]; 
                    txBuffer[2] = appData.usbFeedbackSlowDownValue[2];
                    txBuffer[3] = appData.usbFeedbackSlowDownValue[3];                    
                }                
                else if ( bufferCount < appData.usbAudioBufferLowerLimit*appData.maxAudioSamples)                
                {
                    APP_LED1_OFF();
                    APP_LED2_ON();
                    APP_LED3_OFF();                    
                    led1On = 1; led2On = 1; led3On = 0;
                    led1StuckOn = 0;
                    led2StuckOff = 0;                    
                    /*Speed up the host data rate*/
                    txBuffer[0] = appData.usbFeedbackSpeedUpValue[0];  
                    txBuffer[1] = appData.usbFeedbackSpeedUpValue[1];
                    txBuffer[2] = appData.usbFeedbackSpeedUpValue[2];
                    txBuffer[3] = appData.usbFeedbackSpeedUpValue[3];                      
                }
                else
                {
                    APP_LED1_OFF();
                    APP_LED2_OFF();
                    APP_LED3_ON();
                    led1On = 1; led2On = 0; led3On = 1;
                    led1StuckOn = 0;
                    led3StuckOff = 0;
                    txBuffer[0] =  appData.usbFeedbackNormalValue[0];
                    txBuffer[1] =  appData.usbFeedbackNormalValue[1]; 
                    txBuffer[2] =  appData.usbFeedbackNormalValue[2]; 
                    txBuffer[3] =  appData.usbFeedbackNormalValue[3];                                       
                }
                if(appData.usbReadQueue.dataAvailable[codecNextBuffer] == true)
                {
                    DRV_CODEC_BufferAddWrite(
                            appData.codecClient.handle, 
                            &appData.codecClient.
                            writeBufHandle[codecNextBuffer],
                            &appData.usbReadQueue.rxBuffer[codecNextBuffer], 
                            appData.codecClient.bufferSize[codecNextBuffer]);

                    if(appData.codecClient.writeBufHandle[codecNextBuffer] == DRV_CODEC_BUFFER_HANDLE_INVALID)
                    {
                        bufferWriteError++;
                    }
                    else
                    {                       
                        codecNextBuffer+=1;
                        codecNextBuffer = (codecNextBuffer == APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT)?0:codecNextBuffer;
                    }
                }
                else
                {
                    codecSkip++;
                }
   
                if(usbNextBuffer == codecNextBuffer)
                {
                    if(bufferCount == 0)
                    {
                        usbOverLap++;         
                        /* Initializing appData members*/
                        initData();
                    }
                }
                
                if(appData.usbReadQueue.dataAvailable[usbNextBuffer] == false)
                {
                    audioErr = USB_DEVICE_AUDIO_V2_Read(
                                   USB_DEVICE_INDEX_0, 
                                   &appData.usbReadQueue.
                                       readTransferHandle[usbNextBuffer], 
                                   1, 
                                   &appData.usbReadQueue.rxBuffer[usbNextBuffer], 
                                   1024);

                    APP_LED5_OFF();
                    led5On = 0;
                    if (audioErr == USB_DEVICE_AUDIO_V2_RESULT_OK)
                    {
                        usbNextBuffer+=1;
                        usbNextBuffer = 
                                (usbNextBuffer == 
                                 APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT)? 
                                 0:usbNextBuffer;
                    }
                    else
                    {
                        usbReadError++;
                        if (usbReadError > MAX_BAD_PACKETS)
                        {
                            doErrorRecovery(LEVEL1_ERROR);
                            usbReadError = 0;
                        }
                        break;                        
                    }
                }
                else
                {
                    usbSkip++;
                }

                if (appData.isWriteComplete == true)
                {                        
                    appData.isWriteComplete = false;
                    audioErr = USB_DEVICE_AUDIO_V2_Write(USB_DEVICE_INDEX_0, 
                                   &appData.writeTransferHandle, 
                                   1, 
                                   txBuffer, 
                                   4);
                }
                
            }
        break;

        case APP_REINITIALIZE:    
        {
            memset(&appData.usbReadQueue.rxBuffer, 
                   0, 
                   sizeof(appData.usbReadQueue.rxBuffer));

            memset(&txBuffer, 0, sizeof (txBuffer));            
            DRV_AK4384_BufferQueueFlush(appData.codecClient.handle);
            bufferCount = 0;
            usbSkip = 0;
            codecSkip = 0;
            codecWriteComplete = 0;
            codecWriteCount = 0;
            bufferWriteError = 0;
            usbReadCount = 0;
            appData.isWriteComplete = true;
            usbReadComplete = 0;
            codecNextBuffer = 0;
            usbNextBuffer = 0;
            justOnce = false;
            
            usbReadError = 0;
            usbOverLap = 0;
            
            LRClockSync = LRCLOCKSYNC;

            for (loopCount = 0; 
                 loopCount < APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT; 
                 loopCount++) 
            {
                appData.codecClient.bufferSize[loopCount] = appData.usbAudioFrameSizeUa2;
                appData.codecClient.writeBufHandle[loopCount] = DRV_CODEC_BUFFER_HANDLE_INVALID;
                appData.usbReadQueue.dataAvailable[loopCount] = false;
                appData.usbReadQueue.readTransferHandle[loopCount] = USB_DEVICE_AUDIO_V2_TRANSFER_HANDLE_INVALID;
                appData.CodecWriteComplete[loopCount] = false;
                appData.usbReadComplete[loopCount] = false;
            }
            APP_LED1_OFF();
            APP_LED2_OFF();
            APP_LED3_OFF();
            led1On = 0; led2On = 0; led3On = 0;
            //APP_LED4_OFF();
            appData.state = APP_IDLE;          
        }   
        break;
        
        case APP_MUTE_AUDIO_PLAYBACK:
            if (appData.activeInterfaceAlternateSetting == 
                APP_USB_SPEAKER_PLAYBACK_NONE) 
            {
                appData.state = APP_IDLE;
            } 
            else if (appData.activeInterfaceAlternateSetting == 
                     APP_USB_SPEAKER_PLAYBACK_STEREO) 
            {
                appData.state = APP_DAC_MUTE;
            }                 
        break;

        case APP_CLOCKSOURCE_SET:
            if (appData.activeInterfaceAlternateSetting == 
                APP_USB_SPEAKER_PLAYBACK_NONE)
            {
                DRV_CODEC_SamplingRateSet(appData.codecClient.handle, 
                                          appData.clockSource);                

                if(appData.lastClockSource!= appData.clockSource)
                {
                    appData.lastClockSource = appData.clockSource;  // changed
                    // these two lines commented out because we assume for now user has to pause the audio before changing sample rates
                    //muteAudio();
                    //LRClockSync = LRCLOCKSYNC;      // reset LRCLK
                }
            }

            appData.codecClient.samplingRate = appData.clockSource;

            sprintf(appData.display.samplingRate,
                    "%d",
                    appData.codecClient.samplingRate);            

            appData.display.status = APP_DISP_STATUS_SAMPLING_RATE;
            appData.display.update = true;                                             
            appData.state = APP_IDLE;            
           
            break;

        case APP_CLOCKSELECT_SET:
            appData.state = APP_IDLE;
            break;

        case APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD:
            if (appData.activeInterfaceAlternateSetting == 
                APP_USB_SPEAKER_PLAYBACK_NONE)
            {        

                memset(&appData.usbReadQueue.rxBuffer,
                       0,
                       sizeof(appData.usbReadQueue.rxBuffer));  

                memset(&txBuffer, 0, sizeof (txBuffer));                
                DRV_AK4384_BufferQueueFlush(appData.codecClient.handle);                     
                bufferCount = 0;
                usbSkip = 0; 
                codecSkip = 0; 
                codecWriteComplete = 0;
                codecWriteCount = 0;
                bufferWriteError = 0;
                usbReadCount = 0;
                appData.isWriteComplete = true;
                usbReadComplete = 0;
                codecNextBuffer = 0; 
                usbNextBuffer = 0;
                justOnce = false;
                         
                usbReadError = 0;
                usbOverLap = 0;

                for (loopCount = 0; 
                     loopCount < APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT; 
                     loopCount++)
                {
                    appData.codecClient.bufferSize[loopCount] = appData.usbAudioFrameSizeUa2;                          
                    appData.codecClient.writeBufHandle[loopCount] = DRV_CODEC_BUFFER_HANDLE_INVALID;
                    appData.usbReadQueue.dataAvailable[loopCount] = false;
                    appData.usbReadQueue.readTransferHandle[loopCount] = USB_DEVICE_AUDIO_V2_TRANSFER_HANDLE_INVALID;
                    appData.CodecWriteComplete[loopCount] = false;
                    appData.usbReadComplete[loopCount] = false;
                }                                                                               
                APP_LED1_OFF();
                APP_LED2_OFF();
                APP_LED3_OFF();
                led1On = 0; led2On = 0; led3On = 0;
                //APP_LED4_OFF();               
                appData.state = APP_IDLE;
            }
            else if(appData.activeInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_STEREO)
            {                 
               DRV_CODEC_MuteOff(appData.codecClient.handle);
               appData.state =  APP_SUBMIT_READ_REQUEST;
            }
        break;
    
        case APP_IDLE:           
        break;             

        case APP_DAC_MUTE:
            if(appData.isConfigured == true) 
            {
                if (appData.codecClient.isMute == false) 
                {
                    appData.codecClient.isMute = true;
                    appData.display.status = APP_DISP_STATUS_APP_MUTE_ON; 
                    appData.display.update = true;                        

                    muteAudio();                                      
                    initData();
                    appData.state = APP_IDLE;                                                         
                } 
                else 
                {
                    appData.codecClient.isMute = false;        
                    appData.display.status = APP_DISP_STATUS_MUTE_OFF; 
                    appData.display.update = true; 
                    LRClockSync = LRCLOCKSYNC;
                }
                if ((appData.activeInterfaceAlternateSetting == 
                    APP_USB_SPEAKER_PLAYBACK_NONE)||
                    (appData.codecClient.isMute == true))
                {
                    appData.state = APP_IDLE;
                }
                else
                {
                    appData.state = APP_PROCESS_DATA;                     
                }              
            }   
            else
            {
                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }

        break;

        case APP_DAC_VOLUME_INCREASE:
            if(appData.isConfigured == true)   
            { 
                if(appData.codecClient.volume == APP_VOLUME_MAX_VALUE)
                {
                    ;
                }
                else
                {
                    appData.codecClient.volume += APP_VOLUME_STEP_VALUE;
                    if(appData.codecClient.volume < APP_VOLUME_MAX_VALUE)
                    {
                        DRV_CODEC_VolumeSet(appData.codecClient.handle, 
                                            DRV_CODEC_CHANNEL_LEFT_RIGHT, 
                                            appData.codecClient.volume);                  
                    }
                    else
                    {
                        appData.codecClient.volume = APP_VOLUME_MAX_VALUE;
                        DRV_CODEC_VolumeSet(appData.codecClient.handle, 
                                            DRV_CODEC_CHANNEL_LEFT_RIGHT, 
                                            appData.codecClient.volume);                  
                    }                  
                    appData.display.volumeP = (100*appData.codecClient.volume)/APP_VOLUME_MAX_VALUE;            
                    sprintf(appData.display.volumePercent,"%d%%",appData.display.volumeP);
                    appData.display.status = APP_DISP_STATUS_VOLUME_INCREASE;
                    appData.display.update = true;                
                }
                if ((appData.activeInterfaceAlternateSetting == APP_USB_SPEAKER_PLAYBACK_NONE)||
                    (appData.codecClient.isMute == true))
                {
                    appData.state = APP_IDLE;
                }
                else
                {
                    appData.state = APP_PROCESS_DATA;                     
                }               
            }      
            else
            {
                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }            
        break;

        case APP_DAC_VOLUME_DECREASE:
            if(appData.isConfigured == true)            
            {
                if(appData.codecClient.volume == APP_VOLUME_MIN_VALUE)
                {
                    ;
                }
                else
                {
                    appData.codecClient.volume -= APP_VOLUME_STEP_VALUE;
                    if(appData.codecClient.volume > APP_VOLUME_MIN_VALUE)
                    {
                        DRV_CODEC_VolumeSet(appData.codecClient.handle, 
                                            DRV_CODEC_CHANNEL_LEFT_RIGHT, 
                                            appData.codecClient.volume);                  
                    }
                    else
                    {
                        appData.codecClient.volume = APP_VOLUME_MIN_VALUE;
                        DRV_CODEC_VolumeSet(appData.codecClient.handle, 
                                            DRV_CODEC_CHANNEL_LEFT_RIGHT, 
                                            appData.codecClient.volume);                  
                    }
                    appData.display.volumeP = (100*appData.codecClient.volume)/APP_VOLUME_MAX_VALUE;            
                    sprintf(appData.display.volumePercent,"%d%%",appData.display.volumeP);
                    appData.display.status = APP_DISP_STATUS_VOLUME_DECREASE;
                    appData.display.update = true;      
                }
                if ((appData.activeInterfaceAlternateSetting == APP_USB_SPEAKER_PLAYBACK_NONE)||
                    (appData.codecClient.isMute == true))
                {
                    appData.state = APP_IDLE;
                }
                else
                {
                    appData.state = APP_PROCESS_DATA;                     
                }                               
            }    
            else
            {
                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }                
        break;        
               
        case APP_STATE_ERROR:
        break;
        default:
        break;
    }// end of switch(appData.state)
       
    if(appData.display.update == true)
    {
        APP_DISPLAYTASK();
   }
    
    APP_ButtonTask();
    
} //End of APP_Tasks

/**********************************************************
 * Application AK4384 buffer Event handler.
 * This function is called back by the AK4384 driver when
 * a AK4384 data buffer TX completes.
 ***********************************************************/
void APP_CodecBufferEventHandler(DRV_CODEC_BUFFER_EVENT event,
                                 DRV_CODEC_BUFFER_HANDLE handle, 
                                 uintptr_t context )
{
    /* Transmission has completed. OFF the LEDs */
    switch(event)
    {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        {
            for (writeQueueCount=0; 
                 writeQueueCount<APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT; 
                 writeQueueCount++)
            {
                if (handle == 
                    appData.codecClient.writeBufHandle[writeQueueCount])
                {
                    appData.usbReadQueue.dataAvailable[writeQueueCount] = false;
                    usbReadComplete--;
                }
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

uint32_t __attribute__((nomips16)) APP_ReadCoreTimer(void)
{
    uint32_t timer;
    // get the count reg
    asm volatile("mfc0   %0, $9" : "=r"(timer));
    return (timer);
}

/*******************************************************************************
 End of File
 */

