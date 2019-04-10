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

APP_DATA appData __attribute__((coherent)) __attribute__((aligned(16))) ;

/* Specify the Audio Stream format details that this application supports */
const APP_USB_HOST_AUDIO_STREAM_FORTMAT audioSpeakerStreamFormat = 
{
    
    .streamDirection = USB_HOST_AUDIO_V1_DIRECTION_OUT,
    .format = USB_AUDIO_FORMAT_PCM,
    .nChannels = 2,
    .bitResolution = 16,
    .subFrameSize = 2,
    .samplingRate = 48000
}; 

/* PCM16 samples for 1Khz Sine Wave at 48Khz Sample Rate */
uint16_t audioSamples[96] =  {
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

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state =  APP_STATE_BUS_ENABLE;
    APP_AudioDataSetDefault();
    appData.muteStatus = 0;
}

void APP_AudioDataSetDefault (void)
{
    appData.isAudioDeviceAttached = false;
    appData.isAudioOutStreamFound = false; 
    appData.isAudioWriteCompleted = false;
    appData.isStreamEnabled = false;
    appData.isStreamInterfaceSetZeroComplete = false;
    appData.isStreamInterfaceSetOneComplete = false;
    appData.requestHandle = USB_HOST_AUDIO_V1_REQUEST_HANDLE_INVALID;
    appData.transferHandleAudioWrite = USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE_INVALID;
    appData.isMasterMuteSetComplete = false; 
    appData.isMasterUnmuteSetComplete = false; 
    appData.isMasterVolumeSetComplete = false;
    appData.isLeftVolumeSetComplete = false;
    appData.isRightVolumeSetComplete = false;
    appData.isMuteSwitchPressed = false; 
    appData.ignoreSwitchPress = true;
    appData.sofEventHasOccurred = false; 
    
    /* Set following to true until we start parsing descriptors */
    appData.isMasterMuteSupported = true; 
    appData.isMasterVolumeSupported = false;
    appData.isLeftChannelPresent = true;
    appData.isRightChannelPresent = true;
    appData.isLeftMuteSupported = true;
    appData.isRightMuteSupported = true;
    appData.isLeftVolumeSupported = true;
    appData.isRightVolumeSupported = true;
    
    
}
/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{
    USB_HOST_AUDIO_V1_RESULT audioResult;
    static uint32_t countLedBlink = 0; 
    uint32_t interruptStatus;
    bool status = false; 
  
    APP_ProcessSwitchPress(); 
    
    /* If USB Device was detached, change application state accordingly */
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
            &appData.requestHandle,
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
            &appData.requestHandle,
            0,
            (bool *)&appData.muteStatus
        );
    }
    /* Check the application's current state. */
    switch ( appData.state )
    {
        case APP_STATE_BUS_ENABLE:
            
            /* Register a callback for Audio Device Attach. */
            audioResult = USB_HOST_AUDIO_V1_AttachEventHandlerSet
                          (
                              &App_USBHostAudioV1AttachHandler,
                              0
                          ); 
            
            if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS )
            {
                /* Set Host Event Handler */
                USB_HOST_EventHandlerSet(APP_USBHostEventHandler, 0);
                USB_HOST_BusEnable(0);
                /* Advance application state */
                appData.state = APP_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE; 
            }
            break; 
            
        case APP_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE:
            if(USB_HOST_BusIsEnabled(0))
            {
                appData.state = APP_STATE_WAIT_FOR_DEVICE_ATTACH;
            }
            break; 
			
        case APP_STATE_WAIT_FOR_DEVICE_ATTACH:
            /* Check if an Audio Device has been attached  */
            if(appData.isAudioDeviceAttached != true)
            {
                break; 
            }
            /* Register a Call back with Audio Object for Entity Control 
                 Transfer Receive Complete event */ 
            USB_HOST_AUDIO_V1_EntityRequestCallbackSet
            (
                appData.audioDeviceObj,  
                App_USBAudioControlRequestCallback, 
                0      
            ); 
                
            /* Find an Audio Stream matching to our requirement */
            status = App_USBHostAudioSpeakerStreamFind
                                (
                                  appData.audioDeviceObj, 
                                  audioSpeakerStreamFormat,
                                  &appData.outStreamObj,
                                  &appData.speakerStreamInterfaceSettingObj,
                                  &appData.speakerStreamInterfaceSettingObjZeroBandwidth
                                );
            if (status == false)
            {
                appData.state = APP_STATE_ERROR; 
                break;
            }
                
            /* Now that we have found an audio stream, Get the ID of the terminal 
             * to which this Audio Streaming Interface is connected */ 
            appData.speakerTerminalLink = USB_HOST_AUDIO_V1_StreamingInterfaceTerminalLinkGet
                                          (
                                              appData.audioDeviceObj,
                                              appData.outStreamObj,
                                              appData.speakerStreamInterfaceSettingObj
                                          );

            /* Get the Feature Unit Object */ 
            status = APP_SpeakerFeatureUnitObjectGet(appData.speakerTerminalLink, &appData.speakerFeatureUnitObj); 
            if (status == false)
            {
                appData.state = APP_STATE_ERROR; 
            }
                
            /* Check if Device supports Mute Control on different channels */ 
            appData.isMasterMuteSupported = USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists
                                            (
                                                appData.audioDeviceObj,
                                                appData.speakerFeatureUnitObj,
                                                0
                                            ); 

            appData.isLeftMuteSupported = USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists
                                           (
                                               appData.audioDeviceObj,
                                               appData.speakerFeatureUnitObj,
                                               1
                                           ); 

            appData.isRightMuteSupported = USB_HOST_AUDIO_V1_FeatureUnitChannelMuteExists
                                           (
                                               appData.audioDeviceObj,
                                               appData.speakerFeatureUnitObj,
                                               2
                                           );   

            /* Open Audio Stream */
            appData.outStreamHandle = USB_HOST_AUDIO_V1_StreamOpen
                                      (
                                          appData.outStreamObj
                                      );

            if (appData.outStreamHandle == USB_HOST_AUDIO_V1_STREAM_HANDLE_INVALID)
            {
                appData.state = APP_STATE_ERROR; 
                break;
            }
            
            /* Set Stream Event Handler  */
            audioResult = USB_HOST_AUDIO_V1_StreamEventHandlerSet 
                           (
                               appData.outStreamHandle, 
                               APP_USBHostAudioStreamEventHandler, 
                               (uintptr_t)appData.outStreamObj
                           );

            if (audioResult != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
            {
                appData.state = APP_STATE_ERROR; 
                break;
            }
            
            appData.state = APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET;
            
            break; 
            
            case APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET:
                /* Disable Audio Stream */
                appData.isStreamInterfaceSetComplete = false; 
                audioResult = USB_HOST_AUDIO_V1_StreamingInterfaceSet
                               (
                                   appData.outStreamHandle, 
                                   &appData.requestHandle,
                                   appData.speakerStreamInterfaceSettingObjZeroBandwidth
                               );
                if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
                {
                    appData.state = APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET; 
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
            break;

        case APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO:
            if (appData.isStreamInterfaceSetComplete != true)
            {
                break; 
            }
            /* If  the attached speaker supports Mutes Control, then unmute it*/
            if (appData.isMasterMuteSupported == true)
            {
                /* Unmute the  Device */
                appData.isControlRequestCompleted = false; 
                audioResult = USB_HOST_AUDIO_V1_FeatureUnitChannelMuteSet
                              (
                                  appData.audioDeviceObj,
                                  appData.speakerFeatureUnitObj,
                                  &appData.requestHandle,
                                  0,
                                  (bool *)&appData.muteStatus
                              );
                if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
                {
                    appData.isControlRequestCompleted = true;
                    appData.state = APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO; 
                }
                else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                {
                    appData.state = APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE;
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
            break; 
            
        case APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE:
            if (appData.isControlRequestCompleted != true)
            {
                break; 
            }
            
            appData.currentVolume = 0x0500;
            if (appData.isMasterVolumeSupported == true)
            {
                appData.isControlRequestCompleted = false;

                //Send Master volume Control
                audioResult = USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeSet
                              (
                                  appData.audioDeviceObj,
                                  appData.speakerFeatureUnitObj,
                                  &appData.requestHandle,
                                  0,
                                  &appData.currentVolume
                              ); 
                if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
                {
                    appData.isControlRequestCompleted = true;
                    appData.state = APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE; 
                }
                else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                {
                    appData.state = APP_STATE_AUDIO_WAIT_FOR_MASTER_VOL_CONTROL_COMPLETE;
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
                audioResult = USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeSet
                              (
                                  appData.audioDeviceObj,
                                  appData.speakerFeatureUnitObj,
                                  &appData.requestHandle,
                                  1,
                                  &appData.currentVolume
                              ); 
                if (audioResult == USB_HOST_AUDIO_V1_RESULT_BUSY)
                {
                    appData.isControlRequestCompleted = true;
                    appData.state = APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE; 
                }
                else if (audioResult == USB_HOST_AUDIO_V1_RESULT_SUCCESS)
                {
                    appData.state = APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE;
                }
                else
                {
                    appData.state = APP_STATE_ERROR; 
                }
            }
            else
            {
                appData.state = APP_STATE_ENABLE_AUDIO_STREAM;
            }
            break;
            
        case APP_STATE_AUDIO_WAIT_FOR_MASTER_VOL_CONTROL_COMPLETE:
            if (appData.isControlRequestCompleted == true)
            {
                appData.state = APP_STATE_ENABLE_AUDIO_STREAM; 
            }
            break; 
            
        case APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE:
            if (appData.isControlRequestCompleted != true)
            {
                break; 
            }
            appData.isControlRequestCompleted = false;
            audioResult = USB_HOST_AUDIO_V1_FeatureUnitChannelVolumeSet
                          (
                              appData.audioDeviceObj,
                              appData.speakerFeatureUnitObj,
                              &appData.requestHandle,
                              2,
                              &appData.currentVolume
                          ); 
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
            break;
            
        case APP_STATE_AUDIO_WAIT_FOR_RIGHT_VOL_CONTROL_COMPLETE:
            if (appData.isControlRequestCompleted == true)
            {
                /* Advance Application state */
                appData.state = APP_STATE_ENABLE_AUDIO_STREAM; 
            }
            break;
            
        case  APP_STATE_ENABLE_AUDIO_STREAM:
            appData.isStreamInterfaceSetComplete = false;
            
            /* Set default interface setting of the streaming interface */
            audioResult = USB_HOST_AUDIO_V1_StreamingInterfaceSet
                           (
                               appData.outStreamHandle, 
                               &appData.requestHandle,
                               appData.speakerStreamInterfaceSettingObj
                           );
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
            break; 
            
        case  APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ONE:
            if (appData.isStreamInterfaceSetComplete == true)
            {
                appData.state = APP_STATE_START_STREAM_DATA;
            }
            break; 
            
        case APP_STATE_START_STREAM_DATA:
            interruptStatus = __builtin_disable_interrupts(); 
            appData.isAudioWriteCompleted = false;
            __builtin_mtc0(12,0,interruptStatus);
            appData.state = APP_SATE_WAIT_FOR_WRITE_COMPLETE; 
            USB_HOST_AUDIO_V1_StreamWrite            
            ( 
                appData.outStreamHandle,
                &appData.transferHandleAudioWrite, 
                (void*)&audioSamples, 
                192
            );
            break;

        case APP_SATE_WAIT_FOR_WRITE_COMPLETE:
            if (appData.isAudioWriteCompleted)
            {
                interruptStatus = __builtin_disable_interrupts(); 
                appData.isAudioWriteCompleted = false;
                __builtin_mtc0(12,0,interruptStatus);
                USB_HOST_AUDIO_V1_StreamWrite
                ( 
                    appData.outStreamHandle,
                    &appData.transferHandleAudioWrite,  
                    (void*)&audioSamples, 
                    192
                );
            }
            appData.state = APP_SATE_WAIT_FOR_WRITE_COMPLETE; 
            break;
        /* The default state should never be executed. */
        case APP_STATE_ERROR:
            countLedBlink++; 
            if (countLedBlink >= APP_LED_BLINK_COUNT)
            {
                BSP_LEDToggle(APP_USB_LED_2); 
                countLedBlink = 0; 
            }
            break; 
        default:
        {
            Nop(); 
            break;
        }
    }
}


void APP_FindAudioControls(void)
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
        appData.isLeftChannelPresent = appData.inputTerminaDescriptor->wChannelConfig & 0x0001;

        /* Find if Right channel is supported */
        appData.isRightChannelPresent = appData.inputTerminaDescriptor->wChannelConfig & 0x0002;

        /* Get pointer to Master audio control structure */
        audioControlMaster = (USB_AUDIO_FEATURE_UNIT_BMA_CONTROLS *)((uint8_t*)(&appData.featureUnitDescriptor->bControlSize) +1) ;

        /* Get pointer to Channel1 audio control structure */
        audioControlLeft = (USB_AUDIO_FEATURE_UNIT_BMA_CONTROLS *)((uint8_t*)audioControlMaster + bControlSize);

        /* Get pointer to Channel 2 audio control structure */
        audioControlRight = (USB_AUDIO_FEATURE_UNIT_BMA_CONTROLS *)((uint8_t*)audioControlLeft + bControlSize);

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


bool APP_SpeakerFeatureUnitObjectGet (uint8_t inputTerminalID, USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ* obj )
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
    audioResult = USB_HOST_AUDIO_V1_ControlEntityGetNext(appData.audioDeviceObj, entityObj, &entityObjFeatureUnit);
    if (audioResult != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
    {
        appData.state = APP_STATE_ERROR;
        return false; 
    }
    
    /* Get the Entity Type */ 
    entityType = USB_HOST_AUDIO_V1_EntityTypeGet(appData.audioDeviceObj,entityObjFeatureUnit); 
    
    if (entityType == USB_AUDIO_FEATURE_UNIT)
    {
        *obj = entityObj; 
        status = true; 
    }
    
    return status; 
}
void APP_GetSpeakerTopology (void )
{
    USB_HOST_AUDIO_V1_RESULT audioResult; 
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ entityObj, entityObjNext; 
    USB_AUDIO_V1_ENTITY_TYPE entityType; 
    USB_AUDIO_V1_TERMINAL_TYPE terminalType; 
    bool isSpeakerOutputTerminalFound = false; 
    uint8_t sourceId; 
    
    /* Get the First Audio Control Entity Object */ 
    audioResult = USB_HOST_AUDIO_V1_ControlEntityGetFirst(appData.audioDeviceObj, &entityObj);
    
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
            audioResult = USB_HOST_AUDIO_V1_ControlEntityGetNext (appData.audioDeviceObj, entityObj, &entityObjNext );
            if (audioResult != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
            {
                appData.state = APP_STATE_ERROR;
                return; 
            }
            entityObj = entityObjNext; 
            
            /* Get Entity Type */ 
            entityType = USB_HOST_AUDIO_V1_EntityTypeGet(appData.audioDeviceObj,entityObj);
            
            if (entityType == USB_AUDIO_OUTPUT_TERMINAL)
            {
                terminalType = USB_HOST_AUDIO_V1_TerminalTypeGet(appData.audioDeviceObj,entityObj ); 
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
        sourceId = USB_HOST_AUDIO_V1_TerminalSourceIDGet(appData.audioDeviceObj,entityObj);
        
        audioResult = USB_HOST_AUDIO_V1_EntityObjectGet(appData.audioDeviceObj, sourceId,  &entityObj );
        
        if (audioResult != USB_HOST_AUDIO_V1_RESULT_SUCCESS)
        {
            appData.state = APP_STATE_ERROR; 
            return; 
        }
        
        entityType = USB_HOST_AUDIO_V1_EntityTypeGet(appData.audioDeviceObj, entityObj ); 
        
        if (entityType == USB_AUDIO_FEATURE_UNIT)
        {
            
        }
        else
        {
            
        }   
    }
}

bool App_USBHostAudioSpeakerStreamFind
(
    USB_HOST_AUDIO_V1_OBJ audioDeviceObj,
    APP_USB_HOST_AUDIO_STREAM_FORTMAT audioStream, 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ * audioStreamingInterfaceObj, 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *streamingInterfaceSettingObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *streamingInterfaceSettingObjZeroBandwidth
)
{
    USB_HOST_AUDIO_V1_RESULT result;
    int count;
    bool status = false; 
    USB_AUDIO_V1_FORMAT_TAG formatTag; 
    USB_HOST_AUDIO_V1_STREAM_DIRECTION streamDirection; 
    uint8_t nChannels, bitResolution, subFrameSize, nSamplingFreq; 
    uint8_t *pSamplingFrequency; 
    uint32_t samplingFrequency; 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ streamingInterfaceSettingObjNext; 
    
    
    /* Get Number of Stream Groups */
    result = USB_HOST_AUDIO_V1_StreamingInterfaceGetFirst
             (
                 audioDeviceObj, 
                 audioStreamingInterfaceObj
             );
    if (result !=  USB_HOST_AUDIO_V1_RESULT_SUCCESS)
    {
      status = false;   
      return status; 
    }
    
    /* Get the First Streaming Interface Setting */            
    result = USB_HOST_AUDIO_V1_StreamingInterfaceSettingGetFirst
             (
                 appData.audioDeviceObj, 
                 *audioStreamingInterfaceObj, 
                  streamingInterfaceSettingObjZeroBandwidth);
    if (result !=  USB_HOST_AUDIO_V1_RESULT_SUCCESS)
    {
      status = false;   
      return status; 
    }
    
    
    /* Start searching for a suitable Audio Stream  */
    do 
    {
        result = USB_HOST_AUDIO_V1_StreamingInterfaceSettingGetNext
                 (
                     audioDeviceObj, 
                     *audioStreamingInterfaceObj,
                     *streamingInterfaceSettingObjZeroBandwidth,
                     &streamingInterfaceSettingObjNext
                 );
        *streamingInterfaceSettingObj = streamingInterfaceSettingObjNext;
        
        formatTag = USB_HOST_AUDIO_V1_StreamingInterfaceFormatTagGet
                    (
                        appData.audioDeviceObj,
                        *audioStreamingInterfaceObj,
                        *streamingInterfaceSettingObj
                    ); 
    
        streamDirection = USB_HOST_AUDIO_V1_StreamingInterfaceDirectionGet
                          (
                              appData.audioDeviceObj,
                              *audioStreamingInterfaceObj,
                              *streamingInterfaceSettingObj  
                          ); 

        nChannels = USB_HOST_AUDIO_V1_StreamingInterfaceChannelNumbersGet
                   (
                        appData.audioDeviceObj,
                        *audioStreamingInterfaceObj,
                        *streamingInterfaceSettingObj
                   ); 

        bitResolution = USB_HOST_AUDIO_V1_StreamingInterfaceBitResolutionGet
                        (
                            appData.audioDeviceObj,
                            *audioStreamingInterfaceObj,
                            *streamingInterfaceSettingObj    
                        ); 
        subFrameSize = USB_HOST_AUDIO_V1_StreamingInterfaceSubFrameSizeGet
                       (
                            appData.audioDeviceObj,
                            *audioStreamingInterfaceObj,
                            *streamingInterfaceSettingObj
                       ); 

        nSamplingFreq = USB_HOST_AUDIO_V1_StreamingInterfaceSamplingFrequencyTypeGet
                        (
                            appData.audioDeviceObj,
                            *audioStreamingInterfaceObj,
                            *streamingInterfaceSettingObj
                        ); 

        /* Compare Audio Stream info */
        if ((formatTag == audioStream.format)
            && (streamDirection == audioStream.streamDirection)
            && (nChannels == audioStream.nChannels)
            && (bitResolution == audioStream.bitResolution)
            && (subFrameSize == audioStream.subFrameSize))
        {    
            pSamplingFrequency = USB_HOST_AUDIO_V1_StreamingInterfaceSamplingFrequenciesGet
                                 (
                                     appData.audioDeviceObj,
                                     *audioStreamingInterfaceObj,
                                     *streamingInterfaceSettingObj
                                 );         
            for (count = 0; count < nSamplingFreq; count++)
            {
                samplingFrequency = (uint32_t)pSamplingFrequency[count*3]  |
                                    (uint32_t)pSamplingFrequency[count*3 + 1] << 8 |
                                    (uint32_t)pSamplingFrequency[count*3 + 2] << 16;
                if (samplingFrequency == audioStream.samplingRate )
                {
                    /* We have found an interface setting that is matching our 
                       requirement */ 
                    status = true; 
                    return status; 
                }
            }         
        }
        
    } while (result == USB_HOST_AUDIO_V1_RESULT_SUCCESS); 
    
          
    return status;  
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

void APP_ProcessSwitchPress(void)
{
    /* This function checks if the switch is pressed and then
     * debounces the switch press*/
    if(BSP_SWITCH_STATE_PRESSED == (BSP_SwitchStateGet(APP_USB_SWITCH_1)))
    {
        if(appData.ignoreSwitchPress)
        {
            /* This measn the key press is in progress */
            if(appData.sofEventHasOccurred)
            {
                /* A timer event has occurred. Update the debounce timer */
                appData.switchDebounceTimer ++;
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


/*************************************************************
 * AUDIO Host Attach Event Handler function.
 ************************************************************/
void App_USBHostAudioV1AttachHandler 
(
    USB_HOST_AUDIO_V1_OBJ audioObj, 
    USB_HOST_AUDIO_V1_EVENT event,
    uintptr_t context
)
{
    switch (event)
    {
        case USB_HOST_AUDIO_V1_EVENT_ATTACH: 
            if (appData.isAudioDeviceAttached == false)
            {
                appData.isAudioDeviceAttached = true; 
                appData.audioDeviceObj = audioObj; 
                BSP_LEDOn(APP_USB_LED_1); 
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
                appData.state = APP_STATE_WAIT_FOR_DEVICE_ATTACH;
                APP_AudioDataSetDefault();
                BSP_LEDOff(APP_USB_LED_1);
                break;
            }
        break; 
    }
    
}
/*************************************************************
 * AUDIO Host Application Event Handler function.
 ************************************************************/

USB_HOST_AUDIO_V1_STREAM_EVENT_RESPONSE APP_USBHostAudioStreamEventHandler
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE streamHandle,
    USB_HOST_AUDIO_V1_STREAM_EVENT event,
    void * eventData,
    uintptr_t context
)
{
    USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE_DATA * writeCompleteEventData;
    switch(event)
    {


        case USB_HOST_AUDIO_V1_STREAM_EVENT_INTERFACE_SET_COMPLETE:
            appData.isStreamInterfaceSetComplete = true; 
      
            break;
            
        case USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE:
            appData.sofEventHasOccurred = true;
            /* This means the Write request completed. We can
             * find out if the request was successful. */
            writeCompleteEventData =
                (USB_HOST_AUDIO_V1_STREAM_EVENT_WRITE_COMPLETE_DATA*)eventData;
            if(appData.transferHandleAudioWrite == writeCompleteEventData->transferHandle)
            {
                appData.isAudioWriteCompleted = true;
            }
            break;
         default:
            break;
    }
    return USB_HOST_AUDIO_V1_STREAM_EVENT_RESPONSE_NONE; 
}

/*************************************************************
 * USB Host Layer Application Event Handler function.
 ************************************************************/

USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler (USB_HOST_EVENT event, void * eventData, uintptr_t context)
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
}
void App_USBAudioControlRequestCallback
(
    USB_HOST_AUDIO_V1_OBJ audioObj, 
    USB_HOST_AUDIO_V1_REQUEST_HANDLE requestHandle,
    USB_HOST_AUDIO_V1_RESULT result,
    size_t size,
    uintptr_t context
)
{
    if (requestHandle == appData.requestHandle)
    {
        appData.isControlRequestCompleted = true; 
    }

}
/*******************************************************************************
 End of File
 */
