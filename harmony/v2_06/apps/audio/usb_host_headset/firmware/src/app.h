/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATE" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "system/debug/sys_debug.h"

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
//DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

#define APP_MAX_NO_OF_SAMPLES_IN_A_USB_FRAME 48
#define APP_USB_STEREO_SAMPLE_SIZE_BYTES  4
#define APP_USB_FRAME_SIZE_BYTES \
    APP_MAX_NO_OF_SAMPLES_IN_A_USB_FRAME*APP_USB_STEREO_SAMPLE_SIZE_BYTES

#define SAMPLING_RATE_8000             0x1F40
#define SAMPLING_RATE_11025            0x2B11
#define SAMPLING_RATE_12000            0x2EE0
#define SAMPLING_RATE_16000            0x3E80
#define SAMPLING_RATE_22050            0x5622
#define SAMPLING_RATE_24000            0x5DC0
#define SAMPLING_RATE_32000            0x7D00
#define SAMPLING_RATE_44100            0xAC44
#define SAMPLING_RATE_48000            0xBB80    

// *****************************************************************************
/* Application States

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
   /* Application's state machine's initial state. */
    APP_STATE_INIT=0,
    APP_STATE_CODEC_OPEN,  //Same as INIT state

    //Set the Read and Write Buffer Completion Callbacks via the Client Handles
    APP_STATE_CODEC_SET_BUFFER_HANDLER, 

    /* Application opens host layer */
    APP_STATE_BUS_ENABLE,

    //=====  The states below activate attach detection process code =====
    /* Application waits for Audio Device Attach */
    APP_STATE_WAIT_FOR_DEVICE_ATTACH,

    /* Audio Device is Attached (and is unmuted) */
    APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO,
    APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO_MIC,
            
    // Waiting for the initial unmuting of the Audio Speaker to Complete 
    // --Also set the volume
    APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE,
    APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE_MIC,

    // Wait for Master Volume Set Complete 
    // --When master volume interface available
    APP_STATE_AUDIO_WAIT_FOR_MASTER_VOL_CONTROL_COMPLETE, 
    APP_STATE_AUDIO_WAIT_FOR_MASTER_VOL_CONTROL_COMPLETE_MIC, 

    // Waiting for the initial Left Channel Volume Control to Complete 
    // --When Left and Right volume interface is available
    APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE,
    APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE_MIC,

    // Waiting for the initial Right Channel Volume Control to Complete 
    // --When Left and Right volume interface is available
    APP_STATE_AUDIO_WAIT_FOR_RIGHT_VOL_CONTROL_COMPLETE,
    APP_STATE_AUDIO_WAIT_FOR_RIGHT_VOL_CONTROL_COMPLETE_MIC,

    // Application waits for Zero Bandwidth Set to complete */
    APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ONE,
    APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ONE_MIC,
         
    // Application waits for Set Sampling Rate */        
    APP_STATE_SET_AUDIO_SAMPLING_RATE,
    APP_STATE_WAIT_FOR_SET_AUDIO_SAMPLING_RATE, 

    // Application waits for Set Sampling Rate */        
    APP_STATE_SET_AUDIO_SAMPLING_RATE_MIC,
    APP_STATE_WAIT_FOR_SET_AUDIO_SAMPLING_RATE_MIC, 

    /* Setting of correct alternate setting for the Audio Streaming Interface is
       completed  */
    APP_STATE_START_STREAM_DATA,

    /* Wait for Write Complete */
    APP_STATE_WAIT_FOR_WRITE_COMPLETE,

    /* Error */
    APP_STATE_ERROR,
    
    /* Wait for Bus enable complete */        
    APP_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE, 
            
    /* Start Scheduling Audio Stream Enable */
    APP_STATE_ENABLE_AUDIO_STREAM, 
    APP_STATE_ENABLE_AUDIO_STREAM_MIC, 
            
    /* Set Zero Bandwidth */ 
    APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET,  
    APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET_MIC,  

    //Process Complete USB Host Tx Buffers
    APP_STATE_INITIAL_USB_READ_REQUEST,
    APP_STATE_INITIAL_CODEC_WRITE_REQUEST,
    APP_STATE_PROCESS_DATA

} APP_STATES;

typedef enum
{
    APP_USB_AUDIO_CHANNEL_MASTER = 0,

    APP_USB_AUDIO_CHANNEL_LEFT,
            
    APP_USB_AUDIO_CHANNEL_RIGHT
} APP_USB_AUDIO_CHANNEL ;



// *****************************************************************************
/* Application CODEC/I2S Driver

  Summary:
    Application CODEC/I2S client for Audio.

  Description:
    This object holds the Codec client handle, 
    read and write buffer handle created and the context
*/
typedef struct
{
    DRV_HANDLE handle;
    DRV_CODEC_BUFFER_HANDLE readBufHandle0;
    DRV_CODEC_BUFFER_HANDLE readBufHandle1;    
    DRV_CODEC_BUFFER_EVENT_HANDLER bufferHandler;
    uintptr_t context;
    uint8_t *txbufferObject0;   //USB TX buffer 0
    uint8_t *txbufferObject1;   //USB TX buffer 1
    size_t bufferSize;
    bool isReadBufHandleAvailable0;
    bool isReadBufHandleAvailable1;
    bool isCodecReadComplete0;
    bool isCodecReadComplete1;
} APP_CODEC_READ_CLIENT;

typedef struct
{
    DRV_HANDLE handle;
    uintptr_t context;
    size_t bufferSize;
    DRV_CODEC_BUFFER_EVENT_HANDLER bufferHandler;
} APP_CODEC_WRITE_CLIENT;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */
typedef struct
{
    /* The application's current state */
    APP_STATES state;

    /* Saved Application State */
    APP_STATES savedState;
    
    /* Audio Device attached flag  */
    volatile bool isAudioDeviceAttached;

    /* USB Host layer Handle */
    USB_HOST_DEVICE_OBJ_HANDLE hostHandle;

    /* Object Handle of this Audio Device  */
    volatile USB_HOST_AUDIO_V1_OBJ audioDeviceObj;
    
    /* Audio OUT Stream  Object */
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ outStreamObj;
    
    /* Handle to the OUT Audio Stream */
    USB_HOST_AUDIO_V1_STREAM_HANDLE outStreamHandle; 
    
    /* Audio OUT Streaming Interface  Object */
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ speakerStreamInterfaceSettingObj;
    
    /* Audio OUT Streaming Interface  Object */
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ speakerStreamInterfaceSettingObjZeroBandwidth;
    
    /* Flag indicates if Audio OUT steam has been Found*/
    bool isAudioOutStreamFound; 

//------------------------------------------------------------------------------
    /* Audio IN Stream  Object */
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ inStreamObj;
    
    /* Handle to the IN Audio Stream */
    USB_HOST_AUDIO_V1_STREAM_HANDLE inStreamHandle; 
    
    /* Audio IN Streaming Interface  Object */
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ micStreamInterfaceSettingObj;
    
    /* Audio IN Streaming Interface  Object */
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ micStreamInterfaceSettingObjZeroBandwidth;
    
    /* Flag indicates if Audio IN steam has been Found*/
    bool isAudioInStreamFound; 
//------------------------------------------------------------------------------

    /* Stream Enable/Disable */
    USB_HOST_AUDIO_V1_REQUEST_HANDLE speakerRequestHandle;
    USB_HOST_AUDIO_V1_REQUEST_HANDLE micRequestHandle;

    //A USB Write Complete Buffer 
    //--Buffer ready for Codec Read
    volatile bool isUSBWriteCompleted;    //Changed during runtime by what???
    uint32_t      usbWriteBufferIdx; //USB buffer to complete next 
    bool          isUSBWriteComplete0;   //USB Buffer write complete
    bool          isUSBWriteComplete1;
    uint32_t      USBReadBufSize;

    /* Transfer handle for Audio Write*/
    USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE transferHandleUSBWrite0;
    USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE transferHandleUSBWrite1;

    /* Pointer to Input Terminal Descriptor */
    USB_AUDIO_INPUT_TERMINAL_DESCRIPTOR* inputTerminaDescriptor;
    
    /* Pointer to Feature Unit Descriptor */
    USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER* featureUnitDescriptor;

    /* Number of channels */
    uint8_t numberOfChannels; //Headphone OUT stream (2)
    uint32_t numMicChannels;  //Number of channels on mic IN stream (1)

    /* Flag indicates if Left Channel present */
    bool isLeftChannelPresent;
    bool isMicLeftChannelPresent;

    /* Flag indicates if Right Channel present */
    bool isRightChannelPresent;
    bool isMicRightChannelPresent;

    /* Flag indicates Master mute Control is supported */
    bool isMasterMuteSupported;
    bool isMicMasterMuteSupported;

    /* Flag indicates Left Channel mute is supported */
    bool isLeftMuteSupported;
    bool isMicLeftMuteSupported;

    /* Flag indicates Right Channel mute is supported */
    bool isRightMuteSupported;
    bool isMicRightMuteSupported;
    
    /* Current Mute Status */
    volatile bool muteStatus;
    volatile bool micMuteStatus;

    /* Flag indicates if Device supports Master Volume Control */
    bool isMasterVolumeSupported;
    bool isMicMasterVolumeSupported;

    /* Flag indicates if Device supports Left channel Volume Control */
    bool isLeftVolumeSupported;
    bool isMicLeftVolumeSupported;

    /* Flag indicates if Device supports Right Volume Control */
    bool isRightVolumeSupported;
    bool isMicRightVolumeSupported;

    /* Stores current volume in the Device */
    uint16_t currentVolume;
    
    /* Stream Has been enabled */
    volatile bool isStreamEnabled;
    volatile bool isMicStreamEnabled;

    /* Flag to indicate progress Stream Disable request */
    volatile bool isStreamInterfaceSetZeroComplete;
    volatile bool isMicStreamInterfaceSetZeroComplete;

    /* Flag to indicate progress Stream Enable request */
    volatile bool isStreamInterfaceSetOneComplete;
    volatile bool isMicStreamInterfaceSetOneComplete;
    
    volatile bool isMasterMuteSetComplete; 
    volatile bool isMicMasterMuteSetCompleteMic; 
    
    volatile bool isMasterUnmuteSetComplete; 
    volatile bool isMicMasterUnmuteSetComplete; 
    
    volatile bool isMasterVolumeSetComplete;
    volatile bool isMicMasterVolumeSetComplete;
    
    volatile bool isLeftVolumeSetComplete; 
    //volatile bool isMicLeftVolumeSetComplete; 
    
    volatile bool isRightVolumeSetComplete; 
    //volatile bool isMicRightVolumeSetComplete; 
    
    uint32_t switchDebounceTimer; 
    
    bool isMuteSwitchPressed; 
    
    bool ignoreSwitchPress; 
    
    bool sofEventHasOccurred; 
    
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ speakerFeatureUnitObj; 
    uint8_t speakerTerminalLink; 
    
//------------------------------------------------------------------------------
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ micFeatureUnitObj; 
    uint8_t micTerminalLink; 
//------------------------------------------------------------------------------
    
    volatile bool isControlRequestCompleted;
    
    volatile bool isStreamInterfaceSetComplete; 

    volatile bool isSpeakerSampleRateSetComplete;
    volatile bool isMicSampleRateSetComplete;

    //====================================================
    //CODEC Driver
    /* device configured state */
    bool isConfigured;

    // CODEC Read/Write Client handle
    // --Used to set the buffer completion callback handler
    APP_CODEC_WRITE_CLIENT codecClientWrite;
    APP_CODEC_READ_CLIENT codecClientRead;

    bool     dacMute;       //Audio muted
    uint32_t sampleFreq;    //Host USB Write Rate
    uint32_t sampleFreqMic; //Host USB Read Rate

    // Codec Write Complete Buffer
    bool isCodecReadComplete0;  //Ping-Pong buffer read
    bool isCodecReadComplete1;

    bool codecConfigured;   //Codec has completed configuration

} APP_DATA;

//------------------------------------------------------------------------------
// Application Codec Playback Buffer Queue
// NOTE:  For MZ must be aligned to 16 byte pages for DMA cache coherency 
typedef struct{
    //uint8_t __attribute__((coherent, aligned(32))) buffer[192];
    uint8_t buffer[192];
    DRV_CODEC_BUFFER_HANDLE writeHandle;   //4 bytes (ptr)
    USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE usbReadHandle;  //4 bytes (ptr)
    bool codecInUse;   //1 byte
    bool usbInUse;     //1 byte
    bool usbReadCompleted;  //1 byte
    int  padding;      //4 bytes
}APP_PLAYBACK_BUFFER;

//NOTE: APP_PLAYBACK BUFFER is what is transferred using DMA--> it is padded
// and placed at the beginning of the struct and the struct attribute is the
// allocated with the COHERENT and aligned(16) attributes
typedef struct{
    APP_PLAYBACK_BUFFER playbackBuffer[APP_PLAYBACK_QUEUE_BUFFER_SIZE];
    uint8_t readIdx;
    uint8_t writeIdx;
    uint32_t usbReadCompleteBufferLevel;
}APP_PLAYBACK_BUFFER_QUEUE;
//------------------------------------------------------------------------------

typedef struct 
{
    USB_AUDIO_FORMAT_CODE format; 
    USB_HOST_AUDIO_V1_STREAM_DIRECTION streamDirection; 
    uint8_t nChannels; 
    uint8_t subFrameSize; 
    uint8_t bitResolution; 
    uint32_t samplingRate; 
}APP_USB_HOST_AUDIO_STREAM_FORMAT ;

typedef enum 
{
    APP_USB_AUDIO_MASTER_MUTE_SET,
    APP_USB_AUDIO_MASTER_UNMUTE_SET,
    APP_USB_AUDIO_MASTER_VOLUME_SET,
    APP_USB_AUDIO_LEFT_VOLUME_SET,
    APP_USB_AUDIO_RIGHT_VOLUME_SET,  
} APP_USB_AUDIO_CONTROL_TRANSFER_ACTION;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Interface Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

    
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/
void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_AudioDataSetDefault (void)

  Summary:
    Set Audio Host application data to default values.

  Description:
    
  Precondition:
    
  Parameters:
    None.

  Returns:
    None.
*/
void APP_AudioDataSetDefault (void);

void APP_FindAudioControls(void);

void APP_SendAudioMuteControl( 
         APP_USB_AUDIO_CONTROL_TRANSFER_ACTION action,
         uint32_t* mute);

void APP_SendAudioVolumeControl( 
         APP_USB_AUDIO_CONTROL_TRANSFER_ACTION action,
         uint16_t* volume); 

/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks( void );
USB_HOST_AUDIO_V1_STREAM_EVENT_RESPONSE 
    APP_USBHostAudioStreamEventHandler(
        USB_HOST_AUDIO_V1_STREAM_HANDLE audioHandle,
        USB_HOST_AUDIO_V1_STREAM_EVENT event,
        void * eventData,
        uintptr_t context);

void App_USBHostAudioV1AttachHandler(
         USB_HOST_AUDIO_V1_OBJ audioObj,
         USB_HOST_AUDIO_V1_EVENT event,
         uintptr_t context);

USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler(USB_HOST_EVENT event, 
                                                void * eventData, 
                                                uintptr_t context);

bool App_USBHostAudioSpeakerStreamFind(
            USB_HOST_AUDIO_V1_OBJ audioDeviceObj,
            APP_USB_HOST_AUDIO_STREAM_FORMAT audioStream, 
            USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ * 
                audioStreamingInterfaceObj, 
            USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *
                streamingInterfaceSettingObj,
            USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *
                streamingInterfaceSettingObjZeroBandwidth,
            uint8_t * numChannels); 

void App_USBAudioControlRequestCallback(
            USB_HOST_AUDIO_V1_OBJ audioObj, 
            USB_HOST_AUDIO_V1_REQUEST_HANDLE requestHandle,
            USB_HOST_AUDIO_V1_RESULT result,
            size_t size,
            uintptr_t context); 

void APP_ProcessSwitchPress(void); 
bool APP_SpeakerFeatureUnitObjectGet(
            uint8_t inputTerminalID,
            USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ* obj ); 

bool APP_MicFeatureUnitObjectGet(
            uint8_t inputTerminalID, 
            USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ* obj ); 

/*******************************************************************************
  Function:
    void APP_CODECBufferEventHandlerRead(DRV_CODEC_BUFFER_EVENT event,
                                         DRV_CODEC_BUFFER_HANDLE handle, 
                                         uintptr_t context )

    void APP_CODECBufferEventHandlerWrite(DRV_CODEC_BUFFER_EVENT event,
                                          DRV_CODEC_BUFFER_HANDLE handle, 
                                          uintptr_t context )

  Summary:
    Event Handlers for codec Read/write Task.

  Description:
    This is the Event Handler for Codec Tx Complete Events.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    These routines must be called from SYS_Tasks() routine (or APP_Tasks()).
*/
void APP_CODECBufferEventHandlerWrite(DRV_CODEC_BUFFER_EVENT event, 
                                      DRV_CODEC_BUFFER_HANDLE handle, 
                                      uintptr_t context );

void APP_CODECBufferEventHandlerRead(DRV_CODEC_BUFFER_EVENT event,
                                     DRV_CODEC_BUFFER_HANDLE handle, 
                                     uintptr_t context );


//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* _APP_H */
/*******************************************************************************
 End of File
 */
