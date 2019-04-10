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
    internally by the application (such as the "APP_STATES" definition).  Both
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

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

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
            
   /* Application opens host layer */
    APP_STATE_BUS_ENABLE,

    /* Application waits for Audio Device Attach */
    APP_STATE_WAIT_FOR_DEVICE_ATTACH,

    /* Audio Device is Attached */
    APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ZERO,
            
    /* Waiting for the initial unmuting of the Audio Speaker to Complete */
    APP_STATE_AUDIO_WAIT_FOR_UNMUTE_COMPLETE,

    /* Waiting for the initial Left Channel Volume Control to Complete */
    APP_STATE_AUDIO_WAIT_FOR_LEFT_VOL_CONTROL_COMPLETE,

    /* Waiting for the initial Right Channel Volume Control to Complete */
    APP_STATE_AUDIO_WAIT_FOR_RIGHT_VOL_CONTROL_COMPLETE,

    /* Application waits for Zero Bandwidth Set to complete */
    APP_STATE_WAIT_FOR_AUDIO_STREAM_INTERFACE_SET_ONE,
         
    /* Application waits for Set Sampling Rate */        
    AAP_STATE_WAIT_FOR_SET_STEAM_SAMPLE_RATE, 

    /* Setting of correct alternate setting for the Audio Streaming Interface is
       completed  */
    APP_STATE_START_STREAM_DATA,

    /* Wait for Write Complete */
    APP_SATE_WAIT_FOR_WRITE_COMPLETE,

    /* Error */
    APP_STATE_ERROR,
    
    /* Wait for Bus enable complete */        
    APP_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE, 
            
    /* Wait for Master Volume Set Complete */
    APP_STATE_AUDIO_WAIT_FOR_MASTER_VOL_CONTROL_COMPLETE, 
            
    /* Start Scheduling Audio Stream Enable */
    APP_STATE_ENABLE_AUDIO_STREAM, 
            
    /* Set Zero Bandwidth */ 
    APP_STATE_ZERO_BANDWIDTH_INTERFACE_SET  
} APP_STATES;

typedef enum
{
    APP_USB_AUDIO_CHANNEL_MASTER = 0,

    APP_USB_AUDIO_CHANNEL_LEFT,
            
    APP_USB_AUDIO_CHANNEL_RIGHT
} APP_USB_AUDIO_CHANNEL ;
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

    /* Transfer handle for Audio Write*/
    USB_HOST_AUDIO_V1_STREAM_TRANSFER_HANDLE transferHandleAudioWrite;

    /* Stream Enable/Disable */
    USB_HOST_AUDIO_V1_REQUEST_HANDLE requestHandle;

    /* Audio Write completed */
    volatile bool isAudioWriteCompleted;

    /* Pointer to Input Terminal Descriptor */
    USB_AUDIO_INPUT_TERMINAL_DESCRIPTOR* inputTerminaDescriptor;
    
    /* Pointer to Feature Unit Descriptor */
    USB_AUDIO_FEATURE_UNIT_DESCRIPTOR_HEADER* featureUnitDescriptor;

    /* Number of channels */
    uint8_t numberOfChannels;

    /* Flag indicates if Left Channel present */
    bool isLeftChannelPresent;

    /* Flag indicates if Right Channel present */
    bool isRightChannelPresent;

    /* Flag indicates Master mute Control is supported */
    bool isMasterMuteSupported;

    /* Flag indicates Left Channel mute is supported */
    bool isLeftMuteSupported;

    /* Flag indicates Right Channel mute is supported */
    bool isRightMuteSupported;
    
    /* Current Mute Status */
    volatile bool muteStatus;

    /* Flag indicates if Device supports Master Volume Control */
    bool isMasterVolumeSupported;

    /* Flag indicates if Device supports Left channel Volume Control */
    bool isLeftVolumeSupported;

    /* Flag indicates if Device supports Right Volume Control */
    bool isRightVolumeSupported;

    /* Stores current volume in the Device */
    uint16_t currentVolume;
    
    /* Stream Has been enabled */
    volatile bool isStreamEnabled;

    /* Flag to indicate progress Stream Disable request */
    volatile bool isStreamInterfaceSetZeroComplete;

    /* Flag to indicate progress Stream Enable request */
    volatile bool isStreamInterfaceSetOneComplete;
    
    volatile bool isMasterMuteSetComplete; 
    
    volatile bool isMasterUnmuteSetComplete; 
    
   volatile bool isMasterVolumeSetComplete;
    
    volatile bool isLeftVolumeSetComplete; 
    
    volatile bool isRightVolumeSetComplete; 
    
    uint32_t switchDebounceTimer; 
    
    bool isMuteSwitchPressed; 
    
    bool ignoreSwitchPress; 
    
    bool sofEventHasOccurred; 
    
    USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ speakerFeatureUnitObj; 
    
    uint8_t speakerTerminalLink; 
    
    volatile bool isControlRequestCompleted;
    
    volatile bool isStreamInterfaceSetComplete; 

} APP_DATA;

typedef struct 
{
    USB_AUDIO_FORMAT_CODE format; 
    USB_HOST_AUDIO_V1_STREAM_DIRECTION streamDirection; 
    uint8_t nChannels; 
    uint8_t subFrameSize; 
    uint8_t bitResolution; 
    uint32_t samplingRate; 
}APP_USB_HOST_AUDIO_STREAM_FORTMAT ;

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
// Section: Application Callback Routines
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

void APP_SendAudioMuteControl 
( 
    APP_USB_AUDIO_CONTROL_TRANSFER_ACTION action,
    uint32_t* mute
);

void APP_SendAudioVolumeControl 
( 
    APP_USB_AUDIO_CONTROL_TRANSFER_ACTION action,
    uint16_t* volume
); 

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
USB_HOST_AUDIO_V1_STREAM_EVENT_RESPONSE APP_USBHostAudioStreamEventHandler
(
    USB_HOST_AUDIO_V1_STREAM_HANDLE audioHandle,
    USB_HOST_AUDIO_V1_STREAM_EVENT event,
    void * eventData,
    uintptr_t context
);

void App_USBHostAudioV1AttachHandler
(
    USB_HOST_AUDIO_V1_OBJ audioObj,
    USB_HOST_AUDIO_V1_EVENT event,
    uintptr_t context
);
USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler (USB_HOST_EVENT event, void * eventData, uintptr_t context);

bool App_USBHostAudioSpeakerStreamFind
(
    USB_HOST_AUDIO_V1_OBJ audioDeviceObj,
    APP_USB_HOST_AUDIO_STREAM_FORTMAT audioStream, 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_OBJ * audioStreamingInterfaceObj, 
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *streamingInterfaceSettingObj,
    USB_HOST_AUDIO_V1_STREAMING_INTERFACE_SETTING_OBJ *streamingInterfaceSettingObjZeroBandwidth
); 

void App_USBAudioControlRequestCallback
(
    USB_HOST_AUDIO_V1_OBJ audioObj, 
    USB_HOST_AUDIO_V1_REQUEST_HANDLE requestHandle,
    USB_HOST_AUDIO_V1_RESULT result,
    size_t size,
    uintptr_t context
); 

void APP_ProcessSwitchPress(void); 
bool APP_SpeakerFeatureUnitObjectGet (uint8_t inputTerminalID, USB_HOST_AUDIO_V1_CONTROL_ENTITY_OBJ* obj ); 
#endif /* _APP_H */
/*******************************************************************************
 End of File
 */

