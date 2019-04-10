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
#include "system_definitions.h"
#include "system/debug/sys_debug.h"

//#include "system_config.h"
#include "app_config.h"

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

//==============================================================================
//NOTE: The following corresponds to the USB interfaces and must match the
//      USB descriptor generated in system_init.c
//==============================================================================
#define APP_MAX_NO_OF_SAMPLES_IN_A_USB_FRAME 48
    
#define MICROPHONE_EP 0x81
#define HEADPHONE_EP 0x01   
#define SAMPLING_RATE_8000             0x1F40
#define SAMPLING_RATE_11025            0x2B11
#define SAMPLING_RATE_12000            0x2EE0
#define SAMPLING_RATE_16000            0x3E80
#define SAMPLING_RATE_22050            0x5622
#define SAMPLING_RATE_24000            0x5DC0
#define SAMPLING_RATE_32000            0x7D00
#define SAMPLING_RATE_44100            0xAC44
#define SAMPLING_RATE_48000            0xBB80    
#define USB_DEVICE_AUDIO_CONTROL_INTERFACE_ID           0x00
#define APP_ID_INPUT_TERMINAL  0x01
#define APP_ID_FEATURE_UNIT    0x02
#define APP_ID_OUTPUT_TERMINAL 0x03
#define APP_ID_INPUT_TERMINAL_MICROPHONE  0x04    
#define APP_ID_FEATURE_UNIT_MICROPHONE    0x05
#define APP_ID_OUTPUT_TERMINAL_MICROPHONE 0x06    
#define APP_ID_FEATURE_UNIT_SIDE_TONING    0x07    
#define APP_ID_MIXER_UNIT    0x08    
#define USB_DEVICE_AUDIO_STREAMING_INTERFACE_ID_1       0x01
    
#define APP_PLAYBACK_INTERFACE  0x01
#define APP_RECORDING_INTERFACE 0x02


// *****************************************************************************
/* Application Audio Control

  Summary:
    Defines a list of audio controls supported by this application.

  Description:
    This enumeration defines audio controls supported by this application. This
 could be used for checking which Control Transfer is currently active.
*/
typedef enum
{
    APP_USB_CONTROL_NONE,
    APP_USB_AUDIO_MUTE_CONTROL,
    APP_USB_AUDIO_SAMPLING_FREQ_CONTROL_MP,
    APP_USB_AUDIO_SAMPLING_FREQ_CONTROL_HP,
} APP_AUDIO_CONTROLS;


typedef enum
{
    APP_USB_SPEAKER_PLAYBACK_NONE  = 0,
    APP_USB_SPEAKER_PLAYBACK_STEREO_16KHZ = 1
} APP_ALTERNATE_SETTINGS;

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
    APP_SUBMIT_INITIAL_USB_READ_REQUEST,
    APP_SUBMIT_INITIAL_CODEC_READ_REQUEST,
    APP_SUBMIT_INITIAL_CODEC_WRITE_REQUEST,
    APP_PROCESS_DATA,
    APP_IDLE,
    APP_MUTE_AUDIO_PLAYBACK,
    APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD,
    APP_SAMPLING_FREQUENCY_CHANGE,
    APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD_HP, //headphone
    APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD_MP, //microphone
    APP_STATE_WAIT_FOR_CONFIGURATION,
    APP_STATE_ERROR,
    APP_STATE_CODEC_OPEN,
    APP_STATE_CODEC_SET_BUFFER_HANDLER
} APP_STATES;

// *****************************************************************************
/* Application USART client for BT

  Summary:
    Application USART client for BT.

  Description:
    This object holds the BT USART's client handle, read and write buffer handle
    created and the context
*/
typedef struct
{
    DRV_HANDLE handle;
    DRV_CODEC_BUFFER_HANDLE writeBufHandle1;
    DRV_CODEC_BUFFER_HANDLE writeBufHandle2;    
    DRV_CODEC_BUFFER_EVENT_HANDLER bufferHandler;
    uintptr_t context;
    uint8_t *txbufferObject1;
    uint8_t *txbufferObject2;
    size_t bufferSize;
    bool iswriteBufHandleAvailable1;
    bool iswriteBufHandleAvailable2;
    bool isCodecReadComplete1;
    bool isCodecReadComplete2;
} APP_CODEC_CLIENT;

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
    /* Application's current state*/
    volatile APP_STATES state;
    APP_STATES lastState;  //Debug info

     /* device layer handle returned by device layer open function */
    USB_DEVICE_HANDLE   usbDevHandle;

    /* Instance number of Audio Function driver */
    USB_DEVICE_AUDIO_INDEX audioInstance;

    /* device configured state */
    bool isConfigured;

    /* True if a character was read */
    bool isReadComplete1;
    bool isReadComplete2;

    USB_DEVICE_AUDIO_TRANSFER_HANDLE readTransferHandle1;
    USB_DEVICE_AUDIO_TRANSFER_HANDLE readTransferHandle2;
    
    uint32_t USBReadBufSize;
    
    bool isWriteComplete1;
    bool isWriteComplete2;
    bool isUSBWriteComplete1;
    bool isUSBWriteComplete2;
    
    USB_DEVICE_AUDIO_TRANSFER_HANDLE writeTransferHandle1;
    USB_DEVICE_AUDIO_TRANSFER_HANDLE writeTransferHandle2;

    /* Holds active value of USB Audio Streaming Interface Alternate setting.
     * Every USB Audio Streaming should have 1 or more Alternate settings.
     * Alternate setting Zero (or Default Alternate setting) never has any
     * data payload. An audio streaming can have multiple number of alternate
     settings each having it own sampling rate, bandwidth etc. */
    APP_ALTERNATE_SETTINGS activeInterfaceAlternateSetting;
    
    APP_ALTERNATE_SETTINGS activeMicInterfaceAlternateSetting;

    /* Holds current value of Audio Control Mute settings. A value True indicates
     * audio is currently muted. */
     bool dacMute;
     
     uint32_t sampleFreq;
     
     uint32_t sampleFreqMic;

    /* This is to indicate which Audio Control Transfer is currently
     * in progress. */
    APP_AUDIO_CONTROLS currentAudioControl;

    /* CODEC client handle */
    APP_CODEC_CLIENT codecClientWrite;
    
    APP_CODEC_CLIENT codecClientRead;
    
} APP_DATA;

// Application Codec Playback Buffer Queue
// NOTE:  For MZ must be aligned to 16 byte pages for DMA cache coherency 
typedef struct{
    //uint8_t __attribute__((coherent, aligned(32))) buffer[192];
    uint8_t buffer[192];
    DRV_CODEC_BUFFER_HANDLE writeHandle;   //4 bytes (ptr)
    USB_DEVICE_AUDIO_TRANSFER_HANDLE usbReadHandle;  //4 bytes (ptr)
    bool codecInUse;   //1 byte
    bool usbInUse;     //1 byte
    bool usbReadCompleted;  //1 byte
    int  padding;      //4 bytes
}APP_PLAYBACK_BUFFER;

//#define APP_PLAYBACK_QUEUE_BUFFER_SIZE 64  

//NOTE: APP_PLAYBACK BUFFER is what is transferred using DMA--> it is padded
// and placed at the beginning of the struct and the struct attribute is the
// allocated with the COHERENT and aligned(16) attributes
typedef struct{
    APP_PLAYBACK_BUFFER playbackBuffer[APP_PLAYBACK_QUEUE_BUFFER_SIZE];
    uint8_t readIdx;
    uint8_t writeIdx;
    uint32_t usbReadCompleteBufferLevel;
}APP_PLAYBACK_BUFFER_QUEUE;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Interface Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.*/
void APP_USBDeviceEventHandler
(
    USB_DEVICE_EVENT events,
    void * eventData,
    uintptr_t context
);

void APP_USBDeviceAudioEventHandler
(
    USB_DEVICE_AUDIO_INDEX iAudio ,
    USB_DEVICE_AUDIO_EVENT event ,
    void * pData,
    uintptr_t context
);

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
void APP_Tasks ( void );

/*******************************************************************************
  Function:
    void APP_CODECBufferEventHandler(DRV_CODEC_BUFFER_EVENT event,
        DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context )

  Summary:
    Event Handler for codec Task.

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
    This routine must be called from SYS_Tasks() routine.
*/

void APP_CODECBufferEventHandler(DRV_CODEC_BUFFER_EVENT event, 
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

