/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app_usb_audio_task.h

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
Copyright (c) 2016-2017 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _APP_USB_AUDIO_H
#define _APP_USB_AUDIO_H

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



// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END
#define APP_USB_AUDIO_NO_OF_SAMPLES_IN_A_USB_FRAME      48
#define APP_ID_FEATURE_UNIT                             0x05
#define APP_ID_INPUT_TERMINAL  0x01
#define APP_ID_OUTPUT_TERMINAL 0x02
    
#define APP_USB_AUDIO_QUEUING_DEPTH             USB_DEVICE_AUDIO_QUEUE_DEPTH_COMBINED
#define APP_USB_AUDIO_MAKE_BUFFER_DMA_READY     __attribute__((coherent)) __attribute__((aligned(4)))    


 

   
    
    
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/
// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/
typedef enum
{
    /* Application's state machine's initial state. */    
    APP_USB_AUDIO_STATE_INIT=0,
    APP_USB_AUDIO_STATE_SUBMIT_INITIAL_READ_REQUEST,
    APP_USB_AUDIO_STATE_PROCESS_DATA,
    APP_USB_AUDIO_STATE_IDLE,
    APP_USB_AUDIO_STATE_MUTE_AUDIO_PLAYBACK,
    APP_USB_AUDIO_STATE_USB_INTERFACE_ALTERNATE_SETTING_RCVD,
    APP_USB_AUDIO_STATE_WAIT_FOR_CONFIGURATION,
    APP_USB_AUDIO_STATE_ERROR,
    APP_USB_AUDIO_STATE_CODEC_OPEN,    
} APP_USB_AUDIO_STATES;    
    
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
} APP_USB_AUDIO_CODEC;


typedef enum
{
    APP_USB_AUDIO_ALTERNATE_SETTINGS_SPEAKER_PLAYBACK_NONE  = 0,
    APP_USB_AUDIO_ALTERNATE_SETTINGS_SPEAKER_PLAYBACK_STEREO_48KHZ = 1
} APP_USB_AUDIO_ALTERNATE_SETTINGS;

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
    APP_USB_AUDIO_CONTROL_NONE,
    APP_USB_AUDIO_MUTE_CONTROL
} APP_USB_AUDIO_CONTROLS;



typedef uint32_t (*APP_USB_AUDIO_STREAM_SOURCE) (void); 

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
    volatile APP_USB_AUDIO_STATES state;

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

    /* Holds active value of USB Audio Streaming Interface Alternate setting.
     * Every USB Audio Streaming should have 1 or more Alternate settings.
     * Alternate setting Zero (or Default Alternate setting) never has any
     * data payload. An audio streaming can have multiple number of alternate
     settings each having it own sampling rate, bandwidth etc. */
    APP_USB_AUDIO_ALTERNATE_SETTINGS activeInterfaceAlternateSetting;

    /* Holds current value of Audio Control Mute settings. A value True indicates
     * audio is currently muted. */
     bool dacMute;

    /* This is to indicate which Audio Control Transfer is currently
     * in progress. */
    APP_USB_AUDIO_CONTROLS currentAudioControl;

    /* CODEC client handle */
    APP_USB_AUDIO_CODEC codec;
    
    APP_USB_AUDIO_STREAM_SOURCE streamSource;
    
    volatile bool isUsbDetached;

} APP_USB_AUDIO_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.*/
void APP_USB_AUDIO_DeviceEventHandler 
(
    USB_DEVICE_EVENT events, 
    void * eventData, 
    uintptr_t context
);

void APP_USB_AUDIO_DeviceAudioEventHandler
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
    void APP_USB_AUDIO_Initialize(void);

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
    APP_SDCARD_AUDIO_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_USB_AUDIO_Initialize(void); 

/*******************************************************************************
  Function:
    void APP_USB_AUDIO_Tasks ( void )

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
    APP_SDCARD_AUDIO_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_USB_AUDIO_Tasks( void );
void APP_USB_AUDIO_BufferEventHandler(DRV_CODEC_BUFFER_EVENT event, 
                            DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context );
void APP_USB_AUDIO_ReInitialize (void);
void APP_USB_AUDIO_RegisterStreamSource(APP_USB_AUDIO_STREAM_SOURCE streamFunction);
void APP_USB_AUDIO_SuspendStreaming(void);
void APP_USB_AUDIO_AttachDevice(void);
void APP_USB_AUDIO_MuteOn(void);
void APP_USB_AUDIO_MuteOff(void);
uint8_t APP_USB_AUDIO_VolumeGet(void);
void APP_USB_AUDIO_VolumeSet(uint8_t volume);
bool APP_USB_AUDIO_isUSBDettached(void);
#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

