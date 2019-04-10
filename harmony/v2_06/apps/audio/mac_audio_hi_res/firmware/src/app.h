/*******************************************************************************
  Application Header

  File Name:
    app.h

  Summary:
    ADC data logger demo application definitions (advanced driver-based version)

  Description:
    This file contains the ADC data logger demo application definitions for the
    driver-based version that uses some advanced driver features.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2012 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#ifndef _APP_HEADER_H
#define _APP_HEADER_H
//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <xc.h>
#include "app_config.h"
#include "bsp.h"    // was "bsp_config.h"
#include "system_definitions.h"
#include "usb/usb_audio_v2_0.h"
#include "usb/usb_device_audio_v2_0.h"
    
//#define SUPPORT_ALL_SAMPLE_RATES                              // all sample rates if defined, only 44.1, 88.2, 176.4 and 192 if commented out

#define ASYNCHRONOUS_FEEDBACK
#define AUDIO_FRAME_SIZE                                        (APP_AUDIO_MAX_SAMPLES*4*2)
#define AUDIO_FRAME_SIZE_UAC2                                   (AUDIO_FRAME_SIZE/8)

#define APP_USB_AUDIO_SAMPLING_RATE_INITIAL                     APP_AUDIO_MAX_FREQ     
#define APP_USB_AUDIO_SAMPLING_RATE_32KHZ                       32000
#define APP_USB_AUDIO_SAMPLING_RATE_44_1KHZ                     44100
#define APP_USB_AUDIO_SAMPLING_RATE_48KHZ                       48000
#define APP_USB_AUDIO_SAMPLING_RATE_88_2KHZ                     88200
#define APP_USB_AUDIO_SAMPLING_RATE_96KHZ                       96000    
#define APP_USB_AUDIO_SAMPLING_RATE_176_4KHZ                    176400    
#define APP_USB_AUDIO_SAMPLING_RATE_192KHZ                      192000        
        
/* 24(+8)Left Channel Bits, 24(+8)Right Channel Bits */
#define APP_USB_DATA_PACKET_SIZE                                8       
    
#define APP_USB_AUDIO_BUFFER_QUEUE_TUNE_UPPER_LIMIT_32          63  
#define APP_USB_AUDIO_BUFFER_QUEUE_TUNE_LOWER_LIMIT_32          50     
#define APP_USB_FEEDBACK_32_NORMAL                              0x00   
#define APP_USB_FEEDBACK_32_SPEED_UP                            0x00001500
#define APP_USB_FEEDBACK_32_SLOW_DOWN                           0x00

#define APP_USB_AUDIO_BUFFER_QUEUE_TUNE_UPPER_LIMIT_44_1        63  
#define APP_USB_AUDIO_BUFFER_QUEUE_TUNE_LOWER_LIMIT_44_1        50     
#define APP_USB_FEEDBACK_44_1_NORMAL                            0x00   
#define APP_USB_FEEDBACK_44_1_SPEED_UP                          0x00009450
#define APP_USB_FEEDBACK_44_1_SLOW_DOWN                         0x00    

#define APP_USB_AUDIO_BUFFER_QUEUE_TUNE_UPPER_LIMIT_48          63  
#define APP_USB_AUDIO_BUFFER_QUEUE_TUNE_LOWER_LIMIT_48          45     
#define APP_USB_FEEDBACK_48_NORMAL                              0x00   
#define APP_USB_FEEDBACK_48_SPEED_UP                            0x00002500
#define APP_USB_FEEDBACK_48_SLOW_DOWN                           0x00   

#define APP_USB_AUDIO_BUFFER_QUEUE_TUNE_UPPER_LIMIT_88_2        63  
#define APP_USB_AUDIO_BUFFER_QUEUE_TUNE_LOWER_LIMIT_88_2        45     
#define APP_USB_FEEDBACK_88_2_NORMAL                            0x00   
#define APP_USB_FEEDBACK_88_2_SPEED_UP                          0x00002500
#define APP_USB_FEEDBACK_88_2_SLOW_DOWN                         0x00        

#define APP_USB_AUDIO_BUFFER_QUEUE_TUNE_UPPER_LIMIT_96          63  
#define APP_USB_AUDIO_BUFFER_QUEUE_TUNE_LOWER_LIMIT_96          45      
#define APP_USB_FEEDBACK_96_NORMAL                              0x00   
#define APP_USB_FEEDBACK_96_SPEED_UP                            0x00002500
#define APP_USB_FEEDBACK_96_SLOW_DOWN                           0x00     

#define APP_USB_AUDIO_BUFFER_QUEUE_TUNE_UPPER_LIMIT_176_4       63  
#define APP_USB_AUDIO_BUFFER_QUEUE_TUNE_LOWER_LIMIT_176_4       50    
#define APP_USB_FEEDBACK_176_4_NORMAL                           0x00   
#define APP_USB_FEEDBACK_176_4_SPEED_UP                         0x00008450
#define APP_USB_FEEDBACK_176_4_SLOW_DOWN                        0x00            

#define APP_USB_AUDIO_BUFFER_QUEUE_TUNE_UPPER_LIMIT_192         63  
#define APP_USB_AUDIO_BUFFER_QUEUE_TUNE_LOWER_LIMIT_192         50     
#define APP_USB_FEEDBACK_192_NORMAL                             0x00   
#define APP_USB_FEEDBACK_192_SPEED_UP                           0x00008450
#define APP_USB_FEEDBACK_192_SLOW_DOWN                          0x00    

#define APP_VOLUME_STEP_VALUE                                   4
#define APP_VOLUME_MIN_VALUE                                    0
#define APP_VOLUME_MAX_VALUE                                    (DRV_CODEC_VOLUME_MAX - 7) // Avoiding wrapup after 255 after step by 4

// *****************************************************************************
/* Application States

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behaviour of the application at various times.
*/

typedef enum
{
    APP_STATE_INIT,
    APP_STATE_WAIT_FOR_ALTERNATE_SETTING,
    APP_SUBMIT_READ_REQUEST,
    APP_PROCESS_DATA,
    APP_IDLE,
    APP_MUTE_AUDIO_PLAYBACK,
    APP_USB_INTERFACE_ALTERNATE_SETTING_RCVD,
    INITIALIZE_AUDIO_CODEC,
    APP_STATE_WAIT_FOR_CONFIGURATION,
    APP_CLOCKSOURCE_SET,
    APP_CLOCKSELECT_SET,
    APP_CODEC_BUFFER_HANDLER_SET,
    APP_REINITIALIZE,
    APP_DAC_MUTE,   
    APP_DAC_VOLUME_INCREASE,   
    APP_DAC_VOLUME_DECREASE,            
    APP_IDLE_BLINK,            
    /* Application Error state*/                        
    APP_STATE_ERROR = -1
} APP_STATES;

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
    APP_USB_AUDIO_CLOCKSOURCE_CONTROL,
    APP_USB_AUDIO_CLOCKSELECT_CONTROL
} APP_AUDIO_CONTROLS;

typedef enum
{
    APP_USB_SPEAKER_PLAYBACK_NONE  = 0,
    APP_USB_SPEAKER_PLAYBACK_STEREO = 1
} APP_ALTERNATE_SETTINGS;

typedef struct
{
    DRV_HANDLE handle;
    DRV_CODEC_BUFFER_HANDLE writeBufHandle[APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT];
    DRV_CODEC_BUFFER_EVENT_HANDLER bufferHandler;
    uintptr_t context;
    uint8_t *txbufferObject[APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT];
    size_t bufferSize[APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT];
    uint8_t volume; 
    bool isMute;
    bool isInitialized; 
    uint32_t samplingRate;
} APP_CODEC_CLIENT;

typedef struct __attribute__((packed))
{
    unsigned short  numSampleRate;          // Number of supported sampling rates
#ifdef SUPPORT_ALL_SAMPLE_RATES
    USB_AUDIO_V2_LAYOUT_3_RANGE_PARAMETER_BLOCK sampleRate32;
    USB_AUDIO_V2_LAYOUT_3_RANGE_PARAMETER_BLOCK sampleRate44;
    USB_AUDIO_V2_LAYOUT_3_RANGE_PARAMETER_BLOCK sampleRate48;
    USB_AUDIO_V2_LAYOUT_3_RANGE_PARAMETER_BLOCK sampleRate88;
    USB_AUDIO_V2_LAYOUT_3_RANGE_PARAMETER_BLOCK sampleRate96;
    USB_AUDIO_V2_LAYOUT_3_RANGE_PARAMETER_BLOCK sampleRate176;
    USB_AUDIO_V2_LAYOUT_3_RANGE_PARAMETER_BLOCK sampleRate192;
#else
    USB_AUDIO_V2_LAYOUT_3_RANGE_PARAMETER_BLOCK sampleRate44;
    USB_AUDIO_V2_LAYOUT_3_RANGE_PARAMETER_BLOCK sampleRate88;
    USB_AUDIO_V2_LAYOUT_3_RANGE_PARAMETER_BLOCK sampleRate176;
    USB_AUDIO_V2_LAYOUT_3_RANGE_PARAMETER_BLOCK sampleRate192;    
#endif
} APP_USB_AUDIO_V2_CLOCKSOURCE_RANGE;
// *****************************************************************************
/* Application USB transfer client

  Summary:
    Application USB transfer client.

  Description:
    This object holds the USB queued handles handle and state of its respective buffer
    created and the context
*/
typedef struct
{
    USB_DEVICE_AUDIO_V2_TRANSFER_HANDLE 
            readTransferHandle[APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT];
    volatile bool dataAvailable[APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT];
    uint8_t __attribute__((coherent)) __attribute__((aligned(16))) 
            rxBuffer[APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT]
                    [APP_NO_OF_SAMPLES_IN_A_USB_FRAME];        

} APP_USB_READ_CLIENT;

typedef enum
{
    APP_DISP_STATUS_USB_NOT_CONNECTED,
    APP_DISP_STATUS_USB_CONNECTED,     
    APP_DISP_STATUS_USB_DISCONNECTED,
    APP_DISP_STATUS_MUTE_OFF,            
    APP_DISP_STATUS_APP_MUTE_ON,
    APP_DISP_STATUS_VOLUME_INCREASE,    
    APP_DISP_STATUS_VOLUME_DECREASE,
    APP_DISP_STATUS_SAMPLING_RATE            
} APP_DISP_STATUS;

typedef struct
{
    bool update;    
    APP_DISP_STATUS status;
    uint8_t volumeP;      
    char volumePercent[4];   
    //char lastVolumePercent[4];     
    char samplingRate[10];     
} APP_DISPLAY;

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
    /* device layer handle returned by device layer open function */
    USB_DEVICE_HANDLE usbDevHandle;
      /* Instance number of Audio Function driver */
    USB_DEVICE_AUDIO_V2_INDEX audioInstance;
    /* Application's current state*/
    APP_STATES state;
    /* USART client handle */
    APP_CODEC_CLIENT codecClient;
    /* device configured state */
    bool isConfigured;
    /* True if a character was read */
    bool isReadComplete;
    /* True if a character was written*/
    bool isWriteComplete;
    USB_DEVICE_AUDIO_V2_TRANSFER_HANDLE writeTransferHandle;
    APP_USB_READ_CLIENT usbReadQueue;
    bool usbReadComplete[APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT];
    bool CodecWriteComplete[APP_USB_AUDIO_BUFFER_QUEUE_MAX_COUNT];   
    /* Holds active value of USB Audio Streaming Interface Alternate setting.
     * Every USB Audio Streaming should have 1 or more Alternate settings.
     * Alternate setting Zero (or Default Alternate setting) never has any
     * data payload. An audio streaming can have multiple number of alternate
     settings each having it own sampling rate, bandwidth etc. */
    APP_ALTERNATE_SETTINGS activeInterfaceAlternateSetting;
    /* Holds current value of Audio Control Mute settings. A value True indicates
     * audio is currently muted. */
    bool dacMute;
    uint32_t lastClockSource;    
    uint32_t clockSource;    
    uint8_t clockValid;   
    APP_USB_AUDIO_V2_CLOCKSOURCE_RANGE clockSourceRange;
    // Clock Select
    uint8_t clockSelectIndex;
    /* This is to indicate which Audio Control Transfer is currently
     * in progress. */
    APP_AUDIO_CONTROLS currentAudioControl;      
    APP_DISPLAY display;   
    
    
    uint32_t usbAudioBufferUpperLimit;
    uint32_t usbAudioBufferLowerLimit;    
    uint32_t maxAudioSamples;
    uint32_t usbAudioFrameSize;
    uint32_t usbAudioFrameSizeUa2;

    uint32_t usbFeedbackNormal;
    uint32_t usbFeedbackMax;
    uint32_t usbFeedbackMin;        
    
    uint8_t usbFeedbackNormalValue[4];
    uint8_t usbFeedbackSpeedUpValue[4];
    uint8_t usbFeedbackSlowDownValue[4];            
    
} APP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/


/*******************************************************************************
  Function:
    void APP_UsbDeviceEventCallBack(USB_DEVICE_EVENTS events)

  Summary:
    Device layer event notification callback.

  Description:
    This routine defines the device layer event notification callback.

  Precondition:
    The device layer should be opened by the application and the callback should
    be registered with the device layer.

  Parameters:
    events  - specific device event

  Returns:
    None.

  Remarks:
    None.
*/

void APP_UsbDeviceEventCallBack
(
    USB_DEVICE_EVENT events,
    void * eventData,
    uintptr_t context
);

void APP_USBDeviceAudioEventHandler
(
    USB_DEVICE_AUDIO_V2_INDEX iAudio ,
    USB_DEVICE_AUDIO_V2_EVENT event ,
    void * pData,
    uintptr_t context
);

int APP_AudioCurEntitySettingsHandler
(
    USB_DEVICE_AUDIO_V2_INDEX iAudio ,
    USB_AUDIO_V2_CONTROL_INTERFACE_REQUEST *setupPkt
);

int APP_AudioRangeEntitySettingsHandler
(
    USB_DEVICE_AUDIO_V2_INDEX iAudio ,
    USB_AUDIO_V2_CONTROL_INTERFACE_REQUEST *setupPkt
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
     ADC data logger application initialization routine

  Description:
    This routine initializes ADC data logger application.  This function opens
    the necessary drivers, initializes the timer and registers the application
    callback with the USART driver.

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
    USB Device Audio application tasks function

  Description:
    This routine is the USB Device Audio Speaker application's tasks function.  It
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

void APP_ButtonTask( void );

/*******************************************************************************
  Function:
    void APP_USARTBufferEventHandler(DRV_USART_BUFFER_EVENT event,
        DRV_USART_BUFFER_HANDLE handle, uintptr_t context )

  Summary:
    Event Handler for USART Task.

  Description:
    This is the Event Handler for USART Tx and Rx Complete Events.

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
void APP_CodecBufferEventHandler(DRV_CODEC_BUFFER_EVENT event,
        DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context );
uint32_t __attribute__((nomips16)) APP_ReadCoreTimer(void);
void GFX_MENU_DRAW(void);
void DisplayTasks(void);

void muteAudio ( void );
void syncLRClock( void );
void APP_TasksCheckStuck( void );

// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************
extern APP_DATA appData;
extern const USB_DEVICE_FUNCTION_REGISTRATION_TABLE funcRegistrationTable[];
extern const USB_DEVICE_MASTER_DESCRIPTOR usbMasterDescriptor;

void initData( void );

//APP: Button ICN interrupt handler
#include "btad_buttons.h"


#ifdef __cplusplus
}
#endif


#endif /* _APP_HEADER_H */

/*******************************************************************************
 End of File
*/

