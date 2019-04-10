<#-- audio_usb_dev_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_audio_usb_dev_app_h_includes>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_audio_usb_dev_app_h_type_definitions>
#define ${APP_NAME?upper_case}_NO_OF_SAMPLES_IN_A_USB_FRAME 48
#define APP_ID_FEATURE_UNIT    0x05
#define USB_DEVICE_AUDIO_CONTROL_INTERFACE_ID           0x00
#define APP_ID_INPUT_TERMINAL  0x01
#define APP_ID_OUTPUT_TERMINAL 0x02
#define USB_DEVICE_AUDIO_STREAMING_INTERFACE_ID_1       0x01

typedef struct _I2S_DATA_16
{
    int16_t leftData;
    int16_t rightData;
} I2S_DATA_16;

typedef enum
{
    ${APP_NAME?upper_case}_USB_CONTROL_NONE,
    ${APP_NAME?upper_case}_USB_AUDIO_MUTE_CONTROL
} ${APP_NAME?upper_case}_AUDIO_CONTROLS;

typedef enum
{
    ${APP_NAME?upper_case}_USB_SPEAKER_PLAYBACK_NONE  = 0,
    ${APP_NAME?upper_case}_USB_SPEAKER_PLAYBACK_STEREO_48KHZ = 1
} ${APP_NAME?upper_case}_ALTERNATE_SETTINGS;

typedef enum
{
    ${APP_NAME?upper_case}_AUD_CLASS_DEV_SUBMIT_INITIAL_READ_REQUEST,
    ${APP_NAME?upper_case}_AUD_CLASS_DEV_PROCESS_DATA,
    ${APP_NAME?upper_case}_AUD_CLASS_DEV_IDLE,
    ${APP_NAME?upper_case}_AUD_CLASS_DEV_MUTE_AUDIO_PLAYBACK,
    ${APP_NAME?upper_case}_AUD_CLASS_DEV_USB_INTERFACE_ALTERNATE_SETTING_RCVD,
    ${APP_NAME?upper_case}_AUD_CLASS_DEV_STATE_WAIT_FOR_CONFIGURATION,
    ${APP_NAME?upper_case}_AUD_CLASS_DEV_STATE_ERROR

} ${APP_NAME?upper_case}_AUD_CLASS_DEV_STATES;
</#macro>


<#--
// *****************************************************************************
/* Application Data

typedef struct
{
    /* The application's current state */
    ${APP_NAME?upper_case}_STATES state;

    /* TODO: Define any additional data used by the application. */
-->
<#macro macro_audio_usb_dev_app_h_data>
    volatile ${APP_NAME?upper_case}_AUD_CLASS_DEV_STATES usbACDstate;

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
    ${APP_NAME?upper_case}_ALTERNATE_SETTINGS activeInterfaceAlternateSetting;

    /* Holds current value of Audio Control Mute settings. A value True indicates
     * audio is currently muted. */
     bool mute;

    /* This is to indicate which Audio Control Transfer is currently
     * in progress. */
    ${APP_NAME?upper_case}_AUDIO_CONTROLS currentAudioControl;
</#macro>
<#--
} ${APP_NAME?upper_case}_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
-->
<#macro macro_audio_usb_dev_app_h_callback_function_declarations>
void ${APP_NAME?upper_case}_USBDeviceEventHandler
(
    USB_DEVICE_EVENT events,
    void * eventData,
    uintptr_t context
);

void ${APP_NAME?upper_case}_USBDeviceAudioEventHandler
(
    USB_DEVICE_AUDIO_INDEX iAudio ,
    USB_DEVICE_AUDIO_EVENT event ,
    void * pData,
    uintptr_t context
);
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_audio_usb_dev_app_h_function_declarations>
</#macro>

<#macro macro_audio_usb_dev_app_h_states>
</#macro>

