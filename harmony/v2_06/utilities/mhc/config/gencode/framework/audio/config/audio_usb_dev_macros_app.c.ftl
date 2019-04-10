<#-- audio_usb_dev_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_audio_usb_dev_app_c_includes>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data
*/
-->
<#macro macro_audio_usb_dev_app_c_global_data>
I2S_DATA_16 __attribute__((coherent)) __attribute__((aligned(16))) rxBuffer[2][${APP_NAME?upper_case}_NO_OF_SAMPLES_IN_A_USB_FRAME];

${APP_NAME?upper_case}_DATA ${APP_NAME?lower_case}Data =
{
    /* Device Layer Handle  */
    .usbDevHandle = -1,

    /* USB Audio Instance index for this app object 0*/
    .audioInstance = 0,

     /* app state */
    .state = ${APP_NAME?upper_case}_STATE_INIT,

    /* device configured status */
    .isConfigured = false,

    /* Initialize active interface setting to 0. */
    .activeInterfaceAlternateSetting = ${APP_NAME?upper_case}_USB_SPEAKER_PLAYBACK_NONE,

    /* No Audio control in progress.*/
    .currentAudioControl = ${APP_NAME?upper_case}_USB_CONTROL_NONE
};
</#macro>

<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_audio_usb_dev_app_c_callback_functions>
void ${APP_NAME?upper_case}_USBDeviceEventHandler( USB_DEVICE_EVENT event, void * pEventData, uintptr_t context )
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

            break;

        case USB_DEVICE_EVENT_CONFIGURED:
            /* check the configuration */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED *)pEventData;
            if(configuredEventData->configurationValue == 1)
            {
                USB_DEVICE_AUDIO_EventHandlerSet
                (
                    0,
                    ${APP_NAME?upper_case}_USBDeviceAudioEventHandler ,
                    (uintptr_t)NULL
                );
                /* mark that set configuration is complete */
                ${APP_NAME?lower_case}Data.isConfigured = true;
            }
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_POWER_DETECTED:
            /* VBUS was detected. Notify USB stack about the event */
            USB_DEVICE_Attach (${APP_NAME?lower_case}Data.usbDevHandle);
            break;
        case USB_DEVICE_EVENT_POWER_REMOVED:
            /* VBUS was removed. Notify USB stack about the event*/
            USB_DEVICE_Detach (${APP_NAME?lower_case}Data.usbDevHandle);
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

void ${APP_NAME?upper_case}_USBDeviceAudioEventHandler
(
    USB_DEVICE_AUDIO_INDEX iAudio ,
    USB_DEVICE_AUDIO_EVENT event ,
    void * pData,
    uintptr_t context
)
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
                /* We have received a request from USB host to change the Interface-
                   Alternate setting.*/
                interfaceInfo = (USB_DEVICE_AUDIO_EVENT_DATA_INTERFACE_SETTING_CHANGED *)pData;
                ${APP_NAME?lower_case}Data.activeInterfaceAlternateSetting = interfaceInfo->interfaceAlternateSetting;
                ${APP_NAME?lower_case}Data.usbACDstate = ${APP_NAME?upper_case}_AUD_CLASS_DEV_USB_INTERFACE_ALTERNATE_SETTING_RCVD;
            break;

            case USB_DEVICE_AUDIO_EVENT_READ_COMPLETE:
                readEventData = (USB_DEVICE_AUDIO_EVENT_DATA_READ_COMPLETE *)pData;
                /* We have received an audio frame from the Host. */
                    if(readEventData->handle == ${APP_NAME?lower_case}Data.readTransferHandle1)
                    {
                        ${APP_NAME?lower_case}Data.isReadComplete1 = true;
                    }
                    else if(readEventData->handle == ${APP_NAME?lower_case}Data.readTransferHandle2)
                    {
                        ${APP_NAME?lower_case}Data.isReadComplete2 = true;
                    }
                    else
                    {
                        
                    }
                    ${APP_NAME?lower_case}Data.usbACDstate = ${APP_NAME?upper_case}_AUD_CLASS_DEV_PROCESS_DATA;
            break;

            case USB_DEVICE_AUDIO_EVENT_WRITE_COMPLETE:
            break;

            case USB_DEVICE_AUDIO_EVENT_CONTROL_SET_CUR:
                entityID = ((USB_AUDIO_CONTROL_INTERFACE_REQUEST*)pData)->entityID;
                if (entityID == APP_ID_FEATURE_UNIT)
                {
                   controlSelector = ((USB_AUDIO_FEATURE_UNIT_CONTROL_REQUEST*)pData)->controlSelector;
                   if (controlSelector == USB_AUDIO_MUTE_CONTROL)
                   {
                       /* A control write transfer received from Host. Now receive data from Host. */
                       USB_DEVICE_ControlReceive(${APP_NAME?lower_case}Data.usbDevHandle, (void *) &(${APP_NAME?lower_case}Data.mute), 1 );
                       ${APP_NAME?lower_case}Data.currentAudioControl = ${APP_NAME?upper_case}_USB_AUDIO_MUTE_CONTROL;
                   }
                }
                break;
            case USB_DEVICE_AUDIO_EVENT_CONTROL_GET_CUR:
                entityID = ((USB_AUDIO_CONTROL_INTERFACE_REQUEST*)pData)->entityID;
                if (entityID == APP_ID_FEATURE_UNIT)
                {
                   controlSelector = ((USB_AUDIO_FEATURE_UNIT_CONTROL_REQUEST*)pData)->controlSelector;
                   if (controlSelector == USB_AUDIO_MUTE_CONTROL)
                   {
                       /* Handle Get request*/
                       USB_DEVICE_ControlSend(${APP_NAME?lower_case}Data.usbDevHandle, (void *)&(${APP_NAME?lower_case}Data.mute), 1 );
                   }
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
                USB_DEVICE_ControlStatus (${APP_NAME?lower_case}Data.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            break;
            case USB_DEVICE_AUDIO_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:
                USB_DEVICE_ControlStatus(${APP_NAME?lower_case}Data.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK );
                if (${APP_NAME?lower_case}Data.currentAudioControl == ${APP_NAME?upper_case}_USB_AUDIO_MUTE_CONTROL)
                {
                    ${APP_NAME?lower_case}Data.usbACDstate = ${APP_NAME?upper_case}_AUD_CLASS_DEV_MUTE_AUDIO_PLAYBACK;
                    ${APP_NAME?lower_case}Data.currentAudioControl = ${APP_NAME?upper_case}_USB_CONTROL_NONE;
                    /* Handle Mute Control Here. */
                }
            break;
            case  USB_DEVICE_AUDIO_EVENT_CONTROL_TRANSFER_DATA_SENT:
            break;
            default:
                SYS_ASSERT ( false , "Invalid callback" );
            break;
        }
    }
}
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_audio_usb_dev_app_c_local_functions>
USB_DEVICE_AUDIO_RESULT audioErr1;

void ${APP_NAME?upper_case}_AUD_CLASS_DEV_Tasks (void )
{
    switch(${APP_NAME?lower_case}Data.usbACDstate)
    {
        case ${APP_NAME?upper_case}_AUD_CLASS_DEV_STATE_WAIT_FOR_CONFIGURATION:
            /* Check if Host has configured the Device. */
            if (${APP_NAME?lower_case}Data.isConfigured == true)
            {
                ${APP_NAME?lower_case}Data.usbACDstate = ${APP_NAME?upper_case}_AUD_CLASS_DEV_SUBMIT_INITIAL_READ_REQUEST;
            }
            break;

       case ${APP_NAME?upper_case}_AUD_CLASS_DEV_SUBMIT_INITIAL_READ_REQUEST:
       {
            if (${APP_NAME?lower_case}Data.activeInterfaceAlternateSetting == ${APP_NAME?upper_case}_USB_SPEAKER_PLAYBACK_STEREO_48KHZ)
            {
                audioErr1 = USB_DEVICE_AUDIO_Read ( USB_DEVICE_INDEX_0 , &${APP_NAME?lower_case}Data.readTransferHandle1, 1 , rxBuffer[0], 192 );
                audioErr1 = USB_DEVICE_AUDIO_Read ( USB_DEVICE_INDEX_0 , &${APP_NAME?lower_case}Data.readTransferHandle2, 1 , rxBuffer[1], 192 );
                ${APP_NAME?lower_case}Data.usbACDstate = ${APP_NAME?upper_case}_AUD_CLASS_DEV_PROCESS_DATA;
            }
            
            break;
       }

        case ${APP_NAME?upper_case}_AUD_CLASS_DEV_PROCESS_DATA:
        {
            if(${APP_NAME?lower_case}Data.isReadComplete1 == true)
            {
                ${APP_NAME?lower_case}Data.isReadComplete1 = false;
                /* submit write to codec */
                ${APP_NAME?lower_case}Data.readTransferHandle1 = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;
                audioErr1 = USB_DEVICE_AUDIO_Read ( USB_DEVICE_INDEX_0 , &${APP_NAME?lower_case}Data.readTransferHandle1, 1 , rxBuffer[0], 192 );
            }
            else if(${APP_NAME?lower_case}Data.isReadComplete2 == true)
            {
                ${APP_NAME?lower_case}Data.isReadComplete2 = false;
                /* submit write to codec */
                ${APP_NAME?lower_case}Data.readTransferHandle2 = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;
                audioErr1 = USB_DEVICE_AUDIO_Read ( USB_DEVICE_INDEX_0 , &${APP_NAME?lower_case}Data.readTransferHandle2, 1 , rxBuffer[1], 192 );
            }
            break;
        }

        case ${APP_NAME?upper_case}_AUD_CLASS_DEV_MUTE_AUDIO_PLAYBACK:
        {
            if (${APP_NAME?lower_case}Data.activeInterfaceAlternateSetting == 0)
            {
                /* Send mute command to codec */
                ${APP_NAME?lower_case}Data.usbACDstate = ${APP_NAME?upper_case}_AUD_CLASS_DEV_IDLE;
            }
            else if (${APP_NAME?lower_case}Data.activeInterfaceAlternateSetting == 1)
            {
                /* Send unmute command to codec */
                ${APP_NAME?lower_case}Data.usbACDstate = ${APP_NAME?upper_case}_AUD_CLASS_DEV_SUBMIT_INITIAL_READ_REQUEST;
            }
            break;
        }

        case ${APP_NAME?upper_case}_AUD_CLASS_DEV_USB_INTERFACE_ALTERNATE_SETTING_RCVD:
        {
            ${APP_NAME?lower_case}Data.usbACDstate =  ${APP_NAME?upper_case}_AUD_CLASS_DEV_SUBMIT_INITIAL_READ_REQUEST;
            
            break;
        }

        case ${APP_NAME?upper_case}_AUD_CLASS_DEV_IDLE:

        case ${APP_NAME?upper_case}_AUD_CLASS_DEV_STATE_ERROR:
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
</#macro>

<#--
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Initialize ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_INIT;
-->
<#macro macro_audio_usb_dev_app_c_initialize>
    ${APP_NAME?lower_case}Data.usbACDstate = ${APP_NAME?upper_case}_AUD_CLASS_DEV_STATE_WAIT_FOR_CONFIGURATION;
    
    ${APP_NAME?lower_case}Data.usbDevHandle = USB_DEVICE_HANDLE_INVALID;
    
    ${APP_NAME?lower_case}Data.isReadComplete1 = false;
    ${APP_NAME?lower_case}Data.isReadComplete2 = false;

    ${APP_NAME?lower_case}Data.readTransferHandle1 = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;
    ${APP_NAME?lower_case}Data.readTransferHandle2 = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;
</#macro>

<#--
}


/******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Tasks ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Tasks ( void )
{
-->
<#macro macro_audio_usb_dev_app_c_tasks_data>
</#macro>

<#--
    /* Check the application's current state. */
    switch ( ${APP_NAME?lower_case}Data.state )
    {
        /* Application's initial state. */
        case ${APP_NAME?upper_case}_STATE_INIT:
        {
            bool appInitialized = true;
-->   
<#macro macro_audio_usb_dev_app_c_tasks_state_init>
			if(${APP_NAME?lower_case}Data.usbDevHandle == USB_DEVICE_HANDLE_INVALID)
			{
            	${APP_NAME?lower_case}Data.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,
							               	     DRV_IO_INTENT_READWRITE );
                appInitialized = false;
		 	}
            else
            {
				USB_DEVICE_EventHandlerSet(${APP_NAME?lower_case}Data.usbDevHandle, ${APP_NAME?upper_case}_USBDeviceEventHandler, 0);
            }
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_audio_usb_dev_app_c_tasks_calls_after_init>
</#macro>

<#--            /* Advance to the next state */
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_audio_usb_dev_app_c_tasks_state_service_tasks>
			${APP_NAME?upper_case}_AUD_CLASS_DEV_Tasks();
</#macro>

<#--        
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
-->

<#macro macro_audio_usb_dev_app_c_tasks_states>
</#macro>
