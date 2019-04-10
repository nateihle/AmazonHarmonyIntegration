/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_usb_audio_task.c

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
// DOM-IGNORE-END

#include "app_usb_audio_task.h"

DRV_I2S_DATA16 APP_USB_AUDIO_MAKE_BUFFER_DMA_READY 
    rxBuffer[APP_USB_AUDIO_QUEUING_DEPTH][APP_USB_AUDIO_NO_OF_SAMPLES_IN_A_USB_FRAME];
USB_DEVICE_AUDIO_RESULT AppUsbAudioError;
APP_USB_AUDIO_DATA AppUsbAudioData;
void APP_USB_AUDIO_SubmitBufferToCodec(void);

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************
void APP_USB_AUDIO_DeviceEventHandler( USB_DEVICE_EVENT event, void * pEventData, 
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
        break;

        case USB_DEVICE_EVENT_CONFIGURED:
            /* check the configuration */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED *)pEventData;
            if(configuredEventData->configurationValue == 1)
            {
                USB_DEVICE_AUDIO_EventHandlerSet
                (
                    0,
                    APP_USB_AUDIO_DeviceAudioEventHandler ,
                    (uintptr_t)NULL
                );
                /* mark that set configuration is complete */
                AppUsbAudioData.isConfigured = true;
            }
        break;

        case USB_DEVICE_EVENT_SUSPENDED:
        break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_POWER_DETECTED:
            /* VBUS was detected. Notify USB stack about the event */
            USB_DEVICE_Attach (AppUsbAudioData.usbDevHandle);
        break;
        
        case USB_DEVICE_EVENT_POWER_REMOVED:
            /* VBUS was removed. Notify USB stack about the event*/
            USB_DEVICE_Detach (AppUsbAudioData.usbDevHandle);            
        case USB_DEVICE_EVENT_ERROR:
        default:
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////
void APP_USB_AUDIO_DeviceAudioEventHandler
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
                AppUsbAudioData.activeInterfaceAlternateSetting = interfaceInfo->interfaceAlternateSetting;
                AppUsbAudioData.state = APP_USB_AUDIO_STATE_USB_INTERFACE_ALTERNATE_SETTING_RCVD;
            break;

            case USB_DEVICE_AUDIO_EVENT_READ_COMPLETE:
                readEventData = (USB_DEVICE_AUDIO_EVENT_DATA_READ_COMPLETE *)pData;
                //We have received an audio frame from the Host.
                //Now send this audio frame to Audio Codec for Playback.
                    if(readEventData->handle == AppUsbAudioData.readTransferHandle1)
                    {
                        AppUsbAudioData.isReadComplete1 = true;
                    }
                    else if(readEventData->handle == AppUsbAudioData.readTransferHandle2)
                    {
                        AppUsbAudioData.isReadComplete2 = true;
                    }
                    else
                    {
                        
                    }
                    
                    AppUsbAudioData.state = APP_USB_AUDIO_STATE_PROCESS_DATA;
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
                       //A control write transfer received from Host. Now receive data from Host.
                       USB_DEVICE_ControlReceive(AppUsbAudioData.usbDevHandle, (void *) &(AppUsbAudioData.dacMute), 1 );
                       AppUsbAudioData.currentAudioControl = APP_USB_AUDIO_MUTE_CONTROL;
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
                       /*Handle Get request*/
                       USB_DEVICE_ControlSend(AppUsbAudioData.usbDevHandle, (void *)&(AppUsbAudioData.dacMute), 1 );
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
                USB_DEVICE_ControlStatus (AppUsbAudioData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_ERROR);
            break;
            
            case USB_DEVICE_AUDIO_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:
                USB_DEVICE_ControlStatus(AppUsbAudioData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK );
                if (AppUsbAudioData.currentAudioControl == APP_USB_AUDIO_MUTE_CONTROL)
                {
                    AppUsbAudioData.state = APP_USB_AUDIO_STATE_MUTE_AUDIO_PLAYBACK;
                    AppUsbAudioData.currentAudioControl = APP_USB_AUDIO_CONTROL_NONE;
                    //Handle Mute Control Here.
                }
            break;
            
            case  USB_DEVICE_AUDIO_EVENT_CONTROL_TRANSFER_DATA_SENT:
            break;
            
            default:
                SYS_ASSERT ( false , "Invalid callback" );
            break;
        } //end of switch ( callback )
    }//end of if  if ( iAudio == 0 )
}//end of function APP_AudioEventCallback

////////////////////////////////////////////////////////////////////////////////
/******************************************************
 * Application Initialize. It is
 * called from the SYS_Initialized() function.
 ******************************************************/
void APP_USB_AUDIO_Initialize (void)
{        
    /* Device Layer Handle  */
    AppUsbAudioData.usbDevHandle = -1;
    /* USB Audio Instance index for this app object 0*/
    AppUsbAudioData.audioInstance = 0;
     /* app state */
    AppUsbAudioData.state = APP_USB_AUDIO_STATE_CODEC_OPEN;
    /* device configured status */
    AppUsbAudioData.isConfigured = false;
    /* Initialize active interface setting to 0. */
    AppUsbAudioData.activeInterfaceAlternateSetting = APP_USB_AUDIO_ALTERNATE_SETTINGS_SPEAKER_PLAYBACK_NONE;
    /* DAC is not muted initially */
    AppUsbAudioData.dacMute = false;
    AppUsbAudioData.codec.handle = DRV_HANDLE_INVALID;
    AppUsbAudioData.codec.context = (uintptr_t)0;
    AppUsbAudioData.codec.bufferHandler = (DRV_CODEC_BUFFER_EVENT_HANDLER) APP_USB_AUDIO_BufferEventHandler;
    AppUsbAudioData.codec.bufferSize = sizeof(rxBuffer[0]);
    /* No Audio control in progress.*/
    AppUsbAudioData.currentAudioControl = APP_USB_AUDIO_CONTROL_NONE;
            
    
    AppUsbAudioData.isReadComplete1 = false;
    AppUsbAudioData.isReadComplete2 = false;

    AppUsbAudioData.readTransferHandle1 = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;
    AppUsbAudioData.readTransferHandle2 = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;

    AppUsbAudioData.codec.txbufferObject1 = (uint8_t *) rxBuffer[0];
    AppUsbAudioData.codec.txbufferObject2 = (uint8_t *) rxBuffer[1];

    AppUsbAudioData.codec.iswriteBufHandleAvailable1 = false;
    AppUsbAudioData.codec.iswriteBufHandleAvailable2 = false;
    
    AppUsbAudioData.codec.handle = DRV_HANDLE_INVALID;
    AppUsbAudioData.usbDevHandle = USB_DEVICE_HANDLE_INVALID;
    
    AppUsbAudioData.isUsbDetached = false;
}

////////////////////////////////////////////////////////////////////////////////
void APP_USB_AUDIO_ReInitialize (void)
{        
    /* app state */
    AppUsbAudioData.state = APP_USB_AUDIO_STATE_CODEC_OPEN;

    AppUsbAudioData.codec.handle = DRV_HANDLE_INVALID;
    AppUsbAudioData.codec.context = (uintptr_t)0;
    AppUsbAudioData.codec.bufferHandler = (DRV_CODEC_BUFFER_EVENT_HANDLER) APP_USB_AUDIO_BufferEventHandler;
    AppUsbAudioData.codec.bufferSize = sizeof(rxBuffer[0]);
    /* No Audio control in progress.*/
    AppUsbAudioData.currentAudioControl = APP_USB_AUDIO_CONTROL_NONE;
               
    AppUsbAudioData.isReadComplete1 = false;
    AppUsbAudioData.isReadComplete2 = false;

    AppUsbAudioData.readTransferHandle1 = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;
    AppUsbAudioData.readTransferHandle2 = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;

    AppUsbAudioData.codec.txbufferObject1 = (uint8_t *) rxBuffer[0];
    AppUsbAudioData.codec.txbufferObject2 = (uint8_t *) rxBuffer[1];

    AppUsbAudioData.codec.iswriteBufHandleAvailable1 = false;
    AppUsbAudioData.codec.iswriteBufHandleAvailable2 = false;
}

////////////////////////////////////////////////////////////////////////////////
void APP_USB_AUDIO_RegisterStreamSource(APP_USB_AUDIO_STREAM_SOURCE streamFunction)
{
    AppUsbAudioData.streamSource = streamFunction;
}

////////////////////////////////////////////////////////////////////////////////
/**********************************************************
 * Application tasks routine. This function implements the
 * application state machine.
 ***********************************************************/
void APP_USB_AUDIO_Tasks (void )
{ 
    if(AppUsbAudioData.streamSource() == 1)
    {
        switch(AppUsbAudioData.state)
        {
            /* Application's initial state. */
            case APP_USB_AUDIO_STATE_INIT:
            /* Open the device layer */
                if(AppUsbAudioData.usbDevHandle == USB_DEVICE_HANDLE_INVALID)
                {
                    AppUsbAudioData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,
                            DRV_IO_INTENT_READWRITE );
                    if(AppUsbAudioData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
                    {
                        /* Register a callback with device layer to get event notification (for end point 0) */
                        USB_DEVICE_EventHandlerSet(AppUsbAudioData.usbDevHandle, APP_USB_AUDIO_DeviceEventHandler, 0);
                        AppUsbAudioData.state = APP_USB_AUDIO_STATE_WAIT_FOR_CONFIGURATION;
                    }
                    else
                    {
                        /* The Device Layer is not ready to be opened. We should try
                         * again later. */
                    }
                }
            break;

            case APP_USB_AUDIO_STATE_WAIT_FOR_CONFIGURATION:
                //Check if Host has configured the Device.                                                
                if (AppUsbAudioData.isConfigured == true)
                {
                    AppUsbAudioData.state = APP_USB_AUDIO_STATE_IDLE;
                }
            break;

            case APP_USB_AUDIO_STATE_CODEC_OPEN:
            {
                SYS_STATUS codecStatus;
                if (AppUsbAudioData.codec.handle == DRV_HANDLE_INVALID)
                {
                    codecStatus = DRV_CODEC_Status(sysObjdrvCodec0);
                    if (SYS_STATUS_READY == codecStatus)
                    {
                        // This means the driver can now be be opened.
                        /* A client opens the driver object to get an Handle */
                        AppUsbAudioData.codec.handle = DRV_CODEC_Open(DRV_CODEC_INDEX_0, DRV_IO_INTENT_WRITE);
                        if(AppUsbAudioData.codec.handle != DRV_HANDLE_INVALID)
                        {
                            DRV_CODEC_SamplingRateSet(AppUsbAudioData.codec.handle, DRV_AK4953_AUDIO_SAMPLING_RATE);
                            
                            DRV_CODEC_BufferEventHandlerSet(AppUsbAudioData.codec.handle,
                                AppUsbAudioData.codec.bufferHandler,
                                AppUsbAudioData.codec.context);
                            
                            AppUsbAudioData.state = APP_USB_AUDIO_STATE_INIT;
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
                else
                {
                    AppUsbAudioData.state = APP_USB_AUDIO_STATE_INIT;
                }
            }
            break;

           case APP_USB_AUDIO_STATE_SUBMIT_INITIAL_READ_REQUEST:
                if (AppUsbAudioData.activeInterfaceAlternateSetting == APP_USB_AUDIO_ALTERNATE_SETTINGS_SPEAKER_PLAYBACK_STEREO_48KHZ)
                {
                    AppUsbAudioError = USB_DEVICE_AUDIO_Read ( USB_DEVICE_INDEX_0 , &AppUsbAudioData.readTransferHandle1, 1 , rxBuffer[0], 192 );
                    AppUsbAudioError = USB_DEVICE_AUDIO_Read ( USB_DEVICE_INDEX_0 , &AppUsbAudioData.readTransferHandle2, 1 , rxBuffer[1], 192 );
                    AppUsbAudioData.state = APP_USB_AUDIO_STATE_PROCESS_DATA;
                }
                else
                {
                }
            break;

           case APP_USB_AUDIO_STATE_PROCESS_DATA:
           {

                /* Submit write to the CODEC driver */
                if(AppUsbAudioData.isReadComplete1 == true)
                {                    
                    AppUsbAudioData.isReadComplete1 = false;
                    AppUsbAudioData.codec.writeBufHandle1 = DRV_CODEC_BUFFER_HANDLE_INVALID;
                    DRV_CODEC_BufferAddWrite(AppUsbAudioData.codec.handle, &AppUsbAudioData.codec.writeBufHandle1,
                    AppUsbAudioData.codec.txbufferObject1, AppUsbAudioData.codec.bufferSize);
                }
                else if(AppUsbAudioData.isReadComplete2 == true)
                {                    
                    AppUsbAudioData.isReadComplete2 = false;
                    AppUsbAudioData.codec.writeBufHandle2 = DRV_CODEC_BUFFER_HANDLE_INVALID;
                    DRV_CODEC_BufferAddWrite(AppUsbAudioData.codec.handle, &AppUsbAudioData.codec.writeBufHandle2,
                    AppUsbAudioData.codec.txbufferObject2, AppUsbAudioData.codec.bufferSize);
                }
                else
                {               
                    ;
                }

                /* Submit a read request from USB */
                if (AppUsbAudioData.codec.iswriteBufHandleAvailable1 == true)
                {
                    AppUsbAudioData.codec.iswriteBufHandleAvailable1 = false;
                    AppUsbAudioData.readTransferHandle1 = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;
                    AppUsbAudioError = USB_DEVICE_AUDIO_Read ( USB_DEVICE_INDEX_0 , &AppUsbAudioData.readTransferHandle1, 1 , rxBuffer[0], 192 );
                }
                else if(AppUsbAudioData.codec.iswriteBufHandleAvailable2 == true)
                {
                    AppUsbAudioData.codec.iswriteBufHandleAvailable2 = false;
                    AppUsbAudioData.readTransferHandle2 = USB_DEVICE_AUDIO_TRANSFER_HANDLE_INVALID;
                    AppUsbAudioError = USB_DEVICE_AUDIO_Read ( USB_DEVICE_INDEX_0 , &AppUsbAudioData.readTransferHandle2, 1 , rxBuffer[1], 192 );
                }
                else
                {
                    ;
                }

                if( AppUsbAudioError != USB_DEVICE_AUDIO_RESULT_OK || 
                    AppUsbAudioData.codec.writeBufHandle1 == DRV_CODEC_BUFFER_HANDLE_INVALID ||
                    AppUsbAudioData.codec.writeBufHandle2 == DRV_CODEC_BUFFER_HANDLE_INVALID )
                {                    
                    //Handle error condition..
                }  

           }
           break;
                        
            case APP_USB_AUDIO_STATE_MUTE_AUDIO_PLAYBACK:
            {
                if (AppUsbAudioData.activeInterfaceAlternateSetting == 0)
                {
                    DRV_CODEC_MuteOn(AppUsbAudioData.codec.handle);
                    AppUsbAudioData.state = APP_USB_AUDIO_STATE_IDLE;
                }
                else if (AppUsbAudioData.activeInterfaceAlternateSetting == 1)
                {
                    DRV_CODEC_MuteOff(AppUsbAudioData.codec.handle);
                    AppUsbAudioData.state = APP_USB_AUDIO_STATE_SUBMIT_INITIAL_READ_REQUEST;
                }
            }
            break;

            case APP_USB_AUDIO_STATE_USB_INTERFACE_ALTERNATE_SETTING_RCVD:
            {
               SYS_STATUS codecStatus;

               if (AppUsbAudioData.activeInterfaceAlternateSetting == APP_USB_AUDIO_ALTERNATE_SETTINGS_SPEAKER_PLAYBACK_NONE)
                {                   
                   codecStatus = DRV_CODEC_Status(sysObjdrvCodec0);
                   if (SYS_STATUS_READY == codecStatus)
                    {
                       DRV_CODEC_MuteOn(AppUsbAudioData.codec.handle);
                       AppUsbAudioData.state = APP_USB_AUDIO_STATE_IDLE;
                    }
                }
                else if(AppUsbAudioData.activeInterfaceAlternateSetting == APP_USB_AUDIO_ALTERNATE_SETTINGS_SPEAKER_PLAYBACK_STEREO_48KHZ)
                {
                   codecStatus = DRV_CODEC_Status(sysObjdrvCodec0);
                   if (SYS_STATUS_READY == codecStatus)
               {
                       DRV_CODEC_MuteOff(AppUsbAudioData.codec.handle);
                       AppUsbAudioData.state =  APP_USB_AUDIO_STATE_SUBMIT_INITIAL_READ_REQUEST;
               }
                }
            }
            break;

            case APP_USB_AUDIO_STATE_IDLE:
            {
                if (AppUsbAudioData.activeInterfaceAlternateSetting == APP_USB_AUDIO_ALTERNATE_SETTINGS_SPEAKER_PLAYBACK_NONE)
                {
                    DRV_CODEC_MuteOn(AppUsbAudioData.codec.handle);
                }
            }
            break;

            case APP_USB_AUDIO_STATE_ERROR:
            /* The default state should never be executed. */
            default:
            {
                /* TODO: Handle error in application's state machine. */
                break;
            }
        }
    }
    else
    {
        /* Do Nothing */
        ;
    }
}

////////////////////////////////////////////////////////////////////////////////
void APP_USB_AUDIO_SuspendStreaming(void)
{  
    DRV_CODEC_Close(AppUsbAudioData.codec.handle); 
    AppUsbAudioData.codec.handle = DRV_HANDLE_INVALID;
    USB_DEVICE_Detach(AppUsbAudioData.usbDevHandle); 
    AppUsbAudioData.isUsbDetached = true;            
                    
}

////////////////////////////////////////////////////////////////////////////////
void APP_USB_AUDIO_AttachDevice(void)
{
    if(AppUsbAudioData.isUsbDetached)
    {
        APP_USB_AUDIO_ReInitialize();         
        USB_DEVICE_Attach(AppUsbAudioData.usbDevHandle);
        AppUsbAudioData.isUsbDetached = false;
    }
}

////////////////////////////////////////////////////////////////////////////////
void APP_USB_AUDIO_MuteOn(void)
{
    if (AppUsbAudioData.codec.handle != DRV_HANDLE_INVALID)
    {
        DRV_CODEC_MuteOn(AppUsbAudioData.codec.handle);    
    }
}

////////////////////////////////////////////////////////////////////////////////
void APP_USB_AUDIO_MuteOff(void)
{
    if (AppUsbAudioData.codec.handle != DRV_HANDLE_INVALID)
    {
        DRV_CODEC_MuteOff(AppUsbAudioData.codec.handle);        
    }
}

////////////////////////////////////////////////////////////////////////////////
uint8_t APP_USB_AUDIO_VolumeGet(void)
{
    if (AppUsbAudioData.codec.handle != DRV_HANDLE_INVALID)
    {
        return DRV_CODEC_VolumeGet(AppUsbAudioData.codec.handle, DRV_CODEC_CHANNEL_LEFT_RIGHT);         
    }
    else
    {
        return 0;
    }
}

////////////////////////////////////////////////////////////////////////////////
void APP_USB_AUDIO_VolumeSet(uint8_t volume)
{
    if (AppUsbAudioData.codec.handle != DRV_HANDLE_INVALID)
    {
        DRV_CODEC_VolumeSet(AppUsbAudioData.codec.handle, DRV_CODEC_CHANNEL_LEFT_RIGHT, volume);         
    }
}

////////////////////////////////////////////////////////////////////////////////
bool APP_USB_AUDIO_isUSBDettached(void)
{
    return AppUsbAudioData.isUsbDetached;
}

////////////////////////////////////////////////////////////////////////////////
/**********************************************************
 * Application CODEC buffer Event handler.
 * This function is called back by the CODEC driver when
 * a CODEC data buffer TX completes.
 ***********************************************************/
void APP_USB_AUDIO_BufferEventHandler(DRV_CODEC_BUFFER_EVENT event,
        DRV_CODEC_BUFFER_HANDLE handle, uintptr_t context )
{
    switch(event)
    {
        case DRV_CODEC_BUFFER_EVENT_COMPLETE:
        {
            if(AppUsbAudioData.codec.writeBufHandle1 == handle)
            {
                AppUsbAudioData.codec.iswriteBufHandleAvailable1 = true;
            }
            else if(AppUsbAudioData.codec.writeBufHandle2 == handle)
            {
                AppUsbAudioData.codec.iswriteBufHandleAvailable2 = true;
            }
            else
            {

            }                        
            
            AppUsbAudioData.state = APP_USB_AUDIO_STATE_PROCESS_DATA;
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

/*******************************************************************************
 End of File
 */