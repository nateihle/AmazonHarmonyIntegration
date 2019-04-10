<#-- usb_dev_cdc_basic_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_lib_usb_app_c_includes>
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
<#macro macro_lib_usb_app_c_global_data>
</#macro>

<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_lib_usb_app_c_callback_functions>


/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE ${APP_NAME?upper_case}_USBDeviceCDCEventHandler
(
    USB_DEVICE_CDC_INDEX index ,
    USB_DEVICE_CDC_EVENT event ,
    void * pData,
    uintptr_t userData
)
{
    ${APP_NAME?upper_case}_DATA * appDataObject;
    appDataObject = (${APP_NAME?upper_case}_DATA *)userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;
	uint16_t * breakData;
	
    switch ( event )
    {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->handleUsbDevice, &appDataObject->appCOMPortObjects[index].getLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->handleUsbDevice, &appDataObject->appCOMPortObjects[index].setLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *)pData;
            appDataObject->appCOMPortObjects[index].controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->appCOMPortObjects[index].controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->handleUsbDevice, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent.  */

            breakData = (uint16_t *)pData;
            appDataObject->appCOMPortObjects[index].breakData = *breakData;
            
            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->handleUsbDevice, USB_DEVICE_CONTROL_STATUS_OK);
            break;
			
		case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last CDC  control (write) transfer is
             * complete. For now we accept all the data and use the
             * USB_DEVICE_ControlStatus function to complete the Status Stage of
             * the control transfer. */

            USB_DEVICE_ControlStatus(appDataObject->handleUsbDevice, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means that a CDC related Control Read transfer has
             * completed. This is an indication only event. The application can
             * update it's flags or status to indicate the data has been sent to
             * the host. */

            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data. This event occurs in
             * response to USB_DEVICE_CDC_Read function. */

            break;


        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the host has sent some data. This event occurs in
             * response to the USB_DEVICE_CDC_Write function. */

            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void ${APP_NAME?upper_case}_USBDeviceEventHandler ( USB_DEVICE_EVENT event, void * eventData, uintptr_t context )
{
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch ( event )
    {
        case USB_DEVICE_EVENT_SOF:
            break;

        case USB_DEVICE_EVENT_RESET:

            ${APP_NAME?lower_case}Data.usbDeviceIsConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuration. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData;
            if ( configuredEventData->configurationValue == 1)
            {
                /* Register the CDC Device application event handler here.
                 * Note how the ${APP_NAME?lower_case}Data object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, ${APP_NAME?upper_case}_USBDeviceCDCEventHandler, (uintptr_t)&${APP_NAME?lower_case}Data);
				
				<#if ("${CONFIG_USB_DEV_CDC_BASIC_NUM_PORTS0}")?eval?number = 2> 
				USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_1, ${APP_NAME?upper_case}_USBDeviceCDCEventHandler, (uintptr_t)&${APP_NAME?lower_case}Data);				
				</#if>

                /* Mark that the device is now configured */
                ${APP_NAME?lower_case}Data.usbDeviceIsConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(${APP_NAME?lower_case}Data.handleUsbDevice);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(${APP_NAME?lower_case}Data.handleUsbDevice);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_lib_usb_app_c_local_functions>
/******************************************************************************
  Function:
    static void USB_Task (void)
    
   Remarks:
    Schedule Data transfers here.  
*/
static void USB_Task (void)
{
    if(${APP_NAME?lower_case}Data.usbDeviceIsConfigured)
    {
		/* Write USB CDC Application Logic here. Note that this function is
         * being called periodically the APP_Tasks() function. The application
         * logic should be implemented as state machine. It should not block */
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
<#macro macro_lib_usb_app_c_initialize>

    /* Device Layer Handle  */
    ${APP_NAME?lower_case}Data.handleUsbDevice = USB_DEVICE_HANDLE_INVALID ;

    /* Device configured status */
    ${APP_NAME?lower_case}Data.usbDeviceIsConfigured = false;

    /* Initial get line coding state */
    ${APP_NAME?lower_case}Data.appCOMPortObjects[USB_DEVICE_CDC_INDEX_0].getLineCodingData.dwDTERate   = 9600;
    ${APP_NAME?lower_case}Data.appCOMPortObjects[USB_DEVICE_CDC_INDEX_0].getLineCodingData.bParityType =  0;
    ${APP_NAME?lower_case}Data.appCOMPortObjects[USB_DEVICE_CDC_INDEX_0].getLineCodingData.bParityType = 0;
    ${APP_NAME?lower_case}Data.appCOMPortObjects[USB_DEVICE_CDC_INDEX_0].getLineCodingData.bDataBits   = 8;
	
	<#if ("${CONFIG_USB_DEV_CDC_BASIC_NUM_PORTS0}")?eval?number = 2>  
	${APP_NAME?lower_case}Data.appCOMPortObjects[USB_DEVICE_CDC_INDEX_1].getLineCodingData.dwDTERate   = 9600;
    ${APP_NAME?lower_case}Data.appCOMPortObjects[USB_DEVICE_CDC_INDEX_1].getLineCodingData.bParityType =  0;
    ${APP_NAME?lower_case}Data.appCOMPortObjects[USB_DEVICE_CDC_INDEX_1].getLineCodingData.bParityType = 0;
    ${APP_NAME?lower_case}Data.appCOMPortObjects[USB_DEVICE_CDC_INDEX_1].getLineCodingData.bDataBits   = 8;
	</#if>
	
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
<#macro macro_lib_usb_app_c_tasks_data>
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
<#macro macro_lib_usb_app_c_tasks_state_init>

            /* Open the device layer */
            if (${APP_NAME?lower_case}Data.handleUsbDevice == USB_DEVICE_HANDLE_INVALID)
            {
                ${APP_NAME?lower_case}Data.handleUsbDevice = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );
                if(appData.handleUsbDevice != USB_DEVICE_HANDLE_INVALID)
                {
                    appInitialized = true;
                }
                else
                {
                    appInitialized = false;
                }
            }
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_lib_usb_app_c_tasks_calls_after_init>

                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(${APP_NAME?lower_case}Data.handleUsbDevice, ${APP_NAME?upper_case}_USBDeviceEventHandler, 0);
</#macro>

<#--            /* Advance to the next state */
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_lib_usb_app_c_tasks_state_service_tasks>
<#if ("CONFIG_USB_DEV_CDC_BASIC" + "${HCONFIG_APP_INSTANCE}")?eval>            
            USB_Task();
</#if>
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

<#macro macro_lib_usb_app_c_tasks_states>
</#macro>
