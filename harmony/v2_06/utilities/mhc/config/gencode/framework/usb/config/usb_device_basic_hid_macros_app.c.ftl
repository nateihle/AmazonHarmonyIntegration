<#-- usb_device_basic_hid_macros_app.c.ftl -->

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

/**************************************************************
 * USB Device HID Events - Application Event Handler.
 **************************************************************/

static void APP_USBDeviceHIDEventHandler
(
    USB_DEVICE_HID_INDEX hidInstance,
    USB_DEVICE_HID_EVENT event, 
    void * eventData, 
    uintptr_t userData
)
{
    ${APP_NAME?upper_case}_DATA * ${APP_NAME?lower_case}Data = (${APP_NAME?upper_case}_DATA *)userData;

    switch(event)
    {
        case USB_DEVICE_HID_EVENT_REPORT_SENT:

            /* This means a Report has been sent.  We are free to send next
             * report. An application flag can be updated here. */

            break;

        case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:

            /* This means Report has been received from the Host. Report
             * received can be over Interrupt OUT or Control endpoint based on
             * Interrupt OUT endpoint availability. An application flag can be
             * updated here. */

            break;


        case USB_DEVICE_HID_EVENT_SET_IDLE:

            /* Save Idle rate received from Host */
            ${APP_NAME?lower_case}Data->idleRate = ((USB_DEVICE_HID_EVENT_DATA_SET_IDLE*)eventData)->duration;

            /* Acknowledge the Control Write Transfer */
            USB_DEVICE_ControlStatus(${APP_NAME?lower_case}Data->handleUsbDevice, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_HID_EVENT_GET_IDLE:

            /* Host is requesting for Idle rate. Now send the Idle rate */
            USB_DEVICE_ControlSend(${APP_NAME?lower_case}Data->handleUsbDevice, &(${APP_NAME?lower_case}Data->idleRate),1);

            /* On successfully receiving Idle rate, the Host would acknowledge
             * back with a Zero Length packet. The HID function driver returns
             * an event USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the
             * application upon receiving this Zero Length packet from Host.
             * USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates
             * this control transfer event is complete */

            break;

        case USB_DEVICE_HID_EVENT_SET_PROTOCOL:

            /* Host is trying set protocol. Now receive the protocol and save */
            ${APP_NAME?lower_case}Data->activeProtocol = *(USB_HID_PROTOCOL_CODE *)eventData;

            /* Acknowledge the Control Write Transfer */
            USB_DEVICE_ControlStatus(${APP_NAME?lower_case}Data->handleUsbDevice, USB_DEVICE_CONTROL_STATUS_OK);
            
            break;

        case  USB_DEVICE_HID_EVENT_GET_PROTOCOL:

            /* Host is requesting for Current Protocol. Now send the Idle rate */
             USB_DEVICE_ControlSend(${APP_NAME?lower_case}Data->handleUsbDevice, &(${APP_NAME?lower_case}Data->activeProtocol), 1);

             /* On successfully receiving Idle rate, the Host would acknowledge
             * back with a Zero Length packet. The HID function driver returns
             * an event USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the
             * application upon receiving this Zero Length packet from Host.
             * USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates
             * this control transfer event is complete */

            break;

        case USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This event occurs when the data stage of a control read transfer
             * has completed. This happens after the application uses the
             * USB_DEVICE_ControlSend function to respond to a HID Function
             * Driver Control Transfer Event that requires data to be sent to
             * the host. The pData parameter will be NULL */
            
            break;

        case USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* This event occurs when the data stage of a control write transfer
             * has completed. This happens after the application uses the
             * USB_DEVICE_ControlReceive function to respond to a HID Function
             * Driver Control Transfer Event that requires data to be received
             * from the host. */
            
            break;
        
        case USB_DEVICE_HID_EVENT_GET_REPORT:

            /* This event occurs when the host issues a GET REPORT command. */
            
            break;

        case USB_DEVICE_HID_EVENT_SET_REPORT:

            /* This event occurs when the host issues a SET REPORT command */
            
            break;

        case USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_ABORTED:

            /* This event occurs when an ongoing control transfer was aborted.
             * The application must stop any pending control transfer related
             * activities. */
            
            break;
			
        default:

            break;
    }
}

/******************************************************
 * Application USB Device Layer Event Handler
 ******************************************************/

static void APP_USBDeviceEventHandler
(
    USB_DEVICE_EVENT event, 
    void * eventData, 
    uintptr_t context
)
{
    USB_DEVICE_EVENT_DATA_CONFIGURED * configurationValue;
    
    switch(event)
    {
        case USB_DEVICE_EVENT_SOF:
            
            break;
            
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:
        
            /* Device got deconfigured */
            ${APP_NAME?lower_case}Data.usbDeviceIsConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Device is configured */
            configurationValue = (USB_DEVICE_EVENT_DATA_CONFIGURED *)eventData;
            if(configurationValue->configurationValue == 1)
            {
                /* Register the Application HID Event Handler. */
                USB_DEVICE_HID_EventHandlerSet(USB_DEVICE_HID_INDEX_0, APP_USBDeviceHIDEventHandler, (uintptr_t)&${APP_NAME?lower_case}Data);
                
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

        case USB_DEVICE_EVENT_CONTROL_TRANSFER_ABORTED:
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
    Services the USB task. 
*/
static void USB_Task (void)
{
    if(${APP_NAME?lower_case}Data.usbDeviceIsConfigured)
    {
        /* Write USB HID Application Logic here. Note that this function is
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

    {
        /* Initialize USB Mouse Device application data */
        ${APP_NAME?lower_case}Data.handleUsbDevice       = USB_DEVICE_HANDLE_INVALID;
        ${APP_NAME?lower_case}Data.usbDeviceIsConfigured = false;
        ${APP_NAME?lower_case}Data.idleRate              = 0;
    }
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
                USB_DEVICE_EventHandlerSet(${APP_NAME?lower_case}Data.handleUsbDevice,APP_USBDeviceEventHandler, 0);
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
            USB_Task();
</#macro>

<#--    
            /* TODO: Function calls to other Application Tasks can be placed
             * here */    

            break;
        }

        default:
        {
            /* TODO: The application tasks should never enter this state.
             * Handle error in application's state machine. */
            break;
        }
    }
}
-->

<#macro macro_lib_usb_app_c_tasks_states>
</#macro>
