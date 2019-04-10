<#-- usb_dev_cdc_com_port_macros_app.c.ftl -->

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
<#if ("CONFIG_USB_DEV_CDC_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
</#if>
/* Static buffers, suitable for DMA transfer */
#define ${APP_NAME?upper_case}_MAKE_BUFFER_DMA_READY  __attribute__((coherent)) __attribute__((aligned(16)))

<#if ("CONFIG_USB_DEV_CDC_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
static uint8_t ${APP_NAME?upper_case}_MAKE_BUFFER_DMA_READY writeBuffer[${APP_NAME?upper_case}_USB_CDC_COM_PORT_SINGLE_WRITE_BUFFER_SIZE];
static uint8_t writeString[] = "${("CONFIG_USB_DEV_CDC_TX_STRG" + "${HCONFIG_APP_INSTANCE}")?eval}\r\n";
</#if>
<#if ("CONFIG_USB_DEV_CDC_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
static uint8_t ${APP_NAME?upper_case}_MAKE_BUFFER_DMA_READY readBuffer [${APP_NAME?upper_case}_USB_CDC_COM_PORT_SINGLE_READ_BUFFER_SIZE];
static uint8_t readString[${("CONFIG_USB_DEV_CDC_RX_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval}];
</#if>
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

    switch ( event )
    {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *)pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent.  */
            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:
            /* This means that the host has sent some data*/
<#if ("CONFIG_USB_DEV_CDC_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
            appDataObject->readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
			readString[appDataObject->readProcessedLen] = readBuffer[0];
			if (appDataObject->readProcessedLen < ${("CONFIG_USB_DEV_CDC_RX_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval})
			{
            	appDataObject->readProcessedLen++;
			}
</#if>
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the host has sent some data*/
<#if ("CONFIG_USB_DEV_CDC_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
            appDataObject->writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
</#if>
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

            ${APP_NAME?lower_case}Data.isConfigured = false;

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

                /* Mark that the device is now configured */
                ${APP_NAME?lower_case}Data.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(${APP_NAME?lower_case}Data.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(${APP_NAME?lower_case}Data.deviceHandle);
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
<#if ("CONFIG_USB_DEV_CDC_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
/******************************************************************************
  Function:
    static void USB_TX_Task (void)
    
   Remarks:
    Feeds the USB write function. 
*/
static void USB_TX_Task (void)
{
    if(!${APP_NAME?lower_case}Data.isConfigured)
    {
        ${APP_NAME?lower_case}Data.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
    }
    else
    {
        /* Schedule a write if data is pending 
         */
        if ((${APP_NAME?lower_case}Data.writeLen > 0)/* && (${APP_NAME?lower_case}Data.writeTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID)*/)
        {
            USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                                 &${APP_NAME?lower_case}Data.writeTransferHandle,
                                 writeBuffer, 
                                 ${APP_NAME?lower_case}Data.writeLen,
                                 USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
        }
    }
}
</#if>

<#if ("CONFIG_USB_DEV_CDC_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
/******************************************************************************
  Function:
    static void USB_RX_Task (void)
    
   Remarks:
    Reads from the USB. 
*/
static void USB_RX_Task(void)
{
    if(!${APP_NAME?lower_case}Data.isConfigured)
    {
        ${APP_NAME?lower_case}Data.readTransferHandle  = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        ${APP_NAME?lower_case}Data.readProcessedLen    = 0;
    }
    else
    {
        /* Schedule a read if none is pending and all previously read data
           has been processed
         */
        if((${APP_NAME?lower_case}Data.readProcessedLen < ${("CONFIG_USB_DEV_CDC_RX_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval}) && (${APP_NAME?lower_case}Data.readTransferHandle  == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID))
        {
            USB_DEVICE_CDC_Read (USB_DEVICE_CDC_INDEX_0,
                                 &${APP_NAME?lower_case}Data.readTransferHandle, 
                                 readBuffer,
                                 ${APP_NAME?upper_case}_USB_CDC_COM_PORT_SINGLE_READ_BUFFER_SIZE);
        };
    }
}
</#if>
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
    ${APP_NAME?lower_case}Data.deviceHandle = USB_DEVICE_HANDLE_INVALID ;

    /* Device configured status */
    ${APP_NAME?lower_case}Data.isConfigured = false;

    /* Initial get line coding state */
    ${APP_NAME?lower_case}Data.getLineCodingData.dwDTERate   = 9600;
    ${APP_NAME?lower_case}Data.getLineCodingData.bParityType =  0;
    ${APP_NAME?lower_case}Data.getLineCodingData.bParityType = 0;
    ${APP_NAME?lower_case}Data.getLineCodingData.bDataBits   = 8;

<#if ("CONFIG_USB_DEV_CDC_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
    /* Read Transfer Handle */
    ${APP_NAME?lower_case}Data.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read data */
    ${APP_NAME?lower_case}Data.readProcessedLen = 0;
</#if>

<#if ("CONFIG_USB_DEV_CDC_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
    /* Write Transfer Handle */
    ${APP_NAME?lower_case}Data.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
    
    /*Initialize the write data */
    ${APP_NAME?lower_case}Data.writeLen = sizeof(writeString);
	memcpy(writeBuffer, writeString, ${APP_NAME?lower_case}Data.writeLen);
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
            if (${APP_NAME?lower_case}Data.deviceHandle == USB_DEVICE_HANDLE_INVALID)
            {
                ${APP_NAME?lower_case}Data.deviceHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0,
                                               DRV_IO_INTENT_READWRITE );
                appInitialized &= ( USB_DEVICE_HANDLE_INVALID != ${APP_NAME?lower_case}Data.deviceHandle );
            }
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_lib_usb_app_c_tasks_calls_after_init>

                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(${APP_NAME?lower_case}Data.deviceHandle,
                                           ${APP_NAME?upper_case}_USBDeviceEventHandler, 0);
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
<#if ("CONFIG_USB_DEV_CDC_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
            USB_RX_Task();
</#if>
<#if ("CONFIG_USB_DEV_CDC_TX" + "${HCONFIG_APP_INSTANCE}")?eval>            
            USB_TX_Task();
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
