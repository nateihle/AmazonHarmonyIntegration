<#-- usb_host_cdc_com_port_macros_app.c.ftl -->

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
<#if ("CONFIG_USB_HOST_CDC_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
#define ${APP_NAME?upper_case}_MAKE_BUFFER_DMA_READY  __attribute__((coherent)) __attribute__((aligned(16)))
static uint8_t ${APP_NAME?upper_case}_MAKE_BUFFER_DMA_READY writeBuffer[] = "${("CONFIG_USB_HOST_CDC_TX_STRG" + "${HCONFIG_APP_INSTANCE}")?eval}\r\n";
</#if>
</#macro>

<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_lib_usb_app_c_callback_functions>
USB_HOST_EVENT_RESPONSE ${APP_NAME?upper_case}_USBHostEventHandler 
(
    USB_HOST_EVENT event, 
    void * eventData,
    uintptr_t context
)
{
    /* This function is called by the USB Host whenever a USB Host Layer event
     * has occurred. In this example we only handle the device unsupported event
     * */

    switch (event)
    {
        case USB_HOST_EVENT_DEVICE_UNSUPPORTED:
            
            /* The attached device is not supported for some reason */
            break;
            
        default:
            break;
                    
    }
    
    return(USB_HOST_EVENT_RESPONSE_NONE);
}

void ${APP_NAME?upper_case}_USBHostCDCAttachEventListener(USB_HOST_CDC_OBJ cdcObj, uintptr_t context)
{
    /* This function gets called when the CDC device is attached. Update the
     * application data structure to let the application know that this device
     * is attached */
    
    ${APP_NAME?lower_case}Data.deviceIsAttached = true;
    ${APP_NAME?lower_case}Data.cdcObj = cdcObj;
}

USB_HOST_CDC_EVENT_RESPONSE ${APP_NAME?upper_case}_USBHostCDCEventHandler
(
    USB_HOST_CDC_HANDLE cdcHandle,
    USB_HOST_CDC_EVENT event,
    void * eventData,
    uintptr_t context
)
{
    /* This function is called when a CDC Host event has occurred. A pointer to
     * this function is registered after opening the device. See the call to
     * USB_HOST_CDC_EventHandlerSet() function. */

    USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE_DATA * setLineCodingEventData;
    USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE_DATA * setControlLineStateEventData;
    USB_HOST_CDC_EVENT_WRITE_COMPLETE_DATA * writeCompleteEventData;
    USB_HOST_CDC_EVENT_READ_COMPLETE_DATA * readCompleteEventData;
    
    switch(event)
    {
        case USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE:
            
            /* This means the application requested Set Line Coding request is
             * complete. */
            setLineCodingEventData = (USB_HOST_CDC_EVENT_ACM_SET_LINE_CODING_COMPLETE_DATA *)(eventData);
            ${APP_NAME?lower_case}Data.controlRequestDone = true;
            ${APP_NAME?lower_case}Data.controlRequestResult = setLineCodingEventData->result;
            break;
            
        case USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE:
            
            /* This means the application requested Set Control Line State 
             * request has completed. */
            setControlLineStateEventData = (USB_HOST_CDC_EVENT_ACM_SET_CONTROL_LINE_STATE_COMPLETE_DATA *)(eventData);
            ${APP_NAME?lower_case}Data.controlRequestDone = true;
            ${APP_NAME?lower_case}Data.controlRequestResult = setControlLineStateEventData->result;
            break;
            
        case USB_HOST_CDC_EVENT_WRITE_COMPLETE:
            
            /* This means an application requested write has completed */
            ${APP_NAME?lower_case}Data.writeTransferDone = true;
            writeCompleteEventData = (USB_HOST_CDC_EVENT_WRITE_COMPLETE_DATA *)(eventData);
            ${APP_NAME?lower_case}Data.writeTransferResult = writeCompleteEventData->result;
            break;
            
        case USB_HOST_CDC_EVENT_READ_COMPLETE:
            
            /* This means an application requested write has completed */
            ${APP_NAME?lower_case}Data.readTransferDone = true;
            readCompleteEventData = (USB_HOST_CDC_EVENT_READ_COMPLETE_DATA *)(eventData);
            ${APP_NAME?lower_case}Data.readTransferResult = readCompleteEventData->result;
            break;
            
        case USB_HOST_CDC_EVENT_DEVICE_DETACHED:
            
            /* The device was detached */
            ${APP_NAME?lower_case}Data.deviceWasDetached = true;
            break;
            
        default:
            break;
    }
    
    return(USB_HOST_CDC_EVENT_RESPONE_NONE);
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
    static void USB_Init_Task (void)
    
   Remarks:
    Reads from the USB. 
*/
static void USB_Init_Task(void)
{
	USB_HOST_CDC_RESULT result;

	switch ( ${APP_NAME?lower_case}Data.initState )
    {
		case ${APP_NAME?upper_case}_INIT_STATE_BUS_ENABLE:
            /* In this state the application enables the USB Host Bus. Note
             * how the CDC Attach event handler are registered before the bus
             * is enabled. */
            USB_HOST_EventHandlerSet(${APP_NAME?upper_case}_USBHostEventHandler, (uintptr_t)0);
            USB_HOST_CDC_AttachEventHandlerSet(${APP_NAME?upper_case}_USBHostCDCAttachEventListener, (uintptr_t) 0);
            USB_HOST_BusEnable(0);
            ${APP_NAME?lower_case}Data.initState = ${APP_NAME?upper_case}_INIT_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE;
			break;

        case ${APP_NAME?upper_case}_INIT_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE:
            /* In this state we wait for the Bus enable to complete */
            if(USB_HOST_BusIsEnabled(0))
            {
                ${APP_NAME?lower_case}Data.initState = ${APP_NAME?upper_case}_INIT_STATE_WAIT_FOR_DEVICE_ATTACH;
            }
            break;

        case ${APP_NAME?upper_case}_INIT_STATE_WAIT_FOR_DEVICE_ATTACH:
            /* In this state the application is waiting for the device to be
             * attached */
            if(${APP_NAME?lower_case}Data.deviceIsAttached)
            {
                /* A device is attached. We can open this device */
                ${APP_NAME?lower_case}Data.initState = ${APP_NAME?upper_case}_INIT_STATE_OPEN_DEVICE;
                ${APP_NAME?lower_case}Data.deviceIsAttached = false;
            }
            break;

        case ${APP_NAME?upper_case}_INIT_STATE_OPEN_DEVICE:
            /* In this state the application opens the attached device */
            ${APP_NAME?lower_case}Data.cdcHostHandle = USB_HOST_CDC_Open(${APP_NAME?lower_case}Data.cdcObj);
            if(${APP_NAME?lower_case}Data.cdcHostHandle != USB_HOST_CDC_HANDLE_INVALID)
            {
                /* The driver was opened successfully. Set the event handler
                 * and then go to the next state. */
                USB_HOST_CDC_EventHandlerSet(${APP_NAME?lower_case}Data.cdcHostHandle, ${APP_NAME?upper_case}_USBHostCDCEventHandler, (uintptr_t)0);
                ${APP_NAME?lower_case}Data.initState = ${APP_NAME?upper_case}_INIT_STATE_SET_LINE_CODING;
            }
            break;

        case ${APP_NAME?upper_case}_INIT_STATE_SET_LINE_CODING:
            /* Here we set the Line coding. The control request done flag will
             * be set to true when the control request has completed. */
            ${APP_NAME?lower_case}Data.controlRequestDone = false;
            result = USB_HOST_CDC_ACM_LineCodingSet(${APP_NAME?lower_case}Data.cdcHostHandle, NULL, &${APP_NAME?lower_case}Data.cdcHostLineCoding);
            
            if(result == USB_HOST_CDC_RESULT_SUCCESS)
            {
                /* We wait for the set line coding to complete */
                ${APP_NAME?lower_case}Data.initState = ${APP_NAME?upper_case}_INIT_STATE_WAIT_FOR_SET_LINE_CODING;
            }
            break;

        case ${APP_NAME?upper_case}_INIT_STATE_WAIT_FOR_SET_LINE_CODING:
            if(${APP_NAME?lower_case}Data.controlRequestDone)
            {
                if(${APP_NAME?lower_case}Data.controlRequestResult != USB_HOST_CDC_RESULT_SUCCESS)
                {
                    /* The control request was not successful. */
                    ${APP_NAME?lower_case}Data.initState = ${APP_NAME?upper_case}_INIT_STATE_ERROR;
                }
                else
                {
                    /* Next we set the Control Line State */
                    ${APP_NAME?lower_case}Data.initState = ${APP_NAME?upper_case}_INIT_STATE_SEND_SET_CONTROL_LINE_STATE;
                }
            }
            break;

        case ${APP_NAME?upper_case}_INIT_STATE_SEND_SET_CONTROL_LINE_STATE:
            
            /* Here we set the control line state */
            ${APP_NAME?lower_case}Data.controlRequestDone = false;
            result = USB_HOST_CDC_ACM_ControlLineStateSet(${APP_NAME?lower_case}Data.cdcHostHandle, NULL, 
                    &${APP_NAME?lower_case}Data.controlLineState);
            
            if(result == USB_HOST_CDC_RESULT_SUCCESS)
            {
                /* We wait for the set line coding to complete */
                ${APP_NAME?lower_case}Data.initState = ${APP_NAME?upper_case}_INIT_STATE_WAIT_FOR_SET_CONTROL_LINE_STATE;
            }
            break;

        case ${APP_NAME?upper_case}_INIT_STATE_WAIT_FOR_SET_CONTROL_LINE_STATE:
            
            /* Here we wait for the control line state set request to complete */
            if(${APP_NAME?lower_case}Data.controlRequestDone)
            {
                if(${APP_NAME?lower_case}Data.controlRequestResult != USB_HOST_CDC_RESULT_SUCCESS)
                {
                    /* The control request was not successful. */
                    ${APP_NAME?lower_case}Data.initState = ${APP_NAME?upper_case}_INIT_STATE_ERROR;
                }
				else
				{
					${APP_NAME?lower_case}Data.isConfigured = true;
                	${APP_NAME?lower_case}Data.initState = ${APP_NAME?upper_case}_INIT_STATE_DONE;
				}
            }
            break;

        case ${APP_NAME?upper_case}_INIT_STATE_ERROR:
        case ${APP_NAME?upper_case}_INIT_STATE_DONE:
		default:
			break;
	}
}
<#if ("CONFIG_USB_HOST_CDC_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
/******************************************************************************
  Function:
    static void USB_TX_Task (void)
    
   Remarks:
    Writes to the USB. 
*/
static void USB_TX_Task(void)
{
    USB_HOST_CDC_RESULT result;
    

    switch (${APP_NAME?lower_case}Data.txState)
    {
        case ${APP_NAME?upper_case}_TX_WAIT_FOR_CONFIGURATION:
            if (${APP_NAME?lower_case}Data.isConfigured)
            {
                ${APP_NAME?lower_case}Data.txState = ${APP_NAME?upper_case}_TX_TRANSMIT_STRING;
            }
        case ${APP_NAME?upper_case}_TX_TRANSMIT_STRING:
            ${APP_NAME?lower_case}Data.writeTransferDone = false;
            result = USB_HOST_CDC_Write(${APP_NAME?lower_case}Data.cdcHostHandle, 
					NULL, writeBuffer, sizeof(writeBuffer));
            if(result == USB_HOST_CDC_RESULT_SUCCESS)
            {
                ${APP_NAME?lower_case}Data.txState = ${APP_NAME?upper_case}_TX_WAIT_FOR_SEND_COMPLETE;
            }
            break;
        case ${APP_NAME?upper_case}_TX_WAIT_FOR_SEND_COMPLETE: 
            if(${APP_NAME?lower_case}Data.writeTransferDone)
            {
                if(${APP_NAME?lower_case}Data.writeTransferResult == USB_HOST_CDC_RESULT_SUCCESS)
                {
                    ${APP_NAME?lower_case}Data.txState = ${APP_NAME?upper_case}_TX_DONE;
                }
                else
                {
                    ${APP_NAME?lower_case}Data.txState = ${APP_NAME?upper_case}_TX_TRANSMIT_STRING;
                }
            }
			break;
        case ${APP_NAME?upper_case}_TX_DONE:
        default:
            break;
    }
}
</#if>
<#if ("CONFIG_USB_HOST_CDC_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
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
        ${APP_NAME?lower_case}Data.readLen = ${("CONFIG_USB_HOST_CDC_RX_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval};
    }
    else
    {
        /* Schedule a read if none is pending and all previously read data
           has been processed
         */
        if((${APP_NAME?lower_case}Data.readLen) && (${APP_NAME?lower_case}Data.readTransferDone == true))
        {
			if (${APP_NAME?lower_case}Data.readTransferResult == USB_HOST_CDC_RESULT_SUCCESS)
			{
				${APP_NAME?lower_case}Data.readLen--;
				${APP_NAME?lower_case}Data.readTransferResult = USB_HOST_CDC_RESULT_BUSY;
			}
			else
			{
            	USB_HOST_CDC_Read (${APP_NAME?lower_case}Data.cdcHostHandle, NULL, 
								 &${APP_NAME?lower_case}Data.inDataArray[${("CONFIG_USB_HOST_CDC_RX_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval} - ${APP_NAME?lower_case}Data.readLen], 1);

				${APP_NAME?lower_case}Data.readTransferDone = false;
			}
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
    ${APP_NAME?lower_case}Data.cdcHostLineCoding.dwDTERate		= 9600;
    ${APP_NAME?lower_case}Data.cdcHostLineCoding.bDataBits		= 0;
    ${APP_NAME?lower_case}Data.cdcHostLineCoding.bParityType	= 0;
    ${APP_NAME?lower_case}Data.cdcHostLineCoding.bCharFormat	= 8;
    ${APP_NAME?lower_case}Data.controlLineState.dtr 			= 0;
    ${APP_NAME?lower_case}Data.controlLineState.carrier 		= 0;
<#if ("CONFIG_USB_HOST_CDC_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
    ${APP_NAME?lower_case}Data.readLen							= ${("CONFIG_USB_HOST_CDC_RX_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval};
</#if>
<#if ("CONFIG_USB_HOST_CDC_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
    ${APP_NAME?lower_case}Data.writeLen							= sizeof(writeBuffer);
</#if>
    ${APP_NAME?lower_case}Data.deviceIsAttached 				= false;
    ${APP_NAME?lower_case}Data.deviceWasDetached 				= false;
    ${APP_NAME?lower_case}Data.readTransferDone 				= true;
    ${APP_NAME?lower_case}Data.writeTransferDone 				= true;
    ${APP_NAME?lower_case}Data.controlRequestDone 				= false;
	${APP_NAME?lower_case}Data.isConfigured						= false;
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
	USB_HOST_CDC_RESULT result;

	if(${APP_NAME?lower_case}Data.deviceWasDetached)
	{
	/* This means the device is not attached. Reset the application state */
       
		${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_INIT;
		${APP_NAME?lower_case}Data.readTransferDone = false;
		${APP_NAME?lower_case}Data.writeTransferDone = false;
		${APP_NAME?lower_case}Data.controlRequestDone = false;
		${APP_NAME?lower_case}Data.deviceWasDetached = false;
	}
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
			if (${APP_NAME?lower_case}Data.initState != ${APP_NAME?upper_case}_INIT_STATE_DONE) 
			{ 
				appInitialized = false; 
				USB_Init_Task(); 
			} 
			else 
			{ 
				appInitialized = true; 
			} 
</#macro>    
<#--        
            if (appInitialized)
            {
-->
<#macro macro_lib_usb_app_c_tasks_calls_after_init>
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
<#if ("CONFIG_USB_HOST_CDC_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
            USB_RX_Task();
</#if>
<#if ("CONFIG_USB_HOST_CDC_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
            USB_TX_Task();
</#if>
			if (<#if ("CONFIG_USB_HOST_CDC_RX" + "${HCONFIG_APP_INSTANCE}")?eval>${APP_NAME?lower_case}Data.readLen == 0</#if><#if ("CONFIG_USB_HOST_CDC_RX" + "${HCONFIG_APP_INSTANCE}")?eval><#if ("CONFIG_USB_HOST_CDC_TX" + "${HCONFIG_APP_INSTANCE}")?eval> && </#if></#if><#if ("CONFIG_USB_HOST_CDC_TX" + "${HCONFIG_APP_INSTANCE}")?eval>${APP_NAME?lower_case}Data.txState == ${APP_NAME?upper_case}_TX_DONE</#if>)
			{
				${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_DONE;
			}
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
        case ${APP_NAME?upper_case}_STATE_ERROR:
        case ${APP_NAME?upper_case}_STATE_DONE:
        {
			break;
		}
</#macro>

