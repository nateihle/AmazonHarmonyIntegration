<#-- usb_host_cdc_com_port_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_lib_usb_app_h_includes>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_lib_usb_app_h_type_definitions>
typedef enum
{
	${APP_NAME?upper_case}_INIT_STATE_BUS_ENABLE,
	${APP_NAME?upper_case}_INIT_STATE_WAIT_FOR_BUS_ENABLE_COMPLETE,
	${APP_NAME?upper_case}_INIT_STATE_WAIT_FOR_DEVICE_ATTACH,
	${APP_NAME?upper_case}_INIT_STATE_OPEN_DEVICE,
	${APP_NAME?upper_case}_INIT_STATE_SET_LINE_CODING,
	${APP_NAME?upper_case}_INIT_STATE_WAIT_FOR_SET_LINE_CODING,
	${APP_NAME?upper_case}_INIT_STATE_SEND_SET_CONTROL_LINE_STATE,
	${APP_NAME?upper_case}_INIT_STATE_WAIT_FOR_SET_CONTROL_LINE_STATE,
	${APP_NAME?upper_case}_INIT_STATE_ERROR,
	${APP_NAME?upper_case}_INIT_STATE_DONE
} ${APP_NAME?upper_case}_INIT_STATES;

<#if ("CONFIG_USB_HOST_CDC_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
typedef enum
{
    ${APP_NAME?upper_case}_TX_WAIT_FOR_CONFIGURATION,
    ${APP_NAME?upper_case}_TX_TRANSMIT_STRING,
    ${APP_NAME?upper_case}_TX_WAIT_FOR_SEND_COMPLETE,
    ${APP_NAME?upper_case}_TX_DONE
} ${APP_NAME?upper_case}_TX_STATES;
</#if>
</#macro>

<#--
// *****************************************************************************
/* Application Data
// *****************************************************************************

// *****************************************************************************
/* Application constants

  Summary:
    Constants defined for the application

  Description:
    Constants defined for the application
*/
-->
<#macro macro_lib_usb_app_h_constants>
#define ${APP_NAME?upper_case}_HOST_CDC_BAUDRATE_SUPPORTED 9600UL
#define ${APP_NAME?upper_case}_HOST_CDC_PARITY_TYPE        0
#define ${APP_NAME?upper_case}_HOST_CDC_STOP_BITS          0
#define ${APP_NAME?upper_case}_HOST_CDC_NO_OF_DATA_BITS    8
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
<#macro macro_lib_usb_app_h_data>
<#if ("CONFIG_USB_HOST_CDC_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
    /* Array to hold read data */
    uint8_t inDataArray[${("CONFIG_USB_HOST_CDC_RX_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval}];

</#if>
    /* CDC Object */
    USB_HOST_CDC_OBJ cdcObj;
    
    /* True if a device is attached */
    bool deviceIsAttached;
    
    /* True if control request is done */
    bool controlRequestDone;
    
    /* Control Request Result */
    USB_HOST_CDC_RESULT controlRequestResult;

    /* A CDC Line Coding object */
    USB_CDC_LINE_CODING cdcHostLineCoding;
    
    /* A Control Line State object*/
    USB_CDC_CONTROL_LINE_STATE controlLineState;
    
    /* Handle to the CDC device. */
    USB_HOST_CDC_HANDLE cdcHostHandle;
    
    USB_HOST_CDC_REQUEST_HANDLE  requestHandle;
    
    /* True when a write transfer has complete */
    bool writeTransferDone;
    
    /* Write Transfer Result */
    USB_HOST_CDC_RESULT writeTransferResult;
    
     /* True when a read transfer has complete */
    bool readTransferDone;
    
    /* Read Transfer Result */
    USB_HOST_CDC_RESULT readTransferResult;
    
    /* True if device was detached */
    bool deviceWasDetached;

	bool isConfigured;

	${APP_NAME?upper_case}_INIT_STATES initState;
<#if ("CONFIG_USB_HOST_CDC_RX" + "${HCONFIG_APP_INSTANCE}")?eval>

	uint32_t readLen;
</#if>
<#if ("CONFIG_USB_HOST_CDC_TX" + "${HCONFIG_APP_INSTANCE}")?eval>

	uint32_t writeLen;

	${APP_NAME?upper_case}_TX_STATES txState;;
</#if>
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
<#macro macro_lib_usb_app_h_callback_function_declarations>
USB_HOST_EVENT_RESPONSE APP_USBHostEventHandler 
(
    USB_HOST_EVENT event, 
    void * eventData,
    uintptr_t context
);

void APP_USBHostCDCAttachEventListener
(
    USB_HOST_CDC_OBJ cdcObj, 
    uintptr_t context
);

USB_HOST_CDC_EVENT_RESPONSE APP_USBHostCDCEventHandler
(
    USB_HOST_CDC_HANDLE cdcHandle,
    USB_HOST_CDC_EVENT event,
    void * eventData,
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
<#macro macro_lib_usb_app_h_function_declarations>
</#macro>

<#macro macro_lib_usb_app_h_states>
	${APP_NAME?upper_case}_STATE_ERROR,
	${APP_NAME?upper_case}_STATE_DONE
</#macro>




