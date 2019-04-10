<#-- usb_dev_cdc_basic_macros_app.h.ftl -->

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
/******************************************************
 * Application COM Port Object
 ******************************************************/

typedef struct
{
    USB_DEVICE_CDC_INDEX cdcInstance;

    /* Set Line Coding Data */
    USB_CDC_LINE_CODING setLineCodingData;

    /* Get Line Coding Data */
    USB_CDC_LINE_CODING getLineCodingData;

    /* Control Line State */
    USB_CDC_CONTROL_LINE_STATE controlLineStateData;

    /* Break data */
    uint16_t breakData;

    /* Read transfer handle */
    USB_DEVICE_CDC_TRANSFER_HANDLE readTransferHandle;

    /* Write transfer handle */
    USB_DEVICE_CDC_TRANSFER_HANDLE writeTransferHandle;

    /* True if a character was read */
    bool isReadComplete;

    /* True if a character was written*/
    bool isWriteComplete;

}APP_COM_PORT_OBJECT;
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

    /* Device layer handle returned by device layer open function */
    USB_DEVICE_HANDLE handleUsbDevice;

    /* Device configured state */
    bool usbDeviceIsConfigured;
	
	APP_COM_PORT_OBJECT appCOMPortObjects[${CONFIG_USB_DEV_CDC_BASIC_NUM_PORTS0?number}];

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
</#macro>

