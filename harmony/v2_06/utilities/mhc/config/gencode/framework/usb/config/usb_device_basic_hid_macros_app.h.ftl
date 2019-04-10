<#-- usb_device_basic_hid_macros_app.h.ftl -->

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
    /*
     * USB variables used by the mouse device application:
     * 
     *     handleUsbDevice          : USB Device driver handle
     *     usbDeviceIsConfigured    : If true, USB Device is configured
     *     activeProtocol           : USB HID active Protocol
     *     idleRate                 : USB HID current Idle
     */
    USB_DEVICE_HANDLE                handleUsbDevice;
    bool                             usbDeviceIsConfigured;
    uint8_t                          activeProtocol;
    uint8_t                          idleRate;

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

