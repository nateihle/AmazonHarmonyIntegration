<#-- usb_dev_cdc_com_port_macros_system_config.h.ftl -->

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
<#macro macro_lib_usb_system_config_h_app_constants>

<#if ("CONFIG_USB_DEV_CDC_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
#define ${APP_NAME?upper_case}_USB_CDC_COM_PORT_SINGLE_READ_BUFFER_SIZE  512
</#if>
<#if ("CONFIG_USB_DEV_CDC_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
#define ${APP_NAME?upper_case}_USB_CDC_COM_PORT_SINGLE_WRITE_BUFFER_SIZE 512
</#if>
</#macro>
