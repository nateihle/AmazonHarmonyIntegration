
<#macro macro_usb_host_msd_read_write_h_type_definitions>
<#if ("CONFIG_APP_USB_HOST_MSD_READ" + "${HCONFIG_APP_INSTANCE}")?eval>
typedef enum
{
    ${APP_NAME?upper_case}_MSD_READ_STATE_START,
    ${APP_NAME?upper_case}_MSD_READ_STATE_OPEN,
    ${APP_NAME?upper_case}_MSD_READ_STATE_FILEOPS,
    ${APP_NAME?upper_case}_MSD_READ_STATE_ERROR,
    ${APP_NAME?upper_case}_MSD_READ_STATE_DONE,
} ${APP_NAME?upper_case}_MSD_READ_STATES;

</#if>
<#if ("CONFIG_APP_USB_HOST_MSD_WRITE" + "${HCONFIG_APP_INSTANCE}")?eval>
typedef enum
{
    ${APP_NAME?upper_case}_MSD_WRITE_STATE_START,
    ${APP_NAME?upper_case}_MSD_WRITE_STATE_OPEN,
    ${APP_NAME?upper_case}_MSD_WRITE_STATE_FILEOPS,
    ${APP_NAME?upper_case}_MSD_WRITE_STATE_ERROR,
    ${APP_NAME?upper_case}_MSD_WRITE_STATE_DONE,
} ${APP_NAME?upper_case}_MSD_WRITE_STATES;

</#if>
</#macro>

<#macro macro_usb_host_msd_read_write_h_data>
<#if ("CONFIG_APP_USB_HOST_MSD_READ" + "${HCONFIG_APP_INSTANCE}")?eval>
    /* State Machine for USB MSD read */
    ${APP_NAME?upper_case}_MSD_READ_STATES msdReadStateMachine;

</#if>
<#if ("CONFIG_APP_USB_HOST_MSD_WRITE" + "${HCONFIG_APP_INSTANCE}")?eval>
    /* State Machine for USB MSD write */
    ${APP_NAME?upper_case}_MSD_WRITE_STATES msdWriteStateMachine;

</#if>
    /* true when a USB device is connected */
    bool msdDeviceIsConnected;

</#macro>

<#macro macro_usb_host_msd_read_write_h_callback_function_declarations>
</#macro>

<#macro macro_usb_host_msd_read_write_h_function_declarations>
</#macro>

