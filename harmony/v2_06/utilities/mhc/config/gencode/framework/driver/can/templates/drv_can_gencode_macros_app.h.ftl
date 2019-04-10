
<#macro macro_drv_can_app_h_includes>

typedef enum
{
    ${APP_NAME?upper_case}_CAN_STATE_START,
<#if ("CONFIG_APP_DRV_CAN_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
    ${APP_NAME?upper_case}_CAN_STATE_RX,
</#if>
    ${APP_NAME?upper_case}_CAN_STATE_DONE
} ${APP_NAME?upper_case}_CAN_STATES;
</#macro>


<#macro macro_drv_can_app_h_states>
</#macro>

<#macro macro_drv_can_app_h_data>

    ${APP_NAME?upper_case}_CAN_STATES canStateMachine;

    /* CAN Driver Handle definitions */
    DRV_HANDLE ${("CONFIG_APP_DRV_CAN_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval};

</#macro>

<#macro macro_drv_can_app_h_callback_function_declarations>
</#macro>

<#macro macro_drv_can_app_h_function_declarations>
</#macro>
