
<#macro macro_sys_rtcc_timestamp_app_h_includes>

typedef enum
{
    ${APP_NAME?upper_case}_RTCC_TIMESTAMP_START,
    ${APP_NAME?upper_case}_RTCC_TIMESTAMP_DONE,
    ${APP_NAME?upper_case}_RTCC_TIMESTAMP_CHECK,
} ${APP_NAME?upper_case}_RTCC_TIMESTAMP_STATES;
</#macro>

<#macro macro_sys_rtcc_timestamp_app_h_states>
</#macro>

<#macro macro_sys_rtcc_timestamp_app_h_data>

    /* State Machine for real time clock & calendar time and date setting */
    ${APP_NAME?upper_case}_RTCC_TIMESTAMP_STATES rtccTimeStateMachine;

</#macro>

<#macro macro_sys_rtcc_timestamp_app_h_callback_function_declarations>
</#macro>

<#macro macro_sys_rtcc_timestamp_app_h_function_declarations>
</#macro>
