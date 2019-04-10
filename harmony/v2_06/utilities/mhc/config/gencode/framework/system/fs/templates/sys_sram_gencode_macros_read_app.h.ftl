
<#macro macro_sys_sram_read_app_h_includes>

typedef enum
{
    ${APP_NAME?upper_case}_SRAM_READ_STATE_START,
    ${APP_NAME?upper_case}_SRAM_READ_STATE_OPEN,
    ${APP_NAME?upper_case}_SRAM_READ_STATE_FILEOPS,
    ${APP_NAME?upper_case}_SRAM_READ_STATE_ERROR,
    ${APP_NAME?upper_case}_SRAM_READ_STATE_DONE,
} ${APP_NAME?upper_case}_SRAM_READ_STATES;
</#macro>

<#macro macro_sys_sram_read_app_h_states>
</#macro>

<#macro macro_sys_sram_read_app_h_data>

    /* State Machine for SRAM read */
    ${APP_NAME?upper_case}_SRAM_READ_STATES sramReadStateMachine;
</#macro>

<#macro macro_sys_sram_read_app_h_callback_function_declarations>
</#macro>

<#macro macro_sys_sram_read_app_h_function_declarations>
</#macro>
