
<#macro macro_sys_sram_write_app_h_includes>

typedef enum
{
    ${APP_NAME?upper_case}_SRAM_WRITE_STATE_START,
    ${APP_NAME?upper_case}_SRAM_WRITE_STATE_OPEN,
    ${APP_NAME?upper_case}_SRAM_WRITE_STATE_FILEOPS,
    ${APP_NAME?upper_case}_SRAM_WRITE_STATE_ERROR,
    ${APP_NAME?upper_case}_SRAM_WRITE_STATE_DONE,
} ${APP_NAME?upper_case}_SRAM_WRITE_STATES;
</#macro>

<#macro macro_sys_sram_write_app_h_states>
</#macro>

<#macro macro_sys_sram_write_app_h_data>

    /* State Machine for SRAM */
    ${APP_NAME?upper_case}_SRAM_WRITE_STATES sramWriteStateMachine;
</#macro>

<#macro macro_sys_sram_write_app_h_callback_function_declarations>
</#macro>

<#macro macro_sys_sram_write_app_h_function_declarations>
</#macro>
