
<#macro macro_sys_fs_write_app_h_includes>

typedef enum
{
    ${APP_NAME?upper_case}_FS_WRITE_STATE_START,
    ${APP_NAME?upper_case}_FS_WRITE_STATE_OPEN,
    ${APP_NAME?upper_case}_FS_WRITE_STATE_FILEOPS,
    ${APP_NAME?upper_case}_FS_WRITE_STATE_ERROR,
    ${APP_NAME?upper_case}_FS_WRITE_STATE_DONE,
} ${APP_NAME?upper_case}_FS_WRITE_STATES;
</#macro>

<#macro macro_sys_fs_write_app_h_states>
</#macro>

<#macro macro_sys_fs_write_app_h_data>

    /* State Machine */
    ${APP_NAME?upper_case}_FS_WRITE_STATES fsWriteStateMachine;

</#macro>

<#macro macro_sys_fs_write_app_h_callback_function_declarations>
</#macro>

<#macro macro_sys_fs_write_app_h_function_declarations>
</#macro>
