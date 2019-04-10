
<#macro macro_sys_fs_read_app_h_includes>

typedef enum
{
    ${APP_NAME?upper_case}_FS_READ_STATE_START,
    ${APP_NAME?upper_case}_FS_READ_STATE_OPEN,
    ${APP_NAME?upper_case}_FS_READ_STATE_FILEOPS,
    ${APP_NAME?upper_case}_FS_READ_STATE_ERROR,
    ${APP_NAME?upper_case}_FS_READ_STATE_DONE,
} ${APP_NAME?upper_case}_FS_READ_STATES;
</#macro>

<#macro macro_sys_fs_read_app_h_states>
</#macro>

<#macro macro_sys_fs_read_app_h_data>

    /* State Machine for file system read */
    ${APP_NAME?upper_case}_FS_READ_STATES fsReadStateMachine;

</#macro>

<#macro macro_sys_fs_read_app_h_callback_function_declarations>
</#macro>

<#macro macro_sys_fs_read_app_h_function_declarations>
</#macro>
