
<#macro macro_sys_nvm_app_h_includes>

typedef enum
{
    ${APP_NAME?upper_case}_NVM_STATE_START,
    ${APP_NAME?upper_case}_NVM_STATE_OPEN,
    ${APP_NAME?upper_case}_NVM_STATE_FILEOPS,
    ${APP_NAME?upper_case}_NVM_STATE_ERROR,
    ${APP_NAME?upper_case}_NVM_STATE_DONE,
} ${APP_NAME?upper_case}_NVM_STATES;
</#macro>

<#macro macro_sys_nvm_app_h_states>
</#macro>

<#macro macro_sys_nvm_app_h_data>

    /* State Machine for NVM */
    ${APP_NAME?upper_case}_NVM_STATES nvmStateMachine;
</#macro>

<#macro macro_sys_nvm_app_h_callback_function_declarations>
</#macro>

<#macro macro_sys_nvm_app_h_function_declarations>
</#macro>
