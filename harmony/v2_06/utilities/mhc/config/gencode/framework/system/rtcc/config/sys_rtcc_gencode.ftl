menu "RTCC"

config GENERATE_CODE_SYS_RTCC${INSTANCE}
    bool "Use Real Time Clock and Calendar?"
    default n
    select USE_SYS_RTCC if GENERATE_CODE_SYS_RTCC${INSTANCE}
    ---help---
    IDH_HTML_RTCC_Peripheral_Library
    ---endhelp---


ifblock GENERATE_CODE_SYS_RTCC${INSTANCE}

comment "**** set the time and alarm in driver config area for RTCC ****"

config APP_SYS_RTCC_TIMESTAMP${INSTANCE}
    int "Number of timestamps to collect:"
    default 10
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>MPLAB Harmony Real Time Clock and Calendar Template</h2>
    <p>	This template generates a simple code example which sets
    the RTCC.</p>
    <p>All other filesystem configuration options are set to their
    default values. The driver configuration must be modified by
    the user using MHC, under Harmony Framework Configuration ->
    System Services-> File System and setup the Media to use.</p></html>
    ---endhelp---



add "^#include \"/utilities/mhc/config/gencode/framework/system/rtcc/templates/sys_rtcc_gencode_macros_timestamp_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_sys_rtcc_timestamp_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_sys_rtcc_timestamp_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES
add "^@macro_sys_rtcc_timestamp_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_sys_rtcc_timestamp_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_sys_rtcc_timestamp_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS

add "^#include \"/utilities/mhc/config/gencode/framework/system/rtcc/templates/sys_rtcc_gencode_macros_timestamp_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_sys_rtcc_timestamp_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_sys_rtcc_timestamp_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_sys_rtcc_timestamp_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_sys_rtcc_timestamp_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_sys_rtcc_timestamp_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_sys_rtcc_timestamp_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_sys_rtcc_timestamp_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_sys_rtcc_timestamp_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_sys_rtcc_timestamp_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_sys_rtcc_timestamp_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES


endif
endmenu
