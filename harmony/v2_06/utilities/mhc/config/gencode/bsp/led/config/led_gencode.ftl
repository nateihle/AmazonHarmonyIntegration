menu "LED"
    
config GENERATE_CODE_LED${INSTANCE}
    bool "LED?"
    select USE_BSP_NEEDED
    default n
    
ifblock GENERATE_CODE_LED${INSTANCE}

comment "**** Note: Please configure BSP settings under BSP Configuration menu ****"

config APP_LED_NAME${INSTANCE}
    depends on GENERATE_CODE_LED${INSTANCE}
    string "LED Name"
    default "BSP_LED_1"

<#include "/gencode/framework/global_event/config/global_event_macros.ftl">
<@global_event_triggered_hconfig module_name="LED" action="Toggle"/>

add "^#include \"/utilities/mhc/config/gencode/bsp/led/config/led_gencode_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_led_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_led_app_h_constants/>" to list APP${INSTANCE}_H_CONSTANTS
add "^@macro_led_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_led_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_led_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS

add "^#include \"/utilities/mhc/config/gencode/bsp/led/config/led_gencode_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_led_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_led_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_led_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_led_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_led_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_led_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_led_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_led_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
  
endif

endmenu
