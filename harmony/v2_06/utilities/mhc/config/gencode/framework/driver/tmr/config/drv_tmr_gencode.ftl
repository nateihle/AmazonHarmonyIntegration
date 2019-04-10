enum APP_TMR_DRV_ERRORS${INSTANCE}
    "NONE"
    || "HIGH"
    || "LOW"
    || "OK"


menu "Timer"
    depends on HAVE_TMR

config GENERATE_CODE_DRV_TMR${INSTANCE}
    bool "Timer?"
    default n
    set USE_DRV_TMR_NEEDED to y if GENERATE_CODE_DRV_TMR${INSTANCE}
    ---help---
    IDH_HTML_DRV_TMR_Timer_Driver_Library
    ---endhelp---

ifblock GENERATE_CODE_DRV_TMR${INSTANCE}

comment "**** Note: Please configure Timer Driver settings under Framework Configuration menu ****"


config APP_TMR_DRV_PERIODIC${INSTANCE}
    bool "Timer is periodic?"
    default y

menu "Timer Period"
comment "**** You must use 'Execute' to re-populate the timer period count ****"
comment "**** The timer period count below will be used for the timer period. ****"
    config APP_TMR_DRV_CALC_PERIOD${INSTANCE}
    int "Timer Period ( in micro Seconds )"
    default 10000
    range 1 2000000000
    ---help---
    Enter Period in uS.
    One Second = 1000000
    One millisecond = 1000
    ---endhelp---


execute APP_TMR_DRV_CALCULATE_EXEC${INSTANCE}
    prompt "Click to Calculate Timer Period Count:"
    default "timer_calculate ${INSTANCE}"
    ---help---
    Automatically calculates timer driver configuration states based on the desired period.
    ---endhelp---

config APP_TMR_DRV_PERIOD${INSTANCE}
    hex "Calculated Timer Period Count (in hex)"
    persistent
    default 0x3d09

config APP_TMR_DRV_COMMENT_VAL_IDX${INSTANCE}
	string
	range APP_TMR_DRV_ERRORS${INSTANCE}
	default "NONE"

ifblock APP_TMR_DRV_COMMENT_VAL_IDX${INSTANCE} = "HIGH"
error "**** The Period is too large! Try a 32bit timer. ****"
endif

ifblock APP_TMR_DRV_COMMENT_VAL_IDX${INSTANCE} = "OK"
comment "**** Period Count Calculated Successfully. ****"
endif

ifblock APP_TMR_DRV_COMMENT_VAL_IDX${INSTANCE} = "LOW"
error "**** The Period is too small. Try a faster clock. ****"
endif

endmenu


menu "Options"

config APP_TMR_DRV_INSTANCE_INDEX${INSTANCE}
    int "Timer Driver Instance Index"
    range 0 0 if DRV_TMR_INSTANCES_NUMBER = 1
    range 0 1 if DRV_TMR_INSTANCES_NUMBER = 2
    range 0 2 if DRV_TMR_INSTANCES_NUMBER = 3
    range 0 3 if DRV_TMR_INSTANCES_NUMBER = 4
    range 0 4 if DRV_TMR_INSTANCES_NUMBER = 5
    range 0 5 if DRV_TMR_INSTANCES_NUMBER = 6
    range 0 6 if DRV_TMR_INSTANCES_NUMBER = 7
    range 0 7 if DRV_TMR_INSTANCES_NUMBER = 8
    range 0 8 if DRV_TMR_INSTANCES_NUMBER = 9
    default 0

ifblock USE_SYS_TMR &&
     ((SYS_TMR_DRIVER_INDEX = "DRV_TMR_INDEX_0" && APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 0) ||
      (SYS_TMR_DRIVER_INDEX = "DRV_TMR_INDEX_1" && APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 1) ||
      (SYS_TMR_DRIVER_INDEX = "DRV_TMR_INDEX_2" && APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 2) ||
      (SYS_TMR_DRIVER_INDEX = "DRV_TMR_INDEX_3" && APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 3) ||
      (SYS_TMR_DRIVER_INDEX = "DRV_TMR_INDEX_4" && APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 4) ||
      (SYS_TMR_DRIVER_INDEX = "DRV_TMR_INDEX_5" && APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 5) ||
      (SYS_TMR_DRIVER_INDEX = "DRV_TMR_INDEX_6" && APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 6) ||
      (SYS_TMR_DRIVER_INDEX = "DRV_TMR_INDEX_7" && APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 7) ||
      (SYS_TMR_DRIVER_INDEX = "DRV_TMR_INDEX_8" && APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 8))
    error "Error! - Timer Driver instance index must differ from the System Timer instance index"
endif

config APP_TMR_DRV_HANDLE${INSTANCE}
    string "Name of the timer driver handle?"
    default "handleTimer0" if APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 0
    default "handleTimer1" if APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 1
    default "handleTimer2" if APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 2
    default "handleTimer3" if APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 3
    default "handleTimer4" if APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 4
    default "handleTimer5" if APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 5
    default "handleTimer6" if APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 6
    default "handleTimer7" if APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 7
    default "handleTimer8" if APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 8
    default "handleTimer9" if APP_TMR_DRV_INSTANCE_INDEX${INSTANCE} = 9
    default "handleTimer"

endmenu


<#include "/gencode/framework/global_event/config/global_event_macros.ftl">
<@global_event_trigger_hconfig module_name = "TMR_DRV"/>

add "^#include \"/utilities/mhc/config/gencode/framework/driver/tmr/config/drv_tmr_gencode_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_drv_tmr_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_drv_tmr_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_drv_tmr_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_drv_tmr_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_drv_tmr_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS

add "^#include \"/utilities/mhc/config/gencode/framework/driver/tmr/config/drv_tmr_gencode_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_drv_tmr_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_drv_tmr_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_drv_tmr_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_drv_tmr_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_drv_tmr_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_drv_tmr_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_drv_tmr_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_drv_tmr_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS

add "^#include \"/utilities/mhc/config/gencode/framework/driver/tmr/config/drv_tmr_gencode_macros_system_config.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_drv_tmr_system_config_h_app_constants/>" to list APP${INSTANCE}_H_CONSTANTS

endif

endmenu
