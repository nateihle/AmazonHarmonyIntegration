menu "Class B"
    
config CLASSB_LIBRARY${INSTANCE}
    bool "Include Class B Safety Application"
    set USE_CLASSB_LIB_NEEDED to y if CLASSB_LIBRARY${INSTANCE}
	set USE_CLASSB_CLOCK_TEST to y if CLASSB_LIBRARY${INSTANCE}
	set	CLASSB_CLOCK_TEST_TIMER to "TMR_ID_1" if CLASSB_LIBRARY${INSTANCE}
	set CLASSB_CLOCK_TEST_TIMER_CLOCK to "TMR_CLOCK_SOURCE_LOW_POWER_RC_OSCILLATOR" if CLASSB_LIBRARY${INSTANCE}
	set USE_CLASSB_CPU_PC_TEST to y if CLASSB_LIBRARY${INSTANCE}
	set USE_CLASSB_CPU_REGISTERS_TEST to y if CLASSB_LIBRARY${INSTANCE}
	set USE_CLASSB_FLASH_TEST to y if CLASSB_LIBRARY${INSTANCE}
	set USE_CLASSB_RAM_CHECKERBOARD_TEST to y if CLASSB_LIBRARY${INSTANCE}
	set USE_CLASSB_RAM_MARCHB_TEST to y if CLASSB_LIBRARY${INSTANCE}
	set USE_CLASSB_RAM_MARCHC_TEST to y if CLASSB_LIBRARY${INSTANCE}
	set USE_CLASSB_RAM_MARCHC_TEST_MINUS to y if CLASSB_LIBRARY${INSTANCE}
	set USE_CLASSB_RAM_MARCHC_STACK_TEST to y if CLASSB_LIBRARY${INSTANCE}
	set USE_CLASSB_RAM_NON_DESTRUCT_TEST to y if CLASSB_LIBRARY${INSTANCE}
    default n
	---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony Class B Library Import Template</h2>
	<p>	This template generates a simple code example that exercises Class B
	library with the following Class B tests:</p>
	<br> - Clock Test
	<br> - CPU PC Test
	<br> - CPU Registers Test
	<br> - Flsah CRC Test
	<br> - RAM Checker Board Test
	<br> - RAM March B Test
	<br> - RAM March C Test
	<br> - RAM March C Minus Test
	<br> - RAM March C Stack Test
	<br> - Non-destructive Memory Tests
	<p>All other Class B configuration options are set to their 
	default values. The library configuration may be modified by
	the user using MHC, under Harmony Framework Configuration -> Class B</p></html>
	---endhelp---

ifblock CLASSB_LIBRARY${INSTANCE}

add "^#include \"/utilities/mhc/config/gencode/framework/classb/templates/classb_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/classb/templates/classb_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/classb/templates/classb_macros_system_config.h.ftl\">" to list APP_FREEMARKER_MACROS

endif

ifblock CLASSB_LIBRARY${INSTANCE}

add "^@macro_classb_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_classb_system_config_h_app_constants/>" to list APP${INSTANCE}_H_CONSTANTS
add "^@macro_classb_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_classb_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_classb_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_classb_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS
add "^@macro_classb_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES

add "^@macro_classb_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_classb_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_classb_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_classb_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_classb_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_classb_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_classb_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_classb_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_classb_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_classb_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES

endif

endmenu
