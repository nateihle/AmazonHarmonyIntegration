menu "IC"
	depends on HAVE_IC

config GENERATE_APP_CODE_DRV_IC${INSTANCE}
    bool "IC?"
    default n
    select USE_DRV_IC if GENERATE_APP_CODE_DRV_IC${INSTANCE}
    ---help---
    IDH_HTML_DRV_IC_Introduction
    ---endhelp---

ifblock GENERATE_APP_CODE_DRV_IC${INSTANCE}
	
config APP_DRV_IC_POLLING${INSTANCE}
    bool "Capture timer value(s) on rising edge of ICx pin in Polling Mode"
	set DRV_IC_INTERRUPT_MODE to n if APP_DRV_IC_POLLING${INSTANCE}
    set DRV_TMR_DRIVER_MODE to "STATIC" if APP_DRV_IC_POLLING${INSTANCE}
	set DRV_TMR_INTERRUPT_MODE to n if APP_DRV_IC_POLLING${INSTANCE}
    default n
	---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony Input Capture Application Template</h2>
	<p>	This template generates a simple code example which captures timer
	value on rising edge of IC1 pin. Application will automatically configure IC and TMR drivers
	with the following settings:</p>
	<br><b>- Input Capture Driver </b></br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Implementation: STATIC</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Mode: Polling</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Input Capture Mode: Rising edge</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Input Capture Buffer Size: 32-bit</br>
	<br><b>- Timer Driver</b></br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Implementation: STATIC</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Mode: Polling</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Operation Mode: 32-bit</br>
	<p>All other relavent IC and Timer configuration options are set to their 
	default values.</p>
	<p>The IC driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> Drivers - > IC</p>
	<p>The Timer driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> Drivers - > Timer</p></html>
	</html>
	---endhelp---

config APP_DRV_IC_POLLING_NUM_EDGES${INSTANCE}
    int "Number of Captures"
	depends on APP_DRV_IC_POLLING${INSTANCE}
	default 32
	---help---
	This represents number of Input Capture edges that application captures timer value on.
	---endhelp---
	
menu "Options"
	
config APP_DRV_IC_INSTANCE_INDEX${INSTANCE}
    int "IC Driver Instance Index"
    range 0 0 if DRV_IC_INSTANCES_NUMBER = 1
    range 0 1 if DRV_IC_INSTANCES_NUMBER = 2
    range 0 2 if DRV_IC_INSTANCES_NUMBER = 3
    range 0 3 if DRV_IC_INSTANCES_NUMBER = 4
    range 0 4 if DRV_IC_INSTANCES_NUMBER = 5
    range 0 5 if DRV_IC_INSTANCES_NUMBER = 6
    range 0 6 if DRV_IC_INSTANCES_NUMBER = 7
    range 0 7 if DRV_IC_INSTANCES_NUMBER = 8
    range 0 8 if DRV_IC_INSTANCES_NUMBER = 9
    default 0
	
config APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE}
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

config APP_DRV_IC_HANDLE${INSTANCE}
    string "Name of the IC Driver Handle?"
    default "handleIC0" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 0
    default "handleIC1" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 1
    default "handleIC2" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 2
    default "handleIC3" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 3
    default "handleIC4" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 4
    default "handleIC5" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 5
    default "handleIC6" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 6
    default "handleIC7" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 7
    default "handleIC8" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 8
    default "handleIC9" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 9
    default "handleIC0"
	
config APP_DRV_IC_TMR_HANDLE${INSTANCE}
    string "Name of the Timer Driver Handle?"
    default "handleTMR0" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 0
    default "handleTMR1" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 1
    default "handleTMR2" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 2
    default "handleTMR3" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 3
    default "handleTMR4" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 4
    default "handleTMR5" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 5
    default "handleTMR6" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 6
    default "handleTMR7" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 7
    default "handleTMR8" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 8
    default "handleTMR9" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 9
    default "handleTMR0"	
endmenu

config APP_DRV_IC_MODE${INSTANCE}
	string
	set DRV_IC_INPUT_CAPTURE_MODES_IDX0 to "IC_INPUT_CAPTURE_RISING_EDGE_MODE" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 0
	set DRV_IC_INPUT_CAPTURE_MODES_IDX1 to "IC_INPUT_CAPTURE_RISING_EDGE_MODE" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 1
	set DRV_IC_INPUT_CAPTURE_MODES_IDX2 to "IC_INPUT_CAPTURE_RISING_EDGE_MODE" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_IC_INPUT_CAPTURE_MODES_IDX3 to "IC_INPUT_CAPTURE_RISING_EDGE_MODE" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 3
	set DRV_IC_INPUT_CAPTURE_MODES_IDX4 to "IC_INPUT_CAPTURE_RISING_EDGE_MODE" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 4
	set DRV_IC_INPUT_CAPTURE_MODES_IDX5 to "IC_INPUT_CAPTURE_RISING_EDGE_MODE" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 5		
	set DRV_IC_INPUT_CAPTURE_MODES_IDX6 to "IC_INPUT_CAPTURE_RISING_EDGE_MODE" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_IC_INPUT_CAPTURE_MODES_IDX7 to "IC_INPUT_CAPTURE_RISING_EDGE_MODE" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 7
	set DRV_IC_INPUT_CAPTURE_MODES_IDX8 to "IC_INPUT_CAPTURE_RISING_EDGE_MODE" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 8

config APP_DRV_IC_BUFFER_SIZE${INSTANCE}
	string
	set DRV_IC_BUFFER_SIZE_IDX0 to "IC_BUFFER_SIZE_32BIT" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 0
	set DRV_IC_BUFFER_SIZE_IDX1 to "IC_BUFFER_SIZE_32BIT" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 1
	set DRV_IC_BUFFER_SIZE_IDX2 to "IC_BUFFER_SIZE_32BIT" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_IC_BUFFER_SIZE_IDX3 to "IC_BUFFER_SIZE_32BIT" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 3
	set DRV_IC_BUFFER_SIZE_IDX4 to "IC_BUFFER_SIZE_32BIT" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 4
	set DRV_IC_BUFFER_SIZE_IDX5 to "IC_BUFFER_SIZE_32BIT" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 5		
	set DRV_IC_BUFFER_SIZE_IDX6 to "IC_BUFFER_SIZE_32BIT" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_IC_BUFFER_SIZE_IDX7 to "IC_BUFFER_SIZE_32BIT" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 7
	set DRV_IC_BUFFER_SIZE_IDX8 to "IC_BUFFER_SIZE_32BIT" if APP_DRV_IC_INSTANCE_INDEX${INSTANCE} = 8

config APP_DRV_IC_TMR_MODULE_ID${INSTANCE}
	string
	set DRV_TMR_PERIPHERAL_ID_IDX0 to "TMR_ID_2" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 0
	set DRV_TMR_PERIPHERAL_ID_IDX1 to "TMR_ID_2" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 1
	set DRV_TMR_PERIPHERAL_ID_IDX2 to "TMR_ID_2" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_TMR_PERIPHERAL_ID_IDX3 to "TMR_ID_2" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 3
	set DRV_TMR_PERIPHERAL_ID_IDX4 to "TMR_ID_2" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 4
	set DRV_TMR_PERIPHERAL_ID_IDX5 to "TMR_ID_2" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 5		
	set DRV_TMR_PERIPHERAL_ID_IDX6 to "TMR_ID_2" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_TMR_PERIPHERAL_ID_IDX7 to "TMR_ID_2" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 7
	set DRV_TMR_PERIPHERAL_ID_IDX8 to "TMR_ID_2" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 8
	
config APP_DRV_IC_TMR_OPERATION_MODE${INSTANCE}
	string
	set DRV_TMR_OPERATION_MODE_IDX0 optionally to "DRV_TMR_OPERATION_MODE_32_BIT" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 0
	set DRV_TMR_OPERATION_MODE_IDX1 optionally to "DRV_TMR_OPERATION_MODE_32_BIT" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 1
	set DRV_TMR_OPERATION_MODE_IDX2 optionally to "DRV_TMR_OPERATION_MODE_32_BIT" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_TMR_OPERATION_MODE_IDX3 optionally to "DRV_TMR_OPERATION_MODE_32_BIT" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 3
	set DRV_TMR_OPERATION_MODE_IDX4 optionally to "DRV_TMR_OPERATION_MODE_32_BIT" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 4
	set DRV_TMR_OPERATION_MODE_IDX5 optionally to "DRV_TMR_OPERATION_MODE_32_BIT" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 5		
	set DRV_TMR_OPERATION_MODE_IDX6 optionally to "DRV_TMR_OPERATION_MODE_32_BIT" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_TMR_OPERATION_MODE_IDX7 optionally to "DRV_TMR_OPERATION_MODE_32_BIT" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 7
	set DRV_TMR_OPERATION_MODE_IDX8 optionally to "DRV_TMR_OPERATION_MODE_32_BIT" if APP_DRV_IC_TMR_INSTANCE_INDEX${INSTANCE} = 8

add "^#include \"/utilities/mhc/config/gencode/framework/driver/ic/templates/ic_polling_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/driver/ic/templates/ic_polling_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS

	
add "^@macro_app_drv_ic_polling_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_app_drv_ic_polling_h_constants/>" to list APP${INSTANCE}_H_CONSTANTS
add "^@macro_app_drv_ic_polling_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_app_drv_ic_polling_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_app_drv_ic_polling_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_app_drv_ic_polling_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS
add "^@macro_app_drv_ic_polling_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES

add "^@macro_app_drv_ic_polling_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_app_drv_ic_polling_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_app_drv_ic_polling_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_app_drv_ic_polling_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_app_drv_ic_polling_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_app_drv_ic_polling_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_app_drv_ic_polling_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_app_drv_ic_polling_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_app_drv_ic_polling_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_app_drv_ic_polling_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
add "^@macro_app_drv_ic_polling_app_c_tasks_app_functions/>" to list APP${INSTANCE}_C_APP_TASKS_APP_FUNCTIONS

endif

endmenu
