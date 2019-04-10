menu "OC"
	depends on HAVE_OC

config GENERATE_APP_CODE_DRV_OC${INSTANCE}
    bool "OC?"
    default n
    select USE_DRV_OC if GENERATE_APP_CODE_DRV_OC${INSTANCE}
    ---help---
    IDH_HTML_DRV_OC_Introduction
    ---endhelp---

ifblock GENERATE_APP_CODE_DRV_OC${INSTANCE}
	
config APP_DRV_OC_POLLING${INSTANCE}
    bool "Generate PWM signal on OCx pin"
	set DRV_OC_INTERRUPT_MODE to n if APP_DRV_OC_POLLING${INSTANCE}
    set DRV_TMR_DRIVER_MODE to "STATIC" if APP_DRV_OC_POLLING${INSTANCE}
	set DRV_TMR_INTERRUPT_MODE to n if APP_DRV_OC_POLLING${INSTANCE}
    default n
	---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony Output Compare Application Template</h2>
	<p>	This template generates a fixed pulse width signal on OC1 pin. 
	When timer period acts as the period of the PWM output and output compare
	value acts as duty cycle (Ex: With timer period of 2000 peripheral clocks and
	output compare value of 500 peripheral clocks, PWM output has a period of 2000
	peripheral clocks and a 25-75 duty cycle).<br>
	Application will automatically configure OC and TMR drivers
	with the following settings:</p>
	<br><b>- Output Compare Driver </b></br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Implementation: STATIC</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Mode: Polling</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Output Compare Mode: PWM edge aligned</br>
	<br>&nbsp&nbsp&nbsp&nbsp - PWM Pulse Width: 500 (peripheral clocks) </br>
	<br><b>- Timer Driver</b></br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Implementation: STATIC</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Mode: Polling</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Prescalar: 1</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Timer Period: 2000 (peripheral clocks)</br>	
	<p>All other relavent OC and Timer configuration options are set to their 
	default values.</p>
	<p>The OC driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> Drivers - > OC</p>
	<p>The Timer driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> Drivers - > Timer</p></html>
	</html>
	---endhelp---

menu "Options"
	
config APP_DRV_OC_INSTANCE_INDEX${INSTANCE}
    int "OC Driver Instance Index"
    range 0 0 if DRV_OC_INSTANCES_NUMBER = 1
    range 0 1 if DRV_OC_INSTANCES_NUMBER = 2
    range 0 2 if DRV_OC_INSTANCES_NUMBER = 3
    range 0 3 if DRV_OC_INSTANCES_NUMBER = 4
    range 0 4 if DRV_OC_INSTANCES_NUMBER = 5
    range 0 5 if DRV_OC_INSTANCES_NUMBER = 6
    range 0 6 if DRV_OC_INSTANCES_NUMBER = 7
    range 0 7 if DRV_OC_INSTANCES_NUMBER = 8
    range 0 8 if DRV_OC_INSTANCES_NUMBER = 9
    default 0
	
config APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE}
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

config APP_DRV_OC_HANDLE${INSTANCE}
    string "Name of the OC Driver Handle?"
    default "handleOC0" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 0
    default "handleOC1" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 1
    default "handleOC2" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 2
    default "handleOC3" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 3
    default "handleOC4" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 4
    default "handleOC5" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 5
    default "handleOC6" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 6
    default "handleOC7" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 7
    default "handleOC8" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 8
    default "handleOC9" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 9
    default "handleOC0"
	
config APP_DRV_OC_TMR_HANDLE${INSTANCE}
    string "Name of the Timer Driver Handle?"
    default "handleTMR0" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 0
    default "handleTMR1" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 1
    default "handleTMR2" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 2
    default "handleTMR3" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 3
    default "handleTMR4" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 4
    default "handleTMR5" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 5
    default "handleTMR6" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 6
    default "handleTMR7" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 7
    default "handleTMR8" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 8
    default "handleTMR9" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 9
    default "handleTMR0"

config APP_DRV_OC_PULSE_WIDTH${INSTANCE}
	int "PWM Pulse Width (Peripheral Clocks)"
	set DRV_OC_16BIT_PULSE_WIDTH_IDX0 to "500" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 0
	set DRV_OC_16BIT_PULSE_WIDTH_IDX1 to "500" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 1
	set DRV_OC_16BIT_PULSE_WIDTH_IDX2 to "500" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_OC_16BIT_PULSE_WIDTH_IDX3 to "500" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 3
	set DRV_OC_16BIT_PULSE_WIDTH_IDX4 to "500" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 4
	set DRV_OC_16BIT_PULSE_WIDTH_IDX5 to "500" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 5		
	set DRV_OC_16BIT_PULSE_WIDTH_IDX6 to "500" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_OC_16BIT_PULSE_WIDTH_IDX7 to "500" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 7
	set DRV_OC_16BIT_PULSE_WIDTH_IDX8 to "500" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 8
	default 500
	
config APP_DRV_OC_TMR_PERIOD${INSTANCE}
	int "Timer Period (Peripheral Clocks)"
	set DRV_TMR_PERIOD_IDX0 to "2000" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 0
	set DRV_TMR_PERIOD_IDX1 to "2000" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 1
	set DRV_TMR_PERIOD_IDX2 to "2000" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_TMR_PERIOD_IDX3 to "2000" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 3
	set DRV_TMR_PERIOD_IDX4 to "2000" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 4
	set DRV_TMR_PERIOD_IDX5 to "2000" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 5		
	set DRV_TMR_PERIOD_IDX6 to "2000" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_TMR_PERIOD_IDX7 to "2000" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 7
	set DRV_TMR_PERIOD_IDX8 to "2000" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 8
	default 2000	
endmenu

config APP_DRV_OC_MODES${INSTANCE}
	string
	set DRV_OC_COMPARE_MODES_IDX0 to "OC_COMPARE_PWM_EDGE_ALIGNED_MODE" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 0
	set DRV_OC_COMPARE_MODES_IDX1 to "OC_COMPARE_PWM_EDGE_ALIGNED_MODE" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 1
	set DRV_OC_COMPARE_MODES_IDX2 to "OC_COMPARE_PWM_EDGE_ALIGNED_MODE" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_OC_COMPARE_MODES_IDX3 to "OC_COMPARE_PWM_EDGE_ALIGNED_MODE" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 3
	set DRV_OC_COMPARE_MODES_IDX4 to "OC_COMPARE_PWM_EDGE_ALIGNED_MODE" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 4
	set DRV_OC_COMPARE_MODES_IDX5 to "OC_COMPARE_PWM_EDGE_ALIGNED_MODE" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 5		
	set DRV_OC_COMPARE_MODES_IDX6 to "OC_COMPARE_PWM_EDGE_ALIGNED_MODE" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_OC_COMPARE_MODES_IDX7 to "OC_COMPARE_PWM_EDGE_ALIGNED_MODE" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 7
	set DRV_OC_COMPARE_MODES_IDX8 to "OC_COMPARE_PWM_EDGE_ALIGNED_MODE" if APP_DRV_OC_INSTANCE_INDEX${INSTANCE} = 8	
	
config APP_DRV_OC_TMR_MODULE_ID${INSTANCE}
	string
	set DRV_TMR_PERIPHERAL_ID_IDX0 to "TMR_ID_2" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 0
	set DRV_TMR_PERIPHERAL_ID_IDX1 to "TMR_ID_2" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 1
	set DRV_TMR_PERIPHERAL_ID_IDX2 to "TMR_ID_2" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_TMR_PERIPHERAL_ID_IDX3 to "TMR_ID_2" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 3
	set DRV_TMR_PERIPHERAL_ID_IDX4 to "TMR_ID_2" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 4
	set DRV_TMR_PERIPHERAL_ID_IDX5 to "TMR_ID_2" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 5		
	set DRV_TMR_PERIPHERAL_ID_IDX6 to "TMR_ID_2" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_TMR_PERIPHERAL_ID_IDX7 to "TMR_ID_2" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 7
	set DRV_TMR_PERIPHERAL_ID_IDX8 to "TMR_ID_2" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 8
	
config APP_DRV_OC_TMR_PRESCALE${INSTANCE}
	string
	set DRV_TMR_PRESCALE_IDX0 optionally to "TMR_PRESCALE_VALUE_1" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 0
	set DRV_TMR_PRESCALE_IDX1 optionally to "TMR_PRESCALE_VALUE_1" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 1
	set DRV_TMR_PRESCALE_IDX2 optionally to "TMR_PRESCALE_VALUE_1" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_TMR_PRESCALE_IDX3 optionally to "TMR_PRESCALE_VALUE_1" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 3
	set DRV_TMR_PRESCALE_IDX4 optionally to "TMR_PRESCALE_VALUE_1" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 4
	set DRV_TMR_PRESCALE_IDX5 optionally to "TMR_PRESCALE_VALUE_1" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 5		
	set DRV_TMR_PRESCALE_IDX6 optionally to "TMR_PRESCALE_VALUE_1" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_TMR_PRESCALE_IDX7 optionally to "TMR_PRESCALE_VALUE_1" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 7
	set DRV_TMR_PRESCALE_IDX8 optionally to "TMR_PRESCALE_VALUE_1" if APP_DRV_OC_TMR_INSTANCE_INDEX${INSTANCE} = 8
	
add "^#include \"/utilities/mhc/config/gencode/framework/driver/oc/templates/oc_polling_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/driver/oc/templates/oc_polling_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS

	
add "^@macro_app_drv_oc_polling_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_app_drv_oc_polling_h_constants/>" to list APP${INSTANCE}_H_CONSTANTS
add "^@macro_app_drv_oc_polling_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_app_drv_oc_polling_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_app_drv_oc_polling_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_app_drv_oc_polling_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS
add "^@macro_app_drv_oc_polling_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES

add "^@macro_app_drv_oc_polling_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_app_drv_oc_polling_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_app_drv_oc_polling_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_app_drv_oc_polling_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_app_drv_oc_polling_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_app_drv_oc_polling_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_app_drv_oc_polling_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_app_drv_oc_polling_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_app_drv_oc_polling_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_app_drv_oc_polling_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
add "^@macro_app_drv_oc_polling_app_c_tasks_app_functions/>" to list APP${INSTANCE}_C_APP_TASKS_APP_FUNCTIONS

endif

endmenu
