menu "Power Factor Correction"

config PFC_AVG_CM${INSTANCE}
    bool "Power Factor Correction using Average Current Control on MCHV-3"
	set PFC_ENABLE optionally to y if PFC_AVG_CM${INSTANCE}
	set DRV_ADC_USE_DRIVER_NEEDED optionally to y if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_INTERRUPT_MODE optionally to y if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_POLLED_MODE optionally to n if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CLOCK_DIVIDER optionally to 1 if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_INDIVIDUAL_INTERRUPT_ENABLE to y if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_WARMUP_CLOCK optionally to "ADCHS_WARMUP_CLOCK_32" if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CHANNEL_INST_IDX3 optionally to y if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CHANNEL_ID_IDX3 optionally to "ADCHS_CHANNEL_4" if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CHANNEL_DATA_RDY_INTERRUPT_ENABLE_IDX3 optionally to y if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CHANNEL_DATA_RDY_INTERRUPT_PRIORITY_IDX3 optionally to "INT_PRIORITY_LEVEL4" if PFC_AVG_CM${INSTANCE}
    set DRV_ADCHS_CHANNEL_DATA_RDY_INTERRUPT_SUB_PRIORITY_IDX3 optionally to "INT_SUBPRIORITY_LEVEL0" if PFC_AVG_CM${INSTANCE}
   	set DRV_ADCHS_CHNL_4_ALT_INP_SEL_IDX3 optionally to "ADCHS_ALTERNATE_2_CLASS2_AN9" if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_TRIGGER_SOURCE_IDX3 optionally to "ADCHS_TRIGGER_SOURCE_PWM5" if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CHANNEL_CLOCK_DIVIDER_IDX3 optionally to 1 if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_SAMPLE_TIME_COUNT_IDX3 optionally to 2 if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CHANNEL_INST_IDX4 optionally to y if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CHANNEL_ID_IDX4 optionally to "ADCHS_CHANNEL_0" if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CHNL_0_ALT_INP_SEL_IDX4 optionally to "ADCHS_ALTERNATE_3_CLASS1_AN24" if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_TRIGGER_SOURCE_IDX4 optionally to "ADCHS_TRIGGER_SOURCE_PWM5" if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CHANNEL_CLOCK_DIVIDER_IDX4 optionally to 1 if PFC_AVG_CM${INSTANCE} 
	set DRV_ADCHS_SAMPLE_TIME_COUNT_IDX4 optionally to 2 if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CHANNEL_INST_IDX2 optionally to y if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CHANNEL_ID_IDX2 optionally to "ADCHS_CHANNEL_7" if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CHANNEL_CLOCK_DIVIDER_IDX2 optionally to 1 if PFC_AVG_CM${INSTANCE} 
	set DRV_ADCHS_SAMPLE_TIME_COUNT_IDX2 optionally to 2 if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CLASS_2_ANALOG_INPUT_INST_IDX0 optionally to  y if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CLASS_2_ANALOG_INPUT_IDX0 optionally to "ADCHS_CLASS2_AN10" if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_CLASS_2_TRIGGER_SOURCE_IDX0 optionally to "ADCHS_TRIGGER_SOURCE_PWM5" if PFC_AVG_CM${INSTANCE}
	set USE_DRV_ADCHS_SCAN_MODE optionally to n if PFC_AVG_CM${INSTANCE}
	set DRV_ADCHS_DIGITAL_COMPARATOR optionally to n if PFC_AVG_CM${INSTANCE}
	set USE_DRV_ADCHS_DIGITAL_FILTER optionally to n if PFC_AVG_CM${INSTANCE}
	set USE_DRV_MCPWM optionally to y if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_DRIVER_MODE optionally to "STATIC" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_STOP_IN_IDLE_MODE optionally to n if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_SECONDARY_CLOCK_PRESCALER optionally to "MCPWM_CLOCK_DIVIDE_BY_1" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_SECONDARY_TIMER_PERIOD optionally to "1500" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHOP_CLOCK_PRESCALER optionally to "2" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_INSTANCES_NUMBER optionally to 12 if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_INST_IDX3 optionally to y if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_ID_IDX3 optionally to "MCPWM_CHANNEL5" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_TIME_BASE_SOURCE_IDX3 optionally to "MCPWM_TIME_BASE_SOURCE_SECONDARY" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_TIME_BASE_MODE_IDX3 optionally to "MCPWM_TIME_BASE_SYNCHRONIZED" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_ALIGNMENT_MODE_IDX3 optionally to "MCPWM_EDGE_ALIGNED" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_OUTPUT_MODE_IDX3 optionally to "MCPWM_OUTPUT_LOW_LATCHED_TO_ZERO" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_PWMxH_OUTPUT_POLARITY_IDX3 optionally to "MCPWM_PWMxH_ACTIVEHIGH" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_PWMxL_OUTPUT_POLARITY_IDX3 optionally to "MCPWM_PWMxL_ACTIVEHIGH" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_DEADTIME_MODE_IDX3 optionally to "MCPWM_DEADTIME_DISABLE" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_PHASE_VALUE_IDX3 optionally to "375" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_TRIGGER_POSTSCALER_IDX3 optionally to "MCPWM_TRIGGER_DIVIDE_BY_2" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_ADC_TRIGGER_SOURCE_IDX3 optionally to "MCPWM_ADC_TRIGGER_SOURCE_PRIMARY" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_TRIGGER_INTERRUPT_SOURCE_IDX3 optionally to "MCPWM_TRIGGER_INTERRUPT_SOURCE_PRIMARY" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_PRIMARY_TRIGGER_COMPARE_VALUE_IDX3 optionally to "0" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_LEB_TRIGGER_PWMxHRISINGEDGE_ENABLE_IDX3 optionally to y if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_FAULT_INPUT_LEB_CONTROL_IDX3 optionally to y if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_CURRENTLIMIT_INPUT_LEB_CONTROL_IDX3 optionally to n if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_LEB_PERIOD_VALUE_IDX3 optionally to "100" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_FAULT_SOURCE_IDX3 optionally to "MCPWM_FAULT_SOURCE_IS_COMPARATOR3" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_FAULT_INPUT_POLARITY_IDX3 optionally to "MCPWM_FAULT_INPUT_POLARITY_ACTIVE_LOW" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_FAULT_OVERRIDE_PWMxH_VALUE_IDX3 optionally to "MCPWM_FAULT_OVERRIDE_PWMxH_0" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_FAULT_OVERRIDE_PWMxL_VALUE_IDX3 optionally to "MCPWM_FAULT_OVERRIDE_PWMxL_0" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_FAULT_MODE_IDX3 optionally to "MCPWM_FAULT_MODE_DISABLED" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_OVERRIDE_PWMxH_VALUE_IDX3 optionally to "MCPWM_OVERRIDE_PWMxH_0" if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_PWMH_ENABLE_IDX3 optionally to y if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_PWML_ENABLE_IDX3 optionally to y if PFC_AVG_CM${INSTANCE}
	set DRV_MCPWM_CHANNEL_SWAP_PWMH_PWML_IDX3 optionally to n if PFC_AVG_CM${INSTANCE}
	set ICESEL optionally to "ICS_PGx2" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_1_FUNCTION_NAME optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_1_FUNCTION_TYPE optionally to "GPIO_OUT" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_1_PORT_PIN optionally to "15" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_1_PORT_CHANNEL optionally to "G" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_1_MODE optionally to "DIGITAL" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_1_DIR optionally to "Out" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_1_LAT optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_1_OD optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_1_CN optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_1_PU optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_1_PD optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_7_FUNCTION_NAME to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_7_FUNCTION_TYPE to "PWM5H" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_7_PORT_PIN to "2" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_7_PORT_CHANNEL to "D" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_7_MODE to "DIGITAL" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_20_FUNCTION_NAME optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_20_FUNCTION_TYPE optionally to "AN10" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_20_PORT_PIN optionally to "12" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_20_PORT_CHANNEL optionally to "A" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_20_MODE optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_20_DIR optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_20_LAT optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_20_OD optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_20_CN optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_20_PU optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_20_PD optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_42_FUNCTION_NAME to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_42_FUNCTION_TYPE to "GPIO_OUT" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_42_PORT_PIN to "13" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_42_PORT_CHANNEL to "E" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_42_MODE to "DIGITAL" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_42_DIR to "Out" if PFC_AVG_CM${INSTANCE}	 
	set BSP_PIN_42_LAT to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_42_OD to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_42_CN to "" if PFC_AVG_CM${INSTANCE} 
	set BSP_PIN_42_PU to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_42_PD to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_61_FUNCTION_NAME optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_61_FUNCTION_TYPE optionally to "GPIO_OUT" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_61_PORT_PIN optionally to "5" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_61_PORT_CHANNEL optionally to "F" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_61_MODE optionally to "DIGITAL" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_61_DIR optionally to "Out" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_61_LAT optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_61_OD optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_61_CN optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_61_PU optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_61_PD optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_63_FUNCTION_NAME optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_63_FUNCTION_TYPE optionally to "CLKI" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_63_PORT_PIN optionally to "12" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_63_PORT_CHANNEL optionally to "C" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_63_MODE optionally to "DIGITAL" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_63_DIR optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_63_LAT optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_63_OD optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_63_CN optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_63_PU optionally to "" if PFC_AVG_CM${INSTANCE}
	set BSP_PIN_63_PD optionally to "" if PFC_AVG_CM${INSTANCE}
	

	
	default n
	---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony Power Factor Correction using Average Current Control Application Template</h2>
	<p>	This template generates a code example that exercises Power Factor Correction using Average Current Control with the following MHC options:</p>
	<br><b>- ADC Driver </b></br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Implementation: STATIC</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Mode: Interrupt</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Clock Divider: 1</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Warmup Clock: ADCHS_WARMUP_CLOCK_32</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Interrupt Source: ADC1_DATA4</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Interrupt Vector: ADC1_DATA4</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Number of ADC Channels: 3</br>
	<br><b>&nbsp&nbsp - Channel 3 Setup</b></br>
	<br>&nbsp&nbsp&nbsp&nbsp - ADC instance: SAR4</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Alternate Input: ADCHS_ALTERNATE_2_CLASS1_AN9</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Trigger Source: ADCHS_TRIGGER_SOURCE_PWM5</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Clock Divider: 1</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Sample Time Count: 2</br>
	<br><b>&nbsp&nbsp - Channel 4 Setup</b></br>
	<br>&nbsp&nbsp&nbsp&nbsp - ADC instance: SAR0</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Alternate Input: ADCHS_ALTERNATE_3_CLASS1_AN24</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Trigger Source: ADCHS_TRIGGER_SOURCE_PWM5</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Clock Divider: 1</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Sample Time Count: 2</br>
	<br><b>&nbsp&nbsp - Channel 2 Setup</b></br>
	<br>&nbsp&nbsp&nbsp&nbsp - ADC instance: SAR7</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Clock Divider: 1</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Sample Time Count: 2</br>	
	<br>&nbsp&nbsp&nbsp&nbsp - Inputs (class-2): ADCHS_CLASS2_AN10  </br>
	<br>&nbsp&nbsp&nbsp&nbsp - Triggers: PWM5 </br>
	<br>&nbsp&nbsp&nbsp&nbsp - Sample Time Count: 2</br>	
	<br><b>- MCPWM Driver</b></br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Implementation: STATIC</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Primary Timer Period: 1500</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Primary Timer Clock Divider: 1</br>
	<br><b>&nbsp&nbsp - MCPWM Channel Instance 3 Setup</b></br>
	<br>&nbsp&nbsp&nbsp&nbsp - MCPWM Channel ID: MCPWM Channel 5</br>	
	<br>&nbsp&nbsp&nbsp&nbsp - Time Base Source: Secondary Time Base</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Time Base Mode: Synchronized </br>
	<br>&nbsp&nbsp&nbsp&nbsp - Alignment Mode: Edge Aligned </br>
	<br>&nbsp&nbsp&nbsp&nbsp - Output Mode: Complimentary </br>
	<br>&nbsp&nbsp&nbsp&nbsp - PWMH Polarity: Active High </br>
	<br>&nbsp&nbsp&nbsp&nbsp - PWML Polarity: Active High </br>
	<br>&nbsp&nbsp&nbsp&nbsp - Deadtime Mode: Disabled </br>
	<br>&nbsp&nbsp&nbsp&nbsp - Trigger Post-Scaler: 2 </br>
	<br>&nbsp&nbsp&nbsp&nbsp - Fault Input LEB : Enable </br>
	<br>&nbsp&nbsp&nbsp&nbsp - LEB Period : 100 </br>
	<br>&nbsp&nbsp&nbsp&nbsp - Fault Mode : LATCHED </br>
	<br>&nbsp&nbsp&nbsp&nbsp - Fault Source : Comparator 3 </br>
	<br>&nbsp&nbsp&nbsp&nbsp - Fault Input Polarity : Active Low </br>
	<br>&nbsp&nbsp&nbsp&nbsp - PWMH Fault Override Value  : Low </br>
	<br>&nbsp&nbsp&nbsp&nbsp - PWML Fault Override Value  : Low </br>
	<br>&nbsp&nbsp&nbsp&nbsp - Channel Interrupt  : Enable </br>
	<br>&nbsp&nbsp&nbsp&nbsp - Fault Interrupt  : Enable </br>
	<p>All other ADC and PWM driver configuration options are set optionally to their 
	default values. 
	<p>The ADC driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> Drivers - > ADC</p>
	<p>The PWM driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> Drivers - > PWM</p></html>
	---endhelp---


ifblock PFC_AVG_CM${INSTANCE}

add "^#include \"/utilities/mhc/config/gencode/framework/power_factor_correction/templates/pfc_avg_current_control_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/power_factor_correction/templates/pfc_avg_current_control_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/power_factor_correction/templates/pfc_avg_current_control_macros_system_config.h.ftl\">" to list APP_FREEMARKER_MACROS

endif

ifblock PFC_AVG_CM${INSTANCE}

add "^@macro_pfc_avg_current_control_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_pfc_avg_current_control_system_config_h_app_constants/>" to list APP${INSTANCE}_H_CONSTANTS
add "^@macro_pfc_avg_current_control_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_pfc_avg_current_control_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_pfc_avg_current_control_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_pfc_avg_current_control_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS
add "^@macro_pfc_avg_current_control_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES

add "^@macro_pfc_avg_current_control_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_pfc_avg_current_control_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_pfc_avg_current_control_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_pfc_avg_current_control_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_pfc_avg_current_control_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_pfc_avg_current_control_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_pfc_avg_current_control_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_pfc_avg_current_control_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_pfc_avg_current_control_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_pfc_avg_current_control_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
endif


endmenu
