config DRV_MCPWM_CHANNEL_INSTANCES_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_DRV_MCPWM
<#if INSTANCE != 0>
	default n if DRV_MCPWM_CHANNEL_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_MCPWM_CHANNEL_INSTANCES_NUMBER = ${INSTANCE+1}
	default y
	
config DRV_MCPWM_INST_IDX${INSTANCE}
    depends on USE_DRV_MCPWM 
<#if INSTANCE != 0>
	             && DRV_MCPWM_CHANNEL_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "MCPWM Channel Instance ${INSTANCE}"
    default n

ifblock DRV_MCPWM_INST_IDX${INSTANCE}

config DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE}
    string "MCPWM Channel ID"
    depends on USE_DRV_MCPWM
    range MCPWM_CHANNEL_ID
    default "MCPWM_CHANNEL1"
    ---help---
	IDH_HTML_MCPWM_CHANNEL_SETUP
    ---endhelp---
    

	
config DRV_MCPWM_CHANNEL_TIME_BASE_SOURCE_IDX${INSTANCE}
    string "Channel Time Base Source"
    depends on USE_DRV_MCPWM
    range MCPWM_TIME_BASE_SOURCE
    default "MCPWM_TIME_BASE_SOURCE_PRIMARY"
    ---help---
    IDH_HTML_MCPWM_CHANNEL_SETUP
    ---endhelp---
	
config DRV_MCPWM_CHANNEL_TIME_BASE_MODE_IDX${INSTANCE}
    string "Channel Time Base Mode"
    depends on USE_DRV_MCPWM
    range MCPWM_TIME_BASE_MODE
    default "MCPWM_TIME_BASE_SYNCHRONIZED"
    ---help---
    IDH_HTML_MCPWM_CHANNEL_SETUP
    ---endhelp---
	
config DRV_MCPWM_CHANNEL_ALIGNMENT_MODE_IDX${INSTANCE}
    string "Channel Alignment Mode"
    depends on USE_DRV_MCPWM
    range MCPWM_ALIGNMENT_MODE
    default "MCPWM_EDGE_ALIGNED"
    ---help---
    IDH_HTML_MCPWM_CHANNEL_SETUP
    ---endhelp---
	
config DRV_MCPWM_CHANNEL_OUTPUT_MODE_IDX${INSTANCE}
    string "Channel Output Mode"
    depends on USE_DRV_MCPWM
    range MCPWM_OUTPUT_MODE
    default "MCPWM_OUTPUT_COMPLIMENTARY_MODE"
    ---help---
    IDH_HTML_MCPWM_CHANNEL_SETUP
    ---endhelp---
	
config DRV_MCPWM_CHANNEL_PWMxH_OUTPUT_POLARITY_IDX${INSTANCE}
    string "Channel PWMH Output Polarity"
    depends on USE_DRV_MCPWM
    range MCPWM_PWMxH_OUTPUT_POLARITY
    default "MCPWM_PWMxH_ACTIVEHIGH"
    ---help---
    IDH_HTML_MCPWM_CHANNEL_SETUP
    ---endhelp---

	config DRV_MCPWM_CHANNEL_PWMxL_OUTPUT_POLARITY_IDX${INSTANCE}
    string "Channel PWML Output Polarity"
    depends on USE_DRV_MCPWM
    range MCPWM_PWMxL_OUTPUT_POLARITY
    default "MCPWM_PWMxL_ACTIVEHIGH"
    ---help---
    IDH_HTML_MCPWM_CHANNEL_SETUP
    ---endhelp---

	
config DRV_MCPWM_CHANNEL_DEADTIME_MODE_IDX${INSTANCE}
    string "Channel Deadtime Mode"
    depends on USE_DRV_MCPWM
    range MCPWM_DEADTIME_MODE
    default "MCPWM_DEADTIME_POSITIVE"
    ---help---
    IDH_HTML_MCPWM_CHANNEL_SETUP
    ---endhelp---
	
config DRV_MCPWM_CHANNEL_DEADTIME_COMPENSATION_POLARITY_IDX${INSTANCE}
    string "Channel Deadtime Compensation Input Polarity"
    depends on USE_DRV_MCPWM
    range MCPWM_DEADTIME_COMPENSATION_POLARITY
    default "MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_HIGH"
    ---help---
    IDH_HTML_MCPWM_CHANNEL_SETUP
    ---endhelp---

config DRV_MCPWM_PDC_VALUE_IDX${INSTANCE}
	int "Primary Duty Cycle Value"
	depends on USE_DRV_MCPWM
	range 0 65535
	default 0
	---help---
	IDH_HTML_MCPWM_CHANNEL_SETUP
	---endhelp---
	
config DRV_MCPWM_SDC_VALUE_IDX${INSTANCE}
	int "Secondary Duty Cycle Value"
	depends on USE_DRV_MCPWM
	range 0 65535
	default 0
	---help---
	IDH_HTML_MCPWM_CHANNEL_SETUP
	---endhelp---
	
config DRV_MCPWM_PHASE_VALUE_IDX${INSTANCE}
	int "Phase Value"
	depends on USE_DRV_MCPWM
	range 0 65535
	default 0
	---help---
	IDH_HTML_MCPWM_CHANNEL_SETUP
	---endhelp---

config DRV_MCPWM_DTR_VALUE_IDX${INSTANCE}
	int "PWMxH Deadtime Value"
	depends on USE_DRV_MCPWM
	range 0 16383
	default 0
	---help---
	IDH_HTML_MCPWM_CHANNEL_SETUP
	---endhelp---
	
config DRV_MCPWM_ALTDTR_VALUE_IDX${INSTANCE}
	int "PWMxL Deadtime Value"
	depends on USE_DRV_MCPWM
	range 0 16383
	default 0
	---help---
	IDH_HTML_MCPWM_CHANNEL_SETUP
	---endhelp---
	
config DRV_MCPWM_DTCOMP_VALUE_IDX${INSTANCE}
	int "PWM Deadtime Compensation Value"
	depends on USE_DRV_MCPWM
	range 0 16383
	default 0
	---help---
	IDH_HTML_MCPWM_CHANNEL_SETUP
	---endhelp---
	
menu "Channel Trigger Setup"
	depends on USE_DRV_MCPWM

		config DRV_MCPWM_CHANNEL_TRIGGER_POSTSCALER_IDX${INSTANCE}
			string "Channel Trigger Post-scaler"
			depends on USE_DRV_MCPWM
			range MCPWM_TRIGGER_DIVIDER
			default "MCPWM_TRIGGER_DIVIDE_BY_1"
			---help---
			IDH_HTML_MCPWM_CHANNEL_TRIGGER_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_PRIMARY_TRIGGER_CYCLE_SELECT_IDX${INSTANCE}
			string "Channel Primary Trigger Cycle Select"
			depends on USE_DRV_MCPWM
			range MCPWM_PRIMARY_TRIGGER_CYCLE_SELECT
			default "MCPWM_PRIMARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING"
			---help---
			IDH_HTML_MCPWM_CHANNEL_TRIGGER_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_SECONDARY_TRIGGER_CYCLE_SELECT_IDX${INSTANCE}
			string "Channel Secondary Trigger Cycle Select"
			depends on USE_DRV_MCPWM
			range MCPWM_SECONDARY_TRIGGER_CYCLE_SELECT
			default "MCPWM_SECONDARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING"
			---help---
			IDH_HTML_MCPWM_CHANNEL_TRIGGER_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_ADC_TRIGGER_SOURCE_IDX${INSTANCE}
			string "Channel ADC Trigger Source"
			depends on USE_DRV_MCPWM
			range MCPWM_ADC_TRIGGER_SOURCE
			default "MCPWM_ADC_TRIGGER_SOURCE_PRIMARY"
			---help---
			IDH_HTML_MCPWM_CHANNEL_TRIGGER_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_TRIGGER_INTERRUPT_SOURCE_IDX${INSTANCE}
			string "Channel Trigger Interrupt Source"
			depends on USE_DRV_MCPWM
			range MCPWM_TRIGGER_INTERRUPT_SOURCE
			default "MCPWM_TRIGGER_INTERRUPT_SOURCE_PRIMARY"
			---help---
			IDH_HTML_MCPWM_CHANNEL_TRIGGER_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_PRIMARY_TRIGGER_COMPARE_VALUE_IDX${INSTANCE}
			int "Channel Primary Trigger Compare Value"
			depends on USE_DRV_MCPWM
			range 0 65535
			default 0
			---help---
			IDH_HTML_MCPWM_CHANNEL_TRIGGER_SETUP
			---endhelp---

		config DRV_MCPWM_CHANNEL_SECONDARY_TRIGGER_COMPARE_VALUE_IDX${INSTANCE}
			int "Channel Secondary Trigger Compare Value"
			depends on USE_DRV_MCPWM
			range 0 65535
			default 0
			---help---
			IDH_HTML_MCPWM_CHANNEL_TRIGGER_SETUP
			---endhelp---
			

endmenu

menu "Channel Leading Edge Blanking (LEB) Setup"
	depends on USE_DRV_MCPWM
	
	menu "Select LEB Trigger Sources"
		depends on USE_DRV_MCPWM
			config DRV_MCPWM_CHANNEL_LEB_TRIGGER_PWMxHRISINGEDGE_ENABLE_IDX${INSTANCE}
				bool "PWMH Rising Edge"
				depends on USE_DRV_MCPWM
				default n
				---help---
				IDH_HTML_MCPWM_CHANNEL_LEB_SETUP
				---endhelp---
			config DRV_MCPWM_CHANNEL_LEB_TRIGGER_PWMxHFALLINGEDGE_ENABLE_IDX${INSTANCE}
				bool "PWMH Falling Edge"
				depends on USE_DRV_MCPWM
				default n
				---help---
				IDH_HTML_MCPWM_CHANNEL_LEB_SETUP
				---endhelp---
			config DRV_MCPWM_CHANNEL_LEB_TRIGGER_PWMxLRISINGEDGE_ENABLE_IDX${INSTANCE}
				bool "PWML Rising Edge"
				depends on USE_DRV_MCPWM
				default n
				---help---
				IDH_HTML_MCPWM_CHANNEL_LEB_SETUP
				---endhelp---
			config DRV_MCPWM_CHANNEL_LEB_TRIGGER_PWMxLFALLINGEDGE_ENABLE_IDX${INSTANCE}
				bool "PWML Falling Edge"
				depends on USE_DRV_MCPWM
				default n
				---help---
				IDH_HTML_MCPWM_CHANNEL_LEB_SETUP
				---endhelp---
	endmenu
	
		config DRV_MCPWM_CHANNEL_FAULT_INPUT_LEB_CONTROL_IDX${INSTANCE}
			bool "Enable LEB on Fault Input"
			depends on USE_DRV_MCPWM
			default n
			---help---
			IDH_HTML_MCPWM_CHANNEL_LEB_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_CURRENTLIMIT_INPUT_LEB_CONTROL_IDX${INSTANCE}
			bool "Enable LEB on Current Limit Input"
			depends on USE_DRV_MCPWM
			default n
			---help---
			IDH_HTML_MCPWM_CHANNEL_LEB_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_LEB_PERIOD_VALUE_IDX${INSTANCE}
			int "Channel LEB Period Value"
			depends on USE_DRV_MCPWM
			range 0 4095
			default 0
			---help---
			IDH_HTML_MCPWM_CHANNEL_LEB_SETUP
			---endhelp---
endmenu

menu "Channel Chop Setup"
	depends on USE_DRV_MCPWM
	
		config DRV_MCPWM_CHANNEL_CHOP_CLOCK_SOURCE_IDX${INSTANCE}
			string "Channel Chop Clock Source"
			depends on USE_DRV_MCPWM
			range MCPWM_CHOP_CLOCK_SOURCE
			default "MCPWM_CHOP_CLOCK_SOURCE_IS_CHOP_CLOCK_GENERATOR"
			---help---
			IDH_HTML_MCPWM_CHANNEL_CHOP_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_PWMxH_CHOP_CONTROL_IDX${INSTANCE}
			string "Channel PWMH Chop Control"
			depends on USE_DRV_MCPWM
			range MCPWM_PWMxH_CHOP_CONTROL
			default "MCPWM_PWMxH_CHOP_DISABLED"
			---help---
			IDH_HTML_MCPWM_CHANNEL_CHOP_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_PWMxL_CHOP_CONTROL_IDX${INSTANCE}
			string "Channel PWML Chop Control"
			depends on USE_DRV_MCPWM
			range MCPWM_PWMxL_CHOP_CONTROL
			default "MCPWM_PWMxL_CHOP_DISABLED"
			---help---
			IDH_HTML_MCPWM_CHANNEL_CHOP_SETUP
			---endhelp---
endmenu

menu "Channel Fault Setup"
	depends on USE_DRV_MCPWM
		

		config DRV_MCPWM_CHANNEL_FAULT_SOURCE_IDX${INSTANCE}
			string "Channel Fault Source"
			depends on USE_DRV_MCPWM
			range MCPWM_FAULT_SOURCE
			default "MCPWM_FAULT_SOURCE_IS_FLT15"
			---help---
			IDH_HTML_MCPWM_CHANNEL_FAULT_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_FAULT_INPUT_POLARITY_IDX${INSTANCE}
			string "Channel Fault Input Polarity"
			depends on USE_DRV_MCPWM
			range MCPWM_FAULT_INPUT_POLARITY
			default "MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH"
			---help---
			IDH_HTML_MCPWM_CHANNEL_FAULT_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_FAULT_OVERRIDE_PWMxH_VALUE_IDX${INSTANCE}
			string "Channel PWMH Fault Override Output"
			depends on USE_DRV_MCPWM
			range MCPWM_FAULT_OVERRIDE_PWMxH_VALUE
			default "MCPWM_FAULT_OVERRIDE_PWMxH_0"
			---help---
			IDH_HTML_MCPWM_CHANNEL_FAULT_SETUP
			---endhelp---
			config DRV_MCPWM_CHANNEL_FAULT_OVERRIDE_PWMxL_VALUE_IDX${INSTANCE}
		string "Channel PWML Fault Override Output"
			depends on USE_DRV_MCPWM
			range MCPWM_FAULT_OVERRIDE_PWMxL_VALUE
			default "MCPWM_FAULT_OVERRIDE_PWMxL_0"
			---help---
			IDH_HTML_MCPWM_CHANNEL_FAULT_SETUP
			---endhelp---
			config DRV_MCPWM_CHANNEL_FAULT_MODE_IDX${INSTANCE}
		string "Channel Fault Mode"
			depends on USE_DRV_MCPWM
			range MCPWM_FAULT_MODE
			default "MCPWM_FAULT_MODE_DISABLED"
			---help---
			IDH_HTML_MCPWM_CHANNEL_FAULT_SETUP
			---endhelp---
endmenu
	
menu "Channel Current Limit Setup"
	depends on USE_DRV_MCPWM
		

		config DRV_MCPWM_CHANNEL_CURRENTLIMIT_SOURCE_IDX${INSTANCE}
			string "Channel Current Limit Source"
			depends on USE_DRV_MCPWM
			range MCPWM_CURRENTLIMIT_SOURCE
			default "MCPWM_CURRENTLIMIT_SOURCE_IS_FLT15"
			---help---
			IDH_HTML_MCPWM_CHANNEL_CURRENTLIMIT_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_CURRENTLIMIT_INPUT_POLARITY_IDX${INSTANCE}
			string "Channel Current Limit Input Polarity"
			depends on USE_DRV_MCPWM
			range MCPWM_CURRENTLIMIT_INPUT_POLARITY
			default "MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH"
			---help---
			IDH_HTML_MCPWM_CHANNEL_CURRENTLIMIT_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_CURRENTLIMIT_OVERRIDE_PWMxH_VALUE_IDX${INSTANCE}
			string "Channel PWMH Current Limit Override Output"
			depends on USE_DRV_MCPWM
			range MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_VALUE
			default "MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0"
			---help---
			IDH_HTML_MCPWM_CHANNEL_CURRENTLIMIT_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_CURRENTLIMIT_OVERRIDE_PWMxL_VALUE_IDX${INSTANCE}
			string "Channel PWML Current Limit Override Output"
			depends on USE_DRV_MCPWM
			range MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_VALUE
			default "MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0"
			---help---
			IDH_HTML_MCPWM_CHANNEL_CURRENTLIMIT_SETUP
			---endhelp---

		config DRV_MCPWM_CHANNEL_CURRENTLIMIT_MODE_IDX${INSTANCE}
			string "Channel Current Limit Mode"
			depends on USE_DRV_MCPWM
			range MCPWM_CURRENTLIMIT_MODE
			default "MCPWM_CURRENTLIMIT_DISABLE"
			---help---
			IDH_HTML_MCPWM_CHANNEL_CURRENTLIMIT_SETUP
			---endhelp---
endmenu
menu "Channel Override Setup"	
	depends on USE_DRV_MCPWM

		config DRV_MCPWM_CHANNEL_OVERRIDE_PWMxH_VALUE_IDX${INSTANCE}
			string "Channel PWMH Override Output"
			depends on USE_DRV_MCPWM
			range MCPWM_OVERRIDE_PWMxH_VALUE
			default "MCPWM_OVERRIDE_PWMxH_0"
			---help---
			IDH_HTML_MCPWM_CHANNEL_OVERRIDE_SETUP
			---endhelp---
		
		config DRV_MCPWM_CHANNEL_OVERRIDE_PWMxH_ENABLE_IDX${INSTANCE}
			bool "Channel PWMH Override Enable"
			depends on USE_DRV_MCPWM
			---help---
			IDH_HTML_MCPWM_CHANNEL_OVERRIDE_SETUP
			---endhelp---
		
		config DRV_MCPWM_CHANNEL_OVERRIDE_PWMxL_VALUE_IDX${INSTANCE}
			string "Channel PWML Override Output"
			depends on USE_DRV_MCPWM
			range MCPWM_OVERRIDE_PWMxL_VALUE
			default "MCPWM_OVERRIDE_PWMxL_0"
			---help---
			IDH_HTML_MCPWM_CHANNEL_OVERRIDE_SETUP
			---endhelp---
		
		config DRV_MCPWM_CHANNEL_OVERRIDE_PWMxL_ENABLE_IDX${INSTANCE}
			bool "Channel PWML Override Enable"
			depends on USE_DRV_MCPWM
			---help---
			IDH_HTML_MCPWM_CHANNEL_OVERRIDE_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_SYNC_OVERRIDE_SELECT_IDX${INSTANCE}
			bool "Sync Override at PWM Period Boundary"
			depends on USE_DRV_MCPWM
			---help---
			IDH_HTML_MCPWM_CHANNEL_OVERRIDE_SETUP
			---endhelp---
endmenu

config DRV_MCPWM_CHANNEL_PWMH_ENABLE_IDX${INSTANCE}
	bool "Enable PWMH"
	depends on USE_DRV_MCPWM
	default y
	---help---
	IDH_HTML_MCPWM_CHANNEL_SETUP
	---endhelp---
	

	
config DRV_MCPWM_CHANNEL_PWML_ENABLE_IDX${INSTANCE}
	bool "Enable PWML"
	depends on USE_DRV_MCPWM
	default y
	---help---
	IDH_HTML_MCPWM_CHANNEL_SETUP
	---endhelp---
	

	
config DRV_MCPWM_CHANNEL_SWAP_PWMH_PWML_IDX${INSTANCE}
	bool "Swap PWMH and PWML"
	depends on USE_DRV_MCPWM
	default n
	---help---
	IDH_HTML_MCPWM_CHANNEL_SETUP
	---endhelp---
		
config DRV_MCPWM_CHANNEL_INTERRUPT_ENABLE_IDX${INSTANCE}
	bool "Channel Interrupt Enable"
	depends on USE_DRV_MCPWM
	default n
	---help---
	IDH_HTML_MCPWM_CHANNEL_INTERRUPT
	---endhelp---

		
		config DRV_MCPWM_CHANNEL_FAULT_INTERRUPT_ENABLE_IDX${INSTANCE}
			bool "Channel Fault Interrupt Enable"
			depends on USE_DRV_MCPWM
			depends on DRV_MCPWM_CHANNEL_INTERRUPT_ENABLE_IDX${INSTANCE}
			default n
			---help---
			IDH_HTML_MCPWM_CHANNEL_FAULT_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_CURRENTLIMIT_INTERRUPT_ENABLE_IDX${INSTANCE}
			bool "Channel Current Limit Interrupt Enable"
			depends on USE_DRV_MCPWM
			depends on DRV_MCPWM_CHANNEL_INTERRUPT_ENABLE_IDX${INSTANCE}
			default n
			---help---
			IDH_HTML_MCPWM_CHANNEL_CURRENTLIMIT_SETUP
			---endhelp---

			config DRV_MCPWM_CHANNEL_TRIGGER_INTERRUPT_ENABLE_IDX${INSTANCE}
			bool "Channel Trigger Interrupt Enable"
			depends on USE_DRV_MCPWM
			depends on DRV_MCPWM_CHANNEL_INTERRUPT_ENABLE_IDX${INSTANCE}
			default n
			---help---
			IDH_HTML_MCPWM_CHANNEL_TRIGGER_SETUP
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_PERIOD_MATCH_INTERRUPT_ENABLE_IDX${INSTANCE}
			bool "Enable PWMH Interrupt"
			depends on USE_DRV_MCPWM
			depends on DRV_MCPWM_CHANNEL_INTERRUPT_ENABLE_IDX${INSTANCE}
			default n
			---help---
			IDH_HTML_MCPWM_CHANNEL_SETUP
			---endhelp---
		
		config DRV_MCPWM_CHANNEL_PERIOD_RESET_INTERRUPT_ENABLE_IDX${INSTANCE}
			bool "Enable PWML Interrupt"
			depends on USE_DRV_MCPWM
			depends on DRV_MCPWM_CHANNEL_INTERRUPT_ENABLE_IDX${INSTANCE}
			default n
			---help---
			IDH_HTML_MCPWM_CHANNEL_SETUP
			---endhelp---
	
		config DRV_MCPWM_CHANNEL_INTERRUPT_PRIORITY_IDX${INSTANCE}
			string "Interrupt Priority"
			depends on USE_DRV_MCPWM
			depends on DRV_MCPWM_CHANNEL_INTERRUPT_ENABLE_IDX${INSTANCE}
			range INT_PRIORITY_LEVEL
			default "INT_PRIORITY_LEVEL1"
			
		config DRV_MCPWM_CHANNEL_INTERRUPT_SUBPRIORITY_IDX${INSTANCE}
			string "Interrupt Sub-Priority"
			depends on USE_DRV_MCPWM
			depends on DRV_MCPWM_CHANNEL_INTERRUPT_ENABLE_IDX${INSTANCE}
			range INT_SUBPRIORITY_LEVEL
			default "INT_SUBPRIORITY_LEVEL0"
			
		config DRV_MCPWM_CHANNEL_INTERRUPT_VECTOR_IDX${INSTANCE}
			string
			depends on USE_DRV_MCPWM
			depends on DRV_MCPWM_CHANNEL_INTERRUPT_ENABLE_IDX${INSTANCE}
			default "_PWM1_VECTOR" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL1"
			default "_PWM2_VECTOR" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL2"
			default "_PWM3_VECTOR" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL3"
			default "_PWM4_VECTOR" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL4"
			default "_PWM5_VECTOR" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL5"
			default "_PWM6_VECTOR" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL6"
			default "_PWM7_VECTOR" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL7"
			default "_PWM8_VECTOR" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL8"
			default "_PWM9_VECTOR" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL9"
			default "_PWM10_VECTOR" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL10"
			default "_PWM11_VECTOR" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL11"
			default "_PWM12_VECTOR" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL12"
			---help---
			IDH_HTML_INT_VECTOR
			---endhelp---
			
		config DRV_MCPWM_CHANNEL_INTERRUPT_SOURCE_IDX${INSTANCE}
			string
			depends on USE_DRV_MCPWM
			depends on DRV_MCPWM_CHANNEL_INTERRUPT_ENABLE_IDX${INSTANCE}
			default "INT_SOURCE_PWM1" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL1"
			default "INT_SOURCE_PWM2" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL2"
			default "INT_SOURCE_PWM3" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL3"
			default "INT_SOURCE_PWM4" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL4"
			default "INT_SOURCE_PWM5" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL5"
			default "INT_SOURCE_PWM6" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL6"
			default "INT_SOURCE_PWM7" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL7"
			default "INT_SOURCE_PWM8" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL8"
			default "INT_SOURCE_PWM9" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL9"
			default "INT_SOURCE_PWM10" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL10"
			default "INT_SOURCE_PWM11" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL11"
			default "INT_SOURCE_PWM12" if DRV_MCPWM_CHANNEL_ID_IDX${INSTANCE} = "MCPWM_CHANNEL12"
			---help---
			IDH_HTML_INT_SOURCE
			---endhelp---

		config DRV_MCPWM_CHANNEL_INTERRUPT_PRIORITY_NUM_IDX${INSTANCE}
			string
			depends on USE_DRV_MCPWM
			depends on DRV_MCPWM_CHANNEL_INTERRUPT_ENABLE_IDX${INSTANCE}
			default "0" if DRV_MCPWM_CHANNEL_INTERRUPT_PRIORITY_IDX${INSTANCE} = "INT_DISABLE_INTERRUPT"
			default "1" if DRV_MCPWM_CHANNEL_INTERRUPT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL1"
			default "2" if DRV_MCPWM_CHANNEL_INTERRUPT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL2"
			default "3" if DRV_MCPWM_CHANNEL_INTERRUPT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL3"
			default "4" if DRV_MCPWM_CHANNEL_INTERRUPT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL4"
			default "5" if DRV_MCPWM_CHANNEL_INTERRUPT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL5"
			default "6" if DRV_MCPWM_CHANNEL_INTERRUPT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL6"
			default "7" if DRV_MCPWM_CHANNEL_INTERRUPT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL7"			

			
	endif