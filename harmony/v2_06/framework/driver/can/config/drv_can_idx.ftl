config DRV_CAN_INSTANCES_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_DRV_CAN
<#if INSTANCE != 0>
	default n if DRV_CAN_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if DRV_CAN_INSTANCES_NUMBER = ${INSTANCE+1}
	default y
	
config DRV_CAN_INST_IDX${INSTANCE}
    depends on USE_DRV_CAN 
<#if INSTANCE != 0>
	             && DRV_CAN_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "CAN Driver Instance ${INSTANCE}"
    default y

ifblock DRV_CAN_INST_IDX${INSTANCE}

config DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE}
    string "CAN module ID"
    depends on USE_DRV_CAN
    range CAN_MODULE_ID
    default "CAN_ID_1"
    ---help---
    IDH_HTML_CAN_MODULE_ID
    ---endhelp---

config DRV_CAN_INTERRUPT_MODE_ID${INSTANCE}
    bool "Interrupt Mode for Instance ${INSTANCE}?"
    depends on USE_DRV_CAN
    select USE_SYS_INT_NEEDED
    default n
    ---help---
    Checking this option would enable interrupt mode of the instance and place necessary calls in the driver.
    ---endhelp---

menu "Interrupt Parameters Instance ${INSTANCE}"
    depends on DRV_CAN_INTERRUPT_MODE_ID${INSTANCE}
    ---help---
    IDH_HTML_CAN_MODULE_EVENT
    ---endhelp---

menu "Channel Module Events Instance ${INSTANCE}"

config DRV_CAN_TX_EVENT_IDX${INSTANCE}
        bool "TX Event"
	default n
config DRV_CAN_RX_EVENT_IDX${INSTANCE}
        bool "RX Event"
	default n
config DRV_CAN_TSOVERFLOW_EVENT_IDX${INSTANCE}
        bool "Timestamp Timer OverFlow Event"
	default n
config DRV_CAN_MODECHANGE_EVENT_IDX${INSTANCE}
        bool "Operation Mode Change Event"
	default n
config DRV_CAN_RXOVERFLOW_EVENT_IDX${INSTANCE}
        bool "RX Overflow Event"
	default n
config DRV_CAN_SYSERROR_EVENT_IDX${INSTANCE}
        bool "System Error Event"
	default n
config DRV_CAN_BUSERROR_EVENT_IDX${INSTANCE}
        bool "Bus Error Event"
	default n
config DRV_CAN_WAKEUP_EVENT_IDX${INSTANCE}
        bool "Bus Activity Wakeup Event"
	default n
config DRV_CAN_INVALIDRX_IDX${INSTANCE}
        bool "Invalid Rx Message Event"
	default n
config DRV_CAN_ALLEVENTS_IDX${INSTANCE}
        bool "All Events"
	default n

endmenu

config DRV_CAN_INTERRUPT_SOURCE_IDX${INSTANCE}
    string
    range INT_SOURCE
    default "INT_SOURCE_CAN_1" if DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_1"
    default "INT_SOURCE_CAN_2" if DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_2"
    default "INT_SOURCE_CAN_3" if PIC32MK && (DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_3" )
    default "INT_SOURCE_CAN_4" if PIC32MK && (DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_4"	)
    
config DRV_CAN_INTERRUPT_VECTOR_IDX${INSTANCE}
    string
    range INT_VECTOR
    default "INT_VECTOR_CAN1" if DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_1"
    default "INT_VECTOR_CAN2" if DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_2"
    default "INT_VECTOR_CAN3" if PIC32MK && (DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_3" )
    default "INT_VECTOR_CAN4" if PIC32MK && (DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_4"	)

config DRV_CAN_INT_PRIORITY_IDX${INSTANCE}
    string "Interrupt Priority"
    range INT_PRIORITY_LEVEL
    default "INT_PRIORITY_LEVEL1"
    ---help---
    IDH_HTML_INT_PRIORITY_LEVEL
    ---endhelp---

config DRV_CAN_INT_SUB_PRIORITY_IDX${INSTANCE}
    string "Interrupt Sub-priority"
    range INT_SUBPRIORITY_LEVEL
    default "INT_SUBPRIORITY_LEVEL0"
    ---help---
    IDH_HTML_INT_SUBPRIORITY_LEVEL
    ---endhelp---

config DRV_CAN_INT_PRIO_NUM_IDX${INSTANCE}
    string
    default "0" if DRV_CAN_INT_PRIORITY_IDX${INSTANCE} = "INT_DISABLE_INTERRUPT"
    default "1" if DRV_CAN_INT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL1"
    default "2" if DRV_CAN_INT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL2"
    default "3" if DRV_CAN_INT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL3"
    default "4" if DRV_CAN_INT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL4"
    default "5" if DRV_CAN_INT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL5"
    default "6" if DRV_CAN_INT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL6"
    default "7" if DRV_CAN_INT_PRIORITY_IDX${INSTANCE} = "INT_PRIORITY_LEVEL7"
    
config DRV_CAN_ISR_VECTOR_IDX${INSTANCE}
    string
    default "_CAN1_VECTOR" if (PIC32MZ || PIC32WK || PIC32MK) && DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_1"
    default "_CAN_1_VECTOR" if PIC32MX && DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_1"
    default "_CAN2_VECTOR" if (PIC32MZ || PIC32MK) && DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_2"
    default "_CAN_2_VECTOR" if PIC32MX && DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_2"
    default "_CAN3_VECTOR" if PIC32MK && DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_3"
    default "_CAN4_VECTOR" if PIC32MK && DRV_CAN_PERIPHERAL_ID_IDX${INSTANCE} = "CAN_ID_4"

endmenu

config DRV_CAN_BAUD_RATE_IDX${INSTANCE}
	int "Desired Baud Rate (kbps)"
	range 1 1000
	default 1000
	---help---
	Baud rate valid range is 1 - 1000 kbps. Idealy this value should be a multiple of SYSCLK or you will never achieve 0% error.
	---endhelp---

execute DRV_CAN_CALCULATE_EXEC_IDX${INSTANCE}
    prompt "Click to Calculate CAN Configuration Values"
    default "can_calculate ${INSTANCE}"
	---help---
	Automatically calculates the values below.  Generated values represent the lowest transmission error rate.
	---endhelp---
	
config DRV_CAN_BAUD_RATE_USE_TOLERANCE_IDX${INSTANCE}
	bool "Apply baud rate tolerance in automatic calculation?"
	default n
	---help---
	If indicated, the automatic calculator will attempt to find the fastest baud rate with the lowest error in using the desired baud rate plus or minus the given tolerance.
	---endhelp---
	
config DRV_CAN_BAUD_RATE_TOLERANCE_IDX${INSTANCE}
	depends on DRV_CAN_BAUD_RATE_USE_TOLERANCE_IDX${INSTANCE}
	int "Baud Rate Tolerance"
	default 0
	---help---
	The automatic calculator will attempt to find the fastest baud rate with the lowest error in using the desired baud rate plus or minus this tolerance value.
	---endhelp---
	
ifblock DRV_CAN_TS_SYNC_IDX${INSTANCE} = 0
comment "*** Error.  Values must be calculated by pressing Execute button above. ***"
endif

config DRV_CAN_CLOCK_RATE_IDX${INSTANCE}
	string "Clock Rate"
	default SYS_CLK_FREQ if PIC32MX
	default SYS_CLK_PBCLK4_FREQ if PIC32MZ || PIC32MK
	default SYS_CLK_FREQ if PIC32WK
	persistent

menu "Generated Baud Rate Parameters Instance ${INSTANCE}"
 depends on USE_DRV_CAN
	
config DRV_CAN_TS_SYNC_IDX${INSTANCE}
	int "Sync"
	range 1 4
	default 0
	persistent

config DRV_CAN_TS_PROP_IDX${INSTANCE}
	int "Propagation"
	range 1 8
	default 0
	persistent

config DRV_CAN_TS_PHASE1_IDX${INSTANCE}
	int "Phase 1"
	range 1 8
	default 0
	persistent
	
config DRV_CAN_TS_PHASE2_IDX${INSTANCE}
	int "Phase 2"
	range 1 8
	default 0
	persistent
	
config DRV_CAN_BAUD_RATE_PRESCALER_IDX${INSTANCE}
	int "Baud Rate Prescaler"
	default 0
	persistent

config DRV_CAN_TIME_QUANTA_IDX${INSTANCE}
	string "Time Quanta (sec)"
	persistent
	
config DRV_CAN_BIT_PERIOD_IDX${INSTANCE}
	string "Bit Period (sec)"
	persistent

config DRV_CAN_CALCULATED_BITRATE_IDX${INSTANCE}
	string "Calculated Bit Rate (kbps)"
	persistent
	
config DRV_CAN_ERROR_VALUE_IDX${INSTANCE}
	string "Error %"
	persistent
		
config DRV_CAN_COMMENT_VAL_IDX${INSTANCE}
	string
	range DRV_CAN_ERROR_JUDGEMENT
	default "NONE"
	
ifblock DRV_CAN_COMMENT_VAL_IDX${INSTANCE} = "BAD"
comment "*** CAN error rate is too large!  Must be < 1.0% ***"
endif

ifblock DRV_CAN_COMMENT_VAL_IDX${INSTANCE} = "OK"
comment "*** CAN error rate is satisfactory.  Should be < 0.3% ***"
endif

ifblock DRV_CAN_COMMENT_VAL_IDX${INSTANCE} = "GOOD"
comment "*** CAN error rate is good! ***"
endif

ifblock DRV_CAN_COMMENT_VAL_IDX${INSTANCE} = "INVALID"
comment "*** No valid bitrate can be found.  Try adjusting desired bitrate or use a larger tolerance. ***"
endif

endmenu


config DRV_CAN_OPERATION_MODE_IDX${INSTANCE}
    string "Operation Mode"
    range CAN_OPERATION_MODES
    default "CAN_NORMAL_MODE"
    ---help---
    IDH_HTML_CAN_OPERATION_MODES
    ---endhelp---

config DRV_CAN_FILTERS_NUMBER${INSTANCE}
    int "Number of CAN Driver Filters"
	range 1 16 if PIC32WK
    range 1 32
    default 1

config DRV_CAN_MASKS_NUMBER${INSTANCE}
    int "Number of CAN Driver Masks"
	range 1 3 if PIC32WK
    range 1 4
    default 1

config DRV_CAN_CHANNELS_NUMBER${INSTANCE}
    int "Number of Channels"
    depends on DRV_CAN_INST_IDX${INSTANCE}
    range 1 16 if PIC32WK
	range 1 32
    default 2	

<#if INSTANCE == 0>
source 
"$HARMONY_VERSION_PATH/framework/driver/can/config/drv_can_filterx.ftl" 32 instances

source "$HARMONY_VERSION_PATH/framework/driver/can/config/drv_can_maskx.ftl" 4 instances

source 
"$HARMONY_VERSION_PATH/framework/driver/can/config/drv_can_channelx.ftl" 32 instances
</#if>
<#if INSTANCE == 1>
source 
"$HARMONY_VERSION_PATH/framework/driver/can/config/drv_can_filter1x.ftl" 32 instances

source "$HARMONY_VERSION_PATH/framework/driver/can/config/drv_can_mask1x.ftl" 4 instances

source 
"$HARMONY_VERSION_PATH/framework/driver/can/config/drv_can_channel1x.ftl" 32 instances
</#if>
<#if INSTANCE == 2>
source 
"$HARMONY_VERSION_PATH/framework/driver/can/config/drv_can_filter2x.ftl" 16 instances

source "$HARMONY_VERSION_PATH/framework/driver/can/config/drv_can_mask2x.ftl" 4 instances

source 
"$HARMONY_VERSION_PATH/framework/driver/can/config/drv_can_channel2x.ftl" 16 instances
</#if>
<#if INSTANCE == 3>
source 
"$HARMONY_VERSION_PATH/framework/driver/can/config/drv_can_filter3x.ftl" 16 instances

source "$HARMONY_VERSION_PATH/framework/driver/can/config/drv_can_mask3x.ftl" 4 instances

source 
"$HARMONY_VERSION_PATH/framework/driver/can/config/drv_can_channel3x.ftl" 16 instances
</#if>
endif