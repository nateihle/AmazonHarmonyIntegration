menu "RTDM"

config RTDM${INSTANCE}
    bool "Real Time Data Monitoring Interface - RTDM"
	set USE_DRV_USART optionally to y if RTDM${INSTANCE}
	set DRV_USART_DRIVER_MODE optionally to "STATIC" if RTDM${INSTANCE}
	set DRV_USART_INTERRUPT_MODE optionally to y if RTDM${INSTANCE}
	set DRV_USART_BYTE_MODEL_SUPPORT optionally to y if RTDM${INSTANCE}
	set DRV_USART_BYTE_MODEL_BLOCKING optionally to y if RTDM${INSTANCE}
	set DRV_USART_BYTE_MODEL_CALLBACK optionally to n if RTDM${INSTANCE}
	set DRV_USART_INSTANCES_NUMBER optionally to 1 if RTDM${INSTANCE}
	set DRV_USART_CLIENTS_NUMBER optionally to 1 if RTDM${INSTANCE}
	set DRV_USART_INST_IDX0 optionally to y if RTDM${INSTANCE}
	set DRV_USART_PERIPHERAL_ID_IDX0 optionally to "USART_ID_2" if RTDM${INSTANCE}
	set DRV_USART_BAUD_RATE_IDX0 optionally to 57600 if RTDM${INSTANCE}
	set DRV_USART_XMIT_INT_PRIORITY_IDX0 optionally to "INT_DISABLE_INTERRUPT" if RTDM${INSTANCE}
	set DRV_USART_XMIT_INT_SUB_PRIORITY_IDX0 optionally to "INT_SUBPRIORITY_LEVEL0" if RTDM${INSTANCE}
	set DRV_USART_RCV_INT_PRIORITY_IDX0 optionally to "INT_PRIORITY_LEVEL1" if RTDM${INSTANCE}
	set DRV_USART_RCV_INT_SUB_PRIORITY_IDX0 optionally to "INT_SUBPRIORITY_LEVEL0" if RTDM${INSTANCE}
	set DRV_USART_ERR_INT_PRIORITY_IDX0 optionally to "INT_DISABLE_INTERRUPT" if RTDM${INSTANCE}
	set DRV_USART_ERR_INT_SUB_PRIORITY_IDX0 optionally to "INT_SUBPRIORITY_LEVEL0" if RTDM${INSTANCE}
	set DRV_USART_OPER_MODE_IDX0 optionally to "DRV_USART_OPERATION_MODE_NORMAL" if RTDM${INSTANCE}
	set DRV_USART_INIT_FLAG_WAKE_ON_START_IDX0 optionally to n if RTDM${INSTANCE}
	set DRV_USART_INIT_FLAG_AUTO_BAUD_IDX0 optionally to n if RTDM${INSTANCE}
	set DRV_USART_INIT_FLAG_STOP_IN_IDLE_IDX0 optionally to n if RTDM${INSTANCE}
	set DRV_USART_LINE_CNTRL_IDX0 optionally to "DRV_USART_LINE_CONTROL_8NONE1" if RTDM${INSTANCE}
	set DRV_USART_HANDSHAKE_MODE_IDX0 optionally to "DRV_USART_HANDSHAKE_NONE" if RTDM${INSTANCE}
	set DRV_USART_STATIC_RX_MODES_IDX0 optionally to "USART_HANDSHAKE_MODE_FLOW_CONTROL" if RTDM${INSTANCE}
	set DRV_USART_STATIC_OP_MODES_IDX0 optionally to "USART_ENABLE_TX_RX_USED" if RTDM${INSTANCE}
	set DRV_USART_STATIC_LINECONTROL_MODES_IDX0 optionally to "USART_8N1" if RTDM${INSTANCE}
	set DRV_USART_STATIC_TX_ENABLE_IDX0 optionally to y if RTDM${INSTANCE}
	set DRV_USART_STATIC_RX_ENABLE_IDX0 optionally to y if RTDM${INSTANCE}
	set DRV_USART_STATIC_TX_INTR_MODES_IDX0 optionally to "USART_TRANSMIT_FIFO_NOT_FULL" if RTDM${INSTANCE}
	set DRV_USART_STATIC_RX_INTR_MODES_IDX0 optionally to "USART_RECEIVE_FIFO_ONE_CHAR" if RTDM${INSTANCE}
	
	
	set BSP_PIN_76_FUNCTION_NAME optionally to ""
	set BSP_PIN_76_FUNCTION_TYPE optionally to "U2TX"
	set BSP_PIN_76_PORT_PIN optionally to "9"
	set BSP_PIN_76_PORT_CHANNEL optionally to "B"
	set BSP_PIN_76_MODE optionally to "DIGITAL"
	set BSP_PIN_76_DIR optionally to ""
	set BSP_PIN_76_LAT optionally to ""
	set BSP_PIN_76_OD optionally to ""
	set BSP_PIN_76_CN optionally to ""
	set BSP_PIN_76_PU optionally to ""
	set BSP_PIN_76_PD optionally to ""

	set BSP_PIN_81_FUNCTION_NAME optionally to ""
	set BSP_PIN_81_FUNCTION_TYPE optionally to "U2RX"
	set BSP_PIN_81_PORT_PIN optionally to "8"
	set BSP_PIN_81_PORT_CHANNEL optionally to "C"
	set BSP_PIN_81_MODE optionally to "DIGITAL"
	set BSP_PIN_81_DIR optionally to ""
	set BSP_PIN_81_LAT optionally to ""
	set BSP_PIN_81_OD optionally to ""
	set BSP_PIN_81_CN optionally to ""
	set BSP_PIN_81_PU optionally to ""
	set BSP_PIN_81_PD optionally to ""
   
	default n
	---help---
	<!DOCTYPE HTML>
	
	---endhelp---

	
ifblock RTDM${INSTANCE}

add "^#include \"/utilities/mhc/config/gencode/framework/rtdm/templates/rtdm_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/rtdm/templates/rtdm_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/rtdm/templates/rtdm_macros_system_config.h.ftl\">" to list APP_FREEMARKER_MACROS

endif

ifblock RTDM${INSTANCE}

add "^@macro_rtdm_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_rtdm_system_config_h_app_constants/>" to list APP${INSTANCE}_H_CONSTANTS
add "^@macro_rtdm_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_rtdm_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_rtdm_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_rtdm_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS
add "^@macro_rtdm_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES

add "^@macro_rtdm_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_rtdm_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_rtdm_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_rtdm_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_rtdm_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_rtdm_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_rtdm_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_rtdm_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_rtdm_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_rtdm_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
endif


endmenu
