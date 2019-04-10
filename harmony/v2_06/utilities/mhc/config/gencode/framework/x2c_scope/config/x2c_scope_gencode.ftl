menu "X2C SCOPE"

config X2C_SCOPE${INSTANCE}
    bool "X2C Scope"
	set USE_DRV_USART optionally to y if X2C_SCOPE${INSTANCE}
	set DRV_USART_DRIVER_MODE optionally to "STATIC" if X2C_SCOPE${INSTANCE}
	set DRV_USART_INTERRUPT_MODE optionally to n if X2C_SCOPE${INSTANCE}
	set DRV_USART_BYTE_MODEL_SUPPORT optionally to y if X2C_SCOPE${INSTANCE}
	set DRV_USART_BYTE_MODEL_BLOCKING optionally to y if X2C_SCOPE${INSTANCE}
	set DRV_USART_BYTE_MODEL_CALLBACK optionally to n if X2C_SCOPE${INSTANCE}
	set DRV_USART_INSTANCES_NUMBER optionally to 1 if X2C_SCOPE${INSTANCE}
	set DRV_USART_CLIENTS_NUMBER optionally to 1 if X2C_SCOPE${INSTANCE}
	set DRV_USART_INST_IDX0 optionally to y if X2C_SCOPE${INSTANCE}
	set DRV_USART_PERIPHERAL_ID_IDX0 optionally to "USART_ID_1" if X2C_SCOPE${INSTANCE}
    set DRV_USART_BAUD_RATE_IDX0 optionally to 38400 if X2C_SCOPE${INSTANCE}
	set DRV_USART_XMIT_INT_PRIORITY_IDX0 optionally to "INT_DISABLE_INTERRUPT" if X2C_SCOPE${INSTANCE}
	set DRV_USART_XMIT_INT_SUB_PRIORITY_IDX0 optionally to "INT_SUBPRIORITY_LEVEL0" if X2C_SCOPE${INSTANCE}
	set DRV_USART_RCV_INT_PRIORITY_IDX0 optionally to "INT_PRIORITY_LEVEL1" if X2C_SCOPE${INSTANCE}
	set DRV_USART_RCV_INT_SUB_PRIORITY_IDX0 optionally to "INT_SUBPRIORITY_LEVEL0" if X2C_SCOPE${INSTANCE}
	set DRV_USART_ERR_INT_PRIORITY_IDX0 optionally to "INT_DISABLE_INTERRUPT" if X2C_SCOPE${INSTANCE}
	set DRV_USART_ERR_INT_SUB_PRIORITY_IDX0 optionally to "INT_SUBPRIORITY_LEVEL0" if X2C_SCOPE${INSTANCE}
	set DRV_USART_OPER_MODE_IDX0 optionally to "DRV_USART_OPERATION_MODE_NORMAL" if X2C_SCOPE${INSTANCE}
	set DRV_USART_INIT_FLAG_WAKE_ON_START_IDX0 optionally to n if X2C_SCOPE${INSTANCE}
	set DRV_USART_INIT_FLAG_AUTO_BAUD_IDX0 optionally to n if X2C_SCOPE${INSTANCE}
	set DRV_USART_INIT_FLAG_STOP_IN_IDLE_IDX0 optionally to n if X2C_SCOPE${INSTANCE}
	set DRV_USART_LINE_CNTRL_IDX0 optionally to "DRV_USART_LINE_CONTROL_8NONE1" if X2C_SCOPE${INSTANCE}
	set DRV_USART_HANDSHAKE_MODE_IDX0 optionally to "DRV_USART_HANDSHAKE_NONE" if X2C_SCOPE${INSTANCE}
	set DRV_USART_STATIC_RX_MODES_IDX0 optionally to "USART_HANDSHAKE_MODE_FLOW_CONTROL" if X2C_SCOPE${INSTANCE}
	set DRV_USART_STATIC_OP_MODES_IDX0 optionally to "USART_ENABLE_TX_RX_USED" if X2C_SCOPE${INSTANCE}
	set DRV_USART_STATIC_LINECONTROL_MODES_IDX0 optionally to "USART_8N1" if X2C_SCOPE${INSTANCE}
	set DRV_USART_STATIC_TX_ENABLE_IDX0 optionally to y if X2C_SCOPE${INSTANCE}
	set DRV_USART_STATIC_RX_ENABLE_IDX0 optionally to y if X2C_SCOPE${INSTANCE}
	set DRV_USART_STATIC_TX_INTR_MODES_IDX0 optionally to "USART_TRANSMIT_FIFO_NOT_FULL" if X2C_SCOPE${INSTANCE}
	set DRV_USART_STATIC_RX_INTR_MODES_IDX0 optionally to "USART_RECEIVE_FIFO_ONE_CHAR" if X2C_SCOPE${INSTANCE}
    
	
	
   
	default n
	---help---
	<!DOCTYPE HTML>
	
	---endhelp---
    
config X2C_SCOPE_USART_INSTANCE_INDEX${INSTANCE}
    int "USART Driver Instance Index"
    range 0 0 if DRV_USART_INSTANCES_NUMBER = 1
    range 0 1 if DRV_USART_INSTANCES_NUMBER = 2
    range 0 2 if DRV_USART_INSTANCES_NUMBER = 3
    range 0 3 if DRV_USART_INSTANCES_NUMBER = 4
    range 0 4 if DRV_USART_INSTANCES_NUMBER = 5
    range 0 5 if DRV_USART_INSTANCES_NUMBER = 6
    default 0
    

ifblock X2C_SCOPE${INSTANCE}

add "^#include \"/utilities/mhc/config/gencode/framework/x2c_scope/templates/x2c_scope_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/x2c_scope/templates/x2c_scope_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/x2c_scope/templates/x2c_scope_macros_system_config.h.ftl\">" to list APP_FREEMARKER_MACROS

    ifblock PIC32MK
    library X2C_SCOPE_PIC32MK_${INSTANCE} "$HARMONY_VERSION_PATH/bin/framework/x2c_scope/X2CScopeLib_PIC32MK.X.a"
    endif 
    
    ifblock PIC32MX
    library X2C_SCOPE_PIC32MX_${INSTANCE} "$HARMONY_VERSION_PATH/bin/framework/x2c_scope/X2CScopeLib_PIC32MX.X.a"
    endif 
    
    ifblock DS60001361
    library X2C_SCOPE_PIC32MZ_DA_${INSTANCE} "$HARMONY_VERSION_PATH/bin/framework/x2c_scope/X2CScopeLib_PIC32MZ_DA.X.a"
    endif 
    
    ifblock DS60001320
    library X2C_SCOPE_PIC32MZ_EF_${INSTANCE} "$HARMONY_VERSION_PATH/bin/framework/x2c_scope/X2CScopeLib_PIC32MZ_EF.X.a"
    endif 
    
endif

ifblock X2C_SCOPE${INSTANCE}

add "^@macro_x2c_scope_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_x2c_scope_system_config_h_app_constants/>" to list APP${INSTANCE}_H_CONSTANTS
add "^@macro_x2c_scope_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_x2c_scope_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_x2c_scope_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_x2c_scope_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS
add "^@macro_x2c_scope_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES

add "^@macro_x2c_scope_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_x2c_scope_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_x2c_scope_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_x2c_scope_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_x2c_scope_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_x2c_scope_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_x2c_scope_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_x2c_scope_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_x2c_scope_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_x2c_scope_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
endif


endmenu
