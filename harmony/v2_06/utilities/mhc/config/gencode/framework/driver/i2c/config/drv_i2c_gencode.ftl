menu "I2C"
    depends on HAVE_I2C
    
config GENERATE_CODE_DRV_I2C${INSTANCE}
    bool "I2C?"
    default n
    set USE_DRV_I2C optionally to y if GENERATE_CODE_DRV_I2C${INSTANCE}
    ---help---
    IDH_HTML_I2C_Driver_Library
    ---endhelp---

ifblock GENERATE_CODE_DRV_I2C${INSTANCE}

comment "**** Note: Please configure I2C Driver settings under Framework Configuration menu ****"

config APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE}
    bool "Master, String Trasmit"
    default n
	set DRV_I2C_DRIVER_MODE optionally to "DYNAMIC" if APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE} 
	set DRV_I2C_INTERRUPT_MODE optionally to y if APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE}
    set DRV_I2C_OPERATION_MODE_IDX0 optionally to "DRV_I2C_MODE_MASTER" if APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 0
    set DRV_I2C_OPERATION_MODE_IDX1 optionally to "DRV_I2C_MODE_MASTER" if APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 1
    set DRV_I2C_OPERATION_MODE_IDX2 optionally to "DRV_I2C_MODE_MASTER" if APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 2
    set DRV_I2C_OPERATION_MODE_IDX3 optionally to "DRV_I2C_MODE_MASTER" if APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 3
    set DRV_I2C_OPERATION_MODE_IDX4 optionally to "DRV_I2C_MODE_MASTER" if APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 4
    #set DRV_I2C_OPERATION_MODE_IDX5 optionally to "DRV_I2C_MODE_MASTER" if APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 5
    #set DRV_I2C_OPERATION_MODE_IDX6 optionally to "DRV_I2C_MODE_MASTER" if APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 6
    #set DRV_I2C_OPERATION_MODE_IDX7 optionally to "DRV_I2C_MODE_MASTER" if APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 7
    #set DRV_I2C_OPERATION_MODE_IDX8 optionally to "DRV_I2C_MODE_MASTER" if APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 8
    #set DRV_I2C_OPERATION_MODE_IDX9 optionally to "DRV_I2C_MODE_MASTER" if APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 9
	persistent if APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE}
    ---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony I2C Master TX Application Template</h2>
	<p>	This template generates a simple code example which transmits
	a string of bytes via I2C. It will automatically configure
	the I2C driver with the following settings:</p>
	<br>- Dynamic Driver Mode
	<br>- Queue Transfer Model
	<br>- Non-Blocking Mode
	<br>- Interrupt Mode
	<p>All other I2C driver configuration options are set to their 
	default values. The driver configuration may be modified by 
	the user using MHC, under </p>
    <p>Harmony Framework Configuration -> Drivers -> I2C</p>
    </html>
    ---endhelp---
    
config APP_DRV_I2C_TX_STRING${INSTANCE}
	string "Transmit String"
	default "Hello World"
	depends on APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE}

config APP_DRV_I2C_SLAVE_ADDRESS${INSTANCE}
	hex "Slave Address to use?"
	default 0x40
	depends on APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE}
    
config APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE}
    bool "Slave, Character Receive"
	set DRV_I2C_DRIVER_MODE optionally to "DYNAMIC" if APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE} 
	set DRV_I2C_INTERRUPT_MODE optionally to y if APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE}
    set DRV_I2C_OPERATION_MODE_IDX0 optionally to "DRV_I2C_MODE_SLAVE" if APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 0
    set DRV_I2C_OPERATION_MODE_IDX1 optionally to "DRV_I2C_MODE_SLAVE" if APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 1
    set DRV_I2C_OPERATION_MODE_IDX2 optionally to "DRV_I2C_MODE_SLAVE" if APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 2
    set DRV_I2C_OPERATION_MODE_IDX3 optionally to "DRV_I2C_MODE_SLAVE" if APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 3
    set DRV_I2C_OPERATION_MODE_IDX4 optionally to "DRV_I2C_MODE_SLAVE" if APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 4
    #set DRV_I2C_OPERATION_MODE_IDX5 optionally to "DRV_I2C_MODE_SLAVE" if APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 5
    #set DRV_I2C_OPERATION_MODE_IDX6 optionally to "DRV_I2C_MODE_SLAVE" if APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 6
    #set DRV_I2C_OPERATION_MODE_IDX7 optionally to "DRV_I2C_MODE_SLAVE" if APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 7
    #set DRV_I2C_OPERATION_MODE_IDX8 optionally to "DRV_I2C_MODE_SLAVE" if APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 8
    #set DRV_I2C_OPERATION_MODE_IDX9 optionally to "DRV_I2C_MODE_SLAVE" if APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE} = y  && APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 9
    default n
	persistent if APP_DRV_I2C_MASTER_TRANSMIT${INSTANCE}
    ---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony I2C Slave RX Application Template</h2>
	<p>	This template generates a simple code example which accepts
	a string of bytes via I2C. It will automatically configure
	the I2C driver with the following settings:</p>
	<br>- Static Driver Mode
	<br>- Byte Transfer Model
	<br>- Non-Blocking Mode
	<br>- Polling Mode (Interrupts Disabled)
	<p>All other I2C driver configuration options are set to their 
	default values. The driver configuration may be modified by 
	the user using MHC, under </p>
    <p>Harmony Framework Configuration -> Drivers -> I2C</p>
    <p>The Peripheral Pin Selection for the desired I2C has to be 
    set in the Pin Configurator.</p>
    </html>
    ---endhelp---
    
ifblock APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE}
comment "**** Be aware that the slave device has no control over the number of bytes transmitted. ****"
comment "**** This implementation simply wraps extra bytes to the begining of the buffer. ****"
endif
    
config APP_DRV_I2C_RX_SIZE${INSTANCE}
	int "Receive buffer size"
	default 16
	depends on APP_DRV_I2C_SLAVE_RECEIVE${INSTANCE}

menu "Options"

#config APP_DRV_I2C_REPEAT${INSTANCE}
#	bool "Repeat Transaction?"
#	default n

#config APP_DRV_I2C_QUEUE_DEPTH${INSTANCE}
#	int "I2C Queue Depth to exercise?"
#	depends on APP_DRV_I2C_REPEAT${INSTANCE}
#	default 2

config APP_DRV_I2C_INSTANCE_INDEX${INSTANCE}
    int "I2C Driver Instance Index"
    range 0 0 if DRV_I2C_INSTANCES_NUMBER = 1
    range 0 1 if DRV_I2C_INSTANCES_NUMBER = 2
    range 0 2 if DRV_I2C_INSTANCES_NUMBER = 3
    range 0 3 if DRV_I2C_INSTANCES_NUMBER = 4
    range 0 4 if DRV_I2C_INSTANCES_NUMBER = 5
    range 0 5 if DRV_I2C_INSTANCES_NUMBER = 6
    range 0 6 if DRV_I2C_INSTANCES_NUMBER = 7
    range 0 7 if DRV_I2C_INSTANCES_NUMBER = 8
    range 0 8 if DRV_I2C_INSTANCES_NUMBER = 9
    default 0

config APP_DRV_I2C_HANDLE${INSTANCE}
    string "Name of the I2C driver handle?"
    default "handleI2C0" if APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 0
    default "handleI2C1" if APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 1
    default "handleI2C2" if APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 2
    default "handleI2C3" if APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 3
    default "handleI2C4" if APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 4
    default "handleI2C5" if APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 5
    default "handleI2C6" if APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 6
    default "handleI2C7" if APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 7
    default "handleI2C8" if APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 8
    default "handleI2C9" if APP_DRV_I2C_INSTANCE_INDEX${INSTANCE} = 9
    default "handleI2C"

endmenu




add "^#include \"/utilities/mhc/config/gencode/framework/driver/i2c/config/drv_i2c_gencode_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_drv_i2c_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_drv_i2c_app_h_constants/>" to list APP${INSTANCE}_H_CONSTANTS
add "^@macro_drv_i2c_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_drv_i2c_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_drv_i2c_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_drv_i2c_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS

add "^#include \"/utilities/mhc/config/gencode/framework/driver/i2c/config/drv_i2c_gencode_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_drv_i2c_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_drv_i2c_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_drv_i2c_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_drv_i2c_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_drv_i2c_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_drv_i2c_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_drv_i2c_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_drv_i2c_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_drv_i2c_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
 
add "^#include \"/utilities/mhc/config/gencode/framework/driver/i2c/config/drv_i2c_gencode_macros_system_config.h.ftl\">" to list APP_FREEMARKER_MACROS 
add "^@macro_drv_i2c_system_config_h_app_constants/>" to list APP${INSTANCE}_H_CONSTANTS


endif

endmenu
