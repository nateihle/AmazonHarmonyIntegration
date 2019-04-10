menu "Usart"
    depends on HAVE_USART
    
config GENERATE_CODE_DRV_USART${INSTANCE}
    bool "USART?"
    default n
	select BSP_USART_BRIDGE_NEEDED if GENERATE_CODE_DRV_USART${INSTANCE}
	select USE_BSP_NEEDED if GENERATE_CODE_DRV_USART${INSTANCE}
    select USE_DRV_USART_NEEDED if GENERATE_CODE_DRV_USART${INSTANCE}
    ---help---
    IDH_HTML_USART_Driver_Library
    ---endhelp---

ifblock GENERATE_CODE_DRV_USART${INSTANCE}

config APP_DRV_USART_BM_TX${INSTANCE}
    bool "Transmit String (Byte Mode)"
    default n
	set DRV_USART_DRIVER_MODE to "STATIC" if APP_DRV_USART_BM_TX${INSTANCE} 
	set DRV_USART_INTERRUPT_MODE to n if APP_DRV_USART_BM_TX${INSTANCE}
	set DRV_USART_BYTE_MODEL_SUPPORT to y if APP_DRV_USART_BM_TX${INSTANCE}
	set DRV_USART_BYTE_MODEL_BLOCKING to n if APP_DRV_USART_BM_TX${INSTANCE}
	persistent if APP_DRV_USART_BQ_TX${INSTANCE} || APP_DRV_USART_BQ_RX${INSTANCE} || APP_DRV_USART_RW_TX${INSTANCE} || APP_DRV_USART_RW_RX${INSTANCE}
	---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony USART TX (Byte Mode) Application Template</h2>
	<p>	This template generates a simple code example which transmits
	a string of bytes via UART. It will automatically configure
	the UART driver with the following settings:</p>
	<br>- Static Driver Mode
	<br>- Byte Transfer Model
	<br>- Non-Blocking Mode
	<br>- Polling Mode (Interrupts Disabled)
	<p>All other USART driver configuration options are set to their 
	default values. The driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> 
	Drivers -> USART</p></html>
	---endhelp---

config APP_DRV_USART_BM_TX_STRING${INSTANCE}
	string "Transmit String"
	default "Hello World"
	depends on APP_DRV_USART_BM_TX${INSTANCE}

config APP_DRV_USART_BM_RX${INSTANCE}
    bool "Receive String (Byte Mode)"
    default n
	persistent if APP_DRV_USART_BQ_RX${INSTANCE} || APP_DRV_USART_BQ_TX${INSTANCE} || APP_DRV_USART_RW_TX${INSTANCE} || APP_DRV_USART_RW_RX${INSTANCE}
	set DRV_USART_DRIVER_MODE to "STATIC" if APP_DRV_USART_BM_RX${INSTANCE}
	set DRV_USART_INTERRUPT_MODE to n if APP_DRV_USART_BM_RX${INSTANCE}
	set DRV_USART_BYTE_MODEL_SUPPORT to y if APP_DRV_USART_BM_RX${INSTANCE}
	set DRV_USART_BYTE_MODEL_BLOCKING to n if APP_DRV_USART_BM_RX${INSTANCE}
		---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony USART RX (Byte Mode) Application Template</h2>
	<p>	This template generates a simple code example which receives
	a string of bytes via UART. It will automatically configure
	the UART driver with the following settings:</p>
	<br>- Static Driver Mode
	<br>- Byte Transfer Model
	<br>- Non-Blocking Mode
	<br>- Polling Mode (Interrupts Disabled)
	<p>All other USART driver configuration options are set to their 
	default values. The driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> 
	Drivers -> USART</p></html>
	---endhelp---

config APP_DRV_USART_BM_RX_SIZE${INSTANCE}
	int "Number of Characters to Receive"
	default 10
	depends on APP_DRV_USART_BM_RX${INSTANCE}

config APP_DRV_USART_BQ_TX${INSTANCE}
	bool "Transmit String (Buffer Queue Mode)"
	default n
	persistent if APP_DRV_USART_BM_TX${INSTANCE} || APP_DRV_USART_BM_RX${INSTANCE} || APP_DRV_USART_RW_TX${INSTANCE} || APP_DRV_USART_RW_RX${INSTANCE}
	set DRV_USART_DRIVER_MODE to "DYNAMIC" if APP_DRV_USART_BQ_TX${INSTANCE}
	set DRV_USART_INTERRUPT_MODE to n  if APP_DRV_USART_BQ_TX${INSTANCE}
	set DRV_USART_BUFFER_QUEUE_SUPPORT to y if APP_DRV_USART_BQ_TX${INSTANCE}
	set DRV_USART_BYTE_MODEL_SUPPORT to n  if APP_DRV_USART_BQ_TX${INSTANCE}
	---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony USART TX (Buffer Queue Mode) Application Template</h2>
	<p>	This template generates a simple code example which transmits
	a string of bytes via UART. It will automatically configure
	the UART driver with the following settings:</p>
	<br>- Dynamic Driver Mode
	<br>- Buffer Queue Transfer Model
	<br>- Polling Mode (Interrupts Disabled)
	<p>All other USART driver configuration options are set to their 
	default values. The driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> 
	Drivers -> USART</p></html>
	---endhelp---

config APP_DRV_USART_BQ_TX_STRING${INSTANCE}
	string "Transmit String"
	default "Hello World"
	depends on APP_DRV_USART_BQ_TX${INSTANCE}

config APP_DRV_USART_BQ_RX${INSTANCE}
	bool "Receive String (Buffer Queue Mode)"
	default n
	persistent if APP_DRV_USART_BM_RX${INSTANCE} || APP_DRV_USART_BM_TX${INSTANCE} || APP_DRV_USART_RW_TX${INSTANCE} || APP_DRV_USART_RW_RX${INSTANCE}
	set DRV_USART_DRIVER_MODE to "DYNAMIC" if APP_DRV_USART_BQ_RX${INSTANCE}
	set DRV_USART_INTERRUPT_MODE to n  if APP_DRV_USART_BQ_RX${INSTANCE}
	set DRV_USART_BUFFER_QUEUE_SUPPORT to y if APP_DRV_USART_BQ_RX${INSTANCE}
	set DRV_USART_BYTE_MODEL_SUPPORT to n  if APP_DRV_USART_BQ_RX${INSTANCE}
	---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony USART RX (Buffer Queue Mode) Application Template</h2>
	<p>	This template generates a simple code example which receives
	a string of bytes via UART. It will automatically configure
	the UART driver with the following settings:</p>
	<br>- Dynamic Driver Mode
	<br>- Buffer Queue Transfer Model
	<br>- Polling Mode (Interrupts Disabled)
	<p>All other USART driver configuration options are set to their 
	default values. The driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> 
	Drivers -> USART</p></html>
	---endhelp---

config APP_DRV_USART_BQ_RX_SIZE${INSTANCE}
	int "Number of Characters to Receive"
	default 10
	depends on APP_DRV_USART_BQ_RX${INSTANCE}

config APP_DRV_USART_RW_TX${INSTANCE}
	bool "Transmit String (Read/Write Mode)"
	default n
	persistent if APP_DRV_USART_BM_TX${INSTANCE} || APP_DRV_USART_BM_RX${INSTANCE} || APP_DRV_USART_BQ_TX${INSTANCE} || APP_DRV_USART_BQ_RX${INSTANCE}
	set DRV_USART_DRIVER_MODE to "DYNAMIC" if APP_DRV_USART_RW_TX${INSTANCE}
	set DRV_USART_INTERRUPT_MODE to n  if APP_DRV_USART_RW_TX${INSTANCE}
	set DRV_USART_BYTE_MODEL_SUPPORT to n  if APP_DRV_USART_RW_TX${INSTANCE}
	set DRV_USART_READ_WRITE_MODEL_SUPPORT to y  if APP_DRV_USART_RW_TX${INSTANCE}
		---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony USART TX (Read/Write Mode) Application Template</h2>
	<p>	This template generates a simple code example which transmits
	a string of bytes via UART. It will automatically configure
	the UART driver with the following settings:</p>
	<br>- Dynamic Driver Mode
	<br>- Read/Write Transfer Model
	<br>- Polling Mode (Interrupts Disabled)
	<p>All other USART driver configuration options are set to their 
	default values. The driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> 
	Drivers -> USART</p></html>
	---endhelp---

config APP_DRV_USART_RW_TX_STRING${INSTANCE}
	string "Transmit String"
	default "Hello World"
	depends on APP_DRV_USART_RW_TX${INSTANCE}

config APP_DRV_USART_RW_RX${INSTANCE}
	bool "Receive String (Read/Write Mode)"
	default n
	persistent if APP_DRV_USART_BM_RX${INSTANCE} || APP_DRV_USART_BM_TX${INSTANCE} || APP_DRV_USART_BQ_RX${INSTANCE} || APP_DRV_USART_BQ_TX${INSTANCE}
	set DRV_USART_DRIVER_MODE to "DYNAMIC" if APP_DRV_USART_RW_RX${INSTANCE}
	set DRV_USART_INTERRUPT_MODE to n  if APP_DRV_USART_RW_RX${INSTANCE}
	set DRV_USART_BYTE_MODEL_SUPPORT to n  if APP_DRV_USART_RW_RX${INSTANCE}
	set DRV_USART_READ_WRITE_MODEL_SUPPORT to y  if APP_DRV_USART_RW_RX${INSTANCE}
		---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony USART RX (Read/Write Mode) Application Template</h2>
	<p>	This template generates a simple code example which receives
	a string of bytes via UART. It will automatically configure
	the UART driver with the following settings:</p>
	<br>- Dynamic Driver Mode
	<br>- Read/Write Transfer Model
	<br>- Polling Mode (Interrupts Disabled)
	<p>All other USART driver configuration options are set to their 
	default values. The driver configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> 
	Drivers -> USART</p></html>
	---endhelp---

config APP_DRV_USART_RW_RX_SIZE${INSTANCE}
	int "Number of Characters to Receive"
	default 10
	depends on APP_DRV_USART_RW_RX${INSTANCE}

menu "Options"
config APP_DRV_USART_INSTANCE_INDEX${INSTANCE}
    int "USART Driver Instance Index"
    range 0 0 if DRV_USART_INSTANCES_NUMBER = 1
    range 0 1 if DRV_USART_INSTANCES_NUMBER = 2
    range 0 2 if DRV_USART_INSTANCES_NUMBER = 3
    range 0 3 if DRV_USART_INSTANCES_NUMBER = 4
    range 0 4 if DRV_USART_INSTANCES_NUMBER = 5
    range 0 5 if DRV_USART_INSTANCES_NUMBER = 6
    range 0 6 if DRV_USART_INSTANCES_NUMBER = 7
    range 0 7 if DRV_USART_INSTANCES_NUMBER = 8
    range 0 8 if DRV_USART_INSTANCES_NUMBER = 9
    default 0

config APP_DRV_USART_HANDLE${INSTANCE}
    string "Name of the USART driver handle?"
    default "handleUSART0" if APP_DRV_USART_INSTANCE_INDEX${INSTANCE} = 0
    default "handleUSART1" if APP_DRV_USART_INSTANCE_INDEX${INSTANCE} = 1
    default "handleUSART2" if APP_DRV_USART_INSTANCE_INDEX${INSTANCE} = 2
    default "handleUSART3" if APP_DRV_USART_INSTANCE_INDEX${INSTANCE} = 3
    default "handleUSART4" if APP_DRV_USART_INSTANCE_INDEX${INSTANCE} = 4
    default "handleUSART5" if APP_DRV_USART_INSTANCE_INDEX${INSTANCE} = 5
    default "handleUSART6" if APP_DRV_USART_INSTANCE_INDEX${INSTANCE} = 6
    default "handleUSART7" if APP_DRV_USART_INSTANCE_INDEX${INSTANCE} = 7
    default "handleUSART8" if APP_DRV_USART_INSTANCE_INDEX${INSTANCE} = 8
    default "handleUSART9" if APP_DRV_USART_INSTANCE_INDEX${INSTANCE} = 9
    default "handleUSART"
endmenu

add "^#include \"/utilities/mhc/config/gencode/framework/driver/usart/config/drv_usart_gencode_byte_model_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS if DRV_USART_BYTE_MODEL_SUPPORT
add "^#include \"/utilities/mhc/config/gencode/framework/driver/usart/config/drv_usart_gencode_bq_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS if DRV_USART_BUFFER_QUEUE_SUPPORT
add "^#include \"/utilities/mhc/config/gencode/framework/driver/usart/config/drv_usart_gencode_read_write_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS if DRV_USART_READ_WRITE_MODEL_SUPPORT

add "^@macro_drv_usart_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_drv_usart_app_h_constants/>" to list APP${INSTANCE}_H_CONSTANTS
add "^@macro_drv_usart_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_drv_usart_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_drv_usart_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS
add "^@macro_drv_usart_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES

add "^#include \"/utilities/mhc/config/gencode/framework/driver/usart/config/drv_usart_gencode_byte_model_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS if DRV_USART_BYTE_MODEL_SUPPORT
add "^#include \"/utilities/mhc/config/gencode/framework/driver/usart/config/drv_usart_gencode_bq_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS if DRV_USART_BUFFER_QUEUE_SUPPORT
add "^#include \"/utilities/mhc/config/gencode/framework/driver/usart/config/drv_usart_gencode_read_write_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS if DRV_USART_READ_WRITE_MODEL_SUPPORT

add "^@macro_drv_usart_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_drv_usart_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_drv_usart_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_drv_usart_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_drv_usart_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_drv_usart_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_drv_usart_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_drv_usart_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_drv_usart_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_drv_usart_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
    
endif

endmenu
