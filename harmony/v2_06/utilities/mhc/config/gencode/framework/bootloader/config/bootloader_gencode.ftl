menu "Bootloader"

config BOOTLOADER_USART_APP${INSTANCE}
    bool "Bootload Over UART? (Memory as Trigger Type)"
    set USE_BOOTLOADER to y if BOOTLOADER_USART_APP${INSTANCE}
    set BOOTLOADER_TYPE to "USART" if BOOTLOADER_USART_APP${INSTANCE}
	set USE_BOOTLOADER_LIBRARY to y if BOOTLOADER_USART_APP${INSTANCE}
	set USE_DRV_USART_NEEDED to y if BOOTLOADER_USART_APP${INSTANCE}
	set DRV_USART_DRIVER_MODE to "STATIC" if BOOTLOADER_USART_APP${INSTANCE}
	set DRV_USART_INTERRUPT_MODE to n if BOOTLOADER_USART_APP${INSTANCE}
	set DRV_USART_BYTE_MODEL_SUPPORT to y if BOOTLOADER_USART_APP${INSTANCE}
	set DRV_USART_BYTE_MODEL_BLOCKING to n if BOOTLOADER_USART_APP${INSTANCE}
	set BOOTLOADER_TRIGGER_TYPE to "MEMORY" if BOOTLOADER_USART_APP${INSTANCE}
    default n
	---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony USART Bootloader Application Template</h2>
	<p>	This template generates a simple code example to exercise USART bootloader. 
	Application will automatically setup the Bootloader library and USART module 
	with the following settings.</p>
	<br><b>- Bootloader Libray</b></br>
	<br>&nbsp&nbsp&nbsp&nbsp - Bootloader Type: USART</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Build a Bootloader</br>
	<br><b>- USART Driver</b></br>
	<br>&nbsp&nbsp&nbsp&nbsp - Driver Implementation: STATIC</br>
	<br>&nbsp&nbsp&nbsp&nbsp - Polling mode</br>
	<br>
	<p><i>NOTE: To complete the process, select the appropriate <b>USART module ID through
	USART driver MHC options</b> (Ex: In case of PIC32MZ EF Starter Kit with USB-to-UART bridge on board, select 
	"USART_ID_2" as USART module ID). In addition make sure to select and lock the appropriate pins 
    through pin manager.</i></p>
	<p>All other relavent Bootloader and USART driver configuration options are set 
	to their default values. <br><br>The Bootloader library configuration may be modified by 
	the user using MHC, under <b>"Harmony Framework Configuration -> Bootloader Library"</b>.</p>
	<p>The USART driver configuration may be modified by the user using MHC, under 
	<b>"Harmony Framework Configuration -> Drivers -> USART"</b>.</p>
	</html>
	---endhelp---

config BOOTLOADER_TRIGGER_MEMORY_LOCATION${INSTANCE}
	hex "Trigger Location in Memory"
	depends on BOOTLOADER_USART_APP${INSTANCE}
	default 0x9D000000
	---help---
	<html>
	<p>On startup bootloader checks this location trhough call back and makes a decision to jump to either application (if memory has data)
	or execute the bootloader (if memory has 0xFFFFFFFF - erased state).</p>
	</html>
	---endhelp---

menu "Options"
	depends on BOOTLOADER_USART_APP${INSTANCE}
	
config BOOTLOADER_APP_USART_INSTANCE_INDEX${INSTANCE}
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
endmenu

config BOOTLOADER_APP_USART_BAUDRATE_INDEX${INSTANCE}
	int
	set DRV_USART_BAUD_RATE_IDX0 optionally to "115200" if BOOTLOADER_APP_USART_INSTANCE_INDEX${INSTANCE} = 0
	set DRV_USART_BAUD_RATE_IDX1 optionally to "115200" if BOOTLOADER_APP_USART_INSTANCE_INDEX${INSTANCE} = 1
	set DRV_USART_BAUD_RATE_IDX2 optionally to "115200" if BOOTLOADER_APP_USART_INSTANCE_INDEX${INSTANCE} = 2
	set DRV_USART_BAUD_RATE_IDX3 optionally to "115200" if BOOTLOADER_APP_USART_INSTANCE_INDEX${INSTANCE} = 3
	set DRV_USART_BAUD_RATE_IDX4 optionally to "115200" if BOOTLOADER_APP_USART_INSTANCE_INDEX${INSTANCE} = 4
	set DRV_USART_BAUD_RATE_IDX5 optionally to "115200" if BOOTLOADER_APP_USART_INSTANCE_INDEX${INSTANCE} = 5		
	
ifblock BOOTLOADER_USART_APP${INSTANCE}

add "^#include \"/utilities/mhc/config/gencode/framework/bootloader/templates/bootloader_uart_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/bootloader/templates/bootloader_uart_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS

endif

ifblock BOOTLOADER_USART_APP${INSTANCE}

add "^@macro_lib_bootloader_uart_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_lib_bootloader_uart_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_lib_bootloader_uart_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_lib_bootloader_uart_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_lib_bootloader_uart_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS
add "^@macro_lib_bootloader_uart_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES

add "^@macro_lib_bootloader_uart_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_lib_bootloader_uart_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_lib_bootloader_uart_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_lib_bootloader_uart_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_lib_bootloader_uart_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_lib_bootloader_uart_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_lib_bootloader_uart_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_lib_bootloader_uart_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_lib_bootloader_uart_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_lib_bootloader_uart_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
add "^@macro_lib_bootloader_uart_app_c_tasks_app_functions/>" to list APP${INSTANCE}_C_APP_TASKS_APP_FUNCTIONS

endif

endmenu
