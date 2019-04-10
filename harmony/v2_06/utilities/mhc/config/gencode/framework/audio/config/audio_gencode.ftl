menu "Audio"
    depends on HAVE_I2S
    
menu "USB Audio Device Class"

config AUDIO_USB_DEV_CLASS${INSTANCE}
    bool "USB Audio Class: Receive Audio Data from Host"
    set USE_USB_STACK_NEEDED to y if AUDIO_USB_DEV_CLASS${INSTANCE}
    set DRV_USB_DEVICE_SUPPORT to y if AUDIO_USB_DEV_CLASS${INSTANCE}
    set DRV_USB_ENDPOINTS_NUMBER to 2 if AUDIO_USB_DEV_CLASS${INSTANCE}
	set USB_DEVICE_FUNCTION_1_DEVICE_CLASS_IDX0 to "AUDIO" if AUDIO_USB_DEV_CLASS${INSTANCE}
	set USB_DEVICE_FUNCTION_1_NUMBER_OF_INTERFACES_IDX0 to 2 if AUDIO_USB_DEV_CLASS${INSTANCE}
	set USB_DEVICE_FUNCTION_1_AUDIO_READ_QUEUE_SIZE_IDX0 to 2 if AUDIO_USB_DEV_CLASS${INSTANCE}
	set USB_DEVICE_PRODUCT_ID_SELECT_IDX0 to "usb_speaker_demo" if AUDIO_USB_DEV_CLASS${INSTANCE}
	set DRV_USB_INTERRUPT_PRIORITY_IDX0 to "INT_PRIORITY_LEVEL1" if AUDIO_USB_DEV_CLASS${INSTANCE}
	set XC32_HEAP to "8000" if AUDIO_USB_DEV_CLASS${INSTANCE}
    default n
	---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony USB CDC Client TX Application Template</h2>
	<p>	This template generates a simple code example which transmits
	a string of bytes as a USB CDC Client. It will automatically configure
	the USB stack with the following settings:</p>
	<br>- USB CDC Client
	<br>- Number of Endpoints: 3
	<br>- Product ID: "cdc_com_port_single_demo"
	<p>All other USB stack configuration options are set to their 
	default values. The stack configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> USB</p></html>
	---endhelp---

endmenu

ifblock AUDIO_USB_DEV_CLASS${INSTANCE}

add "^#include \"/utilities/mhc/config/gencode/framework/audio/config/audio_usb_dev_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/audio/config/audio_usb_dev_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/audio/config/audio_usb_dev_macros_system_config.h.ftl\">" to list APP_FREEMARKER_MACROS

endif

ifblock AUDIO_USB_DEV_CLASS${INSTANCE}

add "^@macro_audio_usb_dev_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_audio_usb_dev_system_config_h_app_constants/>" to list APP${INSTANCE}_H_CONSTANTS
add "^@macro_audio_usb_dev_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_audio_usb_dev_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_audio_usb_dev_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_audio_usb_dev_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS
add "^@macro_audio_usb_dev_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES

add "^@macro_audio_usb_dev_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_audio_usb_dev_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_audio_usb_dev_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_audio_usb_dev_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_audio_usb_dev_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_audio_usb_dev_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_audio_usb_dev_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_audio_usb_dev_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_audio_usb_dev_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_audio_usb_dev_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
endif

endmenu
