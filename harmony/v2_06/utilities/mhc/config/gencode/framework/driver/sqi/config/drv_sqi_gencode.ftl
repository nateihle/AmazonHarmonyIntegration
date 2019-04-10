menu "SQI"
    depends on HAVE_SQI

config GENERATE_CODE_DRV_SST26_SQI${INSTANCE}
    bool "SQI?"
    default n
    select DRV_SQI_NEEDED if GENERATE_CODE_DRV_SST26_SQI${INSTANCE}
	select DRV_SST26_NEEDED if GENERATE_CODE_DRV_SST26_SQI${INSTANCE}
    ---help---
    IDH_HTML_SQI_Driver_Library
    ---endhelp---

ifblock GENERATE_CODE_DRV_SST26_SQI${INSTANCE}

config APP_DRV_SST26_SQI_DMA${INSTANCE}
    bool "Write & Read SST26 Quad Flash in DMA mode"
    default n
    set DRV_SQI_DRIVER_MODE optionally to "DYNAMIC" if APP_DRV_SST26_SQI_DMA${INSTANCE}
	set USE_DRV_SQI_DEVICE_0 optionally to n if APP_DRV_SST26_SQI_DMA${INSTANCE}
	set USE_DRV_SQI_DEVICE_1 optionally to y if APP_DRV_SST26_SQI_DMA${INSTANCE}
	set DRV_SQI_DEVICE_1_LSB_FIRST optionally to n if APP_DRV_SST26_SQI_DMA${INSTANCE}
	set SYS_CLK_REFCLK1_ENABLE optionally to y if APP_DRV_SST26_SQI_DMA${INSTANCE}
	set SYS_CLK_RODIV1 optionally to 1 if  APP_DRV_SST26_SQI_DMA${INSTANCE}
    ---help---
    <!DOCTYPE HTML>
    <html>
    <h2>MPLAB Harmony SQI DMA Write/Read Application Template</h2>
    <p>	This template generates a simple code example which writes and reads SST26 series
	flash memory through SQI intreface. This app template uses both SQI and SST26
	drivers. It will automatically configure the SST26 & SQI
	drivers with the following settings:</p>
	<br><b>- SQI:</b>
	<br>&nbsp&nbsp&nbsp - Driver Mode: Dynamic
    <br>&nbsp&nbsp&nbsp - Data Transfer Mode: DMA
	<br>&nbsp&nbsp&nbsp - SPI Mode: 0
	<br>&nbsp&nbsp&nbsp - Clock Speed: 50 MHz
	<br>&nbsp&nbsp&nbsp - Chip Select/Device: 1
	<br><b>- SST26:</b>
	<br>&nbsp&nbsp&nbsp - Flash: SST26VF032B
	<br>&nbsp&nbsp&nbsp - SQI Chip Select: 1
    <p>All other SST26 & SQI driver configuration options are set to their default values. The driver
	configuration may be modified by the user using MHC, under Harmony Framework Configuration ->
    Drivers -> SST26 for SST26 Flash driver and Harmony Framework Configuration ->
    Drivers -> SQI </p></html>
    ---endhelp---

menu "Options"

config APP_DRV_SST26_SQI_START_ADDRESS${INSTANCE}
	hex "Flash Block Start Address (page aligned)"
	default 0x1000

config APP_DRV_SST26_SQI_XFER_SIZE${INSTANCE}
	int "Write & Read Size (in bytes)"
	default 4096
	
config APP_DRV_SST26_SQI_HANDLE${INSTANCE}
    string "Name of the SST26 driver handle?"
    default "handleSST26"

config APP_DRV_SST26_SQI_CMD_HANDLE${INSTANCE}
    string "Name of the SST26 command handle?"
    default "cmdHandleSST26"
endmenu

add "^#include \"/utilities/mhc/config/gencode/framework/driver/sqi/templates/drv_sqi_gencode_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_drv_sqi_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_drv_sqi_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES
add "^@macro_drv_sqi_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_drv_sqi_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_drv_sqi_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS

add "^#include \"/utilities/mhc/config/gencode/framework/driver/sqi/templates/drv_sqi_gencode_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS
add "^@macro_drv_sqi_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_drv_sqi_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_drv_sqi_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_drv_sqi_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_drv_sqi_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_drv_sqi_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_drv_sqi_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_drv_sqi_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_drv_sqi_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_drv_sqi_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES

endif

endmenu
