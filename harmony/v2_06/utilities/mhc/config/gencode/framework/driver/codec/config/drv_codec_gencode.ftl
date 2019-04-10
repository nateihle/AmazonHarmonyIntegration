menu "Codec"
    depends on HAVE_I2S
    
config GENERATE_CODE_DRV_AK4384${INSTANCE}
    bool "AK4384: Generate Tone"
	set SELECT_DRV_AK4384 optionally to y if GENERATE_CODE_DRV_AK4384${INSTANCE} 
	set USE_DRV_AK4384_MCLK optionally to n if GENERATE_CODE_DRV_AK4384${INSTANCE} 
	set DRV_I2S_ENABLE_DMA optionally to y if GENERATE_CODE_DRV_AK4384${INSTANCE} 
	set DRV_I2S_SUPPORT_TRANSMIT_DMA optionally to y if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set DRV_I2S_BAUD_RATE optionally to 48000 if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set DRV_I2S_AUDIO_PROTOCOL_MODE_IDX0 optionally to "DRV_I2S_AUDIO_RIGHT_JUSTIFIED" if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set SPI_AUDIO_COMM_WIDTH_IDX0 optionally to "SPI_AUDIO_COMMUNICATION_16DATA_16FIFO_32CHANNEL" if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set DRV_TMR_PERIPHERAL_ID_IDX0 optionally to "TMR_ID_4" if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set DRV_TMR_INTERRUPT_PRIORITY_IDX0 optionally to "INT_PRIORITY_LEVEL4" if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set SYS_CLK_REFCLK_ENABLE optionally to y if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set SYS_CLK_REFCLK_OE optionally to y if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set SYS_CLK_REFCLK_SOURCE optionally to "USB PLL UPLL" if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set SYS_CLK_RODIV optionally to 3 if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set SYS_CLK_ROTRIM optionally to 464 if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set SYS_CLK_CONFIG_SECONDARY_XTAL optionally to 32768 if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set SYS_DMA_CHANNEL_ID_IDX0 optionally to "DMA_CHANNEL_2" if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set SYS_DMA_INTERRUPT_PRIORITY_CH0 optionally to "INT_PRIORITY_LEVEL2" if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set USE_SYS_PORTS_CN_INTERRUPT optionally to y if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set SYS_PORTS_CN_INTERRUPT_PRIORITY optionally to "INT_PRIORITY_LEVEL5" if GENERATE_CODE_DRV_AK4384${INSTANCE}
	set XC32_HEAP optionally to "8000" if GENERATE_CODE_DRV_AK4384${INSTANCE}
	default n
    ---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony AK4384 Codec Application Template</h2>
	<h3>Generate Tone and play through the AK4384 Codec</h2>
	<p>This template generates a simple code example which demonstrates
	the setup and use of the AK4384 codec to play a simple audio tone.
	It uses the PIC32 Bluetooth Audio Development Board along with the
	PIC32 Audio DAC daughter board with the AK4384 codec. The tone is
	generated from an array of samples representing a sine wave at 1000 Hz.
	The tone is fed to the codec via DMA, and transmitted over I2S. The 
	template automatically configures the project with the following settings:</p>
	<br>- AK4384 Codec Driver
	<br>- Set 16-bit Audio Data, Right Justified
	<br>- Enable DMA Transmit on Channel 2 with IRQ priority 2
	<br>- Enable I2S driver with baud rate 48000 and DMA
	<br>- Enable Timer 4 (for Codec control)
	<br>- Enable REFCLK0 with USBPLL source with ODIV of 3 and TRIM of 464
	<br>- Configure SOSC to 32768 Hz
	<br>- Set heap to 8000 (bytes)
	<p>All other configuration options are set to their default values. 
	The driver configuration may be modified by the user using MHC, 
	under Harmony Framework Configuration -> Drivers -> Codec ->
	"Use Codec AK4384".</p>
	<p> The PPS settings for the I2S and REFCLK must be set up 
	manually.</p>
	<br>- REFCLKO (out) RPF8
	<br>- SS1 (out) RPD9
	<br>- SDO1 (out) RPD0</html>
    ---endhelp---

endmenu

ifblock GENERATE_CODE_DRV_AK4384${INSTANCE}

add "^#include \"/utilities/mhc/config/gencode/framework/driver/codec/config/drv_codec_gencode_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS 

add "^@macro_drv_codec_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_drv_codec_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_drv_codec_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_drv_codec_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_drv_codec_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS
add "^@macro_drv_codec_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES

add "^#include \"/utilities/mhc/config/gencode/framework/driver/codec/config/drv_codec_gencode_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS 

add "^@macro_drv_codec_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_drv_codec_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_drv_codec_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_drv_codec_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_drv_codec_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_drv_codec_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_drv_codec_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_drv_codec_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_drv_codec_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_drv_codec_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
    
endif

