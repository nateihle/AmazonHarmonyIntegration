<#--
/*******************************************************************************
  AK7755 Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ak7755.ftl

  Summary:
    AK7755 Driver Freemarker Template File

  Description:

*******************************************************************************/

/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
-->

/*** Codec Driver Configuration ***/


<#if CONFIG_USE_DRV_CODEC_AK7755 ==  true>
#define DRV_AK7755_CLIENTS_NUMBER							${CONFIG_DRV_AK7755_CLIENTS_NUMBER}
#define DRV_AK7755_INSTANCES_NUMBER							1
<#if CONFIG_SYS_CLK_REFCLK_ENABLE?has_content><#if CONFIG_SYS_CLK_REFCLK_ENABLE == true>
#define DRV_AK7755_INPUT_REFCLOCK    	                	${CONFIG_SYS_CLK_REFCLK_ROSEL}
</#if></#if>
<#if CONFIG_SYS_CLK_REFCLK0_ENABLE?has_content><#if CONFIG_SYS_CLK_REFCLK0_ENABLE == true>
#define DRV_AK7755_INPUT_REFCLOCK    	                	${CONFIG_SYS_CLK_REFCLK_ROSEL0}
</#if></#if>
<#if CONFIG_SYS_CLK_REFCLK1_ENABLE?has_content><#if CONFIG_SYS_CLK_REFCLK1_ENABLE == true>
#define DRV_AK7755_INPUT_REFCLOCK    	                	${CONFIG_SYS_CLK_REFCLK_ROSEL1}
</#if></#if>
<#if CONFIG_SYS_CLK_REFCLK2_ENABLE?has_content><#if CONFIG_SYS_CLK_REFCLK2_ENABLE == true>
#define DRV_AK7755_INPUT_REFCLOCK    	                	${CONFIG_SYS_CLK_REFCLK_ROSEL2}
</#if></#if>
<#if CONFIG_SYS_CLK_REFCLK3_ENABLE?has_content><#if CONFIG_SYS_CLK_REFCLK3_ENABLE == true>
#define DRV_AK7755_INPUT_REFCLOCK    	                	${CONFIG_SYS_CLK_REFCLK_ROSEL3}
</#if></#if>
#define DRV_AK7755_AUDIO_SAMPLING_RATE						${CONFIG_DRV_I2S_BAUD_RATE}
#define DRV_AK7755_MCLK_SAMPLE_FREQ_MULTPLIER	            (SYS_CLK_BUS_REFERENCE_1/DRV_AK7755_AUDIO_SAMPLING_RATE)
#define DRV_AK7755_BCLK_BIT_CLK_DIVISOR	                	${CONFIG_DRV_AK7755_BCLK_BIT_CLK_DIVISOR}
<#if CONFIG_DRV_AK7755_ENABLE_MICROPHONE>
#define DRV_AK7755_ENABLE_MICROPHONE						1
<#else>
#define DRV_AK7755_ENABLE_MICROPHONE						0
</#if>					

<#-- Instance 0 -->
<#if CONFIG_DRV_CODEC_AK7755_INST_IDX0 == true>
#define DRV_AK7755_I2S_DRIVER_MODULE_INDEX_IDX0				${CONFIG_DRV_AK7755_I2S_DRIVER_MODULE_INDEX_IDX0}
#define DRV_AK7755_I2C_DRIVER_MODULE_INDEX_IDX0				${CONFIG_DRV_AK7755_I2C_DRIVER_MODULE_INDEX_IDX0}
#define DRV_AK7755_VOLUME									${CONFIG_DRV_AK7755_VOLUME}
#define DRV_AK7755_VOLUME_MIN                           	0x0                                           
#define DRV_AK7755_VOLUME_MAX                           	0xFF

<#if CONFIG_DRV_AK7755_I2S_DRIVER_MODULE_INDEX_IDX0 == "DRV_I2S_INDEX_0">
<#assign VAR_DRV_I2S_AUDIO_PROTOCOL_MODE = CONFIG_DRV_I2S_AUDIO_PROTOCOL_MODE_IDX0>
<#elseif CONFIG_DRV_AK7755_I2S_DRIVER_MODULE_INDEX_IDX0 == "DRV_I2S_INDEX_1">
<#assign VAR_DRV_I2S_AUDIO_PROTOCOL_MODE = CONFIG_DRV_I2S_AUDIO_PROTOCOL_MODE_IDX1>
<#elseif CONFIG_DRV_AK7755_I2S_DRIVER_MODULE_INDEX_IDX0 == "DRV_I2S_INDEX_2">
<#assign VAR_DRV_I2S_AUDIO_PROTOCOL_MODE = CONFIG_DRV_I2S_AUDIO_PROTOCOL_MODE_IDX2>
<#elseif CONFIG_DRV_AK7755_I2S_DRIVER_MODULE_INDEX_IDX0 == "DRV_I2S_INDEX_3">
<#assign VAR_DRV_I2S_AUDIO_PROTOCOL_MODE = CONFIG_DRV_I2S_AUDIO_PROTOCOL_MODE_IDX3>
<#elseif CONFIG_DRV_AK7755_I2S_DRIVER_MODULE_INDEX_IDX0 == "DRV_I2S_INDEX_4">
<#assign VAR_DRV_I2S_AUDIO_PROTOCOL_MODE = CONFIG_DRV_I2S_AUDIO_PROTOCOL_MODE_IDX4>
<#elseif CONFIG_DRV_AK7755_I2S_DRIVER_MODULE_INDEX_IDX0 == "DRV_I2S_INDEX_5">
<#assign VAR_DRV_I2S_AUDIO_PROTOCOL_MODE = CONFIG_DRV_I2S_AUDIO_PROTOCOL_MODE_IDX5>
</#if>


<#if CONFIG_SPI_AUDIO_COMM_WIDTH_IDX0 == "SPI_AUDIO_COMMUNICATION_16DATA_16FIFO_16CHANNEL">
	<#if VAR_DRV_I2S_AUDIO_PROTOCOL_MODE == "DRV_I2S_AUDIO_I2S">
#define DRV_AK7755_AUDIO_DATA_FORMAT_MACRO              	DRV_AK7755_AUDIO_DATA_FORMAT_I2S
#define DRV_AK7755_LRCK_IF_FORMAT_MACRO						DRV_AK7755_LRCK_IF_I2S_COMPATIBLE
#define DRV_AK7755_CLOCK_BICK_FS                            DRV_AK7755_BICK_64FS
#define DRV_AK7755_DAC_INPUT_FORMAT_MACRO                   DRV_AK7755_DAC_INPUT_24BITMSB
#define DRV_AK7755_DSP_DOUT4_OUTPUT_FORMAT_MACRO            DRV_AK7755_DSP_DOUT4_OUTPUT_24BITMSB
#define DRV_AK7755_DSP_DIN1_INPUT_FORMAT_MACRO              DRV_AK7755_DSP_DIN1_INPUT_24BITMSB
#define DRV_AK7755_DSP_DOUT1_OUTPUT_FORMAT_MACRO            DRV_AK7755_DSP_DOUT1_OUTPUT_24BITMSB
	</#if>
	<#if VAR_DRV_I2S_AUDIO_PROTOCOL_MODE == "DRV_I2S_AUDIO_LFET_JUSTIFIED">
#error "The AK7755 CODEC driver does not supports the selected combination of "Audio Communication Width" and "Audio Protocol Mode" in i2s driver configuration"  
	</#if>
	<#if VAR_DRV_I2S_AUDIO_PROTOCOL_MODE == "DRV_I2S_AUDIO_RIGHT_JUSTIFIED">
#define DRV_AK7755_LRCK_IF_FORMAT_MACRO						DRV_AK7755_LRCK_IF_STANDARD
#define DRV_AK7755_CLOCK_BICK_FS                            DRV_AK7755_BICK_32FS
#define DRV_AK7755_DAC_INPUT_FORMAT_MACRO                   DRV_AK7755_DAC_INPUT_16BITLSB
#define DRV_AK7755_DSP_DOUT4_OUTPUT_FORMAT_MACRO            DRV_AK7755_DSP_DOUT4_OUTPUT_16BITLSB
#define DRV_AK7755_DSP_DIN1_INPUT_FORMAT_MACRO              DRV_AK7755_DSP_DIN1_INPUT_16BITLSB
#define DRV_AK7755_DSP_DOUT1_OUTPUT_FORMAT_MACRO            DRV_AK7755_DSP_DOUT1_OUTPUT_16BITLSB
	</#if>
<#elseif CONFIG_SPI_AUDIO_COMM_WIDTH_IDX0 == "SPI_AUDIO_COMMUNICATION_16DATA_16FIFO_32CHANNEL">
	<#if VAR_DRV_I2S_AUDIO_PROTOCOL_MODE == "DRV_I2S_AUDIO_I2S">
#define DRV_AK7755_AUDIO_DATA_FORMAT_MACRO              	DRV_AK7755_AUDIO_DATA_FORMAT_I2S
#define DRV_AK7755_LRCK_IF_FORMAT_MACRO						DRV_AK7755_LRCK_IF_I2S_COMPATIBLE
#define DRV_AK7755_CLOCK_BICK_FS                            DRV_AK7755_BICK_64FS
#define DRV_AK7755_DAC_INPUT_FORMAT_MACRO                   DRV_AK7755_DAC_INPUT_24BITMSB
#define DRV_AK7755_DSP_DOUT4_OUTPUT_FORMAT_MACRO            DRV_AK7755_DSP_DOUT4_OUTPUT_24BITMSB
#define DRV_AK7755_DSP_DIN1_INPUT_FORMAT_MACRO              DRV_AK7755_DSP_DIN1_INPUT_24BITMSB
#define DRV_AK7755_DSP_DOUT1_OUTPUT_FORMAT_MACRO            DRV_AK7755_DSP_DOUT1_OUTPUT_24BITMSB
	</#if>
	<#if VAR_DRV_I2S_AUDIO_PROTOCOL_MODE == "DRV_I2S_AUDIO_LFET_JUSTIFIED">
#error "The AK7755 CODEC driver does not supports the selected combination of "Audio Communication Width" and "Audio Protocol Mode" in i2s driver configuration" 
	</#if>
	<#if VAR_DRV_I2S_AUDIO_PROTOCOL_MODE == "DRV_I2S_AUDIO_RIGHT_JUSTIFIED">
#define DRV_AK7755_LRCK_IF_FORMAT_MACRO						DRV_AK7755_LRCK_IF_STANDARD
#define DRV_AK7755_CLOCK_BICK_FS                            DRV_AK7755_BICK_64FS
#define DRV_AK7755_DAC_INPUT_FORMAT_MACRO                   DRV_AK7755_DAC_INPUT_16BITLSB
#define DRV_AK7755_DSP_DOUT4_OUTPUT_FORMAT_MACRO            DRV_AK7755_DSP_DOUT4_OUTPUT_16BITLSB
#define DRV_AK7755_DSP_DIN1_INPUT_FORMAT_MACRO              DRV_AK7755_DSP_DIN1_INPUT_16BITLSB
#define DRV_AK7755_DSP_DOUT1_OUTPUT_FORMAT_MACRO            DRV_AK7755_DSP_DOUT1_OUTPUT_16BITLSB
	</#if>
<#elseif CONFIG_SPI_AUDIO_COMM_WIDTH_IDX0 == "SPI_AUDIO_COMMUNICATION_24DATA_32FIFO_32CHANNEL">
	<#if VAR_DRV_I2S_AUDIO_PROTOCOL_MODE == "DRV_I2S_AUDIO_I2S">
#define DRV_AK7755_AUDIO_DATA_FORMAT_MACRO              	DRV_AK7755_AUDIO_DATA_FORMAT_I2S
#define DRV_AK7755_LRCK_IF_FORMAT_MACRO						DRV_AK7755_LRCK_IF_I2S_COMPATIBLE
#define DRV_AK7755_CLOCK_BICK_FS                            DRV_AK7755_BICK_64FS
#define DRV_AK7755_DAC_INPUT_FORMAT_MACRO                   DRV_AK7755_DAC_INPUT_24BITMSB
#define DRV_AK7755_DSP_DOUT4_OUTPUT_FORMAT_MACRO            DRV_AK7755_DSP_DOUT4_OUTPUT_24BITMSB
#define DRV_AK7755_DSP_DIN1_INPUT_FORMAT_MACRO              DRV_AK7755_DSP_DIN1_INPUT_24BITMSB
#define DRV_AK7755_DSP_DOUT1_OUTPUT_FORMAT_MACRO            DRV_AK7755_DSP_DOUT1_OUTPUT_24BITMSB
	</#if>
	<#if VAR_DRV_I2S_AUDIO_PROTOCOL_MODE == "DRV_I2S_AUDIO_LFET_JUSTIFIED">
#define DRV_AK7755_LRCK_IF_FORMAT_MACRO						DRV_AK7755_LRCK_IF_STANDARD
#define DRV_AK7755_CLOCK_BICK_FS                            DRV_AK7755_BICK_64FS
#define DRV_AK7755_DAC_INPUT_FORMAT_MACRO                   DRV_AK7755_DAC_INPUT_24BITMSB
#define DRV_AK7755_DSP_DOUT4_OUTPUT_FORMAT_MACRO            DRV_AK7755_DSP_DOUT4_OUTPUT_24BITMSB
#define DRV_AK7755_DSP_DIN1_INPUT_FORMAT_MACRO              DRV_AK7755_DSP_DIN1_INPUT_24BITMSB
#define DRV_AK7755_DSP_DOUT1_OUTPUT_FORMAT_MACRO            DRV_AK7755_DSP_DOUT1_OUTPUT_24BITMSB
	</#if>
	<#if VAR_DRV_I2S_AUDIO_PROTOCOL_MODE == "DRV_I2S_AUDIO_RIGHT_JUSTIFIED">
#define DRV_AK7755_LRCK_IF_FORMAT_MACRO						DRV_AK7755_LRCK_IF_STANDARD
#define DRV_AK7755_CLOCK_BICK_FS                            DRV_AK7755_BICK_64FS
#define DRV_AK7755_DAC_INPUT_FORMAT_MACRO                   DRV_AK7755_DAC_INPUT_24BITLSB
#define DRV_AK7755_DSP_DOUT4_OUTPUT_FORMAT_MACRO            DRV_AK7755_DSP_DOUT4_OUTPUT_24BITLSB
#define DRV_AK7755_DSP_DIN1_INPUT_FORMAT_MACRO              DRV_AK7755_DSP_DIN1_INPUT_24BITLSB
#define DRV_AK7755_DSP_DOUT1_OUTPUT_FORMAT_MACRO            DRV_AK7755_DSP_DOUT1_OUTPUT_24BITLSB
	</#if>
<#else>
	#error "The AK7755 CODEC driver does not supports the selected combination of "Audio Communication Width" and "Audio Protocol Mode" in i2s driver configuration"  
</#if>

</#if> 
</#if> 
/* CODEC Driver Abstraction definition */

#define DRV_CODEC_INDEX_0                                   DRV_AK7755_INDEX_0
#define sysObjdrvCodec0                                     sysObj.drvak7755Codec0
#define DRV_CODEC_CHANNEL                                   DRV_AK7755_CHANNEL
#define DRV_CODEC_CHANNEL_LEFT                              DRV_AK7755_CHANNEL_LEFT
#define DRV_CODEC_CHANNEL_RIGHT                             DRV_AK7755_CHANNEL_RIGHT
#define DRV_CODEC_CHANNEL_LEFT_RIGHT                        DRV_AK7755_CHANNEL_LEFT_RIGHT
#define DRV_CODEC_BUFFER_HANDLE                             DRV_AK7755_BUFFER_HANDLE
#define DRV_CODEC_BUFFER_HANDLE_INVALID                     DRV_AK7755_BUFFER_HANDLE_INVALID
#define DRV_CODEC_BUFFER_EVENT_HANDLER                      DRV_AK7755_BUFFER_EVENT_HANDLER
#define DRV_CODEC_BUFFER_EVENT                              DRV_AK7755_BUFFER_EVENT
#define DRV_CODEC_BUFFER_EVENT_COMPLETE                     DRV_AK7755_BUFFER_EVENT_COMPLETE
#define DRV_CODEC_BUFFER_EVENT_ERROR                        DRV_AK7755_BUFFER_EVENT_ERROR
#define DRV_CODEC_BUFFER_EVENT_ABORT                        DRV_AK7755_BUFFER_EVENT_ABORT
#define DRV_CODEC_COMMAND_EVENT_HANDLER                     DRV_AK7755_COMMAND_EVENT_HANDLER
#define DRV_CODEC_VOLUME_MIN                                DRV_AK7755_VOLUME_MIN
#define DRV_CODEC_VOLUME_MAX                                DRV_AK7755_VOLUME_MAX
#define DRV_CODEC_MICROPHONE_TYPE                           DRV_AK7755_INT_EXT_MIC
#define DRV_CODEC_MICROPHONE_TYPE_INTERNAL                  INT_MIC
#define DRV_CODEC_MICROPHONE_TYPE_EXTERNAL                  EXT_MIC
#define DRV_CODEC_MICROPHONE_SOUND                          DRV_AK7755_MONO_STEREO_MIC
#define DRV_CODEC_MICROPHONE_SOUND_NONE                     ALL_ZEROS
#define DRV_CODEC_MICROPHONE_SOUND_MONO_RIGHT               MONO_RIGHT_CHANNEL
#define DRV_CODEC_MICROPHONE_SOUND_MONO_LEFT                MONO_LEFT_CHANNEL
#define DRV_CODEC_MICROPHONE_SOUND_STEREO                   STEREO

#define DRV_CODEC_Initialize                                DRV_AK7755_Initialize
#define DRV_CODEC_Deinitialize                              DRV_AK7755_Deinitialize
#define DRV_CODEC_Status                                    DRV_AK7755_Status
#define DRV_CODEC_Tasks                                     DRV_AK7755_Tasks
#define DRV_CODEC_Open                                      DRV_AK7755_Open
#define DRV_CODEC_Close                                     DRV_AK7755_Close
#define DRV_CODEC_BufferEventHandlerSet                     DRV_AK7755_BufferEventHandlerSet
#define DRV_CODEC_BufferAddWrite                            DRV_AK7755_BufferAddWrite
#define DRV_CODEC_BufferAddRead                             DRV_AK7755_BufferAddRead
#define DRV_CODEC_BufferAddWriteRead                        DRV_AK7755_BufferAddWriteRead
#define DRV_CODEC_SamplingRateSet                           DRV_AK7755_SamplingRateSet
#define DRV_CODEC_SamplingRateGet                           DRV_AK7755_SamplingRateGet
#define DRV_CODEC_VolumeSet                                 DRV_AK7755_VolumeSet
#define DRV_CODEC_VolumeGet                                 DRV_AK7755_VolumeGet
#define DRV_CODEC_MuteOn                                    DRV_AK7755_MuteOn
#define DRV_CODEC_MuteOff                                   DRV_AK7755_MuteOff
#define DRV_CODEC_MicrophoneTypeSet                         DRV_AK7755_IntExtMicSet
#define DRV_CODEC_MicrophoneSoundSet                        DRV_AK7755_MonoStereoMicSet
#define DRV_CODEC_SetAudioCommunicationMode				 	DRV_AK7755_SetAudioCommunicationMode
#define DRV_CODEC_CommandEventHandlerSet                    DRV_AK7755_CommandEventHandlerSet


<#--
/*******************************************************************************
 End of File
*/
-->
