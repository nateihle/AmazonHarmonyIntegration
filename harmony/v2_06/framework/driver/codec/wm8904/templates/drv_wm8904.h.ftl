<#--
/*******************************************************************************
  WM8904 Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_wm8904.ftl

  Summary:
    WM8904 Driver Freemarker Template File

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

<#if CONFIG_USE_DRV_CODEC_WM8904 ==  true>
#define DRV_WM8904_CLIENTS_NUMBER                           ${CONFIG_DRV_WM8904_CLIENTS_NUMBER}
#define DRV_WM8904_INSTANCES_NUMBER                         1
<#if CONFIG_SYS_CLK_REFCLK_ENABLE?has_content><#if CONFIG_SYS_CLK_REFCLK_ENABLE == true>
#define DRV_WM8904_INPUT_REFCLOCK    	                	${CONFIG_SYS_CLK_REFCLK_ROSEL}
</#if></#if>
<#if CONFIG_SYS_CLK_REFCLK0_ENABLE?has_content><#if CONFIG_SYS_CLK_REFCLK0_ENABLE == true>
#define DRV_WM8904_INPUT_REFCLOCK    	                	${CONFIG_SYS_CLK_REFCLK_ROSEL0}
</#if></#if>
</#if>

#define DRV_WM8904_AUDIO_SAMPLING_RATE                      ${CONFIG_DRV_WM8904_BAUD_RATE}
#define DRV_WM8904_VOLUME	                      	        ${CONFIG_DRV_WM8904_VOLUME}
#define DRV_WM8904_AUDIO_DATA_FORMAT_MACRO             	    ${CONFIG_DRV_WM8904_AUDIO_DATA_FORMAT}
#define DRV_WM8904_ENABLE_MIC_INPUT             	    	${CONFIG_DRV_WM8904_ENABLE_MIC_INPUT?c}

<#-- Instance 0 -->
<#if CONFIG_DRV_CODEC_WM8904_INST_IDX0 == true>
#define DRV_WM8904_I2S_DRIVER_MODULE_INDEX_IDX0             ${CONFIG_DRV_WM8904_I2S_DRIVER_MODULE_INDEX_IDX0}
#define DRV_WM8904_I2C_DRIVER_MODULE_INDEX_IDX0             ${CONFIG_DRV_WM8904_I2C_DRIVER_MODULE_INDEX_IDX0}
</#if> 
/* CODEC Driver Abstraction definition */

#define DRV_CODEC_INDEX_0                                   DRV_WM8904_INDEX_0
#define sysObjdrvCodec0                                     sysObj.drvwm8904Codec0
#define DRV_CODEC_BUFFER_HANDLE                             DRV_WM8904_BUFFER_HANDLE
#define DRV_CODEC_BUFFER_HANDLE_INVALID                     DRV_WM8904_BUFFER_HANDLE_INVALID
#define DRV_CODEC_BUFFER_EVENT_HANDLER                      DRV_WM8904_BUFFER_EVENT_HANDLER
#define DRV_CODEC_BUFFER_EVENT                              DRV_WM8904_BUFFER_EVENT
#define DRV_CODEC_BUFFER_EVENT_COMPLETE                     DRV_WM8904_BUFFER_EVENT_COMPLETE
#define DRV_CODEC_BUFFER_EVENT_ERROR                        DRV_WM8904_BUFFER_EVENT_ERROR
#define DRV_CODEC_BUFFER_EVENT_ABORT                        DRV_WM8904_BUFFER_EVENT_ABORT
#define DRV_CODEC_COMMAND_EVENT_HANDLER                     DRV_WM8904_COMMAND_EVENT_HANDLER

#define DRV_CODEC_CHANNEL_LEFT                              DRV_WM8904_CHANNEL_LEFT
#define DRV_CODEC_CHANNEL_RIGHT                             DRV_WM8904_CHANNEL_RIGHT
#define DRV_CODEC_CHANNEL_LEFT_RIGHT                        DRV_WM8904_CHANNEL_LEFT_RIGHT

#define DRV_CODEC_Initialize                                DRV_WM8904_Initialize
#define DRV_CODEC_Deinitialize                              DRV_WM8904_Deinitialize
#define DRV_CODEC_Status                                    DRV_WM8904_Status
#define DRV_CODEC_Tasks                                     DRV_WM8904_Tasks
#define DRV_CODEC_Open                                      DRV_WM8904_Open
#define DRV_CODEC_Close                                     DRV_WM8904_Close
#define DRV_CODEC_BufferEventHandlerSet                     DRV_WM8904_BufferEventHandlerSet
#define DRV_CODEC_CommandEventHandlerSet                    DRV_WM8904_CommandEventHandlerSet
#define DRV_CODEC_BufferAddWrite                            DRV_WM8904_BufferAddWrite
#define DRV_CODEC_BufferAddRead                             DRV_WM8904_BufferAddRead
#define DRV_CODEC_BufferAddWriteRead                        DRV_WM8904_BufferAddWriteRead
#define DRV_CODEC_SamplingRateSet                           DRV_WM8904_SamplingRateSet
#define DRV_CODEC_SamplingRateGet                           DRV_WM8904_SamplingRateGet
#define DRV_CODEC_VolumeSet                                 DRV_WM8904_VolumeSet
#define DRV_CODEC_VolumeGet                                 DRV_WM8904_VolumeGet
#define DRV_CODEC_MuteOn                                    DRV_WM8904_MuteOn
#define DRV_CODEC_MuteOff                                   DRV_WM8904_MuteOff

<#--
/*******************************************************************************
 End of File
*/
-->
