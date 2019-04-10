<#--
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
 -->
<#-- Instance 0 -->
<#if CONFIG_DRV_CODEC_AK4642_INST_IDX0 == true>
<#if !(DRV_CODEC_AK4642_TASKS?has_content)>
<#assign DRV_CODEC_AK4642_TASKS = "TASK_CALL_NO_RTOS">
</#if>
<#if (DRV_CODEC_AK4642_TASKS == "PROTO") && (CONFIG_DRV_CODEC_AK4642_RTOS == "Standalone")>
void _DRV_CODEC_AK4642_Tasks(void);
</#if>
<#if DRV_CODEC_AK4642_TASKS == "CREATE_TASK">
 <#if CONFIG_DRV_CODEC_AK4642_RTOS == "Standalone">

    /* Create task for AK4642 Codec state machine*/
    xTaskCreate((TaskFunction_t) _DRV_CODEC_AK4642_Tasks,
    "DRV_CODEC_AK4642 Tasks",
    ${CONFIG_DRV_CODEC_AK4642_RTOS_TASK_SIZE},
    NULL,
    ${CONFIG_DRV_CODEC_AK4642_RTOS_TASK_PRIORITY},
    NULL);
 </#if>
</#if>
<#if (DRV_CODEC_AK4642_TASKS == "TASK_CALL_NO_RTOS") || (DRV_CODEC_AK4642_TASKS == "TASK_CALL" && CONFIG_DRV_CODEC_AK4642_RTOS != "Standalone")>
    DRV_AK4642_Tasks(sysObj.drvak4642Codec0);
</#if>
<#if DRV_CODEC_AK4642_TASKS == "LOCAL_FUNCTION">
 <#if CONFIG_DRV_CODEC_AK4642_RTOS == "Standalone">
 void _DRV_CODEC_AK4642_Tasks(void)
 {
    while(1)
    {
        DRV_AK4642_Tasks(sysObj.drvak4642Codec0);
  <#if CONFIG_DRV_CODEC_AK4642_RTOS_USE_DELAY>
   <#if CONFIG_FREERTOS_ENABLE_BACKWARD_COMPATIBILITY == true>
        vTaskDelay(${CONFIG_DRV_CODEC_AK4642_RTOS_DELAY} / portTICK_RATE_MS);
   <#else>
        vTaskDelay(${CONFIG_DRV_CODEC_AK4642_RTOS_DELAY} / portTICK_PERIOD_MS);
   </#if>
  </#if>
    }
 }
 </#if>
</#if>
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
