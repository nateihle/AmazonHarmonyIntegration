<#--
/*******************************************************************************
Copyright (c) 2016-2017 released Microchip Technology Inc.  All rights reserved.

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
// <editor-fold defaultstate="collapsed" desc="DRV_SQI Initialization Data">
/*** SQI Driver Initialization Data ***/
<#if CONFIG_USE_DRV_SQI == true>
<#assign divValue = CONFIG_SYS_CLK_REFCLK1_FREQ?number / CONFIG_DRV_SQI_CLK_FREQ?number>
const DRV_SQI_INIT drvSqiInit =
{
<#if CONFIG_DRV_SQI_PERIPHERAL_ID?has_content>
    .sqiId = ${CONFIG_DRV_SQI_PERIPHERAL_ID},
</#if>
<#if CONFIG_DRV_SQI_INTERRUPT_SOURCE?has_content>
    .interruptSource = ${CONFIG_DRV_SQI_INTERRUPT_SOURCE},
</#if>
<#if CONFIG_USE_DRV_SQI_DEVICE_0 == true && CONFIG_USE_DRV_SQI_DEVICE_1 == true>
    .enabledDevices = DRV_SQI_ENABLE_BOTH_DEVICES,
<#elseif CONFIG_USE_DRV_SQI_DEVICE_1 == true>
    .enabledDevices = DRV_SQI_ENABLE_DEVICE_1,
<#else>
    .enabledDevices = DRV_SQI_ENABLE_DEVICE_0,
</#if>
<#if divValue lte 1>
    .clockDivider = DRV_SQI_CLK_DIV_1,
<#elseif divValue lte 2>
    .clockDivider = DRV_SQI_CLK_DIV_2,
<#elseif divValue lte 4>
    .clockDivider = DRV_SQI_CLK_DIV_4,
<#elseif divValue lte 8>
    .clockDivider = DRV_SQI_CLK_DIV_8,
<#elseif divValue lte 16>
    .clockDivider = DRV_SQI_CLK_DIV_16,
<#elseif divValue lte 32>
    .clockDivider = DRV_SQI_CLK_DIV_32,
<#elseif divValue lte 64>
    .clockDivider = DRV_SQI_CLK_DIV_64,
<#elseif divValue lte 128>
    .clockDivider = DRV_SQI_CLK_DIV_128,
<#elseif divValue lte 256>
    .clockDivider = DRV_SQI_CLK_DIV_256,
<#elseif divValue lte 512>
    .clockDivider = DRV_SQI_CLK_DIV_512,
<#elseif divValue lte 1024>
    .clockDivider = DRV_SQI_CLK_DIV_1024,
<#else>
    .clockDivider = DRV_SQI_CLK_DIV_2048,
</#if>
<#if CONFIG_USE_DRV_SQI_DEVICE_0 == true>
    .devCfg[0].spiMode = ${CONFIG_DRV_SQI_DEVICE_0_SPI_OP_MODE_VALUE},
    <#if CONFIG_DRV_SQI_DEVICE_0_LSB_FIRST == true>
    .devCfg[0].lsbFirst = true,
    <#else>
    .devCfg[0].lsbFirst = false,
    </#if>
</#if>
<#if CONFIG_USE_DRV_SQI_DEVICE_1 == true>
<#if CONFIG_USE_DRV_SQI_DEVICE_0 == false>
    .devCfg[0].spiMode = ${CONFIG_DRV_SQI_DEVICE_1_SPI_OP_MODE_VALUE},
    <#if CONFIG_DRV_SQI_DEVICE_1_LSB_FIRST == true>
    .devCfg[0].lsbFirst = true,
    <#else>
    .devCfg[0].lsbFirst = false,
    </#if>
<#else>
    .devCfg[1].spiMode = ${CONFIG_DRV_SQI_DEVICE_1_SPI_OP_MODE_VALUE},
    <#if CONFIG_DRV_SQI_DEVICE_1_LSB_FIRST == true>
    .devCfg[1].lsbFirst = true,
    <#else>
    .devCfg[1].lsbFirst = false,
    </#if>
</#if>
</#if>
};
</#if>
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
