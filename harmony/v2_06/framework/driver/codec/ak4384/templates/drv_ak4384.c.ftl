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
// <editor-fold defaultstate="collapsed" desc="DRV_CODEC_AK4384 Initialization Data">
/*** CODEC Driver Initialization Data ***/
<#-- Instance 0 -->
<#if CONFIG_DRV_CODEC_AK4384_INST_IDX0 == true>
const DRV_AK4384_INIT drvak4384Codec0InitData =
{
    .moduleInit.value = SYS_MODULE_POWER_RUN_FULL,
<#if  CONFIG_USE_DRV_AK4384_BIT_BANGED_SPI_CONTROL_INTERFACE == false>
<#if CONFIG_DRV_AK4384_SPI_DRIVER_MODULE_INDEX_IDX0?has_content>
    .spiDriverModuleIndex  = DRV_AK4384_SPI_DRIVER_MODULE_INDEX_IDX0, 
</#if>
</#if>
<#if CONFIG_DRV_AK4384_I2S_DRIVER_MODULE_INDEX_IDX0?has_content>
    .i2sDriverModuleIndex = DRV_AK4384_I2S_DRIVER_MODULE_INDEX_IDX0,
</#if>
<#if CONFIG_DRV_AK4384_VOLUME?has_content>
    .volume = DRV_AK4384_VOLUME,
</#if>
<#if CONFIG_DRV_AK4384_MCLK_MODE_MACRO?has_content>
    .mclkMode = DRV_AK4384_MCLK_MODE_MACRO,
</#if>
<#if CONFIG_DRV_AK4384_DELAY_INITIALIZATION?has_content>
    .delayDriverInitialization = DRV_AK4384_DELAY_INITIALIZATION,
</#if>
};
</#if>
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
