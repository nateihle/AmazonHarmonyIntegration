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
// <editor-fold defaultstate="collapsed" desc="DRV_CODEC_AK7755 Configuration">
/*** CODEC Driver Initialization Data ***/
<#-- Instance 0 -->
<#if CONFIG_DRV_CODEC_AK7755_INST_IDX0 == true>
const DRV_AK7755_INIT drvak7755Codec0InitData =
{
    .moduleInit.value = SYS_MODULE_POWER_RUN_FULL,
<#if CONFIG_DRV_AK7755_I2S_DRIVER_MODULE_INDEX_IDX0?has_content>
    .i2sDriverModuleIndex = DRV_AK7755_I2S_DRIVER_MODULE_INDEX_IDX0,
</#if>
<#if CONFIG_DRV_AK7755_I2C_DRIVER_MODULE_INDEX_IDX0?has_content>
    .i2cDriverModuleIndex = DRV_AK7755_I2C_DRIVER_MODULE_INDEX_IDX0,
</#if>

<#if CONFIG_DRV_AK7755_VOLUME?has_content>
    .volume = DRV_AK7755_VOLUME,
</#if>
};
</#if>
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
