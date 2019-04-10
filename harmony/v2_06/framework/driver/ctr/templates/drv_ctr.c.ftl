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
//<editor-fold defaultstate="collapsed" desc="DRV_CTR Initialization Data">
<#if CONFIG_DRV_CTR_DRIVER_MODE == "DYNAMIC">
// *****************************************************************************
/* CTR Driver Initialization Data
*/

const DRV_CTR_INIT drvCtr0InitData =
{
<#if CONFIG_DRV_CTR_POWER_STATE?has_content>
    .moduleInit = DRV_CTR_POWER_STATE,
</#if>
<#if CONFIG_DRV_CTR_MODULE_ID?has_content>
    .ctrId = DRV_CTR_MODULE_ID, 
</#if>
<#if CONFIG_DRV_CTR_EVENT_INTERRUPT_SOURCE?has_content>
    .ctrEventInterruptSource = DRV_CTR_EVENT_INTERRUPT_SOURCE, 
</#if>
<#if CONFIG_DRV_CTR_EVENT_INT_MODE?has_content>
    .ctrLatchEventMode = DRV_CTR_EVENT_INTERRUPT_MODE, 
</#if>
<#if CONFIG_DRV_CTR_TRIGGER_INTERRUPT_SOURCE?has_content>
    .ctrTriggerInterruptSource = DRV_CTR_TRIGGER_INTERRUPT_SOURCE, 
</#if>
<#if CONFIG_DRV_CTR_M_0?has_content>
    .ctrCounter[0].M = DRV_CTR_M_0, 
</#if>
<#if CONFIG_DRV_CTR_N_0?has_content>
    .ctrCounter[0].N = DRV_CTR_N_0, 
</#if>
<#if CONFIG_DRV_CTR_LSB_0?has_content>
    .ctrCounter[0].LSB = DRV_CTR_LSB_0, 
</#if>
<#if CONFIG_DRV_CTR_Mode_0?has_content>
    .ctrCounter[0].Mode = DRV_CTR_MODE_0, 
</#if>
<#if CONFIG_DRV_CTR_M_1?has_content>
    .ctrCounter[1].M = DRV_CTR_M_1, 
</#if>
<#if CONFIG_DRV_CTR_N_1?has_content>
    .ctrCounter[1].N = DRV_CTR_N_1, 
</#if>
<#if CONFIG_DRV_CTR_LSB_1?has_content>
    .ctrCounter[1].LSB = DRV_CTR_LSB_1, 
</#if>
<#if CONFIG_DRV_CTR_Mode_1?has_content>
    .ctrCounter[1].Mode = DRV_CTR_MODE_1, 
</#if>
<#if CONFIG_DRV_CTR_USE_CASE?has_content>
<#if CONFIG_DRV_CTR_USE_CASE == "Wifi">
<#if CONFIG_DRV_CTR_COUNTER_SEL?has_content>
	.ctrLatch[0].ctrSel = DRV_CTR_COUNTER_SEL,
	.ctrLatch[1].ctrSel = DRV_CTR_COUNTER_SEL,
	.ctrLatch[2].ctrSel = DRV_CTR_COUNTER_SEL,
	.ctrLatch[3].ctrSel = DRV_CTR_COUNTER_SEL,
</#if>
<#if CONFIG_DRV_CTR_USE_CASE?has_content>
	.ctrLatch[0].trigSel = DRV_CTR_LATCH0_TRIG,
	.ctrLatch[1].trigSel = DRV_CTR_LATCH1_TRIG,
	.ctrLatch[2].trigSel = DRV_CTR_LATCH2_TRIG,
	.ctrLatch[3].trigSel = DRV_CTR_LATCH3_TRIG,
</#if>
<#if CONFIG_DRV_CTR_DIVIDER?has_content>	
	.ctrLatch[0].divider = DRV_CTR_DIVIDER,
	.ctrLatch[1].divider = DRV_CTR_DIVIDER,
	.ctrLatch[2].divider = DRV_CTR_DIVIDER,
	.ctrLatch[3].divider = DRV_CTR_DIVIDER,
</#if>
</#if>
<#if CONFIG_DRV_CTR_USE_CASE == "USB">
<#if CONFIG_DRV_CTR_COUNTER_SEL?has_content>
	.ctrLatch[0].ctrSel = DRV_CTR_COUNTER_SEL,
</#if>
<#if CONFIG_DRV_CTR_USE_CASE?has_content>
	.ctrLatch[0].trigSel = DRV_CTR_LATCH0_TRIG,
</#if>
<#if CONFIG_DRV_CTR_DIVIDER?has_content>	
	.ctrLatch[0].divider = DRV_CTR_DIVIDER,
</#if>	
</#if>
<#if CONFIG_DRV_CTR_USE_CASE == "GPIO">
<#if CONFIG_DRV_CTR_COUNTER_SEL?has_content>
	.ctrLatch[0].ctrSel = DRV_CTR_COUNTER_SEL,
</#if>
<#if CONFIG_DRV_CTR_USE_CASE?has_content>
	.ctrLatch[0].trigSel = DRV_CTR_LATCH0_TRIG,
</#if>
<#if CONFIG_DRV_CTR_DIVIDER?has_content>
	.ctrLatch[0].divider = DRV_CTR_DIVIDER,
</#if>
</#if>
</#if>
<#if CONFIG_DRV_CTR_TRIGGER_SOURCE?has_content>
	.ctrTrigger.trigSource = DRV_CTR_TRIGGER_SOURCE,
</#if>
<#if CONFIG_DRV_CTR_TRIGGER_PHASE?has_content>
	.ctrTrigger.phase = DRV_CTR_TRIGGER_PHASE,
</#if>	
<#if CONFIG_DRV_CTR_USE_CASE?has_content>
	.drvMode = DRIVER_MODE
</#if>	
};
</#if>
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
