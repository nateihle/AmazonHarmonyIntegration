<#--
/*******************************************************************************
  CAN Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_can_int.s.ftl

  Summary:
    CAN driver interrupt handler templates.

  Description:
    The CAN device driver provides a simple interface to manage the CAN
    modules on Microchip microcontrollers and this module implements the
    interrupts.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END
-->
<#macro DRV_CAN_ASM_INT DRV_CAN_INSTANCE DRV_CAN_ID CAN_INTERRUPT_MODE DRV_CAN_INT_PRIORITY>
<#if DRV_CAN_ID == "CAN_ID_1">
<#assign CAN_ISR_VECTOR = CONFIG_INT_VECT_CAN1>
</#if>
<#if DRV_CAN_ID == "CAN_ID_2">
<#assign CAN_ISR_VECTOR = CONFIG_INT_VECT_CAN2>
</#if>
<#if DRV_CAN_ID == "CAN_ID_3">
<#assign CAN_ISR_VECTOR = CONFIG_INT_VECT_CAN3>
</#if>
<#if DRV_CAN_ID == "CAN_ID_4">
<#assign CAN_ISR_VECTOR = CONFIG_INT_VECT_CAN4>
</#if>
<#if DRV_CAN_INSTANCE == "0">
<#assign CAN_ISR_NAME = "DrvCANInstance0">
</#if>
<#if DRV_CAN_INSTANCE == "1">
<#assign CAN_ISR_NAME = "DrvCANInstance1">
</#if>
<#if DRV_CAN_INSTANCE == "2">
<#assign CAN_ISR_NAME = "DrvCANInstance2">
</#if>
<#if DRV_CAN_INSTANCE == "3">
<#assign CAN_ISR_NAME = "DrvCANInstance3">
</#if>
/* CAN Instance ${DRV_CAN_INSTANCE} Interrupt */
<@RTOS_ISR VECTOR = CAN_ISR_VECTOR NAME = CAN_ISR_NAME PRIORITY = DRV_CAN_INT_PRIORITY/>
</#macro>
<#if CONFIG_DRV_CAN_INST_IDX0 == true>
<#if CONFIG_DRV_CAN_INTERRUPT_MODE_ID0 == true>
<@DRV_CAN_ASM_INT DRV_CAN_INSTANCE="0" DRV_CAN_ID=CONFIG_DRV_CAN_PERIPHERAL_ID_IDX0 
CAN_INTERRUPT_MODE=CONFIG_DRV_CAN_INTERRUPT_MODE_ID0 DRV_CAN_INT_PRIORITY=CONFIG_DRV_CAN_INT_PRIORITY_IDX0/>
</#if>
</#if>
<#if CONFIG_DRV_CAN_INST_IDX1?has_content>
<#if CONFIG_DRV_CAN_INST_IDX1 == true>
<#if CONFIG_DRV_CAN_INTERRUPT_MODE_ID1 == true>
<@DRV_CAN_ASM_INT DRV_CAN_INSTANCE="1" DRV_CAN_ID=CONFIG_DRV_CAN_PERIPHERAL_ID_IDX1 
CAN_INTERRUPT_MODE=CONFIG_DRV_CAN_INTERRUPT_MODE_ID1 DRV_CAN_INT_PRIORITY=CONFIG_DRV_CAN_INT_PRIORITY_IDX1/>
</#if>
</#if>
</#if>
<#if CONFIG_DRV_CAN_INST_IDX2?has_content>
<#if CONFIG_DRV_CAN_INST_IDX2 == true>
<#if CONFIG_DRV_CAN_INTERRUPT_MODE_ID2 == true>
<@DRV_CAN_ASM_INT DRV_CAN_INSTANCE="2" DRV_CAN_ID=CONFIG_DRV_CAN_PERIPHERAL_ID_IDX2
CAN_INTERRUPT_MODE=CONFIG_DRV_CAN_INTERRUPT_MODE_ID2 DRV_CAN_INT_PRIORITY=CONFIG_DRV_CAN_INT_PRIORITY_IDX2/>
</#if>
</#if>
</#if>
<#if CONFIG_DRV_CAN_INST_IDX3?has_content>
<#if CONFIG_DRV_CAN_INST_IDX3 == true>
<#if CONFIG_DRV_CAN_INTERRUPT_MODE_ID3 == true>
<@DRV_CAN_ASM_INT DRV_CAN_INSTANCE="3" DRV_CAN_ID=CONFIG_DRV_CAN_PERIPHERAL_ID_IDX3
CAN_INTERRUPT_MODE=CONFIG_DRV_CAN_INTERRUPT_MODE_ID3 DRV_CAN_INT_PRIORITY=CONFIG_DRV_CAN_INT_PRIORITY_IDX3/>
</#if>
</#if>
</#if>
