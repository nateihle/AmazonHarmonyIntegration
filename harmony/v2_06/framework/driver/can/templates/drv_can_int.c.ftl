<#--
/*******************************************************************************
  CAN Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_can_int.c.ftl

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
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTOCULAR PURPOSE.
IN NO EVENT SHALL MOCROCHIP OR ITS LOCENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STROCT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVOCES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END
-->
<#macro DRV_CAN_STATIC_FUNCTIONS
    DRV_INSTANCE
    CAN_INT_SRC
    CAN_INT_ISR
    CAN_INT_PRINUM>
<#if CONFIG_USE_3RDPARTY_RTOS>
  <#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CAN_INT_ISR}, IPL${CAN_INT_PRINUM}SOFT) _IntHandlerDrvCANInstance${DRV_INSTANCE}(void)
  <#else>
    <#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CAN_INT_PRINUM}AUTO), vector(${CAN_INT_ISR}))) IntHandlerDrvCANInstance${DRV_INSTANCE}_ISR( void );
    </#if>
void IntHandlerDrvCANInstance${DRV_INSTANCE}(void)
  </#if>
<#else>
void __ISR(${CAN_INT_ISR}, IPL${CAN_INT_PRINUM}AUTO) _IntHandlerDrvCANInstance${DRV_INSTANCE}(void)
</#if>
{
<#if CONFIG_USE_3RDPARTY_RTOS>
  <#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_EnterNestableInterrupt();
  </#if>
</#if>
    PLIB_INT_SourceFlagClear(INT_ID_0, ${CAN_INT_SRC});
<#if CONFIG_USE_3RDPARTY_RTOS>
  <#if CONFIG_3RDPARTY_RTOS_USED == "embOS">

    OS_LeaveNestableInterrupt();
  </#if>
</#if>
}
</#macro>

<#assign CAN_IDS = ["CAN_ID_0", "CAN_ID_1", "CAN_ID_2", "CAN_ID_3"]>

<#if CONFIG_DRV_CAN_INSTANCES_NUMBER?number gt 0>
<#if CONFIG_DRV_CAN_INTERRUPT_MODE_ID0 == true>
<@DRV_CAN_STATIC_FUNCTIONS
DRV_INSTANCE="0"
CAN_INT_SRC=CONFIG_DRV_CAN_INTERRUPT_SOURCE_IDX0
CAN_INT_ISR=CONFIG_DRV_CAN_ISR_VECTOR_IDX0
CAN_INT_PRINUM=CONFIG_DRV_CAN_INT_PRIO_NUM_IDX0/>
</#if>
</#if>

<#if CONFIG_DRV_CAN_INSTANCES_NUMBER?number gt 1>
<#if CONFIG_DRV_CAN_INTERRUPT_MODE_ID1 == true>
<@DRV_CAN_STATIC_FUNCTIONS
DRV_INSTANCE="1"
CAN_INT_SRC=CONFIG_DRV_CAN_INTERRUPT_SOURCE_IDX1
CAN_INT_ISR=CONFIG_DRV_CAN_ISR_VECTOR_IDX1
CAN_INT_PRINUM=CONFIG_DRV_CAN_INT_PRIO_NUM_IDX1/>
</#if>
</#if>

<#if CONFIG_DRV_CAN_INSTANCES_NUMBER?number gt 2>
<#if CONFIG_DRV_CAN_INTERRUPT_MODE_ID2 == true>
<@DRV_CAN_STATIC_FUNCTIONS
DRV_INSTANCE="2"
CAN_INT_SRC=CONFIG_DRV_CAN_INTERRUPT_SOURCE_IDX2
CAN_INT_ISR=CONFIG_DRV_CAN_ISR_VECTOR_IDX2
CAN_INT_PRINUM=CONFIG_DRV_CAN_INT_PRIO_NUM_IDX2/>
</#if>
</#if>

<#if CONFIG_DRV_CAN_INSTANCES_NUMBER?number gt 3>
<#if CONFIG_DRV_CAN_INTERRUPT_MODE_ID3 == true>
<@DRV_CAN_STATIC_FUNCTIONS
DRV_INSTANCE="3"
CAN_INT_SRC=CONFIG_DRV_CAN_INTERRUPT_SOURCE_IDX3
CAN_INT_ISR=CONFIG_DRV_CAN_ISR_VECTOR_IDX3
CAN_INT_PRINUM=CONFIG_DRV_CAN_INT_PRIO_NUM_IDX3/>
</#if>
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
