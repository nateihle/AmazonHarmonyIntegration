<#--
/*******************************************************************************
  ctr Driver Interrupt Handler Template File

  File Name:
    drv_ctr_int.c

  Summary:
    This file contains source code necessary to handle CTR interrupts.

  Description:
    This file contains source code necessary to the CTR interrupt.  It
    implements the ISR function to handle CTR event and trigger interrupts.
 *******************************************************************************/

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
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_CTR_EVENT_ISR_VECTOR}, IPL${CONFIG_DRV_CTR_EVENT_INT_IPL}SOFT) _IntHandlerDrvCTREvent(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_CTR_EVENT_INT_IPL}AUTO), vector(${CONFIG_DRV_CTR_EVENT_ISR_VECTOR}))) IntHandlerDrvCTREvent_ISR( void );
</#if>
void IntHandlerDrvCTREvent(void)
</#if>
<#else>
void __ISR(${CONFIG_DRV_CTR_EVENT_ISR_VECTOR}, ipl${CONFIG_DRV_CTR_EVENT_INT_IPL}AUTO) _IntHandlerDrvCTREvent(void)
</#if>
{
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
   /* Call ThreadX context save.  */
   _tx_thread_context_save();
</#if>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_EnterNestableInterrupt();
</#if>
</#if>
<#if CONFIG_DRV_CTR_DRIVER_MODE == "DYNAMIC">
    DRV_CTR_EventISR(sysObj.drvctr0);
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
   /* Call ThreadX context restore.  */
   _tx_thread_context_restore();
</#if>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_LeaveNestableInterrupt();
</#if>
</#if>
}

<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_CTR_TRIGGER_ISR_VECTOR}, IPL${CONFIG_DRV_CTR_TRIGGER_INT_IPL}SOFT) _IntHandlerDrvCTRTrigger(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_CTR_TRIGGER_INT_IPL}AUTO), vector(${CONFIG_DRV_CTR_TRIGGER_ISR_VECTOR}))) IntHandlerDrvCTRTrigger_ISR( void );
</#if>
void IntHandlerDrvCTRTrigger(void)
</#if>
<#else>
void __ISR(${CONFIG_DRV_CTR_TRIGGER_ISR_VECTOR}, ipl${CONFIG_DRV_CTR_TRIGGER_INT_IPL}AUTO) _IntHandlerDrvCTRTrigger(void)
</#if>
{
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
   /* Call ThreadX context save.  */
   _tx_thread_context_save();
</#if>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_EnterNestableInterrupt();
</#if>
</#if>
<#if CONFIG_DRV_CTR_DRIVER_MODE == "DYNAMIC">
    DRV_CTR_TriggerISR(sysObj.drvctr0);
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
   /* Call ThreadX context restore.  */
   _tx_thread_context_restore();
</#if>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_LeaveNestableInterrupt();
</#if>
</#if>
}

<#--
/*******************************************************************************
 End of File
*/
-->
