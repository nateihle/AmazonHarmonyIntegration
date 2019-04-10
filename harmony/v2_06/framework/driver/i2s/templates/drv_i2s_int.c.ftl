<#--
/*******************************************************************************
  I2S Driver Interrupt Handler Template File

  File Name:
    drv_i2s_int.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, configuration bits, and allocates
    any necessary global system resources, such as the systemObjects structure
    that contains the object handles to all the MPLAB Harmony module objects in
    the system.
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
<#if CONFIG_DRV_I2S_INTERRUPT_MODE == true>
<#if CONFIG_DRV_I2S_INST_IDX0 == true>
<#if CONFIG_PIC32MX == true>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_I2S_ISR_VECTOR_IDX0}, ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX0}SOFT) _IntHandlerDrvI2SInstance0(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX0}AUTO), vector(${CONFIG_DRV_I2S_ISR_VECTOR_IDX0}))) IntHandlerDrvI2SInstance0_ISR( void );
</#if>
void IntHandlerDrvI2SInstance0(void)
</#if>
<#else>
void __ISR(${CONFIG_DRV_I2S_ISR_VECTOR_IDX0}, ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX0}AUTO) _IntHandlerDrvI2SInstance0(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S0);
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
</#if>
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
<#if CONFIG_USE_3RDPARTY_RTOS>

<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX0}, ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX0}SOFT) _IntHandlerDrvI2STxInstance0(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX0}AUTO), vector(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX0}))) IntHandlerDrvI2STxInstance0_ISR( void );
</#if>
void IntHandlerDrvI2STxInstance0(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX0}, ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX0}AUTO) _IntHandlerDrvI2STxInstance0(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S0);
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
void __ISR(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX0}, ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX0}SOFT) _IntHandlerDrvI2SRxInstance0(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX0}AUTO), vector(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX0}))) IntHandlerDrvI2SRxInstance0_ISR( void );
</#if>
void IntHandlerDrvI2SRxInstance0(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX0}, ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX0}AUTO) _IntHandlerDrvI2SRxInstance0(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S0);
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
void __ISR(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX0}, ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX0}SOFT) _IntHandlerDrvI2SFaultInstance0(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX0}AUTO), vector(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX0}))) IntHandlerDrvI2SFaultInstance0_ISR( void );
</#if>
void IntHandlerDrvI2SFaultInstance0(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX0}, ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX0}AUTO) _IntHandlerDrvI2SFaultInstance0(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S0);
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
</#if>
</#if>

<#if CONFIG_DRV_I2S_INST_IDX1 == true>
<#if CONFIG_PIC32MX == true>
<#if CONFIG_USE_3RDPARTY_RTOS>

<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_I2S_ISR_VECTOR_IDX1}, ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX1}SOFT) _IntHandlerDrvI2SInstance1(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX1}AUTO), vector(${CONFIG_DRV_I2S_ISR_VECTOR_IDX1}))) IntHandlerDrvI2SInstance1_ISR( void );
</#if>
void IntHandlerDrvI2SInstance1(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_ISR_VECTOR_IDX1}, ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX1}AUTO) _IntHandlerDrvI2SInstance1(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S1);
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
</#if>
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
<#if CONFIG_USE_3RDPARTY_RTOS>

<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX1}, ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX1}SOFT) _IntHandlerDrvI2STxInstance1(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX1}AUTO), vector(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX1}))) IntHandlerDrvI2STxInstance1_ISR( void );
</#if>
void IntHandlerDrvI2STxInstance1(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX1}, ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX1}AUTO) _IntHandlerDrvI2STxInstance1(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S1);
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
void __ISR(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX1}, ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX1}SOFT) _IntHandlerDrvI2SRxInstance1(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX1}AUTO), vector(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX1}))) IntHandlerDrvI2SRxInstance1_ISR( void );
</#if>
void IntHandlerDrvI2SRxInstance1(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX1}, ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX1}AUTO) _IntHandlerDrvI2SRxInstance1(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S1);
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
void __ISR(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX1}, ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX1}SOFT) _IntHandlerDrvI2SFaultInstance1(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX1}AUTO), vector(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX1}))) IntHandlerDrvI2SFaultInstance1_ISR( void );
</#if>
void IntHandlerDrvI2SFaultInstance1(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX1}, ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX1}AUTO) _IntHandlerDrvI2SFaultInstance1(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S1);
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
</#if>
</#if>

<#if CONFIG_DRV_I2S_INST_IDX2 == true>
<#if CONFIG_PIC32MX == true>
<#if CONFIG_USE_3RDPARTY_RTOS>

<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_I2S_ISR_VECTOR_IDX2}, ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX2}SOFT) _IntHandlerDrvI2SInstance2(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX2}AUTO), vector(${CONFIG_DRV_I2S_ISR_VECTOR_IDX2}))) IntHandlerDrvI2SInstance2_ISR( void );
</#if>
void IntHandlerDrvI2SInstance2(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_ISR_VECTOR_IDX2}, ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX2}AUTO) _IntHandlerDrvI2SInstance2(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S2);
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
</#if>
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
<#if CONFIG_USE_3RDPARTY_RTOS>

<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX2}, ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX2}SOFT) _IntHandlerDrvI2STxInstance2(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX2}AUTO), vector(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX2}))) IntHandlerDrvI2STxInstance2_ISR( void );
</#if>
void IntHandlerDrvI2STxInstance2(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX2}, ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX2}AUTO) _IntHandlerDrvI2STxInstance2(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S2);
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
void __ISR(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX2}, ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX2}SOFT) _IntHandlerDrvI2SRxInstance2(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX2}AUTO), vector(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX2}))) IntHandlerDrvI2SRxInstance2_ISR( void );
</#if>
void IntHandlerDrvI2SRxInstance2(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX2}, ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX2}AUTO) _IntHandlerDrvI2SRxInstance2(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S2);
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
void __ISR(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX2}, ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX2}SOFT) _IntHandlerDrvI2SFaultInstance2(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX2}AUTO), vector(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX2}))) IntHandlerDrvI2SFaultInstance2_ISR( void );
</#if>
void IntHandlerDrvI2SFaultInstance2(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX2}, ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX2}AUTO) _IntHandlerDrvI2SFaultInstance2(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S2);
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
</#if>
</#if>

<#if CONFIG_DRV_I2S_INST_IDX3 == true>
<#if CONFIG_PIC32MX == true>
<#if CONFIG_USE_3RDPARTY_RTOS>

<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_I2S_ISR_VECTOR_IDX3}, ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX3}SOFT) _IntHandlerDrvI2SInstance3(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX3}AUTO), vector(${CONFIG_DRV_I2S_ISR_VECTOR_IDX3}))) IntHandlerDrvI2SInstance3_ISR( void );
</#if>
void IntHandlerDrvI2SInstance3(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_ISR_VECTOR_IDX3}, ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX3}AUTO) _IntHandlerDrvI2SInstance3(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S3);
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
</#if>
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
<#if CONFIG_USE_3RDPARTY_RTOS>

<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX3}, ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX3}SOFT) _IntHandlerDrvI2STxInstance3(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX3}AUTO), vector(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX3}))) IntHandlerDrvI2STxInstance3_ISR( void );
</#if>
void IntHandlerDrvI2STxInstance3(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX3}, ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX3}AUTO) _IntHandlerDrvI2STxInstance3(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S3);
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
void __ISR(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX3}, ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX3}SOFT) _IntHandlerDrvI2SRxInstance3(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX3}AUTO), vector(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX3}))) IntHandlerDrvI2SRxInstance3_ISR( void );
</#if>
void IntHandlerDrvI2SRxInstance3(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX3}, ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX3}AUTO) _IntHandlerDrvI2SRxInstance3(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S3);
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
void __ISR(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX3}, ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX3}SOFT) _IntHandlerDrvI2SFaultInstance3(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX3}AUTO), vector(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX3}))) IntHandlerDrvI2SFaultInstance3_ISR( void );
</#if>
void IntHandlerDrvI2SFaultInstance3(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX3}, ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX3}AUTO) _IntHandlerDrvI2SFaultInstance3(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S3);
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
</#if>
</#if>

<#if CONFIG_DRV_I2S_INST_IDX4 == true>
<#if CONFIG_PIC32MX == true>
<#if CONFIG_USE_3RDPARTY_RTOS>

<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_I2S_ISR_VECTOR_IDX4}, ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX4}SOFT) _IntHandlerDrvI2SInstance4(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX4}AUTO), vector(${CONFIG_DRV_I2S_ISR_VECTOR_IDX4}))) IntHandlerDrvI2SInstance4_ISR( void );
</#if>
void IntHandlerDrvI2SInstance4(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_ISR_VECTOR_IDX4}, ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX4}AUTO) _IntHandlerDrvI2SInstance4(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S4);
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
</#if>
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
<#if CONFIG_USE_3RDPARTY_RTOS>

<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX4}, ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX4}SOFT) _IntHandlerDrvI2STxInstance4(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX4}AUTO), vector(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX4}))) IntHandlerDrvI2STxInstance4_ISR( void );
</#if>
void IntHandlerDrvI2STxInstance4(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX4}, ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX4}AUTO) _IntHandlerDrvI2STxInstance4(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S4);
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
void __ISR(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX4}, ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX4}SOFT) _IntHandlerDrvI2SRxInstance4(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX4}AUTO), vector(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX4}))) IntHandlerDrvI2SRxInstance4_ISR( void );
</#if>
void IntHandlerDrvI2SRxInstance4(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX4}, ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX4}AUTO) _IntHandlerDrvI2SRxInstance4(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S4);
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
void __ISR(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX4}, ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX4}SOFT) _IntHandlerDrvI2SFaultInstance4(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX4}AUTO), vector(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX4}))) IntHandlerDrvI2SFaultInstance4_ISR( void );
</#if>
void IntHandlerDrvI2SFaultInstance4(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX4}, ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX4}AUTO) _IntHandlerDrvI2SFaultInstance4(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S4);
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
</#if>
</#if>

<#if CONFIG_DRV_I2S_INST_IDX5 == true>
<#if CONFIG_PIC32MX == true>
<#if CONFIG_USE_3RDPARTY_RTOS>

<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_I2S_ISR_VECTOR_IDX5}, ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX5}SOFT) _IntHandlerDrvI2SInstance5(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX5}AUTO), vector(${CONFIG_DRV_I2S_ISR_VECTOR_IDX5}))) IntHandlerDrvI2SInstance5_ISR( void );
</#if>
void IntHandlerDrvI2SInstance5(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_ISR_VECTOR_IDX5}, ipl${CONFIG_DRV_I2S_INT_PRIO_NUM_IDX5}AUTO) _IntHandlerDrvI2SInstance5(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S5);
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
</#if>
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
<#if CONFIG_USE_3RDPARTY_RTOS>

<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX5}, ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX5}SOFT) _IntHandlerDrvI2STxInstance5(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX5}AUTO), vector(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX5}))) IntHandlerDrvI2STxInstance5_ISR( void );
</#if>
void IntHandlerDrvI2STxInstance5(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_TX_ISR_VECTOR_IDX5}, ipl${CONFIG_DRV_I2S_TX_INT_PRIO_NUM_IDX5}AUTO) _IntHandlerDrvI2STxInstance5(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S5);
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
void __ISR(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX5}, ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX5}SOFT) _IntHandlerDrvI2SRxInstance5(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX5}AUTO), vector(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX5}))) IntHandlerDrvI2SRxInstance5_ISR( void );
</#if>
void IntHandlerDrvI2SRxInstance5(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_RX_ISR_VECTOR_IDX5}, ipl${CONFIG_DRV_I2S_RX_INT_PRIO_NUM_IDX5}AUTO) _IntHandlerDrvI2SRxInstance5(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S5);
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
void __ISR(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX5}, ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX5}SOFT) _IntHandlerDrvI2SFaultInstance5(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX5}AUTO), vector(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX5}))) IntHandlerDrvI2SFaultInstance5_ISR( void );
</#if>
void IntHandlerDrvI2SFaultInstance5(void)
</#if>

<#else>
void __ISR(${CONFIG_DRV_I2S_ERR_ISR_VECTOR_IDX5}, ipl${CONFIG_DRV_I2S_ERR_INT_PRIO_NUM_IDX5}AUTO) _IntHandlerDrvI2SFaultInstance5(void)
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
    DRV_I2S_Tasks(sysObj.drvI2S5);
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
</#if>
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->
</#if>
