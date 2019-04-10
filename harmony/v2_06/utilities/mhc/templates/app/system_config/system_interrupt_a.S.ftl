/*******************************************************************************
 System Interrupt Source File

  File Name:
    sys_interrupt_a.S

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the 
    interrupt sub-system.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END
<#include "/utilities/mhc/templates/freemarker_functions.ftl">
/*
*********************************************************************************************************
*                                           INCLUDES
*********************************************************************************************************
*/
<#include "/utilities/mhc/templates/app/system_config/rtos_ftl_macros.ftl">
<#if (CONFIG_3RDPARTY_RTOS_USED != "embOS")>
#include <xc.h>
</#if>
<#if LIST_SYSTEM_INTERRUPT_A_S_INCLUDES?has_content>
<@mhc_expand_list list=LIST_SYSTEM_INTERRUPT_A_S_INCLUDES/>
</#if>

<#if (CONFIG_3RDPARTY_RTOS_USED == "embOS")&& (CONFIG_USE_SYS_INT == true)>
<#-- the below Interrupt Priority names are used in system/driver specific FTLs and passed via RTOS_ISR macro (rtos_ftl_macros.ftl) -->
#define OS_INT_PRIORITY_1  0x0400
#define OS_INT_PRIORITY_2  0x0800
#define OS_INT_PRIORITY_3  0x0C00
#define OS_INT_PRIORITY_4  0x1000
#define OS_INT_PRIORITY_5  0x1400
#define OS_INT_PRIORITY_6  0x1800
#define OS_INT_PRIORITY_7  0x1C00

#define _C0_SR              $12       // Coprocessor 0, status register 
#define _C0_EPC             $14       // C0 exception (saved) link pointer

# ####################################################################
# #
# #     MACROS to simplify assembler code
# #
# ####################################################################
.macro OS_CALL_ISR funcptr, prio

    mfc0      $k0,_C0_SR
    mfc0      $k1,_C0_EPC

    # Make new stack frame
    addiu     $sp,$sp,-92

    # Save EPC and Status register on stack
    sw        $k0,84($sp)
    sw        $k1,88($sp)

    # Set new status register value, interrupts are still disabled
    addiu     $k1, $0, 0xFFFFC3FF
    and       $k0, $k1, $k0
    ori       $k0, $k0, \prio
    mtc0      $k0,_C0_SR

    # Save all scratch registers
    # turn off assembler warnings for $at                  
    .set noat                   
    sw        $at,0($sp)        
    # turn on assembler warnings for $at                  
    .set at                     
    sw        $v0,4($sp)        
    sw        $v1,8($sp)        
    sw        $a0,12($sp)        
    sw        $a1,16($sp)        
    sw        $a2,20($sp)        
    sw        $a3,24($sp)        
    sw        $t0,28($sp)        
    sw        $t1,32($sp)        
    sw        $t2,36($sp)        
    sw        $t3,40($sp)        
    sw        $t4,44($sp)        
    sw        $t5,48($sp)        
    sw        $t6,52($sp)        
    sw        $t7,56($sp)        
    sw        $t8,60($sp)        
    sw        $t9,64($sp)        
    sw        $fp,68($sp)        
    sw        $ra,72($sp)        
    mflo      $v0                
    mfhi      $v1                
    sw        $v0,76($sp)       
    sw        $v1,80($sp)

    # If (RegionCnt == 0) {
    jal       OS_GetRegionCnt
    nop
    bne       $v0, $0, __\funcptr

    # OS_ISR_StackFrameAddr = sp + 84}
    addiu     $k0, $sp, 84
    sw        $k0, OS_ISR_StackFrameAddr

    __\funcptr:
    # Call the C function to handle the interrupt
    jal       \funcptr
    nop    

    # Disable interrupt
    di

    # Restore all scratch register 
    # turn off assembler warnings for $at                  
    .set noat        
    lw        $at,0($sp)      
    # turn on assembler warnings for $at                  
    .set at    
    lw        $a0,12($sp)        
    lw        $a1,16($sp)        
    lw        $a2,20($sp)        
    lw        $a3,24($sp)        
    lw        $t0,28($sp)        
    lw        $t1,32($sp)        
    lw        $t2,36($sp)        
    lw        $t3,40($sp)        
    lw        $t4,44($sp)        
    lw        $t5,48($sp)        
    lw        $t6,52($sp)        
    lw        $t7,56($sp)        
    lw        $t8,60($sp)        
    lw        $t9,64($sp)        
    lw        $fp,68($sp)        
    lw        $ra,72($sp)        
    lw        $v0,76($sp)       
    lw        $v1,80($sp)
    mtlo      $v0                
    mthi      $v1                
    lw        $v0,4($sp)        
    lw        $v1,8($sp)        

    # Get saved EPC and status register from stack
    lw        $k0,84($sp)
    lw        $k1,88($sp)

    # Correct stack pointer
    addiu     $sp,$sp,92

    # Restore EPC and status register
    mtc0      $k0,_C0_SR
    ehb
    mtc0      $k1,_C0_EPC
    ehb
    eret  
.endm

# ####################################################################
# #
# #     CODE segment
# #
# ####################################################################

<#if CONFIG_PIC32MZ == true || CONFIG_PIC32WK == true>
.section .text, code
<#if (CONFIG_MIPSMODEMZ == "MICROMIPS")>
.set    micromips
</#if>
<#else>
.text
.align 2
</#if>
    
# #
# #       Wrapper function
# #       This function saves and restores all necessary registers.
# #       Interrupts are not enabled (MPLAB compiler does this in 
# #       interrupt function prolog).
# #
.global  OS_SysTick_ISR    
.extern  OS_SysTick

OS_SysTick_ISR:
    OS_CALL_ISR  OS_Systick OS_INT_PRIORITY_1
</#if>

<#if (CONFIG_3RDPARTY_RTOS_USED == "uC/OS-III")&& (CONFIG_USE_SYS_INT == true)>
#include "os_cpu_a.inc"

#define BSP_TICK_COUNT ((${CONFIG_SYS_CLK_FREQ} / 2) / ${CONFIG_UCOSIII_CFG_TICK_RATE_HZ})
 
.global  BSP_TickHandler
.extern OSIntEnter
.extern OSIntExit
.extern OSIntNestingCtr
.extern OSTCBCurPtr
.extern IFS0CLR
.extern OSTickIntHandler

/*
*********************************************************************************************************
*                                          BSP_TickHandler()
*
* Description : Reloads the appropriate timer source for the next period.  The period is based on the
*               tick rate specified by the user and rate the timer is running at.
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : os_cpu_a.S, which contains the RTOS tick handler function, this function calls out to 
*               have the timer reloaded for the next period.  The user must provide this functionality.
*
* Note(s)     : none.
*********************************************************************************************************
*/

.section .text,code
.set noreorder
.set noat
.set nomips16
#if ( __mips_micromips == 1 )  
.set micromips
#else
.set nomicromips
#endif


.ent BSP_TickHandler

BSP_TickHandler:
    mtc0  $0, $9, 0                         /* clear core timer register                              */
    li    $8, BSP_TICK_COUNT                /* count value based on sys clock and tick rate specified */
    mtc0  $8, $11                           /* reload the core timer period register                  */
    ehb

    li    $8, 1                             /* Clear core timer interrupt                            */
    la    $9, IFS0CLR
    sw    $8, ($9)

    jr    $31
    nop

.end BSP_TickHandler

/*
*********************************************************************************************************
*                                          Core Timer Interrupt Handler()
*
* Description : Calls the RTOS Tick function to maintain the RTOS time services.  Currently implemented 
*               using the core timer as the tick source.  The user does not have to use the timer source
*               and is free to change it.  However, the interrupt must implemented so no context save 
*               occurs before calling OSTickIntHandler().  The appropriate assembler symbols must also be 
*               changed to locate this routine in the correct interrupt vector.  
*
* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : Hardware Interrupt
*
* Note(s)     : none.
*********************************************************************************************************
*/
   .section	.vector_0,code, keep
   .equ     __vector_dispatch_0, CoreTimerInterruptVector
   .global  __vector_dispatch_0
#if ( __mips_micromips == 1 )  
   .set micromips
#else
   .set     nomicromips
#endif
   .set     noreorder
   .set     nomips16
   .set     noat

   .ent  CoreTimerInterruptVector
CoreTimerInterruptVector:
   la    $26, OSTickIntHandler
   jr    $26
   nop

   .end CoreTimerInterruptVector
</#if>

<#if (CONFIG_3RDPARTY_RTOS_USED == "uC/OS-II")&& (CONFIG_USE_SYS_INT == true)>

#define BSP_TICK_COUNT ((${CONFIG_SYS_CLK_FREQ} / 2) / ${CONFIG_UCOSII_OS_TICKS_PER_SEC})
 
.global  BSP_TickISR_Handler
<#if (CONFIG_3RDPARTY_RTOS_USED == "uC/OS-II") && (CONFIG_USE_SYS_INT == true)>
.extern OSIntEnter
.extern OSIntExit
.extern OSIntNestingCtr
.extern OSTCBCurPtr
</#if>
.extern IFS0CLR
.extern CoreTimerIntHandler

/*
*********************************************************************************************************
*                                          BSP_TickHandler()
*
* Description : Reloads the appropriate timer source for the next period.  The period is based on the

*               tick rate specified by the user and rate the timer is running at.
*
* Argument(s) : none
*

* Return(s)   : none
*
* Caller(s)   : os_cpu_a.S, which contains the RTOS tick handler function, this function calls out to 

*               have the timer reloaded for the next period.  The user must provide this functionality.
*

* Note(s)     : none.
*********************************************************************************************************
*/

.section .text,code
.set noreorder
.set noat
.set nomips16
#if ( __mips_micromips == 1 )  
.set micromips
#else
.set nomicromips
#endif
.ent BSP_TickISR_Handler

BSP_TickISR_Handler:
    mtc0  $0, $9, 0                         /* clear core timer register                              */
    li    $8, BSP_TICK_COUNT                /* count value based on sys clock and tick rate specified */
    mtc0  $8, $11                           /* reload the core timer period register                  */
    ehb

    li    $8, 1                             /* Clear core timer interrupt                            */
    la    $9, IFS0CLR
    sw    $8, ($9)

    jr    $31
    nop

.end BSP_TickISR_Handler

</#if>
<#if (CONFIG_3RDPARTY_RTOS_USED == "uC/OS-II") && (CONFIG_USE_SYS_INT == true)>
/*
*********************************************************************************************************
*                                          Core Timer Interrupt Handler()
*
* Description : Calls the RTOS Tick function to maintain the RTOS time services.  Currently implemented 

*               using the core timer as the tick source.  The user does not have to use the timer source
*               and is free to change it.  However, the interrupt must implemented so no context save 
*               occurs before calling OSTickIntHandler().  The appropriate assembler symbols must also be 
*               changed to locate this routine in the correct interrupt vector.  
*

* Argument(s) : none
*
* Return(s)   : none
*
* Caller(s)   : Hardware Interrupt

*
* Note(s)     : none.
*********************************************************************************************************
*/
   .section	.vector_0,code, keep
   .equ     __vector_dispatch_0, CoreTimerInterruptVector
   .global  __vector_dispatch_0
#if ( __mips_micromips == 1 )  
   .set micromips
#else
   .set nomicromips
#endif
   .set     noreorder
   .set     nomips16
   .set     noat

   .ent  CoreTimerInterruptVector
CoreTimerInterruptVector:
   la    $26, CoreTimerIntHandler
   jr    $26
   nop

   .end CoreTimerInterruptVector
</#if>
<#if (CONFIG_3RDPARTY_RTOS_USED == "FreeRTOS") || (CONFIG_3RDPARTY_RTOS_USED == "OpenRTOS_V8.x.x")>
#include "ISR_Support.h"
</#if>

<#if CONFIG_USE_SYS_PORTS_CN_INTERRUPT == true>
<#include "/framework/system/ports/templates/sys_ports_int.s.ftl">
</#if>
<#if CONFIG_USE_DRV_ADC == true>
<#include "/framework/driver/adc/templates/drv_adc_int.s.ftl">
</#if>
<#if CONFIG_USE_EXT_INT?has_content><#if CONFIG_USE_EXT_INT == true>
<#include "/framework/system/int/templates/ext_int_static_int.s.ftl">
</#if> </#if>
<#if CONFIG_USE_DRV_CAN == true>
<#include "/framework/driver/can/templates/drv_can_int.s.ftl">
</#if>
<#if CONFIG_USE_DRV_CTR == true>
<#include "/framework/driver/ctr/templates/drv_ctr_int.s.ftl">
</#if>
<#if CONFIG_USE_DRV_PTG == true>
<#include "/framework/driver/ptg/templates/drv_ptg_int.s.ftl">
</#if>
<#if CONFIG_USE_DRV_TMR == true>
<#include "/framework/driver/tmr/templates/drv_tmr_int.s.ftl">
</#if>
<#if CONFIG_USE_DRV_USART == true>
<#include "/framework/driver/usart/templates/drv_usart_int.s.ftl">
</#if>
<#if CONFIG_DRV_SPI_USE_DRIVER == true>
<#include "/framework/driver/spi/config/drv_spi_int.s.ftl"> 
</#if>
<#if CONFIG_USE_DRV_SQI == true>
<#include "/framework/driver/sqi/config/drv_sqi_int.s.ftl">
</#if>
<#if CONFIG_USE_DRV_NVM == true>
<#include "/framework/driver/nvm/config/drv_nvm_int.s.ftl">
</#if>
<#if CONFIG_USE_DRV_IC == true>
<#include "/framework/driver/ic/templates/drv_ic_int.s.ftl">
</#if>
<#if CONFIG_USE_DRV_CMP == true>
<#include "/framework/driver/cmp/templates/drv_cmp_int.s.ftl">
</#if>
<#if CONFIG_USE_DRV_OC == true>
<#include "/framework/driver/oc/templates/drv_oc_int.s.ftl">
</#if>
<#if CONFIG_USE_SYS_DMA == true> 
<#include "/framework/system/dma/templates/sys_dma_int.s.ftl">
</#if>
<#if CONFIG_USE_DRV_I2C == true>
<#include "/framework/driver/i2c/templates/drv_i2c_int.s.ftl">
</#if>
<#if CONFIG_USE_DRV_I2S == true>
<#include "/framework/driver/i2s/templates/drv_i2s_int.s.ftl">
</#if>
<#if CONFIG_USE_DRV_RTCC == true>
<#include "/framework/driver/rtcc/templates/drv_rtcc_int.s.ftl">
</#if>
<#if CONFIG_USE_USB_STACK == true>
<#include "/framework/usb/templates/usb_interrupt.s.ftl">
</#if>
<#if CONFIG_USE_TCPIP_STACK == true>
<#include "/framework/tcpip/config/tcpip_mac_int.s.ftl">
</#if>
<#include "/framework/net/templates/system_interrupt_a.s.ftl">
/*******************************************************************************
 End of File
 */

