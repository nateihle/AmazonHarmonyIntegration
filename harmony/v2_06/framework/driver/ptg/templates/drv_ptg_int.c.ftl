<#--
/*******************************************************************************
  ADC Driver Interrupt Handler Template File

  File Name:
    drv_adc_int.c

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
<#if CONFIG_DRV_PTG_SINGLE_STEP_INT == true>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_PTG_SSTEP_ISR_VECTOR}, IPL${CONFIG_DRV_PTG_SSTEP_INT_IPL}SOFT) _IntHandlerDrvPtgSStep(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_PTG_SSTEP_INT_IPL}AUTO), vector(${CONFIG_DRV_PTG_SSTEP_ISR_VECTOR}))) IntHandlerDrvPtgSStep_ISR( void );
</#if>
void IntHandlerDrvPtgSStep(void)
</#if>
<#else>
void __ISR(${CONFIG_DRV_PTG_SSTEP_ISR_VECTOR}, ipl${CONFIG_DRV_PTG_SSTEP_INT_IPL}AUTO) _IntHandlerDrvPtgSStep(void)
</#if>	
{
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_EnterNestableInterrupt();
</#if>
</#if>
<#if CONFIG_DRV_PTG_DRIVER_MODE == "STATIC">
    /* Clear PTG Single Step Interrupt Flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, ${CONFIG_DRV_PTG_SSTEP_INTERRUPT_SOURCE});
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_LeaveNestableInterrupt();
</#if>
</#if>
}
</#if>
<#if CONFIG_DRV_PTG_WDT_INT == true>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_PTG_WDT_ISR_VECTOR}, IPL${CONFIG_DRV_PTG_WDT_INT_IPL}SOFT) _IntHandlerDrvPtgWdt(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_PTG_WDT_INT_IPL}AUTO), vector(${CONFIG_DRV_PTG_WDT_ISR_VECTOR}))) IntHandlerDrvPtgWdt_ISR( void );
</#if>
void IntHandlerDrvPtgWdt(void)
</#if>
<#else>
void __ISR(${CONFIG_DRV_PTG_WDT_ISR_VECTOR}, ipl${CONFIG_DRV_PTG_WDT_INT_IPL}AUTO) _IntHandlerDrvPtgWdt(void)
</#if>	
{
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_EnterNestableInterrupt();
</#if>
</#if>
<#if CONFIG_DRV_PTG_DRIVER_MODE == "STATIC">
    /* Clear PTG WDT Interrupt Flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, ${CONFIG_DRV_PTG_WDT_INTERRUPT_SOURCE});
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_LeaveNestableInterrupt();
</#if>
</#if>
}
</#if>
<#if CONFIG_DRV_PTG_IRQ0_INT == true>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_PTG_IRQ0_ISR_VECTOR}, IPL${CONFIG_DRV_PTG_IRQ0_INT_IPL}SOFT) _IntHandlerDrvPtgIrq0(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_PTG_IRQ0_INT_IPL}AUTO), vector(${CONFIG_DRV_PTG_IRQ0_ISR_VECTOR}))) IntHandlerDrvPtgIrq0_ISR( void );
</#if>
void IntHandlerDrvPtgIrq0(void)
</#if>
<#else>

void __ISR(${CONFIG_DRV_PTG_IRQ0_ISR_VECTOR}, ipl${CONFIG_DRV_PTG_IRQ0_INT_IPL}AUTO) _IntHandlerDrvPtgIrq0(void)
</#if>	
{
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_EnterNestableInterrupt();
</#if>
</#if>
<#if CONFIG_DRV_PTG_DRIVER_MODE == "STATIC">
    /* Clear PTG IRQ0 Interrupt Flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, ${CONFIG_DRV_PTG_IRQ0_INTERRUPT_SOURCE});
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_LeaveNestableInterrupt();
</#if>
</#if>
}
</#if>
<#if CONFIG_DRV_PTG_IRQ1_INT == true>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_PTG_IRQ1_ISR_VECTOR}, IPL${CONFIG_DRV_PTG_IRQ1_INT_IPL}SOFT) _IntHandlerDrvPtgIrq1(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_PTG_IRQ1_INT_IPL}AUTO), vector(${CONFIG_DRV_PTG_IRQ1_ISR_VECTOR}))) IntHandlerDrvPtgIrq1_ISR( void );
</#if>
void IntHandlerDrvPtgIrq1(void)
</#if>
<#else>
void __ISR(${CONFIG_DRV_PTG_IRQ1_ISR_VECTOR}, ipl${CONFIG_DRV_PTG_IRQ1_INT_IPL}AUTO) _IntHandlerDrvPtgIrq1(void)
</#if>	
{
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_EnterNestableInterrupt();
</#if>
</#if>
<#if CONFIG_DRV_PTG_DRIVER_MODE == "STATIC">
    /* Clear PTG IRQ1 Interrupt Flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, ${CONFIG_DRV_PTG_IRQ1_INTERRUPT_SOURCE});
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_LeaveNestableInterrupt();
</#if>
</#if>
}
</#if>
<#if CONFIG_DRV_PTG_IRQ2_INT == true>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_PTG_IRQ2_ISR_VECTOR}, IPL${CONFIG_DRV_PTG_IRQ2_INT_IPL}SOFT) _IntHandlerDrvPtgIrq2(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_PTG_IRQ2_INT_IPL}AUTO), vector(${CONFIG_DRV_PTG_IRQ2_ISR_VECTOR}))) IntHandlerDrvPtgIrq2_ISR( void );
</#if>
void IntHandlerDrvPtgIrq2(void)
</#if>
<#else>
void __ISR(${CONFIG_DRV_PTG_IRQ2_ISR_VECTOR}, ipl${CONFIG_DRV_PTG_IRQ2_INT_IPL}AUTO) _IntHandlerDrvPtgIrq2(void)
</#if>	
{
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_EnterNestableInterrupt();
</#if>
</#if>
<#if CONFIG_DRV_PTG_DRIVER_MODE == "STATIC">
    /* Clear PTG IRQ2 Interrupt Flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, ${CONFIG_DRV_PTG_IRQ2_INTERRUPT_SOURCE});
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_LeaveNestableInterrupt();
</#if>
</#if>
}
</#if>
<#if CONFIG_DRV_PTG_IRQ3_INT == true>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void __ISR(${CONFIG_DRV_PTG_IRQ3_ISR_VECTOR}, IPL${CONFIG_DRV_PTG_IRQ3_INT_IPL}SOFT) _IntHandlerDrvPtgIrq3(void)
<#else>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
void __attribute__( (interrupt(ipl${CONFIG_DRV_PTG_IRQ3_INT_IPL}AUTO), vector(${CONFIG_DRV_PTG_IRQ3_ISR_VECTOR}))) IntHandlerDrvPtgIrq3_ISR( void );
</#if>
void IntHandlerDrvPtgIrq3(void)
</#if>
<#else>
void __ISR(${CONFIG_DRV_PTG_IRQ3_ISR_VECTOR}, ipl${CONFIG_DRV_PTG_IRQ3_INT_IPL}AUTO) _IntHandlerDrvPtgIrq3(void)
</#if>	
{
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_EnterNestableInterrupt();
</#if>
</#if>
<#if CONFIG_DRV_PTG_DRIVER_MODE == "STATIC">
    /* Clear PTG IRQ3 Interrupt Flag */
    PLIB_INT_SourceFlagClear(INT_ID_0, ${CONFIG_DRV_PTG_IRQ3_INTERRUPT_SOURCE});
</#if>
<#if CONFIG_USE_3RDPARTY_RTOS>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
    OS_LeaveNestableInterrupt();
</#if>
</#if>
}
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->
