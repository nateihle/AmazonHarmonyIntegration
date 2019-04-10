<#--
/*******************************************************************************
  PTG Driver Interrupt Handler Template File

  File Name:
    drv_ptg_int.s

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
/* PTG Single Step Interrupt */
 <#if CONFIG_DRV_PTG_SINGLE_STEP_INT == true>
<@RTOS_ISR VECTOR = CONFIG_INT_VECT_PTGSTEP NAME = "DrvPtgSStep" PRIORITY = CONFIG_DRV_PTG_SS_INTERRUPT_PRIORITY/>
 </#if>

/* PTG WDT Interrupt */
 <#if CONFIG_DRV_PTG_WDT_INT == true>
<@RTOS_ISR VECTOR = CONFIG_INT_VECT_PTGWDT NAME = "DrvPtgWdt" PRIORITY = CONFIG_DRV_PTG_WDT_INTERRUPT_PRIORITY/>
 </#if>
 
/* PTG IRQ0 Interrupt */
 <#if CONFIG_DRV_PTG_IRQ0_INT == true>
<@RTOS_ISR VECTOR = CONFIG_INT_VECT_PTG0TR0 NAME = "DrvPtgIrq0" PRIORITY = CONFIG_DRV_PTG_IRQ0_INTERRUPT_PRIORITY/>
 </#if>

 /* PTG IRQ1 Interrupt */
 <#if CONFIG_DRV_PTG_IRQ1_INT == true>
<@RTOS_ISR VECTOR = CONFIG_INT_VECT_PTG0TR1 NAME = "DrvPtgIrq1" PRIORITY = CONFIG_DRV_PTG_IRQ1_INTERRUPT_PRIORITY/>
 </#if>

 /* PTG IRQ2 Interrupt */
 <#if CONFIG_DRV_PTG_IRQ2_INT == true>
<@RTOS_ISR VECTOR = CONFIG_INT_VECT_PTG0TR2 NAME = "DrvPtgIrq2" PRIORITY = CONFIG_DRV_PTG_IRQ2_INTERRUPT_PRIORITY/>
 </#if>

 /* PTG IRQ3 Interrupt */
 <#if CONFIG_DRV_PTG_IRQ3_INT == true>
<@RTOS_ISR VECTOR = CONFIG_INT_VECT_PTG0TR3 NAME = "DrvPtgIrq3" PRIORITY = CONFIG_DRV_PTG_IRQ3_INTERRUPT_PRIORITY/>
 </#if>

 <#--
/*******************************************************************************
 End of File
*/
-->
