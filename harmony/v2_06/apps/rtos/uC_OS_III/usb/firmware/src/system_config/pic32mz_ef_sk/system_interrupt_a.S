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
/*
*********************************************************************************************************
*                                           INCLUDES
*********************************************************************************************************
*/
#include <xc.h>


#include "os_cpu_a.inc"

#define BSP_TICK_COUNT ((200000000 / 2) / 1000)
 
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



/* TMR Instance 0 Interrupt */

   .extern  IntHandlerDrvTmrInstance0

   .section	.vector_4,code, keep
   .equ     __vector_dispatch_4, IntVectorDrvTmrInstance0
   .global  __vector_dispatch_4
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
 
   .ent IntVectorDrvTmrInstance0
IntVectorDrvTmrInstance0:
    saveCPU_CONTEXT

    la    $8,  OSIntNestingCtr                /* See if OSIntNesting == 0, save Tasks Stack pointer   */
    lbu   $9,  0($8)                           
    bne   $0,  $9, 0f
    nop   

    la    $10, OSTCBCurPtr                    /* Save the current task's stack pointer                */
    lw    $11, 0($10)
    sw    $29, 0($11)                         /* OSTCBCurPtr->StkPtr = SP;                            */
    
0:   
    la    $8, OSIntEnter                      /* Call OSIntEnter                                      */
    jalr  $8
    nop

    la    $8, IntHandlerDrvTmrInstance0
    jalr  $8
    nop

    la    $8,  OSIntExit                      /* Call OSIntExit()                                     */
    jalr  $8
    nop

    restoreCPU_CONTEXT

    .end IntVectorDrvTmrInstance0



/* USB Device Interrupt */

   .extern  IntHandlerUSBInstance0

   .section	.vector_132,code, keep
   .equ     __vector_dispatch_132, IntVectorUSBInstance0
   .global  __vector_dispatch_132
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
 
   .ent IntVectorUSBInstance0
IntVectorUSBInstance0:
    saveCPU_CONTEXT

    la    $8,  OSIntNestingCtr                /* See if OSIntNesting == 0, save Tasks Stack pointer   */
    lbu   $9,  0($8)                           
    bne   $0,  $9, 0f
    nop   

    la    $10, OSTCBCurPtr                    /* Save the current task's stack pointer                */
    lw    $11, 0($10)
    sw    $29, 0($11)                         /* OSTCBCurPtr->StkPtr = SP;                            */
    
0:   
    la    $8, OSIntEnter                      /* Call OSIntEnter                                      */
    jalr  $8
    nop

    la    $8, IntHandlerUSBInstance0
    jalr  $8
    nop

    la    $8,  OSIntExit                      /* Call OSIntExit()                                     */
    jalr  $8
    nop

    restoreCPU_CONTEXT

    .end IntVectorUSBInstance0


   .extern  IntHandlerUSBInstance0_USBDMA

   .section	.vector_133,code, keep
   .equ     __vector_dispatch_133, IntVectorUSBInstance0_USBDMA
   .global  __vector_dispatch_133
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
 
   .ent IntVectorUSBInstance0_USBDMA
IntVectorUSBInstance0_USBDMA:
    saveCPU_CONTEXT

    la    $8,  OSIntNestingCtr                /* See if OSIntNesting == 0, save Tasks Stack pointer   */
    lbu   $9,  0($8)                           
    bne   $0,  $9, 0f
    nop   

    la    $10, OSTCBCurPtr                    /* Save the current task's stack pointer                */
    lw    $11, 0($10)
    sw    $29, 0($11)                         /* OSTCBCurPtr->StkPtr = SP;                            */
    
0:   
    la    $8, OSIntEnter                      /* Call OSIntEnter                                      */
    jalr  $8
    nop

    la    $8, IntHandlerUSBInstance0_USBDMA
    jalr  $8
    nop

    la    $8,  OSIntExit                      /* Call OSIntExit()                                     */
    jalr  $8
    nop

    restoreCPU_CONTEXT

    .end IntVectorUSBInstance0_USBDMA


/*******************************************************************************
 End of File
 */

