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

.text
.align 2
    
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



/*******************************************************************************
 End of File
 */

