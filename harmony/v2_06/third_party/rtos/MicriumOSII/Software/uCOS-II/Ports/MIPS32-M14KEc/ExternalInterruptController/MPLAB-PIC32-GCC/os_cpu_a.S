/*
*********************************************************************************************************
*                                               uC/OS-II
*                                         The Real-Time Kernel
*
*                             (c) Copyright 2007, Micrium, Inc., Weston, FL
*                                           All Rights Reserved
*
*                                              MIPS32 M14KEc
*
*                                                MPLAB
*
* File    : os_cpu_a.S
* Version : v1.04
*********************************************************************************************************
*/

    .section .text,code
    .set noreorder
    .set nomips16
#if ( __mips_micromips == 1 )  
    .set micromips
#else
    .set nomicromips
#endif 
    .set noat

/*
*********************************************************************************************************
*                                          PUBLIC FUNCTIONS
*********************************************************************************************************
*/

    .global  OSStartHighRdy
    .global  OSIntCtxSw
    .global  _general_exception_context
    .global  CoreTimerIntHandler

/*
*********************************************************************************************************
*                           CONSTANTS USED TO ACCESS TASK CONTEXT STACK
*********************************************************************************************************
*/

.equ    STK_OFFSET_SR,      4
.equ    STK_OFFSET_EPC,     STK_OFFSET_SR      + 4
.equ    STK_OFFSET_LO,      STK_OFFSET_EPC     + 4
.equ    STK_OFFSET_DSPCTRL, STK_OFFSET_EPC     + 4
.equ    STK_OFFSET_AC0LO,   STK_OFFSET_DSPCTRL + 4
.equ    STK_OFFSET_AC0HI,   STK_OFFSET_AC0LO   + 4
.equ    STK_OFFSET_AC1LO,   STK_OFFSET_AC0HI   + 4
.equ    STK_OFFSET_AC1HI,   STK_OFFSET_AC1LO   + 4
.equ    STK_OFFSET_AC2LO,   STK_OFFSET_AC1HI   + 4
.equ    STK_OFFSET_AC2HI,   STK_OFFSET_AC2LO   + 4
.equ    STK_OFFSET_AC3LO,   STK_OFFSET_AC2HI   + 4
.equ    STK_OFFSET_AC3HI,   STK_OFFSET_AC3LO   + 4
 #if defined (__mips_hard_float)    
.equ    STK_OFFSET_FPR0,    STK_OFFSET_AC3HI   + 4
.equ    STK_OFFSET_FPR1,    STK_OFFSET_FPR0    + 8
.equ    STK_OFFSET_FPR2,    STK_OFFSET_FPR1    + 8 
.equ    STK_OFFSET_FPR3,    STK_OFFSET_FPR2    + 8 
.equ    STK_OFFSET_FPR4,    STK_OFFSET_FPR3    + 8 
.equ    STK_OFFSET_FPR5,    STK_OFFSET_FPR4    + 8 
.equ    STK_OFFSET_FPR6,    STK_OFFSET_FPR5    + 8 
.equ    STK_OFFSET_FPR7,    STK_OFFSET_FPR6    + 8 
.equ    STK_OFFSET_FPR8,    STK_OFFSET_FPR7    + 8 
.equ    STK_OFFSET_FPR9,    STK_OFFSET_FPR8    + 8 
.equ    STK_OFFSET_FPR10,   STK_OFFSET_FPR9    + 8 
.equ    STK_OFFSET_FPR11,   STK_OFFSET_FPR10   + 8 
.equ    STK_OFFSET_FPR12,   STK_OFFSET_FPR11   + 8 
.equ    STK_OFFSET_FPR13,   STK_OFFSET_FPR12   + 8 
.equ    STK_OFFSET_FPR14,   STK_OFFSET_FPR13   + 8
.equ    STK_OFFSET_FPR15,   STK_OFFSET_FPR14   + 8 
.equ    STK_OFFSET_FPR16,   STK_OFFSET_FPR15   + 8 
.equ    STK_OFFSET_FPR17,   STK_OFFSET_FPR16   + 8 
.equ    STK_OFFSET_FPR18,   STK_OFFSET_FPR17   + 8  
.equ    STK_OFFSET_FPR19,   STK_OFFSET_FPR18   + 8 
.equ    STK_OFFSET_FPR20,   STK_OFFSET_FPR19   + 8 
.equ    STK_OFFSET_FPR21,   STK_OFFSET_FPR20   + 8 
.equ    STK_OFFSET_FPR22,   STK_OFFSET_FPR21   + 8  
.equ    STK_OFFSET_FPR23,   STK_OFFSET_FPR22   + 8 
.equ    STK_OFFSET_FPR24,   STK_OFFSET_FPR23   + 8 
.equ    STK_OFFSET_FPR25,   STK_OFFSET_FPR24   + 8 
.equ    STK_OFFSET_FPR26,   STK_OFFSET_FPR25   + 8  
.equ    STK_OFFSET_FPR27,   STK_OFFSET_FPR26   + 8 
.equ    STK_OFFSET_FPR28,   STK_OFFSET_FPR27   + 8 
.equ    STK_OFFSET_FPR29,   STK_OFFSET_FPR28   + 8 
.equ    STK_OFFSET_FPR30,   STK_OFFSET_FPR29   + 8      
.equ    STK_OFFSET_FPR31,   STK_OFFSET_FPR30   + 8
.equ    STK_OFFSET_FCSR,    STK_OFFSET_FPR31   + 8
.equ    STK_OFFSET_GPR1,    STK_OFFSET_FCSR    + 4 
#else
.equ    STK_OFFSET_GPR1,    STK_OFFSET_AC3HI   + 4
#endif 
.equ    STK_OFFSET_GPR2,    STK_OFFSET_GPR1    + 4
.equ    STK_OFFSET_GPR3,    STK_OFFSET_GPR2    + 4
.equ    STK_OFFSET_GPR4,    STK_OFFSET_GPR3    + 4
.equ    STK_OFFSET_GPR5,    STK_OFFSET_GPR4    + 4
.equ    STK_OFFSET_GPR6,    STK_OFFSET_GPR5    + 4
.equ    STK_OFFSET_GPR7,    STK_OFFSET_GPR6    + 4
.equ    STK_OFFSET_GPR8,    STK_OFFSET_GPR7    + 4
.equ    STK_OFFSET_GPR9,    STK_OFFSET_GPR8    + 4
.equ    STK_OFFSET_GPR10,   STK_OFFSET_GPR9    + 4
.equ    STK_OFFSET_GPR11,   STK_OFFSET_GPR10   + 4
.equ    STK_OFFSET_GPR12,   STK_OFFSET_GPR11   + 4
.equ    STK_OFFSET_GPR13,   STK_OFFSET_GPR12   + 4
.equ    STK_OFFSET_GPR14,   STK_OFFSET_GPR13   + 4
.equ    STK_OFFSET_GPR15,   STK_OFFSET_GPR14   + 4
.equ    STK_OFFSET_GPR16,   STK_OFFSET_GPR15   + 4
.equ    STK_OFFSET_GPR17,   STK_OFFSET_GPR16   + 4
.equ    STK_OFFSET_GPR18,   STK_OFFSET_GPR17   + 4
.equ    STK_OFFSET_GPR19,   STK_OFFSET_GPR18   + 4
.equ    STK_OFFSET_GPR20,   STK_OFFSET_GPR19   + 4
.equ    STK_OFFSET_GPR21,   STK_OFFSET_GPR20   + 4
.equ    STK_OFFSET_GPR22,   STK_OFFSET_GPR21   + 4
.equ    STK_OFFSET_GPR23,   STK_OFFSET_GPR22   + 4
.equ    STK_OFFSET_GPR24,   STK_OFFSET_GPR23   + 4
.equ    STK_OFFSET_GPR25,   STK_OFFSET_GPR24   + 4
.equ    STK_OFFSET_GPR26,   STK_OFFSET_GPR25   + 4
.equ    STK_OFFSET_GPR27,   STK_OFFSET_GPR26   + 4
.equ    STK_OFFSET_GPR28,   STK_OFFSET_GPR27   + 4
.equ    STK_OFFSET_GPR30,   STK_OFFSET_GPR28   + 4
.equ    STK_OFFSET_GPR31,   STK_OFFSET_GPR30   + 4 
.equ    STK_CTX_SIZE,       STK_OFFSET_GPR31   + 4

/*
*********************************************************************************************************
*                                           OSStartHighRdy()
*
* Description: Starts the highest priority task that is available to run.  OSStartHighRdy() MUST:
*
*              a) Call OSTaskSwHook()
*              b) Set OSRunning to TRUE
*              c) Switch to the highest priority task.
*
*              The stack frame of the task to resume is assumed to look as follows:
*
*              OSTCBHighRdy->OSTCBStkPtr + 0x00    Free Entry                    (LOW Memory)
*                                        + 0x04    Status Register
*                                        + 0x08    EPC
*                                        + 0x0C    Special Purpose LO Register
*                                        + 0x10    Special Purpose HI Register
*                                        + 0x14    GPR[1]
*                                        + 0x18    GPR[2]
*                                        + 0x1C    GPR[3]
*                                        + 0x20    GPR[4]
*                                               |
*                                               |
*                                              \ /
*                                               V
*                                        + 0x80    GPR[28]
*                                        + 0x84    GPR[30]
*                                        + 0x88    GPR[31]                       (HIGH Memory)
*                              
* Note(s): 1) OSTaskStkInit(), which is responsible for initializing each task's stack, sets bit 0 of the
*             entry corresponding to the Status register.  Thus, interrupts will be enabled when each
*             task first runs.
*********************************************************************************************************
*/

    .ent OSStartHighRdy
OSStartHighRdy:

    la    $8,  OSTaskSwHook                    /* Call OSTaskSwHook()                                  */
    jalr  $8
    nop

    addi  $8,  $0, 1                           /* Indicate that the OS is running                      */
    la    $9,  OSRunning
    sb    $8,  0($9)

    la    $8,  OSTCBHighRdy                    /* Update the current TCB                               */
    lw    $9,  0($8) 
    lw    $29, 0($9)                           /* Load the new task's stack pointer                    */

    lw    $8,  STK_OFFSET_SR($29)              /* Restore the Status register                          */
    mtc0  $8,  $12, 0

    lw    $8,  STK_OFFSET_EPC($29)             /* Restore the EPC                                      */
    mtc0  $8,  $14, 0

    lw    $8, STK_OFFSET_DSPCTRL($29)         /* Restore the contents of the DSP Control Register      */
    wrdsp $8
     
    lw    $8, STK_OFFSET_AC0LO($29)           /* Restore the contents of the ac0 LO and HI registers   */
    lw    $9, STK_OFFSET_AC0HI($29)
    mtlo  $8, $ac0
    mthi  $9, $ac0

    lw    $8, STK_OFFSET_AC1LO($29)           /* Restore the contents of the ac1 LO and HI registers   */
    lw    $9, STK_OFFSET_AC1HI($29)
    mtlo  $8, $ac1
    mthi  $9, $ac1
    
    lw    $8, STK_OFFSET_AC2LO($29)           /* Restore the contents of the ac2 LO and HI registers   */
    lw    $9, STK_OFFSET_AC2HI($29)
    mtlo  $8, $ac2
    mthi  $9, $ac2
    
    lw    $8, STK_OFFSET_AC3LO($29)           /* Restore the contents of the ac3 LO and HI registers   */
    lw    $9, STK_OFFSET_AC3HI($29)
    mtlo  $8, $ac3
    mthi  $9, $ac3

#if defined (__mips_hard_float) 
					       
    LDC1  $f0, STK_OFFSET_FPR0($29)
    LDC1  $f1, STK_OFFSET_FPR1($29)
    LDC1  $f2, STK_OFFSET_FPR2($29)
    LDC1  $f3, STK_OFFSET_FPR3($29)
    LDC1  $f4, STK_OFFSET_FPR4($29)
    LDC1  $f5, STK_OFFSET_FPR5($29)
    LDC1  $f6, STK_OFFSET_FPR6($29)
    LDC1  $f7, STK_OFFSET_FPR7($29)
    LDC1  $f8, STK_OFFSET_FPR8($29)
    LDC1  $f9, STK_OFFSET_FPR9($29)
    LDC1  $f10, STK_OFFSET_FPR10($29)
    LDC1  $f11, STK_OFFSET_FPR11($29)
    LDC1  $f12, STK_OFFSET_FPR12($29)
    LDC1  $f13, STK_OFFSET_FPR13($29)
    LDC1  $f14, STK_OFFSET_FPR14($29)
    LDC1  $f15, STK_OFFSET_FPR15($29)
    LDC1  $f16, STK_OFFSET_FPR16($29)
    LDC1  $f17, STK_OFFSET_FPR17($29)
    LDC1  $f18, STK_OFFSET_FPR18($29)
    LDC1  $f19, STK_OFFSET_FPR19($29)
    LDC1  $f20, STK_OFFSET_FPR20($29)
    LDC1  $f21, STK_OFFSET_FPR21($29)
    LDC1  $f22, STK_OFFSET_FPR22($29)
    LDC1  $f23, STK_OFFSET_FPR23($29)
    LDC1  $f24, STK_OFFSET_FPR24($29)
    LDC1  $f25, STK_OFFSET_FPR25($29)
    LDC1  $f26, STK_OFFSET_FPR26($29)
    LDC1  $f27, STK_OFFSET_FPR27($29)  
    LDC1  $f28, STK_OFFSET_FPR28($29)
    LDC1  $f29, STK_OFFSET_FPR29($29)
    LDC1  $f30, STK_OFFSET_FPR30($29)
    LDC1  $f31, STK_OFFSET_FPR31($29)
    lw    $8,STK_OFFSET_FCSR($29)    
    CTC1  $8,$f31
    
#endif

    lw    $31, STK_OFFSET_GPR31($29)           /* Restore the General Purpose Registers                */
    lw    $30, STK_OFFSET_GPR30($29) 
    lw    $28, STK_OFFSET_GPR28($29)
    lw    $27, STK_OFFSET_GPR27($29) 
    lw    $26, STK_OFFSET_GPR26($29) 
    lw    $25, STK_OFFSET_GPR25($29) 
    lw    $24, STK_OFFSET_GPR24($29) 
    lw    $23, STK_OFFSET_GPR23($29) 
    lw    $22, STK_OFFSET_GPR22($29) 
    lw    $21, STK_OFFSET_GPR21($29) 
    lw    $20, STK_OFFSET_GPR20($29) 
    lw    $19, STK_OFFSET_GPR19($29) 
    lw    $18, STK_OFFSET_GPR18($29) 
    lw    $17, STK_OFFSET_GPR17($29) 
    lw    $16, STK_OFFSET_GPR16($29) 
    lw    $15, STK_OFFSET_GPR15($29) 
    lw    $14, STK_OFFSET_GPR14($29) 
    lw    $13, STK_OFFSET_GPR13($29) 
    lw    $12, STK_OFFSET_GPR12($29) 
    lw    $11, STK_OFFSET_GPR11($29) 
    lw    $10, STK_OFFSET_GPR10($29) 
    lw    $9,  STK_OFFSET_GPR9($29)  
    lw    $8,  STK_OFFSET_GPR8($29)  
    lw    $7,  STK_OFFSET_GPR7($29)  
    lw    $6,  STK_OFFSET_GPR6($29)  
    lw    $5,  STK_OFFSET_GPR5($29)  
    lw    $4,  STK_OFFSET_GPR4($29)  
    lw    $3,  STK_OFFSET_GPR3($29) 
    lw    $2,  STK_OFFSET_GPR2($29)  
    lw    $1,  STK_OFFSET_GPR1($29) 

    jr    $31                                  /* Resume execution in the new task                     */
    addi  $29, $29, STK_CTX_SIZE               /* Adjust the stack pointer                             */   

    .end OSStartHighRdy

/*
*********************************************************************************************************
*                                             OSIntCtxSw()
*
* Description: This function is used to perform a context switch following an ISR.
*
*              OSIntCtxSw() implements the following pseudo-code:
*
*                  OSTaskSwHook();
*                  OSPrioCur = OSPrioHighRdy;
*                  OSTCBCur  = OSTCBHighRdy;
*                  SP        = OSTCBHighRdy->OSTCBStkPtr;
*                  Restore the Status register and the EPC to their prior states;
*                  Restore the LO and HI registers;
*                  Restore each of the general purpose registers;
*                  Adjust the stack pointer;
*                  Execute an eret instruction to begin executing the new task;
*
*              Upon entry, the registers of the task being suspended have already been saved onto that
*              task's stack and the SP for the task has been saved in its OS_TCB by the ISR.
*
*              The stack frame of the task to resume is assumed to look as follows:
*
*              OSTCBHighRdy->OSTCBStkPtr + 0x00    Free Entry                    (LOW Memory)
*                                        + 0x04    Status Register
*                                        + 0x08    EPC
*                                        + 0x0C    Special Purpose LO Register
*                                        + 0x10    Special Purpose HI Register
*                                        + 0x14    GPR[1]
*                                        + 0x18    GPR[2]
*                                        + 0x1C    GPR[3]
*                                        + 0x20    GPR[4]
*                                               |
*                                               |
*                                              \ /
*                                               V
*                                        + 0x80    GPR[28]
*                                        + 0x84    GPR[30]
*                                        + 0x88    GPR[31]                       (HIGH Memory)
*********************************************************************************************************
*/            

    .ent OSIntCtxSw
OSIntCtxSw:

    la    $8,  OSTaskSwHook                    /* Call OSTaskSwHook()                                  */
    jalr  $8
    nop

    la    $8,  OSPrioHighRdy                   /* Update the current priority                          */
    lbu   $9,  0($8) 
    la    $10, OSPrioCur
    sb    $9,  0($10)

    la    $8,  OSTCBHighRdy                    /* Update the current TCB                               */
    lw    $9,  0($8)
    la    $10, OSTCBCur
    sw    $9,  0($10)

    lw    $29, 0($9)                           /* Load the new task's stack pointer                    */

    lw    $8,  STK_OFFSET_SR($29)              /* Restore the Status register                          */
    mtc0  $8,  $12, 0 

    lw    $8,  STK_OFFSET_EPC($29)             /* Restore the EPC                                      */
    mtc0  $8,  $14, 0 
    lw    $8, STK_OFFSET_DSPCTRL($29)         /* Restore the contents of the DSP Control Register      */
    wrdsp $8
     
    lw    $8, STK_OFFSET_AC0LO($29)           /* Restore the contents of the ac0 LO and HI registers   */
    lw    $9, STK_OFFSET_AC0HI($29)
    mtlo  $8, $ac0
    mthi  $9, $ac0

    lw    $8, STK_OFFSET_AC1LO($29)           /* Restore the contents of the ac1 LO and HI registers   */
    lw    $9, STK_OFFSET_AC1HI($29)
    mtlo  $8, $ac1
    mthi  $9, $ac1
    
    lw    $8, STK_OFFSET_AC2LO($29)           /* Restore the contents of the ac2 LO and HI registers   */
    lw    $9, STK_OFFSET_AC2HI($29)
    mtlo  $8, $ac2
    mthi  $9, $ac2
    
    lw    $8, STK_OFFSET_AC3LO($29)           /* Restore the contents of the ac3 LO and HI registers   */
    lw    $9, STK_OFFSET_AC3HI($29)
    mtlo  $8, $ac3
    mthi  $9, $ac3

#if defined (__mips_hard_float) 
					       
    LDC1  $f0, STK_OFFSET_FPR0($29)
    LDC1  $f1, STK_OFFSET_FPR1($29)
    LDC1  $f2, STK_OFFSET_FPR2($29)
    LDC1  $f3, STK_OFFSET_FPR3($29)
    LDC1  $f4, STK_OFFSET_FPR4($29)
    LDC1  $f5, STK_OFFSET_FPR5($29)
    LDC1  $f6, STK_OFFSET_FPR6($29)
    LDC1  $f7, STK_OFFSET_FPR7($29)
    LDC1  $f8, STK_OFFSET_FPR8($29)
    LDC1  $f9, STK_OFFSET_FPR9($29)
    LDC1  $f10, STK_OFFSET_FPR10($29)
    LDC1  $f11, STK_OFFSET_FPR11($29)
    LDC1  $f12, STK_OFFSET_FPR12($29)
    LDC1  $f13, STK_OFFSET_FPR13($29)
    LDC1  $f14, STK_OFFSET_FPR14($29)
    LDC1  $f15, STK_OFFSET_FPR15($29)
    LDC1  $f16, STK_OFFSET_FPR16($29)
    LDC1  $f17, STK_OFFSET_FPR17($29)
    LDC1  $f18, STK_OFFSET_FPR18($29)
    LDC1  $f19, STK_OFFSET_FPR19($29)
    LDC1  $f20, STK_OFFSET_FPR20($29)
    LDC1  $f21, STK_OFFSET_FPR21($29)
    LDC1  $f22, STK_OFFSET_FPR22($29)
    LDC1  $f23, STK_OFFSET_FPR23($29)
    LDC1  $f24, STK_OFFSET_FPR24($29)
    LDC1  $f25, STK_OFFSET_FPR25($29)
    LDC1  $f26, STK_OFFSET_FPR26($29)
    LDC1  $f27, STK_OFFSET_FPR27($29)  
    LDC1  $f28, STK_OFFSET_FPR28($29)
    LDC1  $f29, STK_OFFSET_FPR29($29)
    LDC1  $f30, STK_OFFSET_FPR30($29)
    LDC1  $f31, STK_OFFSET_FPR31($29)
    lw    $8,STK_OFFSET_FCSR($29)    
    CTC1  $8,$f31
    
#endif


    lw    $31, STK_OFFSET_GPR31($29)           /* Restore the General Purpose Registers                */
    lw    $30, STK_OFFSET_GPR30($29)
    lw    $28, STK_OFFSET_GPR28($29)
    lw    $27, STK_OFFSET_GPR27($29)
    lw    $26, STK_OFFSET_GPR26($29)
    lw    $25, STK_OFFSET_GPR25($29)
    lw    $24, STK_OFFSET_GPR24($29)
    lw    $23, STK_OFFSET_GPR23($29)
    lw    $22, STK_OFFSET_GPR22($29)
    lw    $21, STK_OFFSET_GPR21($29)
    lw    $20, STK_OFFSET_GPR20($29)
    lw    $19, STK_OFFSET_GPR19($29)
    lw    $18, STK_OFFSET_GPR18($29)
    lw    $17, STK_OFFSET_GPR17($29)
    lw    $16, STK_OFFSET_GPR16($29)
    lw    $15, STK_OFFSET_GPR15($29)
    lw    $14, STK_OFFSET_GPR14($29)
    lw    $13, STK_OFFSET_GPR13($29)
    lw    $12, STK_OFFSET_GPR12($29)
    lw    $11, STK_OFFSET_GPR11($29)
    lw    $10, STK_OFFSET_GPR10($29)
    lw    $9,  STK_OFFSET_GPR9($29)
    lw    $8,  STK_OFFSET_GPR8($29)
    lw    $7,  STK_OFFSET_GPR7($29)
    lw    $6,  STK_OFFSET_GPR6($29)
    lw    $5,  STK_OFFSET_GPR5($29)
    lw    $4,  STK_OFFSET_GPR4($29)
    lw    $3,  STK_OFFSET_GPR3($29)
    lw    $2,  STK_OFFSET_GPR2($29)
    lw    $1,  STK_OFFSET_GPR1($29)

    addi  $29, $29, STK_CTX_SIZE               /* Adjust the stack pointer                             */   

    eret                                       /* Resume execution in new task                         */

    .end OSIntCtxSw



/*
*********************************************************************************************************
*                                               _general_exception_context
*
* Description: Performs a Context switch from a task.  This function is ALWAYS called with interrupts
*              DISABLED.
*
*              _general_exception_context() implements the following pseudo-code:
*
*                  Adjust the stack pointer;
*                  Save each of the general purpose registers;
*                  Save the LO and HI registers;
*                  Save the EPC and the Status register;
*                  Verify that a syscall exception (corresponding to OS_TASK_SW) occurred;
*                  OSTCBCur->OSTCBStkPtr = SP;
*                  OSTaskSwHook();
*                  OSPrioCur = OSPrioHighRdy;
*                  OSTCBCur  = OSTCBHighRdy;
*                  SP        = OSTCBHighRdy->OSTCBStkPtr;
*                  Restore the Status register and the EPC to their prior states;
*                  Restore the LO and HI registers;
*                  Restore each of the general purpose registers;
*                  Adjust the stack pointer;
*                  Execute an eret instruction to begin executing the new task;
*
*              The stack frame of the task to suspend will look as follows when OSCtxSw() is done:
*
*              OSTCBHighRdy->OSTCBStkPtr + 0x00    Free Entry                    (LOW Memory)
*                                        + 0x04    Status Register
*                                        + 0x08    EPC
*                                        + 0x0C    Special Purpose LO Register
*                                        + 0x10    Special Purpose HI Register
*                                        + 0x14    GPR[1]
*                                        + 0x18    GPR[2]
*                                        + 0x1C    GPR[3]
*                                        + 0x20    GPR[4]
*                                               |
*                                               |
*                                              \ /
*                                               V
*                                        + 0x80    GPR[28]
*                                        + 0x84    GPR[30]
*                                        + 0x88    GPR[31]                       (HIGH Memory)
*              
*              The stack frame of the task to resume looks as follows:
*
*              OSTCBHighRdy->OSTCBStkPtr + 0x00    Free Entry                    (LOW Memory)
*                                        + 0x04    Status Register
*                                        + 0x08    EPC
*                                        + 0x0C    Special Purpose LO Register
*                                        + 0x10    Special Purpose HI Register
*                                        + 0x14    GPR[1]
*                                        + 0x18    GPR[2]
*                                        + 0x1C    GPR[3]
*                                        + 0x20    GPR[4]
*                                               |
*                                               |
*                                              \ /
*                                               V
*                                        + 0x80    GPR[28]
*                                        + 0x84    GPR[30]
*                                        + 0x88    GPR[31]                       (HIGH Memory)
*********************************************************************************************************
*/

    .ent _general_exception_context
_general_exception_context:

    addi  $29, $29, -STK_CTX_SIZE              /* Adjust the stack pointer                             */                      

    sw    $1,  STK_OFFSET_GPR1($29)            /* Save the General Pupose Registers                    */
    sw    $2,  STK_OFFSET_GPR2($29)
    sw    $3,  STK_OFFSET_GPR3($29)
    sw    $4,  STK_OFFSET_GPR4($29)
    sw    $5,  STK_OFFSET_GPR5($29)
    sw    $6,  STK_OFFSET_GPR6($29)
    sw    $7,  STK_OFFSET_GPR7($29)
    sw    $8,  STK_OFFSET_GPR8($29)
    sw    $9,  STK_OFFSET_GPR9($29)
    sw    $10, STK_OFFSET_GPR10($29)
    sw    $11, STK_OFFSET_GPR11($29)
    sw    $12, STK_OFFSET_GPR12($29)
    sw    $13, STK_OFFSET_GPR13($29)
    sw    $14, STK_OFFSET_GPR14($29)
    sw    $15, STK_OFFSET_GPR15($29)
    sw    $16, STK_OFFSET_GPR16($29)
    sw    $17, STK_OFFSET_GPR17($29)
    sw    $18, STK_OFFSET_GPR18($29)
    sw    $19, STK_OFFSET_GPR19($29)
    sw    $20, STK_OFFSET_GPR20($29)
    sw    $21, STK_OFFSET_GPR21($29)
    sw    $22, STK_OFFSET_GPR22($29)
    sw    $23, STK_OFFSET_GPR23($29)
    sw    $24, STK_OFFSET_GPR24($29)
    sw    $25, STK_OFFSET_GPR25($29)
    sw    $26, STK_OFFSET_GPR26($29)
    sw    $27, STK_OFFSET_GPR27($29)
    sw    $28, STK_OFFSET_GPR28($29)
    sw    $30, STK_OFFSET_GPR30($29)
    sw    $31, STK_OFFSET_GPR31($29)

#if defined (__mips_hard_float)  
  
    SDC1  $f0, STK_OFFSET_FPR0($29)
    SDC1  $f1, STK_OFFSET_FPR1($29)
    SDC1  $f2, STK_OFFSET_FPR2($29)
    SDC1  $f3, STK_OFFSET_FPR3($29)
    SDC1  $f4, STK_OFFSET_FPR4($29)
    SDC1  $f5, STK_OFFSET_FPR5($29)
    SDC1  $f6, STK_OFFSET_FPR6($29)
    SDC1  $f7, STK_OFFSET_FPR7($29)
    SDC1  $f8, STK_OFFSET_FPR8($29)
    SDC1  $f9, STK_OFFSET_FPR9($29)
    SDC1  $f10, STK_OFFSET_FPR10($29)
    SDC1  $f11, STK_OFFSET_FPR11($29)
    SDC1  $f12, STK_OFFSET_FPR12($29)
    SDC1  $f13, STK_OFFSET_FPR13($29)
    SDC1  $f14, STK_OFFSET_FPR14($29)
    SDC1  $f15, STK_OFFSET_FPR15($29)
    SDC1  $f16, STK_OFFSET_FPR16($29)
    SDC1  $f17, STK_OFFSET_FPR17($29)
    SDC1  $f18, STK_OFFSET_FPR18($29)
    SDC1  $f19, STK_OFFSET_FPR19($29)
    SDC1  $f20, STK_OFFSET_FPR20($29)
    SDC1  $f21, STK_OFFSET_FPR21($29)
    SDC1  $f22, STK_OFFSET_FPR22($29)
    SDC1  $f23, STK_OFFSET_FPR23($29)
    SDC1  $f24, STK_OFFSET_FPR24($29)
    SDC1  $f25, STK_OFFSET_FPR25($29)
    SDC1  $f26, STK_OFFSET_FPR26($29)
    SDC1  $f27, STK_OFFSET_FPR27($29)  
    SDC1  $f28, STK_OFFSET_FPR28($29)
    SDC1  $f29, STK_OFFSET_FPR29($29)
    SDC1  $f30, STK_OFFSET_FPR30($29)
    SDC1  $f31, STK_OFFSET_FPR31($29)
    CFC1  $8,$f31
    sw    $8,STK_OFFSET_FCSR($29)
#endif    
					       /* Save the contents of the ac 0-3 registers         */
    mflo  $8,  $ac0
    mfhi  $9,  $ac0
    sw    $8,  STK_OFFSET_AC0LO($29)
    sw    $9,  STK_OFFSET_AC0HI($29)
    
    mflo  $8,  $ac1
    mfhi  $9,  $ac1
    sw    $8,  STK_OFFSET_AC1LO($29)
    sw    $9,  STK_OFFSET_AC1HI($29)
    
    mflo  $8,  $ac2
    mfhi  $9,  $ac2
    sw    $8,  STK_OFFSET_AC2LO($29)
    sw    $9,  STK_OFFSET_AC2HI($29)

    mflo  $8,  $ac3
    mfhi  $9,  $ac3
    sw    $8,  STK_OFFSET_AC3LO($29)
    sw    $9,  STK_OFFSET_AC3HI($29) 

    rddsp $8
    sw    $8,  STK_OFFSET_DSPCTRL($29)         /* Save the DSP Control Register                        */

    mfc0  $8,  $14, 0                          /* Save the EPC                                         */
    addi  $8,  $8,  4                          /* EPC will reference the instruction following syscall */
    sw    $8,  STK_OFFSET_EPC($29)

    mfc0  $8,  $12, 0                          /* Save the Status register                             */
    sw    $8,  STK_OFFSET_SR($29)
                              
    ori   $8,  $0,  0x007C                     /* Switch context only if a syscall exception occurred  */
    mfc0  $9,  $13, 0
    and   $9,  $9,  $8
    ori   $10, $0,  0x0020
    beq   $9,  $10, SAVE_SP
    nop              

    la    $8,  _general_exception_handler             /* Call BSP_Except_Handler() for exceptions not...       */
    jalr  $8                                  /* ...triggered by syscall.                              */
    nop

    b     RESTORE_CTX
    nop
    
SAVE_SP:

    la    $8,  OSTCBCur                        /* Save the current task's stack pointer                */
    lw    $9,  0($8)
    sw    $29, 0($9)

    la    $8,  OSTaskSwHook                    /* Call OSTaskSwHook()                                  */
    jalr  $8
    nop

    la    $8,  OSPrioHighRdy                   /* Update the current priority                          */
    lbu   $9,  0($8) 
    la    $10, OSPrioCur
    sb    $9,  0($10)

    la    $8,  OSTCBHighRdy                    /* Update the current TCB                               */
    lw    $9,  0($8)
    la    $10, OSTCBCur
    sw    $9,  0($10)

    lw    $29, 0($9)                           /* Load the new task's stack pointer                    */

RESTORE_CTX:

    lw    $8,  STK_OFFSET_SR($29)              /* Restore the Status register                          */
    mtc0  $8,  $12, 0 

    lw    $8,  STK_OFFSET_EPC($29)             /* Restore the EPC                                      */
    mtc0  $8,  $14, 0

    lw    $8, STK_OFFSET_DSPCTRL($29)         /* Restore the contents of the DSP Control Register      */
    wrdsp $8
     
    lw    $8, STK_OFFSET_AC0LO($29)           /* Restore the contents of the ac0 LO and HI registers   */
    lw    $9, STK_OFFSET_AC0HI($29)
    mtlo  $8, $ac0
    mthi  $9, $ac0

    lw    $8, STK_OFFSET_AC1LO($29)           /* Restore the contents of the ac1 LO and HI registers   */
    lw    $9, STK_OFFSET_AC1HI($29)
    mtlo  $8, $ac1
    mthi  $9, $ac1
    
    lw    $8, STK_OFFSET_AC2LO($29)           /* Restore the contents of the ac2 LO and HI registers   */
    lw    $9, STK_OFFSET_AC2HI($29)
    mtlo  $8, $ac2
    mthi  $9, $ac2
    
    lw    $8, STK_OFFSET_AC3LO($29)           /* Restore the contents of the ac3 LO and HI registers   */
    lw    $9, STK_OFFSET_AC3HI($29)
    mtlo  $8, $ac3
    mthi  $9, $ac3

#if defined (__mips_hard_float) 
					       
    LDC1  $f0, STK_OFFSET_FPR0($29)
    LDC1  $f1, STK_OFFSET_FPR1($29)
    LDC1  $f2, STK_OFFSET_FPR2($29)
    LDC1  $f3, STK_OFFSET_FPR3($29)
    LDC1  $f4, STK_OFFSET_FPR4($29)
    LDC1  $f5, STK_OFFSET_FPR5($29)
    LDC1  $f6, STK_OFFSET_FPR6($29)
    LDC1  $f7, STK_OFFSET_FPR7($29)
    LDC1  $f8, STK_OFFSET_FPR8($29)
    LDC1  $f9, STK_OFFSET_FPR9($29)
    LDC1  $f10, STK_OFFSET_FPR10($29)
    LDC1  $f11, STK_OFFSET_FPR11($29)
    LDC1  $f12, STK_OFFSET_FPR12($29)
    LDC1  $f13, STK_OFFSET_FPR13($29)
    LDC1  $f14, STK_OFFSET_FPR14($29)
    LDC1  $f15, STK_OFFSET_FPR15($29)
    LDC1  $f16, STK_OFFSET_FPR16($29)
    LDC1  $f17, STK_OFFSET_FPR17($29)
    LDC1  $f18, STK_OFFSET_FPR18($29)
    LDC1  $f19, STK_OFFSET_FPR19($29)
    LDC1  $f20, STK_OFFSET_FPR20($29)
    LDC1  $f21, STK_OFFSET_FPR21($29)
    LDC1  $f22, STK_OFFSET_FPR22($29)
    LDC1  $f23, STK_OFFSET_FPR23($29)
    LDC1  $f24, STK_OFFSET_FPR24($29)
    LDC1  $f25, STK_OFFSET_FPR25($29)
    LDC1  $f26, STK_OFFSET_FPR26($29)
    LDC1  $f27, STK_OFFSET_FPR27($29)  
    LDC1  $f28, STK_OFFSET_FPR28($29)
    LDC1  $f29, STK_OFFSET_FPR29($29)
    LDC1  $f30, STK_OFFSET_FPR30($29)
    LDC1  $f31, STK_OFFSET_FPR31($29)
    lw    $8,STK_OFFSET_FCSR($29)    
    CTC1  $8,$f31
    
#endif

    lw    $31, STK_OFFSET_GPR31($29)           /* Restore the General Purpose Registers                */
    lw    $30, STK_OFFSET_GPR30($29)
    lw    $28, STK_OFFSET_GPR28($29)
    lw    $27, STK_OFFSET_GPR27($29)
    lw    $26, STK_OFFSET_GPR26($29)
    lw    $25, STK_OFFSET_GPR25($29)
    lw    $24, STK_OFFSET_GPR24($29)
    lw    $23, STK_OFFSET_GPR23($29)
    lw    $22, STK_OFFSET_GPR22($29)
    lw    $21, STK_OFFSET_GPR21($29)
    lw    $20, STK_OFFSET_GPR20($29)
    lw    $19, STK_OFFSET_GPR19($29)
    lw    $18, STK_OFFSET_GPR18($29)
    lw    $17, STK_OFFSET_GPR17($29)
    lw    $16, STK_OFFSET_GPR16($29)
    lw    $15, STK_OFFSET_GPR15($29)
    lw    $14, STK_OFFSET_GPR14($29)
    lw    $13, STK_OFFSET_GPR13($29)
    lw    $12, STK_OFFSET_GPR12($29)
    lw    $11, STK_OFFSET_GPR11($29)
    lw    $10, STK_OFFSET_GPR10($29)
    lw    $9,  STK_OFFSET_GPR9($29)
    lw    $8,  STK_OFFSET_GPR8($29)
    lw    $7,  STK_OFFSET_GPR7($29)
    lw    $6,  STK_OFFSET_GPR6($29)
    lw    $5,  STK_OFFSET_GPR5($29)
    lw    $4,  STK_OFFSET_GPR4($29)
    lw    $3,  STK_OFFSET_GPR3($29)
    lw    $2,  STK_OFFSET_GPR2($29)
    lw    $1,  STK_OFFSET_GPR1($29)

    addi  $29, $29, STK_CTX_SIZE               /* Adjust the stack pointer                             */   

    eret                                       /* Resume execution in new task                         */

    .end _general_exception_context



/*
*********************************************************************************************************
*                                          CoreTimerIntHandler
*
* Description: The core timer, which is implemented by the Coprocessor 0 Count and Compare registers, 
*              is the source of uC/OS-II's tick interrupts.  This function handles those interrupts, 
*              saving the current task's context and then calling OSTimeTick(), uC/OS-II's CPU-
*              independent routine for processing tick interrupts.
*
*              The interrupted task's context is saved onto its stack as follows:
*
*
*              OSTCBHighRdy->OSTCBStkPtr + 0x00    Free Entry                    (LOW Memory)
*                                        + 0x04    Status Register
*                                        + 0x08    EPC
*                                        + 0x0C    Special Purpose LO Register
*                                        + 0x10    Special Purpose HI Register
*                                        + 0x14    GPR[1]
*                                        + 0x18    GPR[2]
*                                        + 0x1C    GPR[3]
*                                        + 0x20    GPR[4]
*                                               |
*                                               |
*                                              \ /
*                                               V
*                                        + 0x80    GPR[28]
*                                        + 0x84    GPR[30]
*                                        + 0x88    GPR[31]                       (HIGH Memory)
*********************************************************************************************************
*/

    .ent CoreTimerIntHandler
CoreTimerIntHandler:

    addi  $29, $29, -STK_CTX_SIZE              /* Adjust the stack pointer                             */                      

    sw    $1,  STK_OFFSET_GPR1($29)            /* Save the General Pupose Registers                    */
    sw    $2,  STK_OFFSET_GPR2($29)
    sw    $3,  STK_OFFSET_GPR3($29)
    sw    $4,  STK_OFFSET_GPR4($29)
    sw    $5,  STK_OFFSET_GPR5($29)
    sw    $6,  STK_OFFSET_GPR6($29)
    sw    $7,  STK_OFFSET_GPR7($29)
    sw    $8,  STK_OFFSET_GPR8($29)
    sw    $9,  STK_OFFSET_GPR9($29)
    sw    $10, STK_OFFSET_GPR10($29)
    sw    $11, STK_OFFSET_GPR11($29)
    sw    $12, STK_OFFSET_GPR12($29)
    sw    $13, STK_OFFSET_GPR13($29)
    sw    $14, STK_OFFSET_GPR14($29)
    sw    $15, STK_OFFSET_GPR15($29)
    sw    $16, STK_OFFSET_GPR16($29)
    sw    $17, STK_OFFSET_GPR17($29)
    sw    $18, STK_OFFSET_GPR18($29)
    sw    $19, STK_OFFSET_GPR19($29)
    sw    $20, STK_OFFSET_GPR20($29)
    sw    $21, STK_OFFSET_GPR21($29)
    sw    $22, STK_OFFSET_GPR22($29)
    sw    $23, STK_OFFSET_GPR23($29)
    sw    $24, STK_OFFSET_GPR24($29)
    sw    $25, STK_OFFSET_GPR25($29)
    sw    $26, STK_OFFSET_GPR26($29)
    sw    $27, STK_OFFSET_GPR27($29)
    sw    $28, STK_OFFSET_GPR28($29)
    sw    $30, STK_OFFSET_GPR30($29)
    sw    $31, STK_OFFSET_GPR31($29)
#if defined (__mips_hard_float)  
  
    SDC1  $f0, STK_OFFSET_FPR0($29)
    SDC1  $f1, STK_OFFSET_FPR1($29)
    SDC1  $f2, STK_OFFSET_FPR2($29)
    SDC1  $f3, STK_OFFSET_FPR3($29)
    SDC1  $f4, STK_OFFSET_FPR4($29)
    SDC1  $f5, STK_OFFSET_FPR5($29)
    SDC1  $f6, STK_OFFSET_FPR6($29)
    SDC1  $f7, STK_OFFSET_FPR7($29)
    SDC1  $f8, STK_OFFSET_FPR8($29)
    SDC1  $f9, STK_OFFSET_FPR9($29)
    SDC1  $f10, STK_OFFSET_FPR10($29)
    SDC1  $f11, STK_OFFSET_FPR11($29)
    SDC1  $f12, STK_OFFSET_FPR12($29)
    SDC1  $f13, STK_OFFSET_FPR13($29)
    SDC1  $f14, STK_OFFSET_FPR14($29)
    SDC1  $f15, STK_OFFSET_FPR15($29)
    SDC1  $f16, STK_OFFSET_FPR16($29)
    SDC1  $f17, STK_OFFSET_FPR17($29)
    SDC1  $f18, STK_OFFSET_FPR18($29)
    SDC1  $f19, STK_OFFSET_FPR19($29)
    SDC1  $f20, STK_OFFSET_FPR20($29)
    SDC1  $f21, STK_OFFSET_FPR21($29)
    SDC1  $f22, STK_OFFSET_FPR22($29)
    SDC1  $f23, STK_OFFSET_FPR23($29)
    SDC1  $f24, STK_OFFSET_FPR24($29)
    SDC1  $f25, STK_OFFSET_FPR25($29)
    SDC1  $f26, STK_OFFSET_FPR26($29)
    SDC1  $f27, STK_OFFSET_FPR27($29)  
    SDC1  $f28, STK_OFFSET_FPR28($29)
    SDC1  $f29, STK_OFFSET_FPR29($29)
    SDC1  $f30, STK_OFFSET_FPR30($29)
    SDC1  $f31, STK_OFFSET_FPR31($29)
    CFC1  $8,$f31
    sw    $8,STK_OFFSET_FCSR($29)
#endif    
					       /* Save the contents of the ac 0-3 registers         */
    mflo  $8,  $ac0
    mfhi  $9,  $ac0
    sw    $8,  STK_OFFSET_AC0LO($29)
    sw    $9,  STK_OFFSET_AC0HI($29)
    
    mflo  $8,  $ac1
    mfhi  $9,  $ac1
    sw    $8,  STK_OFFSET_AC1LO($29)
    sw    $9,  STK_OFFSET_AC1HI($29)
    
    mflo  $8,  $ac2
    mfhi  $9,  $ac2
    sw    $8,  STK_OFFSET_AC2LO($29)
    sw    $9,  STK_OFFSET_AC2HI($29)

    mflo  $8,  $ac3
    mfhi  $9,  $ac3
    sw    $8,  STK_OFFSET_AC3LO($29)
    sw    $9,  STK_OFFSET_AC3HI($29) 

    rddsp $8
    sw    $8,  STK_OFFSET_DSPCTRL($29)         /* Save the DSP Control Register                        */

    mfc0  $8,  $14, 0                          /* Save the EPC                                         */
    sw    $8,  STK_OFFSET_EPC($29)
  
    mfc0  $8,  $12, 0
    sw    $8,  STK_OFFSET_SR($29)

    la    $8,  OSIntNesting                    /* See if OSIntNesting == 0                             */
    lbu   $9,  0($8) 
    bne   $0,  $9, TICK_INC_NESTING
    nop   

    la    $10, OSTCBCur                        /* Save the current task's stack pointer                */
    lw    $11, 0($10)
    sw    $29, 0($11)
  
TICK_INC_NESTING:

    addi  $9,  $9, 1                           /* Increment OSIntNesting                               */
    sb    $9,  0($8)

    mfc0  $8, $12, 0                           /* Update the Status register's IPL bits                */
    mfc0  $9, $13, 0
    andi  $9, $9,  0xFC00
    ins   $8, $0,  10, 6
    or    $8, $8,  $9
    mtc0  $8, $12, 0

    la    $8,  BSP_TickISR_Handler             /* Call BSP_TickISR_Handler() to handle the interrupt   */
    jalr  $8
    nop

    la    $8,  OSTimeTick                      /* Call OSTimeTick()                                    */
    jalr  $8
    nop

    la    $8,  OSIntExit                       /* Call OSIntExit()                                     */
    jalr  $8
    nop

    lw    $8,  STK_OFFSET_SR($29)              /* Restore the Status register                          */
    mtc0  $8,  $12, 0

    lw    $8,  STK_OFFSET_EPC($29)             /* Restore the EPC                                      */
    mtc0  $8,  $14, 0

    lw    $8, STK_OFFSET_DSPCTRL($29)         /* Restore the contents of the DSP Control Register      */
    wrdsp $8
     
    lw    $8, STK_OFFSET_AC0LO($29)           /* Restore the contents of the ac0 LO and HI registers   */
    lw    $9, STK_OFFSET_AC0HI($29)
    mtlo  $8, $ac0
    mthi  $9, $ac0

    lw    $8, STK_OFFSET_AC1LO($29)           /* Restore the contents of the ac1 LO and HI registers   */
    lw    $9, STK_OFFSET_AC1HI($29)
    mtlo  $8, $ac1
    mthi  $9, $ac1
    
    lw    $8, STK_OFFSET_AC2LO($29)           /* Restore the contents of the ac2 LO and HI registers   */
    lw    $9, STK_OFFSET_AC2HI($29)
    mtlo  $8, $ac2
    mthi  $9, $ac2
    
    lw    $8, STK_OFFSET_AC3LO($29)           /* Restore the contents of the ac3 LO and HI registers   */
    lw    $9, STK_OFFSET_AC3HI($29)
    mtlo  $8, $ac3
    mthi  $9, $ac3

#if defined (__mips_hard_float) 
					       
    LDC1  $f0, STK_OFFSET_FPR0($29)
    LDC1  $f1, STK_OFFSET_FPR1($29)
    LDC1  $f2, STK_OFFSET_FPR2($29)
    LDC1  $f3, STK_OFFSET_FPR3($29)
    LDC1  $f4, STK_OFFSET_FPR4($29)
    LDC1  $f5, STK_OFFSET_FPR5($29)
    LDC1  $f6, STK_OFFSET_FPR6($29)
    LDC1  $f7, STK_OFFSET_FPR7($29)
    LDC1  $f8, STK_OFFSET_FPR8($29)
    LDC1  $f9, STK_OFFSET_FPR9($29)
    LDC1  $f10, STK_OFFSET_FPR10($29)
    LDC1  $f11, STK_OFFSET_FPR11($29)
    LDC1  $f12, STK_OFFSET_FPR12($29)
    LDC1  $f13, STK_OFFSET_FPR13($29)
    LDC1  $f14, STK_OFFSET_FPR14($29)
    LDC1  $f15, STK_OFFSET_FPR15($29)
    LDC1  $f16, STK_OFFSET_FPR16($29)
    LDC1  $f17, STK_OFFSET_FPR17($29)
    LDC1  $f18, STK_OFFSET_FPR18($29)
    LDC1  $f19, STK_OFFSET_FPR19($29)
    LDC1  $f20, STK_OFFSET_FPR20($29)
    LDC1  $f21, STK_OFFSET_FPR21($29)
    LDC1  $f22, STK_OFFSET_FPR22($29)
    LDC1  $f23, STK_OFFSET_FPR23($29)
    LDC1  $f24, STK_OFFSET_FPR24($29)
    LDC1  $f25, STK_OFFSET_FPR25($29)
    LDC1  $f26, STK_OFFSET_FPR26($29)
    LDC1  $f27, STK_OFFSET_FPR27($29)  
    LDC1  $f28, STK_OFFSET_FPR28($29)
    LDC1  $f29, STK_OFFSET_FPR29($29)
    LDC1  $f30, STK_OFFSET_FPR30($29)
    LDC1  $f31, STK_OFFSET_FPR31($29)
    lw    $8,STK_OFFSET_FCSR($29)    
    CTC1  $8,$f31
    
#endif


    lw    $31, STK_OFFSET_GPR31($29)           /* Restore the General Purpose Registers                */
    lw    $30, STK_OFFSET_GPR30($29)
    lw    $28, STK_OFFSET_GPR28($29)
    lw    $27, STK_OFFSET_GPR27($29)
    lw    $26, STK_OFFSET_GPR26($29)
    lw    $25, STK_OFFSET_GPR25($29)
    lw    $24, STK_OFFSET_GPR24($29)
    lw    $23, STK_OFFSET_GPR23($29)
    lw    $22, STK_OFFSET_GPR22($29)
    lw    $21, STK_OFFSET_GPR21($29)
    lw    $20, STK_OFFSET_GPR20($29)
    lw    $19, STK_OFFSET_GPR19($29)
    lw    $18, STK_OFFSET_GPR18($29)
    lw    $17, STK_OFFSET_GPR17($29)
    lw    $16, STK_OFFSET_GPR16($29)
    lw    $15, STK_OFFSET_GPR15($29)
    lw    $14, STK_OFFSET_GPR14($29)
    lw    $13, STK_OFFSET_GPR13($29)
    lw    $12, STK_OFFSET_GPR12($29)
    lw    $11, STK_OFFSET_GPR11($29)
    lw    $10, STK_OFFSET_GPR10($29)
    lw    $9,  STK_OFFSET_GPR9($29)
    lw    $8,  STK_OFFSET_GPR8($29)
    lw    $7,  STK_OFFSET_GPR7($29)
    lw    $6,  STK_OFFSET_GPR6($29)
    lw    $5,  STK_OFFSET_GPR5($29)
    lw    $4,  STK_OFFSET_GPR4($29)
    lw    $3,  STK_OFFSET_GPR3($29)
    lw    $2,  STK_OFFSET_GPR2($29)
    lw    $1,  STK_OFFSET_GPR1($29)

    addi  $29, $29, STK_CTX_SIZE               /* Adjust the stack pointer                             */   

    eret                                   

    .end CoreTimerIntHandler


