/*
*********************************************************************************************************
*                                               uC/OS-III
*                                         The Real-Time Kernel
*
*                             (c) Copyright 2010, Micrium, Inc., Weston, FL
*                                           All Rights Reserved
*
*                                              MIPS32 4K
*
*                                                MPLAB
*
* File    : os_cpu_a.S
* Version : v3.00
*********************************************************************************************************
*/

#include "os_cpu_a.inc"

/*
*********************************************************************************************************
*                                          PUBLIC FUNCTIONS
*********************************************************************************************************
*/

    .global  OSStartHighRdy
    .global  OSIntCtxSw
    .global  OS_CPU_SR_Save
    .global  OS_CPU_SR_Restore
    .global  _general_exception_context
    .global  OSTickIntHandler
    .extern  BSP_TickHandler
    .extern  _general_exception_handler
    .extern  OSTimeTick
    .extern  OSTCBCurPtr
    .extern  OSIntExit
    .extern  OSTaskSwHook
    .extern  OSPrioHighRdy
    .extern  OSPrioCur
    .extern  OSTCBHighRdyPtr
    .extern  OSIntNestingCtr

/*
*********************************************************************************************************
*                                          ASSEMBLER DIRECTIVES
*********************************************************************************************************
*/
    .section .text,code
    .set noreorder
    .set noat
    .set nomicromips
    .set nomips16

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

    la    $8,  OSTCBHighRdyPtr                 /* Update the current TCB                               */
    lw    $9,  0($8) 
    lw    $29, 0($9)                           /* Load the new task's stack pointer                    */

    restoreCPU_CONTEXT

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

    la    $8,  OSTCBHighRdyPtr                 /* Update the current TCB                               */
    lw    $9,  0($8)
    la    $10, OSTCBCurPtr
    sw    $9,  0($10)

    lw    $29, 0($9)                           /* Load the new task's stack pointer                    */

    restoreCPU_CONTEXT

    .end OSIntCtxSw

/*
*********************************************************************************************************
*                                          DISABLE INTERRUPTS
*                                   OS_CPU_SR  OS_CPU_SR_Save(void);
*
* Description: This function saves the state of the Status register and then disables interrupts via this
*              register.  This objective is accomplished with a single instruction, di.  The di 
*              instruction's operand, $2, is the general purpose register to which the Status register's 
*              value is saved.  This value can be read by C functions that call OS_CPU_SR_Save().  
*
* Arguments  : None
*
* Returns    : The previous state of the Status register
*********************************************************************************************************
*/

    .ent OS_CPU_SR_Save
OS_CPU_SR_Save:

    di       $2                                /* Disable interrupts, and move the old value of the... */
    jr.hb    $31                               /* ...Status register into v0 ($2)                      */
    nop
                                               
    .end OS_CPU_SR_Save

/*
*********************************************************************************************************
*                                          ENABLE INTERRUPTS
*                                  void OS_CPU_SR_Restore(OS_CPU_SR sr);
*
* Description: This function must be used in tandem with OS_CPU_SR_Save().  Calling OS_CPU_SR_Restore()
*              causes the value returned by OS_CPU_SR_Save() to be placed in the Status register. 
*
* Arguments  : The value to be placed in the Status register
*
* Returns    : None
*********************************************************************************************************
*/

    .ent OS_CPU_SR_Restore
OS_CPU_SR_Restore:

    mtc0  $4, $12, 0                           /* Restore the status register to its previous state    */
    jr.hb $31
    nop

    .end OS_CPU_SR_Restore


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
*              The stack frame of the task to suspend will look as follows when _general_exception_context() is done:
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
*
* Note(s): 1) OSCtxSW() in invoked from task level code by the scheduler.  The SYSCALL instruction is
*             used to trigger this routine.  Because SYSCALL is a precise/synchronous general exception
*             (see MIPS documentation for definition), the EPC will contain the address of the
*             instruction which caused the exception to occur.  Therefore this rountine needs to add 4 to
*             EPC to get the proper return address before storing the EPC value on to the Tasks
*             Stack.  By adding 4 we are calculating the address of the instruction following SYSCALL.
*
*********************************************************************************************************
*/

    .ent _general_exception_context
_general_exception_context:
    saveCPU_CONTEXT
    
    ori   $8,  $0,  0x007C                     /* Switch context only if a syscall exception occurred  */
    mfc0  $9,  $13, 0
    and   $9,  $9,  $8
    ori   $10, $0,  0x0020
    beq   $9,  $10, 0f
    nop              

    la    $8,  _general_exception_handler     /* Call BSP_Except_Handler() for exceptions not...       */
    jalr  $8                                  /* ...triggered by syscall.                              */
    nop

    b     1f
    nop
    
0:
    lw    $8,  STK_OFFSET_EPC($29)             /* Retrieve the stored EPC value from the Task's Stack  */
    addiu $8,  $8,  4                          /* EPC will reference the instruction following syscall */
    sw    $8,  STK_OFFSET_EPC($29)             /* See notes in comment header for explanation          */

    la    $8,  OSTCBCurPtr                     /* Save the current task's stack pointer                */
    lw    $9,  0($8)
    sw    $29, 0($9)

    la    $8,  OSTaskSwHook                    /* Call OSTaskSwHook()                                  */
    jalr  $8
    nop

    la    $8,  OSPrioHighRdy                   /* Update the current priority                          */
    lbu   $9,  0($8) 
    la    $10, OSPrioCur
    sb    $9,  0($10)

    la    $8,  OSTCBHighRdyPtr                 /* Update the current TCB                               */
    lw    $9,  0($8)
    la    $10, OSTCBCurPtr
    sw    $9,  0($10)

    lw    $29, 0($9)                           /* Load the new task's stack pointer                    */

1:
    restoreCPU_CONTEXT

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

    .ent OSTickIntHandler
OSTickIntHandler:

    saveCPU_CONTEXT
    
    la    $8,  OSIntNestingCtr                 /* See if OSIntNesting == 0                             */
    lbu   $9,  0($8) 
    bne   $0,  $9, 0f
    nop   

    la    $10, OSTCBCurPtr                     /* Save the current task's stack pointer                */
    lw    $11, 0($10)
    sw    $29, 0($11)
  
0:
    addiu $9,  $9, 1                           /* Increment OSIntNesting                               */
    sb    $9,  0($8)

    mfc0  $8, $12, 0                           /* Update the Status register's IPL bits                */
    mfc0  $9, $13, 0
    andi  $9, $9,  0xFC00
    ins   $8, $0,  10, 6
    or    $8, $8,  $9
    mtc0  $8, $12, 0
    ehb

    la    $8,  BSP_TickHandler                 /* Call BSP_TickISR_Handler() to handle the interrupt   */
    jalr  $8
    nop

    la    $8,  OSTimeTick                      /* Call OSTimeTick()                                    */
    jalr  $8
    nop

    la    $8,  OSIntExit                       /* Call OSIntExit()                                     */
    jalr  $8
    nop

    restoreCPU_CONTEXT

    .end OSTickIntHandler


