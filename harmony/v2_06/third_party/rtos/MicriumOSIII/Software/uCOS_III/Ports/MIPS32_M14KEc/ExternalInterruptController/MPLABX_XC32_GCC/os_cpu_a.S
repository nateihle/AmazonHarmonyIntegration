/*
*********************************************************************************************************
*                                               uC/OS-III
*                                         The Real-Time Kernel
*
*                             (c) Copyright 2010, Micrium, Inc., Weston, FL
*                                           All Rights Reserved
*
*                                              MIPS32 M14KEc
*
*                                                MPLAB
*
* File    : os_cpu_a.S
* Version : vX.xx
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
    .global  _general_exception_context
    .global  OSTickIntHandler
    
    .extern  OSTaskSwHook
    .extern  _general_exception_handler
    .extern  OSTCBHighRdyPtr
    .extern  OSTCBCurPtr
    .extern  OSPrioHighRdy
    .extern  OSPrioCur
    .extern  OSIntEnter
    .extern  OSIntExit
    .extern  BSP_TickHandler
/*
*********************************************************************************************************
*                                          ASSEMBLER DIRECTIVES
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
*                                           OSStartHighRdy()
*
* Description: Starts the highest priority task that is available to run.  OSStartHighRdy() MUST:
*
*              a) Call OSTaskSwHook()
*              b) Load the new Tasks Stack pointer into the CPU Stack pointer register ($29)
*              c) Switch to the highest priority task by restoring the CPU context from the Task's stack.
*
*              The stack frame of the task to resume is assumed to look as follows:
*
*              OSTCBHighRdy->OSTCBStkPtr + 0x00    Free Entry                    (LOW Memory)
*                                        + 0x04    Status Register
*                                        + 0x08    EPC
*                                        + 0x0C    DSP Control Register
*                                        + 0x10    Accumulator 0 LO Register
*                                        + 0x14    Accumulator 0 HI Register
*                                        + 0x18    Accumulator 1 LO Register
*                                        + 0x1C    Accumulator 1 HI Register
*                                        + 0x20    Accumulator 2 LO Register
*                                        + 0x24    Accumulator 2 HI Register
*                                        + 0x28    Accumulator 3 LO Register
*                                        + 0x2C    Accumulator 3 HI Register
*                                        + 0x30    GPR[1]
*                                        + 0x34    GPR[2]
*                                        + 0x38    GPR[3]
*                                        + 0x3C    GPR[4]
*                                               |
*                                               |
*                                              \ /
*                                               V
*                                        + 0x9C    GPR[28]
*                                        + 0xA0    GPR[30]
*                                        + 0xA4    GPR[31]                       (HIGH Memory)
*                              
* Note(s): 1) OSTaskStkInit(), which is responsible for initializing each task's stack, sets bit 0 and bit 
*             1 (i.e. IE bit and EXL bit) of the entry corresponding to the Status register.  Thus, after
*             OSStartHighRdy returns, interrupts will be enabled when each task first runs.
*          2) OSStartHighRdy does not run within the context of an ISR, however, the COP0 Status register 
*             will have its IE bit and EXL bit set when the CPU context is restored from the tasks stack.  
*             To properly clear the EXL bit OSStartHighRdy executes an ERET (Exception Return) 
*             instruction.  This will re-enable interrupt exceptions, as per the MIPS documenation. 
*          3) Even though the global IE bit is set in COP0 Status Register, interrupt exceptions are not
*             not enabled when the EXL bit is set.
*********************************************************************************************************
*/

    .ent OSStartHighRdy
OSStartHighRdy:
    la    $8, OSTaskSwHook
    jalr  $8
    nop

    la    $8, OSTCBHighRdyPtr                  /* Get the new task's stack pointer address             */
    lw    $9,  0($8) 
    lw    $29, 0($9)                           /* Load the new task's stack pointer                    */
                                               /* SP = OSTCBHighRdyPtr->StkPtr;                        */
                                               
    restoreCPU_CONTEXT                         /* restore the Tasks envioronment                       */

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
*              tasks stack and the SP for the task has been saved in its OS_TCB by the ISR.
*
*              The stack frame of the task to resume is assumed to look as follows:
*
*              OSTCBHighRdy->OSTCBStkPtr + 0x00    Free Entry                    (LOW Memory)
*                                        + 0x04    Status Register
*                                        + 0x08    EPC
*                                        + 0x0C    DSP Control Register
*                                        + 0x10    Accumulator 0 LO Register
*                                        + 0x14    Accumulator 0 HI Register
*                                        + 0x18    Accumulator 1 LO Register
*                                        + 0x1C    Accumulator 1 HI Register
*                                        + 0x20    Accumulator 2 LO Register
*                                        + 0x24    Accumulator 2 HI Register
*                                        + 0x28    Accumulator 3 LO Register
*                                        + 0x2C    Accumulator 3 HI Register
*                                        + 0x30    GPR[1]
*                                        + 0x34    GPR[2]
*                                        + 0x38    GPR[3]
*                                        + 0x3C    GPR[4]
*                                               |
*                                               |
*                                              \ /
*                                               V
*                                        + 0x9C    GPR[28]
*                                        + 0xA0    GPR[30]
*                                        + 0xA4    GPR[31]                       (HIGH Memory)
*********************************************************************************************************
*/            

    .ent OSIntCtxSw
OSIntCtxSw:
    la    $8, OSTaskSwHook
    jalr  $8
    nop

    la    $8,  OSPrioHighRdy                   /* Update the current priority.......                   */
    lbu   $9,  0($8) 
    la    $10, OSPrioCur
    sb    $9,  0($10)                          /* OSPrioCur = OSPrioHighRdy                            */

    
    la    $8,  OSTCBHighRdyPtr                 /* Update the current TCB.....                          */
    lw    $9,  0($8)
    la    $10, OSTCBCurPtr
    sw    $9,  0($10)                          /* OSTCBCurPtr = OSTCBHighRdyPtr                        */

    lw    $29, 0($9)                           /* Load the new tasks stack pointer                     */
                                               /* SP = OSTCBHighRdyPtr->StkPtr;                        */

    restoreCPU_CONTEXT                         /*Restore Task CPU environment                          */

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
*                  Save the LO and HI registers ( 0 - 3);
*                  Save the DSP Control register;
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
*                                        + 0x0C    DSP Control Register
*                                        + 0x10    Accumulator 0 LO Register
*                                        + 0x14    Accumulator 0 HI Register
*                                        + 0x18    Accumulator 1 LO Register
*                                        + 0x1C    Accumulator 1 HI Register
*                                        + 0x20    Accumulator 2 LO Register
*                                        + 0x24    Accumulator 2 HI Register
*                                        + 0x28    Accumulator 3 LO Register
*                                        + 0x2C    Accumulator 3 HI Register
*                                        + 0x30    GPR[1]
*                                        + 0x34    GPR[2]
*                                        + 0x38    GPR[3]
*                                        + 0x3C    GPR[4]
*                                               |
*                                               |
*                                              \ /
*                                               V
*                                        + 0x9C    GPR[28]
*                                        + 0xA0    GPR[30]
*                                        + 0xA4    GPR[31]                       (HIGH Memory)
*              
*              The stack frame of the task to resume looks as follows:
*
*              OSTCBHighRdy->OSTCBStkPtr + 0x00    Free Entry                    (LOW Memory)
*                                        + 0x04    Status Register
*                                        + 0x08    EPC
*                                        + 0x0C    DSP Control Register
*                                        + 0x10    Accumulator 0 LO Register
*                                        + 0x14    Accumulator 0 HI Register
*                                        + 0x18    Accumulator 1 LO Register
*                                        + 0x1C    Accumulator 1 HI Register
*                                        + 0x20    Accumulator 2 LO Register
*                                        + 0x24    Accumulator 2 HI Register
*                                        + 0x28    Accumulator 3 LO Register
*                                        + 0x2C    Accumulator 3 HI Register
*                                        + 0x30    GPR[1]
*                                        + 0x34    GPR[2]
*                                        + 0x38    GPR[3]
*                                        + 0x3C    GPR[4]
*                                               |
*                                               |
*                                              \ /
*                                               V
*                                        + 0x9C    GPR[28]
*                                        + 0xA0    GPR[30]
*                                        + 0xA4    GPR[31]                       (HIGH Memory)
* Note(s): 1) _general_exception_context() in invoked from task level code by the scheduler.  The SYSCALL instruction is 
*             used to trigger this routine.  Because SYSCALL is a precise/synchronous general exception
*             (see MIPS documentation for definition), the EPC will contain the address of the 
*             instruction which caused the exception to occur.  Therefore this rountine needs to add 4 to 
*             EPC to get the proper return address before storing the EPC value on to the Tasks 
*             Stack.  By adding 4 we are calculating the address of the instruction following SYSCALL.
*          2) The macro called to save the CPU context only performs stores.  The macro, saveCPU_CONTEXT
*             will not modify the values of the registers it saves.  Therefore addtional instructions are
*             needed here to implement the information in note 1.
*          3) Since the Task Level context switch uses SYSCALL, this routine is multiplexed with the 
*             other general exceptions which can occur in the CPU.  Therefore, this routine must 
*             check what kind of general exception occurred and make the appropriate adjustments if it 
*             was an exception other than SYSCALL.
*********************************************************************************************************
*/

    .ent _general_exception_context
_general_exception_context:    
    saveCPU_CONTEXT                            /* Save CPU context to current Tasks Stack              */
    
    ori   $8,  $0,  0x007C                     /* Switch context only if a syscall exception occurred  */
    mfc0  $9,  $13, 0
    and   $9,  $9,  $8
    ori   $10, $0,  0x0020
    beq   $9,  $10, 0f
    nop
    
    la    $8,  _general_exception_handler      /* Call default_handler() for general exceptions not    */
    jalr  $8                                   /* triggered by a SYSCALL. See Note 3                   */
    nop
    b     1f                                   /* restore context after processing general exception   */
    nop
    
0:
    lw    $8,  STK_OFFSET_EPC($29)             /* Retrieve the stored EPC value from the Task's Stack  */
    addiu $8,  $8,  4                          /* EPC will reference the instruction following syscall */
    sw    $8,  STK_OFFSET_EPC($29)             /* See notes in comment header for explanation          */
    
    la    $8,  OSTCBCurPtr                     /* Save the current task's stack pointer                */
    lw    $9,  0($8)
    sw    $29, 0($9)                           /* OSTCBHighRdyPtr->StkPtr = SP;                        */

    la    $8,  OSTaskSwHook                    /* Call OSTaskSwHook()                                  */
    jalr  $8
    nop

    la    $8,  OSPrioHighRdy                   /* Update the current priority.......                   */
    lbu   $9,  0($8) 
    la    $10, OSPrioCur
    sb    $9,  0($10)                          /* OSPrioCur = OSPrioHighRdy                            */

    la    $8,  OSTCBHighRdyPtr                 /* Update the current TCB.....                          */
    lw    $9,  0($8)
    la    $10, OSTCBCurPtr
    sw    $9,  0($10)                          /* OSTCBCurPtr = OSTCBHighRdyPtr                        */

    lw    $29, 0($9)                           /* Load the new task's stack pointer                    */
                                               /* SP = OSTCBHighRdyPtr->StkPtr;                        */
1:
    restoreCPU_CONTEXT                         /*Restore Task CPU environment                          */

    .end _general_exception_context


/*
*********************************************************************************************************
*                                          OSTickIntHandler
*
* Description: This assembler routine processes the uC/OS-III's tick interrupts.  This assembler routine
*              can be called from the users selected timer source. This function handles those interrupts,
*              saving the current task's context and then calling OSTimeTick(), uC/OS-III's CPU-
*              independent routine for processing tick interrupts.
*
*              The interrupted task's context is saved onto its stack as follows:
*
*              OSTCBHighRdy->OSTCBStkPtr + 0x00    Free Entry                    (LOW Memory)
*                                        + 0x04    Status Register
*                                        + 0x08    EPC
*                                        + 0x0C    DSP Control Register
*                                        + 0x10    Accumulator 0 LO Register
*                                        + 0x14    Accumulator 0 HI Register
*                                        + 0x18    Accumulator 1 LO Register
*                                        + 0x1C    Accumulator 1 HI Register
*                                        + 0x20    Accumulator 2 LO Register
*                                        + 0x24    Accumulator 2 HI Register
*                                        + 0x28    Accumulator 3 LO Register
*                                        + 0x2C    Accumulator 3 HI Register
*                                        + 0x30    GPR[1]
*                                        + 0x34    GPR[2]
*                                        + 0x38    GPR[3]
*                                        + 0x3C    GPR[4]
*                                               |
*                                               |
*                                              \ /
*                                               V
*                                        + 0x9C    GPR[28]
*                                        + 0xA0    GPR[30]
*                                        + 0xA4    GPR[31]                       (HIGH Memory)
*
* Note(s): 1) The M14KEc core has two CP0 registers, View_IPL and View_RIPL. These registers mirror the 
*             the IPL and RIPL bits in CP0 Status and Cause registers respectively.  The use of View_IPL
*             and View_RIPL allows the IPL and RIPL fields to be read and written without extracting the 
*             specific bit fields from the specific CP0 Status and Cause registers.
*          2) The core timer interrupt can be used as a reference model for a user to write there own OS
*             Aware interrupt.
*********************************************************************************************************
*/

    .ent OSTickIntHandler
OSTickIntHandler:
    
    saveCPU_CONTEXT

    la    $8,  OSIntNestingCtr                 /* See if OSIntNesting == 0, save Tasks Stack pointer   */
    lbu   $9,  0($8)                           
    bne   $0,  $9, 0f
    nop   

    la    $10, OSTCBCurPtr                     /* Save the current task's stack pointer                */
    lw    $11, 0($10)
    sw    $29, 0($11)                          /* OSTCBCurPtr->StkPtr = SP;                            */
  
0:
    la    $8, OSIntEnter                       /* Call OSIntEnter                                      */
    jalr  $8
    nop
       
    mfc0  $8, $12, 4                           /* Update the Status register's IPL bits using CP0      */
    mfc0  $9, $13, 4                           /* View_IPL and View_RIPL registers. See Note 1         */
    ins   $8, $9, 0, 10
    mtc0  $8, $12, 4
    ehb
    
    la    $8,  OSTimeTick                      /* Call OSTimeTick()                                    */
    jalr  $8
    nop

    la    $8,  BSP_TickHandler                 /* Call BSP Tick Handler, reload period                 */
    jalr  $8
    nop

    la    $8,  OSIntExit                       /* Call OSIntExit(), this may cause a task ctx switch    */
    jalr  $8
    nop

    restoreCPU_CONTEXT                         /*Restore Interrupted Task CPU environment              */

    .end OSTickIntHandler


