/*
*********************************************************************************************************
*                                               uC/OS-III
*                                         The Real-Time Kernel
*
*                          (c) Copyright 1992-2010, Micrium, Inc., Weston, FL
*                                          All Rights Reserved
*
*                                              MIPS32 M14KEc
*
*                                                MPLAB
*
* File   : os_cpu_c.c
* Version: vX.xx
*********************************************************************************************************
*/

#define   OS_CPU_GLOBALS
#include <os.h>

/*
************************************************************************************************************************
*                                                OS INITIALIZATION HOOK
*
* Description: This function is called by OSInit() at the beginning of OSInit().
*
* Arguments  : none
*
* Note(s)    : none
************************************************************************************************************************
*/

void  OSInitHook (void)
{
}

/*
************************************************************************************************************************
*                                                  TASK CREATION HOOK
*
* Description: This function is called when a task is created.
*
* Arguments  : p_tcb   is a pointer to the task control block of the task being created.
*
* Note(s)    : none
************************************************************************************************************************
*/

void  OSTaskCreateHook (OS_TCB  *p_tcb)
{
#if OS_CFG_APP_HOOKS_EN > 0u
    if (OS_AppTaskCreateHookPtr != (OS_APP_HOOK_TCB)0) {
        (*OS_AppTaskCreateHookPtr)(p_tcb);
    }
#else
    (void)p_tcb;                                            /* Prevent compiler warning                               */
#endif
}

/*
************************************************************************************************************************
*                                                   TASK DELETION HOOK
*
* Description: This function is called when a task is deleted.
*
* Arguments  : p_tcb   is a pointer to the task control block of the task being deleted.
*
* Note(s)    : none
************************************************************************************************************************
*/

void  OSTaskDelHook (OS_TCB  *p_tcb)
{
#if OS_CFG_APP_HOOKS_EN > 0u
    if (OS_AppTaskDelHookPtr != (OS_APP_HOOK_TCB)0) {
        (*OS_AppTaskDelHookPtr)(p_tcb);
    }
#else
    (void)p_tcb;                                            /* Prevent compiler warning                               */
#endif
}

/*
************************************************************************************************************************
*                                                   IDLE TASK HOOK
*
* Description: This function is called by the idle task.  This hook has been added to allow you to do such things as
*              STOP the CPU to conserve power.
*
* Arguments  : none
*
* Note(s)    : none
************************************************************************************************************************
*/

void  OSIdleTaskHook (void)
{
#if OS_CFG_APP_HOOKS_EN > 0u
    if (OS_AppIdleTaskHookPtr != (OS_APP_HOOK_VOID)0) {
        (*OS_AppIdleTaskHookPtr)();
    }
#endif
}

/*
************************************************************************************************************************
*                                                 STATISTIC TASK HOOK
*
* Description: This function is called every second by uC/OS-III's statistics task.  This allows your application to add
*              functionality to the statistics task.
*
* Arguments  : none
*
* Note(s)    : none
************************************************************************************************************************
*/

void  OSStatTaskHook (void)
{
#if OS_CFG_APP_HOOKS_EN > 0u
    if (OS_AppStatTaskHookPtr != (OS_APP_HOOK_VOID)0) {
        (*OS_AppStatTaskHookPtr)();
    }
#endif
}

/*
*********************************************************************************************************
*                                        INITIALIZE A TASK'S STACK
*
* Description: This function is called by either OSTaskCreate() or OSTaskCreateExt() to initialize the
*              stack frame of the task being created.  This function is processor-specific.
*
* Arguments  : task     is a pointer to the task code.
*
*              p_arg    is a pointer to a user supplied data area 
*
*              ptos     is a pointer to the top of stack.  OSTaskStkInit() assumes that 'ptos' points to 
*                       a free entry on the stack.  If OS_STK_GROWTH is set to 1 then 'ptos' will contain 
*                       the HIGHEST valid address of the stack.  Similarly, if OS_STK_GROWTH is set to 0, 
*                       'ptos' will contain the lowest valid address of the stack.
*
*              opt      specifies options that can be used to alter the behavior of OSTaskStkInit()
*                       (see ucos_ii.h for OS_TASK_OPT_???).
*
* Returns    : The location corresponding to the top of the stack
*
* Note(s)    : 1) Interrupts are enabled when each task starts executing.
* 
*              2) An initialized stack has the structure shown below.
*
*              OSTCBHighRdy->OSTCBStkPtr + 0x00    Free Entry                    (LOW Memory)
*                                        + 0x04    Status Register
*                                        + 0x08    EPC
*                                        + 0x0C    DSPControl Register
*                                        + 0x10    Accumulator 0 LO Register (i.e. ac0)
*                                        + 0x14    Accumulator 0 HI Register (i.e. ac0)
*                                        + 0x18    Accumulator 1 LO Register (i.e. lo1)
*                                        + 0x1C    Accumulator 1 HI Register (i.e. ac1)
*                                        + 0x20    Accumulator 2 LO Register (i.e. ac2)
*                                        + 0x24    Accumulator 2 HI Register (i.e. ac2)
*                                        + 0x28    Accumulator 3 LO Register (i.e. ac3)
*                                        + 0x2C    Accumulator 3 HI Register (i.e. ac3)
*                                        + 0x30    GPR[1]
*                                        + 0x34    GPR[2]
*                                        + 0x38    GPR[3]
*                                        + 0x3C    GPR[4]
*                                        + 0x40    GPR[5]
*                                        + 0x44    GPR[6]
*                                        + 0x48    GPR[7]
*                                        + 0x4C    GPR[8]
*                                        + 0x50    GPR[9]
*                                        + 0x54    GPR[10]
*                                        + 0x58    GPR[11]
*                                        + 0x5C    GPR[12]
*                                        + 0x60    GPR[13]
*                                        + 0x64    GPR[14]
*                                        + 0x68    GPR[15]
*                                        + 0x6C    GPR[16]
*                                        + 0x70    GPR[17]
*                                        + 0x74    GPR[18]
*                                        + 0x78    GPR[19]
*                                        + 0x7C    GPR[20]
*                                        + 0x80    GPR[21]
*                                        + 0x84    GPR[22]
*                                        + 0x88    GPR[23]
*                                        + 0x8C    GPR[24]
*                                        + 0x90    GPR[25]
*                                        + 0x94    GPR[26]
*                                        + 0x98    GPR[27]
*                                        + 0x9C    GPR[28]
*                                        + 0xA0    GPR[30]
*                                        + 0xA4    GPR[31]                       (HIGH Memory)
*********************************************************************************************************
*/

CPU_STK  *OSTaskStkInit (OS_TASK_PTR    p_task,
                         void          *p_arg,
                         CPU_STK       *p_stk_base,
                         CPU_STK       *p_stk_limit,
                         CPU_STK_SIZE   stk_size,
                         OS_OPT         opt)
{
    CPU_INT32U  *pstk;
    CPU_INT32U  sr_val;
    CPU_INT32U  gp_val;
#if defined (__mips_hard_float)       
    CPU_INT32U  fcsr_val;
#endif     


    (void)opt;                                 /* Prevent compiler warning for unused arguments        */              

    asm volatile("mfc0   %0,$12"   : "=r"(sr_val));
    sr_val |= 0x00000003;                      /* set the EXL and IE bit.                              */

    asm volatile("addi   %0,$28,0" : "=r"(gp_val));

#if defined (__mips_hard_float)     
    asm volatile("CFC1   %0,$f31"   : "=r"(fcsr_val));
#endif    
    pstk     = &p_stk_base[stk_size - 1u];

    if(!((unsigned int)pstk & (0x7)))
    {
        /* base address should be double word aligned */
        pstk--; 
    }
     pstk--;                                       /* Ensure that a free entry is being referenced         */
    *pstk--  = (CPU_INT32U)OS_TaskReturn;      /* GPR[31], catch a task that tries to return           */
    *pstk--  = (CPU_INT32U)0x30303030;         /* GPR[30]                                              */
    *pstk--  = gp_val;                         /* GPR[28]                                              */
    *pstk--  = (CPU_INT32U)0x27272727;         /* GPR[27]                                              */
    *pstk--  = (CPU_INT32U)0x26262626;         /* GPR[26]                                              */
    *pstk--  = (CPU_INT32U)0x25252525;         /* GPR[25]                                              */
    *pstk--  = (CPU_INT32U)0x24242424;         /* GPR[24]                                              */
    *pstk--  = (CPU_INT32U)0x23232323;         /* GPR[23]                                              */
    *pstk--  = (CPU_INT32U)0x22222222;         /* GPR[22]                                              */
    *pstk--  = (CPU_INT32U)0x21212121;         /* GPR[21]                                              */
    *pstk--  = (CPU_INT32U)0x20202020;         /* GPR[20]                                              */
    *pstk--  = (CPU_INT32U)0x19191919;         /* GPR[19]                                              */
    *pstk--  = (CPU_INT32U)0x18181818;         /* GPR[18]                                              */
    *pstk--  = (CPU_INT32U)0x17171717;         /* GPR[17]                                              */
    *pstk--  = (CPU_INT32U)0x16161616;         /* GPR[16]                                              */
    *pstk--  = (CPU_INT32U)0x15151515;         /* GPR[15]                                              */
    *pstk--  = (CPU_INT32U)0x14141414;         /* GPR[14]                                              */
    *pstk--  = (CPU_INT32U)0x13131313;         /* GPR[13]                                              */
    *pstk--  = (CPU_INT32U)0x12121212;         /* GPR[12]                                              */
    *pstk--  = (CPU_INT32U)0x11111111;         /* GPR[11]                                              */
    *pstk--  = (CPU_INT32U)0x10101010;         /* GPR[10]                                              */
    *pstk--  = (CPU_INT32U)0x09090909;         /* GPR[9]                                               */
    *pstk--  = (CPU_INT32U)0x08080808;         /* GPR[8]                                               */
    *pstk--  = (CPU_INT32U)0x07070707;         /* GPR[7]                                               */
    *pstk--  = (CPU_INT32U)0x06060606;         /* GPR[6]                                               */
    *pstk--  = (CPU_INT32U)0x05050505;         /* GPR[5]                                               */
    *pstk--  = (CPU_INT32U)0x04040404;         /* GPR[4]                                               */
    *pstk--  = (CPU_INT32U)0x03030303;         /* GPR[3]                                               */
    *pstk--  = (CPU_INT32U)0x02020202;         /* GPR[2]                                               */
    *pstk--  = (CPU_INT32U)0x01010101;         /* GPR[1]                                               */
#if defined (__mips_hard_float)
    *pstk--  = fcsr_val;                       /* FCSR                                                 */    
    *pstk--  = (CPU_INT32U)0x31313131;         /* FPR[31]                                              */
    *pstk--  = (CPU_INT32U)0x31313131;         /* FPR[31]                                              */
    *pstk--  = (CPU_INT32U)0x30303030;         /* FPR[30]                                              */
    *pstk--  = (CPU_INT32U)0x30303030;         /* FPR[30]                                              */
    *pstk--  = (CPU_INT32U)0x29292929;         /* FPR[29]                                              */
    *pstk--  = (CPU_INT32U)0x29292929;         /* FPR[29]                                              */
    *pstk--  = (CPU_INT32U)0x28282828;         /* FPR[28]                                              */
    *pstk--  = (CPU_INT32U)0x28282828;         /* FPR[28]                                              */
    *pstk--  = (CPU_INT32U)0x27272727;         /* FPR[27]                                              */
    *pstk--  = (CPU_INT32U)0x27272727;         /* FPR[27]                                              */
    *pstk--  = (CPU_INT32U)0x26262626;         /* FPR[26]                                              */
    *pstk--  = (CPU_INT32U)0x26262626;         /* FPR[26]                                              */
    *pstk--  = (CPU_INT32U)0x25252525;         /* FPR[25]                                              */
    *pstk--  = (CPU_INT32U)0x25252525;         /* FPR[25]                                              */
    *pstk--  = (CPU_INT32U)0x24242424;         /* FPR[24]                                              */
    *pstk--  = (CPU_INT32U)0x24242424;         /* FPR[24]                                              */
    *pstk--  = (CPU_INT32U)0x23232323;         /* FPR[23]                                              */
    *pstk--  = (CPU_INT32U)0x23232323;         /* FPR[23]                                              */
    *pstk--  = (CPU_INT32U)0x22222222;         /* FPR[22]                                              */
    *pstk--  = (CPU_INT32U)0x22222222;         /* FPR[22]                                              */
    *pstk--  = (CPU_INT32U)0x21212121;         /* FPR[21]                                              */
    *pstk--  = (CPU_INT32U)0x21212121;         /* FPR[21]                                              */
    *pstk--  = (CPU_INT32U)0x20202020;         /* FPR[20]                                              */
    *pstk--  = (CPU_INT32U)0x20202020;         /* FPR[20]                                              */
    *pstk--  = (CPU_INT32U)0x19191919;         /* FPR[19]                                              */
    *pstk--  = (CPU_INT32U)0x19191919;         /* FPR[19]                                              */
    *pstk--  = (CPU_INT32U)0x18181818;         /* FPR[18]                                              */
    *pstk--  = (CPU_INT32U)0x18181818;         /* FPR[18]                                              */
    *pstk--  = (CPU_INT32U)0x17171717;         /* FPR[17]                                              */
    *pstk--  = (CPU_INT32U)0x17171717;         /* FPR[17]                                              */
    *pstk--  = (CPU_INT32U)0x16161616;         /* FPR[16]                                              */
    *pstk--  = (CPU_INT32U)0x16161616;         /* FPR[16]                                              */
    *pstk--  = (CPU_INT32U)0x15151515;         /* FPR[15]                                              */
    *pstk--  = (CPU_INT32U)0x15151515;         /* FPR[15]                                              */
    *pstk--  = (CPU_INT32U)0x14141414;         /* FPR[14]                                              */
    *pstk--  = (CPU_INT32U)0x14141414;         /* FPR[14]                                              */
    *pstk--  = (CPU_INT32U)0x13131313;         /* FPR[13]                                              */
    *pstk--  = (CPU_INT32U)0x13131313;         /* FPR[13]                                              */
    *pstk--  = (CPU_INT32U)0x12121212;         /* FPR[12]                                              */
    *pstk--  = (CPU_INT32U)0x12121212;         /* FPR[12]                                              */
    *pstk--  = (CPU_INT32U)0x11111111;         /* FPR[11]                                              */
    *pstk--  = (CPU_INT32U)0x11111111;         /* FPR[11]                                              */
    *pstk--  = (CPU_INT32U)0x10101010;         /* FPR[10]                                              */
    *pstk--  = (CPU_INT32U)0x10101010;         /* FPR[10]                                              */
    *pstk--  = (CPU_INT32U)0x09090909;         /* FPR[9]                                               */
    *pstk--  = (CPU_INT32U)0x09090909;         /* FPR[9]                                               */
    *pstk--  = (CPU_INT32U)0x08080808;         /* FPR[8]                                               */
    *pstk--  = (CPU_INT32U)0x08080808;         /* FPR[8]                                               */
    *pstk--  = (CPU_INT32U)0x07070707;         /* FPR[7]                                               */
    *pstk--  = (CPU_INT32U)0x07070707;         /* FPR[7]                                               */
    *pstk--  = (CPU_INT32U)0x06060606;         /* FPR[6]                                               */
    *pstk--  = (CPU_INT32U)0x06060606;         /* FPR[6]                                               */
    *pstk--  = (CPU_INT32U)0x05050505;         /* FPR[5]                                               */
    *pstk--  = (CPU_INT32U)0x05050505;         /* FPR[5]                                               */
    *pstk--  = (CPU_INT32U)0x04040404;         /* FPR[4]                                               */
    *pstk--  = (CPU_INT32U)0x04040404;         /* FPR[4]                                               */
    *pstk--  = (CPU_INT32U)0x03030303;         /* FPR[3]                                               */
    *pstk--  = (CPU_INT32U)0x03030303;         /* FPR[3]                                               */
    *pstk--  = (CPU_INT32U)0x02020202;         /* FPR[2]                                               */    
    *pstk--  = (CPU_INT32U)0x02020202;         /* FPR[2]                                               */
    *pstk--  = (CPU_INT32U)0x01010101;         /* FPR[1]                                               */
    *pstk--  = (CPU_INT32U)0x01010101;         /* FPR[1]                                               */
    *pstk--  = (CPU_INT32U)0x00000000;         /* FPR[0]                                               */
    *pstk--  = (CPU_INT32U)0x00000000;         /* FPR[0]                                               */    
    
#endif    
    *pstk--  = (CPU_INT32U)0x00000000;         /* Special Purpose HI Register (ac3)                    */
    *pstk--  = (CPU_INT32U)0x00000000;         /* Special Purpose LO Register (ac3)                    */
    *pstk--  = (CPU_INT32U)0x00000000;         /* Special Purpose HI Register (ac2)                    */
    *pstk--  = (CPU_INT32U)0x00000000;         /* Special Purpose LO Register (ac2)                    */
    *pstk--  = (CPU_INT32U)0x00000000;         /* Special Purpose HI Register (ac1)                    */
    *pstk--  = (CPU_INT32U)0x00000000;         /* Special Purpose LO Register (ac1)                    */
    *pstk--  = (CPU_INT32U)0x00000000;         /* Special Purpose HI Register (ac0)                    */
    *pstk--  = (CPU_INT32U)0x00000000;         /* Special Purpose LO Register (ac0)                    */
    *pstk--  = (CPU_INT32U)0x00000000;         /* DSPControl Register                                  */
    *pstk--  = (CPU_INT32U)p_task;             /* EPC                                                   */
    *pstk--  = sr_val;                         /* SR                                                    */
    
    return ((CPU_STK *)pstk);                  /* Return new top of stack                              */
}

/*
************************************************************************************************************************
*                                                   TASK SWITCH HOOK
*
* Description: This function is called when a task switch is performed.  This allows you to perform other operations
*              during a context switch.
*
* Arguments  : none
*
* Note(s)    : 1) Interrupts are disabled during this call.
*              2) It is assumed that the global pointer 'OSTCBHighRdyPtr' points to the TCB of the task that will be
*                 'switched in' (i.e. the highest priority task) and, 'OSTCBCurPtr' points to the task being switched out
*                 (i.e. the preempted task).
************************************************************************************************************************
*/

void  OSTaskSwHook (void)
{
#if OS_CFG_TASK_PROFILE_EN > 0u
    CPU_TS  ts;
#endif
#ifdef  CPU_CFG_INT_DIS_MEAS_EN
    CPU_TS  int_dis_time;
#endif



#if OS_CFG_APP_HOOKS_EN > 0u
    if (OS_AppTaskSwHookPtr != (OS_APP_HOOK_VOID)0) {
        (*OS_AppTaskSwHookPtr)();
    }
#endif

#if OS_CFG_TASK_PROFILE_EN > 0u
    ts = OS_TS_GET();
    if (OSTCBCurPtr != OSTCBHighRdyPtr) {
        OSTCBCurPtr->CyclesDelta  = ts - OSTCBCurPtr->CyclesStart;
        OSTCBCurPtr->CyclesTotal += (OS_CYCLES)OSTCBCurPtr->CyclesDelta;
    }

    OSTCBHighRdyPtr->CyclesStart = ts;
#endif

#ifdef  CPU_CFG_INT_DIS_MEAS_EN
    int_dis_time = CPU_IntDisMeasMaxCurReset();             /* Keep track of per-task interrupt disable time          */
    if (OSTCBCurPtr->IntDisTimeMax < int_dis_time) {
        OSTCBCurPtr->IntDisTimeMax = int_dis_time;
    }
#endif

#if OS_CFG_SCHED_LOCK_TIME_MEAS_EN > 0u
    if (OSTCBCurPtr->SchedLockTimeMax < OSSchedLockTimeMaxCur) { /* Keep track of per-task scheduler lock time        */
        OSTCBCurPtr->SchedLockTimeMax = OSSchedLockTimeMaxCur;
        OSSchedLockTimeMaxCur         = (CPU_TS)0;               /* Reset the per-task value                          */
    }
#endif
}   

/*$PAGE*/
/*
************************************************************************************************************************
*                                                      TICK HOOK
*
* Description: This function is called every tick.
*
* Arguments  : none
*
* Note(s)    : 1) This function is assumed to be called from the Tick ISR.
************************************************************************************************************************
*/

void  OSTimeTickHook (void)
{
#if OS_CFG_APP_HOOKS_EN > 0u
    if (OS_AppTimeTickHookPtr != (OS_APP_HOOK_VOID)0) {
        (*OS_AppTimeTickHookPtr)();
    }
#endif
}

/*$PAGE*/
/*
************************************************************************************************************************
*                                                   TASK RETURN HOOK
*
* Description: This function is called if a task accidentally returns.  In other words, a task should either be an
*              infinite loop or delete itself when done.
*
* Arguments  : p_tcb   is a pointer to the task control block of the task that is returning.
*
* Note(s)    : none
************************************************************************************************************************
*/

void  OSTaskReturnHook (OS_TCB  *p_tcb)
{
#if OS_CFG_APP_HOOKS_EN > 0u
    if (OS_AppTaskReturnHookPtr != (OS_APP_HOOK_TCB)0) {
        (*OS_AppTaskReturnHookPtr)(p_tcb);
    }
#else
    (void)p_tcb;
#endif
}

