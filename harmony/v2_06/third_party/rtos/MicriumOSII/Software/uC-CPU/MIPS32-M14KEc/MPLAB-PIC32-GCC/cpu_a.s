/*
*********************************************************************************************************
*                                               uC/CPU
*                                    CPU CONFIGURATION & PORT LAYER
*
*                          (c) Copyright 1999-2005; Micrium, Inc.; Weston, FL
*
*                   All rights reserved.  Protected by international copyright laws.
*                   Knowledge of the source code may not be used to write a similar
*                   product.  This file may only be used in accordance with a license
*                   and should not be redistributed in any way.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             MIPS32 4K
*                                               MPLAB  
*
* Filename      : cpu_a.s
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                           PUBLIC FUNCTIONS
*********************************************************************************************************
*/

    .global  CPU_SR_Save
    .global  CPU_SR_Restore

/*
*********************************************************************************************************
*                                      CODE GENERATION DIRECTIVES
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
/*
**********************************************************************************************************
*                                      CRITICAL SECTION FUNCTIONS
*
* Description : Disable/Enable interrupts by preserving the state of interrupts.  Generally speaking, the 
*               state of the interrupt disable flag is stored in the local variable 'cpu_sr' & interrupts
*               are then disabled ('cpu_sr' is allocated in all functions that need to disable interrupts).
*               The previous interrupt state is restored by copying 'cpu_sr' into the CPU's status register.
*
* Prototypes  : CPU_SR  CPU_SR_Save(void);
*               void    CPU_SR_Restore(CPU_SR cpu_sr);
*
*
* Note(s)     : (1) These functions are used in general like this:
*
*                   void  Task (void *p_arg)
*                   {
*                                                             
*                   #if (CPU_CFG_CRITICAL_METHOD == CPU_CRITICAL_METHOD_STATUS_LOCAL)
*                       CPU_SR  cpu_sr;
*                   #endif
*
*                            :
*                            :
*                       CPU_CRITICAL_ENTER();                
*                            :
*                            :
*                       CPU_CRITICAL_EXIT();              
*                            :
*                            :
*                   }
**********************************************************************************************************
*/

    .ent CPU_SR_Save
CPU_SR_Save:
                     
    jr    $31
    di    $2                                   /* Disable interrupts, and move the old value of the... */
                                               /* ...Status register into v0 ($2)                      */
    .end CPU_SR_Save


    .ent CPU_SR_Restore
CPU_SR_Restore:

    jr    $31
    mtc0  $4, $12, 0                           /* Restore the status register to its previous state    */

    .end CPU_SR_Restore




