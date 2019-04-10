/*
************************************************************************************************************************
*                                                      uC/OS-II
*                                                 The Real-Time Kernel
*
*                                  (c) Copyright 1992-2007, Jean J. Labrosse, Weston, FL
*                          All rights reserved.  Protected by international copyright laws.
*
*                                                  CONFIGURATION  FILE
*
* File    : OS_CFG.H
* By      : JJL
* Version : V3.01.1
*
* LICENSING TERMS:
* ---------------
*   uC/OS-II is provided in source form for FREE evaluation, for educational use or for peaceful research.
* If you plan on using  uC/OS-II  in a commercial product you need to contact Micriµm to properly license
* its use in your product. We provide ALL the source code for your convenience and to help you experience
* uC/OS-II.   The fact that the  source is provided does  NOT  mean that you can use it without  paying a
* licensing fee.
************************************************************************************************************************
*/

#ifndef OS_CFG_H
#define OS_CFG_H

                                             /* ---------------------------- MISCELLANEOUS -------------------------- */
#define OS_CPU_HOOKS_EN             	<#if CONFIG_UCOSII_CPU_HOOKS_EN == true>1<#else>0</#if>u /* Enable (1) or Disable (0) application specific hooks   */
#define OS_APP_HOOKS_EN             	<#if CONFIG_UCOSII_APP_HOOKS_EN == true>1<#else>0</#if>u /* Enable (1) or Disable (0) application specific hooks   */
#define OS_ARG_CHK_EN               	<#if CONFIG_UCOSII_ARG_CHK_EN == true>1<#else>0</#if>u   /* Enable (1) or Disable (0) argument checking  */
#define OS_DEBUG_EN               	<#if CONFIG_UCOSII_DEBUG_EN == true>1<#else>0</#if>u   /* Enable(1) debug variables                                    */
#define OS_EVENT_MULTI_EN               <#if CONFIG_UCOSII_EVENT_MULTI_EN == true>1<#else>0</#if>u    /* Include code for OSEventPendMulti()                   */
#define OS_EVENT_NAME_SIZE              <#if CONFIG_UCOSII_EVENT_MULTI_EN == true>${CONFIG_UCOSII_EVENT_NAME_SIZE}<#else>0</#if>u    /* Determine the size of the name of a Sem, Mutex, Mbox or Q    */
#define OS_LOWEST_PRIO                 	${CONFIG_UCOSII_LOWEST_PRIO}u   /* specifies the lowest task priority that you intend to use in your application */	
#define OS_MAX_EVENTS                 	${CONFIG_UCOSII_MAX_EVENTS}u   	/* specifies the maximum number of event control blocks that can be allocated */
#define OS_MAX_FLAGS                 	${CONFIG_UCOSII_MAX_FLAGS}u   	/* specifies the maximum number of event flags that you need in your application */
#define OS_MAX_MEM_PART                 ${CONFIG_UCOSII_MAX_MEM_PART}u  /* specifies the maximum number of memory partitions that can be managed by the memory-	partition manager */
#define OS_MAX_QS                 	${CONFIG_UCOSII_QS}u   		/* specifies the maximum number of message queues that your application can create */
#define OS_MAX_TASKS                 	${CONFIG_UCOSII_TASKS}u   	/* specifies the maximum number of application tasks that can exist in your application */
#define OS_TASK_IDLE_STK_SIZE           ${CONFIG_UCOSII_TASK_IDLE_STK_SIZE}u   	/* specifies the size of the μC/OS-II idle-task stack */
#define OS_TASK_STAT_STK_SIZE		${CONFIG_UCOSII_TASK_STAT_STK_SIZE}u   	/* specifies the size of the μC/OS-II statistic-task stack */
#define OS_SCHED_LOCK_EN		<#if CONFIG_UCOSII_SCHED_LOCK_EN == true>1<#else>0</#if>u /* enables or disables code generation for the two functions
OSSchedLock() and OSSchedUnlock() */
#define OS_TICK_STEP_EN                 <#if CONFIG_UCOSII_TICK_STEP_EN == true>1<#else>0</#if>u    /* Enable tick stepping feature for uC/OS-View                  */
#define OS_TICKS_PER_SEC                ${CONFIG_UCOSII_OS_TICKS_PER_SEC}u   	/* specifies the rate at which you call OSTimeTick() */


                                             /* ----------------------------- Message Mailboxes --------------------------- */
#define OS_MBOX_EN                  	<#if CONFIG_UCOSII_MBOX_EN == true>1<#else>0</#if>u   /* Enable (1) or Disable (0) code generation for message-mailbox services */
#define OS_MBOX_ACCEPT_EN               <#if CONFIG_UCOSII_MBOX_ACCEPT_EN == true>1<#else>0</#if>u   /* enables or disables the code generation of OSMboxAccept() */
#define OS_MBOX_DEL_EN         		<#if CONFIG_UCOSII_MBOX_DEL_EN == true>1<#else>0</#if>u   /* enables or disables the code generation of the function OSMboxDel() */
#define OS_MBOX_POST_EN		        <#if CONFIG_UCOSII_MBOX_POST_EN == true>1<#else>0</#if>u   /* enables or disables the code generation of the function OSMboxPost() */
#define OS_MBOX_POST_OPT_EN		<#if CONFIG_UCOSII_MBOX_POST_OPT_EN == true>1<#else>0</#if>u   /* enables or disables the code generation of the function OSMboxPostOpt() */
#define OS_MBOX_QUERY_EN		<#if CONFIG_UCOSII_MBOX_QUERY_EN == true>1<#else>0</#if>u   /* enables or disables the code generation of the function OSMboxQuery() */
#define OS_MBOX_PEND_ABORT_EN		<#if CONFIG_UCOSII_MBOX_QUERY_EN == true>1<#else>0</#if>u   /* enables or disables the code generation of the function OSMboxPendAbort() */

                                             /* ----------------------------- Event Flags --------------------------- */
#define OS_FLAG_EN                  	<#if CONFIG_UCOSII_FLAG_EN == true>1<#else>0</#if>u   /* Enable (1) or Disable (0) code generation for EVENT FLAGS  */
#define OS_FLAG_WAIT_CLR_EN             <#if CONFIG_UCOSII_FLAG_WAIT_CLR_EN == true>1<#else>0</#if>u   /* enables or disables the code generation used to wait for event flags                                     */
#define OS_FLAG_ACCEPT_EN         	<#if CONFIG_UCOSII_FLAG_ACCEPT_EN == true>1<#else>0</#if>u   /* enables or disables the code generation of the function OSFlagAccept()  */
#define OS_FLAG_DEL_EN		        <#if CONFIG_UCOSII_FLAG_DEL_EN == true>1<#else>0</#if>u   /* enables or disables the code generation of the function OSFlagDel() */
#define OS_FLAG_QUERY_EN		<#if CONFIG_UCOSII_FLAG_QUERY_EN == true>1<#else>0</#if>u   /* enables or disables the code generation of the function OSFlagQuery() */
#define OS_FLAGS_NBITS                  <#if CONFIG_UCOSII_FLAG_EN == true>16<#else>0</#if>   /* Size in #bits of OS_FLAGS data type (8, 16 or 32)  */
#define OS_FLAG_NAME_SIZE                  <#if CONFIG_UCOSII_FLAG_EN == true>16<#else>0</#if>   /* Determine the size of the name of an event flag group  */

                                             /* -------------------------- MEMORY MANAGEMENT ------------------------ */
#define OS_MEM_EN                   	<#if CONFIG_UCOSII_MEM_EN == true>1<#else>0</#if>u   /* Enable (1) or Disable (0) code generation for MEMORY MANAGER    */
#define OS_MEM_NAME_SIZE        	<#if CONFIG_UCOSII_MEM_EN == true>16<#else>0</#if>u  /*     Determine the size of a memory partition name            */
#define OS_MEM_QUERY_EN             	<#if CONFIG_UCOSII_MEM_QUERY_EN == true>1<#else>0</#if>u   /* Enable (1) or Disable (0) code generation OFof function OSMemQuery() */
                                             /* --------------------- MUTUAL EXCLUSION SEMAPHORES ------------------- */
#define OS_MUTEX_EN                 	<#if CONFIG_UCOSII_MUTEX_EN == true>1<#else>0</#if>u   /* Enable (1) or Disable (0) code generation for MUTEX       */
#define OS_MUTEX_ACCEPT_EN          	<#if CONFIG_UCOSII_MUTEX_ACCEPT_EN == true>1<#else>0</#if>u   /* Enable (1) or Disable (0) code generation for OSMutexAccept()                   */
#define OS_MUTEX_DEL_EN            	<#if CONFIG_UCOSII_MUTEX_DEL_EN == true>1<#else>0</#if>u   /*     Include code for OSMutexDel()     */
#define OS_MUTEX_QUERY_EN               <#if CONFIG_UCOSII_MUTEX_QUERY_EN == true>1<#else>0</#if>u   /*     Include code for OSMutexQuery()  */

                                             /* --------------------------- MESSAGE QUEUES -------------------------- */
#define OS_Q_EN                    	<#if CONFIG_UCOSII_Q_EN == true>1<#else>0</#if>u   /* Enable (1) or Disable (0) code generation for QUEUES    */
#define OS_Q_DEL_EN                     <#if CONFIG_UCOSII_Q_DEL_EN == true>1<#else>0</#if>u   /*     Include code for OSQDel()   */
#define OS_Q_ACCEPT_EN                  <#if CONFIG_UCOSII_Q_ACCEPT_EN == true>1<#else>0</#if>u   /*     Include code for OSQAccept()   */
#define OS_Q_FLUSH_EN                   <#if CONFIG_UCOSII_Q_FLUSH_EN == true>1<#else>0</#if>u   /*     Include code for OSQFlush()     */
#define OS_Q_POST_EN                    <#if CONFIG_UCOSII_Q_POST_EN == true>1<#else>0</#if>u   /*     Include code for OSQPost()    */
#define OS_Q_POST_FRONT_EN              <#if CONFIG_UCOSII_Q_POST_FRONT_EN == true>1<#else>0</#if>u   /*   Include code for OSQPostFront()    */
#define OS_Q_POST_OPT_EN                <#if CONFIG_UCOSII_Q_POST_OPT_EN == true>1<#else>0</#if>u   /*   Include code for OSQPostOpt()   */
#define OS_Q_QUERY_EN                   <#if CONFIG_UCOSII_Q_QUERY_EN == true>1<#else>0</#if>u   /*   Include code for OSQQuery()   */
#define OS_Q_PEND_ABORT_EN              <#if CONFIG_UCOSII_Q_PEND_ABORT_EN == true>1<#else>0</#if>u   /*   Include code for OSQPendAbort()   */

                                             /* ----------------------------- SEMAPHORES ---------------------------- */
#define OS_SEM_EN                   	<#if CONFIG_UCOSII_SEM_EN == true>1<#else>0</#if>u   /* Enable (1) or Disable (0) code generation for SEMAPHORES  */
#define OS_SEM_DEL_EN                   <#if CONFIG_UCOSII_SEM_DEL_EN == true>1<#else>0</#if>u   /*    Include code for OSSemDel()   */
#define OS_SEM_ACCEPT_EN                <#if CONFIG_UCOSII_SEM_ACCEPT_EN == true>1<#else>0</#if>u   /*    Include code for OSSemAccept()  */  
#define OS_SEM_QUERY_EN                 <#if CONFIG_UCOSII_SEM_QUERY_EN == true>1<#else>0</#if>u   /*    Include code for OSSemQuery() */
#define OS_SEM_PEND_ABORT_EN            <#if CONFIG_UCOSII_SEM_QUERY_EN == true>1<#else>0</#if>u   /*    Include code for OSSemPendAbort() */
#define OS_SEM_SET_EN            	<#if CONFIG_UCOSII_SEM_QUERY_EN == true>1<#else>0</#if>u   /*    Include code for OSSemSet() */
                                  
                                             /* -------------------------- TASK MANAGEMENT -------------------------- */
#define OS_TASK_CHANGE_PRIO_EN         <#if CONFIG_UCOSII_TASK_CHANGE_PRIO_EN == true>1<#else>0</#if>u   /* Include code for OSTaskChangePrio()  */
#define OS_TASK_CREATE_EN      	       <#if CONFIG_UCOSII_TASK_CREATE_EN == true>1<#else>0</#if>u   /* Include code for OSTaskCreate()  */
#define OS_TASK_CREATE_EXT_EN          <#if CONFIG_UCOSII_TASK_CREATE_EXT_EN == true>1<#else>0</#if>u   /* Include code for OSTaskCreateExt() */
#define OS_TASK_STAT_EN                <#if CONFIG_UCOSII_TASK_CREATE_EN == true>1<#else>0</#if>u /*     Enable (1) or Disable(0) the statistics task             */
#define OS_TASK_STAT_STK_CHK_EN        <#if CONFIG_UCOSII_TASK_STAT_STK_CHK_EN == true>1<#else>0</#if>u /*     Check task stacks from statistic task                    */
#define OS_TASK_DEL_EN                 <#if CONFIG_UCOSII_TASK_DEL_EN == true>1<#else>0</#if>u   /* Include code for OSTaskDel()  */
#define OS_TASK_NAME_SIZE              <#if CONFIG_UCOSII_TASK_CREATE_EN == true>${CONFIG_UCOSII_TASK_NAME_SIZE}<#else>0</#if>u   /*     Determine the size of a task name                        */
#define OS_TASK_PROFILE_EN             <#if CONFIG_UCOSII_TASK_PROFILE_EN == true>1<#else>0</#if>u    /*     Include variables in OS_TCB for profiling               */
#define OS_TASK_SUSPEND_EN             <#if CONFIG_UCOSII_TASK_SUSPEND_EN == true>1<#else>0</#if>u   /* Include code for OSTaskSuspend() and OSTaskResume()   */
#define OS_TASK_QUERY_EN               <#if CONFIG_UCOSII_TASK_QUERY_EN == true>1<#else>0</#if>u   /* Include code for OSTaskQuery()                   */
#define OS_TASK_SW_HOOK_EN             <#if CONFIG_UCOSII_TASK_SW_HOOK_EN == true>1<#else>0</#if>u     /*     Include code for OSTaskSwHook()                          */

                                             /* -------------------------- TIME MANAGEMENT -------------------------- */
#define OS_TIME_DLY_HMSM_EN           <#if CONFIG_UCOSII_TIME_DLY_HMSM_EN == true>1<#else>0</#if>u   /*     Include code for OSTimeDlyHMSM() */
#define OS_TIME_DLY_RESUME_EN         <#if CONFIG_UCOSII_TIME_DLY_RESUME_EN == true>1<#else>0</#if>u   /*     Include code for OSTimeDlyResume()*/
#define OS_TIME_GET_SET_EN            <#if CONFIG_UCOSII_TIME_GET_SET_EN == true>1<#else>0</#if>u   /*     Include code for OSTimeGet() and OSTimeSet()  */
#define OS_TIME_TICK_HOOK_EN          <#if CONFIG_UCOSII_TIME_TICK_HOOK_EN == true>1<#else>0</#if>u     /*     Include code for OSTimeTickHook()                        */

                                             /* -------------------------- TIMER MANAGEMENT -------------------------- */
#define OS_TMR_EN           	      <#if CONFIG_UCOSII_TMR_EN == true>1<#else>0</#if>u   /*     Include TIMERS */
#define OS_TASK_TMR_STK_SIZE          <#if CONFIG_UCOSII_TMR_EN == true>${CONFIG_UCOSII_TASK_TMR_STK_SIZE}<#else>0</#if>u   /*     Timer task stack size */
#define OS_TMR_CFG_MAX                <#if CONFIG_UCOSII_TMR_EN == true>${CONFIG_UCOSII_TMR_CFG_MAX}<#else>0</#if>u   /*     Maximum number of timers  */
#define OS_TMR_CFG_NAME_SIZE          <#if CONFIG_UCOSII_TMR_EN == true>${CONFIG_UCOSII_TMR_CFG_NAME_SIZE}<#else>0</#if>u   /*     Determine the size of a timer name  */
#define OS_TMR_CFG_WHEEL_SIZE         <#if CONFIG_UCOSII_TMR_EN == true>${CONFIG_UCOSII_TMR_CFG_WHEEL_SIZE}<#else>0</#if>u   /*     Size of timer wheel (#Spokes) */
#define OS_TMR_CFG_TICKS_PER_SEC      <#if CONFIG_UCOSII_TMR_EN == true>${CONFIG_UCOSII_TMR_CFG_TICKS_PER_SEC}<#else>0</#if>u   /*     Rate at which timer management task runs (Hz)  */


#endif
