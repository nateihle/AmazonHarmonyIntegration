
<#macro RTOS_TASK_CREATE RTOS_NAME TASK_FUNC_NAME TASK_NAME TASK_PRI TASK_STK_SZ>
<#if (RTOS_NAME == "FreeRTOS") || (RTOS_NAME == "OpenRTOS_V8.x.x")>
    /* Create OS Thread for ${TASK_NAME}. */
    xTaskCreate((TaskFunction_t) ${TASK_FUNC_NAME},
                "${TASK_NAME}",
                ${TASK_STK_SZ}, NULL, ${TASK_PRI}, NULL);

</#if>
<#if RTOS_NAME == "uC/OS-III">
    OSTaskCreate((OS_TCB      *)&${TASK_FUNC_NAME}_TCB,
                (CPU_CHAR    *)"${TASK_NAME}",
                (OS_TASK_PTR  )${TASK_FUNC_NAME},
                (void        *)0,
                (OS_PRIO      )${TASK_PRI},
                (CPU_STK     *)&${TASK_FUNC_NAME}Stk[0],
                (CPU_STK_SIZE )0u,
                (CPU_STK_SIZE )${TASK_STK_SZ},
                (OS_MSG_QTY   )0u,
                (OS_TICK      )0u,
                (void        *)0,
                (OS_OPT       )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                (OS_ERR      *)&os_err);
</#if>
<#if RTOS_NAME == "uC/OS-II">
    OSTaskCreateExt(&${TASK_FUNC_NAME},
                (void        *)0,
                (OS_STK      *)&${TASK_FUNC_NAME}Stk[${TASK_STK_SZ}],
			       ${TASK_PRI},
                	       ${TASK_PRI},
		(OS_STK      *)&${TASK_FUNC_NAME}Stk[0],
                	       ${TASK_STK_SZ},
                (void        *)0,
                	      (OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR));
</#if>
<#if RTOS_NAME == "ThreadX">
    /* Allocate the stack for system and application threads */
    tx_byte_allocate(&_sys_byte_pool_0, (VOID **) &_sys_tx_thread_stk_ptr, 
        ${TASK_STK_SZ},TX_NO_WAIT);

    /* create the RTOS thread*/
    tx_thread_create(&${TASK_FUNC_NAME}_TCB,"${TASK_NAME}",${TASK_FUNC_NAME},0,
        _sys_tx_thread_stk_ptr,${TASK_STK_SZ},${TASK_PRI}, ${TASK_PRI},
        TX_NO_TIME_SLICE,TX_AUTO_START);
</#if>
<#if RTOS_NAME == "embOS">
    /*use macro for creating tasks, user defines which task create func is used in RTOS.h*/
    OS_CREATETASK(&${TASK_FUNC_NAME}_TCB, "${TASK_NAME}",${TASK_FUNC_NAME},${TASK_PRI}, ${TASK_FUNC_NAME}Stk);
</#if>
</#macro>

<#macro RTOS_TASK_DELAY RTOS_NAME TASK_DELAY>
<#if CONFIG_FW_SYS_TASK_USE_DELAY>
<#if (RTOS_NAME == "FreeRTOS") || (CONFIG_3RDPARTY_RTOS_USED == "OpenRTOS_V8.x.x")>
<#if CONFIG_FREERTOS_ENABLE_BACKWARD_COMPATIBILITY == true>
        vTaskDelay(${TASK_DELAY} / portTICK_RATE_MS);
<#else>
        vTaskDelay(${TASK_DELAY} / portTICK_PERIOD_MS);
</#if>
</#if>
<#if RTOS_NAME == "uC/OS-III">
        OSTimeDly (${TASK_DELAY}, OS_OPT_TIME_DLY, &os_err);
</#if>
<#if RTOS_NAME == "uC/OS-II">
        OSTimeDly (${TASK_DELAY});
</#if>
<#if RTOS_NAME == "ThreadX">
        tx_thread_sleep(${TASK_DELAY});
</#if>
<#if RTOS_NAME == "embOS">
        OS_Delay(${TASK_DELAY});
</#if>
</#if>
</#macro>

<#macro RTOS_ISR VECTOR NAME PRIORITY>
<#if (CONFIG_3RDPARTY_RTOS_USED == "FreeRTOS") || (CONFIG_3RDPARTY_RTOS_USED == "OpenRTOS_V8.x.x")>
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32WK == true>
   .extern  IntHandler${NAME}

   .section	.vector_${VECTOR},code, keep
   .equ     __vector_dispatch_${VECTOR}, IntVector${NAME}
   .global  __vector_dispatch_${VECTOR}
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVector${NAME}

IntVector${NAME}:
    portSAVE_CONTEXT
    la    s6,  IntHandler${NAME}
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	IntVector${NAME}
<#else>
   .extern  IntHandler${NAME}

   .section	.vector_${VECTOR},code, keep
   .equ     __vector_dispatch_${VECTOR}, IntVector${NAME}
   .global  __vector_dispatch_${VECTOR}
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVector${NAME}

IntVector${NAME}:
    la    $26,  _IntHandler${NAME}         
    jr    $26
    nop
	.end	IntVector${NAME}

   .section	.${NAME}_vector_text, code, keep
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _IntHandler${NAME}

_IntHandler${NAME}:
    portSAVE_CONTEXT
    la    s6,  IntHandler${NAME}
    jalr  s6
    nop
    portRESTORE_CONTEXT
    .end	_IntHandler${NAME}
</#if>
</#if>

<#if CONFIG_3RDPARTY_RTOS_USED == "uC/OS-III">
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32WK == true>
   .extern  IntHandler${NAME}

   .section	.vector_${VECTOR},code, keep
   .equ     __vector_dispatch_${VECTOR}, IntVector${NAME}
   .global  __vector_dispatch_${VECTOR}
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
 
   .ent IntVector${NAME}
IntVector${NAME}:
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

    la    $8, IntHandler${NAME}
    jalr  $8
    nop

    la    $8,  OSIntExit                      /* Call OSIntExit()                                     */
    jalr  $8
    nop

    restoreCPU_CONTEXT

    .end IntVector${NAME}
<#else>
   .extern  IntHandler${NAME}

   .section	.vector_${VECTOR},code, keep
   .equ     __vector_dispatch_${VECTOR}, IntVector${NAME}
   .global  __vector_dispatch_${VECTOR}
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  IntVector${NAME}

IntVector${NAME}:
    la    $26,  _IntHandler${NAME}         
    jr    $26
    nop
	.end	IntVector${NAME}

   .section	.${NAME}_vector_text, code, keep
   .set     nomicromips
   .set     noreorder
   .set     nomips16
   .set     noat
   .ent  _IntHandler${NAME}

_IntHandler${NAME}:
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

    la    $8, IntHandler${NAME}
    jalr  $8
    nop

    la    $8,  OSIntExit                      /* Call OSIntExit()                                     */
    jalr  $8
    nop

    restoreCPU_CONTEXT

    .end	_IntHandler${NAME}
</#if>
</#if>
<#if CONFIG_3RDPARTY_RTOS_USED == "embOS">
<#if PRIORITY == "INT_DISABLE_INTERRUPT">
<#assign INT_PRIO = "">
</#if>
<#if PRIORITY == "INT_PRIORITY_LEVEL1">
<#assign INT_PRIO = "OS_INT_PRIORITY_1">
</#if>
<#if PRIORITY == "INT_PRIORITY_LEVEL2">
<#assign INT_PRIO = "OS_INT_PRIORITY_2">
</#if>
<#if PRIORITY == "INT_PRIORITY_LEVEL3">
<#assign INT_PRIO = "OS_INT_PRIORITY_3">
</#if>
<#if PRIORITY == "INT_PRIORITY_LEVEL4">
<#assign INT_PRIO = "OS_INT_PRIORITY_4">
</#if>
<#if PRIORITY == "INT_PRIORITY_LEVEL5">
<#assign INT_PRIO = "OS_INT_PRIORITY_5">
</#if>
<#if PRIORITY == "INT_PRIORITY_LEVEL6">
<#assign INT_PRIO = "OS_INT_PRIORITY_6">
</#if>
<#if PRIORITY == "INT_PRIORITY_LEVEL7">
<#assign INT_PRIO = "OS_INT_PRIORITY_7">
</#if>
.global  IntHandler${NAME}_ISR
.extern  IntHandler${NAME}
   
IntHandler${NAME}_ISR:
   OS_CALL_ISR IntHandler${NAME} ${INT_PRIO}
</#if><#-- end of embOS interrupts -->

</#macro>
