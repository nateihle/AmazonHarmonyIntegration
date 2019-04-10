<#--
/*******************************************************************************
  MPLAB Harmony Test Harness Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    system_config.h.ftl

  Summary:
    MPLAB Harmony Test Harness Library Module Freemarker Template File for
    system_config.h

  Description:
    This file defines the template for the system configuration entries 
    inserted into the system_config.h file when the test harness library
    is used.
*******************************************************************************/

/*******************************************************************************
Copyright (c) 2014-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
-->
<#if CONFIG_USE_TEST_HARNESS == true>
/******************************************************************************
  Test Harness Configuration Options
  
  <editor-fold defaultstate="collapsed"
   desc="Test Harness Configuration Options">
*/

#define TEST_HARNESS_MAX_NUM_TASKS                      ${CONFIG_TEST_HARNESS_MAX_NUM_TASKS}
#define TEST_HARNESS_MAX_NUM_INSTANCES_PER_LIBRARY      ${CONFIG_TEST_HARNESS_MAX_NUM_INSTANCES_PER_LIBRARY}
#define TEST_HARNESS_MAX_NUM_TASKS_PER_LIBRARY          ${CONFIG_TEST_HARNESS_MAX_NUM_TASKS_PER_LIBRARY}

/* Test Timeout Timer configuration Options. */
#define TEST_TIMER_ID                       ${CONFIG_TEST_TIMER_ID}
#define TEST_TIMER_INTERRUPT_SOURCE         ${CONFIG_TEST_TIMER_INTERRUPT_SOURCE}
#define TEST_TIMER_CLOCK_SOURCE             ${CONFIG_TEST_TIMER_CLOCK_SOURCE}
#define TEST_TIMER_CLOCK_PRESCALER          ${CONFIG_TEST_TIMER_CLOCK_PRESCALER}
#define TEST_TIMER_INCREMENT_PERIOD         ${CONFIG_TEST_TIMER_INCREMENT_PERIOD}
#define TEST_TIMER_MS_PER_INCREMENT         ${CONFIG_TEST_TIMER_MS_PER_INCREMENT}
#define TEST_TIMER_MS_TIMEOUT               ${CONFIG_TEST_TIMER_MS_TIMEOUT}
#define TEST_TIMER_INTERRUPT_VECTOR         ${CONFIG_TEST_TIMER_INTERRUPT_VECTOR}
#define TEST_TIMER_INTERRUPT_PRIORITY       ${CONFIG_TEST_TIMER_INTERRUPT_PRIORITY}
#define TEST_TIMER_INTERRUPT_SUBPRIORITY    ${CONFIG_TEST_TIMER_INTERRUPT_SUBPRIORITY}

/* Test OSAL Idle Sleep Time Configuration Options. */
#define TEST_IDLE_SLEEP_MS_LIBRARY          ${CONFIG_TEST_IDLE_SLEEP_MS_LIBRARY}
#define TEST_IDLE_SLEEP_MS                  ${CONFIG_TEST_IDLE_SLEEP_MS}
// </editor-fold>

</#if>
<#if CONFIG_USE_SAMPLE_FUNC_TEST == true>
<#include "/framework/test/templates/system_config.h.sample_func.ftl">
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
