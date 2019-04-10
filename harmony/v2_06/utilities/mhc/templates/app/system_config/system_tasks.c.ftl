/*******************************************************************************
 System Tasks File

  File Name:
    system_tasks.c

  Summary:
    This file contains source code necessary to maintain system's polled state
    machines.

  Description:
    This file contains source code necessary to maintain system's polled state
    machines.  It implements the "SYS_Tasks" function that calls the individual
    "Tasks" functions for all polled MPLAB Harmony modules in the system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    polled in the system.  These handles are passed into the individual module
    "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

<#include "/utilities/mhc/templates/freemarker_functions.ftl">
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"


// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/

void SYS_Tasks ( void )
{
    /* Maintain system services */
<#if LIST_SYSTEM_TASKS_C_CALL_SYSTEM_TASKS?has_content>
<@mhc_expand_list list=LIST_SYSTEM_TASKS_C_CALL_SYSTEM_TASKS/>
</#if>

    /* Maintain Device Drivers */
<#if LIST_SYSTEM_TASKS_C_CALL_DRIVER_TASKS?has_content>
<@mhc_expand_list list=LIST_SYSTEM_TASKS_C_CALL_DRIVER_TASKS/>
</#if>

    /* Maintain Middleware & Other Libraries */
<#if LIST_SYSTEM_TASKS_C_CALL_LIB_TASKS?has_content>
<@mhc_expand_list list=LIST_SYSTEM_TASKS_C_CALL_LIB_TASKS/>
</#if>

    /* Maintain the application's state machine. */
<#if CONFIG_APP_IDX_0?has_content>
    ${CONFIG_APP_NAME_0?upper_case}_Tasks();
<#else>
    App_Tasks();
</#if>
<#if CONFIG_APP_IDX_1?has_content>
<#if CONFIG_APP_IDX_1 == true>
    ${CONFIG_APP_NAME_1?upper_case}_Tasks();
</#if>
<#if CONFIG_APP_IDX_2 == true>
    ${CONFIG_APP_NAME_2?upper_case}_Tasks();
</#if>
<#if CONFIG_APP_IDX_3 == true>
    ${CONFIG_APP_NAME_3?upper_case}_Tasks();
</#if>
<#if CONFIG_APP_IDX_4 == true>
    ${CONFIG_APP_NAME_4?upper_case}_Tasks();
</#if>
<#if CONFIG_APP_IDX_5 == true>
    ${CONFIG_APP_NAME_5?upper_case}_Tasks();
</#if>
<#if CONFIG_APP_IDX_6 == true>
    ${CONFIG_APP_NAME_6?upper_case}_Tasks();
</#if>
<#if CONFIG_APP_IDX_7 == true>
    ${CONFIG_APP_NAME_7?upper_case}_Tasks();
</#if>
<#if CONFIG_APP_IDX_8 == true>
    ${CONFIG_APP_NAME_8?upper_case}_Tasks();
</#if>
<#if CONFIG_APP_IDX_9 == true>
    ${CONFIG_APP_NAME_9?upper_case}_Tasks();
</#if>
</#if>
}


/*******************************************************************************
 End of File
 */

