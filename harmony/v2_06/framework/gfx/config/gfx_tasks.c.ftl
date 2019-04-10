<#--
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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
 -->
<#if !(GFX_STACK_TASKS?has_content)>
<#assign GFX_STACK_TASKS = "TASK_CALL_NO_RTOS">
</#if>
<#if (GFX_STACK_TASKS == "PROTO") && (CONFIG_GFX_STACK_RTOS == "Standalone")>
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void _GFX_STACK_Tasks(ULONG thread_input);
<#else>
void _GFX_STACK_Tasks(void);
</#if>
</#if>
<#if GFX_STACK_TASKS == "CREATE_TASK">
 <#if CONFIG_GFX_STACK_RTOS == "Standalone">
 
    /* Create task for System Timer state machine*/
<@RTOS_TASK_CREATE RTOS_NAME=CONFIG_3RDPARTY_RTOS_USED TASK_FUNC_NAME="_GFX_STACK_Tasks" TASK_NAME="GFX_STACK Tasks" TASK_PRI=CONFIG_GFX_STACK_RTOS_TASK_PRIORITY TASK_STK_SZ=CONFIG_GFX_STACK_RTOS_TASK_SIZE/>
 </#if>
</#if>
<#if (GFX_STACK_TASKS == "TASK_CALL_NO_RTOS") || (GFX_STACK_TASKS == "TASK_CALL" && CONFIG_GFX_STACK_RTOS != "Standalone")>
	// update the GFX Abstraction Layer
	GFX_Update();
</#if>
<#if GFX_STACK_TASKS == "LOCAL_FUNCTION">
<#if CONFIG_GFX_STACK_RTOS == "Standalone">
<#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
void _GFX_STACK_Tasks(ULONG thread_input)
<#else>
void _GFX_STACK_Tasks(void)
</#if>
{
<#if CONFIG_3RDPARTY_RTOS_USED == "uC/OS-III">
    OS_ERR os_err;
	
</#if>
    while(1)
    {
	    // update the GFX Abstraction Layer
	    GFX_Update();
<@RTOS_TASK_DELAY RTOS_NAME=CONFIG_3RDPARTY_RTOS_USED TASK_DELAY=CONFIG_GFX_STACK_RTOS_DELAY/>		
    }
 }
</#if>
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
