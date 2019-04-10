<#--
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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
<#if CONFIG_USE_SYS_DEVCON == true && CONFIG_SYS_DEVCON_DYNAMIC == true>
  <#if !(SYS_DEVCON_TASKS?has_content)>
    <#assign SYS_DEVCON_TASKS = "TASK_CALL_NO_RTOS">
  </#if>
  <#if (SYS_DEVCON_TASKS == "PROTO") && (CONFIG_SYS_DEVCON_RTOS == "Standalone")>
    <#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
      <#lt>void _SYS_DEVCON_Tasks(ULONG thread_input);
    <#else>
      <#lt>void _SYS_DEVCON_Tasks(void);
    </#if>
  </#if>
  <#if SYS_DEVCON_TASKS == "CREATE_TASK">
    <#if CONFIG_SYS_DEVCON_RTOS == "Standalone">
      <#lt><@RTOS_TASK_CREATE RTOS_NAME=CONFIG_3RDPARTY_RTOS_USED TASK_FUNC_NAME="_SYS_DEVCON_Tasks" TASK_NAME="Sys Devcon Tasks" TASK_PRI=CONFIG_SYS_DEVCON_RTOS_TASK_PRIORITY TASK_STK_SZ=CONFIG_SYS_DEVCON_RTOS_TASK_SIZE/>

    </#if>
  </#if>
  <#if SYS_DEVCON_TASKS == "TASK_CALL_NO_RTOS">
    <#lt>    SYS_DEVCON_Tasks(sysObj.sysDevcon);
  <#elseif (SYS_DEVCON_TASKS == "TASK_CALL" && CONFIG_SYS_DEVCON_RTOS != "Standalone")>
    <#lt>        SYS_DEVCON_Tasks(sysObj.sysDevcon);
  </#if>
  <#if SYS_DEVCON_TASKS == "LOCAL_FUNCTION">
    <#if CONFIG_SYS_DEVCON_RTOS == "Standalone">
      <#if CONFIG_3RDPARTY_RTOS_USED == "ThreadX">
        <#lt>void _SYS_DEVCON_Tasks(ULONG thread_input)
      <#else>
        <#lt>void _SYS_DEVCON_Tasks(void)
      </#if>
      <#lt>{
      <#lt>    <#if CONFIG_3RDPARTY_RTOS_USED == "uC/OS-III">
      <#lt>    OS_ERR os_err;
      </#if>

      <#lt>    while(1)
      <#lt>    {
      <#lt>        /* Maintain the DEVCON system state machine. */
      <#lt>        SYS_DEVCON_Tasks(sysObj.sysDevcon);
      <#lt>        <#lt><@RTOS_TASK_DELAY RTOS_NAME=CONFIG_3RDPARTY_RTOS_USED TASK_DELAY=CONFIG_SYS_DEVCON_RTOS_DELAY/>
      <#lt>    }
      <#lt>}
    </#if>
  </#if>
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->