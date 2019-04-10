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
<#if CONFIG_USE_SYS_MSG == true>

    /*** Message Service Initialization Code ***/
    <#if CONFIG_SYS_MSG_INST_IDX0 == true>
    msg0Init.nQSizes = queuePriorities0;
    sysObj.sysMsg0 = SYS_MSG_Initialize(SYS_MSG_0, (SYS_OBJ_HANDLE)&msg0Init);
  </#if>
  <#if CONFIG_SYS_MSG_INST_IDX1 == true>
    msg1Init.nQSizes = queuePriorities1;
    sysObj.sysMsg1 = SYS_MSG_Initialize(SYS_MSG_1, (SYS_OBJ_HANDLE)&msg1Init);
  </#if>
  <#if CONFIG_SYS_MSG_INST_IDX2 == true>
    msg2Init.nQSizes = queuePriorities2;
    sysObj.sysMsg2 = SYS_MSG_Initialize(SYS_MSG_2, (SYS_OBJ_HANDLE)&msg2Init);
  </#if>
  <#if CONFIG_SYS_MSG_INST_IDX3 == true>
    msg3Init.nQSizes = queuePriorities3;
    sysObj.sysMsg3 = SYS_MSG_Initialize(SYS_MSG_3, (SYS_OBJ_HANDLE)&msg3Init);
  </#if>
  <#if CONFIG_SYS_MSG_INST_IDX4 == true>
    msg4Init.nQSizes = queuePriorities4;
    sysObj.sysMsg4 = SYS_MSG_Initialize(SYS_MSG_4, (SYS_OBJ_HANDLE)&msg4Init);
  </#if>
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
