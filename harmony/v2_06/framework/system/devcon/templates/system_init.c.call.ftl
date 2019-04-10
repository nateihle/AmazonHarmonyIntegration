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
<#if CONFIG_USE_SYS_DEVCON == true>
  <#if CONFIG_SYS_DEVCON_DYNAMIC == true>
    <#lt>    sysObj.sysDevcon = SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)&sysDevconInit);
  <#else>
    <#lt>    SYS_DEVCON_Initialize(SYS_DEVCON_INDEX_0, (SYS_MODULE_INIT*)NULL);
  </#if>
  <#if CONFIG_PIC32MK == true || CONFIG_PIC32MX == true || CONFIG_PIC32MZ == true>
    <#lt>    SYS_DEVCON_PerformanceConfig(SYS_CLK_SystemFrequencyGet());
  </#if>
  <#if CONFIG_SYS_DEVCON_USE_JTAG == true>
    <#lt>    SYS_DEVCON_JTAGEnable();
  <#else>
    <#if CONFIG_JTAGEN?has_content>
      <#if CONFIG_JTAGEN == "ON">
        <#lt>    SYS_DEVCON_JTAGDisable();
      </#if>
    <#else>
      <#lt>    SYS_DEVCON_JTAGDisable();
    </#if>
  </#if>
  <#if CONFIG_SYS_DEVCON_USE_TRACE == true>
    <#lt>    SYS_DEVCON_TraceEnable();
  </#if>
</#if>