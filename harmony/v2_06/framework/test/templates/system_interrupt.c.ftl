<#if CONFIG_USE_TEST_HARNESS == true>
<#-- Determine IPL value -->
<#if     CONFIG_TEST_TIMER_INTERRUPT_PRIORITY == "INT_PRIORITY_LEVEL1">
    <#assign ipl="IPL1AUTO">
<#elseif CONFIG_TEST_TIMER_INTERRUPT_PRIORITY == "INT_PRIORITY_LEVEL2">
    <#assign ipl="IPL2AUTO">
<#elseif CONFIG_TEST_TIMER_INTERRUPT_PRIORITY == "INT_PRIORITY_LEVEL3">
    <#assign ipl="IPL3AUTO">
<#elseif CONFIG_TEST_TIMER_INTERRUPT_PRIORITY == "INT_PRIORITY_LEVEL4">
    <#assign ipl="IPL4AUTO">
<#elseif CONFIG_TEST_TIMER_INTERRUPT_PRIORITY == "INT_PRIORITY_LEVEL5">
    <#assign ipl="IPL5AUTO">
<#elseif CONFIG_TEST_TIMER_INTERRUPT_PRIORITY == "INT_PRIORITY_LEVEL6">
    <#assign ipl="IPL6AUTO">
<#elseif CONFIG_TEST_TIMER_INTERRUPT_PRIORITY == "INT_PRIORITY_LEVEL7">
    <#assign ipl="IPL7AUTO">
<#else>
    <#assign ipl="#error invalid IPL">
</#if>
<#-- Determine Vector & Source -->
<#if     CONFIG_TEST_TIMER_ID == "TMR_ID_1">
    <#assign vector="_TIMER_1_VECTOR">
    <#assign source="INT_SOURCE_TIMER_1">
<#elseif CONFIG_TEST_TIMER_ID == "TMR_ID_2">
    <#assign vector="_TIMER_2_VECTOR">
    <#assign source="INT_SOURCE_TIMER_2">
<#elseif CONFIG_TEST_TIMER_ID == "TMR_ID_3">
    <#assign vector="_TIMER_3_VECTOR">
    <#assign source="INT_SOURCE_TIMER_3">
<#elseif CONFIG_TEST_TIMER_ID == "TMR_ID_4">
    <#assign vector="_TIMER_4_VECTOR">
    <#assign source="INT_SOURCE_TIMER_4">
<#elseif CONFIG_TEST_TIMER_ID == "TMR_ID_5">
    <#assign vector="_TIMER_5_VECTOR">
    <#assign source="INT_SOURCE_TIMER_5">
<#elseif CONFIG_TEST_TIMER_ID == "TMR_ID_6">
    <#assign vector="_TIMER_6_VECTOR">
    <#assign source="INT_SOURCE_TIMER_6">
<#elseif CONFIG_TEST_TIMER_ID == "TMR_ID_7">
    <#assign vector="_TIMER_7_VECTOR">
    <#assign source="INT_SOURCE_TIMER_7">
<#else>
    <#assign vector="#error unsupported timer">
    <#assign source="#error unsupported timer">
</#if>

/* Test Harness Timer Vector Function. */
void __ISR(${vector}, ${ipl}) _TestHarnessTimerISR(void)
{
    TEST_TimerIncrement();
    PLIB_INT_SourceFlagClear(INT_ID_0, ${source});
}
<#include "/framework/test/templates/system_interrupt.c.idx.ftl">
</#if> <#-- CONFIG_USE_TEST_HARNESS -->