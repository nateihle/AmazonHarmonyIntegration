<#-- Local Macros -->
<#macro test_tasks number>
    TEST_Tasks(TEST_TASKS_FUNCTION_${number});
</#macro>
<#if CONFIG_USE_TEST_HARNESS == true>

    /* Maintain library instance(s) under test. */
<#include "/framework/test/templates/system_tasks.c.idx.ftl">
    
    /* Maintain test tasks */
    <#list 0..CONFIG_TEST_HARNESS_MAX_NUM_TASKS?number-1 as num>
        <@test_tasks number=num/>
    </#list>
    
    /* Maintain the test harness. */
    TEST_HarnessTasks();
</#if>
