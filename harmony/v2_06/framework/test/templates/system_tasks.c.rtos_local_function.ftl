<#if CONFIG_USE_TEST_HARNESS == true>

/*******************************************************************************
  Function:
    void _TEST_HarnessTasks ( void )

  Summary:
    Maintains state machine of the test harness.
*/

static void _TEST_HarnessTasks ( void )
{
    while(1)
    {
        TEST_HarnessTasks();
        <@RTOS_TASK_DELAY RTOS_NAME=CONFIG_3RDPARTY_RTOS_USED TASK_DELAY=CONFIG_TEST_RTOS_DELAY/>
    }
}

<#include "/framework/test/templates/system_tasks.c.rtos_local_function_idx.ftl">
</#if>
