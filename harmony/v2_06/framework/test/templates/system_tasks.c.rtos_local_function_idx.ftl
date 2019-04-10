<#--
  Number of Library Instances:
-->
<#assign lib_instances = [ CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX0,
                           CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX1,
                           CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX2,
                           CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX3,
                           CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX4,
                           CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX5,
                           CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX6,
                           CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX7,
                           CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX8,
                           CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX9,
                           CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX10,
                           CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX11,
                           CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX12,
                           CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX13,
                           CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX14,
                           CONFIG_TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX15 ]>
<#--
  Task Function Names:
-->
<#assign func_name =  [ CONFIG_TEST_LIBRARY_IDX0_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX0_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX0_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX0_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX0_TASKS_IDX4,
                     
                        CONFIG_TEST_LIBRARY_IDX1_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX1_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX1_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX1_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX1_TASKS_IDX4,
                     
                        CONFIG_TEST_LIBRARY_IDX2_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX2_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX2_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX2_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX2_TASKS_IDX4,
                     
                        CONFIG_TEST_LIBRARY_IDX3_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX3_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX3_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX3_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX3_TASKS_IDX4,
                     
                        CONFIG_TEST_LIBRARY_IDX4_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX4_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX4_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX4_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX4_TASKS_IDX4,
                     
                        CONFIG_TEST_LIBRARY_IDX5_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX5_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX5_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX5_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX5_TASKS_IDX4,
                     
                        CONFIG_TEST_LIBRARY_IDX6_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX6_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX6_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX6_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX6_TASKS_IDX4,
                     
                        CONFIG_TEST_LIBRARY_IDX7_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX7_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX7_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX7_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX7_TASKS_IDX4,
                     
                        CONFIG_TEST_LIBRARY_IDX8_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX8_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX8_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX8_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX8_TASKS_IDX4,
                     
                        CONFIG_TEST_LIBRARY_IDX9_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX9_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX9_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX9_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX9_TASKS_IDX4,
                     
                        CONFIG_TEST_LIBRARY_IDX10_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX10_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX10_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX10_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX10_TASKS_IDX4,
                     
                        CONFIG_TEST_LIBRARY_IDX11_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX11_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX11_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX11_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX11_TASKS_IDX4,
                     
                        CONFIG_TEST_LIBRARY_IDX12_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX12_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX12_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX12_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX12_TASKS_IDX4,
                     
                        CONFIG_TEST_LIBRARY_IDX13_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX13_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX13_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX13_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX13_TASKS_IDX4,
                     
                        CONFIG_TEST_LIBRARY_IDX14_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX14_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX14_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX14_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX14_TASKS_IDX4,
                     
                        CONFIG_TEST_LIBRARY_IDX15_TASKS_IDX0,
                        CONFIG_TEST_LIBRARY_IDX15_TASKS_IDX1,
                        CONFIG_TEST_LIBRARY_IDX15_TASKS_IDX2,
                        CONFIG_TEST_LIBRARY_IDX15_TASKS_IDX3,
                        CONFIG_TEST_LIBRARY_IDX15_TASKS_IDX4 ]>
<#--
  Task Interrupt Driven (boolean):
-->
<#assign func_ints  = [ CONFIG_TEST_LIBRARY_IDX0_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX0_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX0_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX0_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX0_TASKS_IDX4_INTERRUPT,
                     
                        CONFIG_TEST_LIBRARY_IDX1_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX1_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX1_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX1_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX1_TASKS_IDX4_INTERRUPT,
                     
                        CONFIG_TEST_LIBRARY_IDX2_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX2_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX2_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX2_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX2_TASKS_IDX4_INTERRUPT,
                     
                        CONFIG_TEST_LIBRARY_IDX3_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX3_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX3_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX3_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX3_TASKS_IDX4_INTERRUPT,
                     
                        CONFIG_TEST_LIBRARY_IDX4_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX4_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX4_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX4_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX4_TASKS_IDX4_INTERRUPT,
                     
                        CONFIG_TEST_LIBRARY_IDX5_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX5_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX5_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX5_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX5_TASKS_IDX4_INTERRUPT,
                     
                        CONFIG_TEST_LIBRARY_IDX6_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX6_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX6_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX6_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX6_TASKS_IDX4_INTERRUPT,
                     
                        CONFIG_TEST_LIBRARY_IDX7_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX7_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX7_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX7_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX7_TASKS_IDX4_INTERRUPT,
                     
                        CONFIG_TEST_LIBRARY_IDX8_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX8_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX8_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX8_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX8_TASKS_IDX4_INTERRUPT,
                     
                        CONFIG_TEST_LIBRARY_IDX9_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX9_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX9_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX9_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX9_TASKS_IDX4_INTERRUPT,
                     
                        CONFIG_TEST_LIBRARY_IDX10_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX10_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX10_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX10_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX10_TASKS_IDX4_INTERRUPT,
                     
                        CONFIG_TEST_LIBRARY_IDX11_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX11_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX11_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX11_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX11_TASKS_IDX4_INTERRUPT,
                     
                        CONFIG_TEST_LIBRARY_IDX12_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX12_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX12_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX12_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX12_TASKS_IDX4_INTERRUPT,
                     
                        CONFIG_TEST_LIBRARY_IDX13_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX13_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX13_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX13_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX13_TASKS_IDX4_INTERRUPT,
                     
                        CONFIG_TEST_LIBRARY_IDX14_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX14_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX14_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX14_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX14_TASKS_IDX4_INTERRUPT,
                     
                        CONFIG_TEST_LIBRARY_IDX15_TASKS_IDX0_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX15_TASKS_IDX1_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX15_TASKS_IDX2_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX15_TASKS_IDX3_INTERRUPT,
                        CONFIG_TEST_LIBRARY_IDX15_TASKS_IDX4_INTERRUPT ]>
<#--
  Macro to translate test position numbers to appropriate labels:
-->
<#macro trans_test_pos test_pos>
  <#if     test_pos == 0>
    <#lt>TEST_POS_0<#rt>
  <#elseif test_pos == 1>
    <#lt>TEST_POS_1<#rt>
  <#elseif test_pos == 2>
    <#lt>TEST_POS_2<#rt>
  <#elseif test_pos == 3>
    <#lt>TEST_POS_3<#rt>
  <#elseif test_pos == 4>
    <#lt>TEST_POS_4<#rt>
  <#elseif test_pos == 5>
    <#lt>TEST_POS_5<#rt>
  <#elseif test_pos == 6>
    <#lt>TEST_POS_6<#rt>
  <#elseif test_pos == 7>
    <#lt>TEST_POS_7<#rt>
  <#elseif test_pos == 8>
    <#lt>TEST_POS_8<#rt>
  <#elseif test_pos == 9>
    <#lt>TEST_POS_9<#rt>
  <#elseif test_pos == 10>
    <#lt>TEST_POS_10<#rt>
  <#elseif test_pos == 11>
    <#lt>TEST_POS_11<#rt>
  <#elseif test_pos == 12>
    <#lt>TEST_POS_12<#rt>
  <#elseif test_pos == 13>
    <#lt>TEST_POS_13<#rt>
  <#elseif test_pos == 14>
    <#lt>TEST_POS_14<#rt>
  <#elseif test_pos == 15>
    <#lt>TEST_POS_15<#rt>
  <#else>
    #error "Invalid test position"
  </#if>
</#macro>
<#--
  Macro to translate library under test index values to appropriate labels:
-->
<#macro trans_lut_index lut_index>
  <#if     lut_index == 0>
    <#lt>TEST_LUT_INDEX_0<#rt>
  <#elseif lut_index == 1>
    <#lt>TEST_LUT_INDEX_1<#rt>
  <#elseif lut_index == 2>
    <#lt>TEST_LUT_INDEX_3<#rt>
  <#else>
    #error "Invalid library under test index"
  </#if>
</#macro>
<#--
  Test Use Delay (boolean):
-->
<#assign tdelay   = [ CONFIG_TEST_TASKS_0_RTOS_USE_DELAY,
                      CONFIG_TEST_TASKS_1_RTOS_USE_DELAY,
                      CONFIG_TEST_TASKS_2_RTOS_USE_DELAY,
                      CONFIG_TEST_TASKS_3_RTOS_USE_DELAY,
                      CONFIG_TEST_TASKS_4_RTOS_USE_DELAY ]>
<#--
  Test Delay in Milliseconds:
-->
<#assign tdelayms = [ CONFIG_TEST_TASKS_0_RTOS_DELAY,
                      CONFIG_TEST_TASKS_1_RTOS_DELAY,
                      CONFIG_TEST_TASKS_2_RTOS_DELAY,
                      CONFIG_TEST_TASKS_3_RTOS_DELAY,
                      CONFIG_TEST_TASKS_4_RTOS_DELAY ]>
<#--
  Test Local Function Generation Loop:
-->
<#list 0..CONFIG_TEST_HARNESS_MAX_NUM_TASKS?number-1 as number>

/*******************************************************************************
  Function:
    void _TEST_Tasks${number} ( void )

  Summary:
    Maintains current test's "Tasks${number} state machine.
*/

static void _TEST_Tasks${number} ( void )
{
    while(1)
    {
        TEST_Tasks(TEST_TASKS_FUNCTION_${number});
        <#if tdelay[number] == true>
          <@RTOS_TASK_DELAY RTOS_NAME=CONFIG_3RDPARTY_RTOS_USED TASK_DELAY=tdelayms[number]/>
        </#if>
    }
}

</#list>
<#--
  Library Under Test Local Function Generation Loop:
-->
<#assign index_libraries = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]>
<#assign index_tasks     = [0, 1, 2, 3, 4]>
<#assign index_instances = [0, 1, 2]>
<#list index_libraries as num_lib>
  <#list index_tasks as num_task>
    <#list index_instances as num_inst>
      <#assign index = 15 * num_lib + 5 * num_inst + num_task>
      <#assign index_func = 5 * num_lib + num_task>
        <#if func_ints[index_func] == false && func_name[index_func]?has_content && func_name[index_func] != "NULL">
          <#if lib_instances[num_lib]?has_content && num_inst lt lib_instances[num_lib]?number>

/*******************************************************************************
  Function:
    void _TEST_Library${num_lib}Instance${num_inst}Tasks${num_task}Daemon ( void )

  Summary:
    Maintains state machine of library under test ${num_lib} instance ${num_inst} tasks
    function ${num_task}:  "${func_name[index_func]}".
*/

static void _TEST_Library${num_lib}Instance${num_inst}Tasks${num_task} ( void )
{
    while(1)
    {
        TEST_LibraryTasksPolled(<@trans_test_pos test_pos=num_lib/>, <@trans_lut_index lut_index=num_inst/>, ${func_name[index_func]});
        <@RTOS_TASK_DELAY RTOS_NAME=CONFIG_3RDPARTY_RTOS_USED TASK_DELAY=CONFIG_TEST_RTOS_DELAY/>
    }
}

          </#if>
        </#if>
    </#list>
  </#list>
</#list>
