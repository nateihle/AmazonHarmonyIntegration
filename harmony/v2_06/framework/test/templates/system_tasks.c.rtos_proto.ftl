<#if CONFIG_USE_TEST_HARNESS == true>
static void _TEST_HarnessTasks ( void );
<#list 0..CONFIG_TEST_HARNESS_MAX_NUM_TASKS?number-1 as number>
static void _TEST_Tasks${number} ( void );
</#list>
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
  Local Function Generation Loop:
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
static void _TEST_Library${num_lib}Instance${num_inst}Tasks${num_task} ( void );
          </#if>
        </#if>
    </#list>
  </#list>
</#list>
</#if>
