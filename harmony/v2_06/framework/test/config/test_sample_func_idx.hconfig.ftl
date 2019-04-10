config TEST_SAMPLE_FUNC_LIB_INSTANCES_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_SAMPLE_FUNC_TEST
<#if INSTANCE != 0>
    default n if TEST_SAMPLE_FUNC_LIB_INSTANCES_NUMBER_GT_${INSTANCE} = n     
</#if>
    default n if TEST_SAMPLE_FUNC_LIB_INSTANCES_NUMBER = ${INSTANCE+1}
    default y

config TEST_SAMPLE_FUNC_LIB_INSTANCES_NUMBER_${INSTANCE}
    depends on USE_SAMPLE_FUNC_TEST
<#if INSTANCE != 0>
     && TEST_SAMPLE_FUNC_LIB_INSTANCES_NUMBER_GT_${INSTANCE}
</#if>
    bool "Sample Library Instance ${INSTANCE}"
    default y
    persistent
    ---help---
    Configuration settings for Sample Library instance ${INSTANCE}.
    ---endhelp---


ifblock TEST_SAMPLE_FUNC_LIB_INSTANCES_NUMBER_${INSTANCE}

menu "Sample Library Instance ${INSTANCE} Test Options"
    depends on USE_SAMPLE_FUNC_TEST

config TEST_SAMPLE_LIB_INIT_DATA_IDX${INSTANCE}
    string "Sample Library Instance ${INSTANCE} Init Data Structure"
    depends on USE_SAMPLE_FUNC_TEST
    default "sampleModule${INSTANCE}InitData"
    ---help---
    The Sample Functional test requires the name of the "init" data structure 
    for instance ${INSTANCE} of the Sample library so that it can verify the 
    initial data given to it.
    ---endhelp---

endmenu

config TEST_SAMPLE_POSITION_IN_HARNESS_LIST_IDX${INSTANCE}
    string
    depends on USE_SAMPLE_FUNC_TEST
    default "TEST_POS_0"  if TEST_NUMBER_0_SELECTED  = "Sample Module Functional Test ${INSTANCE}"
    default "TEST_POS_1"  if TEST_NUMBER_1_SELECTED  = "Sample Module Functional Test ${INSTANCE}"
    default "TEST_POS_2"  if TEST_NUMBER_2_SELECTED  = "Sample Module Functional Test ${INSTANCE}"
    default "TEST_POS_3"  if TEST_NUMBER_3_SELECTED  = "Sample Module Functional Test ${INSTANCE}"
    default "TEST_POS_4"  if TEST_NUMBER_4_SELECTED  = "Sample Module Functional Test ${INSTANCE}"
    default "TEST_POS_5"  if TEST_NUMBER_5_SELECTED  = "Sample Module Functional Test ${INSTANCE}"
    default "TEST_POS_6"  if TEST_NUMBER_6_SELECTED  = "Sample Module Functional Test ${INSTANCE}"
    default "TEST_POS_7"  if TEST_NUMBER_7_SELECTED  = "Sample Module Functional Test ${INSTANCE}"
    default "TEST_POS_8"  if TEST_NUMBER_8_SELECTED  = "Sample Module Functional Test ${INSTANCE}"
    default "TEST_POS_9"  if TEST_NUMBER_9_SELECTED  = "Sample Module Functional Test ${INSTANCE}"
    default "TEST_POS_10" if TEST_NUMBER_10_SELECTED = "Sample Module Functional Test ${INSTANCE}"
    default "TEST_POS_11" if TEST_NUMBER_11_SELECTED = "Sample Module Functional Test ${INSTANCE}"
    default "TEST_POS_12" if TEST_NUMBER_12_SELECTED = "Sample Module Functional Test ${INSTANCE}"
    default "TEST_POS_13" if TEST_NUMBER_13_SELECTED = "Sample Module Functional Test ${INSTANCE}"
    default "TEST_POS_14" if TEST_NUMBER_14_SELECTED = "Sample Module Functional Test ${INSTANCE}"
    default "TEST_POS_15" if TEST_NUMBER_15_SELECTED = "Sample Module Functional Test ${INSTANCE}"

endif
