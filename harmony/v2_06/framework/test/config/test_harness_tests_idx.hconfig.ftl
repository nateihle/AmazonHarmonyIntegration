###############################################################################
# Individual Test Instance Configuration


config TEST_NUMBER_GT_${INSTANCE+1}
    bool
    depends on USE_TEST_HARNESS
<#if INSTANCE != 0>
	default n if TEST_NUMBER_GT_${INSTANCE} = n     
</#if>	
	default n if TEST_NUMBER = ${INSTANCE+1}
	default y

config TEST_NUMBER_${INSTANCE}
    depends on USE_TEST_HARNESS
<#if INSTANCE != 0>
	       && TEST_NUMBER_GT_${INSTANCE}
</#if>
    bool "Test ${INSTANCE}"
    default y
    persistent
    ---help---
    Configuration settings for test ${INSTANCE}.
    ---endhelp---

ifblock TEST_NUMBER_${INSTANCE}

config TEST_NUMBER_${INSTANCE}_SELECTED
    string "Select Predefined Test Instance"
    #depends on TEST_NUMBER_${INSTANCE}_IS_PREDEFINED = y
    range TESTS_AVAILABLE
    default "Custom Test"
    ---help---
    Select from a list of pre-defined tests.
    ---endhelp---


###############################################################################
# Test Instance Menu

menu "Test ${INSTANCE} Configuration Options"
    #depends on TEST_NUMBER_${INSTANCE}_IS_PREDEFINED = n

config TEST_NAME_IDX${INSTANCE}
    string "Test Name"
    depends on USE_TEST_HARNESS
    default "${INSTANCE} <library-name> <test-type>" if TEST_NUMBER_${INSTANCE}_SELECTED = "Custom Test"
    default TEST_NUMBER_${INSTANCE}_SELECTED
    ---help---
    Brief name describing the test, used to identify test output.  Should be in the 
    form "&lt;library-name&gt; &lt;test-type&gt;" for example, "Sample library Functional" for 
    functional testing of the sample library.
    ---endhelp---

config TEST_INSTANCE_INDEX_IDX${INSTANCE}
    int "Test Instance Index"
    depends on USE_TEST_HARNESS
    default 0 if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default 1 if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1"
    default 0
    ---help---
    Test instance index.  This us usually zero (0) unless you are using the 
    same test library more than once. 
    ---endhelp---

config TEST_INSTANCE_INIT_DATA_IDX${INSTANCE}
    string "Test Init Data Structure"
    depends on USE_TEST_HARNESS
    default "testSampleInit" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "testSampleInit" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1"
    default "NULL"
    ---help---
    Each instance of a test needs a pointer to the data structure that contains any 
    necessary initial data the test requires.
    ---endhelp---

config TEST_FUNC_INITIALIZE_IDX${INSTANCE}
    string "Test Initialize Function"
    depends on USE_TEST_HARNESS
    default "TEST_SampleFunctionalInitialize" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "TEST_SampleFunctionalInitialize" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1"
    default "Test${INSTANCE}LibraryNameTestTypeInitialize"
    ---help---
    Pointer to the "Initialize" function for test ${INSTANCE}.
    ---endhelp---

config TEST_FUNC_START_IDX${INSTANCE}
    string "Test Start Function"
    depends on USE_TEST_HARNESS
    default "TEST_SampleFunctionalStart" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "TEST_SampleFunctionalStart" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1"
    default "Test${INSTANCE}LibraryNameTestTypeStart"
    ---help---
    Pointer to the "Start" function for test ${INSTANCE}.
    ---endhelp---

config TEST_FUNC_STATUS_IDX${INSTANCE}
    string "Test Status Function"
    depends on USE_TEST_HARNESS
    default "TEST_SampleFunctionalStatus" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "TEST_SampleFunctionalStatus" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1"
    default "Test${INSTANCE}LibraryNameTestTypeStatus"
    ---help---
    Pointer to the "Status" function for test ${INSTANCE}.
    ---endhelp---

config TEST_${INSTANCE}_FUNC_TASKS_0
    string "Test ${INSTANCE} Tasks Function 0"
    depends on TEST_HARNESS_MAX_NUM_TASKS=1 || TEST_HARNESS_MAX_NUM_TASKS=2 || TEST_HARNESS_MAX_NUM_TASKS=3 || TEST_HARNESS_MAX_NUM_TASKS=4 || TEST_HARNESS_MAX_NUM_TASKS=5
    default "TEST_SampleFunctionalTasks" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "TEST_SampleFunctionalTasks" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1"
    default "NULL"
    ---help---
    Pointer to the "Tasks" function 0 for test ${INSTANCE}.
    ---endhelp---

config TEST_${INSTANCE}_FUNC_TASKS_1
    string "Test ${INSTANCE} Tasks Function 1"
    depends on TEST_HARNESS_MAX_NUM_TASKS=2 || TEST_HARNESS_MAX_NUM_TASKS=3 || TEST_HARNESS_MAX_NUM_TASKS=4 || TEST_HARNESS_MAX_NUM_TASKS=5
    default "TEST_SampleFunctionalTasks" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "TEST_SampleFunctionalTasks" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1"
    default "NULL"
    ---help---
    Pointer to the "Tasks" function 1 for test ${INSTANCE}.
    ---endhelp---

config TEST_${INSTANCE}_FUNC_TASKS_2
    string "Test ${INSTANCE} Tasks Function 2"
    depends on TEST_HARNESS_MAX_NUM_TASKS=3 ||TEST_HARNESS_MAX_NUM_TASKS=4 || TEST_HARNESS_MAX_NUM_TASKS=5
    default "NULL"
    ---help---
    Pointer to the "Tasks" function 2 for test ${INSTANCE}.
    ---endhelp---

config TEST_${INSTANCE}_FUNC_TASKS_3
    string "Test ${INSTANCE} Tasks Function 3"
    depends on TEST_HARNESS_MAX_NUM_TASKS=4 || TEST_HARNESS_MAX_NUM_TASKS=5
    default "NULL"
    ---help---
    Pointer to the "Tasks" function 3 for test ${INSTANCE}.
    ---endhelp---

config TEST_${INSTANCE}_FUNC_TASKS_4
    string "Test ${INSTANCE} Tasks Function 4"
    depends on TEST_HARNESS_MAX_NUM_TASKS=5
    default "NULL"
    ---help---
    Pointer to the "Tasks" function 4 for test ${INSTANCE}.
    ---endhelp---

endmenu # Test Instance


###############################################################################
# Library Under Test (LUT) Menu

menu "Library ${INSTANCE} Configuration Options"
    depends on USE_TEST_HARNESS

config TEST_LIBRARY_IDX${INSTANCE}
    depends on USE_TEST_HARNESS
    bool
    default y
    persistent

config TEST_LIBRARY_STRUCT_NAME_IDX${INSTANCE}
    string "Data Structure Name"
    depends on USE_TEST_HARNESS
    default "testSample0LUTdata" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "testSample1LUTdata" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1"
    default "test${INSTANCE}LUTdata"
    ---help---
    Name of the data structure that identifies the functions and data for 
    for library under test ${INSTANCE}.
    ---endhelp---

config TEST_LIBRARY_FUNC_INITIALIZE_IDX${INSTANCE}
    string "Initialize Function"
    depends on USE_TEST_HARNESS
    default "SAMPLE_Initialize" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "SAMPLE_Initialize" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1"
    default "NULL"
    ---help---
    Pointer to the "Initialize" function for the library under test ${INSTANCE}.
    ---endhelp---

config TEST_LIBRARY_FUNC_REINITIALIZE_IDX${INSTANCE}
    string "Reinitialize Function"
    depends on USE_TEST_HARNESS
    default "SAMPLE_Reinitialize" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "SAMPLE_Reinitialize" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1"
    default "NULL"
    ---help---
    Pointer to the "Renitialize" function for the library under test ${INSTANCE}.
    ---endhelp---

config TEST_LIBRARY_FUNC_DEINITIALIZE_IDX${INSTANCE}
    string "Deinitialize Function"
    depends on USE_TEST_HARNESS
    default "SAMPLE_Deinitialize" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "SAMPLE_Deinitialize" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1"
    default "NULL"
    ---help---
    Pointer to the "Deinitialize" function for the library under test ${INSTANCE}.
    ---endhelp---

config TEST_LIBRARY_FUNC_STATUS_IDX${INSTANCE}
    string "Status Function"
    depends on USE_TEST_HARNESS
    default "SAMPLE_Status" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "SAMPLE_Status" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1"
    default "NULL"
    ---help---
    Pointer to the "Status" function for the library under test ${INSTANCE}.
    ---endhelp---

config TEST_LIBRARY_NUMBER_OF_INSTANCES_IDX${INSTANCE}
    int "Number of Instances"
    depends on USE_TEST_HARNESS
    range 1 3
    default 2 if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default 2 if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1"
    default 1
    ---help---
    Number of instances of library under test ${INSTANCE} that the test harnesss
    will run.  Limited by the MPLAB Harmony Configuator to a maximum of 3.
    ---endhelp---
 
config TEST_LIBRARY_IDX${INSTANCE}_INITDATA_IDX0
    string "Instance 0 Init Data"
    depends on USE_TEST_HARNESS
    default "sampleModule0InitData" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "NULL"
    ---help---
    Pointer to the init data structure for library under test ${INSTANCE} instance 0.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INITDATA_IDX1
    string "Instance 1 Init Data"
    depends on USE_TEST_HARNESS
    default "sampleModule1InitData" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "NULL"
    ---help---
    Pointer to the init data structure for library under test ${INSTANCE} instance 1.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INITDATA_IDX2
    string "Instance 2 Init Data"
    depends on USE_TEST_HARNESS
    default "NULL"
    ---help---
    Pointer to the init data structure for library under test ${INSTANCE} instance 2.
    ---endhelp---

config TEST_LIBRARY_NUMBER_OF_TASKS_IDX${INSTANCE}
    int "Number of Polled Tasks Functions"
    depends on USE_TEST_HARNESS
    range 0 5
    default 1 if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0" && SAMPLE_MODULE_INTERRUPT_MODE = n
    default 1 if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1" && SAMPLE_MODULE_INTERRUPT_MODE = n
    default 0 if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0" && SAMPLE_MODULE_INTERRUPT_MODE = y
    default 0 if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1" && SAMPLE_MODULE_INTERRUPT_MODE = y
    default 0
    ---help---
    The test harness will call each of these when running the library.  Limited by the 
    MPLAB Harmony Configuator to a maximum of 5.
    ---endhelp---

###################
# Library Tasks 0 #
###################

config TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0
    string "Tasks function 0"
    depends on USE_TEST_HARNESS
    default "SAMPLE_Tasks" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "SAMPLE_Tasks" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 1"
    default "NULL"
    ---help---
    Name of the first tasks function for library under test ${INSTANCE}.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT
    bool "Call Tasks function 0 from ISR?"
    depends on USE_TEST_HARNESS
    default y if SAMPLE_MODULE_INTERRUPT_MODE = y
    default n
    ---help---
    Select if you want Library ${INSTANCE} Tasks function 0 to be called from 
    an Interrupt Service Routine (ISR).  Do not select if you want it to be polled.
    ---endhelp---

###############################################################################
# Library Instance 0 Tasks 0 ISR Configuration Menu

menu "Library ${INSTANCE} Instance 0 Tasks 0 ISR Configuration"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX0_INTERRUPT_VECTOR
    string "Interrupt Vector"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT
    range INT_VECTOR
    default "INT_VECTOR_T2" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "INT_VECTOR_CT"
    ---help---
    Select the interrupt vector from which library ${INSTANCE} instance 0 tasks 0
    function should be called.
    ---endhelp---

comment "The XC32 compiler interrupt vector must represent the same vector as the above selection."

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX0_INTERRUPT_VECTOR_XC32
    string "Interrupt Vector as defined by XC32 Compiler"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT
    default "_TIMER_2_VECTOR" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "_CORE_TIMER_VECTOR"
    ---help---
    Enter the interrupt vector name as defined by the XC32 compiler.  This must
    represent the same vector as selected in the "Interrupt Vector".
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX0_INTERRUPT_SOURCE
    string "Interrupt Source"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT
    range INT_SOURCE
    default "INT_SOURCE_TIMER_2" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "INT_SOURCE_TIMER_CORE"
    ---help---
    Select the interrupt source for library ${INSTANCE} instance 0 tasks 0 function.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX0_INTERRUPT_PRIORITY
    string "Interrupt Priority Level"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT
    range INT_PRIORITY_LEVEL
    default "INT_PRIORITY_LEVEL2" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "INT_DISABLE_INTERRUPT"
    ---help---
    Select the interrupt priority level for library ${INSTANCE} instance 0 tasks 0 function.
    ---endhelp---

endmenu  # Library Instance 0 Tasks 0 ISR Configuration

###############################################################################
# Library Instance 1 Tasks 0 ISR Configuration Menu

menu "Library ${INSTANCE} Instance 1 Tasks 0 ISR Configuration"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX0_INTERRUPT_VECTOR
    string "Interrupt Vector"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT
    range INT_VECTOR
    default "INT_VECTOR_T3" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "INT_VECTOR_CT"
    ---help---
    Select the interrupt vector from which library ${INSTANCE} instance 1 tasks 0
    function should be called.
    ---endhelp---

comment "The XC32 compiler interrupt vector must represent the same vector as the above selection."

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX0_INTERRUPT_VECTOR_XC32
    string "Interrupt Vector as defined by XC32 Compiler"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT
    default "_TIMER_3_VECTOR" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "_CORE_TIMER_VECTOR"
    ---help---
    Enter the interrupt vector name as defined by the XC32 compiler.  This must
    represent the same vector as selected in the "Interrupt Vector".
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX0_INTERRUPT_SOURCE
    string "Interrupt Source"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT
    range INT_SOURCE
    default "INT_SOURCE_TIMER_3" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "INT_SOURCE_TIMER_CORE"
    ---help---
    Select the interrupt source for library ${INSTANCE} instance 1 tasks 0 function.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX0_INTERRUPT_PRIORITY
    string "Interrupt Priority Level"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT
    range INT_PRIORITY_LEVEL
    default "INT_PRIORITY_LEVEL2" if TEST_NUMBER_${INSTANCE}_SELECTED = "Sample Module Functional Test 0"
    default "INT_DISABLE_INTERRUPT"
    ---help---
    Select the interrupt priority level for library ${INSTANCE} instance 1 tasks 0 function.
    ---endhelp---

endmenu  # Library Instance 1 Tasks 0 ISR Configuration

###############################################################################
# Library Instance 2 Tasks 0 ISR Configuration Menu

menu "Library ${INSTANCE} Instance 2 Tasks 0 ISR Configuration"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX0_INTERRUPT_VECTOR
    string "Interrupt Vector"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT
    range INT_VECTOR
    default "INT_VECTOR_CT"
    ---help---
    Select the interrupt vector from which library ${INSTANCE} instance 2 tasks 0
    function should be called.
    ---endhelp---

comment "The XC32 compiler interrupt vector must represent the same vector as the above selection."

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX0_INTERRUPT_VECTOR_XC32
    string "Interrupt Vector as defined by XC32 Compiler"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT
    default "_CORE_TIMER_VECTOR"
    ---help---
    Enter the interrupt vector name as defined by the XC32 compiler.  This must
    represent the same vector as selected in the "Interrupt Vector".
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX0_INTERRUPT_SOURCE
    string "Interrupt Source"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT
    range INT_SOURCE
    default "INT_SOURCE_TIMER_CORE"
    ---help---
    Select the interrupt source for library ${INSTANCE} instance 2 tasks 0 function.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX0_INTERRUPT_PRIORITY
    string "Priority Level"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT
    range INT_PRIORITY_LEVEL
    default "INT_DISABLE_INTERRUPT"
    ---help---
    Select the interrupt priority level for library ${INSTANCE} instance 2 tasks 0 function.
    ---endhelp---

endmenu  # Library Instance 2 Tasks 0 ISR Configuration


###############################################################################
# Library Instance 0 Tasks 0 RTOS Configuration Menu

menu "Library ${INSTANCE} Instance 0 Tasks 0 RTOS Configuration"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_0_RTOS_DAEMON
    string "Run Tasks Function As"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_0_RTOS_TASK_SIZE
    int "Task Size"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_0_RTOS_DAEMON = "Standalone"
    default 512

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_0_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_0_RTOS_DAEMON = "Standalone"
    default 2

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_0_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_0_RTOS_DAEMON = "Standalone"
    default y

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_0_RTOS_DELAY
    int "Task Delay"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_0_RTOS_DAEMON = "Standalone"
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_0_RTOS_USE_DELAY
    default 20

endmenu


###############################################################################
# Library Instance 1 Tasks 0 RTOS Configuration Menu

menu "Library ${INSTANCE} Instance 1 Tasks 0 RTOS Configuration"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_0_RTOS_DAEMON
    string "Run Tasks Function As"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_0_RTOS_TASK_SIZE
    int "Task Size"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_0_RTOS_DAEMON = "Standalone"
    default 512

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_0_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_0_RTOS_DAEMON = "Standalone"
    default 2

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_0_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_0_RTOS_DAEMON = "Standalone"
    default y

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_0_RTOS_DELAY
    int "Task Delay"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_0_RTOS_DAEMON = "Standalone"
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_0_RTOS_USE_DELAY
    default 20

endmenu


###############################################################################
# Library Instance 2 Tasks 0 RTOS Configuration Menu

menu "Library ${INSTANCE} Instance 2 Tasks 0 RTOS Configuration"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_0_RTOS_DAEMON
    string "Run Tasks Function As"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_0_RTOS_TASK_SIZE
    int "Task Size"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_0_RTOS_DAEMON = "Standalone"
    default 512

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_0_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_0_RTOS_DAEMON = "Standalone"
    default 2

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_0_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_0_RTOS_DAEMON = "Standalone"
    default y

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_0_RTOS_DELAY
    int "Task Delay"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX0_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_0_RTOS_DAEMON = "Standalone"
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_0_RTOS_USE_DELAY
    default 20

endmenu


###################
# Library Tasks 1 #
###################

config TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1
    string "Tasks function 1"
    depends on USE_TEST_HARNESS
    default "NULL"
    ---help---
    Name of the second tasks function for library under test ${INSTANCE}.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT
    bool "Call Tasks function 1 from ISR?"
    depends on USE_TEST_HARNESS
    default n
    ---help---
    Select if you want Library ${INSTANCE} Tasks function 1 to be called from 
    an Interrupt Service Routine (ISR).  Do not select if you want it to be polled.
    ---endhelp---

###############################################################################
# Library Instance 0 Tasks 1 ISR Configuration Menu

menu "Library ${INSTANCE} Instance 0 Tasks 1 ISR Configuration"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX1_INTERRUPT_VECTOR
    string "Interrupt Vector"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT
    range INT_VECTOR
    default "INT_VECTOR_CT"
    ---help---
    Select the interrupt vector from which library ${INSTANCE} instance 0 tasks 1
    function should be called.
    ---endhelp---

comment "The XC32 compiler interrupt vector must represent the same vector as the above selection."

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX1_INTERRUPT_VECTOR_XC32
    string "Interrupt Vector as defined by XC32 Compiler"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT
    default "_CORE_TIMER_VECTOR"
    ---help---
    Enter the interrupt vector name as defined by the XC32 compiler.  This must
    represent the same vector as selected in the "Interrupt Vector".
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX1_INTERRUPT_SOURCE
    string "Interrupt Source"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT
    range INT_SOURCE
    default "INT_SOURCE_TIMER_CORE"
    ---help---
    Select the interrupt source for library ${INSTANCE} instance 0 tasks 1 function.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX1_INTERRUPT_PRIORITY
    string "Interrupt Priority Level"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT
    range INT_PRIORITY_LEVEL
    default "INT_DISABLE_INTERRUPT"
    ---help---
    Select the interrupt priority level for library ${INSTANCE} instance 0 tasks 1 function.
    ---endhelp---

endmenu  # Library Instance 0 Tasks 1 ISR Configuration

###############################################################################
# Library Instance 1 Tasks 1 ISR Configuration Menu

menu "Library ${INSTANCE} Instance 1 Tasks 1 ISR Configuration"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX1_INTERRUPT_VECTOR
    string "Interrupt Vector"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT
    range INT_VECTOR
    default "INT_VECTOR_CT"
    ---help---
    Select the interrupt vector from which library ${INSTANCE} instance 1 tasks 1
    function should be called.
    ---endhelp---

comment "The XC32 compiler interrupt vector must represent the same vector as the above selection."

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX1_INTERRUPT_VECTOR_XC32
    string "Interrupt Vector as defined by XC32 Compiler"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT
    default "_CORE_TIMER_VECTOR"
    ---help---
    Enter the interrupt vector name as defined by the XC32 compiler.  This must
    represent the same vector as selected in the "Interrupt Vector".
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX1_INTERRUPT_SOURCE
    string "Interrupt Source"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT
    range INT_SOURCE
    default "INT_SOURCE_TIMER_CORE"
    ---help---
    Select the interrupt source for library ${INSTANCE} instance 1 tasks 1 function.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX1_INTERRUPT_PRIORITY
    string "Interrupt Priority Level"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT
    range INT_PRIORITY_LEVEL
    default "INT_DISABLE_INTERRUPT"
    ---help---
    Select the interrupt priority level for library ${INSTANCE} instance 1 tasks 1 function.
    ---endhelp---

endmenu  # Library Instance 1 Tasks 1 ISR Configuration

###############################################################################
# Library Instance 2 Tasks 1 ISR Configuration Menu

menu "Library ${INSTANCE} Instance 2 Tasks 1 ISR Configuration"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX1_INTERRUPT_VECTOR
    string "Interrupt Vector"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT
    range INT_VECTOR
    default "INT_VECTOR_CT"
    ---help---
    Select the interrupt vector from which library ${INSTANCE} instance 2 tasks 1
    function should be called.
    ---endhelp---

comment "The XC32 compiler interrupt vector must represent the same vector as the above selection."

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX1_INTERRUPT_VECTOR_XC32
    string "Interrupt Vector as defined by XC32 Compiler"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT
    default "_CORE_TIMER_VECTOR"
    ---help---
    Enter the interrupt vector name as defined by the XC32 compiler.  This must
    represent the same vector as selected in the "Interrupt Vector".
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX1_INTERRUPT_SOURCE
    string "Interrupt Source"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT
    range INT_SOURCE
    default "INT_SOURCE_TIMER_CORE"
    ---help---
    Select the interrupt source for library ${INSTANCE} instance 2 tasks 1 function.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX1_INTERRUPT_PRIORITY
    string "Interrupt Priority Level"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT
    range INT_PRIORITY_LEVEL
    default "INT_DISABLE_INTERRUPT"
    ---help---
    Select the interrupt priority level for library ${INSTANCE} instance 2 tasks 1 function.
    ---endhelp---

endmenu  # Library Instance 2 Tasks 1 ISR Configuration

###############################################################################
# Library Instance 0 Tasks 1 RTOS Configuration Menu

menu "Library ${INSTANCE} Instance 0 Tasks 1 RTOS Configuration"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_1_RTOS_DAEMON
    string "Run Tasks Function As"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_1_RTOS_TASK_SIZE
    int "Task Size"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_1_RTOS_DAEMON = "Standalone"
    default 512

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_1_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_1_RTOS_DAEMON = "Standalone"
    default 2

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_1_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_1_RTOS_DAEMON = "Standalone"
    default y

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_1_RTOS_DELAY
    int "Task Delay"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_1_RTOS_DAEMON = "Standalone"
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_1_RTOS_USE_DELAY
    default 20

endmenu

###############################################################################
# Library Instance 1 Tasks 1 RTOS Configuration Menu

menu "Library ${INSTANCE} Instance 1 Tasks 1 RTOS Configuration"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_1_RTOS_DAEMON
    string "Run Tasks Function As"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_1_RTOS_TASK_SIZE
    int "Task Size"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_1_RTOS_DAEMON = "Standalone"
    default 512

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_1_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_1_RTOS_DAEMON = "Standalone"
    default 2

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_1_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_1_RTOS_DAEMON = "Standalone"
    default y

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_1_RTOS_DELAY
    int "Task Delay"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_1_RTOS_DAEMON = "Standalone"
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_1_RTOS_USE_DELAY
    default 20

endmenu

###############################################################################
# Library Instance 2 Tasks 1 RTOS Configuration Menu

menu "Library ${INSTANCE} Instance 2 Tasks 1 RTOS Configuration"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_1_RTOS_DAEMON
    string "Run Tasks Function As"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_1_RTOS_TASK_SIZE
    int "Task Size"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_1_RTOS_DAEMON = "Standalone"
    default 512

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_1_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_1_RTOS_DAEMON = "Standalone"
    default 2

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_1_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_1_RTOS_DAEMON = "Standalone"
    default y

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_1_RTOS_DELAY
    int "Task Delay"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX1_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_1_RTOS_DAEMON = "Standalone"
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_1_RTOS_USE_DELAY
    default 20

endmenu


###################
# Library Tasks 2 #
###################

config TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2
    string "Tasks function 2"
    depends on USE_TEST_HARNESS
    default "NULL"
    ---help---
    Name of the third tasks function for library under test ${INSTANCE}.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT
    bool "Call Tasks function 2 from ISR?"
    depends on USE_TEST_HARNESS
    default n
    ---help---
    Select if you want Library ${INSTANCE} Tasks function 2 to be called from 
    an Interrupt Service Routine (ISR).  Do not select if you want it to be polled.
    ---endhelp---

###############################################################################
# Library Instance 0 Tasks 2 ISR Configuration Menu

menu "Library ${INSTANCE} Instance 0 Tasks 2 ISR Configuration"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX2_INTERRUPT_VECTOR
    string "Interrupt Vector"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT
    range INT_VECTOR
    default "INT_VECTOR_CT"
    ---help---
    Select the interrupt vector from which library ${INSTANCE} instance 0 tasks 2
    function should be called.
    ---endhelp---

comment "The XC32 compiler interrupt vector must represent the same vector as the above selection."

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX2_INTERRUPT_VECTOR_XC32
    string "Interrupt Vector as defined by XC32 Compiler"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT
    default "_CORE_TIMER_VECTOR"
    ---help---
    Enter the interrupt vector name as defined by the XC32 compiler.  This must
    represent the same vector as selected in the "Interrupt Vector".
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX2_INTERRUPT_SOURCE
    string "Interrupt Source"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT
    range INT_SOURCE
    default "INT_SOURCE_TIMER_CORE"
    ---help---
    Select the interrupt source for library ${INSTANCE} instance 0 tasks 2 function.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX2_INTERRUPT_PRIORITY
    string "Interrupt Priority Level"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT
    range INT_PRIORITY_LEVEL
    default "INT_DISABLE_INTERRUPT"
    ---help---
    Select the interrupt priority level for library ${INSTANCE} instance 0 tasks 2 function.
    ---endhelp---

endmenu  # Library Instance 0 Tasks 2 ISR Configuration

###############################################################################
# Library Instance 1 Tasks 2 ISR Configuration Menu

menu "Library ${INSTANCE} Instance 1 Tasks 2 ISR Configuration"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX2_INTERRUPT_VECTOR
    string "Interrupt Vector"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT
    range INT_VECTOR
    default "INT_VECTOR_CT"
    ---help---
    Select the interrupt vector from which library ${INSTANCE} instance 1 tasks 2
    function should be called.
    ---endhelp---

comment "The XC32 compiler interrupt vector must represent the same vector as the above selection."

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX2_INTERRUPT_VECTOR_XC32
    string "Interrupt Vector as defined by XC32 Compiler"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT
    default "_CORE_TIMER_VECTOR"
    ---help---
    Enter the interrupt vector name as defined by the XC32 compiler.  This must
    represent the same vector as selected in the "Interrupt Vector".
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX2_INTERRUPT_SOURCE
    string "Interrupt Source"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT
    range INT_SOURCE
    default "INT_SOURCE_TIMER_CORE"
    ---help---
    Select the interrupt source for library ${INSTANCE} instance 1 tasks 2 function.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX2_INTERRUPT_PRIORITY
    string "Interrupt Priority Level"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT
    range INT_PRIORITY_LEVEL
    default "INT_DISABLE_INTERRUPT"
    ---help---
    Select the interrupt priority level for library ${INSTANCE} instance 1 tasks 2 function.
    ---endhelp---

endmenu  # Library Instance 1 Tasks 2 ISR Configuration

###############################################################################
# Library Instance 2 Tasks 2 ISR Configuration Menu

menu "Library ${INSTANCE} Instance 2 Tasks 2 ISR Configuration"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX2_INTERRUPT_VECTOR
    string "Interrupt Vector"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT
    range INT_VECTOR
    default "INT_VECTOR_CT"
    ---help---
    Select the interrupt vector from which library ${INSTANCE} instance 2 tasks 2
    function should be called.
    ---endhelp---

comment "The XC32 compiler interrupt vector must represent the same vector as the above selection."

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX2_INTERRUPT_VECTOR_XC32
    string "Interrupt Vector as defined by XC32 Compiler"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT
    default "_CORE_TIMER_VECTOR"
    ---help---
    Enter the interrupt vector name as defined by the XC32 compiler.  This must
    represent the same vector as selected in the "Interrupt Vector".
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX2_INTERRUPT_SOURCE
    string "Interrupt Source"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT
    range INT_SOURCE
    default "INT_SOURCE_TIMER_CORE"
    ---help---
    Select the interrupt source for library ${INSTANCE} instance 2 tasks 2 function.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX2_INTERRUPT_PRIORITY
    string "Interrupt Priority Level"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT
    range INT_PRIORITY_LEVEL
    default "INT_DISABLE_INTERRUPT"
    ---help---
    Select the interrupt priority level for library ${INSTANCE} instance 2 tasks 2 function.
    ---endhelp---

endmenu  # Library Instance 2 Tasks 2 ISR Configuration


###############################################################################
# Library Instance 0 Tasks 2 RTOS Configuration Menu

menu "Library ${INSTANCE} Instance 0 Tasks 2 RTOS Configuration"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_2_RTOS_DAEMON
    string "Run Tasks Function As"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_2_RTOS_TASK_SIZE
    int "Task Size"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_2_RTOS_DAEMON = "Standalone"
    default 512

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_2_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_2_RTOS_DAEMON = "Standalone"
    default 2

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_2_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_2_RTOS_DAEMON = "Standalone"
    default y

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_2_RTOS_DELAY
    int "Task Delay"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_2_RTOS_DAEMON = "Standalone"
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_2_RTOS_USE_DELAY
    default 20

endmenu


###############################################################################
# Library Instance 1 Tasks 2 RTOS Configuration Menu

menu "Library ${INSTANCE} Instance 1 Tasks 2 RTOS Configuration"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_2_RTOS_DAEMON
    string "Run Tasks Function As"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_2_RTOS_TASK_SIZE
    int "Task Size"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_2_RTOS_DAEMON = "Standalone"
    default 512

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_2_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_2_RTOS_DAEMON = "Standalone"
    default 2

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_2_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_2_RTOS_DAEMON = "Standalone"
    default y

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_2_RTOS_DELAY
    int "Task Delay"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_2_RTOS_DAEMON = "Standalone"
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_2_RTOS_USE_DELAY
    default 20

endmenu


###############################################################################
# Library Instance 2 Tasks 2 RTOS Configuration Menu

menu "Library ${INSTANCE} Instance 2 Tasks 2 RTOS Configuration"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_2_RTOS_DAEMON
    string "Run Tasks Function As"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_2_RTOS_TASK_SIZE
    int "Task Size"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_2_RTOS_DAEMON = "Standalone"
    default 512

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_2_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_2_RTOS_DAEMON = "Standalone"
    default 2

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_2_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_2_RTOS_DAEMON = "Standalone"
    default y

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_2_RTOS_DELAY
    int "Task Delay"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX2_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_2_RTOS_DAEMON = "Standalone"
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_2_RTOS_USE_DELAY
    default 20

endmenu


###################
# Library Tasks 3 #
###################

config TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3
    string "Tasks function 3"
    depends on USE_TEST_HARNESS
    default "NULL"
    ---help---
    Name of the fourth tasks function for library under test ${INSTANCE}.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT
    bool "Call Tasks function 3 from ISR?"
    depends on USE_TEST_HARNESS
    default n
    ---help---
    Select if you want Library ${INSTANCE} Tasks function 3 to be called from 
    an Interrupt Service Routine (ISR).  Do not select if you want it to be polled.
    ---endhelp---

###############################################################################
# Library Instance 0 Tasks 3 ISR Configuration Menu

menu "Library ${INSTANCE} Instance 0 Tasks 3 ISR Configuration"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX3_INTERRUPT_VECTOR
    string "Interrupt Vector"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT
    range INT_VECTOR
    default "INT_VECTOR_CT"
    ---help---
    Select the interrupt vector from which library ${INSTANCE} instance 0 tasks 3
    function should be called.
    ---endhelp---

comment "The XC32 compiler interrupt vector must represent the same vector as the above selection."

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX3_INTERRUPT_VECTOR_XC32
    string "Interrupt Vector as defined by XC32 Compiler"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT
    default "_CORE_TIMER_VECTOR"
    ---help---
    Enter the interrupt vector name as defined by the XC32 compiler.  This must
    represent the same vector as selected in the "Interrupt Vector".
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX3_INTERRUPT_SOURCE
    string "Interrupt Source"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT
    range INT_SOURCE
    default "INT_SOURCE_TIMER_CORE"
    ---help---
    Select the interrupt source for library ${INSTANCE} instance 0 tasks 3 function.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX3_INTERRUPT_PRIORITY
    string "Interrupt Priority Level"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT
    range INT_PRIORITY_LEVEL
    default "INT_DISABLE_INTERRUPT"
    ---help---
    Select the interrupt priority level for library ${INSTANCE} instance 0 tasks 3 function.
    ---endhelp---

endmenu  # Library Instance 0 Tasks 3 ISR Configuration

###############################################################################
# Library Instance 1 Tasks 3 ISR Configuration Menu

menu "Library ${INSTANCE} Instance 1 Tasks 3 ISR Configuration"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX3_INTERRUPT_VECTOR
    string "Interrupt Vector"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT
    range INT_VECTOR
    default "INT_VECTOR_CT"
    ---help---
    Select the interrupt vector from which library ${INSTANCE} instance 1 tasks 3
    function should be called.
    ---endhelp---

comment "The XC32 compiler interrupt vector must represent the same vector as the above selection."

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX3_INTERRUPT_VECTOR_XC32
    string "Interrupt Vector as defined by XC32 Compiler"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT
    default "_CORE_TIMER_VECTOR"
    ---help---
    Enter the interrupt vector name as defined by the XC32 compiler.  This must
    represent the same vector as selected in the "Interrupt Vector".
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX3_INTERRUPT_SOURCE
    string "Interrupt Source"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT
    range INT_SOURCE
    default "INT_SOURCE_TIMER_CORE"
    ---help---
    Select the interrupt source for library ${INSTANCE} instance 1 tasks 3 function.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX3_INTERRUPT_PRIORITY
    string "Interrupt Priority Level"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT
    range INT_PRIORITY_LEVEL
    default "INT_DISABLE_INTERRUPT"
    ---help---
    Select the interrupt priority level for library ${INSTANCE} instance 1 tasks 3 function.
    ---endhelp---

endmenu  # Library Instance 1 Tasks 3 ISR Configuration

###############################################################################
# Library Instance 2 Tasks 3 ISR Configuration Menu

menu "Library ${INSTANCE} Instance 2 Tasks 3 ISR Configuration"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX3_INTERRUPT_VECTOR
    string "Interrupt Vector"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT
    range INT_VECTOR
    default "INT_VECTOR_CT"
    ---help---
    Select the interrupt vector from which library ${INSTANCE} instance 2 tasks 3
    function should be called.
    ---endhelp---

comment "The XC32 compiler interrupt vector must represent the same vector as the above selection."

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX3_INTERRUPT_VECTOR_XC32
    string "Interrupt Vector as defined by XC32 Compiler"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT
    default "_CORE_TIMER_VECTOR"
    ---help---
    Enter the interrupt vector name as defined by the XC32 compiler.  This must
    represent the same vector as selected in the "Interrupt Vector".
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX3_INTERRUPT_SOURCE
    string "Interrupt Source"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT
    range INT_SOURCE
    default "INT_SOURCE_TIMER_CORE"
    ---help---
    Select the interrupt source for library ${INSTANCE} instance 2 tasks 3 function.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX3_INTERRUPT_PRIORITY
    string "Interrupt Priority Level"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT
    range INT_PRIORITY_LEVEL
    default "INT_DISABLE_INTERRUPT"
    ---help---
    Select the interrupt priority level for library ${INSTANCE} instance 2 tasks 3 function.
    ---endhelp---

endmenu  # Library Instance 2 Tasks 3 ISR Configuration


###############################################################################
# Library Instance 0 Tasks 3 RTOS Configuration Menu

menu "Library ${INSTANCE} Instance 0 Tasks 3 RTOS Configuration"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_3_RTOS_DAEMON
    string "Run Tasks Function As"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_3_RTOS_TASK_SIZE
    int "Task Size"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_3_RTOS_DAEMON = "Standalone"
    default 512

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_3_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_3_RTOS_DAEMON = "Standalone"
    default 2

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_3_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_3_RTOS_DAEMON = "Standalone"
    default y

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_3_RTOS_DELAY
    int "Task Delay"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_3_RTOS_DAEMON = "Standalone"
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_3_RTOS_USE_DELAY
    default 20

endmenu


###############################################################################
# Library Instance 1 Tasks 3 RTOS Configuration Menu

menu "Library ${INSTANCE} Instance 1 Tasks 3 RTOS Configuration"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_3_RTOS_DAEMON
    string "Run Tasks Function As"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_3_RTOS_TASK_SIZE
    int "Task Size"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_3_RTOS_DAEMON = "Standalone"
    default 512

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_3_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_3_RTOS_DAEMON = "Standalone"
    default 2

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_3_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_3_RTOS_DAEMON = "Standalone"
    default y

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_3_RTOS_DELAY
    int "Task Delay"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_3_RTOS_DAEMON = "Standalone"
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_3_RTOS_USE_DELAY
    default 20

endmenu


###############################################################################
# Library Instance 2 Tasks 3 RTOS Configuration Menu

menu "Library ${INSTANCE} Instance 2 Tasks 3 RTOS Configuration"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_3_RTOS_DAEMON
    string "Run Tasks Function As"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_3_RTOS_TASK_SIZE
    int "Task Size"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_3_RTOS_DAEMON = "Standalone"
    default 512

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_3_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_3_RTOS_DAEMON = "Standalone"
    default 2

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_3_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_3_RTOS_DAEMON = "Standalone"
    default y

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_3_RTOS_DELAY
    int "Task Delay"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX3_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_3_RTOS_DAEMON = "Standalone"
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_3_RTOS_USE_DELAY
    default 20

endmenu


###################
# Library Tasks 4 #
###################

config TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4
    string "Tasks function 4"
    depends on USE_TEST_HARNESS
    default "NULL"
    ---help---
    Name of the fifth tasks function for library under test ${INSTANCE}.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT
    bool "Call Tasks function 4 from ISR?"
    depends on USE_TEST_HARNESS
    default n
    ---help---
    Select if you want Library ${INSTANCE} Tasks function 4 to be called from 
    an Interrupt Service Routine (ISR).  Do not select if you want it to be polled.
    ---endhelp---
    
###############################################################################
# Library Instance 0 Tasks 4 ISR Configuration Menu

menu "Library ${INSTANCE} Instance 0 Tasks 4 ISR Configuration"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX4_INTERRUPT_VECTOR
    string "Interrupt Vector"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT
    range INT_VECTOR
    default "INT_VECTOR_CT"
    ---help---
    Select the interrupt vector from which library ${INSTANCE} instance 0 tasks 4
    function should be called.
    ---endhelp---

comment "The XC32 compiler interrupt vector must represent the same vector as the above selection."

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX4_INTERRUPT_VECTOR_XC32
    string "Interrupt Vector as defined by XC32 Compiler"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT
    default "_CORE_TIMER_VECTOR"
    ---help---
    Enter the interrupt vector name as defined by the XC32 compiler.  This must
    represent the same vector as selected in the "Interrupt Vector".
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX4_INTERRUPT_SOURCE
    string "Interrupt Source"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT
    range INT_SOURCE
    default "INT_SOURCE_TIMER_CORE"
    ---help---
    Select the interrupt source for library ${INSTANCE} instance 0 tasks 4 function.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE0_TASKS_IDX4_INTERRUPT_PRIORITY
    string "Interrupt Priority Level"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT
    range INT_PRIORITY_LEVEL
    default "INT_DISABLE_INTERRUPT"
    ---help---
    Select the interrupt priority level for library ${INSTANCE} instance 0 tasks 4 function.
    ---endhelp---

endmenu  # Library Instance 0 Tasks 4 ISR Configuration

###############################################################################
# Library Instance 1 Tasks 4 ISR Configuration Menu

menu "Library ${INSTANCE} Instance 1 Tasks 4 ISR Configuration"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX4_INTERRUPT_VECTOR
    string "Interrupt Vector"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT
    range INT_VECTOR
    default "INT_VECTOR_CT"
    ---help---
    Select the interrupt vector from which library ${INSTANCE} instance 1 tasks 4
    function should be called.
    ---endhelp---

comment "The XC32 compiler interrupt vector must represent the same vector as the above selection."

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX4_INTERRUPT_VECTOR_XC32
    string "Interrupt Vector as defined by XC32 Compiler"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT
    default "_CORE_TIMER_VECTOR"
    ---help---
    Enter the interrupt vector name as defined by the XC32 compiler.  This must
    represent the same vector as selected in the "Interrupt Vector".
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX4_INTERRUPT_SOURCE
    string "Interrupt Source"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT
    range INT_SOURCE
    default "INT_SOURCE_TIMER_CORE"
    ---help---
    Select the interrupt source for library ${INSTANCE} instance 1 tasks 4 function.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE1_TASKS_IDX4_INTERRUPT_PRIORITY
    string "Interrupt Priority Level"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT
    range INT_PRIORITY_LEVEL
    default "INT_DISABLE_INTERRUPT"
    ---help---
    Select the interrupt priority level for library ${INSTANCE} instance 1 tasks 4 function.
    ---endhelp---

endmenu  # Library Instance 1 Tasks 4 ISR Configuration

###############################################################################
# Libary Instance 2 Tasks 4 ISR Configuration Menu

menu "Library ${INSTANCE} Instance 2 Tasks 4 ISR Configuration"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX4_INTERRUPT_VECTOR
    string "Interrupt Vector"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT
    range INT_VECTOR
    default "INT_VECTOR_CT"
    ---help---
    Select the interrupt vector from which library ${INSTANCE} instance 2 tasks 4
    function should be called.
    ---endhelp---

comment "The XC32 compiler interrupt vector must represent the same vector as the above selection."

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX4_INTERRUPT_VECTOR_XC32
    string "Interrupt Vector as defined by XC32 Compiler"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT
    default "_CORE_TIMER_VECTOR"
    ---help---
    Enter the interrupt vector name as defined by the XC32 compiler.  This must
    represent the same vector as selected in the "Interrupt Vector".
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX4_INTERRUPT_SOURCE
    string "Interrupt Source"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT
    range INT_SOURCE
    default "INT_SOURCE_TIMER_CORE"
    ---help---
    Select the interrupt source for library ${INSTANCE} instance 2 tasks 4 function.
    ---endhelp---

config TEST_LIBRARY_IDX${INSTANCE}_INSTANCE2_TASKS_IDX4_INTERRUPT_PRIORITY
    string "Interrupt Priority Level"
    depends on TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT
    range INT_PRIORITY_LEVEL
    default "INT_DISABLE_INTERRUPT"
    ---help---
    Select the interrupt priority level for library ${INSTANCE} instance 2 tasks 4 function.
    ---endhelp---

endmenu  # Library Instance 2 Tasks 4 ISR Configuration


###############################################################################
# Library Instance 0 Tasks 4 RTOS Configuration Menu

menu "Library ${INSTANCE} Instance 0 Tasks 4 RTOS Configuration"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_4_RTOS_DAEMON
    string "Run Tasks Function As"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_4_RTOS_TASK_SIZE
    int "Task Size"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_4_RTOS_DAEMON = "Standalone"
    default 512

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_4_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_4_RTOS_DAEMON = "Standalone"
    default 2

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_4_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_4_RTOS_DAEMON = "Standalone"
    default y

config TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_4_RTOS_DELAY
    int "Task Delay"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_4_RTOS_DAEMON = "Standalone"
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_0_TASKS_4_RTOS_USE_DELAY
    default 20

endmenu


###############################################################################
# Library Instance 1 Tasks 4 RTOS Configuration Menu

menu "Library ${INSTANCE} Instance 1 Tasks 4 RTOS Configuration"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_4_RTOS_DAEMON
    string "Run Tasks Function As"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_4_RTOS_TASK_SIZE
    int "Task Size"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_4_RTOS_DAEMON = "Standalone"
    default 512

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_4_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_4_RTOS_DAEMON = "Standalone"
    default 2

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_4_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_4_RTOS_DAEMON = "Standalone"
    default y

config TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_4_RTOS_DELAY
    int "Task Delay"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_4_RTOS_DAEMON = "Standalone"
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_1_TASKS_4_RTOS_USE_DELAY
    default 20

endmenu


###############################################################################
# Library Instance 2 Tasks 4 RTOS Configuration Menu

menu "Library ${INSTANCE} Instance 2 Tasks 4 RTOS Configuration"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_4_RTOS_DAEMON
    string "Run Tasks Function As"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n
    range 3RDPARTY_RTOS_SYS_TASKS_OPTIONS
    default "Standalone"

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_4_RTOS_TASK_SIZE
    int "Task Size"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_4_RTOS_DAEMON = "Standalone"
    default 512

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_4_RTOS_TASK_PRIORITY
    int "Task Priority"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_4_RTOS_DAEMON = "Standalone"
    default 2

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_4_RTOS_USE_DELAY
    bool "Use Task Delay?"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_4_RTOS_DAEMON = "Standalone"
    default y

config TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_4_RTOS_DELAY
    int "Task Delay"
    depends on USE_3RDPARTY_RTOS && TEST_LIBRARY_IDX${INSTANCE}_TASKS_IDX4_INTERRUPT = n
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_4_RTOS_DAEMON = "Standalone"
    depends on TEST_LIBRARY_${INSTANCE}_INSTANCE_2_TASKS_4_RTOS_USE_DELAY
    default 20

endmenu


endmenu # Library Under Test Configuration Option  
 

endif
