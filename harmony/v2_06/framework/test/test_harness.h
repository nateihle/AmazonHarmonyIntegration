/*******************************************************************************
  MPLAB Harmony Test Harness Header File

  Company:
    Microchip Technology Inc.

  File Name:
    test_harness.h

  Summary:
    This header file provides prototypes and definitions for the MPLAB Harmony
    test harness.

  Description:
    This header file provides function prototypes and data type definitions for
    the test harness interface.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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
//DOM-IGNORE-END

#ifndef _TEST_HARNESS_H
#define _TEST_HARNESS_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system/common/sys_module.h"
#include "system/int/sys_int.h"


// *****************************************************************************
// *****************************************************************************
// Section: Constants & Macros
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Maximum Number of Instances of a Library Under Test

  Summary:
    Defines the maximum number of instances of a single library under test.

  Remarks:
    Refer to config/test_config_template.h for additional information.
 */

#ifndef TEST_HARNESS_MAX_NUM_INSTANCES_PER_LIBRARY
    #define TEST_HARNESS_MAX_NUM_INSTANCES_PER_LIBRARY   3
    #warning "TEST_HARNESS_MAX_NUM_INSTANCES_PER_LIBRARY to 3 instances per library."
#endif

// *****************************************************************************
/* Maximum Number of Tasks in a Library Under Test

  Summary:
    Defines the maximum number of "Tasks" routines allowed by a single library
    under test.

  Remarks:
    Refer to config/test_config_template.h for additional information.
 */

#ifndef TEST_HARNESS_MAX_NUM_TASKS_PER_LIBRARY
    #define TEST_HARNESS_MAX_NUM_TASKS_PER_LIBRARY   5
    #warning "TEST_HARNESS_MAX_NUM_TASKS_PER_LIBRARY defaulting to 5 tasks per library under test instance."
#endif


// *****************************************************************************
/* Maximum Number of Tasks in a Test

  Summary:
    Defines the maximum number of "Tasks" routines allowed for a single test.

  Remarks:
    Refer to config/test_config_template.h for additional information.
 */

#ifndef TEST_HARNESS_MAX_NUM_TASKS
    #define TEST_HARNESS_MAX_NUM_TASKS              5
    #warning "TEST_HARNESS_MAX_NUM_TASKS defaulting to 5 tasks per library under test instance."
#endif


// *****************************************************************************
/* Test List Position Indexes

  Summary:
    Convenient macros to identify the position of a test in the test list.
 */

#define TEST_POS_0   0
#define TEST_POS_1   1
#define TEST_POS_2   2
#define TEST_POS_3   3
#define TEST_POS_4   4
#define TEST_POS_5   5
#define TEST_POS_6   6
#define TEST_POS_7   7
#define TEST_POS_8   8
#define TEST_POS_9   9
#define TEST_POS_10  10
#define TEST_POS_11  11
#define TEST_POS_12  12
#define TEST_POS_13  13
#define TEST_POS_14  14
#define TEST_POS_15  15


// *****************************************************************************
/* Test Tasks Function Indexes

  Summary:
    Convenient macros to identify the test tasks function indexes.
 */

#define TEST_TASKS_FUNCTION_0   0
#define TEST_TASKS_FUNCTION_1   1
#define TEST_TASKS_FUNCTION_2   2
#define TEST_TASKS_FUNCTION_3   3
#define TEST_TASKS_FUNCTION_4   4


// *****************************************************************************
/* Test Library Under Test Instance Indexes

  Summary:
    Convenient macros to identify the index of a library under test.
 */

#define TEST_LUT_INDEX_0    0
#define TEST_LUT_INDEX_1    1
#define TEST_LUT_INDEX_2    2


// *****************************************************************************
/* Test Completed Status Value

  Summary:
    Convenient macros to define the "test completed" status value to be reported
    by a test's "Status" function.
 */
 
#define SYS_STATUS_TEST_COMPLETED ((SYS_STATUS)(SYS_STATUS_READY_EXTENDED + 1))



// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Module Under Test Data

  Summary:
    Holds information describing the system interface for the module under test.

  Description:
    This structure holds information describing the system interface for the
     module under test.
     
   Remarks:
    This structure uses the following configuration parameters.
    - TEST_HARNESS_MAX_NUM_TASKS_PER_MODULE         - To define the number of 
                                                      "Tasks" routines a module
                                                      under test may have.
    - TEST_HARNESS_MAX_NUM_INSTANCES_PER_MODULE     - To define the number of 
                                                      module instances that may 
                                                      be tested.
 */

typedef struct _library_under_test
{
    /* Pointer to the library's initialization routine. */
    SYS_MODULE_INITIALIZE_ROUTINE       initialize;

    /* Pointer to the library's reinitialization routine. */
    SYS_MODULE_REINITIALIZE_ROUTINE     reinitialize;

    /* Pointer to the library's deinitialization routine. */
    SYS_MODULE_DEINITIALIZE_ROUTINE     deinitialize;

    /* Pointer to the library's status routine. */
    SYS_MODULE_STATUS_ROUTINE           status;

    /* Number of instances of the library under test. */
    unsigned int                        numberOfInstances;

    /* Number of tasks routines required by each instance of the library under test. */
    unsigned int                        numberOfTasks;

    /* Array of pointers to the library's "Tasks" routine(s). */
    SYS_MODULE_TASKS_ROUTINE            tasks[TEST_HARNESS_MAX_NUM_TASKS_PER_LIBRARY];

    // Array of pointers to the library's init data structures. 
    SYS_MODULE_INIT                    *initData[TEST_HARNESS_MAX_NUM_INSTANCES_PER_LIBRARY];

} LIBRARY_UNDER_TEST;


/*******************************************************************************
  Function:
    SYS_MODULE_OBJ TEST_<MODULE>_Initialize) (
                        const SYS_MODULE_INDEX index,
                        const SYS_MODULE_INIT * const init )

  Summary:
    Pointer to a routine that initializes a test module.

  Description:
    This data type is a pointer to a routine that initializes a test module.

  Precondition:
    The system and harness initialization must have been completed before
    the harness state machine will call the initialization routine for any 
    tests.

  Parameters:
    index           - Identifier for the test instance to be initialized

    init            - Pointer to the data structure containing any data
                      necessary to initialize the test. This pointer may
                      be null if no data is required and default initialization
                      is to be used.

  Returns:
    A handle to the instance of the test module that was initialized.  This
    handle is a necessary parameter to all of the other harness level routines
    for this test.

  Remarks:
    This function will only be called once per test instance when the harness
    is ready to initialize the test instance.
*/

typedef SYS_MODULE_INITIALIZE_ROUTINE   TEST_MODULE_INITIALIZE_ROUTINE;


/*******************************************************************************
  Function:
    void TEST_<MODULE>_Tasks ( SYS_MODULE_OBJ object, unsigned int index )

  Summary:
    Pointer to a routine that performs the tasks necessary to maintain the state
    machine of a test module system module.

  Description:
    This data type is a pointer to a routine that performs the tasks necessary
    to maintain the state machine of a test module module.

  Precondition:
    The system and harness initialization must have been completed and the 
    test's initialization routine called before the harness will call any of the 
    test's tasks routines.

  Parameters:
    object          - Handle to the test instance object
    
    index           - Index identifying which tasks function is being called.
                      This index can be used to identify if a single task 
                      function is called from a different tasks context.

  Returns:
    None.

  Example:
    None.

  Remarks:
    A test module can have one or more (up to TEST_HARNESS_MAX_NUM_TASKS)
    tasks functions.
*/

typedef void (* TEST_MODULE_TASKS_ROUTINE) ( SYS_MODULE_OBJ object, unsigned int index );


/*******************************************************************************
  Function:
    SYS_STATUS TEST_<MODULE>_Status (  SYS_MODULE_OBJ object )

  Summary:
    Pointer to a routine that gets the current status of a test module.

  Description:
    This data type is a pointer to a routine that gets the current status of a
    test module.

  Precondition:
    The system and harness initialization must have (and will be) completed and 
    the test module's initialization routine will have been called before the 
    harness will call the status routine for a test.

  Parameters:
    object          - Handle to the module instance

  Returns:
    One of the possible status codes from SYS_STATUS, extended to include:
    
    SYS_STATUS_TEST_COMPLETED - Indicates the test has completed.

  Example:
    None.

  Remarks:
    A test's status operation can be used to determine when the test is busy or
    has completed as well as to obtain general status of the test.
    
    If the status operation returns SYS_STATUS_BUSY, the previous operation
    has not yet completed. Once the status operation returns SYS_STATUS_READY,
    any previous operations have completed.

    The value of SYS_STATUS_ERROR is negative (-1). A module may define
    module-specific error values of less or equal SYS_STATUS_ERROR_EXTENDED
    (-10).

    The status function must NEVER block.

    If the status operation returns an error value the test has failed.
*/

typedef SYS_MODULE_STATUS_ROUTINE   TEST_MODULE_STATUS_ROUTINE;


/*******************************************************************************
  Function:
    bool TEST_<MODULE>_Start (  SYS_MODULE_INDEX index )
    
  Summary:
    Pointer to a routine that starts the test module.
	
  Description:
    This is a pointer to a routine that starts the test application.
	
  Conditions:
    The test harness application and the test application must have been 
    initialized and "running" (i.e., the test applications tasks function must
    be called from the test harness's state machine).
	
  Input:
    object  -  Object handle to the module instance
	
  Return:
    - true    - If the test module is successfully able to start testing.
    - false   - If the test module is unable to start testing.
	
  Example:
    None.
	
  Remarks:
    Each test module's "Start" function is called via a pointer to begin
    running the test contained in that module.
*/

typedef bool (* TEST_MODULE_START_FUNCTION) (  SYS_MODULE_OBJ object );


// *****************************************************************************
/* Test Data

  Summary:
    Holds data necessary to initialize and run a single test.

  Description:
    This structure holds the data necessary to initialize and run a single test.
*/

typedef struct _test_data
{
    /* Pointer to test name string. */
    char *                          name;

    /* Pointer to test's initialize routine. */
    TEST_MODULE_INITIALIZE_ROUTINE  initialize;

    /* Pointer to test's tasks routine. */
    TEST_MODULE_TASKS_ROUTINE       tasks[TEST_HARNESS_MAX_NUM_TASKS];

    /* Pointer to test's start routine. */
    TEST_MODULE_START_FUNCTION      start;

    /* Pointer to test's status routine. */
    TEST_MODULE_STATUS_ROUTINE      status;

    /* Test module's index. */
    SYS_MODULE_INDEX                index;

    /* Initial data for test module. */
    SYS_MODULE_INIT                *initData;

    /* Pointer to library under test data structure. */
    LIBRARY_UNDER_TEST             *library;

    /* Test module's object handle. */
    SYS_MODULE_OBJ                  testObj;

    /* Count of sub-tests that have passed. */
    unsigned int                    passed;

    /* Count of sub-tests that have been run. */
    unsigned int                    count;

    /* Flag indicating all sub-tests completed (module is idle). */
    bool                            completed;

} TEST_DATA;


// *****************************************************************************
/* Test Harness Initialization Data

  Summary:
    Holds the test harness's initialization data.

  Description:
    This structure holds the test harness's initialization data.
*/

typedef struct _test_init_data
{
    /* Number of tests. */
    unsigned int    numberOfTests;

    /* Pointer to test data array. */
    TEST_DATA      *tests;

} TEST_INIT_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Test Platform Dependency Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    bool TEST_PlatformInitialize ( void )

  Summary:
    Called by the test harness to initialize configuration-specific test 
    support.

  Description:
    This function is called by the test harness to initialize configuration-
    specific test support.

  Precondition:
    The system must have been initialized.

  Parameters:
    None.

  Returns:
    - true    - If successfully able to initialize the configuration-specific 
                test support.
     - false  - If not able to successfully initialize the configuration-specific
                test support.

  Example:
    None.

  Remarks:
    A weak implementation of this function is provided in the test harness 
    library.  If no external implementation is defined, then the internal 
    weak implementation (which simply returns "true") will be used.
*/

bool TEST_PlatformInitialize ( void );


/*******************************************************************************
  Function:
    void TEST_PlatformReady ( void )

  Summary:
    Indicates that the test configuration is ready and testing can begin.

  Description:
    This function indicates that the test configuration is ready and testing can 
    begin.  The purpose of this function is to allow the call to the 
    TEST_PlatformInitialize function to return false when the configuration
    requires some time to complete its initialization.

  Precondition:
    The system, and test harness must have been initialized.

  Parameters:
    None.

  Returns:
    None.

  Example:
    TEST_PlatformReady();

  Remarks:
    If TEST_PlatformInitialize returns "true", then it is not necessary
    to call this function.
*/

void TEST_PlatformReady ( void );


// *****************************************************************************
// *****************************************************************************
// Section: Test Interface Routines
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void TEST_HasPassedSubtest ( bool passed )

  Summary:
    Allows a test module to indicate that an individual subtest has completed 
    and identifies if it has passed or failed.

  Description:
    This function allows a test module to indicate that an individual subtest 
    has completed and identifies if it has passed or failed.

  Precondition:
    The application must have been initialized.

  Parameters:
    passed
	- true if the subtest has passed.
    - false if the subtest has failed.

  Returns:
    None.

  Example:
    <code>
    // Indicate that the subtest has passed.
    TEST_HasPassedSubtest(true);
    </code>

  Remarks:
    The test harness will accumulate and report subtest results using 
    SYS_PRINT and/or SYS_MESSAGE macros.  The test module does not need to 
    report results.  Tests only need to report debug info using SYS_DEBUG_* 
    macros to help identify the reason for any failure.
    
    A test is considered to have passed if all sub-tests pass.  If any sub-tests 
    fail, the test is considered to have failed.  If no subtest results are 
    reported, the test is considered to have failed.
*/

void TEST_HasPassedSubtest ( bool passed );

	
/*******************************************************************************
  Function:
    void TEST_HasCompleted ( void )

  Summary:
    Allows a test to indicate that it has completed.

  Description:
    This function allows a test to indicate that it has completed.  It is used
    to prematurely end the test.

  Precondition:
    The test harness must have been initialized.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    // Indicate that a test with one subtest has passed.
    TEST_HasPassedSubtest(true);
    TEST_HasCompleted();
    </code>

  Remarks:
    The harness will automatically end the test when the test's "Status"
    function indicates it has completed.
    
    The test harness will accumulate and report subtest results using 
    SYS_PRINT and/or SYS_MESSAGE macros.  The test does not need to report 
    results.  Test only need to report debug info using SYS_DEBUG_* macros to 
    help identify the reason for any failure.
    
    A test is considered to have passed if all sub-tests pass.  If any sub-tests 
    fail, the test is considered to have failed.  If no subtest results are 
    reported, the test is considered to have failed.
*/

void TEST_HasCompleted ( void );


/*******************************************************************************
  Function:
    bool TEST_LibraryInitialize ( void )

  Summary:
    Indicates that the test is ready for the harness to initialize the library 
    under test.

  Description:
    This function indicates that the test is ready for the harness to initialize 
    the library under test.

  Precondition:
    The system, test harness, and test must have been initialized.

  Parameters:
    None.

  Returns:
    - true        If initialization of the library under test completed.
    - false       If initialization of the library under test has not yet 
                  completed.

  Example:
    if (TEST_LibraryInitialize())
    {
        // begin testing
    }

  Remarks:
    None.
*/

bool TEST_LibraryInitialize ( void );


/*******************************************************************************
  Function:
    void TEST_TimerIncrement ( void )

  Summary:
    Increments test timeout timer.

  Description:
    This function increments the test timeout counter by a number of 
    milliseconds defined by TEST_TIMER_MS_PER_INCREMENT.

  Precondition:
    The system, test harness, test and timer must have been initialized.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    void __ISR(_TIMER_1_VECTOR, IPL1AUTO) _IntHandlerDrvTmrInstance0(void)
    {
        TEST_TimerIncrement();
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_1);
    }
    </code>

  Remarks:
    This function should be called by the interrupt or alarm callback from the
    timer used by the test harness.
*/

void TEST_TimerIncrement ( void );


// *****************************************************************************
// *****************************************************************************
// Section: Test Harness System Interface Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void TEST_Initialize ( const APP_TH_INIT_DATA *init )

  Summary:
     MPLAB Harmony test harness initialization routine.

  Description:
    This function initializes the MPLAB Harmony test harness.  It places the harness
    in its initial state and prepares it to run so that the TEST_HarnessTasks 
    function can be called.

  Precondition:
    All other system required initialization routines should be called before 
    calling this routine (in "SYS_Initialize").  This function should be called 
    before or in place of the application's in initialization function.

  Parameters:
    init        - Pointer to test initialization data.

  Returns:
    None.

  Example:
    <code>
    TEST_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
    
    The test harness will initialize tests and libraries under test later, 
    under control of its state machine.
*/

void TEST_Initialize ( const TEST_INIT_DATA *init );


/*******************************************************************************
  Function:
    void TEST_HarnessTasks ( void )

  Summary:
    MPLAB Harmony test harness' tasks function.

  Description:
    This routine is the tasks function of the MPLAB Harmony test harness and defines 
	the state machine and core logic of the test harness.

  Precondition:
    Any required system initialization functions must have been called and the
    "TEST_Initialize" function must have been called.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    TEST_HarnessTasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine or from an RTOS thread
    loop.
 */

void TEST_HarnessTasks ( void );


/*******************************************************************************
  Function:
    void TEST_Tasks( unsigned int number )

  Summary:
    Runs the current test's tasks functions.

  Description:
    This function runs the current test's tasks functions, once the harness
    determines that the test should run.

  Precondition:
    The system and test harness must have all been initialized.

  Parameters:
    number      - Test's tasks function number.  (Tests can have more than one
                  tasks function.

  Returns:
    None.

  Example:
    None.

  Remarks:
    This function uses the following configuration parameters.
    - TEST_IDLE_SLEEP_MS    - Number of milliseconds that the test will sleep
                              waiting for the module when it is busy.
*/

void TEST_Tasks( unsigned int number );


/******************************************************************************
  Function:
    void TEST_LibraryTasksPolled ( unsigned int             testIndex, 
                                   SYS_MODULE_INDEX         libraryIndex, 
                                   SYS_MODULE_TASKS_ROUTINE libraryTasks )

  Summary:
    Calls the a tasks function of a library under test when the test harness
    is ready to run the library.

  Description:
    This routine calls a tasks function of a library under test, indirectly 
    under control of the test harness once the test is ready to verify the 
    library.

  Precondition:
    The system, harness, and test must have been initialized.

  Parameters:
    testIndex       - Index in test list to the test for this library.
    
    libraryIndex    - Index in test list for this instance of the library.
    
    libraryTasks    - Pointer to the library's Tasks function.

  Returns:
    None.

  Example:
    <code>
    TEST_LibraryTasksPolled();
    </code>

  Remarks:
    This routine must be called from the SYS_Tasks function or the appropriate
    RTOS thread loop when the library under test is configured for polled 
    mode.
    
    This routine will prevent the library's tasks function from being called
    before the library has been initialized.
*/

void TEST_LibraryTasksPolled ( unsigned int             testIndex, 
                               SYS_MODULE_INDEX         libraryIndex, 
                               SYS_MODULE_TASKS_ROUTINE libraryTasks );


/******************************************************************************
  Function:
    void TEST_LibraryTasksISR ( unsigned int             testIndex, 
                                SYS_MODULE_INDEX         libraryTasks, 
                                SYS_MODULE_TASKS_ROUTINE mutTasks, 
                                INT_SOURCE               source )

  Summary:
    Calls an ISR-based tasks function of the library under test.

  Description:
    This routine calls an ISR-based tasks function of a library under test,
    indirectly under control of the test harness once the test is ready to 
    verify the library.

  Precondition:
    The system, harness, and test must have been initialized.

  Parameters:
    testIndex       - Index to the APP_TH_TEST_DATA data structure passed to
                      the test harness's "TEST_HarnessTasks" routine during system
                      initialization.
    
    moduleIndex     - Index to the appropriate module instance.
    
    libraryTasks    - Pointer to the appropriate "Tasks" function for the 
                      library under test.
    
    source          - Interrupt source ID.

  Returns:
    None.

  Example:
    <code>
    void __ISR(_TIMER_2_VECTOR, ipl2) _IntHandlerDrvTmrInstance1(void)
    {
        TEST_LibraryTasksISR(2, 0, SAMPLE_Tasks, INT_SOURCE_TIMER_2);
    }
    </code>

  Remarks:
    This routine must be called from the appropriate ISR vector function when
    the module under test is configured for interrupt mode.
*/

void TEST_LibraryTasksISR ( unsigned int             testIndex, 
                            SYS_MODULE_INDEX         moduleIndex, 
                            SYS_MODULE_TASKS_ROUTINE libraryTasks, 
                            INT_SOURCE               source );


#endif /* _TEST_HARNESS_H */
/*******************************************************************************
 End of File
 */