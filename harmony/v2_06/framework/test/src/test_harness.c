/*******************************************************************************
  MPLAB Harmony Test Harness Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    test_harness.c

  Summary:
    This file contains the source code for the MPLAB Harmony test harness.

  Description:
    This file contains the source code for the MPLAB Harmony test harness.  It 
    implements the logic of the harness' state machine.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "test/test_harness.h"
#include "peripheral/tmr/plib_tmr.h"
#include "osal/osal.h"
#include "system/debug/sys_debug.h"
#include "system/int/sys_int.h"


// *****************************************************************************
// *****************************************************************************
// Section: Configuration Option Defaults
// *****************************************************************************
// *****************************************************************************
/* This section provides default settings for test harness application 
   configuration options.
*/

/* Use timer 1 by default. */
#ifndef TEST_TIMER_ID
    #define TEST_TIMER_ID                       TMR_ID_1
    #warning "Test harness timeout defaulting to timer 1."
#endif

#ifndef TEST_TIMER_INTERRUPT_SOURCE
    #define TEST_TIMER_INTERRUPT_SOURCE         INT_SOURCE_TIMER_1
    #warning "Test harness timeouut defaulting to timer 1 interrupt source."
#endif

#ifndef TEST_TIMER_CLOCK_SOURCE
    #define TEST_TIMER_CLOCK_SOURCE             TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK
    #warning "Test harness timeout defaulting to peripheral clock source."
#endif

#ifndef TEST_TIMER_CLOCK_PRESCALER
    #define TEST_TIMER_CLOCK_PRESCALER          TMR_PRESCALE_VALUE_256
    #warning "Test harness timeout defaulting to divide-by-256 clock prescaler."
#endif

#ifndef TEST_TIMER_INCREMENT_PERIOD
    #define TEST_TIMER_INCREMENT_PERIOD         31250   // 100 ms @ 80MHZ / 256
    #warning "Test harness timeout defaulting to 31250 (100 ms @ 80MHz/256) timer period."
#endif

#ifndef TEST_TIMER_MS_PER_INCREMENT
    #define TEST_TIMER_MS_PER_INCREMENT         100
    #warning "Test harness timeout defaulting to 100 ms increments."
#endif

#ifndef TEST_TIMER_MS_TIMEOUT
    #define TEST_TIMER_MS_TIMEOUT               2000
    #warning "Test harness timeout defaulting to 2000 ms (2s) timeout."
#endif

#ifndef TEST_TIMER_INTERRUPT_VECTOR
    #define TEST_TIMER_INTERRUPT_VECTOR         INT_VECTOR_T1
    #warning "Test harness timeout defaulting to timer 1 interrupt vector."
#endif

#ifndef TEST_TIMER_INTERRUPT_PRIORITY
    #define TEST_TIMER_INTERRUPT_PRIORITY       INT_PRIORITY_LEVEL1
    #warning "Test harness timeout defaulting to interrupt priority level 1."
    /* If changing interrupt priority level, be sure to change vector function 
       attributes also. */
#endif

#ifndef TEST_TIMER_INTERRUPT_SUBPRIORITY
    #define TEST_TIMER_INTERRUPT_SUBPRIORITY    INT_SUBPRIORITY_LEVEL0
    #warning "Test harness timeout defaulting to interrupt subprioity level 0."
#endif

#ifndef TEST_IDLE_SLEEP_MS_LIBRARY
    #define TEST_IDLE_SLEEP_MS_LIBRARY          5
    #warning "Test harness library idle sleep time defaulting to 5 ms."
#endif 

#ifndef TEST_IDLE_SLEEP_MS
    #define TEST_IDLE_SLEEP_MS                  50
    #warning "Test harness idle sleep time defaulting to 50 ms."
#endif 


// *****************************************************************************
// *****************************************************************************
// Section: Internal Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Test Harness States

  Summary:
    Test harness states enumeration

  Description:
    This enumeration defines the valid test harness states.  These states 
    determine the behavior of the test harness at various times.
*/

typedef enum
{
	/* Prepare to run test sequences. */
	HARNESS_STATE_INIT = 0,

    /* Wait for preparations to complete. */
    HARNESS_STATE_INIT_COMPLETE,

    /* Initialize current test application. */
    HARNESS_STATE_TEST_INITIALIZE,

    /* Let the test run until it is ready to initialize the library under test. */
    HARNESS_STATE_RUN_TEST,

    /* Attempt to initialize the library under test. */
    HARNESS_STATE_LIBRARY_INITIALIZE,

    /* Start the current test application running. */
    HARNESS_STATE_TEST_START,

    /* Let the test and the library run until the test is completed. */
    HARNESS_STATE_RUN_TEST_AND_LIBRARY,

    /* Deinitialize the library under test. */
    HARNESS_STATE_LIBRARY_DEINITIALIZE,

    /* Wait for the library to complete deinitialization. */
    HARNESS_STATE_LIBRARY_DEINITIALIZE_WAIT,

    /* Accumulate the results after the test reports it has completed. */
    HARNESS_STATE_TEST_COMPLETED,

    /* Nothing more to do */
    HARNESS_STATE_IDLE

} TEST_HARNESS_STATES;


// *****************************************************************************
/* Test Harness Data

  Summary:
    Holds the test harness' data.

  Description:
    This structure holds the test harness' internal data.
 */

typedef struct _test_harness_data
{
    /* The test harness's current state */
    TEST_HARNESS_STATES state;

    /* Flag indicating when the test platform configuration is ready. */
    bool                platformReady;

    /* Flag indicating that the current test is running. */
    bool                testRunning;

    /* Flag indicating that the test is ready for the harness to initialize
    the library under test. */
    bool                testReady;

    /* Flag to control the library under test's tasks execution. */
    bool                libraryRunning;

    /* Number of tests. */
    size_t              numberOfTests;

    /* Pointer to the test array. */
    TEST_DATA          *testList;

    /* Pointer to the current test. */
    TEST_DATA          *testCurrent;

    /* Test time counter (in milliseconds) */
    unsigned int        testTime;

    /* Count of subtests run. */
    unsigned int        subtestsCount;

    /* Count of subtests passed. */
    unsigned int        subtestsPassed;

    /* Count of tests passed. */
    unsigned int        testsPassed;

    /* Count of tests run. */
    unsigned int        testsCount;

    /* Flag indicating over-all result (when completed == true) */
    bool                result;

    /* Semaphore to throttle test and harness execution. */
    OSAL_SEM_DECLARE(   testSemaphore );

    /* Semaphore to synchronize library under test tasks execution. */
    OSAL_SEM_DECLARE(   librarySemaphore );

    /* Array of library-under-test object handles for this library. */
    SYS_MODULE_OBJ      libraryObj[TEST_HARNESS_MAX_NUM_INSTANCES_PER_LIBRARY];

} TEST_HARNESS_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure is initialized by the TEST_Initialize function.
*/

static TEST_HARNESS_DATA harness;


// *****************************************************************************
// *****************************************************************************
// Section: Timeout Timer Support Functions
// *****************************************************************************
// *****************************************************************************
/* The test harness directly utilizes the selected timer peripheral.  The 
   selected timer must not be used by any other library in the system.
*/

/*******************************************************************************
  Function:
    void TimeoutTimerInitialize ( void )

  Summary:
    Initializes test timeout timer.

  Description:
    This function initializes the test timeout timer, leaving it stopped with 
    the interrupt disabled, but cleared and ready to run.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Remarks:
    Utilize the following required configuration parameters to setup the
    timer.
    - TEST_TIMER_ID                     - Timer instance ID (PLIB/HW level)
    - TEST_TIMER_INTERRUPT_SOURCE       - Interrupt source ID (for SYS_INT)
    - TEST_TIMER_CLOCK_SOURCE           - Timer clock source
    - TEST_TIMER_CLOCK_PRESCALER        - Timer clock prescaler (divisor)
    - TEST_TIMER_INCREMENT_PERIOD       - Timer increment period
    - TEST_TIMER_MS_PER_INCREMENT       - Milliseconds for each increment
    - TEST_TIMER_INTERRUPT_VECTOR       - Timer interrupt source ID
    - TEST_TIMER_INTERRUPT_PRIORITY     - Interrupt vector priority
    - TEST_TIMER_INTERRUPT_SUBPRIORITY  - Interrupt vector subpriority
*/

static inline void TimeoutTimerInitialize ( void )
{	
    /* Ensure timer is not running. */
    PLIB_TMR_Stop(TEST_TIMER_ID);

    /* Setup timer parameters. */
    PLIB_TMR_ClockSourceSelect(TEST_TIMER_ID, TEST_TIMER_CLOCK_SOURCE);
    PLIB_TMR_PrescaleSelect(TEST_TIMER_ID, TEST_TIMER_CLOCK_PRESCALER);
    PLIB_TMR_Mode16BitEnable(TEST_TIMER_ID);
    PLIB_TMR_Counter16BitClear(TEST_TIMER_ID);
    PLIB_TMR_Period16BitSet(TEST_TIMER_ID, TEST_TIMER_INCREMENT_PERIOD);

    /* Setup Interrupt */   
    SYS_INT_VectorPrioritySet(TEST_TIMER_INTERRUPT_VECTOR, TEST_TIMER_INTERRUPT_PRIORITY);
    SYS_INT_VectorSubprioritySet(TEST_TIMER_INTERRUPT_VECTOR, TEST_TIMER_INTERRUPT_SUBPRIORITY);
    SYS_INT_SourceEnable(TEST_TIMER_INTERRUPT_SOURCE);
}


/*******************************************************************************
  Function:
    void TimeoutTimerStart ( void )

  Summary:
    Starts the timeout timer counting.

  Description:
    This function starts the timeout timer counting.

  Precondition:
    TimeoutTimerInitialize must have been called.

  Parameters:
    None.

  Returns:
    none.

  Remarks:
    Utilize the following required configuration parameters to setup the
    timer.
    - TEST_TIMER_ID                     - Timer instance ID (PLIB/HW level)
*/

static inline void TimeoutTimerStart  (void )
{
    PLIB_TMR_Start(TEST_TIMER_ID);
}


/*******************************************************************************
  Function:
    void TimeoutTimerStop ( void )

  Summary:
    Stops timeout timer from counting.

  Description:
    This function stops timeout timer from counting.

  Precondition:
    TimeoutTimerInitialize must have been called.

  Parameters:
    None.

  Returns:
    none.

  Remarks:
    Utilize the following required configuration parameters to setup the
    timer.
    - TEST_TIMER_ID                     - Timer instance ID (PLIB/HW level)
*/

static inline void TimeoutTimerStop ( void )
{
    PLIB_TMR_Stop(TEST_TIMER_ID);
}


/*******************************************************************************
  Function:
    void TimeoutTimerCounterClear ( void )

  Summary:
    Clears the timeout counter (hardware counter, not the SW increment count).

  Description:
    This function clears the timeout counter (hardware counter, not the SW 
    increment count).

  Precondition:
    TimeoutTimerInitialize must have been called.

  Parameters:
    None.

  Returns:
    none.

  Remarks:
    Utilize the following required configuration parameters to setup the
    timer.
    - TEST_TIMER_ID                     - Timer instance ID (PLIB/HW level)
*/

static inline void TimeoutTimerCounterClear ( void )
{
    PLIB_TMR_Counter16BitClear(TEST_TIMER_ID);    
}


/*******************************************************************************
  Function:
    inline uint32_t TimeoutTimerCounterValueGet ( void )

  Summary:
    Gets the current value of the timeout timer HW timer.

  Description:
    This function gets the current value of the timeout timer HW timer.

  Precondition:
    TimeoutTimerInitialize must have been called.

  Parameters:
    None.

  Returns:
    none.

  Remarks:
    Utilize the following required configuration parameters to setup the
    timer.
    - TEST_TIMER_ID                     - Timer instance ID (PLIB/HW level)
*/

static inline uint32_t TimeoutTimerCounterValueGet ( void )
{
    return (uint32_t) PLIB_TMR_Counter16BitGet(TEST_TIMER_ID);
}


// *****************************************************************************
// *****************************************************************************
// Section: Test Platform Functions
// *****************************************************************************
// *****************************************************************************
/* The test harness provides the ability to perform hardware platform or 
   configuration-specific initialization and wait on that initialization to 
   complete.
*/

/*******************************************************************************
  Function:
    bool TEST_PlatformInitialize ( void )

  Summary:
    Initializes configuration-specific test support.

  Remarks:
    See prototype in test_harness.h.
    
    Local weak implementation, used if no global implementation is provided.
 */

bool __attribute__((weak)) TEST_PlatformInitialize ( void )
{
    return true;
}


// *****************************************************************************
// *****************************************************************************
// Section: Test Harness Interface Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void TEST_PlatformReady ( void )

  Remarks:
    See documentation in test_harness.h
*/

void TEST_PlatformReady ( void )
{
    harness.platformReady = true;
}


/*******************************************************************************
  Function:
    void TEST_HasPassedSubtest ( bool passed )

  Remarks:
    See documentation in test_harness.h
*/

void TEST_HasPassedSubtest ( bool passed )
{
    OSAL_CRITSECT_DATA_TYPE csStatus;

    /* Guard against interrupts & context switches. */
    csStatus = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);

    /* Increment the counts as appropriate. */
    harness.testCurrent->count++;
    if (passed)
    {
        harness.testCurrent->passed++;
    }

    OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, csStatus);
}


/*******************************************************************************
  Function:
    void TEST_HasCompleted ( void )

  Remarks:
    See documentation in test_harness.h
*/

void TEST_HasCompleted ( void )
{
    harness.testCurrent->completed = true;
}


/*******************************************************************************
  Function:
    bool TEST_LibraryInitialize ( void )

  Remarks:
    See documentation in test_harness.h
*/

bool TEST_LibraryInitialize ( void )
{
    harness.testReady = true;
    return false;
}


/*******************************************************************************
  Function:
    void TEST_TimerIncrement ( void )

  Remarks:
    See documentation in test_harness.h
    
    Utilize the following required configuration parameters to setup the
    timer.
    - TEST_TIMER_MS_PER_INCREMENT       - Milliseconds for each increment
*/

void TEST_TimerIncrement ( void )
{
    harness.testTime += TEST_TIMER_MS_PER_INCREMENT;
}


// *****************************************************************************
// *****************************************************************************
// Section: Test Harness Local Functions
// *****************************************************************************
// *****************************************************************************


/*******************************************************************************
  Function:
    bool PrepairForTests ( void )

  Summary:
    Prepares the test harness to start running tests.

  Description:
    This function prints the test harness banner and prepares the harness to 
    start running tests.

  Precondition:
    The system must have been initialized.

  Parameters:
    None.

  Returns:
    true    - If the harness is prepared to run tests.
    
    false   - If the harness is not prepared to run tests.

  Example:
    None.

  Remarks:
    Calls TEST_PlatformInitialize from the system configuration so that 
    configuation-specific initialization can be performed.
*/

static bool PrepairForTests ( void )
{
    bool result = true;

    // Start of test harness banner. */
    SYS_MESSAGE("\n***************************************************************************\n");

    // Verify that the test array is not empty.
    if (harness.numberOfTests <= 0)
    {
        SYS_MESSAGE("No tests provided.\n");
        result = false;
    }

    // Perform any platform or configuration specific initialization that is necessary.
    harness.platformReady = TEST_PlatformInitialize();
    if (!harness.platformReady)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_DEBUG, "Initializing test platform.\n");
    }

    return result;
}


/*******************************************************************************
  Function:
    bool InitializeTest ( void )

  Summary:
    Initializes a test application.

  Description:
    This function initializes test application.

  Precondition:
    The system and app must have been initialized.

  Parameters:
    None.

  Returns:
    true    - If able to initialize the test.
    
    false   - If unable to initialize the test.

  Example:
    None.

  Remarks:
    Calls the test's initialization function and ensures that it initialized
    correctly.
*/

static bool InitializeTest ( void )
{
    bool result = false;
    int i;

    if (NULL == harness.testCurrent)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_ERROR, "No test data available.\n");
    }
    else
    {
        /* Print test name header. */
        SYS_PRINT("Test:    %s\n", harness.testCurrent->name);

        /* Reset the test timeout. */
        TimeoutTimerCounterClear();
        harness.testTime = 0;
        TimeoutTimerStart();

        /* Re-initialize all library object handles. */
        for (i=0; i < TEST_HARNESS_MAX_NUM_INSTANCES_PER_LIBRARY; i++)
        {
            harness.libraryObj[i] = SYS_MODULE_OBJ_INVALID;
        }
        
        /* Initialize the test harness. */
        harness.testCurrent->count      = 0;
        harness.testCurrent->passed     = 0;
        harness.testCurrent->completed  = false;
        harness.testReady               = false;
        harness.testRunning             = false;
        harness.libraryRunning          = false;
        if (NULL != harness.testCurrent->initialize)
        {
            harness.testCurrent->testObj = harness.testCurrent->initialize(harness.testCurrent->index, harness.testCurrent->initData);
            if (SYS_MODULE_OBJ_INVALID == harness.testCurrent->testObj)
            {
                SYS_DEBUG_MESSAGE(SYS_ERROR_ERROR, "Test did not initialize.\n");
            }
            else
            {
                harness.testRunning = true;
                result = true;
            }
        }
        else
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_ERROR, "No initialization function provided.\n");
        }
    }

    return result;
}


/*******************************************************************************
  Function:
    bool InitializeLibraryUnderTest( void )

  Summary:
    Initializes the library currently under test.

  Description:
    This function initializes the library currently under test.

  Precondition:
    The system, harness, and test must have been initialized.

  Parameters:
    None.

  Returns:
    true        - If able to initialize all instances of the library under test.
    
    false       - If unable to initialize all instances of the library under
                  test.

  Example:
    None.

  Remarks:
    Called by the test app when it is ready to examine the library.
*/

static bool InitializeLibraryUnderTest( void )
{
    unsigned int            i;
    bool                    result;
    OSAL_CRITSECT_DATA_TYPE csStatus;


    /* Validate the library under test data pointer. */
    if (NULL == harness.testCurrent->library)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_ERROR, "No data available for the library under test.\n");
        return false;
    }

    /* Validate the number of instances of the library under test. */
    if (TEST_HARNESS_MAX_NUM_INSTANCES_PER_LIBRARY < harness.testCurrent->library->numberOfInstances)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_ERROR, "To many instances of the library under test.\n");
        return false;
    }

    /* Initialize all instances of the library under test. */
    result = true;
    for (i= 0; i < harness.testCurrent->library->numberOfInstances; i++)
    {
        /* If library has an initialization function, call it. */
        if (NULL != harness.testCurrent->library->initialize)
        {
            /* Must guard call to initialize to avoid race conditions with interrupts. 
               Libraries are normally initialized before interrupts are enabled. */
            csStatus = OSAL_CRIT_Enter(OSAL_CRIT_TYPE_HIGH);
            harness.libraryObj[i] = harness.testCurrent->library->initialize(i, harness.testCurrent->library->initData[i]);
            OSAL_CRIT_Leave(OSAL_CRIT_TYPE_HIGH, csStatus);

            /* Fail test if initialization fails. */
            if (SYS_MODULE_OBJ_INVALID == harness.libraryObj[i])
            {
                result = false;
            }
        }
        else
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_DEBUG, "Library has no initialization function\n");
        }
    }

    /* Let the library under test run. */
    if (true == result)
    {
        TEST_HasPassedSubtest(true);
        harness.libraryRunning = true;
        OSAL_SEM_Post(&harness.librarySemaphore);
    }
    else
    {
        TEST_HasPassedSubtest(false);
        SYS_DEBUG_MESSAGE(SYS_ERROR_ERROR, "One or more libraries under test failed to initialize.\n");
    }

    return result;
}


/*******************************************************************************
  Function:
    bool RunUntilTestComplete( void )

  Summary:
    Runs the test and all instances of the library until the test is complete.

  Description:
    This function runs the test and all instances of the library until the test 
    is complete.

  Precondition:
    The system, harness, test, and library must have all been initialized.

  Parameters:
    None.

  Returns:
    - true        - After the test has reported it is complete.
    - false       - If if the test has not reported it is complete.

  Example:
    None.

  Remarks:
    This function uses the following configuration parameters.
    - TEST_IDLE_SLEEP_MS    - Number of milliseconds that the test and 
                              harness will sleep waiting for the library
                              when it is busy.
*/

static bool RunUntilTestComplete( void )
{
    SYS_STATUS  testStatus = SYS_STATUS_UNINITIALIZED;

    /* Check the status of the test (if status function is supported). */
    if (NULL == harness.testCurrent->status)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_INFO, "Test does not provide a status function.\n");
    }
    else
    {
        testStatus = harness.testCurrent->status(harness.testCurrent->testObj);
    }

    /* Take appropriate action, based on the test status. */
    if (SYS_STATUS_TEST_COMPLETED == testStatus)
    {
        harness.testCurrent->completed = true;
    }
    else if (SYS_STATUS_BUSY != testStatus)
    {
        /* Block on a semaphore that is never "put" (in OS configurations) when 
        the test is not busy to give the library under test time to work. */
        OSAL_SEM_Pend(&harness.testSemaphore, TEST_IDLE_SLEEP_MS);
    }

    /* Stop the test when it reports it is complete and move on. */
    if (harness.testCurrent->completed)
    {
        harness.testRunning  = false;
    }
    return (harness.testCurrent->completed);
}


/*******************************************************************************
  Function:
    void DenitializeLibraryUnderTest( void )

  Summary:
    Deinitializes the library currently under test.

  Description:
    This function deinitializes the library currently under test.

  Precondition:
    The system, harness, and library under test must have been initialized.

  Parameters:
    None.

  Returns:
    None.

  Example:
    None.

  Remarks:
    None.
*/

static void DenitializeLibraryUnderTest( void )
{
    int instance;

    /* Prevent calls to the library's Tasks routine(s). */
    harness.libraryRunning = false;

    /* Deinitialize all instances of the library under test. */
    for (instance=0; instance < harness.testCurrent->library->numberOfInstances; instance++)
    {
        /* Deinitialize function is optional. */
        if (harness.testCurrent->library->deinitialize)
        {
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "Deinitializing library under test instance %d.\n", instance);
            harness.testCurrent->library->deinitialize(harness.libraryObj[instance]);
        }
    }
}


/*******************************************************************************
  Function:
    bool VerifyLibraryDeinitialized( void )

  Summary:
    Verifies that all instances of the library under test report their status 
    as deinitialized.

  Description:
    This function verifies that all instances of library under test report their
    status as deinitialized.


  Precondition:
    The system and harness must have been initialized and the library under test
    must have been deinitialized.

  Parameters:
    None.

  Returns:
    - true    - When all instances of the library under test report as 
                deinitialized (SYS_STATUS_UNINITIALIZED) or as having had an 
                error or the test times out.        
    - false   - If any instance of the library under test reports as still 
                running.              

  Example:
    None.

  Remarks:
    None.
*/

static bool VerifyLibraryDeinitialized( void )
{
    unsigned int    instance;
    SYS_STATUS      mutStatus;
    bool            result  = true; // Assume no errors & verify, below.
    bool            done    = true; // Assume we're done & verify, below.

    /* Check status of all instances of the library under test. */
    for (instance= 0; instance < harness.testCurrent->library->numberOfInstances; instance++)
    {
        /* Status function is optional. */
        if (harness.testCurrent->library->status)
        {
            mutStatus = harness.testCurrent->library->status(harness.libraryObj[instance]);
            if (mutStatus <= SYS_STATUS_ERROR)
            {
                // Failure
                SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "MUT instance %d error status %d.\n", instance, mutStatus);
                result = false;
            }
            else if (mutStatus > SYS_STATUS_UNINITIALIZED)
            {
                /* If any MUT instance is busy, we're not done. */
                done = false;
            }
        }
    }

    /* The test is done if any MUT instance reported an error. */
    if (false == result)
    {
        done = true;
    }

    /* Report deinitialize & status subtest results when done. */
    if (done)
    {
        TEST_HasPassedSubtest(result);
    }

    return done;
}


/*******************************************************************************
  Function:
    static void ReportTestResults ( void )

  Summary:
    Reports test results.

  Description:
    This function reports the results of the current test and either advances to 
    the next test or report the final results.

  Precondition:
    The test harness must have run at least one test.

  Parameters:
    None.

  Returns:
    None.

  Example:
    None.

  Remarks:
    None.
*/

static void ReportTestResults ( void )
{
    char *       __attribute__((unused))    result;   // Passed/Failed result printed to console
    unsigned int __attribute__((unused))    percentageSubtests = 0;
    unsigned int __attribute__((unused))    percentageTests = 0;

    /* Determine subtest results. */
    result = "Failed";
    if (harness.testCurrent->completed)
    {
        if (harness.testCurrent->count > 0)
        {
            /* Accumulate Total Results */
            harness.subtestsPassed += harness.testCurrent->passed;
            harness.subtestsCount  += harness.testCurrent->count;

            /* Determine Current Results */
            percentageSubtests = (harness.testCurrent->passed * 100) / harness.testCurrent->count;
            if (harness.testCurrent->passed == harness.testCurrent->count)
            {
                /* Test passed. */
                harness.testsPassed++;
                result = "Passed";
            }

            /* Print test and subtest results. */
            SYS_PRINT("Percent: %d (%d of %d subtests).\n", percentageSubtests, 
                      harness.testCurrent->passed, harness.testCurrent->count);

        }
        else
        {
            SYS_MESSAGE("Test did not complete.\n");
        }
    }
    else
    {
        SYS_MESSAGE("Test did not start.\n");
    }

    /* Print test result. */
    SYS_PRINT("Result:  %s!\n\n", result);

    /* Advance to the next test, if there are any more to run. */
    harness.testCurrent++;
    harness.testsCount++;
    if (harness.testsCount >= harness.numberOfTests)
    {
        /* Determine final results if all tests have completed.*/
        if (harness.subtestsCount == 0)
        {
            percentageSubtests = 0;
        }
        else
        {
            percentageSubtests = (harness.subtestsPassed * 100) / harness.subtestsCount;
        }
        percentageTests = (harness.testsPassed * 100) / harness.testsCount;

        result = "Failed";
        if (harness.testsPassed == harness.testsCount)
        {
            result = "Passed";
            harness.result = true;
        }

        /* Print final results. */
        SYS_MESSAGE("***************************************************************************\n");
        SYS_PRINT("Subtest Percent: %d (%d subtests of %d passed)\n", 
                  percentageSubtests, harness.subtestsPassed,  harness.subtestsCount);
        SYS_PRINT("Test Percent:    %d (%d tests of %d passed)\n", 
                  percentageTests, harness.testsPassed,  harness.testsCount);
        SYS_PRINT("Final Result:    %s!\n", result);
        SYS_MESSAGE("***************************************************************************\n");
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: Test Harness System Interface Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void TEST_Initialize ( const TEST_INIT_DATA *init )

  Remarks:
    See prototype in test_harness.h.
 */

void TEST_Initialize ( const TEST_INIT_DATA *init )
{
    /* Place the harness state machine in its initial state. */
    harness.state               = HARNESS_STATE_INIT;

    /* Capture test data. */
    if (NULL == init)
    {
        harness.numberOfTests   = 0;
        harness.testCurrent     = NULL;
        harness.testList        = NULL;
    }
    else
    {
        harness.numberOfTests   = init->numberOfTests;
        harness.testCurrent     = init->tests;
        harness.testList        = init->tests;
    }

    /* Initialize harness variables. */
    harness.subtestsPassed    = 0;
    harness.subtestsCount     = 0;
    harness.testsPassed       = 0;
    harness.testsCount        = 0;
    harness.testRunning       = false;
    harness.result            = false;
    harness.testTime          = 0;
    harness.libraryRunning    = false;
    harness.platformReady     = false;

    /* Initialize test semaphore (initially not free, first "pend" will block). */
    if (OSAL_SEM_Create(&harness.testSemaphore, OSAL_SEM_TYPE_COUNTING, 1, 0) != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_ERROR, "Failed to create test semaphore.\n");
        SYS_DEBUG_BreakPoint();
    }

    /* Initialize MUT tasks semaphore (initially not free, first "pend" will block). */
    if (OSAL_SEM_Create(&harness.librarySemaphore, OSAL_SEM_TYPE_COUNTING, 1, 0) != OSAL_RESULT_TRUE)
    {
        SYS_DEBUG_MESSAGE(SYS_ERROR_ERROR, "Failed to create MUT tasks semaphore.\n");
        SYS_DEBUG_BreakPoint();
    }

    /* Initialize timeout timer. */
    TimeoutTimerInitialize();
    TimeoutTimerCounterClear();
}


/******************************************************************************
  Function:
    void TEST_HarnessTasks ( void )

  Remarks:
    See prototype in test_harness.h.
 */

void TEST_HarnessTasks ( void )
{
    /* Check for timeout and fail test if it occurs. */
    if (harness.testRunning && harness.testTime >= TEST_TIMER_MS_TIMEOUT)
    {
        harness.testRunning             = false;
        harness.testCurrent->completed  = true;
        TEST_HasPassedSubtest(false);
        SYS_DEBUG_MESSAGE(SYS_ERROR_ERROR, "Test Timed out.\n");
        harness.state = HARNESS_STATE_TEST_COMPLETED;
    }

    /* Test harness state machine. */
    switch ( harness.state )
    {
        /* Prepare to run test sequences. */
        case HARNESS_STATE_INIT:
        {
            if (PrepairForTests())
            {
                harness.state = HARNESS_STATE_INIT_COMPLETE;
            }
            else
            {
                harness.state = HARNESS_STATE_IDLE;
            }
            break;
        }

        /* Wait for preparations to complete. */
        case HARNESS_STATE_INIT_COMPLETE:
        {
            if (harness.platformReady)
            {
                harness.state = HARNESS_STATE_TEST_INITIALIZE;
            }
            break;
        }

        /* Initialize current test. */
        case HARNESS_STATE_TEST_INITIALIZE:
        {
            if (InitializeTest())
            {
                harness.state       = HARNESS_STATE_RUN_TEST;
            }
            else
            {
                harness.state       = HARNESS_STATE_TEST_COMPLETED;
            }
            break;
        }

        /* Let the test run until it is ready to initialize the library under test. */
        case HARNESS_STATE_RUN_TEST:
        {
            if (harness.testReady)
            {
                harness.state = HARNESS_STATE_LIBRARY_INITIALIZE;
            }
            break;
        }

        /* Attempt to initialize the library under test. */
        case HARNESS_STATE_LIBRARY_INITIALIZE:
        {
            if (InitializeLibraryUnderTest())
            {
                harness.state = HARNESS_STATE_TEST_START;
            }
            else
            {
                harness.state = HARNESS_STATE_TEST_COMPLETED;
            }
            break;
        }

        /* Start the current test running. */
        case HARNESS_STATE_TEST_START:
        {
            harness.testCurrent->start(harness.testCurrent->testObj);
            harness.state = HARNESS_STATE_RUN_TEST_AND_LIBRARY;
            break;
        }

        /* Let the test and the library run until the test is completed. */
        case HARNESS_STATE_RUN_TEST_AND_LIBRARY:
        {
            if (RunUntilTestComplete())
            {
                harness.state = HARNESS_STATE_LIBRARY_DEINITIALIZE;
            }
            break;
        }

        /* Deinitialize the library under test. */
        case HARNESS_STATE_LIBRARY_DEINITIALIZE:
        {
            DenitializeLibraryUnderTest();
            harness.state = HARNESS_STATE_LIBRARY_DEINITIALIZE_WAIT;
            break;
        }

        /* Wait for the library to complete deinitialization. */
        case HARNESS_STATE_LIBRARY_DEINITIALIZE_WAIT:
        {
            if (VerifyLibraryDeinitialized())
            {
                harness.state = HARNESS_STATE_TEST_COMPLETED;
            }
            break;
        }

        /* Accumulate the results after the test reports it has completed. */
        case HARNESS_STATE_TEST_COMPLETED:
        {
            TimeoutTimerStop();
            ReportTestResults();
            if (harness.testsCount >= harness.numberOfTests)
            {
                harness.state = HARNESS_STATE_IDLE;
            }
            else
            {
                harness.state = HARNESS_STATE_TEST_INITIALIZE;
            }
            break;
        }

        /* Nothing more to do */
        case HARNESS_STATE_IDLE:
        default:
        {
            /* If harness.result is true, then all tests passed. */
            SYS_DEBUG_BreakPoint();
            break;
        }
    }
}


/*******************************************************************************
  Function:
    void TEST_Tasks( unsigned int number )

  Remarks:
    See prototype in test_harness.h.
*/

void TEST_Tasks( unsigned int number )
{
    if (number >= TEST_HARNESS_MAX_NUM_TASKS)
    {
        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "Invalid test Tasks number:  %d\n", number);
        return;
    }

    /* Ensure that there is currently a test running. */
    if (NULL != harness.testCurrent)
    {
        if (harness.testRunning)
        {
            if (NULL != harness.testCurrent->tasks[number])
            {
                /* Call the test's tasks function. */
                harness.testCurrent->tasks[number](harness.testCurrent->testObj, number);
            }
        }
    }

    return;
}


/******************************************************************************
  Function:
    void TEST_LibraryTasksPolled ( unsigned int             testIndex, 
                                   SYS_MODULE_INDEX         libraryIndex, 
                                   SYS_MODULE_TASKS_ROUTINE libraryTasks )

  Remarks:
    See prototype in test_harness.h.
*/

void TEST_LibraryTasksPolled ( unsigned int             testIndex, 
                               SYS_MODULE_INDEX         libraryIndex, 
                               SYS_MODULE_TASKS_ROUTINE libraryTasks )
{
    SYS_STATUS  status; // Library status

    /* Only poll library Tasks functions for libraries tested by the current 
    test and then only when the library has been initialized and is running. */
    if (testIndex == harness.testsCount && harness.libraryRunning)
    {
        /* Ensure that the test has has not timed out. */
        if (harness.testTime < TEST_TIMER_MS_TIMEOUT)
        {
            /* Ensure the library's object handle is valid. */
            if (SYS_MODULE_OBJ_INVALID != harness.libraryObj[libraryIndex])
            {
                // Call the Tasks function for the library under test.
                libraryTasks(harness.libraryObj[libraryIndex]);

                /* Check library status. */
                if (NULL != harness.testCurrent->library->status)
                {
                    status = harness.testCurrent->library->status(harness.libraryObj[libraryIndex]);
                    if (status <= SYS_STATUS_UNINITIALIZED)
                    {
                        /* Library under test error */
                        harness.libraryRunning = false;
                        SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "Library under test error, status=%d.\n", status);
                    }
                }
            }
            else
            {
                /* Test configuration error. */
                harness.libraryRunning = false;
                SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "Invalid object handle for library instance %d.\n", libraryIndex);
            }
        }
        else
        {
            harness.libraryRunning = false;
        }
    }

    return;
}


/******************************************************************************
  Function:
    void TEST_LibraryTasksISR ( unsigned int             testIndex, 
                                SYS_MODULE_INDEX         moduleIndex, 
                                SYS_MODULE_TASKS_ROUTINE libraryTasks, 
                                INT_SOURCE               source )

  Remarks:
    See prototype in test_harness.h.
*/

void TEST_LibraryTasksISR ( unsigned int             testIndex, 
                            SYS_MODULE_INDEX         moduleIndex, 
                            SYS_MODULE_TASKS_ROUTINE libraryTasks, 
                            INT_SOURCE               source )
{
    // Validate the interrupt
    if (testIndex != harness.testsCount || SYS_MODULE_OBJ_INVALID == harness.libraryObj[moduleIndex])
    {
        // Spurious Interrupt!
        SYS_INT_SourceDisable(source);

        // Cause a test failure
        harness.testList[testIndex].count++;
        SYS_DEBUG_BreakPoint();
    }
    else
    {
        // Call the Tasks function for the library under test.
        libraryTasks(harness.libraryObj[moduleIndex]);
    }

    return;
}


/*******************************************************************************
 End of File
*/