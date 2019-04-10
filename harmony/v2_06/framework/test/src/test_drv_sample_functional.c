/*******************************************************************************
  MPLAB Harmony Sample Driver Functional Test Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    test_drv_sample_functional.c

  Summary:
    This file contains the source code for the MPLAB Harmony Sample driver's
    functional test.

  Description:
    This file contains the source code for the MPLAB Harmony sample driver's 
    functional test.  It implements the logic of the test's state machine and 
    it calls API routines of the sample driver and other MPLAB Harmony 
    libraries such as system services and middleware.  
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

#include "test/test_harness.h"
#include "test/test_drv_sample_functional.h"


// *****************************************************************************
// *****************************************************************************
// Section: Configuration Options
// *****************************************************************************
// *****************************************************************************
/* This section checks settings for sample driver functional test configuration 
   options.
*/

#ifndef TEST_DRV_SAMPLE_FUNC_LIB_INSTANCES_NUMBER
    #error "Number of sample driver instances to test must be defined."
#endif


// *****************************************************************************
// *****************************************************************************
// Section: Local Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Test States

  Summary:
    Test states enumeration.

  Description:
    This enumeration defines the valid sample driver functional test states.  
    These states determine the behavior of the test at various times.
*/

typedef enum
{
    /* Initialize the driver under test, but do not start testing. */
	TEST_DRV_SAMPLE_FUNC_STATE_INIT=0,

    /* Wait for start signal from the test harness. */
    TEST_DRV_SAMPLE_FUNC_STATE_INIT_WAIT,

	/* Check sample driver instances for initial data. */
    TEST_DRV_SAMPLE_FUNC_STATE_INITIAL_DATA,

    /* Write data to the driver. */
    TEST_DRV_SAMPLE_FUNC_STATE_WRITE_DATA,

    /* Read data from the driver. */
    TEST_DRV_SAMPLE_FUNC_STATE_READ_DATA,

    /* TODO:  Add more test states. */

    /* Test sequence Completed. */
    TEST_DRV_SAMPLE_FUNC_STATE_COMPLETED,

    /* App Does Nothing */
    TEST_DRV_SAMPLE_FUNC_STATE_IDLE

} TEST_DRV_SAMPLE_FUNC_STATES;


// *****************************************************************************
/* Sample Driver Functional Test's Driver-Specific Data

  Summary:
    Holds the test's data related to an individual sample driver instance.

  Description:
    This structure holds the test's data related to an individual sample 
    driver instance.
 */

typedef struct _test_drv_sample_func_lib_data
{
    /* The test's current state */
    TEST_DRV_SAMPLE_FUNC_STATES state;

    /* Pointer to Sample Driver's Init Data Structure. */
    const DRV_SAMPLE_INIT  *    initData;
    
    /* TODO:  Add data as necessary. */

} TEST_DRV_SAMPLE_FUNC_LIB_DATA;


// *****************************************************************************
/* Sample Driver Functional Test Data

  Summary:
    Holds sample functional test's internal data.

  Description:
    This structure holds the sample functional test's internal data.

 */

typedef struct
{
    /* The test's current status. */
    SYS_STATUS                      status;

    /* Flag indicating that the test is running. */
    bool                            running;

    /* Sample driver instance-specific test data. */
    TEST_DRV_SAMPLE_FUNC_LIB_DATA   driver[TEST_DRV_SAMPLE_FUNC_LIB_INSTANCES_NUMBER];

} TEST_DRV_SAMPLE_FUNC_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Sample Functional Test Data

  Summary:
    Holds the test's data.

  Description:
    This structure holds the test's data.

  Remarks:
    This structure must be initialized by the TEST_DrvSampleFunctionalInitialize
    function.
*/

static TEST_DRV_SAMPLE_FUNC_DATA testDrvSampleFunc;


// *****************************************************************************
// *****************************************************************************
// Section:  Local Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    TEST_DRV_SAMPLE_FUNC_STATES VerifyInitialData ( 
                                    TEST_DRV_SAMPLE_FUNC_DATA * pObj,
                                    unsigned int                index )

  Summary:
    Reads initial data from an instance of the sample driver and checks to see 
    if matches the data given in the init struct.

  Description:
    This function reads initial data from an instance of the sample driver and
    checks to see if it matches the data given in the init struct.

  Precondition:
    The system and test harness must be initialized and the test harness must 
    have called the TEST_DrvSampleFunctionalInitialize to initialize the test's 
    state machine.  The test harness and this test module must be "running".

  Parameters:
    pObj    - Pointer to the sample driver test data object.
    
    index   - Index identifying tasks context and driver index.
  
  Returns:
    TEST_DRV_SAMPLE_FUNC_STATE_INITIAL_DATA
     - If the driver is busy.
    
    TODO:
     - If the initial data was verified to be correct.
    
    TEST_DRV_SAMPLE_FUNC_STATE_COMPLETED
     - If the subtest fails.

  Remarks:
    Reports results via SYS_DEBUG service.
     - SYS_ERROR_DEBUG messages report test progress information.
     - SYS_ERROR_ERROR messages indicate a subtest test failure.
        
    Reports subtest results to test harness by calling TEST_HasPassedSubtest.
 */

static TEST_DRV_SAMPLE_FUNC_STATES VerifyInitialData ( TEST_DRV_SAMPLE_FUNC_DATA *  pObj, 
                                                       unsigned int                 index )
{
    TEST_DRV_SAMPLE_FUNC_STATES nextState;

    /* Assume failure and reassign, below if passed or delayed. */
    nextState   = TEST_DRV_SAMPLE_FUNC_STATE_COMPLETED;

    /* Validate index */
    if (index >= TEST_DRV_SAMPLE_FUNC_LIB_INSTANCES_NUMBER)
    {
        /* Failure: Invalid index, test ends. */
        TEST_HasPassedSubtest(false);
        SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "DRV_SAMPLE%d Invalid driver index\n", index);
    }
    else
    {
        /* Failure: Invalid init data, test ends. */
        TEST_HasPassedSubtest(false);
        SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "DRV_SAMPLE%d has no initial data\n", index);
    }

    return nextState;
}


// *****************************************************************************
/* Function:
    TEST_DRV_SAMPLE_FUNC_STATES AbleToWriteData ( TEST_DRV_SAMPLE_FUNC_DATA *pObj, unsigned index )

  Summary:
    Attempts to write data to the sample driver instance.

  Description:
    This function attempts to write data to the sample driver instance.

  Precondition:
    The system and test harness must be initialized and the test harness must 
    have called the TEST_DrvSampleFunctionalInitialize to initialize the test's 
    state machine.  The test harness and this test module must be "running".

  Parameters:
    pObj    - Pointer to the sample driver test data object.
    
    index   - Index identifying tasks context and driver index.
  
  Returns:
    TEST_DRV_SAMPLE_FUNC_STATE_WRITE_DATA
     - If successfully able to send data.
                                          
    TEST_DRV_SAMPLE_FUNC_STATE_COMPLETED
     - If unsuccessful.

  Remarks:
    At this test state, the driver must be able to accept data written to it or
    the test fails.
    
    Utilizes global test structure.
    
    Reports results via SYS_DEBUG service.
        - SYS_ERROR_DEBUG messages report test progress information.
        - SYS_ERROR_ERROR messages indicate a test failure.

    Reports subtest results to test harness by calling TEST_HasPassedSubtest.
    
    If this subtest fails, it ends the test by calling TEST_HasCompleted.
*/

static TEST_DRV_SAMPLE_FUNC_STATES AbleToWriteData ( TEST_DRV_SAMPLE_FUNC_DATA *pObj, 
                                                     unsigned int               index )
{
    TEST_DRV_SAMPLE_FUNC_STATES nextState;

    /* TODO:  Write data to the driver instance. */

    /* Failure, test ends. */
    nextState = TEST_DRV_SAMPLE_FUNC_STATE_COMPLETED;
    TEST_HasPassedSubtest(false);
    SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "DRV_SAMPLE%d Unable to write data to driver\n", index);
    TEST_HasCompleted();

    return nextState;
}


// *****************************************************************************
/* Function:
   TEST_DRV_SAMPLE_FUNC_STATES AbleToReadDataWritten ( 
                                    TEST_DRV_SAMPLE_FUNC_DATA * pObj, 
                                    unsigned int                index )

  Summary:
    Attempts to read data previously written to the sample driver instance.

  Description:
    This function attempts to read data previously written to the sample driver 
    instance.

  Precondition:
    The system and test harness must be initialized and the test harness must 
    have called the TEST_DrvSampleFunctionalInitialize to initialize the test's 
    state machine.  The test harness and this test module must be "running".

  Parameters:
    pObj    - Pointer to the sample driver test data object.
    
    index   - Index identifying tasks context and driver index.
  
  Returns:
    - TEST_DRV_SAMPLE_FUNC_STATE_COMPLETED  - If the test has completed (either by succeeding or failing).
    - TEST_DRV_SAMPLE_FUNC_STATE_WRITE_DATA - If the test succeeded, but there are more iterations remaining.
    - TEST_DRV_SAMPLE_FUNC_STATE_READ_DATA  - If the driver is not yet ready and this test state has to be retried.

  Remarks:
    Utilizes global test structure.
    
    Reports results via SYS_DEBUG service.
        - SYS_ERROR_DEBUG messages report test progress information.
        - SYS_ERROR_ERROR messages indicate a test failure.
*/

static TEST_DRV_SAMPLE_FUNC_STATES AbleToReadDataWritten ( TEST_DRV_SAMPLE_FUNC_DATA *pObj, unsigned int index )
{
    TEST_DRV_SAMPLE_FUNC_STATES nextState;

    /* Assume the test is done and reassign nextState below if passed or delayed. */
    nextState = TEST_DRV_SAMPLE_FUNC_STATE_COMPLETED;

    /* TODO:  Read data from driver. */

    TEST_HasPassedSubtest(false);
    SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "DRV_SAMPLE%d Unable to read data from driver\n", index);

    return nextState;
}


// *****************************************************************************
// *****************************************************************************
// Section: Test Interface Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    SYS_MODULE_OBJ TEST_DrvSampleFunctionalInitialize ( 
                            const SYS_MODULE_INDEX index,
                            const SYS_MODULE_INIT * const init )

  Remarks:
    See prototype in test_drv_sample_functional.h.
 */

SYS_MODULE_OBJ TEST_DrvSampleFunctionalInitialize ( 
                    const SYS_MODULE_INDEX index,
                    const SYS_MODULE_INIT * const init )
{
    int i;
    TEST_DRV_SAMPLE_FUNC_DATA              *pObj        = &testDrvSampleFunc;  
    SYS_MODULE_OBJ                          retvalue    = (SYS_MODULE_OBJ)&testDrvSampleFunc;
    //TEST_DRV_SAMPLE_FUNCTIONAL_INIT_DATA   *initTest    = (TEST_DRV_SAMPLE_FUNCTIONAL_INIT_DATA *)init;

    SYS_ASSERT(index == 0, "Invalid sample driver test index");

    /* Initialize test data. */
    pObj->running   = false;

    /* Initialize driver-instance-specific data. */
    for (i=0; i < TEST_DRV_SAMPLE_FUNC_LIB_INSTANCES_NUMBER; i++)
    {
        pObj->driver[i].state = TEST_DRV_SAMPLE_FUNC_STATE_INIT;

        /* TODO:  Remove if not needed. 
        if (NULL != initTest)
        {
            pObj->driver[i].initData = initTest->drvInitData[i];
        }
        else
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_ERROR, "No initial test data provided.\n");
            pObj->status = SYS_STATUS_ERROR;
            retvalue = SYS_MODULE_OBJ_INVALID;
            break;
        }
        */
    }

    /* Set test status and tell harness to initialize the driver. */
    if (SYS_MODULE_OBJ_INVALID != retvalue)
    {
        pObj->status = SYS_STATUS_READY;
        TEST_LibraryInitialize();
    }

    return retvalue;
}


/*******************************************************************************
  Function:
    bool TEST_DrvSampleFunctionalStart ( SYS_MODULE_OBJ object )
    
  Remarks:
    See prototype in test_drv_sample_functional.h.
*/

bool TEST_DrvSampleFunctionalStart ( SYS_MODULE_OBJ object )
{
    TEST_DRV_SAMPLE_FUNC_DATA   *pObj = (TEST_DRV_SAMPLE_FUNC_DATA *)object;

    SYS_ASSERT(pObj == &testDrvSampleFunc, "Invalid sample driver test object");

    pObj->running = true;
    return true;
}


/******************************************************************************
  Function:
    void TEST_DrvSampleFunctionalTasks ( SYS_MODULE_OBJ object, unsigned int index )

  Remarks:
    See prototype in test_drv_sample_functional.h.
 */

void TEST_DrvSampleFunctionalTasks ( SYS_MODULE_OBJ object, unsigned int index )
{
    TEST_DRV_SAMPLE_FUNC_DATA   *pObj = (TEST_DRV_SAMPLE_FUNC_DATA *)object;

    SYS_ASSERT(pObj == &testDrvSampleFunc, "Invalid sample driver test object");

    /* Take state-based actions & transition to other states as needed. */
    pObj->status = SYS_STATUS_BUSY;
    switch ( pObj->driver[index].state )
    {
        /* Finish test initialization tasks, but do not start testing. */
        case TEST_DRV_SAMPLE_FUNC_STATE_INIT:
        {
            pObj->driver[index].state = TEST_DRV_SAMPLE_FUNC_STATE_INIT_WAIT;
            break;
        }
    
        /* Wait for start signal from the test harness. */
        case TEST_DRV_SAMPLE_FUNC_STATE_INIT_WAIT:
        {
            if (pObj->running)
            {
                pObj->driver[index].state = TEST_DRV_SAMPLE_FUNC_STATE_INITIAL_DATA;
            }
            break;
        }

        /* Check sample driver initial data. */
        case TEST_DRV_SAMPLE_FUNC_STATE_INITIAL_DATA:
        {
            pObj->driver[index].state = VerifyInitialData(pObj, index);
            break;
        }

        /* Attempt to write data to the sample driver. */
        case TEST_DRV_SAMPLE_FUNC_STATE_WRITE_DATA:
        {
            pObj->driver[index].state = AbleToWriteData(pObj, index);
            break;
        }

        /* Attempt to read data previously written from the sample driver. */
        case TEST_DRV_SAMPLE_FUNC_STATE_READ_DATA:
        {
            pObj->driver[index].state = AbleToReadDataWritten(pObj, index);
            break;
        }

        /* Test sequence Completed. */
        case TEST_DRV_SAMPLE_FUNC_STATE_COMPLETED:
        {
            int     i;
            bool    done = true;

            /* Ensure that all test instances have completed. */
            for (i=0; i < TEST_DRV_SAMPLE_FUNC_LIB_INSTANCES_NUMBER; i++)
            {
                if (pObj->driver[index].state < TEST_DRV_SAMPLE_FUNC_STATE_COMPLETED)
                {
                    done = false;
                }
            }

            /* If all instances are done, set status to "completed" and go to the idle state. */
            if (done)
            {
                pObj->status = SYS_STATUS_TEST_COMPLETED;
                pObj->driver[index].state  = TEST_DRV_SAMPLE_FUNC_STATE_IDLE;
            }
            break;
        }

        /* Nothing more to do. */
        case TEST_DRV_SAMPLE_FUNC_STATE_IDLE:
        default:
        {
            break;
        }
    }
}
 

/*******************************************************************************
  Function:
    SYS_STATUS TEST_DrvSampleFunctionalStatus (  SYS_MODULE_OBJ object )

  Remarks:
    See prototype in test_drv_sample_functional.h.
*/

SYS_STATUS TEST_DrvSampleFunctionalStatus ( SYS_MODULE_OBJ object )
{
    TEST_DRV_SAMPLE_FUNC_DATA  *pObj = (TEST_DRV_SAMPLE_FUNC_DATA *)object;

    SYS_ASSERT(pObj == &testDrvSampleFunc, "Invalid sample driver functional test object");

    return pObj->status;
}


/*******************************************************************************
 End of File
*/