/*******************************************************************************
  MPLAB Harmony Sample Library Functional Test Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    test_sample_functional.c

  Summary:
    This file contains the source code for the MPLAB Harmony Sample library's
    functional test.

  Description:
    This file contains the source code for the MPLAB Harmony sample library's 
    functional test.  It implements the logic of the test's state machine and 
    it calls API routines of the sample library and other MPLAB Harmony libraries 
    such as system services and middleware.  
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
#include "test/test_sample_functional.h"


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
    This enumeration defines the valid sample library functional test states.  
    These states determine the behavior of the test at various times.
*/

typedef enum
{
    /* Initialize the library under test, but do not start testing. */
	TEST_SAMPLE_FUNC_STATE_INIT=0,

    /* Wait for start signal from the test harness. */
    TEST_SAMPLE_FUNC_STATE_INIT_WAIT,

	/* Check sample library instances for initial data. */
    TEST_SAMPLE_FUNC_STATE_INITIAL_DATA,

	/* Give new data to the sample library. */
    TEST_SAMPLE_FUNC_STATE_GIVE_DATA,

    /* Get back data previously sent to the library. */
    TEST_SAMPLE_FUNC_STATE_GET_DATA,

    /* Test sequence Completed. */
    TEST_SAMPLE_FUNC_STATE_COMPLETED,

    /* App Does Nothing */
    TEST_SAMPLE_FUNC_STATE_IDLE

} TEST_SAMPLE_FUNC_STATES;


// *****************************************************************************
/* Sample Functional Test's Library-Specific Data

  Summary:
    Holds the test's data related to an individual sample library instance.

  Description:
    This structure holds the test's data related to an individual sample 
    library instance.
 */

typedef struct _test_sample_func_lib_data
{
    /* The test's current state */
    TEST_SAMPLE_FUNC_STATES         state;

    /* Pointer to Sample Library's Init Data Structure. */
    const SAMPLE_MODULE_INIT_DATA  *initData;
    
    /* Test data counter. */
    int                             counter;

    /* Previously sent data. */
    int                             dataSent;

} TEST_SAMPLE_FUNC_LIB_DATA;


// *****************************************************************************
/* Sample Functional Test Data

  Summary:
    Holds sample functional test's internal data.

  Description:
    This structure holds the sample functional test's internal data.

 */

typedef struct
{
    /* The test's current status. */
    SYS_STATUS                  status;

    /* Flag indicating that the test is running. */
    bool                        running;

    /* Sample library instance-specific test data. */
    TEST_SAMPLE_FUNC_LIB_DATA   library[TEST_SAMPLE_FUNC_LIB_INSTANCES_NUMBER];

} TEST_SAMPLE_FUNC_DATA;


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
    This structure must be initialized by the APP_InitializeTestSample function.
*/

static TEST_SAMPLE_FUNC_DATA testSampleFunc;


// *****************************************************************************
// *****************************************************************************
// Section:  Local Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    TEST_SAMPLE_FUNC_STATES VerifyInitialData ( TEST_SAMPLE_FUNC_DATA *pObj,
                                                unsigned int index )

  Summary:
    Reads initial data from an instance of the sample library and checks to see 
    if matches the data given in the init struct.

  Description:
    This function reads initial data from an instance of the sample library and
    checks to see if it matches the data given in the init struct.

  Precondition:
    The system and test harness must be initialized and the test harness must 
    have called the TEST_SampleFunctionalInitialize to initialize the test's 
    state machine.  The test harness and this test module must be "running".

  Parameters:
    pObj    - Pointer to the sample test data object.
    
    index   - Index identifying tasks context and library index.
  
  Returns:
    TEST_SAMPLE_FUNC_STATE_INITIAL_DATA - If the library is busy.
    
    TEST_SAMPLE_FUNC_STATE_GIVE_DATA    - If the initial data was verified 
                                          to be correct.
    
    TEST_SAMPLE_FUNC_STATE_COMPLETED    - If the subtest fails.

  Remarks:
    Reports results via SYS_DEBUG service.
        - SYS_ERROR_DEBUG messages report test progress information.
        - SYS_ERROR_ERROR messages indicate a subtest test failure.
        
    Reports subtest results to test harness by calling TEST_HasPassedSubtest.
 */

static TEST_SAMPLE_FUNC_STATES VerifyInitialData ( TEST_SAMPLE_FUNC_DATA *pObj, unsigned int index )
{
    SAMPLE_MODULE_DATA_STATUS   status;
    TEST_SAMPLE_FUNC_STATES     nextState;
    int                         dataRead;
    int                         dataFromInit = pObj->library[index].initData->dataSome;

    /* Assume failure and reassign, below if passed or delayed. */
    nextState   = TEST_SAMPLE_FUNC_STATE_COMPLETED;

    /* Validate index */
    if (index >= TEST_SAMPLE_FUNC_LIB_INSTANCES_NUMBER)
    {
        /* Failure, test ends. */
        TEST_HasPassedSubtest(false);
        SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "SAMPLE%d has no initial data\n", index);
    }
    else
    {
        /* Check to see if the library has initial data. */
        status = SAMPLE_DataStatus(index);
        if (SAMPLE_MODULE_DATA_NONE == status)
        {
            /* Failure, test ends. */
            TEST_HasPassedSubtest(false);
            SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "SAMPLE%d has no initial data\n", index);
        }
        else if (SAMPLE_MODULE_DATA_BUSY == status)
        {
            /* Data pending, stay in this state, we need to try again later. */
            pObj->status    = SYS_STATUS_READY;
            nextState       = TEST_SAMPLE_FUNC_STATE_INITIAL_DATA;
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "SAMPLE%d busy.\n", index);
        }
        else
        {
            /* Check the library's initial data. */
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "SAMPLE%d has initial data.\n", index);

            if(SAMPLE_DataGet(index, &dataRead))
            {
                if (dataRead == dataFromInit)
                {
                    /* Success */
                    TEST_HasPassedSubtest(true);
                    nextState = TEST_SAMPLE_FUNC_STATE_GIVE_DATA;
                    SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "SAMPLE%d initial data = %d matches data read = %d\n", 
                                    index, dataFromInit, dataRead);
                }
                else
                {
                    /* Failure, test ends */
                    TEST_HasPassedSubtest(false);
                    SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "SAMPLE%d initial data = %d does not match data read = %d\n", 
                                    index, dataFromInit, dataRead);
                }
            }
            else
            {
                /* Failure, test ends.  Should not be possible to get here. */
                TEST_HasPassedSubtest(false);
                SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "SAMPLE%d has no initial data\n", index);
            }
        }
    }

    return nextState;
}


// *****************************************************************************
/* Function:
    TEST_SAMPLE_FUNC_STATES AbleToSendNewData ( TEST_SAMPLE_FUNC_DATA *pObj, unsigned index )

  Summary:
    Attempts to send new data to the sample library instance.

  Description:
    This function attempts to send new data to the sample library instance.

  Precondition:
    The system and test harness must be initialized and the test harness must 
    have called the TEST_SampleFunctionalInitialize to initialize the test's 
    state machine.  The test harness and this test module must be "running".

  Parameters:
    pObj    - Pointer to the sample test data object.
    
    index   - Index identifying tasks context and library index.
  
  Returns:
    TEST_SAMPLE_FUNC_STATE_GET_DATA     - If successfully able to send data.
                                          
    TEST_SAMPLE_FUNC_STATE_COMPLETED    - If unsuccessful.

  Remarks:
    At this test state, the library must be able to receive data or the test
    fails.
    
    Utilizes global test structure.
    
    Reports results via SYS_DEBUG service.
        - SYS_ERROR_DEBUG messages report test progress information.
        - SYS_ERROR_ERROR messages indicate a test failure.

    Reports subtest results to test harness by calling TEST_HasPassedSubtest.
    
    If this subtest fails, it ends the test by calling TEST_HasCompleted.
*/

static TEST_SAMPLE_FUNC_STATES AbleToSendNewData ( TEST_SAMPLE_FUNC_DATA *pObj, unsigned int index )
{
    int                     data;
    TEST_SAMPLE_FUNC_STATES nextState;

    /* Send new data to the library instance. */
    data = pObj->library[index].counter * 10 + index;
    if(SAMPLE_DataGive(index, data))
    {
        /* Success. */
        pObj->library[index].dataSent = data;
        pObj->library[index].counter++;
        nextState = TEST_SAMPLE_FUNC_STATE_GET_DATA;
        TEST_HasPassedSubtest(true);
        SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "SAMPLE%d sent data = %d\n", index, data);
    }
    else
    {
        /* Failure, test ends. */
        nextState = TEST_SAMPLE_FUNC_STATE_COMPLETED;
        TEST_HasPassedSubtest(false);
        SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "SAMPLE%d library busy, cannot give data.\n", index);
        TEST_HasCompleted();
    }

    return nextState;
}


// *****************************************************************************
/* Function:
   TEST_SAMPLE_FUNC_STATES AbleToReadSentData ( TEST_SAMPLE_FUNC_DATA *pObj, 
                                                unsigned int index )

  Summary:
    Gets data previously sent  to the sample library instance.

  Description:
    This function gets data previously sent to the sample library instance.

  Precondition:
    The system and test harness must be initialized and the test harness must 
    have called the TEST_SampleFunctionalInitialize to initialize the test's 
    state machine.  The test harness and this test module must be "running".

  Parameters:
    pObj    - Pointer to the sample test data object.
    
    index   - Index identifying tasks context and library index.
  
  Returns:
    TEST_SAMPLE_FUNC_STATE_COMPLETED    - If the test has completed (either
                                          by succeeding or failing).

    TEST_SAMPLE_FUNC_STATE_GIVE_DATA    - If the test succeeded, but there are
                                          more iterations remaining.
                                          
    TEST_SAMPLE_FUNC_STATE_GET_DATA     - If the library is not yet ready and 
                                          this test state has to be retried.

  Remarks:
    Utilizes global test structure.
    
    Reports results via SYS_DEBUG service.
        - SYS_ERROR_DEBUG messages report test progress information.
        - SYS_ERROR_ERROR messages indicate a test failure.
*/

static TEST_SAMPLE_FUNC_STATES AbleToReadSentData ( TEST_SAMPLE_FUNC_DATA *pObj, unsigned int index )
{
    TEST_SAMPLE_FUNC_STATES nextState;
    int                     dataReceived;
    int                     dataSent = pObj->library[index].dataSent;

    /* Assume the test is done and reassign, below if passed or delayed. */
    nextState   = TEST_SAMPLE_FUNC_STATE_COMPLETED;

    /* Get data from library. */
    if(SAMPLE_DataGet(index, &dataReceived))
    {
        /* Check to see if it matches what we sent */  
        if (dataReceived == dataSent)
        {
            TEST_HasPassedSubtest(true);
            SYS_DEBUG_PRINT(SYS_ERROR_DEBUG, "SAMPLE%d returned same data sent = %d, data received = %d\n", 
                            index, dataSent, dataReceived);

            /* There are more test iterations... */
            if (pObj->library[index].counter < TEST_SAMPLE_MAX_ITERATIONS)
            {
                /* Loop back and test again. */
                nextState = TEST_SAMPLE_FUNC_STATE_GIVE_DATA;
            }
        }
        else
        {
            TEST_HasPassedSubtest(false);
            SYS_DEBUG_PRINT(SYS_ERROR_ERROR, "SAMPLE%d returned different data sent = %d, data received = %d\n", 
                            index, dataSent, dataReceived);
        }
    }
    else
    {
        /* Library not ready, try again. */
        nextState   = TEST_SAMPLE_FUNC_STATE_GET_DATA;
    }

    return nextState;
}


// *****************************************************************************
// *****************************************************************************
// Section: Test Interface Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    SYS_MODULE_OBJ TEST_SampleFunctionalInitialize ( 
                            const SYS_MODULE_INDEX index,
                            const SYS_MODULE_INIT * const init )

  Remarks:
    See prototype in test_sample_functional.h.
 */

SYS_MODULE_OBJ TEST_SampleFunctionalInitialize ( const SYS_MODULE_INDEX index,
                                                 const SYS_MODULE_INIT * const init )
{
    int i;
    TEST_SAMPLE_FUNC_DATA              *pObj        = &testSampleFunc;  
    SYS_MODULE_OBJ                      retvalue    = (SYS_MODULE_OBJ)&testSampleFunc;
    TEST_SAMPLE_FUNCTIONAL_INIT_DATA   *initTest    = (TEST_SAMPLE_FUNCTIONAL_INIT_DATA *)init;

    SYS_ASSERT(index == 0, "Invalid sample test index");

    /* Initialize common data. */
    pObj->running   = false;

    /* Initialize test/library specific data. */
    for (i=0; i < TEST_SAMPLE_FUNC_LIB_INSTANCES_NUMBER; i++)
    {
        pObj->library[i].state                  = TEST_SAMPLE_FUNC_STATE_INIT;
        pObj->library[i].counter                = 0;

        if (NULL != initTest)
        {
            pObj->library[i].initData               = initTest->modulesInitData[i];
        }
        else
        {
            SYS_DEBUG_MESSAGE(SYS_ERROR_ERROR, "No initial test data provided.\n");
            retvalue = SYS_MODULE_OBJ_INVALID;
        }
    }

    /* Set test status and tell harness to initialize the library. */
    if (SYS_MODULE_OBJ_INVALID == retvalue)
    {
        pObj->status = SYS_STATUS_ERROR;
    }
    else
    {
        pObj->status = SYS_STATUS_READY;
        TEST_LibraryInitialize();
    }

    return (SYS_MODULE_OBJ)retvalue;
}


/*******************************************************************************
  Function:
    bool TEST_SampleFunctionalStart ( SYS_MODULE_OBJ object )
    
  Remarks:
    See prototype in test_sample_functional.h.
*/

bool TEST_SampleFunctionalStart ( SYS_MODULE_OBJ object )
{
    TEST_SAMPLE_FUNC_DATA   *pObj = (TEST_SAMPLE_FUNC_DATA *)object;

    SYS_ASSERT(pObj == &testSampleFunc, "Invalid sample test object");

    pObj->running = true;
    return true;
}


/******************************************************************************
  Function:
    void TEST_SampleFunctionalTasks ( SYS_MODULE_OBJ object, unsigned int index )

  Remarks:
    See prototype in test_sample_functional.h.
 */

void TEST_SampleFunctionalTasks ( SYS_MODULE_OBJ object, unsigned int index )
{
    TEST_SAMPLE_FUNC_DATA   *pObj = (TEST_SAMPLE_FUNC_DATA *)object;

    SYS_ASSERT(pObj == &testSampleFunc, "Invalid sample test object");

    /* Take state-based actions & transition to other states as needed. */
    pObj->status = SYS_STATUS_BUSY;
    switch ( pObj->library[index].state )
    {
        /* Perform any one-time test initialization tasks, 
        but do not start testing. */
        case TEST_SAMPLE_FUNC_STATE_INIT:
        {
            pObj->library[index].state = TEST_SAMPLE_FUNC_STATE_INIT_WAIT;
            break;
        }
    
        /* Wait for start signal from the test harness. */
        case TEST_SAMPLE_FUNC_STATE_INIT_WAIT:
        {
            if (pObj->running)
            {
                pObj->library[index].state = TEST_SAMPLE_FUNC_STATE_INITIAL_DATA;
            }
            break;
        }

        /* Check sample modules for initial data. */
        case TEST_SAMPLE_FUNC_STATE_INITIAL_DATA:
        {
            pObj->library[index].state = VerifyInitialData(pObj, index);
            break;
        }

        /* Give new data to the sample modules. */
        case TEST_SAMPLE_FUNC_STATE_GIVE_DATA:
        {
            pObj->library[index].state = AbleToSendNewData(pObj, index);
            break;
        }

        /* Get back data previously sent to the modules. */
        case TEST_SAMPLE_FUNC_STATE_GET_DATA:
        {
            pObj->library[index].state = AbleToReadSentData(pObj, index);
            break;
        }

        /* Test sequence Completed. */
        case TEST_SAMPLE_FUNC_STATE_COMPLETED:
        {
            int     i;
            bool    done = true;

            /* Ensure that all state machines are completed. */
            for (i=0; i < TEST_SAMPLE_FUNC_LIB_INSTANCES_NUMBER; i++)
            {
                if (pObj->library[index].state < TEST_SAMPLE_FUNC_STATE_COMPLETED)
                {
                    done = false;
                }
            }
            if (done)
            {
                pObj->status = SYS_STATUS_TEST_COMPLETED;
            }
            /* Go to idle state next. */
            pObj->library[index].state  = TEST_SAMPLE_FUNC_STATE_IDLE;
            break;
        }

        /* Nothing more to do */
        case TEST_SAMPLE_FUNC_STATE_IDLE:
        default:
        {
            break;
        }
    }
}
 

/*******************************************************************************
  Function:
    SYS_STATUS TEST_SampleFunctionalStatus (  SYS_MODULE_OBJ object )

  Remarks:
    See prototype in test_sample_functional.h.
*/

SYS_STATUS TEST_SampleFunctionalStatus ( SYS_MODULE_OBJ object )
{
    TEST_SAMPLE_FUNC_DATA  *pObj = (TEST_SAMPLE_FUNC_DATA *)object;

    SYS_ASSERT(pObj == &testSampleFunc, "Invalid sample functional test object");

    return pObj->status;
}


/*******************************************************************************
 End of File
 */