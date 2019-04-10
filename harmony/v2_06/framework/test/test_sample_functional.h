/*******************************************************************************
  MPLAB Harmony Sample Module Test Header File

  Company:
    Microchip Technology Inc.

  File Name:
    test_sample_functional.h

  Summary:
    This header file provides prototypes and definitions for the test module.

  Description:
    This header file provides function prototypes and data type definitions for
    the test module.
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

#ifndef _TEST_SAMPLE_FUNCTIONAL_H
#define _TEST_SAMPLE_FUNCTIONAL_H


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
//#include "system_definitions.h"
#include "system/debug/sys_debug.h"
#include "sample/sample_module.h"


// *****************************************************************************
// *****************************************************************************
// Section: Configuration Options
// *****************************************************************************
// *****************************************************************************
/* This section checks settings for sample module functional test configuration 
   options.
*/

#ifndef TEST_SAMPLE_FUNC_LIB_INSTANCES_NUMBER
    #error "Number of sample module instances to test must be defined."
#endif


// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Sample Library Functional Test Initialization Data

  Summary:
    Holds the sample library functional test's initialization data.

  Description:
    This structure holds the sample library functional test's initialization 
    data.
 */

typedef struct _test_sample_functional_init_data
{
    /* Module init structure */
    SYS_MODULE_INIT                 sysModule;
    
    /* Array of Pointers to Sample Module Init Data Structures. */
    const SAMPLE_MODULE_INIT_DATA  *modulesInitData[TEST_SAMPLE_FUNC_LIB_INSTANCES_NUMBER];

    /* Add additional test init data if needed. */

} TEST_SAMPLE_FUNCTIONAL_INIT_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Test Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    SYS_MODULE_OBJ TEST_SampleFunctionalInitialize ( 
                            const SYS_MODULE_INDEX index,
                            const SYS_MODULE_INIT * const init )

  Summary:
     MPLAB Harmony Sample module test initialization routine.

  Description:
    This function initializes the Harmony Sample library functional test.  It 
    places the test state machine in its initial state and prepares it to run 
    so that its Tasks function can be called.

  Precondition:
    The system and test harness must be initialized, but the library under test
    should not.  If should be initialized under control of the test harness.

  Parameters:
    index           - index indicating module instance.
    
    init            - pointer to module's init data structure

  Returns:
    SYS_MODULE_OBJ  - Handle to test instance object.

  Example:
    None.

  Remarks:
    This test is normally only executed under the control of the MPLAB Harmony 
    test harness.
*/

SYS_MODULE_OBJ TEST_SampleFunctionalInitialize ( 
                        const SYS_MODULE_INDEX index,
                        const SYS_MODULE_INIT * const init );


/*************************************************************************
  Function:
    bool TEST_SampleFunctionalStart ( SYS_MODULE_OBJ object )
    
  Summary:
    Starts the MPLAB Harmony Sample library functional test.
	
  Description:
    This function starts the MPLAB Harmony Sample library functional test.
	
  Conditions:
    The system and test harness must be initialized and the test harness must 
    have called the TEST_SampleFunctionalInitialize to initialize the test's 
    state machine.  The test harness and this test module must be "running".
	
  Input:
    object          - Handle to the test instance object.
	
  Return:
    - true    - If the test module is successfully able to start testing.
    - false   - If the test module is unable to start testing.
	
  Example:
    None.
	
  Remarks:
    This test is normally only executed under the control of the MPLAB Harmony 
    test harness.
*/

bool TEST_SampleFunctionalStart ( SYS_MODULE_OBJ object );


/*******************************************************************************
  Function:
    void TEST_SampleFunctionalTasks ( SYS_MODULE_OBJ object, unsigned int index )

  Summary:
    MPLAB Harmony Sample library functional test tasks.

  Description:
    This function is the Sample library functional test's tasks routine.  It
    defines the state machine and core logic of the Sample library's functional
    test.

  Precondition:
    The system and test harness must be initialized and the test harness must 
    have called the TEST_SampleFunctionalInitialize to initialize the test's 
    state machine.

  Parameters:
    object          - Handle to the test instance object.
    
    index           - Index identifying which tasks function is being called.
                      This index is used to identify from which tasks context
                      the function is being called.

  Returns:
    None.

  Example:
    None.

  Remarks:
    This test is normally only executed under the control of the MPLAB Harmony 
    test harness.
 */

void TEST_SampleFunctionalTasks ( SYS_MODULE_OBJ object, unsigned int index );


/******************************************************************************
   Function:
    SYS_STATUS TEST_SampleFunctionalStatus (  SYS_MODULE_OBJ object )

  Summary:
    Provides the current status of the sample library's functional test.

  Description:
    This function provides the current status of the sample library's 
    functional test.

  Precondition:
    None.

  Parameters:
    object                      - Handle to the sample test app instance.

  Returns:
    - SYS_STATUS_ERROR            - If an error condition has occurred.
    - SYS_STATUS_UNINITIALIZED    - Before the sample test app has been 
                                    initialized.  
    - SYS_STATUS_BUSY             - If the sample test app is busy working.
    - SYS_STATUS_READY            - If the sample test app is ready and 
                                    waiting for the sample module.                                  

  Example:
    None.

  Remarks:
    This test is normally only executed under the control of the MPLAB Harmony 
    test harness.
*/

SYS_STATUS TEST_SampleFunctionalStatus (  SYS_MODULE_OBJ object );


#endif /* _TEST_SAMPLE_FUNCTIONAL_H */
/*******************************************************************************
 End of File
 */