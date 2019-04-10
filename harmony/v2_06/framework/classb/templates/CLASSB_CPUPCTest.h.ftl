/*******************************************************************************
  Function:
    int CLASSB_CPUPCTest (void)

  Summary:
    The Program Counter test implements one of the functional tests
    H.2.16.5 as defined by the IEC 60730 standard.
    

  Description:
    The Program Counter test is a functional test of the PC.
    It checks that the PC register is not stuck and it properly holds the address
    of the next instruction to be executed.
    
    The tests performs the following major tasks:
        1. The Program Counter test invokes functions that are located in memory at different addresses.
        2. That all of the functions is executed is verified.
                                                                             
  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Result identifying the pass/fail status of the test:
        CLASSB_TEST_PASS    - The test passed. The PC register holds the correct address.

        CLASSB_TEST_FAIL    - The test failed. The PC register has been detected to hold an incorrect address. 

  Example:
    <code>
    int testRes=CLASSB_CPUPCTest();
    if(testRes==PC_TEST_PASS)
    {
        // process test success
    }
    else
    {
        // process tests failure
    }
    </code>

  Remarks:
    The test uses 3 different functions:
        CLASSB_CPUPCTestFunction1()
        CLASSB_CPUPCTestFunction2()          
        CLASSB_CPUPCTestFunction3()          
  *****************************************************************************/
CLASSBRESULT CLASSB_CPUPCTest(void);

