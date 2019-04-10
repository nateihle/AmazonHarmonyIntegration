/*******************************************************************************
  Function:
    int CLASSB_CPURegistersTest ( void )

  Summary:
    The CPU Register test implements the functional test
    H.2.16.5 as defined by the IEC 60730 standard.
    

  Description:
    This routine detects stuck-at Faults in the CPU registers.
    This ensures that the bits in the registers are not stuck at
    a value ‘0’ or ‘1’.

  Precondition:
    None.

  Parameters:
    None.
    
  Returns:
    Result identifying the pass/fail status of the test:
      CLASSB_TEST_PASS    - The test passed. CPU registers have not been detected to have stuck bits.

      CLASSB_TEST_FAIL    - The test failed. Some CPU register(s) has been detected to have stuck bits. 

  Example:
    <code>
    int testRes=CLASSB_CPURegistersTest();
    if(testRes==CLASSB_TEST_PASS)
    {
        // process test success
    }
    else
    {
        // process tests failure
    }
    </code>

  Remarks:
    This is a non-destructive test.
    
    Interrupts should be disabled when calling this test function.

  *****************************************************************************/
CLASSBRESULT CLASSB_CPURegistersTest(void);
