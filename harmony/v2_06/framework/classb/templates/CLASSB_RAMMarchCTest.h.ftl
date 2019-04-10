
<#if CONFIG_USE_CLASSB_RAM_MARCHC_TEST_MINUS>
#define USE_MARCHC_MINUS
</#if>

/*******************************************************************************
  Function:
    CLASSBRESULT CLASSB_RAMMarchCTest(unsigned int* ramStartAddress, 
                                      unsigned int ramSize)

  Summary:
    The RAM March C test is one of the Variable Memory tests
    that implements the Periodic Static Memory test
    H.2.19.6 as defined by the IEC 60730 standard.
    

  Description:
    This test is a complete and non redundant test capable of detecting
    stuck-at, addressing, transition and coupling faults.
    This test is of complexity 11n( Where n is the number of bits tested). 
    The test uses word (32-bit) accesses.
    The address must be properly word aligned and the length of the
    area to be tested must be an integral multiple of the data width access.
                                                                             
  Precondition:
    None.

  Parameters:
    ramStartAddress     - start Address from which the March C test is to be performed
                          Must be properly 32 bit aligned.

    ramSize             - number of consecutive byte locations for which the test is to be performed
                          The size must be a number multiple of 4.
    
  Returns:
    Result identifying the pass/fail status of the test:
      CLASSB_TEST_PASS    - The test passed. RAM area tested has not been detected to have faults. 

      CLASSB_TEST_FAIL    - The test failed. Some RAM area location has been detected to have faults. 

  Example:
    <code>
    int testRes=CLASSB_RAMMarchCTest(startAddress, size);
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
    This is a destructive memory test.
    Either exclude from this test RAM areas that have to be preserved
    or save/restore the memory area before/after running the test
    or run the test at system startup before the memory and the
    run time library is initialized (stack needs to be initialized though).
    
    At least 100 bytes should be available for stack for executing the March C test.
    The tested RAM area must not overlap the stack.
    
    Other statically allocated resources,  such as the MPLAB ICD/Real ICE
    allocated RAM buffers should be excluded from this test.    
    
    The Start Address from which the March C test is to be performed is
    PIC32 variant and application dependent. It is a run-time parameter.
    
    The routine accesses one 4 byte RAM word at a time.        
    
  *****************************************************************************/
CLASSBRESULT CLASSB_RAMMarchCTest(unsigned int* ramStartAddress, unsigned int ramSize);
