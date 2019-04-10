/*******************************************************************************
  Function:
    CLASSBRESULT CLASSB_RAMMarchCStackTest (unsigned int* ramStartAddress, 
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
    The addresses must be properly word aligned and the lengths of the
    areas to be tested must be an integral multiple of the data width access.
                                                                             
  Precondition:
    None.

  Parameters:
    ramStartAddress     - start Address of RAM area to be used as temporary stack.
						  The stack will be copied into this memory, and the stack pointer 
						  redirected here.
                          Has to NOT overlap the Stack area!
                          Must be properly 32 bit aligned.

    ramSize             - number of consecutive byte locations for which the test is to be performed
						  The portion of the stack to be tested will be from the Stack Start (Highest address)
						  to this address minus ramSize.
						  This value also defines the amount of memory available as a temporary stack.
                          The size must be a number multiple of 4.
                          The size of the RAM area tested has to be >= 128 bytes.
                          
    
  Returns:
    Result identifying the pass/fail status of the test:
      CLASSB_TEST_PASS          - The test passed. RAM and Stack area tested have not been detected to have faults. 

      CLASSB_TEST_FAIL          - The test failed. Either some RAM or Stack area location has been detected to have faults, 

      
      
  Example:
    <code>
    int testRes=CLASSB_RAMMarchCStackTest(startAddress, size);
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

    This function is just a helper to March C test both a regular RAM area and a Stack area.
    First the RAM area is tested using the standard March C test.
    If the test succeeded, the requested Stack area is saved/copied in the RAM area
    that has just been tested and then the March C test is run over the Stack area
    as if it were a regular RAM area.
    The saved Stack area is restored and the result of the test is returned to the user.

    The RAM and Stack areas have to NOT overlap!
    The Stack grows downwards so the tested area is:
    [stackTopAddress-stackSize, stackTopAddress]
    Also the size of the Stack area to be tested has to be less than the size of the RAM area.
 
    The processor SP register is changed to the point to the RAM area while the Stack area is tested.
    Since running the MARC C test requires at least 128 bytes of stack, the RAM area size should be at least
    128 bytes long.
    Once the Stack area is tested, the SP register is restored.
    
    This is a destructive memory test.
    Either exclude from this test RAM areas that have to be preserved
    or save/restore the memory area before/after running the test
    or run the test at system startup before the memory and the
    run time library is initialized (stack needs to be initialized though).
    
    At least 128 bytes should be available for stack for executing the March C test.
    The tested RAM area must not overlap the stack.
    
    Other statically allocated resources,  such as the MPLAB ICD/Real ICE
    allocated RAM buffers should be excluded from this test.    
    
    The Start Address from which the March C test is to be performed is
    PIC32 variant and application dependent. It is a run-time parameter.
    
    The routine accesses one 4 byte RAM word at a time.        
    
  *****************************************************************************/
CLASSBRESULT CLASSB_RAMMarchCStackTest(unsigned int* ramStartAddress, unsigned int ramSize);
