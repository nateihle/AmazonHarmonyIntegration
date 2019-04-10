/*******************************************************************************
  Function:
    int CLASSB_RAMCheckerBoardTest (int* ramStartAddress, int ramSize)

  Summary:
    The RAM Checker Board test implements one of the functional tests
    H.2.19.6 as defined by the IEC 60730 standard.
    

  Description:
    This routine detects single bit Faults in the variable memory.
    This ensures that the bits in the tested RAM are not stuck at
    a value ‘0’ or ‘1’.

    The test writes the checkerboard pattern (0x55555555 followed by 0xaaaaaaaa)
    to adjacent memory locations starting at ramStartAddress.
	It performs the following steps:                                           
		1. A temporary memory buffer (stack) is tested for reliability.
		1. The content of a 64 bytes memory chunk to be tested is saved in    
		   temporary memory buffer.
		2. Writes the pattern 0x55555555 followed by 0xaaaaaaaa to adjacent memory locations 
		   filling up the 64 bytes memory chunk.                          
		3. It reads the memory chunk adjacent locations and checks that the read-back values match
           the written pattern.
           If the values match set the success result and go to step 4.
           Else set the error result and go to step 6.
		4. Writes the inverted pattern 0xaaaaaaaa followed by 0x55555555 to adjacent memory locations 
		   filling up the 64 bytes memory chunk.
		5. It reads the memory chunk adjacent locations and checks that the read-back values match
           the written pattern.
           If the values match set the success result.
           Else set the error result.
		6. The content of the tested 64 bytes memory chunk is restored from the
           temporary memory buffer.
        7. If the result shows error the test is done and returns.
        8. The address pointer is incremented to point to the next sequential 64 bytes memory chunk
           and the test is repeated from step 1 until all the number of requested memory locations
           is tested.
                                                                             
  Precondition:
    None.

  Parameters:
    ramStartAddress     - start Address from which the checker Board test is to be performed
                          Must be properly 32 bit aligned.

    ramSize             - number of consecutive byte locations for which the test is to be performed
                          The size must be a number multiple of 64.
    
  Returns:
    Result identifying the pass/fail status of the test:
      CLASSB_TEST_PASS    - The test passed. RAM area tested has not been detected to have stuck bits.

      CLASSB_TEST_FAIL    - The test failed. Some RAM area location has been detected to have stuck bits. 

  Example:
    <code>
    int testRes=CLASSB_RAMCheckerBoardTest(startAddress, size);
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
    This is a non-destructive memory test. The content of the tested memory area is saved and restored.
    The test operates in 64 bytes long memory chunks at a time.
    
    At least 32 bytes should be available for stack for executing the RAM Checker Board test.
    The tested RAM area must not overlap the stack.
    
    The Start Address from which the Checker Board test is to be performed is
    PIC32 variant and application dependent. It is a run-time parameter.
    
    The routine accesses one 4 byte RAM word at a time.

    *****************************************************************************/
CLASSBRESULT CLASSB_RAMCheckerBoardTest(unsigned int *startAddress, unsigned int length);
