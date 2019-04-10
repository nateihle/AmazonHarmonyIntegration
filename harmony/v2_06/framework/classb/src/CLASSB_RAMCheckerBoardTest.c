/*******************************************************************************
  Class B Library implementation file

  Summary:
    This file contains the implementation for the
    the Class B Safety Software Library RAM Checkerboard Tests
    for PIC32 MCUs.
        
*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to  you  the  right  to  use,  modify,  copy  and  distribute
Software only when embedded on a Microchip  microcontroller  or  digital  signal
controller  that  is  integrated  into  your  product  or  third  party  product
(pursuant to the  sublicense  terms  in  the  accompanying  license  agreement).

You should refer  to  the  license  agreement  accompanying  this  Software  for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS  WITHOUT  WARRANTY  OF  ANY  KIND,
EITHER EXPRESS  OR  IMPLIED,  INCLUDING  WITHOUT  LIMITATION,  ANY  WARRANTY  OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A  PARTICULAR  PURPOSE.
IN NO EVENT SHALL MICROCHIP OR  ITS  LICENSORS  BE  LIABLE  OR  OBLIGATED  UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,  BREACH  OF  WARRANTY,  OR
OTHER LEGAL  EQUITABLE  THEORY  ANY  DIRECT  OR  INDIRECT  DAMAGES  OR  EXPENSES
INCLUDING BUT NOT LIMITED TO ANY  INCIDENTAL,  SPECIAL,  INDIRECT,  PUNITIVE  OR
CONSEQUENTIAL DAMAGES, LOST  PROFITS  OR  LOST  DATA,  COST  OF  PROCUREMENT  OF
SUBSTITUTE  GOODS,  TECHNOLOGY,  SERVICES,  OR  ANY  CLAIMS  BY  THIRD   PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE  THEREOF),  OR  OTHER  SIMILAR  COSTS.
*******************************************************************************/
// DOM-IGNORE-END

/* global defines */
#include "classb/classb.h"
#include <string.h>

static CLASSBRESULT RAMCheckerBoardTestCycle(unsigned int *cycleStartAddress, unsigned int cycleSize);

/*******************************************************************************
  Function:
    CLASSBRESULT CLASSB_RAMCheckerboardTest(ramStartAddress, ramSize, bufferAddress)

  Summary:
    The RAM Checker Board test implements one of the functional tests
    H.2.19.6 as defined by the IEC 60730 standard.
    

  Remarks:
    Refer to classb.h for usage information.
    
  *****************************************************************************/
CLASSBRESULT CLASSB_RAMCheckerBoardTest(unsigned int *ramStartAddress, unsigned int ramSize)
{
    unsigned int *currentAddress = ramStartAddress;
    unsigned int *endAddress     = ramStartAddress + (ramSize/sizeof(unsigned int));
    unsigned int cycleramSize;
    CLASSBRESULT returnValue = CLASSB_TEST_FAIL;
	#ifdef CLASSB_RAM_TEST_CYCLE_SIZE
	// If the cycle size is defined, then the test should be non destructive.
    unsigned int bufferAddress[CLASSB_RAM_TEST_CYCLE_SIZE/sizeof(unsigned int)];
	cycleramSize = CLASSB_RAM_TEST_CYCLE_SIZE/sizeof(unsigned int);
	if (RAMCheckerBoardTestCycle(bufferAddress, cycleramSize) != CLASSB_TEST_PASS)
	{
		cycleramSize = 0;  // Since a failure is detected, abort any remaining test.
	}
	#else
	cycleramSize = ramSize/sizeof(unsigned int);
	#endif

	// cycleramSize should only be 0 if 0 was passed as the ramSize, 
	// or if a failure was detected.
    while((currentAddress < endAddress)  &&  (cycleramSize != 0))
    {
		#ifdef CLASSB_RAM_TEST_CYCLE_SIZE
		// If the cycle size is defined, then the test should be non destructive.
        // Save the next block to be tested.
        memcpy(bufferAddress, currentAddress, CLASSB_RAM_TEST_CYCLE_SIZE);
		#endif
		if (RAMCheckerBoardTestCycle(currentAddress, cycleramSize) != CLASSB_TEST_PASS)
		{
			cycleramSize = 0;  // Since a failure is detected, abort any remaining test.
		}

		#ifdef CLASSB_RAM_TEST_CYCLE_SIZE
		// If the cycle size is defined, then the test should be non destructive.
        // Restore data from test block
        // NOTE: Although this is meant to be a non destructive test, if the memory 
        // fails, then restoring information to it may fail also.
        memcpy(currentAddress, bufferAddress, CLASSB_RAM_TEST_CYCLE_SIZE);
		#endif
        // Prepare to test the next block.
        currentAddress += cycleramSize;
    }
	// cycle ramSize should only get set to 0 if a failure is detected.
	if((cycleramSize != 0))
	{
		returnValue = CLASSB_TEST_PASS;
	}
    return returnValue;
}

CLASSBRESULT RAMCheckerBoardTestCycle(unsigned int *cycleStartAddress, unsigned int cycleSize)
{
    unsigned int i;
    CLASSBRESULT returnValue = CLASSB_TEST_FAIL;
	// Fill the next block with the test pattern.
	for(i=0; i < cycleSize; i+=2)
	{
		cycleStartAddress[i] = 0x55555555;
		cycleStartAddress[i+1] = 0xaaaaaaaa;
	}
	// Test that the block contains the test pattern
	for(i=0; i < cycleSize; i+=2)
	{
		if (cycleStartAddress[i] != 0x55555555)
		{
			i = cycleSize+1;  // Since a failure is detected, abort any remaining test.
		}
		else if (cycleStartAddress[i+1] != 0xaaaaaaaa)
		{
			i = cycleSize+1;  // Since a failure is detected, abort any remaining test.
		}
	}
	if(i == cycleSize)
	{
		// Fill the next block with the test pattern.
		for(i=0; i < cycleSize; i+=2)
		{
			cycleStartAddress[i] = 0xaaaaaaaa;
			cycleStartAddress[i+1] = 0x55555555;
		}
		// Test that the block contains the test pattern
		for(i=0; i < cycleSize; i+=2)
		{
			if (cycleStartAddress[i] != 0xaaaaaaaa)
			{
				i = cycleSize+1;  // Since a failure is detected, abort any remaining test.
			}
			else if (cycleStartAddress[i+1] != 0x55555555)
			{
				i = cycleSize+1;  // Since a failure is detected, abort any remaining test.
			}
		}
		if(i == cycleSize)
		{
			returnValue = CLASSB_TEST_PASS;
		}
	}
    return returnValue;
}
