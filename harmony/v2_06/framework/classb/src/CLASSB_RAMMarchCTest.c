/*******************************************************************************
  Class B Library implementation file

  Summary:
    This file contains the implementation for the Class B Safety
    Software Library March C RAM test for PIC32 MCUs.
    
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
#include "classb/classb.h"
#include <string.h>


// definitions
#define MARCHC_BIT_WIDTH            (sizeof(unsigned int)*8)


// prototypes
// 
static CLASSBRESULT RAMMarchCTestCycle( unsigned int* ramStartAddress, unsigned int ramSize, int isFullC);
static CLASSBRESULT RAMtestMarchC( unsigned int* ramStartAddress, unsigned int ramSize, int isFullC);
static CLASSBRESULT ReadZeroWriteOne( unsigned int* ptr);
static CLASSBRESULT ReadOneWriteZero( unsigned int* ptr);
static CLASSBRESULT ReadZero( unsigned int* ptr);


/*******************************************************************************
  Function:
    CLASSBRESULT CLASSB_RAMMarchCTest( unsigned int* ramStartAddress, 
                                       unsigned int ramSize)

  Summary:
    The RAM March C test is one of the Variable Memory tests
    that implements the Periodic Static Memory test
    H.2.19.6 as defined by the IEC 60730 standard.
    
  Remarks:
    Refer to classb.h for usage information.
    
  *****************************************************************************/

CLASSBRESULT CLASSB_RAMMarchCTest( unsigned int* ramStartAddress, unsigned int ramSize)
{
	#ifndef USE_MARCHC_MINUS
    return RAMtestMarchC(ramStartAddress, ramSize, 1);
	#else
    return RAMtestMarchC(ramStartAddress, ramSize, 0);
	#endif
}

  /*******************************************************************
  * Description:
  *     This function performs the March C RAM test
  *     It can perform the full March C or the optimized
  *     March C Minus version
  *     
  * Input:
  *     ramStartAddress :   Address start address to be tested
  *     ramSize:            size of the RAM area to be tested
  *     isFullC:            boolean that indicates the full March C test to be executed or not
  *
  * Return Values:
  *     CLASSB_TEST_FAIL :  test failed
  *     CLASSB_TEST_PASS :  test passed
  *                                                                 
  *******************************************************************/
static CLASSBRESULT RAMtestMarchC( unsigned int* ramStartAddress, unsigned int ramSize, int isFullC)
{
	unsigned int *currentAddress = ramStartAddress;
    unsigned int *endAddress     = ramStartAddress + (ramSize/sizeof(unsigned int));
    unsigned int cycleramSize;
    CLASSBRESULT returnValue = CLASSB_TEST_FAIL;
	#ifdef CLASSB_RAM_TEST_CYCLE_SIZE
	// If the cycle size is defined, then the test should be non destructive.
    unsigned int bufferAddress[CLASSB_RAM_TEST_CYCLE_SIZE/sizeof(unsigned int)];
	cycleramSize = CLASSB_RAM_TEST_CYCLE_SIZE/sizeof(unsigned int);
	if (RAMMarchCTestCycle(bufferAddress, cycleramSize, isFullC) != CLASSB_TEST_PASS)
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

		if (RAMMarchCTestCycle(currentAddress, cycleramSize, isFullC) != CLASSB_TEST_PASS)
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

static CLASSBRESULT RAMMarchCTestCycle( unsigned int* ramStartAddress, unsigned int ramSize, int isFullC)
{
    CLASSBRESULT testResult = CLASSB_TEST_FAIL; 
    unsigned int* ramEndAddress = ramStartAddress + (ramSize/4);    // ramSize should be in bytes;
    unsigned int* ptr = ramStartAddress;
  
    // erase all sram
    for(ptr=ramStartAddress; ptr<ramEndAddress; ptr++)
    {
        *ptr=0;
    }

    // Test Bitwise if 0 and replace it with 1 starting from lower Addresses
    for(ptr=ramStartAddress; ptr<ramEndAddress; ptr++)
    {
        if( ReadZeroWriteOne(ptr) !=CLASSB_TEST_PASS)
        {
            ptr = ramEndAddress + 4;
        }
    } 
    if(ptr == ramEndAddress)
    {
        // Test Bitwise if 1 and replace it with 0 starting from lower Addresses
        for(ptr=ramStartAddress; ptr<ramEndAddress; ptr++)
        {
            if( ReadOneWriteZero(ptr) !=CLASSB_TEST_PASS)
            {
                ptr = ramEndAddress + 4;
            }
        }                  
        if(ptr == ramEndAddress)
        {
            if(isFullC)
            {
                // Test if all bits are zeros starting from lower Addresses
                for(ptr=ramStartAddress; ptr<ramEndAddress; ptr++)
                {
                    if( ReadZero(ptr) !=CLASSB_TEST_PASS)
                    {
                        ptr = ramEndAddress + 4;
                    }
                }
            }
            if(ptr == ramEndAddress)
            {
                // Test Bitwise if 0 and replace it with 1 starting from higher Addresses
                for (ptr=ramEndAddress-1; ptr>=ramStartAddress; ptr--) 
                {  
                    if( ReadZeroWriteOne(ptr) !=CLASSB_TEST_PASS)
                    {
                        ptr = ramStartAddress - 4;
                    }
                }
                if(ptr == ramStartAddress-1)
                {
                    // Test Bitwise if 1 and replace it with 0 starting from higher Addresses
                    for (ptr=ramEndAddress-1; ptr>=ramStartAddress; ptr--) 
                    {  
                        if( ReadOneWriteZero(ptr) !=CLASSB_TEST_PASS)
                        {
                            ptr = ramStartAddress - 4;
                        }
                    }
                    if(ptr == ramStartAddress-1)
                    {
                        // Test if all bits are zeros starting from higher Addresses
                        for (ptr=ramEndAddress-1; ptr>=ramStartAddress; ptr--) 
                        {  
                            if( ReadZero(ptr) !=CLASSB_TEST_PASS)
                            {
                                ptr = ramEndAddress + 4;
                            }
                        }
                        if(ptr == ramStartAddress-1)
                        {
                            testResult = CLASSB_TEST_PASS;
                        }
                    }
                }
            }
        }
    }

    return testResult;
}


  /*******************************************************************
  * Description:
  *     This function tests bitwise if a bit is zero and replaces it
  *     with one.
  * Input:
  *     ptr :  Address location of the the bits to be tested
  * Return Values:
  *     CLASSB_TEST_FAIL :  test failed
  *     CLASSB_TEST_PASS :  test passed
  *                                                                 
  *******************************************************************/

static CLASSBRESULT ReadZeroWriteOne( unsigned int* ptr)
{
    int tempValue;  
    int loopCounter;
    CLASSBRESULT returnValue = CLASSB_TEST_FAIL;

    for (loopCounter=MARCHC_BIT_WIDTH-1; loopCounter>=0; loopCounter--)
    {
        tempValue =(((*ptr)>>loopCounter) & 1);  // read 0 
        if (tempValue!= 0) 
        {
            loopCounter = -10;
        } else {
            *ptr=(*ptr | (1<<loopCounter));             // write 1
        }
    }
    if(loopCounter == -1)
    {
        returnValue = CLASSB_TEST_PASS;
    }

    return returnValue;  
}

  /******************************************************************
  * Description:
  *     This function tests bitwise if a bit is one and replaces it
  *     with zero.
  * Input:
  *     ptr :  Address location of the the bits to be tested
  * Return Values:
  *     CLASSB_TEST_FAIL :  test failed
  *     CLASSB_TEST_PASS :  test passed
  *                                                                
  ******************************************************************/


static CLASSBRESULT ReadOneWriteZero( unsigned int* ptr ) 
{
    int tempValue;  
    int loopCounter;
    CLASSBRESULT returnValue = CLASSB_TEST_FAIL;

    for (loopCounter=0; loopCounter<MARCHC_BIT_WIDTH; loopCounter++)
    {
        tempValue =(((*ptr)>>loopCounter) & 1);     // read 1 
        if (tempValue!= 1) 
        {
            loopCounter = MARCHC_BIT_WIDTH + 4;
        } else {
            tempValue =  *ptr  & ~(1<<loopCounter);       // write 0
            *ptr= tempValue;     
        }
    }
    if(loopCounter == MARCHC_BIT_WIDTH)
    {
        returnValue = CLASSB_TEST_PASS;
    }
    return returnValue; 
}

  /***********************************************************
  * Description:
  *     This function tests bitwise if all bits are zeros .
  * Input:
  *     ptr :  Address location of the the bits to be tested
  * Return Values:
  *     CLASSB_TEST_FAIL :  test failed
  *     CLASSB_TEST_PASS :  test passed
  *                                                         
  ***********************************************************/
static CLASSBRESULT ReadZero( unsigned int * ptr ) 
{
    int tempValue;  
    int loopCounter;
    CLASSBRESULT returnValue = CLASSB_TEST_FAIL;

    for (loopCounter=0; loopCounter<MARCHC_BIT_WIDTH; loopCounter++)
    {
        tempValue =(((*ptr)>>loopCounter) & 1);    // read 0 
        if (tempValue!= 0) 
        {
            loopCounter = MARCHC_BIT_WIDTH + 4;
        }
    }
    if(loopCounter == MARCHC_BIT_WIDTH)
    {
        returnValue = CLASSB_TEST_PASS;
    }
    return returnValue; 
}

