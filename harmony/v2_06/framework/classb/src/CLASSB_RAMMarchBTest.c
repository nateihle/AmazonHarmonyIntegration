/*******************************************************************************
  Class B Library implementation file

  Summary:
    This file contains the implementation for the Class B Safety
    Software Library March B RAM test for PIC32 MCUs.
    
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

#define MARCHB_BIT_WIDTH        32      //  access data width


// prototypes
// 
static CLASSBRESULT Rd0Wr1Rd1Wr0Rd0Wr1( unsigned int* ptr );
static CLASSBRESULT Rd1Wr0Wr1( volatile unsigned int* ptr );
static CLASSBRESULT Rd1Wr0Wr1Wr0( volatile unsigned int* ptr );
static CLASSBRESULT Rd0Wr1Wr0( volatile unsigned int* ptr );
static CLASSBRESULT RAMMarchBTestCycle(unsigned int* cycleStartAddress, unsigned int cycleSize);



/*******************************************************************************
  Function:
    CLASSBRESULT CLASSB_RAMMarchBTest(unsigned int* ramStartAddress, 
                                      unsigned int ramSize)

  Summary:
    The RAM March B test is one of the Variable Memory tests
    that implements the Periodic Static Memory test
    H.2.19.6 as defined by the IEC 60730 standard.
    
  Remarks:
    Refer to classb.h for usage information.
    
  *****************************************************************************/
CLASSBRESULT CLASSB_RAMMarchBTest(unsigned int* ramStartAddress, unsigned int ramSize)
{
	unsigned int *currentAddress = ramStartAddress;
    unsigned int *endAddress     = ramStartAddress + (ramSize/sizeof(unsigned int));
    unsigned int cycleramSize;
    CLASSBRESULT returnValue = CLASSB_TEST_FAIL;
	#ifdef CLASSB_RAM_TEST_CYCLE_SIZE
	// If the cycle size is defined, then the test should be non destructive.
    unsigned int bufferAddress[CLASSB_RAM_TEST_CYCLE_SIZE/sizeof(unsigned int)];
	cycleramSize = CLASSB_RAM_TEST_CYCLE_SIZE/sizeof(unsigned int);
	if (RAMMarchBTestCycle(bufferAddress, cycleramSize) != CLASSB_TEST_PASS)
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

		if (RAMMarchBTestCycle(currentAddress, cycleramSize) != CLASSB_TEST_PASS)
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
		
CLASSBRESULT RAMMarchBTestCycle(unsigned int* cycleStartAddress, unsigned int cycleSize)
{
    unsigned int i;
    CLASSBRESULT returnValue = CLASSB_TEST_FAIL;

	// erase the test memory
	for(i=0; i < cycleSize; i+=1)
	{
		cycleStartAddress[i] = 0;
	}
	if(i == cycleSize)
	{
		for(i=0; i < cycleSize; i+=1)
		{
			if (Rd0Wr1Rd1Wr0Rd0Wr1(&cycleStartAddress[i]) != CLASSB_TEST_PASS)
			{
				i = cycleSize+1;  // Since a failure is detected, abort any remaining test.
			}
		}
		if(i == cycleSize)
		{
			for(i=0; i < cycleSize; i+=1)
			{
				if (Rd1Wr0Wr1(&cycleStartAddress[i]) != CLASSB_TEST_PASS)
				{
					i = cycleSize+1;  // Since a failure is detected, abort any remaining test.
				}
			}

			if(i == cycleSize)
			{
				for(i=0; i < cycleSize; i+=1)
				{
					if (Rd1Wr0Wr1Wr0(&cycleStartAddress[i]) != CLASSB_TEST_PASS)
					{
						i = cycleSize+1;  // Since a failure is detected, abort any remaining test.
					}
				}

				if(i == cycleSize)
				{
					for(i=0; i < cycleSize; i+=1)
					{
						if (Rd0Wr1Wr0(&cycleStartAddress[i]) != CLASSB_TEST_PASS)
						{
							i = cycleSize+1;  // Since a failure is detected, abort any remaining test.
						}
					}
					if(i == cycleSize)
					{
						returnValue = CLASSB_TEST_PASS;
					}
				}
			}
		}
	}
    return returnValue;
}		
		
  /**************************************************************
  * Description:
  *     This function does the following :
  *     
  *     1> Tests bitwise if bit is zero and replace with one .
  *     
  *     2> Tests bitwise if bit is one and replace with zero.
  *     
  *     3> Tests bitwise if bit is zero and replace with one.
  * Input:
  *     ptr :  Address location of the the bits to be tested.
  *
  * Return Values:
  *     CLASSB_TEST_FAIL : test failed
  *     CLASSB_TEST_PASS : test passed
  *                                                            
  **************************************************************/

static CLASSBRESULT Rd0Wr1Rd1Wr0Rd0Wr1( unsigned int *ptr)
{

    int tempValue;  
    int loopCounter;

    for (loopCounter=0; loopCounter<MARCHB_BIT_WIDTH; loopCounter++)
    {

        tempValue =  (((*ptr) >> loopCounter) & 1);   //Read 0

        if( tempValue != 0)
        { 
            return CLASSB_TEST_FAIL;
        }

        tempValue = *ptr | (1<<loopCounter);             // write 1
        *ptr= tempValue;                              

        tempValue =(((*ptr)>>loopCounter) & 1);       // read 1 

        if (tempValue!= 1) 
        {
            return CLASSB_TEST_FAIL;
        }

        tempValue =  *ptr  & ~(1<<loopCounter);         // write 0
        *ptr= tempValue;                                 

        tempValue =(((*ptr)>>loopCounter) & 1);      // read 0 

        if (tempValue!= 0) 
        {
            return CLASSB_TEST_FAIL;
        } 

        *ptr=(*ptr | (1<<loopCounter));                  // write 1

    } 
    return CLASSB_TEST_PASS; 

}

  /******************************************************************
  * Description:
  *     This function does the following:
  *     
  *     1> Tests bitwise if a bit is one and replace it with Zero.
  *     
  *     2> Write One Bitwise.
  * Input:
  *     ptr :  Address location of the the bits to be tested.
  *
  * Return Values:
  *     CLASSB_TEST_FAIL : test failed
  *     CLASSB_TEST_PASS : test passed
  *                                                                
  ******************************************************************/
static CLASSBRESULT Rd1Wr0Wr1( volatile unsigned int * ptr)
{
    int tempValue;  
    int loopCounter;

    for (loopCounter=0; loopCounter<MARCHB_BIT_WIDTH; loopCounter++)
    {
        tempValue =(((*ptr)>>loopCounter) & 1);       // read 1 
        if ( tempValue!= 1) 
        {
            return CLASSB_TEST_FAIL;
        }

        tempValue =  *ptr  & ~(1<<loopCounter);
        *ptr=  tempValue;                                // write 0

        *ptr=(*ptr | (1<<loopCounter));                  // write 1

    }

    return CLASSB_TEST_PASS; 
}

  /******************************************************************
  * Description:
  *     This function does the following:
  *     
  *     1> Tests bitwise if a bit is one and replace it with Zero.
  *     
  *     2> write one Bitwise.
  *     
  *     3> write Zero Bitwise.
  * Input:
  *     ptr :  Address location of the the bits to be tested.
  * Return Values:
  *     CLASSB_TEST_FAIL : test failed
  *     CLASSB_TEST_PASS : test passed
  *                                                                
  ******************************************************************/
 
static CLASSBRESULT Rd1Wr0Wr1Wr0( volatile unsigned int * ptr)
{

    int tempValue;  
    int loopCounter;

    for (loopCounter=MARCHB_BIT_WIDTH-1; loopCounter>=0; loopCounter--)
    {

        tempValue =(((*ptr)>>loopCounter) & 1);     // read 1 
        if (tempValue!= 1) 
        {
            return CLASSB_TEST_FAIL;
        }

        tempValue =  *ptr & ~(1<<loopCounter);
        *ptr= tempValue;                               // write 0

        *ptr=(*ptr | (1<<loopCounter));                // write 1

        tempValue =  *ptr & ~(1<<loopCounter);
        *ptr= tempValue;                               // write 0
    }

    return CLASSB_TEST_PASS; 
}
  /******************************************************************
  * Description:
  *     This function does the following:
  *     
  *     1> Tests bitwise if a bit is zero and replace it with one.
  *     
  *     2> Write Zero.
  * Input:
  *     ptr :  Address location of the the bits to be tested.
  * Return Values:
  *     CLASSB_TEST_FAIL : test failed
  *     CLASSB_TEST_PASS : test passed
  *                                                                
  ******************************************************************/

static CLASSBRESULT Rd0Wr1Wr0( volatile unsigned int * ptr)
{

    int tempValue;  
    int loopCounter;

    for (loopCounter=MARCHB_BIT_WIDTH-1; loopCounter>=0; loopCounter--)
    {
        tempValue =(((*ptr)>>loopCounter) & 1);     // read 0 
        if (tempValue!= 0) 
        {
            return CLASSB_TEST_FAIL;
        } 

        *ptr=(*ptr | (1<<loopCounter));                // write 1

        tempValue =  *ptr & ~(1<<loopCounter);
        *ptr= tempValue;                               // write 0

    }

    return CLASSB_TEST_PASS; 
}


