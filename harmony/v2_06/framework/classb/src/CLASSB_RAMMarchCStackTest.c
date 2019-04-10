/*******************************************************************************
  Class B Library implementation file

  Summary:
    This file contains the implementation for the
    the Class B Safety Software Library March C Stack Tests
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
#include "classb/classb.h"
#include <string.h>

/*******************************************************************************
  Function:
	CLASSBRESULT CLASSB_RAMMarchCStackTest (unsigned int* ramStartAddress, 
											unsigned int ramSize)

  Summary:
    The RAM March C test is one of the Variable Memory tests
    that implements the Periodic Static Memory test
    H.2.19.6 as defined by the IEC 60730 standard.
    
  Remarks:
    Refer to classb.h for usage information.
    
  *****************************************************************************/
    extern int  _stack[];                   // the address of the stack, as placed by the linker
    extern int  _min_stack_size[];          // size of the stack, as defined in the linker
    volatile register unsigned char* regstackptr __asm__("sp");  // Stack register variable.

CLASSBRESULT CLASSB_RAMMarchCStackTest (unsigned int* ramStartAddress, unsigned int ramSize)
{
    register unsigned char* regRamStartAddress asm("s0") = (unsigned char*)ramStartAddress;
    register unsigned int  regRamSize asm("s1")          = ramSize;
    register unsigned char* regStackTopAddress asm("s2") = (unsigned char*)_stack;
    register unsigned char* regStackBottom asm("s4")     = regStackTopAddress-regRamSize;
    register unsigned int regStackOffset asm("s5")       = (regStackTopAddress-regstackptr);
    register CLASSBRESULT returnValue asm("s6")          = CLASSB_TEST_FAIL;
  
	// First the RAM area is tested using the standard March C test.
	returnValue = CLASSB_RAMMarchCTest( (unsigned int *)regRamStartAddress, regRamSize);

	if(returnValue == CLASSB_TEST_PASS)
	{
        returnValue = CLASSB_TEST_FAIL;
        // Copy the testable stack area to the Ram area.
		memcpy(regRamStartAddress, regStackBottom, regRamSize);
		//Switch the stack pointer to this RAM area minus the current offset.
		regstackptr = ((regRamStartAddress+regRamSize)-regStackOffset);
		// then the March C test is run over the Stack area as if it were a regular RAM area
		returnValue = CLASSB_RAMMarchCTest( (unsigned int *)regStackBottom, regRamSize);
		memcpy(regStackBottom, regRamStartAddress, regRamSize);
		regstackptr = regStackTopAddress-regStackOffset;
	}
    return returnValue;
}
