/*******************************************************************************
  Class B Library implementation file.

  Summary:
    This file contains the implementation for the Program Counter test for 
    the Class B Safety Software Library for PIC32.
        
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


void CLASSB_CPUPCTestFunction1(void);
void CLASSB_CPUPCTestFunction2(void);
void CLASSB_CPUPCTestFunction3(void);

void CLASSB_PCTrap1 (void);
void CLASSB_PCTrap2 (void);
void CLASSB_PCTrap3 (void);

/*******************************************************************************
  Function:
    CLASSBRESULT CLASSB_CPUPCTest(void)

  Summary:
    The Program Counter test implements one of the functional tests
    H.2.16.5 as defined by the IEC 60730 standard.

  Remarks:
    Refer to classb.h for usage information.

    *****************************************************************************/
static unsigned int PCTestFlag = 0;

CLASSBRESULT CLASSB_CPUPCTest(void)
{
    PCTestFlag = 3;
    CLASSB_PCTrap1();
    CLASSB_PCTrap2();
    CLASSB_PCTrap3();
    CLASSBRESULT result = CLASSB_TEST_FAIL;
    
    CLASSB_CPUPCTestFunction1();
    CLASSB_CPUPCTestFunction2();
    CLASSB_CPUPCTestFunction3();

    if (PCTestFlag == 0)
    {
        result = CLASSB_TEST_PASS;
    }
    return result;
}

/*******************************************************************************
  Function:
    void CLASSB_CPUPCTestFunction1(void)
    void CLASSB_CPUPCTestFunction2(void)
    void CLASSB_CPUPCTestFunction3(void)

  Summary:
    Helpers for the Program Counter functional test.
    

  Description:
    These are helper functions that return their own address.
                                                                             
  Precondition:
    None.

  Parameters:
    None. 

  Returns:
    None

  Example:
    See the CLASSB_CPUPCTest() example.

  Remarks:
    The functions are placed at specific addresses so that the PC has to set
    and clear as many bits as possible given the size of the programmable 
    memory.
    
  *****************************************************************************/
#define PCTestBaseAddr 0x9D000000
// We base the final address of these functions on the total size of memory
// in the particular part.  This allows us to test more program counter bits.
// NOTE! Depending on modifications to other portions of this library and
// the application which may use it, there is no guarantee that the code fits
// into these spaces.
// Less than 0x4000
#if defined (__PIC32_MEMORY_SIZE)
//#warning "Using Memory Size:"
#if __PIC32_MEMORY_SIZE__ == 016
//#warning "Using Memory Size:016"
#define PCTestAddr1 PCTestBaseAddr + 0x3000
#define PCTestAddr2 PCTestBaseAddr + 0x3AA0
#define PCTestAddr3 PCTestBaseAddr + 0x3550
// Less than 0x8000
#elif __PIC32_MEMORY_SIZE__ == 032
//#warning "Using Memory Size:032"
#define PCTestAddr1 PCTestBaseAddr + 0x6000
#define PCTestAddr2 PCTestBaseAddr + 0x7AA0
#define PCTestAddr3 PCTestBaseAddr + 0x5550

// Less than 0x10000
#elif __PIC32_MEMORY_SIZE__ == 064
//#warning "Using Memory Size:064"
#define PCTestAddr1 PCTestBaseAddr + 0xc000
#define PCTestAddr2 PCTestBaseAddr + 0xAAA0
#define PCTestAddr3 PCTestBaseAddr + 0x5550

// Less than 0x20000
#elif __PIC32_MEMORY_SIZE__ == 128
//#warning "Using Memory Size:128"
#define PCTestAddr1 PCTestBaseAddr + 0x1c000
#define PCTestAddr2 PCTestBaseAddr + 0x1AAA0
#define PCTestAddr3 PCTestBaseAddr + 0x15550

// Less than 0x40000
#elif __PIC32_MEMORY_SIZE__ == 256
//#warning "Using Memory Size:256"
#define PCTestAddr1 PCTestBaseAddr + 0x20000
#define PCTestAddr2 PCTestBaseAddr + 0x2AAA0
#define PCTestAddr3 PCTestBaseAddr + 0x25550

// Less than 0x80000
#elif __PIC32_MEMORY_SIZE__ == 512
//#warning "Using Memory Size:512"
#define PCTestAddr1 PCTestBaseAddr + 0x76000
#define PCTestAddr2 PCTestBaseAddr + 0x7AAA0
#define PCTestAddr3 PCTestBaseAddr + 0x75550

// Less than 0x100000
#elif __PIC32_MEMORY_SIZE__ == 1024
//#warning "Using Memory Size:1024"
#define PCTestAddr1 PCTestBaseAddr + 0xc0000
#define PCTestAddr2 PCTestBaseAddr + 0xAAAA0
#define PCTestAddr3 PCTestBaseAddr + 0x55550

// Less than 0x200000
#elif __PIC32_MEMORY_SIZE__ == 2048
//#warning "Using Memory Size:2048"
#define PCTestAddr1 PCTestBaseAddr + 0x100000
#define PCTestAddr2 PCTestBaseAddr + 0x1AAAA0
#define PCTestAddr3 PCTestBaseAddr + 0x155550

// Assume more than 2048k 0x200000
#else
//#warning "Using Memory Size:OTHER"
#define PCTestAddr1 PCTestBaseAddr + 0x100000
#define PCTestAddr2 PCTestBaseAddr + 0x1AAAA0
#define PCTestAddr3 PCTestBaseAddr + 0x155550
#endif
#elif defined (__PIC32_FLASH_SIZE)
//#warning "Using FLASH Size:"
#if __PIC32_FLASH_SIZE == 016
//#warning "Using FLASH Size:032"
#define PCTestAddr1 PCTestBaseAddr + 0x3000
#define PCTestAddr2 PCTestBaseAddr + 0x3AA0
#define PCTestAddr3 PCTestBaseAddr + 0x3550
// Less than 0x8000
#elif __PIC32_FLASH_SIZE == 032
//#warning "Using FLASH Size:032"
#define PCTestAddr1 PCTestBaseAddr + 0x6000
#define PCTestAddr2 PCTestBaseAddr + 0x7AA0
#define PCTestAddr3 PCTestBaseAddr + 0x5550

// Less than 0x10000
#elif __PIC32_FLASH_SIZE == 064
//#warning "Using FLASH Size:064"
#define PCTestAddr1 PCTestBaseAddr + 0xc000
#define PCTestAddr2 PCTestBaseAddr + 0xAAA0
#define PCTestAddr3 PCTestBaseAddr + 0x5550

// Less than 0x20000
#elif __PIC32_FLASH_SIZE == 128
//#warning "Using FLASH Size:128"
#define PCTestAddr1 PCTestBaseAddr + 0x1c000
#define PCTestAddr2 PCTestBaseAddr + 0x1AAA0
#define PCTestAddr3 PCTestBaseAddr + 0x15550

// Less than 0x40000
#elif __PIC32_FLASH_SIZE == 256
//#warning "Using FLASH Size:256"
#define PCTestAddr1 PCTestBaseAddr + 0x20000
#define PCTestAddr2 PCTestBaseAddr + 0x2AAA0
#define PCTestAddr3 PCTestBaseAddr + 0x25550

// Less than 0x80000
#elif __PIC32_FLASH_SIZE == 512
//#warning "Using FLASH Size:512"
#define PCTestAddr1 PCTestBaseAddr + 0x76000
#define PCTestAddr2 PCTestBaseAddr + 0x7AAA0
#define PCTestAddr3 PCTestBaseAddr + 0x75550

// Less than 0x100000
#elif __PIC32_FLASH_SIZE == 1024
//#warning "Using FLASH Size:1024"
#define PCTestAddr1 PCTestBaseAddr + 0xc0000
#define PCTestAddr2 PCTestBaseAddr + 0xAAAA0
#define PCTestAddr3 PCTestBaseAddr + 0x55550

// Less than 0x200000
#elif __PIC32_FLASH_SIZE == 2048
//#warning "Using FLASH Size:2048"
#define PCTestAddr1 PCTestBaseAddr + 0x1c0000
#define PCTestAddr2 PCTestBaseAddr + 0x1AAAA0
#define PCTestAddr3 PCTestBaseAddr + 0x155550

// Assume more than 2048k 0x200000
#else
//#warning "Using FLASH Size:OTHER"
#define PCTestAddr1 PCTestBaseAddr + 0x1c0000
#define PCTestAddr2 PCTestBaseAddr + 0x1AAAA0
#define PCTestAddr3 PCTestBaseAddr + 0x155550
#endif
#endif
void __attribute__((address(PCTestAddr1))) CLASSB_CPUPCTestFunction1(void) 
{
    PCTestFlag--;
}

void __attribute__((address(PCTestAddr2))) CLASSB_CPUPCTestFunction2(void) 
{
    PCTestFlag--;
}

void __attribute__((address(PCTestAddr3))) CLASSB_CPUPCTestFunction3(void) 
{
    PCTestFlag--;
}


void __attribute__((address(PCTestAddr1-64))) CLASSB_PCTrap1 (void)
{
    return;
}

void __attribute__((address(PCTestAddr2-64))) CLASSB_PCTrap2 (void)
{
    return;
}

void __attribute__((address(PCTestAddr3-64))) CLASSB_PCTrap3 (void)
{
    return;
}
