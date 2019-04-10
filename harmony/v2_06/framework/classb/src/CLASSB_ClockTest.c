/*******************************************************************************
  Class B Library Implementation File

  Summary:
    This file contains the implementation for 
    the Class B Safety Software Library CPU Clock test
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

/*******************************************************************************
  Function:
    CLASSBRESULT CLASSB_ClockTest(unsigned int clockFrequency, 
                                  unsigned int referenceFrequency, 
                                  unsigned int testLengthMsec, 
                                  unsigned int tolerance )

  Summary:
    The CPU Clock test is one of the tests that check the reliability of the 
    system clock.  It implements the independent time slot monitoring 
    H.2.18.10.4 as defined by the IEC 60730 standard.
  
  Remarks:
    Refer to classb.h for usage information.

  *****************************************************************************/

CLASSBRESULT CLASSB_ClockTest(unsigned int clockFrequency, unsigned int referenceFrequency, unsigned int testLengthMsec, unsigned int tolerance )
{

    uint32_t        totalCount;
    uint32_t        tStart;
    uint32_t        tEnd;
    uint32_t        sysCntLoVal;
    uint32_t        sysCntHiVal;
    CLASSBRESULT    testResult=CLASSB_TEST_FAIL;
    uint64_t        sysCountsPerTest;
    uint64_t        refCountsPerTest;
    
    // Calculate the number of counts in the system frequency over the test time.
    // The system clock counts twice for every CPU Clock count.
    sysCountsPerTest=((uint64_t)clockFrequency * (uint64_t)testLengthMsec)/(uint64_t)2000;
    // This is the number of counts on the Timer to wait.
    refCountsPerTest=((uint64_t)testLengthMsec * (uint64_t)referenceFrequency)/(uint64_t)1000;   
    //  Calculate the high number of counts based on the target counts and tolerance.
    sysCntHiVal=sysCountsPerTest + ((sysCountsPerTest * (uint64_t)tolerance)/(uint64_t)100);     // high count limit
    //  Calculate the low number of counts based on the target counts and tolerance.
    sysCntLoVal=sysCountsPerTest - ((sysCountsPerTest * (uint64_t)tolerance)/(uint64_t)100);     // high count limit

    // enable timer1 using the Low Power RC (LPRC) Oscillator
    PLIB_TMR_Mode16BitEnable(CLOCK_TEST_TIMER);
    PLIB_TMR_Period16BitSet(CLOCK_TEST_TIMER, (uint16_t)refCountsPerTest);
    PLIB_TMR_PrescaleSelect(CLOCK_TEST_TIMER, TMR_PRESCALE_VALUE_1);
    PLIB_TMR_ClockSourceSelect(CLOCK_TEST_TIMER, CLOCK_TEST_TIMER_CLOCK);
    PLIB_TMR_Start(CLOCK_TEST_TIMER);
   
    // sync first
    PLIB_INT_SourceFlagClear(INT_ID_0,CLOCK_TEST_INTERRUPT_SOURCE);
    while (!PLIB_INT_SourceFlagGet(INT_ID_0,CLOCK_TEST_INTERRUPT_SOURCE));
    
    // start counting
    asm volatile("mfc0   %0, $9" : "=r"(tStart));
    PLIB_INT_SourceFlagClear(INT_ID_0,CLOCK_TEST_INTERRUPT_SOURCE);
    while (!PLIB_INT_SourceFlagGet(INT_ID_0,CLOCK_TEST_INTERRUPT_SOURCE));
    // stop counting
    asm volatile("mfc0   %0, $9" : "=r"(tEnd));    
    totalCount = tEnd - tStart;
    PLIB_TMR_Stop(CLOCK_TEST_TIMER);
    // check the counter value       
    if(totalCount < sysCntHiVal && totalCount > sysCntLoVal)
    {
        testResult=CLASSB_TEST_PASS;
    }
    return testResult;
}



