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
    CLASSBRESULT CLASSB_ClockLineFreqTest(unsigned int clockFrequency, 
                                          unsigned int lineFrequency, 
                                          unsigned int tolerance)

  Summary:
    The CPU Clock Line Test is one of the tests that check
    the reliability of the system clock.
    It implements the independent time slot monitoring
    H.2.18.10.4 as defined by the IEC 60730 standard.

  Remarks:
    Refer to classb.h for usage information.
  *****************************************************************************/
CLASSBRESULT CLASSB_ClockLineFreqTest(unsigned int clockFrequency, unsigned int lineFrequency, unsigned int tolerance)
{


    unsigned int    t2Div, t2DivPwr;
    unsigned int    refCntHiVal, refCntLoVal;
    unsigned short  t1, t2, currCount;
    CLASSBRESULT    testResult;
    int             hiClkErr;
    int             loClkErr; 
    // T2 input clock
    OSC_PB_CLOCK_DIV_TYPE pbClk;
    pbClk = PLIB_OSC_PBClockDivisorGet(OSC_ID_0, OSC_PERIPHERAL_BUS_1);

    // Tolerance is in units of 1/1000 percent.
    hiClkErr = (tolerance * clockFrequency)/100;
    loClkErr = (tolerance * clockFrequency)/100;
    
    // calculate the T2 minimum divider value
    t2Div=pbClk/(65536*lineFrequency)+1;

    // the max divider supported is 256
    if(t2Div>256)
    {   
        return CLASSB_TEST_FAIL; // we need a too great divider, the reference is too low
    }

    t2DivPwr=0;
	while(t2Div>(1<<t2DivPwr))
	{
		t2DivPwr++;
	}

    // adjust   
    if(t2DivPwr>6)
    {
        t2DivPwr=7;   // 128 divider not supported on T2
        t2Div=256;
    }
    else
    {
        t2Div=1<<t2DivPwr;
    }
    Nop();
    // open the T2 channel
    PLIB_TMR_PrescaleSelect(TMR_ID_1, TMR_PRESCALE_VALUE_1);
    PLIB_TMR_ClockSourceSelect(TMR_ID_1, TMR_CLOCK_SOURCE_EXTERNAL_INPUT_PIN );
    PLIB_TMR_Start(TMR_ID_1);

    
    // calculate the imposed limits
    // Sys Clk ->PB Divider -> T2 divider -> in reg clock time    
    refCntHiVal=(((clockFrequency+hiClkErr)/PLIB_OSC_PBClockDivisorGet(OSC_ID_0, OSC_PERIPHERAL_BUS_1))/lineFrequency)/t2Div;
    refCntLoVal=(((clockFrequency-loClkErr)/PLIB_OSC_PBClockDivisorGet(OSC_ID_0, OSC_PERIPHERAL_BUS_1))/lineFrequency)/t2Div;


    // init the input capture
    #ifdef PLIB_IC_ExistsAlternateClock
        PLIB_IC_AlternateClockEnable( IC_ID_1 );
    #endif
    PLIB_IC_TimerSelect(IC_ID_1, IC_TIMER_TMR2);
    PLIB_IC_EventsPerInterruptSelect(IC_ID_1, IC_INTERRUPT_ON_EVERY_2ND_CAPTURE_EVENT );
    PLIB_IC_ModeSelect(IC_ID_1, IC_INPUT_CAPTURE_RISING_EDGE_MODE);
    PLIB_IC_Enable(IC_ID_1);
    while(!PLIB_IC_BufferIsEmpty(IC_ID_1))
    {
      PLIB_IC_Buffer16BitGet(IC_ID_1);
    };
    
  // sync first
   PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_INPUT_CAPTURE_1);
   
   
   while(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_1));
   PLIB_IC_Buffer16BitGet(IC_ID_1);
   PLIB_IC_Buffer16BitGet(IC_ID_1);

  // wait for two consecutive captures -> interrupt flag set
   PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_INPUT_CAPTURE_1);
   while(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_INPUT_CAPTURE_1));

   // read the two captured values from FIFO
   t1 = PLIB_IC_Buffer16BitGet(IC_ID_1);
   t2 = PLIB_IC_Buffer16BitGet(IC_ID_1);
   
   PLIB_IC_Disable(IC_ID_1);
   PLIB_TMR_Stop(TMR_ID_1);
     
    // check the captured counter value       
    currCount = t2-t1; 
    if((unsigned int)currCount > refCntHiVal)
    {
        testResult=CLASSB_TEST_FAIL;
    }
    else if((unsigned int)currCount < refCntLoVal )
    {
        testResult=CLASSB_TEST_FAIL;
    }
    else
    {
        testResult=CLASSB_TEST_PASS;
    }

    
    return testResult;
   
}

