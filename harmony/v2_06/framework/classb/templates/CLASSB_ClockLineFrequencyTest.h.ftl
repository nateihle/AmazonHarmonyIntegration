/*******************************************************************************
  Function:
    CLASSBRESULT CLASSB_ClockLineFreqTest(unsigned int clockFrequency, unsigned int lineFrequency, uint8_t tolerance)

  Summary:
    The CPU Clock Line Test is one of the tests that check
    the reliability of the system clock.
    It implements the independent time slot monitoring
    H.2.18.10.4 as defined by the IEC 60730 standard.
    

  Description:
    The CPU Clock Line Test verifies that the system clock is within specified limits.
    An external frequency applied on Input Capture 1 pin (IC1) is used as the reference clock.
    The hardware Timer2 that runs on the system Peripheral Bus (PB) clock is used to
    monitor the CPU clock and Peripheral Bus divider.

    
    The test performs the following major steps:
        1. The IC1 input is used as the independent clock source/reference clock
           source to capture the hardware Timer2.
           An external reference frequency, usually the line frequency, has to be applied
           to the IC1 input pin.

        2. The Input Capture 1 is configured as follows:
            - Timer2 is selected as IC1 time base
            - Capture is performed on every rising edge
            - Capture done event is generated on every second capture.

        3. The hardware Timer2 pre-scaler is calculated (based on the input reference
           frequency and the current PB frequency) as being the smallest divider possible
           such that the 16 bit Timer2 does not overflow within a period time of the 
           input reference signal.
           This way, the best resolution is achieved for the given conditions.
           If no valid pre-scaler value can be obtained an error value is returned.
           
        4. The IC1 performs the capture on every rising edge of the input reference frequency.
           For period measurement, the capture done event is generated after the IC1 module
           takes two time stamps i.e. after every period of the input reference (20 ms if
           reference frequency is 50 Hz, 16.66ms if the reference frequency is 60 Hz).
       
        5. Once the capture done event is signalled, the 2 Timer2 captured readings are extracted
           and the number of elapsed PB clocks is calculated as being the difference between
           the two readings.
           If this value crosses the defined boundary limits the function returns an appropriate
           error value, specifying which exactly limit (upper/lower) was violated.
           

       Calculation example 1:
           System Clock     = 80 MHz
           PB Clock         = 80 MHz (PB divider =1:1)
           Input Reference  = 50 Hz

           T2 Min Divider = floor(PBclk/(65536*RefClk))+1 = 25
           Actual T2 Divider = 32.
           The number of cycles counted in the Reference clock period is  = (80,000,000/32)/50 = 50,000.

           
       Calculation example 2:
           System Clock     = 80 MHz
           PB Clock         = 10 MHz (PB divider =1:8)
           Input Reference  = 60 Hz

           T2 Min Divider = floor(PBclk/(65536*RefClk))+1 = 3
           Actual T2 Divider = 4.
           The number of cycles counted in the Reference clock period is  = (10,000,000/4)/60 = 41,666.

           
  Precondition:
    None.

  Parameters:
    clockFrequency      - the current system running frequency, Hz

    lineFrequency       - the frequency of the reference applied to the IC1 input pin, Hz.
                          Usual values are 50/60 Hz 

    tolerance           - the tolerance level of the system oscillator as a percentage (0 - 100).
    
  Returns:
    Result identifying the pass/fail status of the test:
      CLASSB_TEST_PASS      - The test passed. The monitored CPU clock is within the requested limits.

      CLASSB_TEST_FAIL      - The test failed. The monitored CPU clock is greater than the specified upper limit;  
	                          The monitored CPU clock is less than the specified lower limit; 
							  The frequency of the provided reference was too low and could not be used. 
         
      
  Example:
    <code>
    int testRes=CLASSB_ClockLineFreqTest(80000000, 60, 80000, 100000);
    if(testRes==CLASSB_TEST_PASS)
    {
        // process test success
    }
    else
    {
        // process CPU clock high failure
    }
    </code>

  Remarks:


    The test uses the hardware Input Capture 1 module.
    It initializes the module as needed and, after the test is done,
    it shuts it off.
    The previous state of IC1 is not preserved/restored.
  
    The test uses the hardware Timer2. It initializes the timer as needed and,
    after the test is done, shuts off the timer.
    The previous state of Timer1 is not preserved/restored.

    The value of the PB frequency which is used as input by the Timer2 is derived from the
    system CPU clock by dividing it with the PB divider.
    The test does not change the value of the PB divider, it uses the current value.

    The interrupts should be disabled when executing this test as the time spent in
    ISR's affects the accurate timing of this routine.
    
    The frequency of signal used as a reference on IC1 input should normally be the line frequency.
    However, any frequency can be used as long as a a valid Timer2 divider can be obtained
    (see the example calculation below for the 16 bit Timer2).
    If the reference frequency is too low, a valid divider for the Timer 2 won't be possible
    (the maximum divider for Timer2 is 256).
    You can go to a greater PB divider in this case.
    If the selected reference frequency is too high the number of Timer2 captured counts
    will be too small and the measurement won't have enough resolution. 
        
  *****************************************************************************/
CLASSBRESULT CLASSB_ClockLineFreqTest(unsigned int clockFrequency, unsigned int lineFrequency, unsigned int tolerance);
