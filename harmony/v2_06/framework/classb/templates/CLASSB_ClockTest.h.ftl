// *****************************************************************************
// CPU Clock Test Definitions

/* CPU Clock Test Secondary Oscillator (SOSC) reference frequency 

  Summary:
    Defines the reference frequency that is applied to the SOSC input.

  Description:
    This definition is used for internal calculation in the
    CLASSB_ClockTest() function.
    Normally this input is driven by a 32.768 KHz xtal.
    You can adjust this value if the need occurs (temperature drifts, etc).
*/
#define     CLOCK_TEST_TIMER_CLOCK      ${CONFIG_CLASSB_CLOCK_TEST_TIMER_CLOCK}
<#if CONFIG_CLASSB_CLOCK_TEST_REFERENCE_FREQUENCY_USE_STRING == true >
#define     CLOCK_TEST_REFERENCE_FREQ   ${CONFIG_CLASSB_CLOCK_TEST_REFERENCE_FREQUENCY_STRING}
<#else>
#define     CLOCK_TEST_REFERENCE_FREQ   ${CONFIG_CLASSB_CLOCK_TEST_REFERENCE_FREQUENCY}
</#if>
#define     CLOCK_TEST_TIMER            ${CONFIG_CLASSB_CLOCK_TEST_TIMER}
#define     CLOCK_TEST_INTERRUPT_SOURCE INT_SOURCE_TIMER_${CONFIG_CLASSB_CLOCK_TEST_TIMER_NUMBER}
/*******************************************************************************
  Function:
    CLASSBRESULT CLASSB_ClockTest(unsigned int sysClk, int nMs, unsigned int hiClkErr, unsigned int loClkErr )

  Summary:
    The CPU Clock test is one of the tests that check
    the reliability of the system clock.
    It implements the independent time slot monitoring
    H.2.18.10.1 as defined by the IEC 60730 standard.

  Description:
    The CPU Clock test verifies that the system clock is within specified limits.
    The secondary oscillator (SOSC) is used as the reference clock.
    The CPU Core Timer that runs on the CPU system clock is monitored.

    
    The test performs the following major steps:
        1. The LP SOSC is used as the independent clock source/reference clock
           source connected to hardware Timer1.
        2. The CPU Core Timer monitored in this measurement is incremented every other CPU system clock.
           Usually the system runs on the Primary oscillator with PLL as the clock source to the CPU.
           However, any clock source except the SOSC itself which is used as a reference is valid for this test.
        3. Timer1 is configured to time out after the specified interval of time elapsed (e.g. 10 ms).
        4. The content of the Core Timer is saved at the beginning of the measurement, once the Timer 1
           is started.
        5. When the hardware Timer1 times out another reading of the Core Timer is taken and the difference
           is made with the start value.
           This difference value represents the number of CPU clock cycles counted by the Core Timer during
           the SOSC period of time.
        6. If this value crosses the defined boundary limits the function returns an appropriate
           error value, specifying which exactly limit (upper/lower) was violated.
           
                                                                             
  Precondition:
    None.

  Parameters:
    clockFrequency      - the current system running frequency, Hz

    referenceFrequency  - the frequency of the reference applied to the SOSC pin, Hz.

    testLengthMsec      - number of milliseconds to be used for the CPU Clock monitoring.
                          1 <= testLengthMsec <= 1000 

    tolerance           - the tolerance level of the system oscillator as a percentage (0 - 100).
    
  Returns:
    Result identifying the pass/fail status of the test:
      CLASSB_TEST_PASS    - The test passed. The monitored CPU clock is within the requested limits.

      CLASSB_TEST_FAIL      - The test failed. The monitored CPU clock is greater than the specified upper limit;  
	                          The monitored CPU clock is less than the specified lower limit.
  
  Example:
    <code>
    int testRes=CLASSB_ClockTest(80000000, 100, 80000, 100000);
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

    The test uses the hardware Timer1. It initializes the timer as needed and,
    after the test is done, shuts off the timer.
    The previous state of Timer1 is not preserved/restored.

    The test assumes that the Core Timer is enabled and linearly counting up as it
    should do during normal system operation.
    If your code specifically disables the Core Timer, it should enable it before
    this test is called.
    If the value in the Core Timer is updated/changed as part of an ISR, this ISR should
    be disabled.

    The interrupts should be disabled when executing this test as the time spent in
    ISR's affects the accurate timing of this routine.
    
    The value of the CPU clock monitoring time, nMs, is limited because of the use
    of the hardware Timer1 which is a 16 bit timer.
    Therefore, the value loaded into this Timer1 register should not exceed 2^16-1.
    

  *****************************************************************************/
CLASSBRESULT CLASSB_ClockTest(unsigned int clockFrequency, unsigned int referenceFrequency, unsigned int testLengthMsec, unsigned int tolerance );
