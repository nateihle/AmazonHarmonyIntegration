/*******************************************************************************
  Class B Library Interface Header

  Company:
    Microchip Technology Inc.

  File Name:
    classb.h

  Summary:
    Class B interface header for definitions common to the Class B Library.

  Description:
    This header file contains the function prototypes and definitions of
    the data types and constants that make up the interface to the Class B
    Library for all families of Microchip microcontrollers. The definitions in
    this file are common to the Class B Library.
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
#ifndef _CLASSB_H_
#define _CLASSB_H_
/****************************************************************************
  Enumeration:
    CLASSBRESULT

  Description:
    This enumeration is used by the class B test functions to return the results:

    CLASSB_TEST_PASS    - the test finished successfully,
    CLASSB_TEST_FAIL    - the test is failed,
    CLASSB_TEST_TIMEOUT - the test is failed because a timeout was detected,
    CLASSB_TEST_INPROGRESS - the test is still in progress.

  ***************************************************************************/
typedef enum
{
  CLASSB_TEST_PASS = 0,
  CLASSB_TEST_FAIL,
  CLASSB_TEST_TIMEOUT,
  CLASSB_TEST_INPROGRESS
} CLASSBRESULT;
#include <peripheral/peripheral.h>
#include <xc.h>
#include <stdint.h>

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

#define     CLOCK_TEST_REFERENCE_FREQ   32768
#define     CLOCK_TEST_TIMER            TMR_ID_1
#define     CLOCK_TEST_INTERRUPT_SOURCE INT_SOURCE_TIMER_1
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
    
    The SOSC is used as a reference.
    Use the CLOCK_TEST_SOSC_FREQ for adjustments to this value, if needed.
    
    The value of the CPU clock monitoring time, nMs, is limited because of the use
    of the hardware Timer1 which is a 16 bit timer.
    Therefore, the value loaded into this Timer1 register should not exceed 2^16-1.
    

  *****************************************************************************/
CLASSBRESULT CLASSB_ClockTest(unsigned int clockFrequency, unsigned int referenceFrequency, unsigned int testLengthMsec, unsigned int tolerance );
/*******************************************************************************
  Function:
    int CLASSB_CPUPCTest (void)

  Summary:
    The Program Counter test implements one of the functional tests
    H.2.16.5 as defined by the IEC 60730 standard.
    

  Description:
    The Program Counter test is a functional test of the PC.
    It checks that the PC register is not stuck and it properly holds the address
    of the next instruction to be executed.
    
    The tests performs the following major tasks:
        1. The Program Counter test invokes functions that are located in memory at different addresses.
        2. That all of the functions is executed is verified.
                                                                             
  Precondition:
    None.

  Parameters:
    None.

  Returns:
    Result identifying the pass/fail status of the test:
        CLASSB_TEST_PASS    - The test passed. The PC register holds the correct address.

        CLASSB_TEST_FAIL    - The test failed. The PC register has been detected to hold an incorrect address. 

  Example:
    <code>
    int testRes=CLASSB_CPUPCTest();
    if(testRes==PC_TEST_PASS)
    {
        // process test success
    }
    else
    {
        // process tests failure
    }
    </code>

  Remarks:
    The test uses 3 different functions:
        CLASSB_CPUPCTestFunction1()
        CLASSB_CPUPCTestFunction2()          
        CLASSB_CPUPCTestFunction3()          
  *****************************************************************************/
CLASSBRESULT CLASSB_CPUPCTest(void);

/*******************************************************************************
  Function:
    int CLASSB_CPURegistersTest ( void )

  Summary:
    The CPU Register test implements the functional test
    H.2.16.5 as defined by the IEC 60730 standard.
    

  Description:
    This routine detects stuck-at Faults in the CPU registers.
    This ensures that the bits in the registers are not stuck at
    a value ?0? or ?1?.

  Precondition:
    None.

  Parameters:
    None.
    
  Returns:
    Result identifying the pass/fail status of the test:
      CLASSB_TEST_PASS    - The test passed. CPU registers have not been detected to have stuck bits.

      CLASSB_TEST_FAIL    - The test failed. Some CPU register(s) has been detected to have stuck bits. 

  Example:
    <code>
    int testRes=CLASSB_CPURegistersTest();
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
    This is a non-destructive test.
    
    Interrupts should be disabled when calling this test function.

  *****************************************************************************/
CLASSBRESULT CLASSB_CPURegistersTest(void);

// *****************************************************************************
// Flash CRC16 Test Generator Polynomials
/*
  Description:
    The value of the generator polynomial is used as an input parameter for the
    CLASSB_CRCFlashTest() function.
    It specifies what polynomial to be used for the CRC calculation.

    Following is a list of some of the most commonly used 16 bit Generator Polynomials
    that can be used.
    Any other polynomial that has the required fault detection capabilities can be used.
*/    
#define CRC_08_GEN_POLY             0x07     /* x^8 + x^2 + x + 1*/    
#define CRC_16_GEN_POLY             0x8005   /* x^16 + x^15 + x^2 + 1*/    
#define CRC_CCITT_GEN_POLY          0x1021   /* x^16 + x^12 + x^5 + 1*/    
#define CRC_32_GEN_POLY             0x04C11DB7  /* x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1*/    
// definitions
#define     FLASH_CRC8_MASK     ((1L<<8)-1)    		    // mask to retain the useful CRC result for 8 bit polynomial
#define     FLASH_CRC8_MSB      (1L<<(8-1))			    // mask to obtain the MSb-1, transport to the MSb.
#define     FLASH_CRC16_MASK    ((1L<<16)-1)    		// mask to retain the useful CRC result for 16 bit polynomial
#define     FLASH_CRC16_MSB     (1L<<(16-1))			// mask to obtain the MSb-1, transport to the MSb.
#define     FLASH_CRC32_MASK    ((1L<<32)-1)    		// mask to retain the useful CRC result for 32 bit polynomial
#define     FLASH_CRC32_MSB     (1L<<(32-1))			// mask to obtain the MSb-1, transport to the MSb.

/*********************************************************************
 * Function:        unsigned int Flash_CRC16(unsigned char* pBuff, 
                                             unsigned int crcPoly, 
                                             unsigned int crcReg, 
                                             unsigned int bSize)
 *
 * PreCondition:    pBuff valid pointer
 * 
 * Input:           - pBuff:    buffer to calculate CRC over
 *                  - crcPoly:  the generator polynomial to be used
 *                  - crcReg:   initial value of the CRC LFSR (seed)
 * 					- bSize:    buffer size, bytes
 * 
 * Output:          value of the CRC
 * 
 * Side Effects:    None
 * 
 * Overview:        Shifts bytes through the CRC shift register.
 * 
 * Note:            Simple (and slow) CRC16 calculation directly based
 *                  on the hardware LFSR implementation.
 *                  
 ********************************************************************/
unsigned int Flash_CRC16(unsigned char* pBuff, unsigned int crcPoly, unsigned int crcReg, unsigned int bSize);


/*******************************************************************************
  Function:
    unsigned int CLASSB_CRCFlashTest(char* startAddress, char* endAddress, unsigned int crcPoly, unsigned int crcSeed)

  Summary:
    The Flash CRC16 test implements the periodic modified checksum
    H.2.19.3.1 as defined by the IEC 60730 standard.

  Description:
    This routine  detects the single bit Faults in the invariable memory.
    The invariable memory in a system, such as Flash and EEPROM memory,
    contains data that is not intended to vary during the program execution.

    The test calculates the 16 bit CRC of the supplied memory area
    using the standard LFSR (Linear Feedback Shift Register) implementation.
    It calculates over the memory area between the startAddress and endAddress and returns the CRC Value.
    The 16 bit CRC is calculated using the supplied generator polynomial and initial seed.
    Different generator polynomials can be used as indicated above.
                                                                             
  Precondition:
    None.

  Parameters:
    startAddress    - start Address of the memory area to start CRC calculation from

    endAddress      - final address for which the CRC is calculated

    crcPoly         - the generator polynomial to be used.
                      One of the standard supplied polynomials can be used
                      as well as other user defined ones.

    crcSeed         - the initial value in the CRC LFSR.
                      The usual recommended value is 0xffff.
                          
  Returns:
    The value of the calculated CRC over the specified memory area.

  Example:
    <code>
    unsigned int crcRes=CLASSB_CRCFlashTest(startAddress, endAddress, CRC_16_GEN_POLY, 0xffff);
    if(crcRes==prevCalculatedCrc)
    {
        // process test success
    }
    else
    {
        // process tests failure: the CRC of the memory area has changed.
    }
    </code>

  Remarks:
    This is a non-destructive memory test.
    
    The startAddress and endAdress over which the CRC value is calculated are
    PIC32 variant and application dependent. They are run-time parameters.
    
  *****************************************************************************/
unsigned int CLASSB_CRCFlashTest(char* startAddress, char* endAddress, unsigned int crcPoly, unsigned int crcSeed);
#define CLASSB_RAM_TEST_CYCLE_SIZE 64
/*******************************************************************************
  Function:
    int CLASSB_RAMCheckerBoardTest (int* ramStartAddress, int ramSize)

  Summary:
    The RAM Checker Board test implements one of the functional tests
    H.2.19.6 as defined by the IEC 60730 standard.
    

  Description:
    This routine detects single bit Faults in the variable memory.
    This ensures that the bits in the tested RAM are not stuck at
    a value ?0? or ?1?.

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
/*******************************************************************************
  Function:
    int CLASSB_RAMMarchBTest (int* ramStartAddress, int ramSize)

  Summary:
    The RAM March B test is one of the Variable Memory tests
    that implements the Periodic Static Memory test
    H.2.19.6 as defined by the IEC 60730 standard.
    

  Description:
    This test is a complete and non redundant test capable of detecting
    stuck-at, linked idempotent coupling or Inversion Coupling faults.
    This test is of complexity 17n( Where n is the number of bits tested). 
    The test uses word (32-bit) accesses.
    The address must be properly word aligned and the length of the
    area to be tested must be an integral multiple of the data width access.
                                                                             
  Precondition:
    None.

  Parameters:
    ramStartAddress     - start Address from which the March B test is to be performed
                          Must be properly 32 bit aligned.

    ramSize             - number of consecutive byte locations for which the test is to be performed
                          The size must be a number multiple of 4.
    
  Returns:
    Result identifying the pass/fail status of the test:
      CLASSB_TEST_PASS    - The test passed. RAM area tested has not been detected to have faults. 

      CLASSB_TEST_FAIL    - The test failed. Some RAM area location has been detected to have faults. 

  Example:
    <code>
    int testRes=CLASSB_RAMMarchBTest(startAddress, size);
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
    This is a destructive memory test.
    Either exclude from this test RAM areas that have to be preserved
    or save/restore the memory area before/after running the test
    or run the test at system startup before the memory and the
    run time library is initialized (stack needs to be initialized though).
    
    At least 100 bytes should be available for stack for executing the March B test.
    The tested RAM area must not overlap the stack.
    
    Other statically allocated resources,  such as the MPLAB ICD/Real ICE
    allocated RAM buffers should be excluded from this test.    
    
    The Start Address from which the March B test is to be performed is
    PIC32 variant and application dependent. It is a run-time parameter.
    
    The routine accesses one 4 byte RAM word at a time.        
    
    Refer to the AN1229 for details regarding the CLASSB_RAMMarchBTest()
    and the Class B Software Library.
  *****************************************************************************/
CLASSBRESULT CLASSB_RAMMarchBTest(unsigned int* ramStartAddress, unsigned int ramSize);
/*******************************************************************************
  Function:
    CLASSBRESULT CLASSB_RAMMarchCStackTest (unsigned int* ramStartAddress, 
                                            unsigned int ramSize)

  Summary:
    The RAM March C test is one of the Variable Memory tests
    that implements the Periodic Static Memory test
    H.2.19.6 as defined by the IEC 60730 standard.

  Description:
    This test is a complete and non redundant test capable of detecting
    stuck-at, addressing, transition and coupling faults.
    This test is of complexity 11n( Where n is the number of bits tested). 
    The test uses word (32-bit) accesses.
    The addresses must be properly word aligned and the lengths of the
    areas to be tested must be an integral multiple of the data width access.
                                                                             
  Precondition:
    None.

  Parameters:
    ramStartAddress     - start Address of RAM area to be used as temporary stack.
						  The stack will be copied into this memory, and the stack pointer 
						  redirected here.
                          Has to NOT overlap the Stack area!
                          Must be properly 32 bit aligned.

    ramSize             - number of consecutive byte locations for which the test is to be performed
						  The portion of the stack to be tested will be from the Stack Start (Highest address)
						  to this address minus ramSize.
						  This value also defines the amount of memory available as a temporary stack.
                          The size must be a number multiple of 4.
                          The size of the RAM area tested has to be >= 128 bytes.
                          
    
  Returns:
    Result identifying the pass/fail status of the test:
      CLASSB_TEST_PASS          - The test passed. RAM and Stack area tested have not been detected to have faults. 

      CLASSB_TEST_FAIL          - The test failed. Either some RAM or Stack area location has been detected to have faults, 

      
      
  Example:
    <code>
    int testRes=CLASSB_RAMMarchCStackTest(startAddress, size);
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

    This function is just a helper to March C test both a regular RAM area and a Stack area.
    First the RAM area is tested using the standard March C test.
    If the test succeeded, the requested Stack area is saved/copied in the RAM area
    that has just been tested and then the March C test is run over the Stack area
    as if it were a regular RAM area.
    The saved Stack area is restored and the result of the test is returned to the user.

    The RAM and Stack areas have to NOT overlap!
    The Stack grows downwards so the tested area is:
    [stackTopAddress-stackSize, stackTopAddress]
    Also the size of the Stack area to be tested has to be less than the size of the RAM area.
 
    The processor SP register is changed to the point to the RAM area while the Stack area is tested.
    Since running the MARC C test requires at least 128 bytes of stack, the RAM area size should be at least
    128 bytes long.
    Once the Stack area is tested, the SP register is restored.
    
    This is a destructive memory test.
    Either exclude from this test RAM areas that have to be preserved
    or save/restore the memory area before/after running the test
    or run the test at system startup before the memory and the
    run time library is initialized (stack needs to be initialized though).
    
    At least 128 bytes should be available for stack for executing the March C test.
    The tested RAM area must not overlap the stack.
    
    Other statically allocated resources,  such as the MPLAB ICD/Real ICE
    allocated RAM buffers should be excluded from this test.    
    
    The Start Address from which the March C test is to be performed is
    PIC32 variant and application dependent. It is a run-time parameter.
    
    The routine accesses one 4 byte RAM word at a time.        
    
  *****************************************************************************/
CLASSBRESULT CLASSB_RAMMarchCStackTest(unsigned int* ramStartAddress, unsigned int ramSize);

#define USE_MARCHC_MINUS

/*******************************************************************************
  Function:
    CLASSBRESULT CLASSB_RAMMarchCTest(unsigned int* ramStartAddress, 
                                      unsigned int ramSize)

  Summary:
    The RAM March C test is one of the Variable Memory tests
    that implements the Periodic Static Memory test
    H.2.19.6 as defined by the IEC 60730 standard.
    

  Description:
    This test is a complete and non redundant test capable of detecting
    stuck-at, addressing, transition and coupling faults.
    This test is of complexity 11n( Where n is the number of bits tested). 
    The test uses word (32-bit) accesses.
    The address must be properly word aligned and the length of the
    area to be tested must be an integral multiple of the data width access.
                                                                             
  Precondition:
    None.

  Parameters:
    ramStartAddress     - start Address from which the March C test is to be performed
                          Must be properly 32 bit aligned.

    ramSize             - number of consecutive byte locations for which the test is to be performed
                          The size must be a number multiple of 4.
    
  Returns:
    Result identifying the pass/fail status of the test:
      CLASSB_TEST_PASS    - The test passed. RAM area tested has not been detected to have faults. 

      CLASSB_TEST_FAIL    - The test failed. Some RAM area location has been detected to have faults. 

  Example:
    <code>
    int testRes=CLASSB_RAMMarchCTest(startAddress, size);
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
    This is a destructive memory test.
    Either exclude from this test RAM areas that have to be preserved
    or save/restore the memory area before/after running the test
    or run the test at system startup before the memory and the
    run time library is initialized (stack needs to be initialized though).
    
    At least 100 bytes should be available for stack for executing the March C test.
    The tested RAM area must not overlap the stack.
    
    Other statically allocated resources,  such as the MPLAB ICD/Real ICE
    allocated RAM buffers should be excluded from this test.    
    
    The Start Address from which the March C test is to be performed is
    PIC32 variant and application dependent. It is a run-time parameter.
    
    The routine accesses one 4 byte RAM word at a time.        
    
  *****************************************************************************/
CLASSBRESULT CLASSB_RAMMarchCTest(unsigned int* ramStartAddress, unsigned int ramSize);
#endif