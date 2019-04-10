/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
/*******************************************************************************
  Class B Library Implementation Test Function

  Summary:
    This function contains the test program for the 
    the Class B Safety Software Library on PIC32MX MCUs.
    
*******************************************************************************/
#include <classb/classb.h>
#include <stdlib.h>


#define     SYSTEM_CLOCK        SYS_CLK_FREQ

typedef struct 
{
    CLASSBRESULT  cpuRegister_TestResult;
    CLASSBRESULT  programCounter_TestResult;
    CLASSBRESULT  checkerboardRam_TestResult;
    CLASSBRESULT  marchCRam_TestResult;
    CLASSBRESULT  marchCMinusRam_TestResult;
    CLASSBRESULT  marchCRamStack_TestResult;
    CLASSBRESULT  marchBRam_TestResult;
    CLASSBRESULT  flash_TestResult;
    CLASSBRESULT  clock_TestResult;
    CLASSBRESULT  clockLine_TestResult;
}ClassB_Test_Flags ;
ClassB_Test_Flags testFlag;


 CLASSBRESULT ClassBDemo(void){

    CLASSBRESULT returnCode = CLASSB_TEST_FAIL;
    
    testFlag.checkerboardRam_TestResult = CLASSB_TEST_FAIL;
    testFlag.cpuRegister_TestResult=CLASSB_TEST_FAIL;
    testFlag.programCounter_TestResult=CLASSB_TEST_FAIL;
    testFlag.checkerboardRam_TestResult=CLASSB_TEST_FAIL;
    testFlag.marchCRam_TestResult=CLASSB_TEST_FAIL;
    testFlag.marchCMinusRam_TestResult=CLASSB_TEST_FAIL;
    testFlag.marchCRamStack_TestResult=CLASSB_TEST_FAIL;
    testFlag.marchBRam_TestResult=CLASSB_TEST_FAIL;
    testFlag.flash_TestResult=CLASSB_TEST_FAIL;
    testFlag.clock_TestResult=CLASSB_TEST_FAIL;
    testFlag.clockLine_TestResult=CLASSB_TEST_FAIL;


    /**********************************************************************************/
    /*                                  CPU REGISTER TEST                             */                              
    /**********************************************************************************/

    testFlag.cpuRegister_TestResult = CLASSB_CPURegistersTest();

    /**********************************************************************************/
    /*                                  PROGRAM COUNTER TEST                          */
    /*  This requires a special linker script (elf32pic32mx.ld) to be added           */
    /*   as part of the project. See the description in SSL_PcTest.h                  */
    /**********************************************************************************/

    testFlag.programCounter_TestResult = CLASSB_CPUPCTest();

    /**********************************************************************************/
    /*                                  RAM TESTS                                     */                              
    /**********************************************************************************/

    // Variables used for RAM Tests
    extern int  _stack[];                   // the address of the stack, as placed by the linker
    extern int  _min_stack_size[];          // size of the stack, as defined in the project
    extern char _sdata_begin[];              // the address of the data segment, as placed by the linker

    unsigned int     *ramTestStartAddress;           // start address for the test 
    unsigned int     ramTestSize;                    // size of the tested area, bytes
    unsigned int     stackTestSize;                    // size of the tested area, bytes

    
    /*************************/
    /* Checker Board RAM test*/
    /*************************/
    // We'll test 1KB chunk at the middle of the RAM
    // Note that this test is not destructive
    // The size of the RAM area to test has to be multiple of 64.
    // It has to NOT overlap the stack space!

    int         ramTotSize;        // total RAM available, without stack
    unsigned int*       ramStartAddress;   // starting RAM address for the test
    unsigned int*       ramEndAddress;     // end address of RAM

    int                 stackTotSize;        // total RAM available, without stack
    unsigned int*       stackStartAddress;   // starting RAM address for the test
    unsigned int*       stackEndAddress;     // end address of RAM

    // The stack is filled from the high memory to the lower memory.  So, the 
    // sp will begin at stackStartAddress.  And, stackStartAddress will be a 
    // larger value than stackEndAddress
    stackStartAddress= (unsigned int*)_stack;
    stackEndAddress= (unsigned int*)(_stack-((unsigned int)_min_stack_size));
    stackTotSize=(unsigned int)stackStartAddress - (unsigned int)stackEndAddress;
    // The available memory is from the start of RAM to the end of the stack.
    ramStartAddress= (unsigned int*)_sdata_begin;
    ramEndAddress= stackEndAddress;
    ramTotSize=(unsigned int)ramEndAddress - (unsigned int)ramStartAddress;

    // FOR THE RAM
    // Choose either 1k or the total size to test whichever is smaller.
    // Find a spot of that size in the middle of the test area.
    ramTestSize = (ramTotSize>0x400)?0x400:ramTotSize;
    ramTestSize&=0xffffffc0;
    ramTestStartAddress =(unsigned int *)(ramStartAddress+(((ramTotSize-ramTestSize)/2)/sizeof(unsigned int)));

    // FOR THE STACK
    // Choose either 1k or the total size to test whichever is smaller.
    // Find a spot of that size in the middle of the test area.
    // The ram test area is used to save the stack, so its size has to be larger.
    stackTestSize = (stackTotSize>0x400)?0x400:stackTotSize;
    stackTestSize = (ramTestSize>stackTestSize)?stackTestSize:(ramTestSize-sizeof(unsigned int));
    stackTestSize&=0xffffffc0;


    // test it
    testFlag.checkerboardRam_TestResult = CLASSB_RAMCheckerBoardTest(ramTestStartAddress, ramTestSize);

    /****************************/
    /*    March B Ram Test      */
    /****************************/
    // We'll test it using the March B test
    // Note that the size of the RAM to test has to be multiple of 4.
    testFlag.marchBRam_TestResult = CLASSB_RAMMarchBTest(ramTestStartAddress, ramTestSize );

    /*************************/
    /*    MarchC RAM tests    */
    /*************************/
    // We'll test it using the March C test
    // Note that the size of the RAM to test has to be multiple of 4.
    testFlag.marchCRam_TestResult = CLASSB_RAMMarchCTest(ramTestStartAddress, ramTestSize );

    /****************************/
    /* MarchC RAM and Stack Test*/
    /****************************/
    // We want to make sure that both the RAM and
    // the stack space are ok.
    // We'll test it using the March C and Stack tests before we use it
    // Note that the size of the RAM to test has to be multiple of 4.
    // Also, the size of the tested RAM area has to be greater than the
    // size of the tested stack area. 
    // This test is destructive for the RAM area but preserves the Stack area.
    testFlag.marchCRamStack_TestResult = CLASSB_RAMMarchCStackTest(ramTestStartAddress, ramTestSize);

    {
        /**********************************************************************************/
        /*                                  FLASH CRC TEST                                */                              
        /**********************************************************************************/
        // This function can be called at startup to generate the Reference checksum.
        // The same function can be called periodically and the generated checksum can be 
        // compared with the reference checksum.
        // If both are the same the "flash_TestResult" status bit can be set. 

        unsigned int flashCrcRef, flashCrc; // reference and current CRC values
        unsigned int crcSeed=0xffff;        // initial CRC register value
                                            // this is the recommended CRC seed
                                            // for checking properly long 0 streams.

        // calculate the CRC16 of the whole program flash (K0)      
        char* flashStartAddress = (char*)0x9d000000;       // fixed start K0 address on PIC32MX devices
        char* flashEndAddress =  flashStartAddress+0x1000;  // size of the flash on this device
                                                                // the BMX register stores the Flash size for this part

        // first we calculate the reference Flash CRC value
        flashCrcRef = CLASSB_CRCFlashTest( flashStartAddress, flashEndAddress, CRC_16_GEN_POLY, crcSeed);

        // at some time later we calculate again the CRC of the Flash 
        flashCrc = CLASSB_CRCFlashTest( flashStartAddress, flashEndAddress, CRC_16_GEN_POLY, crcSeed);

        // make sure that the periodic check is equal to the reference one
        if ( flashCrc==flashCrcRef)
        {
            testFlag.flash_TestResult=CLASSB_TEST_PASS;
            // we are confident that the data programmed in Flash
            // has not been altered in any way 
        }
        else
        { 
            testFlag.flash_TestResult=CLASSB_TEST_FAIL;
        }
    }


    /**********************************************************************************/
    /*        CLOCK  TEST WITH SECONDARY OSCILLATOR AS REFERENCE CLOCK                */
    /*      This test requires that a standard 32.768 KHz crystal is connected        */
    /*                      at the SOSC input.                                        */
    /**********************************************************************************/
    // we'll count for 1 second
    // check the system clock to be within +/- 40%
    testFlag.clock_TestResult=CLASSB_ClockTest(SYSTEM_CLOCK, CLOCK_TEST_REFERENCE_FREQ, 1, 40);

    /**********************************************************************************/
    /*      CLOCK TEST WITH 50Hz LINE FREQUENCY AS REFERENCE CLOCK                    */
    /*      This test requires an 50 Hz external reference frequency to be fed        */
    /*                      to the IC1 input pin.                                     */    
    /**********************************************************************************/

    // we test using 50 Hz reference frequency
    // check the system clock to be within +/- 40%
    testFlag.clockLine_TestResult=CLASSB_ClockLineFreqTest(SYSTEM_CLOCK, 50, 40);

    /* End of Tests */    
    if(testFlag.checkerboardRam_TestResult == CLASSB_TEST_PASS &&
    testFlag.cpuRegister_TestResult == CLASSB_TEST_PASS &&
    testFlag.programCounter_TestResult == CLASSB_TEST_PASS &&
    testFlag.checkerboardRam_TestResult == CLASSB_TEST_PASS &&
    testFlag.marchCRam_TestResult == CLASSB_TEST_PASS &&
    testFlag.marchCMinusRam_TestResult == CLASSB_TEST_PASS &&
    testFlag.marchCRamStack_TestResult == CLASSB_TEST_PASS &&
    testFlag.marchBRam_TestResult == CLASSB_TEST_PASS &&
    testFlag.flash_TestResult == CLASSB_TEST_PASS &&
    testFlag.clock_TestResult == CLASSB_TEST_PASS &&
    testFlag.clockLine_TestResult == CLASSB_TEST_PASS)
    {
        returnCode = CLASSB_TEST_PASS;
    }

    return returnCode;
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            ClassBDemo();
                appData.state = APP_STATE_DONE;
        
            break;
        }

        case APP_STATE_DONE:
        {
            Nop();
            break;
        }
        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
