/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    mc_app.c

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

#include "mc_app.h"

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

MC_APP_DATA mc_appData;

// Variables used for RAM Tests
extern int  			_stack[];                   // Stack address (placed by the linker)
extern int  			_min_stack_size[];          // Stack size
extern char 			_sdata_begin[];             // Data segment address (placed by the linker)

unsigned int     		*ramTestStartAddress;       // Test area start address 
unsigned int     		ramTestSize;                // Test area size in bytes
unsigned int     		stackTestSize;              // Stack test size in bytes

int         			ramTotSize;        			// Total RAM size (without stack)
unsigned int*       	ramStartAddress;   			// Test RAM start address
unsigned int*       	ramEndAddress;     			// Test RAM end address

int                 	stackTotSize;        		// Total Stack size
unsigned int*       	stackStartAddress;   		// Test stack start address
unsigned int*       	stackEndAddress;     		// Test stack end address
unsigned int 			flashCrcRef, flashCrc; 		// Current and reference CRC values

unsigned int 			crcSeed=0xffff;        		// Initial CRC register value
                                            

/* MC Core Variables */
#define 	SQRT3_BY2     			(float)0.866025403788
#define 	ONE_BY_SQRT3     		(float)0.5773502691

tPIParm     						PIParmQ;        						/* Parameters for Q axis Currrent PI Controller */
tPIParm     						PIParmD;        						/* Parameters for D axis Currrent PI Controller */
tPIParm     						PIParmQref;     						/* Parameters for Speed PI Controller */
tParkParm							ParkParm;   
tSincosParm							SincosParm;
tSVGenParm 							SVGenParm;
tCtrlParm 						    CtrlParm;

float 								T1, T2, Ta, Tb, Tc;
float 								dPWM1, dPWM2, dPWM3;
float 								Startup_Ramp_Angle_Rads_Per_Sec = 0; 	/* ramp angle variable for initial ramp */
unsigned int 						Startup_Lock_Count = 0; 				/* lock variable for initial ramp */ 

short        						potReading;
short                               phaseCurrentA;
short                               phaseCurrentB;

short 								POLARITY;
float								DoControl_Temp1, DoControl_Temp2;


//Cumulative value is initalized to (estimated offset value * 2^MOVING_AVG_WINDOW_SIZE) which helps in expedited tracking of offset value without
//waiting for all the 2^MOVING_AVG_WINDOW_SIZE samples.

unsigned int cumulative_sum_phaseA = (CURRENT_OFFSET_INIT << MOVING_AVG_WINDOW_SIZE); 
unsigned int cumulative_sum_phaseB = (CURRENT_OFFSET_INIT << MOVING_AVG_WINDOW_SIZE);
int moving_average_phaseA = 0;
int moving_average_phaseB = 0;

/* MC Estimator Variables */
tEstimParm 							EstimParm;
tMotorEstimParm 					MotorEstimParm;
#define 							DECIMATE_NOMINAL_SPEED      ((NOMINAL_SPEED_RPM *(M_PI/30))*NOPOLESPAIRS/10)                                            

const float sineTable[TABLE_SIZE] = 
{
0,
0.024541,
0.049068,
0.073565,
0.098017,
0.122411,
0.14673,
0.170962,
0.19509,
0.219101,
0.24298,
0.266713,
0.290285,
0.313682,
0.33689,
0.359895,
0.382683,
0.405241,
0.427555,
0.449611,
0.471397,
0.492898,
0.514103,
0.534998,
0.55557,
0.575808,
0.595699,
0.615232,
0.634393,
0.653173,
0.671559,
0.689541,
0.707107,
0.724247,
0.740951,
0.757209,
0.77301,
0.788346,
0.803208,
0.817585,
0.83147,
0.844854,
0.857729,
0.870087,
0.881921,
0.893224,
0.903989,
0.91421,
0.92388,
0.932993,
0.941544,
0.949528,
0.95694,
0.963776,
0.970031,
0.975702,
0.980785,
0.985278,
0.989177,
0.99248,
0.995185,
0.99729,
0.998795,
0.999699,
1,
0.999699,
0.998795,
0.99729,
0.995185,
0.99248,
0.989177,
0.985278,
0.980785,
0.975702,
0.970031,
0.963776,
0.95694,
0.949528,
0.941544,
0.932993,
0.92388,
0.91421,
0.903989,
0.893224,
0.881921,
0.870087,
0.857729,
0.844854,
0.83147,
0.817585,
0.803208,
0.788346,
0.77301,
0.757209,
0.740951,
0.724247,
0.707107,
0.689541,
0.671559,
0.653173,
0.634393,
0.615232,
0.595699,
0.575808,
0.55557,
0.534998,
0.514103,
0.492898,
0.471397,
0.449611,
0.427555,
0.405241,
0.382683,
0.359895,
0.33689,
0.313682,
0.290285,
0.266713,
0.24298,
0.219101,
0.19509,
0.170962,
0.14673,
0.122411,
0.098017,
0.073565,
0.049068,
0.024541,
0,
-0.024541,
-0.049068,
-0.073565,
-0.098017,
-0.122411,
-0.14673,
-0.170962,
-0.19509,
-0.219101,
-0.24298,
-0.266713,
-0.290285,
-0.313682,
-0.33689,
-0.359895,
-0.382683,
-0.405241,
-0.427555,
-0.449611,
-0.471397,
-0.492898,
-0.514103,
-0.534998,
-0.55557,
-0.575808,
-0.595699,
-0.615232,
-0.634393,
-0.653173,
-0.671559,
-0.689541,
-0.707107,
-0.724247,
-0.740951,
-0.757209,
-0.77301,
-0.788346,
-0.803208,
-0.817585,
-0.83147,
-0.844854,
-0.857729,
-0.870087,
-0.881921,
-0.893224,
-0.903989,
-0.91421,
-0.92388,
-0.932993,
-0.941544,
-0.949528,
-0.95694,
-0.963776,
-0.970031,
-0.975702,
-0.980785,
-0.985278,
-0.989177,
-0.99248,
-0.995185,
-0.99729,
-0.998795,
-0.999699,
-1,
-0.999699,
-0.998795,
-0.99729,
-0.995185,
-0.99248,
-0.989177,
-0.985278,
-0.980785,
-0.975702,
-0.970031,
-0.963776,
-0.95694,
-0.949528,
-0.941544,
-0.932993,
-0.92388,
-0.91421,
-0.903989,
-0.893224,
-0.881921,
-0.870087,
-0.857729,
-0.844854,
-0.83147,
-0.817585,
-0.803208,
-0.788346,
-0.77301,
-0.757209,
-0.740951,
-0.724247,
-0.707107,
-0.689541,
-0.671559,
-0.653173,
-0.634393,
-0.615232,
-0.595699,
-0.575808,
-0.55557,
-0.534998,
-0.514103,
-0.492898,
-0.471397,
-0.449611,
-0.427555,
-0.405241,
-0.382683,
-0.359895,
-0.33689,
-0.313682,
-0.290285,
-0.266713,
-0.24298,
-0.219101,
-0.19509,
-0.170962,
-0.14673,
-0.122411,
-0.098017,
-0.073565,
-0.049068,
-0.024541
};

const float cosineTable[TABLE_SIZE] = 
{
1,
0.999699,
0.998795,
0.99729,
0.995185,
0.99248,
0.989177,
0.985278,
0.980785,
0.975702,
0.970031,
0.963776,
0.95694,
0.949528,
0.941544,
0.932993,
0.92388,
0.91421,
0.903989,
0.893224,
0.881921,
0.870087,
0.857729,
0.844854,
0.83147,
0.817585,
0.803208,
0.788346,
0.77301,
0.757209,
0.740951,
0.724247,
0.707107,
0.689541,
0.671559,
0.653173,
0.634393,
0.615232,
0.595699,
0.575808,
0.55557,
0.534998,
0.514103,
0.492898,
0.471397,
0.449611,
0.427555,
0.405241,
0.382683,
0.359895,
0.33689,
0.313682,
0.290285,
0.266713,
0.24298,
0.219101,
0.19509,
0.170962,
0.14673,
0.122411,
0.098017,
0.073565,
0.049068,
0.024541,
0,
-0.024541,
-0.049068,
-0.073565,
-0.098017,
-0.122411,
-0.14673,
-0.170962,
-0.19509,
-0.219101,
-0.24298,
-0.266713,
-0.290285,
-0.313682,
-0.33689,
-0.359895,
-0.382683,
-0.405241,
-0.427555,
-0.449611,
-0.471397,
-0.492898,
-0.514103,
-0.534998,
-0.55557,
-0.575808,
-0.595699,
-0.615232,
-0.634393,
-0.653173,
-0.671559,
-0.689541,
-0.707107,
-0.724247,
-0.740951,
-0.757209,
-0.77301,
-0.788346,
-0.803208,
-0.817585,
-0.83147,
-0.844854,
-0.857729,
-0.870087,
-0.881921,
-0.893224,
-0.903989,
-0.91421,
-0.92388,
-0.932993,
-0.941544,
-0.949528,
-0.95694,
-0.963776,
-0.970031,
-0.975702,
-0.980785,
-0.985278,
-0.989177,
-0.99248,
-0.995185,
-0.99729,
-0.998795,
-0.999699,
-1,
-0.999699,
-0.998795,
-0.99729,
-0.995185,
-0.99248,
-0.989177,
-0.985278,
-0.980785,
-0.975702,
-0.970031,
-0.963776,
-0.95694,
-0.949528,
-0.941544,
-0.932993,
-0.92388,
-0.91421,
-0.903989,
-0.893224,
-0.881921,
-0.870087,
-0.857729,
-0.844854,
-0.83147,
-0.817585,
-0.803208,
-0.788346,
-0.77301,
-0.757209,
-0.740951,
-0.724247,
-0.707107,
-0.689541,
-0.671559,
-0.653173,
-0.634393,
-0.615232,
-0.595699,
-0.575808,
-0.55557,
-0.534998,
-0.514103,
-0.492898,
-0.471397,
-0.449611,
-0.427555,
-0.405241,
-0.382683,
-0.359895,
-0.33689,
-0.313682,
-0.290285,
-0.266713,
-0.24298,
-0.219101,
-0.19509,
-0.170962,
-0.14673,
-0.122411,
-0.098017,
-0.073565,
-0.049068,
-0.024541,
0,
0.024541,
0.049068,
0.073565,
0.098017,
0.122411,
0.14673,
0.170962,
0.19509,
0.219101,
0.24298,
0.266713,
0.290285,
0.313682,
0.33689,
0.359895,
0.382683,
0.405241,
0.427555,
0.449611,
0.471397,
0.492898,
0.514103,
0.534998,
0.55557,
0.575808,
0.595699,
0.615232,
0.634393,
0.653173,
0.671559,
0.689541,
0.707107,
0.724247,
0.740951,
0.757209,
0.77301,
0.788346,
0.803208,
0.817585,
0.83147,
0.844854,
0.857729,
0.870087,
0.881921,
0.893224,
0.903989,
0.91421,
0.92388,
0.932993,
0.941544,
0.949528,
0.95694,
0.963776,
0.970031,
0.975702,
0.980785,
0.985278,
0.989177,
0.99248,
0.995185,
0.99729,
0.998795,
0.999699
};


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

void MC_APP_CLASSB_Tasks (void )
{
    /* Check the application's current state. */
    switch ( mc_appData.classBState )
    {
        /* Application's initial state. */
        case MC_APP_CLASSB_STATE_INIT:
        {
			// Disable global interrupts
			SYS_INT_Disable();
			
			// The stack is filled from the high memory to the lower memory 
			// Stack pointer will begin at stackStartAddress
			// stackStartAddress will be a larger value than stackEndAddress
			stackStartAddress= (unsigned int*)_stack;
			stackEndAddress= (unsigned int*)(_stack-((unsigned int)_min_stack_size));
			stackTotSize=(unsigned int)stackStartAddress - (unsigned int)stackEndAddress;
    
			// The available memory is from the start of RAM to the end of the stack.
			ramStartAddress= (unsigned int*)_sdata_begin;
			ramEndAddress= stackEndAddress;
			ramTotSize=(unsigned int)ramEndAddress - (unsigned int)ramStartAddress;

			// FOR THE RAM
			ramTestSize = (ramTotSize>0x400)?0x400:ramTotSize;
			ramTestSize&=0xffffffc0;
			ramTestStartAddress =(unsigned int *)(ramStartAddress+(((ramTotSize-ramTestSize)/2)/sizeof(unsigned int)));

			// FOR THE STACK
			// The ram test area is used to save the stack, so its size has to be larger.
			stackTestSize = (stackTotSize>0x400)?0x400:stackTotSize;
			stackTestSize = (ramTestSize>stackTestSize)?stackTestSize:(ramTestSize-sizeof(unsigned int));
			stackTestSize&=0xffffffc0;
            
			mc_appData.classBState = MC_APP_CLASSB_STATE_FLASH_CRC_TEST;
            
            break;
        }

        case MC_APP_CLASSB_STATE_FLASH_CRC_TEST:
        {
			// Calculate the CRC16 of the whole program flash      
			char* flashStartAddress = (char*)0x9d000000;       	
			char* flashEndAddress =  flashStartAddress+0x1000;  

			// calculate the reference Flash CRC value
			flashCrcRef = CLASSB_CRCFlashTest( flashStartAddress, flashEndAddress, CRC_16_GEN_POLY, crcSeed);
			
			// RE-calculate the CRC of the Flash 
			flashCrc = CLASSB_CRCFlashTest( flashStartAddress, flashEndAddress, CRC_16_GEN_POLY, crcSeed);

			// Compare reference and calculated Flash CRC values
			if ( flashCrc==flashCrcRef)
            {
				mc_appData.classBState = MC_APP_CLASSB_STATE_CHECKER_BOARD_RAM_TEST;
            }
			else
            { 
                mc_appData.classBState = MC_APP_CLASSB_STATE_FLASH_CRC_TEST;
            }
            break;
        }
        
        case MC_APP_CLASSB_STATE_CHECKER_BOARD_RAM_TEST :
        {
			if(CLASSB_RAMCheckerBoardTest(ramTestStartAddress, ramTestSize)==CLASSB_TEST_PASS)
			{
                mc_appData.classBState = MC_APP_CLASSB_STATE_RAM_MARCHB_TEST;
			}
			else
            { 
                mc_appData.classBState = MC_APP_CLASSB_STATE_CHECKER_BOARD_RAM_TEST;
            }
            break;
        }
                   
        case MC_APP_CLASSB_STATE_RAM_MARCHB_TEST:
        {
			if(CLASSB_RAMMarchBTest(ramTestStartAddress, ramTestSize )==CLASSB_TEST_PASS)
			{
				mc_appData.classBState = MC_APP_CLASSB_STATE_RAM_MARCHC_TEST;
			}
			else	
            { 
                mc_appData.classBState = MC_APP_CLASSB_STATE_RAM_MARCHB_TEST;
            }
            break;
        }
        
        case MC_APP_CLASSB_STATE_RAM_MARCHC_TEST:
        {
			if(CLASSB_RAMMarchCTest(ramTestStartAddress, ramTestSize )==CLASSB_TEST_PASS)
			{
                mc_appData.classBState = MC_APP_CLASSB_STATE_RAM_MARCHC_STACK_TEST;
			}
			else
			{ 
                mc_appData.classBState = MC_APP_CLASSB_STATE_RAM_MARCHC_TEST;
			}
			break;
        }
           
        case MC_APP_CLASSB_STATE_RAM_MARCHC_STACK_TEST:
        {
			if(CLASSB_RAMMarchCStackTest(ramTestStartAddress, ramTestSize)==CLASSB_TEST_PASS)
			{
                mc_appData.classBState = MC_APP_CLASSB_STATE_CLOCK_TEST;
			}
			else
            { 
                mc_appData.classBState = MC_APP_CLASSB_STATE_RAM_MARCHC_STACK_TEST;
            }
            break;
        }
        
        case MC_APP_CLASSB_STATE_CLOCK_TEST:
        {
			if(CLASSB_ClockTest(SYS_CLK_FREQ , CLOCK_TEST_REFERENCE_FREQ, 1, 10) ==CLASSB_TEST_PASS)
			{
				mc_appData.classBState = MC_APP_CLASSB_STATE_CPU_PC_TEST;
			}
			else
            { 
                mc_appData.classBState = MC_APP_CLASSB_STATE_CLOCK_TEST;
            }
            break;
        }
        
        case MC_APP_CLASSB_STATE_CPU_PC_TEST:
        {
			if(CLASSB_CPUPCTest() == CLASSB_TEST_PASS)
			{
                mc_appData.classBState =  MC_APP_CLASSB_STATE_CPU_REGISTERS_TEST;
			}
			else
            { 
                mc_appData.classBState = MC_APP_CLASSB_STATE_CPU_PC_TEST;
            }
            break;
        }
		
        case MC_APP_CLASSB_STATE_CPU_REGISTERS_TEST:
        {
			if(CLASSB_CPURegistersTest() == CLASSB_TEST_PASS)
			{
                mc_appData.classBState =  MC_APP_CLASSB_STATE_TEST_PASS;
			}
			else
            { 
                mc_appData.classBState = MC_APP_CLASSB_STATE_TEST_FAIL;
            }
            break;
        }
        
        case MC_APP_CLASSB_STATE_TEST_PASS:
        {
            // Enable global interrupts
			SYS_INT_Enable();
			
            break;
        }
              
        case MC_APP_CLASSB_STATE_TEST_FAIL:
        {
            
            break;
        }
      
        /* The default state should never be executed. */
        default:
        {
            break;
        }
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: MC ADC ISR TASKS
// *****************************************************************************
// *****************************************************************************
void MC_APP_MC_ADCISRTasks(void)
{
	phaseCurrentA = PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1 , ADCHS_AN0) ; // Phase Current A is connected to AN24 on ADC0
    phaseCurrentB = PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1 , ADCHS_AN4);// Phase Current B is connected to AN9 on ADC4
    potReading = PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1 , ADCHS_AN15);   // Speed Potentiometer is connected to AN15 on ADC7  
    ParkParm.DCBusVoltage = (float)PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1 , ADCHS_AN10) * VOLTAGE_ADC_TO_PHY_RATIO; // Reads and translates to actual bus voltage
    ParkParm.MaxPhaseVoltage = (float)(ParkParm.DCBusVoltage*ONE_BY_SQRT3); 
	
	/* Moving Average Filter is implemented to calculate the current offset. Window size of the moving average filter = 2^MOVING_AVG_WINDOW_SIZE
	cumulative_sum_phaseX(n) = cumulative_sum_phaseX(n-1) + phaseCurrentX(n) - moving_average_phaseX(n-1)
	moving_average_phaseX(n) =  cumulative_sum_phaseX(n)/(2^MOVING_AVG_WINDOW_SIZE) */

	
	cumulative_sum_phaseA  = cumulative_sum_phaseA + phaseCurrentA - moving_average_phaseA;
    moving_average_phaseA  = cumulative_sum_phaseA >> MOVING_AVG_WINDOW_SIZE;
    
	/*Bounding the offset value */
	if(moving_average_phaseA > CURRENT_OFFSET_MAX)
    {
        moving_average_phaseA = CURRENT_OFFSET_MAX;
    }
    else if (moving_average_phaseA < CURRENT_OFFSET_MIN)
    {
        moving_average_phaseA = CURRENT_OFFSET_MIN;
    }
	
	
	cumulative_sum_phaseB  = cumulative_sum_phaseB + phaseCurrentB - moving_average_phaseB;
    moving_average_phaseB  = cumulative_sum_phaseB >> MOVING_AVG_WINDOW_SIZE;
	
	/*Bounding the offset value */
	if(moving_average_phaseB > CURRENT_OFFSET_MAX)
    {
        moving_average_phaseB = CURRENT_OFFSET_MAX;
    }
    else if (moving_average_phaseB < CURRENT_OFFSET_MIN)
    {
        moving_average_phaseB = CURRENT_OFFSET_MIN;
    }
	
    phaseCurrentA = (phaseCurrentA - moving_average_phaseA); // Removing the offset
    phaseCurrentB = (phaseCurrentB - moving_average_phaseB);

    ParkParm.Ia = (float)phaseCurrentA*ADC_CURRENT_SCALE * (-1); 
    ParkParm.Ib = (float)phaseCurrentB*ADC_CURRENT_SCALE * (-1);
    
    MC_APP_MC_Clarke();
   
    MC_APP_MC_Park();
    
    MC_APP_MC_Estim();
       
    // Calculate control values
    MC_APP_MC_DoControl();
  

    MC_APP_MC_CalculateParkAngle();

    /* if open loop */
    if(MC_APP_MC_CONTROL.bit.OpenLoop == 1)
    {
        /* the angle is given by parkparm */
        SincosParm.Angle = ParkParm.Angle;
       
    } 
    else
    {
        /* if closed loop, angle generated by estim */
        SincosParm.Angle = EstimParm.qRho;
    }
    MC_APP_MC_SinCos();
    ParkParm.Sin = SincosParm.Sin;
    ParkParm.Cos = SincosParm.Cos;
    

 
    MC_APP_MC_InvPark();

	MC_APP_MC_CalcRefVec();
  
     
    MC_APP_MC_CalcSVGen();
	PLIB_MCPWM_ChannelPrimaryDutyCycleSet(MCPWM_ID_0, MCPWM_CHANNEL1,(uint16_t) dPWM1);
    PLIB_MCPWM_ChannelPrimaryDutyCycleSet(MCPWM_ID_0, MCPWM_CHANNEL2,(uint16_t) dPWM2);
	PLIB_MCPWM_ChannelPrimaryDutyCycleSet(MCPWM_ID_0, MCPWM_CHANNEL3,(uint16_t) dPWM3);
}

void MC_APP_MC_Clarke(void)
{
    ParkParm.Ialpha = ParkParm.Ia;
    ParkParm.Ibeta = (ParkParm.Ia * ONE_BY_SQRT3) + (ParkParm.Ib * 2 * ONE_BY_SQRT3);
}

void MC_APP_MC_Park(void)
{
    ParkParm.Id =  ParkParm.Ialpha*ParkParm.Cos + ParkParm.Ibeta*ParkParm.Sin;
    ParkParm.Iq = -ParkParm.Ialpha*ParkParm.Sin + ParkParm.Ibeta*ParkParm.Cos;
}


// *****************************************************************************
// *****************************************************************************
// Section: MC PLL Estimator Routines
// *****************************************************************************
// *****************************************************************************
void MC_APP_MC_Estim(void)
{
	float tempqVelEstim;
    
    if(EstimParm.qVelEstim < 0)
    {
        tempqVelEstim = EstimParm.qVelEstim * (-1);    
    }
    else
    {
        tempqVelEstim = EstimParm.qVelEstim;
    }
    
	EstimParm.qDIalpha	=	(ParkParm.Ialpha-EstimParm.qLastIalpha);
    EstimParm.qVIndalpha = (MotorEstimParm.qLsDt * EstimParm.qDIalpha);
    EstimParm.qDIbeta	=	(ParkParm.Ibeta-EstimParm.qLastIbeta);
    EstimParm.qVIndbeta= (MotorEstimParm.qLsDt * EstimParm.qDIbeta);
    
    // Update LastIalpha and LastIbeta
    EstimParm.qLastIalpha	=	ParkParm.Ialpha;
    EstimParm.qLastIbeta 	=	ParkParm.Ibeta;
    
    // Stator voltage eqations
 	EstimParm.qEsa		= 	EstimParm.qLastValpha -
							((MotorEstimParm.qRs  * ParkParm.Ialpha))
							- EstimParm.qVIndalpha;
							
 	EstimParm.qEsb		= 	EstimParm.qLastVbeta -
							((MotorEstimParm.qRs  * ParkParm.Ibeta ))
							- EstimParm.qVIndbeta;
    
    // Update LastValpha and LastVbeta
	EstimParm.qLastValpha = (ParkParm.MaxPhaseVoltage * ParkParm.Valpha);
	EstimParm.qLastVbeta = (ParkParm.MaxPhaseVoltage * ParkParm.Vbeta);

    // Calculate Sin(Rho) and Cos(Rho)
    SincosParm.Angle 	=	EstimParm.qRho + EstimParm.RhoOffset; 

    if(SincosParm.Angle <= 0) //SINGLE_ELEC_ROT_RADS_PER_SEC)
        SincosParm.Angle = SincosParm.Angle + SINGLE_ELEC_ROT_RADS_PER_SEC; 
    if(SincosParm.Angle >= SINGLE_ELEC_ROT_RADS_PER_SEC)
        SincosParm.Angle = SincosParm.Angle - SINGLE_ELEC_ROT_RADS_PER_SEC; 
  
	MC_APP_MC_SinCos();

    // Translate Back EMF (Alpha,beta)  ESA, ESB to Back EMF(D,Q) ESD, ESQ using Park Transform. 
    EstimParm.qEsd		=	((EstimParm.qEsa * SincosParm.Cos))
							+
							((EstimParm.qEsb * SincosParm.Sin));
    
	EstimParm.qEsq		=	(( EstimParm.qEsb * SincosParm.Cos))
							-
							((EstimParm.qEsa * SincosParm.Sin));

    // Filter first order for Esd and Esq
	EstimParm.qEsdf			= EstimParm.qEsdf+
							  ((EstimParm.qEsd - EstimParm.qEsdf) * EstimParm.qKfilterEsdq) ;

	EstimParm.qEsqf			= EstimParm.qEsqf+
							  ((EstimParm.qEsq - EstimParm.qEsqf) * EstimParm.qKfilterEsdq) ;

	if(CtrlParm.VelRef > NOMINAL_SPEED_RAD_PER_SEC_ELEC)
        {
    
            //Using airgap flux as KFi, i.e. KFi = BackEmF_Constant + (Ls*id/(2*pi)).
            //This factors in the air gap flux variation during Field Weakening. 
            //Ls.id is divided by 2*pi to match the units of BackEmF_Constant given in v-sec/rad.
            MotorEstimParm.qInvKFi = (float)(1/(MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC 
                                        + (MOTOR_PER_PHASE_INDUCTANCE_DIV_2_PI*CtrlParm.IdRef)));  

        }
    else
    {
        MotorEstimParm.qInvKFi = (float)INVKFi_BELOW_BASE_SPEED;
    }
    
							  
      
	if (tempqVelEstim>DECIMATE_NOMINAL_SPEED)
    {
    	if(EstimParm.qEsqf>0)
    	{
    		EstimParm.qOmegaMr	=	((MotorEstimParm.qInvKFi*(EstimParm.qEsqf- EstimParm.qEsdf))) ;
    	} 
		else
    	{
    		EstimParm.qOmegaMr	=	((MotorEstimParm.qInvKFi*(EstimParm.qEsqf + EstimParm.qEsdf)));
    	}
    } 
	else // if est speed<10% => condition VelRef<>0
    {
    	if(EstimParm.qVelEstim>0)
    	{
    		EstimParm.qOmegaMr	=	((MotorEstimParm.qInvKFi*(EstimParm.qEsqf- EstimParm.qEsdf))) ;
    	} 
		else
    	{
    		EstimParm.qOmegaMr	=	((MotorEstimParm.qInvKFi*(EstimParm.qEsqf+ EstimParm.qEsdf))) ;
    	}
    }
    	
	// The integral of the angle is the estimated angle */
	EstimParm.qRho	= 	EstimParm.qRho+
							(EstimParm.qOmegaMr)*(EstimParm.qDeltaT);
    
    if(EstimParm.qRho >= SINGLE_ELEC_ROT_RADS_PER_SEC)
        EstimParm.qRho = EstimParm.qRho - SINGLE_ELEC_ROT_RADS_PER_SEC;      

     if(EstimParm.qRho <= 0)// SINGLE_ELEC_ROT_RADS_PER_SEC)
        EstimParm.qRho = EstimParm.qRho + SINGLE_ELEC_ROT_RADS_PER_SEC; 
    

    // The estimated speed is a filter value of the above calculated OmegaMr. The filter implementation
    // is the same as for BEMF d-q components filtering
	EstimParm.qVelEstim = (EstimParm.qVelEstim+
						( (EstimParm.qOmegaMr-EstimParm.qVelEstim)*EstimParm.qVelEstimFilterK ));
}

// *****************************************************************************
// *****************************************************************************
// Section: MC PLL Estimator Parameter Initialization Routine
// *****************************************************************************
// *****************************************************************************
void MC_APP_MC_InitEstimParm(void)  
{
	MotorEstimParm.qLsDt = (float)(MOTOR_PER_PHASE_INDUCTANCE/LOOPTIME_SEC);
	MotorEstimParm.qRs = MOTOR_PER_PHASE_RESISTANCE;
	
	MotorEstimParm.qInvKFi = (float)INVKFi_BELOW_BASE_SPEED;

   	EstimParm.qRhoStateVar=0;
	EstimParm.qOmegaMr=0;
	 
    EstimParm.qKfilterEsdq = KFILTER_ESDQ;
    EstimParm.qVelEstimFilterK = KFILTER_VELESTIM;

    EstimParm.qDeltaT = 0.00005;
    EstimParm.RhoOffset = (45 * (M_PI/180));
}

// *****************************************************************************
// *****************************************************************************
// Section: MC FOC Control Routine
// *****************************************************************************
// *****************************************************************************

void MC_APP_MC_DoControl( void )
{
  
	  
    if( MC_APP_MC_CONTROL.bit.OpenLoop )
    {
        // OPENLOOP:  force rotating angle,Vd,Vq
        if( MC_APP_MC_CONTROL.bit.ChangeMode )
        {
            // just changed to openloop
            MC_APP_MC_CONTROL.bit.ChangeMode = 0;
            // synchronize angles

            // VqRef & VdRef not used
            CtrlParm.IqRef = 0;
            CtrlParm.IdRef = 0;

			// reinit vars for initial speed ramp
			Startup_Lock_Count = 0;
			Startup_Ramp_Angle_Rads_Per_Sec = 0;
        }
        
        // q current reference is equal to the vel reference
        // while d current reference is equal to 0
        // for maximum startup torque, set the q current to maximum acceptable
        // value represents the maximum peak value
        CtrlParm.IqRef    = Q_CURRENT_REF_OPENLOOP;
       	
        // PI control for Q
        PIParmQ.qInMeas = ParkParm.Iq;
        PIParmQ.qInRef  = CtrlParm.IqRef;
        MC_APP_MC_CalcPI(&PIParmQ);
        ParkParm.Vq = PIParmQ.qOut;
       
        // PI control for D
        PIParmD.qInMeas = ParkParm.Id;
        PIParmD.qInRef  = CtrlParm.IdRef;
        MC_APP_MC_CalcPI(&PIParmD);
        ParkParm.Vd = PIParmD.qOut;
    } 
    else
    // Closed Loop Vector Control
	{ 
        PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_F,PORTS_BIT_POS_5);
        
		if( MC_APP_MC_CONTROL.bit.ChangeMode )
        {
            // just changed from openloop
            MC_APP_MC_CONTROL.bit.ChangeMode = 0;
			PIParmQref.qdSum = CtrlParm.IqRef;
            CtrlParm.VelRef = END_SPEED_RADS_PER_SEC_ELEC;
            PIParmD.qInRef = 0.0;
            CtrlParm.IdRef = 0.0;
        }             
        
        
        CtrlParm.VelRef =(float)((float)potReading * POT_ADC_COUNT_FW_SPEED_RATIO);
        
		if(CtrlParm.VelRef < END_SPEED_RADS_PER_SEC_ELEC)
        {
            CtrlParm.VelRef = END_SPEED_RADS_PER_SEC_ELEC;
        }
        
       
        
        //if TORQUE MODE skip the speed and Field Weakening controller               
#ifndef	TORQUE_MODE
       	// Execute the velocity control loop
		PIParmQref.qInMeas = EstimParm.qVelEstim;
    	PIParmQref.qInRef  = CtrlParm.VelRef;
    	MC_APP_MC_CalcPI(&PIParmQref);
    	CtrlParm.IqRef = PIParmQref.qOut;



		
		// Implement Field Weakening if Speed input is greater than the base speed of the motor
		
		if(CtrlParm.VelRef > NOMINAL_SPEED_RAD_PER_SEC_ELEC)
        {
			 ParkParm.VdqNorm = sqrt((ParkParm.Vd*ParkParm.Vd)+ (ParkParm.Vq*ParkParm.Vq));
			 //Vqref = sqrt(Vmax^2 -Vd^2 )
			 ParkParm.VdSquaredDenorm = ParkParm.Vd*ParkParm.Vd*ParkParm.MaxPhaseVoltage*ParkParm.MaxPhaseVoltage;
             ParkParm.MaxVoltageCircleSquared = 0.98 * ParkParm.MaxPhaseVoltage * ParkParm.MaxPhaseVoltage;
             ParkParm.VqSquaredDenorm = ParkParm.MaxVoltageCircleSquared - ParkParm.VdSquaredDenorm;
             if(ParkParm.VqSquaredDenorm < 0)
             ParkParm.VqSquaredDenorm = 0;
             ParkParm.VqRefVoltage = sqrt(ParkParm.VqSquaredDenorm);  
			 
			 //Calculating Flux Weakening value of Id, Id_flux_Weakening = (Vqref- Rs*Iq - BEMF)/omega*Ls

             CtrlParm.IdRef = (ParkParm.VqRefVoltage - (MOTOR_PER_PHASE_RESISTANCE * CtrlParm.IqRef) 
									-(CtrlParm.VelRef  * MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC))/(CtrlParm.VelRef  * MOTOR_PER_PHASE_INDUCTANCE);
									
									
			 // Limit Id such that MAX_FW_NEGATIVE_ID_REF < Id < 0
			if(CtrlParm.IdRef > 0)
				CtrlParm.IdRef = 0; 
        
			if(CtrlParm.IdRef < MAX_FW_NEGATIVE_ID_REF)
				CtrlParm.IdRef = MAX_FW_NEGATIVE_ID_REF;
			
			// Limit Q axis current such that sqrt(Id^2 +Iq^2) <= MAX_MOTOR_CURRENT
			CtrlParm.IqRefmax = sqrt((MAX_MOTOR_CURRENT_SQUARED) - (CtrlParm.IdRef*CtrlParm.IdRef));		
		}
		else
		{
			CtrlParm.IdRef = 0;
			CtrlParm.IqRefmax = MAX_MOTOR_CURRENT;
		}
		
		
#else
        CtrlParm.IqRef = Q_CURRENT_REF_OPENLOOP; // During torque mode, Iq = Open Loop Iq, Id = 0
		CtrlParm.IdRef = 0;
#endif  // endif for TORQUE_MODE
		
        // PI control for D
        PIParmD.qInMeas = ParkParm.Id;          // This is in Amps
        PIParmD.qInRef  = CtrlParm.IdRef;      // This is in Amps
        MC_APP_MC_CalcPI(&PIParmD);
        ParkParm.Vd    =  PIParmD.qOut;          // This is in %. If should be converted to volts, multiply with (DC/2)

        // dynamic d-q adjustment
        // with d component priority
        // vq=sqrt (vs^2 - vd^2)
        // limit vq maximum to the one resulting from the calculation above
        DoControl_Temp2 = PIParmD.qOut * PIParmD.qOut;
        DoControl_Temp1 = 0.98 - DoControl_Temp2;
        PIParmQ.qOutMax = sqrt(DoControl_Temp1);        
		
		//Limit Q axis current
		if(CtrlParm.IqRef>CtrlParm.IqRefmax)
		{
		CtrlParm.IqRef = CtrlParm.IqRefmax;
		}
		
        // PI control for Q
        PIParmQ.qInMeas = ParkParm.Iq;          // This is in Amps
        PIParmQ.qInRef  = CtrlParm.IqRef;      // This is in Amps
        MC_APP_MC_CalcPI(&PIParmQ);
        ParkParm.Vq    = PIParmQ.qOut;          // This is in %. If should be converted to volts, multiply with (DC/2)       

    }
}
// *****************************************************************************
// *****************************************************************************
// Section: MC FOC Rotor Angle Calculation
// *****************************************************************************
// *****************************************************************************

void MC_APP_MC_CalculateParkAngle(void)
{
    // If open loop
	if(MC_APP_MC_CONTROL.bit.OpenLoop)	
	{
		// begin with the lock sequence, for field alignment. The rotor is locked at angle = 0 for LOCK_COUNT_FOR_LOCK_TIME ~ 2 seconds
		if (Startup_Lock_Count < LOCK_COUNT_FOR_LOCK_TIME)
			Startup_Lock_Count++;
	    // then ramp up till the end speed
		else if (Startup_Ramp_Angle_Rads_Per_Sec < END_SPEED_RADS_PER_SEC_ELEC_IN_LOOPTIME)
			Startup_Ramp_Angle_Rads_Per_Sec+=OPENLOOP_RAMPSPEED_INCREASERATE;
		else // switch to closed loop
		{
#ifndef OPEN_LOOP_FUNCTIONING
            MC_APP_MC_CONTROL.bit.ChangeMode = 1;
            MC_APP_MC_CONTROL.bit.OpenLoop = 0;
#endif
		}
		
		// the angle set depends on startup ramp
		ParkParm.Angle += Startup_Ramp_Angle_Rads_Per_Sec;
        
        if(ParkParm.Angle >= SINGLE_ELEC_ROT_RADS_PER_SEC)
            ParkParm.Angle = ParkParm.Angle - SINGLE_ELEC_ROT_RADS_PER_SEC;        
	}
	else // switched to closed loop
	{
   	    // in closed loop slowly decrease the offset add to the estimated angle
   	    if(EstimParm.RhoOffset>(M_PI/(float)32767))
            EstimParm.RhoOffset = EstimParm.RhoOffset - ((M_PI/(float)32767)) ; 
	}
	return;
}

// *****************************************************************************
// *****************************************************************************
// Section: MC Inverse Park Transform
// *****************************************************************************
// *****************************************************************************

void MC_APP_MC_InvPark(void)
{
    ParkParm.Valpha =  ParkParm.Vd*ParkParm.Cos - ParkParm.Vq*ParkParm.Sin;
    ParkParm.Vbeta  =  ParkParm.Vd*ParkParm.Sin + ParkParm.Vq*ParkParm.Cos;       
}

// *****************************************************************************
// *****************************************************************************
// Section: MC Modified Inverse Clarke Transform
// *****************************************************************************
// *****************************************************************************

void MC_APP_MC_CalcRefVec(void) // rotates the motor in clockwise direction
{
    SVGenParm.Vr1 = ParkParm.Vbeta;
    SVGenParm.Vr2 = (-ParkParm.Vbeta/2 + SQRT3_BY2 * ParkParm.Valpha);
    SVGenParm.Vr3 = (-ParkParm.Vbeta/2 - SQRT3_BY2 * ParkParm.Valpha);       
} 

// *****************************************************************************
// *****************************************************************************
// Section: MC PI Controller Routines
// *****************************************************************************
// *****************************************************************************

void MC_APP_MC_InitControlParameters(void)
{
	// PI D Term     
    PIParmD.qKp = D_CURRCNTR_PTERM;       
    PIParmD.qKi = D_CURRCNTR_ITERM;              
    PIParmD.qKc = D_CURRCNTR_CTERM;       
    PIParmD.qOutMax = D_CURRCNTR_OUTMAX;
    PIParmD.qOutMin = -PIParmD.qOutMax;

    MC_APP_MC_InitPI(&PIParmD);

    // PI Q Term 
    PIParmQ.qKp = Q_CURRCNTR_PTERM;    
    PIParmQ.qKi = Q_CURRCNTR_ITERM;
    PIParmQ.qKc = Q_CURRCNTR_CTERM;
    PIParmQ.qdSum = 0;
    PIParmQ.qOutMax = Q_CURRCNTR_OUTMAX;
    PIParmQ.qOutMin = -PIParmQ.qOutMax;

    MC_APP_MC_InitPI(&PIParmQ);

    // PI Qref Term
    PIParmQref.qKp = SPEEDCNTR_PTERM;       
    PIParmQref.qKi = SPEEDCNTR_ITERM;       
    PIParmQref.qKc = SPEEDCNTR_CTERM;       
    PIParmQref.qOutMax = SPEEDCNTR_OUTMAX;   
    PIParmQref.qOutMin = -PIParmQref.qOutMax;

    MC_APP_MC_InitPI(&PIParmQref);
	
	
	
	return;
}

void MC_APP_MC_InitPI( tPIParm *pParm)
{
    pParm->qdSum = 0;
    pParm->qOut = 0;
}

void MC_APP_MC_CalcPI( tPIParm *pParm)
{
    float Err;
    float U;
    float Exc;
    
    Err  = pParm->qInRef - pParm->qInMeas;
    pParm->qErr =  Err; 
    U  = pParm->qdSum + pParm->qKp * Err;
   
    if( U > pParm->qOutMax )
    {
        pParm->qOut = pParm->qOutMax;
    }    
    else if( U < pParm->qOutMin )
    {
        pParm->qOut = pParm->qOutMin;
    }
    else        
    {
        pParm->qOut = U;  
    }
     
    Exc = U - pParm->qOut;
    pParm->qdSum = pParm->qdSum + pParm->qKi * Err - pParm->qKc * Exc;

}

// *****************************************************************************
// *****************************************************************************
// Section: MC Sine Cosine Functions
// *****************************************************************************
// *****************************************************************************

void MC_APP_MC_SinCos(void)
{
    // IMPORTANT:
    // DO NOT PASS "SincosParm.angle" > 2*PI. There is no software check
    
    // Since we are using "float", it is not possible to get an index of array
    // directly. Almost every time, we will need to do interpolation, as per
    // following equation: -
    // y = y0 + (y1 - y0)*((x - x0)/(x1 - x0))
    
    uint32_t y0_Index;
    uint32_t y0_IndexNext;
    float x0, x1, y0, y1, temp;
    
    y0_Index = (uint32_t)(SincosParm.Angle/ANGLE_STEP);
    
	//Added this condition which detects if y0_Index is >=256.
    //Earlier the only check was for y0_IndexNext. 
    //We observed y0_Index > = 256 when the code to reverse the direction of the motor was added
    if(y0_Index>=TABLE_SIZE)
    {
        y0_Index = 0;
        y0_IndexNext = 1;
        x0 = TOTAL_SINE_TABLE_ANGLE;
        x1 = ANGLE_STEP;
        temp = 0;
    }
    else
    {
        y0_IndexNext = y0_Index + 1;
        if(y0_IndexNext >= TABLE_SIZE )
        {
            y0_IndexNext = 0;
            x1 = TOTAL_SINE_TABLE_ANGLE;
        }
        else
        {
            x1 = ((y0_IndexNext) * ANGLE_STEP);
        }

        x0 = (y0_Index * ANGLE_STEP);  
    
    
    // Since below calculation is same for sin & cosine, we can do it once and reuse
    
	temp = ((SincosParm.Angle - x0)/(x1 - x0));
    }
    
	// Find Sine now
    y0 = sineTable[y0_Index];
    y1 = sineTable[y0_IndexNext];     
    SincosParm.Sin = y0 + ((y1 - y0)*temp);
	
    // Find Cosine now
    y0 = cosineTable[y0_Index];
    y1 = cosineTable[y0_IndexNext];
    SincosParm.Cos = y0 + ((y1 - y0)*temp);
}


// *****************************************************************************
// *****************************************************************************
// Section: MC Space Vector Modulation Routines
// *****************************************************************************
// *****************************************************************************



void MC_APP_MC_CalcSVGen( void )
{
    if( SVGenParm.Vr1 >= 0 )
    {       
		// (xx1)
        if( SVGenParm.Vr2 >= 0 )
        {
            // (x11)
            // Must be Sector 3 since Sector 7 not allowed
            // Sector 3: (0,1,1)  0-60 degrees
            T1 = SVGenParm.Vr2;
            T2 = SVGenParm.Vr1;
            MC_APP_MC_CalcTimes();
            dPWM1 = Ta;
            dPWM2 = Tb;
            dPWM3 = Tc;
        }
        else
        {            
            // (x01)
            if( SVGenParm.Vr3 >= 0 )
            {
                // Sector 5: (1,0,1)  120-180 degrees
                T1 = SVGenParm.Vr1;
                T2 = SVGenParm.Vr3;
                MC_APP_MC_CalcTimes();
                dPWM1 = Tc;
                dPWM2 = Ta;
                dPWM3 = Tb;
            }
            else
            {
                // Sector 1: (0,0,1)  60-120 degrees
                T1 = -SVGenParm.Vr2;
                T2 = -SVGenParm.Vr3;
                MC_APP_MC_CalcTimes();
                dPWM1 = Tb;
                dPWM2 = Ta;
                dPWM3 = Tc;
            }
        }
    }
    else
    {
        // (xx0)
        if( SVGenParm.Vr2 >= 0 )
        {
			// (x10)
            if( SVGenParm.Vr3 >= 0 )
            {
                // Sector 6: (1,1,0)  240-300 degrees
                T1 = SVGenParm.Vr3;
                T2 = SVGenParm.Vr2;
                MC_APP_MC_CalcTimes();
                dPWM1 = Tb;
                dPWM2 = Tc;
                dPWM3 = Ta;
            }
            else
            {
                // Sector 2: (0,1,0)  300-0 degrees
                T1 = -SVGenParm.Vr3;
                T2 = -SVGenParm.Vr1;
                MC_APP_MC_CalcTimes();
                dPWM1 = Ta;
                dPWM2 = Tc;
                dPWM3 = Tb;
            }
        }
        else
        {            
            // (x00)
            // Must be Sector 4 since Sector 0 not allowed
            // Sector 4: (1,0,0)  180-240 degrees
            T1 = -SVGenParm.Vr1;
            T2 = -SVGenParm.Vr2;
            MC_APP_MC_CalcTimes();
            dPWM1 = Tc;
            dPWM2 = Tb;
            dPWM3 = Ta;

        }
    }
}

void MC_APP_MC_CalcTimes(void)
{
    T1 = SVGenParm.PWMPeriod * T1;
    T2 = SVGenParm.PWMPeriod * T2;
    Tc = (SVGenParm.PWMPeriod-T1-T2)/2;
    Tb = Tc + T2;
    Ta = Tb + T1;    
}  

// *****************************************************************************
// *****************************************************************************
// Section: MC Peripheral Initialization Functions
// *****************************************************************************
// *****************************************************************************

void MC_APP_PWM_Enable(void)
{
    DRV_MCPWM_Enable();
}

void MC_APP_PWM_Disable(void)
{
    DRV_MCPWM_Disable();
}

void MC_APP_DAC_Initialize(void)
{
    PLIB_CDAC_ReferenceVoltageSelect(CDAC_ID_2, CDAC_VREF_AVDD);
    PLIB_CDAC_DataWrite(CDAC_ID_2, VREF_DAC_VALUE);
    PLIB_CDAC_OutputEnable(CDAC_ID_2, CDAC_OUTPUT1);
    PLIB_CDAC_Enable(CDAC_ID_2);
    
    PLIB_CDAC_ReferenceVoltageSelect(CDAC_ID_3, CDAC_VREF_AVDD);
    if(CURRENT_LIMIT_CMP_REF <= 0xFFF)
    {
    PLIB_CDAC_DataWrite(CDAC_ID_3, CURRENT_LIMIT_CMP_REF);
    }
    else
    {
       PLIB_CDAC_DataWrite(CDAC_ID_3, 0xFFF); 
    }
    PLIB_CDAC_OutputEnable(CDAC_ID_3, CDAC_OUTPUT1);
    PLIB_CDAC_Enable(CDAC_ID_3);
}

// *****************************************************************************
// *****************************************************************************
// Section: MC Fault ISR Tasks
// *****************************************************************************
// *****************************************************************************

void MC_APP_MC_FaultISRTasks(void)
{
PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G,PORTS_BIT_POS_15); // LED D2 Turns ON upon PWM Fault
PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_F,PORTS_BIT_POS_5); // Turn OFF LED D1 which indicates closed loop operation.
DRV_MCPWM_Disable();
mc_appData.mcState = MC_APP_MC_STATE_STOP;
}

void MC_APP_MC_Tasks (void )
{
    /* Check the application's current state. */
    switch ( mc_appData.mcState )
    {
        /* Application's initial state. */
        case MC_APP_MC_STATE_INIT:
        {
            if(!BTN1)
            {
                 MC_APP_MC_CONTROL.bit.Btn1Pressed = 1;
            }
   
            if((mc_appData.classBState == MC_APP_CLASSB_STATE_TEST_PASS) && (MC_APP_MC_CONTROL.bit.Btn1Pressed == 1))
            {
				MC_APP_MC_CONTROL.bit.OpenLoop = 1;
				MC_APP_MC_CONTROL.bit.ChangeMode = 1;
				MC_APP_MC_InitControlParameters();
				MC_APP_MC_InitEstimParm();
				SincosParm.Angle = 0;
				SVGenParm.PWMPeriod = (float)MAX_DUTY;
				DRV_MCPWM_Enable();
				mc_appData.mcState = MC_APP_MC_STATE_NOP;
            }
			
            break;
        }
		
        case MC_APP_MC_STATE_NOP:
        {
            //Do Nothing in this state, ADC ISR will run the control algorithm
            break;
        }
               
        case MC_APP_MC_STATE_STOP:
        {
            DRV_MCPWM_Disable();
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            break;
        }
    }
}

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MC_APP_Initialize ( void )

  Remarks:
    See prototype in mc_app.h.
 */

void MC_APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    mc_appData.state = MC_APP_STATE_INIT;

	mc_appData.classBState =MC_APP_CLASSB_STATE_INIT;
	mc_appData.mcState = MC_APP_MC_STATE_INIT;
		/* Initialize DAC */
	MC_APP_DAC_Initialize();
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void MC_APP_Tasks ( void )

  Remarks:
    See prototype in mc_app.h.
 */

void MC_APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( mc_appData.state )
    {
        /* Application's initial state. */
        case MC_APP_STATE_INIT:
        {
            bool appInitialized = true;
       
			DRV_ADC0_Open();
			DRV_ADC1_Open();
			DRV_ADC2_Open();
        
            if (appInitialized)
            {
			
            
                mc_appData.state = MC_APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case MC_APP_STATE_SERVICE_TASKS:
        {
			MC_APP_CLASSB_Tasks();
			MC_APP_MC_Tasks();
        
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
