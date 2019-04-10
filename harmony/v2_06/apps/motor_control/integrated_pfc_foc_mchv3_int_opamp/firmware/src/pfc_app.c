/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    pfc_app.c

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

#include "pfc_app.h"

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

PFC_APP_DATA pfc_appData;

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
                                            

PFC_GENERAL_PARAM PFC_APP_pfcParam;

PFC_MEASURED_PARAM PFC_APP_acVoltage;
PFC_MEASURED_PARAM PFC_APP_acCurrent;
PFC_MEASURED_PARAM PFC_APP_busVoltage;


PFC_PI_CONTROLLER PFC_APP_pfcVoltageLoopPI;
PFC_PI_CONTROLLER PFC_APP_pfcCurrentLoopPI;

inline static float limit(float x, float min, float max);

float KP_CURRENT= KP_I;
float KI_CURRENT=KI_I;
float KP_VOLTAGE=KP_V;
float KI_VOLTAGE=KI_V;

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

void PFC_APP_CLASSB_Tasks (void )
{
    /* Check the application's current state. */
    switch ( pfc_appData.classBState )
    {
        /* Application's initial state. */
        case PFC_APP_CLASSB_STATE_INIT:
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
            
			pfc_appData.classBState = PFC_APP_CLASSB_STATE_FLASH_CRC_TEST;
            
            break;
        }

        case PFC_APP_CLASSB_STATE_FLASH_CRC_TEST:
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
				pfc_appData.classBState = PFC_APP_CLASSB_STATE_CHECKER_BOARD_RAM_TEST;
            }
			else
            { 
                pfc_appData.classBState = PFC_APP_CLASSB_STATE_FLASH_CRC_TEST;
            }
            break;
        }
        
        case PFC_APP_CLASSB_STATE_CHECKER_BOARD_RAM_TEST :
        {
			if(CLASSB_RAMCheckerBoardTest(ramTestStartAddress, ramTestSize)==CLASSB_TEST_PASS)
			{
                pfc_appData.classBState = PFC_APP_CLASSB_STATE_RAM_MARCHB_TEST;
			}
			else
            { 
                pfc_appData.classBState = PFC_APP_CLASSB_STATE_CHECKER_BOARD_RAM_TEST;
            }
            break;
        }
                   
        case PFC_APP_CLASSB_STATE_RAM_MARCHB_TEST:
        {
			if(CLASSB_RAMMarchBTest(ramTestStartAddress, ramTestSize )==CLASSB_TEST_PASS)
			{
				pfc_appData.classBState = PFC_APP_CLASSB_STATE_RAM_MARCHC_TEST;
			}
			else	
            { 
                pfc_appData.classBState = PFC_APP_CLASSB_STATE_RAM_MARCHB_TEST;
            }
            break;
        }
        
        case PFC_APP_CLASSB_STATE_RAM_MARCHC_TEST:
        {
			if(CLASSB_RAMMarchCTest(ramTestStartAddress, ramTestSize )==CLASSB_TEST_PASS)
			{
                pfc_appData.classBState = PFC_APP_CLASSB_STATE_RAM_MARCHC_STACK_TEST;
			}
			else
			{ 
                pfc_appData.classBState = PFC_APP_CLASSB_STATE_RAM_MARCHC_TEST;
			}
			break;
        }
           
        case PFC_APP_CLASSB_STATE_RAM_MARCHC_STACK_TEST:
        {
			if(CLASSB_RAMMarchCStackTest(ramTestStartAddress, ramTestSize)==CLASSB_TEST_PASS)
			{
                pfc_appData.classBState = PFC_APP_CLASSB_STATE_CLOCK_TEST;
			}
			else
            { 
                pfc_appData.classBState = PFC_APP_CLASSB_STATE_RAM_MARCHC_STACK_TEST;
            }
            break;
        }
        
        case PFC_APP_CLASSB_STATE_CLOCK_TEST:
        {
			if(CLASSB_ClockTest(SYS_CLK_FREQ , CLOCK_TEST_REFERENCE_FREQ, 1, 10) ==CLASSB_TEST_PASS)
			{
				pfc_appData.classBState = PFC_APP_CLASSB_STATE_CPU_PC_TEST;
			}
			else
            { 
                pfc_appData.classBState = PFC_APP_CLASSB_STATE_CLOCK_TEST;
            }
            break;
        }
        
        case PFC_APP_CLASSB_STATE_CPU_PC_TEST:
        {
			if(CLASSB_CPUPCTest() == CLASSB_TEST_PASS)
			{
                pfc_appData.classBState =  PFC_APP_CLASSB_STATE_CPU_REGISTERS_TEST;
			}
			else
            { 
                pfc_appData.classBState = PFC_APP_CLASSB_STATE_CPU_PC_TEST;
            }
            break;
        }
		
        case PFC_APP_CLASSB_STATE_CPU_REGISTERS_TEST:
        {
			if(CLASSB_CPURegistersTest() == CLASSB_TEST_PASS)
			{
                pfc_appData.classBState =  PFC_APP_CLASSB_STATE_TEST_PASS;
			}
			else
            { 
                pfc_appData.classBState = PFC_APP_CLASSB_STATE_TEST_FAIL;
            }
            break;
        }
        
        case PFC_APP_CLASSB_STATE_TEST_PASS:
        {
            // Enable global interrupts
			SYS_INT_Enable();
			
            break;
        }
              
        case PFC_APP_CLASSB_STATE_TEST_FAIL:
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

void PFC_APP_pfcInit(void)
{
  
    PFC_APP_pfcParam.firstPass = ENABLE;
}

/** Limits the input between numerical limits  */
inline static float limit(float x, float min, float max)
{
    return (x > max ) ? max : ((x < min) ? min : x);
}
/**

This function accumulates input voltages for a period of time to calculate
input average voltage.It also compares average voltage for undervoltage or
overvoltage thresholds

Summary: Function to perform Average Voltage Calculation

@param  void
@return Vavgsquare
*/
inline void PFC_APP_AverageVoltageCalc(void)
{
    // Accumulate Sigma(Vac)
    PFC_APP_acVoltage.avgSum = PFC_APP_acVoltage.avgSum + PFC_APP_acVoltage.measured;
    // Accumulate Frequency Counts
    PFC_APP_acVoltage.samples++;
    // Check if Frequency Counts reach a value corresponding to ~12Hz
    if (PFC_APP_acVoltage.samples >= 4096) 
    {
        //Calculating Vac Average by dividing
        //Sigma Vac with 4096(approx. averaging for 10cycles)
        PFC_APP_acVoltage.avgOutput = (float) (PFC_APP_acVoltage.avgSum/4096);
        // Calculates Vacavg^2, result  
        PFC_APP_acVoltage.avgSquare = (float)(PFC_APP_acVoltage.avgOutput*PFC_APP_acVoltage.avgOutput);
        // Clear Sigma(Vac) to compute new value
        PFC_APP_acVoltage.avgSum = 0;
        // Clear Freq Counts to compute new value
        PFC_APP_acVoltage.samples = 0; 
    }
    return;
}
/**

This function accumulates input current for a period of time to calculate
input average current.

Summary: Function to perform Average Current Caluclation

@param  void
@return Avearage Current
*/
inline void PFC_APP_AverageCurrentCalc(void)
{
    // Accumulate Sigma(Vac)
    PFC_APP_acCurrent.avgSum = PFC_APP_acCurrent.avgSum + PFC_APP_acCurrent.measured;
    // Accumulate Frequency Counts
    PFC_APP_acCurrent.samples++;
    // Check if Frequency Counts reach a value corresponding to ~12Hz
    if (PFC_APP_acCurrent.samples >= 4096) 
    {
        // Calculating Iac Average by dividing Sigma Iac with 4096
        // (approx. averaging for 10cycles)
        PFC_APP_acCurrent.avgOutput = (float) (PFC_APP_acCurrent.avgSum/4096);
        // Clear Sigma(Iac) to compute new value
        PFC_APP_acCurrent.avgSum = 0; 
        PFC_APP_acCurrent.samples = 0;
    }
    return;
}

/**

This function checks for a input undervoltage/overvoltage, 
output undervoltage/overvoltage and overcurrent 
Faults

Summary: Function to perform Fault monitoring

@param  void
@return Fault status
*/
inline void pfcFaults(void)
 {
     if(PFC_APP_pfcParam.softStart == DISABLE)
     {
         //DC bus under voltage and over voltage faults
           if(PFC_APP_busVoltage.measured >= PFC_OVER_VOLTAGE)
            {
                 PFC_APP_pfcParam.overvoltage_faultCount++;
               if(PFC_APP_pfcParam.overvoltage_faultCount  > 200)
               {
                   PFC_APP_pfcParam.faultBit = 1;
                   PFC_APP_pfcParam.overvoltage_faultCount=0;
               }
                
            }
         //AC input under voltage and over voltage faults
           else if((PFC_APP_acVoltage.avgOutput >= PFC_AC_OVER_VOLTAGE || PFC_APP_acVoltage.avgOutput <= PFC_AC_UNDER_VOLTAGE))
            {
                PFC_APP_pfcParam.faultBit = 1;
            }
           //AC input over current fault
           else if(PFC_APP_acCurrent.avgOutput >= PFC_OVER_CURRENT)
            {
               PFC_APP_pfcParam.overcurrent_faultCount++;
               if(PFC_APP_pfcParam.overcurrent_faultCount  > 200)
               {
                   PFC_APP_pfcParam.faultBit = 1;
                   PFC_APP_pfcParam.overcurrent_faultCount=0;
               }
           }
           else
           {
               PFC_APP_pfcParam.overvoltage_faultCount=0;
               PFC_APP_pfcParam.overcurrent_faultCount=0;
           }
           if(PFC_APP_pfcParam.faultBit == 1)
           {
                   DRV_MCPWM_Disable();
                PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G,PORTS_BIT_POS_15); // LED D2 Turns ON upon Fault
                PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_F,PORTS_BIT_POS_5); // Turn OFF LED D1 which indicates closed loop operation.                   
               }
           }
        }

/**

in this Function the voltage loop PI related calculations are performed

Summary: Function to perform Outer loop Control

@param  void
@return PI output
*/
inline void PFC_APP_VoltagePI(void)
{
        
 //Run voltage control loop for every 3Khz
        if (PFC_APP_pfcParam.voltLoopExeRate >= COUNT_3KHz)
        {
              
       // Filter for eliminating 100Hz/120Hz ripple
        PFC_APP_pfcVoltageLoopPI.error = PFC_APP_pfcVoltageLoopPI.reference - PFC_APP_busVoltage.measured;

        PFC_APP_pfcVoltageLoopPI.error = (float) ((PFC_APP_pfcVoltageLoopPI.error*ALPHA)+
                              (PFC_APP_pfcVoltageLoopPI.prevError*BETA));
        //End of Filter Code
        PFC_APP_pfcVoltageLoopPI.prevError = PFC_APP_pfcVoltageLoopPI.error;
      
            //Calculating proportional output of voltage PI

            PFC_APP_pfcVoltageLoopPI.propOut = (float)(KP_VOLTAGE*PFC_APP_pfcVoltageLoopPI.error);
            
                                //Check if voltage PI output is saturated
                if (PFC_APP_pfcVoltageLoopPI.saturationFlag == 0)
                {
                    //Calculating Integral output of voltage PI
                            PFC_APP_pfcVoltageLoopPI.integralOut = PFC_APP_pfcVoltageLoopPI.prevIntegralOut +
                            (float) (KI_VOLTAGE*PFC_APP_pfcVoltageLoopPI.error);
                }
 
            //Check if Voltage PI integral output is saturated to maximum or minimum
            PFC_APP_pfcVoltageLoopPI.integralOut = limit(PFC_APP_pfcVoltageLoopPI.integralOut,-1,1);
            
            PFC_APP_pfcVoltageLoopPI.prevIntegralOut = PFC_APP_pfcVoltageLoopPI.integralOut;
            //Addition of Voltage Loop PI,Integral and Proportional Outputs
            PFC_APP_pfcVoltageLoopPI.sum = PFC_APP_pfcVoltageLoopPI.integralOut + PFC_APP_pfcVoltageLoopPI.propOut;

            //Check if Voltage PI output is saturated
            if (PFC_APP_pfcVoltageLoopPI.sum >= (0.999))
            {
                PFC_APP_pfcVoltageLoopPI.sum = (0.999);
                //if Voltage PI output is saturated make Voltagesaturationflag = 1
                PFC_APP_pfcVoltageLoopPI.saturationFlag = 1;
            }
            else if (PFC_APP_pfcVoltageLoopPI.sum <= 0)
            {
                PFC_APP_pfcVoltageLoopPI.sum = 0;
                PFC_APP_pfcVoltageLoopPI.saturationFlag = 1;
            }
            else
            {
                PFC_APP_pfcVoltageLoopPI.saturationFlag = 0;
            }

            PFC_APP_pfcParam.voltLoopExeRate  = 0;
        }
        else
        {
            PFC_APP_pfcParam.voltLoopExeRate ++;
            PFC_APP_pfcVoltageLoopPI.sum = PFC_APP_pfcVoltageLoopPI.sum;
        }
}

/**

in this Function the current loop PI related calculations are performed

Summary: Function to perform inner loop Control

@param  void
@return PI output
*/
inline void PFC_APP_CurrentPI(void)
{

          PFC_APP_pfcCurrentLoopPI.propOut = (float) (KP_CURRENT*PFC_APP_pfcCurrentLoopPI.error);      

      PFC_APP_pfcCurrentLoopPI.integralOut = (float)(KI_CURRENT*PFC_APP_pfcCurrentLoopPI.error); 
      PFC_APP_pfcCurrentLoopPI.integralOut = PFC_APP_pfcCurrentLoopPI.integralOut + (float) PFC_APP_pfcCurrentLoopPI.prevIntegralOut;
        //Check if Current PI integral output is saturated to maximum or minimum        
            PFC_APP_pfcCurrentLoopPI.integralOut = limit(PFC_APP_pfcCurrentLoopPI.integralOut,-0.998,0.998);

        //Addition of Current Loop PI,Integral and Proportional Outputs
        PFC_APP_pfcCurrentLoopPI.sum =  PFC_APP_pfcCurrentLoopPI.propOut + PFC_APP_pfcCurrentLoopPI.integralOut;
        //PFC_APP_pfcCurrentLoopPI.sum =  PFC_APP_pfcCurrentLoopPI.propOut;
        //Check if Current PI output is saturated to maximum or minimum
        PFC_APP_pfcCurrentLoopPI.sum = limit(PFC_APP_pfcCurrentLoopPI.sum, 0 , 1);
        
        PFC_APP_pfcCurrentLoopPI.prevIntegralOut = PFC_APP_pfcCurrentLoopPI.integralOut;
}

/**

This function generates the constant that is used to reshape the current waveform
into a better rectified sinusoidal wave under light load conditions and
zero crossings

Summary: Function to perform Average Voltage Calculation

@param  void
@return Correction factor
*/
inline void SampleCorrection(void)
{
    float temp;
    
    //Sample Correction Factor Calculation = D*VDC/(VDC-VAC)
    
    temp = (float)(PFC_APP_busVoltage.measured - PFC_APP_acVoltage.measured);
    if(temp > 0)
    {
        //Check if Vdc measured is less than '1'
        if(PFC_APP_busVoltage.measured >  0)
        {
            PFC_APP_pfcParam.read_var1 =(float)(temp/PFC_APP_busVoltage.measured);
            //Dividing Vdc 
            temp = (float)( PFC_APP_busVoltage.measured/temp) ;
        }

    
        if(temp > 0)
        {
            //Calculating sample correction factor
            PFC_APP_pfcParam.sampleCorrection = (PFC_APP_pfcCurrentLoopPI.sum*temp) ;
        }

        if(PFC_APP_pfcParam.sampleCorrection <= 0)
        {
            PFC_APP_pfcParam.sampleCorrection = 1;
        }
        
        if(PFC_APP_pfcParam.sampleCorrection >= 1)
        {
            PFC_APP_pfcParam.sampleCorrection = 1;
        }
    }
}


/**

in this Function the adc channels are configured to sample & convert the
MC signals and it also executes the PFC control loops

Summary: Function to perform PFC Control

@param  void
@return none
*/
inline void PFC_APP_pfc_control(void)
{
    float currentRef;
    float currentMeasured;
  
  
    PFC_APP_busVoltage.measured = (float)(PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1 , ADCHS_AN10) * PFC_DC_VOLTAGE_ADC_TO_PHY_RATIO);
    
   
    currentMeasured = (float)(PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1 , ADCHS_AN0));
    
    PFC_APP_acCurrent.measured = (float)((currentMeasured - AC_CURRENT_OFFSET)*PFC_ADC_CURR_SCALE); //2026

   if (PFC_APP_acCurrent.measured <= 0)
   {
       PFC_APP_acCurrent.measured =(float)PFC_ADC_CURR_SCALE;  
   }
    
    PFC_APP_acVoltage.measured = (float)((PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1 , ADCHS_AN4)- AC_VOLTAGE_OFFSET) * PFC_AC_VOLTAGE_ADC_TO_PHY_RATIO);
    
        if (PFC_APP_acVoltage.measured >= 330)
    {
        PFC_APP_acVoltage.measured = 330;
    }

    //Average input Voltage Calculation
    PFC_APP_AverageVoltageCalc();
    
    //Average input Current Calculation
    PFC_APP_AverageCurrentCalc();

    if(((PFC_APP_pfcParam.firstPass == ENABLE && PFC_APP_acVoltage.avgOutput >= VAVG_88V) || (PFC_APP_pfcParam.firstPass == DISABLE && PFC_APP_busVoltage.measured >= PFC_MIN_VOLTREF))&& PFC_APP_pfcParam.faultBit==0)
    {

        if (PFC_APP_pfcParam.firstPass)
        {
            PFC_APP_pfcVoltageLoopPI.reference = PFC_APP_busVoltage.measured;
            PFC_APP_pfcParam.firstPass = DISABLE;
            PFC_APP_pfcParam.softStart = ENABLE;
        } 
        else
        {
            if (PFC_APP_pfcParam.softStart)
            {
                //check if Voltagereftemp is less than given reference voltage
                //and ramp it slowly
                if (PFC_APP_pfcVoltageLoopPI.reference <= PFC_VOLTREF)
                {
                    if(PFC_APP_pfcParam.rampRate == 0)
                    {
                        PFC_APP_pfcVoltageLoopPI.reference = PFC_APP_pfcVoltageLoopPI.reference + RAMP_COUNT;
                        PFC_APP_pfcParam.rampRate = RAMP_RATE;
                        
                    }
                }
                else
                {
                    PFC_APP_pfcVoltageLoopPI.reference = PFC_VOLTREF;
                    if(PFC_APP_busVoltage.measured >= PFC_VOLTREF)
                    {
                        PFC_APP_pfcParam.softStart = DISABLE;
                        PFC_APP_pfcParam.pfc_good = 1;
                    
                    }
                    
                }
                PFC_APP_pfcParam.rampRate--;
            }
        }

        //Calling Voltage PI function
        PFC_APP_VoltagePI();
        
        //Current reference Calculation is shown below
        // Current reference =
        //                (Voltage PI output)*(Vac Measured)*(1/Vavgsquare)*Kmul
        // where Kmul is calculated such that the current reference value
        // equals it maximum value at minimum line voltage
        // therefore Kmul = (Current reference maximum)/
        //            ((Voltage PI output max)*(Vac minimum)*(1/Vavgsquaremin))

       
         if(PFC_APP_acVoltage.avgSquare<5861) // 5861 = avg(85VACrms)^2
       {
           PFC_APP_acVoltage.avgSquare = 5861;
       }
       currentRef = (float) PFC_APP_pfcVoltageLoopPI.sum*PFC_APP_acVoltage.measured;
      
       PFC_APP_pfcCurrentLoopPI.reference =  (float) ((currentRef*KMUL)/PFC_APP_acVoltage.avgSquare) ;
       
       

        // Check if IndCurrent Reference exceeds 1
        if (PFC_APP_pfcCurrentLoopPI.reference > PFC_OVER_CURRENT_PEAK)
        {
            // If true, saturate it to 1
            PFC_APP_pfcCurrentLoopPI.reference = PFC_OVER_CURRENT_PEAK;
        }
        else if (PFC_APP_pfcCurrentLoopPI.reference <= 0)
        {
            PFC_APP_pfcCurrentLoopPI.reference = 0;
        }
//Calling sample correction  function to shape the current waveform under light loads
        //and near zero crossings
        SampleCorrection();
//        //Multiplying the measured AC current with sample correction factor
        PFC_APP_acCurrent.corrected = (float)(PFC_APP_acCurrent.measured*PFC_APP_pfcParam.sampleCorrection);
//
        //if Vavg greater than average of 200Vrms
        if(PFC_APP_acVoltage.avgOutput < VAVG_200V )
        {
           // Current Error Calculation
           PFC_APP_pfcCurrentLoopPI.error = PFC_APP_pfcCurrentLoopPI.reference - PFC_APP_acCurrent.corrected;
        }
        else
        {
           // Current Error Calculation
           PFC_APP_pfcCurrentLoopPI.error = PFC_APP_pfcCurrentLoopPI.reference- PFC_APP_acCurrent.measured;
        }
     
        //Calling current PI function
        PFC_APP_CurrentPI();

        //Current loop PI output and Multiplying it with OCperiod
        PFC_APP_pfcParam.duty  = (int16_t)(PFC_APP_pfcCurrentLoopPI.sum* MAX_PFC_DC);
       
//        PFC_APP_pfcParam.duty  = (int16_t)(PFC_APP_pfcVoltageLoopPI.sum* MAX_PFC_DC);
        
        if (PFC_APP_pfcParam.duty  >= MAX_PFC_DC)//Check if Final Duty is greater than 95% of max.duty
        {
            PFC_APP_pfcParam.duty = MAX_PFC_DC;
        }
        //Check if Final Duty is less than MINOCDUTY count, in this case minimum
        //duty is limited to MINOCDUTY,since PTG is triggered on the low to high
        //transition of OC, duty cant be '0'
        else if (PFC_APP_pfcParam.duty  <= MIN_PFC_DC)
        {
            PFC_APP_pfcParam.duty  = MIN_PFC_DC;
        }
     
               
        /*Loading calculated value of pfc duty to PDC register*/
       
        PLIB_MCPWM_ChannelPrimaryDutyCycleSet(MCPWM_ID_0, MCPWM_CHANNEL5,(uint16_t) PFC_APP_pfcParam.duty);
        //Sampling point is always chosen to be at half of duty
        PFC_APP_pfcParam.samplePoint = PFC_APP_pfcParam.duty>>1 ;
        PFC_APP_pfcParam.samplePoint = PFC_APP_pfcParam.samplePoint;// + SAMPLE_POINT_DMCI;
        if(PFC_APP_pfcParam.samplePoint > PFC_SAMPLE_POINT_MIN )
        {
            
            PLIB_MCPWM_ChannelPrimaryTriggerCompareSet(MCPWM_ID_0, MCPWM_CHANNEL5,(uint16_t) PFC_APP_pfcParam.samplePoint);
        }
        //if duty is less then minimum required sampling point(1us) limit to 0.5us
        //this is done to avoid sampling the switching ripple
        else
        {
            PLIB_MCPWM_ChannelPrimaryTriggerCompareSet(MCPWM_ID_0, MCPWM_CHANNEL5,(uint16_t) (PFC_SAMPLE_POINT_MIN >> 1));
        }
        
       
        //Calling PFC fault function
        pfcFaults();
        
   }    

    return;
}

void PFC_APP_PFC_Tasks (void )
{
    /* Check the application's current state. */
    switch ( pfc_appData.pfcState )
    {
        /* Application's initial state. */
        case PFC_APP_PFC_STATE_INIT:
        {
            if(!PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_D,PORTS_BIT_POS_8)\
                    && (pfc_appData.classBState ==PFC_APP_CLASSB_STATE_TEST_PASS))
            {
                 PFC_APP_pfcParam.pfcStart = 1;
            }
   
            if(PFC_APP_pfcParam.pfcStart == 1)
            {
				PFC_APP_pfcParam.firstPass = 1;
				DRV_MCPWM_Enable();
				PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E,PORTS_BIT_POS_13);
				pfc_appData.pfcState = PFC_APP_PFC_STATE_NOP;
            }
			
            break;
        }
		
        case PFC_APP_PFC_STATE_NOP:
        {
            //Do Nothing in this state, ADC ISR will run the control algorithm
            break;
        }
               
        case PFC_APP_PFC_STATE_STOP:
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
    void PFC_APP_Initialize ( void )

  Remarks:
    See prototype in pfc_app.h.
 */

void PFC_APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    pfc_appData.state = PFC_APP_STATE_INIT;

	pfc_appData.classBState =PFC_APP_CLASSB_STATE_INIT;
	pfc_appData.pfcState = PFC_APP_PFC_STATE_INIT;
	
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void PFC_APP_Tasks ( void )

  Remarks:
    See prototype in pfc_app.h.
 */

void PFC_APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( pfc_appData.state )
    {
        /* Application's initial state. */
        case PFC_APP_STATE_INIT:
        {
            bool appInitialized = true;
       
			DRV_ADC3_Open();
			DRV_ADC4_Open();
			DRV_ADC2_Open();
        
            if (appInitialized)
            {
            
            
                pfc_appData.state = PFC_APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case PFC_APP_STATE_SERVICE_TASKS:
        {
			PFC_APP_CLASSB_Tasks();
			PFC_APP_PFC_Tasks();
        
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
