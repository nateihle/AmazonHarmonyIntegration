<#-- pfc_avg_current_control_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_pfc_avg_current_control_app_c_includes>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data
*/
-->
<#macro macro_pfc_avg_current_control_app_c_global_data>

PFC_GENERAL_PARAM ${APP_NAME?upper_case}_pfcParam;

PFC_MEASURED_PARAM ${APP_NAME?upper_case}_acVoltage;
PFC_MEASURED_PARAM ${APP_NAME?upper_case}_acCurrent;
PFC_MEASURED_PARAM ${APP_NAME?upper_case}_busVoltage;


PFC_PI_CONTROLLER ${APP_NAME?upper_case}_pfcVoltageLoopPI;
PFC_PI_CONTROLLER ${APP_NAME?upper_case}_pfcCurrentLoopPI;

inline static float limit(float x, float min, float max);

float KP_CURRENT= KP_I;
float KI_CURRENT=KI_I;
float KP_VOLTAGE=KP_V;
float KI_VOLTAGE=KI_V;
</#macro>

<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_pfc_avg_current_control_app_c_callback_functions>
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_pfc_avg_current_control_app_c_local_functions>

void ${APP_NAME?upper_case}_pfcInit(void)
{
  
    ${APP_NAME?upper_case}_pfcParam.firstPass = ENABLE;
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
inline void ${APP_NAME?upper_case}_AverageVoltageCalc(void)
{
    // Accumulate Sigma(Vac)
    ${APP_NAME?upper_case}_acVoltage.avgSum = ${APP_NAME?upper_case}_acVoltage.avgSum + ${APP_NAME?upper_case}_acVoltage.measured;
    // Accumulate Frequency Counts
    ${APP_NAME?upper_case}_acVoltage.samples++;
    // Check if Frequency Counts reach a value corresponding to ~12Hz
    if (${APP_NAME?upper_case}_acVoltage.samples >= 4096) 
    {
        //Calculating Vac Average by dividing
        //Sigma Vac with 4096(approx. averaging for 10cycles)
        ${APP_NAME?upper_case}_acVoltage.avgOutput = (float) (${APP_NAME?upper_case}_acVoltage.avgSum/4096);
        // Calculates Vacavg^2, result  
        ${APP_NAME?upper_case}_acVoltage.avgSquare = (float)(${APP_NAME?upper_case}_acVoltage.avgOutput*${APP_NAME?upper_case}_acVoltage.avgOutput);
        // Clear Sigma(Vac) to compute new value
        ${APP_NAME?upper_case}_acVoltage.avgSum = 0;
        // Clear Freq Counts to compute new value
        ${APP_NAME?upper_case}_acVoltage.samples = 0; 
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
inline void ${APP_NAME?upper_case}_AverageCurrentCalc(void)
{
    // Accumulate Sigma(Vac)
    ${APP_NAME?upper_case}_acCurrent.avgSum = ${APP_NAME?upper_case}_acCurrent.avgSum + ${APP_NAME?upper_case}_acCurrent.measured;
    // Accumulate Frequency Counts
    ${APP_NAME?upper_case}_acCurrent.samples++;
    // Check if Frequency Counts reach a value corresponding to ~12Hz
    if (${APP_NAME?upper_case}_acCurrent.samples >= 4096) 
    {
        // Calculating Iac Average by dividing Sigma Iac with 4096
        // (approx. averaging for 10cycles)
        ${APP_NAME?upper_case}_acCurrent.avgOutput = (float) (${APP_NAME?upper_case}_acCurrent.avgSum/4096);
        // Clear Sigma(Iac) to compute new value
        ${APP_NAME?upper_case}_acCurrent.avgSum = 0; 
        ${APP_NAME?upper_case}_acCurrent.samples = 0;
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
     if(${APP_NAME?upper_case}_pfcParam.softStart == DISABLE)
     {
         //DC bus under voltage and over voltage faults
           if(${APP_NAME?upper_case}_busVoltage.measured >= PFC_OVER_VOLTAGE)
            {
                 ${APP_NAME?upper_case}_pfcParam.overvoltage_faultCount++;
               if(${APP_NAME?upper_case}_pfcParam.overvoltage_faultCount  > 200)
               {
                   ${APP_NAME?upper_case}_pfcParam.faultBit = 1;
                   ${APP_NAME?upper_case}_pfcParam.overvoltage_faultCount=0;
               }
                
            }
         //AC input under voltage and over voltage faults
           else if((${APP_NAME?upper_case}_acVoltage.avgOutput >= PFC_AC_OVER_VOLTAGE || ${APP_NAME?upper_case}_acVoltage.avgOutput <= PFC_AC_UNDER_VOLTAGE))
            {
                ${APP_NAME?upper_case}_pfcParam.faultBit = 1;
            }
           //AC input over current fault
           else if(${APP_NAME?upper_case}_acCurrent.avgOutput >= PFC_OVER_CURRENT)
            {
               ${APP_NAME?upper_case}_pfcParam.overcurrent_faultCount++;
               if(${APP_NAME?upper_case}_pfcParam.overcurrent_faultCount  > 200)
               {
                   ${APP_NAME?upper_case}_pfcParam.faultBit = 1;
                   ${APP_NAME?upper_case}_pfcParam.overcurrent_faultCount=0;
               }
           }
           else
           {
               ${APP_NAME?upper_case}_pfcParam.overvoltage_faultCount=0;
               ${APP_NAME?upper_case}_pfcParam.overcurrent_faultCount=0;
           }
           if(${APP_NAME?upper_case}_pfcParam.faultBit == 1)
           {
                DRV_MCPWM_Disable();
           }
        }
}

/**

in this Function the voltage loop PI related calculations are performed

Summary: Function to perform Outer loop Control

@param  void
@return PI output
*/
inline void ${APP_NAME?upper_case}_VoltagePI(void)
{
        
 //Run voltage control loop for every 3Khz
        if (${APP_NAME?upper_case}_pfcParam.voltLoopExeRate >= COUNT_3KHz)
        {
              
       // Filter for eliminating 100Hz/120Hz ripple
        ${APP_NAME?upper_case}_pfcVoltageLoopPI.error = ${APP_NAME?upper_case}_pfcVoltageLoopPI.reference - ${APP_NAME?upper_case}_busVoltage.measured;

        ${APP_NAME?upper_case}_pfcVoltageLoopPI.error = (float) ((${APP_NAME?upper_case}_pfcVoltageLoopPI.error*ALPHA)+
                              (${APP_NAME?upper_case}_pfcVoltageLoopPI.prevError*BETA));
        //End of Filter Code
        ${APP_NAME?upper_case}_pfcVoltageLoopPI.prevError = ${APP_NAME?upper_case}_pfcVoltageLoopPI.error;
      
            //Calculating proportional output of voltage PI

            ${APP_NAME?upper_case}_pfcVoltageLoopPI.propOut = (float)(KP_VOLTAGE*${APP_NAME?upper_case}_pfcVoltageLoopPI.error);
            
                                //Check if voltage PI output is saturated
                if (${APP_NAME?upper_case}_pfcVoltageLoopPI.saturationFlag == 0)
                {
                    //Calculating Integral output of voltage PI
                            ${APP_NAME?upper_case}_pfcVoltageLoopPI.integralOut = ${APP_NAME?upper_case}_pfcVoltageLoopPI.prevIntegralOut +
                            (float) (KI_VOLTAGE*${APP_NAME?upper_case}_pfcVoltageLoopPI.error);
                }
 
            //Check if Voltage PI integral output is saturated to maximum or minimum
            ${APP_NAME?upper_case}_pfcVoltageLoopPI.integralOut = limit(${APP_NAME?upper_case}_pfcVoltageLoopPI.integralOut,-1,1);
            
            ${APP_NAME?upper_case}_pfcVoltageLoopPI.prevIntegralOut = ${APP_NAME?upper_case}_pfcVoltageLoopPI.integralOut;
            //Addition of Voltage Loop PI,Integral and Proportional Outputs
            ${APP_NAME?upper_case}_pfcVoltageLoopPI.sum = ${APP_NAME?upper_case}_pfcVoltageLoopPI.integralOut + ${APP_NAME?upper_case}_pfcVoltageLoopPI.propOut;

            //Check if Voltage PI output is saturated
            if (${APP_NAME?upper_case}_pfcVoltageLoopPI.sum >= (0.999))
            {
                ${APP_NAME?upper_case}_pfcVoltageLoopPI.sum = (0.999);
                //if Voltage PI output is saturated make Voltagesaturationflag = 1
                ${APP_NAME?upper_case}_pfcVoltageLoopPI.saturationFlag = 1;
            }
            else if (${APP_NAME?upper_case}_pfcVoltageLoopPI.sum <= 0)
            {
                ${APP_NAME?upper_case}_pfcVoltageLoopPI.sum = 0;
                ${APP_NAME?upper_case}_pfcVoltageLoopPI.saturationFlag = 1;
            }
            else
            {
                ${APP_NAME?upper_case}_pfcVoltageLoopPI.saturationFlag = 0;
            }

            ${APP_NAME?upper_case}_pfcParam.voltLoopExeRate  = 0;
        }
        else
        {
            ${APP_NAME?upper_case}_pfcParam.voltLoopExeRate ++;
            ${APP_NAME?upper_case}_pfcVoltageLoopPI.sum = ${APP_NAME?upper_case}_pfcVoltageLoopPI.sum;
        }
}

/**

in this Function the current loop PI related calculations are performed

Summary: Function to perform inner loop Control

@param  void
@return PI output
*/
inline void ${APP_NAME?upper_case}_CurrentPI(void)
{

          ${APP_NAME?upper_case}_pfcCurrentLoopPI.propOut = (float) (KP_CURRENT*${APP_NAME?upper_case}_pfcCurrentLoopPI.error);      

      ${APP_NAME?upper_case}_pfcCurrentLoopPI.integralOut = (float)(KI_CURRENT*${APP_NAME?upper_case}_pfcCurrentLoopPI.error); 
      ${APP_NAME?upper_case}_pfcCurrentLoopPI.integralOut = ${APP_NAME?upper_case}_pfcCurrentLoopPI.integralOut + (float) ${APP_NAME?upper_case}_pfcCurrentLoopPI.prevIntegralOut;
        //Check if Current PI integral output is saturated to maximum or minimum        
            ${APP_NAME?upper_case}_pfcCurrentLoopPI.integralOut = limit(${APP_NAME?upper_case}_pfcCurrentLoopPI.integralOut,-0.998,0.998);

        //Addition of Current Loop PI,Integral and Proportional Outputs
        ${APP_NAME?upper_case}_pfcCurrentLoopPI.sum =  ${APP_NAME?upper_case}_pfcCurrentLoopPI.propOut + ${APP_NAME?upper_case}_pfcCurrentLoopPI.integralOut;
        //${APP_NAME?upper_case}_pfcCurrentLoopPI.sum =  ${APP_NAME?upper_case}_pfcCurrentLoopPI.propOut;
        //Check if Current PI output is saturated to maximum or minimum
        ${APP_NAME?upper_case}_pfcCurrentLoopPI.sum = limit(${APP_NAME?upper_case}_pfcCurrentLoopPI.sum, 0 , 1);
        
        ${APP_NAME?upper_case}_pfcCurrentLoopPI.prevIntegralOut = ${APP_NAME?upper_case}_pfcCurrentLoopPI.integralOut;
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
    
    temp = (float)(${APP_NAME?upper_case}_busVoltage.measured - ${APP_NAME?upper_case}_acVoltage.measured);
    if(temp > 0)
    {
        //Check if Vdc measured is less than '1'
        if(${APP_NAME?upper_case}_busVoltage.measured >  0)
        {
            ${APP_NAME?upper_case}_pfcParam.read_var1 =(float)(temp/${APP_NAME?upper_case}_busVoltage.measured);
            //Dividing Vdc 
            temp = (float)( ${APP_NAME?upper_case}_busVoltage.measured/temp) ;
        }

    
        if(temp > 0)
        {
            //Calculating sample correction factor
            ${APP_NAME?upper_case}_pfcParam.sampleCorrection = (${APP_NAME?upper_case}_pfcCurrentLoopPI.sum*temp) ;
        }

        if(${APP_NAME?upper_case}_pfcParam.sampleCorrection <= 0)
        {
            ${APP_NAME?upper_case}_pfcParam.sampleCorrection = 1;
        }
        
        if(${APP_NAME?upper_case}_pfcParam.sampleCorrection >= 1)
        {
            ${APP_NAME?upper_case}_pfcParam.sampleCorrection = 1;
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
inline void ${APP_NAME?upper_case}_pfc_control(void)
{
    float currentRef;
    float currentMeasured;
      
  
    ${APP_NAME?upper_case}_busVoltage.measured = (float)(PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1 , ADCHS_AN10) * PFC_DC_VOLTAGE_ADC_TO_PHY_RATIO);
    
   
    currentMeasured = (float)(PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1 , ADCHS_AN0));
    
    ${APP_NAME?upper_case}_acCurrent.measured = (float)((currentMeasured - AC_CURRENT_OFFSET)*PFC_ADC_CURR_SCALE); //2026

   if (${APP_NAME?upper_case}_acCurrent.measured <= 0)
   {
       ${APP_NAME?upper_case}_acCurrent.measured =(float)PFC_ADC_CURR_SCALE;  
   }
    
    ${APP_NAME?upper_case}_acVoltage.measured = (float)((PLIB_ADCHS_AnalogInputResultGet(DRV_ADC_ID_1 , ADCHS_AN4)- AC_VOLTAGE_OFFSET) * PFC_AC_VOLTAGE_ADC_TO_PHY_RATIO);
    
        if (${APP_NAME?upper_case}_acVoltage.measured >= 330)
    {
        ${APP_NAME?upper_case}_acVoltage.measured = 330;
    }

    //Average input Voltage Calculation
    ${APP_NAME?upper_case}_AverageVoltageCalc();
    
    //Average input Current Calculation
    ${APP_NAME?upper_case}_AverageCurrentCalc();

    if(((${APP_NAME?upper_case}_pfcParam.firstPass == ENABLE && ${APP_NAME?upper_case}_acVoltage.avgOutput >= VAVG_88V) || (${APP_NAME?upper_case}_pfcParam.firstPass == DISABLE && ${APP_NAME?upper_case}_busVoltage.measured >= PFC_MIN_VOLTREF))&& ${APP_NAME?upper_case}_pfcParam.faultBit==0)
    {

        if (${APP_NAME?upper_case}_pfcParam.firstPass)
        {
            ${APP_NAME?upper_case}_pfcVoltageLoopPI.reference = ${APP_NAME?upper_case}_busVoltage.measured;
            ${APP_NAME?upper_case}_pfcParam.firstPass = DISABLE;
            ${APP_NAME?upper_case}_pfcParam.softStart = ENABLE;
        } 
        else
        {
            if (${APP_NAME?upper_case}_pfcParam.softStart)
            {
                //check if Voltagereftemp is less than given reference voltage
                //and ramp it slowly
                if (${APP_NAME?upper_case}_pfcVoltageLoopPI.reference <= PFC_VOLTREF)
                {
                    if(${APP_NAME?upper_case}_pfcParam.rampRate == 0)
                    {
                        ${APP_NAME?upper_case}_pfcVoltageLoopPI.reference = ${APP_NAME?upper_case}_pfcVoltageLoopPI.reference + RAMP_COUNT;
                        ${APP_NAME?upper_case}_pfcParam.rampRate = RAMP_RATE;
                        
                    }
                }
                else
                {
                    ${APP_NAME?upper_case}_pfcVoltageLoopPI.reference = PFC_VOLTREF;
                    if(${APP_NAME?upper_case}_busVoltage.measured >= PFC_VOLTREF)
                    {
                        ${APP_NAME?upper_case}_pfcParam.softStart = DISABLE;
                        ${APP_NAME?upper_case}_pfcParam.pfc_good = 1;
                    
                    }
                    
                }
                ${APP_NAME?upper_case}_pfcParam.rampRate--;
            }
        }

        //Calling Voltage PI function
        ${APP_NAME?upper_case}_VoltagePI();
        
        //Current reference Calculation is shown below
        // Current reference =
        //                (Voltage PI output)*(Vac Measured)*(1/Vavgsquare)*Kmul
        // where Kmul is calculated such that the current reference value
        // equals it maximum value at minimum line voltage
        // therefore Kmul = (Current reference maximum)/
        //            ((Voltage PI output max)*(Vac minimum)*(1/Vavgsquaremin))

       
         if(${APP_NAME?upper_case}_acVoltage.avgSquare<5861) // 5861 = avg(85VACrms)^2
       {
           ${APP_NAME?upper_case}_acVoltage.avgSquare = 5861;
       }
       currentRef = (float) ${APP_NAME?upper_case}_pfcVoltageLoopPI.sum*${APP_NAME?upper_case}_acVoltage.measured;
      
       ${APP_NAME?upper_case}_pfcCurrentLoopPI.reference =  (float) ((currentRef*KMUL)/${APP_NAME?upper_case}_acVoltage.avgSquare) ;
       
       

        // Check if IndCurrent Reference exceeds 1
        if (${APP_NAME?upper_case}_pfcCurrentLoopPI.reference > PFC_OVER_CURRENT_PEAK)
        {
            // If true, saturate it to 1
            ${APP_NAME?upper_case}_pfcCurrentLoopPI.reference = PFC_OVER_CURRENT_PEAK;
        }
        else if (${APP_NAME?upper_case}_pfcCurrentLoopPI.reference <= 0)
        {
            ${APP_NAME?upper_case}_pfcCurrentLoopPI.reference = 0;
        }
//Calling sample correction  function to shape the current waveform under light loads
        //and near zero crossings
        SampleCorrection();
//        //Multiplying the measured AC current with sample correction factor
        ${APP_NAME?upper_case}_acCurrent.corrected = (float)(${APP_NAME?upper_case}_acCurrent.measured*${APP_NAME?upper_case}_pfcParam.sampleCorrection);
//
        //if Vavg greater than average of 200Vrms
        if(${APP_NAME?upper_case}_acVoltage.avgOutput < VAVG_200V )
        {
           // Current Error Calculation
           ${APP_NAME?upper_case}_pfcCurrentLoopPI.error = ${APP_NAME?upper_case}_pfcCurrentLoopPI.reference - ${APP_NAME?upper_case}_acCurrent.corrected;
        }
        else
        {
           // Current Error Calculation
           ${APP_NAME?upper_case}_pfcCurrentLoopPI.error = ${APP_NAME?upper_case}_pfcCurrentLoopPI.reference- ${APP_NAME?upper_case}_acCurrent.measured;
        }
     
        //Calling current PI function
        ${APP_NAME?upper_case}_CurrentPI();

        //Current loop PI output and Multiplying it with OCperiod
        ${APP_NAME?upper_case}_pfcParam.duty  = (int16_t)(${APP_NAME?upper_case}_pfcCurrentLoopPI.sum* MAX_PFC_DC);
       
//        ${APP_NAME?upper_case}_pfcParam.duty  = (int16_t)(${APP_NAME?upper_case}_pfcVoltageLoopPI.sum* MAX_PFC_DC);
        
        if (${APP_NAME?upper_case}_pfcParam.duty  >= MAX_PFC_DC)//Check if Final Duty is greater than 95% of max.duty
        {
            ${APP_NAME?upper_case}_pfcParam.duty = MAX_PFC_DC;
        }
        //Check if Final Duty is less than MINOCDUTY count, in this case minimum
        //duty is limited to MINOCDUTY,since PTG is triggered on the low to high
        //transition of OC, duty cant be '0'
        else if (${APP_NAME?upper_case}_pfcParam.duty  <= MIN_PFC_DC)
        {
            ${APP_NAME?upper_case}_pfcParam.duty  = MIN_PFC_DC;
        }
     
               
        /*Loading calculated value of pfc duty to PDC register*/
       
        PLIB_MCPWM_ChannelPrimaryDutyCycleSet(MCPWM_ID_0, MCPWM_CHANNEL5,(uint16_t) ${APP_NAME?upper_case}_pfcParam.duty);
        //Sampling point is always chosen to be at half of duty
        ${APP_NAME?upper_case}_pfcParam.samplePoint = ${APP_NAME?upper_case}_pfcParam.duty>>1 ;
        ${APP_NAME?upper_case}_pfcParam.samplePoint = ${APP_NAME?upper_case}_pfcParam.samplePoint;// + SAMPLE_POINT_DMCI;
        if(${APP_NAME?upper_case}_pfcParam.samplePoint > PFC_SAMPLE_POINT_MIN )
        {
            
            PLIB_MCPWM_ChannelPrimaryTriggerCompareSet(MCPWM_ID_0, MCPWM_CHANNEL5,(uint16_t) ${APP_NAME?upper_case}_pfcParam.samplePoint);
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

void ${APP_NAME?upper_case}_PFC_Tasks (void )
{
    /* Check the application's current state. */
    switch ( ${APP_NAME?lower_case}Data.pfcState )
    {
        /* Application's initial state. */
        case ${APP_NAME?upper_case}_PFC_STATE_INIT:
        {
            if(!PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_D,PORTS_BIT_POS_8))
            {
                 ${APP_NAME?upper_case}_pfcParam.pfcStart = 1;
            }
   
            if(${APP_NAME?upper_case}_pfcParam.pfcStart == 1)
            {
				${APP_NAME?upper_case}_pfcParam.firstPass = 1;
				DRV_MCPWM_Enable();
				PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_E,PORTS_BIT_POS_13);
				${APP_NAME?lower_case}Data.pfcState = ${APP_NAME?upper_case}_PFC_STATE_NOP;
            }
			
            break;
        }
		
        case ${APP_NAME?upper_case}_PFC_STATE_NOP:
        {
            //Do Nothing in this state, ADC ISR will run the control algorithm
            break;
        }
               
        case ${APP_NAME?upper_case}_PFC_STATE_STOP:
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
</#macro>

<#--
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Initialize ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_INIT;
-->
<#macro macro_pfc_avg_current_control_app_c_initialize>
	${APP_NAME?lower_case}Data.pfcState = ${APP_NAME?upper_case}_PFC_STATE_INIT;
	
</#macro>

<#--
}


/******************************************************************************
  Function:
    void ${APP_NAME?upper_case}_Tasks ( void )

  Remarks:
    See prototype in ${APP_NAME?lower_case}.h.
 */

void ${APP_NAME?upper_case}_Tasks ( void )
{
-->
<#macro macro_pfc_avg_current_control_app_c_tasks_data>
</#macro>

<#--
    /* Check the application's current state. */
    switch ( ${APP_NAME?lower_case}Data.state )
    {
        /* Application's initial state. */
        case ${APP_NAME?upper_case}_STATE_INIT:
        {
            bool appInitialized = true;
-->   
<#macro macro_pfc_avg_current_control_app_c_tasks_state_init>
			DRV_ADC3_Open();
			DRV_ADC4_Open();
			DRV_ADC2_Open();
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_pfc_avg_current_control_app_c_tasks_calls_after_init>
</#macro>

<#--            /* Advance to the next state */
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_pfc_avg_current_control_app_c_tasks_state_service_tasks>
			${APP_NAME?upper_case}_PFC_Tasks();
</#macro>

<#--        
            break;
        }

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
-->

<#macro macro_pfc_avg_current_control_app_c_tasks_states>
</#macro>
