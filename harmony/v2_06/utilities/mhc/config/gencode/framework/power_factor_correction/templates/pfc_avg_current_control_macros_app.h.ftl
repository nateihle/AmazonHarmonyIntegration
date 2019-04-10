<#-- pfc_avg_current_control_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_pfc_avg_current_control_app_h_includes>
#include <math.h>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_pfc_avg_current_control_app_h_type_definitions>

typedef struct
{
    float avgSum;
    float avgOutput;
    float avgSquare;
    uint16_t samples;
    float measured;
    float corrected;
        
}PFC_MEASURED_PARAM;
typedef struct
{
    float sum;
    float integralOut;
    float propOut;
    float prevIntegralOut;
    float error;
    float input;
    float prevError;
    float reference;
    int8_t saturationFlag;
    
}PFC_PI_CONTROLLER;

typedef struct
{
    uint16_t duty;
    uint16_t samplePoint;
    float sampleCorrection;  // US
    int8_t rampRate;
    int8_t voltLoopExeRate;
    int8_t softStart;
    int8_t pfc_good;
    int8_t adcSwitchCount;
    int8_t firstPass;
    int8_t overcurrent_faultCount;
    int8_t overvoltage_faultCount;
    int8_t faultBit;
	int8_t pfcStart;
    float read_var1;
}PFC_GENERAL_PARAM;

extern PFC_MEASURED_PARAM ${APP_NAME?upper_case}_busVoltage;
extern PFC_MEASURED_PARAM ${APP_NAME?upper_case}_acVoltage;
extern PFC_MEASURED_PARAM ${APP_NAME?upper_case}_acCurrent;

extern PFC_PI_CONTROLLER ${APP_NAME?upper_case}_pfcVoltageLoopPI;
extern PFC_PI_CONTROLLER ${APP_NAME?upper_case}_pfcCurrentLoopPI;
extern PFC_GENERAL_PARAM ${APP_NAME?upper_case}_pfcParam;
/************** GLOBAL DEFINITIONS ***********/
/* double */
#define PI 3.14159265358979323846
#define SQROOTOF2 1.414213562

#define ENABLE 1
#define DISABLE 0
#define     PWM_PFC_FREQ            80000   
//Average value of AC rectified voltage
#define AVG(x)   (float)(2.0*x*SQROOTOF2/PI)

/*==========PFC DEFINITIONS============*/	
// Base Value of Voltage for the System. This value is calculated as follows:
// With the resistor divider used to measure DCBUSSENSE signal, 3.3V
// is generated when an input of 453V is
// present in DCBUS.
#define PFC_FULLSCALE_VOLT	453.0
// Base Value of Current for the System
// This is how this value was calculated:Diff Operational amplifier of
// 3.1kOhm/(560ohm+39ohm) =5.175 So:
// Vinadc = Ishunt*Rshunt*5.175. Diff opamp output is biased at 1.65V hence maximum
// ADC input is +/-1.65V (Max ADC Input = 3.3V)
// and with Rshunt of 0.015 Ohm
// we have a maximum current = +/-21.3 A
#define PFC_FULLSCALE_CURR	21.3
#define PFC_ADC_CURR_SCALE (float)(PFC_FULLSCALE_CURR/(float)2048)
#define PFC_VOLTREF      385
#define PFC_VOLTREF_NORM  (PFC_VOLTREF/PFC_FULLSCALE_VOLT)
#define PFC_MIN_VOLTREF      127  //Corresponds to 90Vrms i.e 127Volts max
#define PFC_MIN_VOLTREF_NORM  (PFC_MIN_VOLTREF/PFC_FULLSCALE_VOLT)

#define HALF_ADC_COUNT                                      (float)2048     // for 12-bit ADC
#define HALF_ADC_INPUT_VOLTAGE                              (float)1.65      // volts    
#define PFC_MAX_ADC_INPUT_VOLTAGE							(float) 3.3  	// volts
#define PFC_MAX_ADC_COUNT 									(float) 4095
#define ACBUS_SENSE_TOP_RESISTOR                            (float)300
#define ACBUS_SENSE_BOTTOM_RESISTOR                         (float)1.1
#define ACBUS_SENSE_RATIO                                   (float)(ACBUS_SENSE_BOTTOM_RESISTOR/(ACBUS_SENSE_BOTTOM_RESISTOR + ACBUS_SENSE_TOP_RESISTOR))    
#define PFC_AC_VOLTAGE_ADC_TO_PHY_RATIO                     (float)(HALF_ADC_INPUT_VOLTAGE/(HALF_ADC_COUNT * ACBUS_SENSE_RATIO)) 
#define MAX_PFC_AC_VOLTAGE                                  (float) (HALF_ADC_INPUT_VOLTAGE/ACBUS_SENSE_RATIO)
    
#define PFC_DCBUS_SENSE_TOP_RESISTOR                            (float)285.0 // Although the resistance value on board is 300K, we choose 285K as
                                                                             // the resistance value appears to be varying with temperature
#define PFC_DCBUS_SENSE_BOTTOM_RESISTOR                         (float)2.2

#define AC_CURRENT_OFFSET                                   2060
#define AC_VOLTAGE_OFFSET                                   2070


#define PFC_DCBUS_SENSE_RATIO                                  (float)(PFC_DCBUS_SENSE_BOTTOM_RESISTOR/(PFC_DCBUS_SENSE_BOTTOM_RESISTOR + PFC_DCBUS_SENSE_TOP_RESISTOR))

#define PFC_DC_VOLTAGE_ADC_TO_PHY_RATIO                            (float)(PFC_MAX_ADC_INPUT_VOLTAGE/(PFC_MAX_ADC_COUNT * PFC_DCBUS_SENSE_RATIO))
	
	
/*PFC Fault Limits*/
//Output dc Over voltage
#define PFC_OVER_VOLTAGE 420
#define PFC_OVER_VOLTAGELIM_NORM (PFC_OVER_VOLTAGE/PFC_FULLSCALE_VOLT)
//Output dc Under Voltage
#define PFC_UNDER_VOLTAGE 300
#define PFC_UNDER_VOLTAGELIM_NORM (PFC_UNDER_VOLTAGE/PFC_FULLSCALE_VOLT)
//Average value of input current- corresponding to 12Arms
#define PFC_OVER_CURRENT_RMS 12
#define PFC_OVER_CURRENT_PEAK PFC_OVER_CURRENT_RMS*SQROOTOF2
#define PFC_OVER_CURRENT AVG(PFC_OVER_CURRENT_RMS)
    
#define PFC_OVER_CURRENTLIM_NORM (PFC_OVER_CURRENT/PFC_FULLSCALE_CURR)
//Average Value of input voltage- corresponding to 85Vrms
#define PFC_AC_UNDER_VOLTAGE AVG(85)
#define PFC_AC_UNDER_VOLTAGELIM_NORM (PFC_AC_UNDER_VOLTAGE/PFC_FULLSCALE_VOLT)
//Average Value of input voltage- corresponding to 265Vrms
#define PFC_AC_OVER_VOLTAGE AVG(265)
#define PFC_AC_OVER_VOLTAGELIM_NORM (PFC_AC_OVER_VOLTAGE/PFC_FULLSCALE_VOLT)

//Calcultion of Average voltages   
#define VAVG_88V_NORM  ((AVG(88)/PFC_FULLSCALE_VOLT))
#define VAVG_150V_NORM ((AVG(150)/PFC_FULLSCALE_VOLT))
#define VAVG_200V_NORM ((AVG(200)/PFC_FULLSCALE_VOLT))

#define VAVG_88V  ((AVG(88)))
#define VAVG_150V ((AVG(150)))
#define VAVG_200V ((AVG(200)))

#define PFC_SAMPLE_POINT_MIN 70
#define     PWM_PFC_PERIOD          ((120000000/PWM_PFC_FREQ))
#define     MIN_PFC_DC              0
#define     MAX_PFC_DC              0.95*PWM_PFC_PERIOD    
//Softsart ramp rate and ramp count
#define RAMP_COUNT 1
#define RAMP_RATE 100

//Voltage loop control frequency count    
#define COUNT_3KHz 15
//Approx 5Volts error
#define PFCVOLTERRMAX   5
#define PFCVOLTERRMAX_NORM   (PFCVOLTERRMAX/PFC_FULLSCALE_VOLT)

//KMUL is used as scaling constant    

#define KMUL  500//1000    
//It is used as a limit for gain scheduling    
#define PFCPIVOLTSUM 0.24 // *#* US: in dsPIC this value was 4000 which is Q15 for 0.24
    
/*=====LPF Coefficients=====*/
// Filter constant alpha
#define ALPHA 0.09
// Filter constant beta
#define BETA 0.91
// Normalized alpha
#define ALPHA_Q15 Q15(ALPHA)                   
#define BETA_Q15 Q15(BETA)	

/*=====PFC Voltage loop parameters=====*/
//Voltage loop Proportional constant


#define KP_V          0.01
//Voltage loop Integral constant


#define KI_V         0.000003

/*=====PFC Current loop parameters=====*/
//Current loop Proportional constant
#define KP_I          0.05 
//Current loop Integral constant
#define KI_I          0.035


extern int KP_CURRENT_DMCI;
extern int KI_CURRENT_DMCI;
extern int KP_VOLTAGE_DMCI;
extern int KI_VOLTAGE_DMCI;
extern int SAMPLE_POINT_DMCI;

typedef enum
{
	/* Application's state machine's initial state. */
	${APP_NAME?upper_case}_PFC_STATE_INIT=0,
	${APP_NAME?upper_case}_PFC_STATE_NOP,
    ${APP_NAME?upper_case}_PFC_STATE_STOP,
} ${APP_NAME?upper_case}_PFC_STATES;

</#macro>

<#--
// *****************************************************************************
/* Application Data
typedef struct
{
    /* The application's current state */
    ${APP_NAME?upper_case}_STATES state;

    /* TODO: Define any additional data used by the application. */
-->
<#macro macro_pfc_avg_current_control_app_h_data>
	${APP_NAME?upper_case}_PFC_STATES pfcState;
</#macro>
<#--
} ${APP_NAME?upper_case}_DATA;
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
-->
<#macro macro_pfc_avg_current_control_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_pfc_avg_current_control_app_h_function_declarations>
/* PFC Routines */
extern void ${APP_NAME?upper_case}_AverageVoltageCalc(void);
extern void ${APP_NAME?upper_case}_pfc_control(void);
extern void ${APP_NAME?upper_case}_AverageCurrentCalc(void);
extern void ${APP_NAME?upper_case}_pfcFaults(void);
extern void ${APP_NAME?upper_case}_VoltagePI(void);
extern void ${APP_NAME?upper_case}_CurrentPI(void);
extern void ${APP_NAME?upper_case}_pfcInit(void);
extern void ${APP_NAME?upper_case}_SampleCorrection(void);

</#macro>

<#macro macro_pfc_avg_current_control_app_h_states>


</#macro>

