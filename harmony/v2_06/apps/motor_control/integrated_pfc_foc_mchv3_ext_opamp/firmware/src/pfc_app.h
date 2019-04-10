/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    pfc_app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _PFC_APP_H
#define _PFC_APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "classb/classb.h"
#include <math.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
typedef enum
{
	/* Application's state machine's initial state. */
	PFC_APP_CLASSB_STATE_INIT=0,
	PFC_APP_CLASSB_STATE_FLASH_CRC_TEST,
    PFC_APP_CLASSB_STATE_CHECKER_BOARD_RAM_TEST,
    PFC_APP_CLASSB_STATE_RAM_MARCHB_TEST,
    PFC_APP_CLASSB_STATE_RAM_MARCHC_TEST,
    PFC_APP_CLASSB_STATE_RAM_MARCHC_STACK_TEST,
    PFC_APP_CLASSB_STATE_CLOCK_TEST,
    PFC_APP_CLASSB_STATE_CPU_PC_TEST,
    PFC_APP_CLASSB_STATE_CPU_REGISTERS_TEST,
    PFC_APP_CLASSB_STATE_TEST_PASS,
    PFC_APP_CLASSB_STATE_TEST_FAIL,
} PFC_APP_CLASSB_STATES;

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

extern PFC_MEASURED_PARAM PFC_APP_busVoltage;
extern PFC_MEASURED_PARAM PFC_APP_acVoltage;
extern PFC_MEASURED_PARAM PFC_APP_acCurrent;

extern PFC_PI_CONTROLLER PFC_APP_pfcVoltageLoopPI;
extern PFC_PI_CONTROLLER PFC_APP_pfcCurrentLoopPI;
extern PFC_GENERAL_PARAM PFC_APP_pfcParam;
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
	PFC_APP_PFC_STATE_INIT=0,
	PFC_APP_PFC_STATE_NOP,
    PFC_APP_PFC_STATE_STOP,
} PFC_APP_PFC_STATES;


// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	PFC_APP_STATE_INIT=0,
	PFC_APP_STATE_SERVICE_TASKS,

	/* TODO: Define states used by the application state machine. */




} PFC_APP_STATES;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    PFC_APP_STATES state;

    /* TODO: Define any additional data used by the application. */
	PFC_APP_CLASSB_STATES classBState;
	PFC_APP_PFC_STATES pfcState;

} PFC_APP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void PFC_APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    PFC_APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void PFC_APP_Initialize ( void );


/*******************************************************************************
  Function:
    void PFC_APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    PFC_APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void PFC_APP_Tasks( void );

/* PFC Routines */
extern void PFC_APP_AverageVoltageCalc(void);
extern void PFC_APP_pfc_control(void);
extern void PFC_APP_AverageCurrentCalc(void);
extern void PFC_APP_pfcFaults(void);
extern void PFC_APP_VoltagePI(void);
extern void PFC_APP_CurrentPI(void);
extern void PFC_APP_pfcInit(void);
extern void PFC_APP_SampleCorrection(void);


#endif /* _PFC_APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

