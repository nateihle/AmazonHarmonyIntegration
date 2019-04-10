<#-- motor_control_ds_foc_pll_lv_int_opamp_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_motor_control_ds_foc_pll_lv_int_opamp_app_h_includes>
#include <math.h>
#include "peripheral/int/plib_int.h"
#include "peripheral/cdac/plib_cdac.h"
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_motor_control_ds_foc_pll_lv_int_opamp_app_h_type_definitions>

typedef struct 
{
    float   PWMPeriod;

    float   Vr1;
    float   Vr2;
    float   Vr3;
    float   Vr4;
    float   Vr5;
    float   Vr6;
} tSVGenParm;

typedef struct 
{
    float   VelRef;    						   	// Reference velocity
    float   IdRef;     						   	// Vd flux reference value
    float   IqRef;     						   	// Vq torque reference value
	float	qRefRamp;							// Ramp for speed reference value
	float   qDiff;								// Speed of the ramp
	float	IdRefFF;							// Feed forward Idref during Field Weakening
	float	IdRefPI;							// Field Weakening Idref PI output
	float 	IqRefmax;							// Maximum Q axis current
} tCtrlParm;

typedef struct 
{
    float   RevAngle;
    float   Angle;
    float   Sin;
    float   Cos;
    float   Ia;
    float   Ib;
    float   Ialpha;
    float   Ibeta;
    float   Id;
    float   Iq;
    float   Vd;
    float   FW_Vd;
    float   Vq;
    float   FW_Vq;
    float   Valpha;
    float   Vbeta;
    float   V1;
    float   V2;
    float   V3;
    float   DCBusVoltage;
	float   MaxPhaseVoltage;   
	float	VdqNorm;
	float	MaxVoltageCircleSquared;
	float   VdSquaredDenorm;
	float	VqSquaredDenorm;
	float	VqRefVoltage;

    
} tParkParm;

typedef struct 
{
    float   RevAngle;
    float   Angle;
    float   Sin;
    float   Cos;
} tSincosParm;

union  SYSTEM_STATUS_UNION 
{
    struct
    {
        unsigned RunMotor:1;  					/* run motor indication */
        unsigned OpenLoop:1;  					/* open loop/clsd loop indication */
        unsigned Btn1Pressed:1; 				/* btn 1 pressed indication */
        unsigned Btn2Pressed:1; 				/* btn 2 pressed indication */
        unsigned ChangeMode:1; 					/* mode changed indication - from open to clsd loop */
        unsigned ChangeSpeed:1; 				/* speed doubled indication */
        unsigned Btn2Debounce:1;
        unsigned ClassBPass:1;
        unsigned    :8;
    }bit;
    
	unsigned short Word;
}${APP_NAME?upper_case}_MC_CONTROL ;        						// general flags

typedef struct 
{
    float   qdSum;          
    float   qKp;
    float   qKi;
    float   qKc;
    float   qOutMax;
    float   qOutMin;
    float   qInRef; 
    float   qInMeas;
    float   qOut;
    float   qErr;
} tPIParm;

typedef struct 
{
    float	qDeltaT;      						// Integration constant
    float   qRho;    	    					// angle of estimation
    float   qRhoStateVar; 						// internal variable for angle
    float   qOmegaMr;     						// primary speed estimation
    float   qLastIalpha;  						// last value for Ialpha
    float   qLastIbeta;   						// last value for Ibeta
    float   qDIalpha;     						// difference Ialpha
    float   qDIbeta;      						// difference Ibeta
	float	qEsa;								// BEMF alpha
	float	qEsb;								// BEMF beta
	float	qEsd;								// BEMF d
	float	qEsq;								// BEMF q
    float	qVIndalpha;   						// dI*Ls/dt alpha
	float	qVIndbeta;    						// dI*Ls/dt beta
	float	qEsdf;        						// BEMF d filtered
	float	qEsqf;        						// BEMF q filtered
	float	qKfilterEsdq; 						// filter constant for d-q BEMF
	float   qVelEstim; 							// Estimated speed 
	float   qVelEstimFilterK; 					// Filter Konstant for Estimated speed 
    float   qLastValpha;  						// Value from last control step Ialpha 
    float   qLastVbeta;   						// Value from last control step Ibeta
	float   RhoOffset;            				// estima angle init offset
} tEstimParm;

typedef struct 
{
	float	qRs;								// Rs value - stator resistance
	float	qLsDt;								// Ls/dt value - stator inductand / dt - variable with speed
   	float	qInvKFi;	    					// InvKfi constant value ( InvKfi = Omega/BEMF )
} tMotorEstimParm;

#define     PBCLK_PWM                                           (SYS_CLK_FREQ/1)
//===============================================================
#define     PWM_FREQ                                            20000
#define     PWM_RELOAD_EDGE                                     ((PBCLK_PWM/PWM_FREQ))
#define     PWM_RELOAD_CENTER                                   (((PBCLK_PWM/PWM_FREQ)/2))
#define     DEADTIME_SEC                                        (float)0.000001
#define     DEADTIME_REG                                        DEADTIME_SEC*SYS_CLK_FREQ
#define     PWM_IOCON_CONFIG                                    0x0054C000    
//===============================================================
// Following parameters for MCLV-2 board
// Gain of opamp = 15
// shunt resistor = 0.025 ohms
// DC offset = 1.65V
// max current = x
// (x * 0.025 * 15) + 1.65V = 3.3V
// x = 4.4Amps
#define     MAX_BOARD_CURRENT                                   (float)(4.4)
#define     MAX_MOTOR_CURRENT                                   (float)(4.2)
#define     MAX_MOTOR_CURRENT_SQUARED                           (float)((float)MAX_MOTOR_CURRENT*(float)MAX_MOTOR_CURRENT)
#define     VREF_DAC_VALUE                                      (int) 2048
#define     ADC_CURRENT_SCALE                                   (float)(MAX_BOARD_CURRENT/(float)2048)

#define     CURRENT_LIMIT_CMP_REF                               (int)(((float)2048*(MAX_MOTOR_CURRENT/MAX_BOARD_CURRENT))+VREF_DAC_VALUE)
#define     MOTOR_PER_PHASE_RESISTANCE                          ((float)2.10)			// Resistance in Ohms
#define     MOTOR_PER_PHASE_INDUCTANCE                          ((float)0.00192)		// Inductance in Henrys
#define     MOTOR_PER_PHASE_INDUCTANCE_DIV_2_PI                 ((float)(MOTOR_PER_PHASE_INDUCTANCE/(2*M_PI)))	
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_Line_Line_KRPM_MECH   (float)7.24				// Back EMF Constant in Vpeak/KRPM
#define     NOPOLESPAIRS                                        5
#define     MAX_ADC_COUNT                                       (float)4095     // for 12-bit ADC
#define     MAX_ADC_INPUT_VOLTAGE                               (float)3.3      // volts


#define     DCBUS_SENSE_TOP_RESISTOR                            (float)30.0
#define     DCBUS_SENSE_BOTTOM_RESISTOR                         (float)2.0
#define     DCBUS_SENSE_RATIO                                   (float)(DCBUS_SENSE_BOTTOM_RESISTOR/(DCBUS_SENSE_BOTTOM_RESISTOR + DCBUS_SENSE_TOP_RESISTOR))
#define     VOLTAGE_ADC_TO_PHY_RATIO                            (float)(MAX_ADC_INPUT_VOLTAGE/(MAX_ADC_COUNT * DCBUS_SENSE_RATIO))
#define     BTN1                                                (PORTGbits.RG1) // BTN_1 -> Start Switch
#define     BTN2                                                (PORTCbits.RC7) // BTN_2   

#define     SINGLE_ELEC_ROT_RADS_PER_SEC                        (float)(2*M_PI)

#define     MAX_DUTY                                            (PWM_RELOAD_CENTER)
#define     LOOPTIME_SEC                                        (float)0.00005           // PWM Period - 50 uSec, 20Khz PWM

#define     LOCK_TIME_IN_SEC                                     2
#define     LOCK_COUNT_FOR_LOCK_TIME                            (unsigned int)((float)LOCK_TIME_IN_SEC/(float)LOOPTIME_SEC)
#define     END_SPEED_RPM                                       500 // Value in RPM

#define     END_SPEED_RPS                                       ((float)END_SPEED_RPM/60)

// In 1 second, the motor maked "END_SPEED_RPS" mechanical rotations
#define     RAMP_TIME_IN_SEC                                     3
#define     END_SPEED_RADS_PER_SEC_MECH                         (float)(END_SPEED_RPS * SINGLE_ELEC_ROT_RADS_PER_SEC)
#define     END_SPEED_RADS_PER_SEC_ELEC                         (float)(END_SPEED_RADS_PER_SEC_MECH * NOPOLESPAIRS)
#define     END_SPEED_RADS_PER_SEC_ELEC_IN_LOOPTIME             (float)(END_SPEED_RADS_PER_SEC_ELEC * LOOPTIME_SEC)
#define     OPENLOOP_RAMPSPEED_INCREASERATE                     (float)(END_SPEED_RADS_PER_SEC_ELEC_IN_LOOPTIME/(RAMP_TIME_IN_SEC/LOOPTIME_SEC))

/* end speed conveted to fit the startup ramp */
#define     Q_CURRENT_REF_OPENLOOP           1.8

/* Nominal speed of the motor in RPM */
#define     NOMINAL_SPEED_RPM                                   (float)2800 // Value in RPM
#define     NOMINAL_SPEED_RAD_PER_SEC_ELEC                      (float)(((NOMINAL_SPEED_RPM/60)*2*M_PI)*NOPOLESPAIRS) // Value in RPM
   
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPM_MECH        (float)((MOTOR_BACK_EMF_CONSTANT_Vpeak_Line_Line_KRPM_MECH/1.732)/1000)
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPS_MECH        (float)(MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPM_MECH * 60)
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_MECH (float)(MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RPS_MECH/(2*M_PI))
#define     MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC (float)(MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_MECH/NOPOLESPAIRS)
#define     INVKFi_BELOW_BASE_SPEED                               (float)(1/MOTOR_BACK_EMF_CONSTANT_Vpeak_PHASE_RAD_PER_SEC_ELEC)

/*Moving Average Filter based Current Offset Calculator Parameters */
#define 	MOVING_AVG_WINDOW_SIZE                              18   // moving average window sample size is 2^18
#define     CURRENT_OFFSET_MAX                                  2200 // current offset max limit
#define     CURRENT_OFFSET_MIN                                  1900 // current offset min limit
#define     CURRENT_OFFSET_INIT                                 2048 // // as the OPAMPs are biased at VDD/2, the estimate offset value is 2048 i.e. half of 4095 which is full scale value of a 12 bit ADC. 

/* PI controllers tuning values - */
//******** D Control Loop Coefficients *******
#define     D_CURRCNTR_PTERM                                    (0.005/MAX_MOTOR_CURRENT)
#define     D_CURRCNTR_ITERM                                    (0.00005 /MAX_MOTOR_CURRENT)
#define     D_CURRCNTR_CTERM                                    (0.999/MAX_MOTOR_CURRENT)
#define     D_CURRCNTR_OUTMAX                                    0.999

//******** Q Control Loop Coefficients *******
#define     Q_CURRCNTR_PTERM                                    (0.005/MAX_MOTOR_CURRENT)
#define     Q_CURRCNTR_ITERM                                    (0.00005 /MAX_MOTOR_CURRENT)
#define     Q_CURRCNTR_CTERM                                    (0.999/MAX_MOTOR_CURRENT)
#define     Q_CURRCNTR_OUTMAX                                    0.999

//*** Velocity Control Loop Coefficients *****
#define     SPEEDCNTR_PTERM                                      (0.05)
#define     SPEEDCNTR_ITERM                                     (0.0000006)
#define     SPEEDCNTR_CTERM                                      0.99
#define     SPEEDCNTR_OUTMAX                                     MAX_MOTOR_CURRENT


#define     KFILTER_ESDQ                                        (float)((float)400/(float)32767)
#define     KFILTER_VELESTIM                                    (float)((float)(1*374)/(float)32767)
#define     INITOFFSET_TRANS_OPEN_CLSD                          (float)((float)0x2000/(float)32767)

#define     FW_SPEED_RPM                                        (float)5000
#define     FW_SPEED_RAD_PER_SEC_ELEC                           (float)(((FW_SPEED_RPM/60)*2*M_PI)*NOPOLESPAIRS)
#define     SPEED_COMMAND_RAMP_UP_TIME                          (float)25.0 // seconds
#define     MAX_FW_NEGATIVE_ID_REF                              (float)(-1.6)

#define     POT_ADC_COUNT_FW_SPEED_RATIO                        (float)(FW_SPEED_RAD_PER_SEC_ELEC/MAX_ADC_COUNT)

extern  	tPIParm     										PIParmQ;        /* parms for PI controlers */
extern  	tPIParm     										PIParmD;        /* parms for PI controlers */
extern 		tParkParm											ParkParm;
extern 		tSincosParm											SincosParm;
extern 		tSVGenParm 											SVGenParm;    
extern 		float 												dPWM1, dPWM2, dPWM3;
extern 		tCtrlParm 											CtrlParm;

typedef 	signed int 											int32_t;
#define 	TOTAL_SINE_TABLE_ANGLE      						(2*M_PI)
#define 	TABLE_SIZE      									256
#define 	ANGLE_STEP      									(float)((float)TOTAL_SINE_TABLE_ANGLE/(float)TABLE_SIZE)

extern 		const float 										sineTable[TABLE_SIZE];
extern 		const float 										cosineTable[TABLE_SIZE];

extern 		tEstimParm 											EstimParm;
extern 		tMotorEstimParm 									MotorEstimParm;

extern      short                                               phaseCurrentA;
extern      short                                               phaseCurrentB;
extern      short                                               potReading;
typedef enum
{
	/* Application's state machine's initial state. */
	${APP_NAME?upper_case}_MC_STATE_INIT=0,
	${APP_NAME?upper_case}_MC_STATE_NOP,
    ${APP_NAME?upper_case}_MC_STATE_STOP,
} ${APP_NAME?upper_case}_MC_STATES;
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
<#macro macro_motor_control_ds_foc_pll_lv_int_opamp_app_h_data>
	${APP_NAME?upper_case}_MC_STATES mcState;
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
<#macro macro_motor_control_ds_foc_pll_lv_int_opamp_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_motor_control_ds_foc_pll_lv_int_opamp_app_h_function_declarations>
/* MC Core Routines */
void ${APP_NAME?upper_case}_MC_InvPark(void);
void ${APP_NAME?upper_case}_MC_CalcRefVec(void);
void ${APP_NAME?upper_case}_MC_Clarke(void);
void ${APP_NAME?upper_case}_MC_Park(void);
void ${APP_NAME?upper_case}_MC_CalcSVGen( void );
void ${APP_NAME?upper_case}_MC_CalcTimes( void );
void ${APP_NAME?upper_case}_MC_InitControlParameters(void);
void ${APP_NAME?upper_case}_MC_InitPI( tPIParm *pParm);
void ${APP_NAME?upper_case}_MC_CalcPI( tPIParm *pParm);
void ${APP_NAME?upper_case}_MC_SinCos(void);
void ${APP_NAME?upper_case}_MC_CalculateParkAngle(void);
void ${APP_NAME?upper_case}_MC_DoControl( void );

/*MC PLL Estimator Routines */
void ${APP_NAME?upper_case}_MC_Estim(void);
void ${APP_NAME?upper_case}_MC_InitEstimParm(void);

/* MC Debug Routines */
void ${APP_NAME?upper_case}_MC_DBG_Init(void);
void ${APP_NAME?upper_case}_MC_DBG_SyncComm(void);
void ${APP_NAME?upper_case}_MC_DBG_SnapUpdate(void);
void ${APP_NAME?upper_case}_MC_DBG_IDLECounter(void);

/* MC Peripheral Init Routines */
void ${APP_NAME?upper_case}_PWM_Initialize(void);
void ${APP_NAME?upper_case}_PWM_Enable(void);
void ${APP_NAME?upper_case}_PWM_Disable(void);
void ${APP_NAME?upper_case}_DAC_Initialize(void);

/*MC ISR Routines*/
void ${APP_NAME?upper_case}_MC_ADCISRTasks(void);
void ${APP_NAME?upper_case}_MC_FaultISRTasks(void);
</#macro>

<#macro macro_motor_control_ds_foc_pll_lv_int_opamp_app_h_states>
</#macro>

