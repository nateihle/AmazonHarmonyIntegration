<#-- rtdm_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_rtdm_app_h_includes>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_rtdm_app_h_type_definitions>

#define RTDM_RXBUFFERSIZE	32		// This is the buffer size used by RTDM to handle messaages 
#define RTDM_MAX_XMIT_LEN   0x1000	//This the size in bytes of the max num of bytes allowed in 
									//the RTDM protocol Frame
#define RTDM_POLLING		YES		// This defines the mode that RTDM will be operating in 
									//user's application. If it is YES then the user should place the 
									//RTDM_ProcessMsgs()	function in the main loop. 
									//In order to make sure that the messages are being preoccessed
									// it is recommended that the main loop always polls this 
/****************************************************/
/* DEFINITIONS FOR  RTDM captured variables */
/****************************************************/


     
    #define DATA_BUFFER_SIZE 50  //Size in 16-bit Words of the snap */
                                  // the value depends on the dsPIC mem
    #define SNAPDELAY	5 // In number of PWM Interrupts
    #define	DEBUG_VARIABLE1		ParkParm.Ia*1000    //Current in mA
    #define	DEBUG_VARIABLE2		ParkParm.Ib*1000    //Current in mA
    #define DEBUG_VARIABLE3		(EstimParm.qVelEstim*60)/(2*M_PI*NOPOLESPAIRS) //Translating Speed in Rad/sec to  RPM
    #define DEBUG_VARIABLE4		EstimParm.qRho*57.28//Converting angle from radians to degrees => 360/2*PI = 57.28

/****************************************************/			


/*+++++++++++++++++++++++++++++++ RTDM Variables ++++++++++++++++++++++++++++++++++++++++*/
/* Received data is stored in array RTDMRxBuffer  */
extern unsigned char RTDMRxBuffer[RTDM_RXBUFFERSIZE];
extern unsigned char * RTDMRxBufferLoLimit;
extern unsigned char * RTDMRxBufferHiLimit;
extern unsigned char * RTDMRxBufferIndex;
extern unsigned char * RTDMRxBufferStartMsgPointer;
extern unsigned char * RTDMRxBufferEndMsgPointer;
/* Data to be transmitted using UART communication module */
extern const unsigned char RTDMTxdata[] ;
extern const unsigned char RTDMSanityCheckOK[]; 
extern const unsigned char RTDMWriteMemoryOK[];
extern const unsigned char RTDMErrorIllegalFunction[];
extern unsigned char RTDMErrorFrame[] ;
extern DRV_HANDLE RTDM_UART_HANDLE;
/* Temp variables used to calculate the CRC16*/
extern unsigned int RTDMcrcTemp,RTDMcrcTempH,RTDMcrcTempL;
typedef struct DMCIFlags{
    		    unsigned Recorder : 1;	// Flag needs to be set to start buffering data
				unsigned StartStop : 1;
    			unsigned unused : 14;  
    } DMCIFLAGS;
/*Structure enclosing the RTDM flags*/
typedef struct RTDMFlags {
			unsigned MessageReceived  :		1;
			unsigned TransmitNow	  :		1;
			unsigned unused :		14;     
		}	RTDMFLAGS; 

    
extern    DMCIFLAGS DMCIFlags;
extern 	  RTDMFLAGS	RTDMFlags;

extern int SpeedReference;
extern signed short int SnapShotDelay;
extern    int RecorderBuffer1[DATA_BUFFER_SIZE] __attribute__ ((aligned));
extern    int RecorderBuffer2[DATA_BUFFER_SIZE] __attribute__ ((aligned));
extern    int RecorderBuffer3[DATA_BUFFER_SIZE] __attribute__ ((aligned));
extern    int RecorderBuffer4[DATA_BUFFER_SIZE] __attribute__ ((aligned));

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
<#macro macro_rtdm_app_h_data>
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
<#macro macro_rtdm_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_rtdm_app_h_function_declarations>
/**********************  RTDM FUNCTIONS **************************/
void ${APP_NAME?upper_case}_RecBufferUpdate();
int ${APP_NAME?upper_case}_RTDM_ProcessMsgs();
int ${APP_NAME?upper_case}_RTDM_Start();
unsigned int ${APP_NAME?upper_case}_RTDM_CumulativeCrc16 (unsigned char *buf, unsigned int u16Length, unsigned int u16CRC);
void  ${APP_NAME?upper_case}_RTDMRXInterruptTasks ();
void APP_MC_DBG_Init(void);
void APP_MC_DBG_SnapStart(void);
void ${APP_NAME?upper_case}_SnapUpdate(void);
</#macro>

<#macro macro_rtdm_app_h_states>
</#macro>

