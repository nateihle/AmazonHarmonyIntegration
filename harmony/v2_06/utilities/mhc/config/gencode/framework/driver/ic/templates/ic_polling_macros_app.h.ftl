<#-- ic_polling_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_app_drv_ic_polling_app_h_includes>
</#macro>

<#--
// *****************************************************************************
/* Application Data
// *****************************************************************************

// *****************************************************************************
/* Application constants

  Summary:
    Constants defined for the application

  Description:
    Constants defined for the application
*/
-->
<#macro macro_app_drv_ic_polling_h_constants>
<#if ("CONFIG_APP_DRV_IC_POLLING" + "${HCONFIG_APP_INSTANCE}")?eval>
/*
IC driver model constants used by the application:
   
${APP_NAME?upper_case}_IC_INDEX 				: Identifies the IC Driver instance to use
${APP_NAME?upper_case}_TMR_INDEX				: Identifies the Timer Driver instance to use
${APP_NAME?upper_case}_DRV)_IC_NUM_EDGES  		: Number of ICx edges that timer value is captured on
*/
#define ${APP_NAME?upper_case}_IC_INDEX       			${("CONFIG_APP_DRV_IC_INSTANCE_INDEX" + "${HCONFIG_APP_INSTANCE}")?eval}
#define ${APP_NAME?upper_case}_TMR_INDEX                ${("CONFIG_APP_DRV_IC_TMR_INSTANCE_INDEX" + "${HCONFIG_APP_INSTANCE}")?eval}
#define ${APP_NAME?upper_case}_IC_NUM_EDGES      		${("CONFIG_APP_DRV_IC_POLLING_NUM_EDGES" + "${HCONFIG_APP_INSTANCE}")?eval}
</#if>
</#macro>


<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_app_drv_ic_polling_app_h_type_definitions>
// *****************************************************************************
/* Crypto States
*/
typedef enum
{
    ${APP_NAME?upper_case}_IC_START,	
    ${APP_NAME?upper_case}_IC_READ_DATA,		
    ${APP_NAME?upper_case}_IC_DONE	
} ${APP_NAME?upper_case}_IC_STATES;
</#macro>

<#macro macro_app_drv_ic_polling_app_h_data>
<#if ("CONFIG_APP_DRV_IC_POLLING" + "${HCONFIG_APP_INSTANCE}")?eval>
	/*
    ${("CONFIG_APP_DRV_IC_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}				: the IC driver handle returned by DRV_IC_Open
    ${("CONFIG_APP_DRV_IC_TMR_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} 			: the Timer driver handle returned by DRV_TMR_Open
    numICEdges			: Number of input capture (ICx) edges to store the timer value
	*/	
	DRV_HANDLE 			${("CONFIG_APP_DRV_IC_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval};
	DRV_HANDLE 			${("CONFIG_APP_DRV_IC_TMR_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval};
	uint32_t			numICEdges; 
	/* Encryption Tasks States */
	${APP_NAME?upper_case}_IC_STATES		icStates;
</#if> 
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
-->
<#macro macro_app_drv_ic_polling_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_app_drv_ic_polling_app_h_function_declarations>
</#macro>

<#macro macro_app_drv_ic_polling_app_h_states>
</#macro>

