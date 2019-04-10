<#-- oc_polling_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_app_drv_oc_polling_app_h_includes>
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
<#macro macro_app_drv_oc_polling_h_constants>
<#if ("CONFIG_APP_DRV_OC_POLLING" + "${HCONFIG_APP_INSTANCE}")?eval>
/*
IC driver model constants used by the application:
   
${APP_NAME?upper_case}_OC_INDEX  					: Identifies the IC Driver instance to use
${APP_NAME?upper_case}_OC_TMR_INDEX					: Identifies the Timer Driver instance to use
*/
#define ${APP_NAME?upper_case}_OC_INDEX                    	${("CONFIG_APP_DRV_OC_INSTANCE_INDEX" + "${HCONFIG_APP_INSTANCE}")?eval}
#define ${APP_NAME?upper_case}_OC_TMR_INDEX                	${("CONFIG_APP_DRV_OC_TMR_INSTANCE_INDEX" + "${HCONFIG_APP_INSTANCE}")?eval}
</#if>
</#macro>


<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_app_drv_oc_polling_app_h_type_definitions>
// *****************************************************************************
/* Crypto States
*/
typedef enum
{
    ${APP_NAME?upper_case}_OC_START,			
    ${APP_NAME?upper_case}_OC_DONE	
} ${APP_NAME?upper_case}_OC_STATES;
</#macro>

<#macro macro_app_drv_oc_polling_app_h_data>
<#if ("CONFIG_APP_DRV_OC_POLLING" + "${HCONFIG_APP_INSTANCE}")?eval>
	/*
    ${("CONFIG_APP_DRV_OC_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}			: the IC driver handle returned by DRV_IC_Open
    ${("CONFIG_APP_DRV_OC_TMR_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} 			: the Timer driver handle returned by DRV_TMR_Open
 	*/	
	DRV_HANDLE 			${("CONFIG_APP_DRV_OC_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval};
	DRV_HANDLE 			${("CONFIG_APP_DRV_OC_TMR_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval};
	/* Output Compare Task States */
	${APP_NAME?upper_case}_OC_STATES		ocStates;
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
<#macro macro_app_drv_oc_polling_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_app_drv_oc_polling_app_h_function_declarations>
</#macro>

<#macro macro_app_drv_oc_polling_app_h_states>
</#macro>

