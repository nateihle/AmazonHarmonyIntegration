<#-- crypto_aes_cbc_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_lib_crypto_random_app_c_includes>
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
<#macro macro_lib_crypto_random_app_c_global_data>

<#if ("CONFIG_CRYPTO_RANDOM_APP" + "${HCONFIG_APP_INSTANCE}")?eval>
/* Random number */
static byte __attribute__ ((coherent, aligned (16))) ${APP_NAME?lower_case}_randomNumbers[${("CONFIG_CRYPTO_RANDOM_NUMBER_COUNT" + "${HCONFIG_APP_INSTANCE}")?eval}];
</#if>

</#macro>
<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_lib_crypto_random_app_c_callback_functions>
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_lib_crypto_random_app_c_local_functions>
<#if ("CONFIG_CRYPTO_RANDOM_APP" + "${HCONFIG_APP_INSTANCE}")?eval>

/******************************************************************************
  Function:
    static void ${APP_NAME?upper_case}_RANDOM_Task (void)
    
   Remarks:
    Populates a buffer with random numbers generated.

*/
static void ${APP_NAME?upper_case}_RANDOM_Task (void)
{
    switch(${APP_NAME?lower_case}Data.randomStates)
	{
		default:
		
		case ${APP_NAME?upper_case}_RANDOM_GENERATE:
			/* Load ${APP_NAME?lower_case}_randomNumbers buffer with generated random numbers */
			SYS_RANDOM_CryptoBlockGet (&${APP_NAME?lower_case}_randomNumbers,${("CONFIG_CRYPTO_RANDOM_NUMBER_COUNT" + "${HCONFIG_APP_INSTANCE}")?eval});
			
			${APP_NAME?lower_case}Data.randomStates = ${APP_NAME?upper_case}_RANDOM_DONE;
			
			break;
			
		case ${APP_NAME?upper_case}_RANDOM_DONE:
			
			break;
	}
}
</#if>
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

<#macro macro_lib_crypto_random_app_c_initialize>
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
<#macro macro_lib_crypto_random_app_c_tasks_data>
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
<#macro macro_lib_crypto_random_app_c_tasks_state_init>
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_lib_crypto_random_app_c_tasks_calls_after_init>
</#macro>

<#--            /* Advance to the next state */
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_lib_crypto_random_app_c_tasks_state_service_tasks>
<#if ("CONFIG_CRYPTO_RANDOM_APP" + "${HCONFIG_APP_INSTANCE}")?eval>

		    /* Run the state machine for servicing Crypto AES encryption task */
            ${APP_NAME?upper_case}_RANDOM_Task();
			
</#if>
</#macro>

<#--        
            break;
        }
-->
<#macro macro_lib_crypto_random_app_c_tasks_states>
</#macro>		

<#--  
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application state machine. */
            break;
        }
    }
}
-->

<#macro macro_lib_crypto_random_app_c_tasks_app_functions>
</#macro>
