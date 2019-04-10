<#-- ic_polling_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_app_drv_ic_polling_app_c_includes>
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
<#macro macro_app_drv_ic_polling_app_c_global_data>
<#if ("CONFIG_APP_DRV_IC_POLLING" + "${HCONFIG_APP_INSTANCE}")?eval>
/* Encryption related variables */
static uint32_t __attribute__ ((coherent, aligned (16))) ${APP_NAME?lower_case}_icBuffer[${APP_NAME?upper_case}_IC_NUM_EDGES];
</#if>
</#macro>
<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_app_drv_ic_polling_app_c_callback_functions>
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_app_drv_ic_polling_app_c_local_functions>
<#if ("CONFIG_APP_DRV_IC_POLLING" + "${HCONFIG_APP_INSTANCE}")?eval>
/******************************************************************************
  Function:
    static void ${APP_NAME?upper_case}_IC_Task (void)
    
   Remarks:
    Captures timer value on programmed edge of ICx pin.

*/
static void ${APP_NAME?upper_case}_IC_Task (void)
{		
	switch(${APP_NAME?lower_case}Data.icStates)
	{
		default:
		
		case ${APP_NAME?upper_case}_IC_START:
			/* Initialize number of IC Edges variable */
			${APP_NAME?lower_case}Data.numICEdges = 0;
			
			/* Start the input capture operation */
			DRV_IC_Start(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_IC_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}, DRV_IO_INTENT_READWRITE);
			
			/* Start Timer Driver */
			DRV_TMR_Start (${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_IC_TMR_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval});		
			
			${APP_NAME?lower_case}Data.icStates = ${APP_NAME?upper_case}_IC_READ_DATA;
			
			break;
			
		case ${APP_NAME?upper_case}_IC_READ_DATA:
			
			/* Check for buffer empty and read the buffer value */
			if (!DRV_IC_BufferIsEmpty(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_IC_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}))
			{
				${APP_NAME?lower_case}_icBuffer[${APP_NAME?lower_case}Data.numICEdges] = DRV_IC_Capture32BitDataRead(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_IC_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval});
				${APP_NAME?lower_case}Data.numICEdges++;
			}
				
			if (${APP_NAME?lower_case}Data.numICEdges == ${APP_NAME?upper_case}_IC_NUM_EDGES)
			{
				/* Stop input capture and timer drivers */
				DRV_IC_Stop (${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_IC_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval});
				DRV_TMR_Stop (${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_IC_TMR_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval});	
	
				${APP_NAME?lower_case}Data.icStates = ${APP_NAME?upper_case}_IC_DONE;
			}
			
			break;
			
		case ${APP_NAME?upper_case}_IC_DONE:
			
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

<#macro macro_app_drv_ic_polling_app_c_initialize>
    ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_IC_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_HANDLE_INVALID;
	${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_IC_TMR_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_HANDLE_INVALID;
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
<#macro macro_app_drv_ic_polling_app_c_tasks_data>
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
<#macro macro_app_drv_ic_polling_app_c_tasks_state_init>
<#if ("CONFIG_APP_DRV_IC_POLLING" + "${HCONFIG_APP_INSTANCE}")?eval>
            /* Open the IC driver if it has not been previously opened */
            if (${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_IC_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} == DRV_HANDLE_INVALID)
            {
                ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_IC_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_IC_Open(${APP_NAME?upper_case}_IC_INDEX, DRV_IO_INTENT_READWRITE);
                appInitialized &= ( DRV_HANDLE_INVALID != ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_IC_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} );
            }

			if (${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_IC_TMR_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} == DRV_HANDLE_INVALID)
            {
                ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_IC_TMR_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_TMR_Open(${APP_NAME?upper_case}_TMR_INDEX, DRV_IO_INTENT_READWRITE);
                appInitialized &= ( DRV_HANDLE_INVALID != ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_IC_TMR_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} );
            }
</#if>			
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_app_drv_ic_polling_app_c_tasks_calls_after_init>
</#macro>

<#--            /* Advance to the next state */
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_app_drv_ic_polling_app_c_tasks_state_service_tasks>
<#if ("CONFIG_APP_DRV_IC_POLLING" + "${HCONFIG_APP_INSTANCE}")?eval>
		    /* Run the state machine for servicing Input Capture */
            ${APP_NAME?upper_case}_IC_Task();
			
</#if>		
</#macro>

<#--        
            break;
        }
-->
<#macro macro_app_drv_ic_polling_app_c_tasks_states>
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

<#macro macro_app_drv_ic_polling_app_c_tasks_app_functions>
</#macro>
