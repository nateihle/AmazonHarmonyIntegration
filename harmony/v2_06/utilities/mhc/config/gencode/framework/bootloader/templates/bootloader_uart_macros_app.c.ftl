<#-- bootloader_uart_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_lib_bootloader_uart_app_c_includes>
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
<#macro macro_lib_bootloader_uart_app_c_global_data>
#define ${APP_NAME?upper_case}_BOOTLOADER_TRIGGER_MEMORY_ADDRESS					${("CONFIG_BOOTLOADER_TRIGGER_MEMORY_LOCATION" + "${HCONFIG_APP_INSTANCE}")?eval}			
</#macro>
<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->

<#macro macro_lib_bootloader_uart_app_c_callback_functions>
<#if ("CONFIG_BOOTLOADER_USART_APP" + "${HCONFIG_APP_INSTANCE}")?eval>
/******************************************************************************
  Function:
    static void ${APP_NAME?upper_case}_Bootloader_ForceEvent (void)
    
   Remarks:
    Sets a trigger to be passed to force bootloader callback.
	Run bootloader if memory location == '0xFFFFFFFF' otherwise jump to user 
	application.
*/ 
int ${APP_NAME?upper_case}_Bootloader_ForceEvent(void)
{
    /* Check the trigger memory location and return true/false. */
    if (*(uint32_t *)${APP_NAME?upper_case}_BOOTLOADER_TRIGGER_MEMORY_ADDRESS == 0xFFFFFFFF)
        return (1);
    else
		return (0);
}
</#if>
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_lib_bootloader_uart_app_c_local_functions>
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

<#macro macro_lib_bootloader_uart_app_c_initialize>
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
<#macro macro_lib_bootloader_uart_app_c_tasks_data>
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
<#macro macro_lib_bootloader_uart_app_c_tasks_state_init>
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_lib_bootloader_uart_app_c_tasks_calls_after_init>
<#if ("CONFIG_BOOTLOADER_USART_APP" + "${HCONFIG_APP_INSTANCE}")?eval>
				/* Register force bootloader call back */
				/* Call back to trigger bootloading operation or jump to user application */
				BOOTLOADER_ForceBootloadRegister(${APP_NAME?upper_case}_Bootloader_ForceEvent);
</#if>
</#macro>

<#--            /* Advance to the next state */
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_lib_bootloader_uart_app_c_tasks_state_service_tasks>
</#macro>

<#--        
            break;
        }
-->
<#macro macro_lib_bootloader_uart_app_c_tasks_states>
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

<#macro macro_lib_bootloader_uart_app_c_tasks_app_functions>
</#macro>
