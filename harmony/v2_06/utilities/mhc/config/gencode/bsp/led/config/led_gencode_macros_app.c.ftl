<#-- led_gencode_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_led_app_c_includes>
#include "bsp.h"
</#macro>

<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_led_app_c_callback_functions>
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_led_app_c_local_functions>

/* Application's LED Task Function */
static void LedTask( void )
{
<#if ("CONFIG_APP_LED_UPON_EVENT" + "${HCONFIG_APP_INSTANCE}")?eval>
    if (<@global_event_triggered eventName=("CONFIG_APP_LED_UPON_EVENT_NAME" + "${HCONFIG_APP_INSTANCE}")?eval/>)
    {
<#if ("CONFIG_APP_LED_UPON_EVENT_COUNT" + "${HCONFIG_APP_INSTANCE}")?eval?number != 1>
        ${APP_NAME?lower_case}Data.ledEventCount++;

        if (${APP_NAME?lower_case}Data.ledEventCount >= ${APP_NAME?upper_case}_LED_EVENT_COUNT)
        {
            ${APP_NAME?lower_case}Data.ledEventCount = 0;
            ${("CONFIG_APP_LED_NAME" + "${HCONFIG_APP_INSTANCE}")?eval}Toggle();
        }
<#else>
            ${("CONFIG_APP_LED_NAME" + "${HCONFIG_APP_INSTANCE}")?eval}Toggle();
</#if>
    }
</#if>
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
<#macro macro_led_app_c_initialize>
<#if ("CONFIG_APP_LED_UPON_EVENT" + "${HCONFIG_APP_INSTANCE}")?eval>
<#if ("CONFIG_APP_LED_UPON_EVENT_COUNT" + "${HCONFIG_APP_INSTANCE}")?eval?number != 1>
    ${APP_NAME?lower_case}Data.ledEventCount = 0;
</#if>
</#if>   
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
<#macro macro_led_app_c_tasks_data>
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
<#macro macro_led_app_c_tasks_state_init>
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_led_app_c_tasks_calls_after_init>
</#macro>

<#--            
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_led_app_c_tasks_state_service_tasks>
            LedTask();
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

