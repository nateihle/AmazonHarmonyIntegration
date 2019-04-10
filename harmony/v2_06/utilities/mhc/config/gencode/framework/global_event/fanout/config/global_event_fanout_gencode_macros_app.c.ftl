<#-- global_event_fanout_gencode_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_global_event_fanout_app_c_includes>
</#macro>

<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_global_event_fanout_app_c_callback_functions>
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_global_event_fanout_app_c_local_functions>
static void Global_Event_Fanout_Task(void)
{
    if (<@global_event_triggered eventName=("CONFIG_GENERATE_CODE_GLOBAL_EVENT_FANOUT_SOURCE" + "${HCONFIG_APP_INSTANCE}")?eval/>)
    {
<#assign fanout_count = ("CONFIG_GENERATE_CODE_GLOBAL_EVENT_FANOUT_NUMBER" + "${HCONFIG_APP_INSTANCE}")?eval?number>
<#list 1..fanout_count as x>
<#assign event_name = "CONFIG_GENERATE_CODE_GLOBAL_EVENT_FANOUT_DEST_" + "${x}" + "${HCONFIG_APP_INSTANCE}">
        <@global_event_trigger eventName=event_name?eval/>  ${" "}
</#list>
    }
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
<#macro macro_global_event_fanout_app_c_initialize>
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
<#macro macro_global_event_fanout_app_c_tasks_data>
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
<#macro macro_global_event_fanout_app_c_tasks_state_init>
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_global_event_fanout_app_c_tasks_calls_after_init>
</#macro>

<#--            
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_global_event_fanout_app_c_tasks_state_service_tasks>
            Global_Event_Fanout_Task();
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

