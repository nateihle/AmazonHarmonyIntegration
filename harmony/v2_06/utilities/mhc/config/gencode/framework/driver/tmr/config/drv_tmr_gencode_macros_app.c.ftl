<#-- drv_tmr_gencode_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_drv_tmr_app_c_includes>
</#macro>

<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_drv_tmr_app_c_callback_functions>

/* Application's Timer Callback Function */
static void TimerCallback (  uintptr_t context, uint32_t alarmCount )
{
<#if ("CONFIG_APP_TMR_DRV_USE_GLOBAL_EVENT" + "${HCONFIG_APP_INSTANCE}")?eval>
<#assign event_name = "CONFIG_APP_TMR_DRV_GLOBAL_EVENT_NAME" + "${HCONFIG_APP_INSTANCE}">
<#if event_name?has_content>
<#if ("CONFIG_APP_TMR_DRV_GLOBAL_EVENT_COUNT" + "${HCONFIG_APP_INSTANCE}")?eval?number != 1>
    ${APP_NAME?lower_case}Data.timerCallbackCount++;

    if (${APP_NAME?lower_case}Data.timerCallbackCount >= ${APP_NAME?upper_case}_TIMER_CALLBACKS_PER_EVENT)
    {
        ${APP_NAME?lower_case}Data.timerCallbackCount = 0;
        
        <@global_event_trigger eventName=event_name?eval/> ${" "}
    }        
<#else>
    <@global_event_trigger eventName=event_name?eval/>  ${" "}
</#if>
</#if>
</#if>
}
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_drv_tmr_app_c_local_functions>

/* Application's Timer Setup Function */
static void TimerSetup( void )
{
    DRV_TMR_AlarmRegister(
        ${APP_NAME?lower_case}Data.${("CONFIG_APP_TMR_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}, 
        ${APP_NAME?upper_case}_TMR_DRV_PERIOD, 
        ${APP_NAME?upper_case}_TMR_DRV_IS_PERIODIC,
        (uintptr_t)NULL, 
        TimerCallback);
    DRV_TMR_Start(${APP_NAME?lower_case}Data.${("CONFIG_APP_TMR_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval});
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
<#macro macro_drv_tmr_app_c_initialize>
    ${APP_NAME?lower_case}Data.${("CONFIG_APP_TMR_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_HANDLE_INVALID;
<#if ("CONFIG_APP_TMR_DRV_USE_GLOBAL_EVENT" + "${HCONFIG_APP_INSTANCE}")?eval>
<#if ("CONFIG_APP_TMR_DRV_GLOBAL_EVENT_NAME" + "${HCONFIG_APP_INSTANCE}")?has_content>
<#if ("CONFIG_APP_TMR_DRV_GLOBAL_EVENT_COUNT" + "${HCONFIG_APP_INSTANCE}")?eval?number != 1>
    ${APP_NAME?lower_case}Data.timerCallbackCount = 0;
</#if>
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
<#macro macro_drv_tmr_app_c_tasks_data>
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
<#macro macro_drv_tmr_app_c_tasks_state_init>
            if (${APP_NAME?lower_case}Data.${("CONFIG_APP_TMR_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} == DRV_HANDLE_INVALID)
            {
                ${APP_NAME?lower_case}Data.${("CONFIG_APP_TMR_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_TMR_Open(${APP_NAME?upper_case}_TMR_DRV, DRV_IO_INTENT_EXCLUSIVE);
                appInitialized &= ( DRV_HANDLE_INVALID != ${APP_NAME?lower_case}Data.${("CONFIG_APP_TMR_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} );
            }
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_drv_tmr_app_c_tasks_calls_after_init>
                TimerSetup();
</#macro>

<#--            
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_drv_tmr_app_c_tasks_state_service_tasks>
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

