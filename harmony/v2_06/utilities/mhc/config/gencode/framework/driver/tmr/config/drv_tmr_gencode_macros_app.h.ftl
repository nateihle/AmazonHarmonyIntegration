<#-- drv_tmr_gencode_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_drv_tmr_app_h_includes>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_drv_tmr_app_h_type_definitions>
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
<#macro macro_drv_tmr_app_h_data>
    DRV_HANDLE ${("CONFIG_APP_TMR_DRV_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval};
<#if ("CONFIG_APP_TMR_DRV_USE_GLOBAL_EVENT" + "${HCONFIG_APP_INSTANCE}")?eval>
<#if ("CONFIG_APP_TMR_DRV_GLOBAL_EVENT_COUNT" + "${HCONFIG_APP_INSTANCE}")?eval?number != 1>
    uint32_t   timerCallbackCount;
</#if>
</#if>
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
<#macro macro_drv_tmr_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_drv_tmr_app_h_function_declarations>
</#macro>




