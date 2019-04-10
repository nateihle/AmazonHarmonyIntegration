<#-- global_event_fanout_gencode_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_global_event_fanout_app_h_includes>
</#macro>

<#--
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
-->
<#macro macro_global_event_fanout_app_h_type_definitions>
</#macro>

<#--
// *****************************************************************************
/* Application Data
// *****************************************************************************


<#--
// *****************************************************************************
/* Application Data

typedef struct
{
    /* The application's current state */
    ${APP_NAME?upper_case}_STATES state;

    /* TODO: Define any additional data used by the application. */
-->
<#macro macro_global_event_fanout_app_h_data>
</#macro>
<#--
} ${APP_NAME?upper_case}_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are calglobal_event_fanout by drivers when certain events occur.
*/
-->
<#macro macro_global_event_fanout_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_global_event_fanout_app_h_function_declarations>
</#macro>




