<#-- led_gencode_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_led_app_h_includes>
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
<#macro macro_led_app_h_constants>
<#if ("CONFIG_APP_LED_UPON_EVENT" + "${HCONFIG_APP_INSTANCE}")?eval>
<#if ("CONFIG_APP_LED_UPON_EVENT_COUNT" + "${HCONFIG_APP_INSTANCE}")?eval?number != 1>
#define ${APP_NAME?upper_case}_LED_EVENT_COUNT        ${("CONFIG_APP_LED_UPON_EVENT_COUNT" + "${HCONFIG_APP_INSTANCE}")?eval}
</#if>
</#if>
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
<#macro macro_led_app_h_data>
<#if ("CONFIG_APP_LED_UPON_EVENT" + "${HCONFIG_APP_INSTANCE}")?eval>
<#if ("CONFIG_APP_LED_UPON_EVENT_COUNT" + "${HCONFIG_APP_INSTANCE}")?eval?number != 1>
    uint32_t   ledEventCount;
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
<#macro macro_led_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_led_app_h_function_declarations>
</#macro>




