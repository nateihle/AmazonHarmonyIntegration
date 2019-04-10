<#-- drv_usart_gencode_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_drv_usart_app_h_includes>
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
<#macro macro_drv_usart_app_h_constants>
#define ${APP_NAME?upper_case}_DRV_USART                     ${("CONFIG_APP_DRV_USART_INSTANCE_INDEX" + "${HCONFIG_APP_INSTANCE}")?eval}
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
<#macro macro_drv_usart_app_h_data>
    DRV_HANDLE ${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval};
<#if ("CONFIG_APP_DRV_USART_BM_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
	int tx_count;
</#if>
<#if ("CONFIG_APP_DRV_USART_BM_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
	int rx_count;
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
<#macro macro_drv_usart_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_drv_usart_app_h_function_declarations>
</#macro>

<#macro macro_drv_usart_app_h_states>
</#macro>



