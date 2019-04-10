<#-- drv_usart_gencode_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_lib_tcpip_app_h_includes>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_lib_tcpip_app_h_type_definitions>
<#if ("CONFIG_TCPIP_TCP_SERVER_TXRX" + "${HCONFIG_APP_INSTANCE}")?eval>
typedef enum
{
    ${APP_NAME?upper_case}_TCPIP_WAIT_FOR_IP,
    ${APP_NAME?upper_case}_TCPIP_OPENING_SERVER,
    ${APP_NAME?upper_case}_TCPIP_WAIT_FOR_CONNECTION,
    ${APP_NAME?upper_case}_TCPIP_SERVING_CONNECTION,
    ${APP_NAME?upper_case}_TCPIP_WAIT_FOR_RESPONSE
} ${APP_NAME?upper_case}_TCP_SERVER_TXRX_STATES;
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
<#macro macro_lib_tcpip_app_h_data>
<#if ("CONFIG_TCPIP_TCP_SERVER_TXRX" + "${HCONFIG_APP_INSTANCE}")?eval>
	TCP_SOCKET socket;
	TCP_PORT port;
	${APP_NAME?upper_case}_TCP_SERVER_TXRX_STATES txrxTaskState;
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
<#macro macro_lib_tcpip_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_lib_tcpip_app_h_function_declarations>
</#macro>

<#macro macro_lib_tcpip_app_h_states>
<#if ("CONFIG_TCPIP_TCP_SERVER_TXRX" + "${HCONFIG_APP_INSTANCE}")?eval>
	${APP_NAME?upper_case}_STATE_ERROR
</#if>
</#macro>

