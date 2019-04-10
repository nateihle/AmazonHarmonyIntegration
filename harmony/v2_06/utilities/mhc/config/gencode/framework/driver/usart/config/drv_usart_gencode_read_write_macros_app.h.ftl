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
/*
   USART Read Write model constants used by the application:
   
       ${APP_NAME?upper_case}_DRV_USART  : Identifies the USART Driver instance to use
<#if ("CONFIG_APP_DRV_USART_RW_RX" + "${HCONFIG_APP_INSTANCE}")?eval>

       ${APP_NAME?upper_case}_DRV_USART_RW_RX_SIZE     : The length of the receive buffer
</#if>
*/
#define ${APP_NAME?upper_case}_DRV_USART                 ${("CONFIG_APP_DRV_USART_INSTANCE_INDEX" + "${HCONFIG_APP_INSTANCE}")?eval}
<#if ("CONFIG_APP_DRV_USART_RW_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
#define ${APP_NAME?upper_case}_DRV_USART_RW_RX_SIZE      ${("CONFIG_APP_DRV_USART_RW_RX_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval}
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
<#macro macro_drv_usart_app_h_data>
/*
   USART Read/Write model variables used by the application:
   
    ${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} : the USART driver handle returned by DRV_USART_Open
<#if ("CONFIG_APP_DRV_USART_RW_RX" + "${HCONFIG_APP_INSTANCE}")?eval>

    usartReadWriteRxIndex  :  The buffer index to USART data to be received
</#if>
<#if ("CONFIG_APP_DRV_USART_RW_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
    usartReadWriteTxIndex  :  The buffer index to USART data to be transmitted
</#if>
*/
    DRV_HANDLE ${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval};
<#if ("CONFIG_APP_DRV_USART_RW_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
    size_t     usartReadWriteRxIndex;
	uint8_t    usartRWRxData[${APP_NAME?upper_case}_DRV_USART_RW_RX_SIZE];
</#if>
<#if ("CONFIG_APP_DRV_USART_RW_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
    size_t     usartReadWriteTxIndex;
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

