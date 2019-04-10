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
   USART Buffer Queue model constants used by the application:
   
       ${APP_NAME?upper_case}_DRV_USART : Identifies the USART Driver instance to use
<#if ("CONFIG_APP_DRV_USART_USE_RX" + "${HCONFIG_APP_INSTANCE}")?eval>

       ${APP_NAME?upper_case}_DRV_USART_BQ_RX_BUFFER_LEN     : The length of each receive buffer
       ${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_RX_BUFFERS : The number of receive buffers
</#if>
<#if ("CONFIG_APP_DRV_USART_USE_TX" + "${HCONFIG_APP_INSTANCE}")?eval>

       ${APP_NAME?upper_case}_DRV_USART_BQ_TX_BUFFER_LEN     : The length of each transmit buffer
       ${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_TX_BUFFERS : The number of transmit buffers
</#if>
*/
#define ${APP_NAME?upper_case}_DRV_USART                     ${("CONFIG_APP_DRV_USART_INSTANCE_INDEX" + "${HCONFIG_APP_INSTANCE}")?eval}
<#if ("CONFIG_APP_DRV_USART_USE_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
#define ${APP_NAME?upper_case}_DRV_USART_BQ_RX_BUFFER_LEN      ${("CONFIG_APP_DRV_USART_BUFFER_QUEUE_RX_BUFFER_LEN" + "${HCONFIG_APP_INSTANCE}")?eval}
#define ${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_RX_BUFFERS  ${("CONFIG_APP_DRV_USART_BUFFER_QUEUE_NUMBER_RX_BUFFERS" + "${HCONFIG_APP_INSTANCE}")?eval}
</#if>
<#if ("CONFIG_APP_DRV_USART_USE_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
#define ${APP_NAME?upper_case}_DRV_USART_BQ_TX_BUFFER_LEN      ${("CONFIG_APP_DRV_USART_BUFFER_QUEUE_TX_BUFFER_LEN" + "${HCONFIG_APP_INSTANCE}")?eval}
#define ${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_TX_BUFFERS  ${("CONFIG_APP_DRV_USART_BUFFER_QUEUE_NUMBER_TX_BUFFERS" + "${HCONFIG_APP_INSTANCE}")?eval}
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
   USART Buffer Queue model variables used by the application:
   
    ${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} : the USART driver handle returned by DRV_USART_Open
<#if ("CONFIG_APP_DRV_USART_USE_RX" + "${HCONFIG_APP_INSTANCE}")?eval>

    usartBQRxData          :  An array of buffers used in a round-robin fashion to store received USART data
    usartBQRxCurrentBuffer :  The index of the current receive buffer 
    usartBQRxRead          :  The number of characters read into each buffer
    usartBQRxWritten       :  The number of characters extracted from each buffer
    usartBQRxBufferHandle  :  The buffer handles returned by the DRV_USART_BufferAddRead function
</#if>
<#if ("CONFIG_APP_DRV_USART_USE_TX" + "${HCONFIG_APP_INSTANCE}")?eval>

    usartBQTxData          :  An array of buffers used in a round-robin fashion to store transmitted USART data
    usartBQTxCurrentBuffer :  The index of the next available transmit buffer
    usartBQTxWritten       :  The number of characters to be transmitted from each buffer
    usartBQTxBufferHandle  :  The buffer handles returned by the DRV_USART_BufferAddWrite function
</#if>
*/
    DRV_HANDLE ${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval};
<#if ("CONFIG_APP_DRV_USART_USE_RX" + "${HCONFIG_APP_INSTANCE}")?eval>

    uint8_t                   usartBQRxData[${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_RX_BUFFERS][${APP_NAME?upper_case}_DRV_USART_BQ_RX_BUFFER_LEN];
    uint32_t                  usartBQRxCurrentBuffer;
    size_t                    usartBQRxRead[${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_RX_BUFFERS];
    size_t                    usartBQRxWritten[${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_RX_BUFFERS];
    DRV_USART_BUFFER_HANDLE   usartBQRxBufferHandle[${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_RX_BUFFERS];
</#if>
<#if ("CONFIG_APP_DRV_USART_USE_TX" + "${HCONFIG_APP_INSTANCE}")?eval>

    uint8_t                   usartBQTxData[${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_TX_BUFFERS][${APP_NAME?upper_case}_DRV_USART_BQ_TX_BUFFER_LEN];
    uint32_t                  usartBQTxCurrentBuffer;
    size_t                    usartBQTxWritten[${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_TX_BUFFERS];
    DRV_USART_BUFFER_HANDLE   usartBQTxBufferHandle[${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_TX_BUFFERS];
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




