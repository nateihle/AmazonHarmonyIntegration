<#-- drv_usart_gencode_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_drv_usart_app_c_includes>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data
*/
-->
<#macro macro_drv_usart_app_c_global_data>
<#if ("CONFIG_APP_DRV_USART_RW_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
static uint8_t usartRWTxData[] = "${("CONFIG_APP_DRV_USART_RW_TX_STRING" + "${HCONFIG_APP_INSTANCE}")?eval}\r\n";
</#if>
static enum 
{
    USART_RW_INIT,
    USART_RW_WORKING,
    USART_RW_DONE,
} usartRWState;
</#macro>

<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_drv_usart_app_c_callback_functions>
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_drv_usart_app_c_local_functions>

static void USART_Task (void)
{
    switch (usartRWState)
    {
        default:
        case USART_RW_INIT:
        {
<#if ("CONFIG_APP_DRV_USART_RW_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
            ${APP_NAME?lower_case}Data.usartReadWriteTxIndex = 0;
</#if>
<#if ("CONFIG_APP_DRV_USART_RW_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
            ${APP_NAME?lower_case}Data.usartReadWriteRxIndex = 0;
</#if>
            usartRWState = USART_RW_WORKING;
            break;
        }

        case USART_RW_WORKING:
        {
<#if ("CONFIG_APP_DRV_USART_RW_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
            if(${APP_NAME?lower_case}Data.usartReadWriteTxIndex < sizeof(usartRWTxData))
            {
                /* Transmit the bytes from the buffer */
                ${APP_NAME?lower_case}Data.usartReadWriteTxIndex += 
                    DRV_USART_Write(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval},
                    &usartRWTxData[${APP_NAME?lower_case}Data.usartReadWriteTxIndex],
                    sizeof(usartRWTxData) - ${APP_NAME?lower_case}Data.usartReadWriteTxIndex);
            }
</#if>
<#if ("CONFIG_APP_DRV_USART_RW_RX" + "${HCONFIG_APP_INSTANCE}")?eval>

            if (${APP_NAME?lower_case}Data.usartReadWriteRxIndex < ${APP_NAME?upper_case}_DRV_USART_RW_RX_SIZE)
            {
                /* Read bytes into the buffer */
                ${APP_NAME?lower_case}Data.usartReadWriteRxIndex += 
                    DRV_USART_Read(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval},
                    &${APP_NAME?lower_case}Data.usartRWRxData[${APP_NAME?lower_case}Data.usartReadWriteRxIndex],
                    ${APP_NAME?upper_case}_DRV_USART_RW_RX_SIZE - ${APP_NAME?lower_case}Data.usartReadWriteRxIndex);
            }
</#if>

            /* have we finished? */
            if (<#if ("CONFIG_APP_DRV_USART_RW_TX" + "${HCONFIG_APP_INSTANCE}")?eval>${APP_NAME?lower_case}Data.usartReadWriteTxIndex == sizeof(usartRWTxData)</#if><#if ("CONFIG_APP_DRV_USART_RW_RX" + "${HCONFIG_APP_INSTANCE}")?eval><#if ("CONFIG_APP_DRV_USART_RW_TX" + "${HCONFIG_APP_INSTANCE}")?eval> &&</#if></#if> <#if ("CONFIG_APP_DRV_USART_RW_RX" + "${HCONFIG_APP_INSTANCE}")?eval>(${APP_NAME?lower_case}Data.usartReadWriteRxIndex ==  ${APP_NAME?upper_case}_DRV_USART_RW_RX_SIZE)</#if>)
            {
                usartRWState = USART_RW_DONE;
            }
            break;
        }
        
        case USART_RW_DONE:
        {
            break;
        }
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
<#macro macro_drv_usart_app_c_initialize>
    ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_HANDLE_INVALID;

    /* initialize the USART state machine */
    usartRWState = USART_RW_INIT;
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
<#macro macro_drv_usart_app_c_tasks_data>
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
<#macro macro_drv_usart_app_c_tasks_state_init>
            if (${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} == DRV_HANDLE_INVALID)
            {
                ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} = DRV_USART_Open(${APP_NAME?upper_case}_DRV_USART, DRV_IO_INTENT_READWRITE|DRV_IO_INTENT_NONBLOCKING);
                appInitialized &= (DRV_HANDLE_INVALID != ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval});
            }

</#macro>    
<#--        
            if (appInitialized)
            {
-->
<#macro macro_drv_usart_app_c_tasks_calls_after_init>
</#macro>
<#--            
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_drv_usart_app_c_tasks_state_service_tasks>
            USART_Task();
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
<#macro macro_drv_usart_app_c_tasks_states>
</#macro>
