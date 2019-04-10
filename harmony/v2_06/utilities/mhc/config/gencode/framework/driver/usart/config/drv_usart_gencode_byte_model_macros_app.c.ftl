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
// ****************************************************************************

// *****************************************************************************
/* Application Data
*/
-->
<#macro macro_drv_usart_app_c_global_data>
<#if ("CONFIG_APP_DRV_USART_BM_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
static uint8_t ${APP_NAME?lower_case}_tx_buf[] = "${("CONFIG_APP_DRV_USART_BM_TX_STRING" + "${HCONFIG_APP_INSTANCE}")?eval}\r\n";
</#if>
<#if ("CONFIG_APP_DRV_USART_BM_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
static uint8_t ${APP_NAME?lower_case}_rx_buf[${("CONFIG_APP_DRV_USART_BM_RX_SIZE" + "${HCONFIG_APP_INSTANCE}")?eval}];
</#if>
static enum 
{
    USART_BM_INIT,
    USART_BM_WORKING,
    USART_BM_DONE,
} usartBMState;

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
/******************************************************************************
  Function:
    static void USART_Task (void)
    
   Remarks:
    Feeds the USART transmitter by reading characters from a specified pipe.  The pipeRead function is a 
    standard interface that allows data to be exchanged between different automatically 
    generated application modules.  Typically, the pipe is connected to the application's
    USART receive function, but could be any other Harmony module which supports the pipe interface. 
*/
static void USART_Task (void)
{
    switch (usartBMState)
    {
        default:
        case USART_BM_INIT:
        {
<#if ("CONFIG_APP_DRV_USART_BM_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
            ${APP_NAME?lower_case}Data.tx_count = 0;
</#if>
<#if ("CONFIG_APP_DRV_USART_BM_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
            ${APP_NAME?lower_case}Data.rx_count = 0;
</#if>
            usartBMState = USART_BM_WORKING;
            break;
        }

        case USART_BM_WORKING:
        {
<#if ("CONFIG_APP_DRV_USART_BM_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
            if (${APP_NAME?lower_case}Data.tx_count < sizeof(${APP_NAME?lower_case}_tx_buf)) 
            {
                if(!DRV_USART_TransmitBufferIsFull(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}))
                {
                    DRV_USART_WriteByte(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}, ${APP_NAME?lower_case}_tx_buf[${APP_NAME?lower_case}Data.tx_count]);
                    ${APP_NAME?lower_case}Data.tx_count++;
                }
            }

</#if>
<#if ("CONFIG_APP_DRV_USART_BM_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
            if (${APP_NAME?lower_case}Data.rx_count < sizeof(${APP_NAME?lower_case}_rx_buf)) 
            {
                if(!DRV_USART_ReceiverBufferIsEmpty(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}))
                {
                    ${APP_NAME?lower_case}_rx_buf[${APP_NAME?lower_case}Data.rx_count] = DRV_USART_ReadByte(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval});
                    ${APP_NAME?lower_case}Data.rx_count++;
                }
            }

</#if>
            /* Have we finished? */
            if (<#if ("CONFIG_APP_DRV_USART_BM_TX" + "${HCONFIG_APP_INSTANCE}")?eval>${APP_NAME?lower_case}Data.tx_count == sizeof(${APP_NAME?lower_case}_tx_buf)</#if><#if ("CONFIG_APP_DRV_USART_BM_RX" + "${HCONFIG_APP_INSTANCE}")?eval><#if ("CONFIG_APP_DRV_USART_BM_TX" + "${HCONFIG_APP_INSTANCE}")?eval> && </#if></#if><#if ("CONFIG_APP_DRV_USART_BM_RX" + "${HCONFIG_APP_INSTANCE}")?eval>${APP_NAME?lower_case}Data.rx_count == sizeof(${APP_NAME?lower_case}_rx_buf)</#if>)
            {
                usartBMState = USART_BM_DONE;
            }
            break;
        }

        case USART_BM_DONE:
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
                appInitialized &= ( DRV_HANDLE_INVALID != ${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} );
            }
</#macro>    

<#--        
            if (appInitialized)
            {
-->
<#macro macro_drv_usart_app_c_tasks_calls_after_init>
                /* initialize the USART state machine */
                usartBMState = USART_BM_INIT;
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

<#macro macro_drv_usart_app_c_tasks_states>
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

