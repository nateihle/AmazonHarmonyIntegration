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
<#if ("CONFIG_APP_DRV_USART_BQ_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
static uint8_t usartBQTxData[] = "${("CONFIG_APP_DRV_USART_BQ_TX_STRING" + "${HCONFIG_APP_INSTANCE}")?eval}\r\n";
</#if>
static enum 
{
    USART_BQ_INIT,
    USART_BQ_WORKING,
    USART_BQ_DONE,
} usartBQState;

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
    void BufferQueueEventHandler ( DRV_USART_BUFFER_EVENT,
                DRV_USART_BUFFER_HANDLE,  uintptr_t );

   Remarks:
    This routine is callback function for the USART buffer queue events. Driver
    uses the application Tx or Rx buffer reference along with the client handle
    to notify the buffer events to the application. This call back function 
    to be registerd with the driver using DRV_USART_BufferAddRead(); 
    or DRV_USART_BufferAddWrite();
 */

static void BufferQueueEventHandler(DRV_USART_BUFFER_EVENT  bufEvent,
                                    DRV_USART_BUFFER_HANDLE bufHandle,
                                    uintptr_t               context )
{
    int i;
<#if ("CONFIG_APP_DRV_USART_BQ_RX" + "${HCONFIG_APP_INSTANCE}")?eval>   
 
    /*  Does bufHandle identify an RX buffer?  */
    if (${APP_NAME?lower_case}Data.usartBQRxBufferHandle == bufHandle)
    {
        /* Transfer buffer to the Application by invalidating the buffer handle */
        ${APP_NAME?lower_case}Data.usartBQRxBufferHandle = DRV_USART_BUFFER_HANDLE_INVALID;

        /* Zero the number of characters written in preparation for transmitting them */
        ${APP_NAME?lower_case}Data.usartBQRxWritten = 0;
          
        /* Set the number of characters read */              
        if (bufEvent == DRV_USART_BUFFER_EVENT_COMPLETE)
        {
            /* Successful read */
            ${APP_NAME?lower_case}Data.usartBQRxRead = ${APP_NAME?upper_case}_DRV_USART_BQ_RX_SIZE;
        }
        else
        {
            /* Read was not successful */
            ${APP_NAME?lower_case}Data.usartBQRxRead = 0;
        }
        
        return;
    }
</#if>
<#if ("CONFIG_APP_DRV_USART_BQ_TX" + "${HCONFIG_APP_INSTANCE}")?eval>   

    /*  Does the buffer handle identify a TX buffer?  */
    if (${APP_NAME?lower_case}Data.usartBQTxBufferHandle == bufHandle)
    {
        /* Transfer buffer to the Application by invalidating the buffer handle */
        ${APP_NAME?lower_case}Data.usartBQTxBufferHandle = DRV_USART_BUFFER_HANDLE_INVALID;
        ${APP_NAME?lower_case}Data.usartBQTxWritten = sizeof(usartBQTxData);
        return;
    }
</#if>
}

/******************************************************************************
  Function:
    static void USART_Task (void)
    
   Remarks:
    Feeds the USART transmitter.
*/
static void USART_Task (void)
{
    switch (usartBQState)
    {
        default:
        case USART_BQ_INIT:
        {
<#if ("CONFIG_APP_DRV_USART_BQ_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
            ${APP_NAME?lower_case}Data.usartBQTxWritten = 0;
</#if>
<#if ("CONFIG_APP_DRV_USART_BQ_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
            ${APP_NAME?lower_case}Data.usartBQRxRead    = 0;
            ${APP_NAME?lower_case}Data.usartBQRxWritten = 0;
</#if>

            usartBQState = USART_BQ_WORKING;
            break;
        }

        case USART_BQ_WORKING:
        {
<#if ("CONFIG_APP_DRV_USART_BQ_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
            /* Check to see if the Application owns the TX buffer */
            if (${APP_NAME?lower_case}Data.usartBQTxBufferHandle == DRV_USART_BUFFER_HANDLE_INVALID)
            {
                /* If we have data to transmit, then queue the buffer */
                if (${APP_NAME?lower_case}Data.usartBQTxWritten == 0)
                {
                    DRV_USART_BufferAddWrite(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval},
                            (DRV_USART_BUFFER_HANDLE * const)&${APP_NAME?lower_case}Data.usartBQTxBufferHandle,
                            &usartBQTxData[0], sizeof(usartBQTxData));
                }
            }

</#if>
<#if ("CONFIG_APP_DRV_USART_BQ_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
            /* Has the RX buffer been read by the USART driver and transferred to the application by the BufferQueueEventHandler function? */
            if (${APP_NAME?lower_case}Data.usartBQRxBufferHandle == DRV_USART_BUFFER_HANDLE_INVALID)
            {
                /* Process this buffer */
                if (${APP_NAME?lower_case}Data.usartBQRxRead == 0)
                {
                    DRV_USART_BufferAddRead(${APP_NAME?lower_case}Data.handleUSART0,
                        (DRV_USART_BUFFER_HANDLE * const)&${APP_NAME?lower_case}Data.usartBQRxBufferHandle,
                        &${APP_NAME?lower_case}Data.usartBQRxData[0], ${APP_NAME?upper_case}_DRV_USART_BQ_RX_SIZE);
                }
            }

</#if>
            if (<#if ("CONFIG_APP_DRV_USART_BQ_TX" + "${HCONFIG_APP_INSTANCE}")?eval>(${APP_NAME?lower_case}Data.usartBQTxWritten == sizeof(usartBQTxData))</#if><#if ("CONFIG_APP_DRV_USART_BQ_RX" + "${HCONFIG_APP_INSTANCE}")?eval><#if ("CONFIG_APP_DRV_USART_BQ_TX" + "${HCONFIG_APP_INSTANCE}")?eval> && </#if></#if><#if ("CONFIG_APP_DRV_USART_BQ_RX" + "${HCONFIG_APP_INSTANCE}")?eval>(${APP_NAME?lower_case}Data.usartBQRxRead == ${APP_NAME?upper_case}_DRV_USART_BQ_RX_SIZE)</#if>)
            {
                usartBQState = USART_BQ_DONE;
            }
            break;
        }

        case USART_BQ_DONE:
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
    
<#if ("CONFIG_APP_DRV_USART_BQ_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
    /* Initialize the receive buffer as owned by the Application with no data */
    ${APP_NAME?lower_case}Data.usartBQRxBufferHandle = DRV_USART_BUFFER_HANDLE_INVALID;
</#if>

<#if ("CONFIG_APP_DRV_USART_BQ_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
    ${APP_NAME?lower_case}Data.usartBQTxBufferHandle = DRV_USART_BUFFER_HANDLE_INVALID;
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
            /* Open the USART driver if it has not been previously opened */
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
                /* Establish the Buffer Queue event handler */
                DRV_USART_BufferEventHandlerSet(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval},
                                                BufferQueueEventHandler,
                                                NULL);

                usartBQState = USART_BQ_INIT;
</#macro>

<#--            /* Advance to the next state */
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
