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
<#if ("CONFIG_APP_DRV_USART_USE_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
<#if !("CONFIG_APP_DRV_USART_USE_RX" + "${HCONFIG_APP_INSTANCE}")?eval || (("CONFIG_APP_DRV_USART_CREATE_GLOBAL_PIPE_NAME" + "${HCONFIG_APP_INSTANCE}")?eval != ("CONFIG_APP_DRV_USART_USE_GLOBAL_PIPE_NAME" + "${HCONFIG_APP_INSTANCE}")?eval)>

extern uint32_t ${("CONFIG_APP_DRV_USART_USE_GLOBAL_PIPE_NAME" + "${HCONFIG_APP_INSTANCE}")?eval}_pipeRead (uint8_t * pBuf, uint32_t n);
</#if>
</#if>
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
<#if ("CONFIG_APP_DRV_USART_USE_RX" + "${HCONFIG_APP_INSTANCE}")?eval>   
 
    /*  Does bufHandle identify an RX buffer?  */
    for (i=0; i<${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_RX_BUFFERS; i++)
    {
        if (${APP_NAME?lower_case}Data.usartBQRxBufferHandle[i] == bufHandle)
        {
            /* Transfer buffer to the Application by invalidating the buffer handle */
            ${APP_NAME?lower_case}Data.usartBQRxBufferHandle[i] = DRV_USART_BUFFER_HANDLE_INVALID;

            /* Zero the number of characters written in preparation for transmitting them */
            ${APP_NAME?lower_case}Data.usartBQRxWritten[i] = 0;
              
            /* Set the number of characters read */              
            if (bufEvent == DRV_USART_BUFFER_EVENT_COMPLETE)
            {
                /* Successful read */
                ${APP_NAME?lower_case}Data.usartBQRxRead[i]    = ${APP_NAME?upper_case}_DRV_USART_BQ_RX_BUFFER_LEN;
            }
            else
            {
                /* Read was not successful */
                ${APP_NAME?lower_case}Data.usartBQRxRead[i]    = 0;
            }
            
            return;
        }
    }
</#if>
<#if ("CONFIG_APP_DRV_USART_USE_TX" + "${HCONFIG_APP_INSTANCE}")?eval>   

    /*  Does the buffer handle identify a TX buffer?  */
    for (i=0; i<${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_TX_BUFFERS; i++)
    {
        if (${APP_NAME?lower_case}Data.usartBQTxBufferHandle[i] == bufHandle)
        {
            /* Transfer buffer to the Application by invalidating the buffer handle */
            ${APP_NAME?lower_case}Data.usartBQTxBufferHandle[i] = DRV_USART_BUFFER_HANDLE_INVALID;
            
            /* Zero the number of characters written in preparation for transmitting them */
            ${APP_NAME?lower_case}Data.usartBQTxWritten[i] = 0;
            
            return;
        }
    }
</#if>
}
<#if ("CONFIG_APP_DRV_USART_USE_RX" + "${HCONFIG_APP_INSTANCE}")?eval>

/******************************************************************************
  Function:
    uint32_t ${("CONFIG_APP_DRV_USART_CREATE_GLOBAL_PIPE_NAME" + "${HCONFIG_APP_INSTANCE}")?eval}_pipeRead (uint8_t * pBuf, uint32_t n)

   Parameters:
    pBuf :  The address of the buffer into which to transfer characters read from the USART
    n    :  The maximum number of characters to transfer
    
   Returns: The number of characters actually transferred (0 .. n)
    
   Remarks:
    This routine pipes USART data as a character stream to the caller.  The pipeRead is a 
    standard interface that allows data to be exchanged between different automatically 
    generated application modules.
*/
uint32_t ${("CONFIG_APP_DRV_USART_CREATE_GLOBAL_PIPE_NAME" + "${HCONFIG_APP_INSTANCE}")?eval}_pipeRead (uint8_t * pBuf, uint32_t n)
{
    if ((n > 0) && (${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval} != DRV_HANDLE_INVALID))
    {
        /* Has the current buffer been read by the USART driver and transferred to the application by the BufferQueueEventHandler function? */
        if (${APP_NAME?lower_case}Data.usartBQRxBufferHandle[${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer] == DRV_USART_BUFFER_HANDLE_INVALID)
        {
            /* Process this buffer */
            if (${APP_NAME?lower_case}Data.usartBQRxRead[${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer] > 0)
            {
                /* Limit the number of characters transferred to a maximum of n */
                if (${APP_NAME?lower_case}Data.usartBQRxRead[${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer] - ${APP_NAME?lower_case}Data.usartBQRxWritten[${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer] < n)
                {
                    n = ${APP_NAME?lower_case}Data.usartBQRxRead[${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer] - ${APP_NAME?lower_case}Data.usartBQRxWritten[${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer];
                }
                
                /* Copy the characters from the read buffer to the caller's buffer */
                memcpy(pBuf, &${APP_NAME?lower_case}Data.usartBQRxData[${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer][${APP_NAME?lower_case}Data.usartBQRxWritten[${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer]], n);
                
                /* Increment the number of characters transferred from this buffer */
                ${APP_NAME?lower_case}Data.usartBQRxWritten[${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer] += n;
            }
            
            /* Check to see if all characters have been transferred from the read buffer */
            if (${APP_NAME?lower_case}Data.usartBQRxWritten[${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer] 
                   >= ${APP_NAME?lower_case}Data.usartBQRxRead[${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer])
            {
                /* All characters have been transferred from the buffer, and so re-queue the read buffer for the USART driver to fill */
                ${APP_NAME?lower_case}Data.usartBQRxWritten[${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer] = 0;
                ${APP_NAME?lower_case}Data.usartBQRxRead[${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer]    = 0;
            
                DRV_USART_BufferAddRead(${APP_NAME?lower_case}Data.handleUSART0,
                    (DRV_USART_BUFFER_HANDLE * const)&${APP_NAME?lower_case}Data.usartBQRxBufferHandle[${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer],
                    &${APP_NAME?lower_case}Data.usartBQRxData[${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer][0],
                    ${APP_NAME?upper_case}_DRV_USART_BQ_RX_BUFFER_LEN);

                /* Advance to the next read buffer */
                ${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer = (${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer + 1) % ${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_RX_BUFFERS;               
            }
        }
        else
        {
            n = 0;
        }
    }
    else
    {
        n = 0;
    }
    
    return n;
}
</#if>
<#if ("CONFIG_APP_DRV_USART_USE_TX" + "${HCONFIG_APP_INSTANCE}")?eval>

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
    /* Check to see if the Application owns the current TX buffer */
    if (${APP_NAME?lower_case}Data.usartBQTxBufferHandle[${APP_NAME?lower_case}Data.usartBQTxCurrentBuffer] == DRV_USART_BUFFER_HANDLE_INVALID)
    {
        /* Process this buffer by filling it with characters obtained from the pipe */
        ${APP_NAME?lower_case}Data.usartBQTxWritten[${APP_NAME?lower_case}Data.usartBQTxCurrentBuffer] = (size_t)${("CONFIG_APP_DRV_USART_USE_GLOBAL_PIPE_NAME" + "${HCONFIG_APP_INSTANCE}")?eval}_pipeRead (&${APP_NAME?lower_case}Data.usartBQTxData[${APP_NAME?lower_case}Data.usartBQTxCurrentBuffer][0], ${APP_NAME?upper_case}_DRV_USART_BQ_TX_BUFFER_LEN);

        /* If we have data to transmit, then queue the buffer */
        if (${APP_NAME?lower_case}Data.usartBQTxWritten[${APP_NAME?lower_case}Data.usartBQTxCurrentBuffer] > 0)
        {
            DRV_USART_BufferAddWrite(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval},
                    (DRV_USART_BUFFER_HANDLE * const)&${APP_NAME?lower_case}Data.usartBQTxBufferHandle[${APP_NAME?lower_case}Data.usartBQTxCurrentBuffer],
                    &${APP_NAME?lower_case}Data.usartBQTxData[${APP_NAME?lower_case}Data.usartBQTxCurrentBuffer][0],
                    ${APP_NAME?lower_case}Data.usartBQTxWritten[${APP_NAME?lower_case}Data.usartBQTxCurrentBuffer]);
            
            /* Advance to the next TX buffer */
            ${APP_NAME?lower_case}Data.usartBQTxCurrentBuffer = (${APP_NAME?lower_case}Data.usartBQTxCurrentBuffer + 1) % ${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_TX_BUFFERS;               
        }
    }
}
</#if>
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
<#if ("CONFIG_APP_DRV_USART_USE_RX" + "${HCONFIG_APP_INSTANCE}")?eval>
    
    /* Start with the first receive buffer */
    ${APP_NAME?lower_case}Data.usartBQRxCurrentBuffer = 0;
    
    /* Initialize all receive buffers as owned by the Application with no data */
    {
        int i;
        
        for (i=0; i<${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_RX_BUFFERS; i++)
        {
            ${APP_NAME?lower_case}Data.usartBQRxBufferHandle[i] = DRV_USART_BUFFER_HANDLE_INVALID;
            ${APP_NAME?lower_case}Data.usartBQRxRead[i]         = 0;
            ${APP_NAME?lower_case}Data.usartBQRxWritten[i]      = 0;
        }
    }
</#if>
<#if ("CONFIG_APP_DRV_USART_USE_TX" + "${HCONFIG_APP_INSTANCE}")?eval>

    /* Start with the first transmit buffer */
    ${APP_NAME?lower_case}Data.usartBQTxCurrentBuffer = 0;
    
    /* Initialize all transmit buffers as owned by the Application with no data */
    {
        int i;
        
        for (i=0; i<${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_TX_BUFFERS; i++)
        {
            ${APP_NAME?lower_case}Data.usartBQTxBufferHandle[i] = DRV_USART_BUFFER_HANDLE_INVALID;
            ${APP_NAME?lower_case}Data.usartBQTxWritten[i]      = 0;
        }
    }
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
<#if ("CONFIG_APP_DRV_USART_USE_RX" + "${HCONFIG_APP_INSTANCE}")?eval>

                /* Queue all read buffers with the driver */
                {
                    int i;
                    
                    for (i=0; i<${APP_NAME?upper_case}_DRV_USART_BQ_NUMBER_RX_BUFFERS; i++)
                    {
                        ${APP_NAME?lower_case}Data.usartBQRxRead[i]         = 0;
                        DRV_USART_BufferAddRead(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval},
                                                (DRV_USART_BUFFER_HANDLE * const)&${APP_NAME?lower_case}Data.usartBQRxBufferHandle[i],
                                                &${APP_NAME?lower_case}Data.usartBQRxData[i][0],
                                                ${APP_NAME?upper_case}_DRV_USART_BQ_RX_BUFFER_LEN);
                    }
                }
</#if>              
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
<#if ("CONFIG_APP_DRV_USART_USE_TX" + "${HCONFIG_APP_INSTANCE}")?eval>
            USART_Task();
</#if>
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

