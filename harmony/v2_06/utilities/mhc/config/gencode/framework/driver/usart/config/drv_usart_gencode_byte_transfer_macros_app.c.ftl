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
        /* Client waits until data is available and then reads byte */
        if (!DRV_USART_ReceiverBufferIsEmpty(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}))
        {
            *pBuf = DRV_USART_ReadByte(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval});
            return 1;
        }
    }
    
    return 0;
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
    uint8_t charRead;
    
    if(!DRV_USART_TransmitBufferIsFull(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}))
    {
        if (${("CONFIG_APP_DRV_USART_USE_GLOBAL_PIPE_NAME" + "${HCONFIG_APP_INSTANCE}")?eval}_pipeRead (&charRead, 1) > 0) 
        {
            DRV_USART_WriteByte(${APP_NAME?lower_case}Data.${("CONFIG_APP_DRV_USART_HANDLE" + "${HCONFIG_APP_INSTANCE}")?eval}, charRead);
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

