<#-- x2c_scope_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_x2c_scope_app_c_includes>
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
<#macro macro_x2c_scope_app_c_global_data>


DRV_HANDLE X2C_SCOPE_UART_HANDLE;

</#macro>

<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_x2c_scope_app_c_callback_functions>
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_x2c_scope_app_c_local_functions>

void X2CScope_Init(void)
{
    X2CScope_HookUARTFunctions(sendSerial, receiveSerial, isReceiveDataAvailable, isSendReady);
    X2CScope_Initialise();
   
}


void sendSerial(uint8_t data)
{
    PLIB_USART_TransmitterByteSend(X2C_SCOPE_UART_MODULE_ID, data);

     
}

uint8_t receiveSerial()
{
    if((PLIB_USART_ReceiverFramingErrorHasOccurred(X2C_SCOPE_UART_MODULE_ID))\
      |(PLIB_USART_ReceiverParityErrorHasOccurred(X2C_SCOPE_UART_MODULE_ID))\
      |(PLIB_USART_ReceiverOverrunHasOccurred(X2C_SCOPE_UART_MODULE_ID)))
     {
       PLIB_USART_ReceiverOverrunErrorClear(X2C_SCOPE_UART_MODULE_ID);

        return ((uint8_t)0);   
     }
   return (PLIB_USART_ReceiverByteReceive(X2C_SCOPE_UART_MODULE_ID));
   
}

uint8_t isReceiveDataAvailable()
{
     return (PLIB_USART_ReceiverDataIsAvailable(X2C_SCOPE_UART_MODULE_ID));
     
}

uint8_t isSendReady()
{
    return(PLIB_USART_TransmitterIsEmpty(X2C_SCOPE_UART_MODULE_ID));
    
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
<#macro macro_x2c_scope_app_c_initialize>
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
<#macro macro_x2c_scope_app_c_tasks_data>
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
<#macro macro_x2c_scope_app_c_tasks_state_init>
			X2CScope_Init();
</#macro>  

<#--        
            if (appInitialized)
            {
-->
<#macro macro_x2c_scope_app_c_tasks_calls_after_init>
</#macro>

<#--            /* Advance to the next state */
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_PROCESS_MSGS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_PROCESS_MSGS:
        {
-->
<#macro macro_x2c_scope_app_c_tasks_state_service_tasks>
			X2CScope_Communicate(); //Communicate with X2C Scope Plugin
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

<#macro macro_x2c_scope_app_c_tasks_states>
</#macro>
