<#-- x2c_scope_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_x2c_scope_app_h_includes>
#include <stdint.h>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_x2c_scope_app_h_type_definitions>

<#if CONFIG_X2C_SCOPE0 == true>
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX0 == "0">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX0}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX0 == "1">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX1}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX0 == "2">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX2}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX0 == "3">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX3}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX0 == "4">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX4}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX0 == "5">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX5}
    </#if>
</#if>

<#if CONFIG_X2C_SCOPE1 == true>
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX1 == "0">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX0}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX1 == "1">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX1}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX1 == "2">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX2}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX1 == "3">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX3}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX1 == "4">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX4}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX1 == "5">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX5}
    </#if>
</#if>

<#if CONFIG_X2C_SCOPE2 == true>
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX2 == "0">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX0}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX2 == "1">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX1}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX2 == "2">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX2}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX2 == "3">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX3}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX2 == "4">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX4}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX2 == "5">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX5}
    </#if>
</#if>

<#if CONFIG_X2C_SCOPE3 == true>
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX3 == "0">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX0}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX3 == "1">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX1}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX3 == "2">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX2}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX3 == "3">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX3}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX3 == "4">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX4}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX3 == "5">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX5}
    </#if>
</#if>

<#if CONFIG_X2C_SCOPE4 == true>
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX4 == "0">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX0}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX4 == "1">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX1}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX4 == "2">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX2}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX4 == "3">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX3}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX4 == "4">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX4}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX4 == "5">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX5}
    </#if>
</#if>

<#if CONFIG_X2C_SCOPE5 == true>
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX5 == "0">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX0}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX5 == "1">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX1}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX5 == "2">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX2}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX5 == "3">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX3}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX5 == "4">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX4}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX5 == "5">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX5}
    </#if>
</#if>

<#if CONFIG_X2C_SCOPE6 == true>
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX6 == "0">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX0}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX6 == "1">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX1}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX6 == "2">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX2}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX6 == "3">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX3}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX6 == "4">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX4}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX6 == "5">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX5}
    </#if>
</#if>


<#if CONFIG_X2C_SCOPE7 == true>
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX7 == "0">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX0}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX7 == "1">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX1}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX7 == "2">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX2}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX7 == "3">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX3}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX7 == "4">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX4}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX7 == "5">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX5}
    </#if>
</#if>

<#if CONFIG_X2C_SCOPE8 == true>
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX8 == "0">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX0}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX8 == "1">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX1}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX8 == "2">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX2}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX8 == "3">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX3}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX8 == "4">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX4}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX8 == "5">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX5}
    </#if>
</#if>

<#if CONFIG_X2C_SCOPE9 == true>
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX9 == "0">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX0}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX9 == "1">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX1}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX9 == "2">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX2}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX9 == "3">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX3}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX9 == "4">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX4}
    </#if>
    
    <#if CONFIG_X2C_SCOPE_USART_INSTANCE_INDEX9 == "5">
    #define X2C_SCOPE_UART_MODULE_ID ${CONFIG_DRV_USART_PERIPHERAL_ID_IDX5}
    </#if>
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
<#macro macro_x2c_scope_app_h_data>
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
<#macro macro_x2c_scope_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_x2c_scope_app_h_function_declarations>
void X2CScope_Init(void);
void X2CScope_Communicate(void);
void X2CScope_Update(void);
void X2CScope_Initialise(void);
void X2CScope_HookUARTFunctions(void (*sendSerialFcnPntr)(uint8_t), uint8_t (*receiveSerialFcnPntr)(), \
        uint8_t (*isReceiveDataAvailableFcnPntr)(), uint8_t (*isSendReadyFcnPntr)());

void sendSerial(uint8_t data);
uint8_t receiveSerial(void);
uint8_t isReceiveDataAvailable(void);
uint8_t isSendReady(void);
</#macro>

<#macro macro_x2c_scope_app_h_states>
</#macro>

