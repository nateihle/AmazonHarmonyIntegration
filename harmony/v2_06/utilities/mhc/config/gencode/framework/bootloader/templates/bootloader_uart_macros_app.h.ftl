<#-- bootloader_uart_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_lib_bootloader_uart_app_h_includes>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_lib_bootloader_uart_app_h_type_definitions>
// *****************************************************************************
/* Bootloader Callback States
*/
typedef enum
{
    ${APP_NAME?upper_case}_BOOTLOADER_REGISTER_CALLBACK,	
    ${APP_NAME?upper_case}_BOOTLOADER_CALLBACK_REGISTERED
} ${APP_NAME?upper_case}_BOOTLOADER_CALLBACK_STATES;
</#macro>

<#macro macro_lib_bootloader_uart_app_h_data>
<#if ("CONFIG_BOOTLOADER_USART_APP" + "${HCONFIG_APP_INSTANCE}")?eval>   
	/* Encryption Tasks States */
	${APP_NAME?upper_case}_BOOTLOADER_CALLBACK_STATES bootloaderStates;
</#if> 
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
-->
<#macro macro_lib_bootloader_uart_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_lib_bootloader_uart_app_h_function_declarations>
</#macro>

<#macro macro_lib_bootloader_uart_app_h_states>
</#macro>

