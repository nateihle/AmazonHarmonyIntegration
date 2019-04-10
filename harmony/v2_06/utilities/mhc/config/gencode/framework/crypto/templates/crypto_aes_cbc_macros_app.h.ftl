<#-- crypto_aes_cbc_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_lib_crypto_app_h_includes>
#include "crypto/crypto.h"
#include "crypto/src/settings.h"
#include "crypto/src/aes.h"
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_lib_crypto_app_h_type_definitions>
// *****************************************************************************
/* Crypto States
*/
typedef enum
{
    ${APP_NAME?upper_case}_ENCRYPT,	
    ${APP_NAME?upper_case}_ENCDONE,		
    ${APP_NAME?upper_case}_DECRYPT,	
    ${APP_NAME?upper_case}_DECDONE	
} ${APP_NAME?upper_case}_CRYPTO_STATES;
</#macro>

<#macro macro_lib_crypto_app_h_data>
<#if ("CONFIG_CRYPTO_AES_ENCRYPT" + "${HCONFIG_APP_INSTANCE}")?eval>   
	/* Encryption Tasks States */
	${APP_NAME?upper_case}_CRYPTO_STATES encStates;
</#if>   
<#if ("CONFIG_CRYPTO_AES_DECRYPT" + "${HCONFIG_APP_INSTANCE}")?eval>   
	/* Decryption Tasks States */
	${APP_NAME?upper_case}_CRYPTO_STATES decStates;
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
<#macro macro_lib_crypto_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_lib_crypto_app_h_function_declarations>
</#macro>

<#macro macro_lib_crypto_app_h_states>
</#macro>

