<#-- random_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_lib_crypto_random_app_h_includes>
#include "crypto/crypto.h"
#include "crypto/src/random.h"
#include "crypto/src/sha256.h"
#include "crypto/src/hmac.h"
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_lib_crypto_random_app_h_type_definitions>
// *****************************************************************************
/* Crypto States
*/
typedef enum
{
    ${APP_NAME?upper_case}_RANDOM_GENERATE,	
    ${APP_NAME?upper_case}_RANDOM_DONE	
} ${APP_NAME?upper_case}_RANDOM_STATES;
</#macro>

<#macro macro_lib_crypto_random_app_h_data>
<#if ("CONFIG_CRYPTO_RANDOM_APP" + "${HCONFIG_APP_INSTANCE}")?eval>   
	/* Encryption Tasks States */
	${APP_NAME?upper_case}_RANDOM_STATES randomStates;
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
<#macro macro_lib_crypto_random_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_lib_crypto_random_app_h_function_declarations>
</#macro>

<#macro macro_lib_crypto_random_app_h_states>
</#macro>

