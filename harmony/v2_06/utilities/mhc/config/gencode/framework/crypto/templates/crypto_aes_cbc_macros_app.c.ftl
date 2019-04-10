<#-- crypto_aes_cbc_macros_app.c.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************

#include "${APP_NAME?lower_case}.h"
-->
<#macro macro_lib_crypto_app_c_includes>
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
<#macro macro_lib_crypto_app_c_global_data>

<#if ("CONFIG_CRYPTO_AES_ENCRYPT" + "${HCONFIG_APP_INSTANCE}")?eval>
/* Encryption related variables */
static byte __attribute__ ((aligned (16))) ${APP_NAME?lower_case}_CBCEncPlainText[] = "${("CONFIG_CRYPTO_AES_ENCRYPT_MESSAGE" + "${HCONFIG_APP_INSTANCE}")?eval}";
static byte __attribute__ ((aligned (16))) ${APP_NAME?lower_case}_CBCEncKey[] = "${("CONFIG_CRYPTO_AES_ENCRYPT_KEY" + "${HCONFIG_APP_INSTANCE}")?eval}";
static byte __attribute__ ((aligned (16))) ${APP_NAME?lower_case}_CBCEncInitVect[] = "${("CONFIG_CRYPTO_AES_ENCRYPT_KEY" + "${HCONFIG_APP_INSTANCE}")?eval}";
static byte __attribute__ ((aligned (16))) ${APP_NAME?lower_case}_CBCEncryptedData[AES_BLOCK_SIZE * 4];
</#if>

<#if ("CONFIG_CRYPTO_AES_DECRYPT" + "${HCONFIG_APP_INSTANCE}")?eval>
/* Decryption related variables */
static byte __attribute__ ((aligned (16))) ${APP_NAME?lower_case}_CBCEncData[] = {${("CONFIG_CRYPTO_AES_DECRYPT_MESSAGE" + "${HCONFIG_APP_INSTANCE}")?eval}};
static byte __attribute__ ((aligned (16))) ${APP_NAME?lower_case}_CBCDecKey[] = "${("CONFIG_CRYPTO_AES_DECRYPT_KEY" + "${HCONFIG_APP_INSTANCE}")?eval}";
static byte __attribute__ ((aligned (16))) ${APP_NAME?lower_case}_CBCDecInitVect[] = "${("CONFIG_CRYPTO_AES_DECRYPT_KEY" + "${HCONFIG_APP_INSTANCE}")?eval}";
static byte __attribute__ ((aligned (16))) ${APP_NAME?lower_case}_CBCDecryptedText[AES_BLOCK_SIZE * 4];
</#if>
</#macro>
<#--
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
-->
<#macro macro_lib_crypto_app_c_callback_functions>
</#macro>

<#--
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
-->
<#macro macro_lib_crypto_app_c_local_functions>
<#if ("CONFIG_CRYPTO_AES_ENCRYPT" + "${HCONFIG_APP_INSTANCE}")?eval>

/******************************************************************************
  Function:
    static void ${APP_NAME?upper_case}_AES_CBC_Encrypt_Task (void)

   Remarks:
    Encrypts plain text using AEC CBC cipher.

*/
static void ${APP_NAME?upper_case}_AES_CBC_Encrypt_Task (void)
{
    static CRYPT_AES_CTX encryptionContext;

	switch(${APP_NAME?lower_case}Data.encStates)
	{
		default:

		case ${APP_NAME?upper_case}_ENCRYPT:
			/* Setup the key for encryption */
			CRYPT_AES_KeySet(&encryptionContext, ${APP_NAME?lower_case}_CBCEncKey, AES_BLOCK_SIZE, ${APP_NAME?lower_case}_CBCEncInitVect, AES_ENCRYPTION);

			/* Encrypt the input text */
			CRYPT_AES_CBC_Encrypt(&encryptionContext, ${APP_NAME?lower_case}_CBCEncryptedData, ${APP_NAME?lower_case}_CBCEncPlainText, AES_BLOCK_SIZE);

			${APP_NAME?lower_case}Data.encStates = ${APP_NAME?upper_case}_ENCDONE;

			break;

		case ${APP_NAME?upper_case}_ENCDONE:

			break;
	}
}
</#if>
<#if ("CONFIG_CRYPTO_AES_DECRYPT" + "${HCONFIG_APP_INSTANCE}")?eval>

/******************************************************************************
  Function:
    static void ${APP_NAME?upper_case}_AES_CBC_Decrypt_Task (void)

   Remarks:
    Decrypts ciphered text using AEC CBC cipher.

*/
static void ${APP_NAME?upper_case}_AES_CBC_Decrypt_Task (void)
{
    static CRYPT_AES_CTX decryptionContext;

	switch(${APP_NAME?lower_case}Data.decStates)
	{
		default:

		case ${APP_NAME?upper_case}_DECRYPT:
			/* Setup the key for decryption */
			CRYPT_AES_KeySet(&decryptionContext, ${APP_NAME?lower_case}_CBCDecKey, AES_BLOCK_SIZE, ${APP_NAME?lower_case}_CBCDecInitVect, AES_DECRYPTION);

			/* Decrypt the ciphered text */
			CRYPT_AES_CBC_Decrypt(&decryptionContext, ${APP_NAME?lower_case}_CBCDecryptedText, ${APP_NAME?lower_case}_CBCEncData, AES_BLOCK_SIZE);

			${APP_NAME?lower_case}Data.decStates = ${APP_NAME?upper_case}_DECDONE;

			break;

		case ${APP_NAME?upper_case}_DECDONE:

			break;
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

<#macro macro_lib_crypto_app_c_initialize>
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
<#macro macro_lib_crypto_app_c_tasks_data>
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
<#macro macro_lib_crypto_app_c_tasks_state_init>
</#macro>

<#--
            if (appInitialized)
            {
-->
<#macro macro_lib_crypto_app_c_tasks_calls_after_init>
</#macro>

<#--            /* Advance to the next state */
                ${APP_NAME?lower_case}Data.state = ${APP_NAME?upper_case}_STATE_SERVICE_TASKS;
            }
            break;
        }

        case ${APP_NAME?upper_case}_STATE_SERVICE_TASKS:
        {
-->
<#macro macro_lib_crypto_app_c_tasks_state_service_tasks>
<#if ("CONFIG_CRYPTO_AES_ENCRYPT" + "${HCONFIG_APP_INSTANCE}")?eval>

		    /* Run the state machine for servicing Crypto AES encryption task */
            ${APP_NAME?upper_case}_AES_CBC_Encrypt_Task();

</#if>
<#if ("CONFIG_CRYPTO_AES_DECRYPT" + "${HCONFIG_APP_INSTANCE}")?eval>

			/* Run the state machine for servicing Crypto AES decryption task */
			${APP_NAME?upper_case}_AES_CBC_Decrypt_Task();
</#if>
</#macro>

<#--
            break;
        }
-->
<#macro macro_lib_crypto_app_c_tasks_states>
</#macro>

<#--
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application state machine. */
            break;
        }
    }
}
-->

<#macro macro_lib_crypto_app_c_tasks_app_functions>
</#macro>
