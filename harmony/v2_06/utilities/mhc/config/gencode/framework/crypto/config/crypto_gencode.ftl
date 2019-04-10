menu "Crypto"

menu "AES"

config CRYPTO_AES_ENCRYPT${INSTANCE}
    bool "Encrypt a Message" if !CRYPTO
	bool "Encrypt a Message (HW Accelerated)" if CRYPTO
    set USE_CRYPTO_LIB_NEEDED to y if CRYPTO_AES_ENCRYPT${INSTANCE}
    set USE_CRYPTO_HW_NEEDED to y if CRYPTO_AES_ENCRYPT${INSTANCE} && CRYPTO
	set USE_CRYPTO_AES_NEEDED to y if CRYPTO_AES_ENCRYPT${INSTANCE}
	set USE_CRYPTO_AES_CBC_NEEDED to y if CRYPTO_AES_ENCRYPT${INSTANCE}
    default n
	---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony Crypto AES Encryption Application Template</h2>
	<p>	This template generates a simple code example which encrypts a message
	using AES CBC (Cipher Block Chaining) mode. Application will automatically
    setup the Crypto library with the following settings.</p>
	<br>- Crypto AES Encryption</br>
	<br>- CBC Cipher Mode</br>
	<br><br><b>Default application has the IV (Initialization Vector) set to the same 
	value as key. Please change the IV value for increased security.</b></br></br>
	<p>All other relavent Crypto configuration options are set to their 
	default values. The crypto library configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> Crypto</p></html>
	---endhelp---

config CRYPTO_AES_ENCRYPT_MESSAGE${INSTANCE}
    string "Input Text (plain)"
	depends on CRYPTO_AES_ENCRYPT${INSTANCE}
	default "Hello World"
	
config CRYPTO_AES_ENCRYPT_KEY${INSTANCE}
    string "Key (128-bit)"
	depends on CRYPTO_AES_ENCRYPT${INSTANCE}
	default "0123456789abcdef"
	
config CRYPTO_AES_DECRYPT${INSTANCE}
    bool "Decrypt a Message" if !CRYPTO
	bool "Decrypt a Message (HW Accelerated)" if CRYPTO
    set USE_CRYPTO_LIB_NEEDED to y if CRYPTO_AES_DECRYPT${INSTANCE}
    set USE_CRYPTO_HW_NEEDED to y if CRYPTO_AES_DECRYPT${INSTANCE} && CRYPTO
	set USE_CRYPTO_AES_NEEDED to y if CRYPTO_AES_DECRYPT${INSTANCE}
	set USE_CRYPTO_AES_CBC_NEEDED to y if CRYPTO_AES_DECRYPT${INSTANCE}
    default n
	---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony Crypto AES Decryption Application Template</h2>
	<p>	This template generates a simple code example which encrypts a message
	using AES CBC (Cipher Block Chaining) mode. Application will automatically
    setup the Crypto library with the following settings.</p>
	<br>- Crypto AES Decryption</br>
	<br>- CBC Cipher Mode</br>
	<br><br><b>Default application has the IV (Initialization Vector) set to the same 
	value as key. Please change the IV value for increased security.</b></br></br>
	<p>All other relavent Crypto configuration options are set to their 
	default values. The crypto library configuration may be modified by 
	the user using MHC, under Harmony Framework Configuration -> Crypto</p></html>
	---endhelp---

config CRYPTO_AES_DECRYPT_MESSAGE${INSTANCE}
    string "Input Text (hex)"
	depends on CRYPTO_AES_DECRYPT${INSTANCE}
	default "0x1a,0x89,0x2f,0x4b,0xcf,0x23,0xd9,0x5f,0x2c,0x5f,0xf0,0x74,0xb5,0x81,0xc0,0xdc"
	
config CRYPTO_AES_DECRYPT_KEY${INSTANCE}
    string "Key (128-bit)"
	depends on CRYPTO_AES_DECRYPT${INSTANCE}
	default "0123456789abcdef"
	
endmenu

ifblock CRYPTO_AES_ENCRYPT${INSTANCE} || CRYPTO_AES_DECRYPT${INSTANCE}

add "^#include \"/utilities/mhc/config/gencode/framework/crypto/templates/crypto_aes_cbc_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/crypto/templates/crypto_aes_cbc_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS

endif

ifblock CRYPTO_AES_ENCRYPT${INSTANCE} || CRYPTO_AES_DECRYPT${INSTANCE}

add "^@macro_lib_crypto_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_lib_crypto_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_lib_crypto_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_lib_crypto_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_lib_crypto_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS
add "^@macro_lib_crypto_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES

add "^@macro_lib_crypto_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_lib_crypto_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_lib_crypto_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_lib_crypto_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_lib_crypto_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_lib_crypto_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_lib_crypto_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_lib_crypto_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_lib_crypto_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_lib_crypto_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
add "^@macro_lib_crypto_app_c_tasks_app_functions/>" to list APP${INSTANCE}_C_APP_TASKS_APP_FUNCTIONS
endif

endmenu
