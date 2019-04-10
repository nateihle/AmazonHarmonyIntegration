menu "Random"

config CRYPTO_RANDOM_APP${INSTANCE}
    bool "Generate Random Number(s)"
    set USE_SYS_RANDOM_NEEDED to y if CRYPTO_RANDOM_APP${INSTANCE}
	set XC32_HEAP to "1024" if CRYPTO_RANDOM_APP${INSTANCE}
    default n
	---help---
	<!DOCTYPE HTML>
	<html>
	<h2>MPLAB Harmony Random Number Application Template</h2>
	<p>	This template generates a simple code example which generates a random
	number. Application will automatically setup the System Random services
	and Crypto random library with the following settings.</p>
	<br>- Random Seed Size: 32</br>
	<br>- Hashes: SHA-256 and HMAC</br>
	<br>- Heap size: 1024</Br>
	<p>The heap size is set to 1024 to meet the basic application needs.
	The heap size may be modified by the user using MHC, Device & Project Configuration ->
	Project Configuration -> XC32 (Global Options) -> xc32-ld -> General -> Heap Size (bytes)</p>
	</html>
	---endhelp---

config CRYPTO_RANDOM_NUMBER_COUNT${INSTANCE}
    int "Random Number Count"
    depends on CRYPTO_RANDOM_APP${INSTANCE}
    default 32
---help---
	<!DOCTYPE HTML>
	<html>
	<p> Random number count represents the size of random numbers the application generates</p>
	</html>
---endhelp---
	
ifblock CRYPTO_RANDOM_APP${INSTANCE}

add "^#include \"/utilities/mhc/config/gencode/framework/system/random/templates/random_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/system/random/templates/random_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS

endif

ifblock CRYPTO_RANDOM_APP${INSTANCE}

add "^@macro_lib_crypto_random_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_lib_crypto_random_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_lib_crypto_random_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_lib_crypto_random_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_lib_crypto_random_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS
add "^@macro_lib_crypto_random_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES

add "^@macro_lib_crypto_random_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_lib_crypto_random_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_lib_crypto_random_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_lib_crypto_random_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_lib_crypto_random_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_lib_crypto_random_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_lib_crypto_random_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_lib_crypto_random_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_lib_crypto_random_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_lib_crypto_random_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
add "^@macro_lib_crypto_random_app_c_tasks_app_functions/>" to list APP${INSTANCE}_C_APP_TASKS_APP_FUNCTIONS
endif

endmenu
