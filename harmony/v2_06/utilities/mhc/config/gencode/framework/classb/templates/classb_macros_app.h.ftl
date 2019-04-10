<#-- classb_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_classb_app_h_includes>
#include "classb/classb.h"
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_classb_app_h_type_definitions>
typedef enum
{
	/* Application's state machine's initial state. */
	${APP_NAME?upper_case}_CLASSB_STATE_INIT=0,
	${APP_NAME?upper_case}_CLASSB_STATE_FLASH_CRC_TEST,
    ${APP_NAME?upper_case}_CLASSB_STATE_CHECKER_BOARD_RAM_TEST,
    ${APP_NAME?upper_case}_CLASSB_STATE_RAM_MARCHB_TEST,
    ${APP_NAME?upper_case}_CLASSB_STATE_RAM_MARCHC_TEST,
    ${APP_NAME?upper_case}_CLASSB_STATE_RAM_MARCHC_STACK_TEST,
    ${APP_NAME?upper_case}_CLASSB_STATE_CLOCK_TEST,
    ${APP_NAME?upper_case}_CLASSB_STATE_CPU_PC_TEST,
    ${APP_NAME?upper_case}_CLASSB_STATE_CPU_REGISTERS_TEST,
    ${APP_NAME?upper_case}_CLASSB_STATE_TEST_PASS,
    ${APP_NAME?upper_case}_CLASSB_STATE_TEST_FAIL,
} ${APP_NAME?upper_case}_CLASSB_STATES;
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
<#macro macro_classb_app_h_data>
	${APP_NAME?upper_case}_CLASSB_STATES classBState;
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
<#macro macro_classb_app_h_callback_function_declarations>
</#macro>

<#--	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_classb_app_h_function_declarations>
</#macro>

<#macro macro_classb_app_h_states>

</#macro>

