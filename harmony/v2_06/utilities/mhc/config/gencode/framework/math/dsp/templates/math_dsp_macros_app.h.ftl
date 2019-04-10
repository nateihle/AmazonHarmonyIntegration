<#-- math_dsp_macros_app.h.ftl -->

<#--
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
-->
<#macro macro_math_dsp_app_h_includes>
#include <math/dsp/dsp.h>
<#if ("CONFIG_DSP_VECTOR_EXP" + "${HCONFIG_APP_INSTANCE}")?eval>
#include <math/libq/libq.h>
</#if>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_math_dsp_app_h_type_definitions>
// *****************************************************************************
/* Math DSP States
*/
typedef enum
{
    ${APP_NAME?upper_case}_MATH_DSP_STATE_START,
    ${APP_NAME?upper_case}_MATH_DSP_STATE_DONE,
} ${APP_NAME?upper_case}_MATH_DSP_STATES;
</#macro>

<#macro macro_math_dsp_app_h_data>
    ${APP_NAME?upper_case}_MATH_DSP_STATES mathDspStates;
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
<#macro macro_math_dsp_app_h_callback_function_declarations>
</#macro>

<#--
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
-->
<#macro macro_math_dsp_app_h_function_declarations>
</#macro>

<#macro macro_math_dsp_app_h_states>
</#macro>

