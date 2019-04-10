menu "Math"

menu "DSP Functions"

config DSP_COMPLEX_CONJ${INSTANCE}
    bool "Use DSP_ComplexConj32"
    select USE_DSP if DSP_COMPLEX_CONJ${INSTANCE}
    default n
    ---help---
        IDH_HTML_DSP_ComplexConj32_int32c___int32c__
    ---endhelp---

config DSP_FFT${INSTANCE}
    bool "Use DSP_TransformFFT16"
    select USE_DSP if DSP_FFT${INSTANCE}
    default n
    ---help---
        IDH_HTML_DSP_TransformFFT16_int16c___int16c___int16c___int16c___int
    ---endhelp---

config DSP_FILTER_FIR${INSTANCE}
    bool "Use DSP_FilterFIR32"
    select USE_DSP if DSP_FILTER_FIR${INSTANCE}
    default n
    ---help---
        IDH_HTML_DSP_FilterFIR32_int32_t___int32_t___int32_t___int32_t___int_int_int
    ---endhelp---

config DSP_FILTER_IIRBQ${INSTANCE}
    bool "Use DSP_FilterIIRBQ16"
    select USE_DSP if DSP_FILTER_IIRBQ${INSTANCE}
    default n
    ---help---
        IDH_HTML_DSP_FilterIIRBQ16_int16_t_PARM_EQUAL_FILTER__
    ---endhelp---

config DSP_FILTER_LMS${INSTANCE}
    bool "Use DSP_FilterLMS16"
    select USE_DSP if DSP_FILTER_LMS${INSTANCE}
    default n
    ---help---
        IDH_HTML_DSP_FilterLMS16_int16_t_int16_t_int16_t___int16_t___int16_t___int_int16_t
    ---endhelp---

config DSP_MATRIX_TRANSPOSE${INSTANCE}
    bool "Use DSP_MatrixTranspose32"
    select USE_DSP if DSP_MATRIX_TRANSPOSE${INSTANCE}
    default n
    ---help---
        IDH_HTML_DSP_MatrixTranspose32_matrix32___matrix32__
    ---endhelp---

config DSP_VECTOR_ABS${INSTANCE}
    bool "Use DSP_VectorAbs32"
    select USE_DSP if DSP_VECTOR_ABS${INSTANCE}
    default n
    ---help---
        IDH_HTML_DSP_VectorAbs32_int32_t___int32_t___int
    ---endhelp---

config DSP_VECTOR_ADD${INSTANCE}
    bool "Use DSP_VectorAdd32"
    select USE_DSP if DSP_VECTOR_ADD${INSTANCE}
    default n
    ---help---
        IDH_HTML_DSP_VectorAdd32_int32_t___int32_t___int32_t___int
    ---endhelp---

config DSP_VECTOR_DOTP${INSTANCE}
    bool "Use DSP_VectorDotp16"
    select USE_DSP if DSP_VECTOR_DOTP${INSTANCE}
    default n
    ---help---
        IDH_HTML_DSP_VectorDotp16_int16_t___int16_t___int_int
    ---endhelp---

config DSP_VECTOR_EXP${INSTANCE}
    bool "Use DSP_VectorExp"
    select USE_DSP if DSP_VECTOR_EXP${INSTANCE}
    select USE_LIBQ if DSP_VECTOR_EXP${INSTANCE}
    default n
    ---help---
        IDH_HTML_DSP_VectorExp__Q16____Q16___int
    ---endhelp---

config DSP_VECTOR_RMS${INSTANCE}
    bool "Use DSP_VectorRMS16"
    select USE_DSP if DSP_VECTOR_RMS${INSTANCE}
    default n
    ---help---
        IDH_HTML_DSP_VectorRMS16_int16_t___int
    ---endhelp---

config DSP_VECTOR_SUMS${INSTANCE}
    bool "Use DSP_VectorSumSquares32"
    select USE_DSP if DSP_VECTOR_SUMS${INSTANCE}
    default n
    ---help---
        IDH_HTML_DSP_VectorSumSquares32_int32_t___int_int
    ---endhelp---

config DSP_VECTOR_VARIANCE${INSTANCE}
    bool "Use DSP_VectorVariance"
    select USE_DSP if DSP_VECTOR_VARIANCE${INSTANCE}
    default n
    ---help---
        IDH_HTML_DSP_VectorVariance_int32_t___int
    ---endhelp---

endmenu


ifblock DSP_COMPLEX_CONJ${INSTANCE} ||
	DSP_FFT${INSTANCE} ||
	DSP_FILTER_FIR${INSTANCE} ||
	DSP_FILTER_IIRBQ${INSTANCE} ||
	DSP_FILTER_LMS${INSTANCE} ||
	DSP_MATRIX_TRANSPOSE${INSTANCE} ||
	DSP_VECTOR_ABS${INSTANCE} ||
	DSP_VECTOR_ADD${INSTANCE} ||
	DSP_VECTOR_DOTP${INSTANCE} ||
	DSP_VECTOR_EXP${INSTANCE} ||
	DSP_VECTOR_RMS${INSTANCE} ||
	DSP_VECTOR_SUMS${INSTANCE} ||
	DSP_VECTOR_VARIANCE${INSTANCE}

add "^#include \"/utilities/mhc/config/gencode/framework/math/dsp/templates/math_dsp_macros_app.h.ftl\">" to list APP_FREEMARKER_MACROS
add "^#include \"/utilities/mhc/config/gencode/framework/math/dsp/templates/math_dsp_macros_app.c.ftl\">" to list APP_FREEMARKER_MACROS

add "^@macro_math_dsp_app_h_includes/>" to list APP${INSTANCE}_H_INCLUDES
add "^@macro_math_dsp_app_h_type_definitions/>" to list APP${INSTANCE}_H_TYPE_DEFINITIONS
add "^@macro_math_dsp_app_h_data/>" to list APP${INSTANCE}_H_APP_DATA
add "^@macro_math_dsp_app_h_callback_function_declarations/>" to list APP${INSTANCE}_H_APP_CALLBACK_FUNCTION_DECLARATIONS
add "^@macro_math_dsp_app_h_function_declarations/>" to list APP${INSTANCE}_H_APP_FUNCTION_DECLARATIONS
add "^@macro_math_dsp_app_h_states/>" to list APP${INSTANCE}_H_APP_STATES

add "^@macro_math_dsp_app_c_includes/>" to list APP${INSTANCE}_C_INCLUDES
add "^@macro_math_dsp_app_c_global_data/>" to list APP${INSTANCE}_C_GLOBAL_DATA
add "^@macro_math_dsp_app_c_callback_functions/>" to list APP${INSTANCE}_C_CALLBACK_FUNCTIONS
add "^@macro_math_dsp_app_c_local_functions/>" to list APP${INSTANCE}_C_LOCAL_FUNCTIONS
add "^@macro_math_dsp_app_c_initialize/>" to list APP${INSTANCE}_C_INITIALIZE
add "^@macro_math_dsp_app_c_tasks_data/>" to list APP${INSTANCE}_C_TASKS_DATA
add "^@macro_math_dsp_app_c_tasks_state_init/>" to list APP${INSTANCE}_C_TASKS_STATE_INIT
add "^@macro_math_dsp_app_c_tasks_calls_after_init/>" to list APP${INSTANCE}_C_TASKS_CALLS_AFTER_INIT
add "^@macro_math_dsp_app_c_tasks_state_service_tasks/>" to list APP${INSTANCE}_C_TASKS_STATE_SERVICE_TASKS
add "^@macro_math_dsp_app_c_tasks_states/>" to list APP${INSTANCE}_C_TASKS_STATES
add "^@macro_math_dsp_app_c_tasks_app_functions/>" to list APP${INSTANCE}_C_APP_TASKS_APP_FUNCTIONS

endif

endmenu
