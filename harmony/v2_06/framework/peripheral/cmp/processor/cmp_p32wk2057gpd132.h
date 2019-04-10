/* Created by plibgen $Revision: 1.31 $ */

#ifndef _CMP_P32WK2057GPD132_H
#define _CMP_P32WK2057GPD132_H

/* Section 1 - Enumerate instances, define constants, VREGs */

#include <xc.h>
#include <stdbool.h>

#include "peripheral/peripheral_common_32bit.h"

/* Default definition used for all API dispatch functions */
#ifndef PLIB_INLINE_API
    #define PLIB_INLINE_API extern inline
#endif

/* Default definition used for all other functions */
#ifndef PLIB_INLINE
    #define PLIB_INLINE extern inline
#endif

typedef enum {

    CMP_MODULE_ID_NONE

} CMP_MODULE_ID;

typedef enum {

    CMP_CVREF_VOLTAGE_SOURCE_NEG_REFERENCE_NONE

} CMP_CVREF_VOLTAGE_SOURCE_NEG_REFERENCE;

typedef enum {

    CMP_MASK_C_NONE

} CMP_MASK_C;

typedef enum {

    CMP_MASK_B_NONE

} CMP_MASK_B;

typedef enum {

    CMP_MASK_A_NONE

} CMP_MASK_A;

typedef enum {

    CMP_SPEED_POWER_NONE

} CMP_SPEED_POWER;

typedef enum {

    CMP_FILTER_CLOCK_NONE

} CMP_FILTER_CLOCK;

typedef enum {

    CMP_CLOCK_DIVIDE_NONE

} CMP_CLOCK_DIVIDE;

typedef enum {

    CMP_CVREF_VALUE_NONE

} CMP_CVREF_VALUE;

typedef enum {

    CMP_CVREF_VOLTAGE_SOURCE_NONE

} CMP_CVREF_VOLTAGE_SOURCE;

typedef enum {

    CMP_INTERRUPT_EVENT_NONE

} CMP_INTERRUPT_EVENT;

typedef enum {

    CMP_INVERTING_INPUT_NONE

} CMP_INVERTING_INPUT;

typedef enum {

    CMP_NON_INVERTING_INPUT_NONE

} CMP_NON_INVERTING_INPUT;

typedef enum {

    CMP_CVREF_REFERENCE_SELECT_NONE

} CMP_CVREF_REFERENCE_SELECT;

typedef enum {

    CMP_CVREF_BANDGAP_SELECT_NONE

} CMP_CVREF_BANDGAP_SELECT;

typedef enum {

    CMP_OUTPUT_FILTER_CLK_DIV_NONE

} CMP_OUTPUT_FILTER_CLK_DIV;

typedef enum {

    CMP_OUTPUT_FILTER_CLK_SOURCE_NONE

} CMP_OUTPUT_FILTER_CLK_SOURCE;

typedef enum {

    CMP_OUTPUT_MASK_SOURCE_NONE

} CMP_OUTPUT_MASK_SOURCE;

typedef enum {

    CMP_INV_AND_GATE_OUTPUT_TO_OR_GATE_INPUT_NONE

} CMP_INV_AND_GATE_OUTPUT_TO_OR_GATE_INPUT;

typedef enum {

    CMP_NON_INV_AND_GATE_OUTPUT_TO_OR_GATE_INPUT_NONE

} CMP_NON_INV_AND_GATE_OUTPUT_TO_OR_GATE_INPUT;

typedef enum {

    CMP_OUTPUT_MASKING_LEVEL_NONE

} CMP_OUTPUT_MASKING_LEVEL;

typedef enum {

    CMP_MASK_ANDOR_LOGIC_NONE

} CMP_MASK_ANDOR_LOGIC;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_CMP_ExistsEnableControl(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_Enable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_Disable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsOutputEnableControl(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_OutputEnable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_OutputDisable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsInvertOutputControl(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_OutputInvertEnable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_OutputInvertDisable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsOutputStatusGet(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_CMP_OutputStatusGet(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_CMP_ExistsInterruptEventSelect(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_InterruptEventSelect(CMP_MODULE_ID index, CMP_INTERRUPT_EVENT event)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsNonInvertingInputSelect(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_NonInvertingInputChannelSelect(CMP_MODULE_ID index, CMP_NON_INVERTING_INPUT input)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsInvertingInputSelect(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_InvertingInputChannelSelect(CMP_MODULE_ID index, CMP_INVERTING_INPUT channel)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsStopInIdle(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_StopInIdleModeEnable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_StopInIdleModeDisable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsCVREFEnableControl(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_Enable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_Disable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsCVREFOutputEnableControl(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_OutputEnable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_OutputDisable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsCVREFWideRangeControl(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_WideRangeEnable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_WideRangeDisable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_CMP_CVREF_WideRangeIsEnabled(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_CMP_ExistsCVREFVoltageRangeSelect(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_SourceVoltageSelect(CMP_MODULE_ID index, CMP_CVREF_VOLTAGE_SOURCE source)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsCVREFRefVoltageRangeSelect(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_ReferenceVoltageSelect(CMP_MODULE_ID index, CMP_CVREF_REFERENCE_SELECT reference)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsCVREFBGRefVoltageRangeSelect(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_BandGapReferenceSourceSelect(CMP_MODULE_ID index, CMP_CVREF_BANDGAP_SELECT select)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsCVREFValueSelect(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_ValueSelect(CMP_MODULE_ID index, CMP_CVREF_VALUE value)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsComparatorEventStatusGet(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_CMP_ComparatorEventStatusGet(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_CMP_ExistsComparatorOutputDigitalFilter(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_ComparatorOutputDigitalFilterClkSetup(CMP_MODULE_ID index, CMP_OUTPUT_FILTER_CLK_SOURCE clkSource, CMP_OUTPUT_FILTER_CLK_DIV clkDivider)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_ComparatorOutputDigitalFilterEnable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_ComparatorOutputDigitalFilterDisable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsOpAmpOutputControl(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_OpAmpOutputEnable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_OpAmpOutputDisable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsOpAmpEnableControl(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_OpAmpEnable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_OpAmpDisable(CMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_CMP_ExistsComparatorOutputMasking(CMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_ComparatorOutputMaskingSetup(CMP_MODULE_ID index, CMP_OUTPUT_MASK_SOURCE sourceA, CMP_OUTPUT_MASK_SOURCE sourceB, CMP_OUTPUT_MASK_SOURCE sourceC, uint32_t logicSettings, CMP_NON_INV_AND_GATE_OUTPUT_TO_OR_GATE_INPUT andGatePositiveOutput, CMP_INV_AND_GATE_OUTPUT_TO_OR_GATE_INPUT andGateNegativeOutput, CMP_OUTPUT_MASKING_LEVEL maskingLevels)
{
     
}

#endif
