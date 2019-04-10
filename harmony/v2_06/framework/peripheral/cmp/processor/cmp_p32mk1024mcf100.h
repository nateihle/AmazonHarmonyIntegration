/* Created by plibgen $Revision: 1.31 $ */

#ifndef _CMP_P32MK1024MCF100_H
#define _CMP_P32MK1024MCF100_H

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

    CMP_ID_1 = _CMP1_BASE_ADDRESS,
    CMP_ID_2 = _CMP2_BASE_ADDRESS,
    CMP_ID_3 = _CMP3_BASE_ADDRESS,
    CMP_ID_4 = _CMP4_BASE_ADDRESS,
    CMP_ID_5 = _CMP5_BASE_ADDRESS,
    CMP_NUMBER_OF_MODULES = 5

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

    CMP_INTERRUPT_GENERATION_DISABLED = 0x00,
    CMP_INTERRUPT_GENERATION_LOW_TO_HIGH = 0x01,
    CMP_INTERRUPT_GENERATION_HIGH_TO_LOW = 0x02,
    CMP_INTERRUPT_GENERATION_BOTH = 0x03

} CMP_INTERRUPT_EVENT;

typedef enum {

    CMP_INVERTING_INPUT_1 = 0x00,
    CMP_INVERTING_INPUT_2 = 0x01,
    CMP_INVERTING_INPUT_3 = 0x02,
    CMP_INVERTING_INPUT_4 = 0x03

} CMP_INVERTING_INPUT;

typedef enum {

    CMP_NON_INVERTING_INPUT_1 = 0x00,
    CMP_NON_INVERTING_INPUT_CDAC = 0x01

} CMP_NON_INVERTING_INPUT;

typedef enum {

    CMP_CVREF_REFERENCE_SELECT_NONE

} CMP_CVREF_REFERENCE_SELECT;

typedef enum {

    CMP_CVREF_BANDGAP_SELECT_NONE

} CMP_CVREF_BANDGAP_SELECT;

typedef enum {

    CMP_OUTPUT_FILTER_CLK_DIV_1 = 0x00,
    CMP_OUTPUT_FILTER_CLK_DIV_2 = 0x01,
    CMP_OUTPUT_FILTER_CLK_DIV_4 = 0x02,
    CMP_OUTPUT_FILTER_CLK_DIV_8 = 0x03,
    CMP_OUTPUT_FILTER_CLK_DIV_16 = 0x04,
    CMP_OUTPUT_FILTER_CLK_DIV_32 = 0x05,
    CMP_OUTPUT_FILTER_CLK_DIV_64 = 0x06,
    CMP_OUTPUT_FILTER_CLK_DIV_128 = 0x07

} CMP_OUTPUT_FILTER_CLK_DIV;

typedef enum {

    CMP_OUTPUT_FILTER_CLK_SYSCLK = 0x00,
    CMP_OUTPUT_FILTER_CLK_PBCLK2 = 0x01,
    CMP_OUTPUT_FILTER_CLK_REFCLK3 = 0x03,
    CMP_OUTPUT_FILTER_CLK_TMR2 = 0x04,
    CMP_OUTPUT_FILTER_CLK_TMR3 = 0x05,
    CMP_OUTPUT_FILTER_CLK_TMR4 = 0x06,
    CMP_OUTPUT_FILTER_CLK_TMR5 = 0x07

} CMP_OUTPUT_FILTER_CLK_SOURCE;

typedef enum {

    CMP_CMP_OUTPUT_MASK_PWM1L = 0x00,
    CMP_CMP_OUTPUT_MASK_PWM1H = 0x01,
    CMP_CMP_OUTPUT_MASK_PWM2L = 0x02,
    CMP_CMP_OUTPUT_MASK_PWM2H = 0x03,
    CMP_CMP_OUTPUT_MASK_PWM3L = 0x04,
    CMP_CMP_OUTPUT_MASK_PWM3H = 0x05,
    CMP_CMP_OUTPUT_MASK_PWM4L = 0x06,
    CMP_CMP_OUTPUT_MASK_PWM4H = 0x07,
    CMP_CMP_OUTPUT_MASK_PWM5L = 0x08,
    CMP_CMP_OUTPUT_MASK_PWM5H = 0x09,
    CMP_CMP_OUTPUT_MASK_PWM6L = 0x0A,
    CMP_CMP_OUTPUT_MASK_PWM6H = 0x0B,
    CMP_CMP_OUTPUT_MASK_PWMFLT1 = 0x00E,
    CMP_CMP_OUTPUT_MASK_PWMFLT2 = 0x00F

} CMP_OUTPUT_MASK_SOURCE;

typedef enum {

    CMP_INV_AND_GATE_OUTPUT_TO_OR_GATE_DISABLED = 0x00,
    CMP_INV_AND_GATE_OUTPUT_TO_OR_GATE_ENABLED = 0x01

} CMP_INV_AND_GATE_OUTPUT_TO_OR_GATE_INPUT;

typedef enum {

    CMP_NON_INV_AND_GATE_OUTPUT_TO_OR_GATE_DISABLED = 0x00,
    CMP_NON_INV_AND_GATE_OUTPUT_TO_OR_GATE_ENABLED = 0x01

} CMP_NON_INV_AND_GATE_OUTPUT_TO_OR_GATE_INPUT;

typedef enum {

    CMP_OUTPUT_MASKING_LOW_LEVEL = 0x00,
    CMP_OUTPUT_MASKING_HIGH_LEVEL = 0x01

} CMP_OUTPUT_MASKING_LEVEL;

typedef enum {

    CMP_OUTPUT_MASK_A_AND_GATE_INPUT_INV = 0x01,
    CMP_OUTPUT_MASK_A_AND_GATE_INPUT_NON_INV = 0x02,
    CMP_OUTPUT_MASK_B_AND_GATE_INPUT_INV = 0x04,
    CMP_OUTPUT_MASK_B_AND_GATE_INPUT_NON_INV = 0x08,
    CMP_OUTPUT_MASK_C_AND_GATE_INPUT_INV = 0x10,
    CMP_OUTPUT_MASK_C_AND_GATE_INPUT_NON_INV = 0x20,
    CMP_OUTPUT_MASK_A_OR_GATE_INPUT_INV = 0x100,
    CMP_OUTPUT_MASK_A_OR_GATE_INPUT_NON_INV = 0x200,
    CMP_OUTPUT_MASK_B_OR_GATE_INPUT_INV = 0x400,
    CMP_OUTPUT_MASK_B_OR_GATE_INPUT_NON_INV = 0x800,
    CMP_OUTPUT_MASK_C_OR_GATE_INPUT_INV = 0x1000,
    CMP_OUTPUT_MASK_C_OR_GATE_INPUT_NON_INV = 0x2000

} CMP_MASK_ANDOR_LOGIC;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/cmp_ComparatorEnableControl_Default.h"
#include "../templates/cmp_ComparatorOutputEnableControl_Default.h"
#include "../templates/cmp_InvertOutputSelectControl_Default.h"
#include "../templates/cmp_OutputStatusGet_Default.h"
#include "../templates/cmp_InterruptEventSelect_Default.h"
#include "../templates/cmp_NonInvertingInputSelect_Default.h"
#include "../templates/cmp_InvertingInputSelect_Default.h"
#include "../templates/cmp_StopInIdle_Default.h"
#include "../templates/cmp_CVREFEnableControl_Unsupported.h"
#include "../templates/cmp_CVREFOutputEnableControl_Unsupported.h"
#include "../templates/cmp_CVREFWideRangeControl_Unsupported.h"
#include "../templates/cmp_CVREFVoltageRangeSelect_Unsupported.h"
#include "../templates/cmp_CVREFRefVoltageRangeSelect_Unsupported.h"
#include "../templates/cmp_CVREFBGRefVoltageRangeSelect_Unsupported.h"
#include "../templates/cmp_CVREFValueSelect_Unsupported.h"
#include "../templates/cmp_ComparatorEventStatusGet_Default.h"
#include "../templates/cmp_ComparatorOutputDigitalFilter_Default.h"
#include "../templates/cmp_OpAmpOutputEnableControl_Default.h"
#include "../templates/cmp_OpAmpEnableControl_Default.h"
#include "../templates/cmp_ComparatorOutputMasking_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_CMP_ExistsEnableControl(CMP_MODULE_ID index)
{
     return CMP_ExistsEnableControl_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_Enable(CMP_MODULE_ID index)
{
     CMP_Enable_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_Disable(CMP_MODULE_ID index)
{
     CMP_Disable_Default(index);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsOutputEnableControl(CMP_MODULE_ID index)
{
     return CMP_ExistsOutputEnableControl_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_OutputEnable(CMP_MODULE_ID index)
{
     CMP_OutputEnable_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_OutputDisable(CMP_MODULE_ID index)
{
     CMP_OutputDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsInvertOutputControl(CMP_MODULE_ID index)
{
     return CMP_ExistsInvertOutputControl_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_OutputInvertEnable(CMP_MODULE_ID index)
{
     CMP_OutputInvertEnable_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_OutputInvertDisable(CMP_MODULE_ID index)
{
     CMP_OutputInvertDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsOutputStatusGet(CMP_MODULE_ID index)
{
     return CMP_ExistsOutputStatusGet_Default(index);
}

PLIB_INLINE_API bool PLIB_CMP_OutputStatusGet(CMP_MODULE_ID index)
{
     return CMP_OutputStatusGet_Default(index);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsInterruptEventSelect(CMP_MODULE_ID index)
{
     return CMP_ExistsInterruptEventSelect_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_InterruptEventSelect(CMP_MODULE_ID index, CMP_INTERRUPT_EVENT event)
{
     CMP_InterruptEventSelect_Default(index, event);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsNonInvertingInputSelect(CMP_MODULE_ID index)
{
     return CMP_ExistsNonInvertingInputSelect_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_NonInvertingInputChannelSelect(CMP_MODULE_ID index, CMP_NON_INVERTING_INPUT input)
{
     CMP_NonInvertingInputChannelSelect_Default(index, input);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsInvertingInputSelect(CMP_MODULE_ID index)
{
     return CMP_ExistsInvertingInputSelect_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_InvertingInputChannelSelect(CMP_MODULE_ID index, CMP_INVERTING_INPUT channel)
{
     CMP_InvertingInputChannelSelect_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsStopInIdle(CMP_MODULE_ID index)
{
     return CMP_ExistsStopInIdle_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_StopInIdleModeEnable(CMP_MODULE_ID index)
{
     CMP_StopInIdleModeEnable_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_StopInIdleModeDisable(CMP_MODULE_ID index)
{
     CMP_StopInIdleModeDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsCVREFEnableControl(CMP_MODULE_ID index)
{
     return CMP_ExistsCVREFEnableControl_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_Enable(CMP_MODULE_ID index)
{
     CMP_CVREF_Enable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_Disable(CMP_MODULE_ID index)
{
     CMP_CVREF_Disable_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsCVREFOutputEnableControl(CMP_MODULE_ID index)
{
     return CMP_ExistsCVREFOutputEnableControl_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_OutputEnable(CMP_MODULE_ID index)
{
     CMP_CVREF_OutputEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_OutputDisable(CMP_MODULE_ID index)
{
     CMP_CVREF_OutputDisable_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsCVREFWideRangeControl(CMP_MODULE_ID index)
{
     return CMP_ExistsCVREFWideRangeControl_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_WideRangeEnable(CMP_MODULE_ID index)
{
     CMP_CVREF_WideRangeEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_WideRangeDisable(CMP_MODULE_ID index)
{
     CMP_CVREF_WideRangeDisable_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_CMP_CVREF_WideRangeIsEnabled(CMP_MODULE_ID index)
{
     return CMP_CVREF_WideRangeIsEnabled_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsCVREFVoltageRangeSelect(CMP_MODULE_ID index)
{
     return CMP_ExistsCVREFVoltageRangeSelect_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_SourceVoltageSelect(CMP_MODULE_ID index, CMP_CVREF_VOLTAGE_SOURCE source)
{
     CMP_CVREF_SourceVoltageSelect_Unsupported(index, source);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsCVREFRefVoltageRangeSelect(CMP_MODULE_ID index)
{
     return CMP_ExistsCVREFRefVoltageRangeSelect_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_ReferenceVoltageSelect(CMP_MODULE_ID index, CMP_CVREF_REFERENCE_SELECT reference)
{
     CMP_CVREF_ReferenceVoltageSelect_Unsupported(index, reference);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsCVREFBGRefVoltageRangeSelect(CMP_MODULE_ID index)
{
     return CMP_ExistsCVREFBGRefVoltageRangeSelect_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_BandGapReferenceSourceSelect(CMP_MODULE_ID index, CMP_CVREF_BANDGAP_SELECT select)
{
     CMP_CVREF_BandGapReferenceSourceSelect_Unsupported(index, select);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsCVREFValueSelect(CMP_MODULE_ID index)
{
     return CMP_ExistsCVREFValueSelect_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_CMP_CVREF_ValueSelect(CMP_MODULE_ID index, CMP_CVREF_VALUE value)
{
     CMP_CVREF_ValueSelect_Unsupported(index, value);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsComparatorEventStatusGet(CMP_MODULE_ID index)
{
     return CMP_ExistsComparatorEventStatusGet_Default(index);
}

PLIB_INLINE_API bool PLIB_CMP_ComparatorEventStatusGet(CMP_MODULE_ID index)
{
     return CMP_ComparatorEventStatusGet_Default(index);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsComparatorOutputDigitalFilter(CMP_MODULE_ID index)
{
     return CMP_ExistsComparatorOutputDigitalFilter_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_ComparatorOutputDigitalFilterClkSetup(CMP_MODULE_ID index, CMP_OUTPUT_FILTER_CLK_SOURCE clkSource, CMP_OUTPUT_FILTER_CLK_DIV clkDivider)
{
     CMP_ComparatorOutputDigitalFilterClkSetup_Default(index, clkSource, clkDivider);
}

PLIB_INLINE_API void PLIB_CMP_ComparatorOutputDigitalFilterEnable(CMP_MODULE_ID index)
{
     CMP_ComparatorOutputDigitalFilterEnable_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_ComparatorOutputDigitalFilterDisable(CMP_MODULE_ID index)
{
     CMP_ComparatorOutputDigitalFilterDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsOpAmpOutputControl(CMP_MODULE_ID index)
{
     return CMP_ExistsOpAmpOutputControl_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_OpAmpOutputEnable(CMP_MODULE_ID index)
{
     CMP_OpAmpOutputEnable_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_OpAmpOutputDisable(CMP_MODULE_ID index)
{
     CMP_OpAmpOutputDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsOpAmpEnableControl(CMP_MODULE_ID index)
{
     return CMP_ExistsOpAmpEnableControl_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_OpAmpEnable(CMP_MODULE_ID index)
{
     CMP_OpAmpEnable_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_OpAmpDisable(CMP_MODULE_ID index)
{
     CMP_OpAmpDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_CMP_ExistsComparatorOutputMasking(CMP_MODULE_ID index)
{
     return CMP_ExistsComparatorOutputMasking_Default(index);
}

PLIB_INLINE_API void PLIB_CMP_ComparatorOutputMaskingSetup(CMP_MODULE_ID index, CMP_OUTPUT_MASK_SOURCE sourceA, CMP_OUTPUT_MASK_SOURCE sourceB, CMP_OUTPUT_MASK_SOURCE sourceC, uint32_t logicSettings, CMP_NON_INV_AND_GATE_OUTPUT_TO_OR_GATE_INPUT andGatePositiveOutput, CMP_INV_AND_GATE_OUTPUT_TO_OR_GATE_INPUT andGateNegativeOutput, CMP_OUTPUT_MASKING_LEVEL maskingLevels)
{
     CMP_ComparatorOutputMaskingSetup_Default(index, sourceA, sourceB, sourceC, logicSettings, andGatePositiveOutput, andGateNegativeOutput, maskingLevels);
}

#endif
