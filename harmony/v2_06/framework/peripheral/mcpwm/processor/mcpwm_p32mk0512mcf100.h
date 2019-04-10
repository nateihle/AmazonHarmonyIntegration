/* Created by plibgen $Revision: 1.31 $ */

#ifndef _MCPWM_P32MK0512MCF100_H
#define _MCPWM_P32MK0512MCF100_H

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

#define _MCPWM_BASE_ADDRESS 	 0xBF82A000
#define _MCPWM_CHANNEL_OFFSET 	 0xBF82A0C0
#define _MCPWM_CHANNEL_SIZE 	 0x00000100

typedef enum {

    MCPWM_ID_0 = _MCPWM_BASE_ADDRESS,
    MCPWM_NUMBER_OF_MODULES = 1

} MCPWM_MODULE_ID;

typedef enum {

    MCPWM_CHANNEL1 = _MCPWM_CHANNEL_OFFSET+0*_MCPWM_CHANNEL_SIZE,
    MCPWM_CHANNEL2 = _MCPWM_CHANNEL_OFFSET+1*_MCPWM_CHANNEL_SIZE,
    MCPWM_CHANNEL3 = _MCPWM_CHANNEL_OFFSET+2*_MCPWM_CHANNEL_SIZE,
    MCPWM_CHANNEL4 = _MCPWM_CHANNEL_OFFSET+3*_MCPWM_CHANNEL_SIZE,
    MCPWM_CHANNEL5 = _MCPWM_CHANNEL_OFFSET+4*_MCPWM_CHANNEL_SIZE,
    MCPWM_CHANNEL6 = _MCPWM_CHANNEL_OFFSET+5*_MCPWM_CHANNEL_SIZE,
    MCPWM_CHANNEL7 = _MCPWM_CHANNEL_OFFSET+6*_MCPWM_CHANNEL_SIZE,
    MCPWM_CHANNEL8 = _MCPWM_CHANNEL_OFFSET+7*_MCPWM_CHANNEL_SIZE,
    MCPWM_CHANNEL9 = _MCPWM_CHANNEL_OFFSET+8*_MCPWM_CHANNEL_SIZE,
    MCPWM_CHANNEL10 = _MCPWM_CHANNEL_OFFSET+9*_MCPWM_CHANNEL_SIZE,
    MCPWM_CHANNEL11 = _MCPWM_CHANNEL_OFFSET+10*_MCPWM_CHANNEL_SIZE,
    MCPWM_CHANNEL12 = _MCPWM_CHANNEL_OFFSET+11*_MCPWM_CHANNEL_SIZE,
    MCPWM_NUMBER_OF_CHANNELS = 12

} MCPWM_CHANNEL_ID;

typedef enum {

    MCPWM_TRIGGER_DIVIDE_BY_1 = 0,
    MCPWM_TRIGGER_DIVIDE_BY_2 = 1,
    MCPWM_TRIGGER_DIVIDE_BY_3 = 2,
    MCPWM_TRIGGER_DIVIDE_BY_4 = 3,
    MCPWM_TRIGGER_DIVIDE_BY_5 = 4,
    MCPWM_TRIGGER_DIVIDE_BY_6 = 5,
    MCPWM_TRIGGER_DIVIDE_BY_7 = 6,
    MCPWM_TRIGGER_DIVIDE_BY_8 = 7,
    MCPWM_TRIGGER_DIVIDE_BY_9 = 8,
    MCPWM_TRIGGER_DIVIDE_BY_10 = 9,
    MCPWM_TRIGGER_DIVIDE_BY_11 = 10,
    MCPWM_TRIGGER_DIVIDE_BY_12 = 11,
    MCPWM_TRIGGER_DIVIDE_BY_13 = 12,
    MCPWM_TRIGGER_DIVIDE_BY_14 = 13,
    MCPWM_TRIGGER_DIVIDE_BY_15 = 14,
    MCPWM_TRIGGER_DIVIDE_BY_16 = 15

} MCPWM_TRIGGER_DIVIDER;

typedef enum {

    MCPWM_CLOCK_DIVIDE_BY_1 = 0,
    MCPWM_CLOCK_DIVIDE_BY_2 = 1,
    MCPWM_CLOCK_DIVIDE_BY_4 = 2,
    MCPWM_CLOCK_DIVIDE_BY_8 = 3,
    MCPWM_CLOCK_DIVIDE_BY_16 = 4,
    MCPWM_CLOCK_DIVIDE_BY_32 = 5,
    MCPWM_CLOCK_DIVIDE_BY_64 = 6,
    MCPWM_CLOCK_DIVIDE_BY_128 = 7

} MCPWM_CLOCK_DIVIDER;

typedef enum {

    MCPWM_CHOP_CLOCK_DISABLE = 0,
    MCPWM_CHOP_CLOCK_ENABLE = 1

} MCPWM_CHOP_CLOCK_CONTROL;

typedef enum {

    MCPWM_TIME_BASE_SOURCE_PRIMARY = 0,
    MCPWM_TIME_BASE_SOURCE_SECONDARY = 1

} MCPWM_TIME_BASE_SOURCE;

typedef enum {

    MCPWM_TIME_BASE_SYNCHRONIZED = 0,
    MCPWM_TIME_BASE_INDEPENDENT = 1

} MCPWM_TIME_BASE_MODE;

typedef enum {

    MCPWM_EDGE_ALIGNED = 0,
    MCPWM_SYMMETRIC_CENTER_ALIGNED = 1,
    MCPWM_ASYMMETRIC_CENTER_ALIGNED = 2,
    MCPWM_ASYMMETRIC_CENTER_ALIGNED_WITH_SIMULTANEOUS_UPDATE = 3

} MCPWM_ALIGNMENT_MODE;

typedef enum {

    MCPWM_OUTPUT_COMPLIMENTARY_MODE = 0,
    MCPWM_OUTPUT_REDUNDANT_MODE = 1,
    MCPWM_OUTPUT_PUSH_PULL_MODE = 2,
    MCPWM_OUTPUT_LOW_LATCHED_TO_ZERO = 3

} MCPWM_OUTPUT_MODE;

typedef enum {

    MCPWM_PWMxH_ACTIVEHIGH = 0,
    MCPWM_PWMxH_ACTIVELOW = 1

} MCPWM_PWMxH_OUTPUT_POLARITY;

typedef enum {

    MCPWM_PWMxL_ACTIVEHIGH = 0,
    MCPWM_PWMxL_ACTIVELOW = 1

} MCPWM_PWMxL_OUTPUT_POLARITY;

typedef enum {

    MCPWM_DEADTIME_POSITIVE = 0,
    MCPWM_DEADTIME_NEGATIVE = 1,
    MCPWM_DEADTIME_DISABLE = 2,
    MCPWM_DEADTIME_COMPENSATION_ENABLED = 3

} MCPWM_DEADTIME_MODE;

typedef enum {

    MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_HIGH = 0,
    MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_LOW = 1

} MCPWM_DEADTIME_COMPENSATION_POLARITY;

typedef enum {

    MCPWM_TIMER_INCREMENTING = 0,
    MCPWM_TIMER_DECREMENTING = 1

} MCPWM_TIMER_DIRECTION;

typedef enum {

    MCPWM_PRIMARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING = 0,
    MCPWM_PRIMARY_TRIGGER_DURING_TIMER_DECREMENTING = 1,
    MCPWM_PRIMARY_TRIGGER_DURING_TIMER_INCREMENTING = 2

} MCPWM_PRIMARY_TRIGGER_CYCLE_SELECT;

typedef enum {

    MCPWM_SECONDARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING = 0,
    MCPWM_SECONDARY_TRIGGER_DURING_TIMER_DECREMENTING = 1,
    MCPWM_SECONDARY_TRIGGER_DURING_TIMER_INCREMENTING = 2

} MCPWM_SECONDARY_TRIGGER_CYCLE_SELECT;

typedef enum {

    MCPWM_ADC_TRIGGER_SOURCE_PRIMARY = 0,
    MCPWM_ADC_TRIGGER_SOURCE_PRIMARY_SECONDARY = 1

} MCPWM_ADC_TRIGGER_SOURCE;

typedef enum {

    MCPWM_TRIGGER_INTERRUPT_SOURCE_PRIMARY = 0,
    MCPWM_TRIGGER_INTERRUPT_SOURCE_SECONDARY = 1

} MCPWM_TRIGGER_INTERRUPT_SOURCE;

typedef enum {

    MCPWM_FAULT_INPUT_LEB_DISABLE = 0,
    MCPWM_FAULT_INPUT_LEB_ENABLE = 1

} MCPWM_FAULT_INPUT_LEB_CONTROL;

typedef enum {

    MCPWM_CURRENTLIMIT_INPUT_LEB_DISABLE = 0,
    MCPWM_CURRENTLIMIT_INPUT_LEB_ENABLE = 1

} MCPWM_CURRENTLIMIT_INPUT_LEB_CONTROL;

typedef enum {

    MCPWM_CHOP_CLOCK_SOURCE_IS_CHOP_CLOCK_GENERATOR = 0,
    MCPWM_CHOP_CLOCK_SOURCE_IS_PMW1H = 1,
    MCPWM_CHOP_CLOCK_SOURCE_IS_PMW2H = 2,
    MCPWM_CHOP_CLOCK_SOURCE_IS_PMW3H = 4,
    MCPWM_CHOP_CLOCK_SOURCE_IS_PMW4H = 5,
    MCPWM_CHOP_CLOCK_SOURCE_IS_PMW5H = 6,
    MCPWM_CHOP_CLOCK_SOURCE_IS_PMW6H = 7,
    MCPWM_CHOP_CLOCK_SOURCE_IS_PMW7H = 8,
    MCPWM_CHOP_CLOCK_SOURCE_IS_PMW8H = 9,
    MCPWM_CHOP_CLOCK_SOURCE_IS_PMW9H = 10,
    MCPWM_CHOP_CLOCK_SOURCE_IS_PMW10H = 11,
    MCPWM_CHOP_CLOCK_SOURCE_IS_PMW11H = 12,
    MCPWM_CHOP_CLOCK_SOURCE_IS_PMW12H = 13

} MCPWM_CHOP_CLOCK_SOURCE;

typedef enum {

    MCPWM_PWMxH_CHOP_DISABLED = 0,
    MCPWM_PWMxH_CHOP_ENABLED = 1

} MCPWM_PWMxH_CHOP_CONTROL;

typedef enum {

    MCPWM_PWMxL_CHOP_DISABLED = 0,
    MCPWM_PWMxL_CHOP_ENABLED = 1

} MCPWM_PWMxL_CHOP_CONTROL;

typedef enum {

    MCPWM_CURRENTLIMIT_SOURCE_IS_FLT1 = 0,
    MCPWM_CURRENTLIMIT_SOURCE_IS_FLT2 = 1,
    MCPWM_CURRENTLIMIT_SOURCE_IS_FLT3 = 2,
    MCPWM_CURRENTLIMIT_SOURCE_IS_FLT4 = 3,
    MCPWM_CURRENTLIMIT_SOURCE_IS_FLT5 = 4,
    MCPWM_CURRENTLIMIT_SOURCE_IS_FLT6 = 5,
    MCPWM_CURRENTLIMIT_SOURCE_IS_FLT7 = 6,
    MCPWM_CURRENTLIMIT_SOURCE_IS_FLT8 = 7,
    MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR1 = 8,
    MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR2 = 9,
    MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR3 = 10,
    MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR4 = 11,
    MCPWM_CURRENTLIMIT_SOURCE_IS_COMPARATOR5 = 12,
    MCPWM_CURRENTLIMIT_SOURCE_IS_FLT15 = 15

} MCPWM_CURRENTLIMIT_SOURCE;

typedef enum {

    MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH = 0,
    MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_LOW = 1

} MCPWM_CURRENTLIMIT_INPUT_POLARITY;

typedef enum {

    MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0 = 0,
    MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_1 = 1

} MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_VALUE;

typedef enum {

    MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0 = 0,
    MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_1 = 1

} MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_VALUE;

typedef enum {

    MCPWM_CURRENTLIMIT_DISABLE = 0,
    MCPWM_STANDARD_CURRENT_LIMIT_MODE_ENABLE = 1,
    MCPWM_XPRES_CURRENT_LIMIT_MODE_ENABLE = 2

} MCPWM_CURRENTLIMIT_MODE;

typedef enum {

    MCPWM_FAULT_SOURCE_IS_FLT1 = 0,
    MCPWM_FAULT_SOURCE_IS_FLT2 = 1,
    MCPWM_FAULT_SOURCE_IS_FLT3 = 2,
    MCPWM_FAULT_SOURCE_IS_FLT4 = 3,
    MCPWM_FAULT_SOURCE_IS_FLT5 = 4,
    MCPWM_FAULT_SOURCE_IS_FLT6 = 5,
    MCPWM_FAULT_SOURCE_IS_FLT7 = 6,
    MCPWM_FAULT_SOURCE_IS_FLT8 = 7,
    MCPWM_FAULT_SOURCE_IS_COMPARATOR1 = 8,
    MCPWM_FAULT_SOURCE_IS_COMPARATOR2 = 9,
    MCPWM_FAULT_SOURCE_IS_COMPARATOR3 = 10,
    MCPWM_FAULT_SOURCE_IS_COMPARATOR4 = 11,
    MCPWM_FAULT_SOURCE_IS_COMPARATOR5 = 12,
    MCPWM_FAULT_SOURCE_IS_FLT15 = 15

} MCPWM_FAULT_SOURCE;

typedef enum {

    MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH = 0,
    MCPWM_FAULT_INPUT_POLARITY_ACTIVE_LOW = 1

} MCPWM_FAULT_INPUT_POLARITY;

typedef enum {

    MCPWM_FAULT_OVERRIDE_PWMxH_0 = 0,
    MCPWM_FAULT_OVERRIDE_PWMxH_1 = 1

} MCPWM_FAULT_OVERRIDE_PWMxH_VALUE;

typedef enum {

    MCPWM_FAULT_OVERRIDE_PWMxL_0 = 0,
    MCPWM_FAULT_OVERRIDE_PWMxL_1 = 1

} MCPWM_FAULT_OVERRIDE_PWMxL_VALUE;

typedef enum {

    MCPWM_FAULT_MODE_LATCHED = 0,
    MCPWM_FAULT_MODE_CYCLE_BY_CYCLE = 1,
    MCPWM_FAULT_MODE_DISABLED = 3

} MCPWM_FAULT_MODE;

typedef enum {

    MCPWM_OVERRIDE_PWMxH_0 = 0,
    MCPWM_OVERRIDE_PWMxH_1 = 1

} MCPWM_OVERRIDE_PWMxH_VALUE;

typedef enum {

    MCPWM_OVERRIDE_PWMxL_0 = 0,
    MCPWM_OVERRIDE_PWMxL_1 = 1

} MCPWM_OVERRIDE_PWMxL_VALUE;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/mcpwm_ModuleControl_Default.h"
#include "../templates/mcpwm_PrimarySpecialEventTrigger_Default.h"
#include "../templates/mcpwm_SecondarySpecialEventTrigger_Default.h"
#include "../templates/mcpwm_PrimaryTimerSetup_Default.h"
#include "../templates/mcpwm_SecondaryTimerSetup_Default.h"
#include "../templates/mcpwm_ChopClockSetup_Default.h"
#include "../templates/mcpwm_ChannelSetup_Default.h"
#include "../templates/mcpwm_ChannelPrimaryDutyCycleSet_Default.h"
#include "../templates/mcpwm_ChannelSecondaryDutyCycleSet_Default.h"
#include "../templates/mcpwm_ChannelPhaseSet_Default.h"
#include "../templates/mcpwm_ChannelDeadtimeSetup_Default.h"
#include "../templates/mcpwm_ChannelTrigger_Default.h"
#include "../templates/mcpwm_ChannelCurrentLimit_Default.h"
#include "../templates/mcpwm_ChannelFault_Default.h"
#include "../templates/mcpwm_ChannelChopSetup_Default.h"
#include "../templates/mcpwm_ChannelLEBSetup_Default.h"
#include "../templates/mcpwm_ChannelPeriodResetInterrupt_Default.h"
#include "../templates/mcpwm_ChannelPeriodMatchInterrupt_Default.h"
#include "../templates/mcpwm_ChannelOverrideSetup_Default.h"
#include "../templates/mcpwm_ChannelGeneralFunctions_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void PLIB_MCPWM_Enable(MCPWM_MODULE_ID index)
{
     MCPWM_Enable_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_Disable(MCPWM_MODULE_ID index)
{
     MCPWM_Disable_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_StopInIdleEnable(MCPWM_MODULE_ID index)
{
     MCPWM_StopInIdleEnable_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_StopInIdleDisable(MCPWM_MODULE_ID index)
{
     MCPWM_StopInIdleDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_MCPWM_ModuleIsReady(MCPWM_MODULE_ID index)
{
     return MCPWM_ModuleIsReady_Default(index);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsModuleControl(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsModuleControl_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_PrimarySpecialEventTriggerSetup(MCPWM_MODULE_ID index, uint16_t compare_value, MCPWM_TRIGGER_DIVIDER postscaler_value)
{
     MCPWM_PrimarySpecialEventTriggerSetup_Default(index, compare_value, postscaler_value);
}

PLIB_INLINE_API bool PLIB_MCPWM_PrimarySpecialEventTriggerInterruptIsPending(MCPWM_MODULE_ID index)
{
     return MCPWM_PrimarySpecialEventTriggerInterruptIsPending_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_PrimarySpecialEventTriggerInterruptEnable(MCPWM_MODULE_ID index)
{
     MCPWM_PrimarySpecialEventTriggerInterruptEnable_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_PrimarySpecialEventTriggerInterruptDisable(MCPWM_MODULE_ID index)
{
     MCPWM_PrimarySpecialEventTriggerInterruptDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsPrimarySpecialEventTrigger(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsPrimarySpecialEventTrigger_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_SecondarySpecialEventTriggerSetup(MCPWM_MODULE_ID index, uint16_t compare_value, MCPWM_TRIGGER_DIVIDER postscaler_value)
{
     MCPWM_SecondarySpecialEventTriggerSetup_Default(index, compare_value, postscaler_value);
}

PLIB_INLINE_API bool PLIB_MCPWM_SecondarySpecialEventTriggerInterruptIsPending(MCPWM_MODULE_ID index)
{
     return MCPWM_SecondarySpecialEventTriggerInterruptIsPending_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_SecondarySpecialEventTriggerInterruptEnable(MCPWM_MODULE_ID index)
{
     MCPWM_SecondarySpecialEventTriggerInterruptEnable_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_SecondarySpecialEventTriggerInterruptDisable(MCPWM_MODULE_ID index)
{
     MCPWM_SecondarySpecialEventTriggerInterruptDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsSecondarySpecialEventTrigger(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsSecondarySpecialEventTrigger_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_PrimaryTimerSetup(MCPWM_MODULE_ID index, MCPWM_CLOCK_DIVIDER clock_div, uint16_t period_value)
{
     MCPWM_PrimaryTimerSetup_Default(index, clock_div, period_value);
}

PLIB_INLINE_API uint16_t PLIB_MCPWM_PrimaryTimerCountRead(MCPWM_MODULE_ID index)
{
     return MCPWM_PrimaryTimerCountRead_Default(index);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsPrimaryTimerSetup(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsPrimaryTimerSetup_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_SecondaryTimerSetup(MCPWM_MODULE_ID index, MCPWM_CLOCK_DIVIDER clock_div, uint16_t period_value)
{
     MCPWM_SecondaryTimerSetup_Default(index, clock_div, period_value);
}

PLIB_INLINE_API uint16_t PLIB_MCPWM_SecondaryTimerCountRead(MCPWM_MODULE_ID index)
{
     return MCPWM_SecondaryTimerCountRead_Default(index);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsSecondaryTimerSetup(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsSecondaryTimerSetup_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_ChopClockSetup(MCPWM_MODULE_ID index, uint16_t mcpwm_chop_clk_divider, MCPWM_CHOP_CLOCK_CONTROL mcpwm_chop_clk_control)
{
     MCPWM_ChopClockSetup_Default(index, mcpwm_chop_clk_divider, mcpwm_chop_clk_control);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChopClockSetup(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsChopClockSetup_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelSetup(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel_id, MCPWM_TIME_BASE_SOURCE time_base_source, MCPWM_TIME_BASE_MODE time_base_mode, MCPWM_ALIGNMENT_MODE mcpwm_alignment_mode, MCPWM_OUTPUT_MODE mcpwm_output_mode, MCPWM_PWMxH_OUTPUT_POLARITY mcpwm_pwmh_output_polarity, MCPWM_PWMxL_OUTPUT_POLARITY mcpwm_pwml_output_polarity, MCPWM_DEADTIME_MODE mcpwm_deadtime_mode, MCPWM_DEADTIME_COMPENSATION_POLARITY mcpwm_deadtime_compensation_polarity)
{
     MCPWM_ChannelSetup_Default(index, channel_id, time_base_source, time_base_mode, mcpwm_alignment_mode, mcpwm_output_mode, mcpwm_pwmh_output_polarity, mcpwm_pwml_output_polarity, mcpwm_deadtime_mode, mcpwm_deadtime_compensation_polarity);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelSetup(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsChannelSetup_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPrimaryDutyCycleSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value)
{
     MCPWM_ChannelPrimaryDutyCycleSet_Default(index, channel, value);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelPrimaryDutyCycleSet(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsChannelPrimaryDutyCycleSet_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelSecondaryDutyCycleSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value)
{
     MCPWM_ChannelSecondaryDutyCycleSet_Default(index, channel, value);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelSecondaryDutyCycleSet(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsChannelSecondaryDutyCycleSet_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPhaseSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value)
{
     MCPWM_ChannelPhaseSet_Default(index, channel, value);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelPhaseSet(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsChannelPhaseSet_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPWMxHDeadtimeSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value)
{
     MCPWM_ChannelPWMxHDeadtimeSet_Default(index, channel, value);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPWMxLDeadtimeSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value)
{
     MCPWM_ChannelPWMxLDeadtimeSet_Default(index, channel, value);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelDeadtimeCompSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t value)
{
     MCPWM_ChannelDeadtimeCompSet_Default(index, channel, value);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelDeadtimeSetup(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsChannelDeadtimeSetup_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelTriggerSetup(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_TRIGGER_DIVIDER trigger_postscaler_value, MCPWM_PRIMARY_TRIGGER_CYCLE_SELECT primary_trigger_cycle_select, MCPWM_SECONDARY_TRIGGER_CYCLE_SELECT secondary_trigger_cycle_select, MCPWM_ADC_TRIGGER_SOURCE mcpwm_adc_trigger_source, MCPWM_TRIGGER_INTERRUPT_SOURCE mcpwm_trigger_interrupt_source, uint16_t primary_trigger_compare_value, uint16_t secondary_trigger_compare_value)
{
     MCPWM_ChannelTriggerSetup_Default(index, channel, trigger_postscaler_value, primary_trigger_cycle_select, secondary_trigger_cycle_select, mcpwm_adc_trigger_source, mcpwm_trigger_interrupt_source, primary_trigger_compare_value, secondary_trigger_compare_value);
}

PLIB_INLINE_API bool PLIB_MCPWM_ChannelTriggerInterruptIsPending(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return MCPWM_ChannelTriggerInterruptIsPending_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_MCPWM_ChannelTriggerInterruptIsEnabled(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return MCPWM_ChannelTriggerInterruptIsEnabled_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelTriggerInterruptFlagClear(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelTriggerInterruptFlagClear_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelTriggerInterruptEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelTriggerInterruptEnable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelTriggerInterruptDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelTriggerInterruptDisable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPrimaryTriggerCompareSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t trigger_compare_value)
{
     MCPWM_ChannelPrimaryTriggerCompareSet_Default(index, channel, trigger_compare_value);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelSecondaryTriggerCompareSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, uint16_t trigger_compare_value)
{
     MCPWM_ChannelSecondaryTriggerCompareSet_Default(index, channel, trigger_compare_value);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelTrigger(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsChannelTrigger_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelCurrentLimitSetup(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_CURRENTLIMIT_SOURCE current_limit_source, MCPWM_CURRENTLIMIT_INPUT_POLARITY current_limit_input_polarity, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_VALUE current_limit_override_pwmh_value, MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_VALUE current_limit_override_pwml_value, MCPWM_CURRENTLIMIT_MODE current_limit_mode)
{
     MCPWM_ChannelCurrentLimitSetup_Default(index, channel, current_limit_source, current_limit_input_polarity, current_limit_override_pwmh_value, current_limit_override_pwml_value, current_limit_mode);
}

PLIB_INLINE_API bool PLIB_MCPWM_ChannelCurrentLimitInterruptIsPending(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return MCPWM_ChannelCurrentLimitInterruptIsPending_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_MCPWM_ChannelCurrentLimitInterruptIsEnabled(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return MCPWM_ChannelCurrentLimitInterruptIsEnabled_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelCurrentLimitInterruptFlagClear(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelCurrentLimitInterruptFlagClear_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelCurrentLimitInterruptEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelCurrentLimitInterruptEnable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelCurrentLimitInterruptDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelCurrentLimitInterruptDisable_Default(index, channel);
}

PLIB_INLINE_API uint16_t PLIB_MCPWM_ChannelCurrentLimitCaptureRead(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return MCPWM_ChannelCurrentLimitCaptureRead_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_MCPWM_ChannelCurrentLimitIsAsserted(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return MCPWM_ChannelCurrentLimitIsAsserted_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelCurrentLimit(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsChannelCurrentLimit_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelFaultSetup(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_FAULT_SOURCE mcpwm_fault_source, MCPWM_FAULT_INPUT_POLARITY mcpwm_fault_input_polarity, MCPWM_FAULT_OVERRIDE_PWMxH_VALUE mcpwm_fault_override_pwmh_value, MCPWM_FAULT_OVERRIDE_PWMxL_VALUE mcpwm_fault_override_pwml_value, MCPWM_FAULT_MODE mcpwm_fault_mode)
{
     MCPWM_ChannelFaultSetup_Default(index, channel, mcpwm_fault_source, mcpwm_fault_input_polarity, mcpwm_fault_override_pwmh_value, mcpwm_fault_override_pwml_value, mcpwm_fault_mode);
}

PLIB_INLINE_API bool PLIB_MCPWM_ChannelFaultInterruptIsPending(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return MCPWM_ChannelFaultInterruptIsPending_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_MCPWM_ChannelFaultInterruptIsEnabled(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return MCPWM_ChannelFaultInterruptIsEnabled_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelFaultInterruptFlagClear(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelFaultInterruptFlagClear_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelFaultInterruptEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelFaultInterruptEnable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelFaultInterruptDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelFaultInterruptDisable_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_MCPWM_ChannelFaultIsAsserted(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return MCPWM_ChannelFaultIsAsserted_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelFault(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsChannelFault_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelChopSetup(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_CHOP_CLOCK_SOURCE chop_clock_source, MCPWM_PWMxH_CHOP_CONTROL mcpwm_pwmh_chop_control, MCPWM_PWMxL_CHOP_CONTROL mcpwm_pwml_chop_control)
{
     MCPWM_ChannelChopSetup_Default(index, channel, chop_clock_source, mcpwm_pwmh_chop_control, mcpwm_pwml_chop_control);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelChopSetup(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsChannelChopSetup_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelLEBSetup(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_FAULT_INPUT_LEB_CONTROL mcpwm_fault_input_leb_control, MCPWM_CURRENTLIMIT_INPUT_LEB_CONTROL mcpwm_currentlimit_input_leb_control, uint16_t leb_delay)
{
     MCPWM_ChannelLEBSetup_Default(index, channel, mcpwm_fault_input_leb_control, mcpwm_currentlimit_input_leb_control, leb_delay);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelLEBTriggerPWMxHRisingEdgeEnable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelLEBTriggerPWMxHFallingEdgeEnable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelLEBTriggerPWMxLRisingEdgeEnable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelLEBTriggerPWMxLFallingEdgeEnable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelLEBTriggerPWMxHRisingEdgeDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelLEBTriggerPWMxHRisingEdgeDisable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelLEBTriggerPWMxHFallingEdgeDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelLEBTriggerPWMxHFallingEdgeDisable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelLEBTriggerPWMxLRisingEdgeDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelLEBTriggerPWMxLRisingEdgeDisable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelLEBTriggerPWMxLFallingEdgeDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelLEBTriggerPWMxLFallingEdgeDisable_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelLEBSetup(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsChannelLEBSetup_Default(index);
}

PLIB_INLINE_API bool PLIB_MCPWM_ChannelPeriodResetInterruptIsPending(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return MCPWM_ChannelPeriodResetInterruptIsPending_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_MCPWM_ChannelPeriodResetInterruptIsEnabled(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return MCPWM_ChannelPeriodResetInterruptIsEnabled_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPeriodResetInterruptFlagClear(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelPeriodResetInterruptFlagClear_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPeriodResetInterruptEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelPeriodResetInterruptEnable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPeriodResetInterruptDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelPeriodResetInterruptDisable_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelPeriodResetInterrupt(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsChannelPeriodResetInterrupt_Default(index);
}

PLIB_INLINE_API bool PLIB_MCPWM_ChannelPeriodMatchInterruptIsPending(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return MCPWM_ChannelPeriodMatchInterruptIsPending_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_MCPWM_ChannelPeriodMatchInterruptIsEnabled(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return MCPWM_ChannelPeriodMatchInterruptIsEnabled_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPeriodMatchInterruptFlagClear(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelPeriodMatchInterruptFlagClear_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPeriodMatchInterruptEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelPeriodMatchInterruptEnable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPeriodMatchInterruptDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelPeriodMatchInterruptDisable_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelPeriodMatchInterrupt(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsChannelPeriodMatchInterrupt_Default(index);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPWMxHOverrideEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelPWMxHOverrideEnable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPWMxHOverrideDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelPWMxHOverrideDisable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPWMxLOverrideEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelPWMxLOverrideEnable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPWMxLOverrideDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelPWMxLOverrideDisable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelOverrideOutputSet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel, MCPWM_OVERRIDE_PWMxH_VALUE override_pwmh_value, MCPWM_OVERRIDE_PWMxL_VALUE override_pwml_value)
{
     MCPWM_ChannelOverrideOutputSet_Default(index, channel, override_pwmh_value, override_pwml_value);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelSyncOverrideAtPeriodBoundary(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelSyncOverrideAtPeriodBoundary_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelSyncOverrideAtCPUClockBoundary(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelSyncOverrideAtCPUClockBoundary_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelOverrideSetup(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsChannelOverrideSetup_Default(index);
}

PLIB_INLINE_API uint16_t PLIB_MCPWM_ChannelLocalPWMTimerCountRead(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return MCPWM_ChannelLocalPWMTimerCountRead_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_IOCONxUnlock(MCPWM_MODULE_ID index)
{
     MCPWM_IOCONxUnlock_Default(index);
}

PLIB_INLINE_API bool PLIB_MCPWM_ChannelTimerDirectionGet(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     return MCPWM_ChannelTimerDirectionGet_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPWMxHEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelPWMxHEnable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPWMxHDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelPWMxHDisable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPWMxLEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelPWMxLEnable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelPWMxLDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelPWMxLDisable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelSwapHighLowEnable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelSwapHighLowEnable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_MCPWM_ChannelSwapHighLowDisable(MCPWM_MODULE_ID index, MCPWM_CHANNEL_ID channel)
{
     MCPWM_ChannelSwapHighLowDisable_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_MCPWM_ExistsChannelGeneralFunctions(MCPWM_MODULE_ID index)
{
     return MCPWM_ExistsChannelGeneralFunctions_Default(index);
}

#endif
