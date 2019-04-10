/* Created by plibgen $Revision: 1.31 $ */

#ifndef _TMR_P32MK1024GPD064_H
#define _TMR_P32MK1024GPD064_H

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

    TMR_ID_1 = _TMR1_BASE_ADDRESS,
    TMR_ID_2 = _TMR2_BASE_ADDRESS,
    TMR_ID_3 = _TMR3_BASE_ADDRESS,
    TMR_ID_4 = _TMR4_BASE_ADDRESS,
    TMR_ID_5 = _TMR5_BASE_ADDRESS,
    TMR_ID_6 = _TMR6_BASE_ADDRESS,
    TMR_ID_7 = _TMR7_BASE_ADDRESS,
    TMR_ID_8 = _TMR8_BASE_ADDRESS,
    TMR_ID_9 = _TMR9_BASE_ADDRESS,
    TMR_NUMBER_OF_MODULES = 9

} TMR_MODULE_ID;

typedef enum {

    TMR_PRESCALE_VALUE_1 = 0,
    TMR_PRESCALE_VALUE_2,
    TMR_PRESCALE_VALUE_4,
    TMR_PRESCALE_VALUE_8,
    TMR_PRESCALE_VALUE_16,
    TMR_PRESCALE_VALUE_32,
    TMR_PRESCALE_VALUE_64,
    TMR_PRESCALE_VALUE_256

} TMR_PRESCALE;

typedef enum {

    TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK = 0,
    TMR_CLOCK_SOURCE_EXTERNAL_INPUT_PIN,
    TMR_CLOCK_SOURCE_SECONDARY_OSCILLATOR,
    TMR_CLOCK_SOURCE_LOW_POWER_RC_OSCILLATOR

} TMR_CLOCK_SOURCE;

PLIB_INLINE SFR_DATA _TMR_PRESCALE_1(TMR_PRESCALE v)
{
    switch (v) {
        case TMR_PRESCALE_VALUE_1 :
            return (SFR_DATA)0x00;
        case TMR_PRESCALE_VALUE_2 :
            return (SFR_DATA)0x01;
        case TMR_PRESCALE_VALUE_4 :
            return (SFR_DATA)0x02;
        case TMR_PRESCALE_VALUE_8 :
            return (SFR_DATA)0x03;
        case TMR_PRESCALE_VALUE_16 :
            return (SFR_DATA)0x04;
        case TMR_PRESCALE_VALUE_32 :
            return (SFR_DATA)0x05;
        case TMR_PRESCALE_VALUE_64 :
            return (SFR_DATA)0x06;
        case TMR_PRESCALE_VALUE_256 :
            return (SFR_DATA)0x07;
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_PRESCALE_2(TMR_PRESCALE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_PRESCALE_3(TMR_PRESCALE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_PRESCALE_4(TMR_PRESCALE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_PRESCALE_5(TMR_PRESCALE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_PRESCALE_6(TMR_PRESCALE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_PRESCALE_7(TMR_PRESCALE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_PRESCALE_8(TMR_PRESCALE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_PRESCALE_9(TMR_PRESCALE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_CLOCK_SOURCE_1(TMR_CLOCK_SOURCE v)
{
    switch (v) {
        case TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK :
            return (SFR_DATA)4;
        case TMR_CLOCK_SOURCE_EXTERNAL_INPUT_PIN :
            return (SFR_DATA)1;
        case TMR_CLOCK_SOURCE_SECONDARY_OSCILLATOR :
            return (SFR_DATA)0;
        case TMR_CLOCK_SOURCE_LOW_POWER_RC_OSCILLATOR :
            return (SFR_DATA)2;
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_CLOCK_SOURCE_2(TMR_CLOCK_SOURCE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_CLOCK_SOURCE_3(TMR_CLOCK_SOURCE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_CLOCK_SOURCE_4(TMR_CLOCK_SOURCE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_CLOCK_SOURCE_5(TMR_CLOCK_SOURCE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_CLOCK_SOURCE_6(TMR_CLOCK_SOURCE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_CLOCK_SOURCE_7(TMR_CLOCK_SOURCE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_CLOCK_SOURCE_8(TMR_CLOCK_SOURCE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE SFR_DATA _TMR_CLOCK_SOURCE_9(TMR_CLOCK_SOURCE v)
{
    switch (v) {
        default :
            return (SFR_DATA)-1;
    }
}

PLIB_INLINE TMR_PRESCALE _INV_TMR_PRESCALE_1(SFR_DATA v)
{
    switch (v) {
        case 0x00 :
            return (SFR_DATA)TMR_PRESCALE_VALUE_1;
        case 0x01 :
            return (SFR_DATA)TMR_PRESCALE_VALUE_2;
        case 0x02 :
            return (SFR_DATA)TMR_PRESCALE_VALUE_4;
        case 0x03 :
            return (SFR_DATA)TMR_PRESCALE_VALUE_8;
        case 0x04 :
            return (SFR_DATA)TMR_PRESCALE_VALUE_16;
        case 0x05 :
            return (SFR_DATA)TMR_PRESCALE_VALUE_32;
        case 0x06 :
            return (SFR_DATA)TMR_PRESCALE_VALUE_64;
        case 0x07 :
            return (SFR_DATA)TMR_PRESCALE_VALUE_256;
        default :
            return (TMR_PRESCALE)-1;
    }
}

PLIB_INLINE TMR_PRESCALE _INV_TMR_PRESCALE_2(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_PRESCALE)-1;
    }
}

PLIB_INLINE TMR_PRESCALE _INV_TMR_PRESCALE_3(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_PRESCALE)-1;
    }
}

PLIB_INLINE TMR_PRESCALE _INV_TMR_PRESCALE_4(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_PRESCALE)-1;
    }
}

PLIB_INLINE TMR_PRESCALE _INV_TMR_PRESCALE_5(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_PRESCALE)-1;
    }
}

PLIB_INLINE TMR_PRESCALE _INV_TMR_PRESCALE_6(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_PRESCALE)-1;
    }
}

PLIB_INLINE TMR_PRESCALE _INV_TMR_PRESCALE_7(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_PRESCALE)-1;
    }
}

PLIB_INLINE TMR_PRESCALE _INV_TMR_PRESCALE_8(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_PRESCALE)-1;
    }
}

PLIB_INLINE TMR_PRESCALE _INV_TMR_PRESCALE_9(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_PRESCALE)-1;
    }
}

PLIB_INLINE TMR_CLOCK_SOURCE _INV_TMR_CLOCK_SOURCE_1(SFR_DATA v)
{
    switch (v) {
        case 4 :
            return (SFR_DATA)TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK;
        case 1 :
            return (SFR_DATA)TMR_CLOCK_SOURCE_EXTERNAL_INPUT_PIN;
        case 0 :
            return (SFR_DATA)TMR_CLOCK_SOURCE_SECONDARY_OSCILLATOR;
        case 2 :
            return (SFR_DATA)TMR_CLOCK_SOURCE_LOW_POWER_RC_OSCILLATOR;
        default :
            return (TMR_CLOCK_SOURCE)-1;
    }
}

PLIB_INLINE TMR_CLOCK_SOURCE _INV_TMR_CLOCK_SOURCE_2(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_CLOCK_SOURCE)-1;
    }
}

PLIB_INLINE TMR_CLOCK_SOURCE _INV_TMR_CLOCK_SOURCE_3(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_CLOCK_SOURCE)-1;
    }
}

PLIB_INLINE TMR_CLOCK_SOURCE _INV_TMR_CLOCK_SOURCE_4(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_CLOCK_SOURCE)-1;
    }
}

PLIB_INLINE TMR_CLOCK_SOURCE _INV_TMR_CLOCK_SOURCE_5(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_CLOCK_SOURCE)-1;
    }
}

PLIB_INLINE TMR_CLOCK_SOURCE _INV_TMR_CLOCK_SOURCE_6(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_CLOCK_SOURCE)-1;
    }
}

PLIB_INLINE TMR_CLOCK_SOURCE _INV_TMR_CLOCK_SOURCE_7(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_CLOCK_SOURCE)-1;
    }
}

PLIB_INLINE TMR_CLOCK_SOURCE _INV_TMR_CLOCK_SOURCE_8(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_CLOCK_SOURCE)-1;
    }
}

PLIB_INLINE TMR_CLOCK_SOURCE _INV_TMR_CLOCK_SOURCE_9(SFR_DATA v)
{
    switch (v) {
        default :
            return (TMR_CLOCK_SOURCE)-1;
    }
}

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/tmr_GatedTimeAccumulation_Default.h"
#include "../templates/tmr_ClockSource_Extended.h"
#include "../templates/tmr_Prescale_Default.h"
#include "../templates/tmr_ClockSourceSync_Inverted.h"
#include "../templates/tmr_Mode16Bit_Default_1.h"
#include "../templates/tmr_Mode32Bit_Default_1.h"
#include "../templates/tmr_EnableControl_Default.h"
#include "../templates/tmr_StopInIdle_Default.h"
#include "../templates/tmr_Counter16Bit_In32BitRegister.h"
#include "../templates/tmr_Counter32Bit_In32BitRegister.h"
#include "../templates/tmr_Period16Bit_In32BitRegister.h"
#include "../templates/tmr_Period32Bit_In32BitRegister.h"
#include "../templates/tmr_CounterAsyncWriteControl_Default.h"
#include "../templates/tmr_CounterAsyncWriteInProgress_Default.h"
#include "../templates/tmr_TimerOperationMode_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void PLIB_TMR_GateEnable(TMR_MODULE_ID index)
{
     TMR_GateEnable_Default(index);
}

PLIB_INLINE_API void PLIB_TMR_GateDisable(TMR_MODULE_ID index)
{
     TMR_GateDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_TMR_ExistsGatedTimeAccumulation(TMR_MODULE_ID index)
{
     return TMR_ExistsGatedTimeAccumulation_Default(index);
}

PLIB_INLINE_API void PLIB_TMR_ClockSourceSelect(TMR_MODULE_ID index, TMR_CLOCK_SOURCE source)
{
     TMR_ClockSourceSelect_Extended(index, _TMR_CLOCK_SOURCE_1(source));
}

PLIB_INLINE_API bool PLIB_TMR_ExistsClockSource(TMR_MODULE_ID index)
{
     return TMR_ExistsClockSource_Extended(index);
}

PLIB_INLINE_API void PLIB_TMR_PrescaleSelect(TMR_MODULE_ID index, TMR_PRESCALE prescale)
{
     TMR_PrescaleSelect_Default(index, _TMR_PRESCALE_1(prescale));
}

PLIB_INLINE_API uint16_t PLIB_TMR_PrescaleGet(TMR_MODULE_ID index)
{
     return TMR_PrescaleGet_Default(index);
}

PLIB_INLINE_API bool PLIB_TMR_ExistsPrescale(TMR_MODULE_ID index)
{
     return TMR_ExistsPrescale_Default(index);
}

PLIB_INLINE_API void PLIB_TMR_ClockSourceExternalSyncEnable(TMR_MODULE_ID index)
{
     TMR_ClockSourceExternalSyncEnable_Inverted(index);
}

PLIB_INLINE_API void PLIB_TMR_ClockSourceExternalSyncDisable(TMR_MODULE_ID index)
{
     TMR_ClockSourceExternalSyncDisable_Inverted(index);
}

PLIB_INLINE_API bool PLIB_TMR_ExistsClockSourceSync(TMR_MODULE_ID index)
{
     return TMR_ExistsClockSourceSync_Inverted(index);
}

PLIB_INLINE_API void PLIB_TMR_Mode16BitEnable(TMR_MODULE_ID index)
{
     TMR_Mode16BitEnable_Default_1(index);
}

PLIB_INLINE_API bool PLIB_TMR_ExistsMode16Bit(TMR_MODULE_ID index)
{
     return TMR_ExistsMode16Bit_Default_1(index);
}

PLIB_INLINE_API void PLIB_TMR_Mode32BitEnable(TMR_MODULE_ID index)
{
     TMR_Mode32BitEnable_Default_1(index);
}

PLIB_INLINE_API bool PLIB_TMR_ExistsMode32Bit(TMR_MODULE_ID index)
{
     return TMR_ExistsMode32Bit_Default_1(index);
}

PLIB_INLINE_API void PLIB_TMR_Start(TMR_MODULE_ID index)
{
     TMR_Start_Default(index);
}

PLIB_INLINE_API void PLIB_TMR_Stop(TMR_MODULE_ID index)
{
     TMR_Stop_Default(index);
}

PLIB_INLINE_API bool PLIB_TMR_ExistsEnableControl(TMR_MODULE_ID index)
{
     return TMR_ExistsEnableControl_Default(index);
}

PLIB_INLINE_API void PLIB_TMR_StopInIdleEnable(TMR_MODULE_ID index)
{
     TMR_StopInIdleEnable_Default(index);
}

PLIB_INLINE_API void PLIB_TMR_StopInIdleDisable(TMR_MODULE_ID index)
{
     TMR_StopInIdleDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_TMR_ExistsStopInIdleControl(TMR_MODULE_ID index)
{
     return TMR_ExistsStopInIdleControl_Default(index);
}

PLIB_INLINE_API void PLIB_TMR_Counter16BitSet(TMR_MODULE_ID index, uint16_t value)
{
     TMR_Counter16BitSet_In32BitRegister(index, value);
}

PLIB_INLINE_API uint16_t PLIB_TMR_Counter16BitGet(TMR_MODULE_ID index)
{
     return TMR_Counter16BitGet_In32BitRegister(index);
}

PLIB_INLINE_API void PLIB_TMR_Counter16BitClear(TMR_MODULE_ID index)
{
     TMR_Counter16BitClear_In32BitRegister(index);
}

PLIB_INLINE_API bool PLIB_TMR_ExistsCounter16Bit(TMR_MODULE_ID index)
{
     return TMR_ExistsCounter16Bit_In32BitRegister(index);
}

PLIB_INLINE_API void PLIB_TMR_Counter32BitSet(TMR_MODULE_ID index, uint32_t value)
{
     TMR_Counter32BitSet_In32BitRegister(index, value);
}

PLIB_INLINE_API uint32_t PLIB_TMR_Counter32BitGet(TMR_MODULE_ID index)
{
     return TMR_Counter32BitGet_In32BitRegister(index);
}

PLIB_INLINE_API void PLIB_TMR_Counter32BitClear(TMR_MODULE_ID index)
{
     TMR_Counter32BitClear_In32BitRegister(index);
}

PLIB_INLINE_API bool PLIB_TMR_ExistsCounter32Bit(TMR_MODULE_ID index)
{
     return TMR_ExistsCounter32Bit_In32BitRegister(index);
}

PLIB_INLINE_API void PLIB_TMR_Period16BitSet(TMR_MODULE_ID index, uint16_t period)
{
     TMR_Period16BitSet_In32BitRegister(index, period);
}

PLIB_INLINE_API uint16_t PLIB_TMR_Period16BitGet(TMR_MODULE_ID index)
{
     return TMR_Period16BitGet_In32BitRegister(index);
}

PLIB_INLINE_API bool PLIB_TMR_ExistsPeriod16Bit(TMR_MODULE_ID index)
{
     return TMR_ExistsPeriod16Bit_In32BitRegister(index);
}

PLIB_INLINE_API void PLIB_TMR_Period32BitSet(TMR_MODULE_ID index, uint32_t period)
{
     TMR_Period32BitSet_In32BitRegister(index, period);
}

PLIB_INLINE_API uint32_t PLIB_TMR_Period32BitGet(TMR_MODULE_ID index)
{
     return TMR_Period32BitGet_In32BitRegister(index);
}

PLIB_INLINE_API bool PLIB_TMR_ExistsPeriod32Bit(TMR_MODULE_ID index)
{
     return TMR_ExistsPeriod32Bit_In32BitRegister(index);
}

PLIB_INLINE_API void PLIB_TMR_CounterAsyncWriteEnable(TMR_MODULE_ID index)
{
     TMR_CounterAsyncWriteEnable_Default(index);
}

PLIB_INLINE_API void PLIB_TMR_CounterAsyncWriteDisable(TMR_MODULE_ID index)
{
     TMR_CounterAsyncWriteDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_TMR_ExistsCounterAsyncWriteControl(TMR_MODULE_ID index)
{
     return TMR_ExistsCounterAsyncWriteControl_Default(index);
}

PLIB_INLINE_API bool PLIB_TMR_CounterAsyncWriteInProgress(TMR_MODULE_ID index)
{
     return TMR_CounterAsyncWriteInProgress_Default(index);
}

PLIB_INLINE_API bool PLIB_TMR_ExistsCounterAsyncWriteInProgress(TMR_MODULE_ID index)
{
     return TMR_ExistsCounterAsyncWriteInProgress_Default(index);
}

PLIB_INLINE_API bool PLIB_TMR_IsPeriodMatchBased(TMR_MODULE_ID index)
{
     return TMR_IsPeriodMatchBased_Default(index);
}

PLIB_INLINE_API bool PLIB_TMR_ExistsTimerOperationMode(TMR_MODULE_ID index)
{
     return TMR_ExistsTimerOperationMode_Default(index);
}

#endif
