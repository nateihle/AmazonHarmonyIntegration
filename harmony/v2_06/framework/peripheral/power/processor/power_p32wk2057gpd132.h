/* Created by plibgen $Revision: 1.31 $ */

#ifndef _POWER_P32WK2057GPD132_H
#define _POWER_P32WK2057GPD132_H

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

    POWER_ID_0 = 0,
    POWER_NUMBER_OF_MODULES = 1

} POWER_MODULE_ID;

typedef enum {

    POWER_MODULE_ADC1 = 0x08,
    POWER_MODULE_IC1 = 0x20,
    POWER_MODULE_IC2 = 0x21,
    POWER_MODULE_IC3 = 0x22,
    POWER_MODULE_IC4 = 0x23,
    POWER_MODULE_OC1 = 0x28,
    POWER_MODULE_OC2 = 0x29,
    POWER_MODULE_OC3 = 0x2A,
    POWER_MODULE_OC4 = 0x2B,
    POWER_MODULE_TMR1 = 0x30,
    POWER_MODULE_TMR2 = 0x31,
    POWER_MODULE_TMR3 = 0x32,
    POWER_MODULE_TMR4 = 0x33,
    POWER_MODULE_TMR5 = 0x34,
    POWER_MODULE_TMR6 = 0x35,
    POWER_MODULE_TMR7 = 0x36,
    POWER_MODULE_UART1 = 0x40,
    POWER_MODULE_UART2 = 0x41,
    POWER_MODULE_SPI1 = 0x48,
    POWER_MODULE_SPI2 = 0x49,
    POWER_MODULE_I2C1 = 0x50,
    POWER_MODULE_I2C2 = 0x51,
    POWER_MODULE_USB = 0x58,
    POWER_MODULE_CAN1 = 0x5C,
    POWER_MODULE_RTCC = 0x10,
    POWER_MODULE_REF_CLK_OUTPUT1 = 0x3C,
    POWER_MODULE_REF_CLK_OUTPUT2 = 0x3D,
    POWER_MODULE_REF_CLK_OUTPUT3 = 0x3E,
    POWER_MODULE_REF_CLK_OUTPUT4 = 0x3F,
    POWER_MODULE_DMA = 0x13,
    POWER_MODULE_ADCSAR = 0x00,
    POWER_MODULE_ADCSARSHR = 0x07,
    POWER_MODULE_PTG = 0x0B

} POWER_MODULE;

typedef enum {

    HLVD_LIMIT_NONE

} HLVD_LIMIT;

typedef enum {

    HLVD_MODE_NONE

} HLVD_MODE;

typedef enum {

    DEEP_SLEEP_MODULE_NONE

} DEEP_SLEEP_MODULE;

typedef enum {

    DEEP_SLEEP_WAKE_UP_EVENT_NONE

} DEEP_SLEEP_WAKE_UP_EVENT;

typedef enum {

    DEEP_SLEEP_EVENT_NONE

} DEEP_SLEEP_EVENT;

typedef enum {

    DEEP_SLEEP_GPR_NONE

} DEEP_SLEEP_GPR;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/power_PeripheralModuleControl_WK.h"
#include "../templates/power_VoltageRegulatorControl_Unsupported.h"
#include "../templates/power_SleepStatus_Default.h"
#include "../templates/power_IdleStatus_Default.h"
#include "../templates/power_HighVoltageOnVDD1V8_Unsupported.h"
#include "../templates/power_DeepSleepModeOccurrence_Unsupported.h"
#include "../templates/power_HLVDEnableControl_Unsupported.h"
#include "../templates/power_HLVDStopInIdleControl_Unsupported.h"
#include "../templates/power_HLVDStatus_Unsupported.h"
#include "../templates/power_HLVDModeControl_Unsupported.h"
#include "../templates/power_HLVDBandGapVoltageStability_Unsupported.h"
#include "../templates/power_HLVDLimitSelection_Unsupported.h"
#include "../templates/power_DeepSleepModeControl_Unsupported.h"
#include "../templates/power_DeepSleepGPRsRetentionControl_Unsupported.h"
#include "../templates/power_DeepSleepModuleControl_Unsupported.h"
#include "../templates/power_DeepSleepWakeupEventControl_Unsupported.h"
#include "../templates/power_DeepSleepPortPinsStateControl_Unsupported.h"
#include "../templates/power_DeepSleepEventStatus_Unsupported.h"
#include "../templates/power_DeepSleepGPROperation_Unsupported.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_POWER_ExistsPeripheralModuleControl(POWER_MODULE_ID index)
{
     return POWER_ExistsPeripheralModuleControl_WK(index);
}

PLIB_INLINE_API void PLIB_POWER_PeripheralModuleDisable(POWER_MODULE_ID index, POWER_MODULE source)
{
     POWER_PeripheralModuleDisable_WK(index, source);
}

PLIB_INLINE_API void PLIB_POWER_PeripheralModuleEnable(POWER_MODULE_ID index, POWER_MODULE source)
{
     POWER_PeripheralModuleEnable_WK(index, source);
}

PLIB_INLINE_API bool PLIB_POWER_PeripheralModuleIsEnabled(POWER_MODULE_ID index, POWER_MODULE source)
{
     return POWER_PeripheralModuleIsEnabled_WK(index, source);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsVoltageRegulatorControl(POWER_MODULE_ID index)
{
     return POWER_ExistsVoltageRegulatorControl_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_VoltageRegulatorEnable(POWER_MODULE_ID index)
{
     POWER_VoltageRegulatorEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_VoltageRegulatorDisable(POWER_MODULE_ID index)
{
     POWER_VoltageRegulatorDisable_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_POWER_VoltageRegulatorIsEnabled(POWER_MODULE_ID index)
{
     return POWER_VoltageRegulatorIsEnabled_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsSleepStatus(POWER_MODULE_ID index)
{
     return POWER_ExistsSleepStatus_Default(index);
}

PLIB_INLINE_API bool PLIB_POWER_DeviceWasInSleepMode(POWER_MODULE_ID index)
{
     return POWER_DeviceWasInSleepMode_Default(index);
}

PLIB_INLINE_API void PLIB_POWER_ClearSleepStatus(POWER_MODULE_ID index)
{
     POWER_ClearSleepStatus_Default(index);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsIdleStatus(POWER_MODULE_ID index)
{
     return POWER_ExistsIdleStatus_Default(index);
}

PLIB_INLINE_API bool PLIB_POWER_DeviceWasInIdleMode(POWER_MODULE_ID index)
{
     return POWER_DeviceWasInIdleMode_Default(index);
}

PLIB_INLINE_API void PLIB_POWER_ClearIdleStatus(POWER_MODULE_ID index)
{
     POWER_ClearIdleStatus_Default(index);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsHighVoltageOnVDD1V8(POWER_MODULE_ID index)
{
     return POWER_ExistsHighVoltageOnVDD1V8_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_POWER_HighVoltageOnVDD1V8HasOccurred(POWER_MODULE_ID index)
{
     return POWER_HighVoltageOnVDD1V8HasOccurred_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsDeepSleepModeOccurrence(POWER_MODULE_ID index)
{
     return POWER_ExistsDeepSleepModeOccurrence_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepModeHasOccurred(POWER_MODULE_ID index)
{
     return POWER_DeepSleepModeHasOccurred_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepStatusClear(POWER_MODULE_ID index)
{
     POWER_DeepSleepStatusClear_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsHLVDEnableControl(POWER_MODULE_ID index)
{
     return POWER_ExistsHLVDEnableControl_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_HLVDEnable(POWER_MODULE_ID index)
{
     POWER_HLVDEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_HLVDDisable(POWER_MODULE_ID index)
{
     POWER_HLVDDisable_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_POWER_HLVDIsEnabled(POWER_MODULE_ID index)
{
     return POWER_HLVDIsEnabled_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsHLVDStopInIdleControl(POWER_MODULE_ID index)
{
     return POWER_ExistsHLVDStopInIdleControl_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_HLVDStopInIdleEnable(POWER_MODULE_ID index)
{
     POWER_HLVDStopInIdleEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_HLVDStopInIdleDisable(POWER_MODULE_ID index)
{
     POWER_HLVDStopInIdleDisable_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_POWER_HLVDStopInIdleIsEnabled(POWER_MODULE_ID index)
{
     return POWER_HLVDStopInIdleIsEnabled_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsHLVDStatus(POWER_MODULE_ID index)
{
     return POWER_ExistsHLVDStatus_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_POWER_HLVDStatusGet(POWER_MODULE_ID index)
{
     return POWER_HLVDStatusGet_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsHLVDModeControl(POWER_MODULE_ID index)
{
     return POWER_ExistsHLVDModeControl_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_HLVDModeSelect(POWER_MODULE_ID index, HLVD_MODE mode)
{
     POWER_HLVDModeSelect_Unsupported(index, mode);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsHLVDBandGapVoltageStability(POWER_MODULE_ID index)
{
     return POWER_ExistsHLVDBandGapVoltageStability_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_POWER_HLVDBandGapVoltageIsStable(POWER_MODULE_ID index)
{
     return POWER_HLVDBandGapVoltageIsStable_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsHLVDLimitSelection(POWER_MODULE_ID index)
{
     return POWER_ExistsHLVDLimitSelection_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_HLVDLimitSelect(POWER_MODULE_ID index, HLVD_LIMIT limit)
{
     POWER_HLVDLimitSelect_Unsupported(index, limit);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsDeepSleepMode(POWER_MODULE_ID index)
{
     return POWER_ExistsDeepSleepMode_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepModeEnable(POWER_MODULE_ID index)
{
     POWER_DeepSleepModeEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepModeDisable(POWER_MODULE_ID index)
{
     POWER_DeepSleepModeDisable_Unsupported(index);
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepModeIsEnabled(POWER_MODULE_ID index)
{
     return POWER_DeepSleepModeIsEnabled_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsDeepSleepGPRsRetentionControl(POWER_MODULE_ID index)
{
     return POWER_ExistsDeepSleepGPRsRetentionControl_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepGPRsRetentionEnable(POWER_MODULE_ID index)
{
     POWER_DeepSleepGPRsRetentionEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepGPRsRetentionDisable(POWER_MODULE_ID index)
{
     POWER_DeepSleepGPRsRetentionDisable_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsDeepSleepModuleControl(POWER_MODULE_ID index)
{
     return POWER_ExistsDeepSleepModuleControl_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepModuleEnable(POWER_MODULE_ID index, DEEP_SLEEP_MODULE module)
{
     POWER_DeepSleepModuleEnable_Unsupported(index, module);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepModuleDisable(POWER_MODULE_ID index, DEEP_SLEEP_MODULE module)
{
     POWER_DeepSleepModuleDisable_Unsupported(index, module);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsDeepSleepWakeupEventControl(POWER_MODULE_ID index)
{
     return POWER_ExistsDeepSleepWakeupEventControl_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepWakeupEventEnable(POWER_MODULE_ID index, DEEP_SLEEP_WAKE_UP_EVENT event)
{
     POWER_DeepSleepWakeupEventEnable_Unsupported(index, event);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepWakeupEventDisable(POWER_MODULE_ID index, DEEP_SLEEP_WAKE_UP_EVENT event)
{
     POWER_DeepSleepWakeupEventDisable_Unsupported(index, event);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsDeepSleepPortPinsStateControl(POWER_MODULE_ID index)
{
     return POWER_ExistsDeepSleepPortPinsStateControl_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepPortPinsStateRetain(POWER_MODULE_ID index)
{
     POWER_DeepSleepPortPinsStateRetain_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepPortPinsStateRelease(POWER_MODULE_ID index)
{
     POWER_DeepSleepPortPinsStateRelease_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsDeepSleepEventStatus(POWER_MODULE_ID index)
{
     return POWER_ExistsDeepSleepEventStatus_Unsupported(index);
}

PLIB_INLINE_API DEEP_SLEEP_EVENT _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepEventStatusGet(POWER_MODULE_ID index)
{
     return POWER_DeepSleepEventStatusGet_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepEventStatusClear(POWER_MODULE_ID index, DEEP_SLEEP_EVENT event)
{
     POWER_DeepSleepEventStatusClear_Unsupported(index, event);
}

PLIB_INLINE_API bool PLIB_POWER_ExistsDeepSleepGPROperation(POWER_MODULE_ID index)
{
     return POWER_ExistsDeepSleepGPROperation_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepGPRWrite(POWER_MODULE_ID index, DEEP_SLEEP_GPR gpr, uint32_t data)
{
     POWER_DeepSleepGPRWrite_Unsupported(index, gpr, data);
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_POWER_DeepSleepGPRRead(POWER_MODULE_ID index, DEEP_SLEEP_GPR gpr)
{
     return POWER_DeepSleepGPRRead_Unsupported(index, gpr);
}

#endif
