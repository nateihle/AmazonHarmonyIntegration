/* Created by plibgen $Revision: 1.31 $ */

#ifndef _PTG_P32WK2057GPD132_H
#define _PTG_P32WK2057GPD132_H

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

    PTG_ID_0 = 0,
    PTG_NUMBER_OF_MODULES = 1

} PTG_MODULE_ID;

typedef enum {

    PTG_CLK_SRC_PB_CLK = 0x00,
    PTG_CLK_SRC_CRU_SYS_CLK = 0x01,
    PTG_CLK_SRC_FRC_CLK = 0x02,
    PTG_CLK_SRC_TMR1_CLK_OUT = 0x03,
    PTG_CLK_SRC_TMR3_CLK_OUT = 0x04,
    PTG_CLK_SRC_TMR5_CLK_OUT = 0x05,
    PTG_CLK_SRC_TMR7_CLK_OUT = 0x06,
    PTG_CLK_SRC_REF_CLK_OUT_3 = 0x07

} PTG_CLK_SRC_SEL;

typedef enum {

    PTG_WDT_DISABLE = 0x00,
    PTG_WDT_TIMEOUT_COUNT_CYC_8 = 0x01,
    PTG_WDT_TIMEOUT_COUNT_CYC_16 = 0x02,
    PTG_WDT_TIMEOUT_COUNT_CYC_32 = 0x03,
    PTG_WDT_TIMEOUT_COUNT_CYC_64 = 0x04,
    PTG_WDT_TIMEOUT_COUNT_CYC_128 = 0x05,
    PTG_WDT_TIMEOUT_COUNT_CYC_256 = 0x06,
    PTG_WDT_TIMEOUT_COUNT_CYC_512 = 0x07

} PTG_WDT_TIMEOUT_SEL;

typedef enum {

    PTG_INPUT_MODE_0 = 0x00,
    PTG_INPUT_MODE_1 = 0x01,
    PTG_INPUT_MODE_2 = 0x02,
    PTG_INPUT_MODE_3 = 0x03

} PTG_INPUT_MODE;

typedef enum {

    PTG_TIMER_0 = 0x00,
    PTG_TIMER_1 = 0x01

} PTG_TIMER_SEL;

typedef enum {

    PTG_COUNTER_0 = 0x00,
    PTG_COUNTER_1 = 0x01

} PTG_COUNTER_SEL;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/ptg_ClockSource_Default.h"
#include "../templates/ptg_Prescale_Default.h"
#include "../templates/ptg_TriggerPulseWidth_Default.h"
#include "../templates/ptg_WDTCount_Default.h"
#include "../templates/ptg_EnableControl_Default.h"
#include "../templates/ptg_StopInIdle_Default.h"
#include "../templates/ptg_OutputTriggerMode_Default.h"
#include "../templates/ptg_SWTControl_Default.h"
#include "../templates/ptg_SingleStepControl_Default.h"
#include "../templates/ptg_VisibilityControl_Default.h"
#include "../templates/ptg_StartExecution_Default.h"
#include "../templates/ptg_WDTStatus_Default.h"
#include "../templates/ptg_PTGBusyStatus_Default.h"
#include "../templates/ptg_InputTriggerMode_Default.h"
#include "../templates/ptg_TriggerBroadcastMask_Default.h"
#include "../templates/ptg_HoldValue_Default.h"
#include "../templates/ptg_TimerLimit_Default.h"
#include "../templates/ptg_StepDelay_Default.h"
#include "../templates/ptg_CounterLimit_Default.h"
#include "../templates/ptg_AdjustValue_Default.h"
#include "../templates/ptg_LiteralStrobe_Default.h"
#include "../templates/ptg_QueuePointer_Default.h"
#include "../templates/ptg_StepCommand_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void PLIB_PTG_ClockSourceSelect(PTG_MODULE_ID index, PTG_CLK_SRC_SEL clkSrcSel)
{
     PTG_ClockSourceSelect_Default(index, clkSrcSel);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsClockSource(PTG_MODULE_ID index)
{
     return PTG_ExistsClockSource_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_PrescaleSelect(PTG_MODULE_ID index, uint8_t preScaleSel)
{
     PTG_PrescaleSelect_Default(index, preScaleSel);
}

PLIB_INLINE_API uint8_t PLIB_PTG_PrescaleGet(PTG_MODULE_ID index)
{
     return PTG_PrescaleGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsPrescale(PTG_MODULE_ID index)
{
     return PTG_ExistsPrescale_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_TriggerPulseWidthSet(PTG_MODULE_ID index, uint8_t trigOuputSel)
{
     PTG_TriggerPulseWidthSet_Default(index, trigOuputSel);
}

PLIB_INLINE_API uint8_t PLIB_PTG_TriggerPulseWidthGet(PTG_MODULE_ID index)
{
     return PTG_TriggerPulseWidthGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsTriggerPulseWidth(PTG_MODULE_ID index)
{
     return PTG_ExistsTriggerPulseWidth_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_WDTCountValueSet(PTG_MODULE_ID index, PTG_WDT_TIMEOUT_SEL wdtTimeOutSel)
{
     PTG_WDTCountValueSet_Default(index, wdtTimeOutSel);
}

PLIB_INLINE_API void PLIB_PTG_DisableWDT(PTG_MODULE_ID index)
{
     PTG_DisableWDT_Default(index);
}

PLIB_INLINE_API PTG_WDT_TIMEOUT_SEL PLIB_PTG_WDTCountValueGet(PTG_MODULE_ID index)
{
     return PTG_WDTCountValueGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsWDTCount(PTG_MODULE_ID index)
{
     return PTG_ExistsWDTCount_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_Enable(PTG_MODULE_ID index)
{
     PTG_Enable_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_Disable(PTG_MODULE_ID index)
{
     PTG_Disable_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsEnableControl(PTG_MODULE_ID index)
{
     return PTG_ExistsEnableControl_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_StopInIdleEnable(PTG_MODULE_ID index)
{
     PTG_StopInIdleEnable_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_StopInIdleDisable(PTG_MODULE_ID index)
{
     PTG_StopInIdleDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsStopInIdle(PTG_MODULE_ID index)
{
     return PTG_ExistsStopInIdle_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_OutputTriggerToggle(PTG_MODULE_ID index)
{
     PTG_OutputTriggerToggle_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_OutputTriggerPulse(PTG_MODULE_ID index)
{
     PTG_OutputTriggerPulse_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsOutputTriggerMode(PTG_MODULE_ID index)
{
     return PTG_ExistsOutputTriggerMode_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_SWTEdgeTrigger(PTG_MODULE_ID index)
{
     PTG_SWTEdgeTrigger_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_SWTLevelTrigger(PTG_MODULE_ID index)
{
     PTG_SWTLevelTrigger_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_SWTClear(PTG_MODULE_ID index)
{
     PTG_SWTClear_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_SWTGet(PTG_MODULE_ID index)
{
     return PTG_SWTGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsSWTControl(PTG_MODULE_ID index)
{
     return PTG_ExistsSWTControl_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_SingleStepEnable(PTG_MODULE_ID index)
{
     PTG_SingleStepEnable_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_SingleStepDisable(PTG_MODULE_ID index)
{
     PTG_SingleStepDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsSingleStepControl(PTG_MODULE_ID index)
{
     return PTG_ExistsSingleStepControl_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_VisiblityEnable(PTG_MODULE_ID index)
{
     PTG_VisiblityEnable_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_VisiblityDisable(PTG_MODULE_ID index)
{
     PTG_VisiblityDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsVisibilityControl(PTG_MODULE_ID index)
{
     return PTG_ExistsVisibilityControl_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_ExecutionStart(PTG_MODULE_ID index)
{
     PTG_ExecutionStart_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_ExecutionHalt(PTG_MODULE_ID index)
{
     PTG_ExecutionHalt_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsStartExecution(PTG_MODULE_ID index)
{
     return PTG_ExistsStartExecution_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_WDTStatusGet(PTG_MODULE_ID index)
{
     return PTG_WDTStatusGet_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_WDTStatusClear(PTG_MODULE_ID index)
{
     PTG_WDTStatusClear_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsWDTStatus(PTG_MODULE_ID index)
{
     return PTG_ExistsWDTStatus_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_IsBusy(PTG_MODULE_ID index)
{
     return PTG_IsBusy_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsPTGBusyStatus(PTG_MODULE_ID index)
{
     return PTG_ExistsPTGBusyStatus_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_InputTriggerModeSelect(PTG_MODULE_ID index, PTG_INPUT_MODE InputTrigMode)
{
     PTG_InputTriggerModeSelect_Default(index, InputTrigMode);
}

PLIB_INLINE_API PTG_INPUT_MODE PLIB_PTG_InputTriggerModeGet(PTG_MODULE_ID index)
{
     return PTG_InputTriggerModeGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsInputTriggerMode(PTG_MODULE_ID index)
{
     return PTG_ExistsInputTriggerMode_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_TriggerBroadcastMaskSet(PTG_MODULE_ID index, uint32_t broadcastMask)
{
     PTG_TriggerBroadcastMaskSet_Default(index, broadcastMask);
}

PLIB_INLINE_API uint32_t PLIB_PTG_TriggerBroadcastMaskGet(PTG_MODULE_ID index)
{
     return PTG_TriggerBroadcastMaskGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsTriggerBroadcastMask(PTG_MODULE_ID index)
{
     return PTG_ExistsTriggerBroadcastMask_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_HoldValueSet(PTG_MODULE_ID index, uint16_t holdValue)
{
     PTG_HoldValueSet_Default(index, holdValue);
}

PLIB_INLINE_API uint16_t PLIB_PTG_HoldValueGet(PTG_MODULE_ID index)
{
     return PTG_HoldValueGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsHoldValue(PTG_MODULE_ID index)
{
     return PTG_ExistsHoldValue_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_TimerLimitSet(PTG_MODULE_ID index, PTG_TIMER_SEL timerSel, uint16_t timerLimitValue)
{
     PTG_TimerLimitSet_Default(index, timerSel, timerLimitValue);
}

PLIB_INLINE_API uint16_t PLIB_PTG_TimerLimitGet(PTG_MODULE_ID index, PTG_TIMER_SEL timerSel)
{
     return PTG_TimerLimitGet_Default(index, timerSel);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsTimerLimit(PTG_MODULE_ID index)
{
     return PTG_ExistsTimerLimit_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_StepDelaySet(PTG_MODULE_ID index, uint16_t stepDelayLimit)
{
     PTG_StepDelaySet_Default(index, stepDelayLimit);
}

PLIB_INLINE_API uint16_t PLIB_PTG_StepDelayGet(PTG_MODULE_ID index)
{
     return PTG_StepDelayGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsStepDelay(PTG_MODULE_ID index)
{
     return PTG_ExistsStepDelay_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_CounterLimitSet(PTG_MODULE_ID index, PTG_COUNTER_SEL counterSel, uint16_t counterLimit)
{
     PTG_CounterLimitSet_Default(index, counterSel, counterLimit);
}

PLIB_INLINE_API uint16_t PLIB_PTG_CounterLimitGet(PTG_MODULE_ID index, PTG_COUNTER_SEL counterSel)
{
     return PTG_CounterLimitGet_Default(index, counterSel);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsCounterLimit(PTG_MODULE_ID index)
{
     return PTG_ExistsCounterLimit_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_AdjustValueSet(PTG_MODULE_ID index, uint16_t adjustValue)
{
     PTG_AdjustValueSet_Default(index, adjustValue);
}

PLIB_INLINE_API uint16_t PLIB_PTG_AdjustValueGet(PTG_MODULE_ID index)
{
     return PTG_AdjustValueGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsAdjustValue(PTG_MODULE_ID index)
{
     return PTG_ExistsAdjustValue_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_LiteralStrobeValueSet(PTG_MODULE_ID index, uint16_t strobeValue)
{
     PTG_LiteralStrobeValueSet_Default(index, strobeValue);
}

PLIB_INLINE_API uint16_t PLIB_PTG_LiteralStrobeValueGet(PTG_MODULE_ID index)
{
     return PTG_LiteralStrobeValueGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsLiteralStrobe(PTG_MODULE_ID index)
{
     return PTG_ExistsLiteralStrobe_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_QueuePointerSet(PTG_MODULE_ID index, uint8_t queueLoc)
{
     PTG_QueuePointerSet_Default(index, queueLoc);
}

PLIB_INLINE_API uint8_t PLIB_PTG_QueuePointerGet(PTG_MODULE_ID index)
{
     return PTG_QueuePointerGet_Default(index);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsQueuePointer(PTG_MODULE_ID index)
{
     return PTG_ExistsQueuePointer_Default(index);
}

PLIB_INLINE_API void PLIB_PTG_StepCommandSet(PTG_MODULE_ID index, uint8_t stepLoc, uint8_t command)
{
     PTG_StepCommandSet_Default(index, stepLoc, command);
}

PLIB_INLINE_API uint8_t PLIB_PTG_StepCommandGet(PTG_MODULE_ID index, uint8_t stepLoc)
{
     return PTG_StepCommandGet_Default(index, stepLoc);
}

PLIB_INLINE_API bool PLIB_PTG_ExistsStepCommand(PTG_MODULE_ID index)
{
     return PTG_ExistsStepCommand_Default(index);
}

#endif
