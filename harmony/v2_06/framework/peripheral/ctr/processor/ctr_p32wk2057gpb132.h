/* Created by plibgen $Revision: 1.31 $ */

#ifndef _CTR_P32WK2057GPB132_H
#define _CTR_P32WK2057GPB132_H

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

    CTR_ID_0 = 0,
    CTR_NUMBER_OF_MODULES = 1

} CTR_MODULE_ID;

typedef enum {

    CTR0 = 0x00,
    CTR1 = 0x01

} CTR_SELECT;

typedef enum {

    CTR_LATCH_UNIT0 = 0x00,
    CTR_LATCH_UNIT1 = 0x01,
    CTR_LATCH_UNIT2 = 0x02,
    CTR_LATCH_UNIT3 = 0x03,
    CTR_LATCH_UNIT4 = 0x04,
    CTR_LATCH_UNIT5 = 0x05

} CTR_LATCH_UNIT_SELECT;

typedef enum {

    CTR_DISABLE = 0x00,
    CTR_ENABLE = 0x01

} CTR_ENABLE_CONTROL;

typedef enum {

    CTR_1394 = 0x00,
    CTR_US = 0x01

} CTR_MODE_SELECT;

typedef enum {

    CTR_AVWS0 = 0x00,
    CTR_AVWS1 = 0x01,
    CTR_AVWS2 = 0x02,
    CTR_AVWS3 = 0x03,
    CTR_AVWS4 = 0x04,
    CTR_RSVD1 = 0x05,
    CTR_RSVD2 = 0x06,
    CTR_RSVD3 = 0x07,
    CTR_RSVD4 = 0x08,
    CTR_WIFI_TM_1 = 0x09,
    CTR_WIFI_TM_2 = 0x0A,
    CTR_WIFI_TM_3 = 0x0B,
    CTR_WIFI_TM_4 = 0x0C,
    CTR_ETH_RX = 0x0D,
    CTR_ETH_TX = 0x0E,
    CTR_RSVD5 = 0x0F,
    CTR_RSVD6 = 0x10,
    CTR_RSVD7 = 0x11,
    CTR_MLBCLK = 0x12,
    CTR_GPIO0 = 0x13,
    CTR_GPIO1 = 0x14,
    CTR_USBSOF = 0x15

} CTR_LATCH_TRIGGER_SELECT;

typedef enum {

    CTR_CTR0_US = 0x00,
    CTR_CTR0_LIN = 0x01,
    CTR_CTR1_US = 0x02,
    CTR_CTR1_LIN = 0x03

} CTR_LATCH_CTR_SELECT;

typedef enum {

    CTR_LATCH0 = 0x01,
    CTR_LATCH1 = 0x02,
    CTR_LATCH2 = 0x04,
    CTR_LATCH3 = 0x08,
    CTR_LATCH4 = 0x10,
    CTR_LATCH5 = 0x20

} CTR_ENABLE_LATCH_INT_GEN;

typedef enum {

    CTR_LATCH_TRIG = 0x00,
    CTR_BUFFER_HALF = 0x01,
    CTR_BUFFER_FULL = 0x02

} CTR_LATCH_INT_MODE;

typedef enum {

    CTR_TEST_BUS1 = 0x01,
    CTR_TEST_BUS2 = 0x02,
    CTR_TEST_BUS3 = 0x03,
    CTR_TEST_BUS4 = 0x04,
    CTR_TEST_BUS5 = 0x05,
    CTR_TEST_BUS6 = 0x06,
    CTR_TEST_BUS7 = 0x07,
    CTR_TEST_BUS8 = 0x08,
    CTR_TEST_BUS9 = 0x09

} CTR_TEST_BUS_SELECT;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/ctr_CTR1394Mode_Default.h"
#include "../templates/ctr_CTRMicroSecondsMode_Default.h"
#include "../templates/ctr_CTRLinear_Default.h"
#include "../templates/ctr_CTREnable_Default.h"
#include "../templates/ctr_CTRFormatSelect_Default.h"
#include "../templates/ctr_CTRAdjustUS_Default.h"
#include "../templates/ctr_CTRDriftUS_Default.h"
#include "../templates/ctr_CTRDriftAccuUS_Default.h"
#include "../templates/ctr_CTRAdjustLinear_Default.h"
#include "../templates/ctr_CTRDriftLinear_Default.h"
#include "../templates/ctr_CTRDriftAccuLinear_Default.h"
#include "../templates/ctr_LatchTriggerSelect_Default.h"
#include "../templates/ctr_LatchValue_Default.h"
#include "../templates/ctr_LatchStatus_Default.h"
#include "../templates/ctr_LatchTriggerCountValue_Default.h"
#include "../templates/ctr_CTRTrigger_Default.h"
#include "../templates/ctr_CTRN_Default.h"
#include "../templates/ctr_CTRM_Default.h"
#include "../templates/ctr_CTRLSB_Default.h"
#include "../templates/ctr_CTRInterrupt_Default.h"
#include "../templates/ctr_CTRSpare_Default.h"
#include "../templates/ctr_CTRTestBusSelect_Default.h"
#include "../templates/ctr_CTRRevision_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_CTR_Exists1394Mode(CTR_MODULE_ID index)
{
     return CTR_Exists1394Mode_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_CTR_1394ModeSecondGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_1394ModeSecondGet_Default(index, ctrSel);
}

PLIB_INLINE_API void PLIB_CTR_1394ModeSecondSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t secVal)
{
     CTR_1394ModeSecondSet_Default(index, ctrSel, secVal);
}

PLIB_INLINE_API uint32_t PLIB_CTR_1394ModeCountGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_1394ModeCountGet_Default(index, ctrSel);
}

PLIB_INLINE_API void PLIB_CTR_1394ModeCountSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t countVal)
{
     CTR_1394ModeCountSet_Default(index, ctrSel, countVal);
}

PLIB_INLINE_API uint32_t PLIB_CTR_1394ModeOffsetGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_1394ModeOffsetGet_Default(index, ctrSel);
}

PLIB_INLINE_API void PLIB_CTR_1394ModeOffsetSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t offsetVal)
{
     CTR_1394ModeOffsetSet_Default(index, ctrSel, offsetVal);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsUSMode(CTR_MODULE_ID index)
{
     return CTR_ExistsUSMode_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_CTR_USModeSecondGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_USModeSecondGet_Default(index, ctrSel);
}

PLIB_INLINE_API void PLIB_CTR_USModeSecondSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t secUSVal)
{
     CTR_USModeSecondSet_Default(index, ctrSel, secUSVal);
}

PLIB_INLINE_API uint32_t PLIB_CTR_USModeValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_USModeValueGet_Default(index, ctrSel);
}

PLIB_INLINE_API void PLIB_CTR_USModeValueSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t usVal)
{
     CTR_USModeValueSet_Default(index, ctrSel, usVal);
}

PLIB_INLINE_API uint32_t PLIB_CTR_USMode10nsGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_USMode10nsGet_Default(index, ctrSel);
}

PLIB_INLINE_API void PLIB_CTR_USMode10nsSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t us10nsVal)
{
     CTR_USMode10nsSet_Default(index, ctrSel, us10nsVal);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsLinearCTR(CTR_MODULE_ID index)
{
     return CTR_ExistsLinearCTR_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_CTR_LinearCTRGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_LinearCTRGet_Default(index, ctrSel);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsEnableCTR(CTR_MODULE_ID index)
{
     return CTR_ExistsEnableCTR_Default(index);
}

PLIB_INLINE_API void PLIB_CTR_EnableCTR(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     CTR_EnableCTR_Default(index, ctrSel);
}

PLIB_INLINE_API void PLIB_CTR_DisableCTR(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     CTR_DisableCTR_Default(index, ctrSel);
}

PLIB_INLINE_API CTR_ENABLE_CONTROL PLIB_CTR_ModuleStatus(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_ModuleStatus_Default(index, ctrSel);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsCTRFormatSel(CTR_MODULE_ID index)
{
     return CTR_ExistsCTRFormatSel_Default(index);
}

PLIB_INLINE_API void PLIB_CTR_CTRModeSelect(CTR_MODULE_ID index, CTR_SELECT ctrSel, CTR_MODE_SELECT modeVal)
{
     CTR_CTRModeSelect_Default(index, ctrSel, modeVal);
}

PLIB_INLINE_API CTR_MODE_SELECT PLIB_CTR_CTRModeStatus(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_CTRModeStatus_Default(index, ctrSel);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsCTRAdjustUS(CTR_MODULE_ID index)
{
     return CTR_ExistsCTRAdjustUS_Default(index);
}

PLIB_INLINE_API void PLIB_CTR_CTRAdjustValueInitialize(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t adjVal)
{
     CTR_CTRAdjustValueInitialize_Default(index, ctrSel, adjVal);
}

PLIB_INLINE_API uint32_t PLIB_CTR_CTRAdjustValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_CTRAdjustValueGet_Default(index, ctrSel);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsCTRDriftUS(CTR_MODULE_ID index)
{
     return CTR_ExistsCTRDriftUS_Default(index);
}

PLIB_INLINE_API void PLIB_CTR_CTRDriftValueSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t driftVal)
{
     CTR_CTRDriftValueSet_Default(index, ctrSel, driftVal);
}

PLIB_INLINE_API uint32_t PLIB_CTR_CTRDriftValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_CTRDriftValueGet_Default(index, ctrSel);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsCTRDriftAccuUS(CTR_MODULE_ID index)
{
     return CTR_ExistsCTRDriftAccuUS_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_CTR_CTRAccuUSDriftValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_CTRAccuUSDriftValueGet_Default(index, ctrSel);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsCTRAdjustLIN(CTR_MODULE_ID index)
{
     return CTR_ExistsCTRAdjustLIN_Default(index);
}

PLIB_INLINE_API void PLIB_CTR_CTRLinearAdjustInitialize(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t linAdjVal)
{
     CTR_CTRLinearAdjustInitialize_Default(index, ctrSel, linAdjVal);
}

PLIB_INLINE_API uint32_t PLIB_CTR_CTRLinearAdjustGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_CTRLinearAdjustGet_Default(index, ctrSel);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsCTRDriftLIN(CTR_MODULE_ID index)
{
     return CTR_ExistsCTRDriftLIN_Default(index);
}

PLIB_INLINE_API void PLIB_CTR_CTRLinearDriftSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t linDriftVal)
{
     CTR_CTRLinearDriftSet_Default(index, ctrSel, linDriftVal);
}

PLIB_INLINE_API uint32_t PLIB_CTR_CTRLinearDriftGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_CTRLinearDriftGet_Default(index, ctrSel);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsCTRDriftAccuLIN(CTR_MODULE_ID index)
{
     return CTR_ExistsCTRDriftAccuLIN_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_CTR_CTRAccuLinDriftValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_CTRAccuLinDriftValueGet_Default(index, ctrSel);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsLatchTriggerSelect(CTR_MODULE_ID index)
{
     return CTR_ExistsLatchTriggerSelect_Default(index);
}

PLIB_INLINE_API void PLIB_CTR_LatchTriggerSelect(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum, CTR_LATCH_TRIGGER_SELECT latTrigSrc)
{
     CTR_LatchTriggerSelect_Default(index, latNum, latTrigSrc);
}

PLIB_INLINE_API CTR_LATCH_TRIGGER_SELECT PLIB_CTR_LatchTriggerGet(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum)
{
     return CTR_LatchTriggerGet_Default(index, latNum);
}

PLIB_INLINE_API void PLIB_CTR_LatchCTRSelect(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum, CTR_LATCH_CTR_SELECT latctrVal)
{
     CTR_LatchCTRSelect_Default(index, latNum, latctrVal);
}

PLIB_INLINE_API CTR_LATCH_CTR_SELECT PLIB_CTR_LatchCTRGet(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum)
{
     return CTR_LatchCTRGet_Default(index, latNum);
}

PLIB_INLINE_API void PLIB_CTR_LatchDivSet(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum, uint32_t divVal)
{
     CTR_LatchDivSet_Default(index, latNum, divVal);
}

PLIB_INLINE_API uint32_t PLIB_CTR_LatchDivGet(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum)
{
     return CTR_LatchDivGet_Default(index, latNum);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsLatchValue(CTR_MODULE_ID index)
{
     return CTR_ExistsLatchValue_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_CTR_LatchGetValue(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum)
{
     return CTR_LatchGetValue_Default(index, latNum);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsLatchStatus(CTR_MODULE_ID index)
{
     return CTR_ExistsLatchStatus_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_CTR_LatchGetStatus(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum)
{
     return CTR_LatchGetStatus_Default(index, latNum);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsLatchTriggerCountValue(CTR_MODULE_ID index)
{
     return CTR_ExistsLatchTriggerCountValue_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_CTR_LatchTriggerCountGet(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum)
{
     return CTR_LatchTriggerCountGet_Default(index, latNum);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsTrigger(CTR_MODULE_ID index)
{
     return CTR_ExistsTrigger_Default(index);
}

PLIB_INLINE_API void PLIB_CTR_TriggerSelect(CTR_MODULE_ID index, CTR_LATCH_CTR_SELECT ctrTrigVal)
{
     CTR_TriggerSelect_Default(index, ctrTrigVal);
}

PLIB_INLINE_API CTR_LATCH_CTR_SELECT PLIB_CTR_TriggerGet(CTR_MODULE_ID index)
{
     return CTR_TriggerGet_Default(index);
}

PLIB_INLINE_API void PLIB_CTR_CycleOffsetValueSet(CTR_MODULE_ID index, uint32_t cycleOffsetVal)
{
     CTR_CycleOffsetValueSet_Default(index, cycleOffsetVal);
}

PLIB_INLINE_API uint32_t PLIB_CTR_CycleOffsetValueGet(CTR_MODULE_ID index)
{
     return CTR_CycleOffsetValueGet_Default(index);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsNValue(CTR_MODULE_ID index)
{
     return CTR_ExistsNValue_Default(index);
}

PLIB_INLINE_API void PLIB_CTR_NValueSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t valueN)
{
     CTR_NValueSet_Default(index, ctrSel, valueN);
}

PLIB_INLINE_API uint32_t PLIB_CTR_NValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_NValueGet_Default(index, ctrSel);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsMValue(CTR_MODULE_ID index)
{
     return CTR_ExistsMValue_Default(index);
}

PLIB_INLINE_API void PLIB_CTR_MValueSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t valueM)
{
     CTR_MValueSet_Default(index, ctrSel, valueM);
}

PLIB_INLINE_API uint32_t PLIB_CTR_MValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_MValueGet_Default(index, ctrSel);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsLSBValue(CTR_MODULE_ID index)
{
     return CTR_ExistsLSBValue_Default(index);
}

PLIB_INLINE_API void PLIB_CTR_LSBValueSet(CTR_MODULE_ID index, CTR_SELECT ctrSel, uint32_t valueLSB)
{
     CTR_LSBValueSet_Default(index, ctrSel, valueLSB);
}

PLIB_INLINE_API uint32_t PLIB_CTR_LSBValueGet(CTR_MODULE_ID index, CTR_SELECT ctrSel)
{
     return CTR_LSBValueGet_Default(index, ctrSel);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsInterrupt(CTR_MODULE_ID index)
{
     return CTR_ExistsInterrupt_Default(index);
}

PLIB_INLINE_API void PLIB_CTR_IntModeLatchSelect(CTR_MODULE_ID index, CTR_LATCH_INT_MODE intMode, CTR_LATCH_UNIT_SELECT latNum)
{
     CTR_IntModeLatchSelect_Default(index, intMode, latNum);
}

PLIB_INLINE_API CTR_LATCH_INT_MODE PLIB_CTR_IntModeLatchGet(CTR_MODULE_ID index, CTR_LATCH_UNIT_SELECT latNum)
{
     return CTR_IntModeLatchGet_Default(index, latNum);
}

PLIB_INLINE_API void PLIB_CTR_IntLatchSelect(CTR_MODULE_ID index, CTR_ENABLE_LATCH_INT_GEN enableLatchVal)
{
     CTR_IntLatchSelect_Default(index, enableLatchVal);
}

PLIB_INLINE_API CTR_ENABLE_LATCH_INT_GEN PLIB_CTR_IntLatchGet(CTR_MODULE_ID index)
{
     return CTR_IntLatchGet_Default(index);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsSpare(CTR_MODULE_ID index)
{
     return CTR_ExistsSpare_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_CTR_SpareValueGet(CTR_MODULE_ID index)
{
     return CTR_SpareValueGet_Default(index);
}

PLIB_INLINE_API void PLIB_CTR_SpareValueSet(CTR_MODULE_ID index, uint32_t spareVal)
{
     CTR_SpareValueSet_Default(index, spareVal);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsTestBusSelect(CTR_MODULE_ID index)
{
     return CTR_ExistsTestBusSelect_Default(index);
}

PLIB_INLINE_API void PLIB_CTR_TestBusSelect(CTR_MODULE_ID index, CTR_TEST_BUS_SELECT testBusVal)
{
     CTR_TestBusSelect_Default(index, testBusVal);
}

PLIB_INLINE_API uint32_t PLIB_CTR_TestBusGet(CTR_MODULE_ID index)
{
     return CTR_TestBusGet_Default(index);
}

PLIB_INLINE_API bool PLIB_CTR_ExistsRevision(CTR_MODULE_ID index)
{
     return CTR_ExistsRevision_Default(index);
}

PLIB_INLINE_API uint32_t PLIB_CTR_BlockRevisionGet(CTR_MODULE_ID index)
{
     return CTR_BlockRevisionGet_Default(index);
}

#endif
