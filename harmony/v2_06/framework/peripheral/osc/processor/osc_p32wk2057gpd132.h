/* Created by plibgen $Revision: 1.31 $ */

#ifndef _OSC_P32WK2057GPD132_H
#define _OSC_P32WK2057GPD132_H

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

    OSC_ID_0 = 0,
    OSC_NUMBER_OF_MODULES = 1

} OSC_MODULE_ID;

typedef enum {

    OSC_ON_WAIT_IDLE = 0x00,
    OSC_ON_WAIT_SLEEP = 0x01

} OSC_OPERATION_ON_WAIT;

typedef enum {

    OSC_FRC_DIV_256 = 0x07,
    OSC_FRC_DIV_64 = 0x06,
    OSC_FRC_DIV_32 = 0x05,
    OSC_FRC_DIV_16 = 0x04,
    OSC_FRC_DIV_8 = 0x03,
    OSC_FRC_DIV_4 = 0x02,
    OSC_FRC_DIV_2 = 0x01,
    OSC_FRC_DIV_1 = 0x00

} OSC_FRC_DIV;

typedef enum {

    OSC_FRC_WITH_PLL = 0x01,
    OSC_PRIMARY = 0x02,
    OSC_PRIMARY_WITH_PLL = 0x06,
    OSC_SECONDARY = 0x04,
    OSC_USBCLK_WITH_PLL = 0x03,
    OSC_LPRC = 0x05,
    OSC_FRC_BY_FRCDIV = 0x00

} OSC_SYS_TYPE;

typedef enum {

    OSC_REF_BASECLOCK_SYSCLK = 0x00,
    OSC_REF_BASECLOCK_PBCLK = 0x01,
    OSC_REF_BASECLOCK_PRIMARY = 0x02,
    OSC_REF_BASECLOCK_FRC = 0x03,
    OSC_REF_BASECLOCK_LPRC = 0x04,
    OSC_REF_BASECLOCK_SOSC = 0x05,
    OSC_REF_BASECLOCK_USBCLK = 0x06,
    OSC_REF_BASECLOCK_SYSPLLOUT = 0x07,
    OSC_REF_BASECLOCK_EXT = 0x08

} OSC_REF_BASECLOCK;

typedef enum {

    SYS_OSC_USBCLK_PRIMARY = 0x00,
    SYS_OSC_USBCLK_FRC = 0x01

} OSC_USBCLOCK_SOURCE;

typedef enum {

    OSC_SYSPLL_OUT_DIV_32 = 0x1F,
    OSC_SYSPLL_OUT_DIV_31 = 0x1E,
    OSC_SYSPLL_OUT_DIV_30 = 0x1D,
    OSC_SYSPLL_OUT_DIV_29 = 0x1C,
    OSC_SYSPLL_OUT_DIV_28 = 0x1B,
    OSC_SYSPLL_OUT_DIV_27 = 0x1A,
    OSC_SYSPLL_OUT_DIV_26 = 0x19,
    OSC_SYSPLL_OUT_DIV_25 = 0x18,
    OSC_SYSPLL_OUT_DIV_24 = 0x17,
    OSC_SYSPLL_OUT_DIV_23 = 0x16,
    OSC_SYSPLL_OUT_DIV_22 = 0x15,
    OSC_SYSPLL_OUT_DIV_21 = 0x14,
    OSC_SYSPLL_OUT_DIV_20 = 0x13,
    OSC_SYSPLL_OUT_DIV_19 = 0x12,
    OSC_SYSPLL_OUT_DIV_18 = 0x11,
    OSC_SYSPLL_OUT_DIV_17 = 0x10,
    OSC_SYSPLL_OUT_DIV_16 = 0x0F,
    OSC_SYSPLL_OUT_DIV_15 = 0x0E,
    OSC_SYSPLL_OUT_DIV_14 = 0x0D,
    OSC_SYSPLL_OUT_DIV_13 = 0x0C,
    OSC_SYSPLL_OUT_DIV_12 = 0x0B,
    OSC_SYSPLL_OUT_DIV_11 = 0x0A,
    OSC_SYSPLL_OUT_DIV_10 = 0x09,
    OSC_SYSPLL_OUT_DIV_9 = 0x08,
    OSC_SYSPLL_OUT_DIV_8 = 0x07,
    OSC_SYSPLL_OUT_DIV_7 = 0x06,
    OSC_SYSPLL_OUT_DIV_6 = 0x05,
    OSC_SYSPLL_OUT_DIV_5 = 0x04,
    OSC_SYSPLL_OUT_DIV_4 = 0x03,
    OSC_SYSPLL_OUT_DIV_3 = 0x02,
    OSC_SYSPLL_OUT_DIV_2 = 0x01,
    OSC_SYSPLL_OUT_DIV_1 = 0x00

} OSC_SYSPLL_OUT_DIV;

typedef enum {

    OSC_UPLL_OUT_DIV_32 = 0x1F,
    OSC_UPLL_OUT_DIV_31 = 0x1E,
    OSC_UPLL_OUT_DIV_30 = 0x1D,
    OSC_UPLL_OUT_DIV_29 = 0x1C,
    OSC_UPLL_OUT_DIV_28 = 0x1B,
    OSC_UPLL_OUT_DIV_27 = 0x1A,
    OSC_UPLL_OUT_DIV_26 = 0x19,
    OSC_UPLL_OUT_DIV_25 = 0x18,
    OSC_UPLL_OUT_DIV_24 = 0x17,
    OSC_UPLL_OUT_DIV_23 = 0x16,
    OSC_UPLL_OUT_DIV_22 = 0x15,
    OSC_UPLL_OUT_DIV_21 = 0x14,
    OSC_UPLL_OUT_DIV_20 = 0x13,
    OSC_UPLL_OUT_DIV_19 = 0x12,
    OSC_UPLL_OUT_DIV_18 = 0x11,
    OSC_UPLL_OUT_DIV_17 = 0x10,
    OSC_UPLL_OUT_DIV_16 = 0x0F,
    OSC_UPLL_OUT_DIV_15 = 0x0E,
    OSC_UPLL_OUT_DIV_14 = 0x0D,
    OSC_UPLL_OUT_DIV_13 = 0x0C,
    OSC_UPLL_OUT_DIV_12 = 0x0B,
    OSC_UPLL_OUT_DIV_11 = 0x0A,
    OSC_UPLL_OUT_DIV_10 = 0x09,
    OSC_UPLL_OUT_DIV_9 = 0x08,
    OSC_UPLL_OUT_DIV_8 = 0x07,
    OSC_UPLL_OUT_DIV_7 = 0x06,
    OSC_UPLL_OUT_DIV_6 = 0x05,
    OSC_UPLL_OUT_DIV_5 = 0x04,
    OSC_UPLL_OUT_DIV_4 = 0x03,
    OSC_UPLL_OUT_DIV_3 = 0x02,
    OSC_UPLL_OUT_DIV_2 = 0x01,
    OSC_UPLL_OUT_DIV_1 = 0x00

} OSC_UPLL_OUT_DIV;

typedef enum {

    OSC_BTPLL_OUT_DIV_32 = 0x1F,
    OSC_BTPLL_OUT_DIV_31 = 0x1E,
    OSC_BTPLL_OUT_DIV_30 = 0x1D,
    OSC_BTPLL_OUT_DIV_29 = 0x1C,
    OSC_BTPLL_OUT_DIV_28 = 0x1B,
    OSC_BTPLL_OUT_DIV_27 = 0x1A,
    OSC_BTPLL_OUT_DIV_26 = 0x19,
    OSC_BTPLL_OUT_DIV_25 = 0x18,
    OSC_BTPLL_OUT_DIV_24 = 0x17,
    OSC_BTPLL_OUT_DIV_23 = 0x16,
    OSC_BTPLL_OUT_DIV_22 = 0x15,
    OSC_BTPLL_OUT_DIV_21 = 0x14,
    OSC_BTPLL_OUT_DIV_20 = 0x13,
    OSC_BTPLL_OUT_DIV_19 = 0x12,
    OSC_BTPLL_OUT_DIV_18 = 0x11,
    OSC_BTPLL_OUT_DIV_17 = 0x10,
    OSC_BTPLL_OUT_DIV_16 = 0x0F,
    OSC_BTPLL_OUT_DIV_15 = 0x0E,
    OSC_BTPLL_OUT_DIV_14 = 0x0D,
    OSC_BTPLL_OUT_DIV_13 = 0x0C,
    OSC_BTPLL_OUT_DIV_12 = 0x0B,
    OSC_BTPLL_OUT_DIV_11 = 0x0A,
    OSC_BTPLL_OUT_DIV_10 = 0x09,
    OSC_BTPLL_OUT_DIV_9 = 0x08,
    OSC_BTPLL_OUT_DIV_8 = 0x07,
    OSC_BTPLL_OUT_DIV_7 = 0x06,
    OSC_BTPLL_OUT_DIV_6 = 0x05,
    OSC_BTPLL_OUT_DIV_5 = 0x04,
    OSC_BTPLL_OUT_DIV_4 = 0x03,
    OSC_BTPLL_OUT_DIV_3 = 0x02,
    OSC_BTPLL_OUT_DIV_2 = 0x01,
    OSC_BTPLL_OUT_DIV_1 = 0x00

} OSC_BTPLL_OUT_DIV;

typedef enum {

    OSC_PLL_SYSTEM = 0x00,
    OSC_PLL_USB = 0x01,
    OSC_PLL_BT = 0x02

} OSC_PLL_SELECT;

typedef enum {

    OSC_PERIPHERAL_BUS_1 = 0x00,
    OSC_PERIPHERAL_BUS_2 = 0x01,
    OSC_PERIPHERAL_BUS_3 = 0x02,
    OSC_PERIPHERAL_BUS_4 = 0x03

} OSC_PERIPHERAL_BUS;

typedef enum {

    OSC_REFERENCE_1 = 0x00,
    OSC_REFERENCE_2 = 0x01,
    OSC_REFERENCE_3 = 0x02,
    OSC_REFERENCE_4 = 0x03

} OSC_REFERENCE;

typedef enum {

    OSC_SYSPLL_IN_DIV_32 = 0x1F,
    OSC_SYSPLL_IN_DIV_31 = 0x1E,
    OSC_SYSPLL_IN_DIV_30 = 0x1D,
    OSC_SYSPLL_IN_DIV_29 = 0x1C,
    OSC_SYSPLL_IN_DIV_28 = 0x1B,
    OSC_SYSPLL_IN_DIV_27 = 0x1A,
    OSC_SYSPLL_IN_DIV_26 = 0x19,
    OSC_SYSPLL_IN_DIV_25 = 0x18,
    OSC_SYSPLL_IN_DIV_24 = 0x17,
    OSC_SYSPLL_IN_DIV_23 = 0x16,
    OSC_SYSPLL_IN_DIV_22 = 0x15,
    OSC_SYSPLL_IN_DIV_21 = 0x14,
    OSC_SYSPLL_IN_DIV_20 = 0x13,
    OSC_SYSPLL_IN_DIV_19 = 0x12,
    OSC_SYSPLL_IN_DIV_18 = 0x11,
    OSC_SYSPLL_IN_DIV_17 = 0x10,
    OSC_SYSPLL_IN_DIV_16 = 0x0F,
    OSC_SYSPLL_IN_DIV_15 = 0x0E,
    OSC_SYSPLL_IN_DIV_14 = 0x0D,
    OSC_SYSPLL_IN_DIV_13 = 0x0C,
    OSC_SYSPLL_IN_DIV_12 = 0x0B,
    OSC_SYSPLL_IN_DIV_11 = 0x0A,
    OSC_SYSPLL_IN_DIV_10 = 0x09,
    OSC_SYSPLL_IN_DIV_9 = 0x08,
    OSC_SYSPLL_IN_DIV_8 = 0x07,
    OSC_SYSPLL_IN_DIV_7 = 0x06,
    OSC_SYSPLL_IN_DIV_6 = 0x05,
    OSC_SYSPLL_IN_DIV_5 = 0x04,
    OSC_SYSPLL_IN_DIV_4 = 0x03,
    OSC_SYSPLL_IN_DIV_3 = 0x02,
    OSC_SYSPLL_IN_DIV_2 = 0x01,
    OSC_SYSPLL_IN_DIV_1 = 0x00

} OSC_SYSPLL_IN_DIV;

typedef enum {

    OSC_UPLL_IN_DIV_32 = 0x1F,
    OSC_UPLL_IN_DIV_31 = 0x1E,
    OSC_UPLL_IN_DIV_30 = 0x1D,
    OSC_UPLL_IN_DIV_29 = 0x1C,
    OSC_UPLL_IN_DIV_28 = 0x1B,
    OSC_UPLL_IN_DIV_27 = 0x1A,
    OSC_UPLL_IN_DIV_26 = 0x19,
    OSC_UPLL_IN_DIV_25 = 0x18,
    OSC_UPLL_IN_DIV_24 = 0x17,
    OSC_UPLL_IN_DIV_23 = 0x16,
    OSC_UPLL_IN_DIV_22 = 0x15,
    OSC_UPLL_IN_DIV_21 = 0x14,
    OSC_UPLL_IN_DIV_20 = 0x13,
    OSC_UPLL_IN_DIV_19 = 0x12,
    OSC_UPLL_IN_DIV_18 = 0x11,
    OSC_UPLL_IN_DIV_17 = 0x10,
    OSC_UPLL_IN_DIV_16 = 0x0F,
    OSC_UPLL_IN_DIV_15 = 0x0E,
    OSC_UPLL_IN_DIV_14 = 0x0D,
    OSC_UPLL_IN_DIV_13 = 0x0C,
    OSC_UPLL_IN_DIV_12 = 0x0B,
    OSC_UPLL_IN_DIV_11 = 0x0A,
    OSC_UPLL_IN_DIV_10 = 0x09,
    OSC_UPLL_IN_DIV_9 = 0x08,
    OSC_UPLL_IN_DIV_8 = 0x07,
    OSC_UPLL_IN_DIV_7 = 0x06,
    OSC_UPLL_IN_DIV_6 = 0x05,
    OSC_UPLL_IN_DIV_5 = 0x04,
    OSC_UPLL_IN_DIV_4 = 0x03,
    OSC_UPLL_IN_DIV_3 = 0x02,
    OSC_UPLL_IN_DIV_2 = 0x01,
    OSC_UPLL_IN_DIV_1 = 0x00

} OSC_UPLL_IN_DIV;

typedef enum {

    OSC_BTPLL_IN_DIV_32 = 0x1F,
    OSC_BTPLL_IN_DIV_31 = 0x1E,
    OSC_BTPLL_IN_DIV_30 = 0x1D,
    OSC_BTPLL_IN_DIV_29 = 0x1C,
    OSC_BTPLL_IN_DIV_28 = 0x1B,
    OSC_BTPLL_IN_DIV_27 = 0x1A,
    OSC_BTPLL_IN_DIV_26 = 0x19,
    OSC_BTPLL_IN_DIV_25 = 0x18,
    OSC_BTPLL_IN_DIV_24 = 0x17,
    OSC_BTPLL_IN_DIV_23 = 0x16,
    OSC_BTPLL_IN_DIV_22 = 0x15,
    OSC_BTPLL_IN_DIV_21 = 0x14,
    OSC_BTPLL_IN_DIV_20 = 0x13,
    OSC_BTPLL_IN_DIV_19 = 0x12,
    OSC_BTPLL_IN_DIV_18 = 0x11,
    OSC_BTPLL_IN_DIV_17 = 0x10,
    OSC_BTPLL_IN_DIV_16 = 0x0F,
    OSC_BTPLL_IN_DIV_15 = 0x0E,
    OSC_BTPLL_IN_DIV_14 = 0x0D,
    OSC_BTPLL_IN_DIV_13 = 0x0C,
    OSC_BTPLL_IN_DIV_12 = 0x0B,
    OSC_BTPLL_IN_DIV_11 = 0x0A,
    OSC_BTPLL_IN_DIV_10 = 0x09,
    OSC_BTPLL_IN_DIV_9 = 0x08,
    OSC_BTPLL_IN_DIV_8 = 0x07,
    OSC_BTPLL_IN_DIV_7 = 0x06,
    OSC_BTPLL_IN_DIV_6 = 0x05,
    OSC_BTPLL_IN_DIV_5 = 0x04,
    OSC_BTPLL_IN_DIV_4 = 0x03,
    OSC_BTPLL_IN_DIV_3 = 0x02,
    OSC_BTPLL_IN_DIV_2 = 0x01,
    OSC_BTPLL_IN_DIV_1 = 0x00

} OSC_BTPLL_IN_DIV;

typedef enum {

    OSC_SYSPLL_FREQ_RANGE_10M_TO_16M = 0x01,
    OSC_SYSPLL_FREQ_RANGE_16M_TO_25M = 0x02,
    OSC_SYSPLL_FREQ_RANGE_25M_TO_40M = 0x03,
    OSC_SYSPLL_FREQ_RANGE_40M_TO_65M = 0x04,
    OSC_SYSPLL_FREQ_RANGE_65M_TO_100M = 0x05,
    OSC_SYSPLL_FREQ_RANGE_100M_TO_160M = 0x06,
    OSC_SYSPLL_FREQ_RANGE_BYPASS = 0x00

} OSC_SYSPLL_FREQ_RANGE;

typedef enum {

    OSC_SYSPLL_IN_CLK_SOURCE_FRC = 0x1,
    OSC_SYSPLL_IN_CLK_SOURCE_PRIMARY = 0x0

} OSC_SYSPLL_IN_CLK_SOURCE;

typedef enum {

    OSC_UPLL_FREQ_RANGE_10M_TO_16M = 0x01,
    OSC_UPLL_FREQ_RANGE_16M_TO_25M = 0x02,
    OSC_UPLL_FREQ_RANGE_25M_TO_40M = 0x03,
    OSC_UPLL_FREQ_RANGE_40M_TO_65M = 0x04,
    OSC_UPLL_FREQ_RANGE_65M_TO_100M = 0x05,
    OSC_UPLL_FREQ_RANGE_100M_TO_160M = 0x06,
    OSC_UPLL_FREQ_RANGE_BYPASS = 0x00

} OSC_UPLL_FREQ_RANGE;

typedef enum {

    OSC_BTPLL_FREQ_RANGE_10M_TO_16M = 0x01,
    OSC_BTPLL_FREQ_RANGE_16M_TO_25M = 0x02,
    OSC_BTPLL_FREQ_RANGE_25M_TO_40M = 0x03,
    OSC_BTPLL_FREQ_RANGE_40M_TO_65M = 0x04,
    OSC_BTPLL_FREQ_RANGE_65M_TO_100M = 0x05,
    OSC_BTPLL_FREQ_RANGE_100M_TO_160M = 0x06,
    OSC_BTPLL_FREQ_RANGE_BYPASS = 0x00

} OSC_BTPLL_FREQ_RANGE;

typedef enum {

    OSC_BTPLL_IN_CLK_SOURCE_FRC = 0x1,
    OSC_BTPLL_IN_CLK_SOURCE_PRIMARY = 0x0

} OSC_BTPLL_IN_CLK_SOURCE;

typedef enum {

    OSC_SLEEP_TO_STARTUP_CLK_FRC = 0x1,
    OSC_SLEEP_TO_STARTUP_NO_ADDITIONAL_CLK = 0x0

} OSC_SLEEP_TO_STARTUP_CLK_TYPE;

typedef enum {

    OSC_CLOCK_FAST_RC = 0x0,
    OSC_CLOCK_PRIMARY_OSC = 0x2,
    OSC_CLOCK_SECONDARY_OSC = 0x4,
    OSC_CLOCK_LOW_POWER_RC = 0x5,
    OSC_CLOCK_USB_PLL = 0x3,
    OSC_CLOCK_SYSTEM_PLL = 0x1,
    OSC_CLOCK_BT_PLL = 0x7

} OSC_CLOCK_ID;

typedef enum {

    OSC_CLOCK_POSC_STOP = 0x0,
    OSC_CLOCK_SOSC_STOP = 0x1,
    OSC_CLOCK_FRC_STOP = 0x2,
    OSC_CLOCK_LPRC_STOP = 0x3,
    OSC_CLOCK_SPLL_STOP = 0x4,
    OSC_CLOCK_UPLL_STOP = 0x5

} OSC_CLOCK_DIAG;

typedef enum {

    OSC_CLOCK_SLEW_DOWNWARD = 0x0000,
    OSC_CLOCK_SLEW_UPWARD = 0x0001

} OSC_CLOCK_SLEW_TYPE;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/osc_OnWaitAction_Default.h"
#include "../templates/osc_SecondaryEnable_Default.h"
#include "../templates/osc_SecondaryReady_Default_1.h"
#include "../templates/osc_FRCDivisor_Default.h"
#include "../templates/osc_OscSelect_wk.h"
#include "../templates/osc_OscSwitchInit_Default.h"
#include "../templates/osc_OscCurrentGet_wk.h"
#include "../templates/osc_PBClockDivisor_PIC32WK.h"
#include "../templates/osc_PBClockReady_PIC32WK.h"
#include "../templates/osc_UsbClockSource_Default.h"
#include "../templates/osc_PLLLockStatus_PIC32WK.h"
#include "../templates/osc_PLLClockLock_Default.h"
#include "../templates/osc_PLLMultiplier_PIC32WK.h"
#include "../templates/osc_UPLLMultiplier_PIC32WK.h"
#include "../templates/osc_BTPLLMultiplier_PIC32WK.h"
#include "../templates/osc_PLLOutputDivisor_PIC32WK.h"
#include "../templates/osc_UPLLOutputDivisor_PIC32WK.h"
#include "../templates/osc_BTPLLOutputDivisor_PIC32WK.h"
#include "../templates/osc_ClockFail_Default.h"
#include "../templates/osc_FRCTuning_Default.h"
#include "../templates/osc_ReferenceOscBaseClock_PIC32_2.h"
#include "../templates/osc_ReferenceOscChange_PIC32_2.h"
#include "../templates/osc_ReferenceOscChangeActive_PIC32_2.h"
#include "../templates/osc_ReferenceOscStopInSleep_PIC32_2.h"
#include "../templates/osc_ReferenceOutputEnable_PIC32_2.h"
#include "../templates/osc_ReferenceOscStopInIdleEnable_PIC32_2.h"
#include "../templates/osc_ReferenceOscEnable_PIC32_2.h"
#include "../templates/osc_ReferenceOscDivisor_PIC32_2.h"
#include "../templates/osc_ReferenceOscTrim_PIC32_2.h"
#include "../templates/osc_PBClockOutputEnable_PIC32WK.h"
#include "../templates/osc_PLLInputDivisor_wk.h"
#include "../templates/osc_UPLLInputDivisor_Default.h"
#include "../templates/osc_BTPLLInputDivisor_Default.h"
#include "../templates/osc_PLLInputClockSource_wk.h"
#include "../templates/osc_BTPLLInputClockSource_Default.h"
#include "../templates/osc_PLLFrequencyRange_wk.h"
#include "../templates/osc_BTPLLFrequencyRange_Default.h"
#include "../templates/osc_UPLLFrequencyRange_Default.h"
#include "../templates/osc_SleepToStartupClock_Default.h"
#include "../templates/osc_ClockReadyStatus_Default.h"
#include "../templates/osc_ClockDiagStatus_Default.h"
#include "../templates/osc_ClockSlewingStatus_Default.h"
#include "../templates/osc_SlewEnableControl_Default.h"
#include "../templates/osc_SlewDivisorStepControl_Default.h"
#include "../templates/osc_SystemClockDivisorControl_Default.h"
#include "../templates/osc_DreamModeControl_WK.h"
#include "../templates/osc_ForceLock_PIC32WK.h"
#include "../templates/osc_ResetPLL_PIC32WK.h"
#include "../templates/osc_PLLBypass_PIC32WK.h"
#include "../templates/osc_BTPLLClockOut_PIC32WK.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_OSC_ExistsOnWaitAction(OSC_MODULE_ID index)
{
     return OSC_ExistsOnWaitAction_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_OnWaitActionSet(OSC_MODULE_ID index, OSC_OPERATION_ON_WAIT onWait)
{
     OSC_OnWaitActionSet_Default(index, onWait);
}

PLIB_INLINE_API OSC_OPERATION_ON_WAIT PLIB_OSC_OnWaitActionGet(OSC_MODULE_ID index)
{
     return OSC_OnWaitActionGet_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsSecondaryEnable(OSC_MODULE_ID index)
{
     return OSC_ExistsSecondaryEnable_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_SecondaryEnable(OSC_MODULE_ID index)
{
     OSC_SecondaryEnable_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_SecondaryDisable(OSC_MODULE_ID index)
{
     OSC_SecondaryDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_SecondaryIsEnabled(OSC_MODULE_ID index)
{
     return OSC_SecondaryIsEnabled_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsSecondaryReady(OSC_MODULE_ID index)
{
     return OSC_ExistsSecondaryReady_Default_1(index);
}

PLIB_INLINE_API bool PLIB_OSC_SecondaryIsReady(OSC_MODULE_ID index)
{
     return OSC_SecondaryIsReady_Default_1(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsFRCDivisor(OSC_MODULE_ID index)
{
     return OSC_ExistsFRCDivisor_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_FRCDivisorSelect(OSC_MODULE_ID index, OSC_FRC_DIV divisorFRC)
{
     OSC_FRCDivisorSelect_Default(index, divisorFRC);
}

PLIB_INLINE_API uint16_t PLIB_OSC_FRCDivisorGet(OSC_MODULE_ID index)
{
     return OSC_FRCDivisorGet_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsOscSelect(OSC_MODULE_ID index)
{
     return OSC_ExistsOscSelect_wk(index);
}

PLIB_INLINE_API void PLIB_OSC_SysClockSelect(OSC_MODULE_ID index, OSC_SYS_TYPE newOsc)
{
     OSC_SysClockSelect_wk(index, newOsc);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsOscSwitchInit(OSC_MODULE_ID index)
{
     return OSC_ExistsOscSwitchInit_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_ClockSwitchingAbort(OSC_MODULE_ID index)
{
     OSC_ClockSwitchingAbort_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ClockSwitchingIsComplete(OSC_MODULE_ID index)
{
     return OSC_ClockSwitchingIsComplete_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsOscCurrentGet(OSC_MODULE_ID index)
{
     return OSC_ExistsOscCurrentGet_wk(index);
}

PLIB_INLINE_API OSC_SYS_TYPE PLIB_OSC_CurrentSysClockGet(OSC_MODULE_ID index)
{
     return OSC_CurrentSysClockGet_wk(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsPBClockDivisor(OSC_MODULE_ID index)
{
     return OSC_ExistsPBClockDivisor_PIC32WK(index);
}

PLIB_INLINE_API OSC_PB_CLOCK_DIV_TYPE PLIB_OSC_PBClockDivisorGet(OSC_MODULE_ID index, OSC_PERIPHERAL_BUS peripheralBusNumber)
{
     return OSC_PBClockDivisorGet_PIC32WK(index, peripheralBusNumber);
}

PLIB_INLINE_API void PLIB_OSC_PBClockDivisorSet(OSC_MODULE_ID index, OSC_PERIPHERAL_BUS peripheralBusNumber, OSC_PB_CLOCK_DIV_TYPE peripheralBusClkDiv)
{
     OSC_PBClockDivisorSet_PIC32WK(index, peripheralBusNumber, peripheralBusClkDiv);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsPBClockReady(OSC_MODULE_ID index)
{
     return OSC_ExistsPBClockReady_PIC32WK(index);
}

PLIB_INLINE_API bool PLIB_OSC_PBClockDivisorIsReady(OSC_MODULE_ID index, OSC_PERIPHERAL_BUS peripheralBusNumber)
{
     return OSC_PBClockDivisorIsReady_PIC32WK(index, peripheralBusNumber);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsUsbClockSource(OSC_MODULE_ID index)
{
     return OSC_ExistsUsbClockSource_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_UsbClockSourceSelect(OSC_MODULE_ID index, OSC_USBCLOCK_SOURCE usbClock)
{
     OSC_UsbClockSourceSelect_Default(index, usbClock);
}

PLIB_INLINE_API OSC_USBCLOCK_SOURCE PLIB_OSC_UsbClockSourceGet(OSC_MODULE_ID index)
{
     return OSC_UsbClockSourceGet_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsPLLLockStatus(OSC_MODULE_ID index)
{
     return OSC_ExistsPLLLockStatus_PIC32WK(index);
}

PLIB_INLINE_API bool PLIB_OSC_PLLIsLocked(OSC_MODULE_ID index, OSC_PLL_SELECT pllselect)
{
     return OSC_PLLIsLocked_PIC32WK(index, pllselect);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsPLLClockLock(OSC_MODULE_ID index)
{
     return OSC_ExistsPLLClockLock_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_PLLClockLock(OSC_MODULE_ID index)
{
     OSC_PLLClockLock_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_PLLClockUnlock(OSC_MODULE_ID index)
{
     OSC_PLLClockUnlock_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_PLLClockIsLocked(OSC_MODULE_ID index)
{
     return OSC_PLLClockIsLocked_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsSysPLLMultiplier(OSC_MODULE_ID index)
{
     return OSC_ExistsSysPLLMultiplier_PIC32WK(index);
}

PLIB_INLINE_API void PLIB_OSC_SysPLLMultiplierSelect(OSC_MODULE_ID index, OSC_SYSPLL_MULTIPLIER_TYPE pll_multiplier)
{
     OSC_SysPLLMultiplierSelect_PIC32WK(index, pll_multiplier);
}

PLIB_INLINE_API OSC_SYSPLL_MULTIPLIER_TYPE PLIB_OSC_SysPLLMultiplierGet(OSC_MODULE_ID index)
{
     return OSC_SysPLLMultiplierGet_PIC32WK(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsUPLLMultiplier(OSC_MODULE_ID index)
{
     return OSC_ExistsUPLLMultiplier_PIC32WK(index);
}

PLIB_INLINE_API void PLIB_OSC_UPLLMultiplierSelect(OSC_MODULE_ID index, OSC_SYSPLL_MULTIPLIER_TYPE pll_multiplier)
{
     OSC_UPLLMultiplierSelect_PIC32WK(index, pll_multiplier);
}

PLIB_INLINE_API OSC_SYSPLL_MULTIPLIER_TYPE PLIB_OSC_UPLLMultiplierGet(OSC_MODULE_ID index)
{
     return OSC_UPLLMultiplierGet_PIC32WK(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsBTPLLMultiplier(OSC_MODULE_ID index)
{
     return OSC_ExistsBTPLLMultiplier_PIC32WK(index);
}

PLIB_INLINE_API void PLIB_OSC_BTPLLMultiplierSelect(OSC_MODULE_ID index, OSC_SYSPLL_MULTIPLIER_TYPE pll_multiplier)
{
     OSC_BTPLLMultiplierSelect_PIC32WK(index, pll_multiplier);
}

PLIB_INLINE_API OSC_SYSPLL_MULTIPLIER_TYPE PLIB_OSC_BTPLLMultiplierGet(OSC_MODULE_ID index)
{
     return OSC_BTPLLMultiplierGet_PIC32WK(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsSysPLLOutputDivisor(OSC_MODULE_ID index)
{
     return OSC_ExistsSysPLLOutputDivisor_PIC32WK(index);
}

PLIB_INLINE_API void PLIB_OSC_SysPLLOutputDivisorSet(OSC_MODULE_ID index, OSC_SYSPLL_OUT_DIV PLLOutDiv)
{
     OSC_SysPLLOutputDivisorSet_PIC32WK(index, PLLOutDiv);
}

PLIB_INLINE_API uint16_t PLIB_OSC_SysPLLOutputDivisorGet(OSC_MODULE_ID index)
{
     return OSC_SysPLLOutputDivisorGet_PIC32WK(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsUPLLOutputDivisor(OSC_MODULE_ID index)
{
     return OSC_ExistsUPLLOutputDivisor_PIC32WK(index);
}

PLIB_INLINE_API void PLIB_OSC_UPLLOutputDivisorSet(OSC_MODULE_ID index, OSC_UPLL_OUT_DIV PLLOutDiv)
{
     OSC_UPLLOutputDivisorSet_PIC32WK(index, PLLOutDiv);
}

PLIB_INLINE_API uint16_t PLIB_OSC_UPLLOutputDivisorGet(OSC_MODULE_ID index)
{
     return OSC_UPLLOutputDivisorGet_PIC32WK(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsBTPLLOutputDivisor(OSC_MODULE_ID index)
{
     return OSC_ExistsBTPLLOutputDivisor_PIC32WK(index);
}

PLIB_INLINE_API void PLIB_OSC_BTPLLOutputDivisorSet(OSC_MODULE_ID index, OSC_BTPLL_OUT_DIV PLLOutDiv)
{
     OSC_BTPLLOutputDivisorSet_PIC32WK(index, PLLOutDiv);
}

PLIB_INLINE_API uint16_t PLIB_OSC_BTPLLOutputDivisorGet(OSC_MODULE_ID index)
{
     return OSC_BTPLLOutputDivisorGet_PIC32WK(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsClockFail(OSC_MODULE_ID index)
{
     return OSC_ExistsClockFail_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ClockHasFailed(OSC_MODULE_ID index)
{
     return OSC_ClockHasFailed_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsFRCTuning(OSC_MODULE_ID index)
{
     return OSC_ExistsFRCTuning_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_FRCTuningSelect(OSC_MODULE_ID index, OSC_FRC_TUNE_TYPE tuningValue)
{
     OSC_FRCTuningSelect_Default(index, tuningValue);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsReferenceOscBaseClock(OSC_MODULE_ID index)
{
     return OSC_ExistsReferenceOscBaseClock_PIC32_2(index);
}

PLIB_INLINE_API void PLIB_OSC_ReferenceOscBaseClockSelect(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc, OSC_REF_BASECLOCK refOscBaseClock)
{
     OSC_ReferenceOscBaseClockSelect_PIC32_2(index, referenceOsc, refOscBaseClock);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsReferenceOscChange(OSC_MODULE_ID index)
{
     return OSC_ExistsReferenceOscChange_PIC32_2(index);
}

PLIB_INLINE_API bool PLIB_OSC_ReferenceOscSwitchIsComplete(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc)
{
     return OSC_ReferenceOscSwitchIsComplete_PIC32_2(index, referenceOsc);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsReferenceOscChangeActive(OSC_MODULE_ID index)
{
     return OSC_ExistsReferenceOscChangeActive_PIC32_2(index);
}

PLIB_INLINE_API bool PLIB_OSC_ReferenceOscSourceChangeIsActive(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc)
{
     return OSC_ReferenceOscSourceChangeIsActive_PIC32_2(index, referenceOsc);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsReferenceOscStopInSleep(OSC_MODULE_ID index)
{
     return OSC_ExistsReferenceOscStopInSleep_PIC32_2(index);
}

PLIB_INLINE_API void PLIB_OSC_ReferenceOscStopInSleepEnable(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc)
{
     OSC_ReferenceOscStopInSleepEnable_PIC32_2(index, referenceOsc);
}

PLIB_INLINE_API void PLIB_OSC_ReferenceOscStopInSleepDisable(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc)
{
     OSC_ReferenceOscStopInSleepDisable_PIC32_2(index, referenceOsc);
}

PLIB_INLINE_API bool PLIB_OSC_ReferenceOscStopInSleepIsEnabled(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc)
{
     return OSC_ReferenceOscStopInSleepIsEnabled_PIC32_2(index, referenceOsc);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsReferenceOutputEnable(OSC_MODULE_ID index)
{
     return OSC_ExistsReferenceOutputEnable_PIC32_2(index);
}

PLIB_INLINE_API void PLIB_OSC_ReferenceOutputEnable(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc)
{
     OSC_ReferenceOutputEnable_PIC32_2(index, referenceOsc);
}

PLIB_INLINE_API void PLIB_OSC_ReferenceOutputDisable(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc)
{
     OSC_ReferenceOutputDisable_PIC32_2(index, referenceOsc);
}

PLIB_INLINE_API bool PLIB_OSC_ReferenceOutputIsEnabled(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc)
{
     return OSC_ReferenceOutputIsEnabled_PIC32_2(index, referenceOsc);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsReferenceOscStopInIdleEnable(OSC_MODULE_ID index)
{
     return OSC_ExistsReferenceOscStopInIdleEnable_PIC32_2(index);
}

PLIB_INLINE_API void PLIB_OSC_ReferenceOscStopInIdleEnable(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc)
{
     OSC_ReferenceOscStopInIdleEnable_PIC32_2(index, referenceOsc);
}

PLIB_INLINE_API void PLIB_OSC_ReferenceOscStopInIdleDisable(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc)
{
     OSC_ReferenceOscStopInIdleDisable_PIC32_2(index, referenceOsc);
}

PLIB_INLINE_API bool PLIB_OSC_ReferenceOscStopInIdleIsEnabled(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc)
{
     return OSC_ReferenceOscStopInIdleIsEnabled_PIC32_2(index, referenceOsc);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsReferenceOscEnable(OSC_MODULE_ID index)
{
     return OSC_ExistsReferenceOscEnable_PIC32_2(index);
}

PLIB_INLINE_API void PLIB_OSC_ReferenceOscEnable(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc)
{
     OSC_ReferenceOscEnable_PIC32_2(index, referenceOsc);
}

PLIB_INLINE_API void PLIB_OSC_ReferenceOscDisable(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc)
{
     OSC_ReferenceOscDisable_PIC32_2(index, referenceOsc);
}

PLIB_INLINE_API bool PLIB_OSC_ReferenceOscIsEnabled(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc)
{
     return OSC_ReferenceOscIsEnabled_PIC32_2(index, referenceOsc);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsReferenceOscDivisor(OSC_MODULE_ID index)
{
     return OSC_ExistsReferenceOscDivisor_PIC32_2(index);
}

PLIB_INLINE_API void PLIB_OSC_ReferenceOscDivisorValueSet(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc, OSC_REF_DIVISOR_TYPE refOscDivValue)
{
     OSC_ReferenceOscDivisorValueSet_PIC32_2(index, referenceOsc, refOscDivValue);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsReferenceOscTrim(OSC_MODULE_ID index)
{
     return OSC_ExistsReferenceOscTrim_PIC32_2(index);
}

PLIB_INLINE_API void PLIB_OSC_ReferenceOscTrimSet(OSC_MODULE_ID index, OSC_REFERENCE referenceOsc, OSC_REF_TRIM_TYPE trimValue)
{
     OSC_ReferenceOscTrimSet_PIC32_2(index, referenceOsc, trimValue);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsPBClockOutputEnable(OSC_MODULE_ID index)
{
     return OSC_ExistsPBClockOutputEnable_PIC32WK(index);
}

PLIB_INLINE_API void PLIB_OSC_PBOutputClockEnable(OSC_MODULE_ID index, OSC_PERIPHERAL_BUS peripheralBusNumber)
{
     OSC_PBOutputClockEnable_PIC32WK(index, peripheralBusNumber);
}

PLIB_INLINE_API void PLIB_OSC_PBOutputClockDisable(OSC_MODULE_ID index, OSC_PERIPHERAL_BUS peripheralBusNumber)
{
     OSC_PBOutputClockDisable_PIC32WK(index, peripheralBusNumber);
}

PLIB_INLINE_API bool PLIB_OSC_PBOutputClockIsEnabled(OSC_MODULE_ID index, OSC_PERIPHERAL_BUS peripheralBusNumber)
{
     return OSC_PBOutputClockIsEnabled_PIC32WK(index, peripheralBusNumber);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsSysPLLInputDivisor(OSC_MODULE_ID index)
{
     return OSC_ExistsSysPLLInputDivisor_wk(index);
}

PLIB_INLINE_API void PLIB_OSC_SysPLLInputDivisorSet(OSC_MODULE_ID index, uint16_t PLLInDiv)
{
     OSC_SysPLLInputDivisorSet_wk(index, PLLInDiv);
}

PLIB_INLINE_API uint16_t PLIB_OSC_SysPLLInputDivisorGet(OSC_MODULE_ID index)
{
     return OSC_SysPLLInputDivisorGet_wk(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsUPLLInputDivisor(OSC_MODULE_ID index)
{
     return OSC_ExistsUPLLInputDivisor_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_UPLLInputDivisorSet(OSC_MODULE_ID index, uint16_t PLLInDiv)
{
     OSC_UPLLInputDivisorSet_Default(index, PLLInDiv);
}

PLIB_INLINE_API uint16_t PLIB_OSC_UPLLInputDivisorGet(OSC_MODULE_ID index)
{
     return OSC_UPLLInputDivisorGet_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsBTPLLInputDivisor(OSC_MODULE_ID index)
{
     return OSC_ExistsBTPLLInputDivisor_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_BTPLLInputDivisorSet(OSC_MODULE_ID index, uint16_t PLLInDiv)
{
     OSC_BTPLLInputDivisorSet_Default(index, PLLInDiv);
}

PLIB_INLINE_API uint16_t PLIB_OSC_BTPLLInputDivisorGet(OSC_MODULE_ID index)
{
     return OSC_BTPLLInputDivisorGet_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsSysPLLInputClockSource(OSC_MODULE_ID index)
{
     return OSC_ExistsSysPLLInputClockSource_wk(index);
}

PLIB_INLINE_API void PLIB_OSC_SysPLLInputClockSourceSet(OSC_MODULE_ID index, OSC_SYSPLL_IN_CLK_SOURCE PLLInClockSource)
{
     OSC_SysPLLInputClockSourceSet_wk(index, PLLInClockSource);
}

PLIB_INLINE_API OSC_SYSPLL_IN_CLK_SOURCE PLIB_OSC_SysPLLInputClockSourceGet(OSC_MODULE_ID index)
{
     return OSC_SysPLLInputClockSourceGet_wk(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsBTPLLInputClockSource(OSC_MODULE_ID index)
{
     return OSC_ExistsBTPLLInputClockSource_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_BTPLLInputClockSourceSet(OSC_MODULE_ID index, OSC_BTPLL_IN_CLK_SOURCE PLLInClockSource)
{
     OSC_BTPLLInputClockSourceSet_Default(index, PLLInClockSource);
}

PLIB_INLINE_API OSC_SYSPLL_IN_CLK_SOURCE PLIB_OSC_BTPLLInputClockSourceGet(OSC_MODULE_ID index)
{
     return OSC_BTPLLInputClockSourceGet_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsSysPLLFrequencyRange(OSC_MODULE_ID index)
{
     return OSC_ExistsSysPLLFrequencyRange_wk(index);
}

PLIB_INLINE_API void PLIB_OSC_SysPLLFrequencyRangeSet(OSC_MODULE_ID index, OSC_SYSPLL_FREQ_RANGE PLLFrequencyRange)
{
     OSC_SysPLLFrequencyRangeSet_wk(index, PLLFrequencyRange);
}

PLIB_INLINE_API OSC_SYSPLL_FREQ_RANGE PLIB_OSC_SysPLLFrequencyRangeGet(OSC_MODULE_ID index)
{
     return OSC_SysPLLFrequencyRangeGet_wk(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsBTPLLFrequencyRange(OSC_MODULE_ID index)
{
     return OSC_ExistsBTPLLFrequencyRange_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_BTPLLFrequencyRangeSet(OSC_MODULE_ID index, OSC_BTPLL_FREQ_RANGE PLLFrequencyRange)
{
     OSC_BTPLLFrequencyRangeSet_Default(index, PLLFrequencyRange);
}

PLIB_INLINE_API OSC_BTPLL_FREQ_RANGE PLIB_OSC_BTPLLFrequencyRangeGet(OSC_MODULE_ID index)
{
     return OSC_BTPLLFrequencyRangeGet_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsUPLLFrequencyRange(OSC_MODULE_ID index)
{
     return OSC_ExistsUPLLFrequencyRange_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_UPLLFrequencyRangeSet(OSC_MODULE_ID index, OSC_UPLL_FREQ_RANGE PLLFrequencyRange)
{
     OSC_UPLLFrequencyRangeSet_Default(index, PLLFrequencyRange);
}

PLIB_INLINE_API OSC_UPLL_FREQ_RANGE PLIB_OSC_UPLLFrequencyRangeGet(OSC_MODULE_ID index)
{
     return OSC_UPLLFrequencyRangeGet_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsSleepToStartupClock(OSC_MODULE_ID index)
{
     return OSC_ExistsSleepToStartupClock_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_SleepToStartupClockSelect(OSC_MODULE_ID index, OSC_SLEEP_TO_STARTUP_CLK_TYPE startupOsc)
{
     OSC_SleepToStartupClockSelect_Default(index, startupOsc);
}

PLIB_INLINE_API OSC_SLEEP_TO_STARTUP_CLK_TYPE PLIB_OSC_SleepToStartupClockGet(OSC_MODULE_ID index)
{
     return OSC_SleepToStartupClockGet_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsClockReadyStatus(OSC_MODULE_ID index)
{
     return OSC_ExistsClockReadyStatus_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ClockIsReady(OSC_MODULE_ID index, OSC_CLOCK_ID clk)
{
     return OSC_ClockIsReady_Default(index, clk);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsClockDiagStatus(OSC_MODULE_ID index)
{
     return OSC_ExistsClockDiagStatus_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_ClockStop(OSC_MODULE_ID index, OSC_CLOCK_DIAG clk)
{
     OSC_ClockStop_Default(index, clk);
}

PLIB_INLINE_API void PLIB_OSC_ClockStart(OSC_MODULE_ID index, OSC_CLOCK_DIAG clk)
{
     OSC_ClockStart_Default(index, clk);
}

PLIB_INLINE_API bool PLIB_OSC_ClockStopStatus(OSC_MODULE_ID index, OSC_CLOCK_DIAG clk)
{
     return OSC_ClockStopStatus_Default(index, clk);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsClockSlewingStatus(OSC_MODULE_ID index)
{
     return OSC_ExistsClockSlewingStatus_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ClockSlewingIsActive(OSC_MODULE_ID index)
{
     return OSC_ClockSlewingIsActive_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsSlewEnableControl(OSC_MODULE_ID index)
{
     return OSC_ExistsSlewEnableControl_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_SlewEnable(OSC_MODULE_ID index, OSC_CLOCK_SLEW_TYPE slewType)
{
     OSC_SlewEnable_Default(index, slewType);
}

PLIB_INLINE_API void PLIB_OSC_SlewDisable(OSC_MODULE_ID index, OSC_CLOCK_SLEW_TYPE slewType)
{
     OSC_SlewDisable_Default(index, slewType);
}

PLIB_INLINE_API bool PLIB_OSC_SlewIsEnabled(OSC_MODULE_ID index, OSC_CLOCK_SLEW_TYPE slewType)
{
     return OSC_SlewIsEnabled_Default(index, slewType);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsSlewDivisorStepControl(OSC_MODULE_ID index)
{
     return OSC_ExistsSlewDivisorStepControl_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_SlewDivisorStepSelect(OSC_MODULE_ID index, uint32_t slewSteps)
{
     OSC_SlewDivisorStepSelect_Default(index, slewSteps);
}

PLIB_INLINE_API uint32_t PLIB_OSC_SlewDivisorStepGet(OSC_MODULE_ID index)
{
     return OSC_SlewDivisorStepGet_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsSystemClockDivisorControl(OSC_MODULE_ID index)
{
     return OSC_ExistsSystemClockDivisorControl_Default(index);
}

PLIB_INLINE_API void PLIB_OSC_SystemClockDivisorSelect(OSC_MODULE_ID index, uint32_t systemClkDivisor)
{
     OSC_SystemClockDivisorSelect_Default(index, systemClkDivisor);
}

PLIB_INLINE_API uint32_t PLIB_OSC_SystemClockDivisorGet(OSC_MODULE_ID index)
{
     return OSC_SystemClockDivisorGet_Default(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsDreamModeControl(OSC_MODULE_ID index)
{
     return OSC_ExistsDreamModeControl_WK(index);
}

PLIB_INLINE_API void PLIB_OSC_DreamModeEnable(OSC_MODULE_ID index)
{
     OSC_DreamModeEnable_WK(index);
}

PLIB_INLINE_API void PLIB_OSC_DreamModeDisable(OSC_MODULE_ID index)
{
     OSC_DreamModeDisable_WK(index);
}

PLIB_INLINE_API bool PLIB_OSC_DreamModeStatus(OSC_MODULE_ID index)
{
     return OSC_DreamModeStatus_WK(index);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsForceLock(OSC_MODULE_ID index)
{
     return OSC_ExistsForceLock_PIC32WK(index);
}

PLIB_INLINE_API void PLIB_OSC_ForceSPLLLockEnable(OSC_MODULE_ID index, OSC_PLL_SELECT pllSel)
{
     OSC_ForceSPLLLockEnable_PIC32WK(index, pllSel);
}

PLIB_INLINE_API void PLIB_OSC_ForceSPLLLockDisable(OSC_MODULE_ID index, OSC_PLL_SELECT pllSel)
{
     OSC_ForceSPLLLockDisable_PIC32WK(index, pllSel);
}

PLIB_INLINE_API bool PLIB_OSC_ForceSPLLLockStatus(OSC_MODULE_ID index, OSC_PLL_SELECT pllSel)
{
     return OSC_ForceSPLLLockStatus_PIC32WK(index, pllSel);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsResetPLL(OSC_MODULE_ID index)
{
     return OSC_ExistsResetPLL_PIC32WK(index);
}

PLIB_INLINE_API void PLIB_OSC_ResetPLLAssert(OSC_MODULE_ID index, OSC_PLL_SELECT pllSel)
{
     OSC_ResetPLLAssert_PIC32WK(index, pllSel);
}

PLIB_INLINE_API void PLIB_OSC_ResetPLLDeassert(OSC_MODULE_ID index, OSC_PLL_SELECT pllSel)
{
     OSC_ResetPLLDeassert_PIC32WK(index, pllSel);
}

PLIB_INLINE_API bool PLIB_OSC_ResetPLLStatus(OSC_MODULE_ID index, OSC_PLL_SELECT pllSel)
{
     return OSC_ResetPLLStatus_PIC32WK(index, pllSel);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsPLLBypass(OSC_MODULE_ID index)
{
     return OSC_ExistsPLLBypass_PIC32WK(index);
}

PLIB_INLINE_API void PLIB_OSC_PLLBypassEnable(OSC_MODULE_ID index, OSC_PLL_SELECT pllSel)
{
     OSC_PLLBypassEnable_PIC32WK(index, pllSel);
}

PLIB_INLINE_API void PLIB_OSC_PLLBypassDisable(OSC_MODULE_ID index, OSC_PLL_SELECT pllSel)
{
     OSC_PLLBypassDisable_PIC32WK(index, pllSel);
}

PLIB_INLINE_API bool PLIB_OSC_PLLBypassStatus(OSC_MODULE_ID index, OSC_PLL_SELECT pllSel)
{
     return OSC_PLLBypassStatus_PIC32WK(index, pllSel);
}

PLIB_INLINE_API bool PLIB_OSC_ExistsBTPLLClockOut(OSC_MODULE_ID index)
{
     return OSC_ExistsBTPLLClockOut_PIC32WK(index);
}

PLIB_INLINE_API void PLIB_OSC_BTPLLClockOutEnable(OSC_MODULE_ID index)
{
     OSC_BTPLLClockOutEnable_PIC32WK(index);
}

PLIB_INLINE_API void PLIB_OSC_BTPLLClockOutDisable(OSC_MODULE_ID index)
{
     OSC_BTPLLClockOutDisable_PIC32WK(index);
}

PLIB_INLINE_API bool PLIB_OSC_BTPLLClockOutStatus(OSC_MODULE_ID index)
{
     return OSC_BTPLLClockOutStatus_PIC32WK(index);
}

#endif
