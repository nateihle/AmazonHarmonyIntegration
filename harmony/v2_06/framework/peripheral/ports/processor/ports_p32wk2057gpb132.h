/* Created by plibgen $Revision: 1.31 $ */

#ifndef _PORTS_P32WK2057GPB132_H
#define _PORTS_P32WK2057GPB132_H

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

    PORTS_ID_0 = 0,
    PORTS_NUMBER_OF_MODULES = 1

} PORTS_MODULE_ID;

typedef enum {

    PORTS_PIN_MODE_ANALOG = 0,
    PORTS_PIN_MODE_DIGITAL = 1

} PORTS_PIN_MODE;

typedef enum {

    PORTS_CHANGE_NOTICE_PIN_NONE

} PORTS_CHANGE_NOTICE_PIN;

typedef enum {

    PORTS_CN_PIN_NONE

} PORTS_CN_PIN;

typedef enum {

    PORTS_ANALOG_PIN_0 = 8,
    PORTS_ANALOG_PIN_1 = 9,
    PORTS_ANALOG_PIN_2 = 18,
    PORTS_ANALOG_PIN_3 = 19,
    PORTS_ANALOG_PIN_4 = 20,
    PORTS_ANALOG_PIN_5 = 21,
    PORTS_ANALOG_PIN_6 = 22,
    PORTS_ANALOG_PIN_7 = 23,
    PORTS_ANALOG_PIN_8 = 24,
    PORTS_ANALOG_PIN_9 = 25,
    PORTS_ANALOG_PIN_10 = 28,
    PORTS_ANALOG_PIN_11 = 26,
    PORTS_ANALOG_PIN_12 = 17,
    PORTS_ANALOG_PIN_13 = 29,
    PORTS_ANALOG_PIN_14 = 10,
    PORTS_ANALOG_PIN_15 = 11

} PORTS_ANALOG_PIN;

typedef enum {

    PORTS_AN_PIN_NONE

} PORTS_AN_PIN;

typedef enum {

    PORTS_BIT_POS_0 = 0,
    PORTS_BIT_POS_1 = 1,
    PORTS_BIT_POS_2 = 2,
    PORTS_BIT_POS_3 = 3,
    PORTS_BIT_POS_4 = 4,
    PORTS_BIT_POS_5 = 5,
    PORTS_BIT_POS_6 = 6,
    PORTS_BIT_POS_7 = 7,
    PORTS_BIT_POS_8 = 8,
    PORTS_BIT_POS_9 = 9,
    PORTS_BIT_POS_10 = 10,
    PORTS_BIT_POS_11 = 11,
    PORTS_BIT_POS_12 = 12,
    PORTS_BIT_POS_13 = 13,
    PORTS_BIT_POS_14 = 14,
    PORTS_BIT_POS_15 = 15

} PORTS_BIT_POS;

typedef enum {

    INPUT_FUNC_INT0 = 0,
    INPUT_FUNC_INT1 = 1,
    INPUT_FUNC_INT2 = 2,
    INPUT_FUNC_INT3 = 3,
    INPUT_FUNC_INT4 = 4,
    INPUT_FUNC_T1CK = 5,
    INPUT_FUNC_T2CK = 6,
    INPUT_FUNC_T3CK = 7,
    INPUT_FUNC_T4CK = 8,
    INPUT_FUNC_T5CK = 9,
    INPUT_FUNC_T6CK = 10,
    INPUT_FUNC_T7CK = 11,
    INPUT_FUNC_IC1 = 14,
    INPUT_FUNC_IC2 = 15,
    INPUT_FUNC_IC3 = 16,
    INPUT_FUNC_IC4 = 17,
    INPUT_FUNC_OCFA = 23,
    INPUT_FUNC_OCFB = 24,
    INPUT_FUNC_OCFC = 25,
    INPUT_FUNC_OCFD = 26,
    INPUT_FUNC_U1RX = 27,
    INPUT_FUNC_U1CTS = 28,
    INPUT_FUNC_U2RX = 29,
    INPUT_FUNC_U2CTS = 30,
    INPUT_FUNC_PTG30 = 35,
    INPUT_FUNC_PTG31 = 36,
    INPUT_FUNC_SDI1 = 38,
    INPUT_FUNC_SS1 = 39,
    INPUT_FUNC_SDI2 = 41,
    INPUT_FUNC_SS2 = 42,
    INPUT_FUNC_REFCLKI1 = 51

} PORTS_REMAP_INPUT_FUNCTION;

typedef enum {

    INPUT_PIN_RPB9 = 0x06,
    INPUT_PIN_RPB10 = 0x06,
    INPUT_PIN_RPB5 = 0x05,
    INPUT_PIN_RPC1 = 0x08,
    INPUT_PIN_RPB1 = 0x04,
    INPUT_PIN_RPB3 = 0x04,
    INPUT_PIN_RPC4 = 0x09,
    INPUT_PIN_RPB8 = 0x06,
    INPUT_PIN_RPB0 = 0x04,
    INPUT_PIN_RPB7 = 0x05,
    INPUT_PIN_RPC3 = 0x08,
    INPUT_PIN_RPB14 = 0x07,
    INPUT_PIN_RPB6 = 0x05,
    INPUT_PIN_RPB2 = 0x04,
    INPUT_PIN_RPC2 = 0x08,
    INPUT_PIN_RPA0 = 0x00,
    INPUT_PIN_RPB4 = 0x05,
    INPUT_PIN_RPC0 = 0x08,
    INPUT_PIN_RPC5 = 0x09,
    INPUT_PIN_RPA1 = 0x00,
    INPUT_PIN_RPB11 = 0x06,
    INPUT_PIN_RPA8 = 0x02,
    INPUT_PIN_RPA9 = 0x02,
    INPUT_PIN_RPA2 = 0x00,
    INPUT_PIN_RPA4 = 0x01,
    INPUT_PIN_RPB13 = 0x07,
    INPUT_PIN_RPA3 = 0x00,
    INPUT_PIN_RPA5 = 0x01,
    INPUT_PIN_RPA6 = 0x01,
    INPUT_PIN_RPA7 = 0x01,
    INPUT_PIN_RPA10 = 0x02,
    INPUT_PIN_RPA11 = 0x02,
    INPUT_PIN_RPB12 = 0x07,
    INPUT_PIN_PTG30 = 0x0E,
    INPUT_PIN_PTG31 = 0x0E

} PORTS_REMAP_INPUT_PIN;

typedef enum {

    OUTPUT_FUNC_SDO1 = 0x03,
    OUTPUT_FUNC_SDO2 = 0x04,
    OUTPUT_FUNC_OC3 = 0x05,
    OUTPUT_FUNC_REFCLKO4 = 0x12,
    OUTPUT_FUNC_U1TX = 0x01,
    OUTPUT_FUNC_U2RTS = 0x02,
    OUTPUT_FUNC_OC4 = 0x05,
    OUTPUT_FUNC_REFCLKO1 = 0x12,
    OUTPUT_FUNC_SS1 = 0x03,
    OUTPUT_FUNC_REFCLKO3 = 0x12,
    OUTPUT_FUNC_U1RTS = 0x01,
    OUTPUT_FUNC_U2TX = 0x02,
    OUTPUT_FUNC_SS2 = 0x04,
    OUTPUT_FUNC_OC2 = 0x05,
    OUTPUT_FUNC_OC1 = 0x05,
    OUTPUT_FUNC_PTG28 = 0x13,
    OUTPUT_FUNC_PTG29 = 0x13,
    OUTPUT_FUNC_PTG30 = 0x13,
    OUTPUT_FUNC_PTG31 = 0x13,
    OUTPUT_FUNC_REFCLKO2 = 0x12,
    OUTPUT_FUNC_NO_CONNECT = 0x00

} PORTS_REMAP_OUTPUT_FUNCTION;

typedef enum {

    OUTPUT_PIN_RPA0 = 0,
    OUTPUT_PIN_RPA1 = 1,
    OUTPUT_PIN_RPA2 = 2,
    OUTPUT_PIN_RPA3 = 3,
    OUTPUT_PIN_RPA4 = 4,
    OUTPUT_PIN_RPA5 = 5,
    OUTPUT_PIN_RPA6 = 6,
    OUTPUT_PIN_RPA7 = 7,
    OUTPUT_PIN_RPA8 = 8,
    OUTPUT_PIN_RPA9 = 9,
    OUTPUT_PIN_RPA10 = 10,
    OUTPUT_PIN_RPA11 = 11,
    OUTPUT_PIN_RPB0 = 16,
    OUTPUT_PIN_RPB1 = 17,
    OUTPUT_PIN_RPB2 = 18,
    OUTPUT_PIN_RPB3 = 19,
    OUTPUT_PIN_RPB4 = 20,
    OUTPUT_PIN_RPB5 = 21,
    OUTPUT_PIN_RPB6 = 22,
    OUTPUT_PIN_RPB7 = 23,
    OUTPUT_PIN_RPB8 = 24,
    OUTPUT_PIN_RPB9 = 25,
    OUTPUT_PIN_RPB10 = 26,
    OUTPUT_PIN_RPB11 = 27,
    OUTPUT_PIN_RPB13 = 29,
    OUTPUT_PIN_RPB14 = 30,
    OUTPUT_PIN_RPC0 = 32,
    OUTPUT_PIN_RPC1 = 33,
    OUTPUT_PIN_RPC2 = 34,
    OUTPUT_PIN_RPC3 = 35,
    OUTPUT_PIN_RPC4 = 36,
    OUTPUT_PIN_RPC5 = 37

} PORTS_REMAP_OUTPUT_PIN;

typedef enum {

    PORT_CHANNEL_A = 0x00,
    PORT_CHANNEL_B = 0x01,
    PORT_CHANNEL_C = 0x02,
    PORT_CHANNEL_K = 0x03

} PORTS_CHANNEL;

typedef enum {

    PORTS_CHANGE_NOTICE_EDGE_RISING = 0,
    PORTS_CHANGE_NOTICE_EDGE_FALLING = 1

} PORTS_CHANGE_NOTICE_EDGE;

typedef enum {

    PORTS_PIN_SLEW_RATE_FASTEST = 0x05,
    PORTS_PIN_SLEW_RATE_FAST = 0x06,
    PORTS_PIN_SLEW_RATE_SLOW = 0x09,
    PORTS_PIN_SLEW_RATE_SLOWEST = 0x0A

} PORTS_PIN_SLEW_RATE;

typedef enum {

    PORTS_CHANGE_NOTICE_METHOD_SAMPLED = 0,
    PORTS_CHANGE_NOTICE_METHOD_EDGE_DETECT = 1

} PORTS_CHANGE_NOTICE_METHOD;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/ports_RemapInput_WK.h"
#include "../templates/ports_RemapOutput_PIC32_2.h"
#include "../templates/ports_PinMode_PPS.h"
#include "../templates/ports_AnPinsMode_Unsupported.h"
#include "../templates/ports_PortsRead_MCU32_PPS.h"
#include "../templates/ports_LatchRead_MCU32_PPS.h"
#include "../templates/ports_PortsWrite_MCU32_PPS.h"
#include "../templates/ports_PortsDirection_MCU32_PPS.h"
#include "../templates/ports_PortsOpenDrain_MCU32_PPS.h"
#include "../templates/ports_ChangeNotice_Unsupported.h"
#include "../templates/ports_PinChangeNotice_Unsupported.h"
#include "../templates/ports_ChangeNoticeInIdle_Unsupported.h"
#include "../templates/ports_ChangeNoticePullup_Unsupported.h"
#include "../templates/ports_PinModePerPort_Default.h"
#include "../templates/ports_ChangeNoticePullDownPerPort_Default.h"
#include "../templates/ports_ChangeNoticePullUpPerPort_Default.h"
#include "../templates/ports_PinChangeNoticePerPort_Default.h"
#include "../templates/ports_ChangeNoticePerPortTurnOn_Default.h"
#include "../templates/ports_ChangeNoticeInIdlePerPort_Default.h"
#include "../templates/ports_ChangeNoticePerPortStatus_Default.h"
#include "../templates/ports_SlewRateControl_Default.h"
#include "../templates/ports_ChannelChangeNoticeMethod_Default.h"
#include "../templates/ports_ChangeNoticeEdgeControl_Default.h"
#include "../templates/ports_ChangeNoticeEdgeStatus_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API void PLIB_PORTS_RemapInput(PORTS_MODULE_ID index, PORTS_REMAP_INPUT_FUNCTION inputFunction, PORTS_REMAP_INPUT_PIN remapInputPin)
{
     PORTS_RemapInput_WK(index, inputFunction, remapInputPin);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsRemapInput(PORTS_MODULE_ID index)
{
     return PORTS_ExistsRemapInput_WK(index);
}

PLIB_INLINE_API void PLIB_PORTS_RemapOutput(PORTS_MODULE_ID index, PORTS_REMAP_OUTPUT_FUNCTION outputFunction, PORTS_REMAP_OUTPUT_PIN remapOutputPin)
{
     PORTS_RemapOutput_PIC32_2(index, outputFunction, remapOutputPin);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsRemapOutput(PORTS_MODULE_ID index)
{
     return PORTS_ExistsRemapOutput_PIC32_2(index);
}

PLIB_INLINE_API void PLIB_PORTS_PinModeSelect(PORTS_MODULE_ID index, PORTS_ANALOG_PIN pin, PORTS_PIN_MODE mode)
{
     PORTS_PinModeSelect_PPS(index, pin, mode);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsPinMode(PORTS_MODULE_ID index)
{
     return PORTS_ExistsPinMode_PPS(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_AnPinsModeSelect(PORTS_MODULE_ID index, PORTS_AN_PIN anPins, PORTS_PIN_MODE mode)
{
     PORTS_AnPinsModeSelect_Unsupported(index, anPins, mode);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsAnPinsMode(PORTS_MODULE_ID index)
{
     return PORTS_ExistsAnPinsMode_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_PORTS_PinGet(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     return PORTS_PinGet_MCU32_PPS(index, channel, bitPos);
}

PLIB_INLINE_API PORTS_DATA_TYPE PLIB_PORTS_Read(PORTS_MODULE_ID index, PORTS_CHANNEL channel)
{
     return PORTS_Read_MCU32_PPS(index, channel);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsPortsRead(PORTS_MODULE_ID index)
{
     return PORTS_ExistsPortsRead_MCU32_PPS(index);
}

PLIB_INLINE_API bool PLIB_PORTS_PinGetLatched(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     return PORTS_PinGetLatched_MCU32_PPS(index, channel, bitPos);
}

PLIB_INLINE_API PORTS_DATA_TYPE PLIB_PORTS_ReadLatched(PORTS_MODULE_ID index, PORTS_CHANNEL channel)
{
     return PORTS_ReadLatched_MCU32_PPS(index, channel);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsLatchRead(PORTS_MODULE_ID index)
{
     return PORTS_ExistsLatchRead_MCU32_PPS(index);
}

PLIB_INLINE_API void PLIB_PORTS_PinWrite(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos, bool value)
{
     PORTS_PinWrite_MCU32_PPS(index, channel, bitPos, value);
}

PLIB_INLINE_API void PLIB_PORTS_PinSet(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     PORTS_PinSet_MCU32_PPS(index, channel, bitPos);
}

PLIB_INLINE_API void PLIB_PORTS_PinClear(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     PORTS_PinClear_MCU32_PPS(index, channel, bitPos);
}

PLIB_INLINE_API void PLIB_PORTS_PinToggle(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     PORTS_PinToggle_MCU32_PPS(index, channel, bitPos);
}

PLIB_INLINE_API void PLIB_PORTS_Write(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_TYPE value)
{
     PORTS_Write_MCU32_PPS(index, channel, value);
}

PLIB_INLINE_API void PLIB_PORTS_Set(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_TYPE value, PORTS_DATA_MASK mask)
{
     PORTS_Set_MCU32_PPS(index, channel, value, mask);
}

PLIB_INLINE_API void PLIB_PORTS_Toggle(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK toggleMask)
{
     PORTS_Toggle_MCU32_PPS(index, channel, toggleMask);
}

PLIB_INLINE_API void PLIB_PORTS_Clear(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK clearMask)
{
     PORTS_Clear_MCU32_PPS(index, channel, clearMask);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsPortsWrite(PORTS_MODULE_ID index)
{
     return PORTS_ExistsPortsWrite_MCU32_PPS(index);
}

PLIB_INLINE_API void PLIB_PORTS_PinDirectionInputSet(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     PORTS_PinDirectionInputSet_MCU32_PPS(index, channel, bitPos);
}

PLIB_INLINE_API void PLIB_PORTS_PinDirectionOutputSet(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     PORTS_PinDirectionOutputSet_MCU32_PPS(index, channel, bitPos);
}

PLIB_INLINE_API void PLIB_PORTS_DirectionInputSet(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK mask)
{
     PORTS_DirectionInputSet_MCU32_PPS(index, channel, mask);
}

PLIB_INLINE_API void PLIB_PORTS_DirectionOutputSet(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK mask)
{
     PORTS_DirectionOutputSet_MCU32_PPS(index, channel, mask);
}

PLIB_INLINE_API PORTS_DATA_MASK PLIB_PORTS_DirectionGet(PORTS_MODULE_ID index, PORTS_CHANNEL channel)
{
     return PORTS_DirectionGet_MCU32_PPS(index, channel);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsPortsDirection(PORTS_MODULE_ID index)
{
     return PORTS_ExistsPortsDirection_MCU32_PPS(index);
}

PLIB_INLINE_API void PLIB_PORTS_PinOpenDrainEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     PORTS_PinOpenDrainEnable_MCU32_PPS(index, channel, bitPos);
}

PLIB_INLINE_API void PLIB_PORTS_PinOpenDrainDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     PORTS_PinOpenDrainDisable_MCU32_PPS(index, channel, bitPos);
}

PLIB_INLINE_API void PLIB_PORTS_OpenDrainEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK mask)
{
     PORTS_OpenDrainEnable_MCU32_PPS(index, channel, mask);
}

PLIB_INLINE_API void PLIB_PORTS_OpenDrainDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK mask)
{
     PORTS_OpenDrainDisable_MCU32_PPS(index, channel, mask);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsPortsOpenDrain(PORTS_MODULE_ID index)
{
     return PORTS_ExistsPortsOpenDrain_MCU32_PPS(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_ChangeNoticeEnable(PORTS_MODULE_ID index)
{
     PORTS_ChangeNoticeEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_ChangeNoticeDisable(PORTS_MODULE_ID index)
{
     PORTS_ChangeNoticeDisable_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsChangeNotice(PORTS_MODULE_ID index)
{
     return PORTS_ExistsChangeNotice_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_PinChangeNoticeEnable(PORTS_MODULE_ID index, PORTS_CHANGE_NOTICE_PIN pinNum)
{
     PORTS_PinChangeNoticeEnable_Unsupported(index, pinNum);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_PinChangeNoticeDisable(PORTS_MODULE_ID index, PORTS_CHANGE_NOTICE_PIN pinNum)
{
     PORTS_PinChangeNoticeDisable_Unsupported(index, pinNum);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_CnPinsEnable(PORTS_MODULE_ID index, PORTS_CN_PIN cnPins)
{
     PORTS_CnPinsEnable_Unsupported(index, cnPins);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_CnPinsDisable(PORTS_MODULE_ID index, PORTS_CN_PIN cnPins)
{
     PORTS_CnPinsDisable_Unsupported(index, cnPins);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsPinChangeNotice(PORTS_MODULE_ID index)
{
     return PORTS_ExistsPinChangeNotice_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_ChangeNoticeInIdleEnable(PORTS_MODULE_ID index)
{
     PORTS_ChangeNoticeInIdleEnable_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_ChangeNoticeInIdleDisable(PORTS_MODULE_ID index)
{
     PORTS_ChangeNoticeInIdleDisable_Unsupported(index);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsChangeNoticeInIdle(PORTS_MODULE_ID index)
{
     return PORTS_ExistsChangeNoticeInIdle_Unsupported(index);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_ChangeNoticePullUpEnable(PORTS_MODULE_ID index, PORTS_CHANGE_NOTICE_PIN pinNum)
{
     PORTS_ChangeNoticePullUpEnable_Unsupported(index, pinNum);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_ChangeNoticePullUpDisable(PORTS_MODULE_ID index, PORTS_CHANGE_NOTICE_PIN pinNum)
{
     PORTS_ChangeNoticePullUpDisable_Unsupported(index, pinNum);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_CnPinsPullUpEnable(PORTS_MODULE_ID index, PORTS_CN_PIN cnPins)
{
     PORTS_CnPinsPullUpEnable_Unsupported(index, cnPins);
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PORTS_CnPinsPullUpDisable(PORTS_MODULE_ID index, PORTS_CN_PIN cnPins)
{
     PORTS_CnPinsPullUpDisable_Unsupported(index, cnPins);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsChangeNoticePullUp(PORTS_MODULE_ID index)
{
     return PORTS_ExistsChangeNoticePullUp_Unsupported(index);
}

PLIB_INLINE_API void PLIB_PORTS_PinModePerPortSelect(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos, PORTS_PIN_MODE mode)
{
     PORTS_PinModePerPortSelect_Default(index, channel, bitPos, mode);
}

PLIB_INLINE_API void PLIB_PORTS_ChannelModeSelect(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK modeMask, PORTS_PIN_MODE mode)
{
     PORTS_ChannelModeSelect_Default(index, channel, modeMask, mode);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsPinModePerPort(PORTS_MODULE_ID index)
{
     return PORTS_ExistsPinModePerPort_Default(index);
}

PLIB_INLINE_API void PLIB_PORTS_ChangeNoticePullDownPerPortEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     PORTS_ChangeNoticePullDownPerPortEnable_Default(index, channel, bitPos);
}

PLIB_INLINE_API void PLIB_PORTS_ChangeNoticePullDownPerPortDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     PORTS_ChangeNoticePullDownPerPortDisable_Default(index, channel, bitPos);
}

PLIB_INLINE_API void PLIB_PORTS_ChannelChangeNoticePullDownEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK mask)
{
     PORTS_ChannelChangeNoticePullDownEnable_Default(index, channel, mask);
}

PLIB_INLINE_API void PLIB_PORTS_ChannelChangeNoticePullDownDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK mask)
{
     PORTS_ChannelChangeNoticePullDownDisable_Default(index, channel, mask);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsChangeNoticePullDownPerPort(PORTS_MODULE_ID index)
{
     return PORTS_ExistsChangeNoticePullDownPerPort_Default(index);
}

PLIB_INLINE_API void PLIB_PORTS_ChangeNoticePullUpPerPortEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     PORTS_ChangeNoticePullUpPerPortEnable_Default(index, channel, bitPos);
}

PLIB_INLINE_API void PLIB_PORTS_ChangeNoticePullUpPerPortDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     PORTS_ChangeNoticePullUpPerPortDisable_Default(index, channel, bitPos);
}

PLIB_INLINE_API void PLIB_PORTS_ChannelChangeNoticePullUpEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK mask)
{
     PORTS_ChannelChangeNoticePullUpEnable_Default(index, channel, mask);
}

PLIB_INLINE_API void PLIB_PORTS_ChannelChangeNoticePullUpDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK mask)
{
     PORTS_ChannelChangeNoticePullUpDisable_Default(index, channel, mask);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsChangeNoticePullUpPerPort(PORTS_MODULE_ID index)
{
     return PORTS_ExistsChangeNoticePullUpPerPort_Default(index);
}

PLIB_INLINE_API void PLIB_PORTS_PinChangeNoticePerPortEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     PORTS_PinChangeNoticePerPortEnable_Default(index, channel, bitPos);
}

PLIB_INLINE_API void PLIB_PORTS_PinChangeNoticePerPortDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     PORTS_PinChangeNoticePerPortDisable_Default(index, channel, bitPos);
}

PLIB_INLINE_API void PLIB_PORTS_ChannelChangeNoticeEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK mask)
{
     PORTS_ChannelChangeNoticeEnable_Default(index, channel, mask);
}

PLIB_INLINE_API void PLIB_PORTS_ChannelChangeNoticeDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK mask)
{
     PORTS_ChannelChangeNoticeDisable_Default(index, channel, mask);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsPinChangeNoticePerPort(PORTS_MODULE_ID index)
{
     return PORTS_ExistsPinChangeNoticePerPort_Default(index);
}

PLIB_INLINE_API void PLIB_PORTS_ChangeNoticePerPortTurnOn(PORTS_MODULE_ID index, PORTS_CHANNEL channel)
{
     PORTS_ChangeNoticePerPortTurnOn_Default(index, channel);
}

PLIB_INLINE_API void PLIB_PORTS_ChangeNoticePerPortTurnOff(PORTS_MODULE_ID index, PORTS_CHANNEL channel)
{
     PORTS_ChangeNoticePerPortTurnOff_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsChangeNoticePerPortTurnOn(PORTS_MODULE_ID index)
{
     return PORTS_ExistsChangeNoticePerPortTurnOn_Default(index);
}

PLIB_INLINE_API void PLIB_PORTS_ChangeNoticeInIdlePerPortEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel)
{
     PORTS_ChangeNoticeInIdlePerPortEnable_Default(index, channel);
}

PLIB_INLINE_API void PLIB_PORTS_ChangeNoticeInIdlePerPortDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel)
{
     PORTS_ChangeNoticeInIdlePerPortDisable_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsChangeNoticePerPortInIdle(PORTS_MODULE_ID index)
{
     return PORTS_ExistsChangeNoticePerPortInIdle_Default(index);
}

PLIB_INLINE_API bool PLIB_PORTS_ChangeNoticePerPortHasOccurred(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     return PORTS_ChangeNoticePerPortHasOccurred_Default(index, channel, bitPos);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsChangeNoticePerPortStatus(PORTS_MODULE_ID index)
{
     return PORTS_ExistsChangeNoticePerPortStatus_Default(index);
}

PLIB_INLINE_API void PLIB_PORTS_ChannelSlewRateSelect(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK channelMask, PORTS_PIN_SLEW_RATE slewRate)
{
     PORTS_ChannelSlewRateSelect_Default(index, channel, channelMask, slewRate);
}

PLIB_INLINE_API PORTS_PIN_SLEW_RATE PLIB_PORTS_PinSlewRateGet(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     return PORTS_PinSlewRateGet_Default(index, channel, bitPos);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsSlewRateControl(PORTS_MODULE_ID index)
{
     return PORTS_ExistsSlewRateControl_Default(index);
}

PLIB_INLINE_API void PLIB_PORTS_ChannelChangeNoticeMethodSelect(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_CHANGE_NOTICE_METHOD changeNoticeMethod)
{
     PORTS_ChannelChangeNoticeMethodSelect_Default(index, channel, changeNoticeMethod);
}

PLIB_INLINE_API PORTS_CHANGE_NOTICE_METHOD PLIB_PORTS_ChannelChangeNoticeMethodGet(PORTS_MODULE_ID index, PORTS_CHANNEL channel)
{
     return PORTS_ChannelChangeNoticeMethodGet_Default(index, channel);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsChannelChangeNoticeMethod(PORTS_MODULE_ID index)
{
     return PORTS_ExistsChannelChangeNoticeMethod_Default(index);
}

PLIB_INLINE_API void PLIB_PORTS_ChannelChangeNoticeEdgeEnable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK edgeRisingMask, PORTS_DATA_MASK edgeFallingMask)
{
     PORTS_ChannelChangeNoticeEdgeEnable_Default(index, channel, edgeRisingMask, edgeFallingMask);
}

PLIB_INLINE_API void PLIB_PORTS_ChannelChangeNoticeEdgeDisable(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_DATA_MASK edgeRisingMask, PORTS_DATA_MASK edgeFallingMask)
{
     PORTS_ChannelChangeNoticeEdgeDisable_Default(index, channel, edgeRisingMask, edgeFallingMask);
}

PLIB_INLINE_API bool PLIB_PORTS_PinChangeNoticeEdgeIsEnabled(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos, PORTS_CHANGE_NOTICE_EDGE cnEdgeType)
{
     return PORTS_PinChangeNoticeEdgeIsEnabled_Default(index, channel, bitPos, cnEdgeType);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsChangeNoticeEdgeControl(PORTS_MODULE_ID index)
{
     return PORTS_ExistsChangeNoticeEdgeControl_Default(index);
}

PLIB_INLINE_API bool PLIB_PORTS_PinChangeNoticeEdgeHasOccurred(PORTS_MODULE_ID index, PORTS_CHANNEL channel, PORTS_BIT_POS bitPos)
{
     return PORTS_PinChangeNoticeEdgeHasOccurred_Default(index, channel, bitPos);
}

PLIB_INLINE_API bool PLIB_PORTS_ExistsChangeNoticeEdgeStatus(PORTS_MODULE_ID index)
{
     return PORTS_ExistsChangeNoticeEdgeStatus_Default(index);
}

#endif
