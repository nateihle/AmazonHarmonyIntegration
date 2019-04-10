/* Created by plibgen $Revision: 1.31 $ */

#ifndef _INT_P32WK2057GPD132_H
#define _INT_P32WK2057GPD132_H

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

    INT_ID_0 = 0,
    INT_NUMBER_OF_MODULES = 1

} INT_MODULE_ID;

typedef enum {

    INT_EXTERNAL_INT_SOURCE0 = 0x01,
    INT_EXTERNAL_INT_SOURCE1 = 0x02,
    INT_EXTERNAL_INT_SOURCE2 = 0x04,
    INT_EXTERNAL_INT_SOURCE3 = 0x08,
    INT_EXTERNAL_INT_SOURCE4 = 0x10

} INT_EXTERNAL_SOURCES;

typedef enum {

    INT_DISABLE_INTERRUPT = 0,
    INT_PRIORITY_LEVEL1 = 1,
    INT_PRIORITY_LEVEL2 = 2,
    INT_PRIORITY_LEVEL3 = 3,
    INT_PRIORITY_LEVEL4 = 4,
    INT_PRIORITY_LEVEL5 = 5,
    INT_PRIORITY_LEVEL6 = 6,
    INT_PRIORITY_LEVEL7 = 7

} INT_PRIORITY_LEVEL;

typedef enum {

    INT_SUBPRIORITY_LEVEL0 = 0x00,
    INT_SUBPRIORITY_LEVEL1 = 0x01,
    INT_SUBPRIORITY_LEVEL2 = 0x02,
    INT_SUBPRIORITY_LEVEL3 = 0x03

} INT_SUBPRIORITY_LEVEL;

typedef enum {

    INT_SOURCE_TIMER_CORE = 0,
    INT_SOURCE_SOFTWARE_0 = 1,
    INT_SOURCE_SOFTWARE_1 = 2,
    INT_SOURCE_EXTERNAL_0 = 3,
    INT_SOURCE_TIMER_1 = 4,
    INT_SOURCE_INPUT_CAPTURE_1_ERROR = 5,
    INT_SOURCE_INPUT_CAPTURE_1 = 6,
    INT_SOURCE_OUTPUT_COMPARE_1 = 7,
    INT_SOURCE_EXTERNAL_1 = 8,
    INT_SOURCE_TIMER_2 = 9,
    INT_SOURCE_INPUT_CAPTURE_2_ERROR = 10,
    INT_SOURCE_INPUT_CAPTURE_2 = 11,
    INT_SOURCE_OUTPUT_COMPARE_2 = 12,
    INT_SOURCE_EXTERNAL_2 = 13,
    INT_SOURCE_TIMER_3 = 14,
    INT_SOURCE_INPUT_CAPTURE_3_ERROR = 15,
    INT_SOURCE_INPUT_CAPTURE_3 = 16,
    INT_SOURCE_OUTPUT_COMPARE_3 = 17,
    INT_SOURCE_EXTERNAL_3 = 18,
    INT_SOURCE_TIMER_4 = 19,
    INT_SOURCE_INPUT_CAPTURE_4_ERROR = 20,
    INT_SOURCE_INPUT_CAPTURE_4 = 21,
    INT_SOURCE_OUTPUT_COMPARE_4 = 22,
    INT_SOURCE_EXTERNAL_4 = 23,
    INT_SOURCE_TIMER_5 = 24,
    INT_SOURCE_TIMER_6 = 76,
    INT_SOURCE_TIMER_7 = 80,
    INT_SOURCE_ADC_1 = 92,
    INT_SOURCE_ADC_1_DC1 = 94,
    INT_SOURCE_ADC_1_DC2 = 95,
    INT_SOURCE_ADC_1_DF1 = 96,
    INT_SOURCE_ADC_1_DF2 = 97,
    INT_SOURCE_ADC_FAULT = 100,
    INT_SOURCE_ADC_1_DATA0 = 106,
    INT_SOURCE_ADC_1_DATA1 = 107,
    INT_SOURCE_ADC_1_DATA2 = 108,
    INT_SOURCE_ADC_1_DATA3 = 109,
    INT_SOURCE_ADC_1_DATA4 = 110,
    INT_SOURCE_ADC_1_DATA5 = 111,
    INT_SOURCE_ADC_1_DATA6 = 112,
    INT_SOURCE_ADC_1_DATA7 = 113,
    INT_SOURCE_ADC_1_DATA8 = 114,
    INT_SOURCE_ADC_1_DATA9 = 115,
    INT_SOURCE_ADC_1_DATA10 = 116,
    INT_SOURCE_ADC_1_DATA11 = 117,
    INT_SOURCE_ADC_1_DATA12 = 118,
    INT_SOURCE_ADC_1_DATA13 = 119,
    INT_SOURCE_ADC_1_DATA14 = 120,
    INT_SOURCE_ADC_1_DATA15 = 121,
    INT_SOURCE_ADC_1_DATA16 = 122,
    INT_SOURCE_ADC_1_DATA17 = 123,
    INT_SOURCE_ADC_1_DATA18 = 124,
    INT_SOURCE_ADC_1_DATA19 = 125,
    INT_SOURCE_CORE_PERF_COUNT = 162,
    INT_SOURCE_FAST_DEBUG = 163,
    INT_SOURCE_SPI_1_ERROR = 35,
    INT_SOURCE_SPI_1_RECEIVE = 36,
    INT_SOURCE_SPI_1_TRANSMIT = 37,
    INT_SOURCE_USART_1_ERROR = 38,
    INT_SOURCE_USART_1_RECEIVE = 39,
    INT_SOURCE_USART_1_TRANSMIT = 40,
    INT_SOURCE_I2C_1_BUS = 41,
    INT_SOURCE_I2C_1_SLAVE = 42,
    INT_SOURCE_I2C_1_MASTER = 43,
    INT_SOURCE_CHANGE_NOTICE_A = 44,
    INT_SOURCE_CHANGE_NOTICE_B = 45,
    INT_SOURCE_CHANGE_NOTICE_C = 46,
    INT_SOURCE_CHANGE_NOTICE_K = 47,
    INT_SOURCE_USB_1 = 34,
    INT_SOURCE_DMA_0 = 68,
    INT_SOURCE_DMA_1 = 69,
    INT_SOURCE_DMA_2 = 70,
    INT_SOURCE_DMA_3 = 71,
    INT_SOURCE_DMA_4 = 72,
    INT_SOURCE_DMA_5 = 73,
    INT_SOURCE_DMA_6 = 74,
    INT_SOURCE_DMA_7 = 75,
    INT_SOURCE_SPI_2_ERROR = 53,
    INT_SOURCE_SPI_2_RECEIVE = 54,
    INT_SOURCE_SPI_2_TRANSMIT = 55,
    INT_SOURCE_USART_2_ERROR = 56,
    INT_SOURCE_USART_2_RECEIVE = 57,
    INT_SOURCE_USART_2_TRANSMIT = 58,
    INT_SOURCE_I2C_2_BUS = 59,
    INT_SOURCE_I2C_2_SLAVE = 60,
    INT_SOURCE_I2C_2_MASTER = 61,
    INT_SOURCE_CAN_1 = 142,
    INT_SOURCE_RTCC = 32,
    INT_SOURCE_SQI1 = 150,
    INT_SOURCE_ADC_END_OF_SCAN = 101,
    INT_SOURCE_ADC_ANALOG_CIRCUIT_READY = 102,
    INT_SOURCE_ADC_UPDATE_READY = 103,
    INT_SOURCE_SPI_0_ERROR = 25,
    INT_SOURCE_SPI_0_RECEIVE = 26,
    INT_SOURCE_SPI_0_TRANSMIT = 27,
    INT_SOURCE_CTR1_EVENT = 28,
    INT_SOURCE_CTR1_TRG = 29,
    INT_SOURCE_OTP_PGM_COMPLETE = 30,
    INT_SOURCE_OTP_PGM_ERROR = 31,
    INT_SOURCE_RFMAC = 84,
    INT_SOURCE_RFSNIF = 85,
    INT_SOURCE_RFTM0 = 86,
    INT_SOURCE_RFTM1 = 87,
    INT_SOURCE_RFTM2 = 88,
    INT_SOURCE_RFTM3 = 89,
    INT_SOURCE_RFARB = 90,
    INT_SOURCE_RFWCOE = 91,
    INT_SOURCE_PTG_STEP = 153,
    INT_SOURCE_PTG_WDT = 152,
    INT_SOURCE_PTG_TR0 = 154,
    INT_SOURCE_PTG_TR1 = 155,
    INT_SOURCE_PTG_TR2 = 156,
    INT_SOURCE_PTG_TR3 = 157,
    INT_SOURCE_ADC_DMA = 104

} INT_SOURCE;

typedef enum {

    INT_VECTOR_CT = _CORE_TIMER_VECTOR,
    INT_VECTOR_CS0 = _CORE_SOFTWARE_0_VECTOR,
    INT_VECTOR_CS1 = _CORE_SOFTWARE_1_VECTOR,
    INT_VECTOR_INT0 = _EXTERNAL_0_VECTOR,
    INT_VECTOR_T1 = _TIMER_1_VECTOR,
    INT_VECTOR_IC1 = _INPUT_CAPTURE_1_VECTOR,
    INT_VECTOR_IC1_ERROR = _INPUT_CAPTURE_1_ERROR_VECTOR,
    INT_VECTOR_OC1 = _OUTPUT_COMPARE_1_VECTOR,
    INT_VECTOR_INT1 = _EXTERNAL_1_VECTOR,
    INT_VECTOR_T2 = _TIMER_2_VECTOR,
    INT_VECTOR_IC2 = _INPUT_CAPTURE_2_VECTOR,
    INT_VECTOR_IC2_ERROR = _INPUT_CAPTURE_2_ERROR_VECTOR,
    INT_VECTOR_OC2 = _OUTPUT_COMPARE_2_VECTOR,
    INT_VECTOR_INT2 = _EXTERNAL_2_VECTOR,
    INT_VECTOR_T3 = _TIMER_3_VECTOR,
    INT_VECTOR_IC3 = _INPUT_CAPTURE_3_VECTOR,
    INT_VECTOR_IC3_ERROR = _INPUT_CAPTURE_3_ERROR_VECTOR,
    INT_VECTOR_OC3 = _OUTPUT_COMPARE_3_VECTOR,
    INT_VECTOR_INT3 = _EXTERNAL_3_VECTOR,
    INT_VECTOR_T4 = _TIMER_4_VECTOR,
    INT_VECTOR_IC4 = _INPUT_CAPTURE_4_VECTOR,
    INT_VECTOR_IC4_ERROR = _INPUT_CAPTURE_4_ERROR_VECTOR,
    INT_VECTOR_OC4 = _OUTPUT_COMPARE_4_VECTOR,
    INT_VECTOR_INT4 = _EXTERNAL_4_VECTOR,
    INT_VECTOR_T5 = _TIMER_5_VECTOR,
    INT_VECTOR_T6 = _TIMER_6_VECTOR,
    INT_VECTOR_T7 = _TIMER_7_VECTOR,
    INT_VECTOR_ADC1 = _ADC_VECTOR,
    INT_VECTOR_ADC1_DC1 = _ADC_DC1_VECTOR,
    INT_VECTOR_ADC1_DC2 = _ADC_DC2_VECTOR,
    INT_VECTOR_ADC1_DF1 = _ADC_DF1_VECTOR,
    INT_VECTOR_ADC1_DF2 = _ADC_DF2_VECTOR,
    INT_VECTOR_ADC_FAULT = _ADC_FAULT_VECTOR,
    INT_VECTOR_ADC1_DATA0 = _ADC_DATA0_VECTOR,
    INT_VECTOR_ADC1_DATA1 = _ADC_DATA1_VECTOR,
    INT_VECTOR_ADC1_DATA2 = _ADC_DATA2_VECTOR,
    INT_VECTOR_ADC1_DATA3 = _ADC_DATA3_VECTOR,
    INT_VECTOR_ADC1_DATA4 = _ADC_DATA4_VECTOR,
    INT_VECTOR_ADC1_DATA5 = _ADC_DATA5_VECTOR,
    INT_VECTOR_ADC1_DATA6 = _ADC_DATA6_VECTOR,
    INT_VECTOR_ADC1_DATA7 = _ADC_DATA7_VECTOR,
    INT_VECTOR_ADC1_DATA8 = _ADC_DATA8_VECTOR,
    INT_VECTOR_ADC1_DATA9 = _ADC_DATA9_VECTOR,
    INT_VECTOR_ADC1_DATA10 = _ADC_DATA10_VECTOR,
    INT_VECTOR_ADC1_DATA11 = _ADC_DATA11_VECTOR,
    INT_VECTOR_ADC1_DATA12 = _ADC_DATA12_VECTOR,
    INT_VECTOR_ADC1_DATA13 = _ADC_DATA13_VECTOR,
    INT_VECTOR_ADC1_DATA14 = _ADC_DATA14_VECTOR,
    INT_VECTOR_ADC1_DATA15 = _ADC_DATA15_VECTOR,
    INT_VECTOR_ADC1_DATA16 = _ADC_DATA16_VECTOR,
    INT_VECTOR_ADC1_DATA17 = _ADC_DATA17_VECTOR,
    INT_VECTOR_ADC1_DATA18 = _ADC_DATA18_VECTOR,
    INT_VECTOR_ADC1_DATA19 = _ADC_DATA19_VECTOR,
    INT_VECTOR_CORE_PERF_COUNT = _MPUPC_VECTOR,
    INT_VECTOR_CORE_FAST_DEBUG_CHANNEL = _FDC_VECTOR,
    INT_VECTOR_SPI1_FAULT = _SPI1_FAULT_VECTOR,
    INT_VECTOR_SPI1_RX = _SPI1_RX_VECTOR,
    INT_VECTOR_SPI1_TX = _SPI1_TX_VECTOR,
    INT_VECTOR_UART1_FAULT = _UART1_FAULT_VECTOR,
    INT_VECTOR_UART1_RX = _UART1_RX_VECTOR,
    INT_VECTOR_UART1_TX = _UART1_TX_VECTOR,
    INT_VECTOR_I2C1_BUS = _I2C1_BUS_VECTOR,
    INT_VECTOR_I2C1_SLAVE = _I2C1_SLAVE_VECTOR,
    INT_VECTOR_I2C1_MASTER = _I2C1_MASTER_VECTOR,
    INT_VECTOR_SPI2_FAULT = _SPI2_FAULT_VECTOR,
    INT_VECTOR_SPI2_RX = _SPI2_RX_VECTOR,
    INT_VECTOR_SPI2_TX = _SPI2_TX_VECTOR,
    INT_VECTOR_UART2_FAULT = _UART2_FAULT_VECTOR,
    INT_VECTOR_UART2_RX = _UART2_RX_VECTOR,
    INT_VECTOR_UART2_TX = _UART2_TX_VECTOR,
    INT_VECTOR_I2C2_BUS = _I2C2_BUS_VECTOR,
    INT_VECTOR_I2C2_SLAVE = _I2C2_SLAVE_VECTOR,
    INT_VECTOR_I2C2_MASTER = _I2C2_MASTER_VECTOR,
    INT_VECTOR_CHANGE_NOTICE_A = _CHANGE_NOTICE_A_VECTOR,
    INT_VECTOR_CHANGE_NOTICE_B = _CHANGE_NOTICE_B_VECTOR,
    INT_VECTOR_CHANGE_NOTICE_C = _CHANGE_NOTICE_C_VECTOR,
    INT_VECTOR_CHANGE_NOTICE_K = _CHANGE_NOTICE_K_VECTOR,
    INT_VECTOR_USB1 = _USB_VECTOR,
    INT_VECTOR_RTCC = _RTCC_VECTOR,
    INT_VECTOR_SQI1 = _SQI1_VECTOR,
    INT_VECTOR_DMA0 = _DMA0_VECTOR,
    INT_VECTOR_DMA1 = _DMA1_VECTOR,
    INT_VECTOR_DMA2 = _DMA2_VECTOR,
    INT_VECTOR_DMA3 = _DMA3_VECTOR,
    INT_VECTOR_DMA4 = _DMA4_VECTOR,
    INT_VECTOR_DMA5 = _DMA5_VECTOR,
    INT_VECTOR_DMA6 = _DMA6_VECTOR,
    INT_VECTOR_DMA7 = _DMA7_VECTOR,
    INT_VECTOR_CAN1 = _CAN1_VECTOR,
    INT_VECTOR_ADC_END_OF_SCAN = _ADC_EOS_VECTOR,
    INT_VECTOR_ADC_ANALOG_CIRCUIT_READY = _ADC_ARDY_VECTOR,
    INT_VECTOR_ADC_UPDATE_READY = _ADC_URDY_VECTOR,
    INT_VECTOR_SPI0_FAULT = _SPI0_FAULT_VECTOR,
    INT_VECTOR_SPI0_RX = _SPI0_RX_VECTOR,
    INT_VECTOR_SPI0_TX = _SPI0_TX_VECTOR,
    INT_VECTOR_CTR1_EVENT = _CTR1_EVENT_VECTOR,
    INT_VECTOR_CTR1_TRG = _CTR1_TRG_VECTOR,
    INT_VECTOR_OTP_PGM_EVENT = _OTP_DONE_VECTOR,
    INT_VECTOR_OTP_PGM_ERROR = _OTP_FAULT_VECTOR,
    INT_VECTOR_RFMAC = _RFMAC_VECTOR,
    INT_VECTOR_RFSNIF = _RFSNIF_VECTOR,
    INT_VECTOR_RFTM0 = _RFTM0_VECTOR,
    INT_VECTOR_RFTM1 = _RFTM1_VECTOR,
    INT_VECTOR_RFTM2 = _RFTM2_VECTOR,
    INT_VECTOR_RFTM3 = _RFTM3_VECTOR,
    INT_VECTOR_RFWCOE = _RFWCOE_VECTOR,
    INT_VECTOR_RFARB = _RFARB_VECTOR,
    INT_VECTOR_PTG_STEP = _PTG0_STEP_VECTOR,
    INT_VECTOR_PTG_WDT = _PTG0_WDT_VECTOR,
    INT_VECTOR_PTG_TR0 = _PTG0_TRG0_VECTOR,
    INT_VECTOR_PTG_TR1 = _PTG0_TRG1_VECTOR,
    INT_VECTOR_PTG_TR2 = _PTG0_TRG2_VECTOR,
    INT_VECTOR_PTG_TR3 = _PTG0_TRG3_VECTOR,
    INT_VECTOR_ADC_DMA = _ADC_DMA_VECTOR

} INT_VECTOR;

typedef enum {

    INT_VECTOR_SPACING_0 = 0x00,
    INT_VECTOR_SPACING_8 = 0x01,
    INT_VECTOR_SPACING_16 = 0x02,
    INT_VECTOR_SPACING_32 = 0x04,
    INT_VECTOR_SPACING_64 = 0x08,
    INT_VECTOR_SPACING_128 = 0x10,
    INT_VECTOR_SPACING_256 = 0x20,
    INT_VECTOR_SPACING_512 = 0x40

} INT_VECTOR_SPACING;

typedef enum {

    INT_SHADOW_REGISTER_0 = 0x00,
    INT_SHADOW_REGISTER_1 = 0x01,
    INT_SHADOW_REGISTER_2 = 0x02,
    INT_SHADOW_REGISTER_3 = 0x03,
    INT_SHADOW_REGISTER_4 = 0x04,
    INT_SHADOW_REGISTER_5 = 0x05,
    INT_SHADOW_REGISTER_6 = 0x06,
    INT_SHADOW_REGISTER_7 = 0x07

} INT_SHADOW_REGISTER;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE
#include "../templates/int_SingleVectorShadowSet_Default.h"
#include "../templates/int_VectorSelect_Default.h"
#include "../templates/int_ProximityTimerEnable_Default.h"
#include "../templates/int_ProximityTimerControl_Default.h"
#include "../templates/int_ExternalINTEdgeSelect_Default.h"
#include "../templates/int_INTCPUPriority_Default.h"
#include "../templates/int_INTCPUVector_Default.h"
#include "../templates/int_SourceFlag_Default.h"
#include "../templates/int_SourceControl_Default.h"
#include "../templates/int_VectorPriority_Default.h"
#include "../templates/int_CPUCurrentPriorityLevel_Default.h"
#include "../templates/int_EnableControl_PIC32.h"
#include "../templates/int_ShadowRegisterAssign_Default.h"
#include "../templates/int_VariableOffset_Default.h"
#include "../templates/int_SoftwareNMI_Default.h"

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_INT_ExistsSingleVectorShadowSet(INT_MODULE_ID index)
{
     return INT_ExistsSingleVectorShadowSet_Default(index);
}

PLIB_INLINE_API void PLIB_INT_SingleVectorShadowSetDisable(INT_MODULE_ID index)
{
     INT_SingleVectorShadowSetDisable_Default(index);
}

PLIB_INLINE_API void PLIB_INT_SingleVectorShadowSetEnable(INT_MODULE_ID index)
{
     INT_SingleVectorShadowSetEnable_Default(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsVectorSelect(INT_MODULE_ID index)
{
     return INT_ExistsVectorSelect_Default(index);
}

PLIB_INLINE_API void PLIB_INT_MultiVectorSelect(INT_MODULE_ID index)
{
     INT_MultiVectorSelect_Default(index);
}

PLIB_INLINE_API void PLIB_INT_SingleVectorSelect(INT_MODULE_ID index)
{
     INT_SingleVectorSelect_Default(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsProximityTimerEnable(INT_MODULE_ID index)
{
     return INT_ExistsProximityTimerEnable_Default(index);
}

PLIB_INLINE_API void PLIB_INT_ProximityTimerEnable(INT_MODULE_ID index, INT_PRIORITY_LEVEL priority)
{
     INT_ProximityTimerEnable_Default(index, priority);
}

PLIB_INLINE_API void PLIB_INT_ProximityTimerDisable(INT_MODULE_ID index)
{
     INT_ProximityTimerDisable_Default(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsProximityTimerControl(INT_MODULE_ID index)
{
     return INT_ExistsProximityTimerControl_Default(index);
}

PLIB_INLINE_API void PLIB_INT_ProximityTimerSet(INT_MODULE_ID index, uint32_t timerreloadvalue)
{
     INT_ProximityTimerSet_Default(index, timerreloadvalue);
}

PLIB_INLINE_API uint32_t PLIB_INT_ProximityTimerGet(INT_MODULE_ID index)
{
     return INT_ProximityTimerGet_Default(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsExternalINTEdgeSelect(INT_MODULE_ID index)
{
     return INT_ExistsExternalINTEdgeSelect_Default(index);
}

PLIB_INLINE_API void PLIB_INT_ExternalRisingEdgeSelect(INT_MODULE_ID index, INT_EXTERNAL_SOURCES source)
{
     INT_ExternalRisingEdgeSelect_Default(index, source);
}

PLIB_INLINE_API void PLIB_INT_ExternalFallingEdgeSelect(INT_MODULE_ID index, INT_EXTERNAL_SOURCES source)
{
     INT_ExternalFallingEdgeSelect_Default(index, source);
}

PLIB_INLINE_API bool PLIB_INT_ExistsINTCPUPriority(INT_MODULE_ID index)
{
     return INT_ExistsINTCPUPriority_Default(index);
}

PLIB_INLINE_API INT_PRIORITY_LEVEL PLIB_INT_PriorityGet(INT_MODULE_ID index)
{
     return INT_PriorityGet_Default(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsINTCPUVector(INT_MODULE_ID index)
{
     return INT_ExistsINTCPUVector_Default(index);
}

PLIB_INLINE_API INT_VECTOR PLIB_INT_VectorGet(INT_MODULE_ID index)
{
     return INT_VectorGet_Default(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsSourceFlag(INT_MODULE_ID index)
{
     return INT_ExistsSourceFlag_Default(index);
}

PLIB_INLINE_API bool PLIB_INT_SourceFlagGet(INT_MODULE_ID index, INT_SOURCE source)
{
     return INT_SourceFlagGet_Default(index, source);
}

PLIB_INLINE_API void PLIB_INT_SourceFlagSet(INT_MODULE_ID index, INT_SOURCE source)
{
     INT_SourceFlagSet_Default(index, source);
}

PLIB_INLINE_API void PLIB_INT_SourceFlagClear(INT_MODULE_ID index, INT_SOURCE source)
{
     INT_SourceFlagClear_Default(index, source);
}

PLIB_INLINE_API bool PLIB_INT_ExistsSourceControl(INT_MODULE_ID index)
{
     return INT_ExistsSourceControl_Default(index);
}

PLIB_INLINE_API void PLIB_INT_SourceEnable(INT_MODULE_ID index, INT_SOURCE source)
{
     INT_SourceEnable_Default(index, source);
}

PLIB_INLINE_API void PLIB_INT_SourceDisable(INT_MODULE_ID index, INT_SOURCE source)
{
     INT_SourceDisable_Default(index, source);
}

PLIB_INLINE_API bool PLIB_INT_SourceIsEnabled(INT_MODULE_ID index, INT_SOURCE source)
{
     return INT_SourceIsEnabled_Default(index, source);
}

PLIB_INLINE_API bool PLIB_INT_ExistsVectorPriority(INT_MODULE_ID index)
{
     return INT_ExistsVectorPriority_Default(index);
}

PLIB_INLINE_API void PLIB_INT_VectorPrioritySet(INT_MODULE_ID index, INT_VECTOR vector, INT_PRIORITY_LEVEL priority)
{
     INT_VectorPrioritySet_Default(index, vector, priority);
}

PLIB_INLINE_API INT_PRIORITY_LEVEL PLIB_INT_VectorPriorityGet(INT_MODULE_ID index, INT_VECTOR vector)
{
     return INT_VectorPriorityGet_Default(index, vector);
}

PLIB_INLINE_API void PLIB_INT_VectorSubPrioritySet(INT_MODULE_ID index, INT_VECTOR vector, INT_SUBPRIORITY_LEVEL subPriority)
{
     INT_VectorSubPrioritySet_Default(index, vector, subPriority);
}

PLIB_INLINE_API INT_SUBPRIORITY_LEVEL PLIB_INT_VectorSubPriorityGet(INT_MODULE_ID index, INT_VECTOR vector)
{
     return INT_VectorSubPriorityGet_Default(index, vector);
}

PLIB_INLINE_API bool PLIB_INT_ExistsCPUCurrentPriorityLevel(INT_MODULE_ID index)
{
     return INT_ExistsCPUCurrentPriorityLevel_Default(index);
}

PLIB_INLINE_API INT_PRIORITY_LEVEL PLIB_INT_CPUCurrentPriorityLevelGet(INT_MODULE_ID index)
{
     return INT_CPUCurrentPriorityLevelGet_Default(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsEnableControl(INT_MODULE_ID index)
{
     return INT_ExistsEnableControl_PIC32(index);
}

PLIB_INLINE_API void PLIB_INT_Enable(INT_MODULE_ID index)
{
     INT_Enable_PIC32(index);
}

PLIB_INLINE_API void PLIB_INT_Disable(INT_MODULE_ID index)
{
     INT_Disable_PIC32(index);
}

PLIB_INLINE_API bool PLIB_INT_IsEnabled(INT_MODULE_ID index)
{
     return INT_IsEnabled_PIC32(index);
}

PLIB_INLINE_API void PLIB_INT_SetState(INT_MODULE_ID index, INT_STATE_GLOBAL interrupt_state)
{
     INT_SetState_PIC32(index, interrupt_state);
}

PLIB_INLINE_API INT_STATE_GLOBAL PLIB_INT_GetStateAndDisable(INT_MODULE_ID index)
{
     return INT_GetStateAndDisable_PIC32(index);
}

PLIB_INLINE_API bool PLIB_INT_ExistsShadowRegisterAssign(INT_MODULE_ID index)
{
     return INT_ExistsShadowRegisterAssign_Default(index);
}

PLIB_INLINE_API void PLIB_INT_ShadowRegisterAssign(INT_MODULE_ID index, INT_PRIORITY_LEVEL priority, INT_SHADOW_REGISTER shadowRegister)
{
     INT_ShadowRegisterAssign_Default(index, priority, shadowRegister);
}

PLIB_INLINE_API INT_SHADOW_REGISTER PLIB_INT_ShadowRegisterGet(INT_MODULE_ID index, INT_PRIORITY_LEVEL priority)
{
     return INT_ShadowRegisterGet_Default(index, priority);
}

PLIB_INLINE_API bool PLIB_INT_ExistsVariableOffset(INT_MODULE_ID index)
{
     return INT_ExistsVariableOffset_Default(index);
}

PLIB_INLINE_API void PLIB_INT_VariableVectorOffsetSet(INT_MODULE_ID index, INT_VECTOR vector, uint32_t offset)
{
     INT_VariableVectorOffsetSet_Default(index, vector, offset);
}

PLIB_INLINE_API uint32_t PLIB_INT_VariableVectorOffsetGet(INT_MODULE_ID index, INT_VECTOR vector)
{
     return INT_VariableVectorOffsetGet_Default(index, vector);
}

PLIB_INLINE_API bool PLIB_INT_ExistsSoftwareNMI(INT_MODULE_ID index)
{
     return INT_ExistsSoftwareNMI_Default(index);
}

PLIB_INLINE_API void PLIB_INT_SoftwareNMITrigger(INT_MODULE_ID index)
{
     INT_SoftwareNMITrigger_Default(index);
}

#endif
