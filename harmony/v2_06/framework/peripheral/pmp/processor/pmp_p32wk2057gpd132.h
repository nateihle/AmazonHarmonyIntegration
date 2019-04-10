/* Created by plibgen $Revision: 1.31 $ */

#ifndef _PMP_P32WK2057GPD132_H
#define _PMP_P32WK2057GPD132_H

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

    PMP_NUMBER_OF_MODULES = 0

} PMP_MODULE_ID;

typedef enum {

    PMP_OPERATION_MODE_NONE

} PMP_OPERATION_MODE;

typedef enum {

    PMP_MUX_MODE_NONE

} PMP_MUX_MODE;

typedef enum {

    PMP_INCREMENT_MODE_NONE

} PMP_INCREMENT_MODE;

typedef enum {

    PMP_ADDRESS_LATCH_NONE

} PMP_ADDRESS_LATCH;

typedef enum {

    PMP_DATA_WAIT_STATES_NONE

} PMP_DATA_WAIT_STATES;

typedef enum {

    PMP_STROBE_WAIT_STATES_NONE

} PMP_STROBE_WAIT_STATES;

typedef enum {

    PMP_DATA_HOLD_STATES_NONE

} PMP_DATA_HOLD_STATES;

typedef enum {

    PMP_INTERRUPT_MODE_NONE

} PMP_INTERRUPT_MODE;

typedef enum {

    PMP_DATA_SIZE_NONE

} PMP_DATA_SIZE;

typedef enum {

    PMP_CHIP_SELECT_NONE

} PMP_CHIP_SELECT;

typedef enum {

    PMP_CHIPSELECT_FUNCTION_NONE

} PMP_CHIPSELECT_FUNCTION;

typedef enum {

    PMP_ADDRESS_PORT_NONE

} PMP_ADDRESS_PORT;

typedef enum {

    PMP_INPUT_BUFFER_TYPE_NONE

} PMP_INPUT_BUFFER_TYPE;

typedef enum {

    PMP_POLARITY_LEVEL_NONE

} PMP_POLARITY_LEVEL;

typedef enum {

    PMP_ACK_MODE_NONE

} PMP_ACK_MODE;

typedef enum {

    PMP_ADDRESS_LATCH_WAIT_STATES_NONE

} PMP_ADDRESS_LATCH_WAIT_STATES;

typedef enum {

    PMP_ADDRESS_HOLD_LATCH_WAIT_STATES_NONE

} PMP_ADDRESS_HOLD_LATCH_WAIT_STATES;

typedef enum {

    PMP_PMBE_PORT_NONE

} PMP_PMBE_PORT;

typedef enum {

    PMP_MASTER_MODE_NONE

} PMP_MASTER_MODE;

typedef enum {

    PMP_ALTERNATE_MASTER_WAIT_STATES_NONE

} PMP_ALTERNATE_MASTER_WAIT_STATES;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_PMP_ExistsStartReadControl(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_ReadCycleStart(PMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PMP_ReadCycleIsStarted(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsDualBufferControl(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_DualBufferDisable(PMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_DualBufferEnable(PMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PMP_DualBufferIsEnabled(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsEnableControl(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_Disable(PMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_Enable(PMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PMP_IsEnabled(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsStopInIdleControl(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_StopInIdleEnable(PMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_StopInIdleDisable(PMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsMUXModeSelect(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_MultiplexModeSelect(PMP_MODULE_ID index, PMP_MUX_MODE multiplexMode)
{
     
}

PLIB_INLINE_API PMP_MUX_MODE _PLIB_UNSUPPORTED PLIB_PMP_MultiplexModeGet(PMP_MODULE_ID index)
{
     return (PMP_MUX_MODE)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsBufferType(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_InputBufferTypeSelect(PMP_MODULE_ID index, PMP_INPUT_BUFFER_TYPE inputBuffer)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsWriteEnablePortControl(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_WriteEnableStrobePortEnable(PMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_WriteEnableStrobePortDisable(PMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsReadWriteStrobePortControl(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_ReadWriteStrobePortEnable(PMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_ReadWriteStrobePortDisable(PMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsChipSelectoperation(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_ChipSelectFunctionSelect(PMP_MODULE_ID index, PMP_CHIPSELECT_FUNCTION chipselfunct)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsAddressLatchPolarity(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_AddressLatchPolaritySelect(PMP_MODULE_ID index, PMP_POLARITY_LEVEL polarity)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsChipXPolarity(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_ChipSelectXPolaritySelect(PMP_MODULE_ID index, PMP_CHIP_SELECT chipSelect, PMP_POLARITY_LEVEL polarity)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsWriteEnablePolarity(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_WriteEnableStrobePolaritySelect(PMP_MODULE_ID index, PMP_POLARITY_LEVEL polarity)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsReadWritePolarity(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_ReadWriteStrobePolaritySelect(PMP_MODULE_ID index, PMP_POLARITY_LEVEL polarity)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsBusyStatus(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PMP_PortIsBusy(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsInterruptMode(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_InterruptModeSelect(PMP_MODULE_ID index, PMP_INTERRUPT_MODE interruptMode)
{
     
}

PLIB_INLINE_API PMP_INTERRUPT_MODE _PLIB_UNSUPPORTED PLIB_PMP_InterruptModeGet(PMP_MODULE_ID index)
{
     return (PMP_INTERRUPT_MODE)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsIncrementMode(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_AddressIncrementModeSelect(PMP_MODULE_ID index, PMP_INCREMENT_MODE incrementMode)
{
     
}

PLIB_INLINE_API PMP_INCREMENT_MODE _PLIB_UNSUPPORTED PLIB_PMP_AddressIncrementModeGet(PMP_MODULE_ID index)
{
     return (PMP_INCREMENT_MODE)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsDataTransferSize(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_DataSizeSelect(PMP_MODULE_ID index, PMP_DATA_SIZE size)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsOperationMode(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_OperationModeSelect(PMP_MODULE_ID index, PMP_OPERATION_MODE operationMode)
{
     
}

PLIB_INLINE_API PMP_OPERATION_MODE _PLIB_UNSUPPORTED PLIB_PMP_OperationModeGet(PMP_MODULE_ID index)
{
     return (PMP_OPERATION_MODE)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsDataSetUpWaitStates(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_WaitStatesDataSetUpSelect(PMP_MODULE_ID index, PMP_DATA_WAIT_STATES dataWaitState)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsDataStrobeWaitStates(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_WaitStatesStrobeSelect(PMP_MODULE_ID index, PMP_STROBE_WAIT_STATES strobeWaitState)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsDataHoldWaitStates(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_WaitStatesDataHoldSelect(PMP_MODULE_ID index, PMP_DATA_HOLD_STATES dataHoldState)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsChipSelectEnable(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_ChipSelectXEnable(PMP_MODULE_ID index, PMP_CHIP_SELECT chipSelect)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_ChipSelectXDisable(PMP_MODULE_ID index, PMP_CHIP_SELECT chipSelect)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsWriteChipSelectEnable(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_WriteChipSelectXEnable(PMP_MODULE_ID index, PMP_CHIP_SELECT chipSelect)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_WriteChipSelectXDisable(PMP_MODULE_ID index, PMP_CHIP_SELECT chipSelect)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsReadChipSelectEnable(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_ReadChipSelectXEnable(PMP_MODULE_ID index, PMP_CHIP_SELECT chipSelect)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_ReadChipSelectXDisable(PMP_MODULE_ID index, PMP_CHIP_SELECT chipSelect)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsAddressControl(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_AddressSet(PMP_MODULE_ID index, uint32_t address)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_PMP_AddressGet(PMP_MODULE_ID index)
{
     return (uint32_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_AddressLinesA0A1Set(PMP_MODULE_ID index, uint8_t address)
{
     
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_PMP_AddressLinesA0A1Get(PMP_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsDualModeWriteAddressControl(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_DualModeWriteAddressSet(PMP_MODULE_ID index, uint32_t writeAddress)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_PMP_DualModeWriteAddressGet(PMP_MODULE_ID index)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsDualModeReadAddressControl(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_DualModeReadAddressSet(PMP_MODULE_ID index, uint32_t readAddress)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_PMP_DualModeReadAddressGet(PMP_MODULE_ID index)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsAddressPortPinControl(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_AddressPortEnable(PMP_MODULE_ID index, PMP_ADDRESS_PORT portfunctions)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_AddressPortDisable(PMP_MODULE_ID index, PMP_ADDRESS_PORT portfunctions)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsAddressLatchStrobePortControl(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_AddressLatchStrobeEnable(PMP_MODULE_ID index, PMP_ADDRESS_LATCH latch)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_AddressLatchStrobeDisable(PMP_MODULE_ID index, PMP_ADDRESS_LATCH latch)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsInputBufferFull(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PMP_InputBuffersAreFull(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsBufferOverFlow(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PMP_InputOverflowHasOccurred(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_InputOverflowClear(PMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsInputBufferXStatus(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PMP_InputBufferXIsFull(PMP_MODULE_ID index, uint8_t buffer)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PMP_IsDataReceived(PMP_MODULE_ID index, uint8_t buffer)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsOutPutBufferEmpty(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PMP_OutputBuffersAreEmpty(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsBufferUnderFlow(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PMP_OutputUnderflowHasOccurred(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_OutputUnderflowClear(PMP_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsOutputBufferXStatus(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PMP_OutputBufferXIsEmpty(PMP_MODULE_ID index, uint8_t buffer)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PMP_IsDataTransmitted(PMP_MODULE_ID index, uint8_t buffer)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsBufferRead(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_PMP_InputBufferXByteReceive(PMP_MODULE_ID index, uint8_t buffer)
{
     return (uint8_t)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsBufferWrite(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_OutputBufferXByteSend(PMP_MODULE_ID index, uint8_t buffer, uint8_t data)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsMasterRXTX(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_MasterSend(PMP_MODULE_ID index, uint16_t data)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_PMP_MasterReceive(PMP_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsDualModeMasterRXTX(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_DualModeMasterSend(PMP_MODULE_ID index, uint16_t data)
{
     
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_PMP_DualModeMasterReceive(PMP_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsSlaveTX(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_PMP_SlaveSend(PMP_MODULE_ID index, uint16_t data)
{
     
}

PLIB_INLINE_API bool PLIB_PMP_ExistsSlaveRX(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint16_t _PLIB_UNSUPPORTED PLIB_PMP_SlaveReceive(PMP_MODULE_ID index)
{
     return (uint16_t)0;
}

PLIB_INLINE_API bool PLIB_PMP_ExistsCSXActiveStatus(PMP_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_PMP_ChipSelectXIsActive(PMP_MODULE_ID index, PMP_CHIP_SELECT chipSelect)
{
     return (bool)0;
}

#endif
