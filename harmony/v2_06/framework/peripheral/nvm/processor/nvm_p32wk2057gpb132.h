/* Created by plibgen $Revision: 1.31 $ */

#ifndef _NVM_P32WK2057GPB132_H
#define _NVM_P32WK2057GPB132_H

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

    NVM_NUMBER_OF_MODULES = 0

} NVM_MODULE_ID;

typedef enum {

    NVM_UNLOCK_KEYS_NONE

} NVM_UNLOCK_KEYS;

typedef enum {

    NVM_ROW_PAGE_SIZE_NONE

} NVM_ROW_PAGE_SIZE;

typedef enum {

    NVM_OPERATION_MODE_NONE

} NVM_OPERATION_MODE;

typedef enum {

    NVM_BOOT_MEMORY_AREA_NONE

} NVM_BOOT_MEMORY_AREA;

typedef enum {

    NVM_BOOT_MEMORY_PAGE_NONE

} NVM_BOOT_MEMORY_PAGE;

typedef enum {

    NVM_FLASH_SWAP_LOCK_TYPE_NONE

} NVM_FLASH_SWAP_LOCK_TYPE;

typedef enum {

    EEPROM_OPERATION_MODE_NONE

} EEPROM_OPERATION_MODE;

typedef enum {

    EEPROM_ERROR_NONE

} EEPROM_ERROR;

typedef enum {

    EEPROM_CALIBRATION_REG_NONE

} EEPROM_CALIBRATION_REG;

/* Section 2 - Feature variant inclusion */

#define PLIB_TEMPLATE PLIB_INLINE

/* Section 3 - PLIB dispatch function definitions */

PLIB_INLINE_API bool PLIB_NVM_ExistsWriteErrorStatus(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_WriteOperationHasTerminated(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsMemoryModificationControl(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_MemoryModifyEnable(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_MemoryModifyInhibit(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_NVM_ExistsOperationMode(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_MemoryOperationSelect(NVM_MODULE_ID index, NVM_OPERATION_MODE operationmode)
{
     
}

PLIB_INLINE_API bool PLIB_NVM_ExistsAddressModifyControl(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_FlashAddressToModify(NVM_MODULE_ID index, uint32_t address)
{
     
}

PLIB_INLINE_API bool PLIB_NVM_ExistsProvideData(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_FlashProvideData(NVM_MODULE_ID index, uint32_t data)
{
     
}

PLIB_INLINE_API bool PLIB_NVM_ExistsWriteOperation(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_NVM_FlashRead(NVM_MODULE_ID index, uint32_t address)
{
     return (uint32_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_FlashWriteStart(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_FlashEraseStart(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_FlashWriteCycleHasCompleted(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsKeySequence(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_FlashWriteKeySequence(NVM_MODULE_ID index, uint32_t keysequence)
{
     
}

PLIB_INLINE_API bool PLIB_NVM_ExistsSourceAddress(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_DataBlockSourceAddress(NVM_MODULE_ID index, uint32_t address)
{
     
}

PLIB_INLINE_API bool PLIB_NVM_ExistsLowVoltageStatus(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_LowVoltageEventIsActive(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsLowVoltageError(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_LowVoltageIsDetected(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsFlashBankRegionSelect(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_ProgramFlashBank1LowerRegion(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_ProgramFlashBank2LowerRegion(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_ProgramFlashBank2IsLowerRegion(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsBootFlashBankRegion(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_BootFlashBank1LowerRegion(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_BootFlashBank2LowerRegion(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_BootFlashBank2IsLowerRegion(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsProvideQuadData(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_FlashProvideQuadData(NVM_MODULE_ID index, uint32_t* data)
{
     
}

PLIB_INLINE_API bool PLIB_NVM_ExistsLockPFMSelect(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_LockProgramFlashMemory(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_IsProgramFlashMemoryLocked(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsFlashWPMemoryRangeProvide(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_FlashWriteProtectMemoryAreaRange(NVM_MODULE_ID index, uint32_t address)
{
     
}

PLIB_INLINE_API bool PLIB_NVM_ExistsLockBootSelect(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_LockBootMemory(NVM_MODULE_ID index, NVM_BOOT_MEMORY_AREA memoryArea)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_IsBootMemoryLocked(NVM_MODULE_ID index, NVM_BOOT_MEMORY_AREA memoryArea)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsBootPageWriteProtect(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_BootPageWriteProtectionEnable(NVM_MODULE_ID index, NVM_BOOT_MEMORY_PAGE bootPage)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_BootPageWriteProtectionDisable(NVM_MODULE_ID index, NVM_BOOT_MEMORY_PAGE bootPage)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_IsBootPageWriteProtected(NVM_MODULE_ID index, NVM_BOOT_MEMORY_PAGE bootPage)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsSwapLockControl(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_FlashSwapLockSelect(NVM_MODULE_ID index, NVM_FLASH_SWAP_LOCK_TYPE lockType)
{
     
}

PLIB_INLINE_API NVM_FLASH_SWAP_LOCK_TYPE _PLIB_UNSUPPORTED PLIB_NVM_FlashSwapLockStatusGet(NVM_MODULE_ID index)
{
     return (NVM_FLASH_SWAP_LOCK_TYPE)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsEEPROMEnableControl(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_EEPROMEnable(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_EEPROMDisable(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_EEPROMIsReady(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsEEPROMStopInIdleControl(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_EEPROMStopInIdleEnable(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_EEPROMStopInIdleDisable(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_EEPROMStopInIdleIsEnabled(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsEEPROMOperationModeControl(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_EEPROMOperationSelect(NVM_MODULE_ID index, EEPROM_OPERATION_MODE mode)
{
     
}

PLIB_INLINE_API bool PLIB_NVM_ExistsEEPROMAddressControl(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_EEPROMAddress(NVM_MODULE_ID index, uint32_t address)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsEEPROMDataControl(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_EEPROMDataToWrite(NVM_MODULE_ID index, uint32_t data)
{
     
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_NVM_EEPROMRead(NVM_MODULE_ID index)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsEEPROMKeySequence(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_EEPROMKeySequenceWrite(NVM_MODULE_ID index, uint32_t keysequence)
{
     
}

PLIB_INLINE_API bool PLIB_NVM_ExistsEEPROMEnableOperationControl(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_EEPROMWriteEnable(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_EEPROMWriteIsEnabled(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_EEPROMReadEnable(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_EEPROMReadIsEnabled(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsEEPROMStartOperationControl(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_EEPROMReadStart(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_EEPROMWriteStart(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_EEPROMEraseStart(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_EEPROMOperationHasCompleted(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsEEPROMLongWriteStatus(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool _PLIB_UNSUPPORTED PLIB_NVM_EEPROMNextWriteCycleIsLong(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsEEPROMOperationAbortControl(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_EEPROMOperationAbort(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_NVM_ExistsEEPROMErrorStatus(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API EEPROM_ERROR _PLIB_UNSUPPORTED PLIB_NVM_EEPROMErrorGet(NVM_MODULE_ID index)
{
     return (EEPROM_ERROR)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_EEPROMErrorClear(NVM_MODULE_ID index)
{
     
}

PLIB_INLINE_API bool PLIB_NVM_ExistsEEPROMCalibrationData(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint32_t _PLIB_UNSUPPORTED PLIB_NVM_EEPROMReadCalibrationData(NVM_MODULE_ID index, EEPROM_CALIBRATION_REG regIndex)
{
     return (uint32_t)0;
}

PLIB_INLINE_API bool PLIB_NVM_ExistsEEPROMWaitStates(NVM_MODULE_ID index)
{
     return (bool)0;
}

PLIB_INLINE_API uint8_t _PLIB_UNSUPPORTED PLIB_NVM_EEPROMWaitStatesGet(NVM_MODULE_ID index)
{
     return (uint8_t)0;
}

PLIB_INLINE_API void _PLIB_UNSUPPORTED PLIB_NVM_EEPROMWaitStatesSet(NVM_MODULE_ID index, uint8_t eews)
{
     
}

#endif
