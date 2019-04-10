/*******************************************************************************
  WINC1500 Wireless Driver Configuration

  File Name:
    wdrv_winc1500_config.c

  Summary:
    Manage WINC1500 configuration.

  Description:
    Stores/retrieves Wi-Fi configuration to/from non-volatile memory (NVM).
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc. All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/

#include "system_config.h"

#include "driver/nvm/drv_nvm.h"

#include "driver/wifi/winc1500/wireless_driver/include/wdrv_winc1500_main.h"

#define WDRV_NVM_SPACE_MAGIC_NUMBER 0x5a5a5a5a

#if defined(WDRV_NVM_SPACE_ENABLE)
static DRV_HANDLE s_nvmHandle;

static bool WDRV_NVM_Read(uint8_t *buf, size_t size, uint32_t startAddr)
{
    DRV_NVM_COMMAND_HANDLE nvmCommandHandle;

    if (startAddr > DRV_NVM_MEDIA_SIZE * 1024)
        return false;

    s_nvmHandle = DRV_NVM_Open(0, DRV_IO_INTENT_READ);
    if (s_nvmHandle == DRV_HANDLE_INVALID) {
        DRV_NVM_Close(s_nvmHandle);
        s_nvmHandle = NULL;
        return false;
    }

    DRV_NVM_Read(s_nvmHandle, &nvmCommandHandle, buf, startAddr, size);
    if (nvmCommandHandle == DRV_NVM_COMMAND_HANDLE_INVALID) {
        DRV_NVM_Close(s_nvmHandle);
        s_nvmHandle = NULL;
        return false;
    }

    DRV_NVM_Close(s_nvmHandle);
    s_nvmHandle = NULL;
    return true;
}

/**
 * Now, this function can only be used to write Wi-Fi configuration. We didn't
 * extend its use cases more broadly because of memory saving reason.
 *
 * It explicitly erases a whole memory page but only writes back the Wi-Fi
 * configuration which occupies a very small chunk of memory. It doesn't read
 * and maintain any data before erasing. So all the previous data is lost.
 * However, this operation is safe under the assumption that the whole memory
 * page starting at startAddr is only managed by WINC1500 driver.
 *
 * If we want to extend this function's usage more in the future, the size of
 * static buffer it maintains should be increased to DRV_NVM_ROW_SIZE or even
 * DRV_NVM_PAGE_SIZE depends on necessity, and it should read data from memory
 * before erasing to prevent data loss.
 */
static bool WDRV_NVM_PageEraseConfigWrite(uint8_t *buf, size_t size, uint32_t startAddr)
{
    static WDRV_CONFIG nvmCfgBuf;
    SYS_FS_MEDIA_GEOMETRY *nvmMediaGeometry;
    DRV_NVM_COMMAND_HANDLE nvmCommandHandle;

    if (size > sizeof(nvmCfgBuf))
        return false;

    if (startAddr > DRV_NVM_MEDIA_SIZE * 1024)
        return false;

    s_nvmHandle = DRV_NVM_Open(0, DRV_IO_INTENT_WRITE);
    if (s_nvmHandle == DRV_HANDLE_INVALID) {
        DRV_NVM_Close(s_nvmHandle);
        s_nvmHandle = NULL;
        return false;
    }

    memcpy(&nvmCfgBuf, buf, size);

    nvmMediaGeometry = DRV_NVM_GeometryGet(s_nvmHandle);
    if (nvmMediaGeometry == NULL) {
        DRV_NVM_Close(s_nvmHandle);
        s_nvmHandle = NULL;
        return false;
    }

    DRV_NVM_EraseWrite(s_nvmHandle,
            &nvmCommandHandle,
            &nvmCfgBuf,
            nvmMediaGeometry->geometryTable[1].numBlocks * startAddr / (DRV_NVM_MEDIA_SIZE * 1024),
            1);

    if (nvmCommandHandle == DRV_NVM_COMMAND_HANDLE_INVALID) {
        DRV_NVM_Close(s_nvmHandle);
        s_nvmHandle = NULL;
        return false;
    }

    DRV_NVM_Close(s_nvmHandle);
    s_nvmHandle = NULL;

    return true;
}

static bool WDRV_NVM_PageErase(uint32_t startAddr)
{
    SYS_FS_MEDIA_GEOMETRY *nvmMediaGeometry;
    DRV_NVM_COMMAND_HANDLE nvmCommandHandle;

    if (startAddr > DRV_NVM_MEDIA_SIZE * 1024)
        return false;

    s_nvmHandle = DRV_NVM_Open(0, DRV_IO_INTENT_WRITE);
    if (s_nvmHandle == DRV_HANDLE_INVALID) {
        DRV_NVM_Close(s_nvmHandle);
        s_nvmHandle = NULL;
        return false;
    }

    nvmMediaGeometry = DRV_NVM_GeometryGet(s_nvmHandle);
    if (nvmMediaGeometry == NULL) {
        DRV_NVM_Close(s_nvmHandle);
        s_nvmHandle = NULL;
        return false;
    }

    DRV_NVM_Erase(s_nvmHandle,
            &nvmCommandHandle,
            nvmMediaGeometry->geometryTable[2].numBlocks * startAddr / (DRV_NVM_MEDIA_SIZE * 1024),
            1);

    if (nvmCommandHandle == DRV_NVM_COMMAND_HANDLE_INVALID) {
        DRV_NVM_Close(s_nvmHandle);
        s_nvmHandle = NULL;
        return false;
    }

    DRV_NVM_Close(s_nvmHandle);
    s_nvmHandle = NULL;

    return true;
}

static bool LoadConfigFromMemory(void)
{
    uint32_t verifyFlag;

    if (s_nvmHandle) {
        WDRV_DBG_INFORM_PRINT("Last Wi-Fi configuration NVM operation is not finished, please wait\r\n");
        return false;
    }

    WDRV_NVM_Read((uint8_t *)&verifyFlag, sizeof(uint32_t), WDRV_NVM_SPACE_ADDR);

    if (verifyFlag == WDRV_NVM_SPACE_MAGIC_NUMBER) {
        return WDRV_NVM_Read((uint8_t *)gp_wdrv_cfg, sizeof(WDRV_CONFIG), WDRV_NVM_SPACE_ADDR);
    } else {
        WDRV_DBG_INFORM_PRINT("No stored Wi-Fi configuration found in NVM\r\n");
        return false;
    }
}

static void SaveConfigToMemory(void)
{
    if (s_nvmHandle) {
        WDRV_DBG_INFORM_PRINT("Last Wi-Fi configuration NVM operation is not finished, please wait\r\n");
        return;
    }

    gp_wdrv_cfg->verifyFlag = WDRV_NVM_SPACE_MAGIC_NUMBER;

    if (WDRV_NVM_PageEraseConfigWrite((uint8_t *)gp_wdrv_cfg, sizeof(WDRV_CONFIG), WDRV_NVM_SPACE_ADDR))
        WDRV_DBG_INFORM_PRINT("WINC1500: NVM operation succeeded\r\n");
    else
        WDRV_DBG_INFORM_PRINT("WINC1500: NVM operation failed\r\n");
}

static void DeleteConfigInMemory(void)
{
    if (s_nvmHandle) {
        WDRV_DBG_INFORM_PRINT("Last Wi-Fi configuration NVM operation is not finished, please wait\r\n");
        return;
    }

    if (WDRV_NVM_PageErase(WDRV_NVM_SPACE_ADDR))
        WDRV_DBG_INFORM_PRINT("WINC1500: NVM operation succeeded\r\n");
    else
        WDRV_DBG_INFORM_PRINT("WINC1500: NVM operation failed\r\n");
}

#else // !defined(WDRV_NVM_SPACE_ENABLE)

static WDRV_CONFIG s_wdrv_cfgBackup;

static bool LoadConfigFromMemory(void)
{
    if (s_wdrv_cfgBackup.verifyFlag == WDRV_NVM_SPACE_MAGIC_NUMBER) {
        /*LDRA_INSPECTED 480 S */ /*Deviation Reference: MH-6783*/
        memcpy((uint8_t *)gp_wdrv_cfg, (uint8_t *)&s_wdrv_cfgBackup, sizeof(WDRV_CONFIG));
        return true;
    } else {
        WDRV_DBG_INFORM_PRINT("No stored Wi-Fi configuration found\r\n");
        return false;
    }
}

static void SaveConfigToMemory(void)
{
    gp_wdrv_cfg->verifyFlag = WDRV_NVM_SPACE_MAGIC_NUMBER;
    /*LDRA_INSPECTED 480 S */ /*Deviation Reference: MH-6784*/
    memcpy((uint8_t *)&s_wdrv_cfgBackup, (uint8_t *)gp_wdrv_cfg, sizeof(WDRV_CONFIG));
    WDRV_DBG_INFORM_PRINT("WINC1500: Save operation succeeded\r\n");
}

static void DeleteConfigInMemory(void)
{
    memset(&s_wdrv_cfgBackup, 0, sizeof(WDRV_CONFIG));
    WDRV_DBG_INFORM_PRINT("WINC1500: Erase operation succeeded\r\n");
}
#endif // defined(WDRV_NVM_SPACE_ENABLE)

bool WDRV_CONFIG_Load(void)
{
    bool res = LoadConfigFromMemory();

    // if driver is open, simply return the result of the load operation
    if (g_wdrv_priv.isDriverOpen)
        return res;

    // if driver is not open yet, there are two options:
    //   1. if NVM or s_wdrv_cfgBackup holds valid Wi-Fi configuration,
    //     use it
    //   2. otherwise use hard-coded Wi-Fi configuration in system_config.h
    if (!res) {
        if (sizeof(WDRV_DEFAULT_SSID) - 1 > WDRV_MAX_SSID_LENGTH) {
            WDRV_DBG_ERROR_PRINT("WDRV_DEFAULT_SSID could not contain more than %u characters\r\n", WDRV_MAX_SSID_LENGTH);
            return false;
        }

        WDRV_DBG_INFORM_PRINT("Using default Wi-Fi configuration\r\n");

        memcpy(gp_wdrv_cfg->ssid, (const void *)WDRV_DEFAULT_SSID, sizeof(WDRV_DEFAULT_SSID));
        gp_wdrv_cfg->ssidLen = sizeof(WDRV_DEFAULT_SSID) - 1;
        gp_wdrv_cfg->networkType = WDRV_DEFAULT_NETWORK_TYPE;
        gp_wdrv_cfg->securityMode = WDRV_DEFAULT_SECURITY_MODE;

        switch (gp_wdrv_cfg->securityMode) {
        case WDRV_SECURITY_OPEN:
            memset(gp_wdrv_cfg->securityKey, 0x00, sizeof(gp_wdrv_cfg->securityKey));
            gp_wdrv_cfg->securityKeyLen = 0;
            break;
        case WDRV_SECURITY_WEP_40:
            memcpy(gp_wdrv_cfg->securityKey, (const void *)WDRV_DEFAULT_WEP_KEYS_40, sizeof(WDRV_DEFAULT_WEP_KEYS_40) - 1);
            gp_wdrv_cfg->securityKeyLen = sizeof(WDRV_DEFAULT_WEP_KEYS_40) - 1;
            break;
        case WDRV_SECURITY_WEP_104:
            memcpy(gp_wdrv_cfg->securityKey, (const void *)WDRV_DEFAULT_WEP_KEYS_104, sizeof(WDRV_DEFAULT_WEP_KEYS_104) - 1);
            gp_wdrv_cfg->securityKeyLen = sizeof(WDRV_DEFAULT_WEP_KEYS_104) - 1;
            break;
        case WDRV_SECURITY_WPA_AUTO_WITH_PASS_PHRASE:
            memcpy(gp_wdrv_cfg->securityKey, (const void *)WDRV_DEFAULT_PSK_PHRASE, sizeof(WDRV_DEFAULT_PSK_PHRASE) - 1);
            gp_wdrv_cfg->securityKeyLen = sizeof(WDRV_DEFAULT_PSK_PHRASE) - 1;
            break;
        case WDRV_SECURITY_WPS_PUSH_BUTTON:
            memset(gp_wdrv_cfg->securityKey, 0x00, sizeof(gp_wdrv_cfg->securityKey));
            gp_wdrv_cfg->securityKeyLen = 0;
            break;
        case WDRV_SECURITY_WPS_PIN:
            memcpy(gp_wdrv_cfg->securityKey, (const void *)WDRV_DEFAULT_WPS_PIN, sizeof(WDRV_DEFAULT_WPS_PIN));
            gp_wdrv_cfg->securityKeyLen = sizeof(WDRV_DEFAULT_WPS_PIN);
            break;
        default:
            WDRV_ASSERT(false, "Unsupported security mode\r\n");
            break;
        }
    }

    return true;
}

void WDRV_CONFIG_Save(void)
{
    SaveConfigToMemory();
}

void WDRV_CONFIG_Delete(void)
{
    DeleteConfigInMemory();
}

// DOM-IGNORE-END
