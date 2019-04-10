/*******************************************************************************
  WINC1500 Wireless Driver Extension

  File Name:
    wdrext_winc1500.c

  Summary:
    WINC1500 wireless driver extension.

  Description:
    WINC1500 wireless driver extension.
 *******************************************************************************/

//DOM-IGNORE-BEGIN
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

#include "driver/wifi/winc1500/wireless_driver_extension/winc1500_task.h"

#include "driver/wifi/winc1500/wireless_driver_extension/driver/source/m2m_hif.h"
#include "driver/wifi/winc1500/wireless_driver_extension/driver/include/m2m_wifi.h"

static WDRV_CONFIG s_wdrv_cfg;
WDRV_CONFIG *gp_wdrv_cfg = &s_wdrv_cfg;

WINC1500_PRIV g_wdrvext_priv = {0, };
static WINC1500_INTF s_winc1500_intf;
static WINC1500_CONNECT_DATA s_connData;

OSAL_MUTEX_HANDLE_TYPE *g_debugConsoleLock;
volatile static bool s_isInitComplete = false;

static void WDRV_EXT_InitConnData(WINC1500_CONNECT_DATA *data)
{
    data->isSoftAP = CONN_DATA_UNSPECIFIED;
    data->channel = CONN_DATA_UNSPECIFIED;
    data->secType = CONN_DATA_UNSPECIFIED;
    data->ssidLen = CONN_DATA_UNSPECIFIED;
    data->keyLen = CONN_DATA_UNSPECIFIED;
    data->keyIdx = CONN_DATA_UNSPECIFIED;
}

static bool is_conn_data_valid(WINC1500_CONNECT_DATA *data)
{
    if (data->isSoftAP == CONN_DATA_UNSPECIFIED) {
        WDRV_ASSERT(false, "Network type is not specified yet");
        return false;
    }
    if (data->isSoftAP) {
        if (data->channel == CONN_DATA_UNSPECIFIED) {
            WDRV_ASSERT(false, "Channel is not specified yet");
            return false;
        } else if (data->keyLen == CONN_DATA_UNSPECIFIED) {
            WDRV_ASSERT(false, "Key length is not specified yet");
            return false;
        }
    }
    if (data->secType == CONN_DATA_UNSPECIFIED) {
        WDRV_ASSERT(false, "Security type is not specified yet");
        return false;
    }
    if (data->ssidLen == CONN_DATA_UNSPECIFIED) {
        WDRV_ASSERT(false, "SSID is not specified yet");
        return false;
    }
    return true;
}

static void InitDoneCB(void)
{
    s_isInitComplete = true;
}

static int32_t _WDRV_EXT_Initialize(WDRV_HOOKS const *const hooks)
{
    int32_t res = 0;

    memset(&s_winc1500_intf, 0, sizeof(WINC1500_INTF));
    memset(&g_wdrvext_priv, 0, sizeof(WINC1500_PRIV));

    WDRV_EXT_InitConnData(&s_connData);
    WDRV_SEM_INIT(&g_wdrvext_priv.scanResultWait);
    WDRV_SEM_INIT(&g_wdrvext_priv.connInfoWait);
    WDRV_SEM_INIT(&g_wdrvext_priv.eventWait);

    s_winc1500_intf.FrameReceived_CB = hooks->frame_received;
    s_winc1500_intf.RFReady_CB = hooks->RFReady;
    s_winc1500_intf.ScanDone_CB = hooks->ScanDone;
    s_winc1500_intf.InitDone_CB = InitDoneCB;
    s_winc1500_intf.get_rx_buf = hooks->get_rx_buf;
    s_winc1500_intf.ConnectEvent_CB = hooks->ConnectEvent;
    s_winc1500_intf.WPSDone_CB = hooks->WPSDone;

    if (hooks->isEthMode)
        s_winc1500_intf.ethMode = true;

    if (hooks->isOtaFwUpdateRequested) {
        s_winc1500_intf.ethMode = false;
        s_winc1500_intf.fwOtaServerUrl = hooks->fwOtaServerUrl;
    }

    if (hooks->isSerialFwUpdateRequested) {
        res = WDRV_TASK_CREATE((void *const)winc1500_fw_update, "WINC1500 task", 2048,
            &s_winc1500_intf, configMAX_PRIORITIES - 1, &g_wdrvext_priv.handle);
    } else {
        res = WDRV_TASK_CREATE((void *const)winc1500_task, "WINC1500 task", WDRV_EXT_RTOS_TASK_SIZE,
            &s_winc1500_intf, WDRV_EXT_RTOS_TASK_PRIORITY, &g_wdrvext_priv.handle);
    }

    return res ? WDRV_ERROR : WDRV_SUCCESS;
}

static void _WDRV_EXT_Deinitialize(void)
{
    g_wdrvext_priv.deinit_in_progress = true;
    WDRV_SEM_GIVE(&g_wdrvext_priv.eventWait);

    while (g_wdrvext_priv.deinit_in_progress)
        WDRV_TIME_DELAY(10);

    WDRV_SEM_DEINIT(&g_wdrvext_priv.eventWait);
    WDRV_SEM_DEINIT(&g_wdrvext_priv.scanResultWait);
    WDRV_SEM_DEINIT(&g_wdrvext_priv.connInfoWait);
    WDRV_TASK_DELETE(g_wdrvext_priv.handle);
}

bool isWdrvExtReady(void)
{
    return s_isInitComplete;
}

void WDRV_EXT_CmdSSIDSet(uint8_t *ssid, uint8_t len)
{
    s_connData.ssidLen = len;
    memcpy(s_connData.ssid, ssid, s_connData.ssidLen);
    s_connData.ssid[s_connData.ssidLen] = '\0';
}

void WDRV_EXT_CmdSSIDGet(uint8_t *ssid, uint8_t *ssidLen)
{
    *ssidLen = s_connData.ssidLen;
    memcpy(ssid, s_connData.ssid, *ssidLen);
    ssid[*ssidLen] = '\0';
}

uint32_t WDRV_EXT_CmdConnectContextBssidGet(uint8_t *bssId)
{
    int8_t ret;
    tstrM2MConnInfo info;

    ret = m2m_wifi_get_connection_info();
    WDRV_ASSERT(ret == 0, "WDRV_EXT_CmdConnectContextBssidGet error");

    if (ret == 0) {
        WDRV_SEM_TAKE(&g_wdrvext_priv.connInfoWait, OSAL_WAIT_FOREVER);
        winc1500_conn_info_read(&info);
        memcpy((void *)bssId, (void *)info.au8MACAddress, 6);
    }

    return ret ? WDRV_ERROR : WDRV_SUCCESS;
}

uint32_t WDRV_EXT_CmdMacAddressGet(uint8_t *MacAddr)
{
    int8_t ret;

    ret = m2m_wifi_get_mac_address(MacAddr);
    WDRV_ASSERT(ret == 0, "WDRV_EXT_CmdMacAddressGet error");

    return ret ? WDRV_ERROR : WDRV_SUCCESS;
}

void WDRV_EXT_CmdNetModeBSSSet(void)
{
    s_connData.isSoftAP = 0;
}

void WDRV_EXT_CmdNetModeAPSet(void)
{
    s_connData.isSoftAP = 1;
}

uint32_t WDRV_EXT_CmdConnect(void)
{
    int8_t ret;

    if (!is_conn_data_valid(&s_connData)) {
        WDRV_ASSERT(false, "Invalid connection data");
        return WDRV_ERROR;
    }

    s_connData.channel = s_connData.channel == CONN_DATA_UNSPECIFIED ? M2M_WIFI_CH_ALL : s_connData.channel;

    ret = winc1500_connect(&s_connData);
    WDRV_ASSERT(ret == 0, "WDRV_EXT_CmdConnect error");

    return ret ? WDRV_ERROR : WDRV_SUCCESS;
}

uint32_t WDRV_EXT_CmdDisconnect(void)
{
    int8_t ret;

    if (s_connData.isSoftAP) {
        ret = m2m_wifi_disable_ap();
        WDRV_ASSERT(ret == 0, "main: m2m_wifi_disable_ap error");
        winc1500_connect_event_received(false, true, NULL);
        WDRV_DBG_INFORM_PRINT("SoftAP is disabled\r\n");
    } else {
        ret = m2m_wifi_disconnect();
        WDRV_ASSERT(ret == 0, "main: m2m_wifi_disconnect error");
    }

    return ret ? WDRV_ERROR : WDRV_SUCCESS;
}

uint32_t WDRV_EXT_CmdTxPowerSet(uint32_t level)
{
    int8_t ret;

    ret = m2m_wifi_set_tx_power(level);

    WDRV_ASSERT(ret == 0, "WDRV_EXT_CmdTxPowerSet error");

    return ret ? WDRV_ERROR : WDRV_SUCCESS;
}

uint32_t
WDRV_EXT_CmdScanOptionsSet(uint8_t numOfSlots, uint8_t slotTime, uint8_t probesPerSlot, uint8_t rssiThreshold)
{
    tstrM2MScanOption scanOptions;
    int8_t ret;

    scanOptions.s8RssiThresh = rssiThreshold;
    scanOptions.u8NumOfSlot = numOfSlots;
    scanOptions.u8ProbesPerSlot = probesPerSlot;
    scanOptions.u8SlotTime = slotTime;

    ret = m2m_wifi_set_scan_options(&scanOptions);
    WDRV_ASSERT(ret == 0, "WDRV_EXT_CmdScanModeSet error");

    return ret ? WDRV_ERROR : WDRV_SUCCESS;
}

uint32_t WDRV_EXT_CmdPowerSavePut(bool enable, uint8_t mode, uint16_t listenInterval)
{
    int32_t ret;

    if (enable) {
        if (mode == 0) { /* ps manual */
            ret = m2m_wifi_set_sleep_mode(M2M_PS_MANUAL, 1);
            if (ret == 0) {
                ret = m2m_wifi_request_sleep(1000);
            }
        } else { /* deep automatic */
            tstrM2mLsnInt strM2mLsnInt;
            ret = m2m_wifi_set_sleep_mode(M2M_PS_DEEP_AUTOMATIC, 1);
            if (ret == 0) {
                strM2mLsnInt.u16LsnInt = listenInterval;
                ret = m2m_wifi_set_lsn_int(&strM2mLsnInt);
            }
        }
    } else {
        ret = m2m_wifi_set_sleep_mode(M2M_NO_PS, 0);
    }

    return ret ? WDRV_ERROR : WDRV_SUCCESS;
}

uint32_t WDRV_EXT_RssiRead(void)
{
    int32_t ret;

    /* Request RSSI for the connected AP. */
    ret = m2m_wifi_req_curr_rssi();

    return ret ? WDRV_ERROR : WDRV_SUCCESS;
}

void WDRV_EXT_CmdSecNoneSet(void)
{
    s_connData.secType = M2M_WIFI_SEC_OPEN;
}

void WDRV_EXT_CmdSecWEPSet(uint8_t *key, uint16_t len)
{
    s_connData.secType = M2M_WIFI_SEC_WEP;
    s_connData.keyLen = len;
    s_connData.keyIdx = 1; /* 1 is chosen per default. This is configurable. */
    memcpy(s_connData.key, key, len);
    s_connData.key[s_connData.keyLen] = '\0';
}

void WDRV_EXT_CmdSecWPASet(uint8_t *key, uint16_t len)
{
    s_connData.secType = M2M_WIFI_SEC_WPA_PSK;
    s_connData.keyLen = len;
    memcpy(s_connData.key, key, len);
    s_connData.key[s_connData.keyLen] = '\0';
}

uint32_t WDRV_EXT_CmdSecWpsSet(bool pinMode, uint8_t *key, uint16_t keyLen)
{
    uint8_t keyStr[9];
    uint8_t type = pinMode ? WPS_PIN_TRIGGER : WPS_PBC_TRIGGER;
    int8_t ret;

    if (type == WPS_PIN_TRIGGER) {
        WDRV_ASSERT(keyLen == 8, "WDRV_EXT_CmdSecWpsSet error, incorrect WPS PIN length");
        memcpy((void *)keyStr, key, keyLen);
        keyStr[8] = '\0';
    }
    WDRV_DBG_INFORM_PRINT("WPS starts...\r\n");
    ret = m2m_wifi_wps(type, (const char *)keyStr);

    WDRV_ASSERT(ret == 0, "WDRV_EXT_CmdSecWpsSet error");

    return ret ? WDRV_ERROR : WDRV_SUCCESS;
}

uint32_t WDRV_EXT_CmdFWVersionGet(uint32_t *major, uint32_t *minor, uint32_t *patch)
{
    tstrM2mRev strtmp;
    int32_t ret;

    ret = nm_get_firmware_info(&strtmp);

    WDRV_DBG_INFORM_PRINT("Firmware ver:    %u.%u.%u\n", strtmp.u8FirmwareMajor, strtmp.u8FirmwareMinor, strtmp.u8FirmwarePatch);
    WDRV_DBG_INFORM_PRINT("Min driver ver:  %u.%u.%u\n", strtmp.u8DriverMajor, strtmp.u8DriverMinor, strtmp.u8DriverPatch);
    WDRV_DBG_INFORM_PRINT("Curr driver ver: %u.%u.%u\n", M2M_RELEASE_VERSION_MAJOR_NO, M2M_RELEASE_VERSION_MINOR_NO, M2M_RELEASE_VERSION_PATCH_NO);
    WDRV_ASSERT(M2M_ERR_FW_VER_MISMATCH != ret, "Firmware version does not match with driver");

    if (ret == 0) {
        *major = strtmp.u8FirmwareMajor;
        *minor = strtmp.u8FirmwareMinor;
        *patch = strtmp.u8FirmwarePatch;
    }

    return ret ? WDRV_ERROR : WDRV_SUCCESS;
}

void WDRV_EXT_CmdChannelSet(uint8_t channel)
{
    s_connData.channel = channel;
}

uint32_t WDRV_EXT_CmdScanStart(void)
{
    int8_t ret;
    uint8_t channel = s_connData.channel == CONN_DATA_UNSPECIFIED ? M2M_WIFI_CH_ALL : s_connData.channel;

    ret = m2m_wifi_request_scan(channel);
    WDRV_ASSERT(ret == 0, "WDRV_EXT_CmdScanStart error");

    return ret ? WDRV_ERROR : WDRV_SUCCESS;
}

void WDRV_EXT_CmdScanGet(uint16_t *numOfResults)
{
   *numOfResults = m2m_wifi_get_num_ap_found();
}

void WDRV_EXT_WPSResultsRead(WDRV_CONFIG *config, uint32_t *status)
{
    tstrM2MWPSInfo info;

    winc1500_wps_info_read(&info);
    config->ssidLen = strlen((char *)info.au8SSID);
    memcpy((void *)config->ssid, (void *)info.au8SSID, sizeof(config->ssid));
    config->securityKeyLen = strlen((char *)info.au8PSK);
    memcpy((void *)config->securityKey, (void *)info.au8PSK, sizeof(config->securityKey));
    if (info.u8AuthType == M2M_WIFI_SEC_OPEN) {
        config->securityMode = WDRV_SECURITY_OPEN;
    } else if (info.u8AuthType == M2M_WIFI_SEC_WPA_PSK) {
        config->securityMode = WDRV_SECURITY_WPA_AUTO_WITH_PASS_PHRASE;
    } else if (info.u8AuthType == M2M_WIFI_SEC_WEP) {
        if (config->securityKeyLen == 10) /* e.g. strlen("1234567890")  */
            config->securityMode = WDRV_SECURITY_WEP_40;
        else /* e.g. strlen("1234567890abcdef1234567890") */
            config->securityMode = WDRV_SECURITY_WEP_104;
    }

    *status = info.u8AuthType != M2M_WIFI_SEC_INVALID ? WDRV_SUCCESS : WDRV_ERROR;
}

void WDRV_EXT_HWInterruptHandler(void)
{
    winc1500_isr(&g_wdrvext_priv.eventWait);
}

uint32_t WDRV_EXT_DataSend(uint16_t segSize, uint8_t *p_segData)
{
    int8_t ret;

    ret = winc1500_eth_data_send(p_segData, segSize);
    if (ret) {
        if (ret == M2M_ERR_MEM_ALLOC) {
            WDRV_DBG_TRACE_PRINT("Memory is not available temporarily\r\n");
            ret = WDRV_OUT_OF_MEMORY;
        } else {
            WDRV_DBG_ERROR_PRINT("Failed to send data - %d\r\n", ret);
            ret = WDRV_ERROR;
        }
    }
    return ret;
}

void WDRV_EXT_ModuleUpDown(uint32_t up)
{
    up ? winc1500_init() : winc1500_deinit();
}

void WDRV_EXT_ScanResultGet(uint8_t idx, WDRV_SCAN_RESULT *p_scanResult)
{
    int8_t ret;
    tstrM2mWifiscanResult result;

    ret = m2m_wifi_req_scan_result(idx);
    WDRV_ASSERT(ret == 0, "WDRV_EXT_ScanResultGet error");
    if (ret)
        return;

    WDRV_SEM_TAKE(&g_wdrvext_priv.scanResultWait, OSAL_WAIT_FOREVER);
    winc1500_scan_result_read(&result);

    memcpy((void *)p_scanResult->bssid, (void *)result.au8BSSID, sizeof(p_scanResult->bssid));
    p_scanResult->bssType = WDRV_NETWORK_TYPE_INFRASTRUCTURE; // so far, always set bssType to infrastructure
    p_scanResult->ssidLen = strlen((const char *)result.au8SSID);
    memcpy((void *)p_scanResult->ssid, (void *)result.au8SSID, sizeof(p_scanResult->ssid));
    p_scanResult->channel = result.u8ch;
    p_scanResult->rssi = result.s8rssi;
    p_scanResult->apConfig = 0;
    if (result.u8AuthType != M2M_WIFI_SEC_OPEN)
        p_scanResult->apConfig |= WDRV_APCONFIG_BIT_PRIVACY;
    if (result.u8AuthType == M2M_WIFI_SEC_WPA_PSK) {
        p_scanResult->apConfig |= WDRV_APCONFIG_BIT_WPA;
        p_scanResult->apConfig |= WDRV_APCONFIG_BIT_WPA2;
    }
}

uint32_t WDRV_EXT_MulticastFilterSet(uint8_t *addr)
{
    int8_t ret;

    ret = m2m_wifi_enable_mac_mcast(addr, 1);

    WDRV_ASSERT(ret == 0, "WDRV_EXT_MulticastFilterSet error");

    return ret ? WDRV_ERROR : WDRV_SUCCESS;
}

void WDRV_EXT_Initialize(const WDRV_HOOKS *const ehooks, bool initWait)
{
    if (s_isInitComplete)
        return;

    g_debugConsoleLock = NULL;
    WDRV_MUTEX_CREATE(&g_debugConsoleLock);

    WDRV_DBG_INFORM_PRINT("WINC1500: Initializing...\r\n");

    WDRV_STUB_GPIO_Initialize();
    WDRV_STUB_SPI_Initialize();

    _WDRV_EXT_Initialize(ehooks);

    while (initWait && !s_isInitComplete)
        WDRV_TIME_DELAY(1);
}

void WDRV_EXT_Deinitialize(void)
{
    if (!s_isInitComplete)
        return;

    WDRV_DBG_INFORM_PRINT("WINC1500: De-initializing...\r\n");
    s_isInitComplete = false;

    _WDRV_EXT_Deinitialize();
    WDRV_STUB_GPIO_DeInitialize();
    WDRV_STUB_SPI_Deinitialize();

    WDRV_MUTEX_DELETE(&g_debugConsoleLock);
    g_debugConsoleLock = NULL;
}

//DOM-IGNORE-END
