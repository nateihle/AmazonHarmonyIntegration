/*******************************************************************************
  WINC1500 Private Configuration Support

  File Name:
    wdrv_winc1500_iwpriv.c

  Summary:
    Configure optional (private) parameters of WINC1500 driver.

  Description:
    Configure optional (private) parameters of WINC1500 driver.
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

/******************/
/*    INCLUDES    */
/******************/
#include "driver/wifi/winc1500/include/wdrv_winc1500_api.h"
#include "driver/wifi/winc1500/wireless_driver/include/wdrv_winc1500_iwpriv.h"
#include "driver/wifi/winc1500/wireless_driver/include/wdrv_winc1500_main.h"

/*******************/
/*    VARIABLES    */
/*******************/
static uint8_t s_prescan_allowed = false;
static uint8_t s_prescan_inprogress = false; // prescan is allowed only once

static bool _prescan_isfinished(void)
{
    if (s_prescan_inprogress && g_wdrv_priv.isScanDone) {
        s_prescan_inprogress = false;
        return true;
    }
    return false;
}

static IWPRIV_SCAN_STATUS _scanstatus_get(void)
{
    IWPRIV_SCAN_STATUS ret;

    WDRV_TIME_DELAY(1); // delay for accepting "scan done" callback from module
    if (g_wdrv_scanStatus.scanInProgress) {
        ret = IWPRIV_SCAN_IN_PROGRESS;
    } else if (g_wdrv_priv.isScanDone && (g_wdrv_scanStatus.numberOfResults > 0)){
        ret = IWPRIV_SCAN_SUCCESSFUL;
    } else if (g_wdrv_priv.isScanDone && (g_wdrv_scanStatus.numberOfResults == 0)) {
        ret = IWPRIV_SCAN_NO_AP_FOUND;
    } else {
        ret = IWPRIV_SCAN_IDLE;
    }
    return ret;
}

static void _ssid_get(uint8_t *ssid, uint8_t *ssidLen)
{
    memcpy(ssid, gp_wdrv_cfg->ssid, gp_wdrv_cfg->ssidLen);
    ssid[gp_wdrv_cfg->ssidLen] = 0;
    *ssidLen = gp_wdrv_cfg->ssidLen;
}

static void _leftclient_get(bool *updated, uint8_t *addr)
{
    bool connected = false;

    *updated = false;
    if (isClientCacheUpdated(&connected, addr))
        *updated = connected ? false: true;
}

static IWPRIV_CONN_STATUS _connstatus_get(void)
{
    WDRV_CONNECTION_STATES conn_state;
    IWPRIV_CONN_STATUS res = IWPRIV_CONNECTION_FAILED;

    conn_state = WDRV_ConnectionState_Get();
    switch (conn_state) {
    case WDRV_CONNECTION_STATE_NOT_CONNECTED:
        if (g_wdrv_priv.isDisconnectRequested)
            res = IWPRIV_CONNECTION_IDLE;
        else
            res = IWPRIV_CONNECTION_FAILED;
        break;
    case WDRV_CONNECTION_STATE_CONNECTED:
        res = IWPRIV_CONNECTION_SUCCESSFUL;
        break;
    case WDRV_CONNECTION_STATE_IN_PROGRESS:
        res = IWPRIV_CONNECTION_IN_PROGRESS;
        break;
    default:
        WDRV_ASSERT(false, "Undefined connection status.");
    }
    return res;
}

static bool _drvstatus_get(void)
{
    return g_wdrv_priv.isDriverOpen;
}

static bool _is_servermode(void)
{
    if (gp_wdrv_cfg->networkType == WDRV_NETWORK_TYPE_SOFT_AP)
        return true;
    else
        return false;
}

static void _prescan_option_set(bool scan)
{
    if (scan) {
        s_prescan_allowed = true;
        s_prescan_inprogress = false;
    } else {
        s_prescan_allowed = false;
        s_prescan_inprogress = false;
    }
}

static void _config_write(void *wifi_cfg)
{
    memcpy(gp_wdrv_cfg, wifi_cfg, sizeof(WDRV_CONFIG));
}

static void _nettype_set(uint8_t netType)
{
    if (netType == WDRV_NETWORK_TYPE_INFRASTRUCTURE) {
        WDRV_EXT_CmdNetModeBSSSet();
    } else if (netType == WDRV_NETWORK_TYPE_SOFT_AP) {
        WDRV_EXT_CmdNetModeAPSet();
    }
}

static IWPRIV_STATUS _mcastfilter_set(uint8_t *addr)
{
    if (WDRV_WINC1500_MulticastFilterSet(NULL, (TCPIP_MAC_ADDR *)addr) == TCPIP_MAC_RES_OK)
        return IWPRIV_READY;
    return IWPRIV_ERROR;
}

static void _prescan_start(void)
{
    s_prescan_inprogress = true;
    WDRV_ScanStart();
}

static void _scan_start(void)
{
    WDRV_DBG_INFORM_PRINT("Scan started...\r\n");
    WDRV_ScanStart();
}

void iwpriv_get(IWPRIV_CMD cmd, IWPRIV_GET_PARAM *param)
{
    switch (cmd) {
    case PRESCAN_OPTION_GET:
        param->scan.prescanAllowed = s_prescan_allowed;
        break;
    case PRESCAN_ISFINISHED_GET:
        param->scan.prescanFinished = _prescan_isfinished();
        break;
    case SCANSTATUS_GET:
        param->scan.scanStatus = _scanstatus_get();
        break;
    case SCANRESULT_GET:
        WDRV_EXT_ScanResultGet(param->scan.index, param->scan.result);
        break;
    case SCANRESULTS_COUNT_GET:
        param->scan.numberOfResults = g_wdrv_scanStatus.numberOfResults;
        break;
    case CONFIG_GET:
        memcpy(param->cfg.config, gp_wdrv_cfg, sizeof(WDRV_CONFIG));
        break;
    case SSID_GET:
        _ssid_get(param->ssid.ssid, &param->ssid.ssidLen);
        break;
    case NETWORKTYPE_GET:
        param->netType.type = gp_wdrv_cfg->networkType;
        break;
    case CLIENTINFO_GET:
        _leftclient_get(&param->clientInfo.updated, param->clientInfo.addr);
        break;
    case CONNSTATUS_GET:
        param->conn.status = _connstatus_get();
        break;
    case DEVICEINFO_GET:
        ((WDRV_DEVICE_INFO *)param->devInfo.info)->deviceType = WDRV_ATWINC1500_DEVICE;
        break;
    case DRVSTATUS_GET:
        param->driverStatus.isOpen = _drvstatus_get();
        break;
    case FWUPGRADEREQUEST_GET:
        param->fwUpgrade.requested = g_wdrv_priv.isOtaFwUpdateRequested;
        break;
    case OPERATIONMODE_GET:
        param->opMode.isServer = _is_servermode();
        break;
    default:
        WDRV_ASSERT(false, "Invalid iwpriv get command");
        break;
    }
}

void iwpriv_set(IWPRIV_CMD cmd, IWPRIV_SET_PARAM *param)
{
    switch (cmd) {
    case PRESCAN_OPTION_SET:
        _prescan_option_set(param->scan.prescanAllowed);
        break;
    case CONFIG_SET:
        _config_write(param->cfg.config);
        break;
    case SSID_SET:
        WDRV_EXT_CmdSSIDSet(param->ssid.ssid, param->ssid.ssidLen);
        break;
    case NETWORKTYPE_SET:
        _nettype_set(param->netType.type);
        break;
    case INITCONN_OPTION_SET:
        g_wdrv_priv.initConn = param->conn.initConnAllowed;
        break;
    case MULTICASTFILTER_SET:
        param->multicast.status = _mcastfilter_set(param->multicast.addr);
        break;
    case POWERSAVE_SET:
        if (param->powerSave.enabled)
            WDRV_EXT_CmdPowerSavePut(true, 1, 3);
        else
            WDRV_EXT_CmdPowerSavePut(false, 0, 0);
        break;
    default:
        WDRV_ASSERT(false, "Invalid iwpriv set command");
        break;
    }
}

void iwpriv_execute(IWPRIV_CMD cmd, IWPRIV_EXECUTE_PARAM *param)
{
    switch (cmd) {
    case PRESCAN_START:
        _prescan_start();
        break;
    case SCAN_START:
        _scan_start();
        break;
    default:
        WDRV_ASSERT(false, "Invalid iwpriv execute command");
        break;
    }
}

// DOM-IGNORE-END
