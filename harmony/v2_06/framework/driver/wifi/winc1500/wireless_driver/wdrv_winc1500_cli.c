/*******************************************************************************
  WINC1500 CLI (Based on System Cmder) Implementation

  File Name:
    wdrv_winc1500_cli.c

  Summary:
    CLI (based on System Cmder) implementation for WINC1500 wireless driver.

  Description:
    CLI (based on System Cmder) implementation for WINC1500 wireless driver.
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

#include "system_config.h"

#include "system/common/sys_module.h"

#include "system/command/sys_command.h"

#include "driver/wifi/winc1500/include/wdrv_winc1500_api.h"
#include "driver/wifi/winc1500/wireless_driver/include/wdrv_winc1500_main.h"

#if TCPIP_STACK_COMMANDS_WIFI_ENABLE

// iwconfig control block
typedef struct
{
    uint8_t powerSaveState; // power save state
    uint8_t connState; // connection state
    bool isIdle; // true if connState is WDRV_CONNECTION_STATE_NOT_CONNECTED
} WDRV_IWCONFIG_CB;

static struct
{
    struct {
        uint8_t name[WDRV_MAX_SSID_LENGTH + 1];
        uint8_t len;
    } SSID;

    uint8_t NetworkType;

    uint16_t Channel;

    uint8_t SecurityType;

    uint8_t MAC[6];

    WDRV_CONNECTION_CONTEXT Context;
} s_ctx;

/*
 * Have to use SYS_CONSOLE_PRINT instead of WDRV_DBG_INFORM_PRINT here.
 * Because g_debugConsoleLock is not available when module is down.
 */
#define check_module_up() if (!g_wdrv_priv.isDriverOpen) \
    {SYS_CONSOLE_PRINT("WINC1500 is down - command cannot work\r\n"); return false;}

static WDRV_IWCONFIG_CB s_IwconfigCb;
static bool s_IwconfigCbInitialized = false;
static char s_helpInfo[800];

static bool IwconfigUpdateCb(SYS_CMD_DEVICE_NODE *pCmdIO);
static void IwconfigDisplayHelp(SYS_CMD_DEVICE_NODE *pCmdIO);
static void IwconfigDisplayStatus(SYS_CMD_DEVICE_NODE *pCmdIO);
static bool IwconfigSetSsid(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv);
static bool IwconfigSetMode(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv);
static bool IwconfigSetChannel(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv);
static bool IwconfigSetSecurity(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv);
static bool IwconfigSetPower(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv);
static int Cmd_Iwconfig(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv);
static int Cmd_GetMacAddr(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv);
static int Cmd_LoadConfig(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv);
static int Cmd_SaveConfig(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv);
static int Cmd_DeleteConfig(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv);
static int Cmd_Ota(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv);

static const SYS_CMD_DESCRIPTOR wfCmdTbl[] =
{
    {"iwconfig",   Cmd_Iwconfig,     ": Wi-Fi configuration"},
    {"mac",        Cmd_GetMacAddr,   ": get MAC address"},
    {"loadconf",   Cmd_LoadConfig,   ": load configuration data from memory"},
    {"saveconf",   Cmd_SaveConfig,   ": save current configuration data to memory"},
    {"deleteconf", Cmd_DeleteConfig, ": delete stored configuration data in memory"},
    {"ota",        Cmd_Ota,          ": upgrade WINC1500 FW over the air"},
};

static void WDRV_CLI_SsidGet(uint8_t *p_ssid, uint8_t *p_ssidLen)
{
    memcpy(p_ssid, gp_wdrv_cfg->ssid, gp_wdrv_cfg->ssidLen);
    p_ssid[gp_wdrv_cfg->ssidLen] = 0;
    *p_ssidLen = gp_wdrv_cfg->ssidLen;
}

static void WDRV_CLI_SsidSet(uint8_t *p_ssid, uint8_t ssidLen)
{
    if (ssidLen <= 32) {
        memcpy(gp_wdrv_cfg->ssid, p_ssid, ssidLen);
        gp_wdrv_cfg->ssid[ssidLen] = 0;
        gp_wdrv_cfg->ssidLen = ssidLen;
    } else {
        WDRV_DBG_ERROR_PRINT("Invalid SSID length %d\r\n", ssidLen);
    }
}

static void WDRV_CLI_NetworkTypeGet(uint8_t *p_networkType)
{
    *p_networkType = gp_wdrv_cfg->networkType;
}

static void WDRV_CLI_ChannelSet(uint8_t channel)
{
    WDRV_EXT_CmdChannelSet(channel);
}

static void WDRV_CLI_SecurityTypeGet(uint8_t *p_securityType)
{
    *p_securityType = gp_wdrv_cfg->securityMode;
}

static void WDRV_CLI_MacAddressGet(uint8_t *p_mac)
{
    WDRV_EXT_CmdMacAddressGet(p_mac);
}

static void WDRV_CLI_PowerSaveStateSet(bool enable)
{
    if (enable)
        WDRV_EXT_CmdPowerSavePut(true, 1, 3);
    else
        WDRV_EXT_CmdPowerSavePut(false, 0, 0);
}

static void WDRV_CLI_ConnectionStateGet(uint8_t *p_state)
{
    *p_state = WDRV_ConnectionState_Get();
}

static void WDRV_CLI_ConnectContextBssidGet(uint8_t *bssid)
{
    if (gp_wdrv_cfg->networkType == WDRV_NETWORK_TYPE_INFRASTRUCTURE)
        WDRV_EXT_CmdConnectContextBssidGet(bssid);
    else
        WDRV_DBG_INFORM_PRINT("Cannot get BSSID in SoftAP mode\r\n");
}

static void WDRV_CLI_Connect(void)
{
    WDRV_Connect();
}

static void WDRV_CLI_Disconnect(void)
{
    WDRV_Disconnect();
}

static void WDRV_CLI_SecNoneSet(void)
{
    gp_wdrv_cfg->securityMode = WDRV_SECURITY_OPEN;
}

static void WDRV_CLI_SecWEPSet(uint8_t *key, uint8_t len)
{
    if (len == 10) {
        gp_wdrv_cfg->securityMode = WDRV_SECURITY_WEP_40;
        gp_wdrv_cfg->securityKeyLen = len;
        memcpy(gp_wdrv_cfg->securityKey, key, len);
    } else if (len == 26) {
        gp_wdrv_cfg->securityMode = WDRV_SECURITY_WEP_104;
        gp_wdrv_cfg->securityKeyLen = len;
        memcpy(gp_wdrv_cfg->securityKey, key, len);
    } else {
        WDRV_DBG_ERROR_PRINT("Invalid WEP key length %d\r\n", len);
    }
}

static void WDRV_CLI_SecWPASet(uint8_t *key, uint8_t len)
{
    if ((len >= 8) && (len <= 63)) {
        gp_wdrv_cfg->securityMode = WDRV_SECURITY_WPA_AUTO_WITH_PASS_PHRASE;
        gp_wdrv_cfg->securityKeyLen = len;
        memcpy(gp_wdrv_cfg->securityKey, key, len);
    } else {
        WDRV_DBG_ERROR_PRINT("Invalid WPA passphrase length %d\r\n", len);
    }
}

static void WDRV_CLI_SecWPSPINSet(uint8_t *key, uint16_t len)
{
    if (len == 8) {
        gp_wdrv_cfg->securityMode = WDRV_SECURITY_WPS_PIN;
        gp_wdrv_cfg->securityKeyLen = len;
        memcpy(gp_wdrv_cfg->securityKey, key, len);
    } else {
        WDRV_DBG_ERROR_PRINT("Invalid PIN length %d\r\n", len);
    }
}

static void WDRV_CLI_SecWPSPushButtonSet(void)
{
    gp_wdrv_cfg->securityMode = WDRV_SECURITY_WPS_PUSH_BUTTON;
}

static void WDRV_CLI_ScanResultDisplay(uint8_t idx)
{
    WDRV_SCAN_RESULT scanResult;

    WDRV_EXT_ScanResultGet(idx, &scanResult);

    WDRV_DBG_INFORM_PRINT("\r\n======================\r\n");
    WDRV_DBG_INFORM_PRINT("Scan Result  %d\r\n", idx + 1);

    // SSID
    WDRV_DBG_INFORM_PRINT("SSID:     %s", scanResult.ssid);

    // BSSID
    WDRV_DBG_INFORM_PRINT("\r\nBSSID:    %02X:%02X:%02X:%02X:%02X:%02X\r\n",
        scanResult.bssid[0], scanResult.bssid[1],
        scanResult.bssid[2], scanResult.bssid[3],
        scanResult.bssid[4], scanResult.bssid[5]);

    // Channel
    WDRV_DBG_INFORM_PRINT("Channel:  %d\r\n", scanResult.channel);

    // Security
    WDRV_DBG_INFORM_PRINT("Security: ");
    if (scanResult.apConfig & WDRV_APCONFIG_BIT_PRIVACY) { // if privacy bit is set
        if ((scanResult.apConfig & WDRV_APCONFIG_BIT_WPA2) ||
            (scanResult.apConfig & WDRV_APCONFIG_BIT_WPA)) { // if WPA bit is set
            WDRV_DBG_INFORM_PRINT("WPA\r\n");
        } else {
            WDRV_DBG_INFORM_PRINT("WEP\r\n");
        }
    } else {
        WDRV_DBG_INFORM_PRINT("Open\r\n");
    }

    // RSSI
    WDRV_DBG_INFORM_PRINT("RSSI:     %d dBm\r\n", scanResult.rssi);
}

static bool IwconfigUpdateCb(SYS_CMD_DEVICE_NODE *pCmdIO)
{
    if (!s_IwconfigCbInitialized) { // first time call of IwconfigUpdateCb
        memset(&s_IwconfigCb, 0, sizeof(s_IwconfigCb));
        s_IwconfigCbInitialized = true;
    }

    WDRV_CLI_ConnectionStateGet(&s_IwconfigCb.connState);
    if (s_IwconfigCb.connState == WDRV_CONNECTION_STATE_NOT_CONNECTED)
        s_IwconfigCb.isIdle = true;
    else
        s_IwconfigCb.isIdle = false;

    return true;
}

static void IwconfigDisplayHelp(SYS_CMD_DEVICE_NODE *pCmdIO)
{
    strcpy(s_helpInfo, "Usage:\r\n  iwconfig\r\n");
    strcat(s_helpInfo, "  iwconfig ssid <ssid>\r\n");
    strcat(s_helpInfo, "  iwconfig mode <mode>\r\n");
    strcat(s_helpInfo, "  iwconfig security <security_mode> <key>/<pin>\r\n");
    strcat(s_helpInfo, "  iwconfig power <enable/disable>\r\n");
    strcat(s_helpInfo, "  iwconfig scan\r\n");
    strcat(s_helpInfo, "  iwconfig scanget <scan_result_index>\r\n");

    strcat(s_helpInfo, "<ssid>:\r\n");
    strcat(s_helpInfo, "  32 characters string - no blank or space allowed in this demo\r\n");
    strcat(s_helpInfo, "<mode>:\r\n");
    strcat(s_helpInfo, "  managed/idle\r\n");
    strcat(s_helpInfo, "<security_mode>:\r\n");
    strcat(s_helpInfo, "  open/wep40/wep104/wpa/pbc/pin\r\n");
    strcat(s_helpInfo, "  No blank or space allowed in <key> in current console commands\r\n");
    strcat(s_helpInfo, "  Ex: iwconfig security open\r\n");
    strcat(s_helpInfo, "      iwconfig security wep40 5AFB6C8E77\r\n");
    strcat(s_helpInfo, "      iwconfig security wep104 90E96780C739409DA50034FCAA\r\n");
    strcat(s_helpInfo, "      iwconfig security wpa microchip_psk\r\n");
    strcat(s_helpInfo, "      iwconfig security pbc\r\n");
    strcat(s_helpInfo, "      iwconfig security pin 12390212\r\n");

    //WDRV_DBG_INFORM_PRINT("Length of s_helpInfo is: %u\r\n", strlen(s_helpInfo));

    WDRV_DBG_INFORM_PRINT("%s", s_helpInfo);
}

static void IwconfigDisplayStatus(SYS_CMD_DEVICE_NODE *pCmdIO)
{
    WDRV_DBG_INFORM_PRINT("Current Wi-Fi configuration:\r\n");

    // Mode
    WDRV_CLI_NetworkTypeGet(&s_ctx.NetworkType);
    WDRV_DBG_INFORM_PRINT("\tMode:      ");
    if (s_IwconfigCb.isIdle) {
        if (s_IwconfigCb.connState == WDRV_CONNECTION_STATE_NOT_CONNECTED) {
            WDRV_DBG_INFORM_PRINT("Idle");
        } else {
            WDRV_DBG_INFORM_PRINT("Idle (undefined)");
        }
    } else {
        if (s_IwconfigCb.connState == WDRV_CONNECTION_STATE_CONNECTED) {
            WDRV_DBG_INFORM_PRINT("Managed (connected)");
        } else if (s_IwconfigCb.connState == WDRV_CONNECTION_STATE_IN_PROGRESS) {
            WDRV_DBG_INFORM_PRINT("Managed (connection in progress)");
        } else  {
            WDRV_DBG_INFORM_PRINT("Managed (undefined)");
        }
    }

    // SSID
    WDRV_CLI_SsidGet(s_ctx.SSID.name, &s_ctx.SSID.len);
    s_ctx.SSID.name[s_ctx.SSID.len] = '\0';
    WDRV_DBG_INFORM_PRINT("\r\n\tSSID:      %s\r\n", s_ctx.SSID.name);

	// BSSID (Only Valid in Infrastructure Mode)
	if (s_ctx.NetworkType == WDRV_NETWORK_TYPE_INFRASTRUCTURE) {
		WDRV_CLI_ConnectContextBssidGet(s_ctx.Context.bssid);
		WDRV_DBG_INFORM_PRINT("\tBSSID:     %02X:%02X:%02X:%02X:%02X:%02X\r\n",
			s_ctx.Context.bssid[0], s_ctx.Context.bssid[1], s_ctx.Context.bssid[2],
			s_ctx.Context.bssid[3], s_ctx.Context.bssid[4], s_ctx.Context.bssid[5]);
	}

    // Network Type
    WDRV_DBG_INFORM_PRINT("\tNetwork:   ");

    if (s_ctx.NetworkType == WDRV_NETWORK_TYPE_SOFT_AP) {
        WDRV_DBG_INFORM_PRINT("SoftAP\r\n");
    } else if (s_ctx.NetworkType == WDRV_NETWORK_TYPE_INFRASTRUCTURE) {
        WDRV_DBG_INFORM_PRINT("Infrastructure\r\n");
    } else {
        WDRV_DBG_INFORM_PRINT("Invalid\r\n");
    }

    // Security Type
    WDRV_DBG_INFORM_PRINT("\tSecurity:  ");
    WDRV_CLI_SecurityTypeGet(&s_ctx.SecurityType);
    switch (s_ctx.SecurityType) {
        case WDRV_SECURITY_OPEN:
            WDRV_DBG_INFORM_PRINT("Open\r\n");
            break;
        case WDRV_SECURITY_WEP_40:
        case WDRV_SECURITY_WEP_104:
            if (s_ctx.SecurityType == WDRV_SECURITY_WEP_40) {
                WDRV_DBG_INFORM_PRINT("WEP40\r\n");
            } else if (s_ctx.SecurityType == WDRV_SECURITY_WEP_104) {
                WDRV_DBG_INFORM_PRINT("WEP104\r\n");
            } else {
                WDRV_DBG_INFORM_PRINT("Invalid WEP mode\r\n");
            }
            break;
        case WDRV_SECURITY_WPA_AUTO_WITH_PASS_PHRASE:
            WDRV_DBG_INFORM_PRINT("WPA-PSK/WPA2-PSK\r\n");
            break;
        case WDRV_SECURITY_WPS_PIN:
            WDRV_DBG_INFORM_PRINT("WPS PIN method\r\n");
            break;
        case WDRV_SECURITY_WPS_PUSH_BUTTON:
            WDRV_DBG_INFORM_PRINT("WPS push button method\r\n");
            break;
        default:
            WDRV_DBG_INFORM_PRINT("Invalid security setting\r\n");
    }

    // MAC
    WDRV_CLI_MacAddressGet(s_ctx.MAC);
    WDRV_DBG_INFORM_PRINT("\tMAC:       ");
    WDRV_DBG_INFORM_PRINT("%02X:%02X:%02X:%02X:%02X:%02X\r\n",
        s_ctx.MAC[0],s_ctx.MAC[1],s_ctx.MAC[2],s_ctx.MAC[3],s_ctx.MAC[4],s_ctx.MAC[5]);
}

static bool IwconfigSetSsid(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv)
{
    if (argc < 3) {
        WDRV_DBG_INFORM_PRINT("Missing value for last parameter\r\n");
        return false;
    }

    if (argc > 3) {
        WDRV_DBG_INFORM_PRINT("SSID cannot contain space character in this demo\r\n");
        return false;
    }

    WDRV_CLI_SsidSet((uint8_t *)argv[2], strlen((char *)argv[2]));

    return true;
}

static bool IwconfigSetMode(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv)
{
    uint8_t networkType;

    WDRV_CLI_NetworkTypeGet(&networkType);

    if ((argc >= 3) && (strcmp((char *)argv[2], "idle") == 0)) {
        if (s_IwconfigCb.isIdle) {
            WDRV_DBG_INFORM_PRINT("Already in Idle mode\r\n");
        } else {
            g_wdrv_priv.isDisconnectRequested = true;
            WDRV_CLI_Disconnect();
        }
    } else if ((argc >= 3) && (strcmp((char *)argv[2], "managed") == 0)) {
        if (s_IwconfigCb.isIdle) {
            WDRV_CLI_Connect();
        } else {
            WDRV_DBG_INFORM_PRINT("Already in Managed mode\r\n");
        }
    } else {
        WDRV_DBG_INFORM_PRINT("Unknown parameter\r\n");
        return false;
    }

    return true;
}

static bool IwconfigSetChannel(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv)
{
    if (argc < 3) {
        WDRV_DBG_INFORM_PRINT("No channel number entered\r\n");
        return false;
    }

    if (!s_IwconfigCb.isIdle) {
        WDRV_DBG_INFORM_PRINT("Channel can only be set in Idle mode\r\n");
        return false;
    }

    WDRV_CLI_ChannelSet((uint16_t)(*argv[1]));

    return true;
}

static bool IwconfigSetSecurity(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv)
{
    if (!s_IwconfigCb.isIdle) {
        WDRV_DBG_INFORM_PRINT("Security mode can only be set in Idle mode\r\n");
        return false;
    }

    if ((argc >= 3) && (strcmp((char *)argv[2], "open") == 0)) {
        WDRV_CLI_SecNoneSet();
    } else if ((argc >= 4) && (strcmp((char *)argv[2], "wep40") == 0)) {
        if (strlen((char *)argv[3]) != 10) {
            WDRV_DBG_INFORM_PRINT("WEP40 key length error\r\n");
            return false;
        }
        WDRV_CLI_SecWEPSet((uint8_t *)argv[3], strlen((char *)argv[3]));
    } else if ((argc >= 4) && (strcmp((char *)argv[2], "wep104") == 0)) {
        if (strlen((char *)argv[3]) != 26) {
            WDRV_DBG_INFORM_PRINT("WEP104 key length error\r\n");
            return false;
        }
        WDRV_CLI_SecWEPSet((uint8_t *)argv[3], strlen((char *)argv[3]));
    } else if ((argc >= 4) && (strcmp((char *)argv[2], "wpa") == 0)) {
        if ((strlen((char *)argv[3]) < 8) || (strlen((char *)argv[3]) > 63)) {
            WDRV_DBG_INFORM_PRINT("WPA pass phrase length error\r\n");
            return false;
        }
        WDRV_CLI_SecWPASet((uint8_t *)argv[3], strlen((char *)argv[3]));
    } else if ((argc >= 3) && (strcmp((char *)argv[2], "pbc") == 0)) {
        WDRV_CLI_SecWPSPushButtonSet();
    } else if ((argc >= 4) && (strcmp((char *)argv[2], "pin") == 0)) {
        if ((strlen((char *)argv[3]) != 8)) {
            WDRV_DBG_INFORM_PRINT("WPS PIN length error\r\n");
            return false;
        }
        WDRV_CLI_SecWPSPINSet((uint8_t *)argv[3], strlen((char *)argv[3]));
    } else {
        WDRV_DBG_INFORM_PRINT("Unknown security mode or wrong parameters were typed\r\n");
        WDRV_DBG_INFORM_PRINT("Usage: \r\n");
        WDRV_DBG_INFORM_PRINT("1. open : iwconfig security open\r\n");
        WDRV_DBG_INFORM_PRINT("2. wep40 : iwconfig security wep40 <key>\r\n");
        WDRV_DBG_INFORM_PRINT("3. wep104 : iwconfig security wep104 <key>\r\n");
        WDRV_DBG_INFORM_PRINT("4. wpa : iwconfig security wpa <key>\r\n");
        WDRV_DBG_INFORM_PRINT("5. wps pbc : iwconfig security pbc\r\n");
        WDRV_DBG_INFORM_PRINT("6. wps pin : iwconfig security pin <pin>\r\n");
        WDRV_DBG_INFORM_PRINT("Try \"iwconfig --help\" or \"iwconfig -h\" for more information\r\n");
        return false;
    }

    return true;
}

static bool IwconfigSetPower(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv)
{
    if (argc < 3) {
        WDRV_DBG_INFORM_PRINT("Missing value for last parameter\r\n");
        return false;
    }

    if ((argc >= 3) && (strcmp((char *)argv[2], "enable") == 0)) {  // re-enable power saving
        WDRV_CLI_PowerSaveStateSet(true);
    } else if ((argc >= 3) && (strcmp((char *)argv[2], "disable") == 0)) {  // disable power saving
        WDRV_CLI_PowerSaveStateSet(false);
    } else {
        WDRV_DBG_INFORM_PRINT("Unknown parameter\r\n");
        return false;
    }

    return true;
}

static int Cmd_Ota(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv)
{
    check_module_up();

    if (argc < 2) {
        WDRV_DBG_INFORM_PRINT("Missing URL. Type, for instance, ota http://192.168.0.4/winc1500_ota.bin\r\n");
        return WDRV_ERROR;
    }

    WDRV_DBG_INFORM_PRINT("echoing target URL: %s\r\n", argv[1]);
    g_wdrv_priv.isOtaFwUpdateRequested = true;
    WDRV_ASSERT(strlen(argv[1]) <= sizeof(g_wdrv_priv.fwOtaServerUrl) - 1, "URL is too long");
    strcpy((void *)g_wdrv_priv.fwOtaServerUrl, argv[1]);

    return WDRV_SUCCESS;
}

static int Cmd_Iwconfig(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv)
{
    check_module_up();

    if (!IwconfigUpdateCb(pCmdIO)) {
        WDRV_DBG_INFORM_PRINT("s_IwconfigCb structure set failed\r\n");
        return false;
    }

    // if user only typed in iwconfig with no other parameters
    if (argc == 1) {
        IwconfigDisplayStatus(pCmdIO);
        WDRV_DBG_INFORM_PRINT("Try \"iwconfig --help\" or \"iwconfig -h\" for more information\r\n");
        return true;
    }

    if ((argc == 2) && (strcmp((char *)argv[1], "-h") == 0 || strcmp((char *)argv[1], "--help") == 0)) {
        IwconfigDisplayHelp(pCmdIO);
    } else if ((argc >= 2) && (strcmp((char *)argv[1], "ssid") == 0)) {
        return IwconfigSetSsid(pCmdIO, argc, argv);
    } else if ((argc >= 2) && (strcmp((char *)argv[1], "mode") == 0)) {
       return IwconfigSetMode(pCmdIO, argc, argv);
    } else if ((argc >= 2) && (strcmp((char *)argv[1], "channel") == 0)) {
        return IwconfigSetChannel(pCmdIO, argc, argv);
    } else if ((argc >= 2) && (strcmp((char *)argv[1], "security") == 0)) {
        return IwconfigSetSecurity(pCmdIO, argc, argv);
    } else if ((argc >= 2) && (strcmp((char *)argv[1], "power") == 0)) {
        return IwconfigSetPower(pCmdIO, argc, argv);
    } else if ((argc == 2) && (strcmp((char *)argv[1], "scan") == 0)) {
        if (WDRV_ScanStart() != WDRV_SUCCESS) {
            WDRV_DBG_INFORM_PRINT("Scan failed\r\n");
            return false;
        } else {
            WDRV_DBG_INFORM_PRINT("Scan started...\r\n");
        }
    } else if ((argc >= 2) && (strcmp((char *)argv[1], "scanget") == 0)) {
        uint16_t scanResultIndex;

        if (g_wdrv_scanStatus.scanInProgress) {
            WDRV_DBG_INFORM_PRINT("Scan is still in progress, please wait...\r\n");
            return false;
        }

        if (argc != 3) {
            WDRV_DBG_INFORM_PRINT("Usage: iwconfig scanget <scan_result_index>\r\n");
            WDRV_DBG_INFORM_PRINT("Please provide <scan_result_index>\r\n");
            return false;
        }

        if (g_wdrv_scanStatus.numberOfResults == 0)
            WDRV_DBG_INFORM_PRINT("No scan was done, or no AP was found\r\n");
        else
            WDRV_DBG_INFORM_PRINT("Found %d AP(s)\r\n", g_wdrv_scanStatus.numberOfResults);

        scanResultIndex = atoi((char *)argv[2]);
        if (scanResultIndex == 0 || scanResultIndex > g_wdrv_scanStatus.numberOfResults) {
            if (g_wdrv_scanStatus.numberOfResults == 1) {
                WDRV_DBG_INFORM_PRINT("<scan_result_index> must be 1\r\n");
            } else if (g_wdrv_scanStatus.numberOfResults > 1) {
                WDRV_DBG_INFORM_PRINT("<scan_result_index> must be between 1 and %d\r\n", g_wdrv_scanStatus.numberOfResults);
            }
            return false;
        } else {
            WDRV_CLI_ScanResultDisplay(scanResultIndex - 1);
        }
    } else  {
        WDRV_DBG_INFORM_PRINT("Unknown parameter\r\n");
        return false;
    }

    return true;
}

static int Cmd_GetMacAddr(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv)
{
    uint8_t macAddr[6];

    check_module_up();

    memset(macAddr, 0x00, sizeof(macAddr));
    WDRV_CLI_MacAddressGet(macAddr);
    WDRV_DBG_INFORM_PRINT("MAC address: %02x:%02x:%02x:%02x:%02x:%02x\r\n", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
    return true;
}

static int Cmd_LoadConfig(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv)
{
    check_module_up();

    WDRV_CONFIG_Load();

    if (!IwconfigUpdateCb(pCmdIO)) {
        WDRV_DBG_INFORM_PRINT("s_IwconfigCb structure set failed\r\n");
        return false;
    }

    IwconfigDisplayStatus(pCmdIO);

    return true;
}

static int Cmd_SaveConfig(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv)
{
    check_module_up();

    WDRV_CONFIG_Save();
    return true;
}

static int Cmd_DeleteConfig(SYS_CMD_DEVICE_NODE *pCmdIO, int argc, char **argv)
{
    check_module_up();

    WDRV_CONFIG_Delete();
    return true;
}

bool WDRV_CLI_Init(void)
{
    if (SYS_CMD_ADDGRP(wfCmdTbl, sizeof(wfCmdTbl)/sizeof(*wfCmdTbl), "wifi", ": Wi-Fi commands") == -1)
        return false;
    return true;
}

#endif /* TCPIP_STACK_COMMANDS_WIFI_ENABLE */

//DOM-IGNORE-END
