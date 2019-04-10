/*******************************************************************************
  WINC1500 Wireless Driver Connection Manager Implementation

  File Name:
    wdrv_winc1500_connmgr.c

  Summary:
    Connection manager for WINC1500 wireless driver.

  Description:
    Connection manager for WINC1500 wireless driver.
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

#include "driver/wifi/winc1500/include/wdrv_winc1500_api.h"
#include "driver/wifi/winc1500/wireless_driver/include/wdrv_winc1500_main.h"

#define DISCONNECT_DONE_NOTIFY() WDRV_SEM_GIVE(&g_wdrv_priv.disconnectDoneSync)

static bool s_logicalConnection = false;
static WDRV_CONNECTION_STATES s_ConnectionState = WDRV_CONNECTION_STATE_NOT_CONNECTED;

static void ConnectionStateSet(bool state);
static void ConnectionStateUpdate(bool connected);

bool isClientCacheUpdated(bool *connected, uint8_t *mac)
{
    if (g_wdrv_priv.clientCache.updated) {
        int i;
        for (i = 0; i <  WDRV_MAX_CLIENT_TABLE_SLOTS; ++i) {
            if (g_wdrv_priv.clientCache.updated & 1 << i) {
                *connected = g_wdrv_priv.clientCache.bitMap & 1 << i ? true: false;
                memcpy(mac, g_wdrv_priv.clientCache.mac[i].addr, 6 * sizeof(uint8_t));
                g_wdrv_priv.clientCache.updated &= ~(i << i);
                return true;
            }
        }
    }
    return false;
}

void ClientCacheUpdate(bool connected, const uint8_t *const mac)
{
    int i;
    int idx = 0;

    if (connected) {
        /* Check if the MAC address is already in the table. If so, we just update timestamp and return. */
        if (g_wdrv_priv.clientCache.bitMap) {
            for (i = 0; i <  WDRV_MAX_CLIENT_TABLE_SLOTS; ++i) {
                if (g_wdrv_priv.clientCache.bitMap & 1 << i) {
                    if (!memcmp(g_wdrv_priv.clientCache.mac[i].addr, mac, 6)) {
                        g_wdrv_priv.clientCache.mac[i].timeStamp = g_wdrv_priv.clientCache.seqNum++;
                        return;
                    }
                }
            }
        }

        /* Try to find an empty slot in the table. */
        for (i = 0; i <  WDRV_MAX_CLIENT_TABLE_SLOTS; ++i) {
            if (!(g_wdrv_priv.clientCache.bitMap & 1 << i)) {
                idx = i;
                g_wdrv_priv.clientCache.bitMap |= 1 << idx;
                g_wdrv_priv.clientCache.updated |= 1 << idx;
                memcpy(g_wdrv_priv.clientCache.mac[idx].addr, mac, 6);
                g_wdrv_priv.clientCache.mac[idx].timeStamp = g_wdrv_priv.clientCache.seqNum++;
                return;
            }
        }

        /* Cache table is full. Let's kick out the oldest. */
        for (i = 0; i <  WDRV_MAX_CLIENT_TABLE_SLOTS; ++i) {
            uint32_t min = 0;
            if (g_wdrv_priv.clientCache.mac[i].timeStamp >= min) {
                min = g_wdrv_priv.clientCache.mac[i].timeStamp;
                idx = i;
            }
        }
        g_wdrv_priv.clientCache.bitMap |= 1 << idx;
        g_wdrv_priv.clientCache.updated |= 1 << idx;
        memcpy(g_wdrv_priv.clientCache.mac[idx].addr, mac, 6);
        g_wdrv_priv.clientCache.mac[idx].timeStamp = g_wdrv_priv.clientCache.seqNum++;
        return;
    } else {
        /* If the MAC address is in the table, update its status to unconnected. */
        for (i = 0; i <  WDRV_MAX_CLIENT_TABLE_SLOTS; ++i) {
            if (g_wdrv_priv.clientCache.bitMap & 1 << i) {
                if (!memcmp(mac, g_wdrv_priv.clientCache.mac[i].addr, 6)) {
                    g_wdrv_priv.clientCache.bitMap &= ~(1 << i);
                    g_wdrv_priv.clientCache.updated |= 1 << i;
                    return;
                }
            }
        }
    }
}

void ConnectEventCB(bool connected, bool isServer, const uint8_t *const client)
{
    static uint8_t mac[6];

    if (gp_wdrv_cfg->networkType == WDRV_NETWORK_TYPE_INFRASTRUCTURE) {
        ConnectionStateUpdate(connected);
    } else if (gp_wdrv_cfg->networkType == WDRV_NETWORK_TYPE_SOFT_AP) {
        if (isServer) {
            ConnectionStateUpdate(connected);
        } else {
            if (connected) {
                ClientCacheUpdate(connected, client);
                WDRV_DBG_INFORM_PRINT("Client %02x:%02x:%02x:%02x:%02x:%02x is connected\r\n", client[0], client[1], client[2], client[3], client[4], client[5]);
                memcpy(mac, client, sizeof(mac));
            } else {
                ClientCacheUpdate(connected, mac);
                WDRV_DBG_INFORM_PRINT("Client %02x:%02x:%02x:%02x:%02x:%02x has left\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            }
        }
    }
}

bool isLinkUp(void)
{
    return s_logicalConnection;
}

static void ConnectionStateSet(bool state)
{
    s_logicalConnection = state;
}

static void NetModeSet(uint8_t networkType)
{
    switch (networkType) {
    case WDRV_NETWORK_TYPE_INFRASTRUCTURE:
        WDRV_EXT_CmdNetModeBSSSet();
        break;
    case WDRV_NETWORK_TYPE_SOFT_AP:
        WDRV_EXT_CmdNetModeAPSet();
        break;
    default:
        WDRV_ASSERT(false, "Undefined network type");
        break;
    }
}

static void SecuritySet(uint8_t securityMode)
{
    bool pinMode;

    switch (securityMode) {
    case WDRV_SECURITY_OPEN:
        WDRV_EXT_CmdSecNoneSet();
        break;
    case WDRV_SECURITY_WEP_40:
    case WDRV_SECURITY_WEP_104:
        if ((gp_wdrv_cfg->securityKeyLen == 5) || (gp_wdrv_cfg->securityKeyLen == 13)) {
            int i;
            uint8_t buf[26] = {0};
            uint8_t c_value;
            for (i = 0; i < gp_wdrv_cfg->securityKeyLen * 2; i++) {
                c_value = (i % 2 == 0 ) ? (gp_wdrv_cfg->securityKey[i / 2] >> 4) : (gp_wdrv_cfg->securityKey[i / 2] & 0x0F);
                buf[i] = (c_value > 9) ? ('A' + c_value - 0x0A) : ('0' + c_value - 0x00);
            }
            gp_wdrv_cfg->securityKeyLen *= 2;
            memcpy(gp_wdrv_cfg->securityKey, buf, gp_wdrv_cfg->securityKeyLen);
            gp_wdrv_cfg->securityKey[gp_wdrv_cfg->securityKeyLen] = 0x00;
        }
        WDRV_EXT_CmdSecWEPSet(gp_wdrv_cfg->securityKey, gp_wdrv_cfg->securityKeyLen);
        break;
    case WDRV_SECURITY_WPA_AUTO_WITH_PASS_PHRASE:
        WDRV_EXT_CmdSecWPASet(gp_wdrv_cfg->securityKey, gp_wdrv_cfg->securityKeyLen);
        break;
    case WDRV_SECURITY_WPS_PIN:
    case WDRV_SECURITY_WPS_PUSH_BUTTON:
        pinMode = gp_wdrv_cfg->securityMode == WDRV_SECURITY_WPS_PIN ? true : false;
        WDRV_EXT_CmdSecWpsSet(pinMode, gp_wdrv_cfg->securityKey, gp_wdrv_cfg->securityKeyLen);
        break;
    default:
        WDRV_ASSERT(false, "Undefined security mode");
        break;
    }
}

void WDRV_Connect(void)
{
    NetModeSet(gp_wdrv_cfg->networkType);
    if (gp_wdrv_cfg->networkType == WDRV_NETWORK_TYPE_SOFT_AP)
        WDRV_EXT_CmdChannelSet(WDRV_DEFAULT_CHANNEL);
    WDRV_EXT_CmdSSIDSet(gp_wdrv_cfg->ssid, gp_wdrv_cfg->ssidLen);
    SecuritySet(gp_wdrv_cfg->securityMode);
    WDRV_EXT_CmdPowerSavePut(false, 0, 0);

    if ((gp_wdrv_cfg->securityMode != WDRV_SECURITY_WPS_PIN) &&
        (gp_wdrv_cfg->securityMode != WDRV_SECURITY_WPS_PUSH_BUTTON)) {
        WDRV_DBG_INFORM_PRINT("\r\nStart Wi-Fi Connection...\r\n");
        g_wdrv_priv.isDisconnectRequested = false;
        s_ConnectionState = WDRV_CONNECTION_STATE_IN_PROGRESS;
        WDRV_EXT_CmdConnect();
    }
}

void WDRV_Disconnect(void)
{
    g_wdrv_priv.isDisconnectRequested = true;
    WDRV_EXT_CmdDisconnect();
}

void WPSDoneCB(void)
{
    uint32_t status = WDRV_ERROR;

    WDRV_EXT_WPSResultsRead(gp_wdrv_cfg, &status);

    if (status == WDRV_SUCCESS) {
        WDRV_DBG_INFORM_PRINT("WPS process complete\r\n");
        WDRV_CONFIG_Save();
    } else {
        WDRV_DBG_INFORM_PRINT("WPS process failed, retry...\r\n");
    }
    WDRV_Connect();
}

static void ConnectionStateUpdate(bool connected)
{
    ConnectionStateSet(connected);
    if (connected) {
        s_ConnectionState = WDRV_CONNECTION_STATE_CONNECTED;
    } else {
        if (g_wdrv_priv.isDisconnectRequested)
            DISCONNECT_DONE_NOTIFY();
        s_ConnectionState = WDRV_CONNECTION_STATE_NOT_CONNECTED;
    }
}

WDRV_CONNECTION_STATES WDRV_ConnectionState_Get(void)
{
    return s_ConnectionState;
}

//DOM-IGNORE-END
