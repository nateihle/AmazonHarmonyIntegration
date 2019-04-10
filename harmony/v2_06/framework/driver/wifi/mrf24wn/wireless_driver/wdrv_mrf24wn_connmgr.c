/*******************************************************************************
  MRF24WN Wireless Driver Connection Manager Implementation

  File Name:
    wdrv_mrf24wn_connmgr.c

  Summary:
    Connection manager for MRF24WN wireless driver.

  Description:
    Connection manager for MRF24WN wireless driver.
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

#include "system/tmr/sys_tmr.h"

#include "driver/wifi/mrf24wn/include/wdrv_mrf24wn_api.h"
#include "driver/wifi/mrf24wn/wireless_driver/include/wdrv_mrf24wn_main.h"

#define DISCONNECT_DONE_NOTIFY() WDRV_SemGive(&g_wdrv_priv.disconnectDoneSync)

static bool s_logicalConnection = false;
static WDRV_CONNECTION_STATES s_ConnectionState = WDRV_CSTATE_NOT_CONNECTED;
static char *s_connect_failure_reason[] = {
    "",
    "NO_NETWORK_AVAIL",
    "LOST_LINK",
    "DISCONNECT_CMD",
    "BSS_DISCONNECTED",
    "AUTH_FAILED",
    "ASSOC_FAILED",
    "NO_RESOURCES_AVAIL",
    "CONNECTION_DENIED",
    "",
    "INVALID_PROFILE",
    "",
    "PROFILE_MISMATCH",
    "CONNECTION_EVICTED",
};

static void ConnectionStateSet(bool state);
static void ConnectionStateUpdate(bool connected, uint8_t reason);

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

static void ClientCacheUpdate(bool connected, uint8_t *mac)
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

void ProceedConnectEventCB(uint32_t connected, uint8_t devID, uint8_t *mac, bool macConn, uint8_t reason)
{
    /*
     * The meaning of variable "mac" varies.
     * In Infrastructure mode, it most likely points to an AP's MAC address, which is actually
     *  the network BSSID.
     * In SoftAP mode, it sometimes points to a client's MAC address, but sometimes points to
     *  MRF24WN's own MAC address. It points to an all "0xFF" MAC address too in certain cases.
     */
    const uint8_t macAllFF[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    static bool softAPStarted = false;

    if (connected == true) {
        if (gp_wdrv_cfg->networkType == WDRV_NETWORK_TYPE_INFRASTRUCTURE) {
            ConnectionStateSet(true);
            ConnectionStateUpdate(connected, reason);
            WDRV_DBG_INFORM_PRINT("Connected to AP\r\n");
        } else if (gp_wdrv_cfg->networkType == WDRV_NETWORK_TYPE_SOFT_AP) {
            if (!softAPStarted) {
                softAPStarted = true;
                ConnectionStateSet(true);
                ConnectionStateUpdate(connected, reason);
                WDRV_DBG_INFORM_PRINT("SoftAP network is enabled\r\n");
            } else {
                ClientCacheUpdate(connected, mac);
                WDRV_DBG_INFORM_PRINT("A client is connected\r\n");
            }
        }
    } else if (connected == false) {
        if (gp_wdrv_cfg->networkType == WDRV_NETWORK_TYPE_INFRASTRUCTURE) {
            ConnectionStateSet(false);
            ConnectionStateUpdate(connected, reason);
            WDRV_DBG_INFORM_PRINT("Connection failed - %s\r\n", s_connect_failure_reason[reason]);
        } else if (gp_wdrv_cfg->networkType == WDRV_NETWORK_TYPE_SOFT_AP) {
            // if the MAC address pointer variable "mac" points to an all "0xFF" MAC address,
            // it means that the SoftAP network is already disabled
            if (memcmp(mac, macAllFF, 6) == 0) {
                softAPStarted = false;
                ConnectionStateSet(false);
                ConnectionStateUpdate(connected, reason);
                if (g_wdrv_priv.isDisconnectRequested)
                    DISCONNECT_DONE_NOTIFY();
                WDRV_DBG_INFORM_PRINT("SoftAP network is disabled\r\n");
            } else {
                ClientCacheUpdate(connected, mac);
                WDRV_DBG_INFORM_PRINT("A client has left\r\n");
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
    case WDRV_NETWORK_TYPE_ADHOC:
        WDRV_ASSERT(false, "Ad hoc is not supported for now");
        break;
    case WDRV_NETWORK_TYPE_SOFT_AP:
        WDRV_EXT_CmdNetModeAPSet();
        break;
    case WDRV_NETWORK_TYPE_P2P:
        WDRV_ASSERT(false, "P2P is not supported for now");
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
        WDRV_EXT_CmdSecWEPSet(gp_wdrv_cfg->securityKey);
        break;
    case WDRV_SECURITY_WPA_WITH_PASS_PHRASE:
        WDRV_EXT_CmdSecWPASet(gp_wdrv_cfg->securityKey, gp_wdrv_cfg->securityKeyLen);
        break;
    case WDRV_SECURITY_WPA2_WITH_PASS_PHRASE:
        WDRV_EXT_CmdSecWPA2Set(gp_wdrv_cfg->securityKey, gp_wdrv_cfg->securityKeyLen);
        break;
    case WDRV_SECURITY_WPS_PIN:
    case WDRV_SECURITY_WPS_PUSH_BUTTON:
        pinMode = gp_wdrv_cfg->securityMode == WDRV_SECURITY_WPS_PIN ? true : false;
        WDRV_EXT_CmdSecWPSSet(pinMode, gp_wdrv_cfg->securityKey);
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
    WDRV_EXT_CmdSSIDSet(gp_wdrv_cfg->ssid);
    SecuritySet(gp_wdrv_cfg->securityMode);
    WDRV_EXT_CmdPowerSaveSet(false);

    if (gp_wdrv_cfg->securityMode != WDRV_SECURITY_WPS_PIN &&
        gp_wdrv_cfg->securityMode != WDRV_SECURITY_WPS_PUSH_BUTTON) {
        WDRV_DBG_INFORM_PRINT("\r\nStart Wi-Fi Connection . . .\r\n");
        WDRV_EXT_CmdConnect();
        g_wdrv_priv.isDisconnectRequested = false;
        s_ConnectionState = WDRV_CSTATE_CONNECTION_IN_PROGRESS;
    }
}

void WDRV_Disconnect(bool requested)
{
    g_wdrv_priv.isDisconnectRequested = requested;
    WDRV_EXT_CmdDisconnect();
}

void WPSDoneCB(void)
{
    WDRV_EXT_WPSResultsRead(gp_wdrv_cfg);
    if (gp_wdrv_cfg->securityMode != WDRV_SECURITY_WPS_PUSH_BUTTON &&
        gp_wdrv_cfg->securityMode != WDRV_SECURITY_WPS_PIN) {
        WDRV_CONFIG_Save();
        WDRV_Connect();
    }
}

static void LinkDownTimeoutCallback(uintptr_t context, uint32_t currTick)
{
    if (s_ConnectionState == WDRV_CSTATE_CONNECTION_TEMPORARY_LOST) {
        s_ConnectionState = WDRV_CSTATE_CONNECTION_PERMANENTLY_LOST;
        WDRV_Disconnect(false);
        WDRV_DBG_INFORM_PRINT("Lost connection permanently\r\n");
    }
}

static void ConnectionStateUpdate(bool connected, uint8_t reason)
{
    static SYS_TMR_HANDLE timer = 0;
    uint16_t timeout;

    if (connected == true) {
        if ((gp_wdrv_cfg->securityMode == WDRV_SECURITY_WPA_WITH_PASS_PHRASE ||
             gp_wdrv_cfg->securityMode == WDRV_SECURITY_WPA2_WITH_PASS_PHRASE) &&
            g_wdrv_priv.wpaCipherRetryCnt > 0x00) {
            if (gp_wdrv_cfg->securityMode == WDRV_SECURITY_WPA_WITH_PASS_PHRASE)
                WDRV_EXT_CmdTKIPEncryptSet();
            else if (gp_wdrv_cfg->securityMode == WDRV_SECURITY_WPA2_WITH_PASS_PHRASE)
                WDRV_EXT_CmdAESEncryptSet();
            g_wdrv_priv.wpaCipherRetryCnt = 0x00;
        }

        if (s_ConnectionState == WDRV_CSTATE_CONNECTION_TEMPORARY_LOST)
            g_wdrv_priv.isConnReestablished = true;

        s_ConnectionState = WDRV_CSTATE_CONNECTED;
    } else {
        switch (reason) {
        case WDRV_DISCONNECT_REASON_NO_NETWORK_AVAIL:           // = 0x01,
            if (g_wdrv_priv.isDisconnectRequested) {
                DISCONNECT_DONE_NOTIFY();
                s_ConnectionState = WDRV_CSTATE_NOT_CONNECTED;
                break;
            }

            if ((gp_wdrv_cfg->securityMode == WDRV_SECURITY_WPA_WITH_PASS_PHRASE ||
                 gp_wdrv_cfg->securityMode == WDRV_SECURITY_WPA2_WITH_PASS_PHRASE) &&
                g_wdrv_priv.wpaCipherRetryCnt == 0x00) {
                if (gp_wdrv_cfg->securityMode == WDRV_SECURITY_WPA_WITH_PASS_PHRASE)
                    WDRV_EXT_CmdAESEncryptSet();
                else if (gp_wdrv_cfg->securityMode == WDRV_SECURITY_WPA2_WITH_PASS_PHRASE)
                    WDRV_EXT_CmdTKIPEncryptSet();
                WDRV_EXT_CmdConnect();
                g_wdrv_priv.wpaCipherRetryCnt++;
                s_ConnectionState = WDRV_CSTATE_CONNECTION_IN_PROGRESS;
            } else if ((gp_wdrv_cfg->securityMode == WDRV_SECURITY_WPA_WITH_PASS_PHRASE ||
                        gp_wdrv_cfg->securityMode == WDRV_SECURITY_WPA2_WITH_PASS_PHRASE) &&
                       g_wdrv_priv.wpaCipherRetryCnt > 0x00) {
                if (gp_wdrv_cfg->securityMode == WDRV_SECURITY_WPA_WITH_PASS_PHRASE)
                    WDRV_EXT_CmdTKIPEncryptSet();
                else if (gp_wdrv_cfg->securityMode == WDRV_SECURITY_WPA2_WITH_PASS_PHRASE)
                    WDRV_EXT_CmdAESEncryptSet();
                g_wdrv_priv.wpaCipherRetryCnt = 0x00;
                s_ConnectionState = WDRV_CSTATE_NOT_CONNECTED;
            } else {
                s_ConnectionState = WDRV_CSTATE_NOT_CONNECTED;
            }

            break;
        case WDRV_DISCONNECT_REASON_LOST_LINK:                  // = 0x02
            if (timer != 0)
                SYS_TMR_CallbackStop(timer);

            /* Wait for 30 seconds */
            timeout = SYS_TMR_TickCounterFrequencyGet() * 30;
            timer = SYS_TMR_CallbackSingle(timeout, 0, LinkDownTimeoutCallback);
            s_ConnectionState = WDRV_CSTATE_CONNECTION_TEMPORARY_LOST;
            break;
        case WDRV_DISCONNECT_REASON_DISCONNECT_CMD:             // = 0x03,
        case WDRV_DISCONNECT_REASON_BSS_DISCONNECTED:           // = 0x04,
        case WDRV_DISCONNECT_REASON_AUTH_FAILED:                // = 0x05,
        case WDRV_DISCONNECT_REASON_ASSOC_FAILED:               // = 0x06,
        case WDRV_DISCONNECT_REASON_NO_RESOURCES_AVAIL:         // = 0x07,
        case WDRV_DISCONNECT_REASON_CONNECTION_DENIED:          // = 0x08,
        case WDRV_DISCONNECT_REASON_INVALID_PROFILE:            // = 0x0A,
        case WDRV_DISCONNECT_REASON_PROFILE_MISMATCH:           // = 0x0C,
        case WDRV_DISCONNECT_REASON_CONNECTION_EVICTED:         // = 0x0D,
            if (g_wdrv_priv.isDisconnectRequested)
                DISCONNECT_DONE_NOTIFY();

            s_ConnectionState = WDRV_CSTATE_NOT_CONNECTED;
            break;
        default:
            WDRV_ASSERT(false, "Undefined reason code");
            break;
        }
    }
}

WDRV_CONNECTION_STATES WDRV_ConnectionState_Get(void)
{
    return s_ConnectionState;
}

//DOM-IGNORE-END
