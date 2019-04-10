/*******************************************************************************
  File Name:
    mode_ap.c

  Summary:
    WINC1500 AP Mode Example

  Description:
    This example demonstrates the use of WINC1500's AP mode.

    Note WPA2 security in SoftAP requires 19.5.2 FW or later. Older FW supports
    only WEP and open securities.

    The configuration defines for this demo are:
        WLAN_SSID           -- WINC1500's SSID in AP mode
        WLAN_AUTH           -- WINC1500's security in AP mode
        WLAN_PSK            -- Passphrase for WPA security
        WLAN_WEP_KEY        -- Key for WEP security
        WLAN_WEP_KEY_INDEX  -- Key index for WEP security
        WLAN_CHANNEL        -- WINC1500's working channel
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

#include "app.h"
#include "socket.h"
#include "m2m_wifi.h"

#if MODE_AP_EXAMPLE

#define WLAN_SSID           "WINC1500_AP_MODE" /* WINC1500's SSID */
#define WLAN_AUTH           M2M_WIFI_SEC_WPA_PSK /* WINC1500's Security, M2M_WIFI_SEC_WPA_PSK, M2M_WIFI_SEC_WEP or M2M_WIFI_SEC_OPEN */
#define WLAN_PSK            "12345678" /* Passphrase for WPA Security */
#define WLAN_WEP_KEY        "1234567890" /* Key for WEP Security */
#define WLAN_WEP_KEY_INDEX  (1) /* Key Index for WEP Security */
#define WLAN_CHANNEL        (6) /* WINC1500's Working Channel */

#define EXAMPLE_HEADER \
"\r\n========================\r\n"\
    "WINC1500 AP Mode Example\r\n"\
    "========================\r\n"

#define app_state_get() s_app_state
#define app_state_set(x) do {s_app_state = x;} while (0)

typedef enum {
    APP_RADIO_INIT,
    APP_WIFI_OPEN,
    APP_NET_UP,
    APP_END,
    APP_PARK
} APP_STATE;

/** Demo state */
static APP_STATE s_app_state;

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
    switch (u8MsgType) {
    case M2M_WIFI_RESP_CON_STATE_CHANGED:
    {
        tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
        if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
        } else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
            SYS_CONSOLE_PRINT("Station disconnected\r\n");
        }
        break;
    }
    case M2M_WIFI_REQ_DHCP_CONF:
    {
        uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
        SYS_CONSOLE_PRINT("Station connected\r\n");
        SYS_CONSOLE_PRINT("Station IP is %u.%u.%u.%u\r\n",
                pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
        break;
    }
    default:
        break;
    }
}

static int8_t wifi_open(void)
{
    tstrWifiInitParam param;
    tstrM2MAPConfig strM2MAPConfig;
    int8_t ret = 0;

    /* Initialize Wi-Fi parameters structure. */
    memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

    /* Initialize Wi-Fi driver with data and status callbacks. */
    param.pfAppWifiCb = wifi_cb;
    ret = m2m_wifi_init(&param);
    if (M2M_SUCCESS != ret) {
        SYS_CONSOLE_PRINT("m2m_wifi_init call error!(%d)\r\n", ret);
        return ret;
    }

    /* Initialize AP mode parameters structure with SSID, channel and open security type. */
    memset(&strM2MAPConfig, 0x00, sizeof(tstrM2MAPConfig));
    strcpy((char *)&strM2MAPConfig.au8SSID, WLAN_SSID);
    strM2MAPConfig.u8ListenChannel = WLAN_CHANNEL;
    strM2MAPConfig.u8SecType = WLAN_AUTH;

    strM2MAPConfig.au8DHCPServerIP[0] = 192;
    strM2MAPConfig.au8DHCPServerIP[1] = 168;
    strM2MAPConfig.au8DHCPServerIP[2] = 1;
    strM2MAPConfig.au8DHCPServerIP[3] = 1;

    if (WLAN_AUTH == M2M_WIFI_SEC_WEP) {
        strM2MAPConfig.u8KeyIndx = WLAN_WEP_KEY_INDEX;
        strM2MAPConfig.u8KeySz = strlen(WLAN_WEP_KEY);
        strcpy((char *)&strM2MAPConfig.au8WepKey, WLAN_WEP_KEY);
    } else if (WLAN_AUTH == M2M_WIFI_SEC_WPA_PSK) {
        strM2MAPConfig.u8KeySz = strlen(WLAN_PSK);
        strcpy((char *)&strM2MAPConfig.au8Key, WLAN_PSK);
    }

    /* Bring up AP mode with parameters structure. */
    ret = m2m_wifi_enable_ap(&strM2MAPConfig);
    if (M2M_SUCCESS != ret) {
        SYS_CONSOLE_PRINT("m2m_wifi_enable_ap call error!\r\n");
        return ret;
    }

    SYS_CONSOLE_PRINT("AP mode started. You can connect to %s.\r\n", (char *)WLAN_SSID);
    return ret;
}

static int custom_app(void)
{
	/* SoftAP is now up. You may run your application here. */
	return 0;
}

void app_init(void)
{
	SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

	app_state_set(APP_RADIO_INIT);
}

void app_task(void)
{
	int8_t ret;

	switch (app_state_get()) {
	case APP_RADIO_INIT:
		radio_init(NULL);
		app_state_set(APP_WIFI_OPEN);
		break;
	case APP_WIFI_OPEN:
		ret = wifi_open();
		if (ret)
			app_state_set(APP_END);
		else
			app_state_set(APP_NET_UP);
		break;
	case APP_NET_UP:
		ret = custom_app();
		if (ret)
			app_state_set(APP_END);
		break;
	case APP_END:
		radio_deinit();
		app_state_set(APP_PARK);
		break;
	case APP_PARK:
		/* Example has finished. Spinning wheels... */
		break;
	default:
		break;
	}
}

#endif /* MODE_AP_EXAMPLE */

//DOM-IGNORE-END
