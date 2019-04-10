/*******************************************************************************
  File Name:
    ap_scan.c

  Summary:
    WINC1500 Scan and Connection Example

  Description:
    This example performs the following steps:
        1) Scans all channels looking for the specified AP
        2) Displays all found APs
        3) Connects to the specified AP
        4) Sends one or more pings to a known IP address on the AP network

    The configuration options for this example are:
        WLAN_SSID           -- AP to search for
        WLAN_AUTH           -- Security for the AP
        WLAN_PSK            -- Passphrase for WPA security
        WLAN_WEP_KEY        -- Key for WEP security
        WLAN_WEP_KEY_INDEX  -- Key index for WEP security
        PING_ADDRESS        -- IP address to ping after successful connection
        PING_COUNT          -- Number of times to ping
        PING_INTERVAL       -- Time between pings, in milliseconds
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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
#include "m2m_socket_host_if.h"

#if AP_SCAN_EXAMPLE

#define WLAN_SSID           "DEMO_AP" /* Target AP */
#define WLAN_AUTH           M2M_WIFI_SEC_WPA_PSK /* AP Security, M2M_WIFI_SEC_WPA_PSK, M2M_WIFI_SEC_WEP or M2M_WIFI_SEC_OPEN */
#define WLAN_PSK            "12345678" /* Passphrase for WPA Security */
#define WLAN_WEP_KEY        "1234567890" /* Key for WEP Security */
#define WLAN_WEP_KEY_INDEX  1 /* Key Index for WEP Security */

#define PING_ADDRESS        "192.168.1.1" /* Address to Ping after Connection */
#define PING_COUNT          3 /* Number of Times to Ping */
#define PING_INTERVAL       100 /* Wait 100ms between Pings */

#define EXAMPLE_HEADER \
"\r\n========================\r\n"\
    "WINC1500 AP Scan Example\r\n"\
    "========================\r\n"

#define app_state_get() s_app_state
#define app_state_set(x) do {s_app_state = x;} while (0)
#define is_scan_done() s_scan_done
#define is_scan_res_ready() s_scan_res_ready
#define is_target_ap_found() s_target_ap_found
#define is_link_up() s_connected
#define is_ping_reply_recv() s_ping_reply_recv

typedef enum {
	APP_RADIO_INIT,
	APP_WIFI_OPEN,
	APP_WIFI_SCAN_WAIT,
	APP_WIFI_SCAN_RES_WAIT,
	APP_WIFI_CONNECT_WAIT,
	APP_NET_UP,
	APP_END,
	APP_PARK
} APP_STATE;

static APP_STATE s_app_state;
static bool s_scan_done;
static uint8_t s_ap_found;
static uint8_t s_scan_res_index;
static bool s_scan_res_ready;
static bool s_target_ap_found;
static bool s_connected;
static uint8_t s_ping_cnt;
static bool s_ping_reply_recv;
static uint32_t s_ping_delay;

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_SCAN_DONE:
		s_scan_done = true;
		s_ap_found = ((tstrM2mScanDone *)pvMsg)->u8NumofCh;
		SYS_CONSOLE_PRINT("wifi_cb: %d AP(s) found\r\n", s_ap_found);
		s_scan_res_index = 0;
		break;
	case M2M_WIFI_RESP_SCAN_RESULT:
	{
		tstrM2mWifiscanResult *pScanRes = (tstrM2mWifiscanResult *)pvMsg;
		s_scan_res_ready = true;
		SYS_CONSOLE_PRINT("wifi_cb: [%02d] SSID: %s\r\n              RSSI: %d\r\n",
			pScanRes->u8index + 1, pScanRes->au8SSID, pScanRes->s8rssi);
		if (strcmp((const char *)pScanRes->au8SSID, WLAN_SSID) == 0)
			s_target_ap_found = true;
	}
		break;
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			int8_t connect_ret;
			s_connected = false;
			SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
			if (WLAN_AUTH == M2M_WIFI_SEC_WEP) {
				tstrM2mWifiWepParams wep_params;
				wep_params.u8KeyIndx = WLAN_WEP_KEY_INDEX;
				wep_params.u8KeySz = sizeof(WLAN_WEP_KEY);
				memcpy(wep_params.au8WepKey, WLAN_WEP_KEY, sizeof(WLAN_WEP_KEY));
				connect_ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID),
					WLAN_AUTH, (void *)&wep_params, M2M_WIFI_CH_ALL);
			} else {
				connect_ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID),
					WLAN_AUTH, (void *)WLAN_PSK, M2M_WIFI_CH_ALL);
			}

			if (connect_ret != M2M_SUCCESS)
				SYS_CONSOLE_PRINT("wifi_cb: m2m_wifi_connect call error!(%d)\r\n", connect_ret);
		}
	}
		break;
	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		s_connected = true;
		SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n",
			pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
	}
		break;
	default:
		break;
	}
}

static void ping_cb(uint32_t ipAddr, uint32_t rtt, uint8_t errCode)
{
	++s_ping_cnt;
	s_ping_reply_recv = true;
	s_ping_delay = SYS_TMR_TickCountGet();
	switch (errCode) {
	case PING_ERR_SUCCESS:
		SYS_CONSOLE_PRINT("ping_cb: Ping successful; RTT = %ld\r\n", rtt);
		break;
	case PING_ERR_DEST_UNREACH:
		SYS_CONSOLE_PRINT("ping_cb: Ping failed; Destination unreachable\r\n");
		break;
	case PING_ERR_TIMEOUT:
		SYS_CONSOLE_PRINT("ping_cb: Ping failed; Timeout\r\n");
		break;
	default:
		break;
	}
}

static int8_t wifi_open(void)
{
	tstrWifiInitParam param;
	int8_t ret;

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (ret != M2M_SUCCESS) {
		SYS_CONSOLE_PRINT("m2m_wifi_init call error!(%d)\r\n", ret);
		return ret;
	}

	/* Initialize socket module. */
	socketInit();

	/* Request scan. */
	ret = m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
	if (ret != M2M_SUCCESS) {
		SYS_CONSOLE_PRINT("m2m_wifi_request_scan call error!(%d)\r\n", ret);
	}

	return ret;
}

static bool scan_all_res_get(void)
{
	if (s_ap_found == 0)
		return true;

	if (s_scan_res_index == 0) {
		m2m_wifi_req_scan_result(s_scan_res_index++);
		return false;
	} else if (s_scan_res_index < s_ap_found) {
		if (is_scan_res_ready()) {
			m2m_wifi_req_scan_result(s_scan_res_index++);
			s_scan_res_ready = false;
		}
		return false;
	} else { // s_scan_res_index >= s_ap_found
		return s_scan_res_ready;
	}
}

static bool target_ap_connect(void)
{
	if (is_target_ap_found()) {
		int8_t ret;

		SYS_CONSOLE_PRINT("Target AP found, trying to connect\r\n");

		/* Connect to target router, if return code is M2M_SUCCESS, succeed. */
		if (WLAN_AUTH == M2M_WIFI_SEC_WEP) {
			tstrM2mWifiWepParams wep_params;
			wep_params.u8KeyIndx = WLAN_WEP_KEY_INDEX;
			wep_params.u8KeySz = sizeof(WLAN_WEP_KEY);
			memcpy(wep_params.au8WepKey, WLAN_WEP_KEY, sizeof(WLAN_WEP_KEY));
			ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID),
				WLAN_AUTH, (void *)&wep_params, M2M_WIFI_CH_ALL);
		} else {
			ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID),
				WLAN_AUTH, (void *)WLAN_PSK, M2M_WIFI_CH_ALL);
		}

		if (ret != M2M_SUCCESS) {
			SYS_CONSOLE_PRINT("m2m_wifi_connect call error!(%d)\r\n", ret);
			return false;
		}

		return true;
	} else {
		SYS_CONSOLE_PRINT("Target AP not found\r\n");
		return false;
	}
}

static bool is_ping_interval_expired(void)
{
	if (SYS_TMR_TickCountGet() - s_ping_delay >=
		PING_INTERVAL * SYS_TMR_TickCounterFrequencyGet() / 1000ul) {
		return true;
	} else {
		return false;
	}
}

static bool ping_all_reply_recv(void)
{
	if (PING_COUNT <= 0)
		return true;

	if (s_ping_cnt == 0) {
		m2m_ping_req(nmi_inet_addr(PING_ADDRESS), 0, (tpfPingCb)ping_cb);
		return false;
	} else if (s_ping_cnt < PING_COUNT) {
		if (is_ping_reply_recv()) {
			if (is_ping_interval_expired()) {
				m2m_ping_req(nmi_inet_addr(PING_ADDRESS), 0, (tpfPingCb)ping_cb);
				s_ping_reply_recv = false;
			}
		}
		return false;
	} else { // s_ping_cnt >= PING_COUNT
		return true;
	}
}

void app_init(void)
{
	SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

	app_state_set(APP_RADIO_INIT);
	s_scan_done = false;
	s_ap_found = 0;
	s_scan_res_index = 0;
	s_scan_res_ready = false;
	s_target_ap_found = false;
	s_connected = false;
	s_ping_cnt = 0;
	s_ping_reply_recv = false;
	s_ping_delay = 0;
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
			app_state_set(APP_WIFI_SCAN_WAIT);
		break;
	case APP_WIFI_SCAN_WAIT:
		if (is_scan_done())
			app_state_set(APP_WIFI_SCAN_RES_WAIT);
		break;
	case APP_WIFI_SCAN_RES_WAIT:
		if (scan_all_res_get()) {
			if (target_ap_connect())
				app_state_set(APP_WIFI_CONNECT_WAIT);
			else
				app_state_set(APP_END);
		}
		break;
	case APP_WIFI_CONNECT_WAIT:
		if (is_link_up())
			app_state_set(APP_NET_UP);
		break;
	case APP_NET_UP:
		if (ping_all_reply_recv())
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

#endif /* AP_SCAN_EXAMPLE */

//DOM-IGNORE-END
