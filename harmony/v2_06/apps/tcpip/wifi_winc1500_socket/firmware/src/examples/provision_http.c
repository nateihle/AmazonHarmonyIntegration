/*******************************************************************************
  File Name:
    provision_http.c

  Summary:
    WINC1500 Provision HTTP Example

  Description:
    This demo performs the following steps:
        1) In this example, WINC1500 firstly works in AP mode
        2) Connect a device (PC, smartphone or tablet) to WINC1500
        3) Access the webpage on http://wincconfig.com or http://192.168.1.1, fill in a new wireless network's information
        4) WINC1500 will be re-directed to the new wireless network

    Note WPA2 security in SoftAP requires 19.5.2 FW or later. Older FW supports
    only WEP and open securities.

    The configuration defines for this demo are:
        WLAN_SSID_MODE          -- If WINC1500's SSID is visible in AP mode
        WLAN_AUTH               -- WINC1500's security in AP mode
        WLAN_PSK                -- Passphrase for WPA security
        WLAN_WEP_KEY            -- Key for WEP security
        WLAN_WEP_KEY_INDEX      -- Key index for WEP security
        WLAN_CHANNEL            -- WINC1500's working channel
        WLAN_HTTP_SERVER_DOMAIN -- WINC1500 HTTP server's domain in provision mode
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

#if PROVISION_HTTP_EXAMPLE

#define WLAN_SSID_MODE              SSID_MODE_VISIBLE /* If WINC1500's SSID is visible in AP mode */
#define WLAN_AUTH                   M2M_WIFI_SEC_WPA_PSK /* WINC1500's security, M2M_WIFI_SEC_WPA_PSK, M2M_WIFI_SEC_WEP or M2M_WIFI_SEC_OPEN */
#define WLAN_PSK                    "12345678" /* Passphrase for WPA security */
#define WLAN_WEP_KEY                "1234567890" /* Key for WEP security */
#define WLAN_WEP_KEY_INDEX          1 /* Key index for WEP security */
#define WLAN_CHANNEL                6 /* WINC1500's working channel */
#define WLAN_HTTP_SERVER_DOMAIN     "wincconfig.com" /* WINC1500 HTTP server's domain in AP mode */

#define WIFI_DEVICE_NAME            "WINC1500_00:00" /* In this example, device name has to be in the format "WINC1500_00:00" */
#define WIFI_DEVICE_MAC_ADDRESS     {0xf8, 0xf0, 0x05, 0x45, 0xD4, 0x84} /* Example default MAC address if cannot get a valid MAC address from WINC1500 */

#define EXAMPLE_HEADER \
"\r\n===============================\r\n"\
    "WINC1500 Provision HTTP Example\r\n"\
    "===============================\r\n"

#define HEX2ASCII(x) (((x) >= 10) ? (((x) - 10) + 'A') : ((x) + '0'))

#define app_state_get() s_app_state
#define app_state_set(x) do {s_app_state = x;} while (0)
#define is_link_up() s_connected

typedef enum {
	APP_RADIO_INIT,
	APP_WIFI_OPEN,
	APP_WIFI_CONNECT_WAIT,
	APP_NET_UP,
	APP_END,
	APP_PARK
} APP_STATE;

static APP_STATE s_app_state;
static bool s_connected;
static bool s_is_ap;
static uint8_t s_device_name[sizeof(WIFI_DEVICE_NAME)]; /* In this example, device name has to be in the format "WINC1500_00:00". */
static uint8_t s_device_mac_addr[6];

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			s_connected = false;
			SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
		}
	}
		break;
	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		s_connected = true;
		if (s_is_ap)
			SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: client's IP is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		else
			SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: WINC1500's IP is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
	}
		break;
	case M2M_WIFI_RESP_PROVISION_INFO:
	{
		tstrM2MProvisionInfo *pstrProvInfo = (tstrM2MProvisionInfo *)pvMsg;
		SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_RESP_PROVISION_INFO\r\n");
		if (pstrProvInfo->u8Status == M2M_SUCCESS) {
			int8_t connect_ret;
			s_is_ap = false;
			connect_ret = m2m_wifi_connect((char *)pstrProvInfo->au8SSID, strlen((char *)pstrProvInfo->au8SSID), pstrProvInfo->u8SecType,
					pstrProvInfo->au8Password, M2M_WIFI_CH_ALL);
			if (connect_ret != M2M_SUCCESS)
   				SYS_CONSOLE_PRINT("socket_cb: m2m_wifi_connect call error!(%d)\r\n", connect_ret);
		} else {
			SYS_CONSOLE_PRINT("wifi_cb: Provision failed\r\n");
		}
	}
		break;
	default:
		break;
	}
}

static void set_dev_name_to_mac(uint8_t *name, uint8_t *mac_addr)
{
	/* In this function, variable "name" must be in the format "WINC1500_00:00". */
	size_t len;

	len = strlen((const char *)name);
	if (len >= 5) {
		name[len - 1] = HEX2ASCII((mac_addr[5] >> 0) & 0x0f);
		name[len - 2] = HEX2ASCII((mac_addr[5] >> 4) & 0x0f);
		name[len - 4] = HEX2ASCII((mac_addr[4] >> 0) & 0x0f);
		name[len - 5] = HEX2ASCII((mac_addr[4] >> 4) & 0x0f);
	}
}

static int8_t wifi_open(void)
{
	tstrWifiInitParam param;
    int8_t ret;
	uint8_t mac_addr[6];
	uint8_t is_mac_addr_valid;
	tstrM2MAPConfig ap_config;

    /* Initialize Wi-Fi parameters structure. */
    memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

    /* Initialize Wi-Fi driver with data and status callbacks. */
    param.pfAppWifiCb = wifi_cb;
    ret = m2m_wifi_init(&param);
    if (ret != M2M_SUCCESS) {
        SYS_CONSOLE_PRINT("m2m_wifi_init call error!(%d)\r\n", ret);
        return ret;
    }

    ret = m2m_wifi_get_otp_mac_address(mac_addr, &is_mac_addr_valid);
    if (ret != M2M_SUCCESS) {
        SYS_CONSOLE_PRINT("m2m_wifi_get_otp_mac_address call error!(%d)\r\n", ret);
        return ret;
    }
	if (!is_mac_addr_valid)
		m2m_wifi_set_mac_address(s_device_mac_addr);

	ret = m2m_wifi_get_mac_address(s_device_mac_addr);
    if (ret != M2M_SUCCESS) {
        SYS_CONSOLE_PRINT("m2m_wifi_get_mac_address call error!(%d)\r\n", ret);
        return ret;
    }

	strcpy((char *)&ap_config.au8SSID, WIFI_DEVICE_NAME);
	ap_config.u8SsidHide = WLAN_SSID_MODE;
	ap_config.u8SecType = WLAN_AUTH;
	ap_config.u8KeySz = strlen(WLAN_PSK);
	strcpy((char *)&ap_config.au8Key, WLAN_PSK);
	ap_config.u8ListenChannel = WLAN_CHANNEL;

	set_dev_name_to_mac((uint8_t *)s_device_name, s_device_mac_addr);
	set_dev_name_to_mac((uint8_t *)ap_config.au8SSID, s_device_mac_addr);
	ret = m2m_wifi_set_device_name((uint8 *)s_device_name, (uint8)strlen((const char *)s_device_name));
    if (ret != M2M_SUCCESS) {
        SYS_CONSOLE_PRINT("m2m_wifi_set_device_name call error!(%d)\r\n", ret);
        return ret;
    }
	ap_config.au8DHCPServerIP[0] = 0xC0; /* 192 */
	ap_config.au8DHCPServerIP[1] = 0xA8; /* 168 */
	ap_config.au8DHCPServerIP[2] = 0x01; /* 1 */
	ap_config.au8DHCPServerIP[3] = 0x01; /* 1 */

	ret = m2m_wifi_start_provision_mode((tstrM2MAPConfig *)&ap_config, (char *)WLAN_HTTP_SERVER_DOMAIN, 1);
    if (ret != M2M_SUCCESS) {
        SYS_CONSOLE_PRINT("m2m_wifi_start_provision_mode call error!(%d)\r\n", ret);
        return ret;
    }
	s_is_ap = true;
	SYS_CONSOLE_PRINT("Provision Mode started\r\nConnect to [%s] via AP[%s] and fill up the page\r\n", WLAN_HTTP_SERVER_DOMAIN, ap_config.au8SSID);

    return ret;
}

int provision_http(void)
{
	if (!s_is_ap && is_link_up()) {
		/* Keep WINC1500 connected in client station mode for 30 seconds, then finish the example. */
		WDRV_TIME_DELAY(30000);
		SYS_CONSOLE_PRINT("Example complete\r\n");
		return -1;
	}
	return 0;
}

void app_init(void)
{
	uint8_t mac_addr[] = WIFI_DEVICE_MAC_ADDRESS;

	SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

	app_state_set(APP_RADIO_INIT);
	s_connected = false;
	s_is_ap = false;
	WDRV_ASSERT(sizeof(WIFI_DEVICE_NAME) <= sizeof(s_device_name), "Invalid device name");
	WDRV_ASSERT(sizeof(mac_addr) <= sizeof(s_device_mac_addr), "Invalid MAC address");
	memcpy(s_device_name, WIFI_DEVICE_NAME, sizeof(WIFI_DEVICE_NAME));
	memcpy(s_device_mac_addr, mac_addr, sizeof(mac_addr));
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
			app_state_set(APP_WIFI_CONNECT_WAIT);
		break;
	case APP_WIFI_CONNECT_WAIT:
		if (is_link_up())
			app_state_set(APP_NET_UP);
		break;
	case APP_NET_UP:
		ret = provision_http();
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

#endif /* PROVISION_HTTP_EXAMPLE */

//DOM-IGNORE-END
