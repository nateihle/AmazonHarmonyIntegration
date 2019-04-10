/*******************************************************************************
  File Name:
    provision_ap.c

  Summary:
    WINC1500 Provision AP Example

  Description:
    This demo performs the following steps:
        1) Install provision_ap.apk on an Android device
        2) In this example, WINC1500 firstly works in AP mode
        3) Connect the Android device to WINC1500
        4) Run the "Provision AP" application on the device, fill in a new wireless network's information
        5) WINC1500 will be re-directed to the new wireless network

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

#if PROVISION_AP_EXAMPLE

/* AP Mode Settings */
#define WLAN_SSID               "WINC1500_PROVISION_AP" /* WINC1500's SSID */
#define WLAN_AUTH               M2M_WIFI_SEC_WPA_PSK /* WINC1500's Security, M2M_WIFI_SEC_WPA_PSK, M2M_WIFI_SEC_WEP or M2M_WIFI_SEC_OPEN */
#define WLAN_PSK                "12345678" /* Passphrase for WPA Security */
#define WLAN_WEP_KEY            "1234567890" /* Key for WEP Security */
#define WLAN_WEP_KEY_INDEX      1 /* Key Index for WEP Security */
#define WLAN_CHANNEL            6 /* WINC1500's Working Channel */

#define WIFI_M2M_SERVER_PORT    80
#define WIFI_M2M_BUFFER_SIZE    1024 /* Receive Buffer Size */

#define EXAMPLE_HEADER \
"\r\n=============================\r\n"\
    "WINC1500 Provision AP Example\r\n"\
    "=============================\r\n"

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
static SOCKET s_tcp_server_sock;
static SOCKET s_tcp_client_sock;
static uint8_t s_sock_buf[WIFI_M2M_BUFFER_SIZE];

/*
 * Callback to get Wi-Fi status updated.
 *
 * u8MsgType: type of Wi-Fi notification.
 * pvMsg: pointer to a buffer containing the notification parameters.
 */
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
	default:
		break;
	}
}

/*
 * Callback function of TCP client socket.
 *
 * sock: socket handler.
 * u8Msg: type of socket notification.
 * pvMsg: structure contains notification information.
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	switch (u8Msg) {
	/* Socket bind */
	case SOCKET_MSG_BIND:
	{
		tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg *)pvMsg;
		if (pstrBind && pstrBind->status == 0) {
			SYS_CONSOLE_PRINT("socket_cb: bind success!\r\n");
			listen(s_tcp_server_sock, 0);
		} else {
			SYS_CONSOLE_PRINT("socket_cb: bind error!\r\n");
			close(s_tcp_server_sock);
			s_tcp_server_sock = -1;
		}
	}
		break;
	/* Socket listen */
	case SOCKET_MSG_LISTEN:
	{
		tstrSocketListenMsg *pstrListen = (tstrSocketListenMsg *)pvMsg;
		if (pstrListen && pstrListen->status == 0) {
			SYS_CONSOLE_PRINT("socket_cb: ready to listen\r\n");
			accept(s_tcp_server_sock, NULL, NULL);
		} else {
			SYS_CONSOLE_PRINT("socket_cb: listen error!\r\n");
			close(s_tcp_server_sock);
			s_tcp_server_sock = -1;
		}
	}
		break;
	/* Connect accept */
	case SOCKET_MSG_ACCEPT:
	{
		tstrSocketAcceptMsg *pstrAccept = (tstrSocketAcceptMsg *)pvMsg;
		if (pstrAccept) {
			accept(s_tcp_server_sock, NULL, NULL);
			s_tcp_client_sock = pstrAccept->sock;
			SYS_CONSOLE_PRINT("socket_cb: client socket is created\r\n");
			recv(s_tcp_client_sock, s_sock_buf, sizeof(s_sock_buf), 0);
		} else {
			SYS_CONSOLE_PRINT("socket_cb: accept error!\r\n");
			close(s_tcp_server_sock);
			s_tcp_server_sock = -1;
		}
	}
		break;
	/* Message receive */
	case SOCKET_MSG_RECV:
	{
		tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
		if (pstrRecv && pstrRecv->s16BufferSize > 0) {
			char *p;

			p = strtok((char *)pstrRecv->pu8Buffer, ",");
			if (p != NULL && !strncmp(p, "apply", 5)) {
			    int8_t m2m_wifi_ret;
				char str_ssid[M2M_MAX_SSID_LEN], str_pw[M2M_MAX_PSK_LEN];
				uint8 sec_type = 0;

				p = strtok(NULL, ",");
				if (p)
					strcpy(str_ssid, p);

				p = strtok(NULL, ",");
				if (p)
					sec_type = atoi(p);

				p = strtok(NULL, ",");
				if (p)
					strcpy(str_pw, p);

				m2m_wifi_ret = m2m_wifi_disable_ap();
				if (m2m_wifi_ret != M2M_SUCCESS) {
    				SYS_CONSOLE_PRINT("socket_cb: m2m_wifi_disable_ap call error!(%d)\r\n", m2m_wifi_ret);
					break;
				}
				s_is_ap = false;
				SYS_CONSOLE_PRINT("socket_cb: disabled AP\r\n");
				WDRV_TIME_DELAY(500);
				SYS_CONSOLE_PRINT("socket_cb: connecting to %s\r\n", (char *)str_ssid);
				m2m_wifi_ret = m2m_wifi_connect((char *)str_ssid, strlen((char *)str_ssid), sec_type, str_pw, M2M_WIFI_CH_ALL);
				if (m2m_wifi_ret != M2M_SUCCESS)
    				SYS_CONSOLE_PRINT("socket_cb: m2m_wifi_connect call error!(%d)\r\n", m2m_wifi_ret);
				break;
			}
		} else {
			SYS_CONSOLE_PRINT("socket_cb: recv error!\r\n");
			close(s_tcp_client_sock);
			s_tcp_client_sock = -1;
		}

		if (s_tcp_client_sock != -1) {
			memset(s_sock_buf, 0, sizeof(s_sock_buf));
			recv(s_tcp_client_sock, s_sock_buf, sizeof(s_sock_buf), 0);
		}
	}
		break;
	default:
		break;
	}
}

static int8_t wifi_open(void)
{
	tstrWifiInitParam param;
	tstrM2MAPConfig strM2MAPConfig;
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
    registerSocketCallback(socket_cb, NULL);

	/* Initialize AP mode parameters structure with SSID, channel and security type. */
	memset(&strM2MAPConfig, 0x00, sizeof(tstrM2MAPConfig));
	strcpy((char *)&strM2MAPConfig.au8SSID, WLAN_SSID);
	strM2MAPConfig.u8ListenChannel = WLAN_CHANNEL;
	strM2MAPConfig.u8SecType = WLAN_AUTH;
	strM2MAPConfig.au8DHCPServerIP[0] = 0xC0; /* 192 */
	strM2MAPConfig.au8DHCPServerIP[1] = 0xA8; /* 168 */
	strM2MAPConfig.au8DHCPServerIP[2] = 0x01; /* 1 */
	strM2MAPConfig.au8DHCPServerIP[3] = 0x01; /* 1 */

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
	s_is_ap = true;
	SYS_CONSOLE_PRINT("AP mode started.\r\nConnect your Android device to %s then run the Provision AP application.\r\n", WLAN_SSID);

    return ret;
}

int provision_ap(void)
{
	if (s_tcp_server_sock < 0) {
		struct sockaddr_in addr;

		/* Initialize socket address structure. */
		addr.sin_family = AF_INET;
		addr.sin_port = _htons((WIFI_M2M_SERVER_PORT));
		addr.sin_addr.s_addr = 0;

		/* Open TCP server socket. */
		if ((s_tcp_server_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
			SYS_CONSOLE_PRINT("Failed to create TCP server socket!\r\n");
			return -1;
		}

		/* Bind service. */
		bind(s_tcp_server_sock, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
	}

	return 0;
}

void app_init(void)
{
	SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

	app_state_set(APP_RADIO_INIT);
	s_connected = false;
	s_is_ap = false;
	s_tcp_server_sock = -1;
	s_tcp_client_sock = -1;
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
		ret = provision_ap();
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

#endif /* PROVISION_AP_EXAMPLE */

//DOM-IGNORE-END
