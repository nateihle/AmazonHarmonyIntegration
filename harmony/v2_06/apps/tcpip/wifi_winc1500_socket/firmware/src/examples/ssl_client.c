/*******************************************************************************
  File Name:
    ssl_client.c

  Summary:
    WINC1500 SSL Client Example

  Description:
    This example demonstrates how to connect to a server via Secure Socket Layer
    (SSL) using the WINC1500 Wi-Fi module.

    For using SSL, the root certificate must be installed.
    Download the root certificate using the root_certificate_downloader from the
    firmware updater.

    The configuration defines for this demo are:
        WLAN_SSID           -- AP to connect to
        WLAN_AUTH           -- Security for the AP
        WLAN_PSK            -- Passphrase for WPA security
        WLAN_WEP_KEY        -- Key for WEP security
        WLAN_WEP_KEY_INDEX  -- Key index for WEP security
        HOST_NAME           -- Host name, for example "www.google.com"
        HOST_PORT           -- Host port, for example 443
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

#if SSL_CLIENT_EXAMPLE

#define WLAN_SSID           "DEMO_AP" /* Target AP */
#define WLAN_AUTH           M2M_WIFI_SEC_WPA_PSK /* AP Security, M2M_WIFI_SEC_WPA_PSK, M2M_WIFI_SEC_WEP or M2M_WIFI_SEC_OPEN */
#define WLAN_PSK            "12345678" /* Passphrase for WPA Security */
#define WLAN_WEP_KEY        "1234567890" /* Key for WEP Security */
#define WLAN_WEP_KEY_INDEX  1 /* Key Index for WEP Security */

#define HOST_NAME           "www.google.com"
#define HOST_PORT           443

#define EXAMPLE_HEADER \
"\r\n===========================\r\n"\
    "WINC1500 SSL Client Example\r\n"\
    "===========================\r\n"

#define IPV4_BYTE(val, index)       ((val >> (index * 8)) & 0xFF)

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

typedef enum {
	SOCKET_INIT = 0,
	SOCKET_CONNECT,
	SOCKET_WAITING,
	SOCKET_COMPLETE,
	SOCKET_ERROR
} SOCKET_STATUS;

static APP_STATE s_app_state;
static bool s_connected;
static bool s_host_ip_by_name; /* Get host IP status variable. */
static SOCKET s_tcp_client_sock; /* TCP client socket handler. */
static uint8_t s_socket_status;
static uint32_t s_host_ip; /* IP address of host. */

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			int8_t connect_ret;
			s_connected = false;
			s_host_ip_by_name = false;
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
					WLAN_AUTH, (char *)WLAN_PSK, M2M_WIFI_CH_ALL);
			}

			if (connect_ret != M2M_SUCCESS) {
				SYS_CONSOLE_PRINT("wifi_cb: m2m_wifi_connect call error!(%d)\r\n", connect_ret);
			}
		}
	}
		break;
	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		int8_t gethostbyname_ret;
		s_connected = true;
		SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n",
			pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);

		/* Obtain the IP Address by network name. */
		gethostbyname_ret = gethostbyname((uint8_t *)HOST_NAME);
		if (gethostbyname_ret != M2M_SUCCESS) {
			SYS_CONSOLE_PRINT("wifi_cb: gethostbyname call error!(%d)\r\n", gethostbyname_ret);
		}
	}
		break;
	default:
		break;
	}
}

/*
 * Callback function of IP address.
 *
 * hostName: domain name.
 * hostIp: server IP.
 */
static void resolve_cb(uint8_t *hostName, uint32_t hostIp)
{
	s_host_ip = hostIp;
	s_host_ip_by_name = true;
	SYS_CONSOLE_PRINT("resolve_cb: Host IP is %d.%d.%d.%d\r\n", (uint8_t)IPV4_BYTE(hostIp, 0), (uint8_t)IPV4_BYTE(hostIp, 1),
		(uint8_t)IPV4_BYTE(hostIp, 2), (uint8_t)IPV4_BYTE(hostIp, 3));
	SYS_CONSOLE_PRINT("resolve_cb: Host Name is %s\r\n", hostName);
}

/*
 * Callback function of TCP client socket.
 *
 * sock: socket handler.
 * u8Msg: type of socket notification.
 * pvMsg: a structure contains notification informations.
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	/* Check for socket event on TCP socket. */
	if (sock == s_tcp_client_sock) {
		switch (u8Msg) {
		case SOCKET_MSG_CONNECT:
		{
			tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
			if (pstrConnect && pstrConnect->s8Error >= SOCK_ERR_NO_ERROR) {
				s_socket_status = SOCKET_CONNECT;
				SYS_CONSOLE_PRINT("socket_cb: CONNECTED\r\n");
			} else {
				s_socket_status = SOCKET_ERROR;
				SYS_CONSOLE_PRINT("socket_cb: CONNECTION ERROR!(%d)\r\n", pstrConnect->s8Error);
			}
		}
			break;
		default:
			break;
		}
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
	registerSocketCallback(socket_cb, (tpfAppResolveCb)resolve_cb);

	/* Connect to AP. */
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
	}

	return ret;
}

/*
 * Creates and connects to a secure socket to be used for SSL client.
 *
 * Return:
 *   SOCK_ERR_NO_ERROR if success,
 *   -1 if socket creation error,
 *   SOCK_ERR_INVALID if socket connection error.
 */
static int8_t ssl_connect(void)
{
	struct sockaddr_in addr_in;

	addr_in.sin_family = AF_INET;
	addr_in.sin_port = _htons(HOST_PORT);
	addr_in.sin_addr.s_addr = s_host_ip;

	/* Create secure socket. */
	if (s_tcp_client_sock < 0) {
		s_tcp_client_sock = socket(AF_INET, SOCK_STREAM, SOCKET_FLAGS_SSL);
	}

	/* Check if socket was created successfully. */
	if (s_tcp_client_sock < 0) {
		SYS_CONSOLE_PRINT("Socket creation error\r\n");
		close(s_tcp_client_sock);
		return -1;
	}

	/* If success, connect to socket. */
	if (connect(s_tcp_client_sock, (struct sockaddr *)&addr_in, sizeof(struct sockaddr_in)) != SOCK_ERR_NO_ERROR) {
		SYS_CONSOLE_PRINT("Socket connection error\r\n");
		return SOCK_ERR_INVALID;
	}

	/* Success. */
	return SOCK_ERR_NO_ERROR;
}

static void socket_close(void)
{
	close(s_tcp_client_sock);
	s_tcp_client_sock = -1;
}

static uint8_t ssl_client(void)
{
	if (s_host_ip_by_name) {
		if (s_socket_status == SOCKET_INIT) {
			if (s_tcp_client_sock < 0) {
				s_socket_status = SOCKET_WAITING;
				if (ssl_connect() != SOCK_ERR_NO_ERROR) {
					s_socket_status = SOCKET_INIT;
				}
			}
		} else if (s_socket_status == SOCKET_CONNECT) {
			/* Keep the secure socket connected for 30 seconds, then close the socket and finish the example. */
			WDRV_TIME_DELAY(30000);
			s_socket_status = SOCKET_COMPLETE;
			socket_close();
		}
	}
	return s_socket_status;
}

void app_init(void)
{
	SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

	app_state_set(APP_RADIO_INIT);
	s_connected = false;
	s_host_ip_by_name = false;
	s_tcp_client_sock = -1;
	s_socket_status = SOCKET_INIT;
	s_host_ip = 0;
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
		ret = ssl_client();
		if (ret == SOCKET_COMPLETE)
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

#endif /* SSL_CLIENT_EXAMPLE */

//DOM-IGNORE-END
