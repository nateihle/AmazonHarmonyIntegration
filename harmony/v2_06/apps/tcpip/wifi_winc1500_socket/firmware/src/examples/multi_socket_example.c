/*******************************************************************************
  File Name:
    multi_socket_example.c

  Summary:
    WINC1500 Multi Socket Example

  Description:
    This example demonstrates the use of multiple TCP client sockets with
    the WINC1500.

    The configuration defines for this demo are:
        WLAN_SSID                   -- AP to connect to
        WLAN_AUTH                   -- Security for the AP
        WLAN_PSK                    -- Passphrase for WPA security
        WIFI_M2M_SERVER_IP          -- IP address for the TCP server
        WIFI_M2M_SERVER_PORT        -- Port number for the TCP server
        WIFI_M2M_BUFFER_SIZE        -- Size of the socket buffer holding the RX data
        TCP_SEND_MESSAGE            -- Customizable TCP packet content
        MAX_SOCKETS                 -- Number of sockets to run simultaneously
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

#if MULTI_SOCKET_EXAMPLE

#define WLAN_SSID                   "DEMO_AP" /**< Destination SSID */
#define WLAN_AUTH                   M2M_WIFI_SEC_WPA_PSK /**< Security manner such as M2M_WIFI_SEC_WPA_PSK and M2M_WIFI_SEC_OPEN */
#define WLAN_PSK                    "12345678" /**< Password for destination SSID */

#define WIFI_M2M_SERVER_IP          "192.168.1.100"
#define WIFI_M2M_SERVER_PORT        6666
#define WIFI_M2M_BUFFER_SIZE        1460

#define TCP_SEND_MESSAGE            "TCP message from WINC1500 module\r\n"

/*
 * TCP can support max 7 sockets at the same time. UDP can support up to 4.
 * In this example, we set 2 for demo purpose. If you wish, you can increase
 * the number on your test.
 */
#define MAX_SOCKETS 2

#define EXAMPLE_HEADER \
"\r\n=============================\r\n"\
    "WINC1500 Multi Scoket Example\r\n"\
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

/** Message format declaration */
static char s_msg_tcp_send[sizeof(TCP_SEND_MESSAGE) + 64] = TCP_SEND_MESSAGE;

/** Receive buffer definition */
static uint8_t s_test_buffer[WIFI_M2M_BUFFER_SIZE];

/** Socket for client */
static SOCKET s_tcp_sockets[MAX_SOCKETS];

/** Demo state */
static APP_STATE s_app_state;

/** Connection flag */
static bool s_connected;

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
			SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
			connect_ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID), WLAN_AUTH, (char *)WLAN_PSK, M2M_WIFI_CH_ALL);
			if (connect_ret != M2M_SUCCESS) {
				SYS_CONSOLE_PRINT("m2m_wifi_connect call error!(%d)\r\n", connect_ret);
			}
		}
	}
		break;
	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n", pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		s_connected = true;
	}
		break;
	default:
		break;
	}
}

int get_sock_idx(SOCKET sock)
{
	int i;

	for (i = 0; i < MAX_SOCKETS; i++) {
		if (s_tcp_sockets[i] == sock)
			break;
	}

	WDRV_ASSERT(i < MAX_SOCKETS, "Invalid socket");

	return i;
}

/**
 * \brief Callback to get the Data from socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg socket event type. Possible values are:
 *  - SOCKET_MSG_BIND
 *  - SOCKET_MSG_LISTEN
 *  - SOCKET_MSG_ACCEPT
 *  - SOCKET_MSG_CONNECT
 *  - SOCKET_MSG_RECV
 *  - SOCKET_MSG_SEND
 *  - SOCKET_MSG_SENDTO
 *  - SOCKET_MSG_RECVFROM
 * \param[in] pvMsg is a pointer to message structure. Existing types are:
 *  - tstrSocketBindMsg
 *  - tstrSocketListenMsg
 *  - tstrSocketAcceptMsg
 *  - tstrSocketConnectMsg
 *  - tstrSocketRecvMsg
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	switch (u8Msg) {
	/* Socket connect */
	case SOCKET_MSG_CONNECT:
	{
		tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
		if (pstrConnect && pstrConnect->s8Error >= 0) {
			char sock_num[64];

			SYS_CONSOLE_PRINT("socket_cb: connect success!\r\n");
			sprintf(sock_num, " from socket number %d", get_sock_idx(sock));
			strcpy(s_msg_tcp_send, sock_num);
			send(sock, s_msg_tcp_send, strlen(s_msg_tcp_send), 0);
		} else {
			SYS_CONSOLE_PRINT("socket_cb: connect error!\r\n");
			close(sock);
			s_tcp_sockets[get_sock_idx(sock)] = -1;
		}
	}
		break;
	/* Message send */
	case SOCKET_MSG_SEND:
	{
		SYS_CONSOLE_PRINT("socket_cb: send success!\r\n");
		recv(sock, s_test_buffer, sizeof(s_test_buffer), 0);
	}
		break;
	/* Message receive */
	case SOCKET_MSG_RECV:
	{
		tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
		if (pstrRecv && pstrRecv->s16BufferSize > 0) {
			SYS_CONSOLE_PRINT("socket_cb: recv success!\r\n");
			SYS_CONSOLE_PRINT("TCP Client Test Complete!\r\n");
		} else {
			SYS_CONSOLE_PRINT("socket_cb: recv error!\r\n");
			close(sock);
			s_tcp_sockets[get_sock_idx(sock)] = -1;
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

	/* Connect to router. */
	ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID), WLAN_AUTH, (char *)WLAN_PSK, M2M_WIFI_CH_ALL);

	return ret;
}

static int8_t tcp_client_run(int idx)
{
	struct sockaddr_in addr;
	int8_t ret = 0;

	if (s_tcp_sockets[idx] < 0) {
		if ((s_tcp_sockets[idx] = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
			SYS_CONSOLE_PRINT("failed to create TCP client socket error!\r\n");
			return -1;
		}

		/* Initialize socket address structure. */
		addr.sin_family = AF_INET;
		addr.sin_port = _htons((WIFI_M2M_SERVER_PORT + idx));
		addr.sin_addr.s_addr = nmi_inet_addr(WIFI_M2M_SERVER_IP);

		/* Connect to server. */
		ret = connect(s_tcp_sockets[idx], (struct sockaddr *)&addr, sizeof(struct sockaddr_in));

		if (ret < 0) {
			close(s_tcp_sockets[idx]);
			s_tcp_sockets[idx] = -1;
		}
	}

	return ret;
}

void app_init(void)
{
	int i;

	SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);
	app_state_set(APP_RADIO_INIT);

	for (i = 0; i < MAX_SOCKETS; i++)
		s_tcp_sockets[i] = -1;

	s_connected = false;
}

void app_task(void)
{
	int8_t ret;
	int i;

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
		for (i = 0; i < MAX_SOCKETS; i++) {
			ret = tcp_client_run(i);
			if (ret) {
				app_state_set(APP_END);
				break;
			}
		}
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

#endif /* MULTI_SOCKET_EXAMPLE */

//DOM-IGNORE-END
