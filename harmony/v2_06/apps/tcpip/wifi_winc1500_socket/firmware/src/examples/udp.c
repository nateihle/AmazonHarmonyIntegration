/*******************************************************************************
  File Name:
    udp.c

  Summary:
    WINC1500 UDP Example

  Description:
    This example demonstrates the use of the WINC1500 to test UDP socket. It
    requires to use two development boards and demonstrates the UDP connection
    between them. Users need to run the same example code on both sides.
    However, WIFI_M2M_SERVER_IP needs to be set differently.

    The configuration defines for this demo are:
        WLAN_SSID                   -- AP to connect to
        WLAN_AUTH                   -- Security for the AP
        WLAN_PSK                    -- Passphrase for WPA security
        WIFI_M2M_SERVER_IP          -- IP address for the UDP server
        WIFI_M2M_SERVER_PORT        -- Port number for the UDP server
        WIFI_M2M_REPORT_INTERVAL    -- Time between sending two UDP packets, in milliseconds
        WIFI_M2M_BUFFER_SIZE        -- Size of the socket buffer holding the RX data
        WIFI_M2M_PACKET_COUNT       -- Number of UDP packets to send to the server
        UDP_SEND_MESSAGE            -- Customizable UDP packet content
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

#if UDP_EXAMPLE

#define WLAN_SSID                   "DEMO_AP" /* Target AP */
#define WLAN_AUTH                   M2M_WIFI_SEC_WPA_PSK /* AP Security such as M2M_WIFI_SEC_WPA_PSK and M2M_WIFI_SEC_OPEN */
#define WLAN_PSK                    "12345678" /* Passphrase for WPA Security */

#define WIFI_M2M_SERVER_IP          "192.168.1.100"
#define WIFI_M2M_SERVER_PORT        6666
#define WIFI_M2M_REPORT_INTERVAL    1000
#define WIFI_M2M_BUFFER_SIZE        1460
#define WIFI_M2M_PACKET_COUNT       10 /* UDP MAX Packet Count */
#define UDP_SEND_MESSAGE            "UDP message from WINC1500 module\r\n"

#define EXAMPLE_HEADER \
"\r\n====================\r\n"\
    "WINC1500 UDP Example\r\n"\
    "====================\r\n"

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
static uint8_t s_packet_cnt; /* UDP Packet Count */
static uint32_t s_ms_ticks; /* SysTick counter to avoid busy wait delay. */
static uint32_t s_delay; /* Global counter delay for timer. */
static SOCKET s_rx_sock; /* Socket for RX */
static SOCKET s_tx_sock; /* Socket for TX */
static uint8_t s_sock_bind_state; /* UDP Socket Bind State */
static uint8_t s_sock_buf[WIFI_M2M_BUFFER_SIZE]; /* Example Socket Buffer */

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

			if (connect_ret != M2M_SUCCESS)
				SYS_CONSOLE_PRINT("wifi_cb: m2m_wifi_connect call error!(%d)\r\n", connect_ret);
		}
	}
		break;
	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		s_connected = true;
		SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n", pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
	}
		break;
	default:
		break;
	}
}

/*
 * Callback to get the data from socket.
 *
 * sock: socket handler.
 * u8Msg: socket event type. Possible values are:
 *   SOCKET_MSG_BIND,
 *   SOCKET_MSG_LISTEN,
 *   SOCKET_MSG_ACCEPT,
 *   SOCKET_MSG_CONNECT,
 *   SOCKET_MSG_RECV,
 *   SOCKET_MSG_SEND,
 *   SOCKET_MSG_SENDTO,
 *   SOCKET_MSG_RECVFROM.
 * pvMsg: pointer to message structure. Existing types are:
 *   tstrSocketBindMsg,
 *   tstrSocketListenMsg,
 *   tstrSocketAcceptMsg,
 *   tstrSocketConnectMsg,
 *   tstrSocketRecvMsg.
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	/* Check for socket event on RX socket. */
	if (sock == s_rx_sock) {
		if (u8Msg == SOCKET_MSG_BIND) {
			tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg *)pvMsg;
			if (pstrBind && pstrBind->status == 0) {
				s_sock_bind_state = 1;
				/* Prepare for next buffer reception. */
				recvfrom(sock, s_sock_buf, WIFI_M2M_BUFFER_SIZE, 0);
				SYS_CONSOLE_PRINT("socket_cb: bind success!\r\n");
			} else {
				SYS_CONSOLE_PRINT("socket_cb: bind error!\r\n");
			}
		} else if (u8Msg == SOCKET_MSG_RECVFROM) {
			tstrSocketRecvMsg *pstrRx = (tstrSocketRecvMsg *)pvMsg;
			if (pstrRx->pu8Buffer && pstrRx->s16BufferSize) {
				s_delay = 0;
				s_sock_bind_state = 1;
				SYS_CONSOLE_PRINT("socket_cb: received app message\r\n");
				/* Prepare for next buffer reception. */
				recvfrom(sock, s_sock_buf, WIFI_M2M_BUFFER_SIZE, 0);
			} else {
				if (pstrRx->s16BufferSize == SOCK_ERR_TIMEOUT) {
					/* Prepare for next buffer reception. */
					recvfrom(sock, s_sock_buf, WIFI_M2M_BUFFER_SIZE, 0);
				}
			}
		}
	} else if (sock == s_tx_sock) {
		if (u8Msg == SOCKET_MSG_SENDTO) {
			SYS_CONSOLE_PRINT("socket_cb: sendto success!\r\n");
			/* Prepare for next buffer reception. */
			recvfrom(s_rx_sock, s_sock_buf, WIFI_M2M_BUFFER_SIZE, 0);
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
    registerSocketCallback(socket_cb, NULL);

    /* Connect to router. */
    ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID), WLAN_AUTH, (char *)WLAN_PSK, M2M_WIFI_CH_ALL);

    return ret;
}

static int8_t udp(void)
{
	int8_t ret;
	static struct sockaddr_in addr;

	if (s_packet_cnt >= WIFI_M2M_PACKET_COUNT) {
		close(s_rx_sock);
		close(s_tx_sock);
		s_rx_sock = -1;
		s_tx_sock = -1;
		return -1;
	}

	if (addr.sin_family == 0) {
		/* Initialize socket address structure. */
		addr.sin_family = AF_INET;
		addr.sin_port = _htons(WIFI_M2M_SERVER_PORT);
		addr.sin_addr.s_addr = nmi_inet_addr(WIFI_M2M_SERVER_IP);
	}

	s_ms_ticks = SYS_TMR_TickCountGet();

	if (s_ms_ticks - s_delay > WIFI_M2M_REPORT_INTERVAL) {
		s_delay = s_ms_ticks;
		/* Create socket for UDP RX. */
		if (s_rx_sock < 0) {
			if ((s_rx_sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
				SYS_CONSOLE_PRINT("Failed to create UDP server RX socket!\r\n");
				return -1;
			}

			/* Socket bind. */
			bind(s_rx_sock, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
		}

		/* Create socket for UDP TX. */
		if (s_tx_sock < 0) {
			uint32 u32EnableCallbacks = 0;
			if ((s_tx_sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
				SYS_CONSOLE_PRINT("Failed to create UDP client TX socket!\r\n");
				return -1;
			}
			setsockopt(s_tx_sock, SOL_SOCKET, SO_SET_UDP_SEND_CALLBACK, &u32EnableCallbacks, 0);
		}

		if (s_sock_bind_state == 1) {
			s_sock_bind_state = 0;

			/* Send client discovery frame. */
			ret = sendto(s_tx_sock, UDP_SEND_MESSAGE, sizeof(UDP_SEND_MESSAGE), 0, (struct sockaddr *)&addr, sizeof(addr));

			if (ret == M2M_SUCCESS) {
				SYS_CONSOLE_PRINT("Message sent\r\n");
				if (++s_packet_cnt >= WIFI_M2M_PACKET_COUNT)
					SYS_CONSOLE_PRINT("UDP Example complete!\r\n");
			} else {
				SYS_CONSOLE_PRINT("Failed to send status report!\r\n");
			}
		}
	}

	return 0;
}

void app_init(void)
{
	SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

	app_state_set(APP_RADIO_INIT);
	s_connected = false;
	s_packet_cnt = 0;
	s_ms_ticks = 0;
	s_delay = 0;
	s_rx_sock = -1;
	s_tx_sock = -1;
	s_sock_bind_state = 0;
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
		ret = udp();
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

#endif /* UDP_EXAMPLE */

//DOM-IGNORE-END
