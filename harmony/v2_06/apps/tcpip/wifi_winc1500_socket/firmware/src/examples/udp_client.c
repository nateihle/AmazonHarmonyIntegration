/*******************************************************************************
  File Name:
    udp_client.c

  Summary:
    WINC1500 UDP client example.

  Description:
    This example demonstrates the use of UDP client socket with the WINC1500.

    The configuration defines for this demo are:
        WLAN_SSID               -- AP to connect to
        WLAN_AUTH               -- Security for the AP
        WLAN_PSK                -- Passphrase for WPA security
        WIFI_M2M_SERVER_IP      -- IP address for the UDP server
        WIFI_M2M_SERVER_PORT    -- Port number for the UDP server
        WIFI_M2M_BUFFER_SIZE    -- Size of the socket buffer holding the RX data
        WIFI_M2M_PACKET_COUNT   -- Number of UDP packets to send to the server
        UDP_SEND_MESSAGE        -- Customizable UDP packet content
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

#if UDP_CLIENT_EXAMPLE

#define WLAN_SSID                   "DEMO_AP" /**< Destination SSID */
#define WLAN_AUTH                   M2M_WIFI_SEC_WPA_PSK /**< Security manner such as M2M_WIFI_SEC_WPA_PSK and M2M_WIFI_SEC_OPEN */
#define WLAN_PSK                    "12345678" /**< Password for destination SSID */

#define WIFI_M2M_SERVER_IP          "192.168.1.100"
#define WIFI_M2M_SERVER_PORT        6666
#define WIFI_M2M_BUFFER_SIZE        1460
#define WIFI_M2M_PACKET_COUNT       10 /* UDP MAX Packet Count */
#define UDP_SEND_MESSAGE            "UDP message from WINC1500 module\r\n"

#define EXAMPLE_HEADER \
"\r\n===========================\r\n"\
    "WINC1500 UDP Client Example\r\n"\
    "===========================\r\n"

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
static bool s_connected; /* Connection Flag */
static uint8_t s_packet_count; /* UDP Packet Count */
static bool s_sendto_recv;
static SOCKET s_tx_socket; /* Socket for TX */
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

static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	if (u8Msg == SOCKET_MSG_SENDTO) {
        SYS_CONSOLE_PRINT("socket_cb: sendto success (%u)!\r\n", s_packet_count);
        /* Prepare for next buffer reception. */
        recvfrom(sock, s_sock_buf, WIFI_M2M_BUFFER_SIZE, 0);
    } else if (u8Msg == SOCKET_MSG_RECVFROM) {
        tstrSocketRecvMsg *pstrRx = (tstrSocketRecvMsg *)pvMsg;
        if (pstrRx->pu8Buffer && pstrRx->s16BufferSize) {
            int i, count;
            SYS_CONSOLE_PRINT("socket_cb: received app message, size = %d [", pstrRx->s16BufferSize);
            count = (pstrRx->s16BufferSize > 20) ? 20 : pstrRx->s16BufferSize;
            for (i = 0; i < count; i++) {
                if (pstrRx->pu8Buffer[i] != '\0')
                    SYS_CONSOLE_PRINT("%c", pstrRx->pu8Buffer[i]);
            }
            if (pstrRx->s16BufferSize > 20)
                SYS_CONSOLE_PRINT("...");
            SYS_CONSOLE_PRINT("]\r\n");
        }
        s_sendto_recv = true;
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

static int8_t udp_client_run(void)
{
	static struct sockaddr_in addr;

	/* Create socket for UDP TX. */
	if (s_tx_socket < 0) {
		if ((s_tx_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
			SYS_CONSOLE_PRINT("Failed to create UDP client TX socket!\r\n");
			return -1;
		}
		/* Initialize socket address structure. */
		addr.sin_family = AF_INET;
		addr.sin_port = _htons(WIFI_M2M_SERVER_PORT);
		addr.sin_addr.s_addr = nmi_inet_addr(WIFI_M2M_SERVER_IP);
	}

	if (s_packet_count == 0) {
		sendto(s_tx_socket, UDP_SEND_MESSAGE, sizeof(UDP_SEND_MESSAGE), 0, (struct sockaddr *)&addr, sizeof(addr));
		s_packet_count++;
		s_sendto_recv = false;
	} else if (s_packet_count < WIFI_M2M_PACKET_COUNT) {
		if (s_sendto_recv) {
			sendto(s_tx_socket, UDP_SEND_MESSAGE, sizeof(UDP_SEND_MESSAGE), 0, (struct sockaddr *)&addr, sizeof(addr));
			s_packet_count++;
			s_sendto_recv = false;
		}
	} else {
		if (s_sendto_recv) {
			s_sendto_recv = false;
			close(s_tx_socket);
			s_tx_socket = -1;
			SYS_CONSOLE_PRINT("UDP Client Example complete!\r\n");
			return -1;
		}
	}

	return 0;
}

void app_init(void)
{
    SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

	app_state_set(APP_RADIO_INIT);
	s_connected = false;
	s_packet_count = 0;
	s_sendto_recv = false;
	s_tx_socket = -1;
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
		ret = udp_client_run();
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

#endif /* #if UDP_CLIENT_EXAMPLE */

//DOM-IGNORE-END