/*******************************************************************************
  WINC1500 SSL Server Example

  File Name:
    ssl_server.c

  Summary:
    WINC1500 SSL Server Example

  Description:
    This example demonstrates the use of SSL server with the WINC1500.
    Example starts SoftAP and a dummy HTTP server to show the server functions.
    Once the example is up, pick up your phone, connect to the SoftAP and open
    your web browser, then type https://192.168.1.1. You should see "Microchip
    WINC1500 SSL server example" string on your browser. The SSL server uses
    the default self-signed certificate chain stored in the flash of WINC1500.
    Note SSL server function requires 19.5.2 FW or later. Older FW does not
    support it.

    The configuration defines for this demo are:
        SSL_APP_AP_SSID         -- WINC1500's SSID in AP mode
        SSL_APP_AP_SEC          -- WINC1500's security in AP mode
        SSL_APP_AP_PWD          -- Passphrase for WPA security
        SSL_APP_AP_KEY_SZ       -- Passphrase length for WPA security
        SSL_APP_AP_CHANNEL      -- WINC1500's working channel
        SSL_APP_AP_SSID_MODE    -- If WINC1500's SSID is visible in AP mode
        SSL_APP_PROV_HTTPS_IP   -- WINC1500 HTTP server's IP address in provision mode
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

#if SSL_SERVER_EXAMPLE

#define SSL_APP_AP_SSID					"WINC1500_HOTSPOT"
/*
 * Note WPA2 security in SoftAP is supported by 19.5.2 FW or later. 
 * The older FW supports only WEP or open security.
 */
#define SSL_APP_AP_SEC					M2M_WIFI_SEC_WPA_PSK /* WINC1500's security, M2M_WIFI_SEC_WPA_PSK, M2M_WIFI_SEC_WEP or M2M_WIFI_SEC_OPEN */
#define SSL_APP_AP_PWD					"12345678" /* Passphrase for WPA security */
#define SSL_APP_AP_KEY_SZ				sizeof(SSL_APP_AP_PWD) - 1 /* Passphrase length for WPA security */
#define SSL_APP_AP_CHANNEL				M2M_WIFI_CH_6 /* WINC1500's working channel */
#define SSL_APP_AP_SSID_MODE			SSID_MODE_VISIBLE /* If WINC1500's SSID is visible in AP mode */
#define SSL_APP_PROV_HTTPS_IP			{192, 168, 1, 1} /* WINC1500 HTTP server's IP address in provision mode */

#define SSL_APP_WINC_HOTSPOT_CONF	\
{\
	SSL_APP_AP_SSID,			\
	SSL_APP_AP_CHANNEL,			\
	0, SSL_APP_AP_KEY_SZ, {0},	\
	(uint8)SSL_APP_AP_SEC,		\
	SSL_APP_AP_SSID_MODE,		\
	SSL_APP_PROV_HTTPS_IP,		\
	SSL_APP_AP_PWD				\
}

#define app_state_get() s_app_state
#define app_state_set(x) do { s_app_state = x; }  while (0)
#define is_link_up() s_connected

#define EXAMPLE_HEADER \
"\r\n===========================\r\n"\
    "WINC1500 SSL Server Example\r\n"\
    "===========================\r\n"

typedef enum
{
	APP_RADIO_INIT,
	APP_WIFI_OPEN,
	APP_WIFI_CONNECT_WAIT,
	APP_NET_UP,
	APP_HTTP_SERVER_RUN,
	APP_END,
	APP_PARK
} APP_STATE;

tstrM2MAPConfig	gstrApConf = SSL_APP_WINC_HOTSPOT_CONF;

static APP_STATE s_app_state;
static bool s_connected;
static bool s_job_done;

static int dummy_http_server_start(void)
{
	SOCKET	sock;
	int ret = 0;

	sock = socket(AF_INET, SOCK_STREAM, SOCKET_FLAGS_SSL);

	if(sock >= 0) {
		struct sockaddr_in	addr;
		
		addr.sin_family	= AF_INET;
		addr.sin_port = _htons(443);
		addr.sin_addr.s_addr = 0;
		bind(sock, (struct sockaddr *)&addr, sizeof(struct sockaddr_in));
		listen(sock, 1);
	} else {
		ret = -1;
	}

	return ret;
}

static void socket_cb(SOCKET sock, uint8 u8Msg, void *pvMsg)
{
	static uint8 rxBuf[1400];
	
	if(u8Msg == SOCKET_MSG_BIND) {
		tstrSocketBindMsg *pstrBind = (tstrSocketBindMsg *)pvMsg;
		SYS_CONSOLE_PRINT("Bind %d\r\n", pstrBind->status);
	} else if(u8Msg == SOCKET_MSG_LISTEN) {
		tstrSocketListenMsg	*pstrListen = (tstrSocketListenMsg*)pvMsg;
		SYS_CONSOLE_PRINT("Listen %d\r\n", pstrListen->status);
	} else if(u8Msg == SOCKET_MSG_ACCEPT) {
		tstrSocketAcceptMsg	*pstrAccept = (tstrSocketAcceptMsg*)pvMsg;
		if(pstrAccept->sock >= 0) {
			SYS_CONSOLE_PRINT("Accepted %d\r\n", pstrAccept->sock);
			recv(pstrAccept->sock, rxBuf, sizeof(rxBuf), 0);
		}
	} else if(u8Msg == SOCKET_MSG_RECV) {
		tstrSocketRecvMsg	*pstrRx = (tstrSocketRecvMsg*)pvMsg;
		if((pstrRx->pu8Buffer != NULL) && (pstrRx->s16BufferSize > 0)) {
			uint8 txBuf[256];
			uint8 ret;
			uint32 u32BufSize;
			char httpResponseHdr[] = "HTTP/1.0 200 OK\r\n"\
				"Content-type: text/html\r\n"\
				"Content-length: %d\r\n"\
				"\r\n%s";

			u32BufSize = sprintf((char *)txBuf, httpResponseHdr, strlen("<html>Microchip WINC1500 SSL server example</html>"), "<html>Microchip WINC1500 SSL server example</html>");
			ret = send(sock, txBuf, u32BufSize, 0);
			if (ret == 0) {
				close(sock);
				s_job_done = true;
				SYS_CONSOLE_PRINT("Server sent default page successfully.\r\n");
			}
		} else {
			close(sock);
			s_job_done = true;
			SYS_CONSOLE_PRINT("Server received an invalid request. Closing the server socket...\r\n");
		}
	}
}

static void wifi_cb(uint8 u8MsgType, void *pvMsg)
{
	if(u8MsgType == M2M_WIFI_RESP_CON_STATE_CHANGED)
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		
		SYS_CONSOLE_PRINT("Wi-Fi State :: %s ::\r\n", pstrWifiState->u8CurrState ? "CONNECTED" : "DISCONNECTED");
		
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			/* Just waiting until DHCP gets completed. */
		} else if(pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			s_connected = false;
		}
	} else if(u8MsgType == M2M_WIFI_REQ_DHCP_CONF) {
		tstrM2MIPConfig *pstrM2MIpConfig = (tstrM2MIPConfig *)pvMsg;
		uint8 *pu8IPAddress = (uint8 *)&pstrM2MIpConfig->u32StaticIP;
		
		SYS_CONSOLE_PRINT("DHCP IP Address :: %u.%u.%u.%u ::\r\n", pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		s_connected = true;
	}
}

static int wifi_open(void)
{
    tstrWifiInitParam param;
    int8_t ret;

    /* Initialize Wi-Fi parameters structure. */
    memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

    /* Initialize Wi-Fi driver with data and status callbacks. */
    param.pfAppWifiCb = wifi_cb;
    ret = m2m_wifi_init(&param);
    if (M2M_SUCCESS != ret) {
        SYS_CONSOLE_PRINT("m2m_wifi_init call error!(%d)\r\n", ret);
        return -1;
    }

	socketInit();
	registerSocketCallback(socket_cb, NULL);
	ret = m2m_wifi_enable_ap(&gstrApConf);

    return ret;
}

void app_init(void)
{
	SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

    s_app_state = APP_RADIO_INIT;
    s_connected = false;
	s_job_done = false;
}

void app_task(void)
{
    int ret;
    
    switch (app_state_get()) {
    case APP_RADIO_INIT:
        radio_init(NULL);
        app_state_set(APP_WIFI_OPEN);
        break;
    case APP_WIFI_OPEN:
        ret = wifi_open();
        if (ret)
            app_state_set(APP_RADIO_INIT);
        else
            app_state_set(APP_WIFI_CONNECT_WAIT);
        break;
    case APP_WIFI_CONNECT_WAIT:
        if (is_link_up())
            app_state_set(APP_NET_UP);
        break;
    case APP_NET_UP:
		ret = dummy_http_server_start();
		if (ret == 0)
			app_state_set(APP_HTTP_SERVER_RUN);
        break;
	case APP_HTTP_SERVER_RUN:
		if (s_job_done) {
			WDRV_TIME_DELAY(5000);
			app_state_set(APP_END);
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

#endif /* SSL_SERVER_EXAMPLE */

//DOM-IGNORE-END
