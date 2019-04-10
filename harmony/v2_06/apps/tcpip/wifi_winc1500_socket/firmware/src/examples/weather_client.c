/*******************************************************************************
  File Name:
    weather_client.c

  Summary:
    WINC1500 Weather Client Example

  Description:
    This example demonstrates retrieving weather data by use of HTTP protocol
    with the WINC1500.

    The configuration defines for this demo are:
        WLAN_SSID               -- AP to connect to
        WLAN_AUTH               -- Security for the AP
        WLAN_PSK                -- Passphrase for WPA security
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

#if WEATHER_CLIENT_EXAMPLE

/** Wi-Fi settings */
#define WLAN_SSID                 "DEMO_AP" /**< Destination SSID */
#define WLAN_AUTH                 M2M_WIFI_SEC_WPA_PSK /**< Security manner such as M2M_WIFI_SEC_WPA_PSK and M2M_WIFI_SEC_OPEN */
#define WLAN_PSK                  "12345678" /**< Password for destination SSID */

#define CITY_NAME                 "paris" /** Input city name */

#define WEATHER_SERVER_NAME       "api.openweathermap.org" /** Weather information provider server */
#define SERVER_PORT               (80) /** Using broadcast address for simplicity */
/** Send buffer of TCP socket */
#define PREFIX_BUFFER             "GET /data/2.5/weather?q="
#define POST_BUFFER               "&appid=c592e14137c3471fa9627b44f6649db4&mode=xml&units=metric HTTP/1.1\r\nHost: api.openweathermap.org\r\nAccept: */*\r\n\r\n"

#define WIFI_M2M_BUFFER_SIZE      1400 /** Receive buffer size */

#define EXAMPLE_HEADER \
"\r\n===============================\r\n"\
    "WINC1500 Weather Client Example\r\n"\
    "===============================\r\n"

#define IPV4_BYTE(val, index)     ((val >> (index * 8)) & 0xFF) /** IP address parsing */

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

static uint32_t s_u32HostIp; /** IP address of host */
static SOCKET tcp_client_socket; /** TCP client socket handler */

/** Receive buffer definition */
static uint8_t s_ReceivedBuffer[WIFI_M2M_BUFFER_SIZE];
static char s_CityNameBuf[32];
static char s_TemperatureBuf[32];
static char s_WeatherConditionBuf[32];

static APP_STATE s_app_state; /** Demo state */
static bool s_connected;
static bool s_bHostIpByName; /** Get host IP status variable */
static bool s_bTcpConnection; /** TCP connection status variable */
static bool s_demo_completed;

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
        break;
    }
    case M2M_WIFI_REQ_DHCP_CONF:
    {
        uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
        SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n", pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
        s_connected = true;

        /* Obtain the IP Address by network name */
        gethostbyname((uint8_t *)WEATHER_SERVER_NAME);
        break;
    }
    default:
        break;
    }
}

static void resolve_cb(uint8 *hostName, uint32 hostIp)
{
    s_u32HostIp = hostIp;
    s_bHostIpByName = true;
    SYS_CONSOLE_PRINT("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", hostName,
            (int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),
            (int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
}

static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
    /* Check for socket event on TCP socket. */
    if (sock == tcp_client_socket) {
        switch (u8Msg) {
        case SOCKET_MSG_CONNECT:
        {
            if (s_bTcpConnection) {
                memset(s_ReceivedBuffer, 0, sizeof(s_ReceivedBuffer));
                sprintf((char *)s_ReceivedBuffer, "%s%s%s", PREFIX_BUFFER, (char *)CITY_NAME, POST_BUFFER);

                tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
                if (pstrConnect && pstrConnect->s8Error >= SOCK_ERR_NO_ERROR) {
                    send(tcp_client_socket, s_ReceivedBuffer, strlen((char *)s_ReceivedBuffer), 0);

                    memset(s_ReceivedBuffer, 0, WIFI_M2M_BUFFER_SIZE);
                    recv(tcp_client_socket, &s_ReceivedBuffer[0], WIFI_M2M_BUFFER_SIZE, 0);
                } else {
                    SYS_CONSOLE_PRINT("socket_cb: connect error!\r\n");
                    s_bTcpConnection = false;
                    close(tcp_client_socket);
                    tcp_client_socket = -1;
                }
            }
        }
        break;

        case SOCKET_MSG_RECV:
        {
            char *pcIndxPtr;
            char *pcEndPtr;

            tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
            if (pstrRecv && pstrRecv->s16BufferSize > 0) {

                /* Get city name. */
                pcIndxPtr = strstr((char *)pstrRecv->pu8Buffer, "name=");
                if (NULL != pcIndxPtr) {
                    pcIndxPtr = pcIndxPtr + strlen("name=") + 1;
                    pcEndPtr = strstr(pcIndxPtr, "\">");
                    if (NULL != pcEndPtr) {
                        *pcEndPtr = 0;
                    }
                    strcpy(s_CityNameBuf, pcIndxPtr);
                } else {
                    SYS_CONSOLE_PRINT("N/A\r\n");
                    break;
                }

                /* Get temperature. */
                pcIndxPtr = strstr(pcEndPtr + 1, "temperature value");
                if (NULL != pcIndxPtr) {
                    pcIndxPtr = pcIndxPtr + strlen("temperature value") + 2;
                    pcEndPtr = strstr(pcIndxPtr, "\" ");
                    if (NULL != pcEndPtr) {
                        *pcEndPtr = 0;
                    }
                    strcpy(s_TemperatureBuf, pcIndxPtr);
                } else {
                    SYS_CONSOLE_PRINT("N/A\r\n");
                    break;
                }

                /* Get weather condition. */
                pcIndxPtr = strstr(pcEndPtr + 1, "weather number");
                if (NULL != pcIndxPtr) {
                    pcIndxPtr = pcIndxPtr + strlen("weather number") + 14;
                    pcEndPtr = strstr(pcIndxPtr, "\" ");
                    if (NULL != pcEndPtr) {
                        *pcEndPtr = 0;
                    }
                    strcpy(s_WeatherConditionBuf, pcIndxPtr);
                    s_demo_completed = true;
                    /* Response processed, now close connection. */
                    close(tcp_client_socket);
                    tcp_client_socket = -1;
                    break;
                }

                memset(s_ReceivedBuffer, 0, sizeof(s_ReceivedBuffer));
                recv(tcp_client_socket, &s_ReceivedBuffer[0], WIFI_M2M_BUFFER_SIZE, 0);
            } else {
                SYS_CONSOLE_PRINT("socket_cb: recv error!\r\n");
                close(tcp_client_socket);
                tcp_client_socket = -1;
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
    if (M2M_SUCCESS != ret) {
        SYS_CONSOLE_PRINT("m2m_wifi_init call error!(%d)\r\n", ret);
        return ret;
    }

    /* Initialize socket module. */
    socketInit();
    registerSocketCallback(socket_cb, resolve_cb);

    /* Connect to router. */
    ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID), WLAN_AUTH, (char *)WLAN_PSK, M2M_WIFI_CH_ALL);

    return ret;
}

static int weather_client_run(void)
{
    int ret = 0;

    struct sockaddr_in addr_in;
    if (is_link_up() && !s_bTcpConnection) {
        if (s_bHostIpByName) {
            /* Open TCP client socket. */
            if (tcp_client_socket < 0) {
                if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
                    SYS_CONSOLE_PRINT("failed to create TCP client socket error!\r\n");
                    return -1;
                }
            }

            /* Connect TCP client socket. */
            addr_in.sin_family = AF_INET;
            addr_in.sin_port = _htons(SERVER_PORT);
            addr_in.sin_addr.s_addr = s_u32HostIp;
            if (connect(tcp_client_socket, (struct sockaddr *)&addr_in, sizeof(struct sockaddr_in)) != SOCK_ERR_NO_ERROR) {
                SYS_CONSOLE_PRINT("failed to connect socket error!\r\n");
                return -1;
            }

            s_bTcpConnection = true;
        }
    }

    if (s_demo_completed) {
        SYS_CONSOLE_PRINT("\r\n**********************************\r\n");
        SYS_CONSOLE_PRINT("\r\n  City:  %s\r\n", s_CityNameBuf);
        SYS_CONSOLE_PRINT("\r\n  Temperature: %s\r\n", s_TemperatureBuf);
        SYS_CONSOLE_PRINT("\r\n  Weather Condition: %s\r\n", s_WeatherConditionBuf);
        SYS_CONSOLE_PRINT("\r\n**********************************\r\n");
        SYS_CONSOLE_PRINT("Testing is done !\r\n");
        ret = -1;
    }
	
	return ret;
}

void app_init(void)
{
    SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

    app_state_set(APP_RADIO_INIT);
    s_u32HostIp = 0;
    tcp_client_socket = -1;
    s_connected = false;
    s_bHostIpByName = false;
    s_bTcpConnection = false;
    s_demo_completed = false;
    memset(s_ReceivedBuffer, 0x00, sizeof(s_ReceivedBuffer));
    memset(s_CityNameBuf, 0x00, sizeof(s_CityNameBuf));
    memset(s_TemperatureBuf, 0x00, sizeof(s_TemperatureBuf));
    memset(s_WeatherConditionBuf, 0x00, sizeof(s_WeatherConditionBuf));
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
        ret = weather_client_run();
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

#endif /* WEATHER_CLIENT_EXAMPLE */

//DOM-IGNORE-END
