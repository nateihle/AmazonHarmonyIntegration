/*******************************************************************************
  File Name:
    time_client.c

  Summary:
    WINC1500 time client example.

  Description:
    This example demonstrates retrieving time data by use of SNTP protocol
    with the WINC1500.

    The configuration defines for this demo are:
        WLAN_SSID                   -- AP to connect to
        WLAN_AUTH                   -- Security for the AP
        WLAN_PSK                    -- Passphrase for WPA security
        SELECTED_NTP_POOL_HOSTNAME  -- NTP server URL
        SERVER_PORT_FOR_UDP         -- Port number of the NTP server
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

#if TIME_CLIENT_EXAMPLE

/** Wi-Fi settings */
#define WLAN_SSID                          "DEMO_AP" /**< Destination SSID */
#define WLAN_AUTH                          M2M_WIFI_SEC_WPA_PSK /**< Security manner such as M2M_WIFI_SEC_WPA_PSK and M2M_WIFI_SEC_OPEN */
#define WLAN_PSK                           "12345678" /**< Password for destination SSID */

/** Using NTP server information */
#define WORLDWIDE_NTP_POOL_HOSTNAME        "pool.ntp.org"
#define ASIA_NTP_POOL_HOSTNAME             "asia.pool.ntp.org"
#define EUROPE_NTP_POOL_HOSTNAME           "europe.pool.ntp.org"
#define NAMERICA_NTP_POOL_HOSTNAME         "north-america.pool.ntp.org"
#define OCEANIA_NTP_POOL_HOSTNAME          "oceania.pool.ntp.org"
#define SAMERICA_NTP_POOL_HOSTNAME         "south-america.pool.ntp.org"
#define SELECTED_NTP_POOL_HOSTNAME         WORLDWIDE_NTP_POOL_HOSTNAME

#define SERVER_PORT_FOR_UDP                123
#define WIFI_M2M_BUFFER_SIZE               1460

#define EXAMPLE_HEADER \
"\r\n============================\r\n"\
    "WINC1500 Time Client Example\r\n"\
    "============================\r\n"

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

/** UDP socket handler */
static SOCKET s_udp_socket;

/** Receive buffer definition */
static uint8_t s_socket_buffer[WIFI_M2M_BUFFER_SIZE];
static uint8_t s_time_buffer[32];

/** Demo state */
static APP_STATE s_app_state;
static bool s_connected;
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
        gethostbyname((uint8_t *)SELECTED_NTP_POOL_HOSTNAME);
        break;
    }
    default:
        break;
    }
}

static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
    /* Check for socket event on socket. */
    int16_t ret;

    switch (u8Msg) {
    case SOCKET_MSG_RECVFROM:
    {
        tstrSocketRecvMsg *pstrRx = (tstrSocketRecvMsg *)pvMsg;
        if (pstrRx->pu8Buffer && pstrRx->s16BufferSize) {
            uint8_t packetBuffer[48];
            memcpy(&packetBuffer, pstrRx->pu8Buffer, sizeof(packetBuffer));

            if ((packetBuffer[0] & 0x7) != 4) {                   /* expect only server response */
                SYS_CONSOLE_PRINT("socket_cb: Expecting response from Server Only!\r\n");
                s_demo_completed = true;
                return;                    /* MODE is not server, abort */
            } else {
                uint32_t secsSince1900 = packetBuffer[40] << 24 |
                        packetBuffer[41] << 16 |
                        packetBuffer[42] << 8 |
                        packetBuffer[43];

                /* Now convert NTP time into everyday time.
                 * Unix time starts on Jan 1 1970. In seconds, that is 2208988800.
                 * Subtract seventy years.
                */
                const uint32_t seventyYears = 2208988800UL;
                uint32_t epoch = secsSince1900 - seventyYears;

                // GMT is the time at Greenwich Meridian.
                sprintf((char *)s_time_buffer, "The GMT time is %lu:%02u:%02u",
                        (epoch  % 86400L) / 3600,           /* hour (86400 equals secs per day) */
                        (epoch  % 3600) / 60,               /* minute (3600 equals secs per minute) */
                        (epoch % 60));                        /* second */
                s_demo_completed = true;
                ret = close(sock);
                if (ret == SOCK_ERR_NO_ERROR) {
                    s_udp_socket = -1;
                }
            }
        }
        break;
    }
	case SOCKET_MSG_SENDTO:
		ret = recvfrom(sock, s_socket_buffer, WIFI_M2M_BUFFER_SIZE, 0);
        if (ret != SOCK_ERR_NO_ERROR) {
            SYS_CONSOLE_PRINT("socket_cb: recv error!\r\n");
        }
		break;
    default:
        break;
    }
}

static void resolve_cb(uint8 *pu8DomainName, uint32 u32ServerIP)
{
    struct sockaddr_in addr;
    int8_t cDataBuf[48];
    int16_t ret;

    memset(cDataBuf, 0, sizeof(cDataBuf));
    cDataBuf[0] = '\x1b'; /* time query */

    SYS_CONSOLE_PRINT("resolve_cb: DomainName %s\r\n", pu8DomainName);

    if (s_udp_socket >= 0) {
        /* Set NTP server socket address structure. */
        addr.sin_family = AF_INET;
        addr.sin_port = _htons(SERVER_PORT_FOR_UDP);
        addr.sin_addr.s_addr = u32ServerIP;

        /*Send an NTP time query to the NTP server*/
        ret = sendto(s_udp_socket, (int8_t *)&cDataBuf, sizeof(cDataBuf), 0, (struct sockaddr *)&addr, sizeof(addr));
        if (ret != M2M_SUCCESS) {
            SYS_CONSOLE_PRINT("resolve_cb: failed to send  error!\r\n");
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

static int time_client_run(void)
{
    int ret = 0;

    /*  Create the socket for the first time.*/
    if (s_udp_socket < 0) {
        s_udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (s_udp_socket < 0) {
            SYS_CONSOLE_PRINT("UDP Client Socket Creation Failed.\r\n");
            return -1;
        }
    }

    if (s_demo_completed) {
        SYS_CONSOLE_PRINT("\r\n**********************************\r\n");
        SYS_CONSOLE_PRINT("\r\n  %s\r\n", s_time_buffer);
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
    s_udp_socket = -1;
    s_connected = false;
    s_demo_completed = false;
    memset(s_time_buffer, 0x00, sizeof(s_time_buffer));
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
        ret = time_client_run();
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

#endif /* TIME_CLIENT_EXAMPLE */

//DOM-IGNORE-END
