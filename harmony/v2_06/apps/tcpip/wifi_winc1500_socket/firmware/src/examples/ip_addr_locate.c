/*******************************************************************************
  File Name:
    ip_addr_locate.c

  Summary:
    This example demonstrates the use of the WINC1500 to get GPS position 
    from my current IP address.

  Description:
    This example demonstrates the use of the WINC1500 to get GPS position 
    from my current IP address.

    The configuration defines for this demo are:
        WLAN_SSID               -- AP to connect to
        WLAN_AUTH               -- Security for the AP
        WLAN_PSK                -- Passphrase for WPA security
        HTTP_CLIENT_TEST_URL    -- A third party server, "http://ipinfo.io"
        HTTP_CLIENT_TEST_METHOD -- HTTP method to retrieve the data
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

#if IP_ADDR_LOCATE_EXAMPLE

#include "iot/iot_error.h"
#include "socket.h"
#include "iot/http/http_client.h"
#include "iot/json.h"

/** Wi-Fi AP Settings */
#define WLAN_SSID                   "DEMO_AP" /**< Destination SSID */
#define WLAN_AUTH                   M2M_WIFI_SEC_WPA_PSK /**< Security manner such as M2M_WIFI_SEC_WPA_PSK and M2M_WIFI_SEC_OPEN */
#define WLAN_PSK                    "12345678" /**< Password for destination SSID */

/** URL which will be requested */
#define HTTP_CLIENT_TEST_URL        "http://ipinfo.io"

/** Method of TEST request */
#define HTTP_CLIENT_TEST_METHOD     HTTP_METHOD_GET

/** IP address parsing */
#define IPV4_BYTE(val, index)       ((val >> (index * 8)) & 0xFF)

#define EXAMPLE_HEADER \
"\r\n==================================\r\n"\
    "WINC1500 IP Address Locate Example\n"\
    "==================================\r\n"

#define app_state_get() s_app_state
#define app_state_set(x) do {s_app_state = x;} while (0)
#define is_link_up() s_connected

typedef enum {
    APP_RADIO_INIT,
    APP_HTTP_INIT,
    APP_WIFI_OPEN,
    APP_WIFI_CONNECT_WAIT,
    APP_NET_UP,
    APP_END,
    APP_PARK
} APP_STATE;

/** Instance of timer module */
struct sw_timer_module swt_module_inst;

/** Instance of HTTP client module */
struct http_client_module http_client_module_inst;

/** Demo state */
static APP_STATE s_app_state;

/** Connection flag */
static bool s_connected;
static bool s_demo_completed;

static char s_regionBuf[32];
static char s_countryBuf[32];
static char s_locationBuf[64];
static char s_httpRecvBuf[256];

static void configure_timer(void);

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
    switch (u8MsgType) {
    case M2M_WIFI_RESP_CON_STATE_CHANGED:
    {
        tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
        if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
            SYS_CONSOLE_PRINT("Wi-Fi connected\r\n");
        } else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
            int8_t connect_ret;
            s_connected = false;
            SYS_CONSOLE_PRINT("Wi-Fi disconnected\r\n");
            connect_ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID),
                    WLAN_AUTH, (char *)WLAN_PSK, M2M_WIFI_CH_ALL);
            if (connect_ret != M2M_SUCCESS) {
                SYS_CONSOLE_PRINT("m2m_wifi_connect call error!(%d)\r\n", connect_ret);
            }
        }
        break;
    }

    case M2M_WIFI_REQ_DHCP_CONF:
    {
        uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
        SYS_CONSOLE_PRINT("Wi-Fi IP is %u.%u.%u.%u\r\n",
                pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
        s_connected = true;
        /* Send the HTTP request. */
        http_client_send_request(&http_client_module_inst, HTTP_CLIENT_TEST_URL, HTTP_CLIENT_TEST_METHOD, NULL, NULL);
        break;
    }

    default:
        break;
    }
}

static void http_client_callback(struct http_client_module *module_inst, int type, union http_client_data *data)
{
    struct json_obj json, loc, region, country;
    switch (type) {
    case HTTP_CLIENT_CALLBACK_SOCK_CONNECTED:
        SYS_CONSOLE_PRINT("http_client_callback: HTTP client socket connected.\r\n");
        break;

    case HTTP_CLIENT_CALLBACK_REQUESTED:
        SYS_CONSOLE_PRINT("http_client_callback: request completed.\r\n");
        break;

    case HTTP_CLIENT_CALLBACK_RECV_RESPONSE:
        SYS_CONSOLE_PRINT("http_client_callback: received response %u data size %u\r\n",
                (unsigned int)data->recv_response.response_code,
                (unsigned int)data->recv_response.content_length);
        if (data->recv_response.content != NULL) {
            if (json_create(&json, data->recv_response.content, data->recv_response.content_length) == 0 &&
                    json_find(&json, "region", &region) == 0) {
                strcpy(s_regionBuf, region.value.s);
            }
            if (json_create(&json, data->recv_response.content, data->recv_response.content_length) == 0 &&
                    json_find(&json, "country", &country) == 0) {
                strcpy(s_countryBuf, country.value.s);
            }
            if (json_create(&json, data->recv_response.content, data->recv_response.content_length) == 0 &&
                    json_find(&json, "loc", &loc) == 0) {
                strcpy(s_locationBuf, loc.value.s); 
            }
            s_demo_completed = true;
        }
        break;

    case HTTP_CLIENT_CALLBACK_DISCONNECTED:
        SYS_CONSOLE_PRINT("http_client_callback: disconnection reason:%d\r\n", data->disconnected.reason);

        /* If disconnect reason is equal to -ECONNRESET(-104),
         * It means the server has closed the connection by the keep alive timeout.
         * This is normal operation.
         */
        if (data->disconnected.reason == -EAGAIN) {
            /* Server has not responded. Retry it immediately. */
            http_client_send_request(&http_client_module_inst, HTTP_CLIENT_TEST_URL, HTTP_CLIENT_TEST_METHOD, NULL, NULL);
        }
        break;
    }
}

static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
    http_client_socket_event_handler(sock, u8Msg, pvMsg);
}

static void resolve_cb(uint8 *pu8DomainName, uint32 u32ServerIP)
{
    SYS_CONSOLE_PRINT("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", pu8DomainName,
            (int)IPV4_BYTE(u32ServerIP, 0), (int)IPV4_BYTE(u32ServerIP, 1),
            (int)IPV4_BYTE(u32ServerIP, 2), (int)IPV4_BYTE(u32ServerIP, 3));
    http_client_socket_resolve_handler(pu8DomainName, u32ServerIP);
}

static int8_t configure_http_client(void)
{
    struct http_client_config httpc_conf;
    int ret;

    http_client_get_config_defaults(&httpc_conf);

    httpc_conf.recv_buffer_size = sizeof(s_httpRecvBuf);
    httpc_conf.timer_inst = &swt_module_inst;
    /* ipinfo.io send json format data if only client is a curl. */
    httpc_conf.user_agent = "curl/7.10.6";

    ret = http_client_init(&http_client_module_inst, &httpc_conf, s_httpRecvBuf);
    if (ret < 0) {
        SYS_CONSOLE_PRINT("configure_http_client: HTTP client initialization has failed(%s)\r\n", strerror(ret));
        return ret;
    }

    http_client_register_callback(&http_client_module_inst, http_client_callback);
    return 0;
}

static void configure_timer(void)
{
    struct sw_timer_config swt_conf;
    sw_timer_get_config_defaults(&swt_conf);

    sw_timer_init(&swt_module_inst, &swt_conf);
    sw_timer_enable(&swt_module_inst);
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
    SYS_CONSOLE_PRINT("connecting to WiFi AP %s...\r\n", (char *)WLAN_SSID);
    ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID), WLAN_AUTH, (char *)WLAN_PSK, M2M_WIFI_CH_ALL);
    if (M2M_SUCCESS != ret) {
        SYS_CONSOLE_PRINT("failed to connect access point!\r\n");
        return ret;
    }

    return ret;
}

static int8_t ip_addr_locate_run(void)
{
    int8_t ret = 0;
    
    sw_timer_task(&swt_module_inst);
    
    if (s_demo_completed) {
        SYS_CONSOLE_PRINT("\r\n******************************************\r\n");
        SYS_CONSOLE_PRINT("\r\n Region:                %s\r\n", s_regionBuf);
        SYS_CONSOLE_PRINT("\r\n Country:               %s\r\n", s_countryBuf);
        SYS_CONSOLE_PRINT("\r\n Latitude & longitude:  %s\r\n", s_locationBuf);
        SYS_CONSOLE_PRINT("\r\n******************************************\r\n");
        SYS_CONSOLE_PRINT("Testing is done !\r\n");
        ret = -1;
    }

    return ret;
}

void app_init(void)
{
    SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

    app_state_set(APP_RADIO_INIT);
    s_connected = false;
    s_demo_completed = false;
    memset(s_regionBuf, 0x00, sizeof(s_regionBuf));
    memset(s_countryBuf, 0x00, sizeof(s_countryBuf));
    memset(s_locationBuf, 0x00, sizeof(s_locationBuf));
}

static int32_t http_init(void)
{
    int32_t ret;

    /* Initialize the Timer. */
    configure_timer();
    /* Initialize the HTTP client service. */
    ret = configure_http_client();

    return ret;
}

void app_task(void)
{
    int8_t ret = 0;

    switch (app_state_get()) {
    case APP_RADIO_INIT:
        radio_init(NULL);
        app_state_set(APP_HTTP_INIT);
        break;
    case APP_HTTP_INIT:
        ret = http_init();
        if (ret)
            app_state_set(APP_END);
        else
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
        ret = ip_addr_locate_run();
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

#endif /* IP_ADDR_LOCATE_EXAMPLE */

//DOM-IGNORE-END
