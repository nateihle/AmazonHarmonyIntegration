/*******************************************************************************
  File Name:
    http_download.c

  Summary:
    This example demonstrates the use of the WINC1500 to download a file
    from website.

  Description:
    This example demonstrates the use of the WINC1500 to download a file
    from website.

    The configuration defines for this demo are:
        WLAN_SSID           -- AP to connect to
        WLAN_AUTH           -- Security for the AP
        WLAN_PSK            -- Passphrase for WPA security
        HTTP_FILE_URL       -- Link of file on website
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

#if HTTP_DOWNLOAD_EXAMPLE

#include "iot/iot_error.h"
#include "iot/http/http_client.h"

/** Wi-Fi AP Settings. */
#define WLAN_SSID               "DEMO_AP" /**< Destination SSID */
#define WLAN_AUTH               M2M_WIFI_SEC_WPA_PSK /**< Security manner such as M2M_WIFI_SEC_WPA_PSK and M2M_WIFI_SEC_OPEN */
#define WLAN_PSK                "12345678" /**< Password for destination SSID */

/** Content URI for download. */
#define HTTP_FILE_URL           "http://ww1.microchip.com/downloads/en/DeviceDoc/Using_MPLAB_Harmony_Help.pdf"

/** IP address parsing. */
#define IPV4_BYTE(val, index)   ((val >> (index * 8)) & 0xFF)

/** Maximum size for packet buffer. */
#define BUFFER_MAX_SIZE         (1446)

#define SIZE_OF_DOWNLOAF_BUFFER (2048)

#define EXAMPLE_HEADER \
"\r\n==============================\r\n"\
    "WINC1500 HTTP Download Example\r\n"\
    "==============================\r\n"

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

typedef enum {
    NOT_READY = 0, /*!< Not ready. */
    STORAGE_READY = 0x01, /*!< Storage is ready. */
    WIFI_CONNECTED = 0x02, /*!< Wi-Fi is connected. */
    GET_REQUESTED = 0x04, /*!< GET request is sent. */
    DOWNLOADING = 0x08, /*!< Running to download. */
    COMPLETED = 0x10, /*!< Download completed. */
    CANCELED = 0x20 /*!< Download canceled. */
} download_state;

/** File download processing state */
static download_state down_state;
/** HTTP content length */
static uint32_t http_file_size;
/** Receiving content length */
static uint32_t received_file_size;
/** Instance of Timer module */
struct sw_timer_module swt_module_inst;
/** Instance of HTTP client module */
struct http_client_module http_client_module_inst;
/** Demo state */
static APP_STATE s_app_state;
/** Connection flag */
static bool s_connected;
static uint8_t s_downloadBuf[SIZE_OF_DOWNLOAF_BUFFER];
static char s_httpRecvBuf[BUFFER_MAX_SIZE];

static void configure_timer(void)
{
    struct sw_timer_config swt_conf;
    sw_timer_get_config_defaults(&swt_conf);

    sw_timer_init(&swt_module_inst, &swt_conf);
    sw_timer_enable(&swt_module_inst);
}

static void init_state(void)
{
    down_state = NOT_READY;
}

static void clear_state(download_state mask)
{
    down_state &= ~mask;
}

static void add_state(download_state mask)
{
    down_state |= mask;
}

static inline bool is_state_set(download_state mask)
{
    return ((down_state & mask) != 0);
}

static void start_download(void)
{
    if (!is_state_set(WIFI_CONNECTED)) {
        SYS_CONSOLE_PRINT("start_download: Wi-Fi is not connected.\r\n");
        return;
    }

    if (is_state_set(GET_REQUESTED)) {
        SYS_CONSOLE_PRINT("start_download: request is sent already.\r\n");
        return;
    }

    if (is_state_set(DOWNLOADING)) {
        SYS_CONSOLE_PRINT("start_download: running download already.\r\n");
        return;
    }

    /* Send the HTTP request. */
    SYS_CONSOLE_PRINT("start_download: sending HTTP request...\r\n");
    http_client_send_request(&http_client_module_inst, HTTP_FILE_URL, HTTP_METHOD_GET, NULL, NULL);
}

static void save_file_packet(char *data, uint32_t length)
{
    int i;

    if ((data == NULL) || (length < 1)) {
        SYS_CONSOLE_PRINT("save_file_packet: empty data.\r\n");
        return;
    }

    if (!is_state_set(DOWNLOADING)) {
        received_file_size = 0;
        add_state(DOWNLOADING);
    }

    if (data != NULL) {
        SYS_CONSOLE_PRINT("save_file_packet: received[%lu], file size[%lu]\r\n", (unsigned long)received_file_size, (unsigned long)http_file_size);
        for (i = 0; i < length; i++) {
            if (received_file_size + i < SIZE_OF_DOWNLOAF_BUFFER)
                s_downloadBuf[received_file_size + i] = data[i];
        }
        received_file_size += length;
        if (received_file_size >= http_file_size) {
            SYS_CONSOLE_PRINT("print_file_packet: file downloaded successfully.\r\n");
            add_state(COMPLETED);
            return;
        }
    }
}

static void http_client_callback(struct http_client_module *module_inst, int type, union http_client_data *data)
{
    switch (type) {
    case HTTP_CLIENT_CALLBACK_SOCK_CONNECTED:
        SYS_CONSOLE_PRINT("http_client_callback: HTTP client socket connected.\r\n");
        break;

    case HTTP_CLIENT_CALLBACK_REQUESTED:
        SYS_CONSOLE_PRINT("http_client_callback: request completed.\r\n");
        add_state(GET_REQUESTED);
        break;

    case HTTP_CLIENT_CALLBACK_RECV_RESPONSE:
        SYS_CONSOLE_PRINT("http_client_callback: received response %u data size %u\r\n",
                (unsigned int)data->recv_response.response_code,
                (unsigned int)data->recv_response.content_length);
        if ((unsigned int)data->recv_response.response_code == 200) {
            http_file_size = data->recv_response.content_length;
            received_file_size = 0;
        }
        else {
            add_state(CANCELED);
            return;
        }
        if (data->recv_response.content_length <= BUFFER_MAX_SIZE) {
            save_file_packet(data->recv_response.content, data->recv_response.content_length);
            add_state(COMPLETED);
        }
        break;

    case HTTP_CLIENT_CALLBACK_RECV_CHUNKED_DATA:
        save_file_packet(data->recv_chunked_data.data, data->recv_chunked_data.length);
        if (data->recv_chunked_data.is_complete) {
            add_state(COMPLETED);
        }
        break;

    case HTTP_CLIENT_CALLBACK_DISCONNECTED:
        SYS_CONSOLE_PRINT("http_client_callback: disconnection reason:%d\r\n", data->disconnected.reason);

        /* If disconnect reason is equal to -ECONNRESET(-104),
         * It means the server has closed the connection (timeout).
         * This is normal operation.
         */
        if (data->disconnected.reason == -EAGAIN) {
            /* Server has not responded. Retry immediately. */
            if (is_state_set(DOWNLOADING)) {
                clear_state(DOWNLOADING);
            }

            if (is_state_set(GET_REQUESTED)) {
                clear_state(GET_REQUESTED);
            }

            start_download();
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

            clear_state(WIFI_CONNECTED);
            if (is_state_set(DOWNLOADING)) {
                clear_state(DOWNLOADING);
            }

            if (is_state_set(GET_REQUESTED)) {
                clear_state(GET_REQUESTED);
            }

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
        SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n", pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
        s_connected = true;
        add_state(WIFI_CONNECTED);
        start_download();
        break;
    }
    default:
        break;
    }
}

static int8_t configure_http_client(void)
{
    struct http_client_config httpc_conf;
    int ret;

    http_client_get_config_defaults(&httpc_conf);

    httpc_conf.recv_buffer_size = sizeof(s_httpRecvBuf);
    httpc_conf.timer_inst = &swt_module_inst;

    ret = http_client_init(&http_client_module_inst, &httpc_conf, s_httpRecvBuf);
    if (ret < 0) {
        SYS_CONSOLE_PRINT("configure_http_client: HTTP client initialization failed! (res %d)\r\n", ret);
        return ret;
    }

    http_client_register_callback(&http_client_module_inst, http_client_callback);

    return 0;
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

    return ret;
}

static int8_t http_download_run(void)
{
    int i;

    if (!(is_state_set(COMPLETED) || is_state_set(CANCELED))) {
        /* Checks the timer timeout. */
        sw_timer_task(&swt_module_inst);
        return 0;
    } else {
        SYS_CONSOLE_PRINT("\r\n******************************************\r\n");
        SYS_CONSOLE_PRINT("File size is %d\r\n", http_file_size);
        SYS_CONSOLE_PRINT("File content is:\r\n");
        for (i = 0; i < (received_file_size < SIZE_OF_DOWNLOAF_BUFFER ? received_file_size : SIZE_OF_DOWNLOAF_BUFFER); i++)
        {
            SYS_CONSOLE_PRINT("%02x ", s_downloadBuf[i]);
            if ((i + 1) % 10 == 0) {
                WDRV_TIME_DELAY(50);
            }
        }
        SYS_CONSOLE_PRINT("\r\n******************************************\r\n");
        SYS_CONSOLE_PRINT("done.\r\n");
        return 1;
    }
}

void app_init(void)
{
    SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

    app_state_set(APP_RADIO_INIT);
    s_connected = false;
    down_state = NOT_READY;
    http_file_size = 0;
    received_file_size = 0;
    memset(s_downloadBuf, 0x00, sizeof(s_downloadBuf));
}


static int32_t http_init(void)
{
    int32_t ret;

    init_state();
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
        ret = http_download_run();
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

#endif /* HTTP_DOWNLOAD_EXAMPLE */

//DOM-IGNORE-END
