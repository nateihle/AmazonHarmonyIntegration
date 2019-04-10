/*******************************************************************************
  File Name:
    pubnub_cloud.c

  Summary:
    PubNub Cloud Example

  Description:
    This WINC1500 example demonstrates the publish and subscribe by using
    PubNub server.

    The configuration defines for this demo are:    
        WLAN_SSID               -- AP to connect to
        WLAN_AUTH               -- Security for the AP
        WLAN_PSK                -- Passphrase for WPA security
        PUBNUB_PUBLISH_KEY      -- Publish key
        PUBNUB_SUBSCRIBE_KEY    -- Subscribe key
        PUBNUB_CHANNEL          -- PubNub channel name
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
#include "pubnub.h"

#if PUBNUB_CLOUD_EXAMPLE

/** Wi-Fi settings */
#define WLAN_SSID                                   "DEMO_AP"
#define WLAN_AUTH                                   M2M_WIFI_SEC_WPA_PSK /**< Security manner such as M2M_WIFI_SEC_WPA_PSK and M2M_WIFI_SEC_OPEN */
#define WLAN_PSK                                    "12345678"

/** PubNub settings */
#define PUBNUB_PUBLISH_KEY                          "demo"              /** Please replace this with your own publish key */
#define PUBNUB_SUBSCRIBE_KEY                        "demo"              /** Please replace this with your own subscribe key */
#define PUBNUB_CHANNEL                              "WINC1500_00:00"    /**< Do not change - last 4 digits will be updated with MAC address */
#define PUBNUB_PUBLISH_INTERVAL                     (3)
#define PUBNUB_SUBSCRIBE_INTERVAL                   (1)

/* BSP LED and Switch Re-directs */
/* This section is highly customizable based on application's specific needs. */
#if defined(BSP_SWITCH_4StateGet) // very roughly assume that pic32mx795_pim__e16 is used

#define APP_LED_1 BSP_LED_3
#define APP_LED_2 BSP_LED_4
#define APP_LED_3 BSP_LED_5

#define APP_SWITCH_1StateGet() BSP_SWITCH_4StateGet()
#define APP_SWITCH_2StateGet() BSP_SWITCH_5StateGet()
#define APP_SWITCH_3StateGet() BSP_SWITCH_6StateGet()

#else

#define APP_LED_1 BSP_LED_3
#define APP_LED_2 BSP_LED_2
#define APP_LED_3 BSP_LED_1

#define APP_SWITCH_1StateGet() true // UART Console occupies this pin on PIC32 MZ EC/EF SK
#define APP_SWITCH_2StateGet() BSP_SWITCH_2StateGet()
#define APP_SWITCH_3StateGet() BSP_SWITCH_1StateGet()

#endif

#define EXAMPLE_HEADER \
"\r\n=============================\r\n"\
    "WINC1500 Pubnub Cloud Example\r\n"\
    "=============================\r\n"

#define IPV4_BYTE(val, index) ((val >> (index * 8)) & 0xFF)
#define HEX2ASCII(x) (((x) >= 10) ? (((x) - 10) + 'A') : ((x) + '0'))

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

/** PubNub global variables */
static const char PubNubPublishKey[] = PUBNUB_PUBLISH_KEY;
static const char PubNubSubscribeKey[] = PUBNUB_SUBSCRIBE_KEY;
static char PubNubChannel[] = PUBNUB_CHANNEL;
static pubnub_t *pPubNubCfg;

/** Demo state */
static APP_STATE s_app_state;
static bool s_connected;

static void m2m_tcp_socket_handler(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
    handle_tcpip(sock, u8Msg, pvMsg);
}

static void socket_resolve_cb(uint8 *hostName, uint32 hostIp)
{
    SYS_CONSOLE_PRINT("socket_resolve_cb: %s resolved with IP %d.%d.%d.%d\r\n",
            hostName,
            (int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),
            (int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
    handle_dns_found((char *)hostName, hostIp);
}

static void m2m_wifi_state(uint8_t u8MsgType, void *pvMsg)
{
    switch (u8MsgType) {
    case M2M_WIFI_RESP_CON_STATE_CHANGED:
    {
        tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
        if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
            SYS_CONSOLE_PRINT("m2m_wifi_state: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
        } else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
            int8_t connect_ret;
            s_connected = false;
            SYS_CONSOLE_PRINT("m2m_wifi_state: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
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
        SYS_CONSOLE_PRINT("m2m_wifi_state: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n",
                pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
        s_connected = true;

        break;
    }

    default:
        break;
    }
}

static void set_dev_name_to_mac(uint8 *name, uint8 *mac_addr)
{
    /* Name must be in the format WINC1500_00:00 */
    uint16 len;

    len = m2m_strlen(name);
    if (len >= 5) {
        name[len - 1] = HEX2ASCII((mac_addr[5] >> 0) & 0x0f);
        name[len - 2] = HEX2ASCII((mac_addr[5] >> 4) & 0x0f);
        name[len - 4] = HEX2ASCII((mac_addr[4] >> 0) & 0x0f);
        name[len - 5] = HEX2ASCII((mac_addr[4] >> 4) & 0x0f);
    }
}

static void set_led_inactive(void)
{
    BSP_LEDStateSet(APP_LED_2, BSP_LED_STATE_OFF);
}

static void set_led_active(void)
{
    BSP_LEDStateSet(APP_LED_2, BSP_LED_STATE_ON);
}

static int get_button(void)
{
    return APP_SWITCH_3StateGet();
}

static int8_t wifi_open(void)
{
    tstrWifiInitParam wifiInitParam;
    int8_t s8InitStatus;
    uint8 mac_addr[6];
    uint8 u8IsMacAddrValid;

    /* Initialize Wi-Fi parameters structure. */
    memset((uint8_t *)&wifiInitParam, 0, sizeof(tstrWifiInitParam));
    wifiInitParam.pfAppWifiCb = m2m_wifi_state;

    /* Initialize WINC1500 Wi-Fi driver with data and status callbacks. */
    s8InitStatus = m2m_wifi_init(&wifiInitParam);
    if (M2M_SUCCESS != s8InitStatus) {
        SYS_CONSOLE_PRINT("m2m_wifi_init call error!\r\n");
        return -1;
    }

    /* Initialize Socket API. */
    socketInit();
    registerSocketCallback(m2m_tcp_socket_handler, socket_resolve_cb);

    /* Read MAC address to customize device name and AP name if enabled. */
    m2m_wifi_get_otp_mac_address(mac_addr, &u8IsMacAddrValid);
    if (!u8IsMacAddrValid) {
        SYS_CONSOLE_PRINT("MAC address fuse bit has not been configured!\r\n");
        SYS_CONSOLE_PRINT("Use m2m_wifi_set_mac_address() API to set MAC address via software.\r\n");
        return -1;
    }
    m2m_wifi_get_mac_address(mac_addr);
    set_dev_name_to_mac((uint8 *)PubNubChannel, mac_addr);

    /* Connect to AP using Wi-Fi settings from main.h. */
    SYS_CONSOLE_PRINT("Wi-Fi connecting to AP using hardcoded credentials...\r\n");
    m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID),
            WLAN_AUTH, (char *)WLAN_PSK, M2M_WIFI_CH_ALL);

        /* Initialize PubNub API. */
    SYS_CONSOLE_PRINT("PubNub configured with following settings:\r\n");
    SYS_CONSOLE_PRINT(" - Publish key: \"%s\", Subscribe key: \"%s\", Channel: \"%s\".\r\n\r\n",
            PubNubPublishKey, PubNubSubscribeKey, PubNubChannel);
    pPubNubCfg = pubnub_get_ctx(0);
    pubnub_init(pPubNubCfg, PubNubPublishKey, PubNubSubscribeKey);

    return 0;
}

static int pubnub_run(void)
{
    double temperature = 0;
    uint16_t light = 0;
    char buf[256] = {0};
    static uint32_t startTick = 0;

    /* PubNub: read event from the cloud. */
    if (pPubNubCfg->state == PS_IDLE) {
        /* Subscribe at the beginning and re-subscribe after every publish. */
        if ((pPubNubCfg->trans == PBTT_NONE) ||
            (pPubNubCfg->trans == PBTT_PUBLISH && pPubNubCfg->last_result == PNR_OK)) {
            SYS_CONSOLE_PRINT("subscribe event, PNR_OK\r\n");
            pubnub_subscribe(pPubNubCfg, PubNubChannel);
        }

        /* Process any received messages from the channel we subscribed. */
        while (1) {
            char const *msg = pubnub_get(pPubNubCfg);
            if (NULL == msg) {
                /* No more message to process. */
                break;
            }

            if (0 == (strncmp(&msg[2], "led", strlen("led")))) {
                /* LED control message. */
                SYS_CONSOLE_PRINT("received LED control message: %s\r\n", msg);
                if (0 == (strncmp(&msg[8], "on", strlen("on")))) {
                    set_led_active();
                } else if (0 == (strncmp(&msg[8], "off", strlen("off")))) {
                    set_led_inactive();
                }
            } else {
                /* Any other type of JSON message. */
                SYS_CONSOLE_PRINT("received message: %s\r\n", msg);
            }
        }

        /* Subscribe to receive pending messages. */
        if (SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet()* PUBNUB_SUBSCRIBE_INTERVAL)
        {
            startTick = SYS_TMR_TickCountGet();
            SYS_CONSOLE_PRINT("subscribe event, interval.\r\n");
            pubnub_subscribe(pPubNubCfg, PubNubChannel);
        }
    }

    /* Publish the temperature measurements periodically. */
    if (SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet() * PUBNUB_PUBLISH_INTERVAL)
    {
        startTick = SYS_TMR_TickCountGet();
        temperature = 0.0;
        light = 30;
        sprintf(buf, "{\"device\":\"%s\", \"temperature\":\"%d.%d\", \"light\":\"%d\", \"led\":\"%s\"}",
                PubNubChannel,
                (int)temperature, (int)((int)(temperature * 100) % 100),
                (((4096 - light) * 100) / 4096),
                get_button() ? "0" : "1");

        SYS_CONSOLE_PRINT("->");//SYS_CONSOLE_PRINT("publish event: {%s}\r\n", buf);
        close(pPubNubCfg->tcp_socket);
        pPubNubCfg->tcp_socket = -1;

        pPubNubCfg->state = PS_IDLE;
        pPubNubCfg->last_result = PNR_IO_ERROR;
        pubnub_publish(pPubNubCfg, PubNubChannel, buf);
    }

    return 0;
}

void app_init(void)
{
    SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

	app_state_set(APP_RADIO_INIT);
	s_connected = false;
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
		ret = pubnub_run();
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

#endif /* PUBNUB_CLOUD_EXAMPLE */

//DOM-IGNORE-END
