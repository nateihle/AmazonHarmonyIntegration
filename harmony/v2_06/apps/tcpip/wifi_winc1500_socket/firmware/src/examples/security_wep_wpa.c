/*******************************************************************************
  File Name:
    security_wep_wpa.c

  Summary:
    WINC1500 Security Mode (WEP & WPA) Example

  Description:
    This example demonstrates WINC1500 WAP/WEP security connection in a client
    station mode. Users can choose the security mode with WLAN_AUTH macro.

    The configuration defines for this demo are:
        WLAN_SSID           -- AP to connect to
        WLAN_AUTH           -- Security for the AP
        WLAN_PSK            -- Passphrase for WPA security
        WLAN_WEP_KEY_40     -- 64-bit key for WEP security
        WLAN_WEP_KEY_104    -- 128-bit key for WEP security
        WLAN_WEP_KEY_INDEX  -- Key index for WEP security
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
#include "nmasic.h"

#if SECURITY_WEP_WPA_EXAMPLE

/** Security information for Wi-Fi connection */
#define WLAN_SSID           	   "DEMO_AP" /**< Destination SSID */
#define WLAN_AUTH                  M2M_WIFI_SEC_WPA_PSK /**< Security manner, M2M_WIFI_SEC_WPA_PSK, M2M_WIFI_SEC_WEP or M2M_WIFI_SEC_OPEN */
#define WLAN_PSK                   "12345678" /**< Password for destination SSID */
#define WLAN_WEP_KEY_INDEX         1 /**< WEP key index */
#define WLAN_WEP_KEY_40            "1234567890" /**< 64-bit WEP key. In case of WEP64, 10 hexadecimal (base 16) characters (0-9 and A-F) */
#define WLAN_WEP_KEY_104           "1234567890abcdef1234567890" /**< 128-bit WEP key. In case of WEP128, 26 hexadecimal (base 16) characters (0-9 and A-F) */

#define EXAMPLE_HEADER \
"\r\n==========================================\r\n"\
    "WINC1500 Security Mode (WEP & WPA) Example\r\n"\
    "==========================================\r\n"

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

/** Demo state */
static APP_STATE s_app_state;
static bool s_connected;

/** Security parameters for 64-bit WEP Encryption @ref m2m_wifi_connect */
tstrM2mWifiWepParams wep64_parameters = {WLAN_WEP_KEY_INDEX, sizeof(WLAN_WEP_KEY_40), WLAN_WEP_KEY_40};
/** Security parameters for 128-bit WEP Encryption @ref m2m_wifi_connect */
tstrM2mWifiWepParams wep128_parameters = {WLAN_WEP_KEY_INDEX, sizeof(WLAN_WEP_KEY_104), WLAN_WEP_KEY_104};

static uint8_t s_sec_type;

static int8_t secure_connect(uint8_t sec_type)
{
    int8_t ret;

    if (sec_type == M2M_WIFI_SEC_WEP) {
        /* Per default for WLAN_WEP_KEY_40. For WLAN_WEP_KEY_104, replace wep64_parameters with wep128_parameters. */
        ret = m2m_wifi_connect((char *)WLAN_SSID, strlen((char *)WLAN_SSID), sec_type, (void *)&wep64_parameters, M2M_WIFI_CH_ALL);
    } else {
        ret = m2m_wifi_connect((char *)WLAN_SSID, strlen((char *)WLAN_SSID), sec_type, (void *)WLAN_PSK, M2M_WIFI_CH_ALL);
    }
    return ret;
}

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
    switch (u8MsgType) {
    case M2M_WIFI_RESP_CON_STATE_CHANGED:
    {
        tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
        if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
        } else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
            s_connected = false;
            SYS_CONSOLE_PRINT("Wi-Fi disconnected\r\n");
            secure_connect(s_sec_type);
        }
        break;
    }
    case M2M_WIFI_REQ_DHCP_CONF:
    {
        uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
        SYS_CONSOLE_PRINT("Wi-Fi connected\r\n");
        SYS_CONSOLE_PRINT("Wi-Fi IP is %u.%u.%u.%u\r\n",
            pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
        s_connected = true;
        break;
    }
    default:
        break;
    }
}

static int8_t wifi_open(void)
{
    tstrWifiInitParam param;
    int8_t ret = 0;

    /* Initialize Wi-Fi parameters structure. */
    memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

    /* Initialize Wi-Fi driver with data and status callbacks. */
    param.pfAppWifiCb = wifi_cb;
    ret = m2m_wifi_init(&param);
    if (M2M_SUCCESS != ret) {
        SYS_CONSOLE_PRINT("m2m_wifi_init call error!(%d)\r\n", ret);
        return ret;
    }

    secure_connect(s_sec_type);
    SYS_CONSOLE_PRINT("Connecting to %s\r\n", (char *)WLAN_SSID);

    return ret;
}

static int custom_app(void)
{
	/* Connected now. You may run your application here. */
    return 0;
}

void app_init(void)
{
    SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

    app_state_set(APP_RADIO_INIT);
    s_connected = false;
	s_sec_type = WLAN_AUTH;
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
        ret = custom_app();
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

#endif /* SECURITY_WEP_WPA_EXAMPLE */

//DOM-IGNORE-END
