/*******************************************************************************
  File Name:
    wps_connect.c

  Summary:
    This example demonstrates the use of WPS (Wi-Fi Protected Setup) security
    with the WINC1500.

  Description:
    Users can choose either push button or pin method by toggling
    WPS_PUSH_BUTTON_FEATURE macro. In pin method, 8-byte length of pin
    should be supplied with WPS_PIN_NUMBER macro. The last byte of pin is
    the checksum of previous 7 bytes.

    The configuration defines for this demo are:
        WPS_PIN_NUMBER          -- 8-byte pin
        WPS_PUSH_BUTTON_FEATURE -- true sets push button method
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

#if WPS_CONNECT_EXAMPLE

/** WPS PIN */
#define WPS_PIN_NUMBER              "12345670"

/** WPS Push Button Feature */
#define WPS_PUSH_BUTTON_FEATURE     true

/* BSP Switch Re-directs */
/* This section is highly customizable based on application's specific needs. */
#if defined(BSP_SWITCH_4StateGet) // very roughly assume that pic32mx795_pim__e16 is used

#define APP_PBC_MESSAGE "Push S3 button to start WPS!\r\n"
#define APP_SWITCH_StateGet() BSP_SWITCH_3StateGet()

#else

#define APP_PBC_MESSAGE "Push S1 button to start WPS!\r\n"
#define APP_SWITCH_StateGet() BSP_SWITCH_1StateGet()

#endif

#define APP_WLAN_AUTH_MESSAGE_INVALID   "Invalid"
#define APP_WLAN_AUTH_MESSAGE_OPEN      "Open"
#define APP_WLAN_AUTH_MESSAGE_WPA_PSK   "WPA-PSK/WPA2-PSK"
#define APP_WLAN_AUTH_MESSAGE_WEP       "WEP-40/WEP-104"
#define APP_WLAN_AUTH_MESSAGE_802_1X    "WPA-Enterprise/WPA2-Enterprise"

#define EXAMPLE_HEADER \
"\r\n============================\r\n"\
    "WINC1500 WPS Connect Example\r\n"\
    "============================\r\n"

#define app_state_get() s_app_state
#define app_state_set(x) do {s_app_state = x;} while (0)
#define is_link_up() s_connected

typedef enum {
    APP_RADIO_INIT,
    APP_WIFI_OPEN,
    APP_WIFI_WPS_START,
    APP_WIFI_WPS_WAIT,
    APP_NET_UP,
    APP_END,
    APP_PARK
} APP_STATE;

static APP_STATE s_app_state;
static bool s_connected;

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
    switch (u8MsgType) {
    case M2M_WIFI_RESP_CON_STATE_CHANGED:
    {
        tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
        if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
            SYS_CONSOLE_PRINT("Wi-Fi connected\r\n");
        } else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
            s_connected = false;
            SYS_CONSOLE_PRINT("Wi-Fi disconnected\r\n");
        }
        break;
    }

    case M2M_WIFI_REQ_DHCP_CONF:
    {
        uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
        SYS_CONSOLE_PRINT("Wi-Fi IP is %u.%u.%u.%u\r\n",
            pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
        s_connected = true;
        break;
    }

    case M2M_WIFI_REQ_WPS:
    {
        tstrM2MWPSInfo *pstrWPS = (tstrM2MWPSInfo *)pvMsg;
        if (pstrWPS->u8AuthType == 0) {
            /* WPS is not enabled by firmware OR WPS monitor timeout. */
            SYS_CONSOLE_PRINT("WPS is disabled OR timeout\r\n");
            m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
        } else {
            const char *pstrAuthType;
            int8_t connect_ret;
            switch (pstrWPS->u8AuthType) {
            case M2M_WIFI_SEC_OPEN:
                pstrAuthType = APP_WLAN_AUTH_MESSAGE_OPEN;
                break;
            case M2M_WIFI_SEC_WPA_PSK:
                pstrAuthType = APP_WLAN_AUTH_MESSAGE_WPA_PSK;
                break;
            case M2M_WIFI_SEC_WEP:
                pstrAuthType = APP_WLAN_AUTH_MESSAGE_WEP;
                break;
            case M2M_WIFI_SEC_802_1X:
                pstrAuthType = APP_WLAN_AUTH_MESSAGE_802_1X;
                break;
            default:
                pstrAuthType = APP_WLAN_AUTH_MESSAGE_INVALID;
                break;
            }
            SYS_CONSOLE_PRINT("Requesting WPS\r\nSSID: %s, Security: %s, Key/Passphrase: %s\r\n",
                pstrWPS->au8SSID, pstrAuthType, pstrWPS->au8PSK);
            connect_ret = m2m_wifi_connect((char *)pstrWPS->au8SSID, (uint8)m2m_strlen(pstrWPS->au8SSID),
                pstrWPS->u8AuthType, pstrWPS->au8PSK, pstrWPS->u8Ch);
            if (connect_ret != M2M_SUCCESS) {
                SYS_CONSOLE_PRINT("m2m_wifi_connect call error!(%d)\r\n", connect_ret);
            }
        }
        break;
    }

    default:
        break;
    }
}

static int8_t wifi_open(void)
{
    tstrWifiInitParam param;
    int8_t ret;
    char devName[] = "WINC1500_WPS";

    /* Initialize Wi-Fi parameters structure. */
    memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

    /* Initialize Wi-Fi driver with data and status callbacks. */
    param.pfAppWifiCb = wifi_cb;
    ret = m2m_wifi_init(&param);
    if (M2M_SUCCESS != ret) {
        SYS_CONSOLE_PRINT("m2m_wifi_init call error!(%d)\r\n", ret);
        return ret;
    }

    /* Device name must be set before enabling WPS mode. */
    m2m_wifi_set_device_name((uint8 *)devName, strlen(devName));

    if (WPS_PUSH_BUTTON_FEATURE) {
        SYS_CONSOLE_PRINT(APP_PBC_MESSAGE);
    }

    return 0;
}

static int wps_start(void)
{
    int ret = -1;

    if (WPS_PUSH_BUTTON_FEATURE) {
        /* case 1 WPS Push Button method */
        if (APP_SWITCH_StateGet() == 0) {
            SYS_CONSOLE_PRINT("WPS starts...\r\n");
            ret = m2m_wifi_wps(WPS_PBC_TRIGGER, NULL);
        }
    } else {
        /* case 2 WPS PIN method */
        ret = m2m_wifi_wps(WPS_PIN_TRIGGER, (const char *)WPS_PIN_NUMBER);
    }
    return ret;
}

void app_init(void)
{
    SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

    app_state_set(APP_RADIO_INIT);
    s_connected = false;
}

static int custom_app(void)
{
    /* Connected now. You may run your application here. */
    return 0;
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
            app_state_set(APP_WIFI_WPS_START);
        break;
    case APP_WIFI_WPS_START:
        ret = wps_start();
        if (ret == 0)
            app_state_set(APP_WIFI_WPS_WAIT);
        break;
    case APP_WIFI_WPS_WAIT:
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

#endif /* WPS_CONNECT_EXAMPLE */

//DOM-IGNORE-END
