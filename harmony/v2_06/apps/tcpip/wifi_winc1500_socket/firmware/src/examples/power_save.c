/*******************************************************************************
  File Name:
    power_save.c

  Summary:
    WINC1500 power save example in a client mode.

  Description:
    WINC1500 supports two distinct power save modes, M2M_PS_DEEP_AUTOMATIC
    and M2M_PS_MANUAL. Deep Automatic mode follows the legacy 802.11 power save
    protocol in infrastructure mode. In Manual mode WINC1500 can wake up
    regardless of listen or dtim intervals, thus there is a risk not to sync
    with AP, but it allows longer sleep time which can translate to more
    power saving.

    The configuration defines for this demo are:
        WLAN_SSID               -- AP to connect to
        WLAN_AUTH               -- Security for the AP
        WLAN_PSK                -- Passphrase for WPA security
        PS_SLEEP_MODE           -- PowerSave mode
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

#if POWER_SAVE_EXAMPLE

/** Wi-Fi Settings */
#define WLAN_SSID        "DEMO_AP" /**< Destination SSID */
#define WLAN_AUTH        M2M_WIFI_SEC_WPA_PSK /**< Security manner such as M2M_WIFI_SEC_WPA_PSK and M2M_WIFI_SEC_OPEN */
#define WLAN_PSK         "12345678" /**< Password for destination SSID */

/** PowerSave Mode Settings */
#define PS_SLEEP_MODE   M2M_PS_DEEP_AUTOMATIC /* M2M_NO_PS / M2M_PS_DEEP_AUTOMATIC / M2M_PS_MANUAL */

/** PowerSave Status */
#define PS_WAKE         0
#define PS_SLEEP        1
#define PS_REQ_SLEEP    3

/** Request Sleep Time for PowerSave Manual Mode */
#define REQUEST_SLEEP_TIME  1000

#define EXAMPLE_HEADER \
"\r\n===========================\r\n"\
    "WINC1500 Power Save Example\r\n"\
    "===========================\r\n"

#define app_state_get() s_app_state
#define app_state_set(x) do {s_app_state = x;} while (0)
#define is_link_up() s_connected

typedef enum {
    APP_RADIO_INIT,
    APP_WIFI_OPEN,
    APP_NET_UP,
    APP_WIFI_CONNECT_WAIT,
    APP_END,
    APP_PARK
} APP_STATE;

/** Demo State */
static APP_STATE s_app_state;
static bool s_connected;
/** Wi-Fi Sleep Status */
static uint8 s_sleepStatus;
static SYS_TMR_HANDLE s_appTimerHandle;
static bool s_hasAppTimerEvent;

static void app_timer_handler(uintptr_t context, uint32_t currTick)
{
    s_hasAppTimerEvent = true;
}

static void timer_callback_register(uint16_t interval_second, void (*cb)(uintptr_t context, uint32_t currTick))
{
    s_appTimerHandle = SYS_TMR_CallbackPeriodic(SYS_TMR_TickCounterFrequencyGet() * interval_second, 0, cb);
}

static void timer_callback_deregister(SYS_TMR_HANDLE timer_handle)
{
    SYS_TMR_CallbackStop(timer_handle);
}

static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
    switch (u8MsgType) {
    case M2M_WIFI_RESP_CON_STATE_CHANGED:
    {
        s_sleepStatus = PS_WAKE;
        tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
        if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
        } else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
            int8_t connect_ret;
            s_connected = false;
            SYS_CONSOLE_PRINT("Wi-Fi disconnected\r\n");
            connect_ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID), WLAN_AUTH, (void *)WLAN_PSK, M2M_WIFI_CH_ALL);
            if (connect_ret != M2M_SUCCESS)
                SYS_CONSOLE_PRINT("m2m_wifi_connect call error!(%d)\r\n", connect_ret);
        }
        break;
    }
    case M2M_WIFI_REQ_DHCP_CONF:
    {
        uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
        SYS_CONSOLE_PRINT("Wi-Fi connected\r\n");
        SYS_CONSOLE_PRINT("Wi-Fi IP is %u.%u.%u.%u\r\n",
                pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
        s_sleepStatus = PS_REQ_SLEEP;
        s_connected = true;
        break;
    }
    case M2M_WIFI_RESP_CURRENT_RSSI:
    {
        /* This message type is triggered by "m2m_wifi_req_curr_rssi()" function. */
        int8_t *rssi = (int8_t *)pvMsg;
        SYS_CONSOLE_PRINT("RSSI for the current connected AP (%d)\r\n", (int8_t)(*rssi));
        s_sleepStatus = PS_REQ_SLEEP;
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

    /* Initialize Wi-Fi parameters structure. */
    memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

    /* Initialize Wi-Fi driver with data and status callbacks. */
    param.pfAppWifiCb = wifi_cb;
    ret = m2m_wifi_init(&param);
    if (M2M_SUCCESS != ret) {
        SYS_CONSOLE_PRINT("m2m_wifi_init call error!(%d)\r\n", ret);
        return ret;
    }

    /* Set defined sleep mode. */
    if (PS_SLEEP_MODE == M2M_PS_MANUAL) {
        SYS_CONSOLE_PRINT("M2M_PS_MANUAL\r\n");
        m2m_wifi_set_sleep_mode(PS_SLEEP_MODE, 1);
    } else if (PS_SLEEP_MODE == M2M_PS_DEEP_AUTOMATIC) {
        SYS_CONSOLE_PRINT("M2M_PS_DEEP_AUTOMATIC\r\n");
        tstrM2mLsnInt strM2mLsnInt;
        m2m_wifi_set_sleep_mode(M2M_PS_DEEP_AUTOMATIC, 1);
        strM2mLsnInt.u16LsnInt = M2M_LISTEN_INTERVAL;
        m2m_wifi_set_lsn_int(&strM2mLsnInt);
    } else {
        SYS_CONSOLE_PRINT("M2M_PS_NO\r\n");
    }

    /* Connect to defined AP. */
    ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID), WLAN_AUTH, (void *)WLAN_PSK, M2M_WIFI_CH_ALL);

    return ret;
}

static int power_mode_manage(void)
{
    int ret = 0;

    if (PS_SLEEP_MODE == M2M_PS_MANUAL) {
        /* Put it back to sleep. */
        if (s_sleepStatus == PS_REQ_SLEEP) {
            ret = m2m_wifi_request_sleep(REQUEST_SLEEP_TIME);
            s_sleepStatus = PS_SLEEP;
        }
    }

    return ret;
}

static int custom_app(void)
{
	int ret = 0;

	/* Connected in power save mode. You may run your application here. */

	ret = power_mode_manage();

	if (ret == 0 && s_hasAppTimerEvent) {
		/* Request RSSI of a received packet from AP for demo purpose. */
		if (is_link_up())
			ret = m2m_wifi_req_curr_rssi();
		s_hasAppTimerEvent = false;
	}

	return ret;
}

void app_init(void)
{
    SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

    app_state_set(APP_RADIO_INIT);
    s_connected = false;
    s_sleepStatus = PS_WAKE;
    s_appTimerHandle = 0;
    s_hasAppTimerEvent = false;
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
        if (is_link_up()) {
            timer_callback_register(1, app_timer_handler);
            app_state_set(APP_NET_UP);
        }
        break;
    case APP_NET_UP:
        ret = custom_app();
        if (ret)
            app_state_set(APP_END);
        break;
    case APP_END:
        timer_callback_deregister(s_appTimerHandle);
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

#endif /* POWER_SAVE_EXAMPLE */

//DOM-IGNORE-END
