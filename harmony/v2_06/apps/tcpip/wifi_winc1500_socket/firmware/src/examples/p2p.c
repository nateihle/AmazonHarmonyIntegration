/*******************************************************************************
  File Name:
    p2p.c

  Summary:
    WINC1500 P2P Example

  Description:
    This example demonstrates the use of the WINC1500 to enter P2P mode.

    The configuration defines for this demo are:
        WLAN_DEVICE_NAME    -- WINC1500's device name in P2P mode
        WLAN_CHANNEL        -- WINC1500's listen channel in P2P mode
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

#if P2P_EXAMPLE

#define WLAN_DEVICE_NAME    "WINC1500_P2P" /* WINC1500's Device Name in P2P Mode */
#define WLAN_CHANNEL        M2M_WIFI_CH_6 /* WINC1500's Listen Channel in P2P Mode */

#define EXAMPLE_HEADER \
"\r\n====================\r\n"\
    "WINC1500 P2P Example\r\n"\
    "====================\r\n"

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
static bool s_connected;

/*
 * Callback to get the Wi-Fi status update.
 *
 * u8MsgType: type of Wi-Fi notification. Possible types are:
 *   M2M_WIFI_RESP_CON_STATE_CHANGED,
 *   M2M_WIFI_REQ_DHCP_CONF.
 * pvMsg: pointer to a buffer containing the notification parameters
 *   (if any). It should be casted to the correct data type corresponding to the
 *   notification type.
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			s_connected = false;
			SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
		}
	}
		break;
	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		s_connected = true;
		SYS_CONSOLE_PRINT("wifi_cb: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n",
			pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
	}
		break;
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
	if (ret != M2M_SUCCESS) {
		SYS_CONSOLE_PRINT("m2m_wifi_init call error!(%d)\r\n", ret);
		return ret;
	}

	/* Set device name to be shown in peer device. */
	ret = m2m_wifi_set_device_name((uint8_t *)WLAN_DEVICE_NAME, strlen(WLAN_DEVICE_NAME));
	if (ret != M2M_SUCCESS) {
		SYS_CONSOLE_PRINT("m2m_wifi_set_device_name call error!(%d)\r\n", ret);
		return ret;
	}

	/* Bring up P2P mode with channel number. */
	ret = m2m_wifi_p2p(WLAN_CHANNEL);
	if (ret != M2M_SUCCESS) {
		SYS_CONSOLE_PRINT("m2m_wifi_p2p call error!(%d)\r\n", ret);
		return ret;
	}

	SYS_CONSOLE_PRINT("Device name: %s\r\nChannel: %d\r\nWaiting for P2P connection\r\n",
		WLAN_DEVICE_NAME, WLAN_CHANNEL);

	return ret;
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
		/* Demo just stays connected for 30 seconds and then ends. */
		WDRV_TIME_DELAY(30000);
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

#endif /* P2P_EXAMPLE */

//DOM-IGNORE-END
