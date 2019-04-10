/*******************************************************************************
  File Name:
    fw_update_ota.c

  Summary:
    WINC1500 FW Update Over the Air

  Description:
    OTA firmware update helper function. Before executing this example, place
    an OTA binary to your web server as demonstrated in OTA_TARGET_SERVER_URL
    below.

    The configuration defines for this demo are:
        WLAN_SSID                   -- AP to connect to
        WLAN_AUTH                   -- Security for the AP
        WLAN_PSK                    -- Passphrase for WPA security
       OTA_TARGET_SERVER_URL        -- Server URL where the OTA binary is stored
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
#include "m2m_wifi.h"

#if FW_UPDATE_OTA

#define OTA_TARGET_SERVER_URL       "http://192.168.1.107/m2m_ota_3a0.bin"

#define WLAN_SSID                   "DEMO_AP" /**< Destination SSID */
#define WLAN_AUTH                   M2M_WIFI_SEC_WPA_PSK /**< Security manner such as M2M_WIFI_SEC_WPA_PSK and M2M_WIFI_SEC_OPEN */
#define WLAN_PSK                    "12345678" /**< Password for destination SSID */

#define app_state_get() s_app_state
#define app_state_set(x) do { s_app_state = x; }  while (0)

typedef enum
{
	APP_RADIO_INIT,
	APP_WIFI_OPEN,
	APP_PARK
} APP_STATE;

static APP_STATE s_app_state;

void app_init(void)
{
	s_app_state = APP_RADIO_INIT;
}

static int wifi_open(void)
{
	int8_t ret;

	/* Connect to router. */
	ret = m2m_wifi_connect((char *)WLAN_SSID, sizeof(WLAN_SSID), WLAN_AUTH, (char *)WLAN_PSK, M2M_WIFI_CH_ALL);

	return ret;
}

void app_task(void)
{
	struct radio_init_param param;

	switch (app_state_get()) {
	case APP_RADIO_INIT:
		memset((void *)&param, 0, sizeof(param));
		param.fw_update_go = true;
		param.fw_update.over_the_air = true;
		param.fw_update.ota_server = OTA_TARGET_SERVER_URL;
		radio_init(&param);
		app_state_set(APP_WIFI_OPEN);
		break;
	case APP_WIFI_OPEN:
		wifi_open();
		app_state_set(APP_PARK);
		break;
	case APP_PARK:
		/* Waiting until ota update finishes... */
		break;
	default:
		break;
	}
}

#endif /* #if FW_UPDATE_OTA */

//DOM-IGNORE-END
