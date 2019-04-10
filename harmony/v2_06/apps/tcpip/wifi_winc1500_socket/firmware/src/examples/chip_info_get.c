/*******************************************************************************
  File Name:
    chip_info_get.c

  Summary:
    Example reading the chip information of WINC1500.

  Description:
    Information involves chip and RF revision IDs.
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

#if CHIP_INFO_GET_EXAMPLE

#define EXAMPLE_HEADER \
"\r\n==============================\r\n"\
    "WINC1500 Chip Info Get Example\r\n"\
    "==============================\r\n"

#define app_state_get() s_app_state
#define app_state_set(x) do {s_app_state = x;} while (0)

typedef enum {
    APP_RADIO_INIT,
    APP_WIFI_OPEN,
    APP_MISC_DEMO,
    APP_END,
    APP_PARK
} APP_STATE;

/** Demo state */
static APP_STATE s_app_state;

static int8_t wifi_open(void)
{
    tstrWifiInitParam param;
    int ret = 0;

    /* Initialize Wi-Fi parameters structure. */
    memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

    /* Initialize Wi-Fi driver with data and status callbacks. */
    ret = m2m_wifi_init(&param);
    if (M2M_SUCCESS != ret) {
        SYS_CONSOLE_PRINT("m2m_wifi_init call error!(%d)\r\n", ret);
        return ret;
    }

    return 0;
}

static int get_chipInfo(void)
{
    /* Display WINC1500 chip information. */
    SYS_CONSOLE_PRINT("Chip ID : \r\t\t\t%x\r\n", (unsigned int)nmi_get_chipid());
    SYS_CONSOLE_PRINT("RF Revision ID : \r\t\t\t%x\r\n", (unsigned int)nmi_get_rfrevid());
    SYS_CONSOLE_PRINT("Done.\r\n\r\n");

    return 0;
}

void app_init(void)
{
    SYS_CONSOLE_MESSAGE(EXAMPLE_HEADER);

    app_state_set(APP_RADIO_INIT);
}

void app_task(void)
{
    int ret;

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
            app_state_set(APP_MISC_DEMO);
        break;
    case APP_MISC_DEMO:
        get_chipInfo();
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

#endif /* CHIP_INFO_GET_EXAMPLE */

//DOM-IGNORE-END
