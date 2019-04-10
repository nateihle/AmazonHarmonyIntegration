/*******************************************************************************
  File Name:
    app.c

  Summary:
    Main entry point of demo examples.

  Description:
    All demo examples start from here.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

/* BSP LED and Switch Re-directs */
/* This section is highly customizable based on application's specific needs. */
#if !defined(BSP_SWITCH_3StateGet) // very roughly assume that chipkit_wf32 is used
#   define APP_LED_1 BSP_LED_1
#   define APP_LED_2 BSP_LED_2
#   define APP_LED_3 BSP_LED_3

#   define APP_SWITCH_1StateGet() BSP_SWITCH_1StateGet()
#   define APP_SWITCH_2StateGet() BSP_SWITCH_2StateGet()
#   define APP_SWITCH_3StateGet() false
#elif defined(BSP_SWITCH_4StateGet) // very roughly assume that pic32mx795_pim__e16 is used
#   define APP_LED_1 BSP_LED_3
#   define APP_LED_2 BSP_LED_4
#   define APP_LED_3 BSP_LED_5

#   define APP_SWITCH_1StateGet() BSP_SWITCH_4StateGet()
#   define APP_SWITCH_2StateGet() BSP_SWITCH_5StateGet()
#   define APP_SWITCH_3StateGet() BSP_SWITCH_6StateGet()
#else
#   define APP_LED_1 BSP_LED_3
#   define APP_LED_2 BSP_LED_2
#   define APP_LED_3 BSP_LED_1

#   define APP_SWITCH_1StateGet() BSP_SWITCH_3StateGet()
#   define APP_SWITCH_2StateGet() BSP_SWITCH_2StateGet()
#   define APP_SWITCH_3StateGet() BSP_SWITCH_1StateGet()
#endif

void app_task(void);
void app_init(void);

void APP_Initialize ( void )
{
	app_init();
}

void radio_init(void *arg)
{
	WDRV_HOOKS hooks;
	struct radio_init_param *param;

	memset((void *)&hooks, 0, sizeof(hooks));
	param = (struct radio_init_param *)arg;

	if (param && param->fw_update_go) {
		struct fw_update_param *fw_update;
		fw_update = (struct fw_update_param *)&param->fw_update;

		if (fw_update->over_serial) {
			hooks.isSerialFwUpdateRequested = true;
		} else if (fw_update->over_the_air) {
			hooks.isOtaFwUpdateRequested = true;
			hooks.fwOtaServerUrl = (uint8_t *)fw_update->ota_server;
		}
	}
	WDRV_EXT_Initialize(&hooks, true);
}

void radio_deinit(void)
{
	WDRV_EXT_Deinitialize();
}

static void led_toggle(void)
{
	static uint32_t startTick = 0;
	static BSP_LED_STATE LEDstate = BSP_LED_STATE_OFF;

	if (SYS_TMR_TickCountGet() - startTick >= SYS_TMR_TickCounterFrequencyGet()/2ul) {
		startTick = SYS_TMR_TickCountGet();
		LEDstate ^= BSP_LED_STATE_ON;
		BSP_LEDStateSet(APP_LED_1, LEDstate);
	}
}

void APP_Tasks(void)
{
	app_task();
	led_toggle();
	SYS_CMD_READY_TO_READ();
}

// DOM-IGNORE-END
