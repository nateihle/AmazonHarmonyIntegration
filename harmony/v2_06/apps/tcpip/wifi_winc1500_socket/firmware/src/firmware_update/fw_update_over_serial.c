/*******************************************************************************
  File Name:
    fw_update_over_serial.c

  Summary:
    WINC1500 FW Update Over Serial

  Description:
    This helper function allows firmware update over UART or USB serial port
    in conjunction with firmware update PC tool. Execute this function first
    before starting the PC tool.
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

#if FW_UPDATE_OVER_SERIAL

#define app_state_get() s_app_state
#define app_state_set(x) do { s_app_state = x; }  while (0)

typedef enum
{
	APP_RADIO_INIT,
	APP_PARK
} APP_STATE;

static APP_STATE s_app_state;

void app_init(void)
{
	s_app_state = APP_RADIO_INIT;
}

void app_task(void)
{
	struct radio_init_param param;

	switch (app_state_get()) {
	case APP_RADIO_INIT:
		memset((void *)&param, 0, sizeof(param));
		param.fw_update_go = true;
		param.fw_update.over_serial = true;
		radio_init(&param);
		app_state_set(APP_PARK);
		break;
	case APP_PARK:
		/* Doing nothing until serial update ends... */
		break;
	default:
		break;
	}
}

#endif /* #if FW_UPDATE_OVER_SERIAL */

//DOM-IGNORE-END
