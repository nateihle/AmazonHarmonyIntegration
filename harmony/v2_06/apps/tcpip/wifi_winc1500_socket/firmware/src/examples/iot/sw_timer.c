/*******************************************************************************
  File Name:
    sw_timer.c

  Summary:
    SW Timer component for the IoT (Internet of Things) service
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

#if IOT_SUPPORT

#include "system_definitions.h"
#include "sw_timer.h"

/** Tick count of timer. */
static uint32_t sw_timer_tick = 0;

static SYS_TMR_HANDLE s_appTimerHandle;
static OSAL_MUTEX_HANDLE_TYPE *s_timerTaskLock;

/**
 * \brief TCC callback of SW timer.
 *
 * This function performs to the increasing the tick count.
 *
 * \param[in] module Instance of the TCC.
 */
static void sw_timer_tcc_callback(void)
{
	// every 0.1 seconds
	sw_timer_tick++;
}

static void app_timer_handler(uintptr_t context, uint32_t currTick)
{
	sw_timer_tcc_callback();
}

static void timer_callback_register(uint16_t interval_second, void (*cb)(uintptr_t context, uint32_t currTick))
{
	s_appTimerHandle = SYS_TMR_CallbackPeriodic(SYS_TMR_TickCounterFrequencyGet() * interval_second / 10, 0, cb);
}

static void timer_callback_deregister(void)
{
	SYS_TMR_CallbackStop(s_appTimerHandle);
}

void sw_timer_get_config_defaults(struct sw_timer_config *const config)
{
	WDRV_ASSERT((config != NULL), " ");

	config->accuracy = 100;
	config->tcc_dev = 0;
	config->tcc_callback_channel = 0;
}

void sw_timer_init(struct sw_timer_module *const module_inst, struct sw_timer_config *const config)
{
	module_inst->accuracy = config->accuracy;
	s_timerTaskLock = NULL;
	WDRV_MUTEX_CREATE(&s_timerTaskLock);
}

void sw_timer_enable(struct sw_timer_module *const module_inst)
{
	timer_callback_register(1, app_timer_handler);
}

void sw_timer_disable(struct sw_timer_module *const module_inst)
{
	timer_callback_deregister();
}

int sw_timer_register_callback(struct sw_timer_module *const module_inst,
	sw_timer_callback_t callback, void *context, uint32_t period)
{
	int index;
	struct sw_timer_handle *handler;

	WDRV_ASSERT((module_inst != NULL), " ");

	for (index = 0; index < CONF_SW_TIMER_COUNT; index++) {
		if (module_inst->handler[index].used == 0) {
			handler = &module_inst->handler[index];
			handler->callback = callback;
			handler->callback_enable = 0;
			handler->context = context;
			handler->period = period / module_inst->accuracy;
			handler->used = 1;
			return index;
		}
	}
	return -1;
}

void sw_timer_unregister_callback(struct sw_timer_module *const module_inst, int timer_id)
{
	struct sw_timer_handle *handler;

	WDRV_ASSERT((module_inst != NULL), " ");
	WDRV_ASSERT((timer_id >= 0 && timer_id < CONF_SW_TIMER_COUNT), " ");

	handler = &module_inst->handler[timer_id];

	handler->used = 0;
}

void sw_timer_enable_callback(struct sw_timer_module *const module_inst, int timer_id, uint32_t delay)
{
	struct sw_timer_handle *handler;

	WDRV_ASSERT((module_inst != NULL), "timer module cannot be NULL");
	WDRV_ASSERT((timer_id >= 0 && timer_id < CONF_SW_TIMER_COUNT), "timer_id is error");

	handler = &module_inst->handler[timer_id];

	handler->callback_enable = 1;
	handler->expire_time = sw_timer_tick + (delay / module_inst->accuracy);
}

void sw_timer_disable_callback(struct sw_timer_module *const module_inst, int timer_id)
{
	struct sw_timer_handle *handler;

	WDRV_ASSERT((module_inst != NULL), " ");
	WDRV_ASSERT((timer_id >= 0 && timer_id < CONF_SW_TIMER_COUNT), " ");

	handler = &module_inst->handler[timer_id];

	handler->callback_enable = 0;
}

void sw_timer_task(struct sw_timer_module *const module_inst)
{
	int index;
	struct sw_timer_handle *handler;
	WDRV_MUTEX_LOCK(s_timerTaskLock, OSAL_WAIT_FOREVER);
	WDRV_ASSERT((module_inst != NULL), " ");
	for (index = 0; index < CONF_SW_TIMER_COUNT; index++) {
		if (module_inst->handler[index].used && module_inst->handler[index].callback_enable) {
			handler = &module_inst->handler[index];
			if ((int)(handler->expire_time - sw_timer_tick) < 0 && handler->busy == 0) {
				/* Enter critical section. */
				handler->busy = 1;
				/* Timer was expired. */
				if (handler->period > 0) {
					handler->expire_time = sw_timer_tick + handler->period;
				} else {
					/* One shot. */
					handler->callback_enable = 0;
				}
				/* Call callback function. */
				handler->callback(module_inst, index, handler->context, handler->period);
				/* Leave critical section. */
				handler->busy = 0;
			}
		}
	}
	WDRV_MUTEX_UNLOCK(s_timerTaskLock);
}

#endif /* #if IOT_SUPPORT  */

// DOM-IGNORE-END
