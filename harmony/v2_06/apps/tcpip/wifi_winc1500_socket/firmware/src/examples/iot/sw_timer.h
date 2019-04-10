/*******************************************************************************
  File Name:
    sw_timer.h

  Summary:
    SW Timer component for the IoT(Internet of things) service
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
// DOM-IGNORE-END

#ifndef _SW_TIMER_H
#define _SW_TIMER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum timer count. */
#define CONF_SW_TIMER_COUNT                1

/* Maximum timer count. */
#define CONF_SW_TIMER_CALLBACK_CHANNEL     0

struct sw_timer_module;

/**
 * Callback Function type of time out event in the timer.
 *
 * \param[in]  timer_id        ID of timer.
 * \param[in]  context         Private data of timer.
 * \param[in]  context         Period of timer handle.
 */
typedef void (*sw_timer_callback_t)(struct sw_timer_module *const module, int timer_id, void *context, int period);

/**
 * \brief SW timer configuration structure
 *
 * Configuration struct for a SW timer instance. This structure should be
 * initialized by the \ref sw_timer_get_config_defaults function before being
 * modified by the user application.
 */
struct sw_timer_config {
	/** HW interface of TCC. */
	uint8_t tcc_dev;
	/** Callback channel of TCC. */
	uint8_t tcc_callback_channel;
	/** Accuracy of timer. If this value is increased, Timer can checks a long time. Unit is milliseconds*/
	uint16_t accuracy;
};

/**
 * \brief SW timer handler structure
 */
struct sw_timer_handle {
	/** A flag that timer is used. */
	uint8_t used                       : 1;
	/** A flag that timer callback is enabled. */
	uint8_t callback_enable            : 1;
	/** A flag that timer is calling the handler function or not. */
	uint8_t busy                       : 1;
	/** Callback of timer. */
	sw_timer_callback_t callback;
	/** Private data of timer. */
	void *context;
	/** Period of timer. If this value is set to zero, it means this timer operated once. */
	uint32_t period;
	/** Expired time of timer. */
	uint32_t expire_time;
};

/**
 * \brief SW timer module structure
 */
struct sw_timer_module {
	/** Timer handler instances. */
	struct sw_timer_handle handler[CONF_SW_TIMER_COUNT];

	/** Accuracy of timer. */
	uint32_t accuracy;
};

/**
 * \brief Get default configuration of SW timer.
 *
 * \param[in]  config          Pointer of configuration structure which will be used in the timer.
 */
void sw_timer_get_config_defaults(struct sw_timer_config *const config);

/**
 * \brief Initialize the SW timer module.
 *
 * \param[in]  module_inst     Pointer of timer.
 * \param[in]  config          Pointer of configuration structure which will be used in the timer.
 */
void sw_timer_init(struct sw_timer_module *const module_inst, struct sw_timer_config *const config);

/**
 * \brief Start the timer.
 *
 * \param[in]  module_inst     Pointer of timer.
 */
void sw_timer_enable(struct sw_timer_module *const module_inst);

/**
 * \brief Stop the timer.
 *
 * \param[in]  module_inst     Pointer of timer.
 */
void sw_timer_disable(struct sw_timer_module *const module_inst);

/**
 * \brief Register callback.
 *
 * \param[in]  module_inst     Pointer of timer.
 * \param[in]  callback        Callback entry of time out.
 * \param[in]  context         Private data of timer.
 * \param[in]  period          Period of timer.
 *
 * \return If negative value, registering timer was failed. Otherwise this value is ID of timer.
 */
int sw_timer_register_callback(struct sw_timer_module *const module_inst,
	sw_timer_callback_t callback, void *context, uint32_t period);

/**
 * \brief Unregister callback.
 *
 * \param[in]  module_inst     Pointer of timer.
 * \param[in]  timer_id        Timer ID .
 */
void sw_timer_unregister_callback(struct sw_timer_module *const module_inst, int timer_id);

/**
 * \brief Enables callback
 *
 * Enables the callback function registered by the \ref sw_timer_register_callback.
 *
 * \param[in]  module_inst     Pointer to USART software instance struct
 * \param[in]  timer_id        Timer ID .
 * \param[in]  delay           Initialization value of timer.
 */
void sw_timer_enable_callback(struct sw_timer_module *const module_inst, int timer_id, uint32_t delay);

/**
 * \brief Disable callback
 *
 * Disable the callback function registered by the \ref sw_timer_register_callback.
 *
 * \param[in]  module_inst     Pointer to USART software instance struct
 * \param[in]  timer_id        Timer ID .
 */
void sw_timer_disable_callback(struct sw_timer_module *const module_inst, int timer_id);

/**
 * \brief Checks the time out of each timer handlers.
 *
 * This function must be called continuously for the checking the expiration of the timer handle.
 *
 * \param[in]  module_inst     Pointer to timer instance struct
 */
void sw_timer_task(struct sw_timer_module *const module_inst);

#ifdef __cplusplus
}
#endif

#endif /* _SW_TIMER_H */