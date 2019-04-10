/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*
* SEARAN LLC is the exclusive licensee and developer of dotstack with
* all its modifications and enhancements.
*
* Contains proprietary and confidential information of CandleDragon and
* may not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2009, 2010, 2011 CandleDragon. All Rights Reserved.
*******************************************************************************/

#ifndef __BTTIMER_H
#define __BTTIMER_H


/**
 * \defgroup timer OEM - Timer Interface
 * 	
 * \details DotStack requires a facility to measure various time intervals.
 * This module declares an interface that the application has to implement in
 * order to provide DotStack with such functionality.
 *
 * The minimum required timer resolution is 100 milliseconds.
 *
 * The maximum number of timers is defined by the \c BT_TIMER_MAX constant.
 *
 * Timer ID values used in the interface are from \c 0 to <tt>BT_TIMER_MAX-1</tt>.
 *
 * The interface consists of the following function:
 *     \li bt_oem_timer_set()
 *     \li bt_oem_timer_clear()
 */
/*@{*/ /* begin doxygen group */


#ifdef __cplusplus
extern "C" {
#endif

typedef enum _bt_timer_id_enum
{
	BT_TIMER_L2CAP,
	BT_TIMER_RFCOMM,
	BT_TIMER_WAKEUP_ACK,
	BT_TIMER_TEST,
	BT_TIMER_HCI,
	BT_TIMER_IAP,
	BT_TIMER_HSP_AG,
	BT_TIMER_HFP_AG,
	BT_TIMER_ATT,
	BT_TIMER_ATT_CLIENT,
	BT_TIMER_AVRCP,
	BT_TIMER_SMP,
	BT_TIMER_IAP2,
	BT_TIMER_HCRP,

	BT_TIMER_3WIRE_T0,
	BT_TIMER_3WIRE_T1,

	BT_TIMER_MAX

} bt_timer_id;


/**
 * \brief Timer callback.
 *
 * \details This callback is called when a timer expires.
 */
typedef void (*bt_timer_callback_fp)(void);


/**
 * \brief Set timer.
 *
 * \details This function must be implemented by the application.
 * When it is called, the application must set the specified timer. When the timer
 * expires, the application must call the passed callback function. The function
 * must not wait until the timer expires. It must set the timer and exit immediately.
 *
 * \param timerId ID of the timer to set.
 * \param milliseconds Timer interval in milliseconds
 * \param callback Timer expiration callback function.
 */
void bt_oem_timer_set(bt_uint timerId, bt_ulong milliseconds, bt_timer_callback_fp callback);


/**
 * \brief Clear timer.
 *
 * \details This function must be implemented by the application.
 * When this function is called the application must clear the specified timer.
 * If it is already expired and a callback is currently pending, the application
 * should also take measures to cancel the callback.
 *
 * \param timerId ID of the timer to clear.
 */
void bt_oem_timer_clear(bt_uint timerId);


#ifdef __cplusplus
}
#endif

/*@}*/ /* end doxygen group */

#endif // __BTTIMER_H
