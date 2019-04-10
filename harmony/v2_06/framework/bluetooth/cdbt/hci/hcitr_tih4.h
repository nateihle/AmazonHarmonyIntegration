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

#ifndef __HCITR_TUH4_H_INCLUDED__
#define __HCITR_TUH4_H_INCLUDED__


#ifdef __cplusplus
extern "C" {
#endif

typedef enum _hcitr_tih4_power_event_e
{
	HCITR_TIH4_POWER_EVENT_PREPARE_TO_SLEEP,
	HCITR_TIH4_POWER_EVENT_SLEEP,
	HCITR_TIH4_POWER_EVENT_WAKE_UP,
	HCITR_TIH4_POWER_EVENT_AWAKE

} bt_hcitr_tih4_power_event_e;


typedef void (*bt_hcitr_tih4_power_callback_fp)(bt_hcitr_tih4_power_event_e event);

void bt_hcitr_tih4_init(bt_hcitr_tih4_power_callback_fp callback);
void bt_hcitr_tih4_reset(void);
void bt_hcitr_tih4_start(void);
void bt_hcitr_tih4_wake_up(void);

#ifdef __cplusplus
}
#endif

#endif // __HCITR_TUH4_H_INCLUDED__

