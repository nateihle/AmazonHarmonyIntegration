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

#ifndef __BT_SIGNAL_H
#define __BT_SIGNAL_H

#ifdef __cplusplus
extern "C" {
#endif

struct _bt_signal_t;
typedef struct _bt_signal_t bt_signal_t;

typedef void (*bt_signal_handler_fp)(bt_signal_t* signal, void* param);

struct _bt_signal_t
{
	bt_signal_t* next_signal;
	bt_byte signaled;
	bt_signal_handler_fp handler;
	void* handler_param;
};

void bt_signal_init(void);

bt_bool bt_signal_register(bt_signal_t* signal, bt_signal_handler_fp handler, void* handler_param);

void bt_signal_unregister(bt_signal_t* signal);

void bt_signal_set(bt_signal_t* signal);

void bt_signal_process_pending(void);

#ifdef __cplusplus
}
#endif

#endif // __BT_SIGNAL_H
