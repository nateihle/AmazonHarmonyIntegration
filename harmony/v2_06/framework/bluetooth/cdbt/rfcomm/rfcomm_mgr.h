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

#ifndef __RFCOMM_MGR_H
#define __RFCOMM_MGR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _bt_rfcomm_session_listener_t
{
	struct _bt_rfcomm_session_listener_t* next_listener;

	bt_rfcomm_state_callback_fp callback;
	void* callback_param;
} bt_rfcomm_session_listener_t;

typedef struct _bt_rfcomm_server_channel_t
{
	bt_byte id;
	bt_rfcomm_dlc_state_callback_fp listen_cb;
	void* listen_param;
} bt_rfcomm_server_channel_t;

typedef struct _bt_rfcomm_mgr_t
{
	bt_rfcomm_session_listener_t* listeners;
	bt_rfcomm_server_channel_t* channels;
} bt_rfcomm_mgr_t;

bt_bool bt_rfcomm_register_listener(bt_rfcomm_session_listener_t* listener);
void bt_rfcomm_unregister_listener(bt_rfcomm_session_listener_t* listener);

void _rfcomm_init_mgr(void);
bt_rfcomm_mgr_t* _bt_rfcomm_get_mgr(void);

void _bt_rfcomm_mgr_notify_listeners(bt_rfcomm_session_t* session, bt_int evt);

bt_rfcomm_server_channel_t* _bt_rfcomm_allocate_channel(bt_byte id);
bt_rfcomm_server_channel_t* _bt_rfcomm_find_channel(bt_byte id);

void _bt_rfcomm_mgr_l2cap_listen_callback(bt_l2cap_channel_t* pch, void* param);


#ifdef __cplusplus
}
#endif

#endif // __RFCOMM_MGR_H
