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

#ifndef __RFCOMM_PRIVATE_H
#define __RFCOMM_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

//
// Global variables defined by OEM configuration
// -------------------------------------------------------------------
//

extern bt_rfcomm_session_t _rfcomm_sessions[];
extern const bt_byte       _rfcomm_max_sessions;
extern bt_rfcomm_dlc_t     _rfcomm_dlcs[];
extern const bt_byte       _rfcomm_max_dlcs;
extern bt_rfcomm_server_channel_t _rfcomm_channels[];
extern const bt_byte       _rfcomm_max_channels;
extern const bt_uint       _rfcomm_pdu_size;
extern bt_buffer_header_t  _rfcomm_data_buffer_headers[];
extern bt_byte             _rfcomm_data_buffers[];
extern const bt_byte       _rfcomm_max_data_buffers;
extern const bt_uint       _rfcomm_info_len;
extern bt_buffer_header_t  _rfcomm_cmd_buffer_headers[];
extern bt_rfcomm_command_t _rfcomm_cmd_buffers[];
extern const bt_byte       _rfcomm_max_cmd_buffers;
extern const bt_byte       _rfcomm_local_credit;
extern const bt_bool       _rfcomm_enable_multidevice_channels;
extern const bt_byte       _rfcomm_local_credit_send_threshold;

#ifdef _DEBUG
extern const bt_uint _ram_size_rfcomm_buffers;
#endif

//
// Private global functions
// -------------------------------------------------------------------
//

// From rfcomm_session.c
void _rfcomm_init_sessions(void);
bt_rfcomm_session_p rfcomm_find_session(bt_l2cap_channel_t* pch);
bt_bool rfcomm_send_cmd(bt_rfcomm_dlc_p pdlc, bt_rfcomm_command_p pcmd);

// From rfcomm_cmdbuffer.c
bt_bool _rfcomm_init_cmd_buffers(void);
bt_rfcomm_command_p _rfcomm_alloc_cmd_buffer(void);
void _rfcomm_free_cmd_buffer(void* p);

// From frame_ua.c
void _rfcomm_send_ua_response(bt_rfcomm_dlc_p pdlc, bt_rfcomm_command_p pcmd, bt_rfcomm_cmd_callback_fp cb);
void _rfcomm_process_cmd_frame_ua(bt_rfcomm_session_p psess, bt_rfcomm_command_p pcmd);
void _rfcomm_process_res_frame_ua(bt_rfcomm_session_p psess, bt_rfcomm_command_p pres);

// From frame_sabm.c
void _rfcomm_send_sabm_cmd(bt_rfcomm_dlc_p pdlc, bt_rfcomm_cmd_callback_fp cb);
void _rfcomm_process_cmd_frame_sabm(bt_rfcomm_session_p psess, bt_rfcomm_command_p pcmd);
void _rfcomm_process_res_frame_sabm(bt_rfcomm_session_p psess, bt_rfcomm_command_p pres);

// From frame_dm.c
void _rfcomm_send_dm_response(bt_rfcomm_dlc_p pdlc, bt_rfcomm_command_p pcmd);
void _rfcomm_process_cmd_frame_dm(bt_rfcomm_session_p psess, bt_rfcomm_command_p pcmd);
void _rfcomm_process_res_frame_dm(bt_rfcomm_session_p psess, bt_rfcomm_command_p pres);

// From frame_uih.c
void _rfcomm_process_cmd_frame_uih(bt_rfcomm_session_p psess, bt_rfcomm_command_p pcmd);
void _rfcomm_process_res_frame_uih(bt_rfcomm_session_p psess, bt_rfcomm_command_p pres);

// From frame_disc.c
void _rfcomm_process_cmd_frame_disc(bt_rfcomm_session_p psess, bt_rfcomm_command_p pcmd);
void _rfcomm_process_res_frame_disc(bt_rfcomm_session_p psess, bt_rfcomm_command_p pres);

// From command_queue.c
void _bt_rfcomm_clear_queue(bt_rfcomm_dlc_t* pdlc);

// From rfcomm_cmd_recv.c
void _rfcomm_l2cap_read_data_callback(struct _bt_l2cap_channel_t *pch, bt_byte_p pdata, bt_int len);

// From rfcomm_fcs.c
bt_bool check_fcs(bt_byte_p buffer, bt_byte len, bt_byte recv_fcs);
bt_byte _calc_fcs(bt_byte_p buffer, bt_byte len);

// From rfcomm_cmd_send.c
bt_bool _rfcomm_send_command(bt_rfcomm_dlc_p pdlc,bt_rfcomm_command_p pcmd);

// From rfcomm.c
void _rfcomm_l2cap_state_changed_callback(bt_l2cap_channel_t* pch, bt_int new_state, void* param);

// Defined by OEM through library configuration
void _rfcomm_allocate_buffers(void);

bt_rfcomm_session_t* _bt_rfcomm_mgr_allocate_session(bt_bdaddr_t* bdaddr_remote);


#ifdef __cplusplus
}
#endif

#endif // __RFCOMM_PRIVATE_H
