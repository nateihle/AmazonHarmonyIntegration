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

#ifndef __L2CAP_PRIVATE_H
#define __L2CAP_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

#define CHANNEL_SIGNAL_CMD_DISCONNECT_FIXED    0

#define L2CAP_EXT_FEATURES_ENABLED()           (_l2cap_eretr_recv_fp != NULL)

//
// Global variables defined in L2CAP modules
// -------------------------------------------------------------------
//

// In l2cap_mgr.c
extern bt_l2cap_mgr_t _mgrs[];
#ifdef _DEBUG
extern const bt_int _ram_size_l2cap_mgr;
#endif


typedef void (*bt_l2cap_request_handler_fp)(bt_l2cap_mgr_p pmgr, bt_hci_conn_state_p pconn, bt_l2cap_cmd_header_p pcmd);
typedef void (*bt_l2cap_response_handler_fp)(bt_l2cap_mgr_p pmgr, bt_l2cap_cmd_header_p pcmd);

typedef bt_l2cap_cmd_header_t* (*bt_l2cap_cmd_parser_fp)(bt_byte* pdata, bt_int len, bt_int* poffset);
typedef bt_bool (*bt_l2cap_cmd_assembler_fp)(bt_l2cap_cmd_header_t* pcmd, bt_byte* buffer, bt_int buffer_len, bt_int* poffset);

//
// Global variables defined by OEM configuration
// -------------------------------------------------------------------
//

extern bt_buffer_header_t _l2cap_cmd_buffer_headers[];
extern bt_l2cap_command_t _l2cap_cmd_buffers[];
extern const bt_byte      _l2cap_max_cmd_buffers;

extern bt_buffer_header_t _frame_buffer_headers[];
extern bt_byte            _frame_buffers[];

extern bt_l2cap_psm_t     _l2cap_psms[];
extern const bt_byte      _l2cap_max_psms;
extern bt_l2cap_channel_t _l2cap_channels[];
extern const bt_byte      _l2cap_max_channels;

extern bt_byte            _l2cap_cmd_frame_buffer[];
extern bt_uint            _l2cap_cmd_frame_buffer_size;

extern bt_uint            _l2cap_hci_connect_packet_type;
extern bt_byte            _l2cap_hci_page_scan_repetition_mode;
extern bt_byte            _l2cap_hci_role_switch;
extern bt_long            _l2cap_idle_hci_connection_timeout;

extern bt_l2cap_fixed_channel_t* _l2cap_fixed_channels;
extern bt_byte                   _l2cap_max_fixed_channels;

extern bt_buffer_header_t _l2cap_connect_params_headers[];
extern bt_l2cap_connect_params_t  _l2cap_connect_params[];

extern const bt_l2cap_request_handler_fp  _l2cap_request_handlers[];
extern const bt_l2cap_response_handler_fp _l2cap_response_handlers[];
extern const bt_l2cap_cmd_parser_fp       _l2cap_cmd_parsers[];
extern const bt_l2cap_cmd_assembler_fp    _l2cap_cmd_assemblers[];

extern void (*_l2cap_eretr_recv_fp)(bt_l2cap_mgr_p pmgr, bt_l2cap_channel_t* pch, bt_byte* pdata, bt_int len);
extern bt_bool (*_l2cap_eretr_send_data_fp)(bt_l2cap_channel_t* pch, bt_byte* data, bt_int len, bt_l2cap_send_data_callback_fp cb, void* cb_param);
extern bt_bool (*_l2cap_eretr_send_smart_data_fp)(bt_l2cap_channel_t* pch, bt_packet_t* packet, bt_int len, bt_l2cap_send_data_callback_fp cb, void* cb_param);
extern bt_bool (*_l2cap_eretr_handle_xmit_event_fp)(bt_l2cap_xmit_event_param_t* param);
extern void (*_l2cap_eretr_pack_config_request_fp)(bt_l2cap_channel_t* channel, bt_byte* buffer, bt_int buffer_len, bt_int* offset);

#ifdef _DEBUG
extern const bt_uint _ram_size_l2cap_buffers;
#endif

//
// Private global functions
// -------------------------------------------------------------------
//

// From cmd_reject.c
void _process_reject(bt_l2cap_mgr_p pmgr, bt_l2cap_cmd_header_p pcmd);
// From cmd_connect.c
void _process_conn_req(bt_l2cap_mgr_p pmgr, bt_hci_conn_state_p pconn, bt_l2cap_cmd_header_p pcmd);
void _process_conn_res(bt_l2cap_mgr_p pmgr, bt_l2cap_cmd_header_p pcmd);
// From cmd_config.c
void _process_config_req(bt_l2cap_mgr_p pmgr, bt_hci_conn_state_p pconn, bt_l2cap_cmd_header_p pcmd);
void _process_config_res(bt_l2cap_mgr_p pmgr, bt_l2cap_cmd_header_p pcmd);
// From cmd_disconnect.c
void _process_dconn_req(bt_l2cap_mgr_p pmgr, bt_hci_conn_state_p pconn, bt_l2cap_cmd_header_p pcmd);
void _process_dconn_res(bt_l2cap_mgr_p pmgr, bt_l2cap_cmd_header_p pcmd);
// From cmd_echo.c
void _process_echo_req(bt_l2cap_mgr_p pmgr, bt_hci_conn_state_p pconn, bt_l2cap_cmd_header_p pcmd);
void _process_echo_res(bt_l2cap_mgr_p pmgr, bt_l2cap_cmd_header_p pcmd);
// From cmd_info.c
void _process_info_req(bt_l2cap_mgr_p pmgr, bt_hci_conn_state_p pconn, bt_l2cap_cmd_header_p pcmd);
void _process_info_res(bt_l2cap_mgr_p pmgr, bt_l2cap_cmd_header_p pcmd);
// From cmd_unknown.c
void _process_unknown_req(bt_l2cap_mgr_p pmgr, bt_hci_conn_state_p pconn, bt_l2cap_cmd_header_p pcmd);
void _process_unknown_res(bt_l2cap_mgr_p pmgr, bt_l2cap_cmd_header_p pcmd);

void _process_conn_param_update_req(bt_l2cap_mgr_p pmgr, bt_hci_conn_state_p pconn, bt_l2cap_cmd_header_t* pcmd);
void _process_conn_param_update_res(bt_l2cap_mgr_p pmgr, bt_l2cap_cmd_header_t* pcmd);

// From channel_cmd_recv.c
bt_l2cap_cmd_header_p read_command(bt_byte_p pdata, bt_int len, bt_int* offset, bt_l2cap_cmd_header_t* pheader);

// From l2cap_recv.c
void _l2cap_data_receive_callback(bt_l2cap_mgr_p pmg, bt_hci_conn_state_p pconn, bt_byte_p pdata, bt_int len);

// Defined by OEM through library configuration
void _l2cap_allocate_buffers(void);

void _bt_l2cap_clear_channel_cmd_queue(bt_l2cap_channel_t* channel);

bt_l2cap_cmd_header_p _read_cmd_reject(bt_byte_p pdata, bt_int len, bt_int_p poffset);
bt_l2cap_cmd_header_p _read_conn_request(bt_byte_p pdata, bt_int len, bt_int_p poffset);
bt_l2cap_cmd_header_p _read_conn_response(bt_byte_p pdata, bt_int len, bt_int_p poffset);
bt_l2cap_cmd_header_p _read_config_request(bt_byte_p pdata, bt_int len, bt_int_p poffset);
bt_l2cap_cmd_header_p _read_config_response(bt_byte_p pdata, bt_int len, bt_int_p poffset);
bt_l2cap_cmd_header_p _read_dconn_request(bt_byte_p pdata, bt_int len, bt_int_p poffset);
bt_l2cap_cmd_header_p _read_dconn_response(bt_byte_p pdata, bt_int len, bt_int_p poffset);
bt_l2cap_cmd_header_p _read_echo_request(bt_byte_p pdata, bt_int len, bt_int_p poffset);
bt_l2cap_cmd_header_p _read_echo_response(bt_byte_p pdata, bt_int len, bt_int_p poffset);
bt_l2cap_cmd_header_p _read_info_request(bt_byte_p pdata, bt_int len, bt_int_p poffset);
bt_l2cap_cmd_header_p _read_info_response(bt_byte_p pdata, bt_int len, bt_int_p poffset);
bt_l2cap_cmd_header_p _read_conn_param_update_request(bt_byte_p pdata, bt_int len, bt_int_p poffset);
bt_l2cap_cmd_header_p _read_conn_param_update_response(bt_byte_p pdata, bt_int len, bt_int_p poffset);

bt_int _pack_cmd_reject(bt_l2cap_cmd_header_t* pcmd, bt_byte_p buffer, bt_int buffer_len, bt_int_p poffset);
bt_int _pack_conn_request(bt_l2cap_cmd_header_t* pcmd, bt_byte_p buffer, bt_int buffer_len, bt_int_p poffset);
bt_int _pack_conn_response(bt_l2cap_cmd_header_t* pcmd, bt_byte_p buffer, bt_int buffer_len, bt_int_p poffset);
bt_int _pack_config_request(bt_l2cap_cmd_header_t* pcmd, bt_byte_p buffer, bt_int buffer_len, bt_int_p poffset);
bt_int _pack_config_response(bt_l2cap_cmd_header_t* pcmd, bt_byte_p buffer, bt_int buffer_len, bt_int_p poffset);
bt_int _pack_dconn_request(bt_l2cap_cmd_header_t* pcmd, bt_byte_p buffer, bt_int buffer_len, bt_int_p poffset);
bt_int _pack_dconn_response(bt_l2cap_cmd_header_t* pcmd, bt_byte_p buffer, bt_int buffer_len, bt_int_p poffset);
bt_int _pack_echo_request(bt_l2cap_cmd_header_t* pcmd, bt_byte_p buffer, bt_int buffer_len, bt_int_p poffset);
bt_int _pack_echo_response(bt_l2cap_cmd_header_t* pcmd, bt_byte_p buffer, bt_int buffer_len, bt_int_p poffset);
bt_int _pack_info_request(bt_l2cap_cmd_header_t* pcmd, bt_byte_p buffer, bt_int buffer_len, bt_int_p poffset);
bt_int _pack_info_response(bt_l2cap_cmd_header_t* pcmd, bt_byte_p buffer, bt_int buffer_len, bt_int_p poffset);
bt_int _pack_conn_param_update_request(bt_l2cap_cmd_header_t* pcmd, bt_byte_p buffer, bt_int buffer_len, bt_int_p poffset);
bt_int _pack_conn_param_update_response(bt_l2cap_cmd_header_t* pcmd, bt_byte_p buffer, bt_int buffer_len, bt_int_p poffset);

void _bt_l2cap_eretr_pack_config_request(bt_l2cap_channel_t* channel, bt_byte* buffer, bt_int buffer_len, bt_int* offset);

void _bt_l2cap_process_connect_signal(bt_l2cap_mgr_t* mgr);

#ifdef __cplusplus
}
#endif

#endif // __L2CAP_PRIVATE_H
