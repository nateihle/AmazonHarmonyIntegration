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

#ifndef __L2CAP_CONFIG_H
#define __L2CAP_CONFIG_H

#ifndef L2CAP_MAX_CMD_BUFFERS
#error "L2CAP_MAX_CMD_BUFFERS is not defined"
#endif

#ifndef L2CAP_MAX_PSMS
#error "L2CAP_MAX_PSMS is not defined"
#endif

#ifndef L2CAP_MAX_CHANNELS
#error "L2CAP_MAX_CHANNELS is not defined"
#endif

#ifndef L2CAP_MAX_FIXED_CHANNELS
#define L2CAP_MAX_FIXED_CHANNELS    0
#define L2CAP_FIXED_CHANNELS_DECL	\
	bt_l2cap_fixed_channel_t* _l2cap_fixed_channels = NULL;	\
	bt_byte _l2cap_max_fixed_channels = 0;
#else
#define L2CAP_FIXED_CHANNELS_DECL	\
	bt_l2cap_fixed_channel_t _l2cap_fixed_channels_buffer[L2CAP_MAX_FIXED_CHANNELS * L2CAP_MAX_MANAGERS];	\
	bt_l2cap_fixed_channel_t* _l2cap_fixed_channels = &_l2cap_fixed_channels_buffer[0];	\
	bt_byte _l2cap_max_fixed_channels = L2CAP_MAX_FIXED_CHANNELS;
#endif

/**
* \brief L2CAP_HCI_PACKET_TYPE.
* \ingroup btconfig
*
* \details Defines a set of packets that link manager is allowed to use when calling bt_l2cap_connect.
* The default value is to enable all packet types.
*/
// enable all packet types
#ifndef L2CAP_HCI_PACKET_TYPE
#define L2CAP_HCI_PACKET_TYPE  \
	HCI_BB_PACKET_TYPE_DM1 | \
	HCI_BB_PACKET_TYPE_DH1 | \
	HCI_BB_PACKET_TYPE_DM3 | \
	HCI_BB_PACKET_TYPE_DH3 | \
	HCI_BB_PACKET_TYPE_DM5 | \
	HCI_BB_PACKET_TYPE_DH5
#endif

// to disable all EDR packets use the following
/*
#define L2CAP_HCI_PACKET_TYPE	\
	HCI_BB_PACKET_TYPE_NO_2_DH1 | \
	HCI_BB_PACKET_TYPE_NO_3_DH1 | \
	HCI_BB_PACKET_TYPE_DM1 | \
	HCI_BB_PACKET_TYPE_DH1 | \
	HCI_BB_PACKET_TYPE_NO_2_DH3 | \
	HCI_BB_PACKET_TYPE_NO_3_DH3 | \
	HCI_BB_PACKET_TYPE_DM3 | \
	HCI_BB_PACKET_TYPE_DH3 | \
	HCI_BB_PACKET_TYPE_NO_2_DH5 | \
	HCI_BB_PACKET_TYPE_NO_3_DH5 | \
	HCI_BB_PACKET_TYPE_DM5 | \
	HCI_BB_PACKET_TYPE_DH5
*/

/**
* \brief L2CAP_HCI_PAGE_SCAN_REPETITION_MODE.
* \ingroup btconfig
*
* \details Defines a default value of the page scan repetition mode when calling bt_l2cap_connect. 
* Must be set to one of the following values:
*     HCI_PAGE_SCAN_REPETITION_MODE_R0
*     HCI_PAGE_SCAN_REPETITION_MODE_R1
*     HCI_PAGE_SCAN_REPETITION_MODE_R2
*     
* The default value is HCI_PAGE_SCAN_REPETITION_MODE_R0. 
*/
#ifndef L2CAP_HCI_PAGE_SCAN_REPETITION_MODE
#define L2CAP_HCI_PAGE_SCAN_REPETITION_MODE   HCI_PAGE_SCAN_REPETITION_MODE_R0
#endif

/**
* \brief L2CAP_HCI_ROLE_SWITCH.
* \ingroup btconfig
*
* \details Defines a default value of the role switch parameter when calling bt_l2cap_connect.
* Must be set to one of the following values:
*     HCI_ROLE_SWITCH_ALLOW
*     HCI_ROLE_SWITCH_DISALLOW
* The default value is to allow the role switch.
*/
#ifndef L2CAP_HCI_ROLE_SWITCH
#define L2CAP_HCI_ROLE_SWITCH   HCI_ROLE_SWITCH_ALLOW
#endif

/**
* \brief L2CAP_IDLE_CONNECTION_TIMEOUT.
* \ingroup btconfig
*
* \details Defines a timeout value in seconds for closing idle HCI connections (connections that do not have L2CAP channels open).
* Connections are closed only if local device is master.
* The default value is 5 seconds.
*/
#ifndef L2CAP_IDLE_CONNECTION_TIMEOUT
#define L2CAP_IDLE_CONNECTION_TIMEOUT	5 /* seconds */
#endif

#ifdef L2CAP_ENABLE_EXT_FEATURES
	#define L2CAP_DECL_ERETR_FUNCTIONS	\
		bt_l2cap_channel_ext_t  _l2cap_channels_ext[(L2CAP_MAX_CHANNELS) * L2CAP_MAX_MANAGERS];	\
		\
		void (*_l2cap_eretr_recv_fp)(bt_l2cap_mgr_p pmgr, bt_l2cap_channel_t* pch, bt_byte* pdata, bt_int len) = &_bt_l2cap_eretr_recv;	\
		bt_bool (*_l2cap_eretr_send_data_fp)(bt_l2cap_channel_t* pch, bt_byte* data, bt_int len, bt_l2cap_send_data_callback_fp cb, void* cb_param) = &_bt_l2cap_eretr_send_data;	\
		bt_bool (*_l2cap_eretr_send_smart_data_fp)(bt_l2cap_channel_t* pch, bt_packet_t* packet, bt_int len, bt_l2cap_send_data_callback_fp cb, void* cb_param) = &_bt_l2cap_eretr_send_smart_data;	\
		bt_bool (*_l2cap_eretr_handle_xmit_event_fp)(bt_l2cap_xmit_event_param_t* param) = &_bt_l2cap_eretr_handle_xmit_event;	\
		void (*_l2cap_eretr_pack_config_request_fp)(bt_l2cap_channel_t* channel, bt_byte* buffer, bt_int buffer_len, bt_int* offset) = &_bt_l2cap_eretr_pack_config_request;

#else
	#define L2CAP_DECL_ERETR_FUNCTIONS	\
		bt_l2cap_channel_ext_t*  _l2cap_channels_ext = NULL;	\
		\
		void (*_l2cap_eretr_recv_fp)(bt_l2cap_mgr_p pmgr, bt_l2cap_channel_t* pch, bt_byte* pdata, bt_int len) = NULL;	\
		bt_bool (*_l2cap_eretr_send_data_fp)(bt_l2cap_channel_t* pch, bt_byte* data, bt_int len, bt_l2cap_send_data_callback_fp cb, void* cb_param) = NULL;	\
		bt_bool (*_l2cap_eretr_send_smart_data_fp)(bt_l2cap_channel_t* pch, bt_packet_t* packet, bt_int len, bt_l2cap_send_data_callback_fp cb, void* cb_param) = NULL;	\
		bt_bool (*_l2cap_eretr_handle_xmit_event_fp)(bt_l2cap_xmit_event_param_t* param) = NULL;	\
		void (*_l2cap_eretr_pack_config_request_fp)(bt_l2cap_channel_t* channel, bt_byte* buffer, bt_int buffer_len, bt_int* offset) = NULL;

#endif

#ifdef _DEBUG


#define L2CAP_ALLOCATE_BUFFERS_RAM_SIZE_VAR	\
	const bt_uint _ram_size_l2cap_buffers =	\
		sizeof(bt_buffer_mgr_t) +	\
		sizeof(_l2cap_cmd_buffer_headers) +	\
		sizeof(_l2cap_cmd_buffers) +	\
		sizeof(bt_l2cap_command_t) +	\
		sizeof(bt_l2cap_cfg_option_t) * L2CAP_MAX_OPTIONS +	\
		sizeof(_l2cap_max_cmd_buffers) +	\
		sizeof(_l2cap_psms) +	\
		sizeof(_l2cap_channels) +	\
		sizeof(_l2cap_max_psms) +	\
		sizeof(_l2cap_max_channels) +	\
		sizeof(_l2cap_hci_connect_packet_type) + \
		sizeof(_l2cap_hci_page_scan_repetition_mode) +	\
		sizeof(_l2cap_hci_role_switch) +	\
		sizeof(_l2cap_idle_hci_connection_timeout) +	\
		sizeof(_l2cap_request_handlers) +	\
		sizeof(_l2cap_response_handlers) +	\
		sizeof(_l2cap_cmd_parsers) +	\
		sizeof(_l2cap_fixed_channels) +	\
		sizeof(_l2cap_max_fixed_channels) +	\
		sizeof(bt_l2cap_fixed_channel_t) * _l2cap_max_fixed_channels;

#else
	#define L2CAP_ALLOCATE_BUFFERS_RAM_SIZE_VAR
#endif

#include "cdbt/l2cap/l2cap_config_handlers.h"

#define L2CAP_ALLOCATE_BUFFERS_VARS()	\
	bt_buffer_header_t  _l2cap_cmd_buffer_headers[L2CAP_MAX_CMD_BUFFERS];	\
	bt_l2cap_command_t  _l2cap_cmd_buffers[L2CAP_MAX_CMD_BUFFERS];	\
	const bt_byte       _l2cap_max_cmd_buffers = L2CAP_MAX_CMD_BUFFERS; \
	\
	bt_l2cap_psm_t      _l2cap_psms[L2CAP_MAX_PSMS * L2CAP_MAX_MANAGERS];	\
	const bt_byte       _l2cap_max_psms = L2CAP_MAX_PSMS; \
	bt_l2cap_channel_t  _l2cap_channels[(L2CAP_MAX_CHANNELS) * L2CAP_MAX_MANAGERS];	\
	const bt_byte       _l2cap_max_channels = L2CAP_MAX_CHANNELS; \
	\
	bt_uint             _l2cap_hci_connect_packet_type;	\
	bt_byte             _l2cap_hci_page_scan_repetition_mode;	\
	bt_byte             _l2cap_hci_role_switch;	\
	bt_long             _l2cap_idle_hci_connection_timeout;	\
	\
	bt_buffer_header_t  _l2cap_connect_params_headers[(L2CAP_MAX_CHANNELS) * (L2CAP_MAX_MANAGERS)];	\
	bt_l2cap_connect_params_t  _l2cap_connect_params[(L2CAP_MAX_CHANNELS) * (L2CAP_MAX_MANAGERS)];	\
	\
	L2CAP_DECL_ERETR_FUNCTIONS	\
	\
	L2CAP_FIXED_CHANNELS_DECL	\
	\
	L2CAP_ALLOCATE_BUFFERS_RAM_SIZE_VAR	\



#define L2CAP_ALLOCATE_BUFFERS_FUNCTION()	\
	void _l2cap_allocate_buffers()	\
	{	\
		bt_byte i;	\
		_l2cap_hci_connect_packet_type = L2CAP_HCI_PACKET_TYPE;	\
		_l2cap_hci_page_scan_repetition_mode = L2CAP_HCI_PAGE_SCAN_REPETITION_MODE;	\
		_l2cap_hci_role_switch = L2CAP_HCI_ROLE_SWITCH;	\
		_l2cap_idle_hci_connection_timeout = L2CAP_IDLE_CONNECTION_TIMEOUT;	\
		\
		_zero_memory(_l2cap_psms, sizeof(bt_l2cap_psm_t) * (L2CAP_MAX_PSMS) * (L2CAP_MAX_MANAGERS));	\
		_zero_memory(_l2cap_channels, sizeof(bt_l2cap_channel_t) * (L2CAP_MAX_CHANNELS) * (L2CAP_MAX_MANAGERS));	\
		if (_l2cap_channels_ext)	\
			_zero_memory(_l2cap_channels_ext, sizeof(bt_l2cap_channel_ext_t) * (L2CAP_MAX_CHANNELS) * (L2CAP_MAX_MANAGERS));	\
		if (_l2cap_max_fixed_channels)	\
			_zero_memory(_l2cap_fixed_channels, sizeof(bt_l2cap_fixed_channel_t) * _l2cap_max_fixed_channels);	\
		\
		for (i = 0; i < L2CAP_MAX_MANAGERS; i++)	\
		{	\
			_mgrs[i]._psms = &_l2cap_psms[i * L2CAP_MAX_PSMS];	\
			_mgrs[i]._channels = &_l2cap_channels[i * (L2CAP_MAX_CHANNELS)];	\
			bt_init_buffer_mgr(&_mgrs[i].connect_params_mgr, L2CAP_MAX_CHANNELS, sizeof(bt_l2cap_connect_params_t), &_l2cap_connect_params_headers[(L2CAP_MAX_CHANNELS) * i], &_l2cap_connect_params[(L2CAP_MAX_CHANNELS) * i]);	\
			\
			if (_l2cap_channels_ext)	\
			{	\
				int j;	\
				\
				for (j = 0; j < L2CAP_MAX_CHANNELS; j++)	\
				{	\
					_mgrs[i]._channels[j].ext = &_l2cap_channels_ext[i * (L2CAP_MAX_CHANNELS) + j];	\
				}	\
			}	\
			if (_l2cap_max_fixed_channels)	\
				_mgrs[i]._fixed_channels = &_l2cap_fixed_channels[i * _l2cap_max_fixed_channels];	\
		}	\
	}	\

#define L2CAP_ALLOCATE_BUFFERS()	\
		L2CAP_ALLOCATE_BUFFERS_VARS()	\
		L2CAP_ALLOCATE_BUFFERS_FUNCTION()	\
		typedef int L2CAP_BUFFERS_ALLOCATED

#endif // __L2CAP_CONFIG_H
