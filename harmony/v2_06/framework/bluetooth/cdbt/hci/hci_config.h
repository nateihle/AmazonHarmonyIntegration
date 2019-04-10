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

#ifndef __HCI_CONFIG_H
#define __HCI_CONFIG_H

#ifdef _DEBUG

#define HCI_ALLOCATE_BUFFERS_RAM_SIZE_VAR	\
	const bt_uint _ram_size_hci_buffers =	\
		sizeof(bt_buffer_mgr_t) * 2 +	\
		sizeof(_hci_cmd_buffer_headers) +	\
		sizeof(_hci_cmd_buffers) +	\
		sizeof(_hci_max_cmd_buffers) +	\
		sizeof(_hci_cmd_param_buffers) +	\
		sizeof(_hci_max_cmd_param_len) +	\
		sizeof(_hci_send_data_buffer_headers) +	\
		sizeof(_hci_send_data_buffers) +	\
		sizeof(_hci_max_data_buffers) +	\
		sizeof(_hci_connections) +	\
		sizeof(_hci_max_hci_connections) +	\
		sizeof(_recv_buffer) +	\
		sizeof(_send_buffer) +	\
		sizeof(_hci_rcv_buffer_len) +	\
		sizeof(_hci_tx_buffer_len) +	\
		sizeof(_hci_l2cap_buffer_len) +	\
		sizeof(_hci_enable_ctrl_to_host_flow_control) +	\
		sizeof(_conn_state_recv_buffers) +	\
		sizeof(_hci_enable_sco) + \
		sizeof(_hci_recv_sco_data_packet_fp) +	\
		sizeof(_bt_ssp_init) + \
		sizeof(_bt_ssp_evt_handler) +	\
		sizeof(_hci_event_handlers) +	\
		HCI_SIZEOF_LE_CONN_STATES +	\
		HCI_SIZEOF_LE_CTRL_STATE +	\
		sizeof(_bt_hci_le_init);

#else
	#define HCI_ALLOCATE_BUFFERS_RAM_SIZE_VAR
#endif

#ifndef HCI_MAX_CMD_BUFFERS
#error "HCI_MAX_CMD_BUFFERS is not defined"
#endif

#ifndef HCI_MAX_DATA_BUFFERS
#error "HCI_MAX_DATA_BUFFERS is not defined"
#endif

#ifndef HCI_MAX_HCI_CONNECTIONS
#error "HCI_MAX_HCI_CONNECTIONS is not defined"
#endif

#ifndef HCI_RX_BUFFER_LEN
#error "HCI_RX_BUFFER_LEN is not defined"
#endif

#if HCI_RX_BUFFER_LEN < 40
#error "HCI_TX_BUFFER_LEN >= 40 required"
#endif

#if HCI_RX_BUFFER_LEN < (HCI_TRANSPORT_HEADER_LEN + HCI_ACL_DATA_HEADER_LEN)
#error "HCI_RX_BUFFER_LEN >= (HCI_TRANSPORT_HEADER_LEN + HCI_ACL_DATA_HEADER_LEN) required"
#endif

#if HCI_RX_BUFFER_LEN < (HCI_TRANSPORT_HEADER_LEN + HCI_MAX_CMD_PARAM_LEN + 2)
#error "HCI_RX_BUFFER_LEN >= (HCI_TRANSPORT_HEADER_LEN + HCI_MAX_CMD_PARAM_LEN + 2) required"
#endif

#ifndef HCI_TX_BUFFER_LEN
#define HCI_TX_BUFFER_LEN HCI_RX_BUFFER_LEN
#endif

#if HCI_TX_BUFFER_LEN < 32
#error "HCI_TX_BUFFER_LEN >= 32 required"
#endif

#if HCI_TX_BUFFER_LEN < HCI_TRANSPORT_HEADER_LEN + HCI_ACL_DATA_HEADER_LEN
#error "HCI_TX_BUFFER_LEN >= HCI_TRANSPORT_HEADER_LEN + HCI_ACL_DATA_HEADER_LEN required"
#endif

#if HCI_TX_BUFFER_LEN < HCI_TRANSPORT_HEADER_LEN + HCI_CMD_HEADER_LEN + HCI_MAX_CMD_PARAM_LEN
#error "HCI_TX_BUFFER_LEN >= HCI_TRANSPORT_HEADER_LEN + HCI_CMD_HEADER_LEN + HCI_MAX_CMD_PARAM_LEN required"
#endif

#ifndef HCI_L2CAP_BUFFER_LEN
#define HCI_L2CAP_BUFFER_LEN (HCI_RX_BUFFER_LEN - HCI_ACL_DATA_HEADER_LEN - HCI_TRANSPORT_HEADER_LEN)
#endif

#ifndef HCI_MAX_CMD_PARAM_LEN
#error "HCI_MAX_CMD_PARAM_LEN is not defined"
#endif

#if HCI_MAX_CMD_BUFFERS < 2
#error "HCI_MAX_CMD_BUFFERS >= 2 required"
#endif

#if HCI_MAX_DATA_BUFFERS < 2
#error "HCI_MAX_DATA_BUFFERS >= 2 required"
#endif

#ifdef BT_ENABLE_SCO
#if HCI_MAX_HCI_CONNECTIONS < 2
#error "HCI_MAX_HCI_CONNECTIONS >= 2 required"
#endif
#undef BT_ENABLE_SCO
#define BT_ENABLE_SCO    BT_TRUE
#define HCI_INIT_SCO_HANDLERS	\
	_hci_recv_sco_data_packet_fp = &_hci_recv_sco_data_packet;
#else
#if HCI_MAX_HCI_CONNECTIONS < 1
#error "HCI_MAX_HCI_CONNECTIONS >= 1 required"
#endif
#define BT_ENABLE_SCO    BT_FALSE
#define HCI_INIT_SCO_HANDLERS	\
	_hci_recv_sco_data_packet_fp = NULL;
#endif

#ifdef BT_ENABLE_SSP
#define HCI_INIT_SSP_HANDLERS	\
	_bt_ssp_evt_handler = &bt_ssp_evt_handler;	\
	_bt_ssp_init = &bt_ssp_init;
#else
#define HCI_INIT_SSP_HANDLERS	\
	_bt_ssp_evt_handler = NULL;	\
	_bt_ssp_init = NULL;
#endif

#ifdef BT_ENABLE_BLE
#define HCI_SIZEOF_LE_CONN_STATES	\
	sizeof(_bt_hci_le_conn_states)
#define HCI_DECLARE_LE_CONN_STATES	\
	bt_hci_le_conn_state_t    _bt_hci_le_conn_states[HCI_MAX_HCI_CONNECTIONS];
#define HCI_INIT_LE_CONN_STATES	\
	_hci_connections[i].le_conn_state = &_bt_hci_le_conn_states[i];
#define HCI_SIZEOF_LE_CTRL_STATE	\
	sizeof(_bt_hci_le_ctrl_state)
#define HCI_DECLARE_LE_CTRL_STATE	\
	bt_hci_le_ctrl_state_t  _bt_hci_le_ctrl_state;
#define HCI_INIT_LE_CTRL_STATE	\
	_phci_ctrl->le_ctrl_state = &_bt_hci_le_ctrl_state;	\
	_bt_hci_le_init = &bt_hci_le_init;
#else
#define HCI_SIZEOF_LE_CONN_STATES	0
#define HCI_DECLARE_LE_CONN_STATES
#define HCI_INIT_LE_CONN_STATES	\
	_hci_connections[i].le_conn_state = NULL;
#define HCI_SIZEOF_LE_CTRL_STATE	0
#define HCI_DECLARE_LE_CTRL_STATE
#define HCI_INIT_LE_CTRL_STATE	\
	_bt_hci_le_init = NULL;
#endif

#if HCI_MAX_CMD_PARAM_LEN < 32 || HCI_MAX_CMD_PARAM_LEN > 248
#error "HCI_MAX_CMD_PARAM_LEN >= 32 && HCI_MAX_CMD_PARAM_LEN <= 248 required"
#endif

/**
* \brief HCI_ENABLE_CTRL_TO_HOST_FLOW_CONTROL.
* \ingroup btconfig
*
* \details Enables controller to host flow control on ACL links.
*/
#ifdef HCI_ENABLE_CTRL_TO_HOST_FLOW_CONTROL
#undef HCI_ENABLE_CTRL_TO_HOST_FLOW_CONTROL
#define HCI_ENABLE_CTRL_TO_HOST_FLOW_CONTROL     BT_TRUE
#else
#define HCI_ENABLE_CTRL_TO_HOST_FLOW_CONTROL	 BT_FALSE
#endif

#ifndef HCI_MAX_CONNECT_ATTEMPTS
	#define HCI_MAX_CONNECT_ATTEMPTS    4
#endif

#include "cdbt/hci/hci_config_event_handlers.h"

#define HCI_ALLOCATE_BUFFERS_VARS()	\
	bt_buffer_header_t  _hci_cmd_buffer_headers[HCI_MAX_CMD_BUFFERS];	\
	bt_hci_command_t    _hci_cmd_buffers[HCI_MAX_CMD_BUFFERS];	\
	const bt_byte       _hci_max_cmd_buffers = HCI_MAX_CMD_BUFFERS; \
	bt_byte             _hci_cmd_param_buffers[HCI_MAX_CMD_BUFFERS * HCI_MAX_CMD_PARAM_LEN];	\
	const bt_byte       _hci_max_cmd_param_len = HCI_MAX_CMD_PARAM_LEN;	\
	\
	bt_buffer_header_t  _hci_send_data_buffer_headers[HCI_MAX_DATA_BUFFERS];	\
	bt_hci_data_t       _hci_send_data_buffers[HCI_MAX_DATA_BUFFERS];	\
	const bt_byte       _hci_max_data_buffers = HCI_MAX_DATA_BUFFERS; \
	\
	bt_hci_conn_state_t _hci_connections[HCI_MAX_HCI_CONNECTIONS];	\
	const bt_byte       _hci_max_hci_connections = HCI_MAX_HCI_CONNECTIONS; \
	\
	bt_byte             _recv_buffer[HCI_RX_BUFFER_LEN];	\
	bt_byte             _send_buffer[HCI_TX_BUFFER_LEN];	\
	const bt_uint       _hci_rcv_buffer_len = HCI_RX_BUFFER_LEN;	\
	const bt_uint       _hci_tx_buffer_len = HCI_TX_BUFFER_LEN;	\
	\
	bt_byte             _conn_state_recv_buffers[(HCI_L2CAP_BUFFER_LEN) * (HCI_MAX_HCI_CONNECTIONS)];	\
	const bt_uint       _hci_l2cap_buffer_len = HCI_L2CAP_BUFFER_LEN;	\
	\
	const bt_bool       _hci_enable_ctrl_to_host_flow_control = HCI_ENABLE_CTRL_TO_HOST_FLOW_CONTROL;	\
	\
	const bt_byte       _hci_max_connect_attempts = HCI_MAX_CONNECT_ATTEMPTS;	\
	\
	const bt_bool       _hci_enable_sco = BT_ENABLE_SCO;	\
	void (*_hci_recv_sco_data_packet_fp)(bt_byte* pbuf);	\
	\
	void (*_bt_ssp_init)(void);	\
	void (*_bt_ssp_evt_handler)(bt_hci_event_t* evt);	\
	\
	void (*_bt_hci_le_init)(bt_hci_le_ctrl_state_t* le_ctrl_state);	\
	HCI_DECLARE_LE_CONN_STATES	\
	HCI_DECLARE_LE_CTRL_STATE	\
	\
	HCI_ALLOCATE_BUFFERS_RAM_SIZE_VAR	\

#define HCI_ALLOCATE_BUFFERS_FUNCTION()	\
	void _hci_allocate_buffers(void)	\
	{	\
		bt_byte i;	\
		_phci_ctrl->connections = _hci_connections;	\
		_zero_memory(_hci_connections, sizeof(bt_hci_conn_state_t) * HCI_MAX_HCI_CONNECTIONS);	\
		for (i = 0; i < HCI_MAX_HCI_CONNECTIONS; i++)	\
		{	\
			_hci_connections[i].recv_data = &_conn_state_recv_buffers[i * (HCI_L2CAP_BUFFER_LEN)];	\
			HCI_INIT_LE_CONN_STATES;	\
		}	\
		HCI_INIT_SCO_HANDLERS	\
		HCI_INIT_SSP_HANDLERS	\
		HCI_INIT_LE_CTRL_STATE	\
	} \

#define HCI_ALLOCATE_BUFFERS()	\
		HCI_ALLOCATE_BUFFERS_VARS()	\
		HCI_ALLOCATE_BUFFERS_FUNCTION()	\
		typedef int HCI_BUFFERS_ALLOCATED

#endif // __HCI_CONFIG_H
