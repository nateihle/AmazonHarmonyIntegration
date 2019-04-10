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

#ifndef __L2CAP_CHANNEL_H
#define __L2CAP_CHANNEL_H

#ifdef __cplusplus
extern "C" {
#endif

#define CID_NULL         0x0000
#define CID_SIG          0x0001
#define CID_RECV         0x0002
#define CID_ATT          0x0004
#define CID_LE_SIG       0x0005
#define CID_SM           0x0006
#define CID_MAX          0xffff
#define CID_MAX_FIXED    0x003f

#define CMODE_BASIC		0
#define CMODE_RETR		1
#define CMODE_FLOW		2
#define CMODE_ERETR		3
#define CMODE_STRM		4

#define CTYPE_CO		0 // connection-oriented bt_l2cap_channel
#define CTYPE_CL		1 // connectionless bt_l2cap_channel

#define CSTATE_FREE					0x00
#define CSTATE_CLOSED				0x01
#define CSTATE_WAIT_CONNECT			0x02
#define CSTATE_WAIT_CONNECT_RSP		0x04
#define CSTATE_WAIT_CONFIG_RSP		0x08
#define CSTATE_WAIT_CONFIG_REQ		0x10
#define CSTATE_WAIT_CONFIG			(CSTATE_WAIT_CONFIG_RSP | CSTATE_WAIT_CONFIG_REQ)
#define CSTATE_WAIT_DISCONNECT		0x20
#define CSTATE_OPEN					0x40

#define L2CAP_CHANNEL_FLAG_SENDING              0x01
#define L2CAP_CHANNEL_FLAG_INCOMING             0x02
#define L2CAP_CHANNEL_FLAG_FORCE_HCI_DISCONNECT 0x04

#define L2CAP_ERETR_RECV_STATE_RECV			0
#define L2CAP_ERETR_RECV_STATE_REJ_SENT		1
#define L2CAP_ERETR_RECV_STATE_SREJ_SENT	2

#define L2CAP_ERETR_XMIT_STATE_XMIT			0
#define L2CAP_ERETR_XMIT_STATE_WAIT_ACK		1
#define L2CAP_ERETR_XMIT_STATE_WAIT_F		2

struct _bt_l2cap_mgr_s;
struct _bt_hci_conn_state_s;

typedef struct _bt_l2cap_channel_t bt_l2cap_channel_t;
typedef struct _bt_l2cap_channel_ext_t bt_l2cap_channel_ext_t;

typedef void (*bt_l2cap_read_data_callback_fp)(bt_l2cap_channel_t* pch, bt_byte_p pdata, bt_int len);
typedef void (*bt_l2cap_send_data_callback_fp)(bt_l2cap_channel_t* pch, bt_byte_p pdata, bt_int len, void* param);
typedef void (*bt_l2cap_state_changed_callback_fp)(bt_l2cap_channel_t* pch, bt_int new_state, void* param);

/*
typedef union _bt_l2cap_frame_control_s
{
	struct
	{
		bt_byte frame_type:1;
		bt_byte txSeq:6;
		bt_byte f:1;
		bt_byte reqSeq:6;
		bt_byte sar:2;
	} iframe;
	struct
	{
		bt_byte frame_type:1;
		bt_byte reserved:1;
		bt_byte s:2;
		bt_byte p:1;
		bt_byte reserved2:2;
		bt_byte f:1;
		bt_byte reqSeq:6;
		bt_byte reserved3:2;
	} sframe;
} bt_l2cap_frame_control_t;
*/

typedef struct _bt_l2cap_frame_desc_s
{
	bt_uint ctl;
	bt_byte data_type;
	union
	{
		bt_byte* raw;
		bt_packet_t* packet;
	} data;
	bt_int len;
	bt_byte tx_seq;
	bt_byte retry_count;
	bt_uint fcs;
	bt_l2cap_send_data_callback_fp callback;
	void* callback_param;
} bt_l2cap_frame_desc_t;

struct _bt_l2cap_channel_t 
{
	bt_id cid;
	struct _bt_l2cap_mgr_s *l2cap_mgr;
	bt_byte mode;
	bt_byte type;
	bt_byte state;
	bt_byte flags;
	bt_id cid_destination;
	bt_int psm;
	bt_int cfg_try_cnt;
	bt_l2cap_read_data_callback_fp _read_data_cb;	
	bt_l2cap_send_data_callback_fp _send_data_cb;	
	void* _send_data_cb_param;
	bt_l2cap_state_changed_callback_fp _state_changed_cb;
	void* _state_changed_param;
	struct _bt_hci_conn_state_s *hci_conn;
	bt_l2cap_packet_t tx_packet;

	// configuration
	bt_uint response_config_options;
	bt_uint response_mtu;
	bt_int response_flash_timeout;
	bt_l2cap_option_rfc_t response_option_rfc;
	bt_byte response_option_unknown_type;
	bt_l2cap_option_unknown_t response_option_unknown;

	bt_uint request_config_options;
	bt_int request_mtu;
	bt_int request_flash_timeout;
	bt_l2cap_option_rfc_t request_option_rfc;

	bt_byte connect_cmd_id;

	bt_byte signal_command;
	bt_signal_t signal;

	bt_l2cap_channel_ext_t* ext;
};

struct _bt_l2cap_channel_ext_t
{
	bt_bool check_fcs;
	bt_byte max_transmit;
	bt_byte next_tx_seq;
	bt_byte expected_ack_seq;
	//bt_byte req_seq;
	bt_byte expected_tx_seq;
	//bt_byte buffer_seq;
	bt_bool remote_busy;
	bt_bool local_busy;
	bt_l2cap_frame_desc_t unacked_frame;
	bt_l2cap_frame_desc_t pending_frame;
	bt_l2cap_frame_desc_t s_frame;
	//bt_byte srej_list[RFC_ERETR_TXWINDOW];
	bt_byte retry_count;
	bt_bool rnr_sent;
	bt_bool rej_actioned;
	//bt_bool srej_actioned;
	//bt_byte srej_save_req_seq;
	//bt_bool send_srej;
	//bt_byte buffer_seq_srej;
	bt_byte frames_sent;
	bt_ulong retr_timer_start_time;
	bt_ulong monitor_timer_start_time;

	bt_byte recv_state;
	bt_byte xmit_state;

	bt_l2cap_packet_t tx_s_packet;
};

bt_bool bt_l2cap_send_cmd(struct _bt_l2cap_mgr_s *mgr, struct _bt_hci_conn_state_s *conn, bt_l2cap_cmd_header_p cmd, pf_l2cap_cmd_callback cb);

bt_bool bt_l2cap_read_data(bt_l2cap_channel_t* channel, bt_l2cap_read_data_callback_fp callback);

/**
 * \brief Send data over an L2CAP channel
 * \ingroup l2cap
 *
 * \details This function sends data over the specified L2CAP channel.
 *
 * \param channel The L2CAP channel to send data over.
 * \param data The pointer to the data.
 * \param len The length of the data.
 * \param callback The callback function that is called when sending the data has been completed.
 *
 * \return
 *        \li \c TRUE if the function succeeds.
 *        \li \c FALSE otherwise. The callback function is not called in this case. 
*/
bt_bool bt_l2cap_send_data(bt_l2cap_channel_t* channel, const bt_byte_p data, bt_int len, 
	bt_l2cap_send_data_callback_fp callback, void* cb_param);

bt_bool bt_l2cap_send_smart_data(bt_l2cap_channel_t* pch, bt_packet_t* packet, bt_int len, 
	bt_l2cap_send_data_callback_fp cb, void* cb_param);

#ifdef __cplusplus
}
#endif

#endif // __L2CAP_CHANNEL_H

