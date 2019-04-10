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

#ifndef __CONN_STATE_H
#define __CONN_STATE_H

#include "cdbt/bt/bt_signal.h"

#define HCI_CONN_TYPE_SCO		0
#define HCI_CONN_TYPE_ACL		1
#define HCI_CONN_TYPE_ESCO		2

#define HCI_CONN_STATE_CLOSED			0
#define HCI_CONN_STATE_AUTHENTICATING	1
#define HCI_CONN_STATE_OPEN				2

#define HCI_CONN_ROLE_MASTER	0
#define HCI_CONN_ROLE_SLAVE		1

#define HCI_LINK_TYPE_BD_EDR	0
#define HCI_LINK_TYPE_LE		1

typedef void (*bt_hci_sco_read_data_callback_fp)(bt_hci_conn_state_t* pconn, bt_byte_p data, bt_byte len);

typedef struct _bt_hci_listener_t bt_hci_listener_t;
struct _bt_hci_listener_t
{
	bt_hci_listener_t* next_listener;

	bt_byte event_id;
	union
	{
		void* ptr;
		bt_hci_disconnect_callback_fp disconnect;
	} callback;
	void* callback_param;
};

typedef union _bt_hci_event_e
{
	bt_hci_evt_disconnection_complete_t disconnect;
} bt_hci_event_e;

struct _bt_hci_ctrl_state_s;
struct _bt_hci_conn_state_s
{
	bt_hci_hconn_t hconn;
	bt_bdaddr_t bdaddr_remote;
	bt_byte type;
	bt_byte state;
	bt_byte mode;
	bt_byte role;
	bt_long l2cap_channel_close_time;
	bt_uint acl_config;
	bt_byte link_type;
	bt_signal_t signal;
	bt_int  outstanding_acl_packet_count;
	bt_hci_le_conn_state_t* le_conn_state;
	bt_ulong data_rx_time;
	bt_ulong data_tx_time;
	bt_byte idle;

	// queue for sending outgoing data packets
	bt_queue_element_t* queue;

	// buffer to receive incoming packets
	bt_byte* recv_data;
	bt_int recv_data_len;
	bt_int recv_data_pos;

//	bt_hci_disconnect_callback_fp hci_disconnect_callback;
//	void *hci_disconnect_param;

	bt_hci_sco_read_data_callback_fp sco_read_data_callback;
	void* sco_read_data_param;

	bt_hci_listener_t* listeners;
};

bt_hci_conn_state_p hci_get_conn_state(struct _bt_hci_ctrl_state_s *pctrl, bt_hci_hconn_t h);
bt_hci_conn_state_p hci_get_conn_state_by_bdaddr(struct _bt_hci_ctrl_state_s *pctrl, const bt_bdaddr_t* pbdaddr_remote, bt_byte conn_type, bt_byte link_type);
bt_hci_conn_state_p hci_allocate_conn_state(struct _bt_hci_ctrl_state_s *pctrl);

bt_bool bt_hci_register_listener(bt_hci_conn_state_t* pconn, bt_hci_listener_t* listener);
void bt_hci_unregister_listener(bt_hci_conn_state_t* pconn, bt_hci_listener_t* listener);


#endif //__CONN_STATE_H
