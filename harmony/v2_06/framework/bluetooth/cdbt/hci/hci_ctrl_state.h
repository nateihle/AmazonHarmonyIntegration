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

#ifndef __HCI_CTRL_STATE_H
#define __HCI_CTRL_STATE_H

#include "cdbt/hci/hci_le.h"
#include "cdbt/ssp/ssp.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HCI_CTRL_STATE_CLOSED					0x00
#define HCI_CTRL_STATE_INIT						0x01
#define HCI_CTRL_STATE_READY					0x02
#define HCI_CTRL_STATE_SLEEP					0x04
#define HCI_CTRL_STATE_WAKING_UP				0x08
#define HCI_CTRL_STATE_DISCOVERABLE				0x10
#define HCI_CTRL_STATE_CONNECTABLE				0x20

#define HCI_CTRL_LISTENER_EVENT                 0
#define HCI_CTRL_LISTENER_ACL_DATA              1
#define HCI_CTRL_LISTENER_SCO_DATA              2

struct _bt_l2cap_mgr_s;
typedef void (*pf_l2cap_receive_callback)(struct _bt_l2cap_mgr_s*, bt_hci_conn_state_p, bt_byte_p, bt_int len);
typedef void (*bt_hci_cmd_listener_fp)(bt_uint event_id, bt_hci_command_t* cmd, void* cb_param);
typedef void (*bt_hci_data_listener_fp)(bt_hci_conn_state_t* connection, const bt_byte* data, bt_int len, bt_bool sent, void* cb_param);

typedef struct _bt_hci_ctrl_listener_t bt_hci_ctrl_listener_t;
struct _bt_hci_ctrl_listener_t
{
	bt_hci_ctrl_listener_t* next_listener;

	bt_byte listener_type;
	bt_uint event_id;
	union
	{
		void* ptr;
		bt_hci_event_listener_fp hci_event;
		bt_hci_cmd_listener_fp   cmd_event;
		bt_hci_data_listener_fp  data;
	} callback;
	void* callback_param;
};


typedef struct _bt_hci_ctrl_state_s 
{
	bt_byte state;
	bt_byte default_link_policy;
	bt_bdaddr_t bdaddr;
	bt_int num_hci_cmd_packets;
	bt_int total_num_acl_data_packets;
	bt_int total_num_sco_data_packets;
	bt_int aclDataPacketLen;
	bt_byte scoDataPacketLen;
	bt_hci_conn_state_t* connections;
	bt_ulong _discoverable_period;
	bt_ulong _connectable_period;
	bt_byte _last_cmd_status;
	bt_hci_le_ctrl_state_t* le_ctrl_state;
	bt_ulong event_mask_l;
	bt_ulong event_mask_h;
	bt_byte incoming_connection_role;
	bt_byte inquiry_response_tx_power_level;

	struct _bt_l2cap_mgr_s* l2cap_mgr;
	pf_l2cap_receive_callback l2cap_data_receive_callback;

	bt_hci_start_callback_fp _init_cb;
	void* _init_param;

	bt_hci_connect_callback_fp _connect_cb;
	void* _connect_param;
	bt_uint _connect_acl_config;

	bt_hci_connect_callback_fp _listen_cb;
	void* _listen_param;
	
	bt_ulong sco_tx_bandwidth;
	bt_ulong sco_rcv_bandwidth;
	bt_uint sco_max_latency;
	bt_uint sco_content_format;
	bt_byte sco_retransmission_effort;
	bt_uint sco_packet_type;

	bt_hci_connect_callback_fp sco_listen_cb;
	void* _sco_listen_param;

	bt_hci_connect_callback_fp sco_connect_cb;
	void* _sco_connect_param;

	bt_hci_inquiry_callback_fp inquiry_cb;
    bt_hci_request_remote_name_callback_fp remote_name_cb;

	bt_byte inquiry_max_responses;
	bt_byte inquiry_length;

	pf_hci_sleep_callback sleep_cb;
	pf_hci_wakeup_callback wakeup_cb;

	bt_hci_ctrl_listener_t* listeners;

} bt_hci_ctrl_state_t;

/** 
* listener is triggered by the following events:
*   HCI_EVT_AUTHENTICATION_COMPLETE
*   HCI_EVT_CONNECTION_COMPLETE
*   HCI_EVT_DISCONNECTION_COMPLETE
*   HCI_EVT_ROLE_CHANGE
*   HCI_EVT_MODE_CHANGE
*/
void bt_hci_set_event_listener(bt_hci_event_listener_fp callback);

bt_bool bt_hci_ctrl_register_listener(bt_hci_ctrl_listener_t* listener);
bt_bool bt_hci_ctrl_register_data_listener(bt_hci_ctrl_listener_t* listener, bt_byte listener_type);
void bt_hci_ctrl_unregister_listener(bt_hci_ctrl_listener_t* listener);

#ifdef __cplusplus
}
#endif

#endif // __HCI_CTRL_STATE_H
