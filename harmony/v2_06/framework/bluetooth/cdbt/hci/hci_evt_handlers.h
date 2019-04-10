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

#ifndef __HCI_EVENT_HANDLERS_H
#define __HCI_EVENT_HANDLERS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _bt_hci_evt_mode_change_t
{
	bt_hci_conn_state_t* pconn;
	bt_byte mode;
	bt_int interval;
} bt_hci_evt_mode_change_t;


typedef struct _bt_hci_evt_role_change_t
{
	bt_hci_conn_state_t* pconn;
	bt_byte role;
} bt_hci_evt_role_change_t;

typedef struct _bt_hci_evt_connection_request_t
{
	bt_bdaddr_t bdaddr;
	bt_long dev_class;
	bt_byte link_type;
} bt_hci_evt_connection_request_t;

typedef struct _bt_hci_evt_connection_complete_t
{
	bt_byte status;
	bt_hci_conn_state_t* pconn;
	bt_bool incomming;
} bt_hci_evt_connection_complete_t;

typedef struct _bt_hci_evt_disconnection_complete_t
{
	bt_byte status;
	bt_hci_conn_state_t* pconn;
	bt_byte reason;
} bt_hci_evt_disconnection_complete_t;

typedef struct _bt_hci_evt_authentication_complete_t
{
	bt_byte status;
	bt_hci_conn_state_t* pconn;
} bt_hci_evt_authentication_complete_t;

typedef struct _bt_hci_evt_encryption_change_s
{
	bt_byte status;
	bt_hci_conn_state_t* pconn;
	bt_byte enabled;
} bt_hci_evt_encryption_change_t;

typedef struct _bt_hci_evt_command_complete_t
{
	bt_byte status;
	bt_int opcode;
	bt_hci_command_t* cmd;
} bt_hci_evt_command_complete_t;

typedef struct _bt_hci_evt_command_status_t
{
	bt_byte status;
	bt_int opcode;
	bt_hci_command_t* cmd;
} bt_hci_evt_command_status_t;

typedef void (*bt_hci_event_handler_fp)(bt_hci_event_p);
typedef void (*bt_hci_event_handler_ex_fp)(bt_hci_event_p, bt_uint params_len);

void bt_hci_set_vendor_specific_event_handler(bt_hci_event_handler_fp handler);

void bt_hci_evt_inquiry_complete_handler(bt_hci_event_p pevt);
void bt_hci_evt_inquiry_result_handler(bt_hci_event_p pevt);
void bt_hci_evt_extended_inquiry_result_handler(bt_hci_event_p pevt);
void bt_hci_evt_inquiry_result_with_rssi_handler(bt_hci_event_p pevt);
void bt_hci_evt_connection_complete_handler(bt_hci_event_p pevt);
void bt_hci_evt_connection_request_handler(bt_hci_event_p pevt);
void bt_hci_evt_disconnection_complete_handler(bt_hci_event_p pevt);
void bt_hci_evt_authentication_complete_handler(bt_hci_event_p pevt);
void bt_hci_evt_remote_name_request_complete_handler(bt_hci_event_p pevt, bt_uint params_len);
void bt_hci_evt_encryption_change_handler(bt_hci_event_p pevt);
void bt_hci_evt_change_conn_link_complete_handler(bt_hci_event_p pevt);
void bt_hci_evt_master_link_key_complete_handler(bt_hci_event_p pevt);
void bt_hci_evt_read_rmt_sup_features_comp_handler(bt_hci_event_p pevt);
void bt_hci_evt_read_rmt_version_info_comp_handler(bt_hci_event_p pevt);
void bt_hci_evt_qos_setup_complete_handler(bt_hci_event_p pevt);
void bt_hci_evt_command_complete_handler(bt_hci_event_p pevt);
void bt_hci_evt_command_status_handler(bt_hci_event_p pevt);
void bt_hci_evt_hardware_error_handler(bt_hci_event_p pevt);
void bt_hci_evt_flush_occured_handler(bt_hci_event_p pevt);
void bt_hci_evt_role_change_handler(bt_hci_event_p pevt);
void bt_hci_evt_num_of_completed_packets_handler(bt_hci_event_p pevt);
void bt_hci_evt_mode_change_handler(bt_hci_event_p pevt);
void bt_hci_evt_return_link_keys_handler(bt_hci_event_p pevt);
void bt_hci_evt_pin_code_request_handler(bt_hci_event_p pevt);
void bt_hci_evt_link_key_request_handler(bt_hci_event_p pevt);
void bt_hci_evt_link_key_notification_handler(bt_hci_event_p pevt);
void bt_hci_evt_loopback_command_handler(bt_hci_event_p pevt);
void bt_hci_evt_data_buffer_overflow_handler(bt_hci_event_p pevt);
void bt_hci_evt_max_slots_change_handler(bt_hci_event_p pevt);
void bt_hci_evt_read_clock_offset_complete_handler(bt_hci_event_p pevt);
void bt_hci_evt_conn_packet_type_changed_handler(bt_hci_event_p pevt);
void bt_hci_evt_qos_violation_handler(bt_hci_event_p pevt);
void bt_hci_evt_page_scan_repet_mode_change_handler(bt_hci_event_p pevt);
void bt_hci_evt_flow_specification_complete_handler(bt_hci_event_p pevt);
void bt_hci_evt_inquiry_result_with_rssi_handler(bt_hci_event_p pevt);
void bt_hci_evt_read_rmt_ext_features_comp_handler(bt_hci_event_p pevt);
void bt_hci_evt_synch_connection_complete_handler(bt_hci_event_p pevt);
void bt_hci_evt_synch_connection_changed_handler(bt_hci_event_p pevt);
void bt_hci_evt_default_handler(bt_hci_event_p pevt);
void bt_hci_evt_hardware_error_handler(bt_hci_event_p pevt);

#ifdef __cplusplus
}
#endif

#endif // __HCI_EVENT_HANDLERS_H
