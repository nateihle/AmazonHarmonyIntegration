/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __HCI_LE_H
#define __HCI_LE_H

#ifdef __cplusplus
extern "C" {
#endif

#define HCI_LE_DISABLED                 0
#define HCI_LE_ENABLED                  1

#define HCI_LE_SIMULTANEOUS_DISABLED    0
#define HCI_LE_SIMULTANEOUS_ENABLED     1

#define HCI_LE_AVERTISING_DISABLED      0
#define HCI_LE_AVERTISING_ENABLED       1

#define HCI_LE_DUPLICATE_FILTERING_DISABLED      0
#define HCI_LE_DUPLICATE_FILTERING_ENABLED       1

#define HCI_LE_SCAN_DISABLED      0
#define HCI_LE_SCAN_ENABLED       1

#define HCI_LE_SCAN_TYPE_PASSIVE  0
#define HCI_LE_SCAN_TYPE_ACTIVE   1

#define HCI_LE_SCAN_FILTER_POLICY_ALL           0
#define HCI_LE_SCAN_FILTER_POLICY_WHITE_LIST    1

#define HCI_LE_EVT_CONNECTION_COMPLETE                   0x01
#define HCI_LE_EVT_ADVERTISING_REPORT                    0x02
#define HCI_LE_EVT_CONNECTION_UPDATE_COMPLETE            0x03
#define HCI_LE_EVT_READ_REMOTE_USED_FEATURES_COMPLETE    0x04
#define HCI_LE_EVT_LONG_TERM_KEY_REQUEST                 0x05

#define HCI_LE_ADDRESS_TYPE_PUBLIC                  0
#define HCI_LE_ADDRESS_TYPE_RANDOM                  1

#define HCI_LE_ADV_TYPE_CONN_UNDIRECT               0x00 // Connectable undirected advertising (ADV_IND) (default)
#define HCI_LE_ADV_TYPE_DIRECT_HIGH                 0x01 // Connectable high duty cycle directed advertising (ADV_DIRECT_IND, high duty cycle)
#define HCI_LE_ADV_TYPE_SCAN                        0x02 // Scannable undirected advertising (ADV_SCAN_IND)
#define HCI_LE_ADV_TYPE_NONCONN                     0x03 // Non connectable undirected advertising (ADV_NONCONN_IND)
#define HCI_LE_ADV_TYPE_DIRECT_LOW                  0x04 // Connectable low duty cycle directed advertising (ADV_DIRECT_IND, low duty cycle)

#define HCI_LE_ADV_CHANNEL_MAP_RESERVED             0x00
#define HCI_LE_ADV_CHANNEL_MAP_ENABLE_37            0x01
#define HCI_LE_ADV_CHANNEL_MAP_ENABLE_38            0x02
#define HCI_LE_ADV_CHANNEL_MAP_ENABLE_39            0x04
#define HCI_LE_ADV_CHANNEL_MAP_ENABLE_ALL           0x07

#define HCI_LE_FILTER_POLICY_NOT_USED                    0 // White list is not used to determine which advertiser to connect to.
                                                           // Peer_Address_Type and Peer_Address shall be used.
#define HCI_LE_FILTER_POLICY_WHITE_LIST                  1 // White list is used to determine which advertiser to connect to.
                                                           // Peer_Address_Type and Peer_Address shall be ignored

#define HCI_LE_SCAN_EVT_STARTED          0
#define HCI_LE_SCAN_EVT_CANCELLED        1
#define HCI_LE_SCAN_EVT_DEVICE_FOUND     2
#define HCI_LE_SCAN_EVT_START_FAILED     3
#define HCI_LE_SCAN_EVT_CANCEL_FAILED    4

#define HCI_LE_FLAG_SIMULTANEOUS_LE_BREDR    1
#define HCI_LE_FLAG_SHARED_ACL_BUFFERS       2

#define HCI_LE_ADVERTISING_FLAG_LIMITED_DISCOVERABLE_MODE           0x01
#define HCI_LE_ADVERTISING_FLAG_GENERAL_DISCOVERABLE_MODE           0x02
#define HCI_LE_ADVERTISING_FLAG_BREDR_NOT_SUPPORTED                 0x04
#define HCI_LE_ADVERTISING_FLAG_SIMULTANEOUS_LE_BREDR_CONTROLLER    0x08
#define HCI_LE_ADVERTISING_FLAG_SIMULTANEOUS_LE_BREDR_HOST          0x10

#define HCI_LE_RANDOM_ADDRESS_TYPE_STATIC           1
#define HCI_LE_RANDOM_ADDRESS_TYPE_NON_RESOLVABLE   2
#define HCI_LE_RANDOM_ADDRESS_TYPE_RESOLVABLE       3

typedef struct _bt_hci_le_advertising_report_t bt_hci_le_advertising_report_t;

typedef void (*bt_hci_le_scan_callback_fp)(bt_byte evt, bt_hci_le_advertising_report_t* report, void* param);
typedef void (*bt_le_evt_handler)(bt_hci_event_t* evt);

struct _bt_hci_conn_state_s;

typedef struct _bt_hci_le_conn_state_t
{
	bt_byte own_address_type; 
	bt_byte peer_address_type;
	bt_uint conn_interval;
	bt_uint conn_latency;
	bt_uint supervision_timeout;
	bt_byte master_clock_accuracy;

} bt_hci_le_conn_state_t;

typedef struct _bt_hci_le_connect_parameters_t
{
	bt_uint scan_interval;
	bt_uint scan_window;
	bt_byte filter_policy;
	bt_byte own_address_type; 
	bt_uint conn_interval_min;
	bt_uint conn_interval_max;
	bt_uint conn_latency;
	bt_uint supervision_timeout;
	bt_uint min_ce_length;
	bt_uint max_ce_length;
} bt_hci_le_connect_parameters_t;

struct _bt_security_mgr_t;

typedef struct _bt_hci_le_ctrl_state_t
{
	bt_byte flags;
	bt_byte total_num_acl_data_packets;
	bt_int acl_data_packet_len;
	struct _bt_security_mgr_t* sm;
	bt_bdaddr_t bdaddr;
	bt_byte advertising_address_type;
	bt_byte connecting_address_type;

	// default connect parameters
	bt_hci_le_connect_parameters_t def_connect_parameters;

	bt_le_evt_handler handler;
	bt_hci_le_scan_callback_fp scan_cb;
	void* scan_cb_param;

	// API calls
	bt_bool (*connect_ex)(
		bt_uint scan_interval, bt_uint scan_window,	bt_byte filter_policy, 
		bt_byte peer_address_type, bt_bdaddr_t* peer_address,	bt_byte own_address_type, 
		bt_uint conn_interval_min, bt_uint conn_interval_max, bt_uint conn_latency,
		bt_uint supervision_timeout, bt_uint min_ce_length, bt_uint max_ce_length,
		bt_uint acl_config,
		bt_hci_connect_callback_fp cb, void *param);
	bt_bool (*set_connect_parameters)(bt_hci_le_connect_parameters_t* params);
	bt_bool (*get_connect_parameters)(bt_hci_le_connect_parameters_t* params);

	void (*sm_long_term_key_request_handler)(bt_hci_event_t* evt);
	bt_uint (*sm_get_session_state)(bt_hci_conn_state_t* conn);
	bt_byte (*sm_get_session_key_size)(bt_hci_conn_state_t* conn);
	bt_bool (*sm_authenticate)(bt_hci_conn_state_t* conn, bt_byte auth_req);



} bt_hci_le_ctrl_state_t;

struct _bt_hci_le_advertising_report_t
{
	bt_byte event_type;
	bt_byte address_type;
	bt_bdaddr_t bdaddr;
	bt_byte data_len;
	bt_byte* data;
	bt_byte rssi;
};

typedef struct _bt_hci_le_evt_connection_updated_t
{
	bt_byte status;
	struct _bt_hci_conn_state_s* conn;
} bt_hci_le_evt_connection_updated_t;

typedef struct _bt_hci_le_evt_read_support_params_t
{
	bt_byte supported;
	bt_byte simultaneous_supported;
} bt_hci_le_evt_read_support_params_t;

typedef struct _bt_hci_le_evt_read_remote_used_features_completed_t
{
	bt_byte status;
	struct _bt_hci_conn_state_s* conn;
	bt_byte features[8];
} bt_hci_le_evt_read_remote_used_features_completed_t;

bt_bool bt_hci_le_supported(void);

bt_bool bt_hci_le_enable(bt_bool enabled, bt_bool simultaneous_enabled, bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_read_support(bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_write_support(bt_bool enabled, bt_bool simultaneous_enabled, bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_connect_ex(
	bt_uint scan_interval, bt_uint scan_window,	bt_byte filter_policy, 
	bt_byte peer_address_type, bt_bdaddr_t* peer_address,	bt_byte own_address_type, 
	bt_uint conn_interval_min, bt_uint conn_interval_max, bt_uint conn_latency,
	bt_uint supervision_timeout, bt_uint min_ce_length, bt_uint max_ce_length,
	bt_uint acl_config,
	bt_hci_connect_callback_fp cb, void *param);

bt_bool bt_hci_le_update_connection(
	struct _bt_hci_conn_state_s* pconn,
	bt_uint conn_interval_min, bt_uint conn_interval_max, bt_uint conn_latency,
	bt_uint supervision_timeout, bt_uint min_ce_length, bt_uint max_ce_length,
	bt_hci_cmd_callback_fp cb, void *param);

bt_bool bt_hci_le_cancel_connect_ex(bt_hci_cmd_callback_fp cb, void *cb_param);

#define bt_hci_le_cancel_connect(cb)	bt_hci_le_cancel_connect_ex(cb, NULL)

bt_bool bt_hci_le_set_host_channel_classification(const bt_byte* channel_map, bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_read_channel_map(struct _bt_hci_conn_state_s* pconn, bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_read_remote_used_features(struct _bt_hci_conn_state_s* pconn, bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_set_advertising_parameters(
	bt_uint adv_interval_min, bt_uint adv_interval_max,
	bt_byte adv_type, bt_byte own_address_type,
	bt_byte direct_address_type, bt_bdaddr_t* direct_address,
	bt_byte adv_channel_map, bt_byte adv_filter_policy,
	bt_hci_cmd_callback_fp cb);

#define bt_hci_le_set_adevrtising_enable(adv_enable, cb)    bt_hci_le_set_adevrtising_enable_ex(adv_enable, cb, NULL)

bt_bool bt_hci_le_set_adevrtising_enable_ex(bt_byte adv_enable, bt_hci_cmd_callback_fp cb, void* cb_param);

bt_bool bt_hci_le_set_scan_parameters(
	bt_byte scan_type, bt_uint scan_interval, bt_uint scan_window,
	bt_byte own_address_type, bt_byte filter_policy,
	bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_set_scan_enable(
	bt_byte scan_enable, bt_byte filter_duplicates,
	bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_find_devices(bt_byte filter_duplicates, bt_hci_le_scan_callback_fp cb, void* param);

bt_bool bt_hci_le_cancel_find_devices(void);

bt_bool bt_hci_le_encrypt(
	const bt_byte* key, bt_byte key_len,
	const bt_byte* data, bt_byte data_len,
	bt_hci_cmd_callback_fp cb, void* param);

bt_bool bt_hci_le_rand(bt_hci_cmd_callback_fp cb, void* param);

bt_bool bt_hci_le_start_encryption(
	struct _bt_hci_conn_state_s* pconn,
	const bt_byte* random_number,
	bt_uint encrypted_diversifier,
	const bt_byte* long_term_key,
	bt_hci_cmd_callback_fp cb, void* param);


bt_bool bt_hci_le_ltk_reply(
	struct _bt_hci_conn_state_s* pconn,
	const bt_byte* long_term_key,
	bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_ltk_negative_reply(struct _bt_hci_conn_state_s* pconn, bt_hci_cmd_callback_fp cb);


bt_bool bt_hci_le_receiver_test(bt_byte frequency, bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_transmitter_test(
	bt_byte frequency, bt_byte test_data_len, bt_byte packet_payload,
	bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_test_end(bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_read_white_list_size(bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_clear_white_list(bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_add_device_to_white_list(
	bt_byte address_type, bt_bdaddr_t* address,
	bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_remove_device_from_white_list(
	bt_byte address_type, bt_bdaddr_t* address,
	bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_set_connect_parameters(bt_hci_le_connect_parameters_t* params);

bt_bool bt_hci_le_get_connect_parameters(bt_hci_le_connect_parameters_t* params);

bt_hci_command_t* bt_hci_le_allocate_set_advertising_data_command(bt_hci_cmd_callback_fp cb);

bt_hci_command_t* bt_hci_le_allocate_set_scan_response_data_command(bt_hci_cmd_callback_fp cb);

bt_bool bt_hci_le_advertising_add_local_name(const char* local_name, bt_hci_command_t* pcmd);

bt_bool bt_hci_le_advertising_uuid16_add(
	bt_byte data_type, const bt_uint* uuid_list, bt_byte uuid_list_size, bt_hci_command_p pcmd);

bt_bool bt_hci_le_advertising_uuid32_add(
	bt_byte data_type, const bt_uuid32* uuid_list, bt_byte uuid_list_size, bt_hci_command_p pcmd);

bt_bool bt_hci_le_advertising_uuid128_add(
	bt_byte data_type, const bt_uuid_t* uuid_list, bt_byte uuid_list_size, bt_hci_command_p pcmd);

bt_bool bt_hci_le_advertising_vendor_add(
	bt_uint vendor_id, bt_byte* data, bt_byte data_len, bt_hci_command_p pcmd);

bt_bool bt_hci_le_advertising_tx_power_level_add(bt_byte tx_power_level, bt_hci_command_p pcmd);

bt_bool bt_hci_le_advertising_device_id_add(
	bt_uint vendor_id_source, bt_uint vendor_id, bt_uint product_id, bt_uint version, bt_hci_command_p pcmd);

bt_bool bt_hci_le_advertising_flags_add(bt_byte flags, bt_hci_command_p pcmd);

bt_bool bt_hci_le_advertising_add(bt_byte data_type, const bt_byte* data, bt_byte length, bt_hci_command_p pcmd);

bt_bool bt_hci_le_ibeacon_add(const bt_uuid_t* uuid, bt_uint major, bt_uint minor, bt_byte tx_power_level, bt_hci_command_p pcmd);

bt_bool bt_hci_le_advertising_get_local_name(const bt_byte* data, bt_byte data_len, const bt_byte** local_name, bt_byte* name_len, bt_byte* name_type);

bt_bool bt_hci_le_set_random_address(bt_bdaddr_t* bdaddr, bt_hci_cmd_callback_fp cb);

#ifdef __cplusplus
}
#endif

#endif // HCI_LE_H

