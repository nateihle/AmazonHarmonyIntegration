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

#ifndef __HCI_CONFIG_EVENT_HANDLERS_H
#define __HCI_CONFIG_EVENT_HANDLERS_H

#ifdef BT_ENABLE_SSP
#define BT_SSP_EVT_HANDLER    bt_ssp_evt_handler
#else
#define BT_SSP_EVT_HANDLER    bt_hci_evt_default_handler
#endif

#ifdef BT_ENABLE_BLE
#define BT_LE_EVT_HANDLER     _bt_le_evt_handler
#else
#define BT_LE_EVT_HANDLER     bt_hci_evt_default_handler
#endif

#if BT_ENABLE_SCO == BT_TRUE
#define BT_HCI_EVT_SYNCH_CONNECTION_COMPLETE_HANDLER    bt_hci_evt_synch_connection_complete_handler
#else
#define BT_HCI_EVT_SYNCH_CONNECTION_COMPLETE_HANDLER    bt_hci_evt_default_handler
#endif

#ifndef BT_BLE_SINGLE_MODE
#define BT_HCI_EVT_INQUIRY_COMPLETE_HANDLER             bt_hci_evt_inquiry_complete_handler
#define BT_HCI_EVT_INQUIRY_RESULT_HANDLER               bt_hci_evt_inquiry_result_handler
#define BT_HCI_EVT_CONNECTION_COMPLETE_HANDLER          bt_hci_evt_connection_complete_handler
#define BT_HCI_EVT_CONNECTION_REQUEST_HANDLER           bt_hci_evt_connection_request_handler
#define BT_HCI_EVT_AUTHENTICATION_COMPLETE_HANDLER      bt_hci_evt_authentication_complete_handler
#define BT_HCI_EVT_REMOTE_NAME_REQUEST_COMPLETE_HANDLER bt_hci_evt_remote_name_request_complete_handler
#define BT_HCI_EVT_ENCRYPTION_CHANGE_HANDLER            bt_hci_evt_encryption_change_handler
#define BT_HCI_EVT_ROLE_CHANGE_HANDLER                  bt_hci_evt_role_change_handler
#define BT_HCI_EVT_MODE_CHANGE_HANDLER                  bt_hci_evt_mode_change_handler
#define BT_HCI_EVT_PIN_CODE_REQUEST_HANDLER             bt_hci_evt_pin_code_request_handler
#define BT_HCI_EVT_LINK_KEY_REQUEST_HANDLER             bt_hci_evt_link_key_request_handler
#define BT_HCI_EVT_LINK_KEY_NOTIFICATION_HANDLER        bt_hci_evt_link_key_notification_handler
#define BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI_HANDLER     bt_hci_evt_inquiry_result_with_rssi_handler
#define BT_HCI_EVT_EXTENDED_INQUIRY_RESULT_HANDLER      bt_hci_evt_extended_inquiry_result_handler
#else
#define BT_HCI_EVT_INQUIRY_COMPLETE_HANDLER             bt_hci_evt_default_handler
#define BT_HCI_EVT_INQUIRY_RESULT_HANDLER               bt_hci_evt_default_handler
#define BT_HCI_EVT_CONNECTION_COMPLETE_HANDLER          bt_hci_evt_default_handler
#define BT_HCI_EVT_CONNECTION_REQUEST_HANDLER           bt_hci_evt_default_handler
#define BT_HCI_EVT_AUTHENTICATION_COMPLETE_HANDLER      bt_hci_evt_default_handler
#define BT_HCI_EVT_REMOTE_NAME_REQUEST_COMPLETE_HANDLER bt_hci_evt_default_handler
#ifndef BT_INCLUDE_SM
	#define BT_HCI_EVT_ENCRYPTION_CHANGE_HANDLER        bt_hci_evt_default_handler
#else
	#define BT_HCI_EVT_ENCRYPTION_CHANGE_HANDLER        bt_hci_evt_encryption_change_handler
#endif
#define BT_HCI_EVT_ROLE_CHANGE_HANDLER                  bt_hci_evt_default_handler
#define BT_HCI_EVT_MODE_CHANGE_HANDLER                  bt_hci_evt_default_handler
#define BT_HCI_EVT_PIN_CODE_REQUEST_HANDLER             bt_hci_evt_default_handler
#define BT_HCI_EVT_LINK_KEY_REQUEST_HANDLER             bt_hci_evt_default_handler
#define BT_HCI_EVT_LINK_KEY_NOTIFICATION_HANDLER        bt_hci_evt_default_handler
#define BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI_HANDLER     bt_hci_evt_default_handler
#define BT_HCI_EVT_EXTENDED_INQUIRY_RESULT_HANDLER      bt_hci_evt_default_handler
#endif

const bt_hci_event_handler_fp _hci_event_handlers[] = 
{
	&BT_HCI_EVT_INQUIRY_COMPLETE_HANDLER,
	&BT_HCI_EVT_INQUIRY_RESULT_HANDLER,
	&BT_HCI_EVT_CONNECTION_COMPLETE_HANDLER,
	&BT_HCI_EVT_CONNECTION_REQUEST_HANDLER,
	&bt_hci_evt_disconnection_complete_handler,
	&BT_HCI_EVT_AUTHENTICATION_COMPLETE_HANDLER,
	(bt_hci_event_handler_fp)&BT_HCI_EVT_REMOTE_NAME_REQUEST_COMPLETE_HANDLER,
	&BT_HCI_EVT_ENCRYPTION_CHANGE_HANDLER,
	&bt_hci_evt_default_handler, // bt_hci_evt_change_conn_link_complete_handler,
	&bt_hci_evt_default_handler, // bt_hci_evt_master_link_key_complete_handler,
	&bt_hci_evt_default_handler, // bt_hci_evt_read_rmt_sup_features_comp_handler,
	&bt_hci_evt_default_handler, // bt_hci_evt_read_rmt_version_info_comp_handler,
	&bt_hci_evt_default_handler, // bt_hci_evt_qos_setup_complete_handler,
	&bt_hci_evt_command_complete_handler,
	&bt_hci_evt_command_status_handler,
	&bt_hci_evt_hardware_error_handler,
	&bt_hci_evt_default_handler, // bt_hci_evt_flush_occured_handler,
	&BT_HCI_EVT_ROLE_CHANGE_HANDLER,
	&bt_hci_evt_num_of_completed_packets_handler,
	&BT_HCI_EVT_MODE_CHANGE_HANDLER,
	&bt_hci_evt_default_handler, // bt_hci_evt_return_link_keys_handler,
	&BT_HCI_EVT_PIN_CODE_REQUEST_HANDLER,
	&BT_HCI_EVT_LINK_KEY_REQUEST_HANDLER,
	&BT_HCI_EVT_LINK_KEY_NOTIFICATION_HANDLER,
	&bt_hci_evt_default_handler, // bt_hci_evt_loopback_command_handler,
	&bt_hci_evt_default_handler, // HCI_EVT_DATA_BUFFER_OVERFLOW
	&bt_hci_evt_default_handler, // HCI_EVT_MAX_SLOTS_CHANGE
	&bt_hci_evt_default_handler, // HCI_EVT_READ_CLOCK_OFFSET_COMPLETE
	&bt_hci_evt_default_handler, // HCI_EVT_CONN_PACKET_TYPE_CHANGED
	&bt_hci_evt_default_handler, // HCI_EVT_QOS_VIOLATION_HANDLER
	&bt_hci_evt_default_handler,
	&bt_hci_evt_default_handler, // HCI_EVT_PAGE_SCAN_REPET_MODE_CHANGE
	&bt_hci_evt_default_handler, // HCI_EVT_FLOW_SPECIFICATION_COMPLETE
	&BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI_HANDLER,
	&bt_hci_evt_default_handler, // HCI_EVT_READ_RMT_EXT_FEATURES_COMP
	&bt_hci_evt_default_handler,
	&bt_hci_evt_default_handler,
	&bt_hci_evt_default_handler,
	&bt_hci_evt_default_handler,
	&bt_hci_evt_default_handler,
	&bt_hci_evt_default_handler,
	&bt_hci_evt_default_handler,
	&bt_hci_evt_default_handler,
	&BT_HCI_EVT_SYNCH_CONNECTION_COMPLETE_HANDLER,
	&bt_hci_evt_default_handler, // HCI_EVT_SYNCH_CONNECTION_CHANGED
	&bt_hci_evt_default_handler, // HCI_EVT_SNIFF_SUBRATING
	&BT_HCI_EVT_EXTENDED_INQUIRY_RESULT_HANDLER,
	&bt_hci_evt_default_handler, // HCI_EVT_ENCRYPTION_KEY_REFRESH_COMPLETE
	&BT_SSP_EVT_HANDLER,         // HCI_EVT_IO_CAPABILITY_REQUEST
	&BT_SSP_EVT_HANDLER,         // HCI_EVT_IO_CAPABILITY_RESPONSE
	&BT_SSP_EVT_HANDLER,         // HCI_EVT_USER_CONFIRMATION_REQUEST
	&BT_SSP_EVT_HANDLER,         // HCI_EVT_USER_PASSKEY_REQUEST
	&BT_SSP_EVT_HANDLER,         // HCI_EVT_REMOTE_OOB_DATA_REQUEST
	&BT_SSP_EVT_HANDLER,         // HCI_EVT_SIMPLE_PAIRING_COMPLETE
	&bt_hci_evt_default_handler,
	&bt_hci_evt_default_handler, // HCI_EVT_LINK_SUPERVISION_TO_CHANGED
	&bt_hci_evt_default_handler, // HCI_EVT_ENHANCED_FLUSH_COMPLETE
	&bt_hci_evt_default_handler,
	&BT_SSP_EVT_HANDLER,         // HCI_EVT_USER_PASSKEY_NOTIFICATION
	&BT_SSP_EVT_HANDLER,         // HCI_EVT_KEYPRESS_NOTIFICATION
	&bt_hci_evt_default_handler, // HCI_EVT_RMT_HOST_SUPP_FEATURES_NTF
	&BT_LE_EVT_HANDLER,          // HCI_EVT_LE_META_EVENT
};

#endif // __HCI_CONFIG_EVENT_HANDLERS_H
