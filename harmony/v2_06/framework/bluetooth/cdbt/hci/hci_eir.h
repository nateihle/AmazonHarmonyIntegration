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

#ifndef __HCI_EIR_H
#define __HCI_EIR_H

#define HCI_EIR_TYPE_FLAGS										0x01
#define HCI_EIR_TYPE_UUID16_LIST_MORE_AVAILABLE					0x02
#define HCI_EIR_TYPE_UUID16_LIST_COMPLETE						0x03
#define HCI_EIR_TYPE_UUID32_LIST_MORE_AVAILABLE					0x04
#define HCI_EIR_TYPE_UUID32_LIST_COMPLETE						0x05
#define HCI_EIR_TYPE_UUID128_LIST_MORE_AVAILABLE				0x06
#define HCI_EIR_TYPE_UUID128_LIST_COMPLETE						0x07
#define HCI_EIR_TYPE_LOCAL_NAME_SHORTENED						0x08
#define HCI_EIR_TYPE_LOCAL_NAME_COMPLETE						0x09
#define HCI_EIR_TYPE_TX_POWER_LEVEL								0x0a
#define HCI_EIR_TYPE_OOB_COD									0x0d
#define HCI_EIR_TYPE_OOB_HASH									0x0e
#define HCI_EIR_TYPE_OOB_RANDOMIZER								0x0f
#define HCI_EIR_TYPE_DEVICE_ID									0x10
#define HCI_EIR_TYPE_SM_TK_VALUE								0x10
#define HCI_EIR_TYPE_SM_OOB_FLAGS								0x11
#define HCI_EIR_TYPE_SLAVE_CONN_INTERVAL_RANGE					0x12
#define HCI_EIR_TYPE_SOLICITATION_UUID16_LIST					0x14
#define HCI_EIR_TYPE_SOLICITATION_UUID32_LIST					0x1F
#define HCI_EIR_TYPE_SOLICITATION_UUID128_LIST					0x15
#define HCI_EIR_TYPE_SERVICE_DATA								0x16
#define HCI_EIR_TYPE_SERVICE_DATA_UUID16						0x16
#define HCI_EIR_TYPE_SERVICE_DATA_UUID32						0x20
#define HCI_EIR_TYPE_SERVICE_DATA_UUID128						0x21
#define HCI_EIR_TYPE_LE_SECURE_CONNECTIONS_CONFIRMATION_VALUE	0x22
#define HCI_EIR_TYPE_LE_SECURE_CONNECTIONS_RANDOM_VALUE			0x23
#define HCI_EIR_TYPE_PUBLIC_TARGET_ADDRESS						0x17
#define HCI_EIR_TYPE_RANDOM_TARGET_ADDRESS						0x18
#define HCI_EIR_TYPE_APPEARANCE									0x19
#define HCI_EIR_TYPE_ADVERTISING_INTERVAL						0x1A
#define HCI_EIR_TYPE_LE_BLUETOOTH_DEVICE_ADDRESS				0x1B
#define HCI_EIR_TYPE_LE_ROLE									0x1C
#define HCI_EIR_TYPE_SIMPLE_PAIRING_HASH_C_256					0x1D
#define HCI_EIR_TYPE_SIMPLE_PAIRING_RANDOMIZER_R_256			0x1E
#define HCI_EIR_TYPE_3D_Information_Data						0x3D

#define HCI_EIR_TYPE_MANUFACTURER_SPECIFIC						0xFF

#define HCI_EIR_FEC_NOT_REQUIRED	0
#define HCI_EIR_FEC_REQUIRED		1

bt_hci_command_p bt_hci_allocate_write_eir_command(bt_byte fec_required);

bt_bool bt_hci_param_eir_local_name_add(const char* local_name, bt_hci_command_p pcmd);

bt_bool bt_hci_param_eir_uuid16_add(
	bt_byte data_type, const bt_uint* uuid_list, bt_byte uuid_list_size, bt_hci_command_p pcmd);

bt_bool bt_hci_param_eir_uuid32_add(
	bt_byte data_type, const bt_uuid32* uuid_list, bt_byte uuid_list_size, bt_hci_command_p pcmd);

bt_bool bt_hci_param_eir_uuid128_add(
	bt_byte data_type, const bt_uuid_t* uuid_list, bt_byte uuid_list_size, bt_hci_command_p pcmd);

bt_bool bt_hci_param_eir_vendor_add(
	bt_uint vendor_id, bt_byte* data, bt_byte data_len, bt_hci_command_p pcmd);

bt_bool bt_hci_param_eir_add(bt_byte data_type, const bt_byte* data, bt_byte length, bt_hci_command_p pcmd);

bt_bool bt_hci_param_tx_power_level_add(bt_byte tx_power_level, bt_hci_command_p pcmd);

bt_bool bt_hci_param_eir_device_id_add(
	bt_uint vendor_id_source, bt_uint vendor_id, bt_uint product_id, bt_uint version, bt_hci_command_p pcmd);

bt_bool bt_hci_write_eir(bt_hci_command_p pcmd, bt_hci_cmd_callback_fp cb);

#endif // __HCI_EIR_H
