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

#ifndef __SSP_H
#define __SSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cdbt/hci/hci.h"

#ifndef SSP_MAX_MANAGERS
#define SSP_MAX_MANAGERS		1
#endif

typedef enum _SSP_MODE
{
	SSP_MODE_DISABLED = 0,
	SSP_MODE_ENABLED
} SSP_MODE;

typedef enum _SSP_IO_CAPABILITY
{
	SSP_IO_CAPABILITY_DISPLAY_ONLY = 0,
	SSP_IO_CAPABILITY_DISPLAY_YESNO,
	SSP_IO_CAPABILITY_KEYBOARD_ONLY,
	SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT

} SSP_IO_CAPABILITY;

typedef enum _SSP_OOB_DATA_PRESENT
{
	SSP_OOB_DATA_NOT_PRESENT = 0,
	SSP_OOB_DATA_REMOTE_DEVICE_DATA_PRESENT

} SSP_OOB_DATA_PRESENT;

typedef enum _SSP_AUTHENTICATION_REQUIREMENTS
{
	SSP_MITM_NOT_REQUIRED_NO_BONDING = 0,
	SSP_MITM_REQUIRED_NO_BONDING,
	SSP_MITM_NOT_REQUIRED_DEDICATED_BONDING,
	SSP_MITM_REQUIRED_DEDICATED_BONDING,
	SSP_MITM_NOT_REQUIRED_GENERAL_BONDING,
	SSP_MITM_REQUIRED_GENERAL_BONDING

} SSP_AUTHENTICATION_REQUIREMENTS;

typedef struct _bt_ssp_io_capability
{
	bt_bdaddr_t bdaddr_remote;
	SSP_IO_CAPABILITY io_capability;
	SSP_OOB_DATA_PRESENT oob_data_present;
	SSP_AUTHENTICATION_REQUIREMENTS authentication_requirements;

} bt_ssp_io_capability;

typedef enum _SSP_KEYPRESS_NOTIFICATION_TYPE
{
	PASSKEY_ENTRY_STYARTED,
	PASSKEY_DIGIT_ENTERED,
	PASSKEY_DIGIT_ERASED,
	PASSKEY_CLEARED,
	PASSKEY_ENTRY_COMPLETED
} SSP_KEYPRESS_NOTIFICATION_TYPE;

// Keypress Notification Event

typedef struct _bt_ssp_keypress_notification
{
	bt_bdaddr_t bdaddr_remote;
	SSP_KEYPRESS_NOTIFICATION_TYPE type;
} bt_ssp_keypress_notification;


// User Confirmation Request Event

typedef struct _bt_ssp_user_confirmation_request
{
	bt_bdaddr_t bdaddr_remote;
	bt_ulong numeric_value;
} bt_ssp_user_confirmation_request;

// User Passkey Request Event

typedef struct _bt_ssp_user_passkey_request
{
	bt_bdaddr_t bdaddr_remote;
	bt_ulong passkey;
} bt_ssp_user_passkey_request;

// User Passkey Notification Event

typedef struct _bt_ssp_user_passkey_notification
{
	bt_bdaddr_t bdaddr_remote;
	bt_ulong passkey;
} bt_ssp_user_passkey_notification;

// Remote OOB Data Request Event

#define OOB_DATA_HASH_LENGTH			16
#define OOB_DATA_RANDOMIZER_LENGTH		16

typedef struct _bt_ssp_oob_data
{
	bt_bdaddr_t bdaddr;
	bt_byte hash[OOB_DATA_HASH_LENGTH];
	bt_byte randomizer[OOB_DATA_RANDOMIZER_LENGTH];
} bt_ssp_oob_data;

typedef void (*bt_spp_read_local_oob_data_callback_fp)(bt_byte status, bt_ssp_oob_data* oob_data, void* init_param);

// Simple Pairing Complete Event

typedef struct _bt_ssp_simple_pairing_complete
{
	bt_byte status;
	bt_bdaddr_t bdaddr_remote;
} bt_ssp_simple_pairing_complete;

#include "cdbt/ssp/ssp_event.h"

bt_bool bt_ssp_set_mode(SSP_MODE mode, bt_hci_cmd_callback_fp callback);

bt_bool bt_ssp_send_keypress_notification(bt_bdaddr_p bdaddr_remote, SSP_KEYPRESS_NOTIFICATION_TYPE keypress_type);

bt_bool bt_ssp_read_local_oob_data(bt_spp_read_local_oob_data_callback_fp callback);

void bt_ssp_init(void);

void bt_ssp_evt_handler(bt_hci_event_t* evt);

bt_bool bt_ssp_send_user_passkey(
	bt_byte status,
	bt_ssp_user_passkey_request* upkr,
	bt_hci_cmd_callback_fp cb);

bt_bool bt_ssp_send_user_confirmation(
	bt_byte status,
	bt_ssp_user_confirmation_request* ucr,
	bt_hci_cmd_callback_fp cb);

bt_bool bt_ssp_set_oob_data(
	bt_byte status,
	bt_ssp_oob_data* oob_data,
	bt_hci_cmd_callback_fp cb);


bt_bool bt_ssp_set_io_capabilities(
	bt_byte status,
	bt_ssp_io_capability* io_caps,
	bt_hci_cmd_callback_fp cb);

#include "cdbt/ssp/ssp_event_handler.h"

extern void (*_bt_ssp_evt_handler)(bt_hci_event_t* evt);
extern void (*_bt_ssp_init)(void);

#ifdef __cplusplus
}
#endif

#endif // __SSP_H
