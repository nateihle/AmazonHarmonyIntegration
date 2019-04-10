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

#ifndef __SSP_EVENT_HANDLER_H
#define __SSP_EVENT_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

void ssp_evt_io_capability_request(bt_hci_event_t* evt);
void ssp_evt_io_capability_response(bt_hci_event_t* evt);
void ssp_evt_user_confirmation_request(bt_hci_event_t* evt);
void ssp_evt_user_passkey_request(bt_hci_event_t* evt);
void ssp_evt_oob_data_request(bt_hci_event_t* evt);
void ssp_evt_ssp_complete(bt_hci_event_t* evt);
void ssp_evt_user_passkey_notification(bt_hci_event_t* evt);
void ssp_evt_keypress_notification(bt_hci_event_t* evt);

#ifdef __cplusplus
}
#endif

#endif // __SSP_EVENT_HANDLER_H
