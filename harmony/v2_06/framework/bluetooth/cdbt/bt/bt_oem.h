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
#ifndef __BT_OEM_H
#define __BT_OEM_H

#include "cdbt/ssp/ssp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _bt_linkkey_notification_t
{
	bt_bdaddr_t bdaddr_remote;
	bt_linkkey_t key;
	bt_byte key_type;

} bt_linkkey_notification_t;

typedef struct _bt_linkkey_request_t
{
	bt_bdaddr_t bdaddr_remote;

} bt_linkkey_request_t;

const char* bt_oem_get_device_name(void);

bt_long bt_oem_get_device_class(void);

void bt_oem_get_pin_code(bt_bdaddr_t* bdaddr_remote);

void bt_oem_ssp_callback(SSP_EVENT spp_event, void* event_param, void* init_param);

void bt_oem_schedule_signals(void);

void bt_oem_linkkey_notification(bt_linkkey_notification_t* lkn);

void bt_oem_linkkey_request(bt_linkkey_request_t* lkr);

void bt_oem_assert(const char* file, int line);


#ifdef __cplusplus
}
#endif

#endif // __BT_OEM_H
