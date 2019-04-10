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

#ifndef __HCITR_3WIRE_H_INCLUDED__
#define __HCITR_3WIRE_H_INCLUDED__


#ifdef __cplusplus
extern "C" {
#endif

#define HCITR_3WIRE_DEFAULT_ACK_TIMEOUT    250

#define bt_hcitr_3wire_init()              bt_hcitr_3wire_init_ex(HCITR_3WIRE_DEFAULT_ACK_TIMEOUT, BT_FALSE)
#define bt_hcitr_3wire_reset()             bt_hcitr_3wire_reset_ex(HCITR_3WIRE_DEFAULT_ACK_TIMEOUT, BT_FALSE)
void bt_hcitr_3wire_start(void);
void bt_hcitr_3wire_cancel_recv_packet(void);

void bt_hcitr_3wire_init_ex(bt_ulong ack_timeout, bt_bool data_integrity_check_enabled);
void bt_hcitr_3wire_reset_ex(bt_ulong ack_timeout, bt_bool data_integrity_check_enabled);

#ifdef __cplusplus
}
#endif

#endif // __HCITR_3WIRE_H_INCLUDED__

