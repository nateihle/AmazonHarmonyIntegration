/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __L2CAP_FIXED_CHANNEL_H
#define __L2CAP_FIXED_CHANNEL_H

#ifdef __cplusplus
extern "C" {
#endif

#define L2CAP_LINK_TYPE_BD_EDR	HCI_LINK_TYPE_BD_EDR
#define L2CAP_LINK_TYPE_LE		HCI_LINK_TYPE_LE

bt_l2cap_fixed_channel_t* bt_l2cap_find_fixed_channel(const bt_l2cap_mgr_p pmgr, bt_id cid, bt_byte link_type);
bt_l2cap_fixed_channel_t* bt_l2cap_allocate_fixed_channel(bt_l2cap_mgr_p pmgr, bt_id cid, bt_byte link_type);
void bt_l2cap_free_fixed_channel(bt_l2cap_fixed_channel_t* fc);

#ifdef __cplusplus
}
#endif

#endif // __L2CAP_FIXED_CHANNEL_H
