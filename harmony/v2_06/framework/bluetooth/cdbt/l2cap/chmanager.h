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

#ifndef __L2CAP_CHMANAGER_H
#define __L2CAP_CHMANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

void bt_l2cap_init_channels(bt_l2cap_mgr_p pmgr);
bt_l2cap_channel_t* bt_l2cap_allocate_channel(bt_l2cap_mgr_p pmgr, bt_int mode, bt_int type);
void bt_l2cap_free_channel(bt_l2cap_channel_t* ch);

bt_l2cap_channel_t* bt_l2cap_get_channel(const bt_l2cap_mgr_p pmgr, bt_id cid);
bt_l2cap_channel_t* bt_l2cap_get_channel_by_hconn_dest_cid(const bt_l2cap_mgr_p pmgr, bt_hci_hconn_t hconn, bt_id cid_dest);
bt_l2cap_channel_t* bt_l2cap_get_channel_by_hconn_cid(const bt_l2cap_mgr_p pmgr, bt_hci_hconn_t hconn, bt_id cid_dest);
bt_l2cap_channel_t* bt_l2cap_get_channel_by_bdaddr_cid(const bt_l2cap_mgr_p pmgr, bt_bdaddr_t* bdaddr, bt_id cid);
bt_l2cap_channel_t* bt_l2cap_get_channel_by_psm(const bt_l2cap_mgr_p pmgr, bt_int psm);

#ifdef __cplusplus
}
#endif

#endif // __L2CAP_CHMANAGER_H
