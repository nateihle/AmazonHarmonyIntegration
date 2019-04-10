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

#ifndef __L2CAP_PSM_H
#define __L2CAP_PSM_H

#ifdef __cplusplus
extern "C" {
#endif

void bt_l2cap_init_psms(bt_l2cap_mgr_p pmgr);
bt_l2cap_psm_t* bt_l2cap_find_psm(const bt_l2cap_mgr_p pmgr, bt_int psmId);
bt_l2cap_psm_t* bt_l2cap_allocate_psm(bt_l2cap_mgr_p pmgr, bt_int psmId);
void bt_l2cap_free_psm(bt_l2cap_psm_t* psm);

#ifdef __cplusplus
}
#endif

#endif // __L2CAP_PSM_H
