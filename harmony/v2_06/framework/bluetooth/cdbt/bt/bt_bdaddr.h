/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __BT_BDADDR_H
#define __BT_BDADDR_H

#ifdef __cplusplus
extern "C" {
#endif

bt_bool bt_bdaddrs_are_equal(const bt_bdaddr_t* bda1, const bt_bdaddr_t* bda2);

bt_bool bt_bdaddr_is_null(const bt_bdaddr_t* bda);


#ifdef __cplusplus
}
#endif

#endif /* __BT_BDADDR_H */
