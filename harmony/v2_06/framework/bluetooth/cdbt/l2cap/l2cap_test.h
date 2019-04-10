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

#ifndef __L2CAP_TEST
#define __L2CAP_TEST

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _ENABLE_TEST

/* if set to FALSE, l2cap_send_config will do nothing */
extern bt_bool _enable_local_config;
extern bt_bool _enable_remote_config;

#define bt_l2cap_test_enable_local_config(enable)	_enable_local_config = enable

#define bt_l2cap_test_enable_remote_config(enable)	_enable_remote_config = enable

#else

#define bt_l2cap_test_enable_local_config(enable)

#define bt_l2cap_test_enable_remote_config(enable)

#endif

#ifdef __cplusplus
}
#endif

#endif // __L2CAP_TEST
