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

#ifndef __HCI_UTILS_H
#define __HCI_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

bt_bool is_hconn_command(bt_int opcode);
bt_bool is_bdaddr_command(bt_int opcode);

#ifdef __cplusplus
}
#endif

#endif // __HCI_UTILS_H
