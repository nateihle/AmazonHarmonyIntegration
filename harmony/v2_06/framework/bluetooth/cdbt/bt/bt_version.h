/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __BT_VERSION_H_INCLUDED__
#define __BT_VERSION_H_INCLUDED__

#ifdef __cplusplus
extern "C" {
#endif

/** 
 * \brief Get the version of the dotstack library.
 * \ingroup sys
 *
 * \return The version of the dotstack library.
*/
const bt_byte* bt_sys_get_version(void);

#ifdef __cplusplus
}
#endif

#endif /* __BT_VERSION_H_INCLUDED__  */

