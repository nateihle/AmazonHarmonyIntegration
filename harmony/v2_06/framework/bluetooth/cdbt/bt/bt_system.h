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

#ifndef __BT_SYSTEM_H_INCLUDED__
#define __BT_SYSTEM_H_INCLUDED__

#include "cdbt/l2cap/l2cap.h"
#include "cdbt/gap/gap.h"


#ifdef __cplusplus
extern "C" {
#endif


/** @defgroup sys System Functions

	Functions in this module provide interface to DotStack functionality that is common to
	all protocols and profiles.
*/
/*@{*/ /* begin doxygen group */


/** System start callback.

	This callback function is called when system start initiated by bt_sys_start() has
	completed.

	@param success Success of the operation:
			\c BT_TRUE if successfull, \c BT_FALSE otherwise.
	
	@param param Callback parameter that was specified when bt_sys_start() was called.
*/
typedef void (*bt_sys_callback_fp)(bt_bool success, void* param);


/** Initialize the Bluetooth system.

	This function initializes all internal variables of HCI, L2CAP and SDP modules. It
	must be called by the application before it can access any functionality provided by
	the library. In addition to this initialization function the application must call
	initialization functions of all other profile modules the application is intended to
	use. E.g., if the application is using the SPP module the bt_spp_init() must be called
	right after calling bt_sys_init().

	This function essentially calls bt_sys_init_ex(HCI_LINK_POLICY_ENABLE_ALL) so all link policy
	setting are enabled.
*/
void bt_sys_init(void);


/** Initialize the Bluetooth system.

	This function initializes all internal variables of HCI, L2CAP and SDP modules. It
	must be called by the application before it can access any functionality provided by
	the library. In addition to this initialization function the application must call
	initialization functions of all other profile modules the application is intended to
	use. E.g., if the application is using the SPP module the bt_spp_init() must be called
	right after calling bt_sys_init().
	
	Also, the caller must provide an SDP database.

	@param default_link_policy default link policy settings. This is a bitmask that defines the initial value of the
	                           link policy settings for all new BR/EDR connections. This value can be a combination of the following values:
	                           \li HCI_LINK_POLICY_ENABLE_ROLE_SWITCH
	                           \li HCI_LINK_POLICY_ENABLE_HOLD_MODE
	                           \li HCI_LINK_POLICY_ENABLE_SNIFF_MODE
	                           \li HCI_LINK_POLICY_ENABLE_PARK_STATE

							   To enable all settings pass HCI_LINK_POLICY_ENABLE_ALL.
*/
void bt_sys_init_ex(bt_byte default_link_policy);

/** Start the Bluetooth system.

	After all modules used by the application have been initialized this function should
	be called to start the Bluetooth system operation. During the start up sequence it
	will reset and initialize the HCI controller and then create the L2CAP manager. The
	application will be notified when the start up sequence completes by calling the
	provided callback function.

	Also, the caller must provide an SDP database.

	@param discoverable defines whether the device is discoverable after reset.
	@param connectable defines whether the device is connectable after reset.
	@param sdp_db SDP database data.
	@param sdp_db_len Length of SDP database data.
	@param callback A callback function that will be called when the start up sequence is
			complete.
	@param callback_param An arbitrary pointer that will be passed to the callback
			function.
			
*/
void bt_sys_start(bt_bool discoverable, bt_bool connectable,
		const bt_byte* sdp_db, bt_uint sdp_db_len, bt_sys_callback_fp callback,
		void* callback_param);

void bt_sys_set_modes(bt_bool discoverable, bt_bool connectable,
		bt_sys_callback_fp callback, void* callback_param);

bt_bool bt_sys_get_discoverable(void);

bt_bool bt_sys_get_connectable(void);

/** Get the L2CAP manager.

	This function returns the L2CAP manager. The L2CAP manager is created as part of the
	start up sequence.
	
	@return The L2CAP manager.
*/
bt_l2cap_mgr_t* bt_sys_get_l2cap_manager(void);

#include "cdbt/bt/bt_version.h"

/*@}*/ /* end doxygen group */

#ifdef __cplusplus
}
#endif

#endif /* __BT_SYSTEM_H_INCLUDED__  */
