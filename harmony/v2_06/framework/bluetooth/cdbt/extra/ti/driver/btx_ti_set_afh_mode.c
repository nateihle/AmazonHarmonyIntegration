/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2015 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#include "cdbt/bt/bt_std.h"
#include "cdbt/extra/ti/ti.h"
#include "cdbt/extra/ti/ti_private.h"


bt_bool btx_ti_set_afh_mode(bt_hci_hconn_t hconn, bt_bool enable, bt_hci_cmd_callback_fp callback)
{
	bt_hci_command_t* cmd;
	bt_bool ret;

	// Enable/disable AFH for a connection.
	cmd = bt_hci_alloc_command(0xFF39, callback);     // HCI_VS_Set_AFH_Mode
	if (cmd)
	{
		bt_hci_add_param_hconn(cmd, hconn);   
		bt_hci_add_param_byte(cmd, enable ? 1 : 0);   // AFH enable = 1; disable = 0
		ret = bt_hci_send_cmd(cmd);
	}
	else
	{
		ret = BT_FALSE;
	}
	return ret;
}
