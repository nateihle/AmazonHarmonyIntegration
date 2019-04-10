/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2015 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#include "cdbt/bt/bt_std.h"
#include "cdbt/bt/bt_private.h"
#include "cdbt/extra/ti/ti.h"
#include "cdbt/extra/ti/ti_private.h"


bt_bool btx_ti_avpr_enable(
	bt_byte enable, bt_byte load_a3dp_code, bt_byte a3dp_role,
	bt_hci_cmd_callback_fp callback)
{
	bt_hci_command_t* cmd;
	bt_bool ret;

	cmd = bt_hci_alloc_command(0xFD92, callback);     // HCI_VS_AVPR_Enable
	if (cmd)
	{
		bt_hci_add_param_byte(cmd, enable);
		bt_hci_add_param_byte(cmd, a3dp_role);
		bt_hci_add_param_byte(cmd, load_a3dp_code);
		bt_hci_add_param_int(cmd, 0);

		ret = bt_hci_send_cmd(cmd);
	}
	else
	{
		ret = BT_FALSE;
	}
	return ret;
}

bt_bool btx_ti_avpr_debug(bt_hci_cmd_callback_fp callback)
{
	bt_hci_command_t* cmd;
	bt_bool ret;

	cmd = bt_hci_alloc_command(0xFD91, callback);     // HCI_VS_AVPR_Debug
	if (cmd)
	{
		bt_hci_add_param_byte(cmd, 1);
		bt_hci_add_param_byte(cmd, 0);
		bt_hci_add_param_byte(cmd, 1);
		bt_hci_add_param_int(cmd, 0);

		ret = bt_hci_send_cmd(cmd);
	}
	else
	{
		ret = BT_FALSE;
	}
	return ret;
}
