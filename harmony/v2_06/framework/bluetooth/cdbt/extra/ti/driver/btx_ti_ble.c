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


bt_bool btx_ti_le_enable(
	bt_byte enable, bt_byte load_le_code,
	bt_hci_cmd_callback_fp callback)
{
	bt_hci_command_t* cmd;
	bt_bool ret;

	cmd = bt_hci_alloc_command(0xFD5B, callback);     // HCI_VS_LE_Enable
	if (cmd)
	{
		bt_hci_add_param_byte(cmd, enable);
		bt_hci_add_param_byte(cmd, load_le_code);

		ret = bt_hci_send_cmd(cmd);
	}
	else
	{
		ret = BT_FALSE;
	}
	return ret;
}
