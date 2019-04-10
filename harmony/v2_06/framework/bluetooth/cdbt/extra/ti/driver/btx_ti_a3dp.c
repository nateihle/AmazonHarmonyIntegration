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

bt_bool btx_ti_a3dp_sink_codec_config(
	bt_byte channels, bt_byte sample_frequency, bt_byte channel_mode, 
	bt_byte blocks, bt_byte sub_bands, bt_byte allocation_method,
	bt_hci_cmd_callback_fp callback)
{
	bt_hci_command_t* cmd;
	bt_bool ret;

	cmd = bt_hci_alloc_command(0xFD9C, callback);     // HCI_VS_A3DP_Sink_Codec_Configuration
	if (cmd)
	{
		bt_hci_add_param_byte(cmd, channels);
		bt_hci_add_param_byte(cmd, sample_frequency);
		bt_hci_add_param_byte(cmd, channel_mode);
		bt_hci_add_param_byte(cmd, blocks);
		bt_hci_add_param_byte(cmd, sub_bands);
		bt_hci_add_param_byte(cmd, allocation_method);
/*
		bt_hci_add_param_byte(cmd, 0x0A);
		bt_hci_add_param_byte(cmd, 0x39);
*/
		bt_hci_add_param_ulong(cmd, 0);
		bt_hci_add_param_ulong(cmd, 0);
		bt_hci_add_param_ulong(cmd, 0);

		ret = bt_hci_send_cmd(cmd);
	}
	else
	{
		ret = BT_FALSE;
	}
	return ret;
}

bt_bool btx_ti_a3dp_sink_open_stream(
	bt_byte acl_handle, bt_uint l2cap_cid, 
	bt_hci_cmd_callback_fp callback)
{
	bt_hci_command_t* cmd;
	bt_bool ret;

	cmd = bt_hci_alloc_command(0xFD9A, callback);     // HCI_VS_A3DP_Sink_Open_Stream
	if (cmd)
	{
		bt_hci_add_param_byte(cmd, acl_handle);
		bt_hci_add_param_uint(cmd, l2cap_cid);
		bt_hci_add_param_ulong(cmd, 0);
		bt_hci_add_param_ulong(cmd, 0);

		ret = bt_hci_send_cmd(cmd);
	}
	else
	{
		ret = BT_FALSE;
	}
	return ret;
}

bt_bool btx_ti_a3dp_sink_close_stream(bt_hci_cmd_callback_fp callback)
{
	bt_hci_command_t* cmd;
	bt_bool ret;

	cmd = bt_hci_alloc_command(0xFD9B, callback);     // HCI_VS_A3DP_Sink_Close_Stream
	if (cmd)
	{
		bt_hci_add_param_ulong(cmd, 0);

		ret = bt_hci_send_cmd(cmd);
	}
	else
	{
		ret = BT_FALSE;
	}
	return ret;
}

bt_bool btx_ti_a3dp_sink_start_stream(bt_hci_cmd_callback_fp callback)
{
	bt_hci_command_t* cmd;
	bt_bool ret;

	cmd = bt_hci_alloc_command(0xFD9D, callback);     // HCI_VS_A3DP_Sink_Start_Stream
	if (cmd)
	{
		bt_hci_add_param_ulong(cmd, 0);

		ret = bt_hci_send_cmd(cmd);
	}
	else
	{
		ret = BT_FALSE;
	}
	return ret;
}

bt_bool btx_ti_a3dp_sink_stop_stream(bt_hci_cmd_callback_fp callback)
{
	bt_hci_command_t* cmd;
	bt_bool ret;

	cmd = bt_hci_alloc_command(0xFD9E, callback);     // HCI_VS_A3DP_Sink_Stop_Stream
	if (cmd)
	{
		bt_hci_add_param_ulong(cmd, 0);

		ret = bt_hci_send_cmd(cmd);
	}
	else
	{
		ret = BT_FALSE;
	}
	return ret;
}
