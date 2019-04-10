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


bt_bool btx_ti_write_codec_config(const btx_ti_codec_config_t* codec_config, bt_hci_cmd_callback_fp callback)
{
	bt_hci_command_t* cmd;
	bt_bool ret;

	if (_hci_max_cmd_param_len < 34)
	{
		LOG("btx_ti_write_codec_config: HCI_MAX_CMD_PARAM_LEN is too small. Increase to at least 34");
		return FALSE;
	}

	cmd = bt_hci_alloc_command(0xFD06, callback);     // HCI_VS_Write_CODEC_Config
	if (cmd)
	{
		bt_hci_add_param_uint(cmd, codec_config->clock_rate);
		bt_hci_add_param_byte(cmd, codec_config->clock_direction);
		bt_hci_add_param_ulong(cmd, codec_config->frame_sync_freq);
		bt_hci_add_param_uint(cmd, codec_config->frame_sync_duty_cycle);
		bt_hci_add_param_byte(cmd, codec_config->frame_sync_edge);
		bt_hci_add_param_byte(cmd, codec_config->frame_sync_polarity);
		bt_hci_add_param_byte(cmd, 0);
		bt_hci_add_param_uint(cmd, codec_config->ch1_data_out_size);
		bt_hci_add_param_uint(cmd, codec_config->ch1_data_out_offset);
		bt_hci_add_param_byte(cmd, codec_config->ch1_data_out_edge);
		bt_hci_add_param_uint(cmd, codec_config->ch1_data_in_size);
		bt_hci_add_param_uint(cmd, codec_config->ch1_data_in_offset);
		bt_hci_add_param_byte(cmd, codec_config->ch1_data_in_edge);
		bt_hci_add_param_byte(cmd, codec_config->frame_sync_multiplier);
		bt_hci_add_param_uint(cmd, codec_config->ch2_data_out_size);
		bt_hci_add_param_uint(cmd, codec_config->ch2_data_out_offset);
		bt_hci_add_param_byte(cmd, codec_config->ch2_data_out_edge);
		bt_hci_add_param_uint(cmd, codec_config->ch2_data_in_size);
		bt_hci_add_param_uint(cmd, codec_config->ch2_data_in_offset);
		bt_hci_add_param_byte(cmd, codec_config->ch2_data_in_edge);
		bt_hci_add_param_byte(cmd, 0);

		ret = bt_hci_send_cmd(cmd);
	}
	else
	{
		ret = BT_FALSE;
	}
	return ret;
}
