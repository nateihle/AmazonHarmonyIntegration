/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2015 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#include "cdbt/bt/bt_std.h"
#include "cdbt/extra/ti/ti.h"
#include "cdbt/extra/ti/ti_private.h"

bt_bool btx_ti_drpb_tester_con_tx(
    bt_byte modulation_scheme,
    bt_byte test_pattern,
    bt_byte frequency_channel,
    bt_byte power_level,
    bt_ulong generator_initialization_value,
    bt_ulong edr_generator_mask,
    bt_hci_cmd_callback_fp callback)
{
    bt_hci_command_t* cmd;
    bt_bool ret;

    cmd = bt_hci_alloc_command(0xFD84, callback);     // HCI_VS_DRPb_Tester_Con_TX
    if (cmd)
    {
        bt_hci_add_param_byte(cmd, modulation_scheme);
        bt_hci_add_param_byte(cmd, test_pattern);
        bt_hci_add_param_byte(cmd, frequency_channel);
        bt_hci_add_param_byte(cmd, power_level);
        bt_hci_add_param_ulong(cmd, generator_initialization_value);
        bt_hci_add_param_ulong(cmd, edr_generator_mask);

        ret = bt_hci_send_cmd(cmd);
    }
    else
    {
        ret = BT_FALSE;
    }
    return ret;
}
