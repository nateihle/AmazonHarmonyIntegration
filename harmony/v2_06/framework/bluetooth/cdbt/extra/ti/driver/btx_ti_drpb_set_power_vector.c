/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2015 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#include "cdbt/bt/bt_std.h"
#include "cdbt/extra/ti/ti.h"
#include "cdbt/extra/ti/ti_private.h"

bt_bool btx_ti_drpb_set_power_vector(
    bt_byte modulation_type, const bt_byte* power_vector,
    bt_byte epc_max_level_threshold, bt_uint external_pa_mode,
    bt_hci_cmd_callback_fp callback)
{
    bt_hci_command_t* cmd;
    bt_bool ret;

    cmd = bt_hci_alloc_command(0xFD82, callback);     // HCI_VS_DRPb_Set_Power_Vector
    if (cmd)
    {
        bt_hci_add_param_byte(cmd, modulation_type);
        _bt_memcpy(cmd->params, cmd->params_len, power_vector, 15, 0);
        cmd->params_len += 15;
        bt_hci_add_param_byte(cmd, epc_max_level_threshold);
        bt_hci_add_param_uint(cmd, external_pa_mode);

        ret = bt_hci_send_cmd(cmd);
    }
    else
    {
        ret = BT_FALSE;
    }
    return ret;
}
