/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2015 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#include "cdbt/bt/bt_std.h"
#include "cdbt/extra/ti/ti.h"
#include "cdbt/extra/ti/ti_private.h"

bt_bool btx_ti_drpb_enable_rf_calibration(
   bt_byte periodic_mode, bt_ulong calibration_procedure,
   bt_byte temp_condition, bt_hci_cmd_callback_fp callback)
{
    bt_hci_command_t* cmd;
    bt_bool ret;

    cmd = bt_hci_alloc_command(0xFD80, callback);     // HCI_VS_DRPb_Enable_RF_Calibration
    if (cmd)
    {
        bt_hci_add_param_byte(cmd, periodic_mode);
        bt_hci_add_param_ulong(cmd, calibration_procedure);
        bt_hci_add_param_byte(cmd, temp_condition);

        ret = bt_hci_send_cmd(cmd);
    }
    else
    {
        ret = BT_FALSE;
    }
    return ret;
}
