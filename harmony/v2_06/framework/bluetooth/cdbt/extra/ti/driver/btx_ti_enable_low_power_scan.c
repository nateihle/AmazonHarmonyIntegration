/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
*
* Copyright (c) 2011-2015 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#include "cdbt/bt/bt_std.h"
#include "cdbt/extra/ti/ti.h"
#include "cdbt/extra/ti/ti_private.h"


bt_bool btx_ti_enable_low_power_scan(bt_bool enable, bt_hci_cmd_callback_fp callback)
{
    bt_hci_command_t* cmd;
    bt_bool ret;

    cmd = bt_hci_alloc_command(0xFD2E, callback);     // HCI_VS_Set_LPS_Params_BTIP
    if (cmd)
    {
        // Enable low-power scan: enable = 1; disable = 0
        bt_hci_add_param_byte(cmd, enable ? 1 : 0);

        // Disable sweeps length.
        // Number of scans after LPS exit to return back to LPS.
        bt_hci_add_param_int(cmd, 600);

        // Positive sweeps Threshold.
        // Number of consequent positive scans to exit LPS.
        // Note: default value is 5, TI recommended value is 6
        bt_hci_add_param_byte(cmd, 6);

        // Enable LPS in active connection.
        // 1 - run LPS scan in active connection. 0 - run LPS only in Idle mode.
        bt_hci_add_param_byte(cmd, 0);

        // Minimum time between scan.
        // The minimum time betweet to LPS scan in frame (1.25 msec)
        bt_hci_add_param_byte(cmd, 30);

        // Scans history max length.
        // The number of lps scan results to remember for positive scans counting (max: 32)
        // Note: default value is 7, TI recommended value is 0
        bt_hci_add_param_byte(cmd, 0);

        // Reserved
        bt_hci_add_param_int(cmd, 0);

        // Reserved
        bt_hci_add_param_long(cmd, 0);

        ret = bt_hci_send_cmd(cmd);
    }
    else
    {
        ret = BT_FALSE;
    }
    return ret;
}


bt_bool btx_ti_enable_low_power_scan_default(bt_bool enable, bt_hci_cmd_callback_fp callback)
{
    bt_hci_command_t* cmd;
    bt_bool ret;

    cmd = bt_hci_alloc_command(0xFD2E, callback);     // HCI_VS_Set_LPS_Params_BTIP
    if (cmd)
    {
        // Enable low-power scan: enable = 1; disable = 0
        bt_hci_add_param_byte(cmd, enable ? 1 : 0);

        // Disable sweeps length.
        // Number of scans after LPS exit to return back to LPS.
        bt_hci_add_param_int(cmd, 600);

        // Positive sweeps Threshold.
        // Number of consequent positive scans to exit LPS.
        // Note: default value is 5, TI recommended value is 6
        bt_hci_add_param_byte(cmd, 5);

        // Enable LPS in active connection.
        // 1 - run LPS scan in active connection. 0 - run LPS only in Idle mode.
        bt_hci_add_param_byte(cmd, 0);

        // Minimum time between scan.
        // The minimum time betweet to LPS scan in frame (1.25 msec)
        bt_hci_add_param_byte(cmd, 30);

        // Scans history max length.
        // The number of lps scan results to remember for positive scans counting (max: 32)
        // Note: default value is 7, TI recommended value is 0
        bt_hci_add_param_byte(cmd, 7);

        // Reserved
        bt_hci_add_param_int(cmd, 0);

        // Reserved
        bt_hci_add_param_long(cmd, 0);

        ret = bt_hci_send_cmd(cmd);
    }
    else
    {
        ret = BT_FALSE;
    }
    return ret;
}
