/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2015 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#include "cdbt/bt/bt_std.h"
#include "cdbt/extra/ti/ti.h"
#include "cdbt/extra/ti/ti_private.h"

#define USE_TI_RECOMMENDED_VALUES 1

bt_bool btx_ti_enable_fast_clock_crystal(bt_hci_cmd_callback_fp callback)
{
    bt_hci_command_t* cmd;
    bt_bool ret;

    // This command is required for proper operations of BL6450.
    // Without this command the fast clock crystal is turned off
    // after first deep sleep.
    cmd = bt_hci_alloc_command(0xFD1C, callback); // HCI_VS_Fast_Clock_Configuration_BTIP
    if (cmd)
    {
        bt_hci_add_param_byte(cmd, 0x1);   // XTAL enable
        bt_hci_add_param_long(cmd, 5000);  // Normal wake-up settling time
        bt_hci_add_param_long(cmd, 2000);  // Fast wake-up settling time
        bt_hci_add_param_byte(cmd, 0xFF);  // Fast wake-up enable (0xFF = do not change)
        bt_hci_add_param_byte(cmd, 0xFF);  // XTAL boost gain (0xFF = do not change)
#ifdef USE_TI_RECOMMENDED_VALUES
        bt_hci_add_param_byte(cmd, 0x04);  // XTAL normal gain (0xFF = do not change)
#else
        bt_hci_add_param_byte(cmd, 0xFF);  // XTAL normal gain (0xFF = do not change)
#endif
        bt_hci_add_param_byte(cmd, 0xFF);  // BT TX slicer trim (0xFF = do not change)
        bt_hci_add_param_byte(cmd, 0xFF);  // BT TX idle slicer trim (0xFF = do not change)
        bt_hci_add_param_byte(cmd, 0xFF);  // Fast clock input AC/DC Size (0xFF = do not change)
        bt_hci_add_param_byte(cmd, 250);   // Slow clock accuracy
        bt_hci_add_param_byte(cmd, 0);     // Clock source (Used only for 5500)
        bt_hci_add_param_byte(cmd, 0);     // GCM extra settling time (WL128x Only)
#ifdef USE_TI_RECOMMENDED_VALUES
        bt_hci_add_param_byte(cmd, 0);
#else
        bt_hci_add_param_byte(cmd, 0x42);  // Reserved (must be set to 0x42)
#endif
        ret = bt_hci_send_cmd(cmd);
    }
    else
    {
        ret = BT_FALSE;
    }
    return ret;
}

