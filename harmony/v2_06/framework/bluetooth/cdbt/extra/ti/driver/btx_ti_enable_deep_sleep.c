// DOM-IGNORE-BEGIN
/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2015 SEARAN LLC. All Rights Reserved.
*******************************************************************************/
// DOM-IGNORE-END

#include "cdbt/bt/bt_std.h"
#include "cdbt/extra/ti/ti.h"
#include "cdbt/extra/ti/ti_private.h"


bt_bool btx_ti_enable_deep_sleep(bt_bool enable, bt_hci_cmd_callback_fp callback)
{
    bt_hci_command_t* cmd;
    bt_bool ret;

    // Enable/disable controller's deep sleep mode.
    cmd = bt_hci_alloc_command(0xFD0C, callback);     // HCI_VS_Sleep_Mode_Configurations
    if (cmd)
    {
        bt_hci_add_param_byte(cmd, enable ? 1 : 0);   // Big sleep enable = 1; disable = 0
        bt_hci_add_param_byte(cmd, enable ? 1 : 0);   // Deep sleep enable = 1; disable = 0
        bt_hci_add_param_byte(cmd, 0);                // Deep sleep mode = HCILL
        bt_hci_add_param_byte(cmd, 0xFF);             // Output I/O select: no change
        bt_hci_add_param_byte(cmd, 0xFF);             // Output pull enable: no change
        bt_hci_add_param_byte(cmd, 0xFF);             // Input pull enable: no change
        bt_hci_add_param_byte(cmd, 0xFF);             // Input I/O select: no change
        bt_hci_add_param_int(cmd, 0x64);              // See note below
        ret = bt_hci_send_cmd(cmd);
    }
    else
    {
        ret = BT_FALSE;
    }
    return ret;
}

// Note: BL6450 datasheet describes the last parameter of the HCI_VS_Sleep_Mode_Configurations
// command as 'Reserved: Default value 0x00 must be used'. Other sources are using value 0x64
// and describe it as 'Host_Wake deasertion timer'. Most likely it specifies the duration of
// the wakeup impulse on the controller's RTS line.
