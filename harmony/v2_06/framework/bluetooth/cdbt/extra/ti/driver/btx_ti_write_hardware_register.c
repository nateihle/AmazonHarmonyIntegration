/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2015 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#include "cdbt/bt/bt_std.h"
#include "cdbt/extra/ti/ti.h"
#include "cdbt/extra/ti/ti_private.h"

bt_bool btx_ti_write_hardware_register(
    bt_ulong address,
    bt_uint value,
    bt_hci_cmd_callback_fp callback)
{
    bt_hci_command_t* cmd;
    bt_bool ret;

    cmd = bt_hci_alloc_command(0xFF01, callback);     // HCI_VS_Write_Hardware_Register
    if (cmd)
    {
        bt_hci_add_param_ulong(cmd, address);
        bt_hci_add_param_uint(cmd, value);

        ret = bt_hci_send_cmd(cmd);
    }
    else
    {
        ret = BT_FALSE;
    }
    return ret;
}
