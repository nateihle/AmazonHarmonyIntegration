/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2015 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#include "cdbt/bt/bt_std.h"
#include "cdbt/extra/ti/ti.h"
#include "cdbt/extra/ti/ti_private.h"


//=============================================================================
// btx_ti_write_bdaddr()
//=============================================================================
bt_bool btx_ti_write_bdaddr(bt_bdaddr_t* bdaddr, bt_hci_cmd_callback_fp callback)
{
    bt_hci_command_t* cmd;
    bt_bool ret;

    cmd = bt_hci_alloc_command(0xFC06, callback);     // HCI_VS_Write_BD_Addr
    if (cmd)
    {
        bt_ulong a;

        // bt_hci_add_param_bdaddr cannot be used here as this TI command
        // excpects the BD address MSB first while standard HCI commands use LSB first.
        a = bdaddr->bd_addr_m;
        bt_hci_add_param_byte(cmd, (bt_byte)((a >> 8) & 0xFF));
        bt_hci_add_param_byte(cmd, (bt_byte)(a & 0xFF));
        a = bdaddr->bd_addr_l;
        bt_hci_add_param_byte(cmd, (bt_byte)((a >> 24) & 0xFF));
        bt_hci_add_param_byte(cmd, (bt_byte)((a >> 16) & 0xFF));
        bt_hci_add_param_byte(cmd, (bt_byte)((a >> 8) & 0xFF));
        bt_hci_add_param_byte(cmd, (bt_byte)(a & 0xFF));

        ret = bt_hci_send_cmd(cmd);
    }
    else
    {
        ret = BT_FALSE;
    }
    return ret;
}

//=============================================================================
//btx_ti_read_bdaddr()
//=============================================================================
bt_bool btx_ti_read_bdaddr(bt_hci_cmd_callback_fp callback)
{
    bt_bool ret;
    bt_hci_command_t* cmd;

    cmd = bt_hci_alloc_command(HCI_READ_BD_ADDR, callback);
    if (cmd)
        ret = bt_hci_send_cmd(cmd);
    else
        ret = BT_FALSE;

    return ret;
}
