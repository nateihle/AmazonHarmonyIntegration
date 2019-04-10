/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2015 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#include "cdbt/bt/bt_std.h"
#include "cdbt/extra/ti/ti.h"
#include "cdbt/extra/ti/ti_private.h"


bt_bool btx_ti_set_uart_baud_rate(bt_ulong baud_rate, bt_hci_cmd_callback_fp callback)
{
    bt_hci_command_t* cmd;
    bt_bool ret;

    cmd = bt_hci_alloc_command(0xFF36, callback);     // HCI_VS_Update_UART_HCI_Baudrate
    if (cmd)
    {
        bt_hci_add_param_long(cmd, baud_rate);
        ret = bt_hci_send_cmd(cmd);
        //#if DEBUGLEVEL >= DEBUGNORMAL
        //printf("bt_hci_send_cmd: %d\n", ret);
        //#endif
    }
    else
    {
        ret = BT_FALSE;
    }
    asm("Nop");
    return ret;
}
