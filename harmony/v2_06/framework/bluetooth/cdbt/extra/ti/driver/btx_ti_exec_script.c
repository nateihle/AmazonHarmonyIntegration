/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2015 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#include "cdbt/bt/bt_std.h"
#include "cdbt/bt/bt_private.h"
#include "cdbt/bt/bt_hcitr.h"
#include "cdbt/hci/hci.h"
#include "cdbt/extra/ti/ti.h"


static btx_ti_exec_script_buffer_t* m_buffer;


static void check_fw_version(void);
static void get_system_status_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt);
static void send_next_command(void);
static void send_command_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt);


//=============================================================================
// btx_ti_exec_script()
//=============================================================================
void btx_ti_exec_script(const btx_ti_script_t* script,
                        btx_ti_exec_script_buffer_t* buffer,
                        btx_ti_completion_callback_fp callback,
                        void* callback_param)
{
    memset(buffer, 0, sizeof(btx_ti_exec_script_buffer_t));
    buffer->script = script;
    buffer->callback = callback;
    buffer->callback_param = callback_param;
    m_buffer = buffer;

    if (script->fw_version_x == 0 && script->fw_version_z == 0)
        send_next_command();
    else
        check_fw_version();
}


//=============================================================================
// check_fw_version()
//=============================================================================
static void check_fw_version(void)
{
    bt_hci_command_t* cmd;
    
    cmd = bt_hci_alloc_command(0xFE1F, &get_system_status_callback); // HCI_VS_Get_System_Status
    bt_hci_send_cmd(cmd);
}


//=============================================================================
// get_system_status_callback()
//=============================================================================
static void get_system_status_callback(bt_int status, 
                                       bt_hci_command_t* cmd, 
                                       bt_hci_event_t* evt)
{
    const btx_ti_script_t* script = m_buffer->script;
    
    //Modification
    bt_byte firmware_version_x, firmware_version_z, chip_hardware_revision;
    
    firmware_version_x     = evt->params[4];
    firmware_version_z     = evt->params[5];
    chip_hardware_revision = evt->params[6];

    // Check that the script is compatible with the hardware.
    if (firmware_version_x != script->fw_version_x || 
        firmware_version_z != script->fw_version_z)
    {
        //_BT_DBG("BT:script - ver_x:%d, ver_z:%d\n", 
        //         script->fw_version_x, script->fw_version_z);
        //_BT_DBG("BT:TI module - ver_x:%d, ver_z:%d, chip_hw_ver:%d\n", 
        //         firmware_version_x, firmware_version_z, chip_hardware_revision);
        m_buffer->callback(BT_FALSE, m_buffer, m_buffer->callback_param);
    }
    else if (script->packet_count)
    {
        // Proceed with executing the init script.    
        //_BT_DBG("BT - send_next_command\n");
        send_next_command();
    }
    else
    {
        m_buffer->callback(BT_TRUE, m_buffer, m_buffer->callback_param);
    }
} //End get_system_status_callback()


//=============================================================================
//=============================================================================
static void send_next_command(void)
{
    btx_ti_exec_script_buffer_t* buffer = m_buffer;
    const bt_byte* packet;
    bt_hci_command_t* cmd;

    packet = buffer->script->packets[buffer->current_packet];
    cmd = bt_hci_alloc_canned_command(packet, &send_command_callback);
    if (cmd != NULL)
    {
        bt_hci_send_cmd(cmd);
    }
    else
    {
        buffer->callback(BT_FALSE, buffer, buffer->callback_param);
    }
}


//=============================================================================
//=============================================================================
static void send_command_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt)
{
    btx_ti_exec_script_buffer_t* buffer = m_buffer;

    if (status != HCI_ERR_SUCCESS)
        buffer->callback(BT_FALSE, buffer, buffer->callback_param);

    buffer->current_packet++;
    if (buffer->current_packet < buffer->script->packet_count)
    {
        send_next_command();
    }
    else
    {
        buffer->callback(BT_TRUE, buffer, buffer->callback_param);
    }
}
