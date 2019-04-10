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

#define BT_CONTROLLER_CC2560_FW_VERSION_X    2
#define BT_CONTROLLER_CC2560_FW_VERSION_Z    31

//Modified for CC2564B
#define BT_CONTROLLER_CC2564_FW_VERSION_X    7
#define BT_CONTROLLER_CC2564_FW_VERSION_Z    16

typedef struct _btx_ti_controller_init_proc_t
{
    bt_byte fw_version_x;
    bt_byte fw_version_z;
    const btx_ti_script_t* (*get_scriopt_fp)(void);

} btx_ti_controller_init_proc_t;

static btx_ti_controller_init_proc_t _scripts[] =
{
#if BT_CONTROLLER == BT_CONTROLLER_CC2560
    { 2, 31, &btx_ti_get_script__CC2560_ServicePack },
#endif
    { 6, 15, &btx_ti_get_script__CC2564_ServicePack },
    { 7, 16, &btx_ti_get_script__CC2564B_ServicePack },
};

static btx_ti_controller_init_proc_t _ble_scripts[] =
{
    { 6, 15, &btx_ti_get_script__CC2564_BLE_Init },
    { 7, 16, &btx_ti_get_script__CC2564B_BLE_Init },
};

static btx_ti_exec_script_buffer_t* m_buffer;
static btx_ti_controller_init_proc_t* m_scripts;
static bt_int m_script_count;

static void get_fw_version_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt);

void btx_ti_init_controller(
    btx_ti_exec_script_buffer_t* buffer,
    btx_ti_completion_callback_fp callback,
    void* callback_param)
{
    bt_hci_command_t* cmd;

    memset(buffer, 0, sizeof(btx_ti_exec_script_buffer_t));
    buffer->callback = callback;
    buffer->callback_param = callback_param;
    m_buffer = buffer;
    m_scripts = _scripts;
    m_script_count = sizeof(_scripts) / sizeof(btx_ti_controller_init_proc_t);
    //printf("Bluetooth btx_ti_init_controller %d scripts\n", m_script_count);

    cmd = bt_hci_alloc_command(0xFE1F, &get_fw_version_callback); // HCI_VS_Get_System_Status
    bt_hci_send_cmd(cmd);
}

static void get_fw_version_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt)
{
    bt_int i;

    for (i = 0; i < m_script_count; i++)
    {
        btx_ti_controller_init_proc_t* p = &m_scripts[i];
        if (evt->params[4] == p->fw_version_x && evt->params[5] == p->fw_version_z)
        {
            //printf("Bluetooth script %d, packet %d, count %d\n", 
            //  i, m_buffer->current_packet, m_buffer->script->packet_count);
            btx_ti_exec_script(
                p->get_scriopt_fp(),
                m_buffer,
                m_buffer->callback,
                m_buffer->callback_param);

            return;
        }

    }

    m_buffer->callback(BT_FALSE, m_buffer, m_buffer->callback_param);
}

void btx_ti_init_ble_controller(
    btx_ti_exec_script_buffer_t* buffer,
    btx_ti_completion_callback_fp callback,
    void* callback_param)
{
    bt_hci_command_t* cmd;

    memset(buffer, 0, sizeof(btx_ti_exec_script_buffer_t));
    buffer->callback = callback;
    buffer->callback_param = callback_param;
    m_buffer = buffer;
    m_scripts = _ble_scripts;
    m_script_count = sizeof(_ble_scripts) / sizeof(btx_ti_controller_init_proc_t);

    cmd = bt_hci_alloc_command(0xFE1F, &get_fw_version_callback); // HCI_VS_Get_System_Status
    bt_hci_send_cmd(cmd);
}
