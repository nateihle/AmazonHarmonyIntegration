/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2015 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#include "cdbt/bt/bt_std.h"
#include "cdbt/bt/bt_private.h"
#include "cdbt/bt/bt_hcitr.h"
#include "cdbt/extra/ti/ti.h"


enum m_state_e
{
    STATE_SEND_COMMAND,
    STATE_RECV_HEADER,
    STATE_RECV_DATA
};

static btx_ti_exec_script_oem_buffer_t* m_buffer;


static void send_next_command(void);
static void send_callback(void);
static void recv_callback(bt_uint len);


void btx_ti_exec_script_oem(const btx_ti_script_t* script,
                            btx_ti_exec_script_oem_buffer_t* buffer,
                            btx_ti_exec_script_oem_callback_fp callback,
                            void* callback_param)
{
    memset(buffer, 0, sizeof(btx_ti_exec_script_oem_buffer_t));
    buffer->script = script;
    buffer->callback = callback;
    buffer->callback_param = callback_param;
    m_buffer = buffer;

    send_next_command();
}


static void send_next_command(void)
{
    btx_ti_exec_script_oem_buffer_t* buffer = m_buffer;
    const bt_byte* packet = buffer->script->packets[buffer->current_packet];
    int packetLen = 4 + packet[3];

    buffer->state = STATE_SEND_COMMAND;
    bt_oem_send(packet, packetLen, &send_callback);
}


static void send_callback(void)
{
    btx_ti_exec_script_oem_buffer_t* buffer = m_buffer;

    buffer->state = STATE_RECV_HEADER;
    bt_oem_recv(buffer->rx_buffer, 3, &recv_callback);
}


static void recv_callback(bt_uint len)
{
    btx_ti_exec_script_oem_buffer_t* buffer = m_buffer;

    if (buffer->state == STATE_RECV_HEADER)
    {
        BT_ASSERT(buffer->rx_buffer[2] <= BTX_TI_EXEC_SCRIPT_OEM_RX_BUFFER_SIZE);

        buffer->state = STATE_RECV_DATA;
        bt_oem_recv(buffer->rx_buffer, buffer->rx_buffer[2], &recv_callback);
    }
    else if (buffer->state == STATE_RECV_DATA)
    {
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
}
