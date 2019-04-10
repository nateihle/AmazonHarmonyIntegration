/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#include "cdbt/bt/bt_std.h"
#include "cdbt/bt/bt_private.h"
#include "cdbt/hci/hci.h"
#include "cdbt/extra/csr/csr.h"


static btx_csr_exec_script_buffer_t* m_buffer;
static btx_csr_exec_hq_script_buffer_t* m_hq_buffer;
static btx_csr_bccmd_listener_t _listener;

bt_bool _btx_csr_send_cmd(bt_hci_command_t* cmd);

static void send_next_command(void);
static void send_command_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt);
static void check_fw_revision(void);
static void read_local_version_info_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt);
static void _start_hq_script_callback(bt_uint status, bt_uint varId, btx_csr_var_t* varValue, void* callbackParam);
static void _hq_request_callback(btx_csr_bccmd_header_t* message, void* cb_param);


void btx_csr_exec_script(
		const btx_csr_script_t* script,
		btx_csr_exec_script_buffer_t* buffer,
		btx_csr_exec_script_callback_fp callback,
		void* callback_param)
{
	memset(buffer, 0, sizeof(btx_csr_exec_script_buffer_t));
	buffer->script = script;
	buffer->callback = callback;
	buffer->callback_param = callback_param;
	m_buffer = buffer;

	if (script->revision)
		check_fw_revision();
	else
		send_next_command();
}

static void check_fw_revision(void)
{
	bt_hci_command_t* cmd;

	cmd = bt_hci_alloc_command(HCI_READ_LOCAL_VERSION_INFORMATION, &read_local_version_info_callback);
	bt_hci_send_cmd(cmd);
}

static void read_local_version_info_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt)
{
	if (status == HCI_ERR_SUCCESS)
	{
		const btx_csr_script_t* script = m_buffer->script;
		bt_int offset = 5;
		bt_uint revision = 0;

		bt_hci_get_evt_param_uint(evt, &revision, &offset);


		// Check that the script is compatible with the hardware.
		if (revision != script->revision)
		{
			m_buffer->callback(BT_FALSE, m_buffer);
		}
		else if (script->packet_count)
		{
			// Proceed with executing the init script.	
			send_next_command();
		}
		else
		{
			m_buffer->callback(BT_TRUE, m_buffer);
		}
	}
	else
	{
		m_buffer->callback(BT_FALSE, m_buffer);
	}
}


static void send_next_command(void)
{
	btx_csr_exec_script_buffer_t* buffer = m_buffer;
	const bt_byte* packet;
	bt_hci_command_t* cmd;

	packet = buffer->script->packets[buffer->current_packet];
	cmd = bt_hci_alloc_canned_command(packet, &send_command_callback);
	if (cmd != NULL)
	{
		_btx_csr_send_cmd(cmd);
	}
	else
	{
		buffer->callback(BT_FALSE, buffer);
	}
}


static void send_command_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt)
{
	btx_csr_exec_script_buffer_t* buffer = m_buffer;

	buffer->current_packet++;
	if (buffer->current_packet < buffer->script->packet_count)
	{
		send_next_command();
	}
	else
	{
		buffer->callback(BT_TRUE, buffer);
	}
}

void btx_csr_exec_hq_script(
	const btx_csr_script_t* script,
	btx_csr_exec_hq_script_buffer_t* buffer,
	btx_csr_exec_hq_script_callback_fp callback,
	void* callback_param)
{
	bt_uint params[3];

	memset(buffer, 0, sizeof(btx_csr_exec_hq_script_buffer_t));
	buffer->script = script;
	buffer->callback = callback;
	buffer->callback_param = callback_param;
	buffer->success = TRUE;
	m_hq_buffer = buffer;

	_listener.callback = &_hq_request_callback;
	_listener.callback_param = NULL;
	btx_csr_register_bccmd_listener(&_listener);

	params[0] = 0;
	params[1] = 0;
	params[2] = 0;
	btx_csr_set_var(CSR_VARID_CREATE_OPERATOR_C, params, 3, &_start_hq_script_callback, NULL);
}

static void _start_hq_script_callback(bt_uint status, bt_uint varId, btx_csr_var_t* varValue, void* callbackParam)
{
	// Per documentation when CREATE_OPERATOR_C request
	// is issued with capability id 0 in order to load
	// DSP configuration data, the request is expected to
	// fail. So we do not check the completion status.
	// The controller sends a series of requests for 
	// configuration packets which are handled in _hq_request_callback.

	btx_csr_unregister_bccmd_listener(&_listener);
	m_hq_buffer->callback(m_hq_buffer->success, m_hq_buffer);
}

void btx_csr_init_hq_script(
	const btx_csr_script_t* script,
	btx_csr_exec_hq_script_buffer_t* buffer)
{
	memset(buffer, 0, sizeof(btx_csr_exec_hq_script_buffer_t));
	buffer->script = script;
}

bt_bool btx_csr_send_next_hq_script_packet(bt_uint seq_no, btx_csr_exec_hq_script_buffer_t* buffer)
{
	if (buffer->current_packet < buffer->script->packet_count)
	{
		const bt_byte* packet = buffer->script->packets[buffer->current_packet++];
		bt_byte packet_len = HCI_TRANSPORT_HEADER_LEN + HCI_CMD_HEADER_LEN + packet[3];
		bt_hci_command_t* cmd;

		memcpy(buffer->packet, packet, packet_len);
		buffer->packet[9] = (bt_byte)(seq_no & 0xFF);
		buffer->packet[10] = (bt_byte)((seq_no >> 8) & 0xFF);

		cmd = bt_hci_alloc_canned_command(buffer->packet, NULL);
		if (cmd != NULL)
		{
			_btx_csr_send_cmd(cmd);

			return TRUE;
		}
	}

	return FALSE;
}

static void _hq_request_callback(btx_csr_bccmd_header_t* message, void* cb_param)
{
	switch (message->var_id)
	{
		case CSR_VARID_DSPMANAGER_CONFIG_REQUEST:
		{
			LOG("CSR: CSR_VARID_DSPMANAGER_CONFIG_REQUEST");

			if (m_hq_buffer->current_packet < m_hq_buffer->script->packet_count)
			{
				const bt_byte* packet = m_hq_buffer->script->packets[m_hq_buffer->current_packet++];
				bt_byte packet_len = HCI_TRANSPORT_HEADER_LEN + HCI_CMD_HEADER_LEN + packet[3];
				bt_hci_command_t* cmd;

				memcpy(m_hq_buffer->packet, packet, packet_len);
				m_hq_buffer->packet[9] = (bt_byte)(message->seq_no & 0xFF);
				m_hq_buffer->packet[10] = (bt_byte)((message->seq_no >> 8) & 0xFF);

				cmd = bt_hci_alloc_canned_command(m_hq_buffer->packet, NULL);
				if (cmd != NULL)
				{
					LOGINT("CSR: sending DSP config packet: ", m_hq_buffer->current_packet);
					_btx_csr_send_cmd(cmd);
				}
				else
				{
					m_hq_buffer->success = FALSE;
				}
			}
			else
			{
				btx_csr_send_dsp_config_data(
					0,
					message->seq_no,
					GETRESP_BAD_REQ,
					NULL,
					0,
					NULL,
					NULL);
			}

			break;
		}
	}
}
