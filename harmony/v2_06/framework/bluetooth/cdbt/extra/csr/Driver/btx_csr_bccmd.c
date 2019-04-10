/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#include "cdbt/bt/bt_std.h"
#include "cdbt/bt/bt_private.h"
#include "cdbt/hci/hci.h"
#include "cdbt/hci/hci_signal.h"
#include "cdbt/extra/csr/csr.h"


static bt_uint                         mBccmdSequenceNumber;
static bt_hci_ctrl_listener_t          _listener;
static bt_hci_cmd_callback_fp          _wr_callback;
static void*                           _wr_callback_param;
static btx_csr_get_var_callback_fp     _get_var_callback;
static btx_csr_set_var_callback_fp     _set_var_callback;
static btx_csr_get_ps_var_callback_fp  _get_ps_var_callback;
static btx_csr_bccmd_listener_t*       _bccmd_listeners;

static void _btx_cmd_sent_callback(bt_uint event_id, bt_hci_command_t* cmd, void* cb_param);
static void _btx_csr_get_req_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt);
static void _btx_csr_set_req_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt);

static void _btx_csr_get_cached_temperature_handler(bt_hci_command_t* cmd, btx_csr_bccmd_header_t* var_value);
static void _btx_csr_get_rssi_acl_handler(bt_hci_command_t* cmd, btx_csr_bccmd_header_t* var_value);
static void _btx_csr_get_pio_handler(bt_hci_command_t* cmd, btx_csr_bccmd_header_t* var_value);

static void _btx_csr_get_ps_var_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt);

static void _btx_csr_hci_vendor_specific_event_handler(bt_hci_event_t* evt);
static void _btx_csr_notify_bccmd_listeners(btx_csr_bccmd_header_t* message);

void _btx_csr_bccmd_init(void)
{
	_bccmd_listeners = NULL;

	_listener.event_id = HCI_EVT_CMD_SEND_FINISHED;
	_listener.callback.cmd_event = &_btx_cmd_sent_callback;
	_listener.callback_param = NULL;

	bt_hci_set_vendor_specific_event_handler(&_btx_csr_hci_vendor_specific_event_handler);
}

bt_bool _btx_csr_send_cmd(bt_hci_command_t* cmd)
{
	bt_hci_ctrl_register_listener(&_listener);
	return bt_hci_send_cmd(cmd);

}

bt_bool btx_csr_register_bccmd_listener(btx_csr_bccmd_listener_t* listener)
{
	btx_csr_bccmd_listener_t* l;

	if (!listener->callback)
	{
		LOG("btx_csr_register_bccmd_listener: Invalid parameter");
		return FALSE;
	}

	l = bt_q_get_head((bt_queue_element_t**)&_bccmd_listeners, FALSE);
	while (l)
	{
		if (l == listener)
		{
			LOG("btx_csr_register_bccmd_listener: Listener already registered");
			return FALSE;
		}

		l = l->next_listener;
	}

	bt_q_add((bt_queue_element_t**)&_bccmd_listeners, listener);

	return TRUE;
}

void btx_csr_unregister_bccmd_listener(btx_csr_bccmd_listener_t* listener)
{
	bt_q_remove((bt_queue_element_t**)&_bccmd_listeners, listener);
}

bt_hci_command_t* btx_csr_alloc_bccmd_setreq(
		bt_uint var_id,
		bt_uint data_word_count,
		bt_hci_cmd_callback_fp callback,
		void* callback_param)
{
	bt_hci_command_t* cmd;
	
	cmd = bt_hci_alloc_command(HCI_OPCODE(OGF_VENDOR, 0x0000), callback);
	if (cmd)
	{
		cmd->callback_param = callback_param;

		// Add CSR payload descriptor:
		//    bit 7: Last fragment = 1
		//    bit 6: First fragment = 1
		//    bits 5-0 Channel ID = 2 (BCCMD)
		bt_hci_add_param_byte(cmd, 0xC2);
		
		// SETREQ message
		bt_hci_add_param_uint(cmd, CSR_MSG_TYPE_SETREQ);
		
		// BCCMD length (5 words of header and data words)
		bt_hci_add_param_uint(cmd, 5 + data_word_count);
		
		// Sequence number
		bt_hci_add_param_uint(cmd, mBccmdSequenceNumber++);
		
		// Variable ID
		bt_hci_add_param_uint(cmd, var_id);
		
		// Status (must be 0 in SETREQ)
		bt_hci_add_param_uint(cmd, 0);
	}
	
	return cmd;
}

bt_hci_command_t* btx_csr_alloc_bccmd_getreq(
	bt_uint var_id,
	bt_uint data_word_count,
	bt_hci_cmd_callback_fp callback,
	void* callback_param)
{
	bt_hci_command_t* cmd;

	cmd = bt_hci_alloc_command(HCI_OPCODE(OGF_VENDOR, 0x0000), callback);
	if (cmd)
	{
		cmd->callback_param = callback_param;

		// Add CSR payload descriptor:
		//    bit 7: Last fragment = 1
		//    bit 6: First fragment = 1
		//    bits 5-0 Channel ID = 2 (BCCMD)
		bt_hci_add_param_byte(cmd, 0xC2);

		// GETREQ message
		bt_hci_add_param_uint(cmd, CSR_MSG_TYPE_GETREQ);

		// BCCMD length (5 words of header and data words)
		bt_hci_add_param_uint(cmd, 5 + data_word_count);

		// Sequence number
		bt_hci_add_param_uint(cmd, mBccmdSequenceNumber++);

		// Variable ID
		bt_hci_add_param_uint(cmd, var_id);

		// Status (must be 0 in SETREQ)
		bt_hci_add_param_uint(cmd, 0);
	}

	return cmd;
}

bt_hci_command_t* btx_csr_alloc_bccmd_getresp(
	bt_uint var_id,
	bt_uint seq_no,
	bt_uint data_word_count,
	bt_uint status,
	bt_hci_cmd_callback_fp callback,
	void* callback_param)
{
	bt_hci_command_t* cmd;

	cmd = bt_hci_alloc_command(HCI_OPCODE(OGF_VENDOR, 0x0000), callback);
	if (cmd)
	{
		cmd->callback_param = callback_param;

		// Add CSR payload descriptor:
		//    bit 7: Last fragment = 1
		//    bit 6: First fragment = 1
		//    bits 5-0 Channel ID = 2 (BCCMD)
		bt_hci_add_param_byte(cmd, 0xC2);

		// GETRESP message
		bt_hci_add_param_uint(cmd, CSR_MSG_TYPE_GETRESP);

		// BCCMD length (5 words of header and data words)
		bt_hci_add_param_uint(cmd, 5 + data_word_count);

		// Sequence number
		bt_hci_add_param_uint(cmd, seq_no);

		// Variable ID
		bt_hci_add_param_uint(cmd, var_id);

		// Status
		bt_hci_add_param_uint(cmd, status);
	}

	return cmd;
}

bt_hci_command_t* btx_csr_alloc_hq_getresp(
	bt_uint var_id,
	bt_uint seq_no,
	bt_uint data_word_count,
	bt_uint status,
	bt_hci_cmd_callback_fp callback,
	void* callback_param)
{
	bt_hci_command_t* cmd;

	cmd = bt_hci_alloc_command(HCI_OPCODE(OGF_VENDOR, 0x0000), callback);
	if (cmd)
	{
		cmd->callback_param = callback_param;

		// Add CSR payload descriptor:
		//    bit 7: Last fragment = 1
		//    bit 6: First fragment = 1
		//    bits 5-0 Channel ID = 3 (HQ)
		bt_hci_add_param_byte(cmd, 0xC3);

		// GETRESP message
		bt_hci_add_param_uint(cmd, CSR_MSG_TYPE_GETRESP);

		// BCCMD length (5 words of header and data words)
		bt_hci_add_param_uint(cmd, 5 + data_word_count);

		// Sequence number
		bt_hci_add_param_uint(cmd, seq_no);

		// Variable ID
		bt_hci_add_param_uint(cmd, var_id);

		// Status
		bt_hci_add_param_uint(cmd, status);
	}

	return cmd;
}


bt_bool btx_csr_set_ps_var(
		bt_uint ps_key,
		const bt_uint* value,
		bt_uint value_word_count,
		bt_hci_cmd_callback_fp callback)
{
	// Use default PS key store
	return btx_csr_set_ps_var_ex(ps_key, value, value_word_count, PS_DEFAULT, callback);
}

bt_bool btx_csr_set_ps_var_ex(
	bt_uint ps_key,
	const bt_uint* value,
	bt_uint value_word_count,
	bt_uint store,
	bt_hci_cmd_callback_fp callback)
{
	bt_hci_command_t* cmd;
	bt_uint i;

	cmd = btx_csr_alloc_bccmd_setreq(
		0x7003, // variable id for the PS command
		3 + value_word_count,
		callback, NULL);
	if (cmd)
	{
		// PS key
		bt_hci_add_param_uint(cmd, ps_key);

		// Data length (in words)
		bt_hci_add_param_uint(cmd, value_word_count);

		// PS key store
		bt_hci_add_param_uint(cmd, store);

		// Value
		for (i = 0; i < value_word_count; i++)
		{
			if (!bt_hci_add_param_uint(cmd, value[i]))
			{
				bt_hci_free_command(cmd);
				cmd = NULL;
				break;
			}
		}
	}

	if (cmd)
	{
		return _btx_csr_send_cmd(cmd);
	}
	else
	{
		return BT_FALSE;
	}
}

bt_bool btx_csr_get_ps_var(
	bt_uint ps_key,
	bt_uint value_word_count,
	btx_csr_get_ps_var_callback_fp callback,
	void* callback_param)
{
	// Use default PS key store
	return btx_csr_get_ps_var_ex(ps_key, value_word_count, PS_DEFAULT, callback, callback_param);
}

bt_bool btx_csr_get_ps_var_ex(
	bt_uint ps_key,
	bt_uint value_word_count,
	bt_uint store,
	btx_csr_get_ps_var_callback_fp callback,
	void* callback_param)
{
	bt_hci_command_t* cmd;
	bt_uint i;

	cmd = btx_csr_alloc_bccmd_getreq(
		0x7003, // variable id for the PS command
		3 + value_word_count,
		&_btx_csr_get_ps_var_callback, callback_param);
	if (cmd)
	{
		// PS key
		bt_hci_add_param_uint(cmd, ps_key);

		// Data length (in words)
		bt_hci_add_param_uint(cmd, value_word_count);

		// PS key store
		bt_hci_add_param_uint(cmd, store);

		// Add dummy value
		for (i = 0; i < value_word_count; i++)
		{
			if (!bt_hci_add_param_uint(cmd, 0))
			{
				bt_hci_free_command(cmd);
				cmd = NULL;
				break;
			}
		}
	}

	if (cmd)
	{
		_get_ps_var_callback = callback;
		return _btx_csr_send_cmd(cmd);
	}
	else
	{
		return BT_FALSE;
	}
}


bt_bool btx_csr_warm_reset(void)
{
	return btx_csr_warm_reset_ex(NULL, NULL);
}

bt_bool btx_csr_warm_reset_ex(
	bt_hci_cmd_callback_fp callback,
	void* callback_param)
{
	bt_hci_command_t* cmd;
	bt_uint i;

	cmd = btx_csr_alloc_bccmd_setreq(
		0x4002, // variable id for the Warm Reset command
		4,      // value length (must be at least 4 words)
		NULL, NULL);
	if (cmd)
	{
		// Add dummy value
		for (i = 0; i < 4; i++)
		{
			if (!bt_hci_add_param_uint(cmd, 0))
			{
				bt_hci_free_command(cmd);
				cmd = NULL;
				break;
			}
		}
	}

	if (cmd)
	{
		_wr_callback = callback;
		_wr_callback_param = callback_param;

		return _btx_csr_send_cmd(cmd);
	}
	else
	{
		return BT_FALSE;
	}
}

/*
if (_pcmd_being_sent->opcode == HCI_HOST_NUM_OF_COMPLETED_PACKETS) 
{
	bt_free_buffer(&_hci_cmd_mgr, _pcmd_being_sent);
}
else 
{	
	bt_q_add(&_resp_cq_head, _pcmd_being_sent);
}
*/

static void _btx_cmd_sent_callback(bt_uint event_id, bt_hci_command_t* cmd, void* cb_param)
{
	if (cmd->opcode == (bt_int)HCI_OPCODE(OGF_VENDOR, 0x0000))
	{
		bt_hci_ctrl_unregister_listener(&_listener);

		// CSR controllers do not report number of completed packets
		// for BCCMD commands so we have to update it manually right after
		// the command has been sent to the controller.
		_phci_ctrl->num_hci_cmd_packets++;


		// If command was sent over HQ channel the controller
		// will not send any response. Remove the command from the queue
		// and delete it.
		if (cmd->params_len < 0) // canned command
		{
			if ((cmd->params[4] & 0x3F) == 3)
			{
				bt_q_remove(&_resp_cq_head, cmd);

				if (cmd->callback)
					cmd->callback(HCI_ERR_SUCCESS, cmd, NULL);

				bt_free_buffer(&_hci_cmd_mgr, cmd);
			}
		}
		else
		{
			if ((*cmd->params & 0x3F) == 3)
			{
				bt_q_remove(&_resp_cq_head, cmd);
				
				if (cmd->callback)
					cmd->callback(HCI_ERR_SUCCESS, cmd, NULL);

				bt_free_buffer(&_hci_cmd_mgr, cmd);
			}
		}

		if (_wr_callback)
		{
			_wr_callback(HCI_ERR_SUCCESS, cmd, _wr_callback_param);
			_wr_callback = NULL;
		}
	}
}

//void (*bt_hci_cmd_listener_fp)(bt_int cmd_evcode, struct _bt_hci_command_s* cmd, void* cb_param);

bt_bool btx_csr_enable_tx(
	bt_bool enable, 
	bt_hci_cmd_callback_fp callback,
	void* callback_param)
{
	bt_hci_command_t* cmd;
	bt_uint i;

	cmd = btx_csr_alloc_bccmd_setreq(
		enable ? 0x4007 : 0x4008, // variable id for the Enable/Disable TX command
		4,                        // value length (must be at least 4 words)
		callback, callback_param);
	if (cmd)
	{
		// Add dummy value
		for (i = 0; i < 4; i++)
		{
			if (!bt_hci_add_param_uint(cmd, 0))
			{
				bt_hci_free_command(cmd);
				cmd = NULL;
				break;
			}
		}
	}

	if (cmd)
	{
		return _btx_csr_send_cmd(cmd);
	}
	else
	{
		return BT_FALSE;
	}
}

bt_bool btx_csr_get_var(
	bt_uint var_id,
	btx_csr_get_var_callback_fp callback,
	void* callback_param)
{
	bt_hci_command_t* cmd;
	bt_uint i;

	cmd = btx_csr_alloc_bccmd_getreq(
		var_id,
		4,                            // value length (must be at least 4 words)
		&_btx_csr_get_req_callback, callback_param);
	if (cmd)
	{
		// Add dummy value
		for (i = 0; i < 4; i++)
		{
			if (!bt_hci_add_param_uint(cmd, 0))
			{
				bt_hci_free_command(cmd);
				cmd = NULL;
				break;
			}
		}
	}

	if (cmd)
	{
		_get_var_callback = callback;
		return _btx_csr_send_cmd(cmd);
	}
	else
	{
		return BT_FALSE;
	}
}

bt_bool btx_csr_set_var(
	bt_uint var_id,
	const bt_uint* value,
	bt_uint value_word_count,
	btx_csr_set_var_callback_fp callback,
	void* callback_param)
{
	bt_hci_command_t* cmd;
	bt_uint i;
	bt_uint cmd_data_word_count;

	if (value_word_count < 4)
		cmd_data_word_count = 4;
	else
		cmd_data_word_count = value_word_count;

	cmd = btx_csr_alloc_bccmd_setreq(
		var_id,
		cmd_data_word_count,
		&_btx_csr_set_req_callback, NULL);

	if (cmd)
	{
		// Value
		for (i = 0; i < value_word_count; i++)
		{
			if (!bt_hci_add_param_uint(cmd, value[i]))
			{
				bt_hci_free_command(cmd);
				cmd = NULL;
				break;
			}
		}

		if (!cmd)
			return BT_FALSE;

		for (; i < cmd_data_word_count; i++)
		{
			if (!bt_hci_add_param_uint(cmd, 0))
			{
				bt_hci_free_command(cmd);
				cmd = NULL;
				break;
			}
		}
	}

	if (cmd)
	{
		_set_var_callback = callback;
		return _btx_csr_send_cmd(cmd);
	}
	else
	{
		return BT_FALSE;
	}
}

bt_bool btx_csr_get_pio(
	btx_csr_get_var_callback_fp callback,
	void* callback_param)
{
	return btx_csr_get_var(CSR_VARID_PIO, callback, callback_param);
}


bt_bool btx_csr_get_cached_temperature(
  btx_csr_get_var_callback_fp callback,
  void* callback_param)
{
	bt_hci_command_t* cmd;
	bt_uint i;

	cmd = btx_csr_alloc_bccmd_getreq(
		CSR_VARID_CACHED_TEMPERATURE, // variable id for the Cached Temperature command
		4,                            // value length (must be at least 4 words)
		&_btx_csr_get_req_callback, callback_param);
	if (cmd)
	{
		// Add dummy value
		for (i = 0; i < 4; i++)
		{
			if (!bt_hci_add_param_uint(cmd, 0))
			{
				bt_hci_free_command(cmd);
				cmd = NULL;
				break;
			}
		}
	}

	if (cmd)
	{
		_get_var_callback = callback;
		return _btx_csr_send_cmd(cmd);
	}
	else
	{
		return BT_FALSE;
	}
}

bt_bool btx_csr_get_rssi_acl(
	bt_hci_hconn_t hconn,
	btx_csr_get_var_callback_fp callback,
	void* callback_param)
{
	bt_hci_command_t* cmd;
	bt_uint i;

	cmd = btx_csr_alloc_bccmd_getreq(
		CSR_VARID_RSSI_ACL,          // variable id for the RSSI_ACL command
		4,                            // value length (must be at least 4 words)
		&_btx_csr_get_req_callback, callback_param);
	if (cmd)
	{
		if (!bt_hci_add_param_hconn(cmd, hconn))
		{
			bt_hci_free_command(cmd);
			return BT_FALSE;
		}

		// Add dummy value
		for (i = 0; i < 3; i++)
		{
			if (!bt_hci_add_param_uint(cmd, 0))
			{
				bt_hci_free_command(cmd);
				cmd = NULL;
				break;
			}
		}
	}

	if (cmd)
	{
		_get_var_callback = callback;
		return _btx_csr_send_cmd(cmd);
	}
	else
	{
		return BT_FALSE;
	}
}

bt_bool btx_csr_send_dsp_config_data(
	bt_uint total_config_blocks,
	bt_uint seq_no,
	bt_uint status,
	const bt_uint* data,
	bt_uint data_word_count,
	bt_hci_cmd_callback_fp callback,
	void* callback_param)
{
	bt_hci_command_t* cmd;
	bt_uint i;
	bt_uint cmd_data_word_count;

	if (data_word_count + 1 < 4)
		cmd_data_word_count = 4;
	else
		cmd_data_word_count = data_word_count + 1;

	cmd = btx_csr_alloc_hq_getresp(
		CSR_VARID_DSPMANAGER_CONFIG_REQUEST,
		seq_no,
		cmd_data_word_count,                            // value length (must be at least 4 words)
		status,
		callback, callback_param);

	if (cmd)
	{
		// Total number of blocks of configuration data
		if (!bt_hci_add_param_uint(cmd, total_config_blocks))
		{
			bt_hci_free_command(cmd);
			return BT_FALSE;
		}

		// Config data
		for (i = 0; i < data_word_count; i++)
		{
			if (!bt_hci_add_param_uint(cmd, data[i]))
			{
				bt_hci_free_command(cmd);
				cmd = NULL;
				break;
			}
		}

		if (!cmd)
			return BT_FALSE;

		i++;
		for (; i < cmd_data_word_count; i++)
		{
			if (!bt_hci_add_param_uint(cmd, 0))
			{
				bt_hci_free_command(cmd);
				cmd = NULL;
				break;
			}
		}
	}

	if (cmd)
	{
		return _btx_csr_send_cmd(cmd);
	}
	else
	{
		return BT_FALSE;
	}
}

static void _btx_csr_get_req_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt)
{
	btx_csr_bccmd_header_t var_value;
	bt_int offset = 0;
	bt_byte payload_descriptor;

	_zero_memory(&var_value, sizeof(var_value));

	bt_hci_get_evt_param_byte(evt, &payload_descriptor, &offset);
	bt_hci_get_evt_param_uint(evt, &var_value.type, &offset);
	bt_hci_get_evt_param_uint(evt, &var_value.len, &offset);
	bt_hci_get_evt_param_uint(evt, &var_value.seq_no, &offset);
	bt_hci_get_evt_param_uint(evt, &var_value.var_id, &offset);
	bt_hci_get_evt_param_uint(evt, &var_value.status, &offset);
	var_value.payload = evt->params + offset;
	var_value.len -= 5; // exclude header len which is 5 16-bit units

	if (var_value.status != GETRESP_OK)
	{
		_get_var_callback(var_value.status, var_value.var_id, (btx_csr_var_t*)&var_value, cmd->callback_param);
	}
	else
	{
		switch (var_value.var_id)
		{
			case CSR_VARID_CACHED_TEMPERATURE:
				_btx_csr_get_cached_temperature_handler(cmd, &var_value);
				break;

			case CSR_VARID_RSSI_ACL:
				_btx_csr_get_rssi_acl_handler(cmd, &var_value);
				break;

			case CSR_VARID_PIO:
			case CSR_VARID_PIO_DIRECTION_MASK:
			case CSR_VARID_PIO_PROTECT_MASK:
				_btx_csr_get_pio_handler(cmd, &var_value);
				break;

			default:
				_get_var_callback(GETRESP_OK, var_value.var_id, (btx_csr_var_t*)&var_value, cmd->callback_param);
		}
	}
}

static void _btx_csr_get_cached_temperature_handler(bt_hci_command_t* cmd, btx_csr_bccmd_header_t* var_value)
{
	btx_csr_cached_temperature_t temp;

	_readui(&temp.temperature, var_value->payload, var_value->len * 2, NULL);
	_get_var_callback(GETRESP_OK, var_value->var_id, (btx_csr_var_t*)&temp, cmd->callback_param);
}

static void _btx_csr_get_rssi_acl_handler(bt_hci_command_t* cmd, btx_csr_bccmd_header_t* var_value)
{
	btx_csr_rssi_acl_t rssi;
	bt_int offset = 0;

	_readi(&rssi.hconn, var_value->payload, var_value->len * 2, &offset);
	_readui(&rssi.rssi, var_value->payload, var_value->len * 2, &offset);
	_get_var_callback(GETRESP_OK, var_value->var_id, (btx_csr_var_t*)&rssi, cmd->callback_param);
}

static void _btx_csr_get_pio_handler(bt_hci_command_t* cmd, btx_csr_bccmd_header_t* var_value)
{
	btx_csr_pio_t pio;

	_readui(&pio.pio, var_value->payload, var_value->len * 2, NULL);
	_get_var_callback(GETRESP_OK, var_value->var_id, (btx_csr_var_t*)&pio, cmd->callback_param);
}

static void _btx_csr_set_req_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt)
{
	btx_csr_var_t var_value;
	bt_int offset = 0;
	bt_byte payload_descriptor;

	_zero_memory(&var_value, sizeof(var_value));

	bt_hci_get_evt_param_byte(evt, &payload_descriptor, &offset);
	bt_hci_get_evt_param_uint(evt, &var_value.message.type, &offset);
	bt_hci_get_evt_param_uint(evt, &var_value.message.len, &offset);
	bt_hci_get_evt_param_uint(evt, &var_value.message.seq_no, &offset);
	bt_hci_get_evt_param_uint(evt, &var_value.message.var_id, &offset);
	bt_hci_get_evt_param_uint(evt, &var_value.message.status, &offset);
	var_value.message.payload = evt->params + offset;
	var_value.message.len -= 5; // exclude header len which is 5 16-bit units

	if (var_value.message.status == GETRESP_OK)
	{
		offset = 0;

		switch (var_value.message.var_id)
		{
			case CSR_VARID_STREAM_GET_SINK:
			case CSR_VARID_STREAM_GET_SOURCE:
				_readui(&var_value.strm_get_sink.sink_id, var_value.message.payload, var_value.message.len * 2, NULL);
				break;

			case CSR_VARID_CREATE_OPERATOR_C:
				_readui(&var_value.create_operator.operator_id, var_value.message.payload, var_value.message.len * 2, &offset);
				if (var_value.create_operator.operator_id == 0)
				{
					_readui(&var_value.create_operator.item_count, var_value.message.payload, var_value.message.len * 2, &offset);
					_readui(&var_value.create_operator.skip_count, var_value.message.payload, var_value.message.len * 2, &offset);
					_readui(&var_value.create_operator.skip_flag, var_value.message.payload, var_value.message.len * 2, &offset);
				}
				break;

			case CSR_VARID_STREAM_CONNECT:
				_readui(&var_value.strm_connect.transform_id, var_value.message.payload, var_value.message.len * 2, NULL);
				break;
		}
	}

	_set_var_callback(var_value.message.status, var_value.message.var_id, &var_value, cmd->callback_param);
}

static void _btx_csr_get_ps_var_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt)
{
	btx_csr_bccmd_header_t var_value;
	bt_int offset = 0;
	bt_byte payload_descriptor;

	_zero_memory(&var_value, sizeof(var_value));

	bt_hci_get_evt_param_byte(evt, &payload_descriptor, &offset);
	bt_hci_get_evt_param_uint(evt, &var_value.type, &offset);
	bt_hci_get_evt_param_uint(evt, &var_value.len, &offset);
	bt_hci_get_evt_param_uint(evt, &var_value.seq_no, &offset);
	bt_hci_get_evt_param_uint(evt, &var_value.var_id, &offset);
	bt_hci_get_evt_param_uint(evt, &var_value.status, &offset);
	var_value.payload = evt->params + offset;
	var_value.len -= 5; // exclude header len which is 5 16-bit units

	if (var_value.status != GETRESP_OK)
	{
		_get_ps_var_callback(var_value.status, NULL, 0, cmd->callback_param);
	}
	else
	{
		bt_uint length = 0;

		offset += 2;
		bt_hci_get_evt_param_uint(evt, &length, &offset);
		offset += 2;

		_get_ps_var_callback(GETRESP_OK, length ? evt->params + offset : NULL, length, cmd->callback_param);
	}
}


static void _btx_csr_hci_vendor_specific_event_handler(bt_hci_event_t* evt)
{
	btx_csr_bccmd_header_t hdr;
	bt_byte payload_descriptor;
	bt_int offset = 0;
	bt_hci_command_t* cmd;

	bt_hci_get_evt_param_byte(evt, &payload_descriptor, &offset);
	bt_hci_get_evt_param_uint(evt, &hdr.type, &offset);
	bt_hci_get_evt_param_uint(evt, &hdr.len, &offset);
	bt_hci_get_evt_param_uint(evt, &hdr.seq_no, &offset);
	bt_hci_get_evt_param_uint(evt, &hdr.var_id, &offset);
	bt_hci_get_evt_param_uint(evt, &hdr.status, &offset);
	hdr.payload = evt->params + offset;
	hdr.len -= 5; // exclude header len which is 5 16-bit units

	if (hdr.type == CSR_MSG_TYPE_GETRESP)
	{
		// CSR controllers use vendor specific event (0xFF) to carry the BCCMD protocol.
		// The following makes sure that callback are called on corresponding vendor specific
		// commands.
		cmd = hci_cq_find_by_opcode(_resp_cq_head, HCI_OPCODE(OGF_VENDOR, 0x0000));
		if (cmd != NULL) 
		{
			btx_csr_bccmd_header_t cmd_hdr;

			if (cmd->params_len < 0) // canned command
			{
				bt_byte params_len;

				offset = 3;
				_readb(&params_len, cmd->params, 4, &offset);
				params_len += 4;
				_readb(&payload_descriptor, cmd->params, params_len, &offset);
				_readui(&cmd_hdr.type, cmd->params, params_len, &offset);
				_readui(&cmd_hdr.len, cmd->params, params_len, &offset);
				_readui(&cmd_hdr.seq_no, cmd->params, params_len, &offset);
				_readui(&cmd_hdr.var_id, cmd->params, params_len, &offset);
				_readui(&cmd_hdr.status, cmd->params, params_len, &offset);
			}
			else
			{
				offset = 0;
				bt_hci_get_param_byte(cmd, &payload_descriptor, &offset);
				bt_hci_get_param_int(cmd, (bt_int*)&cmd_hdr.type, &offset);
				bt_hci_get_param_int(cmd, (bt_int*)&cmd_hdr.len, &offset);
				bt_hci_get_param_int(cmd, (bt_int*)&cmd_hdr.seq_no, &offset);
				bt_hci_get_param_int(cmd, (bt_int*)&cmd_hdr.var_id, &offset);
				bt_hci_get_param_int(cmd, (bt_int*)&cmd_hdr.status, &offset);
			}

			if (cmd_hdr.var_id == hdr.var_id && cmd_hdr.seq_no == hdr.seq_no)
			{
				bt_q_remove(&_resp_cq_head, cmd);
				if (cmd->callback != NULL)
					cmd->callback(0, cmd, evt);

				bt_hci_free_command(cmd);

				_bt_hci_set_signal();
			}
			else
			{
				LOGINT("No matching SETREQ or GETREQ found, var id: 0x", hdr.var_id);
				LOGINT("No matching SETREQ or GETREQ found, seq no: 0x", hdr.seq_no);
			}
		}
		else
		{
			LOG("No matching SETREQ or GETREQ found");
		}
	}
	else
	{
		_btx_csr_notify_bccmd_listeners(&hdr);
	}
}

static void _btx_csr_notify_bccmd_listeners(btx_csr_bccmd_header_t* message) 
{
	btx_csr_bccmd_listener_t* listener = _bccmd_listeners;

	while (listener)
	{
		if (listener->callback)
			listener->callback(message, listener->callback_param);

		listener = listener->next_listener;
	}
}
