/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#include "cdbt/bt/bt_std.h"
#include "cdbt/bt/bt_private.h"
#include "cdbt/bt/bt_hcitr.h"
#include "cdbt/hci/hci.h"
#include "cdbt/extra/csr/csr.h"

static btx_csr_exec_script_buffer_t* _buffer;
static const btx_csr_get_script_fp* _scripts;
static bt_int _script_count;

static void read_local_version_info_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt);

void btx_csr_patch_controller(
	const btx_csr_get_script_fp* scripts,
	bt_int script_count,
	btx_csr_exec_script_buffer_t* buffer,
	btx_csr_exec_script_callback_fp callback,
	void* callback_param)
{
	bt_hci_command_t* cmd;

	memset(buffer, 0, sizeof(btx_csr_exec_script_buffer_t));
	buffer->callback = callback;
	buffer->callback_param = callback_param;
	_buffer = buffer;
	_scripts = scripts;
	_script_count = script_count;

	cmd = bt_hci_alloc_command(HCI_READ_LOCAL_VERSION_INFORMATION, &read_local_version_info_callback);
	bt_hci_send_cmd(cmd);
}

static void read_local_version_info_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt)
{
	if (status == HCI_ERR_SUCCESS)
	{
		bt_int offset = 5;
		bt_uint revision = 0;
		bt_int i;

		bt_hci_get_evt_param_uint(evt, &revision, &offset);

		for (i = 0; i < _script_count; i++)
		{
			const btx_csr_script_t* script = _scripts[i]();
			if (revision == script->revision)
			{
				btx_csr_exec_script(
					script,
					_buffer,
					_buffer->callback,
					_buffer->callback_param);

				return;
			}
		}
	}

	_buffer->callback(BT_FALSE, _buffer);
}
