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


static btx_csr_set_ps_vars_buffer_t* m_buffer;
static bt_uint m_store;


static void set_next_var(void);
static void set_ps_var_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt);


void btx_csr_set_ps_vars(
		const bt_uint* ps_vars,
		btx_csr_set_ps_vars_buffer_t* buffer,
		btx_csr_set_ps_vars_callback_fp callback,
		void* callback_param)
{
	memset(buffer, 0, sizeof(btx_csr_set_ps_vars_buffer_t));
	buffer->ps_vars = ps_vars;
	buffer->callback = callback;
	buffer->callback_param = callback_param;
	m_buffer = buffer;
	m_store = PS_DEFAULT;

	set_next_var();
}

void btx_csr_set_ps_vars_ex(
	const bt_uint* ps_vars,
	btx_csr_set_ps_vars_buffer_t* buffer,
	bt_uint store,
	btx_csr_set_ps_vars_callback_fp callback,
	void* callback_param)
{
	memset(buffer, 0, sizeof(btx_csr_set_ps_vars_buffer_t));
	buffer->ps_vars = ps_vars;
	buffer->callback = callback;
	buffer->callback_param = callback_param;
	m_buffer = buffer;
	m_store = store;

	set_next_var();
}

static void set_next_var(void)
{
	const bt_uint* ps_vars = m_buffer->ps_vars;
	bt_uint current_var = m_buffer->current_var;
	
	if (ps_vars[current_var] == 0)
	{
		m_buffer->callback(BT_TRUE, m_buffer);
	}
	else
	{
		if (btx_csr_set_ps_var_ex(ps_vars[current_var], &ps_vars[current_var+2], ps_vars[current_var+1], m_store, &set_ps_var_callback))
		{
			m_buffer->current_var += 2 + ps_vars[current_var+1];
		}
		else
		{
			m_buffer->callback(BT_FALSE, m_buffer);
		}
	}
}


static void set_ps_var_callback(bt_int status, bt_hci_command_t* cmd, bt_hci_event_t* evt)
{
	if (status == 0)
	{
		set_next_var();
	}
	else
	{
		m_buffer->callback(BT_FALSE, m_buffer);
	}
}

