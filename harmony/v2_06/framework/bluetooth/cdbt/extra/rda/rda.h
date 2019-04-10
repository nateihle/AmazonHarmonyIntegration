/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
*
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __RDA_H_INCLUDED__
#define __RDA_H_INCLUDED__

typedef bt_ulong  _btx_rda_reg_value_t[2];

typedef struct _btx_rda_init_value_set_s
{
    bt_byte         memory_type;
	const _btx_rda_reg_value_t *values;
//    const bt_ulong* (*values)[2];
    bt_uint         value_count;
} btx_rda_init_value_set_t;

typedef struct _btx_rda_init_script_s
{
    const btx_rda_init_value_set_t* value_sets;
    bt_uint                         setCount;
} btx_rda_init_script_t;

typedef void (*btx_rda_exec_script_callback_fp)(bt_bool success);

void btx_rda_execute_script(const btx_rda_init_script_t* script, btx_rda_exec_script_callback_fp callback);
void btx_rda_set_uart_baud_rate(bt_ulong baudRate, btx_rda_exec_script_callback_fp callback);
void btx_rda_configure_uart(bt_ulong config, btx_rda_exec_script_callback_fp callback);
void btx_rda_enable_flow_controll(btx_rda_exec_script_callback_fp callback);

const btx_rda_init_script_t* btx_rda_get_init_script(void);

#endif // __RDA_H_INCLUDED__
