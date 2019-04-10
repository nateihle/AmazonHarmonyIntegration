/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
*
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __ISSC_H_INCLUDED__
#define __ISSC_H_INCLUDED__

#define BTX_ISSC_HCI_HOST_EXTERNAL_CLOCK      HCI_OPCODE(OGF_VENDOR, 0x005B)
#define BTX_ISSC_HCI_HOST_SLEEP_CLOCK         HCI_OPCODE(OGF_VENDOR, 0x005C)
#define BTX_ISSC_HCI_HOST_WRITE_BT_ADDRESS    HCI_OPCODE(OGF_VENDOR, 0x000D)
#define BTX_ISSC_HCI_HOST_UART_SETTINGS       HCI_OPCODE(OGF_VENDOR, 0x005D)
#define BTX_ISSC_HCI_HOST_PCM_MODE            HCI_OPCODE(OGF_VENDOR, 0x005E)
#define BTX_ISSC_HCI_HOST_WARM_RESET          HCI_OPCODE(OGF_VENDOR, 0x005F)
#define BTX_ISSC_HCI_HOST_STACK_OPTIONS       HCI_OPCODE(OGF_VENDOR, 0x0010)

typedef enum {
	BTX_ISSC_SLEEP_CLOCK_RC1024 = 0x00,
	BTX_ISSC_SLEEP_CLOCK_EXTERNAL = 0X01,
} BTX_ISSC_SLEEP_CLOCK;

typedef enum {
	BTX_ISSC_STACK_OPTIONS_DISABLE_3M = 1 << 0, // "DISABLE_3M_PACKET_BY_HOST"
	BTX_ISSC_STACK_OPTIONS_LOW_CLOCK = 1 << 7, // "LOW_CLK_FROM_26M_19_2M_WITH_CRYSTAL"
} BTX_ISSC_STACK_OPTIONS;

typedef enum {
	BTX_ISSC_PCM_MODE_MASTER = 0x00,
	BTX_ISSC_PCM_MODE_SLAVE = 0x01,
} BTX_ISSC_PCM_MODE;

typedef enum {
	BTX_ISSC_HOST_PROTOCOL_H4 = 0x00,
	BTX_ISSC_HOST_PROTOCOL_H5 = 0x01,
	BTX_ISSC_HOST_PROTOCOL_BCSP = 0x02,
} BTX_ISSC_HOST_PROTOCOL;

typedef enum {
	BTX_ISSC_FLOW_CONTROL_DISABLED = 0x00,
	BTX_ISSC_FLOW_CONTROL_ENABLED = 0x02,
} BTX_ISSC_FLOW_CONTROL;

typedef struct _btx_issc_autobaud_buffer_t btx_issc_autobaud_buffer_t;

typedef void(*btx_issc_autobaud_callback_fp)(bt_bool success, btx_issc_autobaud_buffer_t* buffer);

struct _btx_issc_autobaud_buffer_t
{
	bt_byte* recv_buffer;
	bt_uint  recv_buffer_len;
	bt_int   packetsReamining;
	bt_byte  interval;
	const bt_byte* sequence;
	bt_uint sequence_len;
	btx_issc_autobaud_callback_fp callback;
	void* callback_param;
};

void btx_issc_sel_host_interface_h4(
	btx_issc_autobaud_buffer_t* buffer,
	bt_byte interval,
	btx_issc_autobaud_callback_fp callback,
	void* callback_param);

bt_bool btx_issc_host_external_clock(bt_ulong clock, bt_hci_cmd_callback_fp callback, void* callback_param);
bt_bool btx_issc_host_sleep_clock(BTX_ISSC_SLEEP_CLOCK source, bt_ulong clock, bt_hci_cmd_callback_fp callback, void* callback_param);
bt_bool btx_issc_write_bt_address(bt_bdaddr_t* bdaddr, bt_hci_cmd_callback_fp callback, void* callback_param);
bt_bool btx_issc_set_uart_baud_rate(
	bt_ulong baud_rate, 
	BTX_ISSC_HOST_PROTOCOL host_protocol, 
	BTX_ISSC_FLOW_CONTROL flow_control,
	bt_uint ack_timeout,
	bt_uint awake_duration,
	bt_hci_cmd_callback_fp callback, void* callback_param);
bt_bool btx_issc_host_pcm_mode(BTX_ISSC_PCM_MODE mode, bt_hci_cmd_callback_fp callback, void* callback_param);
bt_bool btx_issc_host_stack_options(bt_byte flags, bt_hci_cmd_callback_fp callback, void* callback_param);
bt_bool btx_issc_host_warm_reset(bt_hci_cmd_callback_fp callback, void* callback_param);

#endif // __ISSC_H_INCLUDED__
