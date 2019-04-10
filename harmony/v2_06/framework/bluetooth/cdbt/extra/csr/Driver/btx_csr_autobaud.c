/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/


#include "cdbt/bt/bt_std.h"
#include "cdbt/bt/bt_private.h"
#include "cdbt/bt/bt_hcitr.h"
#include "cdbt/bt/bt_timer.h"
#include "cdbt/hci/hci.h"
#include "cdbt/extra/csr/csr.h"

#define MAX_BC7_AUTOBAUD_PACKETS	20

static const bt_byte AUTOBAUD_SEQUENCE[] = {0xCA, 0x88, 0x55, 0x68, 0x77, 0x34, 0x99 };

static const bt_byte BC7_AUTOBAUD_SEQUENCE[] = {0x81, 0x70, 0x51, 0x1F, 0x68, 0x34, 0x91, 0x80, 0xD9, 0x8F, 0x48, 0x34 };

// HCI command packet for the HCI_Reset command
static const bt_byte HCI_RESET_PACKET[] = {
	0x01,
	0x03, 0x0C, 0x00
};

static btx_csr_autobaud_buffer_t* _buffer;
static bt_int                     _packetsReamining; 
static bt_byte                    _interval; 

static void autobaud_sequence_callback(void);
static void packet_send_callback(void);
static void packet_recv_callback(bt_uint len);
static void timerCallback(void);
static void _btx_csr_autobaud(
	btx_csr_autobaud_buffer_t* buffer,
	btx_csr_autobaud_callback_fp callback,
	void* callback_param,
	const bt_byte* sequence, bt_uint sequence_len,
	bt_int max_packets, bt_byte interval);

void btx_csr_autobaud(
	btx_csr_autobaud_buffer_t* buffer,
	btx_csr_autobaud_callback_fp callback,
	void* callback_param)
{
	_btx_csr_autobaud(
		buffer, callback, callback_param,
		AUTOBAUD_SEQUENCE, sizeof(AUTOBAUD_SEQUENCE), 0, 0);
}

void btx_csr_bc7_sel_host_interface_h4(
	btx_csr_autobaud_buffer_t* buffer,
	bt_byte interval,
	btx_csr_autobaud_callback_fp callback,
	void* callback_param)
{
	_btx_csr_autobaud(
		buffer, callback, callback_param,
		BC7_AUTOBAUD_SEQUENCE, sizeof(BC7_AUTOBAUD_SEQUENCE), MAX_BC7_AUTOBAUD_PACKETS, interval);
}

static void _btx_csr_autobaud(
		btx_csr_autobaud_buffer_t* buffer,
		btx_csr_autobaud_callback_fp callback,
		void* callback_param,
		const bt_byte* sequence, bt_uint sequence_len,
		bt_int max_packets, bt_byte interval)
{
	memset(buffer, 0, sizeof(btx_csr_set_ps_vars_buffer_t));
	buffer->callback = callback;
	buffer->callback_param = callback_param;
	_buffer = buffer;
	_packetsReamining = max_packets;
	_interval = interval;

	// BT stack is not yet running so we can reuse its HCI send/receive buffers.
	buffer->recv_buffer = bt_hci_get_recv_buffer();
	buffer->recv_buffer_len = bt_hci_get_recv_buffer_len();
	
	// Start receiving packets.
	bt_hci_transport_recv_packet(buffer->recv_buffer, buffer->recv_buffer_len, &packet_recv_callback);
	
	// Send autobaud sequence
	bt_oem_send(sequence, sequence_len, &autobaud_sequence_callback);
}


static void autobaud_sequence_callback(void)
{
	if (_interval > 0)
	{
		_packetsReamining--;
		bt_oem_timer_set(BT_TIMER_L2CAP, _interval, &timerCallback);
	}
}


static void packet_send_callback(void)
{
	// nothing to do
}

static void timerCallback(void)
{
	// if _packetsReamining < 0 then we received a response from the controller
	// so stop trying
	if (_packetsReamining < 0)
		return;

	if (_packetsReamining)
	{
		// Send autobaud sequence again
		bt_oem_send(BC7_AUTOBAUD_SEQUENCE, sizeof(BC7_AUTOBAUD_SEQUENCE), &autobaud_sequence_callback);
	}
	else
	{
		_buffer->callback(BT_FALSE, _buffer);
	}
}

static void packet_recv_callback(bt_uint len)
{
	btx_csr_autobaud_buffer_t* buffer = _buffer;
	bt_byte* packet = buffer->recv_buffer;

	if (_packetsReamining > 0)
	{
		_packetsReamining = -1;
		bt_oem_timer_clear(BT_TIMER_L2CAP);
	}

	if (packet[0] == 0x04) // HCI Event
	{
		if (packet[1] == HCI_EVT_COMMAND_COMPLETE)
		{
			_buffer->callback(BT_TRUE, _buffer);
		}
		else
		{
			bt_hci_transport_recv_packet(buffer->recv_buffer, buffer->recv_buffer_len, &packet_recv_callback);
			
			bt_hci_transport_send_packet(HCI_RESET_PACKET, sizeof(HCI_RESET_PACKET), &packet_send_callback);
		}
	}
	else
	{
		_buffer->callback(BT_FALSE, _buffer);
	}
}
