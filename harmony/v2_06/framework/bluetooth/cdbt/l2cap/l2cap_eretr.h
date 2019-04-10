/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*
* SEARAN LLC is the exclusive licensee and developer of dotstack with
* all its modifications and enhancements.
*
* Contains proprietary and confidential information of CandleDragon and
* may not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2009, 2010, 2011 CandleDragon. All Rights Reserved.
*******************************************************************************/

#ifndef __L2CAP_ERETR_H
#define __L2CAP_ERETR_H

#ifdef __cplusplus
extern "C" {
#endif

#define SET_FRAME_TYPE(ctl, type)    ctl = ((ctl & ~1) | (type & 1))
#define SET_TX_SEQ(ctl, seq)         ctl = ((ctl & ~0x7E) | ((seq & 0x3F) << 1))
#define SET_S_FUNCTION(ctl, s)       ctl = ((ctl & ~0x0C) | ((s & 0x3) << 2))
#define SET_P_BIT(ctl, pbit)         ctl = ((ctl & ~0x10) | ((pbit & 1) << 4))
#define SET_F_BIT(ctl, fbit)         ctl = ((ctl & ~0x80) | ((fbit & 1) << 7))
#define SET_REQ_SEQ(ctl, seq)        ctl = ((ctl & ~0x3F00) | ((seq & 0x3F) << 8))
#define SET_SAR(ctl, sar)            ctl = ((ctl & ~0xC000) | ((seq & 0x3) << 14))

#define GET_FRAME_TYPE(ctl)          (ctl & 1)
#define GET_TX_SEQ(ctl)              ((ctl & 0x7E) >> 1)
#define GET_S_FUNCTION(ctl)          ((ctl & 0x0C) >> 2)
#define GET_P_BIT(ctl)               ((ctl & 0x10) >> 4)
#define GET_F_BIT(ctl)               ((ctl & 0x80) >> 7)
#define GET_REQ_SEQ(ctl)             ((ctl & 0x3F00) >> 8)
#define GET_SAR(ctl)                 ((ctl & 0xC000) >> 14)

typedef enum _bt_l2cap_eretr_xmit_event_e
{
	L2CAP_ERETR_EVENT_DATA_REQUEST,
	L2CAP_ERETR_EVENT_LOCAL_BUSY_DETECTED,
	L2CAP_ERETR_EVENT_LOCAL_BUSY_CLEAR,
	L2CAP_ERETR_EVENT_RECV_REQSEQ_AND_FBIT,
	L2CAP_ERETR_EVENT_RECV_FBIT,
	L2CAP_ERETR_EVENT_RETRANSMIT_TIMER_EXPIRED,
	L2CAP_ERETR_EVENT_MONITOR_TIMER_EXPIRED,
} bt_l2cap_eretr_xmit_event_e;

typedef struct _bt_l2cap_xmit_event_param_t
{
	bt_l2cap_eretr_xmit_event_e ev;
	bt_l2cap_mgr_p pmgr;
	bt_l2cap_channel_t* pch;
	union
	{
		struct
		{
			bt_byte req_seq;
			bt_byte fbit;
		} recv_reqseq_and_fbit;

		struct
		{
			bt_byte fbit;
		} recv_bfit;

		struct
		{
			bt_l2cap_frame_desc_t* frame;
		} data_request;
	} params;

} bt_l2cap_xmit_event_param_t;

void _bt_l2cap_eretr_recv(bt_l2cap_mgr_p pmgr, bt_l2cap_channel_t* pch, bt_byte* pdata, bt_int len);

bt_bool _bt_l2cap_send_ack(bt_l2cap_channel_t* pch, bt_byte pbit, bt_byte fbit);
bt_bool _bt_l2cap_send_rej(bt_l2cap_channel_t* pch, bt_byte pbit, bt_byte fbit);
bt_bool _bt_l2cap_send_rr_or_rnr(bt_l2cap_channel_t* pch, bt_byte pbit, bt_byte fbit);
bt_bool _bt_l2cap_send_i_or_rr_or_rnr(bt_l2cap_channel_t* pch, bt_byte pbit, bt_byte fbit);
bt_bool _bt_l2cap_send_rr(bt_l2cap_channel_t* pch, bt_byte pbit, bt_byte fbit);
bt_bool _bt_l2cap_send_rnr(bt_l2cap_channel_t* pch, bt_byte pbit, bt_byte fbit);

void _bt_l2cap_recv_req_seq_and_fbit(bt_l2cap_mgr_p pmgr, bt_l2cap_channel_t* pch, bt_byte req_seq, bt_byte fbit);

bt_bool _bt_l2cap_eretr_send_data(bt_l2cap_channel_t* pch, bt_byte* data, bt_int len, bt_l2cap_send_data_callback_fp cb, void* cb_param);
bt_bool _bt_l2cap_eretr_send_smart_data(bt_l2cap_channel_t* pch, bt_packet_t* packet, bt_int len, bt_l2cap_send_data_callback_fp cb, void* cb_param);
bt_bool _bt_l2cap_eretr_retr_frames(bt_l2cap_channel_t* pch, bt_byte fbit);
bt_bool _bt_l2cap_eretr_send_pending_frames(bt_l2cap_channel_t* pch, bt_byte fbit);
bt_bool _bt_l2cap_eretr_handle_xmit_event(bt_l2cap_xmit_event_param_t* param);

#define _bt_l2cap_start_retr_timer(pch)						pch->ext->retr_timer_start_time = L2CAP_RETR_TIMEOUT
#define _bt_l2cap_start_retr_timer_if_not_running(pch)		{ if (pch->ext->retr_timer_start_time == 0) pch->ext->retr_timer_start_time = L2CAP_RETR_TIMEOUT; }
#define _bt_l2cap_stop_retr_timer(pch)						pch->ext->retr_timer_start_time = 0
#define _bt_l2cap_start_monitor_timer(pch)					pch->ext->monitor_timer_start_time = L2CAP_MONITOR_TIMEOUT
#define _bt_l2cap_start_monitor_timer_if_not_running(pch)	{ if (pch->ext->monitor_timer_start_time == 0) pch->ext->monitor_timer_start_time = L2CAP_MONITOR_TIMEOUT; }
#define _bt_l2cap_stop_monitor_timer(pch)					pch->ext->monitor_timer_start_time = 0

bt_uint _bt_l2cap_fcs(const bt_byte* message, bt_int len, bt_uint start_crc);

#ifdef __cplusplus
}
#endif

#endif // __L2CAP_ERETR_H
