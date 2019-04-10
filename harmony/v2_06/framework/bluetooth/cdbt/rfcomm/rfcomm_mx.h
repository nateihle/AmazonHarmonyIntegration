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

#ifndef __RFCOMM_MX_H
#define __RFCOMM_MX_H

#ifdef __cplusplus
extern "C" {
#endif

#define RFCOMM_MX_MSG_PN	0x80
#define RFCOMM_MX_MSG_TEST	0x20
#define RFCOMM_MX_MSG_FCON	0xA0
#define RFCOMM_MX_MSG_FCOFF	0x60
#define RFCOMM_MX_MSG_MSC	0xE0
#define RFCOMM_MX_MSG_NSC	0x10
#define RFCOMM_MX_MSG_RPN	0x90
#define RFCOMM_MX_MSG_RLS	0x50

#define RFCOMM_MODEM_STATUS_FC	0x02
#define RFCOMM_MODEM_STATUS_RTC	0x04
#define RFCOMM_MODEM_STATUS_RTR	0x08
#define RFCOMM_MODEM_STATUS_IC	0x40
#define RFCOMM_MODEM_STATUS_DV	0x80

#define RFCOMM_LINE_STATUS_OVERRUN	x03
#define RFCOMM_LINE_STATUS_PARITY	x05
#define RFCOMM_LINE_STATUS_FRAMING	x09

bt_rfcomm_command_p _rfcomm_allocate_mx_cmd(bt_rfcomm_session_p psess, bt_rfcomm_cmd_callback_fp cb);

void _rfcomm_send_mx_nsc_response(bt_rfcomm_dlc_p pdlc, bt_rfcomm_command_p pcmd, bt_byte mx_msg_type);
void _rfcomm_send_mx_test_response(bt_rfcomm_dlc_p pdlc, bt_rfcomm_command_p pcmd, bt_int data_start, bt_int len);
void _rfcomm_mx_process_fc(bt_rfcomm_dlc_p pdlc, bt_rfcomm_command_p pcmd, bt_byte mx_msg_type);
void _rfcomm_mx_process_pn(bt_rfcomm_dlc_p pdlc, bt_rfcomm_command_p pcmd, bt_int data_start, bt_int len);
void _rfcomm_mx_process_rpn(bt_rfcomm_dlc_p pdlc, bt_rfcomm_command_p pcmd, bt_int data_start, bt_int len);
void _rfcomm_send_mx_msc_response(bt_rfcomm_dlc_p pdlc, bt_rfcomm_command_p pcmd, bt_int data_start, bt_int len);
void _rfcomm_send_mx_rls_response(bt_rfcomm_dlc_p pdlc, bt_rfcomm_command_p pcmd, bt_int data_start, bt_int len);

void _rfcomm_process_mx_msc_response(bt_rfcomm_dlc_p pdlc, bt_rfcomm_command_p pcmd, bt_int data_start, bt_int len);
void _rfcomm_process_mx_rls_response(bt_rfcomm_dlc_p pdlc, bt_rfcomm_command_p pcmd, bt_int data_start, bt_int len);
void _rfcomm_process_mx_fc_response(bt_rfcomm_dlc_p pdlc, bt_rfcomm_command_p pres);
void _rfcomm_process_mx_pn_response(bt_rfcomm_dlc_p pdlc, bt_rfcomm_command_p pres, bt_int data_start, bt_int len);

void _rfcomm_send_mx_rls_cmd(bt_rfcomm_dlc_p pdlc);
bt_bool rfcomm_send_mx_msc_cmd(bt_rfcomm_dlc_p pdlc, bt_rfcomm_cmd_callback_fp cb);
void _rfcomm_send_mx_fc_cmd(bt_rfcomm_dlc_p pdlc, bt_byte mx_msg_type);
void rfcomm_send_mx_pn_cmd(bt_rfcomm_dlc_p pdlc, bt_rfcomm_cmd_callback_fp cb);


#ifdef __cplusplus
}
#endif

#endif // __RFCOMM_MX_H
