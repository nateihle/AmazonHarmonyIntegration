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

#ifndef __RFCOMM_CMD_QUEUE
#define __RFCOMM_CMD_QUEUE

#ifdef __cplusplus
extern "C" {
#endif

struct _bt_rfcomm_session_t;
struct _bt_rfcomm_command_t;
struct _bt_rfcomm_dlc_t;

struct _bt_rfcomm_command_t* rfcomm_cq_ack_cmd(struct _bt_rfcomm_dlc_t* pdlc, struct _bt_rfcomm_command_t* pres);
void rfcomm_cq_ack_mx_cmd(struct _bt_rfcomm_dlc_t* pdlc, struct _bt_rfcomm_command_t* pres);
struct _bt_rfcomm_command_t* rfcomm_cq_find_failed_pn(struct _bt_rfcomm_dlc_t* pdlc, struct _bt_rfcomm_command_t* dm);

void rfcomm_send_commands_from_queue(struct _bt_rfcomm_session_t *psess);

#ifdef __cplusplus
}
#endif

#endif // __RFCOMM_CMD_QUEUE
