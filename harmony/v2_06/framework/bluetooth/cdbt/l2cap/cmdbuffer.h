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

#ifndef __L2CAP_CMDBUFFER_H
#define __L2CAP_CMDBUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

void bt_l2cap_init_cmd_buffers(void);
bt_l2cap_cmd_header_p bt_l2cap_alloc_cmd_buffer(bt_byte code);
void bt_l2cap_free_cmd_buffer(void* p);

bt_l2cap_cmd_reject_p bt_l2cap_alloc_cmd_reject(void);
bt_l2cap_cmd_connection_req_p bt_l2cap_alloc_cmd_connection_req(void);
bt_l2cap_cmd_connection_res_p bt_l2cap_alloc_cmd_connection_res(void);
bt_l2cap_cmd_config_req_p bt_l2cap_alloc_cmd_config_req(void);
bt_l2cap_cmd_config_res_p bt_l2cap_alloc_cmd_config_res(void);
bt_l2cap_cmd_disconnection_req_p bt_l2cap_alloc_cmd_disconnection_req(void);
pcmd_disconnection_res bt_l2cap_alloc_cmd_disconnection_res(void);
bt_l2cap_cmd_info_req_p bt_l2cap_alloc_cmd_info_req(void);
bt_l2cap_cmd_info_res_p bt_l2cap_alloc_cmd_info_res(void);
bt_l2cap_cmd_echo_req_p bt_l2cap_alloc_cmd_echo_req(void);
bt_l2cap_cmd_echo_res_p bt_l2cap_alloc_cmd_echo_res(void);
bt_l2cap_cmd_conn_param_update_req_t* bt_l2cap_alloc_cmd_conn_param_update_req(void);
bt_l2cap_cmd_conn_param_update_res_t* bt_l2cap_alloc_cmd_conn_param_update_res(void);

#ifdef __cplusplus
}
#endif

#endif // __L2CAP_CMDBUFFER_H
