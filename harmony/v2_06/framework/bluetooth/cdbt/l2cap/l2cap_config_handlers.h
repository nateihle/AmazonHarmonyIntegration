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

#ifndef __L2CAP_CONFIG_HANDLERS_H
#define __L2CAP_CONFIG_HANDLERS_H

#ifndef BT_BLE_SINGLE_MODE
#define PROCESS_CONN_REQ      _process_conn_req
#define PROCESS_CONFIG_REQ    _process_config_req
#define PROCESS_DCONN_REQ     _process_dconn_req
#define PROCESS_INFO_REQ      _process_info_req

#define PROCESS_CONN_RES      _process_conn_res
#define PROCESS_CONFIG_RES    _process_config_res
#define PROCESS_DCONN_RES     _process_dconn_res
#define PROCESS_INFO_RES      _process_info_res

#define READ_CONN_REQUEST     &_read_conn_request
#define READ_CONN_RESPONSE    &_read_conn_response
#define READ_CONFIG_REQUEST   &_read_config_request
#define READ_CONFIG_RESPONSE  &_read_config_response
#define READ_DCONN_REQUEST    &_read_dconn_request
#define READ_DCONN_RESPONSE   &_read_dconn_response
#define READ_INFO_REQUEST     &_read_info_request
#define READ_INFO_RESPONSE    &_read_info_response

#define PACK_CONN_REQUEST     &_pack_conn_request
#define PACK_CONN_RESPONSE    &_pack_conn_response
#define PACK_CONFIG_REQUEST   &_pack_config_request
#define PACK_CONFIG_RESPONSE  &_pack_config_response
#define PACK_DCONN_REQUEST    &_pack_dconn_request
#define PACK_DCONN_RESPONSE   &_pack_dconn_response
#define PACK_INFO_REQUEST     &_pack_info_request
#define PACK_INFO_RESPONSE    &_pack_info_response

#else
#define PROCESS_CONN_REQ      _process_unknown_req
#define PROCESS_CONFIG_REQ    _process_unknown_req
#define PROCESS_DCONN_REQ     _process_unknown_req
#define PROCESS_INFO_REQ      _process_unknown_req

#define PROCESS_CONN_RES      _process_unknown_res
#define PROCESS_CONFIG_RES    _process_unknown_res
#define PROCESS_DCONN_RES     _process_unknown_res
#define PROCESS_INFO_RES      _process_unknown_res

#define READ_CONN_REQUEST     NULL
#define READ_CONN_RESPONSE    NULL
#define READ_CONFIG_REQUEST   NULL
#define READ_CONFIG_RESPONSE  NULL
#define READ_DCONN_REQUEST    NULL
#define READ_DCONN_RESPONSE   NULL
#define READ_INFO_REQUEST     NULL
#define READ_INFO_RESPONSE    NULL

#define PACK_CONN_REQUEST     NULL
#define PACK_CONN_RESPONSE    NULL
#define PACK_CONFIG_REQUEST   NULL
#define PACK_CONFIG_RESPONSE  NULL
#define PACK_DCONN_REQUEST    NULL
#define PACK_DCONN_RESPONSE   NULL
#define PACK_INFO_REQUEST     NULL
#define PACK_INFO_RESPONSE    NULL

#endif

const bt_l2cap_request_handler_fp _l2cap_request_handlers[] =
{
	&_process_unknown_req,
	&PROCESS_CONN_REQ,
	&PROCESS_CONFIG_REQ,
	&PROCESS_DCONN_REQ,
	&_process_echo_req,
	&PROCESS_INFO_REQ,
	&_process_unknown_req,
	&_process_unknown_req,
	&_process_unknown_req,
	&_process_conn_param_update_req,
};

const bt_l2cap_response_handler_fp _l2cap_response_handlers[] =
{
	&_process_reject,
	&PROCESS_CONN_RES,
	&PROCESS_CONFIG_RES,
	&PROCESS_DCONN_RES,
	&_process_echo_res,
	&PROCESS_INFO_RES,
	&_process_unknown_res,
	&_process_unknown_res,
	&_process_unknown_res,
	&_process_conn_param_update_res,
};

const bt_l2cap_cmd_parser_fp _l2cap_cmd_parsers[] =
{
	NULL,
	&_read_cmd_reject,
	READ_CONN_REQUEST,
	READ_CONN_RESPONSE,
	READ_CONFIG_REQUEST,
	READ_CONFIG_RESPONSE,
	READ_DCONN_REQUEST,
	READ_DCONN_RESPONSE,
	&_read_echo_request,
	&_read_echo_response,
	READ_INFO_REQUEST,
	READ_INFO_RESPONSE,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	&_read_conn_param_update_request,
	&_read_conn_param_update_response,
};

const bt_l2cap_cmd_assembler_fp _l2cap_cmd_assemblers[] =
{
	NULL,
	&_pack_cmd_reject,
	PACK_CONN_REQUEST,
	PACK_CONN_RESPONSE,
	PACK_CONFIG_REQUEST,
	PACK_CONFIG_RESPONSE,
	PACK_DCONN_REQUEST,
	PACK_DCONN_RESPONSE,
	&_pack_echo_request,
	&_pack_echo_response,
	PACK_INFO_REQUEST,
	PACK_INFO_RESPONSE,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	&_pack_conn_param_update_request,
	&_pack_conn_param_update_response,
};

#endif // __L2CAP_CONFIG_HANDLERS_H
