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

#ifndef __L2CAP_COMMAND_H
#define __L2CAP_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#define L2CAP_CMD_RESERVED                      0x00
#define L2CAP_CMD_REJECT                        0x01
#define L2CAP_CMD_CONN_REQUEST                  0x02
#define L2CAP_CMD_CONN_RESPONSE                 0x03
#define L2CAP_CMD_CONFIG_REQUEST                0x04
#define L2CAP_CMD_CONFIG_RESPONSE               0x05
#define L2CAP_CMD_DCONN_REQUEST                 0x06
#define L2CAP_CMD_DCONN_RESPONSE                0x07
#define L2CAP_CMD_ECHO_REQUEST                  0x08
#define L2CAP_CMD_ECHO_RESPONSE                 0x09
#define L2CAP_CMD_INFO_REQUEST                  0x0A
#define L2CAP_CMD_INFO_RESPONSE                 0x0B
#define L2CAP_CMD_CONN_PARAM_UPDATE_REQUEST     0x12
#define L2CAP_CMD_CONN_PARAM_UPDATE_RESPONSE    0x13
#define L2CAP_CMD_LAST                          L2CAP_CMD_CONN_PARAM_UPDATE_RESPONSE

#define L2CAP_CMD_STATUS_PENDING			0
#define L2CAP_CMD_STATUS_WAITING_RESPONSE	1
#define L2CAP_CMD_STATUS_BEING_SENT			2

#define L2CAP_CMD_HEADER_LEN			4
#define L2CAP_CMD_DATA_LEN_CMD_REJECT		2
#define L2CAP_CMD_DATA_LEN_CONN_REQUEST		4
#define L2CAP_CMD_DATA_LEN_CONN_RESPONSE	8	
#define L2CAP_CMD_DATA_LEN_CONFIG_REQUEST	4
#define L2CAP_CMD_DATA_LEN_CONFIG_RESPONSE	6
#define L2CAP_CMD_DATA_LEN_DCONN_REQUEST	4
#define L2CAP_CMD_DATA_LEN_DCONN_RESPONSE	4
#define L2CAP_CMD_DATA_LEN_ECHO_REQUEST		0
#define L2CAP_CMD_DATA_LEN_ECHO_RESPONSE	0
#define L2CAP_CMD_DATA_LEN_INFO_REQUEST		2
#define L2CAP_CMD_DATA_LEN_INFO_RESPONSE	4

struct _bt_l2cap_cmd_header_t;
struct _bt_l2cap_mgr_s;
struct _bt_hci_conn_state_s;
struct _bt_l2cap_channel_t;

typedef void (*pf_l2cap_cmd_callback)(struct _bt_l2cap_mgr_s *ppmg, struct _bt_l2cap_cmd_header_t *pcmd);

typedef struct _bt_l2cap_cmd_header_t 
{
	struct _bt_l2cap_cmd_header_t* next_cmd;
	bt_byte code;
	bt_byte id;
	bt_byte status;
	bt_long send_time;
	bt_int rtx;
	bt_byte rtx_count;
	pf_l2cap_cmd_callback cb;
	struct _bt_hci_conn_state_s *phci_conn;
} bt_l2cap_cmd_header_t, *bt_l2cap_cmd_header_p;


#define L2CAP_REJECT_REASON_NOT_UNDERSTOOD   0x0000
#define L2CAP_REJECT_REASON_MTU_EXCEEDED     0x0001
#define L2CAP_REJECT_REASON_INVALID_CHANNEL  0x0002

typedef struct _bt_l2cap_cmd_reject_t
{
	bt_l2cap_cmd_header_t header;
	bt_uint reason;
	union 
	{
		bt_int actual_mtu;
		struct 
		{
			bt_id cid_local;
			bt_id cid_remote;
		} cid;
	} data;
} bt_l2cap_cmd_reject_t, *bt_l2cap_cmd_reject_p;

typedef union _bt_l2cap_cmd_reject_param_t
{
	bt_int actual_mtu;
	struct {
		bt_id cid_local;
		bt_id cid_remote;
	} cid;
} bt_l2cap_cmd_reject_param_t;

typedef struct _bt_l2cap_cmd_connection_req_t 
{
	bt_l2cap_cmd_header_t header;
	bt_int psm;
	bt_id cid_source;
} bt_l2cap_cmd_connection_req_t, *bt_l2cap_cmd_connection_req_p;

#define L2CAP_CONN_REQ_RESULT_SUCCESS                   0x0000
#define L2CAP_CONN_REQ_RESULT_PENDING                   0x0001
#define L2CAP_CONN_REQ_RESULT_INVALID_PSM               0x0002
#define L2CAP_CONN_REQ_RESULT_SECURITY_BLOCK            0x0003
#define L2CAP_CONN_REQ_RESULT_NO_RESOURCES              0x0004
#define L2CAP_CONN_REQ_RESULT_INVALID_SOURCE_CID        0x0006
#define L2CAP_CONN_REQ_RESULT_SRC_CID_ALREADY_ALLOCATED 0x0007

#define L2CAP_CONN_REQ_STATUS_NO_INFO                   0x0000
#define L2CAP_CONN_REQ_STATUS_AUTHENTICATION_PENDING    0x0001
#define L2CAP_CONN_REQ_STATUS_AUTHORIZATION_PENDING     0x0002

typedef struct _bt_l2cap_cmd_connection_res_t 
{
	bt_l2cap_cmd_header_t header;
	bt_id cid_destination;
	bt_id cid_source;
	bt_int result;
	bt_int status;
} bt_l2cap_cmd_connection_res_t, *bt_l2cap_cmd_connection_res_p;


typedef struct _bt_l2cap_cmd_disconnection_req_t 
{
	bt_l2cap_cmd_header_t header;
	bt_id cid_destination;
	bt_id cid_source;
} bt_l2cap_cmd_disconnection_req_t, *bt_l2cap_cmd_disconnection_req_p;

typedef struct _cmd_disconnection_res 
{
	bt_l2cap_cmd_header_t header;
	bt_id cid_destination;
	bt_id cid_source;
} cmd_disconnection_res, *pcmd_disconnection_res;

// configuration request/response

#define L2CAP_MAX_OPTIONS                4

#define L2CAP_OPTION_TYPE_MAX_MTU              0x01
#define L2CAP_OPTION_TYPE_FLASH_TIMEOUT        0x02
#define L2CAP_OPTION_TYPE_FLASH_QOS            0x03
#define L2CAP_OPTION_TYPE_FLASH_RFC            0x04

#define L2CAP_OPTION_TYPE_MAX_MTU_FLAG         0x01
#define L2CAP_OPTION_TYPE_FLASH_TIMEOUT_FLAG   0x02
#define L2CAP_OPTION_TYPE_FLASH_QOS_FLAG       0x04
#define L2CAP_OPTION_TYPE_FLASH_RFC_FLAG       0x08
#define L2CAP_OPTION_TYPE_UNKNOWN_FLAG         0x10

#define L2CAP_OPTION_LEN_MAX_MTU               2
#define L2CAP_OPTION_LEN_FLASH_TIMEOUT         2
#define L2CAP_OPTION_LEN_FLASH_QOS             22
#define L2CAP_OPTION_LEN_FLASH_RFC             9
#define L2CAP_OPTION_LEN_UNKNOWN               22
#define L2CAP_OPTION_LEN_UNKNOWN_DATA          9//21

typedef struct _bt_l2cap_option_max_mtu_t 
{
	bt_uint mtu;
} bt_l2cap_option_max_mtu_t;

typedef struct _bt_l2cap_option_flash_timeout_t 
{
	bt_int timeout;
} bt_l2cap_option_flash_timeout_t;


#define L2CAP_SERVICE_TYPE_NO_TRAFFIC       0x00
#define L2CAP_SERVICE_TYPE_BEST_EFFORT      0x01
#define L2CAP_SERVICE_TYPE_GUARANTEED       0x02

typedef struct _bt_l2cap_option_qos_t 
{
	bt_byte flags;
	bt_byte service_type;
	bt_long token_rate;
	bt_long token_bucket_size;
	bt_long peak_bandwidth;
	bt_long latency;
	bt_long delay_variation;
} bt_l2cap_option_qos_t;


#define L2CAP_RFC_BASIC                      0x00
#define L2CAP_RFC_RETRANSMISSION             0x01
#define L2CAP_RFC_FLOW_CONTROL               0x02
#define L2CAP_RFC_ENHANCED_RETRANSMISSION    0x03
#define L2CAP_RFC_STREAMING                  0x04

#define L2CAP_RFC_ERETR_TX_WINDOW            1
#define L2CAP_RFC_ERETR_TIMEOUT              (3 * 1000)  // 3 secs
#define L2CAP_RFC_MONITOR_TIMEOUT            (12 * 1000) // 12 secs

typedef struct _bt_l2cap_option_rfc_t
{
	bt_byte mode;
	bt_byte tx_window_size;
	bt_byte max_transmit;
	bt_int retr_timeout;
	bt_int monitor_timeout;
	bt_int mps;
} bt_l2cap_option_rfc_t;

typedef struct _bt_l2cap_option_unknown_t
{
	bt_byte data_len;
	bt_byte data[L2CAP_OPTION_LEN_UNKNOWN_DATA];
} bt_l2cap_option_unknown_t;

typedef struct _bt_l2cap_cfg_option_t 
{
	bt_byte type;
	union 
	{
		bt_l2cap_option_max_mtu_t max_mtu;
		bt_l2cap_option_flash_timeout_t flash_timeout;
//		option_qos qos;
		bt_l2cap_option_rfc_t rfc;
		bt_l2cap_option_unknown_t unknown;
	} opt;
} bt_l2cap_cfg_option_t, *bt_l2cap_cfg_option_p;

typedef struct _bt_l2cap_cmd_config_req_t 
{
	bt_l2cap_cmd_header_t header;
	bt_id cid_destination;
	bt_int flags;
	bt_int option_count;
	struct _bt_l2cap_channel_t* channel;
	bt_l2cap_cfg_option_t* options;
} bt_l2cap_cmd_config_req_t, *bt_l2cap_cmd_config_req_p;

#define L2CAP_CONFIG_RESULT_SUCCESS                 0x0000
#define L2CAP_CONFIG_RESULT_UNACCEPTABLE_PARAMETER  0x0001
#define L2CAP_CONFIG_RESULT_REJECTED                0x0002
#define L2CAP_CONFIG_RESULT_UNKNOWN_OPTION          0x0003


typedef struct _bt_l2cap_cmd_config_res_t 
{
	bt_l2cap_cmd_header_t header;
	bt_id cid_source;
	bt_int flags;
	bt_int result;
	bt_int option_count;
	struct _bt_l2cap_channel_t* channel;
	bt_l2cap_cfg_option_t* options;
} bt_l2cap_cmd_config_res_t, *bt_l2cap_cmd_config_res_p;

// echo request and response

#define L2CAP_ECHO_MAX_DATA_LEN  20

typedef struct _bt_l2cap_cmd_echo_req_t 
{
	bt_l2cap_cmd_header_t header;
	bt_int len;
	bt_byte data[L2CAP_ECHO_MAX_DATA_LEN];
} bt_l2cap_cmd_echo_req_t, *bt_l2cap_cmd_echo_req_p;

typedef struct _bt_l2cap_cmd_echo_res_t 
{
	bt_l2cap_cmd_header_t header;
	bt_int len;
	bt_byte data[L2CAP_ECHO_MAX_DATA_LEN];
} bt_l2cap_cmd_echo_res_t, *bt_l2cap_cmd_echo_res_p;

// information request

#define L2CAP_INFO_TYPE_CONNECTIONLESS_MTU  0x0001
#define L2CAP_INFO_TYPE_EXTENDED_SUPPORT    0x0002
#define L2CAP_INFO_TYPE_FIXED_CHANNELS      0x0003

typedef struct _bt_l2cap_cmd_info_req_t 
{
	bt_l2cap_cmd_header_t header;
	bt_int type;
} bt_l2cap_cmd_info_req_t, *bt_l2cap_cmd_info_req_p;

#define L2CAP_INFO_RESULT_SUCCESS           0x0000
#define L2CAP_INFO_NOT_SUPPORTED            0x0000     // REDFLAG: should it be non zero?

#define L2CAP_EXT_FLOW_CONTROL                  0x00000001
#define L2CAP_EXT_RETRANSMISSION                0x00000002
#define L2CAP_EXT_BI_QOS                        0x00000004
#define L2CAP_EXT_ENHANCED_RETRANSMISSION       0x00000008
#define L2CAP_EXT_STREAMING                     0x00000010
#define L2CAP_EXT_FCS_OPTION                    0x00000020
#define L2CAP_EXT_EXT_FLOW_SPEC_BR_EDR          0x00000040
#define L2CAP_EXT_FIXED_CHANNELS                0x00000080
#define L2CAP_EXT_EXT_WINDOW_SIZE               0x00000100
#define L2CAP_EXT_UNICAST_CONNLESS_DATA_RCPT    0x00000200

typedef struct _bt_l2cap_cmd_info_res_t 
{
	bt_l2cap_cmd_header_t header;
	bt_int type;
	bt_int result;
	union 
	{
		bt_int mtu;
		bt_long mask;
		bt_long fixed_channles[2];
	} data;
} bt_l2cap_cmd_info_res_t, *bt_l2cap_cmd_info_res_p;

// connection parameter update
typedef struct _bt_l2cap_cmd_conn_param_update_req_t 
{
	bt_l2cap_cmd_header_t header;
	bt_uint min_interval;
	bt_uint max_interval;
	bt_uint slave_latency;
	bt_uint timeout_multiplier;
} bt_l2cap_cmd_conn_param_update_req_t;

typedef struct _bt_l2cap_cmd_conn_param_update_res_t 
{
	bt_l2cap_cmd_header_t header;
	bt_int result;
} bt_l2cap_cmd_conn_param_update_res_t;


typedef union _bt_l2cap_command_t 
{
	bt_l2cap_cmd_reject_t                reject;
	bt_l2cap_cmd_connection_req_t        connection_req;
	bt_l2cap_cmd_connection_res_t        connection_res;
	bt_l2cap_cmd_disconnection_req_t     disconnection_req;
	cmd_disconnection_res                disconnection_res;
	bt_l2cap_cmd_config_req_t            config_req;
	bt_l2cap_cmd_config_res_t            config_res;
	bt_l2cap_cmd_echo_req_t              echo_req;
	bt_l2cap_cmd_echo_res_t              echo_res;
	bt_l2cap_cmd_info_req_t              info_req;
	bt_l2cap_cmd_info_res_t              info_res;
	bt_l2cap_cmd_conn_param_update_req_t conn_update_req;
	bt_l2cap_cmd_conn_param_update_res_t conn_update_res;
} bt_l2cap_command_t;


#ifdef __cplusplus
}
#endif

#endif // __L2CAP_COMMAND_H
