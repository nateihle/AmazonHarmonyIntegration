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

#ifndef __SDP_PRIVATE_H
#define __SDP_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

//
// Global variables defined in SDP modules
// -------------------------------------------------------------------
//

// In sdp_server.c
extern const bt_byte* sdp_db_main2;
extern bt_uint sdp_db_main2_len;

//
// Global variables defined by OEM configuration
// -------------------------------------------------------------------
//
extern bt_buffer_header_t           _sdp_client_packet_buffer_headers[];
extern bt_sdp_packet_t              _sdp_client_packet_buffers[];
extern bt_buffer_header_t           _sdp_packet_buffer_headers[];
extern bt_sdp_packet_t              _sdp_packet_buffers[];
extern const bt_byte                _sdp_max_buffers;
extern const bt_byte                _sdp_client_max_buffers;
extern const bt_uint                _sdp_max_search_result_len;
extern const bt_uint                _sdp_max_attribute_result_len;
extern bt_sr_handle_t               _sdp_found_sr_lists_buffers[];
extern bt_byte*			            _sdp_found_attr_list_buffers[];
extern bt_sdp_found_attr_list_t     _sdp_found_attr_lists_buffers[];
extern bt_buffer_mgr_t              _sdp_tran_buffer_mgr2;
extern bt_buffer_header_t           _sdp_tran_buffer_headers2[];
extern bt_sdp_transaction_t         _sdp_tran_buffers2[];
extern bt_buffer_mgr_t              _sdp_service_tran_buffer_mgr;
extern bt_buffer_header_t           _sdp_service_tran_buffer_headers[];
extern bt_sdp_service_transaction_t _sdp_service_tran_buffers[];

extern bt_bool             (*_sdp_start_fp)(bt_l2cap_mgr_p l2cap_mgr, const bt_byte* sdp_db, bt_uint sdp_db_len);

void _bt_sdp_client_init(void);

#ifdef _DEBUG
extern const bt_uint _ram_size_sdp_buffers;
#endif

//
// Private types
// -------------------------------------------------------------------
//

typedef struct _bt_sdp_server_data_element_t
{
	bt_byte type;
	bt_uint data_len;
	const bt_byte* data;
} bt_sdp_server_data_element_t;

typedef struct _bt_sdp_server_attribute_t
{
	bt_uint attr_id;
	bt_uint data_len;
	const bt_byte* data;
	bt_sdp_server_data_element_t attr_value;
} bt_sdp_server_attribute_t;

typedef struct _bt_sdp_server_record_t 
{
	bt_sr_handle_t h;
	bt_uint data_len;
	const bt_byte* data;
} bt_sdp_server_record_t;


//
// Private global functions
// -------------------------------------------------------------------
//

// From sdp_utils.c
int sdp_find_service_records2(
	bt_byte_p pparams, bt_int params_len, 
	bt_int offsetInit, bt_byte patternCount,
	bt_sdp_service_transaction_p ptran, bt_int max_handles);
bt_bool sdp_find_attributes(
	bt_sr_handle_p h_list, bt_int h_count, 
	bt_byte_p pparams, bt_int params_len, 
	bt_int offsetInit, bt_byte patternCount,
	bt_sdp_transaction_t* ptran, bt_int max_attrs);

bt_bool sdp_compare_uuid_de(bt_sdp_data_element_cp pde1, bt_sdp_data_element_cp pde2);

// From sdp_tran_buffer.c
bt_bool _sdp_init_tran_buffers(void);
bt_sdp_transaction_t* _sdp_alloc_tran_buffer(bt_int id);
void _sdp_free_tran_buffer(bt_sdp_transaction_t* ptran);
bt_sdp_transaction_t* _sdp_find_transaction(bt_int id);
bt_sdp_service_transaction_p _sdp_alloc_svc_tran_buffer(bt_int id);
void _sdp_free_svc_tran_buffer(bt_sdp_service_transaction_p ptran);
bt_sdp_service_transaction_p _sdp_find_svc_transaction(bt_int id);

// From sdp_db_utils.c
bt_bool bt_sdp_read_attribute(bt_sdp_server_attribute_t* attr, const bt_byte* buffer, bt_int len, bt_int* offset);


#ifdef __cplusplus
}
#endif

#endif // __SDP_PRIVATE_H

