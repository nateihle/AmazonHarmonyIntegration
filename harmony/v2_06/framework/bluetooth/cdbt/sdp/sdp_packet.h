/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __SDP_PACKET_H
#define __SDP_PACKET_H

#include "cdbt/utils/packet.h"

#ifdef __cplusplus
extern "C" {
#endif

struct _bt_sdp_service_transaction_t;
struct _bt_sdp_transaction_t;
struct _bt_sdp_data_element_t;

typedef struct _bt_sdp_packet_t
{
	bt_packet_t header;
	bt_byte sdu_type;

	union
	{
		union
		{
			struct _bt_sdp_service_transaction_t* service;
			struct _bt_sdp_transaction_t* attribute;
		} transaction;

		struct
		{
			bt_int trans_id;
			bt_int err_code;
		} error;

		struct
		{
			bt_int trans_id;
			struct _bt_sdp_data_element_t* pattern;
			bt_int uuid_index;
			bt_int uuid_pos;
		} service_search;

		struct
		{
			bt_int trans_id;
			struct _bt_sdp_data_element_t* pattern;
			bt_long sr;
		} attribute_search;
	} data;

} bt_sdp_packet_t;

bt_int bt_sdp_packet_assembler(bt_packet_t* packet, bt_byte* buffer, bt_int buffer_len);

#ifdef __cplusplus
}
#endif

#endif // __SDP_PACKET_H
