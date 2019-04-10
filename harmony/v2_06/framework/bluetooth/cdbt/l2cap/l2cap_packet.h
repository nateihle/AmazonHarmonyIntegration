/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __L2CAP_PACKET_H
#define __L2CAP_PACKET_H

#include "cdbt/utils/packet.h"

#ifdef __cplusplus
extern "C" {
#endif

#define L2CAP_PACKET_DATA_TYPE_RAW      0
#define L2CAP_PACKET_DATA_TYPE_SMART    1

struct _bt_l2cap_channel_t;

typedef struct _bt_l2cap_packet_t
{
	bt_packet_t header;

	union
	{
		struct _bt_l2cap_channel_t* channel;
		bt_hci_hconn_t hconn;
	} destination;

/*
	bt_uint fcs;
	bt_int ctl;
*/
	bt_byte data_type;
} bt_l2cap_packet_t;

bt_int bt_l2cap_packet_data_assembler(bt_packet_t* packet, bt_byte* buffer, bt_int buffer_len);

bt_int bt_l2cap_packet_cmd_assembler(bt_packet_t* packet, bt_byte* buffer, bt_int buffer_len);

#ifdef __cplusplus
}
#endif

#endif // __L2CAP_PACKET_H
