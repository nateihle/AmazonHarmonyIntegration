/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __UTILS_PACKET_H
#define __UTILS_PACKET_H

#ifdef __cplusplus
extern "C" {
#endif

struct _bt_packet_t;

typedef bt_int (*bt_packet_assembler_fp)(struct _bt_packet_t* packet, bt_byte* buffer, bt_int buffer_len);

typedef struct _bt_packet_t
{
	bt_packet_assembler_fp packet_assembler;
	const bt_byte* data;
	bt_int len;
	bt_byte state;
	bt_int data_pos;
	void* param;
} bt_packet_t;

#ifdef __cplusplus
}
#endif

#endif // __UTILS_PACKET_H
