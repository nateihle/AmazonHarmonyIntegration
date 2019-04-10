/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __RFCOMM_PACKET_H
#define __RFCOMM_PACKET_H

#include "cdbt/utils/packet.h"

#ifdef __cplusplus
extern "C" {
#endif

struct _bt_rfcomm_command_t;
struct _bt_rfcomm_dlc_t;
typedef struct _bt_rfcomm_packet_t
{
	bt_packet_t header;
	struct _bt_rfcomm_command_t* cmd;
	struct _bt_rfcomm_dlc_t* dlc;
	bt_byte iscmd;
	bt_int data_offset;
	bt_byte fcs;

} bt_rfcomm_packet_t;

bt_int _bt_rfcomm_packet_assembler(bt_packet_t* packet, bt_byte* buffer, bt_int buffer_len);

#ifdef __cplusplus
}
#endif

#endif // __RFCOMM_PACKET_H
