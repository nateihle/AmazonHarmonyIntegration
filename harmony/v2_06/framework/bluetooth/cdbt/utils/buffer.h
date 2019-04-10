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

#ifndef __UTILS_BUFFER_H
#define __UTILS_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>      // for offsetof

#define BUFFER_STATE_FREE 0
#define BUFFER_STATE_USED 1

#define BUFFER_HDR_LEN	offsetof(bt_buffer, data)

typedef struct _bt_buffer_header_t 
{
	bt_int state;
} bt_buffer_header_t, *bt_bt_buffer_header_p;

typedef struct _bt_buffer_mgr_t 
{
	bt_int max_buffers;
	bt_int buffer_size;
	bt_buffer_header_t* headers;
	bt_byte_p buffers;
} bt_buffer_mgr_t, *bt_buffer_mgr_p;


bt_bool bt_init_buffer_mgr(bt_buffer_mgr_p pmgr, bt_int num_buffers, bt_int buffer_size, bt_buffer_header_t* buffer_headers, void* buffer_data);
void* bt_alloc_buffer(bt_buffer_mgr_p pmgr);
void bt_free_buffer(bt_buffer_mgr_p pmgr, void* data);
void* bt_get_buffer(bt_buffer_mgr_p pmgr, bt_int index);
bt_buffer_header_t* bt_get_buffer_header(bt_buffer_mgr_p pmgr, bt_int index);
bt_int bt_get_buffer_index(bt_buffer_mgr_p pmgr, void* data);

#ifdef __cplusplus
}
#endif

#endif // __UTILS_BUFFER_H
