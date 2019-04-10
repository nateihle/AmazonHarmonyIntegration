/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __UTILS_FIFO_H
#define __UTILS_FIFO_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _bt_fifo_s
{
	bt_byte* data;
	bt_uint  size;
	bt_uint  size_mask;
	bt_uint  head;
	bt_uint  tail;
} bt_fifo_t;

bt_bool bt_fifo_init(bt_fifo_t* fifo, bt_byte* data, bt_uint size);

bt_bool bt_fifo_reset(bt_fifo_t* fifo);

bt_uint bt_fifo_push(bt_fifo_t* fifo, bt_byte* data, bt_uint len);

bt_uint bt_fifo_pop(bt_fifo_t* fifo, bt_byte* data, bt_uint max_len);

bt_uint bt_fifo_get_avail(bt_fifo_t* fifo);

bt_uint bt_fifo_get_free(bt_fifo_t* fifo);

#ifdef __cplusplus
}
#endif

#endif // __UTILS_FIFO_H
