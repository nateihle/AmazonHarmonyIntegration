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

#ifndef __UTILS_QUEUE_H
#define __UTILS_QUEUE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _bt_queue_element_t
{
	struct _bt_queue_element_t* next;
} bt_queue_element_t;

void bt_q_add(bt_queue_element_t** phead, void* element);
void bt_q_push(bt_queue_element_t** phead, void* element);
void* bt_q_get_head(bt_queue_element_t** phead, bt_bool remove);
void* bt_q_get_next(void* element);
void* bt_q_get(bt_queue_element_t* head, bt_int index);
void* bt_q_remove(bt_queue_element_t** phead, void* pdata);
void* bt_q_remove_by_idx(bt_queue_element_t* head, bt_int index);
bt_int bt_q_get_length(bt_queue_element_t* phead);
bt_bool bt_q_contains(bt_queue_element_t* phead, void* element);

#ifdef __cplusplus
}
#endif

#endif // __UTILS_QUEUE_H
