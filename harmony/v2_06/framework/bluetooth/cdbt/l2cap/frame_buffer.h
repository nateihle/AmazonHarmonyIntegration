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

#ifndef __L2CAP_FRAME_BUFFER_H
#define __L2CAP_FRAME_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef L2CAP_MAX_FRAME_BUFFERS
#define L2CAP_MAX_FRAME_BUFFERS 2
#endif

void bt_l2cap_init_frame_buffers(void);
bt_byte_p bt_l2cap_alloc_frame_buffer(void);
void bt_l2cap_free_frame_buffer(bt_byte_p data);

#ifdef __cplusplus
}
#endif

#endif // __L2CAP_FRAME_BUFFER_H
