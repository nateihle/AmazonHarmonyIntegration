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

#ifndef __HCI_DATA_BUFFER_H
#define __HCI_DATA_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HCI_MAX_DATA_BUFFERS
#define HCI_MAX_DATA_BUFFERS 2
#endif

#define HCI_DATA_BUFFER_STATE_FREE 0
#define HCI_DATA_BUFFER_STATE_USED 1

typedef struct _bt_hci_data_buffer_s {
	bt_int state;
	bt_hci_data_t packet;
} bt_hci_data_buffer_t, *bt_hci_data_buffer_p;

void bt_hci_init_data_buffers(void);
bt_hci_data_p bt_hci_alloc_data_buffer(void);
void bt_hci_free_data_buffer(bt_hci_data_p p);

#ifdef __cplusplus
}
#endif

#endif // __HCI_DATA_BUFFER_H
