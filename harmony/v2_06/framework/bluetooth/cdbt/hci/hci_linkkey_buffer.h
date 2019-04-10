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

#ifndef __HCI_LINKKEY_BUFFER_H
#define __HCI_LINKKEY_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _bt_hci_link_key_s
{
	bt_bdaddr_t bdaddr;
	bt_byte linkkey[HCI_LINK_KEY_LEN];

} bt_hci_link_key_t;

void bt_hci_init_linkkey_buffers(void);

#ifdef __cplusplus
}
#endif

#endif // __HCI_LINKKEY_BUFFER_H
