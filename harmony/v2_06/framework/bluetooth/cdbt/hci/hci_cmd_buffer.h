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

#ifndef __HCI_CMD_BUFFER_H
#define __HCI_CMD_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

void _hci_init_cmd_buffers(void);

bt_hci_command_t* _hci_allocate_cmd(void);

void _hci_free_cmd(bt_hci_command_t* pcmd);

#ifdef __cplusplus
}
#endif

#endif // __HCI_CMD_BUFFER_H
