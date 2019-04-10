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

#ifndef __HCI_CMD_QUEUE_H
#define __HCI_CMD_QUEUE_H

#ifdef __cplusplus
extern "C" {
#endif

void _hci_init_cmd_queues(void);
bt_hci_command_p hci_cq_find_by_opcode(bt_queue_element_t* head, bt_int opcode);
bt_hci_command_p hci_cq_find_by_hconn(bt_queue_element_t* head, bt_hci_hconn_t hconn);
bt_hci_command_p hci_cq_find_by_hconn_and_opcode(bt_queue_element_t* head, bt_hci_hconn_t hconnFind, bt_int opcode);
bt_hci_command_p hci_cq_find_by_bdaddr_and_opcode(bt_queue_element_t* head, bt_bdaddr_t bdaddrFind, bt_int opcode);


#ifdef __cplusplus
}
#endif

#endif // __HCI_CMD_QUEUE_H
