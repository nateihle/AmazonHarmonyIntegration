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

#ifndef __HCI_TRANSPORT_H
#define __HCI_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#define HCI_TRANSPORT_HEADER_LEN	1

typedef void (*bt_hci_transport_send_packet_callback_fp)(void);
typedef void (*bt_hci_transport_recv_packet_callback_fp)(bt_uint len);

typedef struct _hci_transport_t
{
    void (*send_packet)(const bt_byte* buffer, bt_uint len, bt_hci_transport_send_packet_callback_fp callback);
    void (*recv_packet)(bt_byte* buffer, bt_uint len, bt_hci_transport_recv_packet_callback_fp callback);
} hci_transport_t;


void bt_hci_transport_set_transport(const hci_transport_t* transport);
void bt_hci_transport_send_packet(const bt_byte* buffer, bt_uint len, bt_hci_transport_send_packet_callback_fp callback);
void bt_hci_transport_recv_packet(bt_byte* buffer, bt_uint len, bt_hci_transport_recv_packet_callback_fp callback);

// deprecated
void bt_hci_transport_send_cmd(const bt_byte* buffer, bt_uint len, bt_hci_transport_send_packet_callback_fp callback);
void bt_hci_transport_send_data(bt_byte* buffer, bt_uint len, bt_hci_transport_send_packet_callback_fp callback);


#ifdef __cplusplus
}
#endif

#endif // __HCI_TRANSPORT_H
