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

#ifndef __SPP_PRIVATE_H
#define __SPP_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

extern bt_spp_port_t   _spp_ports[];
extern const bt_byte   _spp_max_ports;
extern bt_byte*        _spp_frame_buffers;
extern bt_int*         _spp_frame_len;
extern const bt_byte   _spp_disable_buffering;

extern bt_byte         _spp_remaining_connect_attemtps;
extern bt_bdaddr_t     _spp_connect_device_address;

#ifdef _DEBUG
extern const bt_uint _ram_size_spp_buffers;
#endif

bt_spp_port_t* _bt_spp_find_port(bt_rfcomm_dlc_t* dlc);

void _bt_spp_rfcomm_read_data_callback(bt_rfcomm_dlc_p pdlc, bt_byte_p pdata, bt_int len);
void _bt_spp_handle_rx(bt_spp_port_t* port);

void _bt_spp_rfcomm_send_data_callback(bt_rfcomm_dlc_t* dlc, bt_byte* data, bt_int len, bt_int status);
void _bt_spp_handle_tx(bt_spp_port_t* port);

void _bt_spp_client_init(void);

#ifdef __cplusplus
}
#endif

#endif // __SPP_PRIVATE_H
