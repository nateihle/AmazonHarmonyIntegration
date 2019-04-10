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

#ifndef __SDP_UTILS_H
#define __SDP_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

bt_bool _sdp_write_data_element(bt_sdp_data_element_cp pde, bt_byte_p buffer, bt_int len, bt_int_p poffset);
bt_bool _sdp_read_de_header(bt_byte_cp buffer, bt_int len, bt_int_p poffset, bt_byte_p pde_type, bt_byte_p pde_size_index, bt_ulong_p pde_data_len);
bt_ulong _sdp_get_de_data_len(bt_sdp_data_element_cp pde);
bt_ulong _sdp_get_de_hdr_len(bt_sdp_data_element_cp pde);

void bt_sdp_de_to_uuid(bt_sdp_data_element_cp pde, bt_uuid_p puuid);

#ifdef __cplusplus
}
#endif

#endif // _SDP_UTILS_H
