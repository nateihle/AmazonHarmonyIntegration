/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __GAP_H
#define __GAP_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _bt_device_t
{
	bt_bdaddr_t bdaddr;
	bt_byte* name;
	bt_byte  max_name_len;
	bt_byte pg_scan_rpt_mode;
	bt_byte pg_scan_period_mode;
	bt_long cod;
	bt_int clock_offset;
	bt_byte rssi;
	bt_byte* eir;
	bt_byte  eir_len;
	bt_byte  max_eir_len;
} bt_device_t;

typedef void (*bt_find_devices_callback_fp)(bt_byte count, void* callback_param);

bt_bool bt_gap_find_devices(
	bt_device_t* devices, bt_byte max_devices, 
	bt_byte length, bt_byte mode,
	bt_find_devices_callback_fp callback, void* callback_param);

#ifdef __cplusplus
}
#endif

#endif // __GAP_H
