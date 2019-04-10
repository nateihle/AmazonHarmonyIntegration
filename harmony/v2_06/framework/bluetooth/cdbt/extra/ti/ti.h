/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __BTX_TI_TI_H_INCLUDED
#define __BTX_TI_TI_H_INCLUDED

#include "cdbt/bt/bt_std.h"
#include "cdbt/bt/bt_signal.h"
#include "cdbt/hci/hci.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BTX_TI_MODULATION_TYPE_GFSK    0
#define BTX_TI_MODULATION_TYPE_EDR2    1
#define BTX_TI_MODULATION_TYPE_EDR3    2

#define BTX_TI_MODULATION_SCHEME_CW       0
#define BTX_TI_MODULATION_SCHEME_GFSK     1
#define BTX_TI_MODULATION_SCHEME_P4_DPSK  2
#define BTX_TI_MODULATION_SCHEME_8_DPSK   3

#define BTX_TI_TEST_PATTERN_PN9           0
#define BTX_TI_TEST_PATTERN_PN15          1
#define BTX_TI_TEST_PATTERN_Z0Z0          2
#define BTX_TI_TEST_PATTERN_ALL_1         3
#define BTX_TI_TEST_PATTERN_ALL_0         4
#define BTX_TI_TEST_PATTERN_F0F0          5
#define BTX_TI_TEST_PATTERN_FF00          6
#define BTX_TI_TEST_PATTERN_USER          7

#define BTX_TI_SBC_SAMPLE_FREQUENCY_16000       0
#define BTX_TI_SBC_SAMPLE_FREQUENCY_32000       1
#define BTX_TI_SBC_SAMPLE_FREQUENCY_44100       2
#define BTX_TI_SBC_SAMPLE_FREQUENCY_48000       3

#define BTX_TI_SBC_CHANNEL_MODE_MONO            0
#define BTX_TI_SBC_CHANNEL_MODE_DUAL_CHANNEL    1
#define BTX_TI_SBC_CHANNEL_MODE_STEREO          2
#define BTX_TI_SBC_CHANNEL_MODE_JOINT_STEREO    3

#define BTX_TI_SBC_BLOCKS_4                     4
#define BTX_TI_SBC_BLOCKS_8                     8
#define BTX_TI_SBC_BLOCKS_12                    12
#define BTX_TI_SBC_BLOCKS_16                    16

#define BTX_TI_SBC_ALLOCATION_METHOD_LOUDNESS   0
#define BTX_TI_SBC_ALLOCATION_METHOD_SNR        1

#define BTX_TI_AVPR_ENABLE                      1
#define BTX_TI_AVPR_DISABLE                     0

#define BTX_TI_AVPR_LOAD_A3DP_CODE              1
#define BTX_TI_AVPR_DO_NOT_LOAD_A3DP_CODE       0

#define BTX_TI_A3DP_ROLE_SOURCE                 0
#define BTX_TI_A3DP_ROLE_SINK                   1

typedef struct _btx_ti_codec_config_s
{
	bt_uint clock_rate;
	bt_byte clock_direction;
	bt_ulong frame_sync_freq;
	bt_uint frame_sync_duty_cycle;
	bt_byte frame_sync_edge;
	bt_byte frame_sync_polarity;
	bt_byte frame_sync_multiplier;
	bt_uint ch1_data_out_size;
	bt_uint ch1_data_out_offset;
	bt_byte ch1_data_out_edge;
	bt_uint ch1_data_in_size;
	bt_uint ch1_data_in_offset;
	bt_byte ch1_data_in_edge;
	bt_uint ch2_data_out_size;
	bt_uint ch2_data_out_offset;
	bt_byte ch2_data_out_edge;
	bt_uint ch2_data_in_size;
	bt_uint ch2_data_in_offset;
	bt_byte ch2_data_in_edge;

} btx_ti_codec_config_t;

typedef struct _btx_ti_script_t
{
	const bt_byte* const * packets;
	bt_int packet_count;
	bt_byte fw_version_x;
	bt_byte fw_version_z;
} btx_ti_script_t;

typedef struct _btx_ti_exec_script_buffer_t btx_ti_exec_script_buffer_t;

typedef void (*btx_ti_completion_callback_fp)(bt_bool success, btx_ti_exec_script_buffer_t* buffer, void* callback_param);

struct _btx_ti_exec_script_buffer_t
{
	const btx_ti_script_t* script;
	btx_ti_completion_callback_fp callback;
	void* callback_param;
	bt_int current_packet;
};

void btx_ti_exec_script(const btx_ti_script_t* script,
						btx_ti_exec_script_buffer_t* buffer,
						btx_ti_completion_callback_fp callback,
						void* callback_param);


typedef struct _btx_ti_exec_script_oem_buffer_t btx_ti_exec_script_oem_buffer_t;

typedef void (*btx_ti_exec_script_oem_callback_fp)(bt_bool success, btx_ti_exec_script_oem_buffer_t* buffer, void* callback_param);

#define BTX_TI_EXEC_SCRIPT_OEM_RX_BUFFER_SIZE 10

struct _btx_ti_exec_script_oem_buffer_t
{
	const btx_ti_script_t* script;
	btx_ti_exec_script_oem_callback_fp callback;
	void* callback_param;
	bt_int current_packet;
	bt_int state;
	bt_byte rx_buffer[BTX_TI_EXEC_SCRIPT_OEM_RX_BUFFER_SIZE];
};

void btx_ti_exec_script_oem(const btx_ti_script_t* script,
							btx_ti_exec_script_oem_buffer_t* buffer,
							btx_ti_exec_script_oem_callback_fp callback,
							void* callback_param);

bt_bool btx_ti_enable_deep_sleep(bt_bool enable, bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_enable_fast_clock_crystal(bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_set_uart_baud_rate(bt_ulong baud_rate, bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_enable_low_power_scan(bt_bool enable, bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_enable_low_power_scan_default(bt_bool enable, bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_write_bdaddr(bt_bdaddr_t* bdaddr, bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_le_enable(
	bt_byte enable, bt_byte load_le_code,
	bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_avpr_enable(
	bt_byte enable, bt_byte load_a3dp_code, bt_byte a3dp_role,
	bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_avpr_debug(bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_write_codec_config(const btx_ti_codec_config_t* codec_config, bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_a3dp_sink_codec_config(
	bt_byte channels, bt_byte sample_frequency, bt_byte channel_mode, 
	bt_byte blocks, bt_byte sub_bands, bt_byte allocation_method,
	bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_a3dp_sink_open_stream(bt_byte acl_handle, bt_uint l2cap_cid, bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_a3dp_sink_close_stream(bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_a3dp_sink_start_stream(bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_a3dp_sink_stop_stream(bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_drpb_tester_con_tx(
	bt_byte modulation_scheme, 
	bt_byte test_pattern,
	bt_byte frequency_channel,
	bt_byte power_level,
	bt_ulong generator_initialization_value,
	bt_ulong edr_generator_mask,
	bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_write_hardware_register(
   bt_ulong address, 
   bt_uint value,
   bt_hci_cmd_callback_fp callback);

void btx_ti_init_controller(
	btx_ti_exec_script_buffer_t* buffer,
	btx_ti_completion_callback_fp callback,
	void* callback_param);

void btx_ti_init_ble_controller(
	btx_ti_exec_script_buffer_t* buffer,
	btx_ti_completion_callback_fp callback,
	void* callback_param);

bt_bool btx_ti_drpb_set_power_vector(
	bt_byte modulation_type, const bt_byte* power_vector, 
	bt_byte epc_max_level_threshold, bt_uint external_pa_mode,
	bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_drpb_enable_rf_calibration(
	bt_byte periodic_mode, bt_ulong calibration_procedure, 
	bt_byte temp_condition, bt_hci_cmd_callback_fp callback);

bt_bool btx_ti_set_afh_mode(bt_hci_hconn_t hconn, bt_bool enable, bt_hci_cmd_callback_fp callback);

// CC2560 Scripts (Service pack 2.36)
const btx_ti_script_t* btx_ti_get_script__BL6450_2_0_BT_Service_Pack_2_36(void);

// CC2560 Scripts (Service pack 2.44)
const btx_ti_script_t* btx_ti_get_script__BL6450_2_0_BT_Service_Pack_2_44(void);

// Alias for the latest script
#define btx_ti_get_script__CC2560_ServicePack btx_ti_get_script__BL6450_2_0_BT_Service_Pack_2_44

// CC2564 Scripts (Service pack 1.3)
const btx_ti_script_t* btx_ti_get_script__XWL1271L1_BT_ServicePack_1_3(void);
const btx_ti_script_t* btx_ti_get_script__XWL1271L1_BT_ServicePack_1_3_BLE_Init(void);

// CC2564 Scripts (Service pack 2.3)
const btx_ti_script_t* btx_ti_get_script__WL127xL_BT_Service_Pack_2_3(void);
const btx_ti_script_t* btx_ti_get_script__WL127xL_BT_Service_Pack_2_3_AVPR_AddOn(void);
const btx_ti_script_t* btx_ti_get_script__WL127xL_BT_Service_Pack_2_3_BLE_AddOn(void);
const btx_ti_script_t* btx_ti_get_script__WL127xL_BT_Service_Pack_2_3_DC2DC_AddOn(void);

// CC2564 Scripts (Service pack 2.4)
const btx_ti_script_t* btx_ti_get_script__WL127xL_BT_Service_Pack_2_4(void);
const btx_ti_script_t* btx_ti_get_script__WL127xL_BT_Service_Pack_2_4_AVPR_AddOn(void);
const btx_ti_script_t* btx_ti_get_script__WL127xL_BT_Service_Pack_2_4_BLE_AddOn(void);
const btx_ti_script_t* btx_ti_get_script__WL127xL_BT_Service_Pack_2_4_DC2DC_AddOn(void);

// CC2564 Scripts (Service pack 2.7)
const btx_ti_script_t* btx_ti_get_script__BL6450L_BT_Service_Pack_2_7(void);
const btx_ti_script_t* btx_ti_get_script__BL6450x_BT_Service_Pack_2_7_AVPR_AddOn(void);
const btx_ti_script_t* btx_ti_get_script__BL6450x_BT_Service_Pack_2_7_BLE_AddOn(void);

// CC2564 Scripts (Service pack 2.8)
const btx_ti_script_t* btx_ti_get_script__BL6450L_BT_Service_Pack_2_8_Short(void);
const btx_ti_script_t* btx_ti_get_script__BL6450L_BT_Service_Pack_2_8_BLE_AddOn(void);
const btx_ti_script_t* btx_ti_get_script__BL6450L_BT_Service_Pack_2_8_AVPR_AddOn(void);

// CC2564 Scripts (Service pack 2.10)
const btx_ti_script_t* btx_ti_get_script__BL6450L_BT_Service_Pack_2_10(void);
const btx_ti_script_t* btx_ti_get_script__BL6450L_BT_Service_Pack_2_10_BLE_AddOn(void);

// CC2564 Scripts (Service pack 2.12)
const btx_ti_script_t* btx_ti_get_script__BL6450L_BT_Service_Pack_2_12(void);
const btx_ti_script_t* btx_ti_get_script__BL6450L_BT_Service_Pack_2_12_BLE_AddOn(void);

// CC2564 Scripts (Service pack 2.14)
const btx_ti_script_t* btx_ti_get_script__BL6450L_BT_Service_Pack_2_14(void);
const btx_ti_script_t* btx_ti_get_script__BL6450L_BT_Service_Pack_2_14_BLE_AddOn(void);

// Aliases for latest scripts
#define btx_ti_get_script__CC2564_ServicePack  btx_ti_get_script__BL6450L_BT_Service_Pack_2_14
#define btx_ti_get_script__CC2564_BLE_Init     btx_ti_get_script__BL6450L_BT_Service_Pack_2_14_BLE_AddOn

// CC2564B Scripts (Service pack 0.1)
const btx_ti_script_t* btx_ti_get_script__CC2564B_BT_Service_Pack_0_1(void);
const btx_ti_script_t* btx_ti_get_script__CC2564B_BT_Service_Pack_0_1_BLE_AddOn(void);

// CC2564B Scripts (Service pack 0.2)
const btx_ti_script_t* btx_ti_get_script__CC2564B_BT_Service_Pack_0_2(void);
const btx_ti_script_t* btx_ti_get_script__CC2564B_BT_Service_Pack_0_2_BLE_AddOn(void);

// CC2564B Scripts (Service pack 1.0)
const btx_ti_script_t* btx_ti_get_script__CC2564B_BT_Service_Pack_1_0(void);
const btx_ti_script_t* btx_ti_get_script__CC2564B_BT_Service_Pack_1_0_BLE_AddOn(void);

// CC2564B Scripts (Service pack 1.1)
const btx_ti_script_t* btx_ti_get_script__CC2564B_BT_Service_Pack_1_1(void);
const btx_ti_script_t* btx_ti_get_script__CC2564B_BT_Service_Pack_1_1_BLE_AddOn(void);

// CC2564B Scripts (Service pack 1.2)
const btx_ti_script_t* btx_ti_get_script__CC2564B_BT_Service_Pack_1_2(void);
const btx_ti_script_t* btx_ti_get_script__CC2564B_BT_Service_Pack_1_2_BLE_AddOn(void);
const btx_ti_script_t* btx_ti_get_script__CC2564B_BT_Service_Pack_1_2_AVPR_AddOn(void);

// Aliases for latest scripts
#define btx_ti_get_script__CC2564B_ServicePack  btx_ti_get_script__CC2564B_BT_Service_Pack_1_2
#define btx_ti_get_script__CC2564B_BLE_Init     btx_ti_get_script__CC2564B_BT_Service_Pack_1_2_BLE_AddOn
#define btx_ti_get_script__CC2564B_AVPR_Init    btx_ti_get_script__CC2564B_BT_Service_Pack_1_2_AVPR_AddOn

#ifdef __cplusplus
}
#endif

#endif // __BTX_TI_TI_H_INCLUDED
