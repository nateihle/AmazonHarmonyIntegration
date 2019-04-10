/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __BTX_CSR_H_INCLUDED__
#define __BTX_CSR_H_INCLUDED__

#ifdef __cplusplus
extern "C" {
#endif

/**
* \defgroup btx Vendor specific extensions to HCI
*
* \details This module defines functions and data structures used to access
*          and control various capabilities of CSR's controllers.
*
* \defgroup btx_csr CSR
* \ingroup btx
*/

// Select Variable definitions
#define CSR_VARID_MESSAGE_FROM_OPERATOR             0x101a
#define CSR_VARID_CAPABILITY_DOWNLOAD_INDICATION    0x101b
#define CSR_VARID_DSPMANAGER_CONFIG_REQUEST         0x101c
#define CSR_VARID_STREAM_DRAINED_NOTIFICATION       0x101d
#define PSKEY_BLE_DEFAULT_TX_POWER                  0x22c8
#define CSR_VARID_CACHED_TEMPERATURE                0x2872
#define CSR_VARID_RSSI_ACL                          0x301d
#define CSR_VARID_PIO                               0x681f
#define CSR_VARID_PIO_DIRECTION_MASK                0x681e
#define CSR_VARID_PIO_PROTECT_MASK                  0x2823
#define CSR_VARID_STREAM_GET_SOURCE                 0x505a
#define CSR_VARID_STREAM_GET_SINK                   0x505b
#define CSR_VARID_ENABLE_SCO_STREAMS                0x4876
#define CSR_VARID_STREAM_CLOSE_SOURCE               0x486b
#define CSR_VARID_STREAM_CLOSE_SINK                 0x486c
#define CSR_VARID_STREAM_TRANSFORM_DISCONNECT       0x486d
#define CSR_VARID_START_OPERATORS                   0x5070
#define CSR_VARID_STOP_OPERATORS                    0x5071
#define CSR_VARID_DESTROY_OPERATORS                 0x5073
#define CSR_VARID_CREATE_OPERATOR_C                 0x5075
#define CSR_VARID_CREATE_OPERATOR_P                 0x5076
#define CSR_VARID_STREAM_CONFIGURE                  0x505c
#define CSR_VARID_STREAM_CONNECT                    0x505e
#define CSR_VARID_MAP_SCO_AUDIO                     0x506a
#define PSKEY_DEEP_SLEEP_EXTERNAL_CLOCK_SOURCE_PIO  0x2579

// Select PS key definitions
#define PSKEY_BDADDR                          0x0001
#define PSKEY_H_HC_FC_MAX_SCO_PKT_LEN         0x0012
#define PSKEY_H_HC_FC_MAX_SCO_PKTS            0x0014
#define PSKEY_LC_MAX_TX_POWER                 0x0017
#define PSKEY_LC_DEFAULT_TX_POWER             0x0021
#define PSKEY_LC_MAX_TX_POWER_NO_RSSI         0x002d
#define PSKEY_ANA_FREQ                        0x01fe
#define PSKEY_VM_DISABLE                      0x025d
#define PSKEY_DEEP_SLEEP_STATE                0x0229
#define PSKEY_DEEP_SLEEP_USE_EXTERNAL_CLOCK   0x03c3
#define PSKEY_DEEP_SLEEP_CLEAR_RTS            0x0252
#define PSKEY_DEEP_SLEEP_WAKE_CTS             0x023c
#define PSKEY_CLOCK_REQUEST_ENABLE            0x0246
#define PSKEY_PIO_DEEP_SLEEP_EITHER_LEVEL     0x21bd
#define PSKEY_UART_BAUDRATE                   0x01be
#define PSKEY_UART_CONFIG_BCSP                0x01bf
#define PSKEY_UART_BITRATE                    0x01ea
#define PSKEY_HCI_NOP_DISABLE                 0x00f2
#define PSKEY_HOST_INTERFACE                  0x01f9
#define PSKEY_UART_CONFIG_H4                  0x01c0
#define PSKEY_UART_CONFIG_H5                  0x01c1
#define PSKEY_UART_TX_WINDOW_SIZE             0x01c6
#define PSKEY_UART_CONFIG_H4DS                0x01cb
#define PSKEY_HOSTIO_UART_RESET_TIMEOUT       0x01a4
#define PSKEY_HOSTIO_MAP_SCO_PCM              0x01ab
#define PSKEY_DIGITAL_AUDIO_CONFIG            0x01d9
#define PSKEY_DIGITAL_AUDIO_BITS_PER_SAMPLE   0x01db
#define PSKEY_PCM_PULL_CONTROL                0x01e2
#define PSKEY_PCM_CONFIG32                    0x01b3
#define PSKEY_PCM_FORMAT                      0x01b6
#define PSKEY_PCM_CLOCK_RATE                  0x23bc
#define PSKEY_PCM_SYNC_RATE                   0x23c3
#define PSKEY_PCM_USE_LOW_JITTER_MODE         0x23c9

// BCCMD GETRESP status codes
#define GETRESP_OK                   0x0000
#define GETRESP_NO_SUCH_VARID        0x0001
#define GETRESP_TOO_BIG              0x0002
#define GETRESP_NO_VALUE             0x0003
#define GETRESP_BAD_REQ              0x0004
#define GETRESP_NO_ACCESS            0x0005
#define GETRESP_READ_ONLY            0x0006
#define GETRESP_WRITE_ONLY           0x0007
#define GETRESP_ERROR                0x0008
#define GETRESP_PERMISSION_DENIED    0x0009

#define PS_DEFAULT                   0x0000
#define PS_RAM                       0x0008
#define PS_I                         0x0001
#define PS_F                         0x0002
#define PS_ROM                       0x0004

#define CSR_SRC_PCM                  1
#define CSR_SRC_I2S                  2
#define CSR_SRC_ADC                  3
#define CSR_SRC_FM                   4
#define CSR_SRC_SPDIF                5
#define CSR_SRC_MIC                  6
#define CSR_SRC_L2CAP                7
#define CSR_SRC_FASTPIPE             8
#define CSR_SRC_SCO                  9

#define CSR_SNK_PCM                  1
#define CSR_SNK_I2S                  2
#define CSR_SNK_ADC                  3
#define CSR_SNK_FM                   4
#define CSR_SNK_SPDIF                5
#define CSR_SNK_L2CAP                7
#define CSR_SNK_FASTPIPE             8
#define CSR_SNK_SCO                  9

#define CSR_PCM_STREAM_CFG_KEY_SYNC_RATE           0x100
#define CSR_PCM_STREAM_CFG_KEY_MCLK                0x101
#define CSR_PCM_STREAM_CFG_KEY_MASTER              0x102
#define CSR_PCM_STREAM_CFG_KEY_SLOT_COUNT          0x103
#define CSR_PCM_STREAM_CFG_KEY_MANCHESTER          0x104
#define CSR_PCM_STREAM_CFG_KEY_SHORT_SYNC          0x105
#define CSR_PCM_STREAM_CFG_KEY_MANCHESTER_SLAVE    0x106
#define CSR_PCM_STREAM_CFG_KEY_SIGN_EXTEND         0x107
#define CSR_PCM_STREAM_CFG_KEY_LSB_FIRST           0x108
#define CSR_PCM_STREAM_CFG_KEY_TX_TRISTATE         0x109

#define CSR_I2S_STREAM_CFG_KEY_SYNC_RATE           0x200
#define CSR_I2S_STREAM_CFG_KEY_MCLK                0x201
#define CSR_I2S_STREAM_CFG_KEY_MASTER              0x202
#define CSR_I2S_STREAM_CFG_KEY_JUSTIFY_FORMAT      0x203
#define CSR_I2S_STREAM_CFG_KEY_JUSTIFY_DELAY       0x204
#define CSR_I2S_STREAM_CFG_KEY_POLARITY            0x205
#define CSR_I2S_STREAM_CFG_KEY_AUDIO_ATTEN_ENABLE  0x206
#define CSR_I2S_STREAM_CFG_KEY_AUDIO_ATTEN         0x207
#define CSR_I2S_STREAM_CFG_KEY_JUSTIFY_RESOLUTION  0x208
#define CSR_I2S_STREAM_CFG_KEY_CROP_ENABLE         0x209
#define CSR_I2S_STREAM_CFG_KEY_BITS_PER_SAMPLE     0x20A
#define CSR_I2S_STREAM_CFG_KEY_TX_START_SAMPLE     0x20B
#define CSR_I2S_STREAM_CFG_KEY_RX_START_SAMPLE     0x20C

#define CSR_MSG_TYPE_GETREQ          0
#define CSR_MSG_TYPE_GETRESP         1
#define CSR_MSG_TYPE_SETREQ          2

#define CSR_MAX_HQ_PACKET_LEN        129

// Macros for defining lists of PS values for use with btx_csr_set_ps_vars().
#define SET_PS_VALUE_UINT16(key, value) key, 1, value
#define SET_PS_VALUE_UINT32(key, value) key, 2, (uint16_t)((((uint32_t)value) >> 16) & 0xFFFF), (uint16_t)(value &0xFFFF)
#define SET_PS_VALUE_BDADDR(key, m, l)  key, 4, (uint16_t)((((uint32_t)l) >> 16) & 0xFF), (uint16_t)(l &0xFFFF), (uint16_t)((((uint32_t)l) >> 24) & 0xFF), (uint16_t)(m &0xFFFF)


typedef struct _btx_csr_autobaud_buffer_t btx_csr_autobaud_buffer_t;

typedef void (*btx_csr_autobaud_callback_fp)(bt_bool success, btx_csr_autobaud_buffer_t* buffer);

struct _btx_csr_autobaud_buffer_t
{
	bt_byte* recv_buffer;
	bt_uint  recv_buffer_len;
	btx_csr_autobaud_callback_fp callback;
	void* callback_param;
};

typedef struct _btx_csr_script_t
{
	const bt_byte* const * packets;
	bt_int packet_count;
	bt_uint revision;
} btx_csr_script_t;

typedef const btx_csr_script_t* (*btx_csr_get_script_fp)(void);

typedef struct _btx_csr_exec_script_buffer_t btx_csr_exec_script_buffer_t;

typedef void (*btx_csr_exec_script_callback_fp)(bt_bool success, btx_csr_exec_script_buffer_t* buffer);

typedef struct _btx_csr_exec_hq_script_buffer_t btx_csr_exec_hq_script_buffer_t;

typedef void(*btx_csr_exec_hq_script_callback_fp)(bt_bool success, btx_csr_exec_hq_script_buffer_t* buffer);

struct _btx_csr_exec_script_buffer_t
{
	const btx_csr_script_t* script;
	btx_csr_exec_script_callback_fp callback;
	void* callback_param;
	bt_int current_packet;
};

struct _btx_csr_exec_hq_script_buffer_t
{
	const btx_csr_script_t* script;
	bt_int current_packet;
	bt_byte packet[CSR_MAX_HQ_PACKET_LEN];
	bt_bool success;
	btx_csr_exec_hq_script_callback_fp callback;
	void* callback_param;
};

typedef struct _btx_csr_bccmd_header_s
{
	bt_uint type;
	bt_uint len;
	bt_uint seq_no;
	bt_uint var_id;
	bt_uint status;
	bt_byte* payload;
} btx_csr_bccmd_header_t;

typedef struct _btx_csr_cached_temperature_s
{
	btx_csr_bccmd_header_t message;
	bt_uint                temperature;
} btx_csr_cached_temperature_t;

typedef struct _btx_csr_rssi_acl_s
{
	btx_csr_bccmd_header_t message;
	bt_hci_hconn_t         hconn;
	bt_uint                rssi;
} btx_csr_rssi_acl_t;

typedef struct _btx_csr_pio_s
{
	btx_csr_bccmd_header_t message;
	bt_uint                pio;
} btx_csr_pio_t;

typedef struct _btx_csr_pio_direction_mask_s
{
	btx_csr_bccmd_header_t message;
	bt_uint                direction;
} btx_csr_pio_direction_mask_t;

typedef struct _btx_csr_pio_protection_mask_s
{
	btx_csr_bccmd_header_t message;
	bt_uint                protection;
} btx_csr_pio_protection_mask_t;

typedef struct _btx_csr_strm_get_sink_s
{
	btx_csr_bccmd_header_t message;
	bt_uint                sink_id;
} btx_csr_strm_get_sink_t;

typedef struct _btx_csr_strm_get_source_s
{
	btx_csr_bccmd_header_t message;
	bt_uint                source_id;
} btx_csr_strm_get_source_t;

typedef struct _btx_csr_create_operator_c_s
{
	btx_csr_bccmd_header_t message;
	bt_uint                operator_id;
	bt_uint                item_count;
	bt_uint                skip_count;
	bt_uint                skip_flag;
} btx_csr_create_operator_c_t;

typedef struct _btx_csr_strm_connect_s
{
	btx_csr_bccmd_header_t message;
	bt_uint                transform_id;
} btx_csr_strm_connect_t;

typedef union _btx_csr_var_u
{
	btx_csr_bccmd_header_t        message;
	btx_csr_cached_temperature_t  cached_temperature;
	btx_csr_rssi_acl_t            rssi_acl;
	btx_csr_pio_t                 pio;
	btx_csr_pio_direction_mask_t  pio_direction;
	btx_csr_pio_protection_mask_t pio_protection;
	btx_csr_strm_get_sink_t       strm_get_sink;
	btx_csr_strm_get_source_t     strm_get_source;
	btx_csr_create_operator_c_t   create_operator;
	btx_csr_strm_connect_t        strm_connect;
} btx_csr_var_t;

typedef void (*btx_csr_bccmd_callback_fp)(btx_csr_bccmd_header_t* message, void* cb_param);
typedef struct _btx_csr_bccmd_listener_t btx_csr_bccmd_listener_t;
struct _btx_csr_bccmd_listener_t
{
	btx_csr_bccmd_listener_t* next_listener;

	btx_csr_bccmd_callback_fp   callback;
	void* callback_param;
};

typedef struct _btx_csr_set_ps_vars_buffer_t btx_csr_set_ps_vars_buffer_t;

typedef void (*btx_csr_set_ps_vars_callback_fp)(bt_bool success, btx_csr_set_ps_vars_buffer_t* buffer);

typedef void (*btx_csr_get_var_callback_fp)(bt_uint status, bt_uint var_id, btx_csr_var_t* var_value, void* callback_param);

typedef void (*btx_csr_set_var_callback_fp)(bt_uint status, bt_uint var_id, btx_csr_var_t* var_value, void* callback_param);

typedef void (*btx_csr_get_ps_var_callback_fp)(bt_uint success, const bt_byte* value, bt_uint len, void* callback_param);

struct _btx_csr_set_ps_vars_buffer_t
{
	const bt_uint* ps_vars;
	btx_csr_set_ps_vars_callback_fp callback;
	void* callback_param;
	bt_uint current_var;
};

/**
* \brief Initialize CSR support layer.
* \ingroup btx_csr
*
* \details This function initializes all internal variables of the CSR support layer.
*          CSR controllers use vendor specific event (0xFF) to carry the BCCMD protocol. 
*          They also do not report number of completed packets  for BCCMD commands.
*          This function installs a vendor specific event handler that makes sure 
*          that callback are called on corresponding vendor specific
*          commands and the number of free command buffers in the controller is kept correct.
*
*/
void btx_csr_init(void);

/**
* \brief Configure controller's UART speed.
* \ingroup btx_csr
*
* \details This function makes the controller auto-configure its UART speed.
*          The host transport must be set to H4.
*          This function works only with BC6 controllers.
*/
void btx_csr_autobaud(
		btx_csr_autobaud_buffer_t* buffer,
		btx_csr_autobaud_callback_fp callback,
		void* callback_param);

/**
* \brief Configure controller's UART speed and host interface.
* \ingroup btx_csr
*
* \details This function makes the controller auto-configure its UART speed and select H4 as host interface.
*          PS_KEY_HOST_INTERFACE must not be set. PS_KEY_UART_BITRATE must be set to 0.
*          This function works only with BC7 controllers.
*
*/
void btx_csr_bc7_sel_host_interface_h4(
		btx_csr_autobaud_buffer_t* buffer,
		bt_byte interval,
		btx_csr_autobaud_callback_fp callback,
		void* callback_param);

/**
* \brief Patch controller's firmware
* \ingroup btx_csr
*
* \details This function executes a script that patches the controller's firmware.
*          The \c script must point to a structure that contain a complete patch script for
*          a particular controller model and revision. If the revision specified in the script and 
*          revision read from the controller are the same btx_csr_patch_controller() loads 
*          the script to the controller and calls the \c callback with the first parameter TRUE. 
*          Otherwise the \c callback is called with the first parameter FALSE.
*
*          If support for multiple firmware revisions is neede use btx_csr_patch_controller().
*
* \param script Array of patch scripts.
* \param buffer A buffer for storing temporary data needed for script execution.
* \param callback The callback function that will be called when the script has been executed.
* \param callback_param A pointer to arbitrary data to be passed to the \c callback callback..
*/
void btx_csr_exec_script(
		const btx_csr_script_t* script,
		btx_csr_exec_script_buffer_t* buffer,
		btx_csr_exec_script_callback_fp callback,
		void* callback_param);

bt_hci_command_t* btx_csr_alloc_bccmd_setreq(
		bt_uint var_id,
		bt_uint data_word_count,
		bt_hci_cmd_callback_fp callback,
		void* callback_param);

bt_hci_command_t* btx_csr_alloc_bccmd_getreq(
		bt_uint var_id,
		bt_uint data_word_count,
		bt_hci_cmd_callback_fp callback,
		void* callback_param);

bt_bool btx_csr_set_ps_var(
		bt_uint ps_key,
		const bt_uint* value,
		bt_uint value_word_count,
		bt_hci_cmd_callback_fp callback);

bt_bool btx_csr_set_ps_var_ex(
	   bt_uint ps_key,
	   const bt_uint* value,
	   bt_uint value_word_count,
	   bt_uint store,
	   bt_hci_cmd_callback_fp callback);

bt_bool btx_csr_get_ps_var(
		bt_uint ps_key,
		bt_uint value_word_count,
		btx_csr_get_ps_var_callback_fp callback,
		void* callback_param);

bt_bool btx_csr_get_ps_var_ex(
		bt_uint ps_key,
		bt_uint value_word_count,
		bt_uint store,
		btx_csr_get_ps_var_callback_fp callback,
		void* callback_param);

/**
* \brief Write PS variables
* \ingroup btx_csr
*
* \details 
*
* \param ps_vars PS values
* \param buffer A buffer for storing temporary data during function execution.
* \param callback The callback function that will be called when all PS values have been sent to the controller or error occurred.
* \param callback_param A pointer to arbitrary data to be passed to the \c callback callback..
*/
void btx_csr_set_ps_vars(
		const bt_uint* ps_vars,
		btx_csr_set_ps_vars_buffer_t* buffer,
		btx_csr_set_ps_vars_callback_fp callback,
		void* callback_param);

/**
* \brief Write PS variables
* \ingroup btx_csr
*
* \param ps_vars PS values
* \param buffer A buffer for storing temporary data during function execution.
* \param store
* \param callback The callback function that will be called when all PS values have been sent to the controller or error occurred.
* \param callback_param A pointer to arbitrary data to be passed to the \c callback callback..
*/
void btx_csr_set_ps_vars_ex(
		const bt_uint* ps_vars,
		btx_csr_set_ps_vars_buffer_t* buffer,
		bt_uint store,
		btx_csr_set_ps_vars_callback_fp callback,
		void* callback_param);

/**
* \brief Warm reset
* \ingroup btx_csr
*
* \details This function performs warm reset of the controller. All patches and configuration parameters
*          sent to the controller before warm reset are kept intact.
*
*/
bt_bool btx_csr_warm_reset(void);

/**
* \brief Warm reset
* \ingroup btx_csr
*
* \details This function performs warm reset of the controller. All patches and configuration parameters
*          sent to the controller before warm reset are kept intact. Since the controller does not respond 
*          to the warm reset command as it starts resetting immediately upon receiving the command, 
*          the \c callback is called right after the command packet has been transmitted to the controller.
*
* \param callback The callback function that will be called after the warm reset command has been sent to the controller.
* \param callback_param A pointer to arbitrary data to be passed to the \c callback callback.
*/
bt_bool btx_csr_warm_reset_ex(
		bt_hci_cmd_callback_fp callback,
		void* callback_param);

/**
* \brief Enable/disable transmitter
* \ingroup btx_csr
*
* \param enable Specifies whether the transmitter should be enable or disabled.
* \param callback The callback function that will be called after the command has completed.
* \param callback_param A pointer to arbitrary data to be passed to the \c callback callback.
*/
bt_bool btx_csr_enable_tx(
		bt_bool enable, 
		bt_hci_cmd_callback_fp callback,
		void* callback_param);

bt_bool btx_csr_get_var(
		bt_uint var_id,
		btx_csr_get_var_callback_fp callback,
		void* callback_param);

bt_bool btx_csr_set_var(
		bt_uint var_id,
		const bt_uint* value,
		bt_uint value_word_count,
		btx_csr_set_var_callback_fp callback,
		void* callback_param);

/**
* \brief Get chip's cached temperature
* \ingroup btx_csr
*
* \param callback The callback function that will be called after the command has completed.
* \param callback_param A pointer to arbitrary data to be passed to the \c callback callback.
*/
bt_bool btx_csr_get_cached_temperature(
		btx_csr_get_var_callback_fp callback,
		void* callback_param);

/**
* \brief Get RSSI
* \ingroup btx_csr
*
* \details This function retrieves the RSSI for a given HCI ACL handle.
*
* \param hconn ACL connection handle.
* \param callback The callback function that will be called after the command has completed.
* \param callback_param A pointer to arbitrary data to be passed to the \c callback callback.
*/
bt_bool btx_csr_get_rssi_acl(
		bt_hci_hconn_t hconn,
		btx_csr_get_var_callback_fp callback,
		void* callback_param);

/**
* \brief Patch controller's firmware
* \ingroup btx_csr
*
* \details This function executes a script that patches the controller's firmware. 
*          Each entry of the \c scripts array must be a complete patch script for
*          a particular controller model and revision. btx_csr_patch_controller() reads
*          the revision number from the controller then tries to find the corresponding
*          script in the \c scripts. If there is a matching script it is loaded to the controller
*          and \c callback is called with the first parameter TRUE. If no suitable script found
*          \c callback is called with the first parameter FALSE.
*
* \param scripts Array of patch scripts.
* \param script_count The number of scripts in \c scripts.
* \param buffer A buffer for storing temporary data needed for script execution.
* \param callback The callback function that will be called when the script has been executed.
* \param callback_param A pointer to arbitrary data to be passed to the \c callback callback..
*/
void btx_csr_patch_controller(
		const btx_csr_get_script_fp* scripts,
		bt_int script_count,
		btx_csr_exec_script_buffer_t* buffer,
		btx_csr_exec_script_callback_fp callback,
		void* callback_param);

void btx_csr_init_hq_script(
		const btx_csr_script_t* script,
		btx_csr_exec_hq_script_buffer_t* buffer);

bt_bool btx_csr_send_next_hq_script_packet(bt_uint seq_no, btx_csr_exec_hq_script_buffer_t* buffer);

bt_bool btx_csr_send_dsp_config_data(
		bt_uint total_config_blocks,
		bt_uint seq_no,
		bt_uint status,
		const bt_uint* data,
		bt_uint data_word_count,
		bt_hci_cmd_callback_fp callback,
		void* callback_param);

void btx_csr_exec_hq_script(
	const btx_csr_script_t* script,
	btx_csr_exec_hq_script_buffer_t* buffer,
	btx_csr_exec_hq_script_callback_fp callback,
	void* callback_param);

bt_bool btx_csr_register_bccmd_listener(btx_csr_bccmd_listener_t* listener);
void btx_csr_unregister_bccmd_listener(btx_csr_bccmd_listener_t* listener);

/**
* \brief Return script for patching BlueCore 6
* \ingroup btx_csr
*
*/
const btx_csr_script_t* btx_csr_get_script__PB_27_R20_BC6ROM_A04(void);

/**
* \brief Return script for patching CSR8810 (BlueCore 7)
* \ingroup btx_csr
*
*/
const btx_csr_script_t* btx_csr_get_script__PB_90_REV6(void);

/**
* \brief Return script for patching CSR8x11 A06 (BlueCore 7)
* \ingroup btx_csr
*
*/
const btx_csr_script_t* btx_csr_get_script__PB_101_CSR8811_CSP28_UART(void);

/**
* \brief Return script for patching CSR8x11 A08 (BlueCore 7)
* \ingroup btx_csr
*
*/
const btx_csr_script_t* btx_csr_get_script__PB_109_CSR8811_REV16(void);

/**
* \brief Return script for patching DSP in CSR8x11 A08 (BlueCore 7)
* \ingroup btx_csr
*
*/
const btx_csr_script_t* btx_csr_get_script__dsp_script__PB_109_DSP_rev8(void);

/**
* \brief Return script for patching CSR8x11 A12 (BlueCore 7)
* \ingroup btx_csr
*
*/
const btx_csr_script_t* btx_csr_get_script__PB_173_CSR8X11_REV1(void);

#ifdef __cplusplus
}
#endif

#endif // __BTX_CSR_H_INCLUDED__
