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

#ifndef __RFCOMM_CONFIG_H
#define __RFCOMM_CONFIG_H

/**
 * \defgroup rfcomm_config RFCOMM Configuration
 * \ingroup spp
 *
 * This module describes parameters used to configure RFCOMM layer.
 *
 * dotstack is customized using a configuration file. The configuration file tailors the dotstack to the application being built. It has to have the structure shown below.
 * 
    \code
	#include "cdbt/bt/bt_std.h"

	// HCI and L2CAP must always be present
	// SDP is required only if stack is running in dual mode. This is the default mode.
	// To run the stack in single mode (i.e. only BLE is supported) a BT_BLE_SINGLE_MODE symbol
	// must be defined:
	// #define BT_BLE_SINGLE_MODE

	// HCI configuration parameters
	#define HCI_MAX_CMD_BUFFERS            ...
	#define HCI_MAX_DATA_BUFFERS           ...
	#define HCI_MAX_HCI_CONNECTIONS        ...
	#define HCI_RX_BUFFER_LEN              ...
	#define HCI_TX_BUFFER_LEN              ...
	#define HCI_L2CAP_BUFFER_LEN           ...
	#define HCI_MAX_CMD_PARAM_LEN          ...

	// L2CAP configuration parameters
	#define L2CAP_MAX_CMD_BUFFERS          ...
	#define L2CAP_MAX_FRAME_BUFFERS        ...
	#define L2CAP_MAX_PSMS                 ...
	#define L2CAP_MAX_CHANNELS             ...

	// SDP configuration parameters
	#define SDP_MAX_SEARCH_RESULT_LEN	   ...
	#define SDP_MAX_ATTRIBUTE_RESULT_LEN   ...

	// Depending on protocols and profiles used below goes configuration parameters
	// for each used module. E.g., to use and configure RFCOMM,
	// the following values must be defined:

	#define RFCOMM_MAX_SESSIONS            ...
	#define RFCOMM_MAX_DLCS                ...
	#define RFCOMM_MAX_SERVER_CHANNELS     ...
	#define RFCOMM_INFO_LEN                ...
	#define RFCOMM_MAX_DATA_BUFFERS        ...
	#define RFCOMM_MAX_CMD_BUFFERS         ...
	#define RFCOMM_LOCAL_CREDIT            ...

	#include "cdbt/bt/bt_oem_config.h"

	\endcode
 *
*/

#ifndef RFCOMM_MAX_SESSIONS
#error "RFCOMM_MAX_SESSIONS is not defined"

/**
* \brief  Maximum number of remote devices a local device can be connected to
* \ingroup rfcomm_config
*
* \details This parameter defines the maximum number of remote devices a local device can have simultaneous connections to.
*          This value should not exceed HCI_MAX_HCI_CONNECTIONS.
*/
#define RFCOMM_MAX_SESSIONS
#endif

#ifndef RFCOMM_MAX_DLCS
#error "RFCOMM_MAX_DLCS is not defined"

/**
* \brief  Maximum number of DLCs
* \ingroup rfcomm_config
*
* \details This parameter defines the maximum number of DLCs on each session.
*          This value should be at least 2 because each session uses one DLC to convey multiplexer control messages.
*          All other DLCs are used to emulate serial ports.
*/
#define RFCOMM_MAX_DLCS
#endif

#ifndef RFCOMM_MAX_SERVER_CHANNELS
#error "RFCOMM_MAX_SERVER_CHANNELS is not defined"

/**
* \brief  Maximum number of Server channels
* \ingroup rfcomm_config
*
* \details This parameter defines the maximum number of server channels exposed by the local device.
*          This value should not exceed RFCOMM_MAX_DLCS - 1.
*/
#define RFCOMM_MAX_SERVER_CHANNELS
#endif

#ifndef RFCOMM_INFO_LEN
#error "RFCOMM_INFO_LEN is not defined"

/**
* \brief  Maximum size of the data portion of a UIH frame.
* \ingroup rfcomm_config
*
* \details This parameter defines the maximum size of the data portion of a UIH frame. 
*          If CFC is used the actual length of the data portion will be 1 byte less.
*          This value must be less than or equal to HCI_L2CAP_BUFFER_LEN - RFCOMM_FRAME_HEADER_LEN - L2CAP_HEADER_LEN.
*/
#define RFCOMM_INFO_LEN
#endif

#ifndef RFCOMM_MAX_CMD_BUFFERS
#error "RFCOMM_MAX_CMD_BUFFERS is not defined"

/**
* \brief  Maximum number of command buffers.
* \ingroup rfcomm_config
*
* \details This parameter defines the maximum number of commands that can be sent at the same time.
*          It is usually enough to reserve 2 buffers for each DLC excluding control DLC. Therefore, this value can be defined as \r\n
*             #define RFCOMM_MAX_CMD_BUFFERS    (RFCOMM_MAX_DLCS - 1) * 2
*/
#define RFCOMM_MAX_CMD_BUFFERS
#endif

#ifndef RFCOMM_LOCAL_CREDIT
#error "RFCOMM_LOCAL_CREDIT is not defined"

/**
* \brief  The number of receive buffers.
* \ingroup rfcomm_config
*
* \details This parameter defines the number of received UIH frames that can be stored on the local device. The flow control mechanism used in RFCOMM
*          ensures that the remote side of the link always knows how many free buffers left on the local device. When the number of free buffers reaches 0,
*          the transmitter stops sending data frames until the receiver frees some buffers.
*          The RFCOMM layer does not actually allocate space for buffers. It uses RFCOMM_LOCAL_CREDIT to keep track of free buffers and report them to the remote side.
*          Actual memory allocation is done in SPP layer.
*/
#define RFCOMM_LOCAL_CREDIT
#endif

#ifndef RFCOMM_LOCAL_CREDIT_SEND_THRESHOLD
	#if	((RFCOMM_LOCAL_CREDIT * 100) * 3 / 4 / 100 != 0)
		#define RFCOMM_LOCAL_CREDIT_SEND_THRESHOLD_DECL	const bt_byte _rfcomm_local_credit_send_threshold = (RFCOMM_LOCAL_CREDIT * 100 * 3 / 4) / 100;
	#else
		#define RFCOMM_LOCAL_CREDIT_SEND_THRESHOLD_DECL	const bt_byte _rfcomm_local_credit_send_threshold = 1;
	#endif
#else
	#if (RFCOMM_LOCAL_CREDIT_SEND_THRESHOLD > 1)
		#error "RFCOMM_LOCAL_CREDIT_SEND_THRESHOLD must not exceed 1"
	#endif

	#if	((RFCOMM_LOCAL_CREDIT * 100 * RFCOMM_LOCAL_CREDIT_SEND_THRESHOLD) / 100 != 0)
		#define RFCOMM_LOCAL_CREDIT_SEND_THRESHOLD_DECL	const bt_byte _rfcomm_local_credit_send_threshold = (RFCOMM_LOCAL_CREDIT * 100 * RFCOMM_LOCAL_CREDIT_SEND_THRESHOLD) / 100;
	#else
		#define RFCOMM_LOCAL_CREDIT_SEND_THRESHOLD_DECL	const bt_byte _rfcomm_local_credit_send_threshold = 1;
	#endif
#endif

#ifndef RFCOMM_ENABLE_MULTIDEVICE_CHANNELS
	/**
	* \brief  Enable multi-device server channels.
	* \ingroup rfcomm_config
	*
	* \details Normally each server channel can be used only once. I.e. if device A connected to channel 1, device B cannot connect to channel 1 until device A disconnects.
	*          With this option it is possible to make channels accept connections from several devices at the same time. I.e., if RFCOMM_ENABLE_MULTIDEVICE_CHANNELS is TRUE
	*          both device A and device B can connect to channel 1 at the same time.
	*/
	#define RFCOMM_ENABLE_MULTIDEVICE_CHANNELS BT_FALSE
#else
	#undef RFCOMM_ENABLE_MULTIDEVICE_CHANNELS
	#define RFCOMM_ENABLE_MULTIDEVICE_CHANNELS BT_TRUE
#endif


#if (RFCOMM_INFO_LEN + RFCOMM_FRAME_HEADER_LEN + L2CAP_HEADER_LEN > HCI_L2CAP_BUFFER_LEN)
#error "RFCOMM_INFO_LEN + RFCOMM_FRAME_HEADER_LEN(5) + L2CAP_HEADER_LEN(4) cannot exceed HCI_L2CAP_BUFFER_LEN"
#endif

#ifdef _DEBUG

#define RFCOMM_ALLOCATE_BUFFERS_RAM_SIZE_VAR	\
	const bt_uint _ram_size_rfcomm_buffers =	\
		sizeof(bt_buffer_mgr_t) * 2 +	\
		sizeof(_rfcomm_sessions) +	\
		sizeof(_rfcomm_max_sessions) +	\
		sizeof(_rfcomm_dlcs) +	\
		sizeof(_rfcomm_max_dlcs) +	\
		sizeof(_rfcomm_channels) +	\
		sizeof(_rfcomm_max_channels) +	\
		sizeof(_rfcomm_pdu_size) +	\
		sizeof(_rfcomm_info_len) +	\
		sizeof(_rfcomm_cmd_buffer_headers) +	\
		sizeof(_rfcomm_cmd_buffers) +	\
		sizeof(_rfcomm_local_credit) +	\
		sizeof(_rfcomm_max_cmd_buffers) +	\
		sizeof(_rfcomm_enable_multidevice_channels);

#else
	#define RFCOMM_ALLOCATE_BUFFERS_RAM_SIZE_VAR
#endif

#define RFCOMM_BUFFER_SIZE (RFCOMM_INFO_LEN)

#define RFCOMM_ALLOCATE_BUFFERS_VARS()	\
	bt_rfcomm_session_t _rfcomm_sessions[RFCOMM_MAX_SESSIONS];	\
	const bt_byte       _rfcomm_max_sessions = RFCOMM_MAX_SESSIONS;	\
	bt_rfcomm_dlc_t     _rfcomm_dlcs[(RFCOMM_MAX_DLCS) * (RFCOMM_MAX_SESSIONS)];	\
	const bt_byte       _rfcomm_max_dlcs = RFCOMM_MAX_DLCS;	\
	bt_rfcomm_server_channel_t _rfcomm_channels[(RFCOMM_MAX_SERVER_CHANNELS) * (RFCOMM_MAX_SESSIONS)];	\
	const bt_byte       _rfcomm_max_channels = RFCOMM_MAX_SERVER_CHANNELS;	\
	const bt_uint       _rfcomm_pdu_size = (RFCOMM_INFO_LEN) + (RFCOMM_FRAME_HEADER_LEN);	\
	const bt_uint       _rfcomm_info_len = RFCOMM_INFO_LEN;	\
	bt_buffer_header_t  _rfcomm_cmd_buffer_headers[RFCOMM_MAX_CMD_BUFFERS];	\
	bt_rfcomm_command_t _rfcomm_cmd_buffers[RFCOMM_MAX_CMD_BUFFERS];	\
	const bt_byte       _rfcomm_max_cmd_buffers = RFCOMM_MAX_CMD_BUFFERS;	\
	const bt_byte       _rfcomm_local_credit = RFCOMM_LOCAL_CREDIT;	\
	const bt_bool       _rfcomm_enable_multidevice_channels = RFCOMM_ENABLE_MULTIDEVICE_CHANNELS;	\
	RFCOMM_LOCAL_CREDIT_SEND_THRESHOLD_DECL	\
	RFCOMM_ALLOCATE_BUFFERS_RAM_SIZE_VAR	\


#define RFCOMM_ALLOCATE_BUFFERS_FUNCTION()	\
	void _rfcomm_allocate_buffers(void)	\
	{	\
	}	\

#define RFCOMM_ALLOCATE_BUFFERS()	\
		RFCOMM_ALLOCATE_BUFFERS_VARS()	\
		RFCOMM_ALLOCATE_BUFFERS_FUNCTION()	\
		typedef int RFCOMM_BUFFERS_ALLOCATED

#endif // __RFCOMM_CONFIG_H
