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

#ifndef __SPP_CONFIG_H
#define __SPP_CONFIG_H

/**
 * \defgroup spp_config SPP Configuration
 * \ingroup spp
 *
 * This module describes parameters used to configure SPP layer.
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
	// for each used module. E.g., to use and configure SPP,
	// the following values must be defined:

	#define RFCOMM_MAX_SESSIONS            ...
	#define RFCOMM_MAX_DLCS                ...
	#define RFCOMM_MAX_SERVER_CHANNELS     ...
	#define RFCOMM_INFO_LEN                ...
	#define RFCOMM_MAX_DATA_BUFFERS        ...
	#define RFCOMM_MAX_CMD_BUFFERS         ...
	#define RFCOMM_LOCAL_CREDIT            ...

	#define SPP_MAX_PORTS                  ...

	#include "cdbt/bt/bt_oem_config.h"

	\endcode
 *
*/

#ifndef SPP_MAX_PORTS
#error "SPP_MAX_PORTS is not defined"

/**
* \brief  Maximum number of SPP ports.
* \ingroup spp_config
*
* \details This parameter defines the maximum number of SPP port that can be open between the local and remote devices.
*          If RFCOMM_ENABLE_MULTIDEVICE_CHANNELS is FALSE (default) this value should be equal to RFCOMM_MAX_SERVER_CHANNELS.
*          If RFCOMM_ENABLE_MULTIDEVICE_CHANNELS is TRUE this value should be between RFCOMM_MAX_SERVER_CHANNELS and RFCOMM_MAX_SERVER_CHANNELS * RFCOMM_MAX_SESSIONS.
*/
#define SPP_MAX_PORTS
#endif

#ifdef _DEBUG

#ifndef SPP_DISABLE_BUFFERING
#define SPP_FRAME_BUFFERS_RAM_SIZE	\
	sizeof(_spp_frame_buffers) +	\
	sizeof(_spp_frame_len) +	\
	SPP_FRAME_BUFFERS_SIZE
#else
#define SPP_FRAME_BUFFERS_RAM_SIZE	0
#endif

#define SPP_ALLOCATE_BUFFERS_RAM_SIZE_VAR	\
	const bt_uint _ram_size_spp_buffers =	\
		sizeof(_spp_ports) +	\
		sizeof(_spp_max_ports) +	\
		SPP_FRAME_BUFFERS_RAM_SIZE;

#else
	#define SPP_ALLOCATE_BUFFERS_RAM_SIZE_VAR
#endif

#ifndef SPP_DISABLE_BUFFERING
#define SPP_DISABLE_BUFFERING    0
#define SPP_DECLARE_FRAME_BUFFERS	\
	bt_byte           _spp_frame_buffers_mem[(RFCOMM_LOCAL_CREDIT) * (RFCOMM_INFO_LEN) * (RFCOMM_MAX_SESSIONS) * (SPP_MAX_PORTS)];	\
	bt_int            _spp_frame_len_mem[(RFCOMM_LOCAL_CREDIT) * (RFCOMM_MAX_SESSIONS) * (SPP_MAX_PORTS)];	\
	bt_byte*          _spp_frame_buffers = _spp_frame_buffers_mem;	\
	bt_int*           _spp_frame_len = _spp_frame_len_mem;
#define SPP_FRAME_BUFFERS_SIZE	sizeof(_spp_frame_buffers_mem) + sizeof(_spp_frame_len_mem)
#else
#define SPP_DECLARE_FRAME_BUFFERS	\
	bt_byte*          _spp_frame_buffers = NULL;	\
	bt_int*           _spp_frame_len = NULL;
#define SPP_FRAME_BUFFERS_SIZE
#undef SPP_DISABLE_BUFFERING
#define SPP_DISABLE_BUFFERING    1
#endif

#define SPP_ALLOCATE_BUFFERS_VARS()	\
	bt_spp_port_t     _spp_ports[SPP_MAX_PORTS];	\
	const bt_byte     _spp_max_ports = SPP_MAX_PORTS;	\
	const bt_byte     _spp_disable_buffering = SPP_DISABLE_BUFFERING;	\
	SPP_DECLARE_FRAME_BUFFERS	\
	SPP_ALLOCATE_BUFFERS_RAM_SIZE_VAR	\

#define SPP_ALLOCATE_BUFFERS()	\
		SPP_ALLOCATE_BUFFERS_VARS()	\
		typedef int SPP_BUFFERS_ALLOCATED

#endif // __SPP_CONFIG_H
