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

#ifndef __SDP_CONFIG_H
#define __SDP_CONFIG_H

/**
 * \defgroup sdp_config Configuration
 * \ingroup sdp
 *
 * This module describes parameters used to configure SDP.
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

	#include "cdbt/bt/bt_oem_config.h"

	\endcode
 *
*/

#ifdef BT_BLE_SINGLE_MODE

	#ifdef _DEBUG

	#define SDP_ALLOCATE_BUFFERS_RAM_SIZE_VAR	\
		const bt_uint _ram_size_sdp_buffers =	\
			sizeof(_sdp_start_fp);
	#else
		#define SDP_ALLOCATE_BUFFERS_RAM_SIZE_VAR
	#endif

	#define SDP_ALLOCATE_BUFFERS_VARS()	\
		bt_bool                  (*_sdp_start_fp)(bt_l2cap_mgr_p l2cap_mgr, const bt_byte* sdp_db, bt_uint sdp_db_len) = NULL;	\
		SDP_ALLOCATE_BUFFERS_RAM_SIZE_VAR	\

	#define SDP_ALLOCATE_BUFFERS()	\
		SDP_ALLOCATE_BUFFERS_VARS()	\
		typedef int SDP_BUFFERS_ALLOCATED

#else

	#ifndef SDP_MAX_PDU_BUFFERS
		#error "SDP_MAX_PDU_BUFFERS is not defined"

		/**
		* \brief  Maximum number of SDP server PDU buffers.
		* \ingroup sdp_config
		*
		* \details This parameter defines the maximum number of responses the SDP server can send at the same time.
		*/
		#define SDP_MAX_PDU_BUFFERS
	#endif

	#ifdef _DEBUG

	#define SDP_ALLOCATE_BUFFERS_RAM_SIZE_VAR	\
		const bt_uint _ram_size_sdp_buffers =	\
			sizeof(bt_buffer_mgr_t) +	\
			sizeof(_sdp_packet_buffer_headers) +	\
			sizeof(_sdp_packet_buffers) +	\
			sizeof(_sdp_max_buffers) +	\
			sizeof(_sdp_max_search_result_len) + \
			sizeof(_sdp_max_attribute_result_len) + \
			sizeof(_sdp_found_sr_lists_buffers) + \
			sizeof(_sdp_found_attr_lists_buffers) + \
			sizeof(_sdp_found_attr_list_buffers) +	\
			sizeof(_sdp_tran_buffer_mgr2) +	\
			sizeof(_sdp_tran_buffer_headers2) +	\
			sizeof(_sdp_tran_buffers2) +	\
			sizeof(_sdp_service_tran_buffer_mgr) +	\
			sizeof(_sdp_service_tran_buffer_headers) +	\
			sizeof(_sdp_service_tran_buffers) +	\
			sizeof(_sdp_start_fp);

	#else
		#define SDP_ALLOCATE_BUFFERS_RAM_SIZE_VAR
	#endif

	#ifndef SDP_MAX_SEARCH_RESULT_LEN
		#error "SDP_MAX_SEARCH_RESULT_LEN is not defined"

		/**
		* \brief  Maximum number of service records to find.
		* \ingroup sdp_config
		*
		* \details This parameter defines the maximum number of service records the SDP server will return to the client.
		*/
		#define SDP_MAX_SEARCH_RESULT_LEN
	#endif

	#ifndef SDP_MAX_ATTRIBUTE_RESULT_LEN
		#error "SDP_MAX_ATTRIBUTE_RESULT_LEN is not defined"
	
		/**
		* \brief  Maximum number of attributes to find
		* \ingroup sdp_config
		*
		* \details This parameter defines the maximum number of attributes withing a service record the SDP server will return to the client.
		*/
		#define SDP_MAX_ATTRIBUTE_RESULT_LEN
	#endif

	#if (SDP_MAX_SEARCH_RESULT_LEN < 1)
	#error "SDP_MAX_SEARCH_RESULT_LEN must be >= 1"
	#endif

	#if (SDP_MAX_ATTRIBUTE_RESULT_LEN < 1)
	#error "SDP_MAX_ATTRIBUTE_RESULT_LEN must be >= 1"
	#endif

	#define SDP_ALLOCATE_BUFFERS_VARS()	\
		bt_buffer_header_t           _sdp_packet_buffer_headers[SDP_MAX_PDU_BUFFERS];	\
		bt_sdp_packet_t              _sdp_packet_buffers[SDP_MAX_PDU_BUFFERS];	\
		const bt_byte                _sdp_max_buffers = SDP_MAX_PDU_BUFFERS; \
		bt_buffer_header_t           _sdp_client_packet_buffer_headers[SDP_MAX_PDU_BUFFERS];	\
		bt_sdp_packet_t              _sdp_client_packet_buffers[SDP_MAX_PDU_BUFFERS];	\
		const bt_byte                _sdp_client_max_buffers = SDP_MAX_PDU_BUFFERS; \
		const bt_uint                _sdp_max_search_result_len = SDP_MAX_SEARCH_RESULT_LEN;	\
		const bt_uint                _sdp_max_attribute_result_len = SDP_MAX_ATTRIBUTE_RESULT_LEN;	\
		bt_sr_handle_t               _sdp_found_sr_lists_buffers[SDP_MAX_TRANSACTIONS * SDP_MAX_SEARCH_RESULT_LEN];	\
		bt_sdp_found_attr_list_t     _sdp_found_attr_lists_buffers[SDP_MAX_TRANSACTIONS * SDP_MAX_SEARCH_RESULT_LEN];	\
		bt_byte*                     _sdp_found_attr_list_buffers[SDP_MAX_TRANSACTIONS * SDP_MAX_SEARCH_RESULT_LEN * SDP_MAX_ATTRIBUTE_RESULT_LEN];	\
		\
		bt_buffer_mgr_t              _sdp_tran_buffer_mgr2;	\
		bt_buffer_header_t           _sdp_tran_buffer_headers2[SDP_MAX_TRANSACTIONS];	\
		bt_sdp_transaction_t         _sdp_tran_buffers2[SDP_MAX_TRANSACTIONS];	\
		\
		bt_buffer_mgr_t              _sdp_service_tran_buffer_mgr;	\
		bt_buffer_header_t           _sdp_service_tran_buffer_headers[SDP_MAX_TRANSACTIONS];	\
		bt_sdp_service_transaction_t _sdp_service_tran_buffers[SDP_MAX_TRANSACTIONS];	\
		\
		bt_bool                  (*_sdp_start_fp)(bt_l2cap_mgr_p l2cap_mgr, const bt_byte* sdp_db, bt_uint sdp_db_len) = &bt_sdp_start;	\
		SDP_ALLOCATE_BUFFERS_RAM_SIZE_VAR	\

	#define SDP_ALLOCATE_BUFFERS()	\
			SDP_ALLOCATE_BUFFERS_VARS()	\
			typedef int SDP_BUFFERS_ALLOCATED

#endif

#endif // __SDP_CONFIG_H
