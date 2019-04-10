/*******************************************************************************
* Contains proprietary and confidential information of SEARAN LLC.
* May not be used or disclosed to any other party except in accordance
* with a license from SEARAN LLC.
* Copyright (c) 2011-2016 SEARAN LLC. All Rights Reserved.
*******************************************************************************/

#ifndef __HCITR_BCSP_CONFIG_H
#define __HCITR_BCSP_CONFIG_H

#ifdef _DEBUG

#define HCITR_BCSP_ALLOCATE_BUFFERS_RAM_SIZE_VAR	\
	const bt_uint _ram_size_hcitr_bcsp_buffers =	\
		sizeof(_hcitr_bcsp_rx_buffer) +	\
		sizeof(_hcitr_bcsp_rx_buffer_size);

#else
	#define HCITR_BCSP_ALLOCATE_BUFFERS_RAM_SIZE_VAR
#endif

#ifndef HCITR_BCSP_RX_BUFFER_SIZE
	#define HCITR_BCSP_RX_BUFFER_SIZE    32
#endif


#define HCITR_BCSP_RX_BUFFER_SIZE_ALLOCATE_BUFFERS_VARS()	\
	bt_byte  _hcitr_bcsp_rx_buffer[HCITR_BCSP_RX_BUFFER_SIZE];	\
	bt_uint  _hcitr_bcsp_rx_buffer_size = HCITR_BCSP_RX_BUFFER_SIZE;	\
	\
	HCITR_BCSP_ALLOCATE_BUFFERS_RAM_SIZE_VAR	\

#define HCITR_BCSP_ALLOCATE_BUFFERS()	\
		HCITR_BCSP_RX_BUFFER_SIZE_ALLOCATE_BUFFERS_VARS()	\
		typedef int HCITR_BCSP_BUFFERS_ALLOCATED

#endif // __HCITR_BCSP_CONFIG_H
