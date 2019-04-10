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

#ifndef __BT_APP_CONFIG_H
#define __BT_APP_CONFIG_H

#ifdef BT_INCLUDE_SPP
#ifndef BT_INCLUDE_RFCOMM
#define BT_INCLUDE_RFCOMM
#endif
#endif

#ifdef BT_BLE_SINGLE_MODE
#ifndef BT_ENABLE_BLE
#define BT_ENABLE_BLE
#endif
#endif

#include "cdbt/bt/bt_std.h"
#include "cdbt/hci/hci.h"
#include "cdbt/hci/hci_evt_handlers.h"
#include "cdbt/hci/hci_config.h"
#include "cdbt/hci/hcitr_bcsp_config.h"
#include "cdbt/l2cap/l2cap.h"
#include "cdbt/l2cap/l2cap_config.h"
#include "cdbt/sdp/sdp.h"
#include "cdbt/sdp/sdp_config.h"
#include "cdbt/bt/bt_log.h"

#ifndef BT_LOG_LEVEL
	#define BT_LOG_LEVEL_MIN    BT_LOG_LEVEL_INFO
	#define BT_LOG_LEVEL_MAX    BT_LOG_LEVEL_OFF
#elif defined(BT_LOG_LEVEL_MAX)
	#define BT_LOG_LEVEL_MIN    BT_LOG_LEVEL
#else
	#define BT_LOG_LEVEL_MIN    BT_LOG_LEVEL
	#define BT_LOG_LEVEL_MAX    BT_LOG_LEVEL_OFF
#endif
bt_byte _bt_log_level_min = BT_LOG_LEVEL_MIN;
bt_byte _bt_log_level_max = BT_LOG_LEVEL_MAX;

HCI_ALLOCATE_BUFFERS();

HCITR_BCSP_ALLOCATE_BUFFERS();

L2CAP_ALLOCATE_BUFFERS();

SDP_ALLOCATE_BUFFERS();

#ifdef BT_INCLUDE_RFCOMM
	#include "cdbt/rfcomm/rfcomm.h"
	#include "cdbt/rfcomm/rfcomm_config.h"

	RFCOMM_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_SPP
	#ifndef BT_INCLUDE_RFCOMM
	#error "BT_INCLUDE_RFCOMM required"
	#endif

	#include "cdbt/spp/spp.h"
	#include "cdbt/spp/spp_config.h"

	SPP_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_HFP
	#ifndef BT_INCLUDE_RFCOMM
	#error "BT_INCLUDE_RFCOMM required"
	#endif

	#include "cdbt/hfp/hfp.h"
	#include "cdbt/hfp/hfp_config.h"

	HFP_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_HSP
	#ifndef BT_INCLUDE_RFCOMM
	#error "BT_INCLUDE_RFCOMM required"
	#endif

	#include "cdbt/hsp/hsp.h"
	#include "cdbt/hsp/hsp_config.h"

	HSP_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_HSP_AG
	#ifndef BT_INCLUDE_SPP
	#error "BT_INCLUDE_SPP required"
	#endif

	#include "cdbt/hsp/hsp_ag.h"
	#include "cdbt/hsp/hsp_ag_config.h"

	HSP_AG_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_HID
	#include "cdbt/hid/hid.h"
	#include "cdbt/hid/hid_config.h"

	HID_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_OBEX
	#ifndef BT_INCLUDE_RFCOMM
	#error "BT_INCLUDE_RFCOMM required"
	#endif

	#include "cdbt/obex/obex.h"
	#include "cdbt/obex/obex_config.h"

	OBEX_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_PBAP
	#ifndef BT_INCLUDE_OBEX
	#error "BT_INCLUDE_OBEX required"
	#endif

	#include "cdbt/pbap/pbap.h"
	#include "cdbt/pbap/pbap_config.h"

	PBAP_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_FTP
	#ifndef BT_INCLUDE_OBEX
	#error "BT_INCLUDE_OBEX required"
	#endif

	#include "cdbt/ftp/ftp_config.h"

	FTP_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_MAP
	#ifndef BT_INCLUDE_OBEX
	#error "BT_INCLUDE_OBEX required"
	#endif

	#include "cdbt/map/map.h"
	#include "cdbt/map/map_config.h"

	MAP_ALLOCATE_BUFFERS();
#endif

#if defined(BT_INCLUDE_IAP) || defined(BT_INCLUDE_IAP2) ||  defined(BT_INCLUDE_IAPEA)
	#ifndef BT_INCLUDE_SPP
		#error "BT_INCLUDE_SPP required"
	#endif
#endif

#ifdef BT_INCLUDE_IAPEA
	#ifndef BT_INCLUDE_IAP
		#define BT_INCLUDE_IAP
	#endif
	#ifndef BT_INCLUDE_IAP2
		#define BT_INCLUDE_IAP2
	#endif

	#include "cdbt/iapea/iapea.h"
	#include "cdbt/iapea/iapea_config.h"

	IAPEA_ALLOCATE_BUFFERS();

	#ifdef IAP_BT_MAX_TRANSPORTS
		#undef IAP_BT_MAX_TRANSPORTS
	#endif
	#define IAP_BT_MAX_TRANSPORTS      IAPEA_MAX_SESSIONS

	#ifdef IAP_MAX_SESSIONS
		#undef IAP_MAX_SESSIONS
	#endif
	#define IAP_MAX_SESSIONS          IAPEA_MAX_SESSIONS

	#ifdef IAP2_MAX_SESSIONS
		#undef IAP2_MAX_SESSIONS
	#endif
	#define IAP2_MAX_SESSIONS         IAPEA_MAX_SESSIONS

	#ifndef IAP_RX_BUFFER_SIZE
		#define IAP_RX_BUFFER_SIZE    IAPEA_MAX_PACKET_SIZE
	#endif

	#ifndef IAP2_MAX_PACKET_SIZE
		#define IAP2_MAX_PACKET_SIZE  IAPEA_MAX_PACKET_SIZE
	#endif
#endif

#ifdef BT_INCLUDE_IAP
	#include "cdbt/iapbt/iap_bt.h"
	#include "cdbt/iap/iap_config.h"

	IAP_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_IAP2
	#include "cdbt/iapbt/iap_bt.h"
	#include "cdbt/iap2/iap2.h"
	#include "cdbt/iap2/iap2_config.h"
	#include "cdbt/iapbt/iap_bt_config.h"

	IAP2_ALLOCATE_BUFFERS();
#endif

#if defined(BT_INCLUDE_IAP) || defined(BT_INCLUDE_IAP2) ||  defined(BT_INCLUDE_IAPEA)
	#include "cdbt/iapbt/iap_bt.h"
	#include "cdbt/iapbt/iap_bt_config.h"

	IAP_BT_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_AVDTP
	#include "cdbt/avdtp/avdtp_config.h"

	AVDTP_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_AVCTP
#include "cdbt/avctp/avctp_config.h"

	AVCTP_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_AVRCP
#include "cdbt/avrcp/avrcp_config.h"

	AVRCP_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_ATT
	#include "cdbt/att/att_config.h"

	ATT_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_GATT_CLIENT
	#include "cdbt/att/att_client_config.h"
	#include "cdbt/gatt/gatt_client_config.h"
	#include "cdbt/gatt/gatt_ancs.h"

	ATT_CLIENT_ALLOCATE_BUFFERS();
	GATT_CLIENT_ALLOCATE_BUFFERS();

	#ifdef BT_INCLUDE_ATT_CLIENT
		#undef BT_INCLUDE_ATT_CLIENT
	#endif
#endif

#ifdef BT_INCLUDE_ATT_CLIENT
	#include "cdbt/att/att_client_config.h"

	ATT_CLIENT_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_SM
	#include "cdbt/sm/sm_config.h"

	SMP_ALLOCATE_BUFFERS();
#endif

#ifdef BT_INCLUDE_HCRP
	#include "cdbt/hcrp/hcrp_server.h"
	#include "cdbt/hcrp/hcrp_config.h"

	HCRP_ALLOCATE_BUFFERS();
#endif

#endif // __BT_APP_CONFIG_H

