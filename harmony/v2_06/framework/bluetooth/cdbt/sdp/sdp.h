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

#ifndef __SDP_H
#define __SDP_H

#include "cdbt/l2cap/l2cap.h"
#include "cdbt/sdp/sdp_packet.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
* \defgroup sdp SDP
*
*  This module describe functions and data structures used to start the SDP server and perform SDP queries.
*/

#define SDP_CLSID_ServiceDiscoveryServerServiceClassID	0x1000
#define SDP_CLSID_BrowseGroupDescriptorServiceClassID	0x1001
#define SDP_CLSID_PublicBrowseGroup						0x1002
#define SDP_CLSID_SerialPort							0x1101
#define SDP_CLSID_DialupNetworking						0x1103
#define SDP_CLSID_OBEXObjectPush						0x1105
#define SDP_CLSID_OBEXFileTransfer						0x1106
#define SDP_CLSID_HSP									0x1108
#define SDP_CLSID_AUDIO_SOURCE							0x110A
#define SDP_CLSID_AUDIO_SINK							0x110B
#define SDP_CLSID_AV_REMOTE_CONTROL_TARGET				0x110C
#define SDP_CLSID_ADVANCED_AUDIO_DISTRIBUTION			0x110D
#define SDP_CLSID_AV_REMOTE_CONTROL						0x110E
#define SDP_CLSID_AV_REMOTE_CONTROL_PROFILE_ID			0x110E
#define SDP_CLSID_AV_REMOTE_CONTROL_CONTROLLER			0x110F
#define SDP_CLSID_AVCTP                                 0x17
#define SDP_CLSID_AVDTP                                 0x19
#define SDP_CLSID_HSP_AG								0x1112
#define SDP_CLSID_HFP									0x111E
#define SDP_CLSID_HFP_AG								0x111F
#define SDP_CLSID_HARD_COPY_CABLE_REPLACEMENT           0x1125
#define SDP_CLSID_HCR_PRINT                             0x1126
#define SDP_CLSID_HCR_SCAN                              0x1127
#define SDP_CLSID_PBAP_PCE								0x112E
#define SDP_CLSID_PBAP_PSE								0x112F
#define SDP_CLSID_HSP_HS								0x1131
#define SDP_CLSID_PNPInformation						0x1200
#define SDP_CLSID_L2CAP									0x100
#define SDP_CLSID_RFCOMM								0x3
#define SDP_CLSID_HID									0x1124
#define SDP_CLSID_HIDProtocol							0x11
#define SDP_CLSID_GENERIC_AUDIO							0x1203
#define SDP_CLSID_HDP									0x1400
#define SDP_CLSID_HDP_SOURCE							0x1401
#define SDP_CLSID_HDP_SINK								0x1402
#define SDP_CLSID_HARD_COPY_CONTROL_CHANNEL             0x0012
#define SDP_CLSID_HARD_COPY_DATA_CHANNEL                0x0014
#define SDP_CLSID_HARD_COPY_NOTIFICATION                0x0016

#define SDP_CLSID_MCAP_CONTROL							0x001E
#define SDP_CLSID_MCAP_DATA								0x001F

#define SDP_ATTRID_ServiceRecordHandle					0x0000
#define SDP_ATTRID_ServiceClassIDList					0x0001
#define SDP_ATTRID_ServiceRecordState					0x0002
#define SDP_ATTRID_ServiceID							0x0003
#define SDP_ATTRID_ProtocolDescriptorList				0x0004
#define SDP_ATTRID_BrowseGroupList						0x0005
#define SDP_ATTRID_LanguageBaseAttributeIDList			0x0006
#define SDP_ATTRID_ServiceInfoTimeToLive				0x0007
#define SDP_ATTRID_ServiceAvailability					0x0008
#define SDP_ATTRID_BluetoothProfileDescriptorList		0x0009
#define SDP_ATTRID_DocumentationURL						0x000A
#define SDP_ATTRID_ClientExecutableURL					0x000B
#define SDP_ATTRID_IconURL								0x000C
#define SDP_ATTRID_AdditionalProtocolDescriptorLists	0x000D
#define SDP_ATTRID_GroupID								0x0200
#define SDP_ATTRID_VersionNumberList					0x0200
#define SDP_ATTRID_ServiceDatabaseState					0x0201

#define SDP_ATTRID_PrimaryLanguageBaseId				0x0100
#define SDP_ATTRID_OFFSET_ServiceName					0x0000
#define SDP_ATTRID_OFFSET_ServiceDescription			0x0001
#define SDP_ATTRID_OFFSET_ProviderName					0x0002

#define SDP_ATTRID_HIDDeviceReleaseNumber				0x200
#define SDP_ATTRID_HIDParserVersion						0x201
#define SDP_ATTRID_HIDDeviceSubclass					0x202
#define SDP_ATTRID_HIDCountryCode						0x203
#define SDP_ATTRID_HIDVirtualCable						0x204
#define SDP_ATTRID_HIDReconnectInitiate					0x205
#define SDP_ATTRID_HIDDescriptorList					0x206
#define SDP_ATTRID_HIDLANGIDBaseList					0x207
#define SDP_ATTRID_HIDSDPDisable						0x208
#define SDP_ATTRID_HIDBatteryPower						0x209
#define SDP_ATTRID_HIDRemoteWake						0x20A
#define SDP_ATTRID_HIDProfileVersion					0x20B
#define SDP_ATTRID_HIDSupervisionTimeout				0x20C
#define SDP_ATTRID_HIDNormallyConnectable				0x20D
#define SDP_ATTRID_HIDBootDevice						0x20E

#define SDP_ATTRID_DISpecificationId					0x200
#define SDP_ATTRID_DIVendorId							0x201
#define SDP_ATTRID_DIProductId							0x202
#define SDP_ATTRID_DIVersion							0x203
#define SDP_ATTRID_DIPrimaryRecord						0x204
#define SDP_ATTRID_DIVendorIdSource						0x205

#define SDP_ATTRID_SupportedFeatures					0x311
#define SDP_ATTRID_HFPSupportedFeatures					0x311

#define SDP_ATTRID_HFPAGNetwork							0x301

#define SDP_ATTRID_GAPRemoteAudioVolumeControl			0x302

#define SDP_ATTRID_HDPSuportedFeatures					0x200
#define SDP_ATTRID_HDPDataExchangeSpecification			0x301
#define SDP_ATTRID_HDPMCAPSupportedProcedures			0x302

#define SDP_ATTRID_HCRP_1284ID                          0x300
#define SDP_ATTRID_HCRP_DeviceName                      0x302
#define SDP_ATTRID_HCRP_FriendlyName                    0x304
#define SDP_ATTRID_HCRP_DeviceLocation                  0x306

#define SDP_ATTRID_INVALID	0xFFFF

#define SDP_SR_HANDLE_SERVER			0x0
#define SDP_SR_HANDLE_RFCOMM			0x10000
#define SDP_SR_HANDLE_HID				0x10001
#define SDP_SR_HANDLE_PNPINFORMATION	0x10002
#define SDP_SR_HANDLE_TEST				0x10003 // for simulating service search using continuation state
#define SDP_SR_HANDLE_HSP_HS			0x10004
#define SDP_SR_HANDLE_HID_KEYBOARD		0x10005
#define SDP_SR_HANDLE_OBEXFileTransfer	0x10006
#define SDP_SR_HANDLE_OBEXObjectPush	0x10007
#define SDP_SR_HANDLE_HDP_SOURCE		0x10008
#define SDP_SR_HANDLE_HDP_SINK			0x10009
#define SDP_SR_HANDLE_HFP_HF			0x1000A

#define SDP_DATA_TYPE_NIL			0
#define SDP_DATA_TYPE_UINT			1
#define SDP_DATA_TYPE_UINT8			SDP_DATA_TYPE_UINT
#define SDP_DATA_TYPE_UINT16		0x11
#define SDP_DATA_TYPE_UINT32		0x21
#define SDP_DATA_TYPE_UINT64		0x41
#define SDP_DATA_TYPE_UINT128		0x81
#define SDP_DATA_TYPE_INT			2
#define SDP_DATA_TYPE_INT8			SDP_DATA_TYPE_INT
#define SDP_DATA_TYPE_INT16			0x12
#define SDP_DATA_TYPE_INT32			0x22
#define SDP_DATA_TYPE_INT64			0x42
#define SDP_DATA_TYPE_INT128		0x82
#define SDP_DATA_TYPE_UUID			3
#define SDP_DATA_TYPE_UUID16		SDP_DATA_TYPE_UUID
#define SDP_DATA_TYPE_UUID32		0x13
#define SDP_DATA_TYPE_UUID128		0x23
#define SDP_DATA_TYPE_STRING		4
#define SDP_DATA_TYPE_BOOL			5
#define SDP_DATA_TYPE_SEQUENCE		6
#define SDP_DATA_TYPE_ALTERNATIVE	7
#define SDP_DATA_TYPE_URL			8

#define SDP_MAX_DATA_ELEMENT_LEN	256
#define SDP_MAX_DATA_ELEMENTS		10

#define SDP_ErrorResponse						0x01
#define SDP_ServiceSearchRequest				0x02
#define SDP_ServiceSearchResponse				0x03
#define SDP_ServiceAttributeRequest				0x04
#define SDP_ServiceAttributeResponse			0x05
#define SDP_ServiceSearchAttributeRequest		0x06
#define SDP_ServiceSearchAttributeResponse		0x07

#define SDP_ERROR_RESERVED						0x00
#define SDP_ERROR_INVALID_SDP_VERSION			0x01
#define SDP_ERROR_INVALID_SR_HANDLE				0x02
#define SDP_ERROR_INVALID_REQUEST_SYNTAX		0x03
#define SDP_ERROR_INVALID_PDU_SIZE				0x04
#define SDP_ERROR_INVALID_CONTINUATION_STATE	0x05
#define SDP_ERROR_INSUFFICIENT_RESOURCE			0x06


#define SDP_MAX_SEARCH_PATTERN_LEN		12
#define SDP_MAX_ATTRIBUTE_PATTERN_LEN	12

#define SDP_PDU_HEADER_LEN	5

#define SDP_RFCOMM_SERVICE_ID	0x1234
#define SDP_HID_SERVICE_ID		0x1235
#define SDP_HSP_HS_SERVICE_ID	0x1236
#define SDP_HSP_AG_SERVICE_ID	0x1236
#define SDP_FTP_SERVICE_ID		0x1237

typedef bt_long bt_sr_handle_t, *bt_sr_handle_p;

struct _bt_sdp_sequence_t;
typedef struct _bt_sdp_data_element_t {
	bt_int type;
	bt_int bytecount;

	union {
		bt_ulong     init;
		void*        pdata;
		char*        pstr;
		char*        purl;
		struct _bt_sdp_sequence_t* pseq;
		bt_byte      b;
		bt_int       i;
		bt_uint      ui;
		bt_long      l;
		bt_ulong     ul;
		bt_uuid16    uuid16;
		bt_uuid32    uuid32;
		bt_uuid_p    uuid128;
	} data;

} bt_sdp_data_element_t;
typedef bt_sdp_data_element_t*       bt_sdp_data_element_p;
typedef const bt_sdp_data_element_t* bt_sdp_data_element_cp;

typedef struct _bt_sdp_sequence_t 
{
	bt_int count;
	bt_sdp_data_element_p elements;
} bt_sdp_sequence_t;
typedef bt_sdp_sequence_t* bt_sdp_sequence_p;
typedef const bt_sdp_sequence_t* bt_sdp_sequence_cp;

/*
typedef struct _sdp_attribute {
	bt_sdp_data_element attr_id;
	bt_sdp_data_element attr_value;
} bt_sdp_attribute, *bt_sdp_attribute_p;
*/

/*
typedef struct _sdp_service_record {
	bt_sr_handle h;
	bt_int attr_count;
	bt_sdp_attribute_p attrs;
} bt_sdp_service_record, *bt_sdp_service_record_p;
*/

/*
typedef struct _sdp_db {
	bt_int count;
	bt_sdp_service_record_p *records;
} bt_sdp_db, *bt_sdp_db_p;
*/


#define SDP_MAX_TRANSACTIONS	2

typedef struct _bt_sdp_serialization_state_t 
{
	bt_int listIndex;
	bt_int attrIndex;
	bt_byte attrPart;
	bt_int attrPartPos;
	bt_int attr_bytes;
	bt_int first_list;
	bt_int last_list;
	bt_int first_attr;
	bt_int last_attr;
	bt_uint attr_lists_seq_len;
	bt_bool write_seq_len;
} bt_sdp_serialization_state_t, *bt_sdp_serialization_state_p;

typedef struct _bt_sdp_found_attr_list_t
{
	bt_int found_attr_count;
	const bt_byte** found_attr_list;//SDP_MAX_ATTRIBUTE_RESULT_LEN];
} bt_sdp_found_attr_list_t;

typedef struct _bt_sdp_transaction_t
{
	bt_int id;
	bt_int found_list_count;
	bt_sdp_found_attr_list_t* found_attr_lists;//SDP_MAX_SEARCH_RESULT_LEN];
	bt_uint max_bytecount;
	bt_bool complete;
	bt_bool continuation;

	bt_sdp_serialization_state_t sr_state;
} bt_sdp_transaction_t;

typedef struct _bt_sdp_service_transaction_t 
{
	bt_int id;
	bt_int found_sr_count;
	bt_sr_handle_t* found_sr_list;//SDP_MAX_SEARCH_RESULT_LEN];
	bt_uint max_sr_count;
	bt_int next_tran_index;

} bt_sdp_service_transaction_t, *bt_sdp_service_transaction_p;


/**
 * \brief Begin a data element sequence
 * \ingroup sdp
 *
 * \details BEGIN_DE_SEQUENCE and END_DE_SEQUENCE are used to define a data element sequence which is an array of sdp_data_element structures.
 *          The array is used a search pattern in bt_sdp_request_service_search() and bt_sdp_request_service_attribute().
 *          For example, to find a AVRCP Target the following code can be used:

	\code
	const bt_uuid_t AVRCP_AV_REMOTE_CONTROL_CLSID = { 0x5F9B34FB, 0x80000080, 0x00001000, SDP_CLSID_AV_REMOTE_CONTROL };
	const bt_uuid_t AVRCP_AV_REMOTE_CONTROL_TARGET_CLSID = { 0x5F9B34FB, 0x80000080, 0x00001000, SDP_CLSID_AV_REMOTE_CONTROL_TARGET };

	BEGIN_DE_SEQUENCE(avrcp_target_service_search, 2)
		DE_UUID128(AVRCP_AV_REMOTE_CONTROL_CLSID)
		DE_UUID128(AVRCP_AV_REMOTE_CONTROL_TARGET_CLSID)
	END_DE_SEQUENCE(avrcp_target_service_search)

	.
	.
	.

	void findAvrcpTarget(void)
	{
		INIT_DE_SEQUENCE(avrcp_target_service_search);

		bt_sdp_request_service_search(channel, &seq_avrcp_target_service_search, &callback, NULL);
	}

	\endcode


 *
 * \param id The data element sequence identifier. 
 * \param len The number of elements in the data element sequence. 
 * 
 */
#define BEGIN_DE_SEQUENCE(id, len)	\
	static bt_sdp_data_element_t id[len];	\
	static bt_sdp_sequence_t seq_##id = { len, (bt_sdp_data_element_p)id};	\
	static void init_de_sequence_##id() {	\
		static bt_bool initialized = FALSE;	\
		if (!initialized) {	\
			bt_int i = 0;	\
			bt_int max_len = len;	\
			bt_sdp_data_element_t* cur_de = id;	\
			initialized = TRUE;

/**
 * \brief End a data element sequence
 * \ingroup sdp
 *
 * \details BEGIN_DE_SEQUENCE and END_DE_SEQUENCE are used to define a data element sequence which is an array of bt_sdp_data_element structures.
 *
 * \param id The data element sequence identifier. 
 * 
 */
#define END_DE_SEQUENCE(id)		}}

/**
 * \brief Initialize a data element sequence
 * \ingroup sdp
 *
 * \details This macro calls a function defined in BEGIN_DE_SEQUENCE which initializes the data element sequence.
 *
 * \param id The data element sequence identifier. 
 * 
 */
#define INIT_DE_SEQUENCE(id)	init_de_sequence_##id();

/**
 * \brief Declare a 1-byte unsigned integer data element
 * \ingroup sdp
 * 
 * \details This macro adds a 1-byte unsigned integer data element to a data element sequence. 
 * This macro is to be used between BEGIN_DE_SEQUENCE and END_DE_SEQUENCE.
 *
 * \param value The data element value. 
 * 
 */
#define DE_UINT(value)		cur_de->type = SDP_DATA_TYPE_UINT; cur_de->data.b = value; cur_de++; if (++i == max_len) return;

/**
 * \brief Declare a 2-byte unsigned integer data element
 * \ingroup sdp
 * 
 * \details This macro adds a 2-byte unsigned integer data element to a data element sequence. 
 * This macro is to be used between BEGIN_DE_SEQUENCE and END_DE_SEQUENCE.
 *
 * \param value The data element value. 
 * 
 */
#define DE_UINT16(value)	cur_de->type = SDP_DATA_TYPE_UINT16; cur_de->data.ui = value; cur_de++; if (++i == max_len) return;

/**
 * \brief Declare a 1-byte signed integer data element
 * \ingroup sdp
 * 
 * \details This macro adds a 1-byte signed integer data element to a data element sequence. 
 * This macro is to be used between BEGIN_DE_SEQUENCE and END_DE_SEQUENCE.
 *
 * \param value The data element value. 
 * 
 */
#define DE_INT(value)		cur_de->type = SDP_DATA_TYPE_INT; cur_de->data.b = value; cur_de++; if (++i == max_len) return;

/**
 * \brief Declare a text string data element
 * \ingroup sdp
 * 
 * \details This macro adds a text string data element to a data element sequence. 
 * The length of the generated data element will be the actual length of the string.
 * This macro is to be used between BEGIN_DE_SEQUENCE and END_DE_SEQUENCE.
 *
 * \param value The data element value. 
 * 
 */
#define DE_STRING(value)	cur_de->type = SDP_DATA_TYPE_STRING; cur_de->data.pstr = value; cur_de++;

/**
 * \brief Declare a text string data element
 * \ingroup sdp
 * 
 * \details This macro adds a text string data element to a data element sequence. 
 * The length of the generated data element will be the value specified by the "len" parameter 
 * even if the actual length of the string is not equal to the "len" value.
 * This macro is to be used between BEGIN_DE_SEQUENCE and END_DE_SEQUENCE.
 *
 * \param value The data element value. 
 * \param len The length of the data element value. 
 * 
 */
#define DE_STRING2(value, len)	cur_de->type = SDP_DATA_TYPE_UINT; cur_de->data.pstr = value; cur_de->bytecount = len; cur_de++; if (++i == max_len) return;

/**
 * \brief Declare a boolean data element
 * \ingroup sdp
 * 
 * \details This macro adds a boolean data element to a data element sequence. 
 * This macro is to be used between BEGIN_DE_SEQUENCE and END_DE_SEQUENCE.
 *
 * \param value The data element value. 
 * 
 */
#define DE_BOOL(value)		cur_de->type = SDP_DATA_TYPE_BOOL; cur_de->data.b = value; cur_de++; if (++i == max_len) return;

/**
 * \brief Declare a 16-bit UUID data element
 * \ingroup sdp
 * 
 * \details This macro adds a 16-bit UUID data element to a data element sequence. 
 * This macro is to be used between BEGIN_DE_SEQUENCE and END_DE_SEQUENCE.
 *
 * \param value The data element value. 
 * 
*/
#define DE_UUID16(value)	cur_de->type = SDP_DATA_TYPE_UUID16; cur_de->data.uuid16 = value; cur_de++; if (++i == max_len) return;

/**
 * \brief Declare a 32-bit UUID data element
 * \ingroup sdp
 * 
 * \details This macro adds a 32-bit UUID data element to a data element sequence. 
 * This macro is to be used between BEGIN_DE_SEQUENCE and END_DE_SEQUENCE.
 *
 * \param value The data element value. 
 * 
 */
#define DE_UUID32(value)	cur_de->type = SDP_DATA_TYPE_UUID32; cur_de->data.uuid32 = value; cur_de++; if (++i == max_len) return;

/**
 * \brief Declare a 128-bit UUID data element
 * \ingroup sdp
 * 
 * \details This macro adds a 128-bit UUID data element to a data element sequence. 
 * This macro is to be used between BEGIN_DE_SEQUENCE and END_DE_SEQUENCE.
 *
 * \param value The data element value. The value must be a name of a variable of type bt_uuid. 
 * 
 */
#define DE_UUID128(value)	cur_de->type = SDP_DATA_TYPE_UUID128; cur_de->data.uuid128 = (bt_uuid_t*)&value; cur_de++; if (++i == max_len) return;

/**
 * \brief Declare a URL data element
 * \ingroup sdp
 * 
 * \details This macro adds a URL data element to a data element sequence. 
 * This macro is to be used between BEGIN_DE_SEQUENCE and END_DE_SEQUENCE.
 *
 * \param value The data element value which is a pointer to a string.
 * 
 */
#define DE_URL(value)		cur_de->type = SDP_DATA_TYPE_URL; cur_de->data.purl = value; cur_de++; if (++i == max_len) return;


/**
 * \brief Start SDP server
 * \ingroup sdp
 *
 * \details This function starts the SDP server.
 *
 * \param l2cap_mgr The L2CAP manager on which the SDP server is to be started.
 * \param sdp_db A pointer to the SDP database define with BEGIN_SDP_DB and END_SDP_DB macros.
 * 
 * \return
 *        \li \c TRUE if the function succeeds.
 *        \li \c FALSE otherwise.
 */
bt_bool bt_sdp_start(bt_l2cap_mgr_p l2cap_mgr, const bt_byte* sdp_db, bt_uint sdp_db_len);

#include "cdbt/sdp/sdp_private.h"

#ifdef __cplusplus
}
#endif

#endif // __SDP_H

