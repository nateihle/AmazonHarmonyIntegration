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

#ifndef __UTILS_VCARD_PARSER_H
#define __UTILS_VCARD_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#define	VCARD_PROPERTY_BEGIN		0
#define	VCARD_PROPERTY_END			1
#define	VCARD_PROPERTY_VERSION		2 // vCard Version
#define	VCARD_PROPERTY_FN			3 // Formatted Name
#define	VCARD_PROPERTY_N			4 // Structured Presentation of Name
#define	VCARD_PROPERTY_PHOTO		5 // Associated Image or Photo
#define	VCARD_PROPERTY_BDAY			6 // Birthday
#define	VCARD_PROPERTY_ADR			7 // Delivery Address
#define	VCARD_PROPERTY_LABEL		8 // Delivery
#define	VCARD_PROPERTY_TEL			9 // Telephone Number
#define	VCARD_PROPERTY_EMAIL		10 // Electronic Mail Address
#define	VCARD_PROPERTY_MAILER		11 // Electronic Mail
#define	VCARD_PROPERTY_TZ			12 // Time Zone
#define	VCARD_PROPERTY_GEO			13 // Geographic Position
#define	VCARD_PROPERTY_TITLE		14 // Job
#define	VCARD_PROPERTY_ROLE			15 // Role within the Organization
#define	VCARD_PROPERTY_LOGO			16 // Organization Logo
#define	VCARD_PROPERTY_AGENT		17 // vCard of Person Representing
#define	VCARD_PROPERTY_ORG			18 // Name of Organization
#define	VCARD_PROPERTY_NOTE			19 // Comments
#define	VCARD_PROPERTY_REV			20 // Revision
#define	VCARD_PROPERTY_SOUND		21 // Pronunciation of Name
#define	VCARD_PROPERTY_URL			22 // Uniform Resource Locator
#define	VCARD_PROPERTY_UID			23 // Unique ID
#define	VCARD_PROPERTY_KEY			24 // Public Encryption Key
#define	VCARD_PROPERTY_NICKNAME		25 // Nickname
#define	VCARD_PROPERTY_CATEGORIES	26 // Categories
#define	VCARD_PROPERTY_PROID		27 // Product ID
#define	VCARD_PROPERTY_CLASS		28 // Class information
#define	VCARD_PROPERTY_SORT_STRING	29 // String used for sorting operations
#define	VCARD_PROPERTY_X_IRMC_CALL_DATETIME	30 // Time stamp

#define VCARD_PARAM_TYPE		0
#define VCARD_PARAM_ENCODING	1
#define VCARD_PARAM_VALUE		2
#define VCARD_PARAM_LANGUAGE	3
#define VCARD_PARAM_DIALED		4
#define VCARD_PARAM_RECEIVED	5
#define VCARD_PARAM_MISSED		6
#define VCARD_PARAM_CELL        7
#define VCARD_PARAM_HOME        8
#define VCARD_PARAM_WORK        9

#define VCARD_PARAM_CALL_TYPE	0x80

#define VCARD_CALL_TYPE_DIALED		0
#define VCARD_CALL_TYPE_RECEIVED	1
#define VCARD_CALL_TYPE_MISSED		2


#define VCARD_PARSER_STATE_READ_TYPE_NAME	0
#define VCARD_PARSER_STATE_READ_PARAM_NAME	1
#define VCARD_PARSER_STATE_READ_TYPE_VALUE	2
#define VCARD_PARSER_STATE_READ_PARAM_VALUE	3
#define VCARD_PARSER_STATE_SKIP_PARAM		4
#define VCARD_PARSER_STATE_SKIP_TYPE		5

#define VCARD_EVT_VCARD_STARTED			0
#define VCARD_EVT_VCARD_ENDED			1
#define VCARD_EVT_PROPERTY_STARTED		2
#define VCARD_EVT_PROPERTY_PARAM		3
#define VCARD_EVT_PROPERTY_VALUE		4

struct _bt_vcard_parser_t;

typedef void (*bt_vcard_parser_callback_fp)(struct _bt_vcard_parser_t* parser, bt_byte evt, void* evt_param, void* callback_param);

typedef struct _bt_vcard_evt_prop_t
{
	bt_byte prop_id;
	bt_byte* prop_value;
	bt_byte value_len;
	bt_bool final;
} bt_vcard_evt_prop_t;

typedef struct _bt_vcard_evt_prop_param_t
{
	bt_byte param_id;
	bt_byte* param_value;
	bt_byte param_len;
} bt_vcard_evt_prop_param_t;

typedef struct _bt_vcard_parser_t
{
	bt_byte state;
	bt_byte* buffer;
	bt_uint buffer_size;
	bt_byte write_pos;
	bt_byte prev_c;
	bt_byte cur_prop_id;
	bt_byte cur_param_id;
	bt_bool vcard_started;

	bt_vcard_parser_callback_fp callback;
	void* callback_param;
} bt_vcard_parser_t;

void bt_vcard_parse_fragment(bt_vcard_parser_t* parser, bt_byte* data, bt_uint len);

void bt_vcard_parser_reset(bt_vcard_parser_t* parser);

#ifdef __cplusplus
}
#endif

#endif // __UTILS_VCARD_PARSER_H
