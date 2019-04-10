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

#ifndef __UTILS_AT_PARSER_H
#define __UTILS_AT_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#define AT_EVT_RING          0
#define AT_EVT_OK            1
#define AT_EVT_ERROR         2
#define AT_EVT_CMD_CODE      3
#define AT_EVT_CMD_READ_CODE 4
#define AT_EVT_CMD_PARAM     5
#define AT_EVT_CMD_COMPLETED 6

#define ATCMD_BUFFER_LEN 20

struct _bt_at_parser_t;

typedef void (*bt_at_parser_callback_pf)(struct _bt_at_parser_t* parser, bt_byte evt, void* evt_param, void* callback_param);

typedef struct _bt_at_parser_t
{
	bt_byte state;
	bt_byte buffer[ATCMD_BUFFER_LEN + 1];
	bt_byte write_pos;
	bt_byte param_read_state;
	bt_byte param_read_nesting_level;

	bt_at_parser_callback_pf callback;
	void* callback_param;
} bt_at_parser_t;

void bt_at_parse_fragment(bt_at_parser_t* parser, bt_byte* data, bt_uint len);

void bt_at_parser_reset(bt_at_parser_t* parser);

#ifdef __cplusplus
}
#endif

#endif // __UTILS_AT_PARSER_H
