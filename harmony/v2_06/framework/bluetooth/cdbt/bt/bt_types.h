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

#ifndef __BT_TYPES_H_INCLUDED__
#define __BT_TYPES_H_INCLUDED__

void test(void);


//#ifndef __BT_CONFIG_INCLUDED__
//#error bt_config.h must be included before bt_types.h.  Use bt_std.h to have both included in proper order.
//#endif

#include "cdbt/plat/types.h"

#define BT_TRUE   1
#define BT_FALSE  0

#define BT_YES    1
#define BT_NO     0

typedef bt_int*           bt_int_p;
typedef const bt_int*     bt_int_cp;
typedef bt_long*          bt_long_p;
typedef const bt_long*    bt_long_cp;
typedef bt_byte*          bt_byte_p;
typedef const bt_byte*    bt_byte_cp;
typedef bt_uint*          bt_uint_p;
typedef const bt_uint*    bt_uint_cp;
typedef bt_ulong*         bt_ulong_p;
typedef const bt_ulong*   bt_ulong_cp;
typedef bt_int            bt_id;
typedef bt_int            bt_bool;

typedef char              bt_char;
typedef bt_char*          bt_char_p;
typedef const bt_char*    bt_char_cp;

typedef bt_uint  bt_uuid16;
typedef bt_ulong bt_uuid32;

typedef struct _bt_uuid_s
{
	bt_ulong uuid0;
	bt_ulong uuid1;
	bt_ulong uuid2;
	bt_ulong uuid3;
} bt_uuid_t;
typedef bt_uuid_t*       bt_uuid_p;
typedef const bt_uuid_t* bt_uuid_cp;

typedef struct _bt_bdaddr_s
{
	bt_ulong bd_addr_l;
	bt_ulong bd_addr_m;
} bt_bdaddr_t;
typedef bt_bdaddr_t*       bt_bdaddr_p;
typedef const bt_bdaddr_t* bt_bdaddr_cp;

#define BT_MAKE_BDADDR(bdaddr, maddr, laddr)       bdaddr.bd_addr_m = maddr, bdaddr.bd_addr_l = laddr
#define BT_MAKE_BDADDR_LE(bdaddr, addr_type, maddr, laddr)    bdaddr.bd_addr_m = maddr | 0x80000000 | ((bt_ulong)addr_type << 30), bdaddr.bd_addr_l = laddr


#define BT_LINKKEY_LENGTH 16

typedef struct _bt_linkkey_t
{
	bt_byte data[BT_LINKKEY_LENGTH];
} bt_linkkey_t;
typedef bt_linkkey_t*       bt_linkkey_p;
typedef const bt_linkkey_t* bt_linkkey_cp;

#endif // __BT_TYPES_H_INCLUDED__
