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

#ifndef __UTILS_BUFFERUTILS_H
#define __UTILS_BUFFERUTILS_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define bt_min(a,b) (((a) < (b)) ? (a) : (b))
#ifdef __BT_PRIVATE_H_INCLUDED__
#ifdef min
#undef min
#endif
#define min(a,b) (using_min_is_dangerous = 0, use_bt_min_instead)
#endif

#define bt_max(a,b) (((a) > (b)) ? (a) : (b))
#ifdef __BT_PRIVATE_H_INCLUDED__
#ifdef max
#undef max
#endif
#define max(a,b) (using_max_is_dangerous = 0, use_bt_max_instead)
#endif

bt_bool _readb(bt_byte_p pvalue, bt_byte_cp pb, bt_int buf_len, bt_int_p offset);
bt_bool _readi(bt_int_p  pvalue, bt_byte_cp pb, bt_int buf_len, bt_int_p offset);
bt_bool _readl(bt_long_p pvalue, bt_byte_cp pb, bt_int buf_len, bt_int_p offset);

bt_bool _readin(bt_int_p  pvalue, bt_byte_cp pb, bt_int buf_len, bt_int_p offset);
bt_bool _readln(bt_long_p pvalue, bt_byte_cp pb, bt_int buf_len, bt_int_p offset);

bt_bool _writeb(bt_byte value, bt_byte_p pb, bt_int buf_len, bt_int_p offset);
bt_bool _writei(bt_int  value, bt_byte_p pb, bt_int buf_len, bt_int_p offset);
bt_bool _writel(bt_long value, bt_byte_p pb, bt_int buf_len, bt_int_p offset);
bt_bool _writes(const char* pstr, bt_byte_p pb, bt_int buf_len, bt_int_p offset);
bt_bool _writesx(const char* pstr, bt_int str_len, bt_byte_p pb, bt_int buf_len, bt_int_p offset);

bt_bool _writein(bt_int  value, bt_byte_p pb, bt_int buf_len, bt_int_p offset);
bt_bool _writeln(bt_long value, bt_byte_p pb, bt_int buf_len, bt_int_p offset);

#define _readbn		_readb
#define _writebn	_writeb

#ifdef BT_DO_NOT_USE_TYPE_CAST_MACROS
	bt_bool _readui(bt_uint_ptr  pvalue, bt_byte_cp pb, bt_int buf_len, bt_int_p offset);
	bt_bool _readul(bt_ulong_ptr pvalue, bt_byte_cp pb, bt_int buf_len, bt_int_p offset);
	bt_bool _readuin(bt_uint_ptr  pvalue, bt_byte_cp pb, bt_int buf_len, bt_int_p offset);
	bt_bool _readuln(bt_ulong_ptr pvalue, bt_byte_cp pb, bt_int buf_len, bt_int_p offset);
#else
	#define _readui(pvalue, pb, buf_len, offset)	_readi((bt_int_p)pvalue, pb, buf_len, offset)
	#define _readul(pvalue, pb, buf_len, offset)	_readl((bt_long_p)pvalue, pb, buf_len, offset)
	#define _readuin(pvalue, pb, buf_len, offset)	_readin((bt_int_p)pvalue, pb, buf_len, offset)
	#define _readuln(pvalue, pb, buf_len, offset)	_readln((bt_long_p)pvalue, pb, buf_len, offset)
#endif

void _compact_buffer(bt_byte_p pdst, bt_uint offset, bt_byte_cp psrc, bt_uint len);
void _expand_buffer(bt_byte_p pdst, bt_byte_cp psrc, bt_uint len, bt_uint offset);
void _bt_memcpy(bt_byte_p pdst, bt_uint dst_offset, bt_byte_cp psrc, bt_uint len, bt_uint offset);

const char* _ulong2str(bt_ulong data);
const char* _ulong2str_dec(bt_ulong data);
bt_ulong _str2ulong(const char* s);
bt_ulong _str2ulong_dec(const char* s);
bt_bool _is_empty_str(const char* s);

void _write_bdaddr(bt_bdaddr_cp pbdaddr, bt_byte_p buffer);
void _read_bdaddr(bt_bdaddr_p pbdaddr, bt_byte_cp buffer);

void _to_lower_case(bt_byte* str, bt_uint len);

void _zero_memory(void* ptr, size_t len);

#ifdef __cplusplus
}
#endif

#endif // __UTILS_BUFFERUTILS_H
