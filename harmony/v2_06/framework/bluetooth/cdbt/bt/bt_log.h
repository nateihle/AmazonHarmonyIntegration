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

#ifndef __BT_LOG_H
#define __BT_LOG_H

/**
 * \defgroup log OEM - Logging Interface
 *
 * \details This module declares an interface that DotStack is using to output its debug
 * and diagnostic information. The application must implement this interface.
 *
 * Though the simplest implementation may consis of just empty stubs it is strongly
 * encouraged to provide a useful implementation that allows for viewing and capturing
 * of the data passed through this interface.
 */

#ifdef __cplusplus
extern "C" {
#endif


/**
 * \brief Output log message.
 * \ingroup log
 *
 * \details DotStack calls this function to output its debug information. Implementation
 * should output or store the specified message to whatever device or medium where it
 * can be examined and analyzed.
 */
void bt_oem_log_write(const char* msg);


void bt_log_msg(const char* msg);
void bt_log_int(const char* msg, int i);
void bt_log_bdaddr(const char* msg, const bt_bdaddr_t* addr);
void bt_log_memory(const char* msg, const bt_byte* ptr, bt_ulong count);
void bt_log_linkkey(const char* msg, const bt_linkkey_t* key);

#define BT_LOG_LEVEL_OFF            0xFF
#define BT_LOG_LEVEL_ERROR          240
#define BT_LOG_LEVEL_INFO           160
#define BT_LOG_LEVEL_DEBUG          80
#define BT_LOG_LEVEL_A2DP_PAKCET    10
#define BT_LOG_LEVEL_ALL            0

#ifdef BT_ENABLE_LOG

#define BT_LOG_EX(msg, level)                      if (_bt_log_level_min <= level && _bt_log_level_max >= level) bt_log_msg(msg)
#define BT_LOGINT_EX(msg, i, level)                if (_bt_log_level_min <= level && _bt_log_level_max >= level) bt_log_int(msg, i)
#define BT_LOGADDR_EX(msg, a, level)               if (_bt_log_level_min <= level && _bt_log_level_max >= level) bt_log_bdaddr(msg, a)
#define BT_LOGMEMORY_EX(msg, ptr, count, level)    if (_bt_log_level_min <= level && _bt_log_level_max >= level) bt_log_memory(msg, ptr, count)
#define BT_LOGLINKKEY_EX(msg, key, level)          if (_bt_log_level_min <= level && _bt_log_level_max >= level) bt_log_linkkey(msg, key)

#define BT_LOG(msg)                                BT_LOG_EX(msg, BT_LOG_LEVEL_INFO)
#define BT_LOGINT(msg, i)                          BT_LOGINT_EX(msg, i, BT_LOG_LEVEL_INFO)
#define BT_LOGADDR(msg, a)                         BT_LOGADDR_EX(msg, a, BT_LOG_LEVEL_INFO)
#define BT_LOGMEMORY(msg, ptr, count)              BT_LOGMEMORY_EX(msg, ptr, count, BT_LOG_LEVEL_INFO)
#define BT_LOGLINKKEY(msg, key)                    BT_LOGLINKKEY_EX(msg, key, BT_LOG_LEVEL_INFO)
#define BT_LOGWRITE(msg)                           bt_oem_log_write(msg)

extern bt_byte _bt_log_level_min;
extern bt_byte _bt_log_level_max;

#else
#define BT_LOG(msg)
#define BT_LOGINT(msg, i)
#define BT_LOGADDR(msg, a)
#define BT_LOGMEMORY(msg, ptr, len)
#define BT_LOGLINKKEY(msg, key)
#define BT_LOGWRITE(msg)

#define BT_LOG_EX(msg, level)
#define BT_LOGINT_EX(msg, i, level)
#define BT_LOGADDR_EX(msg, a, level)
#define BT_LOGMEMORY_EX(msg, ptr, count, level)
#define BT_LOGLINKKEY_EX(msg, key, level)
#endif

#ifdef __cplusplus
}
#endif

#endif // __BT_LOG_H
