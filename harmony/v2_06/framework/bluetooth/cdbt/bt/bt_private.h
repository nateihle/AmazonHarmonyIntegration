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

#ifndef __BT_PRIVATE_H_INCLUDED__

// Note: the following symbol is also used in other places to detect if
// we are building the library. If you change it, change the other places
// too.
#define __BT_PRIVATE_H_INCLUDED__


#define ARG_NOT_USED(arg) ((void)(arg))

#ifdef NDEBUG
#define BT_ASSERT(test)  ((void)0)
#else
#ifdef BT_USE_SYSTEM_ASSERT
#include <assert.h>
#define BT_ASSERT(test)  assert(test)
#else
#include "cdbt/bt/bt_oem.h"
#define BT_ASSERT(test)  ((test) ? (void)0 : bt_oem_assert(__FILE__,__LINE__))
#endif
#endif

#ifndef NULL
#undef NULL
#define NULL 0
#endif

#ifdef TRUE
#undef TRUE
#endif
#ifdef FALSE
#undef FALSE
#endif

#define TRUE  1
#define FALSE 0


#include "cdbt/bt/bt_log.h"

#define LOG(msg)                              BT_LOG(msg)
#define LOGINT(msg, i)                        BT_LOGINT(msg, i)
#define LOGADDR(msg, a)                       BT_LOGADDR(msg, a)
#define LOGWRITE(msg)                         BT_LOGWRITE(msg)
#define LOGMEMORY(msg, ptr, count)            BT_LOGMEMORY(msg, ptr, count)
#define LOGCLEAR()                            BT_LOGCLEAR()

#define LOG_EX(msg, level)                    BT_LOG_EX(msg, level)
#define LOGINT_EX(msg, i, level)              BT_LOGINT_EX(msg, i, level)
#define LOGADDR_EX(msg, a, level)             BT_LOGADDR_EX(msg, a, level)
#define LOGMEMORY_EX(msg, ptr, count, level)  BT_LOGMEMORY_EX(msg, ptr, count, level)

#endif // __BT_PRIVATE_H_INCLUDED__
