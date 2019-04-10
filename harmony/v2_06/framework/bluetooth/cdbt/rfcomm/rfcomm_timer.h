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

#ifndef __RFCOMM_TIMER
#define __RFCOMM_TIMER

#ifdef __cplusplus
extern "C" {
#endif

void _rfcomm_init_timer(void);
bt_long _rfcomm_get_tick_count(void);

#ifdef __cplusplus
}
#endif

#endif // __RFCOMM_TIMER
