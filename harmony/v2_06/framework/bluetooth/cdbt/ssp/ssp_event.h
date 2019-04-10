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

#ifndef __SSP_EVENT_H
#define __SSP_EVENT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum _SSP_EVENT
{
	SSP_EVENT_IO_CAPABILITY_RESPONSE,
	SSP_EVENT_KEYPRESS_NOTIFICATION,
	SSP_EVENT_USER_PASSKEY_REQUEST,
	SSP_EVENT_USER_PASSKEY_NOTIFICATION,
	SSP_EVENT_USER_CONFIRMATION_REQUEST,
	SSP_EVENT_OOB_DATA_REQUEST,
	SSP_EVENT_IO_CAPABILITY_REQUEST,
	SSP_EVENT_SIMPLE_PAIRING_COMPLETE
} SSP_EVENT;

#ifdef __cplusplus
}
#endif

#endif //__SSP_EVENT_H
