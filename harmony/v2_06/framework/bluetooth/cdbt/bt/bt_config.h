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

#ifndef __BT_CONFIG_H_INCLUDED__
#define __BT_CONFIG_H_INCLUDED__

#include "cdbt/plat/config.h"

/**
 * \defgroup btconfig DotStack Configuration
 */

/**
 * \brief BT_ENABLE_SCO.
 * \ingroup btconfig
 *
 * \details Enables support of synchronous connections.
 * HCI_MAX_CONNECTIONS must be at least 2
 */
//#define BT_ENABLE_SCO
//#if defined(BT_ENABLE_SCO) && HCI_MAX_CONNECTIONS < 2
//#error "SCO links require at least 2 connections available"
//#endif


/**
* \brief BT_ENABLE_SSP.
* \ingroup btconfig
*
* \details Enables support of Simple Secure Pairing.
*/
//#define BT_ENABLE_SSP

/**
 * \brief L2CAP_ENABLE_EXT_FEATURES.
 * \ingroup btconfig
 *
 * \details Enables support of Enhanced Retransmission ans Streaming modes.
 */


#endif // __BT_CONFIG_H_INCLUDED__
