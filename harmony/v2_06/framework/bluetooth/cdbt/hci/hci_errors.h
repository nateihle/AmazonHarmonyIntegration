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

#ifndef __HCI_ERRORS_H
#define __HCI_ERRORS_H

/** \addtogroup hci
* @{
*/
/** @name Errors
*
*/
/**@{
*/
#define HCI_ERR_SUCCESS                             0x00
#define HCI_SUCCESS                                 0x00
#define HCI_ERR_AUTHENTICATION_FAILURE              0x05
#define HCI_ERR_PIN_OR_KEY_MISSING                  0x06
#define HCI_ERR_MEMORY_CAPACITY_EXCEEDED            0x07
#define HCI_ERR_CONNECTION_TIMEOUT                  0x08
#define HCI_ERR_SCO_CONN_LIMIT_EXCEEDED             0x0a
#define HCI_ERR_ACL_CONN_ALREADY_EXISTS             0x0b
#define HCI_ERR_CONN_REJECT_LIMITED_RESOURCES       0x0d
#define HCI_ERR_INVALID_PARAMETERS                  0x12
#define HCI_ERR_PAIRING_NOT_ALLOWED                 0x18
#define HCI_ERR_UNSPECIFIED                         0x1F
#define HCI_ERR_SIMPLE_PAIRING_NOT_SUPPORTED        0x37
/**@}*/
/**@}*/

#endif // __HCI_ERRROS_H
