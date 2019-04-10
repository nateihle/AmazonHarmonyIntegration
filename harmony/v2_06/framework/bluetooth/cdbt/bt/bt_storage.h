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

#ifndef __BLUETOOTH_STG_H
#define __BLUETOOTH_STG_H

/**
 * \defgroup stg OEM - Non-volatile Storage Interface
 *
 * \details DotStack requires a non-volatile storage for storing link keys.
 * This module declares an interface for accessing such storage. The application
 * must provide implementations of all functions of this interface.
 */

#ifdef __cplusplus
extern "C" {
#endif

#define HCI_BDADDR_LEN		6
#define LKS_MAX_LINK_KEYS	5
#define LKS_SIGNATURE_ADDR	0
#define LKS_SIGNATURE		0x9D
#define LKS_FIRST_KEY_ADDR	2

#define HIDS_SIGNATURE_ADDR 	LKS_SIGNATURE_ADDR + 2 + (HCI_BDADDR_LEN + HCI_LINK_KEY_LEN) * LKS_MAX_LINK_KEYS
#define HIDS_SIGNATURE			0x8E
#define HIDS_LAST_DEVICE_ADDR	HIDS_SIGNATURE_ADDR + 1

#define CDS_SIGNATURE_ADDR 		HIDS_SIGNATURE_ADDR
#define CDS_SIGNATURE			HIDS_SIGNATURE
#define CDS_LAST_DEVICE_ADDR	HIDS_LAST_DEVICE_ADDR


/**
 * \brief Storage callback.
 *
 * \details This callback is called when a non-volatile storage operation completes.
 */
typedef void (*bt_storage_callback_fp) (void);


/**
 * \brief Get non-volatile storage capacity.
 * \ingroup stg
 *
 * \details Implementation of this function must return the capacity of its non-volatile
 * storage.
 */
bt_uint bt_oem_storage_get_capacity(void);


/**
 * \brief Begin a sequence of non-volatile storage operations.
 * \ingroup stg
 *
 * \details DotStack calls this function when it starts a sequence of non-volatile storage
 * operations. When the sequence is finished, DotStack will call bt_oem_storage_stop().
 */
void bt_oem_storage_start(void);


/**
 * \brief End a sequence of non-volatile storage operations.
 * \ingroup stg
 *
 * \details DotStack calls this function when it finishes executing a sequence of
 * non-volatile storage operations.
 */
void bt_oem_storage_stop(void);


/**
 * \brief Write to non-volatile storage.
 * \ingroup stg
 *
 * \details This function is called by the stack to write data to the non-volatile storage.
 * This function must be implemented by the application. When this function is called, the
 * application must start writing specified data to the non-volatile storage. When all data
 * has been written, the application must call the callback function passed in the \c callback
 * parameter. The application does not have to complete the write operation during the call
 * to this function. It may complete the operation later and then call the callback function.
 * In this case, the application does not have to store the data in an internal buffer. The
 * stack guarantees that the passed data will be present until the completion callback is
 * called by the application.
 *
 * \param addr The persitent storage address where to write data to.
 * \param data Pointer to data.
 * \param len Data length.
 * \param callback The completion callback function.
 *
 */
void bt_oem_storage_write(bt_int addr, const bt_byte* data, bt_int len, bt_storage_callback_fp callback);


/**
 * \brief Read from the non-volatile storage.
 * \ingroup stg
 *
 * \details This function is called by the stack to read from the non-volatile storage.
 * This function must be implemented by the application. When this function is called
 * the application must start a read operation. When the number of bytes specified by
 * the \c len parameter is read, the application must call the callback function
 * specified by the \c callback parameter. The application does not have to
 * read the whole number of bytes during the call to this function. It may complete
 * reading later and then call the completion callback. The stack guarantees that
 * the destination data buffer will be available until the application calls the
 * completion callback.
 *
 * \param addr The non-volatile storage address where to read data from.
 * \param buffer The receiving buffer.
 * \param len The number of bytes to read.
 * \param callback The completion callback function.
 * 
 */
void bt_oem_storage_read(bt_int addr, bt_byte* buffer, bt_int len, bt_storage_callback_fp callback);

#ifdef __cplusplus
}
#endif

#endif // __BLUETOOTH_STG_H
