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

#ifndef __BT_HCITR_H_INCLUDED__
#define __BT_HCITR_H_INCLUDED__

#ifdef __cplusplus
extern "C" {
#endif

/**
	@defgroup hcitr OEM - HCI Communication Interface

	@details
	The DotStack library provides several HCI transport protocols (e.g., H4, 3-wire
	protocol). However, the code that actually moves octets of data between the CPU and
	HCI controller is application specific.
	
	This module declares an interface that allows DotStack to communicate with the HCI
	controller. The application has to implement this interface.
	
	The interface consist of the following functions that must be implemented by the
	application:
		\li bt_oem_send()
		\li bt_oem_recv()
 */
/*@{*/ /* begin doxygen group */


/** Send callback.

	This callback function is called when a send operation initiated by bt_oem_send() has
	completed.
*/
typedef void (*bt_oem_send_callback_fp)(void);


/** Receive callback.

	This callback function is called when a receive operation initiated by bt_oem_recv()
	has completed.
	
	@param len Number of received bytes.
		The value of this parameter should always be the same as the number of bytes
		requested in a call to bt_oem_recv().
*/
typedef void (*bt_oem_recv_callback_fp)(bt_uint len);


/** Send data.

	This function is called by the HCI layer when it needs to send data to the HCI
	controller. Implementation of this function must send the specified number of bytes
	to the HCI controller and call the provided callback function. 

	@param buffer Pointer to the data to be sent .
	
	@param len Number of bytes to send.
	
	@param callback A callback function that must be called when all data have been sent.
*/
void bt_oem_send(const bt_byte* buffer, bt_uint len, bt_oem_send_callback_fp callback);


/** Receive data.

	This function is called by the HCI layer when it needs more data from the HCI
	controller. Implementation of this function must receive the specified number of bytes
	from the HCI controller and call the provided callback function. 

	@param buffer Pointer to a buffer for the received data.
		The buffer must be long enough to accommodate the number of bytes specified by the
		\par len parameter.
	
	@param len Number of bytes to receive.
	
	@param callback A callback function that must be called when the requested number
		of bytes have been received.
*/
void bt_oem_recv(bt_byte* buffer, bt_uint len, bt_oem_recv_callback_fp callback);

/*@}*/ /* end doxygen group */

#ifdef __cplusplus
}
#endif

#endif // __BT_HCITR_H_INCLUDED__
