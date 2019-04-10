/*******************************************************************************
    SPP Configuration

  Company:
    Microchip Technology Inc.

  File Name:
    btapp_spp_config.c

  Summary:
    Contains the functional implementation of SPP configuration.

  Description:
    This file contains the functional implementation of SPP configuration.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END
//#include "app.h"
#define BTAPP_MAX_SESSIONS              7
#define BT_INCLUDE_RFCOMM
#define BT_INCLUDE_SPP

#ifndef BT_PASSKEY_ENABLE
#define BT_ENABLE_SSP
#endif

// Number of HCI command buffers.
#define HCI_MAX_CMD_BUFFERS      3 * BTAPP_MAX_SESSIONS

// Number of HCI data buffers.
#define HCI_MAX_DATA_BUFFERS     2 * BTAPP_MAX_SESSIONS

// Maximum number of simultaneous HCI connections (ACL or SCO).
#define HCI_MAX_HCI_CONNECTIONS  BTAPP_MAX_SESSIONS + 1

#if !defined(BT_UART_HOST_BC_PROTOCOL) || BT_UART_HOST_BC_PROTOCOL == BT_UART_HOST_BC_PROTOCOL_H4
	// Size of the HCI receive buffer
	// Cannot be less than either
	// 40 or
	// HCI_TRANSPORT_HEADER_LEN(1 byte) + HCI_ACL_DATA_HEADER_LEN(4 bytes) or
	// HCI_TRANSPORT_HEADER_LEN(1 byte) + HCI_MAX_CMD_PARAM_LEN + 2
	#define HCI_RX_BUFFER_LEN          71
	// Size of the HCI send buffer
	// Cannot be less than either
	// 32 or
	// HCI_TRANSPORT_HEADER_LEN(1 byte) + HCI_ACL_DATA_HEADER_LEN(4 bytes) or
	// HCI_TRANSPORT_HEADER_LEN(1 byte) + HCI_CMD_HEADER_LEN(3 bytes) + HCI_MAX_CMD_PARAM_LEN
	// If not defined, it is set to HCI_RX_BUFFER_LEN
	#define HCI_TX_BUFFER_LEN          40
	// Maximum length in bytes of an HCI command parameter.
	#define HCI_MAX_CMD_PARAM_LEN       36
#else
	#define HCI_RX_BUFFER_LEN          256
	#define HCI_TX_BUFFER_LEN          252
	#define HCI_MAX_CMD_PARAM_LEN      248
#endif

// Maximum size of a complete L2CAP packet including packet header
// Cannot be less than HCI_RX_BUFFER_LEN - HCI_ACL_DATA_HEADER_LEN(4 bytes) - HCI_TRANSPORT_HEADER_LEN(1 byte)
// If not defined, it is set to HCI_RX_BUFFER_LEN - HCI_ACL_DATA_HEADER_LEN - HCI_TRANSPORT_HEADER_LEN
#define HCI_L2CAP_BUFFER_LEN        192

// Number of L2CAP command buffers
#define L2CAP_MAX_CMD_BUFFERS    (4 * BTAPP_MAX_SESSIONS)

#define L2CAP_MAX_PSMS           4

#define L2CAP_MAX_CHANNELS       (2 + BTAPP_MAX_SESSIONS * 3 )

// SDP buffers
#define SDP_MAX_SEARCH_RESULT_LEN	    6

#define SDP_MAX_ATTRIBUTE_RESULT_LEN	15

#define SDP_MAX_PDU_BUFFERS      3

// RFCOMM buffers
#define RFCOMM_MAX_SESSIONS      6

#define RFCOMM_MAX_DLCS          3

#define RFCOMM_MAX_SERVER_CHANNELS RFCOMM_MAX_DLCS - 1

// Maximum size of the data portion of a UIH frame. If CFC is used the actual length of the data portion will be 1 byte less.
// Must be less than or equal to HCI_L2CAP_BUFFER_LEN - RFCOMM_FRAME_HEADER_LEN - L2CAP_HEADER_LEN.
#define RFCOMM_INFO_LEN             (HCI_L2CAP_BUFFER_LEN - RFCOMM_FRAME_HEADER_LEN - L2CAP_HEADER_LEN)//31//(HCI_L2CAP_BUFFER_LEN - RFCOMM_FRAME_HEADER_LEN - L2CAP_HEADER_LEN)//31

#define RFCOMM_MAX_DATA_BUFFERS  (RFCOMM_MAX_DLCS - 1) * 2 * BTAPP_MAX_SESSIONS

#define RFCOMM_MAX_CMD_BUFFERS   (RFCOMM_MAX_DLCS - 1) * 2 * BTAPP_MAX_SESSIONS

#define RFCOMM_LOCAL_CREDIT         1
#define RFCOMM_ENABLE_MULTIDEVICE_CHANNELS
// SPP ports
#define SPP_MAX_PORTS               BTAPP_MAX_SESSIONS

#include "bluetooth/cdbt/bt/bt_oem_config.h"
/*******************************************************************************
 End of File
 */
