/*******************************************************************************
  TFTP Server Module API Header File

  Company:
    Microchip Technology Inc.
    
  File Name:
    tftps_config.h

  Summary:
    The TFTP server module implements the server for trivial file transfer protocol. 

  Description:
    The TFTP module is the simple protocol which is used to transfer files.It 
    has been implemented on top of Internet User Datagram Protocol.
    If the server grants the request, the connection is opened and the file is 
    sent in fixed length blocks of 512 bytes. 
    Each data packet contains one block of data, and must be acknowledged by an
    acknowledgment packet before the next packet can be sent.
********************************************************************************/
    
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright © 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _TFTPS_CONFIG_H_
#define _TFTPS_CONFIG_H_


// The number of seconds to wait before declaring a TIMEOUT error on GET or PUT.
#define TCPIP_TFTP_SERVER_TMO         (3ul)

// The number of Clients requests will be processed for the TFTP server stack.
#define TCPIP_TFTPS_CLIENT_NUMBER                 (2ul)

// The number of attempts to retransmit  the previous packet before declaring a TIMEOUT error.
#define TCPIP_TFTPS_RETRANSMIT_COUNT                 (3u)

// The TFTP server task rate in milliseconds.
// The default value is 100 milliseconds.
// The lower the rate (higher the frequency) the higher the module priority
// and higher module performance can be obtained
// The value cannot be lower than the TCPIP_STACK_TICK_RATE.
#define TCPIP_TFTPS_TASK_TICK_RATE              (60)

// The maximum size for a file to be accepted.TFTP Opcode defined by RFC 2347
#define TCPIP_TFTPS_DEFAULT_FILESIZE                (64000)

// The maximum value for the file name size.
// Even though the RFC allows blocks of up to 65464 bytes, 
// In practice the limit is set to 1468 bytes: the size of an 
// Ethernet MTU minus the headers of TFTP (4 bytes), UDP (8 bytes) and IP (20 bytes)
#define TCPIP_TFTPS_DEFAULT_BLOCK_SIZE                (32)

// This time is used during the TFTP retransmission 
// and TFTP Client wait for these many seconds before retransmitting .
#define TCPIP_TFTPS_DEFAULT_TIMEOUT           false 

#endif  // _TFTPS_CONFIG_H_
