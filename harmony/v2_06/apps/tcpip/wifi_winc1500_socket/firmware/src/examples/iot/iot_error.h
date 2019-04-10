/*******************************************************************************
  File Name:
    iot_error.h

  Summary:
    Error codes
 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#ifndef _IOT_ERROR_H
#define _IOT_ERROR_H

#define	EINVAL	1
#define	ENOENT	2
#define	NOPATH	3
#define	EMFILE	4
#define	EACCESS	5
#define	EBADF	6
#define	EMCBD	7
#define	ENOMEM	8
#define	EIMBA	9
#define	EINVENV	10
#define	ENOEXEC	11
#define	EPERM	12
#define	EDATA	13
#define	EDRIVE	15
#define	ECURDIR	16
#define	EXDEV	17
#define	ENFILE	18

#define ENOSPC  19
#define EIO     20
#define EDESTADDRREQ    21
#define ECONNRESET      22
#define EAGAIN  23
#define EBUSY   24
#define EADDRINUSE      25
#define EALREADY        26
#define ENOTCONN        27
#define ECONNREFUSED    28
#define EOVERFLOW       29
#define EBADMSG         30
#define EHOSTUNREACH    31
#define	ETIME   32
#define ENAMETOOLONG    33
#define ENOTSUP         34

#endif /* _IOT_ERROR_H */
