/*******************************************************************************
  Application System Log Messaging Services

  Company:
    Microchip Technology Inc.

  File Name:
    bt_oem_log_write.c

  Summary:
    Provides messaging functions for CDBT stack event logging.

  Description:
    Provides messaging functions for CDBT stack event logging.

*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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
//DOM-IGNORE-END

#if defined( ENABLE_SYS_LOG )

#include "sys_log.h"
#include "sys_log_local.h"
#include "sys_log_define.h"
#include "sys_log_messaging.h"

// To support CDBT Bluetooth Stack.
// Function prototype for this is found in .\Bluetooth\cdbt\bt\bt_log.h
void bt_oem_log_write(const char* msg)
{
    SYS_LOG_LibEntryWrite(msg);
}

#else

void bt_oem_log_write(const char* msg)
{
    // Do Nothing: System Logging disabled.
}

#endif//defined( ENABLE_SYS_LOG )
