/*******************************************************************************
  MPLAB Harmony Exceptions Source File

  File Name:
    exceptions.c

  Summary:
    This file contains a sinlge function which overrides the deafult _weak_
    exception handler provided by the XC32 compiler.  It is not part of the
    Harmony framework and is not required in the project.  It is provided to
    simplify debugged when an unexpected exception occurs.

  Description:
    This file redefines the default _weak_  exception handler with a more debug
    friendly one. If an unexpected exception occurs the code will stop in a
    while(1) loop.  The debugger can be halted and two variables _excep_code and
    _except_addr can be examined to determine the cause and address where the
    exception occured.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>          /* Defines special funciton registers, CP0 regs  */
#include "system_config.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
/* static in case exception condition would stop auto variable from being created */
static unsigned int _excep_code;
static unsigned int _excep_addr;

// *****************************************************************************
// *****************************************************************************
// Section: Exception Handling
// *****************************************************************************
// *****************************************************************************

/* This function overrides the normal _weak_ _generic_exception_handler which
is defined in the XC32 User's Guide. */

void _general_exception_handler(void)
{
    /* Mask off Mask of the ExcCode Field from the Cause Register
    Refer to the MIPs Software User's manual */
    _excep_code=_CP0_GET_CAUSE() & 0x0000007C >> 2;
    _excep_addr=_CP0_GET_EPC();

    _CP0_SET_STATUS(_CP0_GET_STATUS()&0xFFFFFFE); /* Disable Interrupts */

    while (1)
    {
        /* Examine _excep_code to identify the type of exception */
        /* Examine _excep_addr to find the address that caused the exception */
    }
}
