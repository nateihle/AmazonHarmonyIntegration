/*******************************************************************************
  Initialization System Service Library Definitions
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    sys_init.h

  Summary:
    Initialization System Service Library definitions and declarations.

  Description:
    This file contains Initialization System Service Library definitions and 
    declarations.
    
  Remarks:
    This file is included by "system.h" and need not be included directly.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _SYS_INIT_H    // Guards against multiple inclusion
#define _SYS_INIT_H


// *****************************************************************************
// *****************************************************************************
// Section: Included files
// *****************************************************************************
// *****************************************************************************

#include <stdbool.h>
#include "system/common/sys_common.h"
#include "system/common/sys_module.h"


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************

//*******************************************************************************
/*
  Function:
    void SYS_Initialize ( void *data )

  Summary:
    Initializes the board, services, drivers, and other modules

  Description:
    This function initializes the board, services, drivers, and other modules
    as configured at build time.  It should be called almost immediately after 
    entering the "main" function.

  Precondition:
    The C-language run-time environment and stack must have been initialized.
  
  Parameters:
    data        - Pointer to any system initialization data required.  Normally
                  passed as NULL for static system implementations.

  Returns:
    None.

  Example:
    <code>
    int main ( void )
    {
        SYS_Initialize(NULL);
        
        while (true)
        {
            SYS_Tasks();
        }
    }
    </code>

  Remarks:
    Basic System Initialization Sequence:
    
    1.  Initialize core processor services.
    2.  Initialize board support package.
    3.  Initialize RTOS (if utilized).
    4.  Initialize drivers.
    5.  Initialize system services.
    6.  Initialize middleware. 
    7.  Initialize application(s).

    This function may be overridden and implemented as desired as long as it
    appropriately initializes all necessary board, services, and modules.

    Most MPLAB Harmony libraries are designed so that the order in which they 
    are initialized is not important.  However, core processor services and
    board support packgage initialization should be completed before any other
    initialization takes place and RTOS initialization (if utilized) shoudl take 
    palce before drivers, system services and middleware are initialized. 
    Applications should be initialized last.
*/

void SYS_Initialize ( void *data );


#endif // _SYS_INIT_H

/*******************************************************************************
 End of File
*/