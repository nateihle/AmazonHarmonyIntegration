/*******************************************************************************
  MXT336T Driver Interface Declarations for Processor Compatibility

  Company:
    Microchip Technology Inc.

  File Name:
    drv_mxt_prc_specific_static.h

  Summary:
    MXT336T driver interface declarations for Processor Compatibility.

  Description:
    The MXT336T device driver provides a simple interface to manage the MXT336T
    modules. This file defines the interface Declarations for the MXT336T driver.

  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.

    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _DRV_TOUCH_MXT336T_INTERFACE_PRC_STATIC_H
#define _DRV_TOUCH_MXT336T_INTERFACE_PRC_STATIC_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "driver/touch/drv_touch.h"
#include "driver/driver_common.h"
#include "system/system.h"

// *****************************************************************************

/*

  Summary:
    A function to check the status of the ~CHG.

  Description:
    A function to check the status of the ~CHG.


  Remarks:
    None.

*/

bool MXT_INTERRUPT_PIN_VALUE_GET(void);

#endif //_DRV_TOUCH_MXT336T_INTERFACE_PRC_STATIC_H