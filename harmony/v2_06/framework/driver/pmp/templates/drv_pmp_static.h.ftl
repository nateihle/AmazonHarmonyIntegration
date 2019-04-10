/*******************************************************************************
  PMP Driver Interface Declarations for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_pmp_static.h

  Summary:
    PMP driver interface declarations for the static single instance driver.

  Description:
    The PMP device driver provides a simple interface to manage the PMP
    modules on Microchip microcontrollers. This file defines the interface
    Declarations for the PMP driver.
    
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

#ifndef _DRV_PMP_STATIC_H
#define _DRV_PMP_STATIC_H

#include "peripheral/pmp/plib_pmp.h"

<#macro DRV_PMP_STATIC_API DRV_INSTANCE DATA_SIZE>
// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance ${DRV_INSTANCE} for the static driver
// *****************************************************************************
// *****************************************************************************

void DRV_PMP${DRV_INSTANCE}_Initialize(void);

void DRV_PMP${DRV_INSTANCE}_ModeConfig(void);

void DRV_PMP${DRV_INSTANCE}_TimingSet(PMP_DATA_WAIT_STATES dataWait,
                   PMP_STROBE_WAIT_STATES strobeWait,
                   PMP_DATA_HOLD_STATES dataHold);

${DATA_SIZE} DRV_PMP${DRV_INSTANCE}_Read(void);

void DRV_PMP${DRV_INSTANCE}_Write(${DATA_SIZE} data);

</#macro>


<#if CONFIG_DRV_PMP_DATA_SIZE == "PMP_DATA_SIZE_8_BITS">
<@DRV_PMP_STATIC_API DRV_INSTANCE="0" DATA_SIZE="uint8_t"/>
</#if>
<#if CONFIG_DRV_PMP_DATA_SIZE == "PMP_DATA_SIZE_16_BITS">
<@DRV_PMP_STATIC_API DRV_INSTANCE="0" DATA_SIZE="uint16_t"/>
</#if>

#endif // #ifndef _DRV_PMP_STATIC_H

/*******************************************************************************
 End of File
*/
