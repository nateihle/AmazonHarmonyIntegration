/*******************************************************************************
  CTMU Driver Interface Declarations for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ctmu_static.h

  Summary:
    CTMU driver interface declarations for the static single instance driver.

  Description:
    The CTMU device driver provides a simple interface to manage the CTMU
    modules on Microchip microcontrollers. This file defines the interface
    Declarations for the CTMU driver.
    
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

SOFTWARE AND DCTMUUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTCTMUULAR PURPOSE.
IN NO EVENT SHALL MCTMURCTMUHIP OR ITS LCTMUENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRCTMUT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PRCTMUUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVCTMUES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END

#ifndef _DRV_CTMU_STATIC_H
#define _DRV_CTMU_STATIC_H

#include "peripheral/ctmu/plib_ctmu.h"

<#macro DRV_CTMU_STATIC_API DRV_INSTANCE>
// *****************************************************************************
// *****************************************************************************
// Section: Interface Headers for Instance ${DRV_INSTANCE} for the static driver
// *****************************************************************************
// *****************************************************************************
void DRV_CTMU${DRV_INSTANCE}_Initialize(void);
void DRV_CTMU${DRV_INSTANCE}_Deinitialize(void);
void DRV_CTMU${DRV_INSTANCE}_Enable(void);
void DRV_CTMU${DRV_INSTANCE}_Disable(void);
void DRV_CTMU${DRV_INSTANCE}_CurrentRangeSet(CTMU_CURRENT_RANGE range);
void DRV_CTMU${DRV_INSTANCE}_CurrentTrimSet(uint16_t trim);
void DRV_CTMU${DRV_INSTANCE}_CurrentSourceGround(bool onOff);
</#macro>
<#if CONFIG_DRV_CTMU_DRIVER_MODE == "STATIC">
<@DRV_CTMU_STATIC_API DRV_INSTANCE="0"/>
#define DRV_CTMU_Initialize(index, init) (DRV_CTMU0_Initialize(), NULL)
#define DRV_CTMU_Reinitialize(index, init) (DRV_CTMU0_Initialize(), NULL)
#define DRV_CTMU_Deinitialize( object ) DRV_CTMU0_Deinitialize()
#define DRV_CTMU_Open(drvIndex,intent) (DRV_CTMU0_Enable(), DRV_CTMU_INDEX_0)
#define DRV_CTMU_Close(handle) DRV_CTMU0_Disable()
#define DRV_CTMU_CurrentRangeSet(handle, range) DRV_CTMU0_CurrentRangeSet(range)
#define DRV_CTMU_CurrentTrimSet(handle, trim) DRV_CTMU0_CurrentTrimSet(trim)
#define DRV_CTMU_CurrentSourceGround(handle, onOff) DRV_CTMU0_CurrentSourceGround(onOff)
</#if>

#endif // #ifndef _DRV_CTMU_STATIC_H

/*******************************************************************************
 End of File
*/
