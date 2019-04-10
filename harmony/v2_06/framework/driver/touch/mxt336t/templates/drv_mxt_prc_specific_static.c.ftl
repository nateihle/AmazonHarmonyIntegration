/*******************************************************************************
  MTCH6303 Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_mtch6303_static.c

  Summary:
    MTCH6303 driver impementation for the static single instance driver.

  Description:
    The MTCH6303 device driver provides a simple interface to manage the MTCH6303
    modules. This file contains implemenation for the MTCH6303 driver.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    
    Static single-open interfaces also eliminate the need for the open handle.
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

// *****************************************************************************
// *****************************************************************************
// Section: Include Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "system/ports/sys_ports.h"

<#if CONFIG_PIC32MX || CONFIG_PIC32MZ || CONFIG_PIC32WK || CONFIG_PIC32MK>
#include "peripheral/devcon/plib_devcon.h"
#include "peripheral/ports/plib_ports.h"
#include "peripheral/int/plib_int.h"
</#if>

 
<#if CONFIG_PIC32MX || CONFIG_PIC32MZ || CONFIG_PIC32WK || CONFIG_PIC32MK>
bool MXT_INTERRUPT_PIN_VALUE_GET(void)
{
    return(PLIB_PORTS_PinGet(PORTS_ID_0, ${CONFIG_DRV_TOUCH_MXT336T_UPDATED_CHANGE_PIN_CHANNEL}, ${CONFIG_DRV_TOUCH_MXT336T_UPDATED_CHANGE_PIN_NUMBER} ));
}
<#else>
bool MXT_INTERRUPT_PIN_VALUE_GET(void)
{
	#ifndef MXT336T_TOUCH_INTStateGet 
	#error " MXT336T_TOUCH_INTStateGet is not defined. Please use Pin Settings Tab in MHC to define the Touch Interrupt pin with the name 'MXT336T_TOUCH_INT' as GPIO_IN"   
	#else
	return(MXT336T_TOUCH_INTStateGet());
	#endif
}
</#if>

 