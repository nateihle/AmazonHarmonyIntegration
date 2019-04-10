/*******************************************************************************
  In-package flash Driver Pin Control Functions

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ipf_pins.c
	
  Summary:
    Pin control function for IPF

  Description:
    The IPF device driver provides a simple interface to manage the in-package 
	flash on PIC32WK devices. This file contains implemenation
    for the IPF pin control function.
    
  Remarks:
   
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "driver/spi_flash/pic32wk_ipf/src/drv_ipf_local.h"


<#if CONFIG_DRV_IPF_DRIVER_MODE == "DYNAMIC">

void _DRV_IPF_WPAssert()
{
	<#if CONFIG_DRV_IPF_WRITE_PROTECT_PIN == true>
	SYS_PORTS_PinClear(PORTS_ID_0, DRV_IPF_WRITE_PROTECT_PIN_PORT_CHANNEL, DRV_IPF_WRITE_PROTECT_PIN_BIT_POS);
	<#else>
	return;
	</#if>
}

void _DRV_IPF_WPDeAssert()
{
	<#if CONFIG_DRV_IPF_WRITE_PROTECT_PIN == true>
	SYS_PORTS_PinSet(PORTS_ID_0, DRV_IPF_WRITE_PROTECT_PIN_PORT_CHANNEL, DRV_IPF_WRITE_PROTECT_PIN_BIT_POS);
	<#else>
	return;
	</#if>
}

void _DRV_IPF_HoldAssert()
{
	<#if CONFIG_DRV_IPF_HOLD_PIN == true>
	SYS_PORTS_PinClear(PORTS_ID_0, DRV_IPF_HOLD_PIN_PORT_CHANNEL, DRV_IPF_HOLD_PIN_PORT_BIT_POS);
	<#else>
	return;
	</#if>
}	

void _DRV_IPF_HoldDeAssert()
{
	<#if CONFIG_DRV_IPF_HOLD_PIN == true>
	SYS_PORTS_PinSet(PORTS_ID_0, DRV_IPF_HOLD_PIN_PORT_CHANNEL, DRV_IPF_HOLD_PIN_PORT_BIT_POS);
	<#else>
	return;
	</#if>
}	

</#if>

<#--
/*******************************************************************************
 End of File
*/
-->
