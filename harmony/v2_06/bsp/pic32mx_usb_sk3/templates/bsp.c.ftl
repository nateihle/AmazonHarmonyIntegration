<#include "/bsp/templates/bsp_freemarker_functions.ftl">
/*******************************************************************************
  Board Support Package Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    bsp.c

  Summary:
    Board Support Package implementation for PIC32MZ Embedded Connectivity (EC)
    Starter Kit.

  Description:
    This file contains routines that implement the board support package for
    PIC32MZ Embedded Connectivity (EC) Starter Kit.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "bsp.h"

<#if (Switch_Name_List?size > 0)>
<#include "/bsp/templates/bsp_switches.c.ftl">
</#if>
<#if (LED_Name_List?size > 0)>
<#include "/bsp/templates/bsp_leds.c.ftl">
</#if>
<#if (VBUS_PortPin_List?size > 0)>
<#include "/bsp/templates/bsp_vbus.c.ftl">
</#if>

// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function: 
    void BSP_Initialize(void)

  Summary:
    Performs the necessary actions to initialize a board
  
  Description:
    This function initializes the LED, Switch and other ports on the board.
    This function must be called by the user before using any APIs present in
    this BSP.  

  Remarks:
    Refer to bsp.h for usage information.
*/

void BSP_Initialize(void )
{
<#if (VBUS_PortPin_List?size > 0)>
    /* Setup the USB VBUS Switch Control Pin */
    BSP_USBVBUSSwitchStateSet(BSP_USB_VBUS_SWITCH_STATE_DISABLE);
</#if>

<#if (LED_Name_List?size > 0)>
    /* Switch off LEDs */
<#list LED_Name_List as led>
    BSP_LEDOff(${led});
</#list>
</#if>
}

// *****************************************************************************
/* Function:
    void BSP_GFX_Initialize(void)
  Summary:
    Sets up the PPS for the PIC32MX to work with SPI2 on MOST GFX PICTails
  Description:

  Parameters:

  Return Values:
    None.
  Conditions:
    None
*/

void BSP_GFX_Initialize(uint8_t spichannel)
{
    DDPCONbits.JTAGEN = 0; // Disable JTAG

    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 0;
    ANSELG = 0;            // Disable all ADC inputs

    if(spichannel == 2)
    {
        SYSKEY = 0x00000000;
        SYSKEY = 0xAA996655;
        SYSKEY = 0x556699AA;
        CFGCONbits.IOLOCK = 0;
        SDI2Rbits.SDI2R = 1;     //PMA4 = SDI2 = RG6
        RPG8Rbits.RPG8R = 6;     //PMA3 = SDO2 = RG8
        SYSKEY = 0x33333333;
    }
    else if(spichannel == 1)
    {
        SYSKEY = 0x00000000;
        SYSKEY = 0xAA996655;
        SYSKEY = 0x556699AA;
        CFGCONbits.IOLOCK = 0;
        SDI1Rbits.SDI1R = 10;     //RPC4 = SDI1 = RC4
        RPD0Rbits.RPD0R = 8;     //PMA3 = SDO2 = RG8
        SYSKEY = 0x33333333;
    }
}

/*******************************************************************************
 End of File
*/
