<#include "/bsp/templates/bsp_freemarker_functions.ftl">
/*******************************************************************************
  Board Support Package Header File.

  Company:      
    Microchip Technology Inc.

  File Name:    
    bsp.h

  Summary:      
    Board Support Package Header File for PIC32MZ Embedded Connectivity (EF)
    Starter Kit.

  Description:
    This file contains constants, macros, type definitions and function
    declarations required by the PIC32MZ Embedded Connectivity (EC) Starter Kit.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _BSP_H
#define _BSP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "peripheral/ports/plib_ports.h"

// *****************************************************************************
// *****************************************************************************
// Section: Constants and Type Definitions.
// *****************************************************************************
// *****************************************************************************

/*** Functions for BSP_AK4953_PDN pin ***/
/*** !!!Workaround of Pin Control Macro Function Generation!!! ***/ 
#define BSP_AK4953_PDNToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_3)
#define BSP_AK4953_PDNOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_3)
#define BSP_AK4953_PDNOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_3)
#define BSP_AK4953_PDNStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_3)

// *****************************************************************************
/* BT Pin State

  Summary:
    Enumerates the BT pin states.

  Description:
    This enumeration defines the supported BT pin states.

  Remarks:
    None.
*/
typedef enum
{
    /* BT pin is low */
    BSP_BT_STATE_LOW = /*DOM-IGNORE-BEGIN*/0/*DOM-IGNORE-END*/,

    /* BT pin is high */
    BSP_BT_STATE_HIGH = /*DOM-IGNORE-BEGIN*/1/*DOM-IGNORE-END*/

} BSP_BT_STATE;

/*** Functions for BSP_AK4953_PDN pin ***/
#define BSP_AK4953_PDNToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_3)
#define BSP_AK4953_PDNOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_3)
#define BSP_AK4953_PDNOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_3)
#define BSP_AK4953_PDNStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_3)

<#if (Switch_Name_List?size > 0)>
<#include "/bsp/templates/bsp_switches.h.ftl">
</#if>
<#if (LED_Name_List?size > 0)>
<#include "/bsp/templates/bsp_leds.h.ftl">
</#if>
<#if (VBUS_PortPin_List?size > 0)>
<#include "/bsp/templates/bsp_vbus.h.ftl">
</#if>

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
    This function initializes the LED and Switch ports on the board.  This
    function must be called by the user before using any APIs present on this
    BSP.  

  Precondition:
    None.

  Parameters:
    None
  
  Returns:
    None.

  Example:
    <code>
    //Initialize the BSP
    BSP_Initialize();
    </code>

  Remarks:
    None                                                                   
*/

void BSP_Initialize(void);

// *****************************************************************************
/* Function:
    void BSP_BluetoothPinStateSet(BSP_BT_STATE state);

  Summary:
    Sets to low/high the pin to the Bluetooth module.

  Description:
    This function sets the pin to the Bluetooth module to low/high.
    A reset to the bluetooth module needs to be given by making
    transition from low to high. This function helps in setting
    the interface line from the microcontroller to the bluetooth
    module low/high.

  Preconditions:
    None

  Parameters:
    None

  Return Values:
    None.

  Example:
    <code>

    BSP_BluetoothPinStateSet(BSP_BT_STATE_HIGH);
    </code>

  Remarks:
    None.
*/
void BSP_BluetoothPinStateSet(BSP_BT_STATE state);

#endif //_BSP_H

/*******************************************************************************
 End of File
*/
