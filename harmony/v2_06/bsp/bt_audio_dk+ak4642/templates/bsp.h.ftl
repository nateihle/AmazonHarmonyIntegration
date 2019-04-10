<#include "/bsp/templates/bsp_freemarker_functions.ftl">
/*******************************************************************************
  Board Support Package Header File.

  Company:      
    Microchip Technology Inc.

  File Name:    
    bsp.h 

  Summary:      
    bt_audio_dk+ak4642

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
#include "peripheral/devcon/plib_devcon.h"

// *****************************************************************************
// *****************************************************************************
// Section: Constants and Type Definitions.
// *****************************************************************************
// *****************************************************************************

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


// *****************************************************************************
/* BSP USB Switch

  Summary:
    Defines the USB Switch options.

  Description:
    This enumeration defines the USB Switch options. A switch option can be
    selected by using BSP_USBSwitchSelect() function.

  Remarks:
    None.
*/

typedef enum
{
    /* Connects the PIC32MX USB module to the USB host connector */
	BSP_USB_HOST    = /*DOM-IGNORE-BEGIN*/0x02/*DOM-IGNORE-END*/,

    /* Connects the PIC32MX USB module to the USB device connector */
	BSP_USB_DEVICE  = /*DOM-IGNORE-BEGIN*/0x03/*DOM-IGNORE-END*/

} BSP_USB_SWITCH;

<#if (Switch_Name_List?size > 0)>
<#include "/bsp/templates/bsp_switches.h.ftl">
</#if>
<#if (LED_Name_List?size > 0)>
<#include "/bsp/templates/bsp_leds.h.ftl">
</#if>
<#if (VBUS_PortPin_List?size > 0)>
<#include "/bsp/templates/bsp_vbus.h.ftl">
</#if>

/*** Functions for BSP_AK4642_PDN pin ***/
/*** !!!Workaround of Pin Control Macro Function Generation!!! ***/ 
#define BSP_AK4642_PDNToggle() PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15)
#define BSP_AK4642_PDNOn() PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15)
#define BSP_AK4642_PDNOff() PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15)
#define BSP_AK4642_PDNStateGet() PLIB_PORTS_PinGetLatched(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15)

/*** !!!Workaround of Function Generation!!! ***/ 
// *****************************************************************************
/* Function: 
    bool BSP_USBVBUSOverCurrentDetect(uint8_t port)

  Summary:
    Returns true if the over current is detected on the VBUS supply.
  
  Description:
    This function returns true if over current is detected on the VBUS supply.

  Remarks:
    None.
*/
bool BSP_USBVBUSSwitchOverCurrentDetect(uint8_t port);

/*** !!!Workaround of Function Generation!!! ***/ 
// *****************************************************************************
/* Function: 
    bool BSP_USBVBUSPowerEnable(uint8_t port, bool enable)

  Summary:
    This function controls the USB VBUS supply.
  
  Description:
    This function controls the USB VBUS supply.

  Remarks:
    None.
*/
void BSP_USBVBUSPowerEnable(uint8_t port, bool enable);

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
    BSP_SWITCH_STATE BSP_SwitchStateGet(BSP_SWITCH switch);

  Summary:
    Returns the present state (pressed or not pressed) of the specified switch.
  
  Description:
    This function returns the present state (pressed or not pressed) of the
    specified switch.

  Precondition:
    BSP_Initialize() should have been called.

  Parameters:
    switch  - The switch whose state needs to be obtained.
  
  Returns:
    The pressed released state of the switch.

  Example:
    <code>
    
    // Initialize the BSP
    BSP_Initialize();

    // Check the state of the switch.
    if(BSP_SWITCH_STATE_PRESSED == BSP_SwitchStateGet(BSP_SWITCH_1))
    {
        // This means that Switch 1 on the board is pressed.
    }

    </code>

  Remarks:
    None                                                                   
*/

BSP_SWITCH_STATE BSP_SwitchStateGet(BSP_SWITCH bspSwitch);


// *****************************************************************************
/* Function:
    void BSP_USBSwitchSelect(uint32_t value);

  Summary:
    Selects the USB Switch position on Bluetooth Audio board.

  Description:
    This function selects the USB Switch position on the Bluetooth Audio Board.
    This board features a 3 to 1 electronic USB switch that needs to be
    configured for either host or device operation.

  Preconditions:
    None

  Parameters:
    value - USB Switch position.

  Return Values:
    None.

  Example:
    <code>
    // Connect the PIC32MX USB module to the Bluetooth Audio Board USB Device
    // Connector.

    BSP_USBSwitchSelect(BSP_USB_DEVICE);
    </code>

  Remarks:
    None.
*/

void BSP_USBSwitchSelect(uint32_t value);


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


#endif // _BSP_H

/*******************************************************************************
 End of File
*/
