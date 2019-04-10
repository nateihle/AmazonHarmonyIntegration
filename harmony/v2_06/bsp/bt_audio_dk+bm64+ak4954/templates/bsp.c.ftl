<#include "/bsp/templates/bsp_freemarker_functions.ftl">
/*******************************************************************************
  Board Support Package Implementation

  Company:
    Microchip Technology Inc.

  File Name:
    bsp.c

  Summary:
    Board Support Package implementation for PIC32 Bluetooth Audio Development
    Kit BTADK, AK4954 Codec daughter board, and BM64 Bluetooth DB

  Description:
    This file contains routines that implement the board support package for
    the BTADK with AK4954 plus BM64 DB.
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2018 released Microchip Technology Inc.  All rights reserved.

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

/****  WORKAROUND ****/
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

bool BSP_USBVBUSSwitchOverCurrentDetect(uint8_t port)
{
    return(false);
}

/****  WORKAROUND ****/
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
void BSP_USBVBUSPowerEnable(uint8_t port, bool enable)
{
    if(enable)
    {
        PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_5);
    }
    else
    {
        PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_5);
    }
}


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
    /* Disable JTAG so that all LED ports are available */
    PLIB_DEVCON_JTAGPortDisable(DEVCON_ID_0);

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

    /* BT Pin */
    PLIB_PORTS_PinClear( PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15 );
}

/*******************************************************************************
  Function: void BSP_USBSwitchSelect(void)

  Summary:
   Sets the USB Switch to define USB Mode to be Device or Host.

  Description:
    Sets the USB Switch to define USB Mode to be Device or Host.

  Remarks:
    None.

*/
void BSP_USBSwitchSelect(uint32_t value)
{
    PLIB_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_G, value);
}

// *****************************************************************************
/* Function:
    void BSP_BluetoothPinStateSet(BSP_BT_STATE state);
    void BSP_BluetoothPinStateSet(BSP_BT_STATE state);
    void BSP_BluetoothPinSetHigh(void);
    void BSP_BluetoothPinSetLow(void);
    void BSP_BluetoothPinToggle(void);
    uint32_t BSP_BluetoothPinGetValue(void);

  Summary:
    Sets to low/high the pin to the Bluetooth module.

  Description:
    These functions set the reset pin to the Bluetooth module to low/high, or
    fetches the current state.  A reset to the bluetooth module needs to be given
    by making     transition from low to high. This function helps in setting
    the interface line from the microcontroller to the bluetooth module low/high.

  Remarks:
    None.
*/
void BSP_BluetoothPinStateSet(BSP_BT_STATE state)
{
    PLIB_PORTS_PinWrite (PORTS_ID_0 , PORT_CHANNEL_G , PORTS_BIT_POS_15, state );
}

void BSP_BluetoothPinSetHigh(void)
{   
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15);   
}

void BSP_BluetoothPinSetLow(void)
{    
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15);    
}

void BSP_BluetoothPinToggle(void)
{
    PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15);    
}

uint32_t BSP_BluetoothPinGetValue(void)
{
    return PLIB_PORTS_PinGet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_15);
}


/*******************************************************************************
 End of File
*/
