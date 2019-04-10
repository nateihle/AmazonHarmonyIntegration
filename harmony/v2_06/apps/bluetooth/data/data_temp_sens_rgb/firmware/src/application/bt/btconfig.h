/*******************************************************************************
  BT Config Interface

  Company:
    Microchip Technology Inc.

  File Name:
    btconfig.h

  Summary:
    Contains the BT Config Interface specific defintions and function prototypes.

  Description:
    This file contains the BT Config Interface specific defintions and function
    prototypes.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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

#ifndef __CONFIG_H_INCLUDED__
#define __CONFIG_H_INCLUDED__

//// DOM-IGNORE-BEGIN
#ifdef __cplusplus
extern "C" {
#endif

// DOM-IGNORE-END
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
/* Bluetooth Device ID settings
 *
 * These macros define the bluetooth device ID.
 *
 * BT_DEVICE_DESIGN_ID         - sets the most significant 8 bytes
 * BT_DEVICE_ID_2LSB           - sets the least significant 4 bytes
 *
 * BT_DEVICE_ID_2LSB_RANDOMIZE - controls randomization of the 2LSB portion.
 *                               Note: 2LSB is randomized at power-on only.
 *
 *                               0 = disabled
 *                               1 = enabled
 *
 * device ID scheme:
 *        Design   2LSB
 *     -- -- -- -- ++ ++
 * ID :XX:XX:XX:XX:XX:XX
*/
//                                      Device
//                                      DesignID ID
//                                      --------++++
#define BT_DEVICE_DESIGN_ID           	0x12345678
#define BT_DEVICE_ID_2LSB             	0xFFFF
#define BT_DEVICE_ID_2LSB_RANDOMIZE   	1 // Set to one to enable, zero to disable


// BT controller type
// (select one from the list of supported controllers
// defined in btcontroller.h)
#define BT_CONTROLLER BT_CONTROLLER_FLC_BTM805

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Function Prototypes
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************

#ifdef __cplusplus
}
#endif

#endif // __CONFIG_H_INCLUDED__
/*******************************************************************************
 End of File
*/
