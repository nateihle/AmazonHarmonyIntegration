/*******************************************************************************
 BM64 Driver Configuration Template

  Company:
    Microchip Technology Inc.

  File Name:
    drv_bm64_config_template.h

  Summary:
    BM64 Bluetooth Driver Configuration Template.

  Description:
    These file provides the list of all the configurations that can be used with
    the driver. This file should not be included in the driver.

*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

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

#ifndef _DRV_BM64_CONFIG_TEMPLATE_H
#define _DRV_BM64_CONFIG_TEMPLATE_H

//DOM-IGNORE-BEGIN
#error "This is a configuration template file.  Do not include it directly."
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Core Functionality Configuration Options
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Use Deprecated MMI Commands?

  Summary
    Identifies whether the driver should use deprecated MMI commands. 

  Description
    There are currently two versions of the BM64 Audio UART Command Set, which is used
    by the PIC32 to send commands to the BM64 module and receive responses (events) back
    from the BM64.  The original is version 1.00 and the updated one is version 2.0x.
    Version 2.0x deprecates some MMI commands, and adds some new commands to replace them.
    
    If the DRV_BM64_PlayPreviousSong and DRV_BM64_PlayNextSong functions are not working
    but other AVRCP functions are working properly, try unchcing this option. 

    true (checked, default) - use deprecated MMI commands.
    false (unchecked) - do not deprecated MMI commands.

  Remarks:
    None
*/

#define INCLUDE_DEPRECATED_MMI_COMMANDS

// *****************************************************************************
/* Include HFP,A2DP,AVRCP protocols?

  Summary
    Identifies whether the driver should include HFP,A2DP,AVRCP functionality. 

  Description
    Identifies whether the driver should include the interface to support HFP, A2DP and AVRCP
    protocols, which by default also brings in the I2S driver and the default codec based on
    the BSP selected.

    If you are building a BLE-only application, uncheck this option.

    true (checked, default) - include HFP,A2DP,AVRCP functionality.
    false (unchecked) - do not include HFP,A2DP,AVRCP functionality.

  Remarks:
    None
*/

#define INCLUDE_BM64_I2S


// *****************************************************************************
/* Include BLE features?

  Summary
    Identifies whether the driver should include BLE

  Description
    Identifies whether the driver should include BLE (Bluetooth Low Energy)
    functions.

    This option currently does not have any effect on the code size.

    true (checked, default) - include BLE functionality.
    false (unchecked) - do not include BLE functionality.

  Remarks:
    None
*/

#define INCLUDE_BM64_BLE

#endif // #ifndef _DRV_BM64_CONFIG_TEMPLATE_H

/*******************************************************************************
 End of File
*/
