/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    wifi_init.h

  Summary:
    PIC32WK Wi-Fi Driver initialization Interface File

  Description:
    Contains function prototypes for interfacing to the PIC32WK Wi-Fi driver initialization.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc. All rights reserved.

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

#ifndef _WIFI_INIT_H
#define _WIFI_INIT_H

#include <stdint.h>
#include <stdbool.h>


// DOM-IGNORE-BEGIN
#ifdef __cplusplus // Provide C++ Compatibility
    extern "C" {
#endif
// DOM-IGNORE-END

//*******************************************************************************
/*
  Function:
        WIFI_initialization()

  Summary:
    Implements PIC32WK Wi-Fi driver initialization.
    <p><b>Implementation:</b> static</p>

  Description:
    This function implements PIC32WK Wi-Fi driver initialization.
	User has to ensure PIC32WK Initialization completed before starting any other PIC32WK Wi-Fi driver functionality.

  Precondition:
    None.

  Returns:
    true  - Initialization was successful
    false - Initialization was not successful

  Remarks:
    None.
*/
bool WIFI_initialization();

//*******************************************************************************
/*
  Function:
        void do_switchMode()

  Summary:
    Implements PIC32WK Wi-Fi driver switch mode.
    <p><b>Implementation:</b> static</p>

  Description:
    This function gives Wi-Fi driver switch mode functionality.
	
	This functionality needed only for STA-AP_Mode where Single application image contain 
	Wi-Fi STA(station) and OOBAP(Out Of Box access point) functionality.
	
	To set the desire boot mode manually ,command is provided to user:
	wlan bootmode <BOOT MODE>
	where BOOT MODE : 0 - PIC32WK boot as STA(station) mode
			          1 - PIC32WK boot as OOBAP(Out Of Box access point) mode
	After setting the mode,User need save the configuration in in-package flash using 
	"wlan save config" command and reboot the PIC32WK device.
					 
  Precondition:
    The Wi-Fi driver should be initialized using WIFI_initialization function call.

  Returns:
    None.

  Remarks:
    None.
*/
void do_switchMode();

// DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
// DOM-IGNORE-END

#endif /* _WIFI_INIT_H */
