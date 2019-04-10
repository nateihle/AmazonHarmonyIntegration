/*******************************************************************************
  File Name:
    WiFi_commands.c

  Summary:
    commands for the communicating with WLAN MAC library.

  Description:
    
 *******************************************************************************/


// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

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
#include <ctype.h>
#include "system_config.h"
#include "system_definitions.h"
#include "WiFi_commands.h"
#include "WiFi_MW.h"

#if defined(TCPIP_STACK_COMMAND_ENABLE)


static int WLANCMDProcessing(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);
static int WLANCMDHelp(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv);


static const SYS_CMD_DESCRIPTOR    WiFiCmdTbl[]=
{
    {"wlan",     WLANCMDProcessing,              ": WLAN MAC commands processing"},
    {"wlanhelp",  WLANCMDHelp,			  		 ": WLAN MAC commands help "},
};


bool WiFi_Commands_Init()
{
    if (!SYS_CMD_ADDGRP(WiFiCmdTbl, sizeof(WiFiCmdTbl)/sizeof(*WiFiCmdTbl), "wlan", ": WLAN commands"))
    {
        SYS_ERROR(SYS_ERROR_ERROR, "Failed to create WLAN Commands\r\n", 0);
		return false;
    }

    return true;
}

int WLANCMDProcessing(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv) {
	
	wifi_command_process(argc,argv);

	return 0;
}

static int WLANCMDHelp(SYS_CMD_DEVICE_NODE* pCmdIO, int argc, char** argv){
	
	SYS_CONSOLE_PRINT("WLAN commands:\r\n");
    SYS_CONSOLE_PRINT("\n1) wlan open                   -- To open wlan command list\r\n");
    SYS_CONSOLE_PRINT("\n2) wlan close                  -- To close wlan command list\r\n");
    SYS_CONSOLE_PRINT("\n3) wlan set ssid <SSID NAME>   -- setting ssid of HOMEAP\r\n");
    SYS_CONSOLE_PRINT("\n4) wlan set psk <Password>     -- To set PSK\r\n");
    SYS_CONSOLE_PRINT("\n5) wlan set imode <MODE>       -- To set Security Mode, 0x29:WPA-AES,0x31:WPA2-AES, 0x79:WPA/WPA2\r\n");
    SYS_CONSOLE_PRINT("\n6) wlan save config            -- saving wlan configuration in in-package flash\r\n");
    SYS_CONSOLE_PRINT("\n7) wlan macinfo                -- Display the Mac config wrt Flash and DUT connection details\r\n");
	return 0;
}

#endif
