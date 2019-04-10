/*******************************************************************************
  MRF24WN Wireless Driver Misc Functions

  File Name:  
    wdrv_mrf24wn_misc.c  

  Summary:
    MRF24WN Wireless Driver Misc Functions

  Description:
    MRF24WN Wireless Driver Misc Functions
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc. All rights reserved.

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

#include "system_definitions.h"

#include "system/reset/sys_reset.h"

#include "driver/wifi/mrf24wn/include/wdrv_mrf24wn_common.h"

void WDRV_Assert(int condition, const char *msg, const char *file, int line)
{
    if (!condition)
    {
        if (*msg)
            WDRV_DBG_ERROR_PRINT("\r\nWi-Fi driver ASSERTS: %s\r\n%s, line %u\r\n", msg, file, line);
        else
            WDRV_DBG_ERROR_PRINT("\r\nWi-Fi driver ASSERTS:\r\n%s, line %u\r\n", file, line);

        while (1) {
#if defined(SYS_CONSOLE_INSTANCES_NUMBER)
            SYS_CONSOLE_Tasks(sysObj.sysConsole0);
#endif

#if !defined(__DEBUG)
            uint32_t delayCount = 0;

            // Need to wait for WDRV_DBG_ERROR_PRINT() message to be displayed
            if (++delayCount == 35000)
                SYS_RESET_SoftwareReset();
#endif // !defined(__DEBUG)
        }
    }
    return;
}

//DOM-IGNORE-END
