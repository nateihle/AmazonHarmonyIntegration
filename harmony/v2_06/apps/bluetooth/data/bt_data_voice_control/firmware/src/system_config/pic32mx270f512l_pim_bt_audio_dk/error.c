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

#include <stdint.h>
#include <xc.h>
#include "system/clk/sys_clk.h"
#include "system/int/sys_int.h"
#include "error.h"
#include "system_config.h"
#include "app.h"
#include "assert_to_display.h"

static void _DELAY(int ms)
{
    uint32_t start = _CP0_GET_COUNT();
    uint32_t end = start + SYS_CLK_SystemFrequencyGet() / 1000 / 2 * ms;
    if (end > start)
    {
        while (_CP0_GET_COUNT() < end);
    }
    else
    {
        while (_CP0_GET_COUNT() > start || _CP0_GET_COUNT() < end);
    }
}


void error_onFatalError(void)
{
    SYS_INT_Disable();

    APP_LED1_ON ();
    APP_LED2_OFF();
    APP_LED3_ON ();
    APP_LED4_OFF();
    APP_LED5_ON ();

    for (;;)
    {
        _DELAY(500);
        APP_LED2_TOGGLE();
        APP_LED3_TOGGLE();
        APP_LED4_TOGGLE();
        APP_LED5_TOGGLE();
    }
}


// For use by Bluetooth Stack
void bt_oem_assert(const char* file, int line)
{
    assert_to_display((char *)"BT OEM Assert Failed",line,(char *)file);
}
