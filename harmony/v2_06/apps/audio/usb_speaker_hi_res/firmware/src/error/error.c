//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2014-2016 released Microchip Technology Inc.  All rights reserved.

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
#include "system_definitions.h"
#include "system/clk/sys_clk.h"
#include "system/int/sys_int.h"
#include "error/error.h"
#include "system_config.h"
//#include "app_config.h"

#if defined( ENABLE_SYS_LOG )
  #include "sys_log/sys_log.h"
  #include "sys_log/sys_log_messaging.h"
#endif

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
        while ( start < _CP0_GET_COUNT() || _CP0_GET_COUNT() < end);
    }
}


//******************************************************************************
// error_onFatalError()
//
// 1) Flash the LED's
// 2) Complete SYS_LOG Logging
//******************************************************************************
void error_onFatalError(void)
{
  #if !defined( ENABLE_SYS_LOG )
    SYS_INT_Disable();
  #endif

    APP_LED1_ON ();
    APP_LED2_OFF();
    APP_LED3_ON ();
    APP_LED4_OFF();
    APP_LED5_ON ();

    for (;;)
    {
        #if defined( ENABLE_SYS_LOG )
          if (SYS_LOG_MESSAGE_QueueStatusGet() == SYS_LOG_QUEUE_EMPTY)
          {
              SYS_INT_Disable();
          }
          else
          {
              SYS_LOG_Task();
          }
        #endif
        _DELAY(500);
        APP_LED2_TOGGLE();
        APP_LED3_TOGGLE();
        APP_LED4_TOGGLE();
        APP_LED5_TOGGLE();
    }
}


// For cdbt Bluetooth Stack
//void bt_oem_assert(const char* file, int line)
//{
//    assert_to_display((char *)"BT OEM Assert Failed",line,(char *)file);
//}
