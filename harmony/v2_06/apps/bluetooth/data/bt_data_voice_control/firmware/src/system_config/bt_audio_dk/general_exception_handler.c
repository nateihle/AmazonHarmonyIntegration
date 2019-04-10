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
// *****************************************************************************
// *****************************************************************************
// Section: Exception handling
// *****************************************************************************
// *****************************************************************************
#include <stdio.h>
#include <stdint.h>
#include <xc.h>
#include "system_config.h"
#include "system/debug/sys_debug.h"
#include "system/clk/sys_clk.h"
#include "app.h"

typedef struct _XCPT_FRAME
{
    uint32_t at;
    uint32_t v0;
    uint32_t v1;
    uint32_t a0;
    uint32_t a1;
    uint32_t a2;
    uint32_t a3;
    uint32_t t0;
    uint32_t t1;
    uint32_t t2;
    uint32_t t3;
    uint32_t t4;
    uint32_t t5;
    uint32_t t6;
    uint32_t t7;
    uint32_t t8;
    uint32_t t9;
    uint32_t ra;
    uint32_t lo;
    uint32_t hi;
    uint32_t cause;
    uint32_t status;
    uint32_t epc;

} XCPT_FRAME;

static enum {
    EXCEP_IRQ      =  0, // interrupt
    EXCEP_AdEL     =  4, // address error exception (load or ifetch)
    EXCEP_AdES     =  5, // address error exception (store)
    EXCEP_IBE      =  6, // bus error (ifetch)
    EXCEP_DBE      =  7, // bus error (load/store)
    EXCEP_Sys      =  8, // syscall
    EXCEP_Bp       =  9, // breakpoint
    EXCEP_RI       = 10, // reserved instruction
    EXCEP_CpU      = 11, // coprocessor unusable
    EXCEP_Overflow = 12, // arithmetic overflow
    EXCEP_Trap     = 13, // trap (possible divide by zero)
    EXCEP_IS1      = 16, // implementation specfic 1
    EXCEP_CEU      = 17, // CorExtend Unuseable
    EXCEP_C2E      = 18, // coprocessor 2
} _excep_code;

static unsigned int _excep_code;
static unsigned int _excep_addr;

#define STRING_MAX_SIZE 133
extern char ioString[STRING_MAX_SIZE];

static void _DisplayExceptionCode(bool bDisplayCode, unsigned int excep_code )
{
    APP_LED1_ON();
    
    if ( bDisplayCode )
    {
        if ( excep_code & 1<<4 )
        {
            APP_LED1_ON();
        }
        else
        {
            APP_LED1_OFF();
        }
                
        if ( excep_code & 1<<3 )
        {
            APP_LED2_ON();
        }
        else
        {
            APP_LED2_OFF();
        }

        if ( excep_code & 1<<2 )
        {
            APP_LED3_ON();
        }
        else
        {
            APP_LED3_OFF();
        }

        if ( excep_code & 1<<1 )
        {
            APP_LED4_ON();
        }
        else
        {
            APP_LED4_OFF();
        }

        if ( excep_code & 1<<0 )
        {
            APP_LED5_ON();
        }
        else
        {
            APP_LED5_OFF();
        }
    }
    else
    {
        APP_LED1_OFF();
        APP_LED2_OFF();
        APP_LED3_OFF();
        APP_LED4_OFF();
        APP_LED5_OFF();
    }
}

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

void __attribute__((nomips16)) _general_exception_handler (XCPT_FRAME* const pXFrame)
{
    bool bDisplayCode = true;
    
    _excep_addr = pXFrame->epc;
    _excep_code = pXFrame->cause;
    _excep_code = (_excep_code & 0x0000007C) >> 2;

    _DisplayExceptionCode(bDisplayCode,_excep_code);
    
    SYS_DEBUG_BreakPoint();  // Stop here if in debugger.
    while (1) {
        _DELAY(250);
        bDisplayCode = !bDisplayCode;
        _DisplayExceptionCode(bDisplayCode,_excep_code);
    }
}
