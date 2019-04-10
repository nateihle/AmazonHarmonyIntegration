/*****************************************************************************
* FileName:         assert_to_display.c
*
* Dependencies:     ??
* Processor:        PIC32
* Compiler:         C32
* Linker:           MPLAB LINK32
* Company:          Microchip Technology Incorporated
*/
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

#include "app.h"
#include "error/error.h"
//#include "gfx_resources.h"
//#include "gfx/gfx.h"
#include "system_definitions.h" // for access to sysObj

#if defined( ENABLE_SYS_LOG )
  #include "sys_log/sys_log.h"
#endif

#include "system/debug/sys_debug.h"

char MessageString[250];
char DisplayString[250];

//static void _UnsignedInt2String(uint16_t UnsignedInt,const uint16_t NumberBase,char * String)
//{
//    int16_t  iDigit;
//    uint16_t iMaxDigit  = 0;
//    uint16_t DigitPower = 1;
//
//    while (UnsignedInt/DigitPower >= NumberBase)
//    {
//        DigitPower *= NumberBase;
//        iMaxDigit++;
//    }
//
//    for (iDigit = iMaxDigit; iDigit >= 0; iDigit--)
//    {
//        uint16_t DigitValue = UnsignedInt / DigitPower;
//        UnsignedInt %= DigitPower;
//        DigitPower/=NumberBase;
//        *String++ = DigitValue + (DigitValue<10 ? '0' : 'A'-10);
//    }
//
//    *String=0; //Terminate string

//}//_UnsignedInt2String


// See also system_init.c.  Value used should match call to GFX_Initialize.
//static GFX_INDEX  _GFX_Index = GFX_INDEX_0;

void assert_to_display(char * message, uint16_t Line, char *Filename)
{
    //TODO: Update for GFX2
    error_onFatalError();
}//assert_to_display
