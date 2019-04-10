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

#include "assert_to_display.h"
#include "error.h"
#include "app.h"
//#include "gfx_resources.h"
//#include "gfx/gfx.h"
#include "system_definitions.h" // for access to sysObj

char MessageString[250];
char DisplayString[250];
//
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
//
//}//_UnsignedInt2String


// See also system_init.c.  Value used should match call to GFX_Initialize.
//static GFX_INDEX  _GFX_Index = GFX_INDEX_0;

void assert_to_display(char *Expression, uint16_t Line, char *Filename)
{
//    char *pDisplayString, *pEndDisplayString, *pEndString;
//    uint16_t LabelHeight;
//    uint16_t nLines;
//
//    if ( sysObj.gfxObject0 != (uintptr_t)NULL )
//    { // Graphics initialized, write out error message
//        GFX_ColorSet(_GFX_Index,BLACK);
//        GFX_ScreenClear(_GFX_Index);
//
//        GFX_ColorSet(_GFX_Index, BRIGHTRED );
//        
//        GFX_FontSet(_GFX_Index, (GFX_RESOURCE_HDR *)&Arial14pt);
//        LabelHeight =  GFX_TextStringHeightGet(GFX_FontGet(_GFX_Index));
//
//        strcpy(MessageString,"Failed assertion ");
//        strcat(MessageString,Expression);
//        strcat(MessageString," at line ");
//        _UnsignedInt2String(Line,10,MessageString + strlen(MessageString));
//        strcat(MessageString," of file ");
//        strcat(MessageString,Filename);
//      //strcat(MessageString,"  **HIT SW7 TO RESTART**");
//
//        nLines = 1;
//        strcpy(DisplayString,MessageString);
//        pDisplayString = DisplayString;
//        pEndDisplayString = pEndString = pDisplayString + strlen(pDisplayString);
//
//        while( pEndString - pDisplayString > 0 )
//        {
//            while( GFX_TextStringWidthGet(pDisplayString,GFX_FontGet(_GFX_Index)) > GFX_MaxXGet(_GFX_Index) )
//            {
//                *(pEndString--)=0;
//            }
//            GFX_TextStringDraw(_GFX_Index,1,(nLines-1)*(1+LabelHeight),pDisplayString,0);
//            nLines++;
//            strcpy(DisplayString,MessageString);
//            pDisplayString = pEndString+1;
//            pEndString = pEndDisplayString;
//        }     
//    }
//    
//    // Flash LEDs and drop into while(1) loop to wait for user reset
//    error_onFatalError();   

}//assert_to_display
