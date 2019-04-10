/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Implementation File

  File Name:
    libaria_events.c

  Summary:
    Build-time generated implementation from the MPLAB Harmony
    Graphics Composer.

  Description:
    Build-time generated implementation from the MPLAB Harmony
    Graphics Composer.

    Created with MPLAB Harmony Version 2.06
*******************************************************************************/
// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "gfx/libaria/libaria_events.h"
// CUSTOM CODE - DO NOT DELETE
extern bool bDisplay_S1State;
// END OF CUSTOM CODE

// ButtonWidget1 - PressedEvent
void ButtonWidget1_PressedEvent(laButtonWidget* btn)
{
    // ButtonDown - Set Text - ButtonWidget1
    laButtonWidget_SetText((laButtonWidget*)ButtonWidget1, laString_CreateFromID(string_OuchOuchOuch));
}

// ButtonWidget1 - ReleasedEvent
void ButtonWidget1_ReleasedEvent(laButtonWidget* btn)
{
    // ButtonUp - Set Text - ButtonWidget1
    laButtonWidget_SetText((laButtonWidget*)ButtonWidget1, laString_CreateFromID(string_Instructions));
}

// Display_S1 - PressedEvent
void Display_S1_PressedEvent(laButtonWidget* btn)
{
  // CUSTOM CODE - DO NOT DELETE
    bDisplay_S1State = true;
  // END OF CUSTOM CODE
}

// Display_S1 - ReleasedEvent
void Display_S1_ReleasedEvent(laButtonWidget* btn)
{
  // CUSTOM CODE - DO NOT DELETE
    bDisplay_S1State = false;
  // END OF CUSTOM CODE
}





