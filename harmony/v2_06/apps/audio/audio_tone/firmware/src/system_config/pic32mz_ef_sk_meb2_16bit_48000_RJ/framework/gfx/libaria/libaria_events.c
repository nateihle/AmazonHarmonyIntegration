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
#include "buttons.h"        // KEEP THIS LINE

// KEEP ALL USER-CREATED CODE INSIDE FUNCTIONS!
// sinechirp - PressedEvent
void sinechirp_PressedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_SINECHIRP] = true;        // KEEP
    buttons_handleInterrupt();              // KEEP -- ETC.    
}

// sinechirp - ReleasedEvent
void sinechirp_ReleasedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_SINECHIRP] = false;
    buttons_handleInterrupt();    
}

// f1plus - PressedEvent
void f1plus_PressedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_F1_PLUS] = true;
    buttons_handleInterrupt(); 
}

// f1plus - ReleasedEvent
void f1plus_ReleasedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_F1_PLUS] = false;
    buttons_handleInterrupt();
}

// f1mnus - PressedEvent
void f1mnus_PressedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_F1_MINUS] = true;
    buttons_handleInterrupt();
}

// f1mnus - ReleasedEvent
void f1mnus_ReleasedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_F1_MINUS] = false;
    buttons_handleInterrupt();
}

// f2plus - PressedEvent
void f2plus_PressedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_F2_PLUS] = true;
    buttons_handleInterrupt();
}

// f2plus - ReleasedEvent
void f2plus_ReleasedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_F2_PLUS] = false;
    buttons_handleInterrupt();
}

// f2minus - PressedEvent
void f2minus_PressedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_F2_MINUS] = true;
    buttons_handleInterrupt();
}

// f2minus - ReleasedEvent
void f2minus_ReleasedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_F2_MINUS] = false;
    buttons_handleInterrupt();
}

// tplus - PressedEvent
void tplus_PressedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_T_PLUS] = true;
    buttons_handleInterrupt();
}

// tplus - ReleasedEvent
void tplus_ReleasedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_T_PLUS] = false;
    buttons_handleInterrupt();
}

// tminus - PressedEvent
void tminus_PressedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_T_MINUS] = true;
    buttons_handleInterrupt();
}

// tminus - ReleasedEvent
void tminus_ReleasedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_T_MINUS] = false;
    buttons_handleInterrupt();
}

// volumeplus - PressedEvent
void volumeplus_PressedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_VOLUME_PLUS] = true;
    buttons_handleInterrupt();
}

// volumeplus - ReleasedEvent
void volumeplus_ReleasedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_VOLUME_PLUS] = false;
    buttons_handleInterrupt();
}

// volumeminus - PressedEvent
void volumeminus_PressedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_VOLUME_MINUS] = true;
    buttons_handleInterrupt();
}

// volumeminus - ReleasedEvent
void volumeminus_ReleasedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_VOLUME_MINUS] = false;
    buttons_handleInterrupt();
}

// GFX_PLAYPAUSE - PressedEvent
void GFX_PLAYPAUSE_PressedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_PLAYPAUSE] = true;
    buttons_handleInterrupt();    
}

// GFX_PLAYPAUSE - ReleasedEvent
void GFX_PLAYPAUSE_ReleasedEvent(laButtonWidget* btn)
{
    buttons[TOUCH_PLAYPAUSE] = false;
    buttons_handleInterrupt();    
}





