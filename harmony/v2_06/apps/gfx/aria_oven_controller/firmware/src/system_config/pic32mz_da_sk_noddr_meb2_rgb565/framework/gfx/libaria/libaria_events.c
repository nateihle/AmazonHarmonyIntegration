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

// homeScreen - ShowEvent
void homeScreen_ShowEvent(laScreen* scr)
{
    // animateHomeScreen
    APP_AnimateHomeScreen();
}

// ButtonWidget3 - ReleasedEvent
void ButtonWidget3_ReleasedEvent(laButtonWidget* btn)
{
    // showInfoScreen - Show Screen - infoScreen
    laContext_SetActiveScreen(infoScreen_ID);
}

// StartButton - PressedEvent
void StartButton_PressedEvent(laButtonWidget* btn)
{
    // onStartButtonPressed - Show Screen - controllerScreen
    laContext_SetActiveScreen(controllerScreen_ID);
}

// controllerScreen - ShowEvent
void controllerScreen_ShowEvent(laScreen* scr)
{
    // activateControllerScreen
    APP_AnimateControllerScreen();
}

// ButtonWidget - ReleasedEvent
void ButtonWidget_ReleasedEvent(laButtonWidget* btn)
{
    // showHomeScreen - Show Screen - homeScreen
    laContext_SetActiveScreen(homeScreen_ID);
}

// FishButtonWidget - ReleasedEvent
void FishButtonWidget_ReleasedEvent(laButtonWidget* btn)
{
    // onFishButtonRelease
    APP_OnFishButtonRelease();
}

// PizzaButtonWidget - ReleasedEvent
void PizzaButtonWidget_ReleasedEvent(laButtonWidget* btn)
{
    // onPizzaButtonRelease
    APP_OnPizzaButtonRelease();
}

// VegeButtonWidget - ReleasedEvent
void VegeButtonWidget_ReleasedEvent(laButtonWidget* btn)
{
    // onVegeButtonRelease
    APP_OnVegeButtonRelease();
}

// StartStopButton - PressedEvent
void StartStopButton_PressedEvent(laButtonWidget* btn)
{
    // onStartButtonToggled
    APP_OnStartButtonToggled();
}

// StartStopButton - ReleasedEvent
void StartStopButton_ReleasedEvent(laButtonWidget* btn)
{
    // onStartButtonToggled
    APP_OnStartButtonToggled();
}

// DoneButton - ReleasedEvent
void DoneButton_ReleasedEvent(laButtonWidget* btn)
{
    // onButtonDone
    APP_OnButtonDoneReleased();
}

// ButtonWidget22 - ReleasedEvent
void ButtonWidget22_ReleasedEvent(laButtonWidget* btn)
{
    // showControllerScreen1 - Show Screen - homeScreen
    laContext_SetActiveScreen(homeScreen_ID);
}





