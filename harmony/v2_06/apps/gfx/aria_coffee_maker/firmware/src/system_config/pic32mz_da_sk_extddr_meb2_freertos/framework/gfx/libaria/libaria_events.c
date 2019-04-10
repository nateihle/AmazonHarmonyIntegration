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

// SizeSelect - SelectedItemChangedEvent
void SizeSelect_SelectedItemChangedEvent(laListWheelWidget* whl, uint32_t idx)
{
    // NotifyApp
    APP_CycleBrewSize();
}

// BrewButton - PressedEvent
void BrewButton_PressedEvent(laButtonWidget* btn)
{
    // TriggerBrew
    APP_TriggerBrewPressed();
}

// BrewButton - ReleasedEvent
void BrewButton_ReleasedEvent(laButtonWidget* btn)
{
    // BrewTrigger
    APP_TriggerBrew();
}

// ChangeLanguage - ReleasedEvent
void ChangeLanguage_ReleasedEvent(laButtonWidget* btn)
{
    // CycleLanguage
    APP_CycleLanguage();
}

// CoffeeButton - ReleasedEvent
void CoffeeButton_ReleasedEvent(laButtonWidget* btn)
{
    // AdjustRightAlpha
    APP_CycleRightTrayAlpha();
}

// CoffeeBeanButton - ReleasedEvent
void CoffeeBeanButton_ReleasedEvent(laButtonWidget* btn)
{
    // CycleBackground
    APP_CycleBeansBackground();
}

// TeaButton - ReleasedEvent
void TeaButton_ReleasedEvent(laButtonWidget* btn)
{
    // AdjustLeftAlpha
    APP_CycleLeftTrayAlpha();
}

// InfoPageButton - ReleasedEvent
void InfoPageButton_ReleasedEvent(laButtonWidget* btn)
{
    // TellAppToGoToInfo
    APP_GoToInfoState();

    // GoToInfo - Show Screen - InfoScreen
    laContext_SetActiveScreen(InfoScreen_ID);
}

// GPUButton - ReleasedEvent
void GPUButton_ReleasedEvent(laButtonWidget* btn)
{
    // ToggleGPU
    APP_ToggleGPU();
}

// ReturnToMainButton - ReleasedEvent
void ReturnToMainButton_ReleasedEvent(laButtonWidget* btn)
{
    // ReturnToMain - Show Screen - MainScreen
    laContext_SetActiveScreen(MainScreen_ID);

    // SwitchAppState
    APP_GoToMainState();
}

// ButtonWidget1 - ReleasedEvent
void ButtonWidget1_ReleasedEvent(laButtonWidget* btn)
{
    // ChangeScreenToSplash - Show Screen - SplashScreen
    laContext_SetActiveScreen(SplashScreen_ID);

    // UpdateAppToSplash
    APP_GoToSplashState();
}





