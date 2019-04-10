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

// MainScreen - ShowEvent
void MainScreen_ShowEvent(laScreen* scr)
{
    // Custom Action
    APP_InitializeMainMenuScreen();
}

// PlayUSBButton - ReleasedEvent
void PlayUSBButton_ReleasedEvent(laButtonWidget* btn)
{
    // PlayUSB
    APP_SetUserEvent(APP_EVENT_USER_START_USB_PLAY);
}

// PlaySDButton - ReleasedEvent
void PlaySDButton_ReleasedEvent(laButtonWidget* btn)
{
    // PaySD
    APP_SetUserEvent(APP_EVENT_USER_START_SD_PLAY);
}

// ButtonWidget15 - ReleasedEvent
void ButtonWidget15_ReleasedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_OPEN_HELP);   
}

// ButtonWidget12 - ReleasedEvent
void ButtonWidget12_ReleasedEvent(laButtonWidget* btn)
{
    // OpenSettings
    APP_SetUserEvent(APP_EVENT_USER_OPEN_SETTINGS);
}

// ButtonWidget7 - ReleasedEvent
void ButtonWidget7_ReleasedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_BACK_TO_MAIN_MENU);
}

// SettingsScreen - ShowEvent
void SettingsScreen_ShowEvent(laScreen* scr)
{
    // Custom Action
    APP_InitializeSettingsScreen();
}

// BackToMenuButton - ReleasedEvent
void BackToMenuButton_ReleasedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_BACK_TO_MAIN_MENU);
}

// VertAlignListWidget - SelectionChangedEvent
void VertAlignListWidget_SelectionChangedEvent(laListWidget* img, uint32_t idx, laBool selected)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_SETTINGS_SET_VERT_ALIGN);
}

// HorzAlignListWidget - SelectionChangedEvent
void HorzAlignListWidget_SelectionChangedEvent(laListWidget* img, uint32_t idx, laBool selected)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_SETTINGS_SET_HORZ_ALIGN);
}

// ResolutionListWidget - SelectionChangedEvent
void ResolutionListWidget_SelectionChangedEvent(laListWidget* img, uint32_t idx, laBool selected)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_SETTINGS_SET_RESOLUTION);
}

// PlaySpeedListWidget - SelectionChangedEvent
void PlaySpeedListWidget_SelectionChangedEvent(laListWidget* img, uint32_t idx, laBool selected)
{
    // Custom Action
     APP_SetUserEvent(APP_EVENT_USER_SETTINGS_SET_FPS);
}

// ShowFrameRateButtonWidget - PressedEvent
void ShowFrameRateButtonWidget_PressedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_SETTINGS_SHOW_FRAME_RATE);
}

// ShowFrameRateButtonWidget - ReleasedEvent
void ShowFrameRateButtonWidget_ReleasedEvent(laButtonWidget* btn)
{
    // Custom Action
     APP_SetUserEvent(APP_EVENT_USER_SETTINGS_HIDE_FRAME_RATE); 
}

// PlayBackScreen - ShowEvent
void PlayBackScreen_ShowEvent(laScreen* scr)
{
    // Custom Action
    APP_InitializePlaybackScreen();
}

// PlayPauseButtonWidget - PressedEvent
void PlayPauseButtonWidget_PressedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_PLAY);   
}

// PlayPauseButtonWidget - ReleasedEvent
void PlayPauseButtonWidget_ReleasedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_PAUSE);
}

// FFButtonWidget - PressedEvent
void FFButtonWidget_PressedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_FF); 
}

// RewindButtonWidget - PressedEvent
void RewindButtonWidget_PressedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_RW);
}

// StopButtonWidget - ReleasedEvent
void StopButtonWidget_ReleasedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_STOP);
}

// RestartButtonWidget - PressedEvent
void RestartButtonWidget_PressedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_RESTART); 
}

// SliderControlFull - ValueChangedEvent
void SliderControlFull_ValueChangedEvent(laSliderWidget* sld)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_SEEK);
}

// PlayBackScreenSmall - ShowEvent
void PlayBackScreenSmall_ShowEvent(laScreen* scr)
{
    // Custom Action
    APP_InitializePlaybackScreen();
}

// StopButtonWidget2 - PressedEvent
void StopButtonWidget2_PressedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_STOP);
}

// FFButtonWidget2 - PressedEvent
void FFButtonWidget2_PressedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_FF);
}

// RWButtonWidget2 - PressedEvent
void RWButtonWidget2_PressedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_RW);
}

// PauseButtonWidget2 - PressedEvent
void PauseButtonWidget2_PressedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_PLAY);
}

// PauseButtonWidget2 - ReleasedEvent
void PauseButtonWidget2_ReleasedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_PAUSE);
}

// RestartButtonWidget2 - PressedEvent
void RestartButtonWidget2_PressedEvent(laButtonWidget* btn)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_RESTART); 
}

// SliderControlSmall - ValueChangedEvent
void SliderControlSmall_ValueChangedEvent(laSliderWidget* sld)
{
    // Custom Action
    APP_SetUserEvent(APP_EVENT_USER_PLAYBACK_SEEK); 
}





