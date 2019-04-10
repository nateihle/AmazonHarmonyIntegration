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

// CUSTOM CODE DO NOT DELETE
#include "app_display_task.h"
// END OF CUSTOM CODE

// TimePerDivIncButton - ReleasedEvent
void TimePerDivIncButton_ReleasedEvent(laButtonWidget* btn)
{
    // TimePerDiv_Increment
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_TIME_PER_DIV_INC_BTN_PRESSED);
}

// TimePerDivDecButton - ReleasedEvent
void TimePerDivDecButton_ReleasedEvent(laButtonWidget* btn)
{
    // TimePerDiv_Decrement
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_TIME_PER_DIV_DEC_BTN_PRESSED);
}

// AmpIncButton - ReleasedEvent
void AmpIncButton_ReleasedEvent(laButtonWidget* btn)
{
    // AmpScale_Increment
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_AMP_SCALE_INC_BTN_PRESSED);
}

// AmpDecButton - ReleasedEvent
void AmpDecButton_ReleasedEvent(laButtonWidget* btn)
{
    // AmpScale_Decrement
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_AMP_SCALE_DEC_BTN_PRESSED);
}

// GridButton - ReleasedEvent
void GridButton_ReleasedEvent(laButtonWidget* btn)
{
    // Grid_ButtonPressEvent
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_GRID_BTN_PRESSED);
}

// MainScreenButton_InvisibleTouchArea - ReleasedEvent
void MainScreenButton_InvisibleTouchArea_ReleasedEvent(laButtonWidget* btn)
{
    // FreqDomainScreen_ButtonPressed - Show Screen - MainScreen
    laContext_SetActiveScreen(MainScreen_ID);
    // CUSTOM CODE DO NOT DELETE
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_MAIN_SCREEN_BTN_PRESSED);
    // END OF CUSTOM CODE
}

// MicButton - ReleasedEvent
void MicButton_ReleasedEvent(laButtonWidget* btn)
{
    // Mic_Button_Release
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_MIC_BTN_PRESSED);
}

// ToneButton - ReleasedEvent
void ToneButton_ReleasedEvent(laButtonWidget* btn)
{
    // Tone_Button_Release
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_TONE_BTN_PRESSED);
}

// TimeDomainButton_InvisibleTouchArea - ReleasedEvent
void TimeDomainButton_InvisibleTouchArea_ReleasedEvent(laButtonWidget* btn)
{
    // TimeDomainScreen_ButtonPressed - Show Screen - TimeDomainScreen
    laContext_SetActiveScreen(TimeDomainScreen_ID);
    // CUSTOM CODE DO NOT DELETE
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_TIME_DOMAIN_SCREEN_BTN_PRESSED);
    // END OF CUSTOM CODE
}

// VolumeControl - ValueChangedEvent
void VolumeControl_ValueChangedEvent(laSliderWidget* sld)
{
    // volume_control_value_changed
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_VOLUME_SLIDER_CHANGED);
}

// PlayButton - ReleasedEvent
void PlayButton_ReleasedEvent(laButtonWidget* btn)
{
    // Play_Button_Release
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_PLAY_BTN_PRESSED);
}

// F1_InvisibleTouchArea - ReleasedEvent
void F1_InvisibleTouchArea_ReleasedEvent(laButtonWidget* btn)
{
    // F1_RadioButtonPressEvent
    laRadioButtonWidget_SetSelected(F1_RadioButton);
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_F1_RADIO_BTN_SEL);
}

// F2_InvisibleTouchArea - ReleasedEvent
void F2_InvisibleTouchArea_ReleasedEvent(laButtonWidget* btn)
{
    // F2_RadioButtonPressEvent
    laRadioButtonWidget_SetSelected(F2_RadioButton);
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_F2_RADIO_BTN_SEL);
}

// F3_InvisibleTouchArea - ReleasedEvent
void F3_InvisibleTouchArea_ReleasedEvent(laButtonWidget* btn)
{
    // F3_RadioButtonPressEvent
    laRadioButtonWidget_SetSelected(F3_RadioButton);
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_F3_RADIO_BTN_SEL);
}

// IncButton - ReleasedEvent
void IncButton_ReleasedEvent(laButtonWidget* btn)
{
    // Plus_Button_Release
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_INC_BTN_PRESSED);
}

// DecButton - ReleasedEvent
void DecButton_ReleasedEvent(laButtonWidget* btn)
{
    // Minus_Button_Release
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_DEC_BTN_PRESSED);
}

// ClrButton - ReleasedEvent
void ClrButton_ReleasedEvent(laButtonWidget* btn)
{
    // Clr_Button_Release
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_CLR_BTN_PRESSED);
}

// UnitsButtonHz - ReleasedEvent
void UnitsButtonHz_ReleasedEvent(laButtonWidget* btn)
{
    // Hz_Button_Release
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_HZ_BTN_PRESSED);
}

// UnitsButtonKHz - ReleasedEvent
void UnitsButtonKHz_ReleasedEvent(laButtonWidget* btn)
{
    // Khz_Button_Release
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_KHZ_BTN_PRESSED);
}

// UnitsButtondBFS - ReleasedEvent
void UnitsButtondBFS_ReleasedEvent(laButtonWidget* btn)
{
    // dbfs_Button_Release
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_DBFS_BTN_PRESSED);
}

// Hanning_Win_InvisibleTouchArea - ReleasedEvent
void Hanning_Win_InvisibleTouchArea_ReleasedEvent(laButtonWidget* btn)
{
    // HannWindow_RadioButtonPressed
    laRadioButtonWidget_SetSelected(HannWindow_RadioButton);
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_HANN_WIN_RADIO_BTN_PRESSED);
}

// Blackman_Win_InvisibleTouchArea - ReleasedEvent
void Blackman_Win_InvisibleTouchArea_ReleasedEvent(laButtonWidget* btn)
{
    // BlackmanWindow_RadioButtonPressed
    laRadioButtonWidget_SetSelected(BlackmanWindow_RadioButton);
    APP_DISPLAY_AddCommand(APP_DISPLAY_CMD_UI_EVENT_BLACKMAN_WIN_RADIO_BTN_PRESSED);
}





