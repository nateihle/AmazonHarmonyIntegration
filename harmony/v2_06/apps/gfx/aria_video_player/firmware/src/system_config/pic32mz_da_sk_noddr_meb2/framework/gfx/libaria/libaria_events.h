/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Definitions Header

  File Name:
    libaria_events.h

  Summary:
    Build-time generated definitions header based on output by the MPLAB Harmony
    Graphics Composer.

  Description:
    Build-time generated definitions header based on output by the MPLAB Harmony
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

#ifndef _LIBARIA_EVENTS_H
#define _LIBARIA_EVENTS_H

#include "gfx/libaria/libaria.h"
#include "gfx/libaria/libaria_init.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// Generated Event Handler - Origin: MainScreen, Event: ShowEvent
void MainScreen_ShowEvent(laScreen* scr);

// Generated Event Handler - Origin: PlaySDButton, Event: ReleasedEvent
void PlaySDButton_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: ButtonWidget15, Event: ReleasedEvent
void ButtonWidget15_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: ButtonWidget12, Event: ReleasedEvent
void ButtonWidget12_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: PlayUSBButton, Event: ReleasedEvent
void PlayUSBButton_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: ButtonWidget7, Event: ReleasedEvent
void ButtonWidget7_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: SettingsScreen, Event: ShowEvent
void SettingsScreen_ShowEvent(laScreen* scr);

// Generated Event Handler - Origin: BackToMenuButton, Event: ReleasedEvent
void BackToMenuButton_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: VertAlignListWidget, Event: SelectionChangedEvent
void VertAlignListWidget_SelectionChangedEvent(laListWidget* img, uint32_t idx, laBool selected);

// Generated Event Handler - Origin: HorzAlignListWidget, Event: SelectionChangedEvent
void HorzAlignListWidget_SelectionChangedEvent(laListWidget* img, uint32_t idx, laBool selected);

// Generated Event Handler - Origin: ResolutionListWidget, Event: SelectionChangedEvent
void ResolutionListWidget_SelectionChangedEvent(laListWidget* img, uint32_t idx, laBool selected);

// Generated Event Handler - Origin: PlaySpeedListWidget, Event: SelectionChangedEvent
void PlaySpeedListWidget_SelectionChangedEvent(laListWidget* img, uint32_t idx, laBool selected);

// Generated Event Handler - Origin: ShowFrameRateButtonWidget, Event: PressedEvent
void ShowFrameRateButtonWidget_PressedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: ShowFrameRateButtonWidget, Event: ReleasedEvent
void ShowFrameRateButtonWidget_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: PlayBackScreen, Event: ShowEvent
void PlayBackScreen_ShowEvent(laScreen* scr);

// Generated Event Handler - Origin: PlayPauseButtonWidget, Event: PressedEvent
void PlayPauseButtonWidget_PressedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: PlayPauseButtonWidget, Event: ReleasedEvent
void PlayPauseButtonWidget_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: FFButtonWidget, Event: PressedEvent
void FFButtonWidget_PressedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: RewindButtonWidget, Event: PressedEvent
void RewindButtonWidget_PressedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: StopButtonWidget, Event: ReleasedEvent
void StopButtonWidget_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: RestartButtonWidget, Event: PressedEvent
void RestartButtonWidget_PressedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: SliderControlFull, Event: ValueChangedEvent
void SliderControlFull_ValueChangedEvent(laSliderWidget* sld);

// Generated Event Handler - Origin: PlayBackScreenSmall, Event: ShowEvent
void PlayBackScreenSmall_ShowEvent(laScreen* scr);

// Generated Event Handler - Origin: StopButtonWidget2, Event: PressedEvent
void StopButtonWidget2_PressedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: FFButtonWidget2, Event: PressedEvent
void FFButtonWidget2_PressedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: RWButtonWidget2, Event: PressedEvent
void RWButtonWidget2_PressedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: PauseButtonWidget2, Event: PressedEvent
void PauseButtonWidget2_PressedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: PauseButtonWidget2, Event: ReleasedEvent
void PauseButtonWidget2_ReleasedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: RestartButtonWidget2, Event: PressedEvent
void RestartButtonWidget2_PressedEvent(laButtonWidget* btn);

// Generated Event Handler - Origin: SliderControlSmall, Event: ValueChangedEvent
void SliderControlSmall_ValueChangedEvent(laSliderWidget* sld);



//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _LIBARIA_EVENTS_H
/*******************************************************************************
 End of File
*/
