/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Definitions Header

  File Name:
    libaria_macros.h

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

#ifndef _LIBARIA_INIT_H
#define _LIBARIA_INIT_H

#ifndef NATIVE
#include "system_config.h"
#include "system_definitions.h"
#endif

#include "gfx/libaria/libaria.h"
#include "gfx/libaria/libaria_events.h"

#include "gfx/gfx_assets.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

#define LIBARIA_SCREEN_COUNT   6

// reference IDs for generated libaria screens
// screen "HelpScreen"
#define HelpScreen_ID    2

// screen "MainScreen"
#define MainScreen_ID    1

// screen "PlayBackScreen"
#define PlayBackScreen_ID    4

// screen "PlayBackScreenSmall"
#define PlayBackScreenSmall_ID    5

// screen "SettingsScreen"
#define SettingsScreen_ID    3

// screen "SplashScreen"
#define SplashScreen_ID    0



extern laScheme RedTextScheme;
extern laScheme StreakScheme;
extern laScheme ClearScheme;
extern laScheme RedLineScheme;
extern laScheme BlackBackgroundScheme;
extern laScheme ButtonTextScheme;
extern laScheme defaultScheme;
extern laScheme BlueTextScheme;
extern laScheme whiteScheme;
extern laScheme PanelScheme;
extern laScheme AriaColorScheme;
extern laScheme FilmBackgroundScheme;
extern laScheme WhiteTextBlackBackgroundScheme;
extern laScheme WhiteTextScheme;
extern laImageWidget* ImageWidget1;
extern laImageWidget* ImageWidget3;
extern laImageWidget* ImageWidget2;
extern laImageWidget* ImageWidget4;
extern laImageWidget* ImageWidget;
extern laButtonWidget* PlayUSBButton;
extern laButtonWidget* PlaySDButton;
extern laButtonWidget* ButtonWidget15;
extern laButtonWidget* ButtonWidget12;
extern laLabelWidget* NoMediaLabelWidget;
extern laWidget* AppLogoPanel;
extern laLabelWidget* LabelWidget1;
extern laLabelWidget* LabelWidget2;
extern laLineWidget* LineWidget;
extern laImageWidget* ImageWidget;
extern laLabelWidget* LabelWidget5;
extern laButtonWidget* ButtonWidget7;
extern laLabelWidget* LabelWidget8;
extern laLabelWidget* LabelWidget12;
extern laLabelWidget* LabelWidget6;
extern laLabelWidget* LabelWidget16;
extern laLabelWidget* LabelWidget;
extern laWidget* PanelWidget10;
extern laLabelWidget* LabelWidget3;
extern laLabelWidget* LabelWidget4;
extern laLabelWidget* LabelWidget480x272;
extern laLabelWidget* LabelWidgetvideo1rgb;
extern laLabelWidget* LabelWidget7;
extern laLabelWidget* LabelWidgetvideo2rgb;
extern laLabelWidget* LabelWidget320x180;
extern laLabelWidget* LabelWidget9;
extern laButtonWidget* BackToMenuButton;
extern laGroupBoxWidget* GroupBoxWidget1;
extern laGroupBoxWidget* GroupBoxWidget14;
extern laGroupBoxWidget* GroupBoxWidget2;
extern laListWidget* VertAlignListWidget;
extern laListWidget* HorzAlignListWidget;
extern laListWidget* ResolutionListWidget;
extern laListWidget* PlaySpeedListWidget;
extern laButtonWidget* ShowFrameRateButtonWidget;
extern laCheckBoxWidget* CheckBoxWidget13;
extern laWidget* TouchPanelWidget;
extern laLabelWidget* PlaybackMessageLabel;
extern laWidget* PlayBackControlPanel;
extern laButtonWidget* PlayPauseButtonWidget;
extern laButtonWidget* FFButtonWidget;
extern laButtonWidget* RewindButtonWidget;
extern laButtonWidget* StopButtonWidget;
extern laButtonWidget* RestartButtonWidget;
extern laLabelWidget* PlaybackMultiplierLabelWidget;
extern laWidget* MetricsPanelWidget1;
extern laLabelWidget* frameRateLabelWidget;
extern laLabelWidget* BandWidthLabelWidget;
extern laSliderWidget* SliderControlFull;
extern laWidget* TouchPanelWidget2;
extern laLabelWidget* PlaybackMessageLabel2;
extern laWidget* RightPanelWidget;
extern laButtonWidget* StopButtonWidget2;
extern laButtonWidget* FFButtonWidget2;
extern laButtonWidget* RWButtonWidget2;
extern laButtonWidget* PauseButtonWidget2;
extern laButtonWidget* RestartButtonWidget2;
extern laLabelWidget* PlaybackMultiplierLabelWidget2;
extern laWidget* MetricsPanelWidget2;
extern laLabelWidget* BandWidthLabelWidget2;
extern laLabelWidget* FrameRateLabelWidget2;
extern laSliderWidget* SliderControlSmall;


int32_t libaria_initialize(void);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _LIBARIA_INIT_H
/*******************************************************************************
 End of File
*/
