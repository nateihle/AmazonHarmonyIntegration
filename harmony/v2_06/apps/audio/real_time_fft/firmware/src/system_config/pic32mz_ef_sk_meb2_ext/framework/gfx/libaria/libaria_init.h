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

#define LIBARIA_SCREEN_COUNT   2

// reference IDs for generated libaria screens
// screen "MainScreen"
#define MainScreen_ID    1

// screen "TimeDomainScreen"
#define TimeDomainScreen_ID    0



extern laScheme LineScheme;
extern laScheme BasePanelScheme;
extern laScheme ButtonSchemeInActive;
extern laScheme GradientScheme;
extern laScheme RadioButtonScheme;
extern laScheme LabelScheme;
extern laScheme YellowTextScheme;
extern laScheme PanelScheme;
extern laScheme ButtonSchemeActive;
extern laScheme WhiteTextScheme;
extern laScheme BarScheme;
extern laScheme VolumeButtonScheme;
extern laScheme BackgroundScheme;
extern laGradientWidget* Background_Gradient2;
extern laWidget* StatusPanel;
extern laWidget* TimePerDivStatusPanel;
extern laLabelWidget* CurrentTimePerDivLabel;
extern laLabelWidget* CurrentTimePerDivValueLabel;
extern laWidget* AmpPerDivStatusPanel;
extern laLabelWidget* CurrentAmpPerDivLabel;
extern laLabelWidget* CurrentAmpPerDivValueLabel;
extern laWidget* ControlPanel;
extern laWidget* TimePerDivControlPanel;
extern laLabelWidget* TimeDivLabel;
extern laButtonWidget* TimePerDivIncButton;
extern laButtonWidget* TimePerDivDecButton;
extern laWidget* AmpPerDivControlPanel;
extern laLabelWidget* AmpScaleLabel;
extern laButtonWidget* AmpIncButton;
extern laButtonWidget* AmpDecButton;
extern laWidget* GridControlPanel;
extern laLabelWidget* GridLabel;
extern laButtonWidget* GridButton;
extern laDrawSurfaceWidget* GraphSurfaceWidget;
extern laButtonWidget* MainScreenButton_InvisibleTouchArea;
extern laButtonWidget* MainScreenButton;
extern laGradientWidget* Background_Gradient1;
extern laWidget* ModePanel;
extern laButtonWidget* MicButton;
extern laButtonWidget* ToneButton;
extern laWidget* SelectedSignalsPanel;
extern laLabelWidget* F1_Label;
extern laLabelWidget* F2_Label;
extern laLabelWidget* F3_Label;
extern laLabelWidget* PlusSign1Label;
extern laLabelWidget* PlusSign2Label;
extern laButtonWidget* TimeDomainButton_InvisibleTouchArea;
extern laButtonWidget* TimeDomainScreenButton;
extern laImageWidget* MicrochipLogo;
extern laWidget* FFTDisplayPanel;
extern laWidget* YAxisPanel;
extern laWidget* XAxisPanel;
extern laLabelWidget* dBFSMaxLabel;
extern laLabelWidget* dBFSMinLabel;
extern laLabelWidget* YAxisUnitLabel;
extern laLabelWidget* FreqLabel1;
extern laLabelWidget* FreqLabel2;
extern laLabelWidget* FreqLabel3;
extern laLabelWidget* FreqLabel4;
extern laLabelWidget* FreqLabel5;
extern laLabelWidget* FreqLabel6;
extern laLabelWidget* FreqLabel7;
extern laLabelWidget* FreqLabel8;
extern laLabelWidget* FreqLabel9;
extern laLabelWidget* FreqLabel10;
extern laLabelWidget* FreqLabel11;
extern laLabelWidget* FreqLabel12;
extern laLabelWidget* FreqLabel13;
extern laLabelWidget* FreqLabel14;
extern laLabelWidget* FreqLabel15;
extern laLabelWidget* FreqLabel16;
extern laLabelWidget* FreqLabel17;
extern laLabelWidget* FreqLabel18;
extern laLabelWidget* FreqLabel19;
extern laLabelWidget* FreqLabel20;
extern laLabelWidget* FreqLabel21;
extern laLabelWidget* FreqLabel22;
extern laLabelWidget* FreqLabel23;
extern laLabelWidget* FreqLabel24;
extern laLabelWidget* XAxisUnitLabel;
extern laProgressBarWidget* ProgressBarWidget1;
extern laProgressBarWidget* ProgressBarWidget2;
extern laProgressBarWidget* ProgressBarWidget3;
extern laProgressBarWidget* ProgressBarWidget4;
extern laProgressBarWidget* ProgressBarWidget5;
extern laProgressBarWidget* ProgressBarWidget6;
extern laProgressBarWidget* ProgressBarWidget7;
extern laProgressBarWidget* ProgressBarWidget8;
extern laProgressBarWidget* ProgressBarWidget9;
extern laProgressBarWidget* ProgressBarWidget10;
extern laProgressBarWidget* ProgressBarWidget11;
extern laProgressBarWidget* ProgressBarWidget12;
extern laProgressBarWidget* ProgressBarWidget13;
extern laProgressBarWidget* ProgressBarWidget14;
extern laProgressBarWidget* ProgressBarWidget15;
extern laProgressBarWidget* ProgressBarWidget16;
extern laProgressBarWidget* ProgressBarWidget17;
extern laProgressBarWidget* ProgressBarWidget18;
extern laProgressBarWidget* ProgressBarWidget19;
extern laProgressBarWidget* ProgressBarWidget20;
extern laProgressBarWidget* ProgressBarWidget21;
extern laProgressBarWidget* ProgressBarWidget22;
extern laProgressBarWidget* ProgressBarWidget23;
extern laProgressBarWidget* ProgressBarWidget24;
extern laLabelWidget* MessageLabel;
extern laWidget* VolumeSliderPanel;
extern laSliderWidget* VolumeControl;
extern laLabelWidget* VolumeLabel;
extern laWidget* PlayButtonPanel;
extern laButtonWidget* PlayButton;
extern laWidget* SignalSelectionPanel;
extern laButtonWidget* F1_InvisibleTouchArea;
extern laButtonWidget* F2_InvisibleTouchArea;
extern laButtonWidget* F3_InvisibleTouchArea;
extern laRadioButtonWidget* F1_RadioButton;
extern laRadioButtonWidget* F2_RadioButton;
extern laRadioButtonWidget* F3_RadioButton;
extern laButtonWidget* IncButton;
extern laButtonWidget* DecButton;
extern laButtonWidget* ClrButton;
extern laButtonWidget* UnitsButtonHz;
extern laButtonWidget* UnitsButtonKHz;
extern laButtonWidget* UnitsButtondBFS;
extern laTextFieldWidget* TextBox;
extern laWidget* WindowFuncSelectionPanel;
extern laLabelWidget* WindowFuncLabel;
extern laButtonWidget* Hanning_Win_InvisibleTouchArea;
extern laButtonWidget* Blackman_Win_InvisibleTouchArea;
extern laRadioButtonWidget* HannWindow_RadioButton;
extern laRadioButtonWidget* BlackmanWindow_RadioButton;


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
