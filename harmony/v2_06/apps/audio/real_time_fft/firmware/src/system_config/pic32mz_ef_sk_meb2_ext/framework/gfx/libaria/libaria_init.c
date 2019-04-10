/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Implementation File

  File Name:
    libaria_init.c

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

#include "gfx/libaria/libaria_init.h"

laScheme LineScheme;
laScheme BasePanelScheme;
laScheme ButtonSchemeInActive;
laScheme GradientScheme;
laScheme RadioButtonScheme;
laScheme LabelScheme;
laScheme YellowTextScheme;
laScheme PanelScheme;
laScheme ButtonSchemeActive;
laScheme WhiteTextScheme;
laScheme BarScheme;
laScheme VolumeButtonScheme;
laScheme BackgroundScheme;
laGradientWidget* Background_Gradient2;
laWidget* StatusPanel;
laWidget* TimePerDivStatusPanel;
laLabelWidget* CurrentTimePerDivLabel;
laLabelWidget* CurrentTimePerDivValueLabel;
laWidget* AmpPerDivStatusPanel;
laLabelWidget* CurrentAmpPerDivLabel;
laLabelWidget* CurrentAmpPerDivValueLabel;
laWidget* ControlPanel;
laWidget* TimePerDivControlPanel;
laLabelWidget* TimeDivLabel;
laButtonWidget* TimePerDivIncButton;
laButtonWidget* TimePerDivDecButton;
laWidget* AmpPerDivControlPanel;
laLabelWidget* AmpScaleLabel;
laButtonWidget* AmpIncButton;
laButtonWidget* AmpDecButton;
laWidget* GridControlPanel;
laLabelWidget* GridLabel;
laButtonWidget* GridButton;
laDrawSurfaceWidget* GraphSurfaceWidget;
laButtonWidget* MainScreenButton_InvisibleTouchArea;
laButtonWidget* MainScreenButton;
laGradientWidget* Background_Gradient1;
laWidget* ModePanel;
laButtonWidget* MicButton;
laButtonWidget* ToneButton;
laWidget* SelectedSignalsPanel;
laLabelWidget* F1_Label;
laLabelWidget* F2_Label;
laLabelWidget* F3_Label;
laLabelWidget* PlusSign1Label;
laLabelWidget* PlusSign2Label;
laButtonWidget* TimeDomainButton_InvisibleTouchArea;
laButtonWidget* TimeDomainScreenButton;
laImageWidget* MicrochipLogo;
laWidget* FFTDisplayPanel;
laWidget* YAxisPanel;
laWidget* XAxisPanel;
laLabelWidget* dBFSMaxLabel;
laLabelWidget* dBFSMinLabel;
laLabelWidget* YAxisUnitLabel;
laLabelWidget* FreqLabel1;
laLabelWidget* FreqLabel2;
laLabelWidget* FreqLabel3;
laLabelWidget* FreqLabel4;
laLabelWidget* FreqLabel5;
laLabelWidget* FreqLabel6;
laLabelWidget* FreqLabel7;
laLabelWidget* FreqLabel8;
laLabelWidget* FreqLabel9;
laLabelWidget* FreqLabel10;
laLabelWidget* FreqLabel11;
laLabelWidget* FreqLabel12;
laLabelWidget* FreqLabel13;
laLabelWidget* FreqLabel14;
laLabelWidget* FreqLabel15;
laLabelWidget* FreqLabel16;
laLabelWidget* FreqLabel17;
laLabelWidget* FreqLabel18;
laLabelWidget* FreqLabel19;
laLabelWidget* FreqLabel20;
laLabelWidget* FreqLabel21;
laLabelWidget* FreqLabel22;
laLabelWidget* FreqLabel23;
laLabelWidget* FreqLabel24;
laLabelWidget* XAxisUnitLabel;
laProgressBarWidget* ProgressBarWidget1;
laProgressBarWidget* ProgressBarWidget2;
laProgressBarWidget* ProgressBarWidget3;
laProgressBarWidget* ProgressBarWidget4;
laProgressBarWidget* ProgressBarWidget5;
laProgressBarWidget* ProgressBarWidget6;
laProgressBarWidget* ProgressBarWidget7;
laProgressBarWidget* ProgressBarWidget8;
laProgressBarWidget* ProgressBarWidget9;
laProgressBarWidget* ProgressBarWidget10;
laProgressBarWidget* ProgressBarWidget11;
laProgressBarWidget* ProgressBarWidget12;
laProgressBarWidget* ProgressBarWidget13;
laProgressBarWidget* ProgressBarWidget14;
laProgressBarWidget* ProgressBarWidget15;
laProgressBarWidget* ProgressBarWidget16;
laProgressBarWidget* ProgressBarWidget17;
laProgressBarWidget* ProgressBarWidget18;
laProgressBarWidget* ProgressBarWidget19;
laProgressBarWidget* ProgressBarWidget20;
laProgressBarWidget* ProgressBarWidget21;
laProgressBarWidget* ProgressBarWidget22;
laProgressBarWidget* ProgressBarWidget23;
laProgressBarWidget* ProgressBarWidget24;
laLabelWidget* MessageLabel;
laWidget* VolumeSliderPanel;
laSliderWidget* VolumeControl;
laLabelWidget* VolumeLabel;
laWidget* PlayButtonPanel;
laButtonWidget* PlayButton;
laWidget* SignalSelectionPanel;
laButtonWidget* F1_InvisibleTouchArea;
laButtonWidget* F2_InvisibleTouchArea;
laButtonWidget* F3_InvisibleTouchArea;
laRadioButtonWidget* F1_RadioButton;
laRadioButtonWidget* F2_RadioButton;
laRadioButtonWidget* F3_RadioButton;
laButtonWidget* IncButton;
laButtonWidget* DecButton;
laButtonWidget* ClrButton;
laButtonWidget* UnitsButtonHz;
laButtonWidget* UnitsButtonKHz;
laButtonWidget* UnitsButtondBFS;
laTextFieldWidget* TextBox;
laWidget* WindowFuncSelectionPanel;
laLabelWidget* WindowFuncLabel;
laButtonWidget* Hanning_Win_InvisibleTouchArea;
laButtonWidget* Blackman_Win_InvisibleTouchArea;
laRadioButtonWidget* HannWindow_RadioButton;
laRadioButtonWidget* BlackmanWindow_RadioButton;


static void ScreenCreate_TimeDomainScreen(laScreen* screen);
static void ScreenCreate_MainScreen(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&LineScheme, GFX_COLOR_MODE_RGB_565);
    LineScheme.base = 0xFFE0;
    LineScheme.highlight = 0xC67A;
    LineScheme.highlightLight = 0xFFFF;
    LineScheme.shadow = 0x8410;
    LineScheme.shadowDark = 0x4208;
    LineScheme.foreground = 0x0;
    LineScheme.foregroundInactive = 0xD71C;
    LineScheme.foregroundDisabled = 0x8410;
    LineScheme.background = 0xFFFF;
    LineScheme.backgroundInactive = 0xD71C;
    LineScheme.backgroundDisabled = 0xC67A;
    LineScheme.text = 0x0;
    LineScheme.textHighlight = 0x1F;
    LineScheme.textHighlightText = 0xFFFF;
    LineScheme.textInactive = 0xD71C;
    LineScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&BasePanelScheme, GFX_COLOR_MODE_RGB_565);
    BasePanelScheme.base = 0x73AE;
    BasePanelScheme.highlight = 0xDEFB;
    BasePanelScheme.highlightLight = 0xFFFF;
    BasePanelScheme.shadow = 0xFFFF;
    BasePanelScheme.shadowDark = 0xFFFF;
    BasePanelScheme.foreground = 0x0;
    BasePanelScheme.foregroundInactive = 0xD71C;
    BasePanelScheme.foregroundDisabled = 0x9CF3;
    BasePanelScheme.background = 0x4208;
    BasePanelScheme.backgroundInactive = 0xD71C;
    BasePanelScheme.backgroundDisabled = 0x0;
    BasePanelScheme.text = 0x0;
    BasePanelScheme.textHighlight = 0x1F;
    BasePanelScheme.textHighlightText = 0xFFFF;
    BasePanelScheme.textInactive = 0xD71C;
    BasePanelScheme.textDisabled = 0x0;

    laScheme_Initialize(&ButtonSchemeInActive, GFX_COLOR_MODE_RGB_565);
    ButtonSchemeInActive.base = 0xAD96;
    ButtonSchemeInActive.highlight = 0xC67A;
    ButtonSchemeInActive.highlightLight = 0xFFFF;
    ButtonSchemeInActive.shadow = 0x8410;
    ButtonSchemeInActive.shadowDark = 0x4208;
    ButtonSchemeInActive.foreground = 0x0;
    ButtonSchemeInActive.foregroundInactive = 0xD71C;
    ButtonSchemeInActive.foregroundDisabled = 0x8410;
    ButtonSchemeInActive.background = 0xFFFF;
    ButtonSchemeInActive.backgroundInactive = 0xD71C;
    ButtonSchemeInActive.backgroundDisabled = 0xC67A;
    ButtonSchemeInActive.text = 0x0;
    ButtonSchemeInActive.textHighlight = 0x1F;
    ButtonSchemeInActive.textHighlightText = 0xFFFF;
    ButtonSchemeInActive.textInactive = 0xD71C;
    ButtonSchemeInActive.textDisabled = 0x8C92;

    laScheme_Initialize(&GradientScheme, GFX_COLOR_MODE_RGB_565);
    GradientScheme.base = 0xC67A;
    GradientScheme.highlight = 0xC67A;
    GradientScheme.highlightLight = 0xFFFF;
    GradientScheme.shadow = 0x8410;
    GradientScheme.shadowDark = 0x4208;
    GradientScheme.foreground = 0x0;
    GradientScheme.foregroundInactive = 0xD71C;
    GradientScheme.foregroundDisabled = 0x8410;
    GradientScheme.background = 0xFFFF;
    GradientScheme.backgroundInactive = 0xD71C;
    GradientScheme.backgroundDisabled = 0xC67A;
    GradientScheme.text = 0x0;
    GradientScheme.textHighlight = 0x1F;
    GradientScheme.textHighlightText = 0xFFFF;
    GradientScheme.textInactive = 0xD71C;
    GradientScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&RadioButtonScheme, GFX_COLOR_MODE_RGB_565);
    RadioButtonScheme.base = 0x4208;
    RadioButtonScheme.highlight = 0xBDF7;
    RadioButtonScheme.highlightLight = 0xFFFF;
    RadioButtonScheme.shadow = 0x8410;
    RadioButtonScheme.shadowDark = 0x4208;
    RadioButtonScheme.foreground = 0x200;
    RadioButtonScheme.foregroundInactive = 0xBDF7;
    RadioButtonScheme.foregroundDisabled = 0x8410;
    RadioButtonScheme.background = 0xFFFF;
    RadioButtonScheme.backgroundInactive = 0xD71C;
    RadioButtonScheme.backgroundDisabled = 0xC67A;
    RadioButtonScheme.text = 0xFFE0;
    RadioButtonScheme.textHighlight = 0x1F;
    RadioButtonScheme.textHighlightText = 0xFFFF;
    RadioButtonScheme.textInactive = 0xD71C;
    RadioButtonScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&LabelScheme, GFX_COLOR_MODE_RGB_565);
    LabelScheme.base = 0x4208;
    LabelScheme.highlight = 0xC67A;
    LabelScheme.highlightLight = 0xFFFF;
    LabelScheme.shadow = 0x8410;
    LabelScheme.shadowDark = 0x4208;
    LabelScheme.foreground = 0x0;
    LabelScheme.foregroundInactive = 0xD71C;
    LabelScheme.foregroundDisabled = 0x8410;
    LabelScheme.background = 0xFFFF;
    LabelScheme.backgroundInactive = 0xD71C;
    LabelScheme.backgroundDisabled = 0xC67A;
    LabelScheme.text = 0xFFFF;
    LabelScheme.textHighlight = 0x1F;
    LabelScheme.textHighlightText = 0xFFFF;
    LabelScheme.textInactive = 0xD71C;
    LabelScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&YellowTextScheme, GFX_COLOR_MODE_RGB_565);
    YellowTextScheme.base = 0x0;
    YellowTextScheme.highlight = 0xC67A;
    YellowTextScheme.highlightLight = 0xFFFF;
    YellowTextScheme.shadow = 0x8410;
    YellowTextScheme.shadowDark = 0x4208;
    YellowTextScheme.foreground = 0x0;
    YellowTextScheme.foregroundInactive = 0xD71C;
    YellowTextScheme.foregroundDisabled = 0x8410;
    YellowTextScheme.background = 0xFFFF;
    YellowTextScheme.backgroundInactive = 0xD71C;
    YellowTextScheme.backgroundDisabled = 0xC67A;
    YellowTextScheme.text = 0xFFE0;
    YellowTextScheme.textHighlight = 0x1F;
    YellowTextScheme.textHighlightText = 0xFFFF;
    YellowTextScheme.textInactive = 0xD71C;
    YellowTextScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&PanelScheme, GFX_COLOR_MODE_RGB_565);
    PanelScheme.base = 0x4208;
    PanelScheme.highlight = 0xDEFB;
    PanelScheme.highlightLight = 0xFFFF;
    PanelScheme.shadow = 0xFFFF;
    PanelScheme.shadowDark = 0xFFFF;
    PanelScheme.foreground = 0x0;
    PanelScheme.foregroundInactive = 0xD71C;
    PanelScheme.foregroundDisabled = 0x8410;
    PanelScheme.background = 0x4208;
    PanelScheme.backgroundInactive = 0xD71C;
    PanelScheme.backgroundDisabled = 0x0;
    PanelScheme.text = 0x0;
    PanelScheme.textHighlight = 0x1F;
    PanelScheme.textHighlightText = 0xFFFF;
    PanelScheme.textInactive = 0xD71C;
    PanelScheme.textDisabled = 0x0;

    laScheme_Initialize(&ButtonSchemeActive, GFX_COLOR_MODE_RGB_565);
    ButtonSchemeActive.base = 0x3A0;
    ButtonSchemeActive.highlight = 0x4208;
    ButtonSchemeActive.highlightLight = 0xBDF7;
    ButtonSchemeActive.shadow = 0x8410;
    ButtonSchemeActive.shadowDark = 0x4208;
    ButtonSchemeActive.foreground = 0x0;
    ButtonSchemeActive.foregroundInactive = 0xD71C;
    ButtonSchemeActive.foregroundDisabled = 0x8410;
    ButtonSchemeActive.background = 0xFFFF;
    ButtonSchemeActive.backgroundInactive = 0xD71C;
    ButtonSchemeActive.backgroundDisabled = 0xC67A;
    ButtonSchemeActive.text = 0x0;
    ButtonSchemeActive.textHighlight = 0x1F;
    ButtonSchemeActive.textHighlightText = 0xFFFF;
    ButtonSchemeActive.textInactive = 0xD71C;
    ButtonSchemeActive.textDisabled = 0x8C92;

    laScheme_Initialize(&WhiteTextScheme, GFX_COLOR_MODE_RGB_565);
    WhiteTextScheme.base = 0x0;
    WhiteTextScheme.highlight = 0xC67A;
    WhiteTextScheme.highlightLight = 0xFFFF;
    WhiteTextScheme.shadow = 0x8410;
    WhiteTextScheme.shadowDark = 0x4208;
    WhiteTextScheme.foreground = 0x0;
    WhiteTextScheme.foregroundInactive = 0xD71C;
    WhiteTextScheme.foregroundDisabled = 0x8410;
    WhiteTextScheme.background = 0xFFFF;
    WhiteTextScheme.backgroundInactive = 0xD71C;
    WhiteTextScheme.backgroundDisabled = 0xC67A;
    WhiteTextScheme.text = 0xFFFF;
    WhiteTextScheme.textHighlight = 0x1F;
    WhiteTextScheme.textHighlightText = 0xFFFF;
    WhiteTextScheme.textInactive = 0xD71C;
    WhiteTextScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&BarScheme, GFX_COLOR_MODE_RGB_565);
    BarScheme.base = 0xC67A;
    BarScheme.highlight = 0x5F7;
    BarScheme.highlightLight = 0xFFFF;
    BarScheme.shadow = 0x8410;
    BarScheme.shadowDark = 0x4208;
    BarScheme.foreground = 0x0;
    BarScheme.foregroundInactive = 0xD71C;
    BarScheme.foregroundDisabled = 0x8410;
    BarScheme.background = 0x0;
    BarScheme.backgroundInactive = 0xD71C;
    BarScheme.backgroundDisabled = 0xC67A;
    BarScheme.text = 0x0;
    BarScheme.textHighlight = 0x5F7;
    BarScheme.textHighlightText = 0xFFFF;
    BarScheme.textInactive = 0xD71C;
    BarScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&VolumeButtonScheme, GFX_COLOR_MODE_RGB_565);
    VolumeButtonScheme.base = 0x4208;
    VolumeButtonScheme.highlight = 0xDEFB;
    VolumeButtonScheme.highlightLight = 0xFFFF;
    VolumeButtonScheme.shadow = 0xFFFF;
    VolumeButtonScheme.shadowDark = 0xFFFF;
    VolumeButtonScheme.foreground = 0x0;
    VolumeButtonScheme.foregroundInactive = 0xD71C;
    VolumeButtonScheme.foregroundDisabled = 0x8410;
    VolumeButtonScheme.background = 0x8000;
    VolumeButtonScheme.backgroundInactive = 0x0;
    VolumeButtonScheme.backgroundDisabled = 0x0;
    VolumeButtonScheme.text = 0x0;
    VolumeButtonScheme.textHighlight = 0x1F;
    VolumeButtonScheme.textHighlightText = 0xFFFF;
    VolumeButtonScheme.textInactive = 0xD71C;
    VolumeButtonScheme.textDisabled = 0x0;

    laScheme_Initialize(&BackgroundScheme, GFX_COLOR_MODE_RGB_565);
    BackgroundScheme.base = 0x0;
    BackgroundScheme.highlight = 0xFFFF;
    BackgroundScheme.highlightLight = 0xDEFB;
    BackgroundScheme.shadow = 0xFFFF;
    BackgroundScheme.shadowDark = 0xDEFB;
    BackgroundScheme.foreground = 0x0;
    BackgroundScheme.foregroundInactive = 0x0;
    BackgroundScheme.foregroundDisabled = 0x0;
    BackgroundScheme.background = 0x0;
    BackgroundScheme.backgroundInactive = 0xD71C;
    BackgroundScheme.backgroundDisabled = 0x0;
    BackgroundScheme.text = 0x0;
    BackgroundScheme.textHighlight = 0x0;
    BackgroundScheme.textHighlightText = 0x0;
    BackgroundScheme.textInactive = 0x0;
    BackgroundScheme.textDisabled = 0x0;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_TRUE, LA_TRUE, &ScreenCreate_TimeDomainScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_TRUE, LA_FALSE, &ScreenCreate_MainScreen);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(1);

	return 0;
}

static void ScreenCreate_TimeDomainScreen(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_NONE);
    laLayer_SetBufferCount(layer0, 2);

    laScreen_SetLayer(screen, 0, layer0);

    Background_Gradient2 = laGradientWidget_New();
    laWidget_SetSize((laWidget*)Background_Gradient2, 480, 272);
    laWidget_SetScheme((laWidget*)Background_Gradient2, &BackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)Background_Gradient2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)Background_Gradient2, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Background_Gradient2);

    StatusPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)StatusPanel, 2, 226);
    laWidget_SetSize((laWidget*)StatusPanel, 384, 43);
    laWidget_SetScheme((laWidget*)StatusPanel, &BasePanelScheme);
    laWidget_SetBackgroundType((laWidget*)StatusPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)StatusPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, StatusPanel);

    TimePerDivStatusPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)TimePerDivStatusPanel, 2, 2);
    laWidget_SetSize((laWidget*)TimePerDivStatusPanel, 100, 40);
    laWidget_SetScheme((laWidget*)TimePerDivStatusPanel, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)TimePerDivStatusPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TimePerDivStatusPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)StatusPanel, TimePerDivStatusPanel);

    CurrentTimePerDivLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CurrentTimePerDivLabel, 9, 2);
    laWidget_SetSize((laWidget*)CurrentTimePerDivLabel, 81, 13);
    laWidget_SetScheme((laWidget*)CurrentTimePerDivLabel, &LabelScheme);
    laWidget_SetBackgroundType((laWidget*)CurrentTimePerDivLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CurrentTimePerDivLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CurrentTimePerDivLabel, laString_CreateFromID(string_TimePerDiv));
    laWidget_AddChild((laWidget*)TimePerDivStatusPanel, (laWidget*)CurrentTimePerDivLabel);

    CurrentTimePerDivValueLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CurrentTimePerDivValueLabel, 4, 20);
    laWidget_SetSize((laWidget*)CurrentTimePerDivValueLabel, 92, 18);
    laWidget_SetScheme((laWidget*)CurrentTimePerDivValueLabel, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)CurrentTimePerDivValueLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CurrentTimePerDivValueLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CurrentTimePerDivValueLabel, laString_CreateFromID(string_TimePerDivValue));
    laWidget_AddChild((laWidget*)TimePerDivStatusPanel, (laWidget*)CurrentTimePerDivValueLabel);

    AmpPerDivStatusPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)AmpPerDivStatusPanel, 105, 2);
    laWidget_SetSize((laWidget*)AmpPerDivStatusPanel, 100, 40);
    laWidget_SetScheme((laWidget*)AmpPerDivStatusPanel, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)AmpPerDivStatusPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)AmpPerDivStatusPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)StatusPanel, AmpPerDivStatusPanel);

    CurrentAmpPerDivLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CurrentAmpPerDivLabel, 5, 0);
    laWidget_SetSize((laWidget*)CurrentAmpPerDivLabel, 91, 18);
    laWidget_SetScheme((laWidget*)CurrentAmpPerDivLabel, &LabelScheme);
    laWidget_SetBackgroundType((laWidget*)CurrentAmpPerDivLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CurrentAmpPerDivLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CurrentAmpPerDivLabel, laString_CreateFromID(string_AmpScale));
    laWidget_AddChild((laWidget*)AmpPerDivStatusPanel, (laWidget*)CurrentAmpPerDivLabel);

    CurrentAmpPerDivValueLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CurrentAmpPerDivValueLabel, 4, 20);
    laWidget_SetSize((laWidget*)CurrentAmpPerDivValueLabel, 92, 18);
    laWidget_SetScheme((laWidget*)CurrentAmpPerDivValueLabel, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)CurrentAmpPerDivValueLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CurrentAmpPerDivValueLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CurrentAmpPerDivValueLabel, laString_CreateFromID(string_AmpScaleValue));
    laWidget_AddChild((laWidget*)AmpPerDivStatusPanel, (laWidget*)CurrentAmpPerDivValueLabel);

    ControlPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)ControlPanel, 390, 0);
    laWidget_SetSize((laWidget*)ControlPanel, 87, 224);
    laWidget_SetScheme((laWidget*)ControlPanel, &BasePanelScheme);
    laWidget_SetBackgroundType((laWidget*)ControlPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ControlPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, ControlPanel);

    TimePerDivControlPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)TimePerDivControlPanel, 0, 5);
    laWidget_SetSize((laWidget*)TimePerDivControlPanel, 87, 70);
    laWidget_SetScheme((laWidget*)TimePerDivControlPanel, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)TimePerDivControlPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TimePerDivControlPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)ControlPanel, TimePerDivControlPanel);

    TimeDivLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TimeDivLabel, 12, 2);
    laWidget_SetSize((laWidget*)TimeDivLabel, 65, 22);
    laWidget_SetScheme((laWidget*)TimeDivLabel, &LabelScheme);
    laWidget_SetBackgroundType((laWidget*)TimeDivLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TimeDivLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TimeDivLabel, laString_CreateFromID(string_TimePerDiv));
    laWidget_AddChild((laWidget*)TimePerDivControlPanel, (laWidget*)TimeDivLabel);

    TimePerDivIncButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)TimePerDivIncButton, 2, 26);
    laWidget_SetSize((laWidget*)TimePerDivIncButton, 41, 39);
    laWidget_SetBackgroundType((laWidget*)TimePerDivIncButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TimePerDivIncButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(TimePerDivIncButton, laString_CreateFromID(string_Inc));
    laButtonWidget_SetReleasedEventCallback(TimePerDivIncButton, &TimePerDivIncButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)TimePerDivControlPanel, (laWidget*)TimePerDivIncButton);

    TimePerDivDecButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)TimePerDivDecButton, 46, 26);
    laWidget_SetSize((laWidget*)TimePerDivDecButton, 41, 38);
    laWidget_SetBackgroundType((laWidget*)TimePerDivDecButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TimePerDivDecButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(TimePerDivDecButton, laString_CreateFromID(string_Dec));
    laButtonWidget_SetReleasedEventCallback(TimePerDivDecButton, &TimePerDivDecButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)TimePerDivControlPanel, (laWidget*)TimePerDivDecButton);

    AmpPerDivControlPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)AmpPerDivControlPanel, 0, 78);
    laWidget_SetSize((laWidget*)AmpPerDivControlPanel, 87, 70);
    laWidget_SetScheme((laWidget*)AmpPerDivControlPanel, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)AmpPerDivControlPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)AmpPerDivControlPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)ControlPanel, AmpPerDivControlPanel);

    AmpScaleLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)AmpScaleLabel, 2, 1);
    laWidget_SetSize((laWidget*)AmpScaleLabel, 83, 22);
    laWidget_SetScheme((laWidget*)AmpScaleLabel, &LabelScheme);
    laWidget_SetBackgroundType((laWidget*)AmpScaleLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)AmpScaleLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(AmpScaleLabel, laString_CreateFromID(string_AmpScale));
    laWidget_AddChild((laWidget*)AmpPerDivControlPanel, (laWidget*)AmpScaleLabel);

    AmpIncButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)AmpIncButton, 1, 26);
    laWidget_SetSize((laWidget*)AmpIncButton, 41, 39);
    laWidget_SetBackgroundType((laWidget*)AmpIncButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)AmpIncButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(AmpIncButton, laString_CreateFromID(string_Inc));
    laButtonWidget_SetReleasedEventCallback(AmpIncButton, &AmpIncButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)AmpPerDivControlPanel, (laWidget*)AmpIncButton);

    AmpDecButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)AmpDecButton, 46, 26);
    laWidget_SetSize((laWidget*)AmpDecButton, 41, 39);
    laWidget_SetBackgroundType((laWidget*)AmpDecButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)AmpDecButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(AmpDecButton, laString_CreateFromID(string_Dec));
    laButtonWidget_SetReleasedEventCallback(AmpDecButton, &AmpDecButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)AmpPerDivControlPanel, (laWidget*)AmpDecButton);

    GridControlPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)GridControlPanel, 0, 152);
    laWidget_SetSize((laWidget*)GridControlPanel, 87, 70);
    laWidget_SetScheme((laWidget*)GridControlPanel, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)GridControlPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GridControlPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)ControlPanel, GridControlPanel);

    GridLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GridLabel, 3, 4);
    laWidget_SetSize((laWidget*)GridLabel, 82, 19);
    laWidget_SetScheme((laWidget*)GridLabel, &LabelScheme);
    laWidget_SetBackgroundType((laWidget*)GridLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GridLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GridLabel, laString_CreateFromID(string_Grid));
    laWidget_AddChild((laWidget*)GridControlPanel, (laWidget*)GridLabel);

    GridButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)GridButton, 25, 25);
    laWidget_SetSize((laWidget*)GridButton, 41, 39);
    laWidget_SetBackgroundType((laWidget*)GridButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GridButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(GridButton, laString_CreateFromID(string_OFF));
    laButtonWidget_SetReleasedEventCallback(GridButton, &GridButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)GridControlPanel, (laWidget*)GridButton);

    GraphSurfaceWidget = laDrawSurfaceWidget_New();
    laWidget_SetPosition((laWidget*)GraphSurfaceWidget, 2, 2);
    laWidget_SetSize((laWidget*)GraphSurfaceWidget, 384, 221);
    laWidget_SetScheme((laWidget*)GraphSurfaceWidget, &BackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)GraphSurfaceWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GraphSurfaceWidget, LA_WIDGET_BORDER_BEVEL);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GraphSurfaceWidget);

    MainScreenButton_InvisibleTouchArea = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)MainScreenButton_InvisibleTouchArea, 388, 227);
    laWidget_SetSize((laWidget*)MainScreenButton_InvisibleTouchArea, 90, 42);
    laWidget_SetScheme((laWidget*)MainScreenButton_InvisibleTouchArea, &BackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)MainScreenButton_InvisibleTouchArea, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)MainScreenButton_InvisibleTouchArea, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(MainScreenButton_InvisibleTouchArea, &MainScreenButton_InvisibleTouchArea_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)MainScreenButton_InvisibleTouchArea);

    MainScreenButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)MainScreenButton, 439, 241);
    laWidget_SetSize((laWidget*)MainScreenButton, 40, 30);
    laWidget_SetEnabled((laWidget*)MainScreenButton, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)MainScreenButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)MainScreenButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetReleasedImage(MainScreenButton, &Freq_Domain_Signal);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)MainScreenButton);

}

static void ScreenCreate_MainScreen(laScreen* screen)
{
    laLayer* layer0;
    laRadioButtonGroup* radioButtonGroup_1;
    laRadioButtonGroup* radioButtonGroup_2;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_NONE);
    laLayer_SetBufferCount(layer0, 2);

    laScreen_SetLayer(screen, 0, layer0);

    Background_Gradient1 = laGradientWidget_New();
    laWidget_SetSize((laWidget*)Background_Gradient1, 480, 272);
    laWidget_SetScheme((laWidget*)Background_Gradient1, &BackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)Background_Gradient1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)Background_Gradient1, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Background_Gradient1);

    ModePanel = laWidget_New();
    laWidget_SetSize((laWidget*)ModePanel, 109, 51);
    laWidget_SetScheme((laWidget*)ModePanel, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)ModePanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ModePanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, ModePanel);

    MicButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)MicButton, 5, 5);
    laWidget_SetSize((laWidget*)MicButton, 47, 44);
    laWidget_SetScheme((laWidget*)MicButton, &ButtonSchemeActive);
    laWidget_SetBackgroundType((laWidget*)MicButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)MicButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(MicButton, laString_CreateFromID(string_Mic));
    laButtonWidget_SetReleasedEventCallback(MicButton, &MicButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)ModePanel, (laWidget*)MicButton);

    ToneButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ToneButton, 59, 5);
    laWidget_SetSize((laWidget*)ToneButton, 47, 44);
    laWidget_SetScheme((laWidget*)ToneButton, &ButtonSchemeInActive);
    laWidget_SetBackgroundType((laWidget*)ToneButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ToneButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(ToneButton, laString_CreateFromID(string_Tone));
    laButtonWidget_SetReleasedEventCallback(ToneButton, &ToneButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)ModePanel, (laWidget*)ToneButton);

    SelectedSignalsPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)SelectedSignalsPanel, 0, 54);
    laWidget_SetSize((laWidget*)SelectedSignalsPanel, 109, 41);
    laWidget_SetScheme((laWidget*)SelectedSignalsPanel, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)SelectedSignalsPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)SelectedSignalsPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, SelectedSignalsPanel);

    F1_Label = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)F1_Label, 2, 8);
    laWidget_SetSize((laWidget*)F1_Label, 24, 24);
    laWidget_SetScheme((laWidget*)F1_Label, &ButtonSchemeInActive);
    laWidget_SetBackgroundType((laWidget*)F1_Label, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)F1_Label, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(F1_Label, laString_CreateFromID(string_F1));
    laWidget_AddChild((laWidget*)SelectedSignalsPanel, (laWidget*)F1_Label);

    F2_Label = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)F2_Label, 43, 8);
    laWidget_SetSize((laWidget*)F2_Label, 23, 24);
    laWidget_SetScheme((laWidget*)F2_Label, &ButtonSchemeInActive);
    laWidget_SetBackgroundType((laWidget*)F2_Label, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)F2_Label, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(F2_Label, laString_CreateFromID(string_F2));
    laWidget_AddChild((laWidget*)SelectedSignalsPanel, (laWidget*)F2_Label);

    F3_Label = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)F3_Label, 82, 8);
    laWidget_SetSize((laWidget*)F3_Label, 23, 24);
    laWidget_SetScheme((laWidget*)F3_Label, &ButtonSchemeInActive);
    laWidget_SetBackgroundType((laWidget*)F3_Label, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)F3_Label, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(F3_Label, laString_CreateFromID(string_F3));
    laWidget_AddChild((laWidget*)SelectedSignalsPanel, (laWidget*)F3_Label);

    PlusSign1Label = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)PlusSign1Label, 26, 12);
    laWidget_SetSize((laWidget*)PlusSign1Label, 16, 16);
    laWidget_SetScheme((laWidget*)PlusSign1Label, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)PlusSign1Label, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PlusSign1Label, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(PlusSign1Label, laString_CreateFromID(string_Inc));
    laWidget_AddChild((laWidget*)SelectedSignalsPanel, (laWidget*)PlusSign1Label);

    PlusSign2Label = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)PlusSign2Label, 66, 12);
    laWidget_SetSize((laWidget*)PlusSign2Label, 16, 16);
    laWidget_SetScheme((laWidget*)PlusSign2Label, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)PlusSign2Label, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PlusSign2Label, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(PlusSign2Label, laString_CreateFromID(string_Inc));
    laWidget_AddChild((laWidget*)SelectedSignalsPanel, (laWidget*)PlusSign2Label);

    TimeDomainButton_InvisibleTouchArea = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)TimeDomainButton_InvisibleTouchArea, 410, 218);
    laWidget_SetSize((laWidget*)TimeDomainButton_InvisibleTouchArea, 68, 52);
    laWidget_SetScheme((laWidget*)TimeDomainButton_InvisibleTouchArea, &BackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)TimeDomainButton_InvisibleTouchArea, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TimeDomainButton_InvisibleTouchArea, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(TimeDomainButton_InvisibleTouchArea, &TimeDomainButton_InvisibleTouchArea_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)TimeDomainButton_InvisibleTouchArea);

    TimeDomainScreenButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)TimeDomainScreenButton, 439, 241);
    laWidget_SetSize((laWidget*)TimeDomainScreenButton, 40, 30);
    laWidget_SetEnabled((laWidget*)TimeDomainScreenButton, LA_FALSE);
    laWidget_SetScheme((laWidget*)TimeDomainScreenButton, &ButtonSchemeInActive);
    laWidget_SetBackgroundType((laWidget*)TimeDomainScreenButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TimeDomainScreenButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetReleasedImage(TimeDomainScreenButton, &Time_Domain_Signal);
    laButtonWidget_SetImageMargin(TimeDomainScreenButton, 0);
    laButtonWidget_SetPressedOffset(TimeDomainScreenButton, 0);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)TimeDomainScreenButton);

    MicrochipLogo = laImageWidget_New();
    laWidget_SetPosition((laWidget*)MicrochipLogo, 451, 107);
    laWidget_SetSize((laWidget*)MicrochipLogo, 28, 122);
    laWidget_SetBackgroundType((laWidget*)MicrochipLogo, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)MicrochipLogo, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(MicrochipLogo, &MCHP_LOGO);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)MicrochipLogo);

    FFTDisplayPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)FFTDisplayPanel, 3, 97);
    laWidget_SetSize((laWidget*)FFTDisplayPanel, 434, 173);
    laWidget_SetScheme((laWidget*)FFTDisplayPanel, &BackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)FFTDisplayPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FFTDisplayPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, FFTDisplayPanel);

    YAxisPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)YAxisPanel, 36, 13);
    laWidget_SetSize((laWidget*)YAxisPanel, 3, 130);
    laWidget_SetScheme((laWidget*)YAxisPanel, &LineScheme);
    laWidget_SetBackgroundType((laWidget*)YAxisPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)YAxisPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, YAxisPanel);

    XAxisPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)XAxisPanel, 36, 141);
    laWidget_SetSize((laWidget*)XAxisPanel, 370, 3);
    laWidget_SetScheme((laWidget*)XAxisPanel, &LineScheme);
    laWidget_SetBackgroundType((laWidget*)XAxisPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)XAxisPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, XAxisPanel);

    dBFSMaxLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)dBFSMaxLabel, 24, 10);
    laWidget_SetSize((laWidget*)dBFSMaxLabel, 11, 14);
    laWidget_SetScheme((laWidget*)dBFSMaxLabel, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)dBFSMaxLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)dBFSMaxLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(dBFSMaxLabel, laString_CreateFromID(string_dBFS_Max));
    laLabelWidget_SetHAlignment(dBFSMaxLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)dBFSMaxLabel);

    dBFSMinLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)dBFSMinLabel, 12, 134);
    laWidget_SetSize((laWidget*)dBFSMinLabel, 24, 13);
    laWidget_SetScheme((laWidget*)dBFSMinLabel, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)dBFSMinLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)dBFSMinLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(dBFSMinLabel, laString_CreateFromID(string_dBFS_Min));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)dBFSMinLabel);

    YAxisUnitLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)YAxisUnitLabel, 2, 68);
    laWidget_SetSize((laWidget*)YAxisUnitLabel, 34, 17);
    laWidget_SetScheme((laWidget*)YAxisUnitLabel, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)YAxisUnitLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)YAxisUnitLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(YAxisUnitLabel, laString_CreateFromID(string_dBFS));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)YAxisUnitLabel);

    FreqLabel1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel1, 39, 146);
    laWidget_SetSize((laWidget*)FreqLabel1, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel1, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel1, laString_CreateFromID(string_Band1));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel1);

    FreqLabel2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel2, 54, 159);
    laWidget_SetSize((laWidget*)FreqLabel2, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel2, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel2, laString_CreateFromID(string_Band2));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel2);

    FreqLabel3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel3, 69, 146);
    laWidget_SetSize((laWidget*)FreqLabel3, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel3, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel3, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel3, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel3, laString_CreateFromID(string_Band3));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel3);

    FreqLabel4 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel4, 84, 159);
    laWidget_SetSize((laWidget*)FreqLabel4, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel4, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel4, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel4, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel4, laString_CreateFromID(string_Band4));
    laLabelWidget_SetHAlignment(FreqLabel4, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel4);

    FreqLabel5 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel5, 99, 146);
    laWidget_SetSize((laWidget*)FreqLabel5, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel5, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel5, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel5, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel5, laString_CreateFromID(string_Band5));
    laLabelWidget_SetHAlignment(FreqLabel5, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel5);

    FreqLabel6 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel6, 114, 159);
    laWidget_SetSize((laWidget*)FreqLabel6, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel6, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel6, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel6, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel6, laString_CreateFromID(string_Band6));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel6);

    FreqLabel7 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel7, 129, 146);
    laWidget_SetSize((laWidget*)FreqLabel7, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel7, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel7, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel7, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel7, laString_CreateFromID(string_Band7));
    laLabelWidget_SetHAlignment(FreqLabel7, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel7);

    FreqLabel8 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel8, 144, 159);
    laWidget_SetSize((laWidget*)FreqLabel8, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel8, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel8, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel8, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel8, laString_CreateFromID(string_Band8));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel8);

    FreqLabel9 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel9, 159, 146);
    laWidget_SetSize((laWidget*)FreqLabel9, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel9, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel9, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel9, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel9, laString_CreateFromID(string_Band9));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel9);

    FreqLabel10 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel10, 174, 159);
    laWidget_SetSize((laWidget*)FreqLabel10, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel10, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel10, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel10, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel10, laString_CreateFromID(string_Band10));
    laLabelWidget_SetHAlignment(FreqLabel10, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel10);

    FreqLabel11 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel11, 189, 146);
    laWidget_SetSize((laWidget*)FreqLabel11, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel11, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel11, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel11, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel11, laString_CreateFromID(string_Band11));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel11);

    FreqLabel12 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel12, 204, 159);
    laWidget_SetSize((laWidget*)FreqLabel12, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel12, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel12, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel12, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel12, laString_CreateFromID(string_Band12));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel12);

    FreqLabel13 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel13, 219, 146);
    laWidget_SetSize((laWidget*)FreqLabel13, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel13, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel13, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel13, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel13, laString_CreateFromID(string_Band13));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel13);

    FreqLabel14 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel14, 234, 159);
    laWidget_SetSize((laWidget*)FreqLabel14, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel14, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel14, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel14, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel14, laString_CreateFromID(string_Band14));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel14);

    FreqLabel15 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel15, 249, 146);
    laWidget_SetSize((laWidget*)FreqLabel15, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel15, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel15, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel15, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel15, laString_CreateFromID(string_Band15));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel15);

    FreqLabel16 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel16, 264, 159);
    laWidget_SetSize((laWidget*)FreqLabel16, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel16, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel16, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel16, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel16, laString_CreateFromID(string_Band16));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel16);

    FreqLabel17 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel17, 280, 146);
    laWidget_SetSize((laWidget*)FreqLabel17, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel17, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel17, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel17, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel17, laString_CreateFromID(string_Band17));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel17);

    FreqLabel18 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel18, 294, 159);
    laWidget_SetSize((laWidget*)FreqLabel18, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel18, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel18, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel18, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel18, laString_CreateFromID(string_Band18));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel18);

    FreqLabel19 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel19, 310, 146);
    laWidget_SetSize((laWidget*)FreqLabel19, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel19, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel19, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel19, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel19, laString_CreateFromID(string_Band19));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel19);

    FreqLabel20 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel20, 324, 159);
    laWidget_SetSize((laWidget*)FreqLabel20, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel20, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel20, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel20, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel20, laString_CreateFromID(string_Band20));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel20);

    FreqLabel21 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel21, 336, 146);
    laWidget_SetSize((laWidget*)FreqLabel21, 31, 10);
    laWidget_SetScheme((laWidget*)FreqLabel21, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel21, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel21, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel21, laString_CreateFromID(string_Band21));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel21);

    FreqLabel22 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel22, 350, 159);
    laWidget_SetSize((laWidget*)FreqLabel22, 30, 10);
    laWidget_SetScheme((laWidget*)FreqLabel22, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel22, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel22, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel22, laString_CreateFromID(string_Band22));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel22);

    FreqLabel23 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel23, 367, 146);
    laWidget_SetSize((laWidget*)FreqLabel23, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel23, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel23, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel23, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel23, laString_CreateFromID(string_Band23));
    laLabelWidget_SetHAlignment(FreqLabel23, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel23);

    FreqLabel24 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FreqLabel24, 384, 159);
    laWidget_SetSize((laWidget*)FreqLabel24, 22, 10);
    laWidget_SetScheme((laWidget*)FreqLabel24, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)FreqLabel24, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FreqLabel24, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FreqLabel24, laString_CreateFromID(string_Band24));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)FreqLabel24);

    XAxisUnitLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)XAxisUnitLabel, 411, 151);
    laWidget_SetSize((laWidget*)XAxisUnitLabel, 24, 12);
    laWidget_SetScheme((laWidget*)XAxisUnitLabel, &YellowTextScheme);
    laWidget_SetBackgroundType((laWidget*)XAxisUnitLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)XAxisUnitLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(XAxisUnitLabel, laString_CreateFromID(string_Hz));
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)XAxisUnitLabel);

    ProgressBarWidget1 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget1, 45, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget1, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget1, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget1, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget1, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget1, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget1);

    ProgressBarWidget2 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget2, 60, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget2, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget2, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget2, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget2, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget2, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget2);

    ProgressBarWidget3 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget3, 75, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget3, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget3, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget3, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget3, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget3, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget3);

    ProgressBarWidget4 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget4, 90, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget4, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget4, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget4, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget4, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget4, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget4);

    ProgressBarWidget5 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget5, 105, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget5, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget5, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget5, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget5, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget5, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget5, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget5);

    ProgressBarWidget6 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget6, 120, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget6, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget6, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget6, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget6, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget6, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget6, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget6);

    ProgressBarWidget7 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget7, 135, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget7, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget7, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget7, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget7, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget7, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget7, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget7);

    ProgressBarWidget8 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget8, 150, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget8, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget8, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget8, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget8, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget8, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget8, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget8);

    ProgressBarWidget9 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget9, 165, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget9, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget9, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget9, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget9, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget9, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget9, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget9);

    ProgressBarWidget10 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget10, 180, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget10, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget10, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget10, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget10, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget10, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget10, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget10);

    ProgressBarWidget11 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget11, 195, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget11, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget11, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget11, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget11, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget11, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget11, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget11);

    ProgressBarWidget12 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget12, 210, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget12, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget12, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget12, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget12, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget12, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget12, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget12);

    ProgressBarWidget13 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget13, 225, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget13, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget13, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget13, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget13, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget13, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget13, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget13);

    ProgressBarWidget14 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget14, 240, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget14, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget14, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget14, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget14, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget14, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget14, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget14);

    ProgressBarWidget15 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget15, 255, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget15, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget15, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget15, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget15, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget15, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget15, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget15);

    ProgressBarWidget16 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget16, 270, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget16, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget16, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget16, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget16, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget16, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget16, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget16);

    ProgressBarWidget17 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget17, 285, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget17, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget17, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget17, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget17, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget17, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget17, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget17);

    ProgressBarWidget18 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget18, 300, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget18, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget18, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget18, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget18, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget18, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget18, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget18);

    ProgressBarWidget19 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget19, 315, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget19, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget19, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget19, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget19, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget19, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget19, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget19);

    ProgressBarWidget20 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget20, 330, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget20, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget20, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget20, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget20, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget20, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget20, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget20);

    ProgressBarWidget21 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget21, 345, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget21, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget21, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget21, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget21, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget21, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget21, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget21);

    ProgressBarWidget22 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget22, 360, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget22, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget22, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget22, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget22, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget22, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget22, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget22);

    ProgressBarWidget23 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget23, 375, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget23, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget23, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget23, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget23, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget23, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget23, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget23);

    ProgressBarWidget24 = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget24, 390, 13);
    laWidget_SetSize((laWidget*)ProgressBarWidget24, 10, 125);
    laWidget_SetScheme((laWidget*)ProgressBarWidget24, &BarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget24, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget24, LA_WIDGET_BORDER_NONE);
    laProgressBarWidget_SetDirection(ProgressBarWidget24, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(ProgressBarWidget24, 1);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)ProgressBarWidget24);

    MessageLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)MessageLabel, 2, -1);
    laWidget_SetSize((laWidget*)MessageLabel, 400, 13);
    laWidget_SetScheme((laWidget*)MessageLabel, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)MessageLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)MessageLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(MessageLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)FFTDisplayPanel, (laWidget*)MessageLabel);

    VolumeSliderPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)VolumeSliderPanel, 424, 0);
    laWidget_SetSize((laWidget*)VolumeSliderPanel, 49, 95);
    laWidget_SetScheme((laWidget*)VolumeSliderPanel, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)VolumeSliderPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)VolumeSliderPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, VolumeSliderPanel);

    VolumeControl = laSliderWidget_New();
    laWidget_SetPosition((laWidget*)VolumeControl, 7, 2);
    laWidget_SetSize((laWidget*)VolumeControl, 36, 76);
    laWidget_SetScheme((laWidget*)VolumeControl, &VolumeButtonScheme);
    laWidget_SetBackgroundType((laWidget*)VolumeControl, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)VolumeControl, LA_WIDGET_BORDER_BEVEL);
    laSliderWidget_SetSliderValue(VolumeControl, 40);
    laSliderWidget_SetValueChangedEventCallback(VolumeControl, &VolumeControl_ValueChangedEvent);

    laWidget_AddChild((laWidget*)VolumeSliderPanel, (laWidget*)VolumeControl);

    VolumeLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)VolumeLabel, 3, 78);
    laWidget_SetSize((laWidget*)VolumeLabel, 45, 15);
    laWidget_SetScheme((laWidget*)VolumeLabel, &LabelScheme);
    laWidget_SetBackgroundType((laWidget*)VolumeLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)VolumeLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(VolumeLabel, laString_CreateFromID(string_Vol));
    laWidget_AddChild((laWidget*)VolumeSliderPanel, (laWidget*)VolumeLabel);

    PlayButtonPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)PlayButtonPanel, 333, 0);
    laWidget_SetSize((laWidget*)PlayButtonPanel, 89, 35);
    laWidget_SetScheme((laWidget*)PlayButtonPanel, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)PlayButtonPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PlayButtonPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PlayButtonPanel);

    PlayButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)PlayButton, 0, 1);
    laWidget_SetSize((laWidget*)PlayButton, 89, 34);
    laWidget_SetScheme((laWidget*)PlayButton, &ButtonSchemeInActive);
    laWidget_SetBackgroundType((laWidget*)PlayButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PlayButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(PlayButton, laString_CreateFromID(string_Play_Start));
    laButtonWidget_SetReleasedEventCallback(PlayButton, &PlayButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)PlayButtonPanel, (laWidget*)PlayButton);

    SignalSelectionPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)SignalSelectionPanel, 111, 0);
    laWidget_SetSize((laWidget*)SignalSelectionPanel, 219, 95);
    laWidget_SetScheme((laWidget*)SignalSelectionPanel, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)SignalSelectionPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)SignalSelectionPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, SignalSelectionPanel);

    F1_InvisibleTouchArea = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)F1_InvisibleTouchArea, 1, 1);
    laWidget_SetSize((laWidget*)F1_InvisibleTouchArea, 52, 31);
    laWidget_SetScheme((laWidget*)F1_InvisibleTouchArea, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)F1_InvisibleTouchArea, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)F1_InvisibleTouchArea, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(F1_InvisibleTouchArea, &F1_InvisibleTouchArea_ReleasedEvent);

    laWidget_AddChild((laWidget*)SignalSelectionPanel, (laWidget*)F1_InvisibleTouchArea);

    F2_InvisibleTouchArea = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)F2_InvisibleTouchArea, 1, 34);
    laWidget_SetSize((laWidget*)F2_InvisibleTouchArea, 52, 31);
    laWidget_SetScheme((laWidget*)F2_InvisibleTouchArea, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)F2_InvisibleTouchArea, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)F2_InvisibleTouchArea, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(F2_InvisibleTouchArea, &F2_InvisibleTouchArea_ReleasedEvent);

    laWidget_AddChild((laWidget*)SignalSelectionPanel, (laWidget*)F2_InvisibleTouchArea);

    F3_InvisibleTouchArea = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)F3_InvisibleTouchArea, 1, 67);
    laWidget_SetSize((laWidget*)F3_InvisibleTouchArea, 52, 28);
    laWidget_SetScheme((laWidget*)F3_InvisibleTouchArea, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)F3_InvisibleTouchArea, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)F3_InvisibleTouchArea, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(F3_InvisibleTouchArea, &F3_InvisibleTouchArea_ReleasedEvent);

    laWidget_AddChild((laWidget*)SignalSelectionPanel, (laWidget*)F3_InvisibleTouchArea);

    F1_RadioButton = laRadioButtonWidget_New();
    laWidget_SetPosition((laWidget*)F1_RadioButton, 1, 8);
    laWidget_SetSize((laWidget*)F1_RadioButton, 42, 25);
    laWidget_SetEnabled((laWidget*)F1_RadioButton, LA_FALSE);
    laWidget_SetScheme((laWidget*)F1_RadioButton, &RadioButtonScheme);
    laWidget_SetBackgroundType((laWidget*)F1_RadioButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)F1_RadioButton, LA_WIDGET_BORDER_NONE);
    laRadioButtonWidget_SetText(F1_RadioButton, laString_CreateFromID(string_F1));
    laRadioButtonWidget_SetHAlignment(F1_RadioButton, LA_HALIGN_LEFT);
    laRadioButtonGroup_Create(&radioButtonGroup_1);
    laRadioButtonGroup_AddButton(radioButtonGroup_1, F1_RadioButton);
    laRadioButtonWidget_SetSelected(F1_RadioButton);
    laWidget_AddChild((laWidget*)SignalSelectionPanel, (laWidget*)F1_RadioButton);

    F2_RadioButton = laRadioButtonWidget_New();
    laWidget_SetPosition((laWidget*)F2_RadioButton, 1, 37);
    laWidget_SetSize((laWidget*)F2_RadioButton, 42, 25);
    laWidget_SetEnabled((laWidget*)F2_RadioButton, LA_FALSE);
    laWidget_SetScheme((laWidget*)F2_RadioButton, &RadioButtonScheme);
    laWidget_SetBackgroundType((laWidget*)F2_RadioButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)F2_RadioButton, LA_WIDGET_BORDER_NONE);
    laRadioButtonWidget_SetText(F2_RadioButton, laString_CreateFromID(string_F2));
    laRadioButtonWidget_SetHAlignment(F2_RadioButton, LA_HALIGN_LEFT);
    laRadioButtonGroup_AddButton(radioButtonGroup_1, F2_RadioButton);
    laWidget_AddChild((laWidget*)SignalSelectionPanel, (laWidget*)F2_RadioButton);

    F3_RadioButton = laRadioButtonWidget_New();
    laWidget_SetPosition((laWidget*)F3_RadioButton, 1, 66);
    laWidget_SetSize((laWidget*)F3_RadioButton, 42, 25);
    laWidget_SetEnabled((laWidget*)F3_RadioButton, LA_FALSE);
    laWidget_SetScheme((laWidget*)F3_RadioButton, &RadioButtonScheme);
    laWidget_SetBackgroundType((laWidget*)F3_RadioButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)F3_RadioButton, LA_WIDGET_BORDER_NONE);
    laRadioButtonWidget_SetText(F3_RadioButton, laString_CreateFromID(string_F3));
    laRadioButtonWidget_SetHAlignment(F3_RadioButton, LA_HALIGN_LEFT);
    laRadioButtonGroup_AddButton(radioButtonGroup_1, F3_RadioButton);
    laWidget_AddChild((laWidget*)SignalSelectionPanel, (laWidget*)F3_RadioButton);

    IncButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)IncButton, 51, 48);
    laWidget_SetSize((laWidget*)IncButton, 39, 43);
    laWidget_SetScheme((laWidget*)IncButton, &ButtonSchemeInActive);
    laWidget_SetBackgroundType((laWidget*)IncButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)IncButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(IncButton, laString_CreateFromID(string_Inc));
    laButtonWidget_SetReleasedEventCallback(IncButton, &IncButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)SignalSelectionPanel, (laWidget*)IncButton);

    DecButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)DecButton, 94, 48);
    laWidget_SetSize((laWidget*)DecButton, 39, 43);
    laWidget_SetScheme((laWidget*)DecButton, &ButtonSchemeInActive);
    laWidget_SetBackgroundType((laWidget*)DecButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)DecButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(DecButton, laString_CreateFromID(string_Dec));
    laButtonWidget_SetReleasedEventCallback(DecButton, &DecButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)SignalSelectionPanel, (laWidget*)DecButton);

    ClrButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ClrButton, 137, 48);
    laWidget_SetSize((laWidget*)ClrButton, 39, 43);
    laWidget_SetScheme((laWidget*)ClrButton, &ButtonSchemeInActive);
    laWidget_SetBackgroundType((laWidget*)ClrButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ClrButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(ClrButton, laString_CreateFromID(string_CLR));
    laButtonWidget_SetReleasedEventCallback(ClrButton, &ClrButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)SignalSelectionPanel, (laWidget*)ClrButton);

    UnitsButtonHz = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)UnitsButtonHz, 180, 4);
    laWidget_SetSize((laWidget*)UnitsButtonHz, 37, 28);
    laWidget_SetScheme((laWidget*)UnitsButtonHz, &ButtonSchemeInActive);
    laWidget_SetBackgroundType((laWidget*)UnitsButtonHz, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)UnitsButtonHz, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(UnitsButtonHz, laString_CreateFromID(string_Hz));
    laButtonWidget_SetReleasedEventCallback(UnitsButtonHz, &UnitsButtonHz_ReleasedEvent);

    laWidget_AddChild((laWidget*)SignalSelectionPanel, (laWidget*)UnitsButtonHz);

    UnitsButtonKHz = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)UnitsButtonKHz, 180, 34);
    laWidget_SetSize((laWidget*)UnitsButtonKHz, 37, 28);
    laWidget_SetScheme((laWidget*)UnitsButtonKHz, &ButtonSchemeInActive);
    laWidget_SetBackgroundType((laWidget*)UnitsButtonKHz, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)UnitsButtonKHz, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(UnitsButtonKHz, laString_CreateFromID(string_kHz));
    laButtonWidget_SetReleasedEventCallback(UnitsButtonKHz, &UnitsButtonKHz_ReleasedEvent);

    laWidget_AddChild((laWidget*)SignalSelectionPanel, (laWidget*)UnitsButtonKHz);

    UnitsButtondBFS = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)UnitsButtondBFS, 180, 64);
    laWidget_SetSize((laWidget*)UnitsButtondBFS, 37, 28);
    laWidget_SetScheme((laWidget*)UnitsButtondBFS, &ButtonSchemeInActive);
    laWidget_SetBackgroundType((laWidget*)UnitsButtondBFS, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)UnitsButtondBFS, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(UnitsButtondBFS, laString_CreateFromID(string_dBFS));
    laButtonWidget_SetReleasedEventCallback(UnitsButtondBFS, &UnitsButtondBFS_ReleasedEvent);

    laWidget_AddChild((laWidget*)SignalSelectionPanel, (laWidget*)UnitsButtondBFS);

    TextBox = laTextFieldWidget_New();
    laWidget_SetPosition((laWidget*)TextBox, 50, 4);
    laWidget_SetSize((laWidget*)TextBox, 126, 38);
    laWidget_SetEnabled((laWidget*)TextBox, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)TextBox, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TextBox, LA_WIDGET_BORDER_BEVEL);
    laTextFieldWidget_SetText(TextBox, laString_CreateFromID(string_TextBoxInitialValue));
    laTextFieldWidget_SetCursorEnabled(TextBox, LA_TRUE);
    laWidget_AddChild((laWidget*)SignalSelectionPanel, (laWidget*)TextBox);

    WindowFuncSelectionPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)WindowFuncSelectionPanel, 333, 38);
    laWidget_SetSize((laWidget*)WindowFuncSelectionPanel, 89, 57);
    laWidget_SetScheme((laWidget*)WindowFuncSelectionPanel, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)WindowFuncSelectionPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)WindowFuncSelectionPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, WindowFuncSelectionPanel);

    WindowFuncLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)WindowFuncLabel, 1, 0);
    laWidget_SetSize((laWidget*)WindowFuncLabel, 87, 16);
    laWidget_SetScheme((laWidget*)WindowFuncLabel, &LabelScheme);
    laWidget_SetBackgroundType((laWidget*)WindowFuncLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)WindowFuncLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(WindowFuncLabel, laString_CreateFromID(string_Window_Func));
    laWidget_AddChild((laWidget*)WindowFuncSelectionPanel, (laWidget*)WindowFuncLabel);

    Hanning_Win_InvisibleTouchArea = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)Hanning_Win_InvisibleTouchArea, 3, 15);
    laWidget_SetSize((laWidget*)Hanning_Win_InvisibleTouchArea, 84, 20);
    laWidget_SetScheme((laWidget*)Hanning_Win_InvisibleTouchArea, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)Hanning_Win_InvisibleTouchArea, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)Hanning_Win_InvisibleTouchArea, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetReleasedEventCallback(Hanning_Win_InvisibleTouchArea, &Hanning_Win_InvisibleTouchArea_ReleasedEvent);

    laWidget_AddChild((laWidget*)WindowFuncSelectionPanel, (laWidget*)Hanning_Win_InvisibleTouchArea);

    Blackman_Win_InvisibleTouchArea = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)Blackman_Win_InvisibleTouchArea, 3, 36);
    laWidget_SetSize((laWidget*)Blackman_Win_InvisibleTouchArea, 84, 21);
    laWidget_SetScheme((laWidget*)Blackman_Win_InvisibleTouchArea, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)Blackman_Win_InvisibleTouchArea, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)Blackman_Win_InvisibleTouchArea, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetReleasedEventCallback(Blackman_Win_InvisibleTouchArea, &Blackman_Win_InvisibleTouchArea_ReleasedEvent);

    laWidget_AddChild((laWidget*)WindowFuncSelectionPanel, (laWidget*)Blackman_Win_InvisibleTouchArea);

    HannWindow_RadioButton = laRadioButtonWidget_New();
    laWidget_SetPosition((laWidget*)HannWindow_RadioButton, 2, 15);
    laWidget_SetSize((laWidget*)HannWindow_RadioButton, 85, 21);
    laWidget_SetEnabled((laWidget*)HannWindow_RadioButton, LA_FALSE);
    laWidget_SetScheme((laWidget*)HannWindow_RadioButton, &RadioButtonScheme);
    laWidget_SetBackgroundType((laWidget*)HannWindow_RadioButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)HannWindow_RadioButton, LA_WIDGET_BORDER_NONE);
    laRadioButtonWidget_SetText(HannWindow_RadioButton, laString_CreateFromID(string_Hann_Window));
    laRadioButtonWidget_SetHAlignment(HannWindow_RadioButton, LA_HALIGN_LEFT);
    laRadioButtonGroup_Create(&radioButtonGroup_2);
    laRadioButtonGroup_AddButton(radioButtonGroup_2, HannWindow_RadioButton);
    laRadioButtonWidget_SetSelected(HannWindow_RadioButton);
    laWidget_AddChild((laWidget*)WindowFuncSelectionPanel, (laWidget*)HannWindow_RadioButton);

    BlackmanWindow_RadioButton = laRadioButtonWidget_New();
    laWidget_SetPosition((laWidget*)BlackmanWindow_RadioButton, 2, 36);
    laWidget_SetSize((laWidget*)BlackmanWindow_RadioButton, 85, 21);
    laWidget_SetEnabled((laWidget*)BlackmanWindow_RadioButton, LA_FALSE);
    laWidget_SetScheme((laWidget*)BlackmanWindow_RadioButton, &RadioButtonScheme);
    laWidget_SetBackgroundType((laWidget*)BlackmanWindow_RadioButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)BlackmanWindow_RadioButton, LA_WIDGET_BORDER_NONE);
    laRadioButtonWidget_SetText(BlackmanWindow_RadioButton, laString_CreateFromID(string_Blackman_Window));
    laRadioButtonWidget_SetHAlignment(BlackmanWindow_RadioButton, LA_HALIGN_LEFT);
    laRadioButtonGroup_AddButton(radioButtonGroup_2, BlackmanWindow_RadioButton);
    laWidget_AddChild((laWidget*)WindowFuncSelectionPanel, (laWidget*)BlackmanWindow_RadioButton);

}



