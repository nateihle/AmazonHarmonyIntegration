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

laScheme RedTextScheme;
laScheme StreakScheme;
laScheme ClearScheme;
laScheme RedLineScheme;
laScheme BlackBackgroundScheme;
laScheme ButtonTextScheme;
laScheme defaultScheme;
laScheme BlueTextScheme;
laScheme whiteScheme;
laScheme PanelScheme;
laScheme AriaColorScheme;
laScheme FilmBackgroundScheme;
laScheme WhiteTextBlackBackgroundScheme;
laScheme WhiteTextScheme;
laImageWidget* ImageWidget1;
laImageWidget* ImageWidget3;
laImageWidget* ImageWidget2;
laImageWidget* ImageWidget4;
laImageWidget* ImageWidget;
laButtonWidget* PlayUSBButton;
laButtonWidget* PlaySDButton;
laButtonWidget* ButtonWidget15;
laButtonWidget* ButtonWidget12;
laLabelWidget* NoMediaLabelWidget;
laWidget* AppLogoPanel;
laLabelWidget* LabelWidget1;
laLabelWidget* LabelWidget2;
laLineWidget* LineWidget;
laImageWidget* ImageWidget;
laLabelWidget* LabelWidget5;
laButtonWidget* ButtonWidget7;
laLabelWidget* LabelWidget8;
laLabelWidget* LabelWidget12;
laLabelWidget* LabelWidget6;
laLabelWidget* LabelWidget16;
laLabelWidget* LabelWidget;
laWidget* PanelWidget16;
laLabelWidget* LabelWidget13;
laLabelWidget* LabelWidget14;
laLabelWidget* LabelWidget15;
laLabelWidget* LabelWidget7;
laLabelWidget* LabelWidget4;
laLabelWidget* LabelWidget3;
laLabelWidget* LabelWidgetvideo3rgb;
laLabelWidget* LabelWidget11;
laLabelWidget* LabelWidget10;
laLabelWidget* LabelWidget9;
laButtonWidget* BackToMenuButton;
laGroupBoxWidget* GroupBoxWidget1;
laGroupBoxWidget* GroupBoxWidget2;
laGroupBoxWidget* GroupBoxWidget14;
laListWidget* VertAlignListWidget;
laListWidget* HorzAlignListWidget;
laListWidget* ResolutionListWidget;
laListWidget* PlaySpeedListWidget;
laButtonWidget* ShowFrameRateButtonWidget;
laCheckBoxWidget* CheckBoxWidget13;
laWidget* TouchPanelWidget;
laLabelWidget* PlaybackMessageLabel;
laWidget* PlayBackControlPanel;
laButtonWidget* PlayPauseButtonWidget;
laButtonWidget* FFButtonWidget;
laButtonWidget* RewindButtonWidget;
laButtonWidget* StopButtonWidget;
laLabelWidget* PlaybackMultiplierLabelWidget;
laButtonWidget* RestartButtonWidget;
laSliderWidget* SliderControlFull;
laWidget* MetricsPanelWidget1;
laLabelWidget* frameRateLabelWidget;
laLabelWidget* BandWidthLabelWidget;
laWidget* TouchPanelWidget2;
laLabelWidget* PlaybackMessageLabel2;
laWidget* RightPanelWidget;
laButtonWidget* StopButtonWidget2;
laButtonWidget* FFButtonWidget2;
laButtonWidget* RWButtonWidget2;
laButtonWidget* PauseButtonWidget2;
laButtonWidget* RestartButtonWidget2;
laLabelWidget* PlaybackMultiplierLabelWidget2;
laWidget* MetricsPanelWidget2;
laLabelWidget* BandWidthLabelWidget2;
laLabelWidget* FrameRateLabelWidget2;
laSliderWidget* SliderControlSmall;


static void ScreenCreate_SplashScreen(laScreen* screen);
static void ScreenCreate_MainScreen(laScreen* screen);
static void ScreenCreate_HelpScreen(laScreen* screen);
static void ScreenCreate_SettingsScreen(laScreen* screen);
static void ScreenCreate_PlayBackScreen(laScreen* screen);
static void ScreenCreate_PlayBackScreenSmall(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&RedTextScheme, GFX_COLOR_MODE_RGBA_8888);
    RedTextScheme.base = 0xC8D0D4FF;
    RedTextScheme.highlight = 0xC8D0D4FF;
    RedTextScheme.highlightLight = 0xFFFFFFFF;
    RedTextScheme.shadow = 0x808080FF;
    RedTextScheme.shadowDark = 0x404040FF;
    RedTextScheme.foreground = 0xFF;
    RedTextScheme.foregroundInactive = 0xD6E3E7FF;
    RedTextScheme.foregroundDisabled = 0x808080FF;
    RedTextScheme.background = 0xFFFFFFFF;
    RedTextScheme.backgroundInactive = 0xD6E3E7FF;
    RedTextScheme.backgroundDisabled = 0xC8D0D4FF;
    RedTextScheme.text = 0xFF0000FF;
    RedTextScheme.textHighlight = 0xFF0000FF;
    RedTextScheme.textHighlightText = 0xFFFFFFFF;
    RedTextScheme.textInactive = 0xD6E3E7FF;
    RedTextScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&StreakScheme, GFX_COLOR_MODE_RGBA_8888);
    StreakScheme.base = 0xC8D0D4FF;
    StreakScheme.highlight = 0xFFFF;
    StreakScheme.highlightLight = 0xFFFFFFFF;
    StreakScheme.shadow = 0x808080FF;
    StreakScheme.shadowDark = 0x404040FF;
    StreakScheme.foreground = 0xFFFF;
    StreakScheme.foregroundInactive = 0xD6E3E7FF;
    StreakScheme.foregroundDisabled = 0x808080FF;
    StreakScheme.background = 0xFFFFFFFF;
    StreakScheme.backgroundInactive = 0xD6E3E7FF;
    StreakScheme.backgroundDisabled = 0xC8D0D4FF;
    StreakScheme.text = 0xFF;
    StreakScheme.textHighlight = 0xFFFF;
    StreakScheme.textHighlightText = 0xFFFFFFFF;
    StreakScheme.textInactive = 0xD6E3E7FF;
    StreakScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&ClearScheme, GFX_COLOR_MODE_RGBA_8888);
    ClearScheme.base = 0x0;
    ClearScheme.highlight = 0xC8D0D4FF;
    ClearScheme.highlightLight = 0xFFFFFFFF;
    ClearScheme.shadow = 0x808080FF;
    ClearScheme.shadowDark = 0x404040FF;
    ClearScheme.foreground = 0xFF;
    ClearScheme.foregroundInactive = 0xD6E3E7FF;
    ClearScheme.foregroundDisabled = 0x808080FF;
    ClearScheme.background = 0x0;
    ClearScheme.backgroundInactive = 0xD6E3E7FF;
    ClearScheme.backgroundDisabled = 0xC8D0D4FF;
    ClearScheme.text = 0xFF;
    ClearScheme.textHighlight = 0xFFFF;
    ClearScheme.textHighlightText = 0xFFFFFFFF;
    ClearScheme.textInactive = 0xD6E3E7FF;
    ClearScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&RedLineScheme, GFX_COLOR_MODE_RGBA_8888);
    RedLineScheme.base = 0xC8D0D4FF;
    RedLineScheme.highlight = 0xC8D0D4FF;
    RedLineScheme.highlightLight = 0xFFFFFFFF;
    RedLineScheme.shadow = 0x808080FF;
    RedLineScheme.shadowDark = 0x404040FF;
    RedLineScheme.foreground = 0xFF0000FF;
    RedLineScheme.foregroundInactive = 0xD6E3E7FF;
    RedLineScheme.foregroundDisabled = 0x808080FF;
    RedLineScheme.background = 0xFFFFFFFF;
    RedLineScheme.backgroundInactive = 0xD6E3E7FF;
    RedLineScheme.backgroundDisabled = 0xC8D0D4FF;
    RedLineScheme.text = 0xFF;
    RedLineScheme.textHighlight = 0xFFFF;
    RedLineScheme.textHighlightText = 0xFFFFFFFF;
    RedLineScheme.textInactive = 0xD6E3E7FF;
    RedLineScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&BlackBackgroundScheme, GFX_COLOR_MODE_RGBA_8888);
    BlackBackgroundScheme.base = 0xFF;
    BlackBackgroundScheme.highlight = 0xFF;
    BlackBackgroundScheme.highlightLight = 0xFF;
    BlackBackgroundScheme.shadow = 0xFF;
    BlackBackgroundScheme.shadowDark = 0x404040FF;
    BlackBackgroundScheme.foreground = 0xFF;
    BlackBackgroundScheme.foregroundInactive = 0xFF;
    BlackBackgroundScheme.foregroundDisabled = 0xFF;
    BlackBackgroundScheme.background = 0xFF;
    BlackBackgroundScheme.backgroundInactive = 0xFF;
    BlackBackgroundScheme.backgroundDisabled = 0xFF;
    BlackBackgroundScheme.text = 0xFFFFFFFF;
    BlackBackgroundScheme.textHighlight = 0xFFFF;
    BlackBackgroundScheme.textHighlightText = 0xFF;
    BlackBackgroundScheme.textInactive = 0xFF;
    BlackBackgroundScheme.textDisabled = 0xFF;

    laScheme_Initialize(&ButtonTextScheme, GFX_COLOR_MODE_RGBA_8888);
    ButtonTextScheme.base = 0xC8D0D4FF;
    ButtonTextScheme.highlight = 0xC8D0D4FF;
    ButtonTextScheme.highlightLight = 0xFFFFFFFF;
    ButtonTextScheme.shadow = 0x808080FF;
    ButtonTextScheme.shadowDark = 0x404040FF;
    ButtonTextScheme.foreground = 0xFF;
    ButtonTextScheme.foregroundInactive = 0xD6E3E7FF;
    ButtonTextScheme.foregroundDisabled = 0x808080FF;
    ButtonTextScheme.background = 0xFFFFFFFF;
    ButtonTextScheme.backgroundInactive = 0xD6E3E7FF;
    ButtonTextScheme.backgroundDisabled = 0xC8D0D4FF;
    ButtonTextScheme.text = 0xDEDFDEFF;
    ButtonTextScheme.textHighlight = 0xFFFF;
    ButtonTextScheme.textHighlightText = 0xFFFFFFFF;
    ButtonTextScheme.textInactive = 0xD6E3E7FF;
    ButtonTextScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&defaultScheme, GFX_COLOR_MODE_RGBA_8888);
    defaultScheme.base = 0xC8D0D4FF;
    defaultScheme.highlight = 0xC8D0D4FF;
    defaultScheme.highlightLight = 0xFFFFFFFF;
    defaultScheme.shadow = 0x808080FF;
    defaultScheme.shadowDark = 0x404040FF;
    defaultScheme.foreground = 0xFF;
    defaultScheme.foregroundInactive = 0xD6E3E7FF;
    defaultScheme.foregroundDisabled = 0x808080FF;
    defaultScheme.background = 0xFFFFFFFF;
    defaultScheme.backgroundInactive = 0xD6E3E7FF;
    defaultScheme.backgroundDisabled = 0xC8D0D4FF;
    defaultScheme.text = 0xFF;
    defaultScheme.textHighlight = 0xFFFF;
    defaultScheme.textHighlightText = 0xFFFFFFFF;
    defaultScheme.textInactive = 0xD6E3E7FF;
    defaultScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&BlueTextScheme, GFX_COLOR_MODE_RGBA_8888);
    BlueTextScheme.base = 0xC8D0D4FF;
    BlueTextScheme.highlight = 0xC8D0D4FF;
    BlueTextScheme.highlightLight = 0xFFFFFFFF;
    BlueTextScheme.shadow = 0x808080FF;
    BlueTextScheme.shadowDark = 0x404040FF;
    BlueTextScheme.foreground = 0xFF;
    BlueTextScheme.foregroundInactive = 0xD6E3E7FF;
    BlueTextScheme.foregroundDisabled = 0x808080FF;
    BlueTextScheme.background = 0xFFFFFFFF;
    BlueTextScheme.backgroundInactive = 0xD6E3E7FF;
    BlueTextScheme.backgroundDisabled = 0xC8D0D4FF;
    BlueTextScheme.text = 0x3A69BDFF;
    BlueTextScheme.textHighlight = 0xFFFF;
    BlueTextScheme.textHighlightText = 0xFFFFFFFF;
    BlueTextScheme.textInactive = 0xD6E3E7FF;
    BlueTextScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&whiteScheme, GFX_COLOR_MODE_RGBA_8888);
    whiteScheme.base = 0xFFFFFFFF;
    whiteScheme.highlight = 0xC8D0D4FF;
    whiteScheme.highlightLight = 0xFFFFFFFF;
    whiteScheme.shadow = 0x808080FF;
    whiteScheme.shadowDark = 0x404040FF;
    whiteScheme.foreground = 0xFF;
    whiteScheme.foregroundInactive = 0xD6E3E7FF;
    whiteScheme.foregroundDisabled = 0x808080FF;
    whiteScheme.background = 0xFFFFFFFF;
    whiteScheme.backgroundInactive = 0xD6E3E7FF;
    whiteScheme.backgroundDisabled = 0xC8D0D4FF;
    whiteScheme.text = 0xFF;
    whiteScheme.textHighlight = 0xFFFF;
    whiteScheme.textHighlightText = 0xFFFFFFFF;
    whiteScheme.textInactive = 0xD6E3E7FF;
    whiteScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&PanelScheme, GFX_COLOR_MODE_RGBA_8888);
    PanelScheme.base = 0xFF;
    PanelScheme.highlight = 0xC8D0D4FF;
    PanelScheme.highlightLight = 0xFFFFFFFF;
    PanelScheme.shadow = 0x808080FF;
    PanelScheme.shadowDark = 0x404040FF;
    PanelScheme.foreground = 0xFF;
    PanelScheme.foregroundInactive = 0xD6E3E7FF;
    PanelScheme.foregroundDisabled = 0x808080FF;
    PanelScheme.background = 0xFF;
    PanelScheme.backgroundInactive = 0xD6E3E7FF;
    PanelScheme.backgroundDisabled = 0xC8D0D4FF;
    PanelScheme.text = 0xFF;
    PanelScheme.textHighlight = 0xFFFF;
    PanelScheme.textHighlightText = 0xFFFFFFFF;
    PanelScheme.textInactive = 0xD6E3E7FF;
    PanelScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&AriaColorScheme, GFX_COLOR_MODE_RGBA_8888);
    AriaColorScheme.base = 0xC8D0D4FF;
    AriaColorScheme.highlight = 0xC8D0D4FF;
    AriaColorScheme.highlightLight = 0xFFFFFFFF;
    AriaColorScheme.shadow = 0x808080FF;
    AriaColorScheme.shadowDark = 0x404040FF;
    AriaColorScheme.foreground = 0xFF;
    AriaColorScheme.foregroundInactive = 0xD6E3E7FF;
    AriaColorScheme.foregroundDisabled = 0x808080FF;
    AriaColorScheme.background = 0xFFFFFFFF;
    AriaColorScheme.backgroundInactive = 0xD6E3E7FF;
    AriaColorScheme.backgroundDisabled = 0xC8D0D4FF;
    AriaColorScheme.text = 0xFF8200FF;
    AriaColorScheme.textHighlight = 0xFFFF;
    AriaColorScheme.textHighlightText = 0xFFFFFFFF;
    AriaColorScheme.textInactive = 0xD6E3E7FF;
    AriaColorScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&FilmBackgroundScheme, GFX_COLOR_MODE_RGBA_8888);
    FilmBackgroundScheme.base = 0x8284FF;
    FilmBackgroundScheme.highlight = 0xC8D0D4FF;
    FilmBackgroundScheme.highlightLight = 0xFFFFFFFF;
    FilmBackgroundScheme.shadow = 0x808080FF;
    FilmBackgroundScheme.shadowDark = 0x404040FF;
    FilmBackgroundScheme.foreground = 0xFF;
    FilmBackgroundScheme.foregroundInactive = 0xD6E3E7FF;
    FilmBackgroundScheme.foregroundDisabled = 0x808080FF;
    FilmBackgroundScheme.background = 0x948EFFFF;
    FilmBackgroundScheme.backgroundInactive = 0xD6E3E7FF;
    FilmBackgroundScheme.backgroundDisabled = 0xC8D0D4FF;
    FilmBackgroundScheme.text = 0xFF;
    FilmBackgroundScheme.textHighlight = 0xFFFF;
    FilmBackgroundScheme.textHighlightText = 0xFFFFFFFF;
    FilmBackgroundScheme.textInactive = 0xD6E3E7FF;
    FilmBackgroundScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&WhiteTextBlackBackgroundScheme, GFX_COLOR_MODE_RGBA_8888);
    WhiteTextBlackBackgroundScheme.base = 0xFF;
    WhiteTextBlackBackgroundScheme.highlight = 0xC8D0D4FF;
    WhiteTextBlackBackgroundScheme.highlightLight = 0xFFFFFFFF;
    WhiteTextBlackBackgroundScheme.shadow = 0x808080FF;
    WhiteTextBlackBackgroundScheme.shadowDark = 0x404040FF;
    WhiteTextBlackBackgroundScheme.foreground = 0xFF00FF;
    WhiteTextBlackBackgroundScheme.foregroundInactive = 0xD6E3E7FF;
    WhiteTextBlackBackgroundScheme.foregroundDisabled = 0x808080FF;
    WhiteTextBlackBackgroundScheme.background = 0xFF;
    WhiteTextBlackBackgroundScheme.backgroundInactive = 0xD6E3E7FF;
    WhiteTextBlackBackgroundScheme.backgroundDisabled = 0xC8D0D4FF;
    WhiteTextBlackBackgroundScheme.text = 0xFFFFFFFF;
    WhiteTextBlackBackgroundScheme.textHighlight = 0xFFFF;
    WhiteTextBlackBackgroundScheme.textHighlightText = 0xFFFFFFFF;
    WhiteTextBlackBackgroundScheme.textInactive = 0xD6E3E7FF;
    WhiteTextBlackBackgroundScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&WhiteTextScheme, GFX_COLOR_MODE_RGBA_8888);
    WhiteTextScheme.base = 0xC8D0D4FF;
    WhiteTextScheme.highlight = 0xC8D0D4FF;
    WhiteTextScheme.highlightLight = 0xFFFFFFFF;
    WhiteTextScheme.shadow = 0x808080FF;
    WhiteTextScheme.shadowDark = 0x404040FF;
    WhiteTextScheme.foreground = 0xFF;
    WhiteTextScheme.foregroundInactive = 0xD6E3E7FF;
    WhiteTextScheme.foregroundDisabled = 0x808080FF;
    WhiteTextScheme.background = 0xFFFFFFFF;
    WhiteTextScheme.backgroundInactive = 0xD6E3E7FF;
    WhiteTextScheme.backgroundDisabled = 0xC8D0D4FF;
    WhiteTextScheme.text = 0xFFFFFFFF;
    WhiteTextScheme.textHighlight = 0xFFFF;
    WhiteTextScheme.textHighlightText = 0xFFFFFFFF;
    WhiteTextScheme.textInactive = 0xD6E3E7FF;
    WhiteTextScheme.textDisabled = 0x8C9294FF;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_SplashScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_MainScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_HelpScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_SettingsScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_PlayBackScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_PlayBackScreenSmall);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(0);

	return 0;
}

static void ScreenCreate_SplashScreen(laScreen* screen)
{
    laLayer* layer0;
    laLayer* layer1;
    laLayer* layer2;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);
    laLayer_SetAlphaEnable(layer0, LA_TRUE);
    laLayer_SetAlphaAmount(layer0, 0xFF);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 217, 175);
    laWidget_SetSize((laWidget*)ImageWidget1, 372, 96);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &PIC32Logo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_NONE);
    laLayer_SetBufferCount(layer1, 1);
    laLayer_SetAlphaEnable(layer1, LA_TRUE);
    laLayer_SetAlphaAmount(layer1, 0xFF);

    laScreen_SetLayer(screen, 1, layer1);

    ImageWidget3 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget3, 800, 389);
    laWidget_SetSize((laWidget*)ImageWidget3, 800, 91);
    laWidget_SetBackgroundType((laWidget*)ImageWidget3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget3, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget3, &Bar);
    laWidget_AddChild((laWidget*)layer1, (laWidget*)ImageWidget3);

    layer2 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer2, 0, 0);
    laWidget_SetSize((laWidget*)layer2, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer2, LA_WIDGET_BACKGROUND_NONE);
    laLayer_SetBufferCount(layer2, 1);
    laWidget_SetOptimizationFlags((laWidget*)layer2, LA_WIDGET_OPT_DRAW_ONCE);
    laLayer_SetAlphaEnable(layer2, LA_TRUE);
    laLayer_SetAlphaAmount(layer2, 0x0);
    laLayer_SetVSync(layer2, LA_FALSE);

    laScreen_SetLayer(screen, 2, layer2);

    ImageWidget2 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget2, 245, 113);
    laWidget_SetSize((laWidget*)ImageWidget2, 307, 214);
    laWidget_SetBackgroundType((laWidget*)ImageWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget2, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget2, &HarmonyLogo);
    laWidget_AddChild((laWidget*)layer2, (laWidget*)ImageWidget2);

    ImageWidget4 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget4, 18, 417);
    laWidget_SetSize((laWidget*)ImageWidget4, 202, 55);
    laWidget_SetBackgroundType((laWidget*)ImageWidget4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget4, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget4, &MicrochipLogo);
    laWidget_AddChild((laWidget*)layer2, (laWidget*)ImageWidget4);

}

static void ScreenCreate_MainScreen(laScreen* screen)
{
    laLayer* layer0;

    laScreen_SetShowEventCallback(screen, &MainScreen_ShowEvent);

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &whiteScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget = laImageWidget_New();
    laWidget_SetSize((laWidget*)ImageWidget, 360, 481);
    laWidget_SetScheme((laWidget*)ImageWidget, &FilmBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &VideoFilm);
    laImageWidget_SetHAlignment(ImageWidget, LA_HALIGN_RIGHT);
    laImageWidget_SetVAlignment(ImageWidget, LA_VALIGN_BOTTOM);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

    PlayUSBButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)PlayUSBButton, 431, 78);
    laWidget_SetSize((laWidget*)PlayUSBButton, 138, 133);
    laWidget_SetScheme((laWidget*)PlayUSBButton, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)PlayUSBButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PlayUSBButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetHAlignment(PlayUSBButton, LA_HALIGN_LEFT);
    laButtonWidget_SetPressedImage(PlayUSBButton, &USBDriveSquare);
    laButtonWidget_SetReleasedImage(PlayUSBButton, &USBDriveSquare);
    laButtonWidget_SetImageMargin(PlayUSBButton, 1);
    laButtonWidget_SetReleasedEventCallback(PlayUSBButton, &PlayUSBButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)PlayUSBButton);

    PlaySDButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)PlaySDButton, 600, 76);
    laWidget_SetSize((laWidget*)PlaySDButton, 139, 137);
    laWidget_SetScheme((laWidget*)PlaySDButton, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)PlaySDButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PlaySDButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetHAlignment(PlaySDButton, LA_HALIGN_LEFT);
    laButtonWidget_SetPressedImage(PlaySDButton, &SDCardSquare);
    laButtonWidget_SetReleasedImage(PlaySDButton, &SDCardSquare);
    laButtonWidget_SetImageMargin(PlaySDButton, 1);
    laButtonWidget_SetReleasedEventCallback(PlaySDButton, &PlaySDButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)PlaySDButton);

    ButtonWidget15 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget15, 600, 235);
    laWidget_SetSize((laWidget*)ButtonWidget15, 140, 133);
    laWidget_SetScheme((laWidget*)ButtonWidget15, &ButtonTextScheme);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget15, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget15, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetHAlignment(ButtonWidget15, LA_HALIGN_LEFT);
    laButtonWidget_SetVAlignment(ButtonWidget15, LA_VALIGN_TOP);
    laButtonWidget_SetPressedImage(ButtonWidget15, &HelpSquare);
    laButtonWidget_SetReleasedImage(ButtonWidget15, &HelpSquare);
    laButtonWidget_SetImageMargin(ButtonWidget15, 1);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget15, &ButtonWidget15_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget15);

    ButtonWidget12 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget12, 430, 233);
    laWidget_SetSize((laWidget*)ButtonWidget12, 140, 137);
    laWidget_SetScheme((laWidget*)ButtonWidget12, &ButtonTextScheme);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget12, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget12, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetHAlignment(ButtonWidget12, LA_HALIGN_LEFT);
    laButtonWidget_SetVAlignment(ButtonWidget12, LA_VALIGN_TOP);
    laButtonWidget_SetPressedImage(ButtonWidget12, &SettingsSquare);
    laButtonWidget_SetReleasedImage(ButtonWidget12, &SettingsSquare);
    laButtonWidget_SetImageMargin(ButtonWidget12, 1);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget12, &ButtonWidget12_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget12);

    NoMediaLabelWidget = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)NoMediaLabelWidget, 491, 133);
    laWidget_SetSize((laWidget*)NoMediaLabelWidget, 199, 25);
    laWidget_SetVisible((laWidget*)NoMediaLabelWidget, LA_FALSE);
    laWidget_SetScheme((laWidget*)NoMediaLabelWidget, &RedTextScheme);
    laWidget_SetBackgroundType((laWidget*)NoMediaLabelWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)NoMediaLabelWidget, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(NoMediaLabelWidget, laString_CreateFromID(string_NoMedia));
    laLabelWidget_SetHAlignment(NoMediaLabelWidget, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)NoMediaLabelWidget);

    AppLogoPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)AppLogoPanel, 36, 32);
    laWidget_SetSize((laWidget*)AppLogoPanel, 324, 126);
    laWidget_SetScheme((laWidget*)AppLogoPanel, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)AppLogoPanel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)AppLogoPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, AppLogoPanel);

    LabelWidget1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget1, 11, 0);
    laWidget_SetSize((laWidget*)LabelWidget1, 164, 82);
    laWidget_SetScheme((laWidget*)LabelWidget1, &AriaColorScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget1, laString_CreateFromID(string_Aria));
    laLabelWidget_SetHAlignment(LabelWidget1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)AppLogoPanel, (laWidget*)LabelWidget1);

    LabelWidget2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget2, 53, 43);
    laWidget_SetSize((laWidget*)LabelWidget2, 265, 84);
    laWidget_SetScheme((laWidget*)LabelWidget2, &RedTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget2, laString_CreateFromID(string_Player));
    laLabelWidget_SetHAlignment(LabelWidget2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)AppLogoPanel, (laWidget*)LabelWidget2);

    LineWidget = laLineWidget_New();
    laWidget_SetPosition((laWidget*)LineWidget, 391, 117);
    laWidget_SetSize((laWidget*)LineWidget, 26, 252);
    laWidget_SetScheme((laWidget*)LineWidget, &RedLineScheme);
    laWidget_SetBackgroundType((laWidget*)LineWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LineWidget, LA_WIDGET_BORDER_NONE);
    laLineWidget_SetStartPoint(LineWidget, 10, 0);
    laLineWidget_SetEndPoint(LineWidget, 10, 220);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LineWidget);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 643, 0);
    laWidget_SetSize((laWidget*)ImageWidget, 158, 40);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &Microchip_logo_150x30);
    laImageWidget_SetHAlignment(ImageWidget, LA_HALIGN_RIGHT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

}

static void ScreenCreate_HelpScreen(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    LabelWidget5 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget5, 5, 23);
    laWidget_SetSize((laWidget*)LabelWidget5, 370, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget5, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget5, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget5, laString_CreateFromID(string_GenerateHelp));
    laLabelWidget_SetHAlignment(LabelWidget5, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget5);

    ButtonWidget7 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget7, 690, 430);
    laWidget_SetSize((laWidget*)ButtonWidget7, 105, 45);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget7, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget7, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(ButtonWidget7, laString_CreateFromID(string_OK));
    laButtonWidget_SetImagePosition(ButtonWidget7, LA_RELATIVE_POSITION_RIGHTOF);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget7, &ButtonWidget7_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget7);

    LabelWidget8 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget8, 5, 103);
    laWidget_SetSize((laWidget*)LabelWidget8, 545, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget8, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget8, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget8, laString_CreateFromID(string_FFMPEGParams));
    laLabelWidget_SetHAlignment(LabelWidget8, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget8);

    LabelWidget12 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget12, 5, 287);
    laWidget_SetSize((laWidget*)LabelWidget12, 461, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget12, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget12, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget12, laString_CreateFromID(string_CopyUSBorSD));
    laLabelWidget_SetHAlignment(LabelWidget12, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget12);

    LabelWidget6 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget6, 16, 57);
    laWidget_SetSize((laWidget*)LabelWidget6, 485, 25);
    laWidget_SetScheme((laWidget*)LabelWidget6, &BlueTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget6, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget6, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget6, laString_CreateFromID(string_FFMPEGCommand));
    laLabelWidget_SetHAlignment(LabelWidget6, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget6);

    LabelWidget16 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget16, 486, 57);
    laWidget_SetSize((laWidget*)LabelWidget16, 315, 25);
    laWidget_SetScheme((laWidget*)LabelWidget16, &BlueTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget16, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget16, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget16, laString_CreateFromID(string_FFMPEGCommand2));
    laLabelWidget_SetHAlignment(LabelWidget16, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget16);

    LabelWidget = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget, 5, 322);
    laWidget_SetSize((laWidget*)LabelWidget, 461, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget, laString_CreateFromID(string_InsertToBoard));
    laLabelWidget_SetHAlignment(LabelWidget, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget);

    PanelWidget16 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget16, 177, 144);
    laWidget_SetSize((laWidget*)PanelWidget16, 472, 144);
    laWidget_SetBackgroundType((laWidget*)PanelWidget16, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PanelWidget16, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget16);

    LabelWidget13 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget13, 12, 104);
    laWidget_SetSize((laWidget*)LabelWidget13, 218, 25);
    laWidget_SetScheme((laWidget*)LabelWidget13, &BlueTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget13, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget13, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget13, laString_CreateFromID(string_VRes800x480small));
    laWidget_AddChild((laWidget*)PanelWidget16, (laWidget*)LabelWidget13);

    LabelWidget14 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget14, 12, 5);
    laWidget_SetSize((laWidget*)LabelWidget14, 218, 25);
    laWidget_SetScheme((laWidget*)LabelWidget14, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget14, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget14, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget14, laString_CreateFromID(string_VideoResolutionSmall));
    laWidget_AddChild((laWidget*)PanelWidget16, (laWidget*)LabelWidget14);

    LabelWidget15 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget15, 230, 5);
    laWidget_SetSize((laWidget*)LabelWidget15, 218, 25);
    laWidget_SetScheme((laWidget*)LabelWidget15, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget15, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget15, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget15, laString_CreateFromID(string_Filename));
    laWidget_AddChild((laWidget*)PanelWidget16, (laWidget*)LabelWidget15);

    LabelWidget7 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget7, 12, 79);
    laWidget_SetSize((laWidget*)LabelWidget7, 218, 25);
    laWidget_SetScheme((laWidget*)LabelWidget7, &BlueTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget7, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget7, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget7, laString_CreateFromID(string_VRes480x272small));
    laWidget_AddChild((laWidget*)PanelWidget16, (laWidget*)LabelWidget7);

    LabelWidget4 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget4, 12, 54);
    laWidget_SetSize((laWidget*)LabelWidget4, 218, 25);
    laWidget_SetScheme((laWidget*)LabelWidget4, &BlueTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget4, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget4, laString_CreateFromID(string_VRes320x240small));
    laWidget_AddChild((laWidget*)PanelWidget16, (laWidget*)LabelWidget4);

    LabelWidget3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget3, 12, 29);
    laWidget_SetSize((laWidget*)LabelWidget3, 218, 25);
    laWidget_SetScheme((laWidget*)LabelWidget3, &BlueTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget3, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget3, laString_CreateFromID(string_VRes320x180small));
    laWidget_AddChild((laWidget*)PanelWidget16, (laWidget*)LabelWidget3);

    LabelWidgetvideo3rgb = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidgetvideo3rgb, 230, 104);
    laWidget_SetSize((laWidget*)LabelWidgetvideo3rgb, 218, 25);
    laWidget_SetScheme((laWidget*)LabelWidgetvideo3rgb, &BlueTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidgetvideo3rgb, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidgetvideo3rgb, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidgetvideo3rgb, laString_CreateFromID(string_video3rgb));
    laWidget_AddChild((laWidget*)PanelWidget16, (laWidget*)LabelWidgetvideo3rgb);

    LabelWidget11 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget11, 230, 79);
    laWidget_SetSize((laWidget*)LabelWidget11, 218, 25);
    laWidget_SetScheme((laWidget*)LabelWidget11, &BlueTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget11, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget11, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget11, laString_CreateFromID(string_video2rgb));
    laWidget_AddChild((laWidget*)PanelWidget16, (laWidget*)LabelWidget11);

    LabelWidget10 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget10, 230, 54);
    laWidget_SetSize((laWidget*)LabelWidget10, 218, 25);
    laWidget_SetScheme((laWidget*)LabelWidget10, &BlueTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget10, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget10, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget10, laString_CreateFromID(string_video1rgb));
    laWidget_AddChild((laWidget*)PanelWidget16, (laWidget*)LabelWidget10);

    LabelWidget9 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget9, 230, 29);
    laWidget_SetSize((laWidget*)LabelWidget9, 218, 25);
    laWidget_SetScheme((laWidget*)LabelWidget9, &BlueTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget9, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget9, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget9, laString_CreateFromID(string_video0rgb));
    laWidget_AddChild((laWidget*)PanelWidget16, (laWidget*)LabelWidget9);

}

static void ScreenCreate_SettingsScreen(laScreen* screen)
{
    laLayer* layer0;

    laScreen_SetShowEventCallback(screen, &SettingsScreen_ShowEvent);

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &BlackBackgroundScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    BackToMenuButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)BackToMenuButton, 687, 432);
    laWidget_SetSize((laWidget*)BackToMenuButton, 109, 44);
    laWidget_SetScheme((laWidget*)BackToMenuButton, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)BackToMenuButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BackToMenuButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(BackToMenuButton, laString_CreateFromID(string_OK));
    laButtonWidget_SetReleasedEventCallback(BackToMenuButton, &BackToMenuButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)BackToMenuButton);

    GroupBoxWidget1 = laGroupBoxWidget_New();
    laWidget_SetPosition((laWidget*)GroupBoxWidget1, 309, 3);
    laWidget_SetSize((laWidget*)GroupBoxWidget1, 330, 226);
    laWidget_SetScheme((laWidget*)GroupBoxWidget1, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)GroupBoxWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GroupBoxWidget1, LA_WIDGET_BORDER_NONE);
    laGroupBoxWidget_SetText(GroupBoxWidget1, laString_CreateFromID(string_FramesPerSec));
    laGroupBoxWidget_SetAlignment(GroupBoxWidget1, LA_HALIGN_CENTER);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GroupBoxWidget1);

    GroupBoxWidget2 = laGroupBoxWidget_New();
    laWidget_SetPosition((laWidget*)GroupBoxWidget2, 4, 4);
    laWidget_SetSize((laWidget*)GroupBoxWidget2, 297, 218);
    laWidget_SetScheme((laWidget*)GroupBoxWidget2, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)GroupBoxWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GroupBoxWidget2, LA_WIDGET_BORDER_NONE);
    laGroupBoxWidget_SetText(GroupBoxWidget2, laString_CreateFromID(string_VideoResolution));
    laGroupBoxWidget_SetAlignment(GroupBoxWidget2, LA_HALIGN_CENTER);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GroupBoxWidget2);

    GroupBoxWidget14 = laGroupBoxWidget_New();
    laWidget_SetPosition((laWidget*)GroupBoxWidget14, 5, 222);
    laWidget_SetSize((laWidget*)GroupBoxWidget14, 297, 187);
    laWidget_SetScheme((laWidget*)GroupBoxWidget14, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)GroupBoxWidget14, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GroupBoxWidget14, LA_WIDGET_BORDER_NONE);
    laGroupBoxWidget_SetText(GroupBoxWidget14, laString_CreateFromID(string_Alignment));
    laGroupBoxWidget_SetAlignment(GroupBoxWidget14, LA_HALIGN_CENTER);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GroupBoxWidget14);

    VertAlignListWidget = laListWidget_New();
    laWidget_SetPosition((laWidget*)VertAlignListWidget, 21, 269);
    laWidget_SetSize((laWidget*)VertAlignListWidget, 126, 121);
    laWidget_SetScheme((laWidget*)VertAlignListWidget, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)VertAlignListWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)VertAlignListWidget, LA_WIDGET_BORDER_LINE);
    laListWidget_SetSelectionMode(VertAlignListWidget, LA_LIST_WIDGET_SELECTION_MODE_SINGLE);
    laListWidget_SetAlignment(VertAlignListWidget, LA_HALIGN_CENTER);
    laListWidget_AppendItem(VertAlignListWidget);
    laListWidget_SetItemText(VertAlignListWidget, 0, laString_CreateFromID(string_Center));
    laListWidget_AppendItem(VertAlignListWidget);
    laListWidget_SetItemText(VertAlignListWidget, 1, laString_CreateFromID(string_Top));
    laListWidget_AppendItem(VertAlignListWidget);
    laListWidget_SetItemText(VertAlignListWidget, 2, laString_CreateFromID(string_Bottom));
    laListWidget_SetSelectedItemChangedEventCallback(VertAlignListWidget, &VertAlignListWidget_SelectionChangedEvent);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)VertAlignListWidget);

    HorzAlignListWidget = laListWidget_New();
    laWidget_SetPosition((laWidget*)HorzAlignListWidget, 169, 269);
    laWidget_SetSize((laWidget*)HorzAlignListWidget, 119, 121);
    laWidget_SetScheme((laWidget*)HorzAlignListWidget, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)HorzAlignListWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)HorzAlignListWidget, LA_WIDGET_BORDER_LINE);
    laListWidget_SetSelectionMode(HorzAlignListWidget, LA_LIST_WIDGET_SELECTION_MODE_SINGLE);
    laListWidget_SetAlignment(HorzAlignListWidget, LA_HALIGN_CENTER);
    laListWidget_AppendItem(HorzAlignListWidget);
    laListWidget_SetItemText(HorzAlignListWidget, 0, laString_CreateFromID(string_Center));
    laListWidget_AppendItem(HorzAlignListWidget);
    laListWidget_SetItemText(HorzAlignListWidget, 1, laString_CreateFromID(string_Left));
    laListWidget_AppendItem(HorzAlignListWidget);
    laListWidget_SetItemText(HorzAlignListWidget, 2, laString_CreateFromID(string_Right));
    laListWidget_SetSelectedItemChangedEventCallback(HorzAlignListWidget, &HorzAlignListWidget_SelectionChangedEvent);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)HorzAlignListWidget);

    ResolutionListWidget = laListWidget_New();
    laWidget_SetPosition((laWidget*)ResolutionListWidget, 21, 42);
    laWidget_SetSize((laWidget*)ResolutionListWidget, 257, 160);
    laWidget_SetScheme((laWidget*)ResolutionListWidget, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)ResolutionListWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ResolutionListWidget, LA_WIDGET_BORDER_LINE);
    laListWidget_SetSelectionMode(ResolutionListWidget, LA_LIST_WIDGET_SELECTION_MODE_SINGLE);
    laListWidget_SetAlignment(ResolutionListWidget, LA_HALIGN_CENTER);
    laListWidget_AppendItem(ResolutionListWidget);
    laListWidget_SetItemText(ResolutionListWidget, 0, laString_CreateFromID(string_VRes320x180));
    laListWidget_AppendItem(ResolutionListWidget);
    laListWidget_SetItemText(ResolutionListWidget, 1, laString_CreateFromID(string_VRes320x240));
    laListWidget_AppendItem(ResolutionListWidget);
    laListWidget_SetItemText(ResolutionListWidget, 2, laString_CreateFromID(string_VRes480x272));
    laListWidget_AppendItem(ResolutionListWidget);
    laListWidget_SetItemText(ResolutionListWidget, 3, laString_CreateFromID(string_VRes800x480));
    laListWidget_SetSelectedItemChangedEventCallback(ResolutionListWidget, &ResolutionListWidget_SelectionChangedEvent);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ResolutionListWidget);

    PlaySpeedListWidget = laListWidget_New();
    laWidget_SetPosition((laWidget*)PlaySpeedListWidget, 359, 42);
    laWidget_SetSize((laWidget*)PlaySpeedListWidget, 229, 160);
    laWidget_SetScheme((laWidget*)PlaySpeedListWidget, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)PlaySpeedListWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PlaySpeedListWidget, LA_WIDGET_BORDER_LINE);
    laListWidget_SetSelectionMode(PlaySpeedListWidget, LA_LIST_WIDGET_SELECTION_MODE_SINGLE);
    laListWidget_SetAlignment(PlaySpeedListWidget, LA_HALIGN_CENTER);
    laListWidget_AppendItem(PlaySpeedListWidget);
    laListWidget_SetItemText(PlaySpeedListWidget, 0, laString_CreateFromID(string_Speed_Slow));
    laListWidget_AppendItem(PlaySpeedListWidget);
    laListWidget_SetItemText(PlaySpeedListWidget, 1, laString_CreateFromID(string_Speed_Normal));
    laListWidget_AppendItem(PlaySpeedListWidget);
    laListWidget_SetItemText(PlaySpeedListWidget, 2, laString_CreateFromID(string_Speed_Fast));
    laListWidget_AppendItem(PlaySpeedListWidget);
    laListWidget_SetItemText(PlaySpeedListWidget, 3, laString_CreateFromID(string_Speed_Max));
    laListWidget_SetSelectedItemChangedEventCallback(PlaySpeedListWidget, &PlaySpeedListWidget_SelectionChangedEvent);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)PlaySpeedListWidget);

    ShowFrameRateButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ShowFrameRateButtonWidget, 311, 245);
    laWidget_SetSize((laWidget*)ShowFrameRateButtonWidget, 328, 52);
    laWidget_SetScheme((laWidget*)ShowFrameRateButtonWidget, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)ShowFrameRateButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ShowFrameRateButtonWidget, LA_WIDGET_BORDER_BEVEL);
    laWidget_SetMargins((laWidget*)ShowFrameRateButtonWidget, 10, 4, 4, 4);
    laButtonWidget_SetToggleable(ShowFrameRateButtonWidget, LA_TRUE);
    laButtonWidget_SetText(ShowFrameRateButtonWidget, laString_CreateFromID(string_ShowFrameRate));
    laButtonWidget_SetHAlignment(ShowFrameRateButtonWidget, LA_HALIGN_LEFT);
    laButtonWidget_SetPressedEventCallback(ShowFrameRateButtonWidget, &ShowFrameRateButtonWidget_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(ShowFrameRateButtonWidget, &ShowFrameRateButtonWidget_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ShowFrameRateButtonWidget);

    CheckBoxWidget13 = laCheckBoxWidget_New();
    laWidget_SetPosition((laWidget*)CheckBoxWidget13, 290, 11);
    laWidget_SetSize((laWidget*)CheckBoxWidget13, 24, 27);
    laWidget_SetEnabled((laWidget*)CheckBoxWidget13, LA_FALSE);
    laWidget_SetScheme((laWidget*)CheckBoxWidget13, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)CheckBoxWidget13, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CheckBoxWidget13, LA_WIDGET_BORDER_NONE);
    laCheckBoxWidget_SetHAlignment(CheckBoxWidget13, LA_HALIGN_CENTER);
    laWidget_AddChild((laWidget*)ShowFrameRateButtonWidget, (laWidget*)CheckBoxWidget13);

}

static void ScreenCreate_PlayBackScreen(laScreen* screen)
{
    laLayer* layer0;
    laLayer* layer1;
    laLayer* layer2;

    laScreen_SetShowEventCallback(screen, &PlayBackScreen_ShowEvent);

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &ClearScheme);
    laLayer_SetBufferCount(layer0, 1);
    laWidget_SetOptimizationFlags((laWidget*)layer0, LA_WIDGET_OPT_DRAW_ONCE);
    laLayer_SetAlphaEnable(layer0, LA_TRUE);
    laLayer_SetAlphaAmount(layer0, 0xFF);

    laScreen_SetLayer(screen, 0, layer0);

    TouchPanelWidget = laWidget_New();
    laWidget_SetSize((laWidget*)TouchPanelWidget, 800, 480);
    laWidget_SetScheme((laWidget*)TouchPanelWidget, &ClearScheme);
    laWidget_SetBackgroundType((laWidget*)TouchPanelWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TouchPanelWidget, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, TouchPanelWidget);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer1, &BlackBackgroundScheme);
    laLayer_SetBufferCount(layer1, 1);
    laLayer_SetAlphaEnable(layer1, LA_TRUE);
    laLayer_SetAlphaAmount(layer1, 0xFF);

    laScreen_SetLayer(screen, 1, layer1);

    PlaybackMessageLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)PlaybackMessageLabel, 310, 193);
    laWidget_SetSize((laWidget*)PlaybackMessageLabel, 206, 25);
    laWidget_SetScheme((laWidget*)PlaybackMessageLabel, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)PlaybackMessageLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PlaybackMessageLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(PlaybackMessageLabel, laString_CreateFromID(string_Playing));
    laWidget_AddChild((laWidget*)layer1, (laWidget*)PlaybackMessageLabel);

    layer2 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer2, 0, 0);
    laWidget_SetSize((laWidget*)layer2, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer2, &ClearScheme);
    laLayer_SetBufferCount(layer2, 1);
    laLayer_SetAlphaEnable(layer2, LA_TRUE);
    laLayer_SetAlphaAmount(layer2, 0xFF);

    laScreen_SetLayer(screen, 2, layer2);

    PlayBackControlPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)PlayBackControlPanel, 0, 411);
    laWidget_SetSize((laWidget*)PlayBackControlPanel, 800, 69);
    laWidget_SetScheme((laWidget*)PlayBackControlPanel, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)PlayBackControlPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PlayBackControlPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer2, PlayBackControlPanel);

    PlayPauseButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)PlayPauseButtonWidget, 390, 20);
    laWidget_SetSize((laWidget*)PlayPauseButtonWidget, 40, 40);
    laWidget_SetBackgroundType((laWidget*)PlayPauseButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PlayPauseButtonWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetToggleable(PlayPauseButtonWidget, LA_TRUE);
    laButtonWidget_SetPressed(PlayPauseButtonWidget, LA_TRUE);
    laButtonWidget_SetPressedImage(PlayPauseButtonWidget, &PauseButton);
    laButtonWidget_SetReleasedImage(PlayPauseButtonWidget, &PlayButton);
    laButtonWidget_SetPressedEventCallback(PlayPauseButtonWidget, &PlayPauseButtonWidget_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(PlayPauseButtonWidget, &PlayPauseButtonWidget_ReleasedEvent);

    laWidget_AddChild((laWidget*)PlayBackControlPanel, (laWidget*)PlayPauseButtonWidget);

    FFButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)FFButtonWidget, 470, 20);
    laWidget_SetSize((laWidget*)FFButtonWidget, 40, 40);
    laWidget_SetBackgroundType((laWidget*)FFButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)FFButtonWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(FFButtonWidget, &FFButton);
    laButtonWidget_SetReleasedImage(FFButtonWidget, &FFButton);
    laButtonWidget_SetPressedEventCallback(FFButtonWidget, &FFButtonWidget_PressedEvent);

    laWidget_AddChild((laWidget*)PlayBackControlPanel, (laWidget*)FFButtonWidget);

    RewindButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)RewindButtonWidget, 310, 20);
    laWidget_SetSize((laWidget*)RewindButtonWidget, 40, 40);
    laWidget_SetBackgroundType((laWidget*)RewindButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RewindButtonWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(RewindButtonWidget, &RewindButton);
    laButtonWidget_SetReleasedImage(RewindButtonWidget, &RewindButton);
    laButtonWidget_SetPressedEventCallback(RewindButtonWidget, &RewindButtonWidget_PressedEvent);

    laWidget_AddChild((laWidget*)PlayBackControlPanel, (laWidget*)RewindButtonWidget);

    StopButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)StopButtonWidget, 610, 24);
    laWidget_SetSize((laWidget*)StopButtonWidget, 40, 40);
    laWidget_SetBackgroundType((laWidget*)StopButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)StopButtonWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(StopButtonWidget, &StopButton_Red);
    laButtonWidget_SetReleasedImage(StopButtonWidget, &StopButton_Red);
    laButtonWidget_SetReleasedEventCallback(StopButtonWidget, &StopButtonWidget_ReleasedEvent);

    laWidget_AddChild((laWidget*)PlayBackControlPanel, (laWidget*)StopButtonWidget);

    PlaybackMultiplierLabelWidget = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)PlaybackMultiplierLabelWidget, 533, 32);
    laWidget_SetSize((laWidget*)PlaybackMultiplierLabelWidget, 44, 25);
    laWidget_SetScheme((laWidget*)PlaybackMultiplierLabelWidget, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)PlaybackMultiplierLabelWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PlaybackMultiplierLabelWidget, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(PlaybackMultiplierLabelWidget, laString_CreateFromID(string_zero));
    laLabelWidget_SetHAlignment(PlaybackMultiplierLabelWidget, LA_HALIGN_RIGHT);
    laWidget_AddChild((laWidget*)PlayBackControlPanel, (laWidget*)PlaybackMultiplierLabelWidget);

    RestartButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)RestartButtonWidget, 230, 20);
    laWidget_SetSize((laWidget*)RestartButtonWidget, 40, 40);
    laWidget_SetBackgroundType((laWidget*)RestartButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RestartButtonWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(RestartButtonWidget, &RestartButton);
    laButtonWidget_SetReleasedImage(RestartButtonWidget, &RestartButton);
    laButtonWidget_SetPressedEventCallback(RestartButtonWidget, &RestartButtonWidget_PressedEvent);

    laWidget_AddChild((laWidget*)PlayBackControlPanel, (laWidget*)RestartButtonWidget);

    SliderControlFull = laSliderWidget_New();
    laWidget_SetPosition((laWidget*)SliderControlFull, 1, 0);
    laWidget_SetSize((laWidget*)SliderControlFull, 799, 15);
    laWidget_SetBackgroundType((laWidget*)SliderControlFull, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)SliderControlFull, LA_WIDGET_BORDER_LINE);
    laSliderWidget_SetOrientation(SliderControlFull, LA_SLIDER_ORIENT_HORIZONTAL, LA_FALSE);
    laSliderWidget_SetGripSize(SliderControlFull, 30);
    laSliderWidget_SetValueChangedEventCallback(SliderControlFull, &SliderControlFull_ValueChangedEvent);

    laWidget_AddChild((laWidget*)PlayBackControlPanel, (laWidget*)SliderControlFull);

    MetricsPanelWidget1 = laWidget_New();
    laWidget_SetPosition((laWidget*)MetricsPanelWidget1, 0, 429);
    laWidget_SetSize((laWidget*)MetricsPanelWidget1, 97, 50);
    laWidget_SetBackgroundType((laWidget*)MetricsPanelWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MetricsPanelWidget1, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer2, MetricsPanelWidget1);

    frameRateLabelWidget = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)frameRateLabelWidget, 0, 25);
    laWidget_SetSize((laWidget*)frameRateLabelWidget, 95, 25);
    laWidget_SetScheme((laWidget*)frameRateLabelWidget, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)frameRateLabelWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)frameRateLabelWidget, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(frameRateLabelWidget, laString_CreateFromID(string_zero));
    laLabelWidget_SetHAlignment(frameRateLabelWidget, LA_HALIGN_RIGHT);
    laWidget_AddChild((laWidget*)MetricsPanelWidget1, (laWidget*)frameRateLabelWidget);

    BandWidthLabelWidget = laLabelWidget_New();
    laWidget_SetSize((laWidget*)BandWidthLabelWidget, 95, 25);
    laWidget_SetScheme((laWidget*)BandWidthLabelWidget, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)BandWidthLabelWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)BandWidthLabelWidget, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(BandWidthLabelWidget, laString_CreateFromID(string_zero));
    laLabelWidget_SetHAlignment(BandWidthLabelWidget, LA_HALIGN_RIGHT);
    laWidget_AddChild((laWidget*)MetricsPanelWidget1, (laWidget*)BandWidthLabelWidget);

}

static void ScreenCreate_PlayBackScreenSmall(laScreen* screen)
{
    laLayer* layer0;
    laLayer* layer1;
    laLayer* layer2;

    laScreen_SetShowEventCallback(screen, &PlayBackScreenSmall_ShowEvent);

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &ClearScheme);
    laLayer_SetBufferCount(layer0, 1);
    laWidget_SetOptimizationFlags((laWidget*)layer0, LA_WIDGET_OPT_DRAW_ONCE);
    laLayer_SetAlphaEnable(layer0, LA_TRUE);
    laLayer_SetAlphaAmount(layer0, 0xFF);

    laScreen_SetLayer(screen, 0, layer0);

    TouchPanelWidget2 = laWidget_New();
    laWidget_SetPosition((laWidget*)TouchPanelWidget2, 1, 0);
    laWidget_SetSize((laWidget*)TouchPanelWidget2, 800, 480);
    laWidget_SetScheme((laWidget*)TouchPanelWidget2, &ClearScheme);
    laWidget_SetBackgroundType((laWidget*)TouchPanelWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TouchPanelWidget2, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, TouchPanelWidget2);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer1, &ClearScheme);
    laLayer_SetBufferCount(layer1, 1);
    laLayer_SetAlphaEnable(layer1, LA_TRUE);
    laLayer_SetAlphaAmount(layer1, 0xFF);

    laScreen_SetLayer(screen, 1, layer1);

    PlaybackMessageLabel2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)PlaybackMessageLabel2, 292, 179);
    laWidget_SetSize((laWidget*)PlaybackMessageLabel2, 225, 25);
    laWidget_SetScheme((laWidget*)PlaybackMessageLabel2, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)PlaybackMessageLabel2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PlaybackMessageLabel2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(PlaybackMessageLabel2, laString_CreateFromID(string_Playing));
    laWidget_AddChild((laWidget*)layer1, (laWidget*)PlaybackMessageLabel2);

    layer2 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer2, 0, 0);
    laWidget_SetSize((laWidget*)layer2, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer2, &ClearScheme);
    laLayer_SetBufferCount(layer2, 1);
    laWidget_SetOptimizationFlags((laWidget*)layer2, LA_WIDGET_OPT_DRAW_ONCE);
    laLayer_SetAlphaEnable(layer2, LA_TRUE);
    laLayer_SetAlphaAmount(layer2, 0xFF);

    laScreen_SetLayer(screen, 2, layer2);

    RightPanelWidget = laWidget_New();
    laWidget_SetPosition((laWidget*)RightPanelWidget, 646, 0);
    laWidget_SetSize((laWidget*)RightPanelWidget, 155, 480);
    laWidget_SetScheme((laWidget*)RightPanelWidget, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)RightPanelWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)RightPanelWidget, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer2, RightPanelWidget);

    StopButtonWidget2 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)StopButtonWidget2, 82, 305);
    laWidget_SetSize((laWidget*)StopButtonWidget2, 40, 40);
    laWidget_SetBackgroundType((laWidget*)StopButtonWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)StopButtonWidget2, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(StopButtonWidget2, &StopButton_Red);
    laButtonWidget_SetReleasedImage(StopButtonWidget2, &StopButton_Red);
    laButtonWidget_SetPressedEventCallback(StopButtonWidget2, &StopButtonWidget2_PressedEvent);

    laWidget_AddChild((laWidget*)RightPanelWidget, (laWidget*)StopButtonWidget2);

    FFButtonWidget2 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)FFButtonWidget2, 83, 233);
    laWidget_SetSize((laWidget*)FFButtonWidget2, 40, 40);
    laWidget_SetBackgroundType((laWidget*)FFButtonWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)FFButtonWidget2, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(FFButtonWidget2, &FFButton);
    laButtonWidget_SetReleasedImage(FFButtonWidget2, &FFButton);
    laButtonWidget_SetPressedEventCallback(FFButtonWidget2, &FFButtonWidget2_PressedEvent);

    laWidget_AddChild((laWidget*)RightPanelWidget, (laWidget*)FFButtonWidget2);

    RWButtonWidget2 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)RWButtonWidget2, 80, 99);
    laWidget_SetSize((laWidget*)RWButtonWidget2, 40, 40);
    laWidget_SetBackgroundType((laWidget*)RWButtonWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RWButtonWidget2, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(RWButtonWidget2, &RewindButton);
    laButtonWidget_SetReleasedImage(RWButtonWidget2, &RewindButton);
    laButtonWidget_SetPressedEventCallback(RWButtonWidget2, &RWButtonWidget2_PressedEvent);

    laWidget_AddChild((laWidget*)RightPanelWidget, (laWidget*)RWButtonWidget2);

    PauseButtonWidget2 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)PauseButtonWidget2, 81, 165);
    laWidget_SetSize((laWidget*)PauseButtonWidget2, 40, 40);
    laWidget_SetBackgroundType((laWidget*)PauseButtonWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PauseButtonWidget2, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetToggleable(PauseButtonWidget2, LA_TRUE);
    laButtonWidget_SetPressed(PauseButtonWidget2, LA_TRUE);
    laButtonWidget_SetPressedImage(PauseButtonWidget2, &PauseButton);
    laButtonWidget_SetReleasedImage(PauseButtonWidget2, &PlayButton);
    laButtonWidget_SetPressedEventCallback(PauseButtonWidget2, &PauseButtonWidget2_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(PauseButtonWidget2, &PauseButtonWidget2_ReleasedEvent);

    laWidget_AddChild((laWidget*)RightPanelWidget, (laWidget*)PauseButtonWidget2);

    RestartButtonWidget2 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)RestartButtonWidget2, 80, 34);
    laWidget_SetSize((laWidget*)RestartButtonWidget2, 40, 40);
    laWidget_SetBackgroundType((laWidget*)RestartButtonWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RestartButtonWidget2, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(RestartButtonWidget2, &RestartButton);
    laButtonWidget_SetReleasedImage(RestartButtonWidget2, &RestartButton);
    laButtonWidget_SetPressedEventCallback(RestartButtonWidget2, &RestartButtonWidget2_PressedEvent);

    laWidget_AddChild((laWidget*)RightPanelWidget, (laWidget*)RestartButtonWidget2);

    PlaybackMultiplierLabelWidget2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)PlaybackMultiplierLabelWidget2, 43, 368);
    laWidget_SetSize((laWidget*)PlaybackMultiplierLabelWidget2, 111, 32);
    laWidget_SetScheme((laWidget*)PlaybackMultiplierLabelWidget2, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)PlaybackMultiplierLabelWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PlaybackMultiplierLabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(PlaybackMultiplierLabelWidget2, laString_CreateFromID(string_zero));
    laWidget_AddChild((laWidget*)RightPanelWidget, (laWidget*)PlaybackMultiplierLabelWidget2);

    MetricsPanelWidget2 = laWidget_New();
    laWidget_SetPosition((laWidget*)MetricsPanelWidget2, 0, 399);
    laWidget_SetSize((laWidget*)MetricsPanelWidget2, 155, 80);
    laWidget_SetBackgroundType((laWidget*)MetricsPanelWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MetricsPanelWidget2, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)RightPanelWidget, MetricsPanelWidget2);

    BandWidthLabelWidget2 = laLabelWidget_New();
    laWidget_SetSize((laWidget*)BandWidthLabelWidget2, 154, 37);
    laWidget_SetScheme((laWidget*)BandWidthLabelWidget2, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)BandWidthLabelWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)BandWidthLabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(BandWidthLabelWidget2, laString_CreateFromID(string_zero));
    laLabelWidget_SetHAlignment(BandWidthLabelWidget2, LA_HALIGN_RIGHT);
    laWidget_AddChild((laWidget*)MetricsPanelWidget2, (laWidget*)BandWidthLabelWidget2);

    FrameRateLabelWidget2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FrameRateLabelWidget2, 0, 37);
    laWidget_SetSize((laWidget*)FrameRateLabelWidget2, 154, 44);
    laWidget_SetScheme((laWidget*)FrameRateLabelWidget2, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)FrameRateLabelWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FrameRateLabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FrameRateLabelWidget2, laString_CreateFromID(string_zero));
    laLabelWidget_SetHAlignment(FrameRateLabelWidget2, LA_HALIGN_RIGHT);
    laWidget_AddChild((laWidget*)MetricsPanelWidget2, (laWidget*)FrameRateLabelWidget2);

    SliderControlSmall = laSliderWidget_New();
    laWidget_SetPosition((laWidget*)SliderControlSmall, 28, 18);
    laWidget_SetSize((laWidget*)SliderControlSmall, 15, 381);
    laWidget_SetBackgroundType((laWidget*)SliderControlSmall, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)SliderControlSmall, LA_WIDGET_BORDER_LINE);
    laSliderWidget_SetGripSize(SliderControlSmall, 30);
    laSliderWidget_SetValueChangedEventCallback(SliderControlSmall, &SliderControlSmall_ValueChangedEvent);

    laWidget_AddChild((laWidget*)RightPanelWidget, (laWidget*)SliderControlSmall);

}



