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
laImageWidget* ImageWidget2;
laWidget* PanelWidget1;
laImageWidget* ImageWidget3;
laImageWidget* ImageWidget4;
laImageWidget* ImageWidget;
laWidget* AppLogoPanel;
laLabelWidget* LabelWidget1;
laLabelWidget* LabelWidget2;
laButtonWidget* PlaySDButton;
laButtonWidget* ButtonWidget15;
laButtonWidget* ButtonWidget12;
laButtonWidget* PlayUSBButton;
laLabelWidget* NoMediaLabelWidget;
laLineWidget* LineWidget;
laImageWidget* ImageWidget;
laLabelWidget* LabelWidget5;
laButtonWidget* ButtonWidget7;
laLabelWidget* LabelWidget8;
laLabelWidget* LabelWidget12;
laLabelWidget* LabelWidget6;
laLabelWidget* LabelWidget16;
laLabelWidget* LabelWidget;
laWidget* PanelWidget15;
laLabelWidget* LabelWidget14;
laLabelWidget* LabelWidget13;
laLabelWidget* LabelWidget7;
laLabelWidget* LabelWidget4;
laLabelWidget* LabelWidget3;
laLabelWidget* LabelWidget11;
laLabelWidget* LabelWidget10;
laLabelWidget* LabelWidget9;
laButtonWidget* BackToMenuButton;
laGroupBoxWidget* GroupBoxWidget1;
laGroupBoxWidget* GroupBoxWidget14;
laGroupBoxWidget* GroupBoxWidget2;
laListWidget* VertAlignListWidget;
laListWidget* HorzAlignListWidget;
laListWidget* ResolutionListWidget;
laListWidget* PlaySpeedListWidget;
laButtonWidget* ShowFrameRateButtonWidget;
laCheckBoxWidget* CheckBoxWidget13;
laWidget* TouchPanelWidget;
laWidget* PlayBackControlPanel;
laButtonWidget* PlayPauseButtonWidget;
laButtonWidget* FFButtonWidget;
laButtonWidget* RewindButtonWidget;
laButtonWidget* StopButtonWidget;
laLabelWidget* PlaybackMultiplierLabelWidget;
laButtonWidget* RestartButtonWidget;
laWidget* MetricsPanelWidget1;
laLabelWidget* BandWidthLabelWidget;
laLabelWidget* frameRateLabelWidget;
laSliderWidget* SliderControlFull;
laLabelWidget* PlaybackMessageLabel;
laWidget* TouchPanelWidget2;
laWidget* RightPanelWidget;
laButtonWidget* StopButtonWidget2;
laButtonWidget* FFButtonWidget2;
laButtonWidget* RWButtonWidget2;
laButtonWidget* PauseButtonWidget2;
laButtonWidget* RestartButtonWidget2;
laLabelWidget* PlaybackMessageLabel2;
laLabelWidget* PlaybackMultiplierLabelWidget2;
laWidget* MetricsPanelWidget2;
laLabelWidget* FrameRateLabelWidget2;
laLabelWidget* BandWidthLabelWidget2;
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

    laScheme_Initialize(&RedTextScheme, GFX_COLOR_MODE_RGB_565);
    RedTextScheme.base = 0xC67A;
    RedTextScheme.highlight = 0xC67A;
    RedTextScheme.highlightLight = 0xFFFF;
    RedTextScheme.shadow = 0x8410;
    RedTextScheme.shadowDark = 0x4208;
    RedTextScheme.foreground = 0x0;
    RedTextScheme.foregroundInactive = 0xD71C;
    RedTextScheme.foregroundDisabled = 0x8410;
    RedTextScheme.background = 0xFFFF;
    RedTextScheme.backgroundInactive = 0xD71C;
    RedTextScheme.backgroundDisabled = 0xC67A;
    RedTextScheme.text = 0xF800;
    RedTextScheme.textHighlight = 0xF800;
    RedTextScheme.textHighlightText = 0xFFFF;
    RedTextScheme.textInactive = 0xD71C;
    RedTextScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&StreakScheme, GFX_COLOR_MODE_RGB_565);
    StreakScheme.base = 0xC67A;
    StreakScheme.highlight = 0x1F;
    StreakScheme.highlightLight = 0xFFFF;
    StreakScheme.shadow = 0x8410;
    StreakScheme.shadowDark = 0x4208;
    StreakScheme.foreground = 0x1F;
    StreakScheme.foregroundInactive = 0xD71C;
    StreakScheme.foregroundDisabled = 0x8410;
    StreakScheme.background = 0xFFFF;
    StreakScheme.backgroundInactive = 0xD71C;
    StreakScheme.backgroundDisabled = 0xC67A;
    StreakScheme.text = 0x0;
    StreakScheme.textHighlight = 0x1F;
    StreakScheme.textHighlightText = 0xFFFF;
    StreakScheme.textInactive = 0xD71C;
    StreakScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&RedLineScheme, GFX_COLOR_MODE_RGB_565);
    RedLineScheme.base = 0xC67A;
    RedLineScheme.highlight = 0xC67A;
    RedLineScheme.highlightLight = 0xFFFF;
    RedLineScheme.shadow = 0x8410;
    RedLineScheme.shadowDark = 0x4208;
    RedLineScheme.foreground = 0xF800;
    RedLineScheme.foregroundInactive = 0xD71C;
    RedLineScheme.foregroundDisabled = 0x8410;
    RedLineScheme.background = 0xFFFF;
    RedLineScheme.backgroundInactive = 0xD71C;
    RedLineScheme.backgroundDisabled = 0xC67A;
    RedLineScheme.text = 0x0;
    RedLineScheme.textHighlight = 0x1F;
    RedLineScheme.textHighlightText = 0xFFFF;
    RedLineScheme.textInactive = 0xD71C;
    RedLineScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&BlackBackgroundScheme, GFX_COLOR_MODE_RGB_565);
    BlackBackgroundScheme.base = 0x0;
    BlackBackgroundScheme.highlight = 0x0;
    BlackBackgroundScheme.highlightLight = 0x0;
    BlackBackgroundScheme.shadow = 0x0;
    BlackBackgroundScheme.shadowDark = 0x4208;
    BlackBackgroundScheme.foreground = 0x0;
    BlackBackgroundScheme.foregroundInactive = 0x0;
    BlackBackgroundScheme.foregroundDisabled = 0x0;
    BlackBackgroundScheme.background = 0x0;
    BlackBackgroundScheme.backgroundInactive = 0x0;
    BlackBackgroundScheme.backgroundDisabled = 0x0;
    BlackBackgroundScheme.text = 0xFFFF;
    BlackBackgroundScheme.textHighlight = 0x1F;
    BlackBackgroundScheme.textHighlightText = 0x0;
    BlackBackgroundScheme.textInactive = 0x0;
    BlackBackgroundScheme.textDisabled = 0x0;

    laScheme_Initialize(&ButtonTextScheme, GFX_COLOR_MODE_RGB_565);
    ButtonTextScheme.base = 0xC67A;
    ButtonTextScheme.highlight = 0xC67A;
    ButtonTextScheme.highlightLight = 0xFFFF;
    ButtonTextScheme.shadow = 0x8410;
    ButtonTextScheme.shadowDark = 0x4208;
    ButtonTextScheme.foreground = 0x0;
    ButtonTextScheme.foregroundInactive = 0xD71C;
    ButtonTextScheme.foregroundDisabled = 0x8410;
    ButtonTextScheme.background = 0xFFFF;
    ButtonTextScheme.backgroundInactive = 0xD71C;
    ButtonTextScheme.backgroundDisabled = 0xC67A;
    ButtonTextScheme.text = 0xDEFB;
    ButtonTextScheme.textHighlight = 0x1F;
    ButtonTextScheme.textHighlightText = 0xFFFF;
    ButtonTextScheme.textInactive = 0xD71C;
    ButtonTextScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&defaultScheme, GFX_COLOR_MODE_RGB_565);
    defaultScheme.base = 0xC67A;
    defaultScheme.highlight = 0xC67A;
    defaultScheme.highlightLight = 0xFFFF;
    defaultScheme.shadow = 0x8410;
    defaultScheme.shadowDark = 0x4208;
    defaultScheme.foreground = 0x0;
    defaultScheme.foregroundInactive = 0xD71C;
    defaultScheme.foregroundDisabled = 0x8410;
    defaultScheme.background = 0xFFFF;
    defaultScheme.backgroundInactive = 0xD71C;
    defaultScheme.backgroundDisabled = 0xC67A;
    defaultScheme.text = 0x0;
    defaultScheme.textHighlight = 0x1F;
    defaultScheme.textHighlightText = 0xFFFF;
    defaultScheme.textInactive = 0xD71C;
    defaultScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&BlueTextScheme, GFX_COLOR_MODE_RGB_565);
    BlueTextScheme.base = 0xC67A;
    BlueTextScheme.highlight = 0xC67A;
    BlueTextScheme.highlightLight = 0xFFFF;
    BlueTextScheme.shadow = 0x8410;
    BlueTextScheme.shadowDark = 0x4208;
    BlueTextScheme.foreground = 0x0;
    BlueTextScheme.foregroundInactive = 0xD71C;
    BlueTextScheme.foregroundDisabled = 0x8410;
    BlueTextScheme.background = 0xFFFF;
    BlueTextScheme.backgroundInactive = 0xD71C;
    BlueTextScheme.backgroundDisabled = 0xC67A;
    BlueTextScheme.text = 0x3B57;
    BlueTextScheme.textHighlight = 0x1F;
    BlueTextScheme.textHighlightText = 0xFFFF;
    BlueTextScheme.textInactive = 0xD71C;
    BlueTextScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&whiteScheme, GFX_COLOR_MODE_RGB_565);
    whiteScheme.base = 0xFFFF;
    whiteScheme.highlight = 0xC67A;
    whiteScheme.highlightLight = 0xFFFF;
    whiteScheme.shadow = 0x8410;
    whiteScheme.shadowDark = 0x4208;
    whiteScheme.foreground = 0x0;
    whiteScheme.foregroundInactive = 0xD71C;
    whiteScheme.foregroundDisabled = 0x8410;
    whiteScheme.background = 0xFFFF;
    whiteScheme.backgroundInactive = 0xD71C;
    whiteScheme.backgroundDisabled = 0xC67A;
    whiteScheme.text = 0x0;
    whiteScheme.textHighlight = 0x1F;
    whiteScheme.textHighlightText = 0xFFFF;
    whiteScheme.textInactive = 0xD71C;
    whiteScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&PanelScheme, GFX_COLOR_MODE_RGB_565);
    PanelScheme.base = 0x0;
    PanelScheme.highlight = 0xC67A;
    PanelScheme.highlightLight = 0xFFFF;
    PanelScheme.shadow = 0x8410;
    PanelScheme.shadowDark = 0x4208;
    PanelScheme.foreground = 0x0;
    PanelScheme.foregroundInactive = 0xD71C;
    PanelScheme.foregroundDisabled = 0x8410;
    PanelScheme.background = 0x0;
    PanelScheme.backgroundInactive = 0xD71C;
    PanelScheme.backgroundDisabled = 0xC67A;
    PanelScheme.text = 0x0;
    PanelScheme.textHighlight = 0x1F;
    PanelScheme.textHighlightText = 0xFFFF;
    PanelScheme.textInactive = 0xD71C;
    PanelScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&AriaColorScheme, GFX_COLOR_MODE_RGB_565);
    AriaColorScheme.base = 0xC67A;
    AriaColorScheme.highlight = 0xC67A;
    AriaColorScheme.highlightLight = 0xFFFF;
    AriaColorScheme.shadow = 0x8410;
    AriaColorScheme.shadowDark = 0x4208;
    AriaColorScheme.foreground = 0x0;
    AriaColorScheme.foregroundInactive = 0xD71C;
    AriaColorScheme.foregroundDisabled = 0x8410;
    AriaColorScheme.background = 0xFFFF;
    AriaColorScheme.backgroundInactive = 0xD71C;
    AriaColorScheme.backgroundDisabled = 0xC67A;
    AriaColorScheme.text = 0xFC00;
    AriaColorScheme.textHighlight = 0x1F;
    AriaColorScheme.textHighlightText = 0xFFFF;
    AriaColorScheme.textInactive = 0xD71C;
    AriaColorScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&FilmBackgroundScheme, GFX_COLOR_MODE_RGB_565);
    FilmBackgroundScheme.base = 0x410;
    FilmBackgroundScheme.highlight = 0xC67A;
    FilmBackgroundScheme.highlightLight = 0xFFFF;
    FilmBackgroundScheme.shadow = 0x8410;
    FilmBackgroundScheme.shadowDark = 0x4208;
    FilmBackgroundScheme.foreground = 0x0;
    FilmBackgroundScheme.foregroundInactive = 0xD71C;
    FilmBackgroundScheme.foregroundDisabled = 0x8410;
    FilmBackgroundScheme.background = 0x947F;
    FilmBackgroundScheme.backgroundInactive = 0xD71C;
    FilmBackgroundScheme.backgroundDisabled = 0xC67A;
    FilmBackgroundScheme.text = 0x0;
    FilmBackgroundScheme.textHighlight = 0x1F;
    FilmBackgroundScheme.textHighlightText = 0xFFFF;
    FilmBackgroundScheme.textInactive = 0xD71C;
    FilmBackgroundScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&WhiteTextBlackBackgroundScheme, GFX_COLOR_MODE_RGB_565);
    WhiteTextBlackBackgroundScheme.base = 0x0;
    WhiteTextBlackBackgroundScheme.highlight = 0xC67A;
    WhiteTextBlackBackgroundScheme.highlightLight = 0xFFFF;
    WhiteTextBlackBackgroundScheme.shadow = 0x8410;
    WhiteTextBlackBackgroundScheme.shadowDark = 0x4208;
    WhiteTextBlackBackgroundScheme.foreground = 0x7E0;
    WhiteTextBlackBackgroundScheme.foregroundInactive = 0xD71C;
    WhiteTextBlackBackgroundScheme.foregroundDisabled = 0x8410;
    WhiteTextBlackBackgroundScheme.background = 0x0;
    WhiteTextBlackBackgroundScheme.backgroundInactive = 0xD71C;
    WhiteTextBlackBackgroundScheme.backgroundDisabled = 0xC67A;
    WhiteTextBlackBackgroundScheme.text = 0xFFFF;
    WhiteTextBlackBackgroundScheme.textHighlight = 0x1F;
    WhiteTextBlackBackgroundScheme.textHighlightText = 0xFFFF;
    WhiteTextBlackBackgroundScheme.textInactive = 0xD71C;
    WhiteTextBlackBackgroundScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&WhiteTextScheme, GFX_COLOR_MODE_RGB_565);
    WhiteTextScheme.base = 0xC67A;
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

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);
    laWidget_SetOptimizationFlags((laWidget*)layer0, LA_WIDGET_OPT_DRAW_ONCE);
    laLayer_SetAlphaEnable(layer0, LA_TRUE);
    laLayer_SetAlphaAmount(layer0, 0xFF);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 120, 40);
    laWidget_SetSize((laWidget*)ImageWidget1, 240, 139);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &PIC32Logo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    ImageWidget2 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget2, 120, 40);
    laWidget_SetSize((laWidget*)ImageWidget2, 240, 139);
    laWidget_SetVisible((laWidget*)ImageWidget2, LA_FALSE);
    laWidget_SetScheme((laWidget*)ImageWidget2, &whiteScheme);
    laWidget_SetBackgroundType((laWidget*)ImageWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget2, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget2, &HarmonyLogo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget2);

    PanelWidget1 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget1, 0, 207);
    laWidget_SetSize((laWidget*)PanelWidget1, 480, 65);
    laWidget_SetOptimizationFlags((laWidget*)PanelWidget1, LA_WIDGET_OPT_DRAW_ONCE);
    laWidget_SetScheme((laWidget*)PanelWidget1, &whiteScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget1, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget1);

    ImageWidget3 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget3, 480, 0);
    laWidget_SetSize((laWidget*)ImageWidget3, 480, 65);
    laWidget_SetBackgroundType((laWidget*)ImageWidget3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget3, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget3, &Bar);
    laWidget_AddChild((laWidget*)PanelWidget1, (laWidget*)ImageWidget3);

    ImageWidget4 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget4, 17, 23);
    laWidget_SetSize((laWidget*)ImageWidget4, 144, 39);
    laWidget_SetVisible((laWidget*)ImageWidget4, LA_FALSE);
    laWidget_SetOptimizationFlags((laWidget*)ImageWidget4, LA_WIDGET_OPT_LOCAL_REDRAW);
    laWidget_SetBackgroundType((laWidget*)ImageWidget4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget4, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget4, &MicrochipLogo);
    laWidget_AddChild((laWidget*)PanelWidget1, (laWidget*)ImageWidget4);

}

static void ScreenCreate_MainScreen(laScreen* screen)
{
    laLayer* layer0;

    laScreen_SetShowEventCallback(screen, &MainScreen_ShowEvent);

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &whiteScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 6, 8);
    laWidget_SetSize((laWidget*)ImageWidget, 199, 228);
    laWidget_SetScheme((laWidget*)ImageWidget, &FilmBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &VideoFilm);
    laImageWidget_SetHAlignment(ImageWidget, LA_HALIGN_RIGHT);
    laImageWidget_SetVAlignment(ImageWidget, LA_VALIGN_BOTTOM);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

    AppLogoPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)AppLogoPanel, -5, 16);
    laWidget_SetSize((laWidget*)AppLogoPanel, 196, 72);
    laWidget_SetScheme((laWidget*)AppLogoPanel, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)AppLogoPanel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)AppLogoPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)ImageWidget, AppLogoPanel);

    LabelWidget1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget1, 16, 6);
    laWidget_SetSize((laWidget*)LabelWidget1, 117, 40);
    laWidget_SetScheme((laWidget*)LabelWidget1, &AriaColorScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget1, laString_CreateFromID(string_Aria));
    laLabelWidget_SetHAlignment(LabelWidget1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)AppLogoPanel, (laWidget*)LabelWidget1);

    LabelWidget2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget2, 44, 42);
    laWidget_SetSize((laWidget*)LabelWidget2, 133, 25);
    laWidget_SetScheme((laWidget*)LabelWidget2, &RedTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget2, laString_CreateFromID(string_Player));
    laLabelWidget_SetHAlignment(LabelWidget2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)AppLogoPanel, (laWidget*)LabelWidget2);

    PlaySDButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)PlaySDButton, 341, 0);
    laWidget_SetSize((laWidget*)PlaySDButton, 139, 140);
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
    laWidget_SetPosition((laWidget*)ButtonWidget15, 341, 132);
    laWidget_SetSize((laWidget*)ButtonWidget15, 140, 140);
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
    laWidget_SetPosition((laWidget*)ButtonWidget12, 208, 130);
    laWidget_SetSize((laWidget*)ButtonWidget12, 138, 142);
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

    PlayUSBButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)PlayUSBButton, 208, 3);
    laWidget_SetSize((laWidget*)PlayUSBButton, 138, 136);
    laWidget_SetScheme((laWidget*)PlayUSBButton, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)PlayUSBButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PlayUSBButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetHAlignment(PlayUSBButton, LA_HALIGN_LEFT);
    laButtonWidget_SetPressedImage(PlayUSBButton, &USBDriveSquare);
    laButtonWidget_SetReleasedImage(PlayUSBButton, &USBDriveSquare);
    laButtonWidget_SetImageMargin(PlayUSBButton, 1);
    laButtonWidget_SetReleasedEventCallback(PlayUSBButton, &PlayUSBButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)PlayUSBButton);

    NoMediaLabelWidget = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)NoMediaLabelWidget, 242, 52);
    laWidget_SetSize((laWidget*)NoMediaLabelWidget, 199, 25);
    laWidget_SetVisible((laWidget*)NoMediaLabelWidget, LA_FALSE);
    laWidget_SetScheme((laWidget*)NoMediaLabelWidget, &RedTextScheme);
    laWidget_SetBackgroundType((laWidget*)NoMediaLabelWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)NoMediaLabelWidget, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(NoMediaLabelWidget, laString_CreateFromID(string_NoMedia));
    laLabelWidget_SetHAlignment(NoMediaLabelWidget, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)NoMediaLabelWidget);

    LineWidget = laLineWidget_New();
    laWidget_SetPosition((laWidget*)LineWidget, -114, 8);
    laWidget_SetSize((laWidget*)LineWidget, 26, 252);
    laWidget_SetScheme((laWidget*)LineWidget, &RedLineScheme);
    laWidget_SetBackgroundType((laWidget*)LineWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LineWidget, LA_WIDGET_BORDER_NONE);
    laLineWidget_SetStartPoint(LineWidget, 10, 0);
    laLineWidget_SetEndPoint(LineWidget, 10, 220);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LineWidget);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 19, 238);
    laWidget_SetSize((laWidget*)ImageWidget, 195, 35);
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
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    LabelWidget5 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget5, 15, 1);
    laWidget_SetSize((laWidget*)LabelWidget5, 370, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget5, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget5, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget5, laString_CreateFromID(string_GenerateHelp));
    laLabelWidget_SetHAlignment(LabelWidget5, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget5);

    ButtonWidget7 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget7, 432, 228);
    laWidget_SetSize((laWidget*)ButtonWidget7, 44, 40);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget7, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget7, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(ButtonWidget7, laString_CreateFromID(string_OK));
    laButtonWidget_SetReleasedEventCallback(ButtonWidget7, &ButtonWidget7_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget7);

    LabelWidget8 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget8, 15, 71);
    laWidget_SetSize((laWidget*)LabelWidget8, 446, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget8, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget8, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget8, laString_CreateFromID(string_FFMPEGParams));
    laLabelWidget_SetHAlignment(LabelWidget8, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget8);

    LabelWidget12 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget12, 15, 215);
    laWidget_SetSize((laWidget*)LabelWidget12, 401, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget12, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget12, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget12, laString_CreateFromID(string_CopyUSBorSD));
    laLabelWidget_SetHAlignment(LabelWidget12, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget12);

    LabelWidget6 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget6, 26, 23);
    laWidget_SetSize((laWidget*)LabelWidget6, 421, 25);
    laWidget_SetScheme((laWidget*)LabelWidget6, &BlueTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget6, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget6, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget6, laString_CreateFromID(string_FFMPEGCommand));
    laLabelWidget_SetHAlignment(LabelWidget6, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget6);

    LabelWidget16 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget16, 94, 48);
    laWidget_SetSize((laWidget*)LabelWidget16, 369, 25);
    laWidget_SetScheme((laWidget*)LabelWidget16, &BlueTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget16, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget16, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget16, laString_CreateFromID(string_FFMPEGCommand2));
    laLabelWidget_SetHAlignment(LabelWidget16, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget16);

    LabelWidget = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget, 15, 240);
    laWidget_SetSize((laWidget*)LabelWidget, 401, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget, laString_CreateFromID(string_InsertToBoard));
    laLabelWidget_SetHAlignment(LabelWidget, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget);

    PanelWidget15 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget15, 59, 97);
    laWidget_SetSize((laWidget*)PanelWidget15, 349, 113);
    laWidget_SetBackgroundType((laWidget*)PanelWidget15, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PanelWidget15, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget15);

    LabelWidget14 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget14, 5, 80);
    laWidget_SetSize((laWidget*)LabelWidget14, 184, 25);
    laWidget_SetScheme((laWidget*)LabelWidget14, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget14, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget14, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget14, laString_CreateFromID(string_FullScreen));
    laWidget_AddChild((laWidget*)PanelWidget15, (laWidget*)LabelWidget14);

    LabelWidget13 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget13, 5, 56);
    laWidget_SetSize((laWidget*)LabelWidget13, 184, 25);
    laWidget_SetScheme((laWidget*)LabelWidget13, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget13, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget13, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget13, laString_CreateFromID(string_VRes320x240));
    laWidget_AddChild((laWidget*)PanelWidget15, (laWidget*)LabelWidget13);

    LabelWidget7 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget7, 5, 32);
    laWidget_SetSize((laWidget*)LabelWidget7, 184, 25);
    laWidget_SetScheme((laWidget*)LabelWidget7, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget7, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget7, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget7, laString_CreateFromID(string_VRes320x180));
    laWidget_AddChild((laWidget*)PanelWidget15, (laWidget*)LabelWidget7);

    LabelWidget4 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget4, 5, 8);
    laWidget_SetSize((laWidget*)LabelWidget4, 184, 25);
    laWidget_SetScheme((laWidget*)LabelWidget4, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget4, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget4, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget4, laString_CreateFromID(string_VideoResolution));
    laWidget_AddChild((laWidget*)PanelWidget15, (laWidget*)LabelWidget4);

    LabelWidget3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget3, 188, 8);
    laWidget_SetSize((laWidget*)LabelWidget3, 155, 25);
    laWidget_SetScheme((laWidget*)LabelWidget3, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget3, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget3, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget3, laString_CreateFromID(string_Filename));
    laWidget_AddChild((laWidget*)PanelWidget15, (laWidget*)LabelWidget3);

    LabelWidget11 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget11, 188, 80);
    laWidget_SetSize((laWidget*)LabelWidget11, 155, 25);
    laWidget_SetScheme((laWidget*)LabelWidget11, &BlueTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget11, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget11, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget11, laString_CreateFromID(string_video2rgb));
    laWidget_AddChild((laWidget*)PanelWidget15, (laWidget*)LabelWidget11);

    LabelWidget10 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget10, 188, 56);
    laWidget_SetSize((laWidget*)LabelWidget10, 155, 25);
    laWidget_SetScheme((laWidget*)LabelWidget10, &BlueTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget10, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget10, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget10, laString_CreateFromID(string_video1rgb));
    laWidget_AddChild((laWidget*)PanelWidget15, (laWidget*)LabelWidget10);

    LabelWidget9 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget9, 188, 32);
    laWidget_SetSize((laWidget*)LabelWidget9, 155, 25);
    laWidget_SetScheme((laWidget*)LabelWidget9, &BlueTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget9, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget9, LA_WIDGET_BORDER_LINE);
    laLabelWidget_SetText(LabelWidget9, laString_CreateFromID(string_video0rgb));
    laWidget_AddChild((laWidget*)PanelWidget15, (laWidget*)LabelWidget9);

}

static void ScreenCreate_SettingsScreen(laScreen* screen)
{
    laLayer* layer0;

    laScreen_SetShowEventCallback(screen, &SettingsScreen_ShowEvent);

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &BlackBackgroundScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    BackToMenuButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)BackToMenuButton, 432, 229);
    laWidget_SetSize((laWidget*)BackToMenuButton, 45, 40);
    laWidget_SetScheme((laWidget*)BackToMenuButton, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)BackToMenuButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BackToMenuButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(BackToMenuButton, laString_CreateFromID(string_OK));
    laButtonWidget_SetReleasedEventCallback(BackToMenuButton, &BackToMenuButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)BackToMenuButton);

    GroupBoxWidget1 = laGroupBoxWidget_New();
    laWidget_SetPosition((laWidget*)GroupBoxWidget1, 240, 4);
    laWidget_SetSize((laWidget*)GroupBoxWidget1, 233, 149);
    laWidget_SetScheme((laWidget*)GroupBoxWidget1, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)GroupBoxWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GroupBoxWidget1, LA_WIDGET_BORDER_NONE);
    laGroupBoxWidget_SetText(GroupBoxWidget1, laString_CreateFromID(string_FramesPerSec));
    laGroupBoxWidget_SetAlignment(GroupBoxWidget1, LA_HALIGN_CENTER);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GroupBoxWidget1);

    GroupBoxWidget14 = laGroupBoxWidget_New();
    laWidget_SetPosition((laWidget*)GroupBoxWidget14, 3, 123);
    laWidget_SetSize((laWidget*)GroupBoxWidget14, 220, 136);
    laWidget_SetScheme((laWidget*)GroupBoxWidget14, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)GroupBoxWidget14, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GroupBoxWidget14, LA_WIDGET_BORDER_NONE);
    laGroupBoxWidget_SetText(GroupBoxWidget14, laString_CreateFromID(string_Alignment));
    laGroupBoxWidget_SetAlignment(GroupBoxWidget14, LA_HALIGN_CENTER);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GroupBoxWidget14);

    GroupBoxWidget2 = laGroupBoxWidget_New();
    laWidget_SetPosition((laWidget*)GroupBoxWidget2, 3, 4);
    laWidget_SetSize((laWidget*)GroupBoxWidget2, 221, 119);
    laWidget_SetScheme((laWidget*)GroupBoxWidget2, &WhiteTextScheme);
    laWidget_SetBackgroundType((laWidget*)GroupBoxWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GroupBoxWidget2, LA_WIDGET_BORDER_NONE);
    laGroupBoxWidget_SetText(GroupBoxWidget2, laString_CreateFromID(string_VideoResolution));
    laGroupBoxWidget_SetAlignment(GroupBoxWidget2, LA_HALIGN_CENTER);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GroupBoxWidget2);

    VertAlignListWidget = laListWidget_New();
    laWidget_SetPosition((laWidget*)VertAlignListWidget, 16, 153);
    laWidget_SetSize((laWidget*)VertAlignListWidget, 90, 90);
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
    laWidget_SetPosition((laWidget*)HorzAlignListWidget, 117, 153);
    laWidget_SetSize((laWidget*)HorzAlignListWidget, 90, 90);
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
    laWidget_SetPosition((laWidget*)ResolutionListWidget, 21, 29);
    laWidget_SetSize((laWidget*)ResolutionListWidget, 186, 87);
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
    laListWidget_SetItemText(ResolutionListWidget, 2, laString_CreateFromID(string_FullScreen));
    laListWidget_SetSelectedItemChangedEventCallback(ResolutionListWidget, &ResolutionListWidget_SelectionChangedEvent);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ResolutionListWidget);

    PlaySpeedListWidget = laListWidget_New();
    laWidget_SetPosition((laWidget*)PlaySpeedListWidget, 263, 29);
    laWidget_SetSize((laWidget*)PlaySpeedListWidget, 186, 113);
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
    laWidget_SetPosition((laWidget*)ShowFrameRateButtonWidget, 242, 162);
    laWidget_SetSize((laWidget*)ShowFrameRateButtonWidget, 231, 32);
    laWidget_SetScheme((laWidget*)ShowFrameRateButtonWidget, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)ShowFrameRateButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ShowFrameRateButtonWidget, LA_WIDGET_BORDER_BEVEL);
    laWidget_SetMargins((laWidget*)ShowFrameRateButtonWidget, 10, 4, 4, 4);
    laButtonWidget_SetToggleable(ShowFrameRateButtonWidget, LA_TRUE);
    laButtonWidget_SetText(ShowFrameRateButtonWidget, laString_CreateFromID(string_ShowFrameRate));
    laButtonWidget_SetHAlignment(ShowFrameRateButtonWidget, LA_HALIGN_LEFT);
    laButtonWidget_SetImagePosition(ShowFrameRateButtonWidget, LA_RELATIVE_POSITION_RIGHTOF);
    laButtonWidget_SetPressedEventCallback(ShowFrameRateButtonWidget, &ShowFrameRateButtonWidget_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(ShowFrameRateButtonWidget, &ShowFrameRateButtonWidget_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ShowFrameRateButtonWidget);

    CheckBoxWidget13 = laCheckBoxWidget_New();
    laWidget_SetPosition((laWidget*)CheckBoxWidget13, 195, 0);
    laWidget_SetSize((laWidget*)CheckBoxWidget13, 26, 32);
    laWidget_SetScheme((laWidget*)CheckBoxWidget13, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)CheckBoxWidget13, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)CheckBoxWidget13, LA_WIDGET_BORDER_NONE);
    laCheckBoxWidget_SetHAlignment(CheckBoxWidget13, LA_HALIGN_CENTER);
    laWidget_AddChild((laWidget*)ShowFrameRateButtonWidget, (laWidget*)CheckBoxWidget13);

}

static void ScreenCreate_PlayBackScreen(laScreen* screen)
{
    laLayer* layer0;

    laScreen_SetShowEventCallback(screen, &PlayBackScreen_ShowEvent);

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &BlackBackgroundScheme);
    laLayer_SetBufferCount(layer0, 1);
    laWidget_SetOptimizationFlags((laWidget*)layer0, LA_WIDGET_OPT_DRAW_ONCE);

    laScreen_SetLayer(screen, 0, layer0);

    TouchPanelWidget = laWidget_New();
    laWidget_SetSize((laWidget*)TouchPanelWidget, 480, 272);
    laWidget_SetScheme((laWidget*)TouchPanelWidget, &BlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)TouchPanelWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TouchPanelWidget, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, TouchPanelWidget);

    PlayBackControlPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)PlayBackControlPanel, 0, 212);
    laWidget_SetSize((laWidget*)PlayBackControlPanel, 480, 60);
    laWidget_SetScheme((laWidget*)PlayBackControlPanel, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)PlayBackControlPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PlayBackControlPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PlayBackControlPanel);

    PlayPauseButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)PlayPauseButtonWidget, 250, 20);
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
    laWidget_SetPosition((laWidget*)FFButtonWidget, 305, 20);
    laWidget_SetSize((laWidget*)FFButtonWidget, 40, 40);
    laWidget_SetBackgroundType((laWidget*)FFButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)FFButtonWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(FFButtonWidget, &FFButton);
    laButtonWidget_SetReleasedImage(FFButtonWidget, &FFButton);
    laButtonWidget_SetPressedEventCallback(FFButtonWidget, &FFButtonWidget_PressedEvent);

    laWidget_AddChild((laWidget*)PlayBackControlPanel, (laWidget*)FFButtonWidget);

    RewindButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)RewindButtonWidget, 196, 20);
    laWidget_SetSize((laWidget*)RewindButtonWidget, 40, 40);
    laWidget_SetBackgroundType((laWidget*)RewindButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RewindButtonWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(RewindButtonWidget, &RewindButton);
    laButtonWidget_SetReleasedImage(RewindButtonWidget, &RewindButton);
    laButtonWidget_SetPressedEventCallback(RewindButtonWidget, &RewindButtonWidget_PressedEvent);

    laWidget_AddChild((laWidget*)PlayBackControlPanel, (laWidget*)RewindButtonWidget);

    StopButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)StopButtonWidget, 435, 20);
    laWidget_SetSize((laWidget*)StopButtonWidget, 40, 40);
    laWidget_SetBackgroundType((laWidget*)StopButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)StopButtonWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(StopButtonWidget, &StopButton_Red);
    laButtonWidget_SetReleasedImage(StopButtonWidget, &StopButton_Red);
    laButtonWidget_SetReleasedEventCallback(StopButtonWidget, &StopButtonWidget_ReleasedEvent);

    laWidget_AddChild((laWidget*)PlayBackControlPanel, (laWidget*)StopButtonWidget);

    PlaybackMultiplierLabelWidget = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)PlaybackMultiplierLabelWidget, 362, 29);
    laWidget_SetSize((laWidget*)PlaybackMultiplierLabelWidget, 49, 25);
    laWidget_SetScheme((laWidget*)PlaybackMultiplierLabelWidget, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)PlaybackMultiplierLabelWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PlaybackMultiplierLabelWidget, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PlayBackControlPanel, (laWidget*)PlaybackMultiplierLabelWidget);

    RestartButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)RestartButtonWidget, 145, 20);
    laWidget_SetSize((laWidget*)RestartButtonWidget, 40, 40);
    laWidget_SetBackgroundType((laWidget*)RestartButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RestartButtonWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(RestartButtonWidget, &RestartButton);
    laButtonWidget_SetReleasedImage(RestartButtonWidget, &RestartButton);
    laButtonWidget_SetPressedEventCallback(RestartButtonWidget, &RestartButtonWidget_PressedEvent);

    laWidget_AddChild((laWidget*)PlayBackControlPanel, (laWidget*)RestartButtonWidget);

    MetricsPanelWidget1 = laWidget_New();
    laWidget_SetSize((laWidget*)MetricsPanelWidget1, 100, 50);
    laWidget_SetBackgroundType((laWidget*)MetricsPanelWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MetricsPanelWidget1, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PlayBackControlPanel, MetricsPanelWidget1);

    BandWidthLabelWidget = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)BandWidthLabelWidget, 0, 22);
    laWidget_SetSize((laWidget*)BandWidthLabelWidget, 100, 16);
    laWidget_SetScheme((laWidget*)BandWidthLabelWidget, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)BandWidthLabelWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)BandWidthLabelWidget, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(BandWidthLabelWidget, laString_CreateFromID(string_zero));
    laLabelWidget_SetHAlignment(BandWidthLabelWidget, LA_HALIGN_RIGHT);
    laWidget_AddChild((laWidget*)MetricsPanelWidget1, (laWidget*)BandWidthLabelWidget);

    frameRateLabelWidget = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)frameRateLabelWidget, 0, 39);
    laWidget_SetSize((laWidget*)frameRateLabelWidget, 100, 21);
    laWidget_SetScheme((laWidget*)frameRateLabelWidget, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)frameRateLabelWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)frameRateLabelWidget, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(frameRateLabelWidget, laString_CreateFromID(string_zero));
    laLabelWidget_SetHAlignment(frameRateLabelWidget, LA_HALIGN_RIGHT);
    laWidget_AddChild((laWidget*)MetricsPanelWidget1, (laWidget*)frameRateLabelWidget);

    SliderControlFull = laSliderWidget_New();
    laWidget_SetSize((laWidget*)SliderControlFull, 480, 15);
    laWidget_SetBackgroundType((laWidget*)SliderControlFull, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)SliderControlFull, LA_WIDGET_BORDER_LINE);
    laSliderWidget_SetOrientation(SliderControlFull, LA_SLIDER_ORIENT_HORIZONTAL, LA_FALSE);
    laSliderWidget_SetGripSize(SliderControlFull, 30);
    laSliderWidget_SetValueChangedEventCallback(SliderControlFull, &SliderControlFull_ValueChangedEvent);

    laWidget_AddChild((laWidget*)PlayBackControlPanel, (laWidget*)SliderControlFull);

    PlaybackMessageLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)PlaybackMessageLabel, 125, 90);
    laWidget_SetSize((laWidget*)PlaybackMessageLabel, 206, 25);
    laWidget_SetScheme((laWidget*)PlaybackMessageLabel, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)PlaybackMessageLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PlaybackMessageLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(PlaybackMessageLabel, laString_CreateFromID(string_Playing));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)PlaybackMessageLabel);

}

static void ScreenCreate_PlayBackScreenSmall(laScreen* screen)
{
    laLayer* layer0;

    laScreen_SetShowEventCallback(screen, &PlayBackScreenSmall_ShowEvent);

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &BlackBackgroundScheme);
    laLayer_SetBufferCount(layer0, 1);
    laWidget_SetOptimizationFlags((laWidget*)layer0, LA_WIDGET_OPT_DRAW_ONCE);

    laScreen_SetLayer(screen, 0, layer0);

    TouchPanelWidget2 = laWidget_New();
    laWidget_SetSize((laWidget*)TouchPanelWidget2, 480, 272);
    laWidget_SetScheme((laWidget*)TouchPanelWidget2, &BlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)TouchPanelWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TouchPanelWidget2, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, TouchPanelWidget2);

    RightPanelWidget = laWidget_New();
    laWidget_SetPosition((laWidget*)RightPanelWidget, 401, 0);
    laWidget_SetSize((laWidget*)RightPanelWidget, 79, 272);
    laWidget_SetScheme((laWidget*)RightPanelWidget, &PanelScheme);
    laWidget_SetBackgroundType((laWidget*)RightPanelWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)RightPanelWidget, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, RightPanelWidget);

    StopButtonWidget2 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)StopButtonWidget2, 34, 170);
    laWidget_SetSize((laWidget*)StopButtonWidget2, 40, 40);
    laWidget_SetBackgroundType((laWidget*)StopButtonWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)StopButtonWidget2, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(StopButtonWidget2, &StopButton_Red);
    laButtonWidget_SetReleasedImage(StopButtonWidget2, &StopButton_Red);
    laButtonWidget_SetPressedEventCallback(StopButtonWidget2, &StopButtonWidget2_PressedEvent);

    laWidget_AddChild((laWidget*)RightPanelWidget, (laWidget*)StopButtonWidget2);

    FFButtonWidget2 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)FFButtonWidget2, 33, 130);
    laWidget_SetSize((laWidget*)FFButtonWidget2, 40, 40);
    laWidget_SetBackgroundType((laWidget*)FFButtonWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)FFButtonWidget2, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(FFButtonWidget2, &FFButton);
    laButtonWidget_SetReleasedImage(FFButtonWidget2, &FFButton);
    laButtonWidget_SetPressedEventCallback(FFButtonWidget2, &FFButtonWidget2_PressedEvent);

    laWidget_AddChild((laWidget*)RightPanelWidget, (laWidget*)FFButtonWidget2);

    RWButtonWidget2 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)RWButtonWidget2, 33, 50);
    laWidget_SetSize((laWidget*)RWButtonWidget2, 40, 40);
    laWidget_SetBackgroundType((laWidget*)RWButtonWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RWButtonWidget2, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(RWButtonWidget2, &RewindButton);
    laButtonWidget_SetReleasedImage(RWButtonWidget2, &RewindButton);
    laButtonWidget_SetPressedEventCallback(RWButtonWidget2, &RWButtonWidget2_PressedEvent);

    laWidget_AddChild((laWidget*)RightPanelWidget, (laWidget*)RWButtonWidget2);

    PauseButtonWidget2 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)PauseButtonWidget2, 33, 90);
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
    laWidget_SetPosition((laWidget*)RestartButtonWidget2, 33, 10);
    laWidget_SetSize((laWidget*)RestartButtonWidget2, 40, 40);
    laWidget_SetBackgroundType((laWidget*)RestartButtonWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RestartButtonWidget2, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(RestartButtonWidget2, &RestartButton);
    laButtonWidget_SetReleasedImage(RestartButtonWidget2, &RestartButton);
    laButtonWidget_SetPressedEventCallback(RestartButtonWidget2, &RestartButtonWidget2_PressedEvent);

    laWidget_AddChild((laWidget*)RightPanelWidget, (laWidget*)RestartButtonWidget2);

    PlaybackMessageLabel2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)PlaybackMessageLabel2, -286, 90);
    laWidget_SetSize((laWidget*)PlaybackMessageLabel2, 225, 25);
    laWidget_SetScheme((laWidget*)PlaybackMessageLabel2, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)PlaybackMessageLabel2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PlaybackMessageLabel2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(PlaybackMessageLabel2, laString_CreateFromID(string_Playing));
    laWidget_AddChild((laWidget*)RightPanelWidget, (laWidget*)PlaybackMessageLabel2);

    PlaybackMultiplierLabelWidget2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)PlaybackMultiplierLabelWidget2, 0, 210);
    laWidget_SetSize((laWidget*)PlaybackMultiplierLabelWidget2, 79, 20);
    laWidget_SetVisible((laWidget*)PlaybackMultiplierLabelWidget2, LA_FALSE);
    laWidget_SetScheme((laWidget*)PlaybackMultiplierLabelWidget2, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)PlaybackMultiplierLabelWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PlaybackMultiplierLabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(PlaybackMultiplierLabelWidget2, laString_CreateFromID(string_zero));
    laWidget_AddChild((laWidget*)RightPanelWidget, (laWidget*)PlaybackMultiplierLabelWidget2);

    MetricsPanelWidget2 = laWidget_New();
    laWidget_SetPosition((laWidget*)MetricsPanelWidget2, 0, 230);
    laWidget_SetSize((laWidget*)MetricsPanelWidget2, 79, 43);
    laWidget_SetBackgroundType((laWidget*)MetricsPanelWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MetricsPanelWidget2, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)RightPanelWidget, MetricsPanelWidget2);

    FrameRateLabelWidget2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)FrameRateLabelWidget2, 14, 22);
    laWidget_SetSize((laWidget*)FrameRateLabelWidget2, 65, 20);
    laWidget_SetScheme((laWidget*)FrameRateLabelWidget2, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)FrameRateLabelWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)FrameRateLabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(FrameRateLabelWidget2, laString_CreateFromID(string_zero));
    laLabelWidget_SetHAlignment(FrameRateLabelWidget2, LA_HALIGN_RIGHT);
    laWidget_AddChild((laWidget*)MetricsPanelWidget2, (laWidget*)FrameRateLabelWidget2);

    BandWidthLabelWidget2 = laLabelWidget_New();
    laWidget_SetSize((laWidget*)BandWidthLabelWidget2, 79, 20);
    laWidget_SetScheme((laWidget*)BandWidthLabelWidget2, &WhiteTextBlackBackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)BandWidthLabelWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)BandWidthLabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(BandWidthLabelWidget2, laString_CreateFromID(string_zero));
    laLabelWidget_SetHAlignment(BandWidthLabelWidget2, LA_HALIGN_RIGHT);
    laWidget_AddChild((laWidget*)MetricsPanelWidget2, (laWidget*)BandWidthLabelWidget2);

    SliderControlSmall = laSliderWidget_New();
    laWidget_SetPosition((laWidget*)SliderControlSmall, 7, 0);
    laWidget_SetSize((laWidget*)SliderControlSmall, 15, 205);
    laWidget_SetBackgroundType((laWidget*)SliderControlSmall, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)SliderControlSmall, LA_WIDGET_BORDER_LINE);
    laSliderWidget_SetGripSize(SliderControlSmall, 30);
    laSliderWidget_SetValueChangedEventCallback(SliderControlSmall, &SliderControlSmall_ValueChangedEvent);

    laWidget_AddChild((laWidget*)RightPanelWidget, (laWidget*)SliderControlSmall);

}



