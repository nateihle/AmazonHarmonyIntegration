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

laScheme defaultScheme;
laScheme redScheme;
laScheme yellowScheme;
laScheme ClearScheme;
laScheme greenScheme;
laImageWidget* ImageWidget1;
laImageWidget* ImageWidget3;
laImageWidget* ImageWidget2;
laImageWidget* ImageWidget4;
laWidget* BackPanel;
laImageWidget* BackImage0;
laImageWidget* BackImage1;
laImageWidget* BackImage2;
laWidget* MiddlePanel;
laImageWidget* MiddleImage0;
laImageWidget* MiddleImage1;
laImageWidget* MiddleImage2;
laWidget* FrontPanel;
laImageWidget* FrontImage0;
laImageWidget* FrontImage1;
laImageWidget* FrontImage2;
laButtonWidget* MicrochipLogoWidget;
laCircularGaugeWidget* CircularGaugeWidget;
laWidget* SpriteAnchor;
laImageSequenceWidget* RunRightSequence;
laImageSequenceWidget* RunLeftSequence;
laImageSequenceWidget* IdleRightSequence;
laImageSequenceWidget* IdleLeftSequence;
laImageSequenceWidget* BlazeRightSequence;
laImageSequenceWidget* BlazeLeftSequence;
laImageSequenceWidget* DizzySequence;
laImageSequenceWidget* DizzyLeftSequence;
laImageSequenceWidget* HurtSequence;
laImageSequenceWidget* HurtLeftSequence;
laImageSequenceWidget* FallSequence;
laImageSequenceWidget* FallLeftSequence;
laWidget* TopScoreAnchor;
laLabelWidget* TopScoreLabel;
laLabelWidget* TopDigit5;
laLabelWidget* TopDigit4;
laLabelWidget* TopDigit3;
laLabelWidget* TopDigit2;
laLabelWidget* TopDigit1;
laLabelWidget* TopDigit0;
laWidget* ScoreAnchor;
laLabelWidget* ScoreLabel;
laLabelWidget* Digit5;
laLabelWidget* Digit4;
laLabelWidget* Digit3;
laLabelWidget* Digit2;
laLabelWidget* Digit1;
laLabelWidget* Digit0;
laWidget* GPUUsageAnchor;
laLabelWidget* GPUUsageLabel;
laLabelWidget* GPUScore6;
laLabelWidget* GPUScore5;
laLabelWidget* GPUScore4;
laLabelWidget* GPUScore3;
laLabelWidget* GPUScore2;
laLabelWidget* PixelLabel;
laLabelWidget* GPUScore1;
laLabelWidget* GPUScore0;
laButtonWidget* ButtonWidget1;
laButtonWidget* ButtonWidget0;
laImageWidget* InfoPageHarmonyLogo;
laImageWidget* TouchUpDownImage;
laImageWidget* ImageWidget;
laWidget* InfoTextDragPanel;
laImageWidget* InfoTextImage;
laWidget* InfoTitlePanelWidget;
laLabelWidget* TextTitle;
laButtonWidget* ReturnToMainButton;
laButtonWidget* EasterEggButton;


static void ScreenCreate_SplashScreen(laScreen* screen);
static void ScreenCreate_MainScreen(laScreen* screen);
static void ScreenCreate_InfoScreen(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&defaultScheme, GFX_COLOR_MODE_RGBA_8888);
    defaultScheme.base = 0x0;
    defaultScheme.highlight = 0xC8D0D4FF;
    defaultScheme.highlightLight = 0xFFFFFFFF;
    defaultScheme.shadow = 0x808080FF;
    defaultScheme.shadowDark = 0x404040FF;
    defaultScheme.foreground = 0xFF;
    defaultScheme.foregroundInactive = 0xD6E3E7FF;
    defaultScheme.foregroundDisabled = 0x808080FF;
    defaultScheme.background = 0x0;
    defaultScheme.backgroundInactive = 0xD6E3E7FF;
    defaultScheme.backgroundDisabled = 0xC8D0D4FF;
    defaultScheme.text = 0xFF;
    defaultScheme.textHighlight = 0xFFFF;
    defaultScheme.textHighlightText = 0xFFFFFFFF;
    defaultScheme.textInactive = 0xD6E3E7FF;
    defaultScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&redScheme, GFX_COLOR_MODE_RGBA_8888);
    redScheme.base = 0xC8D0D4FF;
    redScheme.highlight = 0xC8D0D4FF;
    redScheme.highlightLight = 0xFFFFFFFF;
    redScheme.shadow = 0x808080FF;
    redScheme.shadowDark = 0x404040FF;
    redScheme.foreground = 0xFF0000FF;
    redScheme.foregroundInactive = 0xD6E3E7FF;
    redScheme.foregroundDisabled = 0x808080FF;
    redScheme.background = 0xFFFFFFFF;
    redScheme.backgroundInactive = 0xD6E3E7FF;
    redScheme.backgroundDisabled = 0xC8D0D4FF;
    redScheme.text = 0xFF0000FF;
    redScheme.textHighlight = 0xFFFF;
    redScheme.textHighlightText = 0xFFFFFFFF;
    redScheme.textInactive = 0xD6E3E7FF;
    redScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&yellowScheme, GFX_COLOR_MODE_RGBA_8888);
    yellowScheme.base = 0xC8D0D4FF;
    yellowScheme.highlight = 0xC8D0D4FF;
    yellowScheme.highlightLight = 0xFFFFFFFF;
    yellowScheme.shadow = 0x808080FF;
    yellowScheme.shadowDark = 0x404040FF;
    yellowScheme.foreground = 0xFFFF00FF;
    yellowScheme.foregroundInactive = 0xD6E3E7FF;
    yellowScheme.foregroundDisabled = 0x808080FF;
    yellowScheme.background = 0xFFFFFFFF;
    yellowScheme.backgroundInactive = 0xD6E3E7FF;
    yellowScheme.backgroundDisabled = 0xC8D0D4FF;
    yellowScheme.text = 0xFFFF00FF;
    yellowScheme.textHighlight = 0xFFFF;
    yellowScheme.textHighlightText = 0xFFFFFFFF;
    yellowScheme.textInactive = 0xD6E3E7FF;
    yellowScheme.textDisabled = 0x8C9294FF;

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

    laScheme_Initialize(&greenScheme, GFX_COLOR_MODE_RGBA_8888);
    greenScheme.base = 0xC8D0D4FF;
    greenScheme.highlight = 0xC8D0D4FF;
    greenScheme.highlightLight = 0xFFFFFFFF;
    greenScheme.shadow = 0x808080FF;
    greenScheme.shadowDark = 0x404040FF;
    greenScheme.foreground = 0xFF00FF;
    greenScheme.foregroundInactive = 0xD6E3E7FF;
    greenScheme.foregroundDisabled = 0x808080FF;
    greenScheme.background = 0xFFFFFFFF;
    greenScheme.backgroundInactive = 0xD6E3E7FF;
    greenScheme.backgroundDisabled = 0xC8D0D4FF;
    greenScheme.text = 0xFF00FF;
    greenScheme.textHighlight = 0xFFFF;
    greenScheme.textHighlightText = 0xFFFFFFFF;
    greenScheme.textInactive = 0xD6E3E7FF;
    greenScheme.textDisabled = 0x8C9294FF;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_SplashScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_MainScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_InfoScreen);
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
    laLayer_SetBufferCount(layer0, 2);
    laLayer_SetAlphaEnable(layer0, LA_TRUE);
    laLayer_SetAlphaAmount(layer0, 0xFF);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 221, 145);
    laWidget_SetSize((laWidget*)ImageWidget1, 372, 96);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &PIC32Logo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_NONE);
    laLayer_SetBufferCount(layer1, 2);
    laLayer_SetAlphaEnable(layer1, LA_TRUE);
    laLayer_SetAlphaAmount(layer1, 0xFF);
    laLayer_SetVSync(layer1, LA_FALSE);

    laScreen_SetLayer(screen, 1, layer1);

    ImageWidget3 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget3, 800, 391);
    laWidget_SetSize((laWidget*)ImageWidget3, 800, 91);
    laWidget_SetBackgroundType((laWidget*)ImageWidget3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget3, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget3, &Bar);
    laWidget_AddChild((laWidget*)layer1, (laWidget*)ImageWidget3);

    layer2 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer2, 0, 0);
    laWidget_SetSize((laWidget*)layer2, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer2, LA_WIDGET_BACKGROUND_NONE);
    laLayer_SetBufferCount(layer2, 2);
    laLayer_SetAlphaEnable(layer2, LA_TRUE);
    laLayer_SetAlphaAmount(layer2, 0x0);
    laLayer_SetVSync(layer2, LA_FALSE);

    laScreen_SetLayer(screen, 2, layer2);

    ImageWidget2 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget2, 249, 89);
    laWidget_SetSize((laWidget*)ImageWidget2, 307, 214);
    laWidget_SetBackgroundType((laWidget*)ImageWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget2, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget2, &HarmonyLogo);
    laWidget_AddChild((laWidget*)layer2, (laWidget*)ImageWidget2);

    ImageWidget4 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget4, 18, 413);
    laWidget_SetSize((laWidget*)ImageWidget4, 202, 55);
    laWidget_SetBackgroundType((laWidget*)ImageWidget4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget4, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget4, &MicrochipLogo);
    laWidget_AddChild((laWidget*)layer2, (laWidget*)ImageWidget4);

}

static void ScreenCreate_MainScreen(laScreen* screen)
{
    laLayer* layer0;
    laLayer* layer1;
    laLayer* layer2;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 2);

    laScreen_SetLayer(screen, 0, layer0);

    BackPanel = laWidget_New();
    laWidget_SetSize((laWidget*)BackPanel, 1440, 240);
    laWidget_SetBackgroundType((laWidget*)BackPanel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BackPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, BackPanel);

    BackImage0 = laImageWidget_New();
    laWidget_SetSize((laWidget*)BackImage0, 640, 240);
    laWidget_SetBackgroundType((laWidget*)BackImage0, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BackImage0, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(BackImage0, &BackLayer);
    laWidget_AddChild((laWidget*)BackPanel, (laWidget*)BackImage0);

    BackImage1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)BackImage1, 640, 0);
    laWidget_SetSize((laWidget*)BackImage1, 640, 240);
    laWidget_SetBackgroundType((laWidget*)BackImage1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BackImage1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(BackImage1, &BackLayer);
    laWidget_AddChild((laWidget*)BackPanel, (laWidget*)BackImage1);

    BackImage2 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)BackImage2, 1280, 0);
    laWidget_SetSize((laWidget*)BackImage2, 640, 240);
    laWidget_SetBackgroundType((laWidget*)BackImage2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BackImage2, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(BackImage2, &BackLayer);
    laWidget_AddChild((laWidget*)BackPanel, (laWidget*)BackImage2);

    MiddlePanel = laWidget_New();
    laWidget_SetPosition((laWidget*)MiddlePanel, 0, 80);
    laWidget_SetSize((laWidget*)MiddlePanel, 1440, 400);
    laWidget_SetBackgroundType((laWidget*)MiddlePanel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MiddlePanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, MiddlePanel);

    MiddleImage0 = laImageWidget_New();
    laWidget_SetSize((laWidget*)MiddleImage0, 640, 400);
    laWidget_SetBackgroundType((laWidget*)MiddleImage0, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MiddleImage0, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(MiddleImage0, &MiddleLayer);
    laImageWidget_SetVAlignment(MiddleImage0, LA_VALIGN_BOTTOM);
    laWidget_AddChild((laWidget*)MiddlePanel, (laWidget*)MiddleImage0);

    MiddleImage1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)MiddleImage1, 640, 0);
    laWidget_SetSize((laWidget*)MiddleImage1, 640, 400);
    laWidget_SetBackgroundType((laWidget*)MiddleImage1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MiddleImage1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(MiddleImage1, &MiddleLayer);
    laImageWidget_SetVAlignment(MiddleImage1, LA_VALIGN_BOTTOM);
    laWidget_AddChild((laWidget*)MiddlePanel, (laWidget*)MiddleImage1);

    MiddleImage2 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)MiddleImage2, 1280, 0);
    laWidget_SetSize((laWidget*)MiddleImage2, 640, 400);
    laWidget_SetBackgroundType((laWidget*)MiddleImage2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MiddleImage2, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(MiddleImage2, &MiddleLayer);
    laImageWidget_SetVAlignment(MiddleImage2, LA_VALIGN_BOTTOM);
    laWidget_AddChild((laWidget*)MiddlePanel, (laWidget*)MiddleImage2);

    FrontPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)FrontPanel, 0, 300);
    laWidget_SetSize((laWidget*)FrontPanel, 1440, 180);
    laWidget_SetBackgroundType((laWidget*)FrontPanel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)FrontPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, FrontPanel);

    FrontImage0 = laImageWidget_New();
    laWidget_SetSize((laWidget*)FrontImage0, 640, 180);
    laWidget_SetBackgroundType((laWidget*)FrontImage0, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)FrontImage0, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(FrontImage0, &FrontLayer);
    laWidget_AddChild((laWidget*)FrontPanel, (laWidget*)FrontImage0);

    FrontImage1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)FrontImage1, 640, 0);
    laWidget_SetSize((laWidget*)FrontImage1, 640, 180);
    laWidget_SetBackgroundType((laWidget*)FrontImage1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)FrontImage1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(FrontImage1, &FrontLayer);
    laWidget_AddChild((laWidget*)FrontPanel, (laWidget*)FrontImage1);

    FrontImage2 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)FrontImage2, 1280, 0);
    laWidget_SetSize((laWidget*)FrontImage2, 640, 180);
    laWidget_SetBackgroundType((laWidget*)FrontImage2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)FrontImage2, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(FrontImage2, &FrontLayer);
    laWidget_AddChild((laWidget*)FrontPanel, (laWidget*)FrontImage2);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer1, &ClearScheme);
    laLayer_SetBufferCount(layer1, 2);

    laScreen_SetLayer(screen, 1, layer1);

    MicrochipLogoWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)MicrochipLogoWidget, 10, 10);
    laWidget_SetSize((laWidget*)MicrochipLogoWidget, 181, 42);
    laWidget_SetBackgroundType((laWidget*)MicrochipLogoWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MicrochipLogoWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(MicrochipLogoWidget, &mchp_logo);
    laButtonWidget_SetReleasedImage(MicrochipLogoWidget, &mchp_logo);
    laButtonWidget_SetReleasedEventCallback(MicrochipLogoWidget, &MicrochipLogoWidget_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer1, (laWidget*)MicrochipLogoWidget);

    CircularGaugeWidget = laCircularGaugeWidget_New();
    laWidget_SetPosition((laWidget*)CircularGaugeWidget, 620, 0);
    laWidget_SetSize((laWidget*)CircularGaugeWidget, 180, 180);
    laWidget_SetBackgroundType((laWidget*)CircularGaugeWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CircularGaugeWidget, LA_WIDGET_BORDER_NONE);
    laCircularGaugeWidget_SetRadius(CircularGaugeWidget, 80);
    laCircularGaugeWidget_SetStartAngle(CircularGaugeWidget, -180);
    laCircularGaugeWidget_SetCenterAngle(CircularGaugeWidget, -180);
    laCircularGaugeWidget_SetValue(CircularGaugeWidget, 100);
    laCircularGaugeWidget_SetEndValue(CircularGaugeWidget, 100);
    laCircularGaugeWidget_SetTicksVisible(CircularGaugeWidget, LA_FALSE);
    laCircularGaugeWidget_SetTickValue(CircularGaugeWidget, 5);
    laCircularGaugeWidget_SetTickLabelsVisible(CircularGaugeWidget, LA_FALSE);
    laCircularGaugeWidget_SetTicksLabelsStringID(CircularGaugeWidget, string_gauge_numbers);
    laCircularGaugeWidget_SetStringTable(CircularGaugeWidget, &stringTable);
    laCircularGaugeWidget_SetHandRadius(CircularGaugeWidget, 55);
    laCircularGaugeWidget_SetCenterCircleThickness(CircularGaugeWidget, 3);
    laCircularGaugeWidget_AddAngularArc(CircularGaugeWidget, 160, 180, 80, 15, &redScheme);
    laCircularGaugeWidget_AddAngularArc(CircularGaugeWidget, 125, 160, 80, 15, &yellowScheme);
    laCircularGaugeWidget_AddAngularArc(CircularGaugeWidget, 0, 125, 80, 15, &greenScheme);
    laCircularGaugeWidget_AddAngularArc(CircularGaugeWidget, 180, 0, 66, 2, &defaultScheme);
    laCircularGaugeWidget_AddAngularArc(CircularGaugeWidget, 159, 161, 80, 15, &defaultScheme);
    laCircularGaugeWidget_AddAngularArc(CircularGaugeWidget, 124, 126, 80, 15, &defaultScheme);
    laCircularGaugeWidget_AddAngularArc(CircularGaugeWidget, 178, 180, 80, 15, &defaultScheme);
    laCircularGaugeWidget_AddAngularArc(CircularGaugeWidget, 0, 2, 80, 15, &defaultScheme);
    laWidget_AddChild((laWidget*)layer1, (laWidget*)CircularGaugeWidget);

    SpriteAnchor = laWidget_New();
    laWidget_SetPosition((laWidget*)SpriteAnchor, 250, 130);
    laWidget_SetSize((laWidget*)SpriteAnchor, 320, 320);
    laWidget_SetBackgroundType((laWidget*)SpriteAnchor, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)SpriteAnchor, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer1, SpriteAnchor);

    RunRightSequence = laImageSequenceWidget_New();
    laWidget_SetPosition((laWidget*)RunRightSequence, 0, 160);
    laWidget_SetSize((laWidget*)RunRightSequence, 320, 160);
    laWidget_SetVisible((laWidget*)RunRightSequence, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)RunRightSequence, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RunRightSequence, LA_WIDGET_BORDER_NONE);
    laImageSequenceWidget_SetImageCount(RunRightSequence, 3);
    laImageSequenceWidget_SetImage(RunRightSequence, 0, &Run0);
    laImageSequenceWidget_SetImage(RunRightSequence, 1, &Run1);
    laImageSequenceWidget_SetImage(RunRightSequence, 2, &Run2);
    laImageSequenceWidget_SetRepeat(RunRightSequence, LA_TRUE);
    laWidget_AddChild((laWidget*)SpriteAnchor, (laWidget*)RunRightSequence);

    RunLeftSequence = laImageSequenceWidget_New();
    laWidget_SetPosition((laWidget*)RunLeftSequence, 0, 160);
    laWidget_SetSize((laWidget*)RunLeftSequence, 320, 160);
    laWidget_SetVisible((laWidget*)RunLeftSequence, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)RunLeftSequence, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RunLeftSequence, LA_WIDGET_BORDER_NONE);
    laImageSequenceWidget_SetImageCount(RunLeftSequence, 3);
    laImageSequenceWidget_SetImage(RunLeftSequence, 0, &RunToLeft0);
    laImageSequenceWidget_SetImage(RunLeftSequence, 1, &RunToLeft1);
    laImageSequenceWidget_SetImage(RunLeftSequence, 2, &RunToLeft2);
    laImageSequenceWidget_SetRepeat(RunLeftSequence, LA_TRUE);
    laWidget_AddChild((laWidget*)SpriteAnchor, (laWidget*)RunLeftSequence);

    IdleRightSequence = laImageSequenceWidget_New();
    laWidget_SetPosition((laWidget*)IdleRightSequence, 0, 160);
    laWidget_SetSize((laWidget*)IdleRightSequence, 320, 160);
    laWidget_SetBackgroundType((laWidget*)IdleRightSequence, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)IdleRightSequence, LA_WIDGET_BORDER_NONE);
    laImageSequenceWidget_SetImageCount(IdleRightSequence, 4);
    laImageSequenceWidget_SetImage(IdleRightSequence, 0, &Idle0);
    laImageSequenceWidget_SetImage(IdleRightSequence, 1, &Idle1);
    laImageSequenceWidget_SetImage(IdleRightSequence, 2, &Idle2);
    laImageSequenceWidget_SetImage(IdleRightSequence, 3, &Idle3);
    laImageSequenceWidget_SetRepeat(IdleRightSequence, LA_TRUE);
    laWidget_AddChild((laWidget*)SpriteAnchor, (laWidget*)IdleRightSequence);

    IdleLeftSequence = laImageSequenceWidget_New();
    laWidget_SetPosition((laWidget*)IdleLeftSequence, 0, 160);
    laWidget_SetSize((laWidget*)IdleLeftSequence, 320, 160);
    laWidget_SetVisible((laWidget*)IdleLeftSequence, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)IdleLeftSequence, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)IdleLeftSequence, LA_WIDGET_BORDER_NONE);
    laImageSequenceWidget_SetImageCount(IdleLeftSequence, 4);
    laImageSequenceWidget_SetImage(IdleLeftSequence, 0, &IdleLeft0);
    laImageSequenceWidget_SetImage(IdleLeftSequence, 1, &IdleLeft1);
    laImageSequenceWidget_SetImage(IdleLeftSequence, 2, &IdleLeft2);
    laImageSequenceWidget_SetImage(IdleLeftSequence, 3, &IdleLeft3);
    laImageSequenceWidget_SetRepeat(IdleLeftSequence, LA_TRUE);
    laWidget_AddChild((laWidget*)SpriteAnchor, (laWidget*)IdleLeftSequence);

    BlazeRightSequence = laImageSequenceWidget_New();
    laWidget_SetPosition((laWidget*)BlazeRightSequence, 0, 160);
    laWidget_SetSize((laWidget*)BlazeRightSequence, 320, 160);
    laWidget_SetVisible((laWidget*)BlazeRightSequence, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)BlazeRightSequence, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BlazeRightSequence, LA_WIDGET_BORDER_NONE);
    laImageSequenceWidget_SetImageCount(BlazeRightSequence, 3);
    laImageSequenceWidget_SetImage(BlazeRightSequence, 0, &Blaze0);
    laImageSequenceWidget_SetImage(BlazeRightSequence, 1, &Blaze1);
    laImageSequenceWidget_SetImage(BlazeRightSequence, 2, &Blaze2);
    laImageSequenceWidget_SetRepeat(BlazeRightSequence, LA_TRUE);
    laWidget_AddChild((laWidget*)SpriteAnchor, (laWidget*)BlazeRightSequence);

    BlazeLeftSequence = laImageSequenceWidget_New();
    laWidget_SetPosition((laWidget*)BlazeLeftSequence, 0, 160);
    laWidget_SetSize((laWidget*)BlazeLeftSequence, 320, 160);
    laWidget_SetVisible((laWidget*)BlazeLeftSequence, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)BlazeLeftSequence, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BlazeLeftSequence, LA_WIDGET_BORDER_NONE);
    laImageSequenceWidget_SetImageCount(BlazeLeftSequence, 3);
    laImageSequenceWidget_SetImage(BlazeLeftSequence, 0, &BlazeLeft0);
    laImageSequenceWidget_SetImage(BlazeLeftSequence, 1, &BlazeLeft1);
    laImageSequenceWidget_SetImage(BlazeLeftSequence, 2, &BlazeLeft2);
    laImageSequenceWidget_SetRepeat(BlazeLeftSequence, LA_TRUE);
    laWidget_AddChild((laWidget*)SpriteAnchor, (laWidget*)BlazeLeftSequence);

    DizzySequence = laImageSequenceWidget_New();
    laWidget_SetPosition((laWidget*)DizzySequence, 0, 160);
    laWidget_SetSize((laWidget*)DizzySequence, 320, 160);
    laWidget_SetVisible((laWidget*)DizzySequence, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)DizzySequence, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)DizzySequence, LA_WIDGET_BORDER_NONE);
    laImageSequenceWidget_SetImageCount(DizzySequence, 3);
    laImageSequenceWidget_SetImage(DizzySequence, 0, &Dizzy0);
    laImageSequenceWidget_SetImage(DizzySequence, 1, &Dizzy1);
    laImageSequenceWidget_SetImage(DizzySequence, 2, &Dizzy2);
    laImageSequenceWidget_SetRepeat(DizzySequence, LA_TRUE);
    laWidget_AddChild((laWidget*)SpriteAnchor, (laWidget*)DizzySequence);

    DizzyLeftSequence = laImageSequenceWidget_New();
    laWidget_SetPosition((laWidget*)DizzyLeftSequence, 0, 160);
    laWidget_SetSize((laWidget*)DizzyLeftSequence, 320, 160);
    laWidget_SetVisible((laWidget*)DizzyLeftSequence, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)DizzyLeftSequence, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)DizzyLeftSequence, LA_WIDGET_BORDER_NONE);
    laImageSequenceWidget_SetImageCount(DizzyLeftSequence, 3);
    laImageSequenceWidget_SetImage(DizzyLeftSequence, 0, &DizzyLeft0);
    laImageSequenceWidget_SetImage(DizzyLeftSequence, 1, &DizzyLeft1);
    laImageSequenceWidget_SetImage(DizzyLeftSequence, 2, &DizzyLeft2);
    laImageSequenceWidget_SetRepeat(DizzyLeftSequence, LA_TRUE);
    laWidget_AddChild((laWidget*)SpriteAnchor, (laWidget*)DizzyLeftSequence);

    HurtSequence = laImageSequenceWidget_New();
    laWidget_SetPosition((laWidget*)HurtSequence, 0, 160);
    laWidget_SetSize((laWidget*)HurtSequence, 320, 160);
    laWidget_SetVisible((laWidget*)HurtSequence, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)HurtSequence, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)HurtSequence, LA_WIDGET_BORDER_NONE);
    laImageSequenceWidget_SetImageCount(HurtSequence, 4);
    laImageSequenceWidget_SetImage(HurtSequence, 0, &Hurt0);
    laImageSequenceWidget_SetImage(HurtSequence, 1, &Hurt1);
    laImageSequenceWidget_SetImage(HurtSequence, 2, &Hurt2);
    laImageSequenceWidget_SetImage(HurtSequence, 3, &Hurt3);
    laImageSequenceWidget_SetRepeat(HurtSequence, LA_TRUE);
    laWidget_AddChild((laWidget*)SpriteAnchor, (laWidget*)HurtSequence);

    HurtLeftSequence = laImageSequenceWidget_New();
    laWidget_SetPosition((laWidget*)HurtLeftSequence, 0, 160);
    laWidget_SetSize((laWidget*)HurtLeftSequence, 320, 160);
    laWidget_SetVisible((laWidget*)HurtLeftSequence, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)HurtLeftSequence, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)HurtLeftSequence, LA_WIDGET_BORDER_NONE);
    laImageSequenceWidget_SetImageCount(HurtLeftSequence, 4);
    laImageSequenceWidget_SetImage(HurtLeftSequence, 0, &HurtLeft0);
    laImageSequenceWidget_SetImage(HurtLeftSequence, 1, &HurtLeft1);
    laImageSequenceWidget_SetImage(HurtLeftSequence, 2, &HurtLeft2);
    laImageSequenceWidget_SetImage(HurtLeftSequence, 3, &HurtLeft3);
    laImageSequenceWidget_SetRepeat(HurtLeftSequence, LA_TRUE);
    laWidget_AddChild((laWidget*)SpriteAnchor, (laWidget*)HurtLeftSequence);

    FallSequence = laImageSequenceWidget_New();
    laWidget_SetPosition((laWidget*)FallSequence, 0, 160);
    laWidget_SetSize((laWidget*)FallSequence, 320, 160);
    laWidget_SetVisible((laWidget*)FallSequence, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)FallSequence, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)FallSequence, LA_WIDGET_BORDER_NONE);
    laImageSequenceWidget_SetImageCount(FallSequence, 5);
    laImageSequenceWidget_SetImage(FallSequence, 0, &Fall0);
    laImageSequenceWidget_SetImage(FallSequence, 1, &Fall1);
    laImageSequenceWidget_SetImage(FallSequence, 2, &Fall2);
    laImageSequenceWidget_SetImage(FallSequence, 3, &Fall3);
    laImageSequenceWidget_SetImage(FallSequence, 4, &Fall4);
    laWidget_AddChild((laWidget*)SpriteAnchor, (laWidget*)FallSequence);

    FallLeftSequence = laImageSequenceWidget_New();
    laWidget_SetPosition((laWidget*)FallLeftSequence, 0, 160);
    laWidget_SetSize((laWidget*)FallLeftSequence, 320, 160);
    laWidget_SetVisible((laWidget*)FallLeftSequence, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)FallLeftSequence, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)FallLeftSequence, LA_WIDGET_BORDER_NONE);
    laImageSequenceWidget_SetImageCount(FallLeftSequence, 5);
    laImageSequenceWidget_SetImage(FallLeftSequence, 0, &FallLeft0);
    laImageSequenceWidget_SetImage(FallLeftSequence, 1, &FallLeft1);
    laImageSequenceWidget_SetImage(FallLeftSequence, 2, &FallLeft2);
    laImageSequenceWidget_SetImage(FallLeftSequence, 3, &FallLeft3);
    laImageSequenceWidget_SetImage(FallLeftSequence, 4, &FallLeft4);
    laWidget_AddChild((laWidget*)SpriteAnchor, (laWidget*)FallLeftSequence);

    TopScoreAnchor = laWidget_New();
    laWidget_SetPosition((laWidget*)TopScoreAnchor, 320, 10);
    laWidget_SetSize((laWidget*)TopScoreAnchor, 292, 40);
    laWidget_SetBackgroundType((laWidget*)TopScoreAnchor, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TopScoreAnchor, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer1, TopScoreAnchor);

    TopScoreLabel = laLabelWidget_New();
    laWidget_SetSize((laWidget*)TopScoreLabel, 160, 40);
    laWidget_SetBackgroundType((laWidget*)TopScoreLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TopScoreLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TopScoreLabel, laString_CreateFromID(string_TopScore));
    laLabelWidget_SetHAlignment(TopScoreLabel, LA_HALIGN_RIGHT);
    laWidget_AddChild((laWidget*)TopScoreAnchor, (laWidget*)TopScoreLabel);

    TopDigit5 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TopDigit5, 160, 0);
    laWidget_SetSize((laWidget*)TopDigit5, 22, 40);
    laWidget_SetBackgroundType((laWidget*)TopDigit5, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TopDigit5, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TopDigit5, laString_CreateFromID(string_Zero));
    laLabelWidget_SetHAlignment(TopDigit5, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TopScoreAnchor, (laWidget*)TopDigit5);

    TopDigit4 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TopDigit4, 182, 0);
    laWidget_SetSize((laWidget*)TopDigit4, 22, 40);
    laWidget_SetBackgroundType((laWidget*)TopDigit4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TopDigit4, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TopDigit4, laString_CreateFromID(string_One));
    laLabelWidget_SetHAlignment(TopDigit4, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TopScoreAnchor, (laWidget*)TopDigit4);

    TopDigit3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TopDigit3, 204, 0);
    laWidget_SetSize((laWidget*)TopDigit3, 22, 40);
    laWidget_SetBackgroundType((laWidget*)TopDigit3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TopDigit3, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TopDigit3, laString_CreateFromID(string_Zero));
    laLabelWidget_SetHAlignment(TopDigit3, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TopScoreAnchor, (laWidget*)TopDigit3);

    TopDigit2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TopDigit2, 226, 0);
    laWidget_SetSize((laWidget*)TopDigit2, 22, 40);
    laWidget_SetBackgroundType((laWidget*)TopDigit2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TopDigit2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TopDigit2, laString_CreateFromID(string_Zero));
    laLabelWidget_SetHAlignment(TopDigit2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TopScoreAnchor, (laWidget*)TopDigit2);

    TopDigit1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TopDigit1, 248, 0);
    laWidget_SetSize((laWidget*)TopDigit1, 22, 40);
    laWidget_SetBackgroundType((laWidget*)TopDigit1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TopDigit1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TopDigit1, laString_CreateFromID(string_Zero));
    laLabelWidget_SetHAlignment(TopDigit1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TopScoreAnchor, (laWidget*)TopDigit1);

    TopDigit0 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TopDigit0, 270, 0);
    laWidget_SetSize((laWidget*)TopDigit0, 22, 40);
    laWidget_SetBackgroundType((laWidget*)TopDigit0, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TopDigit0, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TopDigit0, laString_CreateFromID(string_Zero));
    laLabelWidget_SetHAlignment(TopDigit0, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TopScoreAnchor, (laWidget*)TopDigit0);

    ScoreAnchor = laWidget_New();
    laWidget_SetPosition((laWidget*)ScoreAnchor, 320, 50);
    laWidget_SetSize((laWidget*)ScoreAnchor, 292, 40);
    laWidget_SetBackgroundType((laWidget*)ScoreAnchor, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ScoreAnchor, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer1, ScoreAnchor);

    ScoreLabel = laLabelWidget_New();
    laWidget_SetSize((laWidget*)ScoreLabel, 160, 40);
    laWidget_SetBackgroundType((laWidget*)ScoreLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ScoreLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(ScoreLabel, laString_CreateFromID(string_Score));
    laLabelWidget_SetHAlignment(ScoreLabel, LA_HALIGN_RIGHT);
    laWidget_AddChild((laWidget*)ScoreAnchor, (laWidget*)ScoreLabel);

    Digit5 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit5, 160, 0);
    laWidget_SetSize((laWidget*)Digit5, 22, 40);
    laWidget_SetBackgroundType((laWidget*)Digit5, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit5, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit5, laString_CreateFromID(string_Zero));
    laLabelWidget_SetHAlignment(Digit5, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ScoreAnchor, (laWidget*)Digit5);

    Digit4 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit4, 182, 0);
    laWidget_SetSize((laWidget*)Digit4, 22, 40);
    laWidget_SetBackgroundType((laWidget*)Digit4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit4, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit4, laString_CreateFromID(string_Zero));
    laLabelWidget_SetHAlignment(Digit4, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ScoreAnchor, (laWidget*)Digit4);

    Digit3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit3, 204, 0);
    laWidget_SetSize((laWidget*)Digit3, 22, 40);
    laWidget_SetBackgroundType((laWidget*)Digit3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit3, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit3, laString_CreateFromID(string_Zero));
    laLabelWidget_SetHAlignment(Digit3, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ScoreAnchor, (laWidget*)Digit3);

    Digit2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit2, 226, 0);
    laWidget_SetSize((laWidget*)Digit2, 22, 40);
    laWidget_SetBackgroundType((laWidget*)Digit2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit2, laString_CreateFromID(string_Zero));
    laLabelWidget_SetHAlignment(Digit2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ScoreAnchor, (laWidget*)Digit2);

    Digit1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit1, 248, 0);
    laWidget_SetSize((laWidget*)Digit1, 22, 40);
    laWidget_SetBackgroundType((laWidget*)Digit1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit1, laString_CreateFromID(string_Zero));
    laLabelWidget_SetHAlignment(Digit1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ScoreAnchor, (laWidget*)Digit1);

    Digit0 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit0, 270, 0);
    laWidget_SetSize((laWidget*)Digit0, 22, 40);
    laWidget_SetBackgroundType((laWidget*)Digit0, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit0, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit0, laString_CreateFromID(string_Zero));
    laLabelWidget_SetHAlignment(Digit0, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ScoreAnchor, (laWidget*)Digit0);

    GPUUsageAnchor = laWidget_New();
    laWidget_SetPosition((laWidget*)GPUUsageAnchor, 0, 450);
    laWidget_SetSize((laWidget*)GPUUsageAnchor, 300, 30);
    laWidget_SetEnabled((laWidget*)GPUUsageAnchor, LA_FALSE);
    laWidget_SetVisible((laWidget*)GPUUsageAnchor, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)GPUUsageAnchor, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GPUUsageAnchor, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer1, GPUUsageAnchor);

    GPUUsageLabel = laLabelWidget_New();
    laWidget_SetSize((laWidget*)GPUUsageLabel, 125, 30);
    laWidget_SetBackgroundType((laWidget*)GPUUsageLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GPUUsageLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GPUUsageLabel, laString_CreateFromID(string_GPUThroughput));
    laLabelWidget_SetHAlignment(GPUUsageLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GPUUsageAnchor, (laWidget*)GPUUsageLabel);

    GPUScore6 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GPUScore6, 130, 0);
    laWidget_SetSize((laWidget*)GPUScore6, 12, 30);
    laWidget_SetBackgroundType((laWidget*)GPUScore6, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GPUScore6, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GPUScore6, laString_CreateFromID(string_ZeroSmall));
    laWidget_AddChild((laWidget*)GPUUsageAnchor, (laWidget*)GPUScore6);

    GPUScore5 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GPUScore5, 142, 0);
    laWidget_SetSize((laWidget*)GPUScore5, 12, 30);
    laWidget_SetBackgroundType((laWidget*)GPUScore5, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GPUScore5, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GPUScore5, laString_CreateFromID(string_ZeroSmall));
    laWidget_AddChild((laWidget*)GPUUsageAnchor, (laWidget*)GPUScore5);

    GPUScore4 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GPUScore4, 154, 0);
    laWidget_SetSize((laWidget*)GPUScore4, 12, 30);
    laWidget_SetBackgroundType((laWidget*)GPUScore4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GPUScore4, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GPUScore4, laString_CreateFromID(string_ZeroSmall));
    laWidget_AddChild((laWidget*)GPUUsageAnchor, (laWidget*)GPUScore4);

    GPUScore3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GPUScore3, 166, 0);
    laWidget_SetSize((laWidget*)GPUScore3, 12, 30);
    laWidget_SetBackgroundType((laWidget*)GPUScore3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GPUScore3, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GPUScore3, laString_CreateFromID(string_ZeroSmall));
    laWidget_AddChild((laWidget*)GPUUsageAnchor, (laWidget*)GPUScore3);

    GPUScore2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GPUScore2, 178, 0);
    laWidget_SetSize((laWidget*)GPUScore2, 12, 30);
    laWidget_SetBackgroundType((laWidget*)GPUScore2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GPUScore2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GPUScore2, laString_CreateFromID(string_ZeroSmall));
    laWidget_AddChild((laWidget*)GPUUsageAnchor, (laWidget*)GPUScore2);

    PixelLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)PixelLabel, 214, 0);
    laWidget_SetSize((laWidget*)PixelLabel, 50, 30);
    laWidget_SetBackgroundType((laWidget*)PixelLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PixelLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(PixelLabel, laString_CreateFromID(string_Pixels));
    laLabelWidget_SetHAlignment(PixelLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GPUUsageAnchor, (laWidget*)PixelLabel);

    GPUScore1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GPUScore1, 190, 0);
    laWidget_SetSize((laWidget*)GPUScore1, 12, 30);
    laWidget_SetBackgroundType((laWidget*)GPUScore1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GPUScore1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GPUScore1, laString_CreateFromID(string_ZeroSmall));
    laWidget_AddChild((laWidget*)GPUUsageAnchor, (laWidget*)GPUScore1);

    GPUScore0 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GPUScore0, 202, 0);
    laWidget_SetSize((laWidget*)GPUScore0, 12, 30);
    laWidget_SetBackgroundType((laWidget*)GPUScore0, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GPUScore0, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GPUScore0, laString_CreateFromID(string_ZeroSmall));
    laWidget_AddChild((laWidget*)GPUUsageAnchor, (laWidget*)GPUScore0);

    layer2 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer2, 0, 0);
    laWidget_SetSize((laWidget*)layer2, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer2, &ClearScheme);
    laLayer_SetBufferCount(layer2, 2);
    laLayer_SetAlphaEnable(layer2, LA_TRUE);
    laLayer_SetAlphaAmount(layer2, 0x9B);

    laScreen_SetLayer(screen, 2, layer2);

    ButtonWidget1 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget1, 400, 80);
    laWidget_SetSize((laWidget*)ButtonWidget1, 400, 400);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget1, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetHAlignment(ButtonWidget1, LA_HALIGN_RIGHT);
    laButtonWidget_SetPressedImage(ButtonWidget1, &right_bw);
    laButtonWidget_SetReleasedImage(ButtonWidget1, &right_bw);
    laButtonWidget_SetPressedEventCallback(ButtonWidget1, &ButtonWidget1_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget1, &ButtonWidget1_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer2, (laWidget*)ButtonWidget1);

    ButtonWidget0 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget0, 0, 80);
    laWidget_SetSize((laWidget*)ButtonWidget0, 400, 400);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget0, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget0, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetHAlignment(ButtonWidget0, LA_HALIGN_LEFT);
    laButtonWidget_SetPressedImage(ButtonWidget0, &left_bw);
    laButtonWidget_SetReleasedImage(ButtonWidget0, &left_bw);
    laButtonWidget_SetPressedEventCallback(ButtonWidget0, &ButtonWidget0_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget0, &ButtonWidget0_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer2, (laWidget*)ButtonWidget0);

}

static void ScreenCreate_InfoScreen(laScreen* screen)
{
    laLayer* layer0;
    laLayer* layer1;
    laLayer* layer2;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 2);

    laScreen_SetLayer(screen, 0, layer0);

    InfoPageHarmonyLogo = laImageWidget_New();
    laWidget_SetSize((laWidget*)InfoPageHarmonyLogo, 800, 480);
    laWidget_SetBackgroundType((laWidget*)InfoPageHarmonyLogo, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)InfoPageHarmonyLogo, LA_WIDGET_BORDER_NONE);
    laWidget_SetAlphaEnable((laWidget*)InfoPageHarmonyLogo, GFX_TRUE);
    laWidget_SetAlphaAmount((laWidget*)InfoPageHarmonyLogo, 128);
    laImageWidget_SetImage(InfoPageHarmonyLogo, &HarmonyLogo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)InfoPageHarmonyLogo);

    TouchUpDownImage = laImageWidget_New();
    laWidget_SetPosition((laWidget*)TouchUpDownImage, 701, 210);
    laWidget_SetSize((laWidget*)TouchUpDownImage, 60, 72);
    laWidget_SetBackgroundType((laWidget*)TouchUpDownImage, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TouchUpDownImage, LA_WIDGET_BORDER_NONE);
    laWidget_SetAlphaEnable((laWidget*)TouchUpDownImage, GFX_TRUE);
    laWidget_SetAlphaAmount((laWidget*)TouchUpDownImage, 128);
    laImageWidget_SetImage(TouchUpDownImage, &vertical_touch);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)TouchUpDownImage);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 21, 62);
    laWidget_SetSize((laWidget*)ImageWidget, 100, 100);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer1, &ClearScheme);
    laLayer_SetBufferCount(layer1, 2);

    laScreen_SetLayer(screen, 1, layer1);

    InfoTextDragPanel = laWidget_New();
    laWidget_SetSize((laWidget*)InfoTextDragPanel, 800, 650);
    laWidget_SetScheme((laWidget*)InfoTextDragPanel, &ClearScheme);
    laWidget_SetBackgroundType((laWidget*)InfoTextDragPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)InfoTextDragPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer1, InfoTextDragPanel);

    InfoTextImage = laImageWidget_New();
    laWidget_SetPosition((laWidget*)InfoTextImage, 1, 51);
    laWidget_SetSize((laWidget*)InfoTextImage, 709, 560);
    laWidget_SetScheme((laWidget*)InfoTextImage, &ClearScheme);
    laWidget_SetBackgroundType((laWidget*)InfoTextImage, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)InfoTextImage, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(InfoTextImage, &info_text);
    laImageWidget_SetHAlignment(InfoTextImage, LA_HALIGN_LEFT);
    laImageWidget_SetVAlignment(InfoTextImage, LA_VALIGN_TOP);
    laWidget_AddChild((laWidget*)InfoTextDragPanel, (laWidget*)InfoTextImage);

    InfoTitlePanelWidget = laWidget_New();
    laWidget_SetSize((laWidget*)InfoTitlePanelWidget, 800, 50);
    laWidget_SetScheme((laWidget*)InfoTitlePanelWidget, &ClearScheme);
    laWidget_SetBackgroundType((laWidget*)InfoTitlePanelWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)InfoTitlePanelWidget, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer1, InfoTitlePanelWidget);

    TextTitle = laLabelWidget_New();
    laWidget_SetSize((laWidget*)TextTitle, 790, 50);
    laWidget_SetBackgroundType((laWidget*)TextTitle, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TextTitle, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TextTitle, laString_CreateFromID(string_InfoTitle));
    laLabelWidget_SetHAlignment(TextTitle, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)InfoTitlePanelWidget, (laWidget*)TextTitle);

    layer2 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer2, 0, 0);
    laWidget_SetSize((laWidget*)layer2, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer2, &ClearScheme);
    laLayer_SetBufferCount(layer2, 2);

    laScreen_SetLayer(screen, 2, layer2);

    ReturnToMainButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ReturnToMainButton, 692, 380);
    laWidget_SetSize((laWidget*)ReturnToMainButton, 90, 90);
    laWidget_SetBackgroundType((laWidget*)ReturnToMainButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ReturnToMainButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ReturnToMainButton, &HomeButton);
    laButtonWidget_SetReleasedImage(ReturnToMainButton, &HomeButton);
    laButtonWidget_SetReleasedEventCallback(ReturnToMainButton, &ReturnToMainButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer2, (laWidget*)ReturnToMainButton);

    EasterEggButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)EasterEggButton, 693, 20);
    laWidget_SetSize((laWidget*)EasterEggButton, 90, 90);
    laWidget_SetBackgroundType((laWidget*)EasterEggButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)EasterEggButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(EasterEggButton, &microchip_button);
    laButtonWidget_SetReleasedImage(EasterEggButton, &microchip_button);
    laButtonWidget_SetReleasedEventCallback(EasterEggButton, &EasterEggButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer2, (laWidget*)EasterEggButton);

}



