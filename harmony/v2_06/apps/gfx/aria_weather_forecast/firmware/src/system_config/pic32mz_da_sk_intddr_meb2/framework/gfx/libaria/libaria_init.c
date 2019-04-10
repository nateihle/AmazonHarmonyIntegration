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

laScheme Red;
laScheme MyScheme;
laScheme Blue;
laScheme Yellow;
laScheme Purple;
laScheme ClearScheme;
laScheme GrayScheme;
laScheme TitleBarScheme;
laScheme TextBoxScheme;
laScheme Green;
laScheme TextWhite;
laScheme GradientScheme;
laImageWidget* ImageWidget1;
laImageWidget* ImageWidget3;
laImageWidget* ImageWidget2;
laImageWidget* ImageWidget4;
laImageWidget* ImageWidget5;
laLabelWidget* TimeLabel;
laLabelWidget* LabelWidget2;
laImageSequenceWidget* ImageSequenceWidget1;
laLabelWidget* MinuteLabel;
laButtonWidget* ButtonWidget2;
laLabelWidget* LabelWidget5;
laWidget* PanelWidget2;
laWidget* PanelWidget6;
laWidget* PanelWidget1;
laWidget* PanelWidget5;
laButtonWidget* ButtonWidget1;
laImageWidget* CloudIcon;
laImageWidget* ImageWidget5;
laLabelWidget* LabelWidget1;
laLabelWidget* LabelWidget3;
laLabelWidget* LabelWidget11;
laLabelWidget* LabelWidget12;
laImageWidget* ImageWidget5;
laLabelWidget* LabelWidget6;
laLabelWidget* LabelWidget8;
laLabelWidget* LabelWidget9;
laLabelWidget* LabelWidget10;


static void ScreenCreate_SplashScreen(laScreen* screen);
static void ScreenCreate_MainScreen(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&Red, GFX_COLOR_MODE_RGBA_8888);
    Red.base = 0x3161FFFF;
    Red.highlight = 0xC8D0D4FF;
    Red.highlightLight = 0xFFFFFFFF;
    Red.shadow = 0x808080FF;
    Red.shadowDark = 0x404040FF;
    Red.foreground = 0xFF;
    Red.foregroundInactive = 0x808080FF;
    Red.foregroundDisabled = 0x808080FF;
    Red.background = 0xFFFFFFFF;
    Red.backgroundInactive = 0xC8D0D4FF;
    Red.backgroundDisabled = 0xC8D0D4FF;
    Red.text = 0xFF;
    Red.textHighlight = 0xFFFF;
    Red.textHighlightText = 0xFFFFFFFF;
    Red.textInactive = 0xC8D0D4FF;
    Red.textDisabled = 0xC8D0D4FF;

    laScheme_Initialize(&MyScheme, GFX_COLOR_MODE_RGBA_8888);
    MyScheme.base = 0xC8D0D4FF;
    MyScheme.highlight = 0xC8D0D4FF;
    MyScheme.highlightLight = 0xFFFFFFFF;
    MyScheme.shadow = 0x808080FF;
    MyScheme.shadowDark = 0x404040FF;
    MyScheme.foreground = 0xD63DCEFF;
    MyScheme.foregroundInactive = 0x808080FF;
    MyScheme.foregroundDisabled = 0x808080FF;
    MyScheme.background = 0xFFFFFFFF;
    MyScheme.backgroundInactive = 0xC8D0D4FF;
    MyScheme.backgroundDisabled = 0xC8D0D4FF;
    MyScheme.text = 0xFF;
    MyScheme.textHighlight = 0xFFFF;
    MyScheme.textHighlightText = 0xFFFFFFFF;
    MyScheme.textInactive = 0xC8D0D4FF;
    MyScheme.textDisabled = 0xC8D0D4FF;

    laScheme_Initialize(&Blue, GFX_COLOR_MODE_RGBA_8888);
    Blue.base = 0xC0FFFFFF;
    Blue.highlight = 0xC8D0D4FF;
    Blue.highlightLight = 0xFFFFFFFF;
    Blue.shadow = 0x808080FF;
    Blue.shadowDark = 0x404040FF;
    Blue.foreground = 0xFF;
    Blue.foregroundInactive = 0x808080FF;
    Blue.foregroundDisabled = 0x808080FF;
    Blue.background = 0xFFFFFFFF;
    Blue.backgroundInactive = 0xC8D0D4FF;
    Blue.backgroundDisabled = 0xC8D0D4FF;
    Blue.text = 0xFF;
    Blue.textHighlight = 0xFFFF;
    Blue.textHighlightText = 0xFFFFFFFF;
    Blue.textInactive = 0xC8D0D4FF;
    Blue.textDisabled = 0xC8D0D4FF;

    laScheme_Initialize(&Yellow, GFX_COLOR_MODE_RGBA_8888);
    Yellow.base = 0xFFFF00FF;
    Yellow.highlight = 0xC8D0D4FF;
    Yellow.highlightLight = 0xFFFFFFFF;
    Yellow.shadow = 0x808080FF;
    Yellow.shadowDark = 0x404040FF;
    Yellow.foreground = 0xFF;
    Yellow.foregroundInactive = 0x808080FF;
    Yellow.foregroundDisabled = 0x808080FF;
    Yellow.background = 0xFFFFFFFF;
    Yellow.backgroundInactive = 0xC8D0D4FF;
    Yellow.backgroundDisabled = 0xC8D0D4FF;
    Yellow.text = 0xFF;
    Yellow.textHighlight = 0xFFFF;
    Yellow.textHighlightText = 0xFFFFFFFF;
    Yellow.textInactive = 0xC8D0D4FF;
    Yellow.textDisabled = 0xC8D0D4FF;

    laScheme_Initialize(&Purple, GFX_COLOR_MODE_RGBA_8888);
    Purple.base = 0xC8D0D4FF;
    Purple.highlight = 0xC8D0D4FF;
    Purple.highlightLight = 0xFFFFFFFF;
    Purple.shadow = 0x808080FF;
    Purple.shadowDark = 0x404040FF;
    Purple.foreground = 0xFF0000FF;
    Purple.foregroundInactive = 0xFFFF;
    Purple.foregroundDisabled = 0x808080FF;
    Purple.background = 0xFF00FF;
    Purple.backgroundInactive = 0xFFFF00FF;
    Purple.backgroundDisabled = 0xC8D0D4FF;
    Purple.text = 0xFF;
    Purple.textHighlight = 0xFFFF;
    Purple.textHighlightText = 0xFFFFFFFF;
    Purple.textInactive = 0xC8D0D4FF;
    Purple.textDisabled = 0xC8D0D4FF;

    laScheme_Initialize(&ClearScheme, GFX_COLOR_MODE_RGBA_8888);
    ClearScheme.base = 0xFFFFFF00;
    ClearScheme.highlight = 0xC8D0D4FF;
    ClearScheme.highlightLight = 0xFFFFFFFF;
    ClearScheme.shadow = 0x808080FF;
    ClearScheme.shadowDark = 0x404040FF;
    ClearScheme.foreground = 0xFF;
    ClearScheme.foregroundInactive = 0x808080FF;
    ClearScheme.foregroundDisabled = 0x808080FF;
    ClearScheme.background = 0xFFFFFFFF;
    ClearScheme.backgroundInactive = 0xC8D0D4FF;
    ClearScheme.backgroundDisabled = 0xC8D0D4FF;
    ClearScheme.text = 0x736D73FF;
    ClearScheme.textHighlight = 0xFFFF;
    ClearScheme.textHighlightText = 0xFFFFFFFF;
    ClearScheme.textInactive = 0xC8D0D4FF;
    ClearScheme.textDisabled = 0xC8D0D4FF;

    laScheme_Initialize(&GrayScheme, GFX_COLOR_MODE_RGBA_8888);
    GrayScheme.base = 0xD6DFE6FF;
    GrayScheme.highlight = 0xC8D0D4FF;
    GrayScheme.highlightLight = 0xFFFFFFFF;
    GrayScheme.shadow = 0x808080FF;
    GrayScheme.shadowDark = 0x404040FF;
    GrayScheme.foreground = 0xFF;
    GrayScheme.foregroundInactive = 0x808080FF;
    GrayScheme.foregroundDisabled = 0x808080FF;
    GrayScheme.background = 0xFFFFFFFF;
    GrayScheme.backgroundInactive = 0xC8D0D4FF;
    GrayScheme.backgroundDisabled = 0xC8D0D4FF;
    GrayScheme.text = 0x736D73FF;
    GrayScheme.textHighlight = 0xFFFF;
    GrayScheme.textHighlightText = 0xFFFFFFFF;
    GrayScheme.textInactive = 0xC8D0D4FF;
    GrayScheme.textDisabled = 0xC8D0D4FF;

    laScheme_Initialize(&TitleBarScheme, GFX_COLOR_MODE_RGBA_8888);
    TitleBarScheme.base = 0xEFF7FFFF;
    TitleBarScheme.highlight = 0xC8D0D4FF;
    TitleBarScheme.highlightLight = 0xFFFFFFFF;
    TitleBarScheme.shadow = 0x808080FF;
    TitleBarScheme.shadowDark = 0x404040FF;
    TitleBarScheme.foreground = 0xFF;
    TitleBarScheme.foregroundInactive = 0x808080FF;
    TitleBarScheme.foregroundDisabled = 0x808080FF;
    TitleBarScheme.background = 0xFFFFFFFF;
    TitleBarScheme.backgroundInactive = 0xC8D0D4FF;
    TitleBarScheme.backgroundDisabled = 0xC8D0D4FF;
    TitleBarScheme.text = 0xFF;
    TitleBarScheme.textHighlight = 0xFFFF;
    TitleBarScheme.textHighlightText = 0xFFFFFFFF;
    TitleBarScheme.textInactive = 0xC8D0D4FF;
    TitleBarScheme.textDisabled = 0xC8D0D4FF;

    laScheme_Initialize(&TextBoxScheme, GFX_COLOR_MODE_RGBA_8888);
    TextBoxScheme.base = 0xEFEFEFFF;
    TextBoxScheme.highlight = 0xC8D0D4FF;
    TextBoxScheme.highlightLight = 0xFFFFFFFF;
    TextBoxScheme.shadow = 0x808080FF;
    TextBoxScheme.shadowDark = 0x404040FF;
    TextBoxScheme.foreground = 0xEFEFEFFF;
    TextBoxScheme.foregroundInactive = 0xDEDFDEFF;
    TextBoxScheme.foregroundDisabled = 0x808080FF;
    TextBoxScheme.background = 0xFFFFFFFF;
    TextBoxScheme.backgroundInactive = 0xC8D0D4FF;
    TextBoxScheme.backgroundDisabled = 0xC8D0D4FF;
    TextBoxScheme.text = 0xFF;
    TextBoxScheme.textHighlight = 0xFFFF;
    TextBoxScheme.textHighlightText = 0xFFFFFFFF;
    TextBoxScheme.textInactive = 0xC8D0D4FF;
    TextBoxScheme.textDisabled = 0xC8D0D4FF;

    laScheme_Initialize(&Green, GFX_COLOR_MODE_RGBA_8888);
    Green.base = 0xFFC0C0FF;
    Green.highlight = 0xC8D0D4FF;
    Green.highlightLight = 0xFFFFFFFF;
    Green.shadow = 0x808080FF;
    Green.shadowDark = 0x404040FF;
    Green.foreground = 0xFF;
    Green.foregroundInactive = 0x808080FF;
    Green.foregroundDisabled = 0x808080FF;
    Green.background = 0xFFFFFFFF;
    Green.backgroundInactive = 0xC8D0D4FF;
    Green.backgroundDisabled = 0xC8D0D4FF;
    Green.text = 0x5296A5FF;
    Green.textHighlight = 0xFFFF;
    Green.textHighlightText = 0xFFFFFFFF;
    Green.textInactive = 0xC8D0D4FF;
    Green.textDisabled = 0xC8D0D4FF;

    laScheme_Initialize(&TextWhite, GFX_COLOR_MODE_RGBA_8888);
    TextWhite.base = 0xC8D0D4FF;
    TextWhite.highlight = 0xC8D0D4FF;
    TextWhite.highlightLight = 0xFFFFFFFF;
    TextWhite.shadow = 0x808080FF;
    TextWhite.shadowDark = 0x404040FF;
    TextWhite.foreground = 0xFF;
    TextWhite.foregroundInactive = 0x808080FF;
    TextWhite.foregroundDisabled = 0x808080FF;
    TextWhite.background = 0xFFFFFFFF;
    TextWhite.backgroundInactive = 0xC8D0D4FF;
    TextWhite.backgroundDisabled = 0xC8D0D4FF;
    TextWhite.text = 0xFFFFFFFF;
    TextWhite.textHighlight = 0xFFFF;
    TextWhite.textHighlightText = 0xFFFFFFFF;
    TextWhite.textInactive = 0xC8D0D4FF;
    TextWhite.textDisabled = 0xC8D0D4FF;

    laScheme_Initialize(&GradientScheme, GFX_COLOR_MODE_RGBA_8888);
    GradientScheme.base = 0xC8D0D4FF;
    GradientScheme.highlight = 0xC8D0D4FF;
    GradientScheme.highlightLight = 0xFFFFFFFF;
    GradientScheme.shadow = 0x808080FF;
    GradientScheme.shadowDark = 0x404040FF;
    GradientScheme.foreground = 0xFF;
    GradientScheme.foregroundInactive = 0x808080FF;
    GradientScheme.foregroundDisabled = 0x808080FF;
    GradientScheme.background = 0xFFFFFFFF;
    GradientScheme.backgroundInactive = 0xC8D0D4FF;
    GradientScheme.backgroundDisabled = 0xC8D0D4FF;
    GradientScheme.text = 0xFF;
    GradientScheme.textHighlight = 0xFFFF;
    GradientScheme.textHighlightText = 0xFFFFFFFF;
    GradientScheme.textInactive = 0xC8D0D4FF;
    GradientScheme.textDisabled = 0xC8D0D4FF;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_SplashScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_MainScreen);
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
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 2);
    laLayer_SetAlphaEnable(layer0, LA_TRUE);
    laLayer_SetAlphaAmount(layer0, 0xFF);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 120, 89);
    laWidget_SetSize((laWidget*)ImageWidget1, 240, 62);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &PIC32Logo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_NONE);
    laLayer_SetBufferCount(layer1, 2);
    laLayer_SetAlphaEnable(layer1, LA_TRUE);
    laLayer_SetAlphaAmount(layer1, 0xFF);
    laLayer_SetVSync(layer1, LA_FALSE);

    laScreen_SetLayer(screen, 1, layer1);

    ImageWidget3 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget3, 480, 207);
    laWidget_SetSize((laWidget*)ImageWidget3, 480, 65);
    laWidget_SetBackgroundType((laWidget*)ImageWidget3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget3, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget3, &Bar);
    laWidget_AddChild((laWidget*)layer1, (laWidget*)ImageWidget3);

    layer2 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer2, 0, 0);
    laWidget_SetSize((laWidget*)layer2, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer2, LA_WIDGET_BACKGROUND_NONE);
    laLayer_SetBufferCount(layer2, 2);
    laLayer_SetAlphaEnable(layer2, LA_TRUE);
    laLayer_SetAlphaAmount(layer2, 0x0);
    laLayer_SetVSync(layer2, LA_FALSE);

    laScreen_SetLayer(screen, 2, layer2);

    ImageWidget2 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget2, 144, 42);
    laWidget_SetSize((laWidget*)ImageWidget2, 197, 139);
    laWidget_SetBackgroundType((laWidget*)ImageWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget2, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget2, &HarmonyLogo);
    laWidget_AddChild((laWidget*)layer2, (laWidget*)ImageWidget2);

    ImageWidget4 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget4, 17, 230);
    laWidget_SetSize((laWidget*)ImageWidget4, 144, 39);
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
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &MyScheme);
    laLayer_SetBufferCount(layer0, 2);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget5 = laImageWidget_New();
    laWidget_SetSize((laWidget*)ImageWidget5, 480, 104);
    laWidget_SetOptimizationFlags((laWidget*)ImageWidget5, LA_WIDGET_OPT_LOCAL_REDRAW);
    laWidget_SetBackgroundType((laWidget*)ImageWidget5, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget5, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)ImageWidget5, 0, 0, 4, 0);
    laImageWidget_SetImage(ImageWidget5, &clouds3);
    laImageWidget_SetHAlignment(ImageWidget5, LA_HALIGN_LEFT);
    laImageWidget_SetVAlignment(ImageWidget5, LA_VALIGN_TOP);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget5);

    TimeLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TimeLabel, 240, 10);
    laWidget_SetSize((laWidget*)TimeLabel, 21, 25);
    laWidget_SetScheme((laWidget*)TimeLabel, &TextWhite);
    laWidget_SetBackgroundType((laWidget*)TimeLabel, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)TimeLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TimeLabel, laString_CreateFromID(string_TimeHour));
    laLabelWidget_SetHAlignment(TimeLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget5, (laWidget*)TimeLabel);

    LabelWidget2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget2, 15, 7);
    laWidget_SetSize((laWidget*)LabelWidget2, 105, 36);
    laWidget_SetScheme((laWidget*)LabelWidget2, &TextWhite);
    laWidget_SetBackgroundType((laWidget*)LabelWidget2, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget2, laString_CreateFromID(string_Chandler));
    laLabelWidget_SetHAlignment(LabelWidget2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget5, (laWidget*)LabelWidget2);

    ImageSequenceWidget1 = laImageSequenceWidget_New();
    laWidget_SetPosition((laWidget*)ImageSequenceWidget1, 253, 10);
    laWidget_SetSize((laWidget*)ImageSequenceWidget1, 18, 25);
    laWidget_SetBackgroundType((laWidget*)ImageSequenceWidget1, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)ImageSequenceWidget1, LA_WIDGET_BORDER_NONE);
    laImageSequenceWidget_SetImageCount(ImageSequenceWidget1, 2);
    laImageSequenceWidget_SetImage(ImageSequenceWidget1, 0, &colon);
    laImageSequenceWidget_SetRepeat(ImageSequenceWidget1, LA_TRUE);
    laWidget_AddChild((laWidget*)ImageWidget5, (laWidget*)ImageSequenceWidget1);

    MinuteLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)MinuteLabel, 267, 10);
    laWidget_SetSize((laWidget*)MinuteLabel, 163, 25);
    laWidget_SetScheme((laWidget*)MinuteLabel, &TextWhite);
    laWidget_SetBackgroundType((laWidget*)MinuteLabel, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)MinuteLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(MinuteLabel, laString_CreateFromID(string_TimeMinute));
    laLabelWidget_SetHAlignment(MinuteLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget5, (laWidget*)MinuteLabel);

    ButtonWidget2 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget2, 435, 5);
    laWidget_SetSize((laWidget*)ButtonWidget2, 42, 36);
    laWidget_SetOptimizationFlags((laWidget*)ButtonWidget2, LA_WIDGET_OPT_LOCAL_REDRAW);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget2, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedImage(ButtonWidget2, &mchpLogo);
    laButtonWidget_SetImagePosition(ButtonWidget2, LA_RELATIVE_POSITION_BEHIND);
    laButtonWidget_SetPressedOffset(ButtonWidget2, 0);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget2, &ButtonWidget2_ReleasedEvent);

    laWidget_AddChild((laWidget*)ImageWidget5, (laWidget*)ButtonWidget2);

    LabelWidget5 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget5, 20, 43);
    laWidget_SetSize((laWidget*)LabelWidget5, 124, 54);
    laWidget_SetOptimizationFlags((laWidget*)LabelWidget5, LA_WIDGET_OPT_LOCAL_REDRAW);
    laWidget_SetScheme((laWidget*)LabelWidget5, &TextWhite);
    laWidget_SetBackgroundType((laWidget*)LabelWidget5, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget5, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget5, laString_CreateFromID(string_degrees));
    laLabelWidget_SetHAlignment(LabelWidget5, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget5, (laWidget*)LabelWidget5);

    PanelWidget2 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget2, 0, 104);
    laWidget_SetSize((laWidget*)PanelWidget2, 480, 168);
    laWidget_SetScheme((laWidget*)PanelWidget2, &GrayScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget2, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget2);

    PanelWidget6 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget6, 472, 11);
    laWidget_SetSize((laWidget*)PanelWidget6, 6, 152);
    laWidget_SetBackgroundType((laWidget*)PanelWidget6, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget6, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelWidget2, PanelWidget6);

    PanelWidget1 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget1, 9, 159);
    laWidget_SetSize((laWidget*)PanelWidget1, 468, 4);
    laWidget_SetBackgroundType((laWidget*)PanelWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget1, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelWidget2, PanelWidget1);

    PanelWidget5 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget5, 352, 8);
    laWidget_SetSize((laWidget*)PanelWidget5, 120, 152);
    laWidget_SetScheme((laWidget*)PanelWidget5, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget5, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget5, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelWidget2, PanelWidget5);

    ButtonWidget1 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget1, 16, 7);
    laWidget_SetSize((laWidget*)ButtonWidget1, 100, 74);
    laWidget_SetScheme((laWidget*)ButtonWidget1, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget1, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetImagePosition(ButtonWidget1, LA_RELATIVE_POSITION_BEHIND);
    laButtonWidget_SetPressedOffset(ButtonWidget1, 0);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget1, &ButtonWidget1_ReleasedEvent);

    laWidget_AddChild((laWidget*)PanelWidget5, (laWidget*)ButtonWidget1);

    CloudIcon = laImageWidget_New();
    laWidget_SetPosition((laWidget*)CloudIcon, 18, 8);
    laWidget_SetSize((laWidget*)CloudIcon, 64, 64);
    laWidget_SetScheme((laWidget*)CloudIcon, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)CloudIcon, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CloudIcon, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(CloudIcon, &cloud_icon);
    laWidget_AddChild((laWidget*)ButtonWidget1, (laWidget*)CloudIcon);

    ImageWidget5 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget5, 6, 8);
    laWidget_SetSize((laWidget*)ImageWidget5, 346, 76);
    laWidget_SetOptimizationFlags((laWidget*)ImageWidget5, LA_WIDGET_OPT_LOCAL_REDRAW);
    laWidget_SetBackgroundType((laWidget*)ImageWidget5, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget5, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget5, &gradient2);
    laWidget_AddChild((laWidget*)PanelWidget2, (laWidget*)ImageWidget5);

    LabelWidget1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget1, 10, 10);
    laWidget_SetSize((laWidget*)LabelWidget1, 100, 25);
    laWidget_SetScheme((laWidget*)LabelWidget1, &Green);
    laWidget_SetBackgroundType((laWidget*)LabelWidget1, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget1, laString_CreateFromID(string_Current));
    laLabelWidget_SetHAlignment(LabelWidget1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget5, (laWidget*)LabelWidget1);

    LabelWidget3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget3, 225, 10);
    laWidget_SetSize((laWidget*)LabelWidget3, 84, 30);
    laWidget_SetScheme((laWidget*)LabelWidget3, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget3, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget3, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget3, laString_CreateFromID(string_Cloudy));
    laLabelWidget_SetHAlignment(LabelWidget3, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget5, (laWidget*)LabelWidget3);

    LabelWidget11 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget11, 17, 48);
    laWidget_SetSize((laWidget*)LabelWidget11, 124, 25);
    laWidget_SetScheme((laWidget*)LabelWidget11, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget11, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget11, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget11, laString_CreateFromID(string_humidity));
    laLabelWidget_SetHAlignment(LabelWidget11, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget5, (laWidget*)LabelWidget11);

    LabelWidget12 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget12, 225, 48);
    laWidget_SetSize((laWidget*)LabelWidget12, 100, 25);
    laWidget_SetScheme((laWidget*)LabelWidget12, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget12, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget12, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget12, laString_CreateFromID(string_thirteenpercent));
    laLabelWidget_SetHAlignment(LabelWidget12, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget5, (laWidget*)LabelWidget12);

    ImageWidget5 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget5, 6, 84);
    laWidget_SetSize((laWidget*)ImageWidget5, 346, 76);
    laWidget_SetOptimizationFlags((laWidget*)ImageWidget5, LA_WIDGET_OPT_LOCAL_REDRAW);
    laWidget_SetBackgroundType((laWidget*)ImageWidget5, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget5, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget5, &gradient2);
    laWidget_AddChild((laWidget*)PanelWidget2, (laWidget*)ImageWidget5);

    LabelWidget6 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget6, 18, 4);
    laWidget_SetSize((laWidget*)LabelWidget6, 100, 25);
    laWidget_SetScheme((laWidget*)LabelWidget6, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget6, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget6, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget6, laString_CreateFromID(string_uvindex));
    laLabelWidget_SetHAlignment(LabelWidget6, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget5, (laWidget*)LabelWidget6);

    LabelWidget8 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget8, 225, 1);
    laWidget_SetSize((laWidget*)LabelWidget8, 100, 25);
    laWidget_SetScheme((laWidget*)LabelWidget8, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget8, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget8, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget8, laString_CreateFromID(string_uvlow));
    laLabelWidget_SetHAlignment(LabelWidget8, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget5, (laWidget*)LabelWidget8);

    LabelWidget9 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget9, 18, 36);
    laWidget_SetSize((laWidget*)LabelWidget9, 187, 25);
    laWidget_SetScheme((laWidget*)LabelWidget9, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget9, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget9, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget9, laString_CreateFromID(string_windspeed));
    laLabelWidget_SetHAlignment(LabelWidget9, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget5, (laWidget*)LabelWidget9);

    LabelWidget10 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget10, 225, 36);
    laWidget_SetSize((laWidget*)LabelWidget10, 83, 25);
    laWidget_SetScheme((laWidget*)LabelWidget10, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget10, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget10, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget10, laString_CreateFromID(string_ninemph));
    laLabelWidget_SetHAlignment(LabelWidget10, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget5, (laWidget*)LabelWidget10);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer1, &ClearScheme);
    laLayer_SetBufferCount(layer1, 2);
    laLayer_SetAlphaEnable(layer1, LA_TRUE);
    laLayer_SetAlphaAmount(layer1, 0x0);

    laScreen_SetLayer(screen, 1, layer1);

    layer2 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer2, 0, 0);
    laWidget_SetSize((laWidget*)layer2, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer2, &ClearScheme);
    laLayer_SetBufferCount(layer2, 2);
    laLayer_SetAlphaEnable(layer2, LA_TRUE);
    laLayer_SetAlphaAmount(layer2, 0x0);

    laScreen_SetLayer(screen, 2, layer2);

}



