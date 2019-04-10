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
laScheme GrayScheme;
laScheme TitleBarScheme;
laScheme WhiteScheme;
laScheme TextBoxScheme;
laScheme Green;
laScheme TextWhite;
laScheme GradientScheme;
laImageWidget* ImageWidget1;
laImageWidget* ImageWidget3;
laImageWidget* ImageWidget2;
laImageWidget* ImageWidget4;
laImageWidget* ImageWidget6;
laLabelWidget* LabelWidget2;
laLabelWidget* TimeLabel;
laLabelWidget* MinuteLabel;
laLabelWidget* LabelWidget5;
laButtonWidget* ButtonWidget2;
laLabelWidget* LabelWidget4;
laWidget* PanelWidget1;
laGradientWidget* GradientWidget1;
laLabelWidget* LabelWidget1;
laLabelWidget* LabelWidget3;
laLabelWidget* LabelWidget6;
laLabelWidget* LabelWidget8;
laLabelWidget* LabelWidget9;
laLabelWidget* LabelWidget10;
laLabelWidget* LabelWidget11;
laLabelWidget* LabelWidget12;
laWidget* PanelWidget2;
laImageWidget* CloudIcon;
laButtonWidget* ButtonWidget1;


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

    laScheme_Initialize(&WhiteScheme, GFX_COLOR_MODE_RGBA_8888);
    WhiteScheme.base = 0xFFFFFFFF;
    WhiteScheme.highlight = 0xFFFFFFFF;
    WhiteScheme.highlightLight = 0xFFFFFFFF;
    WhiteScheme.shadow = 0x808080FF;
    WhiteScheme.shadowDark = 0x404040FF;
    WhiteScheme.foreground = 0xFFFFFFFF;
    WhiteScheme.foregroundInactive = 0x808080FF;
    WhiteScheme.foregroundDisabled = 0x808080FF;
    WhiteScheme.background = 0xFFFFFFFF;
    WhiteScheme.backgroundInactive = 0xFFFFFFFF;
    WhiteScheme.backgroundDisabled = 0xC8D0D4FF;
    WhiteScheme.text = 0x736D73FF;
    WhiteScheme.textHighlight = 0xFFFF;
    WhiteScheme.textHighlightText = 0xFFFFFFFF;
    WhiteScheme.textInactive = 0xC8D0D4FF;
    WhiteScheme.textDisabled = 0xC8D0D4FF;

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
    GradientScheme.base = 0xFF;
    GradientScheme.highlight = 0xC8D0D4FF;
    GradientScheme.highlightLight = 0xFFFFFFFF;
    GradientScheme.shadow = 0x808080FF;
    GradientScheme.shadowDark = 0x404040FF;
    GradientScheme.foreground = 0xFFFFFFFF;
    GradientScheme.foregroundInactive = 0x808080FF;
    GradientScheme.foregroundDisabled = 0x808080FF;
    GradientScheme.background = 0xFF;
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
    laWidget_SetSize((laWidget*)layer0, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 2);
    laWidget_SetOptimizationFlags((laWidget*)layer0, LA_WIDGET_OPT_DRAW_ONCE);
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
    laWidget_SetOptimizationFlags((laWidget*)layer1, LA_WIDGET_OPT_DRAW_ONCE);
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
    laLayer_SetBufferCount(layer2, 1);
    laWidget_SetOptimizationFlags((laWidget*)layer2, LA_WIDGET_OPT_DRAW_ONCE);
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
    laWidget_SetScheme((laWidget*)layer0, &MyScheme);
    laLayer_SetBufferCount(layer0, 2);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget6 = laImageWidget_New();
    laWidget_SetSize((laWidget*)ImageWidget6, 800, 177);
    laWidget_SetOptimizationFlags((laWidget*)ImageWidget6, LA_WIDGET_OPT_LOCAL_REDRAW);
    laWidget_SetBackgroundType((laWidget*)ImageWidget6, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget6, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)ImageWidget6, 0, 0, 4, 0);
    laImageWidget_SetImage(ImageWidget6, &clouds3_large);
    laImageWidget_SetHAlignment(ImageWidget6, LA_HALIGN_LEFT);
    laImageWidget_SetVAlignment(ImageWidget6, LA_VALIGN_TOP);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget6);

    LabelWidget2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget2, 24, 18);
    laWidget_SetSize((laWidget*)LabelWidget2, 167, 38);
    laWidget_SetScheme((laWidget*)LabelWidget2, &TextWhite);
    laWidget_SetBackgroundType((laWidget*)LabelWidget2, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget2, laString_CreateFromID(string_Chandler));
    laLabelWidget_SetHAlignment(LabelWidget2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget6, (laWidget*)LabelWidget2);

    TimeLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TimeLabel, 371, 30);
    laWidget_SetSize((laWidget*)TimeLabel, 28, 35);
    laWidget_SetScheme((laWidget*)TimeLabel, &TextWhite);
    laWidget_SetBackgroundType((laWidget*)TimeLabel, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)TimeLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TimeLabel, laString_CreateFromID(string_TimeHour));
    laLabelWidget_SetHAlignment(TimeLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget6, (laWidget*)TimeLabel);

    MinuteLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)MinuteLabel, 410, 30);
    laWidget_SetSize((laWidget*)MinuteLabel, 284, 35);
    laWidget_SetScheme((laWidget*)MinuteLabel, &TextWhite);
    laWidget_SetBackgroundType((laWidget*)MinuteLabel, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)MinuteLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(MinuteLabel, laString_CreateFromID(string_TimeMinute));
    laLabelWidget_SetHAlignment(MinuteLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget6, (laWidget*)MinuteLabel);

    LabelWidget5 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget5, 21, 74);
    laWidget_SetSize((laWidget*)LabelWidget5, 184, 62);
    laWidget_SetOptimizationFlags((laWidget*)LabelWidget5, LA_WIDGET_OPT_LOCAL_REDRAW);
    laWidget_SetScheme((laWidget*)LabelWidget5, &TextWhite);
    laWidget_SetBackgroundType((laWidget*)LabelWidget5, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget5, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget5, laString_CreateFromID(string_degrees));
    laLabelWidget_SetHAlignment(LabelWidget5, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)ImageWidget6, (laWidget*)LabelWidget5);

    ButtonWidget2 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget2, 711, 20);
    laWidget_SetSize((laWidget*)ButtonWidget2, 70, 70);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget2, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonWidget2, &mchpLogo);
    laButtonWidget_SetReleasedImage(ButtonWidget2, &mchpLogo);
    laButtonWidget_SetPressedOffset(ButtonWidget2, 0);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget2, &ButtonWidget2_ReleasedEvent);

    laWidget_AddChild((laWidget*)ImageWidget6, (laWidget*)ButtonWidget2);

    LabelWidget4 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget4, 393, 28);
    laWidget_SetSize((laWidget*)LabelWidget4, 20, 35);
    laWidget_SetScheme((laWidget*)LabelWidget4, &TextWhite);
    laWidget_SetBackgroundType((laWidget*)LabelWidget4, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget4, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget4, laString_CreateFromID(string_Colon));
    laWidget_AddChild((laWidget*)ImageWidget6, (laWidget*)LabelWidget4);

    PanelWidget1 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget1, 0, 177);
    laWidget_SetSize((laWidget*)PanelWidget1, 800, 303);
    laWidget_SetScheme((laWidget*)PanelWidget1, &GrayScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget1, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget1);

    GradientWidget1 = laGradientWidget_New();
    laWidget_SetPosition((laWidget*)GradientWidget1, 8, 8);
    laWidget_SetSize((laWidget*)GradientWidget1, 612, 287);
    laWidget_SetScheme((laWidget*)GradientWidget1, &GradientScheme);
    laWidget_SetBackgroundType((laWidget*)GradientWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GradientWidget1, LA_WIDGET_BORDER_NONE);
    laGradientWidget_SetDirection((laGradientWidget*)GradientWidget1, LA_GRADIENT_DIRECTION_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget1, (laWidget*)GradientWidget1);

    LabelWidget1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget1, 24, 23);
    laWidget_SetSize((laWidget*)LabelWidget1, 188, 55);
    laWidget_SetScheme((laWidget*)LabelWidget1, &Green);
    laWidget_SetBackgroundType((laWidget*)LabelWidget1, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget1, laString_CreateFromID(string_Current));
    laLabelWidget_SetHAlignment(LabelWidget1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GradientWidget1, (laWidget*)LabelWidget1);

    LabelWidget3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget3, 400, 23);
    laWidget_SetSize((laWidget*)LabelWidget3, 142, 55);
    laWidget_SetScheme((laWidget*)LabelWidget3, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget3, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget3, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget3, laString_CreateFromID(string_Cloudy));
    laLabelWidget_SetHAlignment(LabelWidget3, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GradientWidget1, (laWidget*)LabelWidget3);

    LabelWidget6 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget6, 23, 155);
    laWidget_SetSize((laWidget*)LabelWidget6, 179, 35);
    laWidget_SetScheme((laWidget*)LabelWidget6, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget6, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget6, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget6, laString_CreateFromID(string_uvindex));
    laLabelWidget_SetHAlignment(LabelWidget6, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GradientWidget1, (laWidget*)LabelWidget6);

    LabelWidget8 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget8, 400, 155);
    laWidget_SetSize((laWidget*)LabelWidget8, 76, 35);
    laWidget_SetScheme((laWidget*)LabelWidget8, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget8, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget8, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget8, laString_CreateFromID(string_uvlow));
    laLabelWidget_SetHAlignment(LabelWidget8, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GradientWidget1, (laWidget*)LabelWidget8);

    LabelWidget9 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget9, 23, 217);
    laWidget_SetSize((laWidget*)LabelWidget9, 329, 35);
    laWidget_SetScheme((laWidget*)LabelWidget9, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget9, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget9, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget9, laString_CreateFromID(string_windspeed));
    laLabelWidget_SetHAlignment(LabelWidget9, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GradientWidget1, (laWidget*)LabelWidget9);

    LabelWidget10 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget10, 400, 217);
    laWidget_SetSize((laWidget*)LabelWidget10, 139, 43);
    laWidget_SetScheme((laWidget*)LabelWidget10, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget10, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget10, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget10, laString_CreateFromID(string_ninemph));
    laLabelWidget_SetHAlignment(LabelWidget10, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GradientWidget1, (laWidget*)LabelWidget10);

    LabelWidget11 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget11, 24, 99);
    laWidget_SetSize((laWidget*)LabelWidget11, 187, 35);
    laWidget_SetScheme((laWidget*)LabelWidget11, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget11, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget11, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget11, laString_CreateFromID(string_humidity));
    laLabelWidget_SetHAlignment(LabelWidget11, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GradientWidget1, (laWidget*)LabelWidget11);

    LabelWidget12 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget12, 400, 99);
    laWidget_SetSize((laWidget*)LabelWidget12, 87, 35);
    laWidget_SetScheme((laWidget*)LabelWidget12, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget12, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget12, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget12, laString_CreateFromID(string_thirteenpercent));
    laLabelWidget_SetHAlignment(LabelWidget12, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GradientWidget1, (laWidget*)LabelWidget12);

    PanelWidget2 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget2, 620, 8);
    laWidget_SetSize((laWidget*)PanelWidget2, 172, 287);
    laWidget_SetScheme((laWidget*)PanelWidget2, &WhiteScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget2, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelWidget1, PanelWidget2);

    CloudIcon = laImageWidget_New();
    laWidget_SetPosition((laWidget*)CloudIcon, 17, 23);
    laWidget_SetSize((laWidget*)CloudIcon, 130, 128);
    laWidget_SetBackgroundType((laWidget*)CloudIcon, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CloudIcon, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(CloudIcon, &cloud_icon);
    laWidget_AddChild((laWidget*)PanelWidget2, (laWidget*)CloudIcon);

    ButtonWidget1 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget1, 17, 23);
    laWidget_SetSize((laWidget*)ButtonWidget1, 130, 128);
    laWidget_SetScheme((laWidget*)ButtonWidget1, &TextBoxScheme);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget1, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetImagePosition(ButtonWidget1, LA_RELATIVE_POSITION_BEHIND);
    laButtonWidget_SetPressedOffset(ButtonWidget1, 0);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget1, &ButtonWidget1_ReleasedEvent);

    laWidget_AddChild((laWidget*)PanelWidget2, (laWidget*)ButtonWidget1);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer1, 2);
    laLayer_SetAlphaEnable(layer1, LA_TRUE);
    laLayer_SetAlphaAmount(layer1, 0x0);

    laScreen_SetLayer(screen, 1, layer1);

    layer2 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer2, 0, 0);
    laWidget_SetSize((laWidget*)layer2, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer2, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer2, 2);
    laLayer_SetAlphaEnable(layer2, LA_TRUE);
    laLayer_SetAlphaAmount(layer2, 0x0);

    laScreen_SetLayer(screen, 2, layer2);

}



