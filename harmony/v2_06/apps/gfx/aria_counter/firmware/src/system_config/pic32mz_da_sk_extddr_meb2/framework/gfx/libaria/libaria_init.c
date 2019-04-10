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
laScheme GrayText;
laScheme whiteScheme;
laScheme WhiteText;
laScheme GreenText;
laScheme ClearScheme;
laImageWidget* ImageWidget1;
laImageWidget* ImageWidget3;
laImageWidget* ImageWidget2;
laImageWidget* ImageWidget4;
laImageWidget* RoadBackground;
laImageWidget* ImageWidget5;
laLabelWidget* LabelWidget2;
laLabelWidget* Digit0;
laLabelWidget* Digit1;
laLabelWidget* Digit2;
laLabelWidget* Digit3;
laLabelWidget* Digit4;
laButtonWidget* ButtonWidget2;
laLabelWidget* LabelWidget1;
laImageWidget* ImageWidget7;
laImageWidget* ImageWidget6;
laButtonWidget* ButtonWidget3;
laLabelWidget* LabelWidget_Single;
laLabelWidget* Digit0_1;
laLabelWidget* Digit1_1;
laLabelWidget* Digit2_1;
laLabelWidget* Digit3_1;
laLabelWidget* Digit4_1;
laLabelWidget* LabelWidget3;


static void ScreenCreate_SplashScreen(laScreen* screen);
static void ScreenCreate_MainScreen(laScreen* screen);
static void ScreenCreate_MainScreenSingle(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

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

    laScheme_Initialize(&GrayText, GFX_COLOR_MODE_RGBA_8888);
    GrayText.base = 0xC8D0D4FF;
    GrayText.highlight = 0xC8D0D4FF;
    GrayText.highlightLight = 0xFFFFFFFF;
    GrayText.shadow = 0x808080FF;
    GrayText.shadowDark = 0x404040FF;
    GrayText.foreground = 0xFF;
    GrayText.foregroundInactive = 0xD6E3E7FF;
    GrayText.foregroundDisabled = 0x808080FF;
    GrayText.background = 0xFFFFFFFF;
    GrayText.backgroundInactive = 0xD6E3E7FF;
    GrayText.backgroundDisabled = 0xC8D0D4FF;
    GrayText.text = 0xDEDFDEFF;
    GrayText.textHighlight = 0xFFFF;
    GrayText.textHighlightText = 0xFFFFFFFF;
    GrayText.textInactive = 0xD6E3E7FF;
    GrayText.textDisabled = 0x8C9294FF;

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

    laScheme_Initialize(&WhiteText, GFX_COLOR_MODE_RGBA_8888);
    WhiteText.base = 0xFFFFFFFF;
    WhiteText.highlight = 0xC8D0D4FF;
    WhiteText.highlightLight = 0xFFFFFFFF;
    WhiteText.shadow = 0x808080FF;
    WhiteText.shadowDark = 0x404040FF;
    WhiteText.foreground = 0xFFFFFFFF;
    WhiteText.foregroundInactive = 0xD6E3E7FF;
    WhiteText.foregroundDisabled = 0x808080FF;
    WhiteText.background = 0xFFFFFFFF;
    WhiteText.backgroundInactive = 0xD6E3E7FF;
    WhiteText.backgroundDisabled = 0xC8D0D4FF;
    WhiteText.text = 0xFFFFFFFF;
    WhiteText.textHighlight = 0xFFFF;
    WhiteText.textHighlightText = 0xFFFFFFFF;
    WhiteText.textInactive = 0xD6E3E7FF;
    WhiteText.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&GreenText, GFX_COLOR_MODE_RGBA_8888);
    GreenText.base = 0xC8D0D4FF;
    GreenText.highlight = 0xC8D0D4FF;
    GreenText.highlightLight = 0xFFFFFFFF;
    GreenText.shadow = 0x808080FF;
    GreenText.shadowDark = 0x404040FF;
    GreenText.foreground = 0xFF;
    GreenText.foregroundInactive = 0xD6E3E7FF;
    GreenText.foregroundDisabled = 0x808080FF;
    GreenText.background = 0xFFFFFFFF;
    GreenText.backgroundInactive = 0xD6E3E7FF;
    GreenText.backgroundDisabled = 0xC8D0D4FF;
    GreenText.text = 0xFF00FF;
    GreenText.textHighlight = 0xFFFF;
    GreenText.textHighlightText = 0xFFFFFFFF;
    GreenText.textInactive = 0xD6E3E7FF;
    GreenText.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&ClearScheme, GFX_COLOR_MODE_RGBA_8888);
    ClearScheme.base = 0xC8D0D4FF;
    ClearScheme.highlight = 0xC8D0D4FF;
    ClearScheme.highlightLight = 0xFFFFFFFF;
    ClearScheme.shadow = 0x808080FF;
    ClearScheme.shadowDark = 0x404040FF;
    ClearScheme.foreground = 0xFF;
    ClearScheme.foregroundInactive = 0xD6E3E7FF;
    ClearScheme.foregroundDisabled = 0x808080FF;
    ClearScheme.background = 0xFF;
    ClearScheme.backgroundInactive = 0xD6E3E7FF;
    ClearScheme.backgroundDisabled = 0xC8D0D4FF;
    ClearScheme.text = 0x0;
    ClearScheme.textHighlight = 0xFFFF;
    ClearScheme.textHighlightText = 0xFFFFFFFF;
    ClearScheme.textInactive = 0xD6E3E7FF;
    ClearScheme.textDisabled = 0x8C9294FF;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_SplashScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_MainScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_MainScreenSingle);
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
    laWidget_SetScheme((laWidget*)layer0, &whiteScheme);
    laLayer_SetBufferCount(layer0, 2);
    laWidget_SetOptimizationFlags((laWidget*)layer0, LA_WIDGET_OPT_DRAW_ONCE);
    laLayer_SetAlphaEnable(layer0, LA_TRUE);
    laLayer_SetAlphaAmount(layer0, 0xFF);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 120, 40);
    laWidget_SetSize((laWidget*)ImageWidget1, 240, 139);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &PIC32Logo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetScheme((laWidget*)layer1, &ClearScheme);
    laLayer_SetBufferCount(layer1, 2);
    laLayer_SetAlphaEnable(layer1, LA_TRUE);
    laLayer_SetAlphaAmount(layer1, 0xFF);

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
    laWidget_SetScheme((laWidget*)layer2, &ClearScheme);
    laLayer_SetBufferCount(layer2, 2);
    laLayer_SetAlphaEnable(layer2, LA_TRUE);
    laLayer_SetAlphaAmount(layer2, 0x0);

    laScreen_SetLayer(screen, 2, layer2);

    ImageWidget2 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget2, 120, 40);
    laWidget_SetSize((laWidget*)ImageWidget2, 240, 139);
    laWidget_SetBackgroundType((laWidget*)ImageWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget2, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget2, &HarmonyLogo);
    laWidget_AddChild((laWidget*)layer2, (laWidget*)ImageWidget2);

    ImageWidget4 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget4, 17, 230);
    laWidget_SetSize((laWidget*)ImageWidget4, 144, 39);
    laWidget_SetVisible((laWidget*)ImageWidget4, LA_FALSE);
    laWidget_SetOptimizationFlags((laWidget*)ImageWidget4, LA_WIDGET_OPT_LOCAL_REDRAW);
    laWidget_SetBackgroundType((laWidget*)ImageWidget4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget4, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget4, &MicrochipLogo);
    laWidget_AddChild((laWidget*)layer2, (laWidget*)ImageWidget4);

}

static void ScreenCreate_MainScreen(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_NONE);
    laLayer_SetBufferCount(layer0, 2);

    laScreen_SetLayer(screen, 0, layer0);

    RoadBackground = laImageWidget_New();
    laWidget_SetSize((laWidget*)RoadBackground, 480, 272);
    laWidget_SetBackgroundType((laWidget*)RoadBackground, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RoadBackground, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(RoadBackground, &road480x300);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)RoadBackground);

    ImageWidget5 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget5, 12, 8);
    laWidget_SetSize((laWidget*)ImageWidget5, 96, 80);
    laWidget_SetBackgroundType((laWidget*)ImageWidget5, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)ImageWidget5, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget5, &NewHarmonyLogo_VerySmall);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget5);

    LabelWidget2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget2, 65, 229);
    laWidget_SetSize((laWidget*)LabelWidget2, 108, 29);
    laWidget_SetScheme((laWidget*)LabelWidget2, &WhiteText);
    laWidget_SetBackgroundType((laWidget*)LabelWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget2, laString_CreateFromID(string_RoadVal));
    laLabelWidget_SetHAlignment(LabelWidget2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget2);

    Digit0 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit0, 450, 110);
    laWidget_SetSize((laWidget*)Digit0, 23, 31);
    laWidget_SetBackgroundType((laWidget*)Digit0, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit0, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit0, laString_CreateFromID(string_Eight));
    laLabelWidget_SetHAlignment(Digit0, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Digit0);

    Digit1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit1, 425, 110);
    laWidget_SetSize((laWidget*)Digit1, 23, 31);
    laWidget_SetBackgroundType((laWidget*)Digit1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit1, laString_CreateFromID(string_Eight));
    laLabelWidget_SetHAlignment(Digit1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Digit1);

    Digit2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit2, 400, 110);
    laWidget_SetSize((laWidget*)Digit2, 23, 31);
    laWidget_SetBackgroundType((laWidget*)Digit2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit2, laString_CreateFromID(string_Eight));
    laLabelWidget_SetHAlignment(Digit2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Digit2);

    Digit3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit3, 375, 110);
    laWidget_SetSize((laWidget*)Digit3, 23, 31);
    laWidget_SetBackgroundType((laWidget*)Digit3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit3, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit3, laString_CreateFromID(string_Eight));
    laLabelWidget_SetHAlignment(Digit3, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Digit3);

    Digit4 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit4, 351, 110);
    laWidget_SetSize((laWidget*)Digit4, 23, 31);
    laWidget_SetBackgroundType((laWidget*)Digit4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit4, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit4, laString_CreateFromID(string_Eight));
    laLabelWidget_SetHAlignment(Digit4, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Digit4);

    ButtonWidget2 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget2, 323, 233);
    laWidget_SetSize((laWidget*)ButtonWidget2, 150, 30);
    laWidget_SetScheme((laWidget*)ButtonWidget2, &GrayText);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget2, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(ButtonWidget2, laString_CreateFromID(string_SingleBuffered_Norm));
    laButtonWidget_SetPressedEventCallback(ButtonWidget2, &ButtonWidget2_PressedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget2);

    LabelWidget1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget1, 330, 0);
    laWidget_SetSize((laWidget*)LabelWidget1, 140, 25);
    laWidget_SetScheme((laWidget*)LabelWidget1, &GreenText);
    laWidget_SetBackgroundType((laWidget*)LabelWidget1, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget1, laString_CreateFromID(string_DoubleBuffered_Bold));
    laLabelWidget_SetHAlignment(LabelWidget1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget1);

}

static void ScreenCreate_MainScreenSingle(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_NONE);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget7 = laImageWidget_New();
    laWidget_SetSize((laWidget*)ImageWidget7, 480, 272);
    laWidget_SetBackgroundType((laWidget*)ImageWidget7, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget7, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget7, &road480x300);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget7);

    ImageWidget6 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget6, 12, 8);
    laWidget_SetSize((laWidget*)ImageWidget6, 96, 80);
    laWidget_SetBackgroundType((laWidget*)ImageWidget6, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)ImageWidget6, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget6, &NewHarmonyLogo_VerySmall);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget6);

    ButtonWidget3 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget3, 323, 233);
    laWidget_SetSize((laWidget*)ButtonWidget3, 150, 30);
    laWidget_SetScheme((laWidget*)ButtonWidget3, &GrayText);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget3, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)ButtonWidget3, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(ButtonWidget3, laString_CreateFromID(string_DoubleBuffered_Norm));
    laButtonWidget_SetPressedEventCallback(ButtonWidget3, &ButtonWidget3_PressedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget3);

    LabelWidget_Single = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget_Single, 65, 229);
    laWidget_SetSize((laWidget*)LabelWidget_Single, 108, 29);
    laWidget_SetScheme((laWidget*)LabelWidget_Single, &WhiteText);
    laWidget_SetBackgroundType((laWidget*)LabelWidget_Single, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget_Single, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget_Single, laString_CreateFromID(string_RoadVal));
    laLabelWidget_SetHAlignment(LabelWidget_Single, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget_Single);

    Digit0_1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit0_1, 450, 110);
    laWidget_SetSize((laWidget*)Digit0_1, 23, 31);
    laWidget_SetBackgroundType((laWidget*)Digit0_1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit0_1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit0_1, laString_CreateFromID(string_Eight));
    laLabelWidget_SetHAlignment(Digit0_1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Digit0_1);

    Digit1_1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit1_1, 425, 110);
    laWidget_SetSize((laWidget*)Digit1_1, 23, 31);
    laWidget_SetBackgroundType((laWidget*)Digit1_1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit1_1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit1_1, laString_CreateFromID(string_Eight));
    laLabelWidget_SetHAlignment(Digit1_1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Digit1_1);

    Digit2_1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit2_1, 400, 110);
    laWidget_SetSize((laWidget*)Digit2_1, 23, 31);
    laWidget_SetBackgroundType((laWidget*)Digit2_1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit2_1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit2_1, laString_CreateFromID(string_Eight));
    laLabelWidget_SetHAlignment(Digit2_1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Digit2_1);

    Digit3_1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit3_1, 375, 110);
    laWidget_SetSize((laWidget*)Digit3_1, 23, 31);
    laWidget_SetBackgroundType((laWidget*)Digit3_1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit3_1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit3_1, laString_CreateFromID(string_Eight));
    laLabelWidget_SetHAlignment(Digit3_1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Digit3_1);

    Digit4_1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Digit4_1, 351, 110);
    laWidget_SetSize((laWidget*)Digit4_1, 23, 31);
    laWidget_SetBackgroundType((laWidget*)Digit4_1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Digit4_1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Digit4_1, laString_CreateFromID(string_Eight));
    laLabelWidget_SetHAlignment(Digit4_1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Digit4_1);

    LabelWidget3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget3, 330, 0);
    laWidget_SetSize((laWidget*)LabelWidget3, 140, 25);
    laWidget_SetScheme((laWidget*)LabelWidget3, &GreenText);
    laWidget_SetBackgroundType((laWidget*)LabelWidget3, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget3, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget3, laString_CreateFromID(string_SingleBuffered_Bold));
    laLabelWidget_SetHAlignment(LabelWidget3, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget3);

}



