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
laScheme GreenText;
laScheme WhiteText;
laImageWidget* ImageWidget1;
laImageWidget* ImageWidget2;
laWidget* PanelWidget1;
laImageWidget* ImageWidget3;
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

// START OF CUSTOM CODE - DO NOT MODIFY OR REMOVE!!!
extern void forceLayerSwapSingleBuffer(void);
extern void APP_IncrementCount(laLabelWidget* digit0,
                                    laLabelWidget* digit1,
                                    laLabelWidget* digit2,
                                    laLabelWidget* digit3,
                                    laLabelWidget* digit4);
void APP_DecrementCount(laLabelWidget* labelWidget);
// END OF CUSTOM CODE

int32_t libaria_initialize(void)
{
    laScreen* screen;

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

    laScheme_Initialize(&GrayText, GFX_COLOR_MODE_RGB_565);
    GrayText.base = 0xC67A;
    GrayText.highlight = 0xC67A;
    GrayText.highlightLight = 0xFFFF;
    GrayText.shadow = 0x8410;
    GrayText.shadowDark = 0x4208;
    GrayText.foreground = 0x0;
    GrayText.foregroundInactive = 0xD71C;
    GrayText.foregroundDisabled = 0x8410;
    GrayText.background = 0xFFFF;
    GrayText.backgroundInactive = 0xD71C;
    GrayText.backgroundDisabled = 0xC67A;
    GrayText.text = 0xDEFB;
    GrayText.textHighlight = 0x1F;
    GrayText.textHighlightText = 0xFFFF;
    GrayText.textInactive = 0xD71C;
    GrayText.textDisabled = 0x8C92;

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

    laScheme_Initialize(&GreenText, GFX_COLOR_MODE_RGB_565);
    GreenText.base = 0xC67A;
    GreenText.highlight = 0xC67A;
    GreenText.highlightLight = 0xFFFF;
    GreenText.shadow = 0x8410;
    GreenText.shadowDark = 0x4208;
    GreenText.foreground = 0x0;
    GreenText.foregroundInactive = 0xD71C;
    GreenText.foregroundDisabled = 0x8410;
    GreenText.background = 0xFFFF;
    GreenText.backgroundInactive = 0xD71C;
    GreenText.backgroundDisabled = 0xC67A;
    GreenText.text = 0x7E0;
    GreenText.textHighlight = 0x1F;
    GreenText.textHighlightText = 0xFFFF;
    GreenText.textInactive = 0xD71C;
    GreenText.textDisabled = 0x8C92;
    
    laScheme_Initialize(&WhiteText, GFX_COLOR_MODE_RGB_565);
    WhiteText.base = 0xFFFF;
    WhiteText.highlight = 0xC67A;
    WhiteText.highlightLight = 0xFFFF;
    WhiteText.shadow = 0x8410;
    WhiteText.shadowDark = 0x4208;
    WhiteText.foreground = 0xFFFF;
    WhiteText.foregroundInactive = 0xD71C;
    WhiteText.foregroundDisabled = 0x8410;
    WhiteText.background = 0xFFFF;
    WhiteText.backgroundInactive = 0xD71C;
    WhiteText.backgroundDisabled = 0xC67A;
    WhiteText.text = 0xFFF8;
    WhiteText.textHighlight = 0x1F;
    WhiteText.textHighlightText = 0xFFFF;
    WhiteText.textInactive = 0xD71C;
    WhiteText.textDisabled = 0x8C92;

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

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &whiteScheme);
    laLayer_SetBufferCount(layer0, 2);
    laWidget_SetOptimizationFlags((laWidget*)layer0, LA_WIDGET_OPT_DRAW_ONCE);
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

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_NONE);
    laLayer_SetBufferCount(layer0, 2);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    RoadBackground = laImageWidget_New();
    laWidget_SetSize((laWidget*)RoadBackground, 480, 272);
    laWidget_SetBackgroundType((laWidget*)RoadBackground, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RoadBackground, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(RoadBackground, &road480x300);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)RoadBackground);

    ImageWidget5 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget5, 11, 10);
    laWidget_SetSize((laWidget*)ImageWidget5, 80, 56);
    laWidget_SetBackgroundType((laWidget*)ImageWidget5, LA_WIDGET_BACKGROUND_NONE);
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

// START OF CUSTOM CODE - DO NOT MODIFY OR REMOVE!!!    
    APP_IncrementCount(Digit0, Digit1, Digit2, Digit3, Digit4);
    APP_DecrementCount(LabelWidget2);
// END OF CUSTOM CODE
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
    laWidget_SetPosition((laWidget*)ImageWidget6, 11, 10);
    laWidget_SetSize((laWidget*)ImageWidget6, 80, 56);
    laWidget_SetBackgroundType((laWidget*)ImageWidget6, LA_WIDGET_BACKGROUND_NONE);
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

// START OF CUSTOM CODE - DO NOT MODIFY OR REMOVE!!!
    APP_IncrementCount(Digit0_1, Digit1_1, Digit2_1, Digit3_1, Digit4_1);
    APP_DecrementCount(LabelWidget_Single);
    forceLayerSwapSingleBuffer();
// END OF CUSTOM CODE
}



