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

laScheme LabelScheme;
laScheme defaultScheme;
laScheme defaultPaletteScheme;
laImageSequenceWidget* ImageSequenceWidget1;
laLabelWidget* LabelWidget12;
laImageSequenceWidget* ImageSequenceWidget;
laButtonWidget* ButtonWidget1;
laButtonWidget* ButtonWidget2;
laButtonWidget* ButtonWidget;
laButtonWidget* ButtonWidget4;
laLabelWidget* LabelWidget5;
laLabelWidget* LabelWidget6;
laLabelWidget* LabelWidget7;
laLabelWidget* LabelWidget8;
laLabelWidget* LabelWidget9;
laLabelWidget* LabelWidget10;


static void ScreenCreate_default(laScreen* screen);
static void ScreenCreate_info_screen(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&LabelScheme, GFX_COLOR_MODE_RGB_565);
    LabelScheme.base = 0x0;
    LabelScheme.highlight = 0x0;
    LabelScheme.highlightLight = 0x31;
    LabelScheme.shadow = 0x2;
    LabelScheme.shadowDark = 0x3;
    LabelScheme.foreground = 0x4;
    LabelScheme.foregroundInactive = 0x5;
    LabelScheme.foregroundDisabled = 0x2;
    LabelScheme.background = 0x31;
    LabelScheme.backgroundInactive = 0x5;
    LabelScheme.backgroundDisabled = 0x0;
    LabelScheme.text = 0x4;
    LabelScheme.textHighlight = 0x6;
    LabelScheme.textHighlightText = 0x31;
    LabelScheme.textInactive = 0x5;
    LabelScheme.textDisabled = 0x4D;

    laScheme_Initialize(&defaultScheme, GFX_COLOR_MODE_RGB_565);
    defaultScheme.base = 0x0;
    defaultScheme.highlight = 0x0;
    defaultScheme.highlightLight = 0x31;
    defaultScheme.shadow = 0x2;
    defaultScheme.shadowDark = 0x3;
    defaultScheme.foreground = 0x4;
    defaultScheme.foregroundInactive = 0x5;
    defaultScheme.foregroundDisabled = 0x2;
    defaultScheme.background = 0x31;
    defaultScheme.backgroundInactive = 0x5;
    defaultScheme.backgroundDisabled = 0x0;
    defaultScheme.text = 0x4;
    defaultScheme.textHighlight = 0x6;
    defaultScheme.textHighlightText = 0x31;
    defaultScheme.textInactive = 0x5;
    defaultScheme.textDisabled = 0x4D;

    laScheme_Initialize(&defaultPaletteScheme, GFX_COLOR_MODE_RGB_565);
    defaultPaletteScheme.base = 0x0;
    defaultPaletteScheme.highlight = 0x0;
    defaultPaletteScheme.highlightLight = 0x31;
    defaultPaletteScheme.shadow = 0x2;
    defaultPaletteScheme.shadowDark = 0x3;
    defaultPaletteScheme.foreground = 0x4;
    defaultPaletteScheme.foregroundInactive = 0x5;
    defaultPaletteScheme.foregroundDisabled = 0x2;
    defaultPaletteScheme.background = 0x31;
    defaultPaletteScheme.backgroundInactive = 0x5;
    defaultPaletteScheme.backgroundDisabled = 0x0;
    defaultPaletteScheme.text = 0x4;
    defaultPaletteScheme.textHighlight = 0x6;
    defaultPaletteScheme.textHighlightText = 0x31;
    defaultPaletteScheme.textInactive = 0x5;
    defaultPaletteScheme.textDisabled = 0x4D;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    GFX_Set(GFXF_GLOBAL_PALETTE, globalColorPalette);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_TRUE, LA_FALSE, &ScreenCreate_default);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_info_screen);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(0);

	return 0;
}

static void ScreenCreate_default(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &defaultScheme);
    laLayer_SetBufferCount(layer0, 2);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    ImageSequenceWidget1 = laImageSequenceWidget_New();
    laWidget_SetPosition((laWidget*)ImageSequenceWidget1, 450, 7);
    laWidget_SetSize((laWidget*)ImageSequenceWidget1, 28, 70);
    laWidget_SetScheme((laWidget*)ImageSequenceWidget1, &defaultPaletteScheme);
    laWidget_SetBackgroundType((laWidget*)ImageSequenceWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageSequenceWidget1, LA_WIDGET_BORDER_NONE);
    laImageSequenceWidget_SetImageCount(ImageSequenceWidget1, 7);
    laImageSequenceWidget_SetImage(ImageSequenceWidget1, 0, &darkGreenBat);
    laImageSequenceWidget_SetImage(ImageSequenceWidget1, 1, &lightGreenBat);
    laImageSequenceWidget_SetImage(ImageSequenceWidget1, 2, &yellowBat);
    laImageSequenceWidget_SetImage(ImageSequenceWidget1, 3, &orangeBat);
    laImageSequenceWidget_SetImage(ImageSequenceWidget1, 4, &redOrangeBat);
    laImageSequenceWidget_SetImage(ImageSequenceWidget1, 5, &redBat);
    laImageSequenceWidget_SetImage(ImageSequenceWidget1, 6, &emptyBat);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageSequenceWidget1);

    LabelWidget12 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget12, 185, 16);
    laWidget_SetSize((laWidget*)LabelWidget12, 110, 25);
    laWidget_SetScheme((laWidget*)LabelWidget12, &LabelScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget12, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget12, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget12, laString_CreateFromID(string_GFX_BasicMotion));
    laLabelWidget_SetHAlignment(LabelWidget12, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget12);

    ImageSequenceWidget = laImageSequenceWidget_New();
    laWidget_SetPosition((laWidget*)ImageSequenceWidget, 140, 76);
    laWidget_SetSize((laWidget*)ImageSequenceWidget, 200, 115);
    laWidget_SetOptimizationFlags((laWidget*)ImageSequenceWidget, LA_WIDGET_OPT_OPAQUE);
    laWidget_SetScheme((laWidget*)ImageSequenceWidget, &defaultPaletteScheme);
    laWidget_SetBackgroundType((laWidget*)ImageSequenceWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageSequenceWidget, LA_WIDGET_BORDER_NONE);
    laImageSequenceWidget_SetImageCount(ImageSequenceWidget, 25);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 0, &mchip_logo_1);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 1, &mchip_logo_2);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 2, &mchip_logo_3);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 3, &mchip_logo_4);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 4, &mchip_logo_5);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 5, &mchip_logo_6);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 6, &mchip_logo_7);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 7, &mchip_logo_8);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 8, &mchip_logo_9);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 9, &mchip_logo_10);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 10, &mchip_logo_11);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 11, &mchip_logo_12);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 12, &mchip_logo_13);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 13, &mchip_logo_14);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 14, &mchip_logo_15);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 15, &mchip_logo_16);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 16, &mchip_logo_17);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 17, &mchip_logo_18);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 18, &mchip_logo_19);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 19, &mchip_logo_20);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 20, &mchip_logo_21);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 21, &mchip_logo_22);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 22, &mchip_logo_23);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 23, &mchip_logo_24);
    laImageSequenceWidget_SetImage(ImageSequenceWidget, 24, &mchip_logo_25);
    laImageSequenceWidget_SetRepeat(ImageSequenceWidget, LA_TRUE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageSequenceWidget);

    ButtonWidget1 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget1, 380, 190);
    laWidget_SetSize((laWidget*)ButtonWidget1, 99, 81);
    laWidget_SetScheme((laWidget*)ButtonWidget1, &defaultPaletteScheme);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget1, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetToggleable(ButtonWidget1, LA_TRUE);
    laButtonWidget_SetPressedImage(ButtonWidget1, &greenPowerBut);
    laButtonWidget_SetReleasedImage(ButtonWidget1, &blackPowerBut);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget1);

    ButtonWidget2 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget2, 90, 210);
    laWidget_SetSize((laWidget*)ButtonWidget2, 90, 50);
    laWidget_SetVisible((laWidget*)ButtonWidget2, LA_FALSE);
    laWidget_SetScheme((laWidget*)ButtonWidget2, &defaultPaletteScheme);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget2, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetToggleable(ButtonWidget2, LA_TRUE);
    laButtonWidget_SetPressedImage(ButtonWidget2, &plugged);
    laButtonWidget_SetReleasedImage(ButtonWidget2, &unplugged);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget2);

    ButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget, 10, 203);
    laWidget_SetSize((laWidget*)ButtonWidget, 70, 65);
    laWidget_SetScheme((laWidget*)ButtonWidget, &defaultPaletteScheme);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedImage(ButtonWidget, &info);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget, &ButtonWidget_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget);

}

static void ScreenCreate_info_screen(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &defaultScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    ButtonWidget4 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget4, 420, 7);
    laWidget_SetSize((laWidget*)ButtonWidget4, 61, 43);
    laWidget_SetScheme((laWidget*)ButtonWidget4, &defaultPaletteScheme);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget4, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedImage(ButtonWidget4, &close);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget4, &ButtonWidget4_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget4);

    LabelWidget5 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget5, 60, 11);
    laWidget_SetSize((laWidget*)LabelWidget5, 356, 40);
    laWidget_SetScheme((laWidget*)LabelWidget5, &LabelScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget5, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget5, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget5, laString_CreateFromID(string_GFX_BasicMotion));
    laLabelWidget_SetHAlignment(LabelWidget5, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget5);

    LabelWidget6 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget6, 20, 79);
    laWidget_SetSize((laWidget*)LabelWidget6, 100, 25);
    laWidget_SetScheme((laWidget*)LabelWidget6, &LabelScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget6, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget6, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget6, laString_CreateFromID(string_Instructions));
    laLabelWidget_SetHAlignment(LabelWidget6, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget6);

    LabelWidget7 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget7, 60, 120);
    laWidget_SetSize((laWidget*)LabelWidget7, 390, 25);
    laWidget_SetScheme((laWidget*)LabelWidget7, &LabelScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget7, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget7, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget7, laString_CreateFromID(string_Line1));
    laLabelWidget_SetHAlignment(LabelWidget7, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget7);

    LabelWidget8 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget8, 62, 156);
    laWidget_SetSize((laWidget*)LabelWidget8, 398, 25);
    laWidget_SetScheme((laWidget*)LabelWidget8, &LabelScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget8, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget8, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget8, laString_CreateFromID(string_Line2));
    laLabelWidget_SetHAlignment(LabelWidget8, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget8);

    LabelWidget9 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget9, 62, 189);
    laWidget_SetSize((laWidget*)LabelWidget9, 398, 25);
    laWidget_SetScheme((laWidget*)LabelWidget9, &LabelScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget9, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget9, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget9, laString_CreateFromID(string_Line3));
    laLabelWidget_SetHAlignment(LabelWidget9, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget9);

    LabelWidget10 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget10, 63, 224);
    laWidget_SetSize((laWidget*)LabelWidget10, 387, 25);
    laWidget_SetScheme((laWidget*)LabelWidget10, &LabelScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget10, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget10, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget10, laString_CreateFromID(string_Line5));
    laLabelWidget_SetHAlignment(LabelWidget10, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget10);

}



