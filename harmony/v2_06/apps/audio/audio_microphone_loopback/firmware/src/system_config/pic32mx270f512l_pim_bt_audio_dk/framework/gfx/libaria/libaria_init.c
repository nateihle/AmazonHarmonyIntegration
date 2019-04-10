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

laScheme _default;
laScheme blueblackbackground;
laImageWidget* GFX_MCHPLOGO;
laLabelWidget* GFX_TITLE;
laLabelWidget* GFX_CODEC;


static void ScreenCreate_Screen1(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&_default, GFX_COLOR_MODE_RGB_565);
    _default.base = 0x0;
    _default.highlight = 0xBDF7;
    _default.highlightLight = 0xFFFF;
    _default.shadow = 0x8410;
    _default.shadowDark = 0x4208;
    _default.foreground = 0x4208;
    _default.foregroundInactive = 0xD71C;
    _default.foregroundDisabled = 0xBDF7;
    _default.background = 0xFFFF;
    _default.backgroundInactive = 0xD71C;
    _default.backgroundDisabled = 0xC67A;
    _default.text = 0xF7DE;
    _default.textHighlight = 0x1F;
    _default.textHighlightText = 0xFFFF;
    _default.textInactive = 0xFFFF;
    _default.textDisabled = 0x4208;

    laScheme_Initialize(&blueblackbackground, GFX_COLOR_MODE_RGB_565);
    blueblackbackground.base = 0x1967;
    blueblackbackground.highlight = 0xC67A;
    blueblackbackground.highlightLight = 0xFFFF;
    blueblackbackground.shadow = 0x8410;
    blueblackbackground.shadowDark = 0x4208;
    blueblackbackground.foreground = 0x1967;
    blueblackbackground.foregroundInactive = 0x4228;
    blueblackbackground.foregroundDisabled = 0x8410;
    blueblackbackground.background = 0x1967;
    blueblackbackground.backgroundInactive = 0xD71C;
    blueblackbackground.backgroundDisabled = 0xC67A;
    blueblackbackground.text = 0xFFFF;
    blueblackbackground.textHighlight = 0x1F;
    blueblackbackground.textHighlightText = 0xFFFF;
    blueblackbackground.textInactive = 0xD71C;
    blueblackbackground.textDisabled = 0x8C92;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_TRUE, LA_TRUE, &ScreenCreate_Screen1);
    laScreen_SetOrientation(screen, LA_SCREEN_ORIENTATION_270);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(0);

	return 0;
}

static void ScreenCreate_Screen1(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 220, 176);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &blueblackbackground);
    laLayer_SetBufferCount(layer0, 1);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    GFX_MCHPLOGO = laImageWidget_New();
    laWidget_SetSize((laWidget*)GFX_MCHPLOGO, 128, 30);
    laWidget_SetScheme((laWidget*)GFX_MCHPLOGO, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_MCHPLOGO, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GFX_MCHPLOGO, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_MCHPLOGO, &MCHP_LOGO2);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_MCHPLOGO);

    GFX_TITLE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_TITLE, 15, 41);
    laWidget_SetSize((laWidget*)GFX_TITLE, 189, 22);
    laWidget_SetScheme((laWidget*)GFX_TITLE, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_TITLE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_TITLE, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)GFX_TITLE, 0, 0, 0, 0);
    laLabelWidget_SetText(GFX_TITLE, laString_CreateFromID(string_String0));
    laLabelWidget_SetHAlignment(GFX_TITLE, LA_HALIGN_LEFT);
    laLabelWidget_SetVAlignment(GFX_TITLE, LA_VALIGN_TOP);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_TITLE);

    GFX_CODEC = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_CODEC, 15, 62);
    laWidget_SetSize((laWidget*)GFX_CODEC, 114, 22);
    laWidget_SetScheme((laWidget*)GFX_CODEC, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_CODEC, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_CODEC, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)GFX_CODEC, 0, 0, 0, 0);
    laLabelWidget_SetText(GFX_CODEC, laString_CreateFromID(string_String1));
    laLabelWidget_SetHAlignment(GFX_CODEC, LA_HALIGN_LEFT);
    laLabelWidget_SetVAlignment(GFX_CODEC, LA_VALIGN_TOP);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_CODEC);

}



