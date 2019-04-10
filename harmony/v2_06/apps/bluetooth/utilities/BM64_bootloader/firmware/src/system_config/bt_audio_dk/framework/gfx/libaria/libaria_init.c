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
laScheme tealtext;
laScheme orangetext;
laScheme redtext;
laScheme blueblackBackground;
laRectangleWidget* GFX_BACKGROUND;
laImageWidget* GFX_MCHPLOGO;
laLabelWidget* GFX_TITLE;


static void ScreenCreate_default(laScreen* screen);


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

    laScheme_Initialize(&tealtext, GFX_COLOR_MODE_RGB_565);
    tealtext.base = 0x1967;
    tealtext.highlight = 0xC67A;
    tealtext.highlightLight = 0xFFFF;
    tealtext.shadow = 0x8410;
    tealtext.shadowDark = 0x4208;
    tealtext.foreground = 0x0;
    tealtext.foregroundInactive = 0xD71C;
    tealtext.foregroundDisabled = 0x8410;
    tealtext.background = 0xFFFF;
    tealtext.backgroundInactive = 0xD71C;
    tealtext.backgroundDisabled = 0xC67A;
    tealtext.text = 0x7FF;
    tealtext.textHighlight = 0x1F;
    tealtext.textHighlightText = 0xFFFF;
    tealtext.textInactive = 0xD71C;
    tealtext.textDisabled = 0x8C92;

    laScheme_Initialize(&orangetext, GFX_COLOR_MODE_RGB_565);
    orangetext.base = 0x1967;
    orangetext.highlight = 0xC67A;
    orangetext.highlightLight = 0xFFFF;
    orangetext.shadow = 0x8410;
    orangetext.shadowDark = 0x4208;
    orangetext.foreground = 0x0;
    orangetext.foregroundInactive = 0xD71C;
    orangetext.foregroundDisabled = 0x8410;
    orangetext.background = 0xFFFF;
    orangetext.backgroundInactive = 0xD71C;
    orangetext.backgroundDisabled = 0xC67A;
    orangetext.text = 0xFC00;
    orangetext.textHighlight = 0x1F;
    orangetext.textHighlightText = 0xFFFF;
    orangetext.textInactive = 0xD71C;
    orangetext.textDisabled = 0x8C92;

    laScheme_Initialize(&redtext, GFX_COLOR_MODE_RGB_565);
    redtext.base = 0x1967;
    redtext.highlight = 0xC67A;
    redtext.highlightLight = 0xFFFF;
    redtext.shadow = 0x8410;
    redtext.shadowDark = 0x4208;
    redtext.foreground = 0x0;
    redtext.foregroundInactive = 0xD71C;
    redtext.foregroundDisabled = 0x8410;
    redtext.background = 0xFFFF;
    redtext.backgroundInactive = 0xD71C;
    redtext.backgroundDisabled = 0xC67A;
    redtext.text = 0xF800;
    redtext.textHighlight = 0x1F;
    redtext.textHighlightText = 0xFFFF;
    redtext.textInactive = 0xD71C;
    redtext.textDisabled = 0x8C92;

    laScheme_Initialize(&blueblackBackground, GFX_COLOR_MODE_RGB_565);
    blueblackBackground.base = 0x1967;
    blueblackBackground.highlight = 0xC67A;
    blueblackBackground.highlightLight = 0xFFFF;
    blueblackBackground.shadow = 0x8410;
    blueblackBackground.shadowDark = 0x4208;
    blueblackBackground.foreground = 0x1967;
    blueblackBackground.foregroundInactive = 0xD71C;
    blueblackBackground.foregroundDisabled = 0x8410;
    blueblackBackground.background = 0xFFFF;
    blueblackBackground.backgroundInactive = 0xD71C;
    blueblackBackground.backgroundDisabled = 0xC67A;
    blueblackBackground.text = 0xFFFF;
    blueblackBackground.textHighlight = 0x1F;
    blueblackBackground.textHighlightText = 0xFFFF;
    blueblackBackground.textInactive = 0xD71C;
    blueblackBackground.textDisabled = 0x8C92;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_default);
    laScreen_SetOrientation(screen, LA_SCREEN_ORIENTATION_270);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(0);

	return 0;
}

static void ScreenCreate_default(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 220, 176);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &defaultScheme);
    laLayer_SetBufferCount(layer0, 1);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    GFX_BACKGROUND = laRectangleWidget_New();
    laWidget_SetSize((laWidget*)GFX_BACKGROUND, 220, 176);
    laWidget_SetScheme((laWidget*)GFX_BACKGROUND, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_BACKGROUND, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_BACKGROUND, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_BACKGROUND);

    GFX_MCHPLOGO = laImageWidget_New();
    laWidget_SetSize((laWidget*)GFX_MCHPLOGO, 122, 30);
    laWidget_SetScheme((laWidget*)GFX_MCHPLOGO, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_MCHPLOGO, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_MCHPLOGO, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_MCHPLOGO, &MCHP_LOGO2);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_MCHPLOGO);

    GFX_TITLE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_TITLE, 55, 33);
    laWidget_SetSize((laWidget*)GFX_TITLE, 110, 25);
    laWidget_SetScheme((laWidget*)GFX_TITLE, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_TITLE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_TITLE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_TITLE, laString_CreateFromID(string_Title));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_TITLE);

}



