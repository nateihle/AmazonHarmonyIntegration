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
laScheme buttonScheme;
laScheme labelScheme;
laScheme layerScheme;
laScheme defaultPaletteScheme;
laImageWidget* ImageWidget1;
laButtonWidget* ButtonWidget1;
laLabelWidget* LabelWidget1;


static void ScreenCreate_default(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&defaultScheme, GFX_COLOR_MODE_RGBA_8888);
    defaultScheme.base = 0x90;
    defaultScheme.highlight = 0x90;
    defaultScheme.highlightLight = 0x1;
    defaultScheme.shadow = 0xD2;
    defaultScheme.shadowDark = 0x96;
    defaultScheme.foreground = 0x94;
    defaultScheme.foregroundInactive = 0xCE;
    defaultScheme.foregroundDisabled = 0xD2;
    defaultScheme.background = 0x1;
    defaultScheme.backgroundInactive = 0xCE;
    defaultScheme.backgroundDisabled = 0x90;
    defaultScheme.text = 0x94;
    defaultScheme.textHighlight = 0x5;
    defaultScheme.textHighlightText = 0x1;
    defaultScheme.textInactive = 0xCE;
    defaultScheme.textDisabled = 0x93;

    laScheme_Initialize(&buttonScheme, GFX_COLOR_MODE_RGBA_8888);
    buttonScheme.base = 0x90;
    buttonScheme.highlight = 0x90;
    buttonScheme.highlightLight = 0x1;
    buttonScheme.shadow = 0xD2;
    buttonScheme.shadowDark = 0x96;
    buttonScheme.foreground = 0x94;
    buttonScheme.foregroundInactive = 0xCE;
    buttonScheme.foregroundDisabled = 0xD2;
    buttonScheme.background = 0x1;
    buttonScheme.backgroundInactive = 0xCE;
    buttonScheme.backgroundDisabled = 0x90;
    buttonScheme.text = 0x94;
    buttonScheme.textHighlight = 0x5;
    buttonScheme.textHighlightText = 0x1;
    buttonScheme.textInactive = 0xCE;
    buttonScheme.textDisabled = 0x93;

    laScheme_Initialize(&labelScheme, GFX_COLOR_MODE_RGBA_8888);
    labelScheme.base = 0x90;
    labelScheme.highlight = 0x90;
    labelScheme.highlightLight = 0x1;
    labelScheme.shadow = 0xD2;
    labelScheme.shadowDark = 0x96;
    labelScheme.foreground = 0x94;
    labelScheme.foregroundInactive = 0xCE;
    labelScheme.foregroundDisabled = 0xD2;
    labelScheme.background = 0x1;
    labelScheme.backgroundInactive = 0xCE;
    labelScheme.backgroundDisabled = 0x90;
    labelScheme.text = 0x94;
    labelScheme.textHighlight = 0x5;
    labelScheme.textHighlightText = 0x1;
    labelScheme.textInactive = 0xCE;
    labelScheme.textDisabled = 0x93;

    laScheme_Initialize(&layerScheme, GFX_COLOR_MODE_RGBA_8888);
    layerScheme.base = 0x90;
    layerScheme.highlight = 0x90;
    layerScheme.highlightLight = 0x1;
    layerScheme.shadow = 0xD2;
    layerScheme.shadowDark = 0x96;
    layerScheme.foreground = 0x94;
    layerScheme.foregroundInactive = 0xCE;
    layerScheme.foregroundDisabled = 0xD2;
    layerScheme.background = 0x1;
    layerScheme.backgroundInactive = 0xCE;
    layerScheme.backgroundDisabled = 0x90;
    layerScheme.text = 0x94;
    layerScheme.textHighlight = 0x5;
    layerScheme.textHighlightText = 0x1;
    layerScheme.textInactive = 0xCE;
    layerScheme.textDisabled = 0x93;

    laScheme_Initialize(&defaultPaletteScheme, GFX_COLOR_MODE_RGBA_8888);
    defaultPaletteScheme.base = 0x90;
    defaultPaletteScheme.highlight = 0x90;
    defaultPaletteScheme.highlightLight = 0x1;
    defaultPaletteScheme.shadow = 0xD2;
    defaultPaletteScheme.shadowDark = 0x96;
    defaultPaletteScheme.foreground = 0x94;
    defaultPaletteScheme.foregroundInactive = 0xCE;
    defaultPaletteScheme.foregroundDisabled = 0xD2;
    defaultPaletteScheme.background = 0x1;
    defaultPaletteScheme.backgroundInactive = 0xCE;
    defaultPaletteScheme.backgroundDisabled = 0x90;
    defaultPaletteScheme.text = 0x94;
    defaultPaletteScheme.textHighlight = 0x5;
    defaultPaletteScheme.textHighlightText = 0x1;
    defaultPaletteScheme.textInactive = 0xCE;
    defaultPaletteScheme.textDisabled = 0x93;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    GFX_Set(GFXF_GLOBAL_PALETTE, globalColorPalette);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_default);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(0);

	return 0;
}

static void ScreenCreate_default(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &layerScheme);
    laLayer_SetBufferCount(layer0, 1);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 220, 118);
    laWidget_SetSize((laWidget*)ImageWidget1, 360, 223);
    laWidget_SetScheme((laWidget*)ImageWidget1, &defaultPaletteScheme);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &NewHarmonyLogo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    ButtonWidget1 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget1, 161, 370);
    laWidget_SetSize((laWidget*)ButtonWidget1, 470, 84);
    laWidget_SetScheme((laWidget*)ButtonWidget1, &buttonScheme);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ButtonWidget1, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(ButtonWidget1, laString_CreateFromID(string_Button_Text));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget1);

    LabelWidget1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget1, 300, 40);
    laWidget_SetSize((laWidget*)LabelWidget1, 210, 57);
    laWidget_SetScheme((laWidget*)LabelWidget1, &labelScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget1, laString_CreateFromID(string_Title));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget1);

}



