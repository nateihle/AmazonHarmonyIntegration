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
laScheme NewScheme;
laWidget* DragArea;
laWidget* ScrollArea;
laButtonWidget* ButtonUS;
laButtonWidget* ButtonCA;
laButtonWidget* ButtonFR;
laButtonWidget* ButtonUK;
laButtonWidget* ButtonCH;
laLabelWidget* Title;
laLabelWidget* Instructions;


static void ScreenCreate_default(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&defaultScheme, GFX_COLOR_MODE_RGB_565);
    defaultScheme.base = 0xFC;
    defaultScheme.highlight = 0xFC;
    defaultScheme.highlightLight = 0xF;
    defaultScheme.shadow = 0x8;
    defaultScheme.shadowDark = 0xED;
    defaultScheme.foreground = 0x0;
    defaultScheme.foregroundInactive = 0xFE;
    defaultScheme.foregroundDisabled = 0x8;
    defaultScheme.background = 0xF;
    defaultScheme.backgroundInactive = 0xFE;
    defaultScheme.backgroundDisabled = 0xFC;
    defaultScheme.text = 0x0;
    defaultScheme.textHighlight = 0xC;
    defaultScheme.textHighlightText = 0xF;
    defaultScheme.textInactive = 0xFE;
    defaultScheme.textDisabled = 0xF6;

    laScheme_Initialize(&NewScheme, GFX_COLOR_MODE_RGB_565);
    NewScheme.base = 0xF;
    NewScheme.highlight = 0xFC;
    NewScheme.highlightLight = 0xF;
    NewScheme.shadow = 0x8;
    NewScheme.shadowDark = 0xED;
    NewScheme.foreground = 0x0;
    NewScheme.foregroundInactive = 0xFE;
    NewScheme.foregroundDisabled = 0x8;
    NewScheme.background = 0xF;
    NewScheme.backgroundInactive = 0xFE;
    NewScheme.backgroundDisabled = 0xFC;
    NewScheme.text = 0x0;
    NewScheme.textHighlight = 0xC;
    NewScheme.textHighlightText = 0xF;
    NewScheme.textInactive = 0xFE;
    NewScheme.textDisabled = 0xF6;

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
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &defaultScheme);
    laLayer_SetBufferCount(layer0, 2);

    laScreen_SetLayer(screen, 0, layer0);

    DragArea = laWidget_New();
    laWidget_SetSize((laWidget*)DragArea, 480, 272);
    laWidget_SetScheme((laWidget*)DragArea, &NewScheme);
    laWidget_SetBackgroundType((laWidget*)DragArea, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)DragArea, LA_WIDGET_BORDER_LINE);
    laWidget_AddChild((laWidget*)layer0, DragArea);

    ScrollArea = laWidget_New();
    laWidget_SetPosition((laWidget*)ScrollArea, 1, 80);
    laWidget_SetSize((laWidget*)ScrollArea, 800, 101);
    laWidget_SetScheme((laWidget*)ScrollArea, &NewScheme);
    laWidget_SetBackgroundType((laWidget*)ScrollArea, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ScrollArea, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)DragArea, ScrollArea);

    ButtonUS = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonUS, 31, 0);
    laWidget_SetSize((laWidget*)ButtonUS, 100, 100);
    laWidget_SetScheme((laWidget*)ButtonUS, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)ButtonUS, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ButtonUS, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetPressedImage(ButtonUS, &USA);
    laButtonWidget_SetReleasedImage(ButtonUS, &USA);
    laWidget_AddChild((laWidget*)ScrollArea, (laWidget*)ButtonUS);

    ButtonCA = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonCA, 190, 0);
    laWidget_SetSize((laWidget*)ButtonCA, 100, 100);
    laWidget_SetScheme((laWidget*)ButtonCA, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)ButtonCA, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ButtonCA, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetPressedImage(ButtonCA, &Canada);
    laButtonWidget_SetReleasedImage(ButtonCA, &Canada);
    laWidget_AddChild((laWidget*)ScrollArea, (laWidget*)ButtonCA);

    ButtonFR = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonFR, 350, 0);
    laWidget_SetSize((laWidget*)ButtonFR, 100, 100);
    laWidget_SetScheme((laWidget*)ButtonFR, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)ButtonFR, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ButtonFR, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetPressedImage(ButtonFR, &France);
    laButtonWidget_SetReleasedImage(ButtonFR, &France);
    laWidget_AddChild((laWidget*)ScrollArea, (laWidget*)ButtonFR);

    ButtonUK = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonUK, 510, 0);
    laWidget_SetSize((laWidget*)ButtonUK, 100, 100);
    laWidget_SetScheme((laWidget*)ButtonUK, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)ButtonUK, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ButtonUK, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetPressedImage(ButtonUK, &UK);
    laButtonWidget_SetReleasedImage(ButtonUK, &UK);
    laWidget_AddChild((laWidget*)ScrollArea, (laWidget*)ButtonUK);

    ButtonCH = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonCH, 671, 0);
    laWidget_SetSize((laWidget*)ButtonCH, 100, 100);
    laWidget_SetScheme((laWidget*)ButtonCH, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)ButtonCH, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ButtonCH, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetPressedImage(ButtonCH, &China);
    laButtonWidget_SetReleasedImage(ButtonCH, &China);
    laWidget_AddChild((laWidget*)ScrollArea, (laWidget*)ButtonCH);

    Title = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Title, 146, 6);
    laWidget_SetSize((laWidget*)Title, 185, 25);
    laWidget_SetScheme((laWidget*)Title, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)Title, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Title, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Title, laString_CreateFromID(string_Title));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Title);

    Instructions = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Instructions, 109, 236);
    laWidget_SetSize((laWidget*)Instructions, 262, 25);
    laWidget_SetScheme((laWidget*)Instructions, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)Instructions, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Instructions, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Instructions, laString_CreateFromID(string_Instructions));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Instructions);

}



