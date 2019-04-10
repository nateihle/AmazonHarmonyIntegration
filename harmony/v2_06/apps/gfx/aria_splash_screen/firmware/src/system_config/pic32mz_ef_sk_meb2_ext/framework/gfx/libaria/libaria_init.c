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
laScheme whiteScheme;
laImageWidget* ImageWidget1;
laImageWidget* ImageWidget2;
laWidget* PanelWidget1;
laImageWidget* ImageWidget3;
laImageWidget* ImageWidget4;


static void ScreenCreate_SplashScreen(laScreen* screen);


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

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_SplashScreen);
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



