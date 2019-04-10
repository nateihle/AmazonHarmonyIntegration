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
laScheme RightEdgeScheme;
laScheme BottomEdgeScheme;
laScheme LeftEdgeScheme;
laScheme TopEdgeScheme;
laScheme TouchTestScheme;
laLineWidget* LeftEdge;
laLineWidget* TopEdge;
laLineWidget* BottomEdge;
laLineWidget* RightEdge;
laTouchTestWidget* TouchTestWidget1;


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
    defaultScheme.foreground = 0xF800;
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

    laScheme_Initialize(&RightEdgeScheme, GFX_COLOR_MODE_RGB_565);
    RightEdgeScheme.base = 0x7E0;
    RightEdgeScheme.highlight = 0xC67A;
    RightEdgeScheme.highlightLight = 0xFFFF;
    RightEdgeScheme.shadow = 0x8410;
    RightEdgeScheme.shadowDark = 0x4208;
    RightEdgeScheme.foreground = 0x7E0;
    RightEdgeScheme.foregroundInactive = 0xD71C;
    RightEdgeScheme.foregroundDisabled = 0x8410;
    RightEdgeScheme.background = 0xFFFF;
    RightEdgeScheme.backgroundInactive = 0xD71C;
    RightEdgeScheme.backgroundDisabled = 0xC67A;
    RightEdgeScheme.text = 0x0;
    RightEdgeScheme.textHighlight = 0x1F;
    RightEdgeScheme.textHighlightText = 0xFFFF;
    RightEdgeScheme.textInactive = 0xD71C;
    RightEdgeScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&BottomEdgeScheme, GFX_COLOR_MODE_RGB_565);
    BottomEdgeScheme.base = 0xF81F;
    BottomEdgeScheme.highlight = 0xC67A;
    BottomEdgeScheme.highlightLight = 0xFFFF;
    BottomEdgeScheme.shadow = 0x8410;
    BottomEdgeScheme.shadowDark = 0x4208;
    BottomEdgeScheme.foreground = 0xF81F;
    BottomEdgeScheme.foregroundInactive = 0xD71C;
    BottomEdgeScheme.foregroundDisabled = 0x8410;
    BottomEdgeScheme.background = 0xFFFF;
    BottomEdgeScheme.backgroundInactive = 0xFFFF;
    BottomEdgeScheme.backgroundDisabled = 0xC67A;
    BottomEdgeScheme.text = 0x0;
    BottomEdgeScheme.textHighlight = 0x1F;
    BottomEdgeScheme.textHighlightText = 0xFFFF;
    BottomEdgeScheme.textInactive = 0xD71C;
    BottomEdgeScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&LeftEdgeScheme, GFX_COLOR_MODE_RGB_565);
    LeftEdgeScheme.base = 0xF800;
    LeftEdgeScheme.highlight = 0xC67A;
    LeftEdgeScheme.highlightLight = 0xFFFF;
    LeftEdgeScheme.shadow = 0x8410;
    LeftEdgeScheme.shadowDark = 0x4208;
    LeftEdgeScheme.foreground = 0xF800;
    LeftEdgeScheme.foregroundInactive = 0xD71C;
    LeftEdgeScheme.foregroundDisabled = 0x8410;
    LeftEdgeScheme.background = 0xFFFF;
    LeftEdgeScheme.backgroundInactive = 0xD71C;
    LeftEdgeScheme.backgroundDisabled = 0xC67A;
    LeftEdgeScheme.text = 0x0;
    LeftEdgeScheme.textHighlight = 0x1F;
    LeftEdgeScheme.textHighlightText = 0xFFFF;
    LeftEdgeScheme.textInactive = 0xD71C;
    LeftEdgeScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&TopEdgeScheme, GFX_COLOR_MODE_RGB_565);
    TopEdgeScheme.base = 0x1F;
    TopEdgeScheme.highlight = 0xC67A;
    TopEdgeScheme.highlightLight = 0xFFFF;
    TopEdgeScheme.shadow = 0x8410;
    TopEdgeScheme.shadowDark = 0x4208;
    TopEdgeScheme.foreground = 0x1F;
    TopEdgeScheme.foregroundInactive = 0xD71C;
    TopEdgeScheme.foregroundDisabled = 0x8410;
    TopEdgeScheme.background = 0xFFFF;
    TopEdgeScheme.backgroundInactive = 0xD71C;
    TopEdgeScheme.backgroundDisabled = 0xC67A;
    TopEdgeScheme.text = 0x0;
    TopEdgeScheme.textHighlight = 0x1F;
    TopEdgeScheme.textHighlightText = 0xFFFF;
    TopEdgeScheme.textInactive = 0xD71C;
    TopEdgeScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&TouchTestScheme, GFX_COLOR_MODE_RGB_565);
    TouchTestScheme.base = 0xC67A;
    TouchTestScheme.highlight = 0xC67A;
    TouchTestScheme.highlightLight = 0xFFFF;
    TouchTestScheme.shadow = 0x8410;
    TouchTestScheme.shadowDark = 0x4208;
    TouchTestScheme.foreground = 0xF800;
    TouchTestScheme.foregroundInactive = 0xD71C;
    TouchTestScheme.foregroundDisabled = 0x8410;
    TouchTestScheme.background = 0xF800;
    TouchTestScheme.backgroundInactive = 0xD71C;
    TouchTestScheme.backgroundDisabled = 0xC67A;
    TouchTestScheme.text = 0x0;
    TouchTestScheme.textHighlight = 0x1F;
    TouchTestScheme.textHighlightText = 0xFFFF;
    TouchTestScheme.textInactive = 0xD71C;
    TouchTestScheme.textDisabled = 0x8C92;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
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
    laWidget_SetScheme((laWidget*)layer0, &defaultScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    LeftEdge = laLineWidget_New();
    laWidget_SetSize((laWidget*)LeftEdge, 4, 480);
    laWidget_SetScheme((laWidget*)LeftEdge, &LeftEdgeScheme);
    laWidget_SetBackgroundType((laWidget*)LeftEdge, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LeftEdge, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)LeftEdge, 0, 0, 0, 0);
    laLineWidget_SetStartPoint(LeftEdge, 0, 0);
    laLineWidget_SetEndPoint(LeftEdge, 0, 0);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LeftEdge);

    TopEdge = laLineWidget_New();
    laWidget_SetSize((laWidget*)TopEdge, 800, 4);
    laWidget_SetScheme((laWidget*)TopEdge, &TopEdgeScheme);
    laWidget_SetBackgroundType((laWidget*)TopEdge, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TopEdge, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)TopEdge, 0, 0, 0, 0);
    laLineWidget_SetStartPoint(TopEdge, 0, 0);
    laLineWidget_SetEndPoint(TopEdge, 0, 0);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)TopEdge);

    BottomEdge = laLineWidget_New();
    laWidget_SetPosition((laWidget*)BottomEdge, 0, 476);
    laWidget_SetSize((laWidget*)BottomEdge, 800, 4);
    laWidget_SetScheme((laWidget*)BottomEdge, &BottomEdgeScheme);
    laWidget_SetBackgroundType((laWidget*)BottomEdge, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)BottomEdge, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)BottomEdge, 0, 0, 0, 0);
    laLineWidget_SetStartPoint(BottomEdge, 0, 0);
    laLineWidget_SetEndPoint(BottomEdge, 0, 0);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)BottomEdge);

    RightEdge = laLineWidget_New();
    laWidget_SetPosition((laWidget*)RightEdge, 796, 0);
    laWidget_SetSize((laWidget*)RightEdge, 4, 480);
    laWidget_SetScheme((laWidget*)RightEdge, &RightEdgeScheme);
    laWidget_SetBackgroundType((laWidget*)RightEdge, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)RightEdge, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)RightEdge, 0, 0, 0, 0);
    laLineWidget_SetStartPoint(RightEdge, 0, 0);
    laLineWidget_SetEndPoint(RightEdge, 0, 0);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)RightEdge);

    TouchTestWidget1 = laTouchTestWidget_New();
    laWidget_SetPosition((laWidget*)TouchTestWidget1, 90, 80);
    laWidget_SetSize((laWidget*)TouchTestWidget1, 620, 310);
    laWidget_SetScheme((laWidget*)TouchTestWidget1, &TouchTestScheme);
    laWidget_SetBackgroundType((laWidget*)TouchTestWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TouchTestWidget1, LA_WIDGET_BORDER_LINE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)TouchTestWidget1);

}



