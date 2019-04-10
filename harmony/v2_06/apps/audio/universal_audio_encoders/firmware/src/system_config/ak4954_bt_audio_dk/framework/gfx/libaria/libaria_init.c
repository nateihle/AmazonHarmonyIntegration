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
laImageWidget* ImageWidget1;
laLabelWidget* DEMO_TEXT_WIDGET;
laWidget* PanelWidget1;
laLabelWidget* PromptTextLabel;
laListWidget* EncoderList;


static void ScreenCreate_default(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&defaultScheme, GFX_COLOR_MODE_RGB_565);
    defaultScheme.base = 0x0;
    defaultScheme.highlight = 0xC67A;
    defaultScheme.highlightLight = 0xFFFF;
    defaultScheme.shadow = 0x8410;
    defaultScheme.shadowDark = 0x4208;
    defaultScheme.foreground = 0x0;
    defaultScheme.foregroundInactive = 0xD71C;
    defaultScheme.foregroundDisabled = 0x8410;
    defaultScheme.background = 0x0;
    defaultScheme.backgroundInactive = 0xD71C;
    defaultScheme.backgroundDisabled = 0xC67A;
    defaultScheme.text = 0xFFFF;
    defaultScheme.textHighlight = 0x1F;
    defaultScheme.textHighlightText = 0xFFFF;
    defaultScheme.textInactive = 0xD71C;
    defaultScheme.textDisabled = 0x8C92;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_default);
    laScreen_SetOrientation(screen, LA_SCREEN_ORIENTATION_270);
    laContext_AddScreen(screen);

    laContext_SetPreemptionLevel(LA_PREEMPTION_LEVEL_1);
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

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetSize((laWidget*)ImageWidget1, 131, 33);
    laWidget_SetScheme((laWidget*)ImageWidget1, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &MCHP_LOGO);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    DEMO_TEXT_WIDGET = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)DEMO_TEXT_WIDGET, 1, 35);
    laWidget_SetSize((laWidget*)DEMO_TEXT_WIDGET, 179, 27);
    laWidget_SetScheme((laWidget*)DEMO_TEXT_WIDGET, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)DEMO_TEXT_WIDGET, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)DEMO_TEXT_WIDGET, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(DEMO_TEXT_WIDGET, laString_CreateFromID(string_DEMO_NAME));
    laLabelWidget_SetHAlignment(DEMO_TEXT_WIDGET, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)DEMO_TEXT_WIDGET);

    PanelWidget1 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget1, 0, 55);
    laWidget_SetSize((laWidget*)PanelWidget1, 218, 25);
    laWidget_SetOptimizationFlags((laWidget*)PanelWidget1, LA_WIDGET_OPT_LOCAL_REDRAW);
    laWidget_SetScheme((laWidget*)PanelWidget1, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget1, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget1);

    PromptTextLabel = laLabelWidget_New();
    laWidget_SetSize((laWidget*)PromptTextLabel, 216, 25);
    laWidget_SetScheme((laWidget*)PromptTextLabel, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)PromptTextLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PromptTextLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(PromptTextLabel, laString_CreateFromID(string_InsertUSB));
    laWidget_AddChild((laWidget*)PanelWidget1, (laWidget*)PromptTextLabel);

    EncoderList = laListWidget_New();
    laWidget_SetPosition((laWidget*)EncoderList, 48, 74);
    laWidget_SetSize((laWidget*)EncoderList, 116, 99);
    laWidget_SetBackgroundType((laWidget*)EncoderList, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)EncoderList, LA_WIDGET_BORDER_BEVEL);
    laListWidget_SetSelectionMode(EncoderList, LA_LIST_WIDGET_SELECTION_MODE_SINGLE);
    laListWidget_SetIconMargin(EncoderList, 1);
    laListWidget_AppendItem(EncoderList);
    laListWidget_SetItemText(EncoderList, 0, laString_CreateFromID(string_PCMEncoder));
    laListWidget_SetItemIcon(EncoderList, 0, &check);
    laListWidget_AppendItem(EncoderList);
    laListWidget_SetItemText(EncoderList, 1, laString_CreateFromID(string_ADPCMEncoder));
    laListWidget_SetItemIcon(EncoderList, 1, &check);
    laListWidget_AppendItem(EncoderList);
    laListWidget_SetItemText(EncoderList, 2, laString_CreateFromID(string_OPUSEncoder));
    laListWidget_SetItemIcon(EncoderList, 2, &cross);
    laListWidget_AppendItem(EncoderList);
    laListWidget_SetItemText(EncoderList, 3, laString_CreateFromID(string_SPEEXEncoder));
    laListWidget_SetItemIcon(EncoderList, 3, &cross);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)EncoderList);

}



