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
laLabelWidget* PromptTextLabel;
laImageWidget* ImageWidget1;
laLabelWidget* LabelWidget1;
laListWheelWidget* EncoderList;


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
    laContext_AddScreen(screen);

    laContext_SetPreemptionLevel(LA_PREEMPTION_LEVEL_2);
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
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    PromptTextLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)PromptTextLabel, 45, 70);
    laWidget_SetSize((laWidget*)PromptTextLabel, 362, 52);
    laWidget_SetScheme((laWidget*)PromptTextLabel, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)PromptTextLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PromptTextLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(PromptTextLabel, laString_CreateFromID(string_InsertUSB));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)PromptTextLabel);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 9, 6);
    laWidget_SetSize((laWidget*)ImageWidget1, 163, 36);
    laWidget_SetScheme((laWidget*)ImageWidget1, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &MCHP_LOGO);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    LabelWidget1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget1, 28, 50);
    laWidget_SetSize((laWidget*)LabelWidget1, 183, 25);
    laWidget_SetScheme((laWidget*)LabelWidget1, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget1, laString_CreateFromID(string_DEMO_Name));
    laLabelWidget_SetHAlignment(LabelWidget1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget1);

    EncoderList = laListWheelWidget_New();
    laWidget_SetPosition((laWidget*)EncoderList, 142, 116);
    laWidget_SetSize((laWidget*)EncoderList, 184, 128);
    laWidget_SetBackgroundType((laWidget*)EncoderList, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)EncoderList, LA_WIDGET_BORDER_BEVEL);
    laListWheelWidget_SetAlignment(EncoderList, LA_HALIGN_CENTER);
    laListWheelWidget_SetMaxMomentum(EncoderList, 60);
    laListWheelWidget_AppendItem(EncoderList);
    laListWheelWidget_SetItemText(EncoderList, 0, laString_CreateFromID(string_PCMEncoder));
    laListWheelWidget_AppendItem(EncoderList);
    laListWheelWidget_SetItemText(EncoderList, 1, laString_CreateFromID(string_ADPCMEncoder));
    laListWheelWidget_AppendItem(EncoderList);
    laListWheelWidget_SetItemText(EncoderList, 2, laString_CreateFromID(string_OPUSEncoder));
    laListWheelWidget_AppendItem(EncoderList);
    laListWheelWidget_SetItemText(EncoderList, 3, laString_CreateFromID(string_SPEEXEncoder));
    laListWheelWidget_SetSelectedItemChangedEventCallback(EncoderList, &EncoderList_SelectedItemChangedEvent);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)EncoderList);

}



