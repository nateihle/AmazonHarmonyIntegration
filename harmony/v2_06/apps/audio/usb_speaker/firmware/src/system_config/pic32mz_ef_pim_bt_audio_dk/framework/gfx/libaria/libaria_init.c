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

laScheme text_label;
laScheme _default;
laScheme track_time;
laScheme image_button;
laScheme filled_circle;
laScheme track_info;
laRectangleWidget* RectangleWidget1;
laLabelWidget* DemoNameLabel;
laLabelWidget* LabelWidget2;
laLabelWidget* MHVersion;
laImageWidget* ImageWidget1;
laLabelWidget* CodecType;
laLabelWidget* ProcType;


static void ScreenCreate_Welcome(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&text_label, GFX_COLOR_MODE_RGB_565);
    text_label.base = 0xC67A;
    text_label.highlight = 0xC67A;
    text_label.highlightLight = 0xFFFF;
    text_label.shadow = 0x8410;
    text_label.shadowDark = 0x4208;
    text_label.foreground = 0xDEFB;
    text_label.foregroundInactive = 0xD71C;
    text_label.foregroundDisabled = 0x8410;
    text_label.background = 0xFFFF;
    text_label.backgroundInactive = 0xD71C;
    text_label.backgroundDisabled = 0xC67A;
    text_label.text = 0xDEFB;
    text_label.textHighlight = 0x1F;
    text_label.textHighlightText = 0xFFFF;
    text_label.textInactive = 0xD71C;
    text_label.textDisabled = 0x8C92;

    laScheme_Initialize(&_default, GFX_COLOR_MODE_RGB_565);
    _default.base = 0x2104;
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
    _default.text = 0x0;
    _default.textHighlight = 0x1F;
    _default.textHighlightText = 0xFFFF;
    _default.textInactive = 0xFFFF;
    _default.textDisabled = 0x4208;

    laScheme_Initialize(&track_time, GFX_COLOR_MODE_RGB_565);
    track_time.base = 0xC67A;
    track_time.highlight = 0xC67A;
    track_time.highlightLight = 0xFFFF;
    track_time.shadow = 0x8410;
    track_time.shadowDark = 0x4208;
    track_time.foreground = 0x0;
    track_time.foregroundInactive = 0xD71C;
    track_time.foregroundDisabled = 0x8410;
    track_time.background = 0xFFFF;
    track_time.backgroundInactive = 0xD71C;
    track_time.backgroundDisabled = 0xC67A;
    track_time.text = 0xDEFB;
    track_time.textHighlight = 0x1F;
    track_time.textHighlightText = 0xFFFF;
    track_time.textInactive = 0xD71C;
    track_time.textDisabled = 0x8C92;

    laScheme_Initialize(&image_button, GFX_COLOR_MODE_RGB_565);
    image_button.base = 0x2104;
    image_button.highlight = 0xC67A;
    image_button.highlightLight = 0xFFFF;
    image_button.shadow = 0x8410;
    image_button.shadowDark = 0x4208;
    image_button.foreground = 0x0;
    image_button.foregroundInactive = 0xD71C;
    image_button.foregroundDisabled = 0x8410;
    image_button.background = 0x2104;
    image_button.backgroundInactive = 0xD71C;
    image_button.backgroundDisabled = 0xC67A;
    image_button.text = 0x0;
    image_button.textHighlight = 0x1F;
    image_button.textHighlightText = 0xFFFF;
    image_button.textInactive = 0xD71C;
    image_button.textDisabled = 0x8C92;

    laScheme_Initialize(&filled_circle, GFX_COLOR_MODE_RGB_565);
    filled_circle.base = 0xFFFF;
    filled_circle.highlight = 0xC67A;
    filled_circle.highlightLight = 0xFFFF;
    filled_circle.shadow = 0x8410;
    filled_circle.shadowDark = 0x4208;
    filled_circle.foreground = 0xFFFF;
    filled_circle.foregroundInactive = 0xD71C;
    filled_circle.foregroundDisabled = 0x8410;
    filled_circle.background = 0x2104;
    filled_circle.backgroundInactive = 0xD71C;
    filled_circle.backgroundDisabled = 0xC67A;
    filled_circle.text = 0x0;
    filled_circle.textHighlight = 0x1F;
    filled_circle.textHighlightText = 0xFFFF;
    filled_circle.textInactive = 0xD71C;
    filled_circle.textDisabled = 0x8C92;

    laScheme_Initialize(&track_info, GFX_COLOR_MODE_RGB_565);
    track_info.base = 0x9492;
    track_info.highlight = 0xC67A;
    track_info.highlightLight = 0xFFFF;
    track_info.shadow = 0x8410;
    track_info.shadowDark = 0x4208;
    track_info.foreground = 0x8047;
    track_info.foregroundInactive = 0xD71C;
    track_info.foregroundDisabled = 0x8410;
    track_info.background = 0x8410;
    track_info.backgroundInactive = 0xD71C;
    track_info.backgroundDisabled = 0xC67A;
    track_info.text = 0xEF5D;
    track_info.textHighlight = 0x1F;
    track_info.textHighlightText = 0xFFFF;
    track_info.textInactive = 0xD71C;
    track_info.textDisabled = 0x8C92;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_Welcome);
    laScreen_SetOrientation(screen, LA_SCREEN_ORIENTATION_270);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(0);

	return 0;
}

static void ScreenCreate_Welcome(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 220, 176);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    RectangleWidget1 = laRectangleWidget_New();
    laWidget_SetSize((laWidget*)RectangleWidget1, 215, 175);
    laWidget_SetScheme((laWidget*)RectangleWidget1, &_default);
    laWidget_SetBackgroundType((laWidget*)RectangleWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)RectangleWidget1, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)RectangleWidget1);

    DemoNameLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)DemoNameLabel, 6, 46);
    laWidget_SetSize((laWidget*)DemoNameLabel, 113, 25);
    laWidget_SetScheme((laWidget*)DemoNameLabel, &text_label);
    laWidget_SetBackgroundType((laWidget*)DemoNameLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)DemoNameLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(DemoNameLabel, laString_CreateFromID(string_DEMO_NAME));
    laLabelWidget_SetHAlignment(DemoNameLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)DemoNameLabel);

    LabelWidget2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget2, 18, 124);
    laWidget_SetSize((laWidget*)LabelWidget2, 180, 25);
    laWidget_SetScheme((laWidget*)LabelWidget2, &text_label);
    laWidget_SetBackgroundType((laWidget*)LabelWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget2, laString_CreateFromID(string_InsertUSB));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget2);

    MHVersion = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)MHVersion, 148, 45);
    laWidget_SetSize((laWidget*)MHVersion, 63, 25);
    laWidget_SetScheme((laWidget*)MHVersion, &text_label);
    laWidget_SetBackgroundType((laWidget*)MHVersion, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MHVersion, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(MHVersion, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)MHVersion);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 3, 7);
    laWidget_SetSize((laWidget*)ImageWidget1, 146, 34);
    laWidget_SetScheme((laWidget*)ImageWidget1, &image_button);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &MCHP_LOGO);
    laImageWidget_SetHAlignment(ImageWidget1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    CodecType = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CodecType, 6, 66);
    laWidget_SetSize((laWidget*)CodecType, 121, 25);
    laWidget_SetScheme((laWidget*)CodecType, &text_label);
    laWidget_SetBackgroundType((laWidget*)CodecType, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CodecType, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CodecType, laString_CreateFromID(string_CodecType));
    laLabelWidget_SetHAlignment(CodecType, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)CodecType);

    ProcType = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)ProcType, 6, 80);
    laWidget_SetSize((laWidget*)ProcType, 100, 25);
    laWidget_SetScheme((laWidget*)ProcType, &text_label);
    laWidget_SetBackgroundType((laWidget*)ProcType, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ProcType, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(ProcType, laString_CreateFromID(string_ProcType));
    laLabelWidget_SetHAlignment(ProcType, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ProcType);

}



