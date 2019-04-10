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

laScheme orangeText;
laScheme whiteText;
laScheme labelText;
laScheme blueblackbackground;
laScheme blackBoxes;
laScheme redBoxes;
laRectangleWidget* GFX_BACKGROUND;
laRectangleWidget* RectangleWidget3;
laImageWidget* GFX_MCHPLOGO;
laLabelWidget* GFX_TITLE;
laGroupBoxWidget* GroupBoxWidget5;
laLabelWidget* GFX_TONEMODE;
laLabelWidget* GFX_SAMPLEFREQ;
laButtonWidget* sinechirp;
laLabelWidget* GFX_SAMPLEFREQ_VALUE;
laLabelWidget* GFX_RESOLUTION;
laLabelWidget* GFX_RESOLUTION_VALUE;
laGroupBoxWidget* GroupBoxWidget1;
laLabelWidget* GFX_F1HZ;
laRectangleWidget* GFX_F1_BOX;
laLabelWidget* GFX_F1_VALUE;
laButtonWidget* f1plus;
laButtonWidget* f1mnus;
laGroupBoxWidget* GroupBoxWidget2;
laLabelWidget* GFX_F2HZ;
laRectangleWidget* GFX_F2_BOX;
laLabelWidget* GFX_F2_VALUE;
laButtonWidget* f2plus;
laButtonWidget* f2minus;
laGroupBoxWidget* GroupBoxWidget3;
laLabelWidget* GFX_TMS;
laRectangleWidget* GFX_TMS_BOX;
laLabelWidget* GFX_TMS_VALUE;
laButtonWidget* tplus;
laButtonWidget* tminus;
laGroupBoxWidget* GroupBoxWidget4;
laLabelWidget* GFX_VOLUME;
laRectangleWidget* GFX_VOLUME_BOX;
laLabelWidget* GFX_VOLUME_VALUE;
laButtonWidget* volumeplus;
laButtonWidget* volumeminus;
laButtonWidget* GFX_PLAYPAUSE;


static void ScreenCreate_DemoScreen(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&orangeText, GFX_COLOR_MODE_RGB_565);
    orangeText.base = 0x1967;
    orangeText.highlight = 0xC67A;
    orangeText.highlightLight = 0xFFFF;
    orangeText.shadow = 0x8410;
    orangeText.shadowDark = 0x4208;
    orangeText.foreground = 0x1967;
    orangeText.foregroundInactive = 0xD71C;
    orangeText.foregroundDisabled = 0x8410;
    orangeText.background = 0xFFFF;
    orangeText.backgroundInactive = 0xD71C;
    orangeText.backgroundDisabled = 0xC67A;
    orangeText.text = 0xFC00;
    orangeText.textHighlight = 0xFFFF;
    orangeText.textHighlightText = 0xFFFF;
    orangeText.textInactive = 0xD71C;
    orangeText.textDisabled = 0x8C92;

    laScheme_Initialize(&whiteText, GFX_COLOR_MODE_RGB_565);
    whiteText.base = 0x1967;
    whiteText.highlight = 0xC67A;
    whiteText.highlightLight = 0xFFFF;
    whiteText.shadow = 0x8410;
    whiteText.shadowDark = 0x4208;
    whiteText.foreground = 0x0;
    whiteText.foregroundInactive = 0xD71C;
    whiteText.foregroundDisabled = 0x8410;
    whiteText.background = 0xFFFF;
    whiteText.backgroundInactive = 0xD71C;
    whiteText.backgroundDisabled = 0xC67A;
    whiteText.text = 0xFFFF;
    whiteText.textHighlight = 0x1F;
    whiteText.textHighlightText = 0xFFFF;
    whiteText.textInactive = 0xD71C;
    whiteText.textDisabled = 0x8C92;

    laScheme_Initialize(&labelText, GFX_COLOR_MODE_RGB_565);
    labelText.base = 0x1967;
    labelText.highlight = 0xC67A;
    labelText.highlightLight = 0xFFFF;
    labelText.shadow = 0x8410;
    labelText.shadowDark = 0x4208;
    labelText.foreground = 0x0;
    labelText.foregroundInactive = 0xD71C;
    labelText.foregroundDisabled = 0x8410;
    labelText.background = 0xFFFF;
    labelText.backgroundInactive = 0xD71C;
    labelText.backgroundDisabled = 0xC67A;
    labelText.text = 0x7FF;
    labelText.textHighlight = 0x1F;
    labelText.textHighlightText = 0xFFFF;
    labelText.textInactive = 0xD71C;
    labelText.textDisabled = 0x8C92;

    laScheme_Initialize(&blueblackbackground, GFX_COLOR_MODE_RGB_565);
    blueblackbackground.base = 0x1967;
    blueblackbackground.highlight = 0xBDF7;
    blueblackbackground.highlightLight = 0xFFFF;
    blueblackbackground.shadow = 0x8410;
    blueblackbackground.shadowDark = 0x4208;
    blueblackbackground.foreground = 0x1967;
    blueblackbackground.foregroundInactive = 0xD71C;
    blueblackbackground.foregroundDisabled = 0x191E;
    blueblackbackground.background = 0xFFFF;
    blueblackbackground.backgroundInactive = 0xD71C;
    blueblackbackground.backgroundDisabled = 0xC67A;
    blueblackbackground.text = 0xF7DE;
    blueblackbackground.textHighlight = 0x1F;
    blueblackbackground.textHighlightText = 0xFFFF;
    blueblackbackground.textInactive = 0xFFFF;
    blueblackbackground.textDisabled = 0x4208;

    laScheme_Initialize(&blackBoxes, GFX_COLOR_MODE_RGB_565);
    blackBoxes.base = 0xBDF7;
    blackBoxes.highlight = 0xC67A;
    blackBoxes.highlightLight = 0xFFFF;
    blackBoxes.shadow = 0x8410;
    blackBoxes.shadowDark = 0x4208;
    blackBoxes.foreground = 0x0;
    blackBoxes.foregroundInactive = 0xD71C;
    blackBoxes.foregroundDisabled = 0x8410;
    blackBoxes.background = 0xFFFF;
    blackBoxes.backgroundInactive = 0xD71C;
    blackBoxes.backgroundDisabled = 0xC67A;
    blackBoxes.text = 0x0;
    blackBoxes.textHighlight = 0x1F;
    blackBoxes.textHighlightText = 0xFFFF;
    blackBoxes.textInactive = 0xD71C;
    blackBoxes.textDisabled = 0x8C92;

    laScheme_Initialize(&redBoxes, GFX_COLOR_MODE_RGB_565);
    redBoxes.base = 0xBDF7;
    redBoxes.highlight = 0xC67A;
    redBoxes.highlightLight = 0x8410;
    redBoxes.shadow = 0x8410;
    redBoxes.shadowDark = 0x4208;
    redBoxes.foreground = 0xF800;
    redBoxes.foregroundInactive = 0xD71C;
    redBoxes.foregroundDisabled = 0x8410;
    redBoxes.background = 0x1967;
    redBoxes.backgroundInactive = 0xD71C;
    redBoxes.backgroundDisabled = 0xC67A;
    redBoxes.text = 0x0;
    redBoxes.textHighlight = 0x1F;
    redBoxes.textHighlightText = 0xFFFF;
    redBoxes.textInactive = 0xD71C;
    redBoxes.textDisabled = 0x8C92;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_DemoScreen);
    laContext_AddScreen(screen);

    laContext_SetPreemptionLevel(LA_PREEMPTION_LEVEL_2);
    laContext_SetActiveScreen(0);

	return 0;
}

static void ScreenCreate_DemoScreen(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    GFX_BACKGROUND = laRectangleWidget_New();
    laWidget_SetSize((laWidget*)GFX_BACKGROUND, 480, 272);
    laWidget_SetEnabled((laWidget*)GFX_BACKGROUND, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_BACKGROUND, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_BACKGROUND, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_BACKGROUND, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_BACKGROUND);

    RectangleWidget3 = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)RectangleWidget3, -1, 4);
    laWidget_SetSize((laWidget*)RectangleWidget3, 480, 272);
    laWidget_SetEnabled((laWidget*)RectangleWidget3, LA_FALSE);
    laWidget_SetScheme((laWidget*)RectangleWidget3, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)RectangleWidget3, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)RectangleWidget3, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)GFX_BACKGROUND, (laWidget*)RectangleWidget3);

    GFX_MCHPLOGO = laImageWidget_New();
    laWidget_SetPosition((laWidget*)GFX_MCHPLOGO, 1, 1);
    laWidget_SetSize((laWidget*)GFX_MCHPLOGO, 122, 30);
    laWidget_SetScheme((laWidget*)GFX_MCHPLOGO, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_MCHPLOGO, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_MCHPLOGO, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_MCHPLOGO, &MCHP_LOGO);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_MCHPLOGO);

    GFX_TITLE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_TITLE, 26, 37);
    laWidget_SetSize((laWidget*)GFX_TITLE, 113, 20);
    laWidget_SetScheme((laWidget*)GFX_TITLE, &whiteText);
    laWidget_SetBackgroundType((laWidget*)GFX_TITLE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_TITLE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_TITLE, laString_CreateFromID(string_Title));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_TITLE);

    GroupBoxWidget5 = laGroupBoxWidget_New();
    laWidget_SetPosition((laWidget*)GroupBoxWidget5, 16, 174);
    laWidget_SetSize((laWidget*)GroupBoxWidget5, 82, 72);
    laWidget_SetScheme((laWidget*)GroupBoxWidget5, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GroupBoxWidget5, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GroupBoxWidget5, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GroupBoxWidget5);

    GFX_TONEMODE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_TONEMODE, 26, 183);
    laWidget_SetSize((laWidget*)GFX_TONEMODE, 63, 16);
    laWidget_SetScheme((laWidget*)GFX_TONEMODE, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_TONEMODE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_TONEMODE, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)GFX_TONEMODE, 0, 0, 0, 0);
    laLabelWidget_SetVAlignment(GFX_TONEMODE, LA_VALIGN_TOP);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_TONEMODE);

    GFX_SAMPLEFREQ = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_SAMPLEFREQ, 21, 69);
    laWidget_SetSize((laWidget*)GFX_SAMPLEFREQ, 78, 20);
    laWidget_SetScheme((laWidget*)GFX_SAMPLEFREQ, &labelText);
    laWidget_SetBackgroundType((laWidget*)GFX_SAMPLEFREQ, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_SAMPLEFREQ, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_SAMPLEFREQ, laString_CreateFromID(string_Sample_Frequency));
    laLabelWidget_SetHAlignment(GFX_SAMPLEFREQ, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_SAMPLEFREQ);

    sinechirp = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)sinechirp, 31, 210);
    laWidget_SetSize((laWidget*)sinechirp, 54, 25);
    laWidget_SetScheme((laWidget*)sinechirp, &blackBoxes);
    laWidget_SetBackgroundType((laWidget*)sinechirp, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)sinechirp, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(sinechirp, laString_CreateFromID(string_Mode));
    laButtonWidget_SetPressedEventCallback(sinechirp, &sinechirp_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(sinechirp, &sinechirp_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)sinechirp);

    GFX_SAMPLEFREQ_VALUE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_SAMPLEFREQ_VALUE, 34, 89);
    laWidget_SetSize((laWidget*)GFX_SAMPLEFREQ_VALUE, 58, 20);
    laWidget_SetScheme((laWidget*)GFX_SAMPLEFREQ_VALUE, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_SAMPLEFREQ_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_SAMPLEFREQ_VALUE, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)GFX_SAMPLEFREQ_VALUE, 0, 0, 0, 0);
    laLabelWidget_SetText(GFX_SAMPLEFREQ_VALUE, laString_CreateFromID(string_Sample_Frequency_value));
    laLabelWidget_SetHAlignment(GFX_SAMPLEFREQ_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_SAMPLEFREQ_VALUE);

    GFX_RESOLUTION = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_RESOLUTION, 28, 123);
    laWidget_SetSize((laWidget*)GFX_RESOLUTION, 75, 20);
    laWidget_SetScheme((laWidget*)GFX_RESOLUTION, &labelText);
    laWidget_SetBackgroundType((laWidget*)GFX_RESOLUTION, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_RESOLUTION, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_RESOLUTION, laString_CreateFromID(string_Resolution));
    laLabelWidget_SetHAlignment(GFX_RESOLUTION, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_RESOLUTION);

    GFX_RESOLUTION_VALUE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_RESOLUTION_VALUE, 42, 143);
    laWidget_SetSize((laWidget*)GFX_RESOLUTION_VALUE, 48, 20);
    laWidget_SetScheme((laWidget*)GFX_RESOLUTION_VALUE, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_RESOLUTION_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_RESOLUTION_VALUE, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)GFX_RESOLUTION_VALUE, 0, 0, 0, 0);
    laLabelWidget_SetText(GFX_RESOLUTION_VALUE, laString_CreateFromID(string_Resolution_value));
    laLabelWidget_SetHAlignment(GFX_RESOLUTION_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_RESOLUTION_VALUE);

    GroupBoxWidget1 = laGroupBoxWidget_New();
    laWidget_SetPosition((laWidget*)GroupBoxWidget1, 215, 30);
    laWidget_SetSize((laWidget*)GroupBoxWidget1, 109, 89);
    laWidget_SetScheme((laWidget*)GroupBoxWidget1, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GroupBoxWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GroupBoxWidget1, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GroupBoxWidget1);

    GFX_F1HZ = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_F1HZ, 227, 46);
    laWidget_SetSize((laWidget*)GFX_F1HZ, 50, 20);
    laWidget_SetScheme((laWidget*)GFX_F1HZ, &labelText);
    laWidget_SetBackgroundType((laWidget*)GFX_F1HZ, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_F1HZ, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_F1HZ, laString_CreateFromID(string_f1_Hz));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_F1HZ);

    GFX_F1_BOX = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)GFX_F1_BOX, 226, 69);
    laWidget_SetSize((laWidget*)GFX_F1_BOX, 50, 24);
    laWidget_SetEnabled((laWidget*)GFX_F1_BOX, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_F1_BOX, &redBoxes);
    laWidget_SetBackgroundType((laWidget*)GFX_F1_BOX, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_F1_BOX, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_F1_BOX);

    GFX_F1_VALUE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_F1_VALUE, 228, 71);
    laWidget_SetSize((laWidget*)GFX_F1_VALUE, 46, 20);
    laWidget_SetEnabled((laWidget*)GFX_F1_VALUE, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_F1_VALUE, &redBoxes);
    laWidget_SetBackgroundType((laWidget*)GFX_F1_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_F1_VALUE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(GFX_F1_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_F1_VALUE);

    f1plus = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)f1plus, 293, 43);
    laWidget_SetSize((laWidget*)f1plus, 23, 23);
    laWidget_SetBackgroundType((laWidget*)f1plus, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)f1plus, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(f1plus, laString_CreateFromID(string_Plus));
    laButtonWidget_SetPressedEventCallback(f1plus, &f1plus_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(f1plus, &f1plus_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)f1plus);

    f1mnus = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)f1mnus, 293, 85);
    laWidget_SetSize((laWidget*)f1mnus, 23, 23);
    laWidget_SetBackgroundType((laWidget*)f1mnus, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)f1mnus, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(f1mnus, laString_CreateFromID(string_Minus));
    laButtonWidget_SetPressedEventCallback(f1mnus, &f1mnus_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(f1mnus, &f1mnus_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)f1mnus);

    GroupBoxWidget2 = laGroupBoxWidget_New();
    laWidget_SetPosition((laWidget*)GroupBoxWidget2, 347, 30);
    laWidget_SetSize((laWidget*)GroupBoxWidget2, 109, 89);
    laWidget_SetScheme((laWidget*)GroupBoxWidget2, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GroupBoxWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GroupBoxWidget2, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GroupBoxWidget2);

    GFX_F2HZ = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_F2HZ, 359, 46);
    laWidget_SetSize((laWidget*)GFX_F2HZ, 50, 20);
    laWidget_SetScheme((laWidget*)GFX_F2HZ, &labelText);
    laWidget_SetBackgroundType((laWidget*)GFX_F2HZ, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_F2HZ, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_F2HZ, laString_CreateFromID(string_f2_Hz));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_F2HZ);

    GFX_F2_BOX = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)GFX_F2_BOX, 358, 69);
    laWidget_SetSize((laWidget*)GFX_F2_BOX, 50, 24);
    laWidget_SetEnabled((laWidget*)GFX_F2_BOX, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_F2_BOX, &redBoxes);
    laWidget_SetBackgroundType((laWidget*)GFX_F2_BOX, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_F2_BOX, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_F2_BOX);

    GFX_F2_VALUE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_F2_VALUE, 360, 71);
    laWidget_SetSize((laWidget*)GFX_F2_VALUE, 46, 20);
    laWidget_SetScheme((laWidget*)GFX_F2_VALUE, &redBoxes);
    laWidget_SetBackgroundType((laWidget*)GFX_F2_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_F2_VALUE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(GFX_F2_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_F2_VALUE);

    f2plus = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)f2plus, 423, 43);
    laWidget_SetSize((laWidget*)f2plus, 23, 23);
    laWidget_SetBackgroundType((laWidget*)f2plus, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)f2plus, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(f2plus, laString_CreateFromID(string_Plus));
    laButtonWidget_SetPressedEventCallback(f2plus, &f2plus_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(f2plus, &f2plus_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)f2plus);

    f2minus = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)f2minus, 423, 85);
    laWidget_SetSize((laWidget*)f2minus, 23, 23);
    laWidget_SetBackgroundType((laWidget*)f2minus, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)f2minus, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(f2minus, laString_CreateFromID(string_Minus));
    laButtonWidget_SetPressedEventCallback(f2minus, &f2minus_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(f2minus, &f2minus_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)f2minus);

    GroupBoxWidget3 = laGroupBoxWidget_New();
    laWidget_SetPosition((laWidget*)GroupBoxWidget3, 215, 158);
    laWidget_SetSize((laWidget*)GroupBoxWidget3, 109, 89);
    laWidget_SetScheme((laWidget*)GroupBoxWidget3, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GroupBoxWidget3, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GroupBoxWidget3, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GroupBoxWidget3);

    GFX_TMS = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_TMS, 233, 166);
    laWidget_SetSize((laWidget*)GFX_TMS, 50, 20);
    laWidget_SetScheme((laWidget*)GFX_TMS, &labelText);
    laWidget_SetBackgroundType((laWidget*)GFX_TMS, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_TMS, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_TMS, laString_CreateFromID(string_t_ms));
    laLabelWidget_SetHAlignment(GFX_TMS, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_TMS);

    GFX_TMS_BOX = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)GFX_TMS_BOX, 226, 189);
    laWidget_SetSize((laWidget*)GFX_TMS_BOX, 50, 24);
    laWidget_SetEnabled((laWidget*)GFX_TMS_BOX, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_TMS_BOX, &redBoxes);
    laWidget_SetBackgroundType((laWidget*)GFX_TMS_BOX, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_TMS_BOX, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_TMS_BOX);

    GFX_TMS_VALUE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_TMS_VALUE, 228, 191);
    laWidget_SetSize((laWidget*)GFX_TMS_VALUE, 46, 20);
    laWidget_SetScheme((laWidget*)GFX_TMS_VALUE, &redBoxes);
    laWidget_SetBackgroundType((laWidget*)GFX_TMS_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_TMS_VALUE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(GFX_TMS_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_TMS_VALUE);

    tplus = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)tplus, 293, 173);
    laWidget_SetSize((laWidget*)tplus, 23, 23);
    laWidget_SetBackgroundType((laWidget*)tplus, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)tplus, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(tplus, laString_CreateFromID(string_Plus));
    laButtonWidget_SetPressedEventCallback(tplus, &tplus_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(tplus, &tplus_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)tplus);

    tminus = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)tminus, 293, 215);
    laWidget_SetSize((laWidget*)tminus, 23, 23);
    laWidget_SetBackgroundType((laWidget*)tminus, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)tminus, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(tminus, laString_CreateFromID(string_Minus));
    laButtonWidget_SetPressedEventCallback(tminus, &tminus_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(tminus, &tminus_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)tminus);

    GroupBoxWidget4 = laGroupBoxWidget_New();
    laWidget_SetPosition((laWidget*)GroupBoxWidget4, 347, 157);
    laWidget_SetSize((laWidget*)GroupBoxWidget4, 109, 89);
    laWidget_SetScheme((laWidget*)GroupBoxWidget4, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GroupBoxWidget4, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GroupBoxWidget4, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GroupBoxWidget4);

    GFX_VOLUME = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_VOLUME, 352, 166);
    laWidget_SetSize((laWidget*)GFX_VOLUME, 64, 20);
    laWidget_SetScheme((laWidget*)GFX_VOLUME, &labelText);
    laWidget_SetBackgroundType((laWidget*)GFX_VOLUME, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_VOLUME, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_VOLUME, laString_CreateFromID(string_Volume));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_VOLUME);

    GFX_VOLUME_BOX = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)GFX_VOLUME_BOX, 360, 189);
    laWidget_SetSize((laWidget*)GFX_VOLUME_BOX, 50, 24);
    laWidget_SetEnabled((laWidget*)GFX_VOLUME_BOX, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_VOLUME_BOX, &redBoxes);
    laWidget_SetBackgroundType((laWidget*)GFX_VOLUME_BOX, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_VOLUME_BOX, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_VOLUME_BOX);

    GFX_VOLUME_VALUE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_VOLUME_VALUE, 362, 191);
    laWidget_SetSize((laWidget*)GFX_VOLUME_VALUE, 46, 20);
    laWidget_SetScheme((laWidget*)GFX_VOLUME_VALUE, &redBoxes);
    laWidget_SetBackgroundType((laWidget*)GFX_VOLUME_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_VOLUME_VALUE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(GFX_VOLUME_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_VOLUME_VALUE);

    volumeplus = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)volumeplus, 423, 173);
    laWidget_SetSize((laWidget*)volumeplus, 23, 23);
    laWidget_SetBackgroundType((laWidget*)volumeplus, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)volumeplus, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(volumeplus, laString_CreateFromID(string_Plus));
    laButtonWidget_SetPressedEventCallback(volumeplus, &volumeplus_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(volumeplus, &volumeplus_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)volumeplus);

    volumeminus = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)volumeminus, 423, 215);
    laWidget_SetSize((laWidget*)volumeminus, 23, 23);
    laWidget_SetBackgroundType((laWidget*)volumeminus, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)volumeminus, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(volumeminus, laString_CreateFromID(string_Minus));
    laButtonWidget_SetPressedEventCallback(volumeminus, &volumeminus_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(volumeminus, &volumeminus_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)volumeminus);

    GFX_PLAYPAUSE = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)GFX_PLAYPAUSE, 126, 112);
    laWidget_SetSize((laWidget*)GFX_PLAYPAUSE, 64, 50);
    laWidget_SetBackgroundType((laWidget*)GFX_PLAYPAUSE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_PLAYPAUSE, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetPressedImage(GFX_PLAYPAUSE, &AudioPlay16_3p);
    laButtonWidget_SetReleasedImage(GFX_PLAYPAUSE, &AudioPlay16_3);
    laButtonWidget_SetPressedEventCallback(GFX_PLAYPAUSE, &GFX_PLAYPAUSE_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(GFX_PLAYPAUSE, &GFX_PLAYPAUSE_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_PLAYPAUSE);

}



