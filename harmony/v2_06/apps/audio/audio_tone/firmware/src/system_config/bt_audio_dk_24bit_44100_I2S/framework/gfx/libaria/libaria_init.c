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
laImageWidget* GFX_MCHPLOGO;
laLabelWidget* GFX_TONEMODE;
laLabelWidget* GFX_SAMPLEFREQ_VALUE;
laLabelWidget* GFX_RESOLUTION_VALUE;
laRectangleWidget* GFX_F1_BOX;
laLabelWidget* GFX_F1_VALUE;
laRectangleWidget* GFX_F2_BOX;
laLabelWidget* GFX_F2_VALUE;
laRectangleWidget* GFX_TMS_BOX;
laLabelWidget* GFX_TMS_VALUE;
laLabelWidget* GFX_F1HZ;
laLabelWidget* GFX_F2HZ;
laLabelWidget* GFX_TMS;
laLabelWidget* GFX_SAMPLEFREQ;
laLabelWidget* GFX_RESOLUTION;
laLabelWidget* GFX_TITLE;
laLabelWidget* GFX_VOLUME_VALUE;
laLabelWidget* GFX_VOLUME;
laWidget* GFX_MIDPANEL;
laImageWidget* GFX_AUDIO_PLAY;
laImageWidget* GFX_AUDIO_MUTE;


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
    laScreen_SetOrientation(screen, LA_SCREEN_ORIENTATION_270);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(0);

	return 0;
}

static void ScreenCreate_DemoScreen(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 220, 176);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    GFX_BACKGROUND = laRectangleWidget_New();
    laWidget_SetSize((laWidget*)GFX_BACKGROUND, 220, 176);
    laWidget_SetEnabled((laWidget*)GFX_BACKGROUND, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_BACKGROUND, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_BACKGROUND, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_BACKGROUND, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_BACKGROUND);

    GFX_MCHPLOGO = laImageWidget_New();
    laWidget_SetPosition((laWidget*)GFX_MCHPLOGO, 1, 1);
    laWidget_SetSize((laWidget*)GFX_MCHPLOGO, 122, 30);
    laWidget_SetScheme((laWidget*)GFX_MCHPLOGO, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_MCHPLOGO, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_MCHPLOGO, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_MCHPLOGO, &MCHP_LOGO);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_MCHPLOGO);

    GFX_TONEMODE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_TONEMODE, 7, 143);
    laWidget_SetSize((laWidget*)GFX_TONEMODE, 63, 16);
    laWidget_SetScheme((laWidget*)GFX_TONEMODE, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_TONEMODE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_TONEMODE, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)GFX_TONEMODE, 0, 0, 0, 0);
    laLabelWidget_SetVAlignment(GFX_TONEMODE, LA_VALIGN_TOP);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_TONEMODE);

    GFX_SAMPLEFREQ_VALUE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_SAMPLEFREQ_VALUE, 19, 68);
    laWidget_SetSize((laWidget*)GFX_SAMPLEFREQ_VALUE, 58, 20);
    laWidget_SetScheme((laWidget*)GFX_SAMPLEFREQ_VALUE, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_SAMPLEFREQ_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_SAMPLEFREQ_VALUE, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)GFX_SAMPLEFREQ_VALUE, 0, 0, 0, 0);
    laLabelWidget_SetText(GFX_SAMPLEFREQ_VALUE, laString_CreateFromID(string_Sample_Frequency_value));
    laLabelWidget_SetHAlignment(GFX_SAMPLEFREQ_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_SAMPLEFREQ_VALUE);

    GFX_RESOLUTION_VALUE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_RESOLUTION_VALUE, 21, 105);
    laWidget_SetSize((laWidget*)GFX_RESOLUTION_VALUE, 48, 20);
    laWidget_SetScheme((laWidget*)GFX_RESOLUTION_VALUE, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_RESOLUTION_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_RESOLUTION_VALUE, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)GFX_RESOLUTION_VALUE, 0, 0, 0, 0);
    laLabelWidget_SetText(GFX_RESOLUTION_VALUE, laString_CreateFromID(string_Resolution_value));
    laLabelWidget_SetHAlignment(GFX_RESOLUTION_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_RESOLUTION_VALUE);

    GFX_F1_BOX = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)GFX_F1_BOX, 99, 57);
    laWidget_SetSize((laWidget*)GFX_F1_BOX, 50, 25);
    laWidget_SetScheme((laWidget*)GFX_F1_BOX, &redBoxes);
    laWidget_SetBackgroundType((laWidget*)GFX_F1_BOX, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_F1_BOX, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_F1_BOX);

    GFX_F1_VALUE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_F1_VALUE, 101, 60);
    laWidget_SetSize((laWidget*)GFX_F1_VALUE, 46, 20);
    laWidget_SetScheme((laWidget*)GFX_F1_VALUE, &redBoxes);
    laWidget_SetBackgroundType((laWidget*)GFX_F1_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_F1_VALUE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(GFX_F1_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_F1_VALUE);

    GFX_F2_BOX = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)GFX_F2_BOX, 162, 57);
    laWidget_SetSize((laWidget*)GFX_F2_BOX, 50, 25);
    laWidget_SetScheme((laWidget*)GFX_F2_BOX, &redBoxes);
    laWidget_SetBackgroundType((laWidget*)GFX_F2_BOX, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_F2_BOX, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_F2_BOX);

    GFX_F2_VALUE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_F2_VALUE, 164, 60);
    laWidget_SetSize((laWidget*)GFX_F2_VALUE, 46, 20);
    laWidget_SetScheme((laWidget*)GFX_F2_VALUE, &redBoxes);
    laWidget_SetBackgroundType((laWidget*)GFX_F2_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_F2_VALUE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(GFX_F2_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_F2_VALUE);

    GFX_TMS_BOX = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)GFX_TMS_BOX, 99, 102);
    laWidget_SetSize((laWidget*)GFX_TMS_BOX, 50, 25);
    laWidget_SetEnabled((laWidget*)GFX_TMS_BOX, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_TMS_BOX, &redBoxes);
    laWidget_SetBackgroundType((laWidget*)GFX_TMS_BOX, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_TMS_BOX, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_TMS_BOX);

    GFX_TMS_VALUE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_TMS_VALUE, 101, 105);
    laWidget_SetSize((laWidget*)GFX_TMS_VALUE, 46, 20);
    laWidget_SetScheme((laWidget*)GFX_TMS_VALUE, &redBoxes);
    laWidget_SetBackgroundType((laWidget*)GFX_TMS_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_TMS_VALUE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(GFX_TMS_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_TMS_VALUE);

    GFX_F1HZ = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_F1HZ, 99, 37);
    laWidget_SetSize((laWidget*)GFX_F1HZ, 50, 20);
    laWidget_SetScheme((laWidget*)GFX_F1HZ, &labelText);
    laWidget_SetBackgroundType((laWidget*)GFX_F1HZ, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_F1HZ, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_F1HZ, laString_CreateFromID(string_f1_Hz));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_F1HZ);

    GFX_F2HZ = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_F2HZ, 163, 37);
    laWidget_SetSize((laWidget*)GFX_F2HZ, 50, 20);
    laWidget_SetScheme((laWidget*)GFX_F2HZ, &labelText);
    laWidget_SetBackgroundType((laWidget*)GFX_F2HZ, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_F2HZ, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_F2HZ, laString_CreateFromID(string_f2_Hz));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_F2HZ);

    GFX_TMS = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_TMS, 105, 82);
    laWidget_SetSize((laWidget*)GFX_TMS, 50, 20);
    laWidget_SetScheme((laWidget*)GFX_TMS, &labelText);
    laWidget_SetBackgroundType((laWidget*)GFX_TMS, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_TMS, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_TMS, laString_CreateFromID(string_t_ms));
    laLabelWidget_SetHAlignment(GFX_TMS, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_TMS);

    GFX_SAMPLEFREQ = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_SAMPLEFREQ, 3, 53);
    laWidget_SetSize((laWidget*)GFX_SAMPLEFREQ, 78, 20);
    laWidget_SetScheme((laWidget*)GFX_SAMPLEFREQ, &labelText);
    laWidget_SetBackgroundType((laWidget*)GFX_SAMPLEFREQ, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_SAMPLEFREQ, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_SAMPLEFREQ, laString_CreateFromID(string_Sample_Frequency));
    laLabelWidget_SetHAlignment(GFX_SAMPLEFREQ, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_SAMPLEFREQ);

    GFX_RESOLUTION = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_RESOLUTION, 3, 90);
    laWidget_SetSize((laWidget*)GFX_RESOLUTION, 75, 20);
    laWidget_SetScheme((laWidget*)GFX_RESOLUTION, &labelText);
    laWidget_SetBackgroundType((laWidget*)GFX_RESOLUTION, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_RESOLUTION, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_RESOLUTION, laString_CreateFromID(string_Resolution));
    laLabelWidget_SetHAlignment(GFX_RESOLUTION, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_RESOLUTION);

    GFX_TITLE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_TITLE, 4, 30);
    laWidget_SetSize((laWidget*)GFX_TITLE, 80, 20);
    laWidget_SetScheme((laWidget*)GFX_TITLE, &whiteText);
    laWidget_SetBackgroundType((laWidget*)GFX_TITLE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_TITLE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_TITLE, laString_CreateFromID(string_Title));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_TITLE);

    GFX_VOLUME_VALUE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_VOLUME_VALUE, 172, 103);
    laWidget_SetSize((laWidget*)GFX_VOLUME_VALUE, 30, 20);
    laWidget_SetScheme((laWidget*)GFX_VOLUME_VALUE, &whiteText);
    laWidget_SetBackgroundType((laWidget*)GFX_VOLUME_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_VOLUME_VALUE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(GFX_VOLUME_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_VOLUME_VALUE);

    GFX_VOLUME = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_VOLUME, 150, 82);
    laWidget_SetSize((laWidget*)GFX_VOLUME, 70, 20);
    laWidget_SetScheme((laWidget*)GFX_VOLUME, &labelText);
    laWidget_SetBackgroundType((laWidget*)GFX_VOLUME, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_VOLUME, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_VOLUME, laString_CreateFromID(string_Volume));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_VOLUME);

    GFX_MIDPANEL = laWidget_New();
    laWidget_SetPosition((laWidget*)GFX_MIDPANEL, 94, 141);
    laWidget_SetSize((laWidget*)GFX_MIDPANEL, 32, 25);
    laWidget_SetScheme((laWidget*)GFX_MIDPANEL, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_MIDPANEL, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_MIDPANEL, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, GFX_MIDPANEL);

    GFX_AUDIO_PLAY = laImageWidget_New();
    laWidget_SetSize((laWidget*)GFX_AUDIO_PLAY, 32, 25);
    laWidget_SetVisible((laWidget*)GFX_AUDIO_PLAY, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_AUDIO_PLAY, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_AUDIO_PLAY, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_AUDIO_PLAY, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_AUDIO_PLAY, &AudioPlay16_2);
    laWidget_AddChild((laWidget*)GFX_MIDPANEL, (laWidget*)GFX_AUDIO_PLAY);

    GFX_AUDIO_MUTE = laImageWidget_New();
    laWidget_SetSize((laWidget*)GFX_AUDIO_MUTE, 32, 25);
    laWidget_SetVisible((laWidget*)GFX_AUDIO_MUTE, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_AUDIO_MUTE, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_AUDIO_MUTE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_AUDIO_MUTE, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_AUDIO_MUTE, &AudioMute16_2);
    laWidget_AddChild((laWidget*)GFX_MIDPANEL, (laWidget*)GFX_AUDIO_MUTE);

}



