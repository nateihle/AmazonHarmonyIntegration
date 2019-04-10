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
laWidget* GFX_DEVPANEL;
laLabelWidget* GFX_BTNAME;
laLabelWidget* GFX_BTADDRESS;
laLabelWidget* GFX_BTNAME_VALUE;
laLabelWidget* GFX_BTADDRESS_VALUE;
laWidget* GFX_MIDPANEL;
laImageWidget* GFX_AUDIO_MUTE;
laImageWidget* GFX_AUDIO_PLAY;
laWidget* GFX_LEFTPANEL;
laImageWidget* GFX_PREVIOUS;
laImageWidget* GFX_REWIND;
laWidget* GFX_RIGHTPANEL;
laImageWidget* GFX_FASTFORWARD;
laImageWidget* GFX_NEXT;
laWidget* GFX_VOLPANEL;
laLabelWidget* GFX_VOLUME_VALUE;
laWidget* GFX_CONNECT;
laImageWidget* GFX_CONNECTED;
laImageWidget* GFX_PAIRED;
laImageWidget* GFX_NOPAIR_NOCONNECTION;


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
    laWidget_SetPosition((laWidget*)GFX_TITLE, 1, 37);
    laWidget_SetSize((laWidget*)GFX_TITLE, 218, 25);
    laWidget_SetScheme((laWidget*)GFX_TITLE, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_TITLE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_TITLE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_TITLE, laString_CreateFromID(string_Title));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_TITLE);

    GFX_DEVPANEL = laWidget_New();
    laWidget_SetPosition((laWidget*)GFX_DEVPANEL, 0, 65);
    laWidget_SetSize((laWidget*)GFX_DEVPANEL, 177, 49);
    laWidget_SetScheme((laWidget*)GFX_DEVPANEL, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_DEVPANEL, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_DEVPANEL, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, GFX_DEVPANEL);

    GFX_BTNAME = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_BTNAME, 12, 0);
    laWidget_SetSize((laWidget*)GFX_BTNAME, 44, 22);
    laWidget_SetVisible((laWidget*)GFX_BTNAME, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_BTNAME, &tealtext);
    laWidget_SetBackgroundType((laWidget*)GFX_BTNAME, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_BTNAME, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_BTNAME, laString_CreateFromID(string_DeviceName));
    laLabelWidget_SetHAlignment(GFX_BTNAME, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GFX_DEVPANEL, (laWidget*)GFX_BTNAME);

    GFX_BTADDRESS = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_BTADDRESS, 0, 25);
    laWidget_SetSize((laWidget*)GFX_BTADDRESS, 57, 22);
    laWidget_SetVisible((laWidget*)GFX_BTADDRESS, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_BTADDRESS, &tealtext);
    laWidget_SetBackgroundType((laWidget*)GFX_BTADDRESS, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_BTADDRESS, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_BTADDRESS, laString_CreateFromID(string_DeviceAddress));
    laLabelWidget_SetHAlignment(GFX_BTADDRESS, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GFX_DEVPANEL, (laWidget*)GFX_BTADDRESS);

    GFX_BTNAME_VALUE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_BTNAME_VALUE, 55, 0);
    laWidget_SetSize((laWidget*)GFX_BTNAME_VALUE, 120, 22);
    laWidget_SetScheme((laWidget*)GFX_BTNAME_VALUE, &tealtext);
    laWidget_SetBackgroundType((laWidget*)GFX_BTNAME_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_BTNAME_VALUE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(GFX_BTNAME_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GFX_DEVPANEL, (laWidget*)GFX_BTNAME_VALUE);

    GFX_BTADDRESS_VALUE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_BTADDRESS_VALUE, 55, 25);
    laWidget_SetSize((laWidget*)GFX_BTADDRESS_VALUE, 120, 22);
    laWidget_SetScheme((laWidget*)GFX_BTADDRESS_VALUE, &tealtext);
    laWidget_SetBackgroundType((laWidget*)GFX_BTADDRESS_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_BTADDRESS_VALUE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(GFX_BTADDRESS_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GFX_DEVPANEL, (laWidget*)GFX_BTADDRESS_VALUE);

    GFX_MIDPANEL = laWidget_New();
    laWidget_SetPosition((laWidget*)GFX_MIDPANEL, 94, 141);
    laWidget_SetSize((laWidget*)GFX_MIDPANEL, 32, 25);
    laWidget_SetScheme((laWidget*)GFX_MIDPANEL, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_MIDPANEL, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_MIDPANEL, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, GFX_MIDPANEL);

    GFX_AUDIO_MUTE = laImageWidget_New();
    laWidget_SetSize((laWidget*)GFX_AUDIO_MUTE, 32, 25);
    laWidget_SetVisible((laWidget*)GFX_AUDIO_MUTE, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_AUDIO_MUTE, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_AUDIO_MUTE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_AUDIO_MUTE, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_AUDIO_MUTE, &AudioMute16_2);
    laWidget_AddChild((laWidget*)GFX_MIDPANEL, (laWidget*)GFX_AUDIO_MUTE);

    GFX_AUDIO_PLAY = laImageWidget_New();
    laWidget_SetSize((laWidget*)GFX_AUDIO_PLAY, 32, 25);
    laWidget_SetVisible((laWidget*)GFX_AUDIO_PLAY, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_AUDIO_PLAY, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_AUDIO_PLAY, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_AUDIO_PLAY, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_AUDIO_PLAY, &AudioPlay16_2);
    laWidget_AddChild((laWidget*)GFX_MIDPANEL, (laWidget*)GFX_AUDIO_PLAY);

    GFX_LEFTPANEL = laWidget_New();
    laWidget_SetPosition((laWidget*)GFX_LEFTPANEL, 32, 144);
    laWidget_SetSize((laWidget*)GFX_LEFTPANEL, 28, 19);
    laWidget_SetScheme((laWidget*)GFX_LEFTPANEL, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_LEFTPANEL, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_LEFTPANEL, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, GFX_LEFTPANEL);

    GFX_PREVIOUS = laImageWidget_New();
    laWidget_SetPosition((laWidget*)GFX_PREVIOUS, 13, 0);
    laWidget_SetSize((laWidget*)GFX_PREVIOUS, 15, 19);
    laWidget_SetVisible((laWidget*)GFX_PREVIOUS, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_PREVIOUS, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_PREVIOUS, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_PREVIOUS, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_PREVIOUS, &previous);
    laWidget_AddChild((laWidget*)GFX_LEFTPANEL, (laWidget*)GFX_PREVIOUS);

    GFX_REWIND = laImageWidget_New();
    laWidget_SetSize((laWidget*)GFX_REWIND, 28, 19);
    laWidget_SetVisible((laWidget*)GFX_REWIND, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_REWIND, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_REWIND, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_REWIND, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_REWIND, &rewind_1);
    laWidget_AddChild((laWidget*)GFX_LEFTPANEL, (laWidget*)GFX_REWIND);

    GFX_RIGHTPANEL = laWidget_New();
    laWidget_SetPosition((laWidget*)GFX_RIGHTPANEL, 160, 144);
    laWidget_SetSize((laWidget*)GFX_RIGHTPANEL, 28, 19);
    laWidget_SetScheme((laWidget*)GFX_RIGHTPANEL, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_RIGHTPANEL, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_RIGHTPANEL, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, GFX_RIGHTPANEL);

    GFX_FASTFORWARD = laImageWidget_New();
    laWidget_SetSize((laWidget*)GFX_FASTFORWARD, 28, 19);
    laWidget_SetVisible((laWidget*)GFX_FASTFORWARD, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_FASTFORWARD, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_FASTFORWARD, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_FASTFORWARD, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_FASTFORWARD, &fastforward);
    laWidget_AddChild((laWidget*)GFX_RIGHTPANEL, (laWidget*)GFX_FASTFORWARD);

    GFX_NEXT = laImageWidget_New();
    laWidget_SetSize((laWidget*)GFX_NEXT, 15, 19);
    laWidget_SetVisible((laWidget*)GFX_NEXT, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_NEXT, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_NEXT, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_NEXT, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_NEXT, &next);
    laWidget_AddChild((laWidget*)GFX_RIGHTPANEL, (laWidget*)GFX_NEXT);

    GFX_VOLPANEL = laWidget_New();
    laWidget_SetPosition((laWidget*)GFX_VOLPANEL, 183, 69);
    laWidget_SetSize((laWidget*)GFX_VOLPANEL, 35, 22);
    laWidget_SetScheme((laWidget*)GFX_VOLPANEL, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_VOLPANEL, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_VOLPANEL, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, GFX_VOLPANEL);

    GFX_VOLUME_VALUE = laLabelWidget_New();
    laWidget_SetSize((laWidget*)GFX_VOLUME_VALUE, 35, 22);
    laWidget_SetVisible((laWidget*)GFX_VOLUME_VALUE, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_VOLUME_VALUE, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_VOLUME_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_VOLUME_VALUE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(GFX_VOLUME_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GFX_VOLPANEL, (laWidget*)GFX_VOLUME_VALUE);

    GFX_CONNECT = laWidget_New();
    laWidget_SetPosition((laWidget*)GFX_CONNECT, 190, 0);
    laWidget_SetSize((laWidget*)GFX_CONNECT, 30, 30);
    laWidget_SetScheme((laWidget*)GFX_CONNECT, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_CONNECT, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_CONNECT, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, GFX_CONNECT);

    GFX_CONNECTED = laImageWidget_New();
    laWidget_SetSize((laWidget*)GFX_CONNECTED, 30, 30);
    laWidget_SetVisible((laWidget*)GFX_CONNECTED, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_CONNECTED, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_CONNECTED, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_CONNECTED, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_CONNECTED, &CONNECTED);
    laWidget_AddChild((laWidget*)GFX_CONNECT, (laWidget*)GFX_CONNECTED);

    GFX_PAIRED = laImageWidget_New();
    laWidget_SetSize((laWidget*)GFX_PAIRED, 30, 30);
    laWidget_SetVisible((laWidget*)GFX_PAIRED, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_PAIRED, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_PAIRED, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_PAIRED, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_PAIRED, &PAIRED);
    laWidget_AddChild((laWidget*)GFX_CONNECT, (laWidget*)GFX_PAIRED);

    GFX_NOPAIR_NOCONNECTION = laImageWidget_New();
    laWidget_SetSize((laWidget*)GFX_NOPAIR_NOCONNECTION, 30, 30);
    laWidget_SetScheme((laWidget*)GFX_NOPAIR_NOCONNECTION, &blueblackBackground);
    laWidget_SetBackgroundType((laWidget*)GFX_NOPAIR_NOCONNECTION, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_NOPAIR_NOCONNECTION, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_NOPAIR_NOCONNECTION, &NO_PAIR_NO_CONNECTION);
    laWidget_AddChild((laWidget*)GFX_CONNECT, (laWidget*)GFX_NOPAIR_NOCONNECTION);

}



