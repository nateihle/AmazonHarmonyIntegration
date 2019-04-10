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
laScheme MP3Scheme;
laScheme ADPCMScheme;
laScheme staticTextScheme;
laScheme WarningTextScheme;
laScheme AACScheme;
laScheme WAVScheme;
laScheme ProgressBarScheme;
laImageWidget* ImageWidget2;
laLabelWidget* LabelWidget1;
laLabelWidget* MessageLabel;
laLabelWidget* LabelWidget;
laLabelWidget* LabelWidget2;
laImageWidget* ImageWidget1;
laLabelWidget* DemoNameLabel;
laWidget* TrackInfoPanel;
laLabelWidget* TrackNameLabel;
laLabelWidget* TrackArtistLabel;
laLabelWidget* TrackAlbumLabel;
laWidget* ProgressBarWidget;
laRectangleWidget* ProgressBarRect;
laImageWidget* CircleIndicator;
laLabelWidget* WAVIndicator;
laLabelWidget* MP3Indicator;
laWidget* PlaytimePanel;
laLabelWidget* PlaytimeLabel;
laWidget* TrackLengthPanel;
laLabelWidget* TrackLengthLabel;
laLabelWidget* AACIndicator;
laLabelWidget* PCMIndicator;


static void ScreenCreate_Welcome(laScreen* screen);
static void ScreenCreate_MainScreen(laScreen* screen);


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
    defaultScheme.text = 0x0;
    defaultScheme.textHighlight = 0x1F;
    defaultScheme.textHighlightText = 0xFFFF;
    defaultScheme.textInactive = 0xD71C;
    defaultScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&MP3Scheme, GFX_COLOR_MODE_RGB_565);
    MP3Scheme.base = 0xD027;
    MP3Scheme.highlight = 0xC67A;
    MP3Scheme.highlightLight = 0xFFFF;
    MP3Scheme.shadow = 0x8410;
    MP3Scheme.shadowDark = 0x4208;
    MP3Scheme.foreground = 0x0;
    MP3Scheme.foregroundInactive = 0xD71C;
    MP3Scheme.foregroundDisabled = 0x8410;
    MP3Scheme.background = 0xFFFF;
    MP3Scheme.backgroundInactive = 0xD71C;
    MP3Scheme.backgroundDisabled = 0xC67A;
    MP3Scheme.text = 0xC638;
    MP3Scheme.textHighlight = 0x1F;
    MP3Scheme.textHighlightText = 0xFFFF;
    MP3Scheme.textInactive = 0xD71C;
    MP3Scheme.textDisabled = 0x8C92;

    laScheme_Initialize(&ADPCMScheme, GFX_COLOR_MODE_RGB_565);
    ADPCMScheme.base = 0xD027;
    ADPCMScheme.highlight = 0xC67A;
    ADPCMScheme.highlightLight = 0xFFFF;
    ADPCMScheme.shadow = 0x8410;
    ADPCMScheme.shadowDark = 0x4208;
    ADPCMScheme.foreground = 0x0;
    ADPCMScheme.foregroundInactive = 0xD71C;
    ADPCMScheme.foregroundDisabled = 0x8410;
    ADPCMScheme.background = 0xFFFF;
    ADPCMScheme.backgroundInactive = 0xD71C;
    ADPCMScheme.backgroundDisabled = 0xC67A;
    ADPCMScheme.text = 0xC638;
    ADPCMScheme.textHighlight = 0x1F;
    ADPCMScheme.textHighlightText = 0xFFFF;
    ADPCMScheme.textInactive = 0xD71C;
    ADPCMScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&staticTextScheme, GFX_COLOR_MODE_RGB_565);
    staticTextScheme.base = 0xC67A;
    staticTextScheme.highlight = 0xC67A;
    staticTextScheme.highlightLight = 0xFFFF;
    staticTextScheme.shadow = 0x8410;
    staticTextScheme.shadowDark = 0x4208;
    staticTextScheme.foreground = 0x0;
    staticTextScheme.foregroundInactive = 0xD71C;
    staticTextScheme.foregroundDisabled = 0x8410;
    staticTextScheme.background = 0xFFFF;
    staticTextScheme.backgroundInactive = 0xD71C;
    staticTextScheme.backgroundDisabled = 0xC67A;
    staticTextScheme.text = 0xFFFF;
    staticTextScheme.textHighlight = 0x1F;
    staticTextScheme.textHighlightText = 0xFFFF;
    staticTextScheme.textInactive = 0xD71C;
    staticTextScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&WarningTextScheme, GFX_COLOR_MODE_RGB_565);
    WarningTextScheme.base = 0xD027;
    WarningTextScheme.highlight = 0xC67A;
    WarningTextScheme.highlightLight = 0xFFFF;
    WarningTextScheme.shadow = 0x8410;
    WarningTextScheme.shadowDark = 0x4208;
    WarningTextScheme.foreground = 0x0;
    WarningTextScheme.foregroundInactive = 0xD71C;
    WarningTextScheme.foregroundDisabled = 0x8410;
    WarningTextScheme.background = 0xFFFF;
    WarningTextScheme.backgroundInactive = 0xD71C;
    WarningTextScheme.backgroundDisabled = 0xC67A;
    WarningTextScheme.text = 0xF800;
    WarningTextScheme.textHighlight = 0x1F;
    WarningTextScheme.textHighlightText = 0xFFFF;
    WarningTextScheme.textInactive = 0xD71C;
    WarningTextScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&AACScheme, GFX_COLOR_MODE_RGB_565);
    AACScheme.base = 0xD027;
    AACScheme.highlight = 0xC67A;
    AACScheme.highlightLight = 0xFFFF;
    AACScheme.shadow = 0x8410;
    AACScheme.shadowDark = 0x4208;
    AACScheme.foreground = 0x0;
    AACScheme.foregroundInactive = 0xD71C;
    AACScheme.foregroundDisabled = 0x8410;
    AACScheme.background = 0xFFFF;
    AACScheme.backgroundInactive = 0xD71C;
    AACScheme.backgroundDisabled = 0xC67A;
    AACScheme.text = 0xC638;
    AACScheme.textHighlight = 0x1F;
    AACScheme.textHighlightText = 0xFFFF;
    AACScheme.textInactive = 0xD71C;
    AACScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&WAVScheme, GFX_COLOR_MODE_RGB_565);
    WAVScheme.base = 0xD027;
    WAVScheme.highlight = 0xC67A;
    WAVScheme.highlightLight = 0xFFFF;
    WAVScheme.shadow = 0x8410;
    WAVScheme.shadowDark = 0x4208;
    WAVScheme.foreground = 0x0;
    WAVScheme.foregroundInactive = 0xD71C;
    WAVScheme.foregroundDisabled = 0x8410;
    WAVScheme.background = 0xFFFF;
    WAVScheme.backgroundInactive = 0xD71C;
    WAVScheme.backgroundDisabled = 0xC67A;
    WAVScheme.text = 0xC638;
    WAVScheme.textHighlight = 0x1F;
    WAVScheme.textHighlightText = 0xFFFF;
    WAVScheme.textInactive = 0xD71C;
    WAVScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&ProgressBarScheme, GFX_COLOR_MODE_RGB_565);
    ProgressBarScheme.base = 0xD00C;
    ProgressBarScheme.highlight = 0xC67A;
    ProgressBarScheme.highlightLight = 0xFFFF;
    ProgressBarScheme.shadow = 0x8410;
    ProgressBarScheme.shadowDark = 0x4208;
    ProgressBarScheme.foreground = 0x0;
    ProgressBarScheme.foregroundInactive = 0xD71C;
    ProgressBarScheme.foregroundDisabled = 0x8410;
    ProgressBarScheme.background = 0xFC10;
    ProgressBarScheme.backgroundInactive = 0xD71C;
    ProgressBarScheme.backgroundDisabled = 0xC67A;
    ProgressBarScheme.text = 0x0;
    ProgressBarScheme.textHighlight = 0x1F;
    ProgressBarScheme.textHighlightText = 0xFFFF;
    ProgressBarScheme.textInactive = 0xD71C;
    ProgressBarScheme.textDisabled = 0x8C92;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_Welcome);
    laScreen_SetOrientation(screen, LA_SCREEN_ORIENTATION_270);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_MainScreen);
    laScreen_SetOrientation(screen, LA_SCREEN_ORIENTATION_270);
    laContext_AddScreen(screen);

    laContext_SetPreemptionLevel(LA_PREEMPTION_LEVEL_2);
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
    laWidget_SetScheme((laWidget*)layer0, &defaultScheme);
    laLayer_SetBufferCount(layer0, 1);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget2 = laImageWidget_New();
    laWidget_SetSize((laWidget*)ImageWidget2, 126, 32);
    laWidget_SetBackgroundType((laWidget*)ImageWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget2, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget2, &MCHP_LOGO);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget2);

    LabelWidget1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget1, 1, 30);
    laWidget_SetSize((laWidget*)LabelWidget1, 126, 25);
    laWidget_SetScheme((laWidget*)LabelWidget1, &staticTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget1, laString_CreateFromID(string_DemoName));
    laLabelWidget_SetHAlignment(LabelWidget1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget1);

    MessageLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)MessageLabel, 4, 105);
    laWidget_SetSize((laWidget*)MessageLabel, 210, 25);
    laWidget_SetScheme((laWidget*)MessageLabel, &staticTextScheme);
    laWidget_SetBackgroundType((laWidget*)MessageLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MessageLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(MessageLabel, laString_CreateFromID(string_InsertUSB));
    laLabelWidget_SetHAlignment(MessageLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)MessageLabel);

    LabelWidget = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget, 3, 55);
    laWidget_SetSize((laWidget*)LabelWidget, 214, 22);
    laWidget_SetScheme((laWidget*)LabelWidget, &WarningTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget, laString_CreateFromID(string_NOPlayerDisplay));
    laLabelWidget_SetHAlignment(LabelWidget, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget);

    LabelWidget2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget2, 0, 78);
    laWidget_SetSize((laWidget*)LabelWidget2, 201, 25);
    laWidget_SetScheme((laWidget*)LabelWidget2, &WarningTextScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget2, laString_CreateFromID(string_NOPlayerDisplay1));
    laLabelWidget_SetHAlignment(LabelWidget2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget2);

}

static void ScreenCreate_MainScreen(laScreen* screen)
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

    ImageWidget1 = laImageWidget_New();
    laWidget_SetSize((laWidget*)ImageWidget1, 125, 32);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &MCHP_LOGO);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    DemoNameLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)DemoNameLabel, 2, 30);
    laWidget_SetSize((laWidget*)DemoNameLabel, 128, 25);
    laWidget_SetScheme((laWidget*)DemoNameLabel, &staticTextScheme);
    laWidget_SetBackgroundType((laWidget*)DemoNameLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)DemoNameLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(DemoNameLabel, laString_CreateFromID(string_DemoName));
    laLabelWidget_SetHAlignment(DemoNameLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)DemoNameLabel);

    TrackInfoPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)TrackInfoPanel, 0, 53);
    laWidget_SetSize((laWidget*)TrackInfoPanel, 220, 54);
    laWidget_SetBackgroundType((laWidget*)TrackInfoPanel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TrackInfoPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, TrackInfoPanel);

    TrackNameLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TrackNameLabel, 0, 3);
    laWidget_SetSize((laWidget*)TrackNameLabel, 220, 22);
    laWidget_SetScheme((laWidget*)TrackNameLabel, &staticTextScheme);
    laWidget_SetBackgroundType((laWidget*)TrackNameLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TrackNameLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TrackNameLabel, laString_CreateFromID(string_TrackName));
    laLabelWidget_SetVAlignment(TrackNameLabel, LA_VALIGN_TOP);
    laWidget_AddChild((laWidget*)TrackInfoPanel, (laWidget*)TrackNameLabel);

    TrackArtistLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TrackArtistLabel, 0, 28);
    laWidget_SetSize((laWidget*)TrackArtistLabel, 105, 25);
    laWidget_SetScheme((laWidget*)TrackArtistLabel, &staticTextScheme);
    laWidget_SetBackgroundType((laWidget*)TrackArtistLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TrackArtistLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TrackArtistLabel, laString_CreateFromID(string_ArtistName));
    laLabelWidget_SetHAlignment(TrackArtistLabel, LA_HALIGN_RIGHT);
    laWidget_AddChild((laWidget*)TrackInfoPanel, (laWidget*)TrackArtistLabel);

    TrackAlbumLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TrackAlbumLabel, 105, 28);
    laWidget_SetSize((laWidget*)TrackAlbumLabel, 111, 25);
    laWidget_SetScheme((laWidget*)TrackAlbumLabel, &staticTextScheme);
    laWidget_SetBackgroundType((laWidget*)TrackAlbumLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TrackAlbumLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TrackAlbumLabel, laString_CreateFromID(string_AlbumName));
    laLabelWidget_SetHAlignment(TrackAlbumLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TrackInfoPanel, (laWidget*)TrackAlbumLabel);

    ProgressBarWidget = laWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarWidget, 21, 126);
    laWidget_SetSize((laWidget*)ProgressBarWidget, 173, 20);
    laWidget_SetOptimizationFlags((laWidget*)ProgressBarWidget, LA_WIDGET_OPT_LOCAL_REDRAW);
    laWidget_SetScheme((laWidget*)ProgressBarWidget, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ProgressBarWidget, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, ProgressBarWidget);

    ProgressBarRect = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)ProgressBarRect, 5, 8);
    laWidget_SetSize((laWidget*)ProgressBarRect, 160, 5);
    laWidget_SetScheme((laWidget*)ProgressBarRect, &ProgressBarScheme);
    laWidget_SetBackgroundType((laWidget*)ProgressBarRect, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ProgressBarRect, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)ProgressBarWidget, (laWidget*)ProgressBarRect);

    CircleIndicator = laImageWidget_New();
    laWidget_SetPosition((laWidget*)CircleIndicator, 5, 0);
    laWidget_SetSize((laWidget*)CircleIndicator, 17, 19);
    laWidget_SetBackgroundType((laWidget*)CircleIndicator, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CircleIndicator, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(CircleIndicator, &progress_indicator_circle);
    laWidget_AddChild((laWidget*)ProgressBarWidget, (laWidget*)CircleIndicator);

    WAVIndicator = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)WAVIndicator, 133, 8);
    laWidget_SetSize((laWidget*)WAVIndicator, 28, 14);
    laWidget_SetScheme((laWidget*)WAVIndicator, &WAVScheme);
    laWidget_SetBackgroundType((laWidget*)WAVIndicator, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)WAVIndicator, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)WAVIndicator, 1, 2, 1, 2);
    laLabelWidget_SetText(WAVIndicator, laString_CreateFromID(string_WAVDecoder));
    laLabelWidget_SetHAlignment(WAVIndicator, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)WAVIndicator);

    MP3Indicator = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)MP3Indicator, 161, 8);
    laWidget_SetSize((laWidget*)MP3Indicator, 22, 14);
    laWidget_SetScheme((laWidget*)MP3Indicator, &MP3Scheme);
    laWidget_SetBackgroundType((laWidget*)MP3Indicator, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MP3Indicator, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)MP3Indicator, 1, 2, 1, 2);
    laLabelWidget_SetText(MP3Indicator, laString_CreateFromID(string_MP3Decoder));
    laLabelWidget_SetHAlignment(MP3Indicator, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)MP3Indicator);

    PlaytimePanel = laWidget_New();
    laWidget_SetPosition((laWidget*)PlaytimePanel, 5, 147);
    laWidget_SetSize((laWidget*)PlaytimePanel, 45, 25);
    laWidget_SetOptimizationFlags((laWidget*)PlaytimePanel, LA_WIDGET_OPT_LOCAL_REDRAW);
    laWidget_SetScheme((laWidget*)PlaytimePanel, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)PlaytimePanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PlaytimePanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PlaytimePanel);

    PlaytimeLabel = laLabelWidget_New();
    laWidget_SetSize((laWidget*)PlaytimeLabel, 40, 20);
    laWidget_SetScheme((laWidget*)PlaytimeLabel, &staticTextScheme);
    laWidget_SetBackgroundType((laWidget*)PlaytimeLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PlaytimeLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(PlaytimeLabel, laString_CreateFromID(string_Playtime));
    laLabelWidget_SetHAlignment(PlaytimeLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PlaytimePanel, (laWidget*)PlaytimeLabel);

    TrackLengthPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)TrackLengthPanel, 171, 148);
    laWidget_SetSize((laWidget*)TrackLengthPanel, 40, 20);
    laWidget_SetOptimizationFlags((laWidget*)TrackLengthPanel, LA_WIDGET_OPT_LOCAL_REDRAW);
    laWidget_SetScheme((laWidget*)TrackLengthPanel, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)TrackLengthPanel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TrackLengthPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, TrackLengthPanel);

    TrackLengthLabel = laLabelWidget_New();
    laWidget_SetSize((laWidget*)TrackLengthLabel, 40, 20);
    laWidget_SetScheme((laWidget*)TrackLengthLabel, &staticTextScheme);
    laWidget_SetBackgroundType((laWidget*)TrackLengthLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TrackLengthLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TrackLengthLabel, laString_CreateFromID(string_TrackLength));
    laLabelWidget_SetHAlignment(TrackLengthLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TrackLengthPanel, (laWidget*)TrackLengthLabel);

    AACIndicator = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)AACIndicator, 183, 8);
    laWidget_SetSize((laWidget*)AACIndicator, 25, 14);
    laWidget_SetScheme((laWidget*)AACIndicator, &AACScheme);
    laWidget_SetBackgroundType((laWidget*)AACIndicator, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)AACIndicator, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)AACIndicator, 1, 2, 1, 2);
    laLabelWidget_SetText(AACIndicator, laString_CreateFromID(string_AACDecoder));
    laLabelWidget_SetHAlignment(AACIndicator, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)AACIndicator);

    PCMIndicator = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)PCMIndicator, 135, 26);
    laWidget_SetSize((laWidget*)PCMIndicator, 38, 13);
    laWidget_SetScheme((laWidget*)PCMIndicator, &ADPCMScheme);
    laWidget_SetBackgroundType((laWidget*)PCMIndicator, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PCMIndicator, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)PCMIndicator, 1, 2, 1, 2);
    laLabelWidget_SetText(PCMIndicator, laString_CreateFromID(string_NOPlayerDisplay));
    laLabelWidget_SetHAlignment(PCMIndicator, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)PCMIndicator);

}



