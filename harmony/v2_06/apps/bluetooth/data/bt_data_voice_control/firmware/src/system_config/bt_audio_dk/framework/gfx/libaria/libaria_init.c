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
laScheme RGBLedScheme;
laScheme image_button;
laScheme filled_circle;
laScheme track_info;
laRectangleWidget* BackgroundRect;
laLabelWidget* DemoName;
laLabelWidget* MHVersion;
laLabelWidget* BtName;
laImageWidget* MHCPicon;
laLabelWidget* BtMacAddr;
laWidget* PanelWidget1;
laImageWidget* IconPaired;
laImageWidget* IconNoConnect;
laImageWidget* IconConnected;
laWidget* PanelVLED;
laRectangleWidget* VLED4;
laRectangleWidget* VLED3;
laRectangleWidget* VLED2;
laRectangleWidget* VLED1;
laRectangleWidget* VLED0;
laLabelWidget* LEDTitle;
laWidget* RGBPanel;
laLabelWidget* RGBLabel;
laRectangleWidget* RGBLed;
laLabelWidget* RecvLabel;
laLabelWidget* Line1;
laLabelWidget* Line2;
laLabelWidget* Line3;
laLabelWidget* Line4;


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
    _default.base = 0x1967;
    _default.highlight = 0xBDF7;
    _default.highlightLight = 0xFFFF;
    _default.shadow = 0x8410;
    _default.shadowDark = 0x4208;
    _default.foreground = 0x4208;
    _default.foregroundInactive = 0xD71C;
    _default.foregroundDisabled = 0xBDF7;
    _default.background = 0x4208;
    _default.backgroundInactive = 0x4208;
    _default.backgroundDisabled = 0x0;
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

    laScheme_Initialize(&RGBLedScheme, GFX_COLOR_MODE_RGB_565);
    RGBLedScheme.base = 0xA065;
    RGBLedScheme.highlight = 0xC67A;
    RGBLedScheme.highlightLight = 0xFFFF;
    RGBLedScheme.shadow = 0x8410;
    RGBLedScheme.shadowDark = 0x4208;
    RGBLedScheme.foreground = 0xA065;
    RGBLedScheme.foregroundInactive = 0xD71C;
    RGBLedScheme.foregroundDisabled = 0x8410;
    RGBLedScheme.background = 0xA065;
    RGBLedScheme.backgroundInactive = 0xD71C;
    RGBLedScheme.backgroundDisabled = 0xC67A;
    RGBLedScheme.text = 0x0;
    RGBLedScheme.textHighlight = 0x1F;
    RGBLedScheme.textHighlightText = 0xFFFF;
    RGBLedScheme.textInactive = 0xD71C;
    RGBLedScheme.textDisabled = 0x8C92;

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

    BackgroundRect = laRectangleWidget_New();
    laWidget_SetSize((laWidget*)BackgroundRect, 220, 176);
    laWidget_SetScheme((laWidget*)BackgroundRect, &_default);
    laWidget_SetBackgroundType((laWidget*)BackgroundRect, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)BackgroundRect, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)BackgroundRect);

    DemoName = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)DemoName, 6, 30);
    laWidget_SetSize((laWidget*)DemoName, 156, 25);
    laWidget_SetScheme((laWidget*)DemoName, &text_label);
    laWidget_SetBackgroundType((laWidget*)DemoName, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)DemoName, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(DemoName, laString_CreateFromID(string_DEMO_NAME));
    laLabelWidget_SetHAlignment(DemoName, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)DemoName);

    MHVersion = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)MHVersion, 132, 4);
    laWidget_SetSize((laWidget*)MHVersion, 44, 21);
    laWidget_SetScheme((laWidget*)MHVersion, &text_label);
    laWidget_SetBackgroundType((laWidget*)MHVersion, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MHVersion, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(MHVersion, laString_CreateFromID(string_MHVersion));
    laLabelWidget_SetHAlignment(MHVersion, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)MHVersion);

    BtName = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)BtName, 6, 50);
    laWidget_SetSize((laWidget*)BtName, 104, 17);
    laWidget_SetScheme((laWidget*)BtName, &text_label);
    laWidget_SetBackgroundType((laWidget*)BtName, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BtName, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(BtName, laString_CreateFromID(string_BtDemoName));
    laLabelWidget_SetHAlignment(BtName, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)BtName);

    MHCPicon = laImageWidget_New();
    laWidget_SetSize((laWidget*)MHCPicon, 130, 33);
    laWidget_SetBackgroundType((laWidget*)MHCPicon, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MHCPicon, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(MHCPicon, &MCHP_LOGO);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)MHCPicon);

    BtMacAddr = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)BtMacAddr, 112, 46);
    laWidget_SetSize((laWidget*)BtMacAddr, 139, 25);
    laWidget_SetScheme((laWidget*)BtMacAddr, &text_label);
    laWidget_SetBackgroundType((laWidget*)BtMacAddr, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BtMacAddr, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(BtMacAddr, laString_CreateFromID(string_MacAddr));
    laLabelWidget_SetHAlignment(BtMacAddr, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)BtMacAddr);

    PanelWidget1 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget1, 179, 0);
    laWidget_SetSize((laWidget*)PanelWidget1, 40, 40);
    laWidget_SetScheme((laWidget*)PanelWidget1, &_default);
    laWidget_SetBackgroundType((laWidget*)PanelWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget1, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget1);

    IconPaired = laImageWidget_New();
    laWidget_SetSize((laWidget*)IconPaired, 40, 40);
    laWidget_SetBackgroundType((laWidget*)IconPaired, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)IconPaired, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(IconPaired, &PAIRED_1);
    laWidget_AddChild((laWidget*)PanelWidget1, (laWidget*)IconPaired);

    IconNoConnect = laImageWidget_New();
    laWidget_SetSize((laWidget*)IconNoConnect, 40, 40);
    laWidget_SetScheme((laWidget*)IconNoConnect, &image_button);
    laWidget_SetBackgroundType((laWidget*)IconNoConnect, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)IconNoConnect, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(IconNoConnect, &NO_PAIR_NO_CONNECTION_1);
    laWidget_AddChild((laWidget*)PanelWidget1, (laWidget*)IconNoConnect);

    IconConnected = laImageWidget_New();
    laWidget_SetSize((laWidget*)IconConnected, 40, 40);
    laWidget_SetScheme((laWidget*)IconConnected, &image_button);
    laWidget_SetBackgroundType((laWidget*)IconConnected, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)IconConnected, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(IconConnected, &CONNECTED_1);
    laWidget_AddChild((laWidget*)PanelWidget1, (laWidget*)IconConnected);

    PanelVLED = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelVLED, 8, 141);
    laWidget_SetSize((laWidget*)PanelVLED, 139, 30);
    laWidget_SetScheme((laWidget*)PanelVLED, &_default);
    laWidget_SetBackgroundType((laWidget*)PanelVLED, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelVLED, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelVLED);

    VLED4 = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)VLED4, 25, 15);
    laWidget_SetSize((laWidget*)VLED4, 12, 12);
    laWidget_SetScheme((laWidget*)VLED4, &filled_circle);
    laWidget_SetBackgroundType((laWidget*)VLED4, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)VLED4, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelVLED, (laWidget*)VLED4);

    VLED3 = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)VLED3, 49, 15);
    laWidget_SetSize((laWidget*)VLED3, 12, 12);
    laWidget_SetScheme((laWidget*)VLED3, &filled_circle);
    laWidget_SetBackgroundType((laWidget*)VLED3, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)VLED3, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelVLED, (laWidget*)VLED3);

    VLED2 = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)VLED2, 74, 15);
    laWidget_SetSize((laWidget*)VLED2, 12, 12);
    laWidget_SetScheme((laWidget*)VLED2, &filled_circle);
    laWidget_SetBackgroundType((laWidget*)VLED2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)VLED2, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelVLED, (laWidget*)VLED2);

    VLED1 = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)VLED1, 97, 15);
    laWidget_SetSize((laWidget*)VLED1, 12, 12);
    laWidget_SetScheme((laWidget*)VLED1, &filled_circle);
    laWidget_SetBackgroundType((laWidget*)VLED1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)VLED1, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelVLED, (laWidget*)VLED1);

    VLED0 = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)VLED0, 121, 15);
    laWidget_SetSize((laWidget*)VLED0, 12, 12);
    laWidget_SetScheme((laWidget*)VLED0, &filled_circle);
    laWidget_SetBackgroundType((laWidget*)VLED0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)VLED0, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelVLED, (laWidget*)VLED0);

    LEDTitle = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LEDTitle, -4, 0);
    laWidget_SetSize((laWidget*)LEDTitle, 140, 13);
    laWidget_SetScheme((laWidget*)LEDTitle, &text_label);
    laWidget_SetBackgroundType((laWidget*)LEDTitle, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LEDTitle, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LEDTitle, laString_CreateFromID(string_LEDstring));
    laLabelWidget_SetHAlignment(LEDTitle, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelVLED, (laWidget*)LEDTitle);

    RGBPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)RGBPanel, 176, 128);
    laWidget_SetSize((laWidget*)RGBPanel, 31, 48);
    laWidget_SetScheme((laWidget*)RGBPanel, &_default);
    laWidget_SetBackgroundType((laWidget*)RGBPanel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RGBPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, RGBPanel);

    RGBLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)RGBLabel, -6, 0);
    laWidget_SetSize((laWidget*)RGBLabel, 42, 20);
    laWidget_SetScheme((laWidget*)RGBLabel, &text_label);
    laWidget_SetBackgroundType((laWidget*)RGBLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RGBLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(RGBLabel, laString_CreateFromID(string_RGBLabel));
    laWidget_AddChild((laWidget*)RGBPanel, (laWidget*)RGBLabel);

    RGBLed = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)RGBLed, 4, 19);
    laWidget_SetSize((laWidget*)RGBLed, 23, 24);
    laWidget_SetScheme((laWidget*)RGBLed, &RGBLedScheme);
    laWidget_SetBackgroundType((laWidget*)RGBLed, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)RGBLed, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)RGBPanel, (laWidget*)RGBLed);

    RecvLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)RecvLabel, 21, 74);
    laWidget_SetSize((laWidget*)RecvLabel, 100, 13);
    laWidget_SetScheme((laWidget*)RecvLabel, &text_label);
    laWidget_SetBackgroundType((laWidget*)RecvLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RecvLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(RecvLabel, laString_CreateFromID(string_RecvTest));
    laLabelWidget_SetHAlignment(RecvLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)RecvLabel);

    Line1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Line1, 20, 90);
    laWidget_SetSize((laWidget*)Line1, 129, 10);
    laWidget_SetScheme((laWidget*)Line1, &text_label);
    laWidget_SetBackgroundType((laWidget*)Line1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Line1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Line1, laString_CreateFromID(string_Blank1));
    laLabelWidget_SetHAlignment(Line1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Line1);

    Line2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Line2, 20, 101);
    laWidget_SetSize((laWidget*)Line2, 130, 10);
    laWidget_SetScheme((laWidget*)Line2, &text_label);
    laWidget_SetBackgroundType((laWidget*)Line2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Line2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Line2, laString_CreateFromID(string_Blank1));
    laLabelWidget_SetHAlignment(Line2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Line2);

    Line3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Line3, 20, 112);
    laWidget_SetSize((laWidget*)Line3, 130, 10);
    laWidget_SetScheme((laWidget*)Line3, &text_label);
    laWidget_SetBackgroundType((laWidget*)Line3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Line3, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Line3, laString_CreateFromID(string_Blank1));
    laLabelWidget_SetHAlignment(Line3, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Line3);

    Line4 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Line4, 20, 123);
    laWidget_SetSize((laWidget*)Line4, 130, 10);
    laWidget_SetScheme((laWidget*)Line4, &text_label);
    laWidget_SetBackgroundType((laWidget*)Line4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Line4, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Line4, laString_CreateFromID(string_Blank1));
    laLabelWidget_SetHAlignment(Line4, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Line4);

}



