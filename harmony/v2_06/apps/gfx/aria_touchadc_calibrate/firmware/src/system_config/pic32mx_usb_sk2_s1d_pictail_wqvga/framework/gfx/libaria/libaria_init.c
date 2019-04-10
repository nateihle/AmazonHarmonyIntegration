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
laScheme CalibrationScheme;
laScheme TestWidgetScheme;
laImageWidget* ImageWidget;
laImageWidget* ImageWidget;
laImageWidget* ImageWidget;
laLabelWidget* LabelWidget1;
laButtonWidget* ButtonWidget1;
laButtonWidget* ButtonWidget;
laTouchTestWidget* TouchTestWidget3;
laButtonWidget* ButtonWidget2;
laLabelWidget* LabelWidget;
laKeyPadWidget* KeyPadWidget1;
laImageWidget* ImageWidget1;
laImageWidget* ImageWidget2;
laImageWidget* ImageWidget3;
laImageWidget* ImageWidget4;
laLabelWidget* LabelWidget5;
laLabelWidget* LabelWidget6;
laLabelWidget* LabelWidget7;
laLabelWidget* LabelWidget8;
laLabelWidget* Continue;


static void ScreenCreate_splash_screen(laScreen* screen);
static void ScreenCreate_test_screen1(laScreen* screen);
static void ScreenCreate_test_screen2(laScreen* screen);
static void ScreenCreate_calibration_screen(laScreen* screen);


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

    laScheme_Initialize(&CalibrationScheme, GFX_COLOR_MODE_RGB_565);
    CalibrationScheme.base = 0x0;
    CalibrationScheme.highlight = 0xC67A;
    CalibrationScheme.highlightLight = 0xFFFF;
    CalibrationScheme.shadow = 0x8410;
    CalibrationScheme.shadowDark = 0x4208;
    CalibrationScheme.foreground = 0x0;
    CalibrationScheme.foregroundInactive = 0xD71C;
    CalibrationScheme.foregroundDisabled = 0x8410;
    CalibrationScheme.background = 0x0;
    CalibrationScheme.backgroundInactive = 0xD71C;
    CalibrationScheme.backgroundDisabled = 0xC67A;
    CalibrationScheme.text = 0xFFFF;
    CalibrationScheme.textHighlight = 0x1F;
    CalibrationScheme.textHighlightText = 0xFFFF;
    CalibrationScheme.textInactive = 0xD71C;
    CalibrationScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&TestWidgetScheme, GFX_COLOR_MODE_RGB_565);
    TestWidgetScheme.base = 0xC67A;
    TestWidgetScheme.highlight = 0xC67A;
    TestWidgetScheme.highlightLight = 0xFFFF;
    TestWidgetScheme.shadow = 0x8410;
    TestWidgetScheme.shadowDark = 0x4208;
    TestWidgetScheme.foreground = 0xF800;
    TestWidgetScheme.foregroundInactive = 0xFC00;
    TestWidgetScheme.foregroundDisabled = 0xFFE0;
    TestWidgetScheme.background = 0x7E0;
    TestWidgetScheme.backgroundInactive = 0x7FF;
    TestWidgetScheme.backgroundDisabled = 0x1F;
    TestWidgetScheme.text = 0x0;
    TestWidgetScheme.textHighlight = 0x1F;
    TestWidgetScheme.textHighlightText = 0xFFFF;
    TestWidgetScheme.textInactive = 0xD71C;
    TestWidgetScheme.textDisabled = 0x8C92;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_splash_screen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_test_screen1);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_test_screen2);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_calibration_screen);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(0);

	return 0;
}

static void ScreenCreate_splash_screen(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 90, 21);
    laWidget_SetSize((laWidget*)ImageWidget, 300, 180);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &NewHarmonyLogo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 0, 207);
    laWidget_SetSize((laWidget*)ImageWidget, 480, 65);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &Bar);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 20, 230);
    laWidget_SetSize((laWidget*)ImageWidget, 144, 39);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &MicrochipLogo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

}

static void ScreenCreate_test_screen1(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    LabelWidget1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget1, 140, 10);
    laWidget_SetSize((laWidget*)LabelWidget1, 200, 39);
    laWidget_SetBackgroundType((laWidget*)LabelWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget1, laString_CreateFromID(string_test_string1));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget1);

    ButtonWidget1 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget1, 5, 12);
    laWidget_SetSize((laWidget*)ButtonWidget1, 119, 40);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ButtonWidget1, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(ButtonWidget1, laString_CreateFromID(string_calibrate_string));
    laButtonWidget_SetReleasedEventCallback(ButtonWidget1, &ButtonWidget1_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget1);

    ButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget, 375, 12);
    laWidget_SetSize((laWidget*)ButtonWidget, 100, 40);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ButtonWidget, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(ButtonWidget, laString_CreateFromID(string_keypad_string));
    laButtonWidget_SetReleasedEventCallback(ButtonWidget, &ButtonWidget_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget);

    TouchTestWidget3 = laTouchTestWidget_New();
    laWidget_SetPosition((laWidget*)TouchTestWidget3, 20, 70);
    laWidget_SetSize((laWidget*)TouchTestWidget3, 440, 180);
    laWidget_SetScheme((laWidget*)TouchTestWidget3, &TestWidgetScheme);
    laWidget_SetBackgroundType((laWidget*)TouchTestWidget3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TouchTestWidget3, LA_WIDGET_BORDER_LINE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)TouchTestWidget3);

}

static void ScreenCreate_test_screen2(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    ButtonWidget2 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget2, 360, 11);
    laWidget_SetSize((laWidget*)ButtonWidget2, 111, 39);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ButtonWidget2, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(ButtonWidget2, laString_CreateFromID(string_touchwidget_string));
    laButtonWidget_SetReleasedEventCallback(ButtonWidget2, &ButtonWidget2_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget2);

    LabelWidget = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget, 180, 20);
    laWidget_SetSize((laWidget*)LabelWidget, 130, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget, laString_CreateFromID(string_test_string2));
    laLabelWidget_SetHAlignment(LabelWidget, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget);

    KeyPadWidget1 = laKeyPadWidget_New(5, 5);
    laWidget_SetPosition((laWidget*)KeyPadWidget1, 10, 61);
    laWidget_SetSize((laWidget*)KeyPadWidget1, 461, 200);
    laWidget_SetBackgroundType((laWidget*)KeyPadWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)KeyPadWidget1, LA_WIDGET_BORDER_BEVEL);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 0, 0, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 0, 0, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 0, 1, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 0, 1, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 0, 2, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 0, 2, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 0, 3, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 0, 3, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 0, 4, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 0, 4, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 1, 0, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 1, 0, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 1, 1, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 1, 1, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 1, 2, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 1, 2, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 1, 3, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 1, 3, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 1, 4, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 1, 4, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 2, 0, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 2, 0, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 2, 1, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 2, 1, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 2, 2, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 2, 2, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 2, 3, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 2, 3, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 2, 4, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 2, 4, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 3, 0, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 3, 0, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 3, 1, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 3, 1, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 3, 2, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 3, 2, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 3, 3, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 3, 3, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 3, 4, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 3, 4, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 4, 0, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 4, 0, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 4, 1, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 4, 1, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 4, 2, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 4, 2, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 4, 3, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 4, 3, LA_RELATIVE_POSITION_BEHIND);
    laKeyPadWidget_SetKeyAction(KeyPadWidget1, 4, 4, LA_KEYPAD_CELL_ACTION_NONE);
    laKeyPadWidget_SetKeyImagePosition(KeyPadWidget1, 4, 4, LA_RELATIVE_POSITION_BEHIND);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)KeyPadWidget1);

}

static void ScreenCreate_calibration_screen(laScreen* screen)
{
    laLayer* layer0;

    laScreen_SetShowEventCallback(screen, &calibration_screen_ShowEvent);

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &CalibrationScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 24, 3);
    laWidget_SetSize((laWidget*)ImageWidget1, 48, 48);
    laWidget_SetVisible((laWidget*)ImageWidget1, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &red_crosshair);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    ImageWidget2 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget2, 408, 3);
    laWidget_SetSize((laWidget*)ImageWidget2, 48, 48);
    laWidget_SetVisible((laWidget*)ImageWidget2, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)ImageWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget2, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget2, &red_crosshair);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget2);

    ImageWidget3 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget3, 408, 220);
    laWidget_SetSize((laWidget*)ImageWidget3, 48, 48);
    laWidget_SetVisible((laWidget*)ImageWidget3, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)ImageWidget3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget3, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget3, &red_crosshair);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget3);

    ImageWidget4 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget4, 24, 220);
    laWidget_SetSize((laWidget*)ImageWidget4, 48, 48);
    laWidget_SetVisible((laWidget*)ImageWidget4, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)ImageWidget4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget4, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget4, &red_crosshair);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget4);

    LabelWidget5 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget5, 55, 54);
    laWidget_SetSize((laWidget*)LabelWidget5, 100, 25);
    laWidget_SetScheme((laWidget*)LabelWidget5, &CalibrationScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget5, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget5, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(LabelWidget5, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget5);

    LabelWidget6 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget6, 336, 55);
    laWidget_SetSize((laWidget*)LabelWidget6, 100, 25);
    laWidget_SetScheme((laWidget*)LabelWidget6, &CalibrationScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget6, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget6, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(LabelWidget6, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget6);

    LabelWidget7 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget7, 336, 199);
    laWidget_SetSize((laWidget*)LabelWidget7, 100, 25);
    laWidget_SetScheme((laWidget*)LabelWidget7, &CalibrationScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget7, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget7, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(LabelWidget7, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget7);

    LabelWidget8 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget8, 55, 199);
    laWidget_SetSize((laWidget*)LabelWidget8, 100, 25);
    laWidget_SetScheme((laWidget*)LabelWidget8, &CalibrationScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget8, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget8, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(LabelWidget8, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget8);

    Continue = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Continue, 130, 86);
    laWidget_SetSize((laWidget*)Continue, 210, 95);
    laWidget_SetVisible((laWidget*)Continue, LA_FALSE);
    laWidget_SetScheme((laWidget*)Continue, &CalibrationScheme);
    laWidget_SetBackgroundType((laWidget*)Continue, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Continue, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Continue, laString_CreateFromID(string_continue_string));
    laLabelWidget_SetHAlignment(Continue, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Continue);

}



