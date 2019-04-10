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

laScheme controllerTitle;
laScheme instructionScheme;
laScheme RedScheme;
laScheme helpScheme;
laScheme SettingsScheme;
laScheme GreenScheme;
laScheme infoScreen;
laScheme WhiteScheme;
laScheme YellowScheme;
laScheme clockScheme;
laImageWidget* ImageWidget1;
laImageWidget* ImageWidget2;
laWidget* PanelWidget1;
laImageWidget* ImageWidget3;
laImageWidget* ImageWidget4;
laButtonWidget* ButtonWidget3;
laLabelWidget* CenterClockLabel;
laImageWidget* ImageWidget6;
laButtonWidget* StartButton;
laLabelWidget* LabelWidget;
laLabelWidget* LabelWidget2;
laButtonWidget* ButtonWidget;
laButtonWidget* FishButtonWidget;
laButtonWidget* PizzaButtonWidget;
laButtonWidget* VegeButtonWidget;
laButtonWidget* StartStopButton;
laButtonWidget* DoneButton;
laCircularGaugeWidget* FishGaugeWidget;
laCircularGaugeWidget* PizzaGaugeWidget;
laCircularGaugeWidget* TurkeyGaugeWidget;
laLabelWidget* LabelWidget7;
laButtonWidget* ButtonWidget22;
laLabelWidget* LabelWidget9;
laLabelWidget* LabelWidget13;
laImageWidget* ImageWidget;
laImageWidget* Vegetables;
laImageWidget* Pizza;
laImageWidget* ImageWidget5;
laImageWidget* ImageWidget;
laImageWidget* ImageWidget7;
laImageWidget* ImageWidget11;
laLabelWidget* LabelWidget12;
laLabelWidget* LabelWidget8;
laLabelWidget* LabelWidget10;
laCircularGaugeWidget* CircularGaugeWidget3;
laLabelWidget* LabelWidget4;
laImageWidget* ImageWidget;
laImageWidget* ImageWidget;
laImageWidget* ImageWidget;
laImageWidget* ImageWidget;
laImageWidget* ImageWidget;
laImageWidget* ImageWidget;
laImageWidget* ImageWidget;
laImageWidget* ImageWidget8;
laImageWidget* ImageWidget9;
laImageWidget* ImageWidget10;
laImageWidget* ImageWidget12;
laImageWidget* ImageWidget13;


static void ScreenCreate_splashScreen(laScreen* screen);
static void ScreenCreate_homeScreen(laScreen* screen);
static void ScreenCreate_controllerScreen(laScreen* screen);
static void ScreenCreate_infoScreen(laScreen* screen);
static void ScreenCreate_AssetLayout(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&controllerTitle, GFX_COLOR_MODE_RGB_565);
    controllerTitle.base = 0xC67A;
    controllerTitle.highlight = 0xC67A;
    controllerTitle.highlightLight = 0xFFFF;
    controllerTitle.shadow = 0x8410;
    controllerTitle.shadowDark = 0x4208;
    controllerTitle.foreground = 0x0;
    controllerTitle.foregroundInactive = 0xD71C;
    controllerTitle.foregroundDisabled = 0x8410;
    controllerTitle.background = 0xFC00;
    controllerTitle.backgroundInactive = 0xD71C;
    controllerTitle.backgroundDisabled = 0xC67A;
    controllerTitle.text = 0xFC00;
    controllerTitle.textHighlight = 0x1F;
    controllerTitle.textHighlightText = 0xFFFF;
    controllerTitle.textInactive = 0xD71C;
    controllerTitle.textDisabled = 0x8C92;

    laScheme_Initialize(&instructionScheme, GFX_COLOR_MODE_RGB_565);
    instructionScheme.base = 0xDEFB;
    instructionScheme.highlight = 0xC67A;
    instructionScheme.highlightLight = 0xFFFF;
    instructionScheme.shadow = 0x8410;
    instructionScheme.shadowDark = 0x4208;
    instructionScheme.foreground = 0x0;
    instructionScheme.foregroundInactive = 0xD71C;
    instructionScheme.foregroundDisabled = 0x8410;
    instructionScheme.background = 0xFFFF;
    instructionScheme.backgroundInactive = 0xD71C;
    instructionScheme.backgroundDisabled = 0xC67A;
    instructionScheme.text = 0x0;
    instructionScheme.textHighlight = 0x1F;
    instructionScheme.textHighlightText = 0xFFFF;
    instructionScheme.textInactive = 0xD71C;
    instructionScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&RedScheme, GFX_COLOR_MODE_RGB_565);
    RedScheme.base = 0xC67A;
    RedScheme.highlight = 0xC67A;
    RedScheme.highlightLight = 0xFFFF;
    RedScheme.shadow = 0x8410;
    RedScheme.shadowDark = 0x4208;
    RedScheme.foreground = 0xF800;
    RedScheme.foregroundInactive = 0xD71C;
    RedScheme.foregroundDisabled = 0x8410;
    RedScheme.background = 0xFFFF;
    RedScheme.backgroundInactive = 0xD71C;
    RedScheme.backgroundDisabled = 0xC67A;
    RedScheme.text = 0x0;
    RedScheme.textHighlight = 0x1F;
    RedScheme.textHighlightText = 0xFFFF;
    RedScheme.textInactive = 0xD71C;
    RedScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&helpScheme, GFX_COLOR_MODE_RGB_565);
    helpScheme.base = 0x0;
    helpScheme.highlight = 0xC67A;
    helpScheme.highlightLight = 0x0;
    helpScheme.shadow = 0x8410;
    helpScheme.shadowDark = 0x4208;
    helpScheme.foreground = 0xFFFF;
    helpScheme.foregroundInactive = 0xD71C;
    helpScheme.foregroundDisabled = 0x8410;
    helpScheme.background = 0xFDF0;
    helpScheme.backgroundInactive = 0xD71C;
    helpScheme.backgroundDisabled = 0xC67A;
    helpScheme.text = 0xFFFF;
    helpScheme.textHighlight = 0x1F;
    helpScheme.textHighlightText = 0xFFFF;
    helpScheme.textInactive = 0xD71C;
    helpScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&SettingsScheme, GFX_COLOR_MODE_RGB_565);
    SettingsScheme.base = 0x0;
    SettingsScheme.highlight = 0xC67A;
    SettingsScheme.highlightLight = 0xFFFF;
    SettingsScheme.shadow = 0x8410;
    SettingsScheme.shadowDark = 0x4208;
    SettingsScheme.foreground = 0x0;
    SettingsScheme.foregroundInactive = 0xD71C;
    SettingsScheme.foregroundDisabled = 0x8410;
    SettingsScheme.background = 0x0;
    SettingsScheme.backgroundInactive = 0xD71C;
    SettingsScheme.backgroundDisabled = 0xC67A;
    SettingsScheme.text = 0x0;
    SettingsScheme.textHighlight = 0x1F;
    SettingsScheme.textHighlightText = 0xFFFF;
    SettingsScheme.textInactive = 0xD71C;
    SettingsScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&GreenScheme, GFX_COLOR_MODE_RGB_565);
    GreenScheme.base = 0xC67A;
    GreenScheme.highlight = 0xC67A;
    GreenScheme.highlightLight = 0xFFFF;
    GreenScheme.shadow = 0x8410;
    GreenScheme.shadowDark = 0x4208;
    GreenScheme.foreground = 0x7E0;
    GreenScheme.foregroundInactive = 0xD71C;
    GreenScheme.foregroundDisabled = 0x8410;
    GreenScheme.background = 0xFFFF;
    GreenScheme.backgroundInactive = 0xD71C;
    GreenScheme.backgroundDisabled = 0xC67A;
    GreenScheme.text = 0x0;
    GreenScheme.textHighlight = 0x1F;
    GreenScheme.textHighlightText = 0xFFFF;
    GreenScheme.textInactive = 0xD71C;
    GreenScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&infoScreen, GFX_COLOR_MODE_RGB_565);
    infoScreen.base = 0xC67A;
    infoScreen.highlight = 0xC67A;
    infoScreen.highlightLight = 0xFFFF;
    infoScreen.shadow = 0x8410;
    infoScreen.shadowDark = 0x4208;
    infoScreen.foreground = 0x0;
    infoScreen.foregroundInactive = 0xD71C;
    infoScreen.foregroundDisabled = 0x8410;
    infoScreen.background = 0xBDF7;
    infoScreen.backgroundInactive = 0xD71C;
    infoScreen.backgroundDisabled = 0xC67A;
    infoScreen.text = 0x0;
    infoScreen.textHighlight = 0x1F;
    infoScreen.textHighlightText = 0xFFFF;
    infoScreen.textInactive = 0xD71C;
    infoScreen.textDisabled = 0x8C92;

    laScheme_Initialize(&WhiteScheme, GFX_COLOR_MODE_RGB_565);
    WhiteScheme.base = 0xFFFF;
    WhiteScheme.highlight = 0xC67A;
    WhiteScheme.highlightLight = 0xFFFF;
    WhiteScheme.shadow = 0x8410;
    WhiteScheme.shadowDark = 0x4208;
    WhiteScheme.foreground = 0xFFFF;
    WhiteScheme.foregroundInactive = 0xD71C;
    WhiteScheme.foregroundDisabled = 0x8410;
    WhiteScheme.background = 0xFFFF;
    WhiteScheme.backgroundInactive = 0xD71C;
    WhiteScheme.backgroundDisabled = 0xC67A;
    WhiteScheme.text = 0x0;
    WhiteScheme.textHighlight = 0x1F;
    WhiteScheme.textHighlightText = 0xFFFF;
    WhiteScheme.textInactive = 0xD71C;
    WhiteScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&YellowScheme, GFX_COLOR_MODE_RGB_565);
    YellowScheme.base = 0xC67A;
    YellowScheme.highlight = 0xC67A;
    YellowScheme.highlightLight = 0xFFFF;
    YellowScheme.shadow = 0x8410;
    YellowScheme.shadowDark = 0x4208;
    YellowScheme.foreground = 0xFFE0;
    YellowScheme.foregroundInactive = 0xD71C;
    YellowScheme.foregroundDisabled = 0x8410;
    YellowScheme.background = 0xFFFF;
    YellowScheme.backgroundInactive = 0xD71C;
    YellowScheme.backgroundDisabled = 0xC67A;
    YellowScheme.text = 0x0;
    YellowScheme.textHighlight = 0x1F;
    YellowScheme.textHighlightText = 0xFFFF;
    YellowScheme.textInactive = 0xD71C;
    YellowScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&clockScheme, GFX_COLOR_MODE_RGB_565);
    clockScheme.base = 0xC67A;
    clockScheme.highlight = 0xC67A;
    clockScheme.highlightLight = 0xFFFF;
    clockScheme.shadow = 0x8410;
    clockScheme.shadowDark = 0x4208;
    clockScheme.foreground = 0x7FF;
    clockScheme.foregroundInactive = 0x7FF;
    clockScheme.foregroundDisabled = 0x8410;
    clockScheme.background = 0x0;
    clockScheme.backgroundInactive = 0xD71C;
    clockScheme.backgroundDisabled = 0xC67A;
    clockScheme.text = 0x7FF;
    clockScheme.textHighlight = 0x1F;
    clockScheme.textHighlightText = 0xFFFF;
    clockScheme.textInactive = 0xD71C;
    clockScheme.textDisabled = 0x8C92;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_splashScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_homeScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_controllerScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_infoScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_AssetLayout);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(1);

	return 0;
}

static void ScreenCreate_splashScreen(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);
    laWidget_SetOptimizationFlags((laWidget*)layer0, LA_WIDGET_OPT_DRAW_ONCE);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 120, 40);
    laWidget_SetSize((laWidget*)ImageWidget1, 240, 139);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &PIC32Logo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    ImageWidget2 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget2, 120, 40);
    laWidget_SetSize((laWidget*)ImageWidget2, 240, 139);
    laWidget_SetVisible((laWidget*)ImageWidget2, LA_FALSE);
    laWidget_SetScheme((laWidget*)ImageWidget2, &WhiteScheme);
    laWidget_SetBackgroundType((laWidget*)ImageWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget2, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget2, &HarmonyLogo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget2);

    PanelWidget1 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget1, 0, 207);
    laWidget_SetSize((laWidget*)PanelWidget1, 480, 65);
    laWidget_SetOptimizationFlags((laWidget*)PanelWidget1, LA_WIDGET_OPT_DRAW_ONCE);
    laWidget_SetScheme((laWidget*)PanelWidget1, &WhiteScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PanelWidget1, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget1);

    ImageWidget3 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget3, 480, 0);
    laWidget_SetSize((laWidget*)ImageWidget3, 480, 65);
    laWidget_SetBackgroundType((laWidget*)ImageWidget3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget3, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget3, &Bar);
    laWidget_AddChild((laWidget*)PanelWidget1, (laWidget*)ImageWidget3);

    ImageWidget4 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget4, 17, 23);
    laWidget_SetSize((laWidget*)ImageWidget4, 144, 39);
    laWidget_SetVisible((laWidget*)ImageWidget4, LA_FALSE);
    laWidget_SetOptimizationFlags((laWidget*)ImageWidget4, LA_WIDGET_OPT_LOCAL_REDRAW);
    laWidget_SetBackgroundType((laWidget*)ImageWidget4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget4, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget4, &MicrochipLogo);
    laWidget_AddChild((laWidget*)PanelWidget1, (laWidget*)ImageWidget4);

}

static void ScreenCreate_homeScreen(laScreen* screen)
{
    laLayer* layer0;

    laScreen_SetShowEventCallback(screen, &homeScreen_ShowEvent);

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &clockScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    ButtonWidget3 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget3, 409, 6);
    laWidget_SetSize((laWidget*)ButtonWidget3, 64, 72);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget3, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedImage(ButtonWidget3, &info);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget3, &ButtonWidget3_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget3);

    CenterClockLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CenterClockLabel, 147, 108);
    laWidget_SetSize((laWidget*)CenterClockLabel, 185, 97);
    laWidget_SetScheme((laWidget*)CenterClockLabel, &clockScheme);
    laWidget_SetBackgroundType((laWidget*)CenterClockLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CenterClockLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CenterClockLabel, laString_CreateFromID(string_centerClock));
    laLabelWidget_SetHAlignment(CenterClockLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)CenterClockLabel);

    ImageWidget6 = laImageWidget_New();
    laWidget_SetSize((laWidget*)ImageWidget6, 190, 87);
    laWidget_SetBackgroundType((laWidget*)ImageWidget6, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget6, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget6, &brickovenlogo1);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget6);

    StartButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)StartButton, 36, 41);
    laWidget_SetSize((laWidget*)StartButton, 378, 202);
    laWidget_SetBackgroundType((laWidget*)StartButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)StartButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedEventCallback(StartButton, &StartButton_PressedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)StartButton);

    LabelWidget = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget, 126, 247);
    laWidget_SetSize((laWidget*)LabelWidget, 232, 25);
    laWidget_SetScheme((laWidget*)LabelWidget, &controllerTitle);
    laWidget_SetBackgroundType((laWidget*)LabelWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget, laString_CreateFromID(string_press));
    laLabelWidget_SetHAlignment(LabelWidget, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget);

}

static void ScreenCreate_controllerScreen(laScreen* screen)
{
    laLayer* layer0;

    laScreen_SetShowEventCallback(screen, &controllerScreen_ShowEvent);

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &clockScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    LabelWidget2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget2, 215, 5);
    laWidget_SetSize((laWidget*)LabelWidget2, 53, 25);
    laWidget_SetScheme((laWidget*)LabelWidget2, &clockScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget2, laString_CreateFromID(string_upperClock));
    laLabelWidget_SetHAlignment(LabelWidget2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget2);

    ButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget, 406, 2);
    laWidget_SetSize((laWidget*)ButtonWidget, 72, 71);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedImage(ButtonWidget, &asset_1);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget, &ButtonWidget_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget);

    FishButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)FishButtonWidget, 92, 80);
    laWidget_SetSize((laWidget*)FishButtonWidget, 92, 92);
    laWidget_SetBackgroundType((laWidget*)FishButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)FishButtonWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(FishButtonWidget, &FishButtonWidget_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)FishButtonWidget);

    PizzaButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)PizzaButtonWidget, 194, 80);
    laWidget_SetSize((laWidget*)PizzaButtonWidget, 92, 92);
    laWidget_SetBackgroundType((laWidget*)PizzaButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PizzaButtonWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(PizzaButtonWidget, &PizzaButtonWidget_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)PizzaButtonWidget);

    VegeButtonWidget = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)VegeButtonWidget, 298, 80);
    laWidget_SetSize((laWidget*)VegeButtonWidget, 92, 92);
    laWidget_SetBackgroundType((laWidget*)VegeButtonWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)VegeButtonWidget, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(VegeButtonWidget, &VegeButtonWidget_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)VegeButtonWidget);

    StartStopButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)StartStopButton, 189, 213);
    laWidget_SetSize((laWidget*)StartStopButton, 90, 51);
    laWidget_SetVisible((laWidget*)StartStopButton, LA_FALSE);
    laWidget_SetScheme((laWidget*)StartStopButton, &SettingsScheme);
    laWidget_SetBackgroundType((laWidget*)StartStopButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)StartStopButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetToggleable(StartStopButton, LA_TRUE);
    laButtonWidget_SetPressedImage(StartStopButton, &cancel);
    laButtonWidget_SetReleasedImage(StartStopButton, &start_1);
    laButtonWidget_SetPressedEventCallback(StartStopButton, &StartStopButton_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(StartStopButton, &StartStopButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)StartStopButton);

    DoneButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)DoneButton, 189, 213);
    laWidget_SetSize((laWidget*)DoneButton, 90, 51);
    laWidget_SetVisible((laWidget*)DoneButton, LA_FALSE);
    laWidget_SetScheme((laWidget*)DoneButton, &SettingsScheme);
    laWidget_SetBackgroundType((laWidget*)DoneButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)DoneButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(DoneButton, &done);
    laButtonWidget_SetReleasedImage(DoneButton, &done);
    laButtonWidget_SetReleasedEventCallback(DoneButton, &DoneButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)DoneButton);

    FishGaugeWidget = laCircularGaugeWidget_New();
    laWidget_SetPosition((laWidget*)FishGaugeWidget, 4, 176);
    laWidget_SetSize((laWidget*)FishGaugeWidget, 101, 90);
    laWidget_SetVisible((laWidget*)FishGaugeWidget, LA_FALSE);
    laWidget_SetScheme((laWidget*)FishGaugeWidget, &clockScheme);
    laWidget_SetBackgroundType((laWidget*)FishGaugeWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)FishGaugeWidget, LA_WIDGET_BORDER_NONE);
    laCircularGaugeWidget_SetRadius(FishGaugeWidget, 40);
    laCircularGaugeWidget_SetStartAngle(FishGaugeWidget, 90);
    laCircularGaugeWidget_SetCenterAngle(FishGaugeWidget, 360);
    laCircularGaugeWidget_SetValue(FishGaugeWidget, 0);
    laCircularGaugeWidget_SetEndValue(FishGaugeWidget, 180);
    laCircularGaugeWidget_SetTickValue(FishGaugeWidget, 15);
    laCircularGaugeWidget_SetHandRadius(FishGaugeWidget, 30);
    laCircularGaugeWidget_AddValueArc(FishGaugeWidget, 0, 30, 35, 6, &GreenScheme);
    laCircularGaugeWidget_AddValueArc(FishGaugeWidget, 30, 60, 35, 6, &YellowScheme);
    laCircularGaugeWidget_AddValueArc(FishGaugeWidget, 60, 120, 35, 6, &RedScheme);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)FishGaugeWidget);

    PizzaGaugeWidget = laCircularGaugeWidget_New();
    laWidget_SetPosition((laWidget*)PizzaGaugeWidget, 4, 176);
    laWidget_SetSize((laWidget*)PizzaGaugeWidget, 101, 90);
    laWidget_SetVisible((laWidget*)PizzaGaugeWidget, LA_FALSE);
    laWidget_SetScheme((laWidget*)PizzaGaugeWidget, &clockScheme);
    laWidget_SetBackgroundType((laWidget*)PizzaGaugeWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PizzaGaugeWidget, LA_WIDGET_BORDER_NONE);
    laCircularGaugeWidget_SetRadius(PizzaGaugeWidget, 40);
    laCircularGaugeWidget_SetStartAngle(PizzaGaugeWidget, 90);
    laCircularGaugeWidget_SetCenterAngle(PizzaGaugeWidget, 360);
    laCircularGaugeWidget_SetValue(PizzaGaugeWidget, 0);
    laCircularGaugeWidget_SetEndValue(PizzaGaugeWidget, 180);
    laCircularGaugeWidget_SetTickValue(PizzaGaugeWidget, 15);
    laCircularGaugeWidget_SetHandRadius(PizzaGaugeWidget, 30);
    laCircularGaugeWidget_AddValueArc(PizzaGaugeWidget, 0, 20, 35, 6, &GreenScheme);
    laCircularGaugeWidget_AddValueArc(PizzaGaugeWidget, 20, 45, 35, 6, &YellowScheme);
    laCircularGaugeWidget_AddValueArc(PizzaGaugeWidget, 45, 86, 35, 6, &RedScheme);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)PizzaGaugeWidget);

    TurkeyGaugeWidget = laCircularGaugeWidget_New();
    laWidget_SetPosition((laWidget*)TurkeyGaugeWidget, 4, 176);
    laWidget_SetSize((laWidget*)TurkeyGaugeWidget, 101, 90);
    laWidget_SetVisible((laWidget*)TurkeyGaugeWidget, LA_FALSE);
    laWidget_SetScheme((laWidget*)TurkeyGaugeWidget, &clockScheme);
    laWidget_SetBackgroundType((laWidget*)TurkeyGaugeWidget, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TurkeyGaugeWidget, LA_WIDGET_BORDER_NONE);
    laCircularGaugeWidget_SetRadius(TurkeyGaugeWidget, 40);
    laCircularGaugeWidget_SetStartAngle(TurkeyGaugeWidget, 90);
    laCircularGaugeWidget_SetCenterAngle(TurkeyGaugeWidget, 360);
    laCircularGaugeWidget_SetValue(TurkeyGaugeWidget, 0);
    laCircularGaugeWidget_SetEndValue(TurkeyGaugeWidget, 180);
    laCircularGaugeWidget_SetTickValue(TurkeyGaugeWidget, 15);
    laCircularGaugeWidget_SetHandRadius(TurkeyGaugeWidget, 30);
    laCircularGaugeWidget_AddValueArc(TurkeyGaugeWidget, 0, 25, 35, 6, &GreenScheme);
    laCircularGaugeWidget_AddValueArc(TurkeyGaugeWidget, 25, 55, 35, 6, &YellowScheme);
    laCircularGaugeWidget_AddValueArc(TurkeyGaugeWidget, 55, 160, 35, 6, &RedScheme);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)TurkeyGaugeWidget);

}

static void ScreenCreate_infoScreen(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &clockScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    LabelWidget7 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget7, 19, 9);
    laWidget_SetSize((laWidget*)LabelWidget7, 57, 35);
    laWidget_SetScheme((laWidget*)LabelWidget7, &clockScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget7, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget7, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget7, laString_CreateFromID(string_upperClock));
    laLabelWidget_SetHAlignment(LabelWidget7, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget7);

    ButtonWidget22 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget22, 418, 3);
    laWidget_SetSize((laWidget*)ButtonWidget22, 56, 61);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget22, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget22, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedImage(ButtonWidget22, &asset_1);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget22, &ButtonWidget22_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget22);

    LabelWidget9 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget9, 71, 11);
    laWidget_SetSize((laWidget*)LabelWidget9, 203, 30);
    laWidget_SetScheme((laWidget*)LabelWidget9, &helpScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget9, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget9, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget9, laString_CreateFromID(string_timeHelp));
    laLabelWidget_SetHAlignment(LabelWidget9, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget9);

    LabelWidget13 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget13, 346, 56);
    laWidget_SetSize((laWidget*)LabelWidget13, 125, 30);
    laWidget_SetScheme((laWidget*)LabelWidget13, &helpScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget13, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget13, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget13, laString_CreateFromID(string_returnHome));
    laLabelWidget_SetHAlignment(LabelWidget13, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget13);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 17, 127);
    laWidget_SetSize((laWidget*)ImageWidget, 80, 73);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &menufish);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

    Vegetables = laImageWidget_New();
    laWidget_SetPosition((laWidget*)Vegetables, 106, 126);
    laWidget_SetSize((laWidget*)Vegetables, 80, 74);
    laWidget_SetBackgroundType((laWidget*)Vegetables, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)Vegetables, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(Vegetables, &menuTurkey);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Vegetables);

    Pizza = laImageWidget_New();
    laWidget_SetPosition((laWidget*)Pizza, 196, 125);
    laWidget_SetSize((laWidget*)Pizza, 83, 74);
    laWidget_SetBackgroundType((laWidget*)Pizza, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)Pizza, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(Pizza, &menuPizza);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Pizza);

    ImageWidget5 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget5, 3, 218);
    laWidget_SetSize((laWidget*)ImageWidget5, 88, 44);
    laWidget_SetBackgroundType((laWidget*)ImageWidget5, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget5, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget5, &start_1);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget5);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 84, 216);
    laWidget_SetSize((laWidget*)ImageWidget, 87, 39);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &cancel);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

    ImageWidget7 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget7, 170, 224);
    laWidget_SetSize((laWidget*)ImageWidget7, 82, 33);
    laWidget_SetScheme((laWidget*)ImageWidget7, &SettingsScheme);
    laWidget_SetBackgroundType((laWidget*)ImageWidget7, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget7, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget7, &done);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget7);

    ImageWidget11 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget11, 21, 48);
    laWidget_SetSize((laWidget*)ImageWidget11, 42, 54);
    laWidget_SetBackgroundType((laWidget*)ImageWidget11, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget11, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget11, &info);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget11);

    LabelWidget12 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget12, 69, 86);
    laWidget_SetSize((laWidget*)LabelWidget12, 134, 25);
    laWidget_SetScheme((laWidget*)LabelWidget12, &helpScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget12, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget12, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget12, laString_CreateFromID(string_information));
    laLabelWidget_SetHAlignment(LabelWidget12, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget12);

    LabelWidget8 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget8, 297, 181);
    laWidget_SetSize((laWidget*)LabelWidget8, 144, 30);
    laWidget_SetScheme((laWidget*)LabelWidget8, &helpScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget8, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget8, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget8, laString_CreateFromID(string_options));
    laLabelWidget_SetHAlignment(LabelWidget8, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget8);

    LabelWidget10 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget10, 315, 232);
    laWidget_SetSize((laWidget*)LabelWidget10, 149, 30);
    laWidget_SetScheme((laWidget*)LabelWidget10, &helpScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget10, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget10, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget10, laString_CreateFromID(string_controls));
    laLabelWidget_SetHAlignment(LabelWidget10, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget10);

    CircularGaugeWidget3 = laCircularGaugeWidget_New();
    laWidget_SetPosition((laWidget*)CircularGaugeWidget3, 288, 93);
    laWidget_SetSize((laWidget*)CircularGaugeWidget3, 71, 75);
    laWidget_SetScheme((laWidget*)CircularGaugeWidget3, &clockScheme);
    laWidget_SetBackgroundType((laWidget*)CircularGaugeWidget3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CircularGaugeWidget3, LA_WIDGET_BORDER_NONE);
    laCircularGaugeWidget_SetRadius(CircularGaugeWidget3, 30);
    laCircularGaugeWidget_SetStartAngle(CircularGaugeWidget3, 90);
    laCircularGaugeWidget_SetCenterAngle(CircularGaugeWidget3, 360);
    laCircularGaugeWidget_SetValue(CircularGaugeWidget3, 120);
    laCircularGaugeWidget_SetEndValue(CircularGaugeWidget3, 180);
    laCircularGaugeWidget_SetTickValue(CircularGaugeWidget3, 15);
    laCircularGaugeWidget_SetHandRadius(CircularGaugeWidget3, 20);
    laCircularGaugeWidget_AddValueArc(CircularGaugeWidget3, 0, 15, 23, 6, &GreenScheme);
    laCircularGaugeWidget_AddValueArc(CircularGaugeWidget3, 15, 60, 23, 6, &YellowScheme);
    laCircularGaugeWidget_AddValueArc(CircularGaugeWidget3, 60, 123, 23, 6, &RedScheme);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)CircularGaugeWidget3);

    LabelWidget4 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget4, 373, 139);
    laWidget_SetSize((laWidget*)LabelWidget4, 100, 25);
    laWidget_SetScheme((laWidget*)LabelWidget4, &helpScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget4, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget4, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget4, laString_CreateFromID(string_cookTimer));
    laLabelWidget_SetHAlignment(LabelWidget4, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget4);

}

static void ScreenCreate_AssetLayout(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &clockScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget = laImageWidget_New();
    laWidget_SetSize((laWidget*)ImageWidget, 190, 87);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &brickovenlogo1);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 201, 0);
    laWidget_SetSize((laWidget*)ImageWidget, 92, 92);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &menufish);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 294, 0);
    laWidget_SetSize((laWidget*)ImageWidget, 92, 92);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &menuPizza);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 386, 0);
    laWidget_SetSize((laWidget*)ImageWidget, 92, 92);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &menuTurkey);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 0, 93);
    laWidget_SetSize((laWidget*)ImageWidget, 88, 8);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &orangeBar);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 90, 93);
    laWidget_SetSize((laWidget*)ImageWidget, 60, 100);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &smoke1);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 151, 93);
    laWidget_SetSize((laWidget*)ImageWidget, 60, 100);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &smoke2);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget);

    ImageWidget8 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget8, 212, 93);
    laWidget_SetSize((laWidget*)ImageWidget8, 60, 100);
    laWidget_SetBackgroundType((laWidget*)ImageWidget8, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget8, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget8, &smoke3);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget8);

    ImageWidget9 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget9, 273, 93);
    laWidget_SetSize((laWidget*)ImageWidget9, 60, 100);
    laWidget_SetBackgroundType((laWidget*)ImageWidget9, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget9, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget9, &smoke4);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget9);

    ImageWidget10 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget10, 334, 93);
    laWidget_SetSize((laWidget*)ImageWidget10, 60, 100);
    laWidget_SetBackgroundType((laWidget*)ImageWidget10, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget10, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget10, &smoke5);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget10);

    ImageWidget12 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget12, 400, 93);
    laWidget_SetSize((laWidget*)ImageWidget12, 60, 100);
    laWidget_SetBackgroundType((laWidget*)ImageWidget12, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget12, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget12, &smoke6);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget12);

    ImageWidget13 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget13, 0, 200);
    laWidget_SetSize((laWidget*)ImageWidget13, 64, 64);
    laWidget_SetBackgroundType((laWidget*)ImageWidget13, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget13, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget13, &clock);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget13);

}



