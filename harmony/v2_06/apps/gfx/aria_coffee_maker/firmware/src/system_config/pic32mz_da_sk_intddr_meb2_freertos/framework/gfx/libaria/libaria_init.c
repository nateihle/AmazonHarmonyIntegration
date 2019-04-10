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
laScheme trayScheme;
laScheme ButtonWhiteScheme;
laScheme ClearScheme;
laScheme WhiteScheme;
laImageWidget* ImageWidget1;
laImageWidget* ImageWidget3;
laImageWidget* ImageWidget2;
laImageWidget* ImageWidget4;
laImageWidget* BackgroundImage;
laLabelWidget* RoastLabel;
laWidget* DragPanelRight;
laListWheelWidget* SizeSelect;
laWidget* PanelWidget4;
laButtonWidget* BrewButton;
laWidget* PanelWidget5;
laWidget* PanelWidget6;
laWidget* PanelWidget7;
laButtonWidget* RightTrayLid;
laImageWidget* ImageWidget6;
laWidget* DragPanel;
laWidget* PanelWidget2;
laButtonWidget* ChangeLanguage;
laButtonWidget* CoffeeButton;
laButtonWidget* CoffeeBeanButton;
laButtonWidget* TeaButton;
laButtonWidget* InfoPageButton;
laLabelWidget* AppTitleLabel;
laButtonWidget* GPUButton;
laLabelWidget* GPUButtonLabel;
laButtonWidget* LeftTrayLid;
laImageWidget* ImageWidget5;
laImageWidget* InfoPageHarmonyLogo;
laImageWidget* ImageWidget7;
laWidget* InfoTextDragPanel;
laImageWidget* ImageWidget;
laWidget* PanelWidget1;
laLabelWidget* TextTitle;
laButtonWidget* ReturnToMainButton;
laButtonWidget* ButtonWidget1;


static void ScreenCreate_SplashScreen(laScreen* screen);
static void ScreenCreate_MainScreen(laScreen* screen);
static void ScreenCreate_InfoScreen(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&defaultScheme, GFX_COLOR_MODE_RGBA_8888);
    defaultScheme.base = 0xFF;
    defaultScheme.highlight = 0xFF;
    defaultScheme.highlightLight = 0xFF;
    defaultScheme.shadow = 0xFF;
    defaultScheme.shadowDark = 0xFF;
    defaultScheme.foreground = 0xFF;
    defaultScheme.foregroundInactive = 0xFF;
    defaultScheme.foregroundDisabled = 0xFF;
    defaultScheme.background = 0xFF;
    defaultScheme.backgroundInactive = 0xFF;
    defaultScheme.backgroundDisabled = 0xFF;
    defaultScheme.text = 0xFF;
    defaultScheme.textHighlight = 0xFF;
    defaultScheme.textHighlightText = 0xFF;
    defaultScheme.textInactive = 0xFF;
    defaultScheme.textDisabled = 0xFF;

    laScheme_Initialize(&trayScheme, GFX_COLOR_MODE_RGBA_8888);
    trayScheme.base = 0xE0E0E0FF;
    trayScheme.highlight = 0xC8D0D4FF;
    trayScheme.highlightLight = 0xFFFFFFFF;
    trayScheme.shadow = 0x808080FF;
    trayScheme.shadowDark = 0x404040FF;
    trayScheme.foreground = 0xFF;
    trayScheme.foregroundInactive = 0xD6E3E7FF;
    trayScheme.foregroundDisabled = 0x808080FF;
    trayScheme.background = 0xFFFFFFFF;
    trayScheme.backgroundInactive = 0xD6E3E7FF;
    trayScheme.backgroundDisabled = 0xC8D0D4FF;
    trayScheme.text = 0xFF;
    trayScheme.textHighlight = 0xFFFF;
    trayScheme.textHighlightText = 0xFFFFFFFF;
    trayScheme.textInactive = 0xD6E3E7FF;
    trayScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&ButtonWhiteScheme, GFX_COLOR_MODE_RGBA_8888);
    ButtonWhiteScheme.base = 0xFFFFFFFF;
    ButtonWhiteScheme.highlight = 0xC8D0D4FF;
    ButtonWhiteScheme.highlightLight = 0xFFFFFFFF;
    ButtonWhiteScheme.shadow = 0x808080FF;
    ButtonWhiteScheme.shadowDark = 0x404040FF;
    ButtonWhiteScheme.foreground = 0xFF;
    ButtonWhiteScheme.foregroundInactive = 0xD6E3E7FF;
    ButtonWhiteScheme.foregroundDisabled = 0x808080FF;
    ButtonWhiteScheme.background = 0xFFFFFFFF;
    ButtonWhiteScheme.backgroundInactive = 0xD6E3E7FF;
    ButtonWhiteScheme.backgroundDisabled = 0xC8D0D4FF;
    ButtonWhiteScheme.text = 0xFF;
    ButtonWhiteScheme.textHighlight = 0xFFFF;
    ButtonWhiteScheme.textHighlightText = 0xFFFFFFFF;
    ButtonWhiteScheme.textInactive = 0xD6E3E7FF;
    ButtonWhiteScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&ClearScheme, GFX_COLOR_MODE_RGBA_8888);
    ClearScheme.base = 0x0;
    ClearScheme.highlight = 0xC8D0D4FF;
    ClearScheme.highlightLight = 0xFFFFFFFF;
    ClearScheme.shadow = 0x808080FF;
    ClearScheme.shadowDark = 0x404040FF;
    ClearScheme.foreground = 0xFF;
    ClearScheme.foregroundInactive = 0xD6E3E7FF;
    ClearScheme.foregroundDisabled = 0x808080FF;
    ClearScheme.background = 0x0;
    ClearScheme.backgroundInactive = 0xD6E3E7FF;
    ClearScheme.backgroundDisabled = 0xC8D0D4FF;
    ClearScheme.text = 0xFF;
    ClearScheme.textHighlight = 0xFFFF;
    ClearScheme.textHighlightText = 0xFFFFFFFF;
    ClearScheme.textInactive = 0xD6E3E7FF;
    ClearScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&WhiteScheme, GFX_COLOR_MODE_RGBA_8888);
    WhiteScheme.base = 0xFFFFFFFF;
    WhiteScheme.highlight = 0xC8D0D4FF;
    WhiteScheme.highlightLight = 0xFFFFFFFF;
    WhiteScheme.shadow = 0x808080FF;
    WhiteScheme.shadowDark = 0x404040FF;
    WhiteScheme.foreground = 0xFF;
    WhiteScheme.foregroundInactive = 0xD6E3E7FF;
    WhiteScheme.foregroundDisabled = 0x808080FF;
    WhiteScheme.background = 0xFFFFFFFF;
    WhiteScheme.backgroundInactive = 0xD6E3E7FF;
    WhiteScheme.backgroundDisabled = 0xC8D0D4FF;
    WhiteScheme.text = 0xFF;
    WhiteScheme.textHighlight = 0xFFFF;
    WhiteScheme.textHighlightText = 0xFFFFFFFF;
    WhiteScheme.textInactive = 0xD6E3E7FF;
    WhiteScheme.textDisabled = 0x8C9294FF;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_SplashScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_MainScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_InfoScreen);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen_RTOS(0);

	return 0;
}

static void ScreenCreate_SplashScreen(laScreen* screen)
{
    laLayer* layer0;
    laLayer* layer1;
    laLayer* layer2;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 2);
    laLayer_SetAlphaEnable(layer0, LA_TRUE);
    laLayer_SetAlphaAmount(layer0, 0xFF);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 120, 89);
    laWidget_SetSize((laWidget*)ImageWidget1, 240, 62);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &PIC32Logo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer1, &ClearScheme);
    laLayer_SetBufferCount(layer1, 2);
    laLayer_SetAlphaEnable(layer1, LA_TRUE);
    laLayer_SetAlphaAmount(layer1, 0xFF);

    laScreen_SetLayer(screen, 1, layer1);

    ImageWidget3 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget3, 480, 207);
    laWidget_SetSize((laWidget*)ImageWidget3, 480, 65);
    laWidget_SetBackgroundType((laWidget*)ImageWidget3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget3, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget3, &Bar);
    laWidget_AddChild((laWidget*)layer1, (laWidget*)ImageWidget3);

    layer2 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer2, 0, 0);
    laWidget_SetSize((laWidget*)layer2, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer2, &ClearScheme);
    laLayer_SetBufferCount(layer2, 2);
    laLayer_SetAlphaEnable(layer2, LA_TRUE);
    laLayer_SetAlphaAmount(layer2, 0x0);

    laScreen_SetLayer(screen, 2, layer2);

    ImageWidget2 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget2, 144, 42);
    laWidget_SetSize((laWidget*)ImageWidget2, 197, 139);
    laWidget_SetBackgroundType((laWidget*)ImageWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget2, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget2, &HarmonyLogo);
    laWidget_AddChild((laWidget*)layer2, (laWidget*)ImageWidget2);

    ImageWidget4 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget4, 17, 230);
    laWidget_SetSize((laWidget*)ImageWidget4, 144, 39);
    laWidget_SetBackgroundType((laWidget*)ImageWidget4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget4, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget4, &MicrochipLogo);
    laWidget_AddChild((laWidget*)layer2, (laWidget*)ImageWidget4);

}

static void ScreenCreate_MainScreen(laScreen* screen)
{
    laLayer* layer0;
    laLayer* layer1;
    laLayer* layer2;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 2);

    laScreen_SetLayer(screen, 0, layer0);

    BackgroundImage = laImageWidget_New();
    laWidget_SetSize((laWidget*)BackgroundImage, 480, 272);
    laWidget_SetBackgroundType((laWidget*)BackgroundImage, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BackgroundImage, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(BackgroundImage, &coffee_beans_1);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)BackgroundImage);

    RoastLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)RoastLabel, 220, 16);
    laWidget_SetSize((laWidget*)RoastLabel, 115, 45);
    laWidget_SetScheme((laWidget*)RoastLabel, &WhiteScheme);
    laWidget_SetBackgroundType((laWidget*)RoastLabel, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)RoastLabel, LA_WIDGET_BORDER_BEVEL);
    laLabelWidget_SetText(RoastLabel, laString_CreateFromID(string_Medium_Roast));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)RoastLabel);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer1, &ClearScheme);
    laLayer_SetBufferCount(layer1, 2);
    laLayer_SetAlphaEnable(layer1, LA_TRUE);
    laLayer_SetAlphaAmount(layer1, 0xE1);

    laScreen_SetLayer(screen, 1, layer1);

    DragPanelRight = laWidget_New();
    laWidget_SetPosition((laWidget*)DragPanelRight, 409, 0);
    laWidget_SetSize((laWidget*)DragPanelRight, 210, 272);
    laWidget_SetBackgroundType((laWidget*)DragPanelRight, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)DragPanelRight, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer1, DragPanelRight);

    SizeSelect = laListWheelWidget_New();
    laWidget_SetPosition((laWidget*)SizeSelect, 90, 10);
    laWidget_SetSize((laWidget*)SizeSelect, 100, 200);
    laWidget_SetBackgroundType((laWidget*)SizeSelect, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)SizeSelect, LA_WIDGET_BORDER_LINE);
    laListWheelWidget_SetSelectedItem(SizeSelect, 0);
    laListWheelWidget_SetIconMargin(SizeSelect, 0);
    laListWheelWidget_SetVisibleItemCount(SizeSelect, 3);
    laListWheelWidget_SetShowIndicators(SizeSelect, LA_FALSE);
    laListWheelWidget_SetIndicatorArea(SizeSelect, 36);
    laListWheelWidget_SetFlickInitSpeed(SizeSelect, 5);
    laListWheelWidget_SetMaxMomentum(SizeSelect, 30000);
    laListWheelWidget_SetRotationUpdateRate(SizeSelect, 50);
    laListWheelWidget_AppendItem(SizeSelect);
    laListWheelWidget_SetItemIcon(SizeSelect, 0, &Coffee_Small_Hot);
    laListWheelWidget_AppendItem(SizeSelect);
    laListWheelWidget_SetItemIcon(SizeSelect, 1, &Coffee_Medium_Hot);
    laListWheelWidget_AppendItem(SizeSelect);
    laListWheelWidget_SetItemIcon(SizeSelect, 2, &Coffee_Grande_Hot);
    laListWheelWidget_AppendItem(SizeSelect);
    laListWheelWidget_SetItemIcon(SizeSelect, 3, &Coffee_Venti_Hot);
    laListWheelWidget_SetSelectedItemChangedEventCallback(SizeSelect, &SizeSelect_SelectedItemChangedEvent);
    laWidget_AddChild((laWidget*)DragPanelRight, (laWidget*)SizeSelect);

    PanelWidget4 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget4, 70, 210);
    laWidget_SetSize((laWidget*)PanelWidget4, 140, 62);
    laWidget_SetScheme((laWidget*)PanelWidget4, &trayScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget4, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget4, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)DragPanelRight, PanelWidget4);

    BrewButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)BrewButton, 16, 10);
    laWidget_SetSize((laWidget*)BrewButton, 110, 40);
    laWidget_SetBackgroundType((laWidget*)BrewButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)BrewButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetText(BrewButton, laString_CreateFromID(string_BrewEightOunce));
    laButtonWidget_SetPressedImage(BrewButton, &OvalButton);
    laButtonWidget_SetReleasedImage(BrewButton, &OvalButtonPressed);
    laButtonWidget_SetImagePosition(BrewButton, LA_RELATIVE_POSITION_BEHIND);
    laButtonWidget_SetPressedEventCallback(BrewButton, &BrewButton_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(BrewButton, &BrewButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)PanelWidget4, (laWidget*)BrewButton);

    PanelWidget5 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget5, 70, 0);
    laWidget_SetSize((laWidget*)PanelWidget5, 20, 210);
    laWidget_SetScheme((laWidget*)PanelWidget5, &trayScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget5, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget5, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)DragPanelRight, PanelWidget5);

    PanelWidget6 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget6, 190, 0);
    laWidget_SetSize((laWidget*)PanelWidget6, 20, 210);
    laWidget_SetScheme((laWidget*)PanelWidget6, &trayScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget6, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget6, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)DragPanelRight, PanelWidget6);

    PanelWidget7 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget7, 90, 0);
    laWidget_SetSize((laWidget*)PanelWidget7, 100, 10);
    laWidget_SetScheme((laWidget*)PanelWidget7, &trayScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget7, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget7, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)DragPanelRight, PanelWidget7);

    RightTrayLid = laButtonWidget_New();
    laWidget_SetSize((laWidget*)RightTrayLid, 70, 272);
    laWidget_SetScheme((laWidget*)RightTrayLid, &trayScheme);
    laWidget_SetBackgroundType((laWidget*)RightTrayLid, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)RightTrayLid, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)RightTrayLid, 0, 0, 0, 0);
    laButtonWidget_SetHAlignment(RightTrayLid, LA_HALIGN_RIGHT);
    laButtonWidget_SetPressedImage(RightTrayLid, &tray_right_pressed);
    laButtonWidget_SetReleasedImage(RightTrayLid, &tray_right);
    laButtonWidget_SetPressedOffset(RightTrayLid, 0);
    laWidget_AddChild((laWidget*)DragPanelRight, (laWidget*)RightTrayLid);

    ImageWidget6 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget6, 40, 120);
    laWidget_SetSize((laWidget*)ImageWidget6, 28, 34);
    laWidget_SetBackgroundType((laWidget*)ImageWidget6, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget6, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget6, &left_right_touch_icon_small);
    laWidget_AddChild((laWidget*)RightTrayLid, (laWidget*)ImageWidget6);

    layer2 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer2, 0, 0);
    laWidget_SetSize((laWidget*)layer2, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer2, &ClearScheme);
    laLayer_SetBufferCount(layer2, 2);
    laLayer_SetAlphaEnable(layer2, LA_TRUE);
    laLayer_SetAlphaAmount(layer2, 0xE1);

    laScreen_SetLayer(screen, 2, layer2);

    DragPanel = laWidget_New();
    laWidget_SetPosition((laWidget*)DragPanel, -98, 0);
    laWidget_SetSize((laWidget*)DragPanel, 280, 272);
    laWidget_SetBackgroundType((laWidget*)DragPanel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)DragPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer2, DragPanel);

    PanelWidget2 = laWidget_New();
    laWidget_SetSize((laWidget*)PanelWidget2, 210, 272);
    laWidget_SetScheme((laWidget*)PanelWidget2, &trayScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget2, LA_WIDGET_BORDER_LINE);
    laWidget_AddChild((laWidget*)DragPanel, PanelWidget2);

    ChangeLanguage = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ChangeLanguage, 31, 55);
    laWidget_SetSize((laWidget*)ChangeLanguage, 65, 65);
    laWidget_SetBackgroundType((laWidget*)ChangeLanguage, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ChangeLanguage, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ChangeLanguage, &USA);
    laButtonWidget_SetReleasedImage(ChangeLanguage, &USA);
    laButtonWidget_SetReleasedEventCallback(ChangeLanguage, &ChangeLanguage_ReleasedEvent);

    laWidget_AddChild((laWidget*)PanelWidget2, (laWidget*)ChangeLanguage);

    CoffeeButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)CoffeeButton, 121, 195);
    laWidget_SetSize((laWidget*)CoffeeButton, 65, 65);
    laWidget_SetBackgroundType((laWidget*)CoffeeButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CoffeeButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(CoffeeButton, &temp_control_hot);
    laButtonWidget_SetReleasedImage(CoffeeButton, &temp_control_hot);
    laButtonWidget_SetReleasedEventCallback(CoffeeButton, &CoffeeButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)PanelWidget2, (laWidget*)CoffeeButton);

    CoffeeBeanButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)CoffeeBeanButton, 121, 55);
    laWidget_SetSize((laWidget*)CoffeeBeanButton, 65, 65);
    laWidget_SetBackgroundType((laWidget*)CoffeeBeanButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)CoffeeBeanButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(CoffeeBeanButton, &coffee_bean_button);
    laButtonWidget_SetReleasedImage(CoffeeBeanButton, &coffee_bean_button);
    laButtonWidget_SetReleasedEventCallback(CoffeeBeanButton, &CoffeeBeanButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)PanelWidget2, (laWidget*)CoffeeBeanButton);

    TeaButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)TeaButton, 121, 125);
    laWidget_SetSize((laWidget*)TeaButton, 65, 65);
    laWidget_SetBackgroundType((laWidget*)TeaButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TeaButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(TeaButton, &tea_mode_icon_hot);
    laButtonWidget_SetReleasedImage(TeaButton, &tea_mode_icon_hot);
    laButtonWidget_SetReleasedEventCallback(TeaButton, &TeaButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)PanelWidget2, (laWidget*)TeaButton);

    InfoPageButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)InfoPageButton, 31, 195);
    laWidget_SetSize((laWidget*)InfoPageButton, 65, 65);
    laWidget_SetBackgroundType((laWidget*)InfoPageButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)InfoPageButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(InfoPageButton, &question_button);
    laButtonWidget_SetReleasedImage(InfoPageButton, &question_button);
    laButtonWidget_SetReleasedEventCallback(InfoPageButton, &InfoPageButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)PanelWidget2, (laWidget*)InfoPageButton);

    AppTitleLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)AppTitleLabel, 10, 10);
    laWidget_SetSize((laWidget*)AppTitleLabel, 195, 38);
    laWidget_SetScheme((laWidget*)AppTitleLabel, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)AppTitleLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)AppTitleLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(AppTitleLabel, laString_CreateFromID(string_AppTitle));
    laWidget_AddChild((laWidget*)PanelWidget2, (laWidget*)AppTitleLabel);

    GPUButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)GPUButton, 31, 125);
    laWidget_SetSize((laWidget*)GPUButton, 65, 65);
    laWidget_SetBackgroundType((laWidget*)GPUButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GPUButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(GPUButton, &PlainButton);
    laButtonWidget_SetReleasedImage(GPUButton, &PlainButton);
    laButtonWidget_SetReleasedEventCallback(GPUButton, &GPUButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)PanelWidget2, (laWidget*)GPUButton);

    GPUButtonLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GPUButtonLabel, 2, 20);
    laWidget_SetSize((laWidget*)GPUButtonLabel, 60, 25);
    laWidget_SetBackgroundType((laWidget*)GPUButtonLabel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)GPUButtonLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GPUButtonLabel, laString_CreateFromID(string_GPU_On));
    laWidget_AddChild((laWidget*)GPUButton, (laWidget*)GPUButtonLabel);

    LeftTrayLid = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)LeftTrayLid, 210, 0);
    laWidget_SetSize((laWidget*)LeftTrayLid, 70, 272);
    laWidget_SetScheme((laWidget*)LeftTrayLid, &trayScheme);
    laWidget_SetBackgroundType((laWidget*)LeftTrayLid, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LeftTrayLid, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)LeftTrayLid, 0, 0, 0, 0);
    laButtonWidget_SetHAlignment(LeftTrayLid, LA_HALIGN_LEFT);
    laButtonWidget_SetPressedImage(LeftTrayLid, &tray_left_pressed);
    laButtonWidget_SetReleasedImage(LeftTrayLid, &tray_left);
    laButtonWidget_SetPressedOffset(LeftTrayLid, 0);
    laWidget_AddChild((laWidget*)DragPanel, (laWidget*)LeftTrayLid);

    ImageWidget5 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget5, 0, 120);
    laWidget_SetSize((laWidget*)ImageWidget5, 28, 34);
    laWidget_SetBackgroundType((laWidget*)ImageWidget5, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget5, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget5, &left_right_touch_icon_small);
    laWidget_AddChild((laWidget*)LeftTrayLid, (laWidget*)ImageWidget5);

}

static void ScreenCreate_InfoScreen(laScreen* screen)
{
    laLayer* layer0;
    laLayer* layer1;
    laLayer* layer2;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 2);

    laScreen_SetLayer(screen, 0, layer0);

    InfoPageHarmonyLogo = laImageWidget_New();
    laWidget_SetPosition((laWidget*)InfoPageHarmonyLogo, 0, -1);
    laWidget_SetSize((laWidget*)InfoPageHarmonyLogo, 480, 272);
    laWidget_SetBackgroundType((laWidget*)InfoPageHarmonyLogo, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)InfoPageHarmonyLogo, LA_WIDGET_BORDER_NONE);
    laWidget_SetAlphaEnable((laWidget*)InfoPageHarmonyLogo, GFX_TRUE);
    laWidget_SetAlphaAmount((laWidget*)InfoPageHarmonyLogo, 128);
    laImageWidget_SetImage(InfoPageHarmonyLogo, &HarmonyLogo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)InfoPageHarmonyLogo);

    ImageWidget7 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget7, 410, 110);
    laWidget_SetSize((laWidget*)ImageWidget7, 60, 67);
    laWidget_SetBackgroundType((laWidget*)ImageWidget7, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget7, LA_WIDGET_BORDER_NONE);
    laWidget_SetAlphaEnable((laWidget*)ImageWidget7, GFX_TRUE);
    laWidget_SetAlphaAmount((laWidget*)ImageWidget7, 128);
    laImageWidget_SetImage(ImageWidget7, &vertical_touch);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget7);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer1, &ClearScheme);
    laLayer_SetBufferCount(layer1, 2);

    laScreen_SetLayer(screen, 1, layer1);

    InfoTextDragPanel = laWidget_New();
    laWidget_SetSize((laWidget*)InfoTextDragPanel, 480, 1000);
    laWidget_SetScheme((laWidget*)InfoTextDragPanel, &ClearScheme);
    laWidget_SetBackgroundType((laWidget*)InfoTextDragPanel, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)InfoTextDragPanel, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer1, InfoTextDragPanel);

    ImageWidget = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget, 0, 50);
    laWidget_SetSize((laWidget*)ImageWidget, 480, 400);
    laWidget_SetScheme((laWidget*)ImageWidget, &ClearScheme);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &info_text_en);
    laImageWidget_SetHAlignment(ImageWidget, LA_HALIGN_LEFT);
    laImageWidget_SetVAlignment(ImageWidget, LA_VALIGN_TOP);
    laWidget_AddChild((laWidget*)InfoTextDragPanel, (laWidget*)ImageWidget);

    PanelWidget1 = laWidget_New();
    laWidget_SetSize((laWidget*)PanelWidget1, 480, 50);
    laWidget_SetScheme((laWidget*)PanelWidget1, &WhiteScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget1, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer1, PanelWidget1);

    TextTitle = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TextTitle, 10, 0);
    laWidget_SetSize((laWidget*)TextTitle, 470, 50);
    laWidget_SetBackgroundType((laWidget*)TextTitle, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)TextTitle, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TextTitle, laString_CreateFromID(string_InfoText_Title));
    laLabelWidget_SetHAlignment(TextTitle, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget1, (laWidget*)TextTitle);

    layer2 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer2, 0, 0);
    laWidget_SetSize((laWidget*)layer2, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer2, &ClearScheme);
    laLayer_SetBufferCount(layer2, 2);

    laScreen_SetLayer(screen, 2, layer2);

    ReturnToMainButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ReturnToMainButton, 400, 191);
    laWidget_SetSize((laWidget*)ReturnToMainButton, 80, 80);
    laWidget_SetBackgroundType((laWidget*)ReturnToMainButton, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ReturnToMainButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ReturnToMainButton, &HomeButton);
    laButtonWidget_SetReleasedImage(ReturnToMainButton, &HomeButton);
    laButtonWidget_SetReleasedEventCallback(ReturnToMainButton, &ReturnToMainButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer2, (laWidget*)ReturnToMainButton);

    ButtonWidget1 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget1, 400, 10);
    laWidget_SetSize((laWidget*)ButtonWidget1, 80, 80);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget1, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonWidget1, &microchip_button);
    laButtonWidget_SetReleasedImage(ButtonWidget1, &microchip_button);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget1, &ButtonWidget1_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer2, (laWidget*)ButtonWidget1);

}



