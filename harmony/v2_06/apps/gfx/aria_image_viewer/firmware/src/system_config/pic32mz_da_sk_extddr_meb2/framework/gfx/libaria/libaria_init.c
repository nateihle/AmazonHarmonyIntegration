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
laScheme ClearScreen;
laScheme NewScheme;
laImageWidget* ImageWidget1;
laLabelWidget* LabelWidget3;
laImageWidget* ImageWidget3;
laImageWidget* ImageWidget2;
laImageWidget* ImageWidget4;
laImagePlusWidget* ImagePlusWidget2;
laLabelWidget* Hint;
laButtonWidget* ButtonWidget1;
laWidget* HelpArea;
laLabelWidget* LabelWidget6;
laLabelWidget* LabelWidget5;
laImageWidget* ImageWidget;


static void ScreenCreate_SplashScreen(laScreen* screen);
static void ScreenCreate_ImageScreen(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&defaultScheme, GFX_COLOR_MODE_RGBA_8888);
    defaultScheme.base = 0xC8D0D4FF;
    defaultScheme.highlight = 0xC8D0D4FF;
    defaultScheme.highlightLight = 0xFFFFFFFF;
    defaultScheme.shadow = 0x808080FF;
    defaultScheme.shadowDark = 0x404040FF;
    defaultScheme.foreground = 0xFF;
    defaultScheme.foregroundInactive = 0xD6E3E7FF;
    defaultScheme.foregroundDisabled = 0x808080FF;
    defaultScheme.background = 0xFFFFFFFF;
    defaultScheme.backgroundInactive = 0xD6E3E7FF;
    defaultScheme.backgroundDisabled = 0xC8D0D4FF;
    defaultScheme.text = 0xFFFFFFFF;
    defaultScheme.textHighlight = 0xFFFF;
    defaultScheme.textHighlightText = 0xFFFFFFFF;
    defaultScheme.textInactive = 0xD6E3E7FF;
    defaultScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&ClearScreen, GFX_COLOR_MODE_RGBA_8888);
    ClearScreen.base = 0xC8D0D4FF;
    ClearScreen.highlight = 0xC8D0D4FF;
    ClearScreen.highlightLight = 0xFFFFFFFF;
    ClearScreen.shadow = 0x808080FF;
    ClearScreen.shadowDark = 0x404040FF;
    ClearScreen.foreground = 0xFF;
    ClearScreen.foregroundInactive = 0xD6E3E7FF;
    ClearScreen.foregroundDisabled = 0x808080FF;
    ClearScreen.background = 0xFFFFFFFF;
    ClearScreen.backgroundInactive = 0xD6E3E7FF;
    ClearScreen.backgroundDisabled = 0xC8D0D4FF;
    ClearScreen.text = 0xFF00FF;
    ClearScreen.textHighlight = 0xFFFF;
    ClearScreen.textHighlightText = 0xFFFFFFFF;
    ClearScreen.textInactive = 0xD6E3E7FF;
    ClearScreen.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&NewScheme, GFX_COLOR_MODE_RGBA_8888);
    NewScheme.base = 0xFF8000FF;
    NewScheme.highlight = 0xC8D0D4FF;
    NewScheme.highlightLight = 0xFFFFFFFF;
    NewScheme.shadow = 0x808080FF;
    NewScheme.shadowDark = 0x404040FF;
    NewScheme.foreground = 0xFF;
    NewScheme.foregroundInactive = 0xD6E3E7FF;
    NewScheme.foregroundDisabled = 0x808080FF;
    NewScheme.background = 0xFFFFFFFF;
    NewScheme.backgroundInactive = 0xD6E3E7FF;
    NewScheme.backgroundDisabled = 0xC8D0D4FF;
    NewScheme.text = 0xFF;
    NewScheme.textHighlight = 0xFFFF;
    NewScheme.textHighlightText = 0xFFFFFFFF;
    NewScheme.textInactive = 0xD6E3E7FF;
    NewScheme.textDisabled = 0x8C9294FF;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCU);

    // process images configured for preprocessing
    GFXU_PreprocessImage(&iv_bridge,0xA97AFC00,GFX_COLOR_MODE_RGBA_8888,GFX_FALSE);
    GFXU_PreprocessImage(&iv_car,0xAAA707E0,GFX_COLOR_MODE_RGBA_8888,GFX_FALSE);
    GFXU_PreprocessImage(&iv_watch,0xABD313C0,GFX_COLOR_MODE_RGBA_8888,GFX_FALSE);
    GFXU_PreprocessImage(&iv_meter,0xACFF1FA0,GFX_COLOR_MODE_RGBA_8888,GFX_FALSE);

    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_SplashScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_ImageScreen);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(0);

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

    LabelWidget3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget3, 137, 171);
    laWidget_SetSize((laWidget*)LabelWidget3, 213, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget3, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget3, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget3, laString_CreateFromID(string_Name));
    laLabelWidget_SetHAlignment(LabelWidget3, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget3);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetScheme((laWidget*)layer1, &ClearScreen);
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
    laWidget_SetBackgroundType((laWidget*)layer2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetScheme((laWidget*)layer2, &ClearScreen);
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

static void ScreenCreate_ImageScreen(laScreen* screen)
{
    laLayer* layer0;
    laLayer* layer1;
    laLayer* layer2;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetScheme((laWidget*)layer0, &ClearScreen);
    laLayer_SetBufferCount(layer0, 1);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    ImagePlusWidget2 = laImagePlusWidget_New();
    laWidget_SetSize((laWidget*)ImagePlusWidget2, 480, 260);
    laWidget_SetBackgroundType((laWidget*)ImagePlusWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImagePlusWidget2, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImagePlusWidget2);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetScheme((laWidget*)layer1, &ClearScreen);
    laLayer_SetBufferCount(layer1, 1);
    laLayer_SetVSync(layer1, LA_FALSE);

    laScreen_SetLayer(screen, 1, layer1);

    Hint = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Hint, 175, 2);
    laWidget_SetSize((laWidget*)Hint, 140, 25);
    laWidget_SetScheme((laWidget*)Hint, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)Hint, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)Hint, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(Hint, laString_CreateFromID(string_Hint));
    laLabelWidget_SetHAlignment(Hint, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer1, (laWidget*)Hint);

    ButtonWidget1 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget1, 417, 202);
    laWidget_SetSize((laWidget*)ButtonWidget1, 64, 64);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonWidget1, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonWidget1, &iv_zero_pressed);
    laButtonWidget_SetReleasedImage(ButtonWidget1, &iv_zero);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget1, &ButtonWidget1_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer1, (laWidget*)ButtonWidget1);

    HelpArea = laWidget_New();
    laWidget_SetPosition((laWidget*)HelpArea, 103, 3);
    laWidget_SetSize((laWidget*)HelpArea, 287, 37);
    laWidget_SetBackgroundType((laWidget*)HelpArea, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)HelpArea, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer1, HelpArea);

    LabelWidget6 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget6, 10, 240);
    laWidget_SetSize((laWidget*)LabelWidget6, 150, 25);
    laWidget_SetScheme((laWidget*)LabelWidget6, &ClearScreen);
    laWidget_SetBackgroundType((laWidget*)LabelWidget6, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget6, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget6, laString_CreateFromID(string_Name));
    laLabelWidget_SetHAlignment(LabelWidget6, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer1, (laWidget*)LabelWidget6);

    layer2 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer2, 0, 0);
    laWidget_SetSize((laWidget*)layer2, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer2, LA_WIDGET_BACKGROUND_NONE);
    laLayer_SetBufferCount(layer2, 1);
    laLayer_SetVSync(layer2, LA_FALSE);

    laScreen_SetLayer(screen, 2, layer2);

    LabelWidget5 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget5, 99, 7);
    laWidget_SetSize((laWidget*)LabelWidget5, 299, 25);
    laWidget_SetScheme((laWidget*)LabelWidget5, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget5, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget5, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget5, laString_CreateFromID(string_Help));
    laLabelWidget_SetHAlignment(LabelWidget5, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer2, (laWidget*)LabelWidget5);

    ImageWidget = laImageWidget_New();
    laWidget_SetSize((laWidget*)ImageWidget, 480, 272);
    laWidget_SetBackgroundType((laWidget*)ImageWidget, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ImageWidget, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget, &iv_help);
    laWidget_AddChild((laWidget*)layer2, (laWidget*)ImageWidget);

}



