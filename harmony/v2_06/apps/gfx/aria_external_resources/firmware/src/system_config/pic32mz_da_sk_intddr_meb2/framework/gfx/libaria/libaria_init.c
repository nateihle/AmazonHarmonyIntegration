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

laScheme InstructionScheme;
laScheme whiteScheme;
laScheme ListWheelImage1;
laScheme ListWheelType;
laScheme Clearscheme;
laImageWidget* ImageWidget1;
laImageWidget* ImageWidget3;
laImageWidget* ImageWidget2;
laImageWidget* ImageWidget4;
laImageWidget* ImageWidget_newapp;
laWidget* PanelWidget_timebackgrnd;
laLabelWidget* Label_Usb_time;
laLabelWidget* Label_Int_time;
laLabelWidget* Label_Sqi_time;
laLabelWidget* LabelWidget1_IntTimeLabel;
laLabelWidget* LabelWidget_SqiTimeLabel;
laLabelWidget* LabelWidget_UsbTimeLabel;
laLabelWidget* label_ms_1;
laLabelWidget* label_ms_2;
laLabelWidget* label_ms_3;
laLabelWidget* LabelWidget_source;
laWidget* PanelWidget_loadtimeslides;
laLabelWidget* LabelWidget_load_time;
laLabelWidget* LabelWidget_time;
laLabelWidget* LabelWidget_ImageType;
laButtonWidget* ButtonUSB;
laButtonWidget* ButtonSQI;
laButtonWidget* ButtonIntMem;
laButtonWidget* ButtonInfo;
laButtonWidget* ButtonSlides;
laButtonWidget* ButtonAllPlay;
laImageWidget* ImageWidget_help;
laButtonWidget* ButtonWidget_back;


static void ScreenCreate_SplashScreen(laScreen* screen);
static void ScreenCreate_New_appScreen(laScreen* screen);
static void ScreenCreate_helpscreen(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&InstructionScheme, GFX_COLOR_MODE_RGBA_8888);
    InstructionScheme.base = 0xC8D0D4FF;
    InstructionScheme.highlight = 0xC8D0D4FF;
    InstructionScheme.highlightLight = 0xFFFFFFFF;
    InstructionScheme.shadow = 0x808080FF;
    InstructionScheme.shadowDark = 0x404040FF;
    InstructionScheme.foreground = 0xFF;
    InstructionScheme.foregroundInactive = 0xD6E3E7FF;
    InstructionScheme.foregroundDisabled = 0x808080FF;
    InstructionScheme.background = 0xFFFFFFFF;
    InstructionScheme.backgroundInactive = 0xD6E3E7FF;
    InstructionScheme.backgroundDisabled = 0xC8D0D4FF;
    InstructionScheme.text = 0xBDBEBDFF;
    InstructionScheme.textHighlight = 0xFFFF;
    InstructionScheme.textHighlightText = 0xFFFFBDFF;
    InstructionScheme.textInactive = 0xD6E3E7FF;
    InstructionScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&whiteScheme, GFX_COLOR_MODE_RGBA_8888);
    whiteScheme.base = 0xFFFFFFFF;
    whiteScheme.highlight = 0xC8D0D4FF;
    whiteScheme.highlightLight = 0xFFFFFFFF;
    whiteScheme.shadow = 0x808080FF;
    whiteScheme.shadowDark = 0x404040FF;
    whiteScheme.foreground = 0xFF;
    whiteScheme.foregroundInactive = 0xD6E3E7FF;
    whiteScheme.foregroundDisabled = 0x808080FF;
    whiteScheme.background = 0xFFFFFFFF;
    whiteScheme.backgroundInactive = 0xD6E3E7FF;
    whiteScheme.backgroundDisabled = 0xC8D0D4FF;
    whiteScheme.text = 0xFF;
    whiteScheme.textHighlight = 0xFFFF;
    whiteScheme.textHighlightText = 0xFFFFFFFF;
    whiteScheme.textInactive = 0xD6E3E7FF;
    whiteScheme.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&ListWheelImage1, GFX_COLOR_MODE_RGBA_8888);
    ListWheelImage1.base = 0x94AED6FF;
    ListWheelImage1.highlight = 0x848284FF;
    ListWheelImage1.highlightLight = 0xADB6FFFF;
    ListWheelImage1.shadow = 0x808080FF;
    ListWheelImage1.shadowDark = 0x404040FF;
    ListWheelImage1.foreground = 0xFF;
    ListWheelImage1.foregroundInactive = 0x7BAED6FF;
    ListWheelImage1.foregroundDisabled = 0xBDBEBDFF;
    ListWheelImage1.background = 0x7BBAE6FF;
    ListWheelImage1.backgroundInactive = 0x7BA2BDFF;
    ListWheelImage1.backgroundDisabled = 0x4275A5FF;
    ListWheelImage1.text = 0xFF;
    ListWheelImage1.textHighlight = 0xFFFF;
    ListWheelImage1.textHighlightText = 0xFFFFFFFF;
    ListWheelImage1.textInactive = 0xD6E3E7FF;
    ListWheelImage1.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&ListWheelType, GFX_COLOR_MODE_RGBA_8888);
    ListWheelType.base = 0xC8D0D4FF;
    ListWheelType.highlight = 0xC8D0D4FF;
    ListWheelType.highlightLight = 0xFFFFFFFF;
    ListWheelType.shadow = 0x808080FF;
    ListWheelType.shadowDark = 0x404040FF;
    ListWheelType.foreground = 0xFF;
    ListWheelType.foregroundInactive = 0xC5D7E6FF;
    ListWheelType.foregroundDisabled = 0x808080FF;
    ListWheelType.background = 0x94D2FFFF;
    ListWheelType.backgroundInactive = 0xD6E3E7FF;
    ListWheelType.backgroundDisabled = 0x73A6D6FF;
    ListWheelType.text = 0xFF;
    ListWheelType.textHighlight = 0xFFFF;
    ListWheelType.textHighlightText = 0xFFFFFFFF;
    ListWheelType.textInactive = 0xD6E3E7FF;
    ListWheelType.textDisabled = 0x8C9294FF;

    laScheme_Initialize(&Clearscheme, GFX_COLOR_MODE_RGBA_8888);
    Clearscheme.base = 0x0;
    Clearscheme.highlight = 0x5A2D00FF;
    Clearscheme.highlightLight = 0x522800FF;
    Clearscheme.shadow = 0x6B3500FF;
    Clearscheme.shadowDark = 0x522800FF;
    Clearscheme.foreground = 0x4A2400FF;
    Clearscheme.foregroundInactive = 0xD6E3E7FF;
    Clearscheme.foregroundDisabled = 0x808080FF;
    Clearscheme.background = 0x0;
    Clearscheme.backgroundInactive = 0xD6E3E7FF;
    Clearscheme.backgroundDisabled = 0xC8D0D4FF;
    Clearscheme.text = 0xFF;
    Clearscheme.textHighlight = 0xFF;
    Clearscheme.textHighlightText = 0xFFFFFFFF;
    Clearscheme.textInactive = 0xD6E3E7FF;
    Clearscheme.textDisabled = 0x8C9294FF;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_SplashScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_New_appScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_helpscreen);
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
    laLayer_SetBufferCount(layer0, 1);
    laLayer_SetAlphaEnable(layer0, LA_TRUE);
    laLayer_SetAlphaAmount(layer0, 0xFF);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 124, 68);
    laWidget_SetSize((laWidget*)ImageWidget1, 240, 78);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &PIC32Logo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_NONE);
    laLayer_SetBufferCount(layer1, 1);
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
    laLayer_SetBufferCount(layer2, 1);
    laLayer_SetAlphaEnable(layer2, LA_TRUE);
    laLayer_SetAlphaAmount(layer2, 0x0);

    laScreen_SetLayer(screen, 2, layer2);

    ImageWidget2 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget2, 136, 40);
    laWidget_SetSize((laWidget*)ImageWidget2, 206, 139);
    laWidget_SetBackgroundType((laWidget*)ImageWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget2, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget2, &HarmonyLogo_1);
    laWidget_AddChild((laWidget*)layer2, (laWidget*)ImageWidget2);

    ImageWidget4 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget4, 5, 228);
    laWidget_SetSize((laWidget*)ImageWidget4, 144, 39);
    laWidget_SetBackgroundType((laWidget*)ImageWidget4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget4, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget4, &MicrochipLogo_1);
    laWidget_AddChild((laWidget*)layer2, (laWidget*)ImageWidget4);

}

static void ScreenCreate_New_appScreen(laScreen* screen)
{
    laLayer* layer0;
    laLayer* layer1;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);
    laLayer_SetAlphaEnable(layer0, LA_TRUE);
    laLayer_SetAlphaAmount(layer0, 0xFF);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget_newapp = laImageWidget_New();
    laWidget_SetSize((laWidget*)ImageWidget_newapp, 480, 272);
    laWidget_SetBackgroundType((laWidget*)ImageWidget_newapp, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget_newapp, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget_newapp);

    PanelWidget_timebackgrnd = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget_timebackgrnd, 75, 75);
    laWidget_SetSize((laWidget*)PanelWidget_timebackgrnd, 290, 107);
    laWidget_SetVisible((laWidget*)PanelWidget_timebackgrnd, LA_FALSE);
    laWidget_SetScheme((laWidget*)PanelWidget_timebackgrnd, &whiteScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget_timebackgrnd, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget_timebackgrnd, LA_WIDGET_BORDER_LINE);
    laWidget_SetMargins((laWidget*)PanelWidget_timebackgrnd, 40, 40, 40, 40);
    laWidget_AddChild((laWidget*)layer0, PanelWidget_timebackgrnd);

    Label_Usb_time = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Label_Usb_time, 147, 77);
    laWidget_SetSize((laWidget*)Label_Usb_time, 102, 25);
    laWidget_SetVisible((laWidget*)Label_Usb_time, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)Label_Usb_time, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)Label_Usb_time, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelWidget_timebackgrnd, (laWidget*)Label_Usb_time);

    Label_Int_time = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Label_Int_time, 144, 4);
    laWidget_SetSize((laWidget*)Label_Int_time, 102, 25);
    laWidget_SetVisible((laWidget*)Label_Int_time, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)Label_Int_time, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)Label_Int_time, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelWidget_timebackgrnd, (laWidget*)Label_Int_time);

    Label_Sqi_time = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Label_Sqi_time, 146, 39);
    laWidget_SetSize((laWidget*)Label_Sqi_time, 103, 25);
    laWidget_SetVisible((laWidget*)Label_Sqi_time, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)Label_Sqi_time, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)Label_Sqi_time, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelWidget_timebackgrnd, (laWidget*)Label_Sqi_time);

    LabelWidget1_IntTimeLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget1_IntTimeLabel, 5, 5);
    laWidget_SetSize((laWidget*)LabelWidget1_IntTimeLabel, 149, 25);
    laWidget_SetVisible((laWidget*)LabelWidget1_IntTimeLabel, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)LabelWidget1_IntTimeLabel, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget1_IntTimeLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget1_IntTimeLabel, laString_CreateFromID(string_IntFLash));
    laLabelWidget_SetHAlignment(LabelWidget1_IntTimeLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget_timebackgrnd, (laWidget*)LabelWidget1_IntTimeLabel);

    LabelWidget_SqiTimeLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget_SqiTimeLabel, 6, 41);
    laWidget_SetSize((laWidget*)LabelWidget_SqiTimeLabel, 149, 25);
    laWidget_SetVisible((laWidget*)LabelWidget_SqiTimeLabel, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)LabelWidget_SqiTimeLabel, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget_SqiTimeLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget_SqiTimeLabel, laString_CreateFromID(string_SQI));
    laLabelWidget_SetHAlignment(LabelWidget_SqiTimeLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget_timebackgrnd, (laWidget*)LabelWidget_SqiTimeLabel);

    LabelWidget_UsbTimeLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget_UsbTimeLabel, 6, 76);
    laWidget_SetSize((laWidget*)LabelWidget_UsbTimeLabel, 148, 25);
    laWidget_SetVisible((laWidget*)LabelWidget_UsbTimeLabel, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)LabelWidget_UsbTimeLabel, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget_UsbTimeLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget_UsbTimeLabel, laString_CreateFromID(string_USBFile));
    laLabelWidget_SetHAlignment(LabelWidget_UsbTimeLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget_timebackgrnd, (laWidget*)LabelWidget_UsbTimeLabel);

    label_ms_1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)label_ms_1, 257, 5);
    laWidget_SetSize((laWidget*)label_ms_1, 30, 25);
    laWidget_SetVisible((laWidget*)label_ms_1, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)label_ms_1, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)label_ms_1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(label_ms_1, laString_CreateFromID(string_ms));
    laLabelWidget_SetHAlignment(label_ms_1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget_timebackgrnd, (laWidget*)label_ms_1);

    label_ms_2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)label_ms_2, 256, 42);
    laWidget_SetSize((laWidget*)label_ms_2, 32, 25);
    laWidget_SetVisible((laWidget*)label_ms_2, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)label_ms_2, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)label_ms_2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(label_ms_2, laString_CreateFromID(string_ms));
    laLabelWidget_SetHAlignment(label_ms_2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget_timebackgrnd, (laWidget*)label_ms_2);

    label_ms_3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)label_ms_3, 256, 78);
    laWidget_SetSize((laWidget*)label_ms_3, 31, 25);
    laWidget_SetVisible((laWidget*)label_ms_3, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)label_ms_3, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)label_ms_3, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(label_ms_3, laString_CreateFromID(string_ms));
    laLabelWidget_SetHAlignment(label_ms_3, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget_timebackgrnd, (laWidget*)label_ms_3);

    LabelWidget_source = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget_source, 6, 4);
    laWidget_SetSize((laWidget*)LabelWidget_source, 337, 22);
    laWidget_SetVisible((laWidget*)LabelWidget_source, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)LabelWidget_source, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget_source, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(LabelWidget_source, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget_source);

    layer1 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer1, 0, 0);
    laWidget_SetSize((laWidget*)layer1, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer1, &Clearscheme);
    laLayer_SetBufferCount(layer1, 1);
    laLayer_SetAlphaEnable(layer1, LA_TRUE);
    laLayer_SetAlphaAmount(layer1, 0xFF);

    laScreen_SetLayer(screen, 1, layer1);

    PanelWidget_loadtimeslides = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget_loadtimeslides, 2, 28);
    laWidget_SetSize((laWidget*)PanelWidget_loadtimeslides, 205, 54);
    laWidget_SetBackgroundType((laWidget*)PanelWidget_loadtimeslides, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PanelWidget_loadtimeslides, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer1, PanelWidget_loadtimeslides);

    LabelWidget_load_time = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget_load_time, 4, 2);
    laWidget_SetSize((laWidget*)LabelWidget_load_time, 117, 25);
    laWidget_SetVisible((laWidget*)LabelWidget_load_time, LA_FALSE);
    laWidget_SetScheme((laWidget*)LabelWidget_load_time, &Clearscheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget_load_time, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget_load_time, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(LabelWidget_load_time, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget_loadtimeslides, (laWidget*)LabelWidget_load_time);

    LabelWidget_time = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget_time, 119, 3);
    laWidget_SetSize((laWidget*)LabelWidget_time, 79, 25);
    laWidget_SetVisible((laWidget*)LabelWidget_time, LA_FALSE);
    laWidget_SetScheme((laWidget*)LabelWidget_time, &Clearscheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget_time, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget_time, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(LabelWidget_time, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget_loadtimeslides, (laWidget*)LabelWidget_time);

    LabelWidget_ImageType = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget_ImageType, 2, 27);
    laWidget_SetSize((laWidget*)LabelWidget_ImageType, 100, 25);
    laWidget_SetVisible((laWidget*)LabelWidget_ImageType, LA_FALSE);
    laWidget_SetScheme((laWidget*)LabelWidget_ImageType, &Clearscheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget_ImageType, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget_ImageType, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(LabelWidget_ImageType, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget_loadtimeslides, (laWidget*)LabelWidget_ImageType);

    ButtonUSB = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonUSB, 159, 191);
    laWidget_SetSize((laWidget*)ButtonUSB, 73, 73);
    laWidget_SetBackgroundType((laWidget*)ButtonUSB, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonUSB, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonUSB, &usbrect_gimp_70);
    laButtonWidget_SetReleasedImage(ButtonUSB, &usbrect_gimp_70);
    laButtonWidget_SetReleasedEventCallback(ButtonUSB, &ButtonUSB_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer1, (laWidget*)ButtonUSB);

    ButtonSQI = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonSQI, 238, 192);
    laWidget_SetSize((laWidget*)ButtonSQI, 79, 71);
    laWidget_SetBackgroundType((laWidget*)ButtonSQI, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonSQI, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonSQI, &sqiFlashrect_red_gimp_70);
    laButtonWidget_SetReleasedImage(ButtonSQI, &sqiFlashrect_red_gimp_70);
    laButtonWidget_SetReleasedEventCallback(ButtonSQI, &ButtonSQI_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer1, (laWidget*)ButtonSQI);

    ButtonIntMem = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonIntMem, 324, 191);
    laWidget_SetSize((laWidget*)ButtonIntMem, 71, 71);
    laWidget_SetBackgroundType((laWidget*)ButtonIntMem, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonIntMem, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonIntMem, &intMemrect_pic32_gimp_70);
    laButtonWidget_SetReleasedImage(ButtonIntMem, &intMemrect_pic32_gimp_70);
    laButtonWidget_SetReleasedEventCallback(ButtonIntMem, &ButtonIntMem_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer1, (laWidget*)ButtonIntMem);

    ButtonInfo = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonInfo, 4, 192);
    laWidget_SetSize((laWidget*)ButtonInfo, 77, 73);
    laWidget_SetBackgroundType((laWidget*)ButtonInfo, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonInfo, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonInfo, &questionmark_gimp_70);
    laButtonWidget_SetReleasedImage(ButtonInfo, &questionmark_gimp_70);
    laButtonWidget_SetReleasedEventCallback(ButtonInfo, &ButtonInfo_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer1, (laWidget*)ButtonInfo);

    ButtonSlides = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonSlides, 400, 185);
    laWidget_SetSize((laWidget*)ButtonSlides, 79, 80);
    laWidget_SetBackgroundType((laWidget*)ButtonSlides, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonSlides, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonSlides, &slides_icon_gimp_70);
    laButtonWidget_SetReleasedImage(ButtonSlides, &slides_icon_gimp_70);
    laButtonWidget_SetReleasedEventCallback(ButtonSlides, &ButtonSlides_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer1, (laWidget*)ButtonSlides);

    ButtonAllPlay = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonAllPlay, 402, 3);
    laWidget_SetSize((laWidget*)ButtonAllPlay, 75, 74);
    laWidget_SetBackgroundType((laWidget*)ButtonAllPlay, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ButtonAllPlay, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonAllPlay, &loadtimes_gimp_70);
    laButtonWidget_SetReleasedImage(ButtonAllPlay, &loadtimes_gimp_70);
    laButtonWidget_SetReleasedEventCallback(ButtonAllPlay, &ButtonAllPlay_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer1, (laWidget*)ButtonAllPlay);

}

static void ScreenCreate_helpscreen(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget_help = laImageWidget_New();
    laWidget_SetSize((laWidget*)ImageWidget_help, 480, 272);
    laWidget_SetBackgroundType((laWidget*)ImageWidget_help, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget_help, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget_help, &ext_res_help);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget_help);

    ButtonWidget_back = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget_back, 407, 183);
    laWidget_SetSize((laWidget*)ButtonWidget_back, 72, 77);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget_back, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)ButtonWidget_back, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonWidget_back, &back_gimp_70);
    laButtonWidget_SetReleasedImage(ButtonWidget_back, &back_gimp_70);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget_back, &ButtonWidget_back_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget_back);

}



