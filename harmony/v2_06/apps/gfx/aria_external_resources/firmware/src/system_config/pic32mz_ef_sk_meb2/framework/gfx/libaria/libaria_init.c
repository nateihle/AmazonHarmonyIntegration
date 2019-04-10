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

laScheme whiteScheme;
laScheme clearscheme;
laScheme ListWheelImage1;
laScheme NewScheme;
laScheme ListWheelType;
laImageWidget* SplashImageWidget1;
laImageWidget* SplashImageWidget2;
laWidget* PanelWidget1;
laImageWidget* SplashImageWidget3;
laImageWidget* SplashImageWidget4;
laImageWidget* ImageWidget_newapp;
laButtonWidget* ButtonUSB;
laButtonWidget* ButtonSQI;
laButtonWidget* ButtonIntMem;
laButtonWidget* ButtonInfo;
laButtonWidget* ButtonSlides;
laLabelWidget* LabelWidget_source;
laButtonWidget* ButtonAllPlay;
laWidget* PanelWidget_timebackgrnd;
laLabelWidget* Label_Usb_time;
laLabelWidget* Label_Int_time;
laLabelWidget* Label_Sqi_time;
laLabelWidget* LabelWidget1_IntTimeLabel;
laLabelWidget* LabelWidget_SqiTimeLabel;
laLabelWidget* LabelWidget_UsbTimeLabel;
laWidget* PanelWidget_loadtimeslides;
laLabelWidget* LabelWidget_load_time;
laLabelWidget* LabelWidget_time;
laLabelWidget* LabelWidget_ImageType;
laLabelWidget* label_ms_1;
laLabelWidget* label_ms_2;
laLabelWidget* label_ms_3;
laImageWidget* ImageWidget_help;
laButtonWidget* ButtonWidget_back;


static void ScreenCreate_SplashScreen(laScreen* screen);
static void ScreenCreate_New_appScreen(laScreen* screen);
static void ScreenCreate_helpScreen(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&whiteScheme, GFX_COLOR_MODE_RGB_565);
    whiteScheme.base = 0xFFFF;
    whiteScheme.highlight = 0xC67A;
    whiteScheme.highlightLight = 0xFFFF;
    whiteScheme.shadow = 0x8410;
    whiteScheme.shadowDark = 0x4208;
    whiteScheme.foreground = 0x0;
    whiteScheme.foregroundInactive = 0xD71C;
    whiteScheme.foregroundDisabled = 0x8410;
    whiteScheme.background = 0xFFFF;
    whiteScheme.backgroundInactive = 0xD71C;
    whiteScheme.backgroundDisabled = 0xC67A;
    whiteScheme.text = 0x0;
    whiteScheme.textHighlight = 0x1F;
    whiteScheme.textHighlightText = 0xFFFF;
    whiteScheme.textInactive = 0xD71C;
    whiteScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&clearscheme, GFX_COLOR_MODE_RGB_565);
    clearscheme.base = 0x0;
    clearscheme.highlight = 0xC67A;
    clearscheme.highlightLight = 0xFFFF;
    clearscheme.shadow = 0x8410;
    clearscheme.shadowDark = 0x4208;
    clearscheme.foreground = 0x0;
    clearscheme.foregroundInactive = 0xD71C;
    clearscheme.foregroundDisabled = 0x8410;
    clearscheme.background = 0x0;
    clearscheme.backgroundInactive = 0xD71C;
    clearscheme.backgroundDisabled = 0xC67A;
    clearscheme.text = 0xBDF7;
    clearscheme.textHighlight = 0x1F;
    clearscheme.textHighlightText = 0xFFF7;
    clearscheme.textInactive = 0xD71C;
    clearscheme.textDisabled = 0x8C92;

    laScheme_Initialize(&ListWheelImage1, GFX_COLOR_MODE_RGB_565);
    ListWheelImage1.base = 0x957A;
    ListWheelImage1.highlight = 0x8410;
    ListWheelImage1.highlightLight = 0xADBF;
    ListWheelImage1.shadow = 0x8410;
    ListWheelImage1.shadowDark = 0x4208;
    ListWheelImage1.foreground = 0x0;
    ListWheelImage1.foregroundInactive = 0x7D7A;
    ListWheelImage1.foregroundDisabled = 0xBDF7;
    ListWheelImage1.background = 0x7DDC;
    ListWheelImage1.backgroundInactive = 0x7D17;
    ListWheelImage1.backgroundDisabled = 0x43B4;
    ListWheelImage1.text = 0x0;
    ListWheelImage1.textHighlight = 0x1F;
    ListWheelImage1.textHighlightText = 0xFFFF;
    ListWheelImage1.textInactive = 0xD71C;
    ListWheelImage1.textDisabled = 0x8C92;

    laScheme_Initialize(&NewScheme, GFX_COLOR_MODE_RGB_565);
    NewScheme.base = 0xFEF7;
    NewScheme.highlight = 0x5960;
    NewScheme.highlightLight = 0x5140;
    NewScheme.shadow = 0x69A0;
    NewScheme.shadowDark = 0x5140;
    NewScheme.foreground = 0x4920;
    NewScheme.foregroundInactive = 0xD71C;
    NewScheme.foregroundDisabled = 0x8410;
    NewScheme.background = 0xFEF7;
    NewScheme.backgroundInactive = 0xD71C;
    NewScheme.backgroundDisabled = 0xC67A;
    NewScheme.text = 0x10;
    NewScheme.textHighlight = 0x1F;
    NewScheme.textHighlightText = 0xFFFF;
    NewScheme.textInactive = 0xD71C;
    NewScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&ListWheelType, GFX_COLOR_MODE_RGB_565);
    ListWheelType.base = 0xC67A;
    ListWheelType.highlight = 0xC67A;
    ListWheelType.highlightLight = 0xFFFF;
    ListWheelType.shadow = 0x8410;
    ListWheelType.shadowDark = 0x4208;
    ListWheelType.foreground = 0x0;
    ListWheelType.foregroundInactive = 0xC6BC;
    ListWheelType.foregroundDisabled = 0x8410;
    ListWheelType.background = 0x969F;
    ListWheelType.backgroundInactive = 0xD71C;
    ListWheelType.backgroundDisabled = 0x753A;
    ListWheelType.text = 0x0;
    ListWheelType.textHighlight = 0x1F;
    ListWheelType.textHighlightText = 0xFFFF;
    ListWheelType.textInactive = 0xD71C;
    ListWheelType.textDisabled = 0x8C92;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_SplashScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_New_appScreen);
    laContext_AddScreen(screen);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_helpScreen);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(1);

	return 0;
}

static void ScreenCreate_SplashScreen(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    SplashImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)SplashImageWidget1, 120, 40);
    laWidget_SetSize((laWidget*)SplashImageWidget1, 240, 139);
    laWidget_SetScheme((laWidget*)SplashImageWidget1, &whiteScheme);
    laWidget_SetBackgroundType((laWidget*)SplashImageWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)SplashImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(SplashImageWidget1, &PIC32Logo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)SplashImageWidget1);

    SplashImageWidget2 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)SplashImageWidget2, 120, 40);
    laWidget_SetSize((laWidget*)SplashImageWidget2, 240, 139);
    laWidget_SetVisible((laWidget*)SplashImageWidget2, LA_FALSE);
    laWidget_SetScheme((laWidget*)SplashImageWidget2, &whiteScheme);
    laWidget_SetBackgroundType((laWidget*)SplashImageWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)SplashImageWidget2, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(SplashImageWidget2, &HarmonyLogo_1);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)SplashImageWidget2);

    PanelWidget1 = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget1, 0, 207);
    laWidget_SetSize((laWidget*)PanelWidget1, 480, 65);
    laWidget_SetBackgroundType((laWidget*)PanelWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)PanelWidget1, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget1);

    SplashImageWidget3 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)SplashImageWidget3, 480, 0);
    laWidget_SetSize((laWidget*)SplashImageWidget3, 480, 65);
    laWidget_SetBackgroundType((laWidget*)SplashImageWidget3, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)SplashImageWidget3, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(SplashImageWidget3, &Bar);
    laWidget_AddChild((laWidget*)PanelWidget1, (laWidget*)SplashImageWidget3);

    SplashImageWidget4 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)SplashImageWidget4, 17, 33);
    laWidget_SetSize((laWidget*)SplashImageWidget4, 144, 39);
    laWidget_SetVisible((laWidget*)SplashImageWidget4, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)SplashImageWidget4, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)SplashImageWidget4, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(SplashImageWidget4, &MicrochipLogo_1);
    laWidget_AddChild((laWidget*)PanelWidget1, (laWidget*)SplashImageWidget4);

}

static void ScreenCreate_New_appScreen(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    ImageWidget_newapp = laImageWidget_New();
    laWidget_SetSize((laWidget*)ImageWidget_newapp, 480, 272);
    laWidget_SetBackgroundType((laWidget*)ImageWidget_newapp, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget_newapp, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget_newapp);

    ButtonUSB = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonUSB, 159, 191);
    laWidget_SetSize((laWidget*)ButtonUSB, 73, 73);
    laWidget_SetBackgroundType((laWidget*)ButtonUSB, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)ButtonUSB, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonUSB, &usbrect_gimp_70);
    laButtonWidget_SetReleasedImage(ButtonUSB, &usbrect_gimp_70);
    laButtonWidget_SetReleasedEventCallback(ButtonUSB, &ButtonUSB_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonUSB);

    ButtonSQI = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonSQI, 238, 192);
    laWidget_SetSize((laWidget*)ButtonSQI, 79, 71);
    laWidget_SetBackgroundType((laWidget*)ButtonSQI, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)ButtonSQI, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonSQI, &sqiFlashrect_red_gimp_70);
    laButtonWidget_SetReleasedImage(ButtonSQI, &sqiFlashrect_red_gimp_70);
    laButtonWidget_SetReleasedEventCallback(ButtonSQI, &ButtonSQI_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonSQI);

    ButtonIntMem = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonIntMem, 324, 191);
    laWidget_SetSize((laWidget*)ButtonIntMem, 71, 71);
    laWidget_SetBackgroundType((laWidget*)ButtonIntMem, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)ButtonIntMem, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedImage(ButtonIntMem, &intMemrect_pic32_gimp_70);
    laButtonWidget_SetReleasedEventCallback(ButtonIntMem, &ButtonIntMem_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonIntMem);

    ButtonInfo = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonInfo, 6, 191);
    laWidget_SetSize((laWidget*)ButtonInfo, 77, 73);
    laWidget_SetBackgroundType((laWidget*)ButtonInfo, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)ButtonInfo, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonInfo, &questionmark_gimp_70);
    laButtonWidget_SetReleasedImage(ButtonInfo, &questionmark_gimp_70);
    laButtonWidget_SetReleasedEventCallback(ButtonInfo, &ButtonInfo_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonInfo);

    ButtonSlides = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonSlides, 400, 185);
    laWidget_SetSize((laWidget*)ButtonSlides, 79, 80);
    laWidget_SetBackgroundType((laWidget*)ButtonSlides, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)ButtonSlides, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonSlides, &slides_icon_gimp_70);
    laButtonWidget_SetReleasedImage(ButtonSlides, &slides_icon_gimp_70);
    laButtonWidget_SetReleasedEventCallback(ButtonSlides, &ButtonSlides_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonSlides);

    LabelWidget_source = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget_source, 5, 6);
    laWidget_SetSize((laWidget*)LabelWidget_source, 337, 25);
    laWidget_SetVisible((laWidget*)LabelWidget_source, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)LabelWidget_source, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget_source, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(LabelWidget_source, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget_source);

    ButtonAllPlay = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonAllPlay, 398, 5);
    laWidget_SetSize((laWidget*)ButtonAllPlay, 77, 73);
    laWidget_SetBackgroundType((laWidget*)ButtonAllPlay, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)ButtonAllPlay, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonAllPlay, &loadtimes_gimp_70);
    laButtonWidget_SetReleasedImage(ButtonAllPlay, &loadtimes_gimp_70);
    laButtonWidget_SetReleasedEventCallback(ButtonAllPlay, &ButtonAllPlay_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonAllPlay);

    PanelWidget_timebackgrnd = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget_timebackgrnd, 75, 75);
    laWidget_SetSize((laWidget*)PanelWidget_timebackgrnd, 290, 107);
    laWidget_SetVisible((laWidget*)PanelWidget_timebackgrnd, LA_FALSE);
    laWidget_SetScheme((laWidget*)PanelWidget_timebackgrnd, &NewScheme);
    laWidget_SetBackgroundType((laWidget*)PanelWidget_timebackgrnd, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelWidget_timebackgrnd, LA_WIDGET_BORDER_LINE);
    laWidget_SetMargins((laWidget*)PanelWidget_timebackgrnd, 40, 40, 40, 40);
    laWidget_AddChild((laWidget*)layer0, PanelWidget_timebackgrnd);

    Label_Usb_time = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Label_Usb_time, 142, 75);
    laWidget_SetSize((laWidget*)Label_Usb_time, 102, 25);
    laWidget_SetVisible((laWidget*)Label_Usb_time, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)Label_Usb_time, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)Label_Usb_time, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelWidget_timebackgrnd, (laWidget*)Label_Usb_time);

    Label_Int_time = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Label_Int_time, 145, 6);
    laWidget_SetSize((laWidget*)Label_Int_time, 102, 25);
    laWidget_SetVisible((laWidget*)Label_Int_time, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)Label_Int_time, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)Label_Int_time, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelWidget_timebackgrnd, (laWidget*)Label_Int_time);

    Label_Sqi_time = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)Label_Sqi_time, 144, 40);
    laWidget_SetSize((laWidget*)Label_Sqi_time, 103, 25);
    laWidget_SetVisible((laWidget*)Label_Sqi_time, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)Label_Sqi_time, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)Label_Sqi_time, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelWidget_timebackgrnd, (laWidget*)Label_Sqi_time);

    LabelWidget1_IntTimeLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget1_IntTimeLabel, 5, 5);
    laWidget_SetSize((laWidget*)LabelWidget1_IntTimeLabel, 169, 25);
    laWidget_SetVisible((laWidget*)LabelWidget1_IntTimeLabel, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)LabelWidget1_IntTimeLabel, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget1_IntTimeLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget1_IntTimeLabel, laString_CreateFromID(string_IntFLash));
    laLabelWidget_SetHAlignment(LabelWidget1_IntTimeLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget_timebackgrnd, (laWidget*)LabelWidget1_IntTimeLabel);

    LabelWidget_SqiTimeLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget_SqiTimeLabel, 6, 41);
    laWidget_SetSize((laWidget*)LabelWidget_SqiTimeLabel, 166, 25);
    laWidget_SetVisible((laWidget*)LabelWidget_SqiTimeLabel, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)LabelWidget_SqiTimeLabel, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget_SqiTimeLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget_SqiTimeLabel, laString_CreateFromID(string_SQI));
    laLabelWidget_SetHAlignment(LabelWidget_SqiTimeLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget_timebackgrnd, (laWidget*)LabelWidget_SqiTimeLabel);

    LabelWidget_UsbTimeLabel = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget_UsbTimeLabel, 6, 76);
    laWidget_SetSize((laWidget*)LabelWidget_UsbTimeLabel, 164, 25);
    laWidget_SetVisible((laWidget*)LabelWidget_UsbTimeLabel, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)LabelWidget_UsbTimeLabel, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget_UsbTimeLabel, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget_UsbTimeLabel, laString_CreateFromID(string_USBFile));
    laLabelWidget_SetHAlignment(LabelWidget_UsbTimeLabel, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget_timebackgrnd, (laWidget*)LabelWidget_UsbTimeLabel);

    PanelWidget_loadtimeslides = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelWidget_loadtimeslides, 6, 28);
    laWidget_SetSize((laWidget*)PanelWidget_loadtimeslides, 222, 52);
    laWidget_SetBackgroundType((laWidget*)PanelWidget_loadtimeslides, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)PanelWidget_loadtimeslides, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelWidget_loadtimeslides);

    LabelWidget_load_time = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget_load_time, 7, 4);
    laWidget_SetSize((laWidget*)LabelWidget_load_time, 114, 25);
    laWidget_SetVisible((laWidget*)LabelWidget_load_time, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)LabelWidget_load_time, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget_load_time, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(LabelWidget_load_time, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget_loadtimeslides, (laWidget*)LabelWidget_load_time);

    LabelWidget_time = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget_time, 123, 3);
    laWidget_SetSize((laWidget*)LabelWidget_time, 84, 25);
    laWidget_SetVisible((laWidget*)LabelWidget_time, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)LabelWidget_time, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget_time, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(LabelWidget_time, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget_loadtimeslides, (laWidget*)LabelWidget_time);

    LabelWidget_ImageType = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget_ImageType, 6, 27);
    laWidget_SetSize((laWidget*)LabelWidget_ImageType, 95, 25);
    laWidget_SetVisible((laWidget*)LabelWidget_ImageType, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)LabelWidget_ImageType, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)LabelWidget_ImageType, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetHAlignment(LabelWidget_ImageType, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)PanelWidget_loadtimeslides, (laWidget*)LabelWidget_ImageType);

    label_ms_1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)label_ms_1, 328, 82);
    laWidget_SetSize((laWidget*)label_ms_1, 32, 25);
    laWidget_SetVisible((laWidget*)label_ms_1, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)label_ms_1, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)label_ms_1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(label_ms_1, laString_CreateFromID(string_ms));
    laLabelWidget_SetHAlignment(label_ms_1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)label_ms_1);

    label_ms_2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)label_ms_2, 329, 116);
    laWidget_SetSize((laWidget*)label_ms_2, 32, 25);
    laWidget_SetVisible((laWidget*)label_ms_2, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)label_ms_2, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)label_ms_2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(label_ms_2, laString_CreateFromID(string_ms));
    laLabelWidget_SetHAlignment(label_ms_2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)label_ms_2);

    label_ms_3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)label_ms_3, 329, 152);
    laWidget_SetSize((laWidget*)label_ms_3, 31, 25);
    laWidget_SetVisible((laWidget*)label_ms_3, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)label_ms_3, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)label_ms_3, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(label_ms_3, laString_CreateFromID(string_ms));
    laLabelWidget_SetHAlignment(label_ms_3, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)label_ms_3);

}

static void ScreenCreate_helpScreen(laScreen* screen)
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
    laWidget_SetPosition((laWidget*)ButtonWidget_back, 405, 192);
    laWidget_SetSize((laWidget*)ButtonWidget_back, 74, 74);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget_back, LA_WIDGET_BACKGROUND_CACHE);
    laWidget_SetBorderType((laWidget*)ButtonWidget_back, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetPressedImage(ButtonWidget_back, &back_gimp_70);
    laButtonWidget_SetReleasedImage(ButtonWidget_back, &back_gimp_70);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget_back, &ButtonWidget_back_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget_back);

}



