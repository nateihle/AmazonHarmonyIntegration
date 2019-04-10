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

laScheme RadioScheme;
laScheme MuteButtonScheme;
laScheme BackgroundScheme;
laScheme TrackListScheme;
laScheme GradientScheme;
laRectangleWidget* Background;
laGradientWidget* GradientWidget1;
laGroupBoxWidget* GroupBoxWidget1;
laLabelWidget* DemoName;
laButtonWidget* RadioButtonSd_InvisibleTouchArea;
laRadioButtonWidget* RadioButtonSd;
laButtonWidget* RadioButtonUSB_InvisibleTouchArea;
laRadioButtonWidget* RadioButtonUSB;
laListWidget* TrackListBox;
laButtonWidget* MuteButton;
laImageWidget* MicrochipLogo;
laButtonWidget* VolumeDownButton;
laButtonWidget* VolumeUpButton;
laProgressBarWidget* VolumeBar;


static void ScreenCreate_MchpName(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&RadioScheme, GFX_COLOR_MODE_RGB_565);
    RadioScheme.base = 0x0;
    RadioScheme.highlight = 0xB000;
    RadioScheme.highlightLight = 0xD024;
    RadioScheme.shadow = 0xB0C3;
    RadioScheme.shadowDark = 0xE826;
    RadioScheme.foreground = 0xFFFF;
    RadioScheme.foregroundInactive = 0xD71C;
    RadioScheme.foregroundDisabled = 0x8410;
    RadioScheme.background = 0x0;
    RadioScheme.backgroundInactive = 0x0;
    RadioScheme.backgroundDisabled = 0x0;
    RadioScheme.text = 0xFFFF;
    RadioScheme.textHighlight = 0x1F;
    RadioScheme.textHighlightText = 0xFFFF;
    RadioScheme.textInactive = 0xD71C;
    RadioScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&MuteButtonScheme, GFX_COLOR_MODE_RGB_565);
    MuteButtonScheme.base = 0x0;
    MuteButtonScheme.highlight = 0x0;
    MuteButtonScheme.highlightLight = 0x0;
    MuteButtonScheme.shadow = 0x0;
    MuteButtonScheme.shadowDark = 0x0;
    MuteButtonScheme.foreground = 0x0;
    MuteButtonScheme.foregroundInactive = 0xD71C;
    MuteButtonScheme.foregroundDisabled = 0x8410;
    MuteButtonScheme.background = 0x0;
    MuteButtonScheme.backgroundInactive = 0x0;
    MuteButtonScheme.backgroundDisabled = 0x0;
    MuteButtonScheme.text = 0xFFFF;
    MuteButtonScheme.textHighlight = 0x1F;
    MuteButtonScheme.textHighlightText = 0xFFFF;
    MuteButtonScheme.textInactive = 0xD71C;
    MuteButtonScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&BackgroundScheme, GFX_COLOR_MODE_RGB_565);
    BackgroundScheme.base = 0x0;
    BackgroundScheme.highlight = 0xC67A;
    BackgroundScheme.highlightLight = 0xFFFF;
    BackgroundScheme.shadow = 0x8410;
    BackgroundScheme.shadowDark = 0x4208;
    BackgroundScheme.foreground = 0x0;
    BackgroundScheme.foregroundInactive = 0xD71C;
    BackgroundScheme.foregroundDisabled = 0x8410;
    BackgroundScheme.background = 0xFFFF;
    BackgroundScheme.backgroundInactive = 0xD71C;
    BackgroundScheme.backgroundDisabled = 0xC67A;
    BackgroundScheme.text = 0x0;
    BackgroundScheme.textHighlight = 0x1F;
    BackgroundScheme.textHighlightText = 0xFFFF;
    BackgroundScheme.textInactive = 0xD71C;
    BackgroundScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&TrackListScheme, GFX_COLOR_MODE_RGB_565);
    TrackListScheme.base = 0x0;
    TrackListScheme.highlight = 0xB800;
    TrackListScheme.highlightLight = 0xB800;
    TrackListScheme.shadow = 0xF800;
    TrackListScheme.shadowDark = 0xB000;
    TrackListScheme.foreground = 0x0;
    TrackListScheme.foregroundInactive = 0xD71C;
    TrackListScheme.foregroundDisabled = 0x8410;
    TrackListScheme.background = 0x0;
    TrackListScheme.backgroundInactive = 0x0;
    TrackListScheme.backgroundDisabled = 0x0;
    TrackListScheme.text = 0xFFFF;
    TrackListScheme.textHighlight = 0xB000;
    TrackListScheme.textHighlightText = 0xFFFF;
    TrackListScheme.textInactive = 0xD71C;
    TrackListScheme.textDisabled = 0x8C92;

    laScheme_Initialize(&GradientScheme, GFX_COLOR_MODE_RGB_565);
    GradientScheme.base = 0x0;
    GradientScheme.highlight = 0xC67A;
    GradientScheme.highlightLight = 0xFFFF;
    GradientScheme.shadow = 0x8410;
    GradientScheme.shadowDark = 0x0;
    GradientScheme.foreground = 0xF800;
    GradientScheme.foregroundInactive = 0x0;
    GradientScheme.foregroundDisabled = 0x8410;
    GradientScheme.background = 0xF800;
    GradientScheme.backgroundInactive = 0xD71C;
    GradientScheme.backgroundDisabled = 0xC67A;
    GradientScheme.text = 0x0;
    GradientScheme.textHighlight = 0x1F;
    GradientScheme.textHighlightText = 0xFFFF;
    GradientScheme.textInactive = 0xD71C;
    GradientScheme.textDisabled = 0x8C92;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_MchpName);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(0);

	return 0;
}

static void ScreenCreate_MchpName(laScreen* screen)
{
    laLayer* layer0;
    laRadioButtonGroup* radioButtonGroup_1;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 480, 272);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laLayer_SetBufferCount(layer0, 1);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    Background = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)Background, 0, 1);
    laWidget_SetSize((laWidget*)Background, 480, 271);
    laWidget_SetScheme((laWidget*)Background, &BackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)Background, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)Background, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)Background);

    GradientWidget1 = laGradientWidget_New();
    laWidget_SetSize((laWidget*)GradientWidget1, 480, 48);
    laWidget_SetScheme((laWidget*)GradientWidget1, &GradientScheme);
    laWidget_SetBackgroundType((laWidget*)GradientWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GradientWidget1, LA_WIDGET_BORDER_NONE);
    laGradientWidget_SetDirection((laGradientWidget*)GradientWidget1, LA_GRADIENT_DIRECTION_DOWN);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GradientWidget1);

    GroupBoxWidget1 = laGroupBoxWidget_New();
    laWidget_SetPosition((laWidget*)GroupBoxWidget1, 32, 76);
    laWidget_SetSize((laWidget*)GroupBoxWidget1, 120, 159);
    laWidget_SetScheme((laWidget*)GroupBoxWidget1, &TrackListScheme);
    laWidget_SetBackgroundType((laWidget*)GroupBoxWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GroupBoxWidget1, LA_WIDGET_BORDER_NONE);
    laGroupBoxWidget_SetText(GroupBoxWidget1, laString_CreateFromID(string_group_box_str));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GroupBoxWidget1);

    DemoName = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)DemoName, 31, 51);
    laWidget_SetSize((laWidget*)DemoName, 219, 25);
    laWidget_SetScheme((laWidget*)DemoName, &RadioScheme);
    laWidget_SetBackgroundType((laWidget*)DemoName, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)DemoName, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(DemoName, laString_CreateFromID(string_demo_title_str));
    laLabelWidget_SetHAlignment(DemoName, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)DemoName);

    RadioButtonSd_InvisibleTouchArea = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)RadioButtonSd_InvisibleTouchArea, 40, 114);
    laWidget_SetSize((laWidget*)RadioButtonSd_InvisibleTouchArea, 100, 25);
    laWidget_SetScheme((laWidget*)RadioButtonSd_InvisibleTouchArea, &BackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)RadioButtonSd_InvisibleTouchArea, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)RadioButtonSd_InvisibleTouchArea, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(RadioButtonSd_InvisibleTouchArea, &RadioButtonSd_InvisibleTouchArea_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)RadioButtonSd_InvisibleTouchArea);

    RadioButtonSd = laRadioButtonWidget_New();
    laWidget_SetPosition((laWidget*)RadioButtonSd, 43, 111);
    laWidget_SetSize((laWidget*)RadioButtonSd, 101, 35);
    laWidget_SetEnabled((laWidget*)RadioButtonSd, LA_FALSE);
    laWidget_SetScheme((laWidget*)RadioButtonSd, &RadioScheme);
    laWidget_SetBackgroundType((laWidget*)RadioButtonSd, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)RadioButtonSd, LA_WIDGET_BORDER_NONE);
    laRadioButtonWidget_SetText(RadioButtonSd, laString_CreateFromID(string_sd_radio_button_str));
    laRadioButtonWidget_SetHAlignment(RadioButtonSd, LA_HALIGN_LEFT);
    laRadioButtonGroup_Create(&radioButtonGroup_1);
    laRadioButtonGroup_AddButton(radioButtonGroup_1, RadioButtonSd);
    laRadioButtonWidget_SetSelected(RadioButtonSd);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)RadioButtonSd);

    RadioButtonUSB_InvisibleTouchArea = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)RadioButtonUSB_InvisibleTouchArea, 42, 176);
    laWidget_SetSize((laWidget*)RadioButtonUSB_InvisibleTouchArea, 100, 25);
    laWidget_SetScheme((laWidget*)RadioButtonUSB_InvisibleTouchArea, &BackgroundScheme);
    laWidget_SetBackgroundType((laWidget*)RadioButtonUSB_InvisibleTouchArea, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)RadioButtonUSB_InvisibleTouchArea, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetReleasedEventCallback(RadioButtonUSB_InvisibleTouchArea, &RadioButtonUSB_InvisibleTouchArea_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)RadioButtonUSB_InvisibleTouchArea);

    RadioButtonUSB = laRadioButtonWidget_New();
    laWidget_SetPosition((laWidget*)RadioButtonUSB, 43, 172);
    laWidget_SetSize((laWidget*)RadioButtonUSB, 99, 35);
    laWidget_SetEnabled((laWidget*)RadioButtonUSB, LA_FALSE);
    laWidget_SetScheme((laWidget*)RadioButtonUSB, &RadioScheme);
    laWidget_SetBackgroundType((laWidget*)RadioButtonUSB, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)RadioButtonUSB, LA_WIDGET_BORDER_NONE);
    laRadioButtonWidget_SetText(RadioButtonUSB, laString_CreateFromID(string_usb_radio_button_str));
    laRadioButtonWidget_SetHAlignment(RadioButtonUSB, LA_HALIGN_LEFT);
    laRadioButtonGroup_AddButton(radioButtonGroup_1, RadioButtonUSB);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)RadioButtonUSB);

    TrackListBox = laListWidget_New();
    laWidget_SetPosition((laWidget*)TrackListBox, 172, 82);
    laWidget_SetSize((laWidget*)TrackListBox, 192, 151);
    laWidget_SetScheme((laWidget*)TrackListBox, &TrackListScheme);
    laWidget_SetBackgroundType((laWidget*)TrackListBox, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TrackListBox, LA_WIDGET_BORDER_BEVEL);
    laListWidget_SetSelectionMode(TrackListBox, LA_LIST_WIDGET_SELECTION_MODE_SINGLE);
    laListWidget_SetAllowEmptySelection(TrackListBox, LA_TRUE);
    laListWidget_SetSelectedItemChangedEventCallback(TrackListBox, &TrackListBox_SelectionChangedEvent);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)TrackListBox);

    MuteButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)MuteButton, 406, 237);
    laWidget_SetSize((laWidget*)MuteButton, 42, 33);
    laWidget_SetScheme((laWidget*)MuteButton, &MuteButtonScheme);
    laWidget_SetBackgroundType((laWidget*)MuteButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)MuteButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetToggleable(MuteButton, LA_TRUE);
    laButtonWidget_SetPressed(MuteButton, LA_TRUE);
    laButtonWidget_SetPressedImage(MuteButton, &AudioPlay16_2);
    laButtonWidget_SetReleasedImage(MuteButton, &AudioMute16_2);
    laButtonWidget_SetPressedEventCallback(MuteButton, &MuteButton_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(MuteButton, &MuteButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)MuteButton);

    MicrochipLogo = laImageWidget_New();
    laWidget_SetPosition((laWidget*)MicrochipLogo, 1, 2);
    laWidget_SetSize((laWidget*)MicrochipLogo, 132, 44);
    laWidget_SetBackgroundType((laWidget*)MicrochipLogo, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MicrochipLogo, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(MicrochipLogo, &MCHP_LOGO);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)MicrochipLogo);

    VolumeDownButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)VolumeDownButton, 413, 200);
    laWidget_SetSize((laWidget*)VolumeDownButton, 31, 31);
    laWidget_SetScheme((laWidget*)VolumeDownButton, &TrackListScheme);
    laWidget_SetBackgroundType((laWidget*)VolumeDownButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)VolumeDownButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(VolumeDownButton, laString_CreateFromID(string_volume_down_button_str));
    laButtonWidget_SetReleasedEventCallback(VolumeDownButton, &VolumeDownButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)VolumeDownButton);

    VolumeUpButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)VolumeUpButton, 413, 82);
    laWidget_SetSize((laWidget*)VolumeUpButton, 31, 31);
    laWidget_SetScheme((laWidget*)VolumeUpButton, &TrackListScheme);
    laWidget_SetBackgroundType((laWidget*)VolumeUpButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)VolumeUpButton, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(VolumeUpButton, laString_CreateFromID(string_volume_up_button_str));
    laButtonWidget_SetReleasedEventCallback(VolumeUpButton, &VolumeUpButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)VolumeUpButton);

    VolumeBar = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)VolumeBar, 413, 111);
    laWidget_SetSize((laWidget*)VolumeBar, 31, 89);
    laWidget_SetEnabled((laWidget*)VolumeBar, LA_FALSE);
    laWidget_SetScheme((laWidget*)VolumeBar, &TrackListScheme);
    laWidget_SetBackgroundType((laWidget*)VolumeBar, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)VolumeBar, LA_WIDGET_BORDER_BEVEL);
    laProgressBarWidget_SetDirection(VolumeBar, LA_PROGRESSBAR_DIRECTION_UP);
    laProgressBarWidget_SetValue(VolumeBar, 60);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)VolumeBar);

}



