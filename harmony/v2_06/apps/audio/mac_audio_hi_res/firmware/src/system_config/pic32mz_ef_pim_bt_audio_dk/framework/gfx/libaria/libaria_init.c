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

laScheme VolumeText;
laScheme VolumeProgressBar;
laScheme blueblackbackground;
laScheme FrequencyText;
laImageWidget* GFX_MICROCHIP_LOGO;
laWidget* GFX_MIDPANEL;
laImageWidget* GFX_AUDIO_MUTE;
laImageWidget* GFX_AUDIO_PLAY;
laWidget* GFX_USBPANEL;
laImageWidget* GFX_USB_CONNECTED;
laImageWidget* GFX_USB_DISCONNECTED;
laLabelWidget* GFX_TITLE;
laWidget* GFX_VOLUMEPANEL;
laLabelWidget* GFX_VOLUME_VALUE;
laImageWidget* GFX_FREQ;
laLabelWidget* GFX_FREQ_VALUE;
laProgressBarWidget* GFX_VOLUME_PBAR;


static void ScreenCreate_MainScreen(laScreen* screen);


int32_t libaria_initialize(void)
{
    laScreen* screen;

    laScheme_Initialize(&VolumeText, GFX_COLOR_MODE_RGB_565);
    VolumeText.base = 0x1967;
    VolumeText.highlight = 0xBDF7;
    VolumeText.highlightLight = 0xFFFF;
    VolumeText.shadow = 0x8410;
    VolumeText.shadowDark = 0x4208;
    VolumeText.foreground = 0x1967;
    VolumeText.foregroundInactive = 0xD71C;
    VolumeText.foregroundDisabled = 0xBDF7;
    VolumeText.background = 0x0;
    VolumeText.backgroundInactive = 0xD71C;
    VolumeText.backgroundDisabled = 0xC67A;
    VolumeText.text = 0xFC00;
    VolumeText.textHighlight = 0x1F;
    VolumeText.textHighlightText = 0xFFFF;
    VolumeText.textInactive = 0xFFFF;
    VolumeText.textDisabled = 0x4208;

    laScheme_Initialize(&VolumeProgressBar, GFX_COLOR_MODE_RGB_565);
    VolumeProgressBar.base = 0x13E1;
    VolumeProgressBar.highlight = 0x0;
    VolumeProgressBar.highlightLight = 0xFFFF;
    VolumeProgressBar.shadow = 0x8410;
    VolumeProgressBar.shadowDark = 0x40A2;
    VolumeProgressBar.foreground = 0x13E1;
    VolumeProgressBar.foregroundInactive = 0xD71C;
    VolumeProgressBar.foregroundDisabled = 0xBDF7;
    VolumeProgressBar.background = 0x8792;
    VolumeProgressBar.backgroundInactive = 0xD71C;
    VolumeProgressBar.backgroundDisabled = 0xC67A;
    VolumeProgressBar.text = 0x821;
    VolumeProgressBar.textHighlight = 0x13E1;
    VolumeProgressBar.textHighlightText = 0xFFFF;
    VolumeProgressBar.textInactive = 0xFFFF;
    VolumeProgressBar.textDisabled = 0x4208;

    laScheme_Initialize(&blueblackbackground, GFX_COLOR_MODE_RGB_565);
    blueblackbackground.base = 0x1967;
    blueblackbackground.highlight = 0xC67A;
    blueblackbackground.highlightLight = 0xFFFF;
    blueblackbackground.shadow = 0x8410;
    blueblackbackground.shadowDark = 0x4208;
    blueblackbackground.foreground = 0x1967;
    blueblackbackground.foregroundInactive = 0xD71C;
    blueblackbackground.foregroundDisabled = 0x8410;
    blueblackbackground.background = 0x1967;
    blueblackbackground.backgroundInactive = 0xD71C;
    blueblackbackground.backgroundDisabled = 0xC67A;
    blueblackbackground.text = 0xFFFF;
    blueblackbackground.textHighlight = 0x1F;
    blueblackbackground.textHighlightText = 0xFFFF;
    blueblackbackground.textInactive = 0xD71C;
    blueblackbackground.textDisabled = 0x8C92;

    laScheme_Initialize(&FrequencyText, GFX_COLOR_MODE_RGB_565);
    FrequencyText.base = 0x1967;
    FrequencyText.highlight = 0xBDF7;
    FrequencyText.highlightLight = 0xFFFF;
    FrequencyText.shadow = 0x8410;
    FrequencyText.shadowDark = 0x4208;
    FrequencyText.foreground = 0x1967;
    FrequencyText.foregroundInactive = 0xD71C;
    FrequencyText.foregroundDisabled = 0xBDF7;
    FrequencyText.background = 0x0;
    FrequencyText.backgroundInactive = 0xD71C;
    FrequencyText.backgroundDisabled = 0xC67A;
    FrequencyText.text = 0x7FDE;
    FrequencyText.textHighlight = 0x1F;
    FrequencyText.textHighlightText = 0xFFFF;
    FrequencyText.textInactive = 0xFFFF;
    FrequencyText.textDisabled = 0x4208;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_MainScreen);
    laScreen_SetOrientation(screen, LA_SCREEN_ORIENTATION_270);
    laContext_AddScreen(screen);

    laContext_SetPreemptionLevel(LA_PREEMPTION_LEVEL_2);
    laContext_SetActiveScreen(0);

	return 0;
}

static void ScreenCreate_MainScreen(laScreen* screen)
{
    laLayer* layer0;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 220, 176);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &blueblackbackground);
    laLayer_SetBufferCount(layer0, 1);
    laLayer_SetVSync(layer0, LA_FALSE);

    laScreen_SetLayer(screen, 0, layer0);

    GFX_MICROCHIP_LOGO = laImageWidget_New();
    laWidget_SetSize((laWidget*)GFX_MICROCHIP_LOGO, 122, 30);
    laWidget_SetScheme((laWidget*)GFX_MICROCHIP_LOGO, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_MICROCHIP_LOGO, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_MICROCHIP_LOGO, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_MICROCHIP_LOGO, &MCHP_LOGO);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_MICROCHIP_LOGO);

    GFX_MIDPANEL = laWidget_New();
    laWidget_SetPosition((laWidget*)GFX_MIDPANEL, 95, 142);
    laWidget_SetSize((laWidget*)GFX_MIDPANEL, 32, 25);
    laWidget_SetScheme((laWidget*)GFX_MIDPANEL, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_MIDPANEL, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_MIDPANEL, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, GFX_MIDPANEL);

    GFX_AUDIO_MUTE = laImageWidget_New();
    laWidget_SetPosition((laWidget*)GFX_AUDIO_MUTE, -1, -1);
    laWidget_SetSize((laWidget*)GFX_AUDIO_MUTE, 32, 25);
    laWidget_SetScheme((laWidget*)GFX_AUDIO_MUTE, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_AUDIO_MUTE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_AUDIO_MUTE, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)GFX_AUDIO_MUTE, 0, 0, 0, 0);
    laImageWidget_SetImage(GFX_AUDIO_MUTE, &AudioMute16_2);
    laWidget_AddChild((laWidget*)GFX_MIDPANEL, (laWidget*)GFX_AUDIO_MUTE);

    GFX_AUDIO_PLAY = laImageWidget_New();
    laWidget_SetPosition((laWidget*)GFX_AUDIO_PLAY, -1, -1);
    laWidget_SetSize((laWidget*)GFX_AUDIO_PLAY, 32, 25);
    laWidget_SetScheme((laWidget*)GFX_AUDIO_PLAY, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_AUDIO_PLAY, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_AUDIO_PLAY, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)GFX_AUDIO_PLAY, 0, 0, 0, 0);
    laImageWidget_SetImage(GFX_AUDIO_PLAY, &AudioPlay16_2);
    laWidget_AddChild((laWidget*)GFX_MIDPANEL, (laWidget*)GFX_AUDIO_PLAY);

    GFX_USBPANEL = laWidget_New();
    laWidget_SetPosition((laWidget*)GFX_USBPANEL, 198, 2);
    laWidget_SetSize((laWidget*)GFX_USBPANEL, 22, 25);
    laWidget_SetBackgroundType((laWidget*)GFX_USBPANEL, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_USBPANEL, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, GFX_USBPANEL);

    GFX_USB_CONNECTED = laImageWidget_New();
    laWidget_SetSize((laWidget*)GFX_USB_CONNECTED, 22, 25);
    laWidget_SetVisible((laWidget*)GFX_USB_CONNECTED, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_USB_CONNECTED, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_USB_CONNECTED, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_USB_CONNECTED, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)GFX_USB_CONNECTED, 0, 0, 0, 0);
    laImageWidget_SetImage(GFX_USB_CONNECTED, &USB_CONNECTED);
    laWidget_AddChild((laWidget*)GFX_USBPANEL, (laWidget*)GFX_USB_CONNECTED);

    GFX_USB_DISCONNECTED = laImageWidget_New();
    laWidget_SetSize((laWidget*)GFX_USB_DISCONNECTED, 22, 25);
    laWidget_SetVisible((laWidget*)GFX_USB_DISCONNECTED, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_USB_DISCONNECTED, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_USB_DISCONNECTED, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_USB_DISCONNECTED, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)GFX_USB_DISCONNECTED, 0, 0, 0, 0);
    laImageWidget_SetImage(GFX_USB_DISCONNECTED, &USB_DISCONNECTED);
    laWidget_AddChild((laWidget*)GFX_USBPANEL, (laWidget*)GFX_USB_DISCONNECTED);

    GFX_TITLE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_TITLE, 1, 29);
    laWidget_SetSize((laWidget*)GFX_TITLE, 195, 25);
    laWidget_SetScheme((laWidget*)GFX_TITLE, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_TITLE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_TITLE, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(GFX_TITLE, laString_CreateFromID(string_Title));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_TITLE);

    GFX_VOLUMEPANEL = laWidget_New();
    laWidget_SetPosition((laWidget*)GFX_VOLUMEPANEL, 187, 75);
    laWidget_SetSize((laWidget*)GFX_VOLUMEPANEL, 31, 22);
    laWidget_SetScheme((laWidget*)GFX_VOLUMEPANEL, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_VOLUMEPANEL, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_VOLUMEPANEL, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, GFX_VOLUMEPANEL);

    GFX_VOLUME_VALUE = laLabelWidget_New();
    laWidget_SetSize((laWidget*)GFX_VOLUME_VALUE, 31, 22);
    laWidget_SetScheme((laWidget*)GFX_VOLUME_VALUE, &VolumeText);
    laWidget_SetBackgroundType((laWidget*)GFX_VOLUME_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_VOLUME_VALUE, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)GFX_VOLUME_VALUE, 0, 0, 0, 0);
    laLabelWidget_SetHAlignment(GFX_VOLUME_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)GFX_VOLUMEPANEL, (laWidget*)GFX_VOLUME_VALUE);

    GFX_FREQ = laImageWidget_New();
    laWidget_SetPosition((laWidget*)GFX_FREQ, 74, 75);
    laWidget_SetSize((laWidget*)GFX_FREQ, 16, 21);
    laWidget_SetScheme((laWidget*)GFX_FREQ, &blueblackbackground);
    laWidget_SetBackgroundType((laWidget*)GFX_FREQ, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_FREQ, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(GFX_FREQ, &Frequency);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_FREQ);

    GFX_FREQ_VALUE = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)GFX_FREQ_VALUE, 96, 75);
    laWidget_SetSize((laWidget*)GFX_FREQ_VALUE, 49, 22);
    laWidget_SetScheme((laWidget*)GFX_FREQ_VALUE, &FrequencyText);
    laWidget_SetBackgroundType((laWidget*)GFX_FREQ_VALUE, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_FREQ_VALUE, LA_WIDGET_BORDER_NONE);
    laWidget_SetMargins((laWidget*)GFX_FREQ_VALUE, 0, 0, 0, 0);
    laLabelWidget_SetHAlignment(GFX_FREQ_VALUE, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_FREQ_VALUE);

    GFX_VOLUME_PBAR = laProgressBarWidget_New();
    laWidget_SetPosition((laWidget*)GFX_VOLUME_PBAR, 73, 100);
    laWidget_SetSize((laWidget*)GFX_VOLUME_PBAR, 88, 24);
    laWidget_SetEnabled((laWidget*)GFX_VOLUME_PBAR, LA_FALSE);
    laWidget_SetVisible((laWidget*)GFX_VOLUME_PBAR, LA_FALSE);
    laWidget_SetScheme((laWidget*)GFX_VOLUME_PBAR, &VolumeProgressBar);
    laWidget_SetBackgroundType((laWidget*)GFX_VOLUME_PBAR, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GFX_VOLUME_PBAR, LA_WIDGET_BORDER_BEVEL);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GFX_VOLUME_PBAR);

}



