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
laScheme LED_OFF;
laScheme LED_ON;
laLabelWidget* LabelWidget1;
laImageWidget* ImageWidget1;
laButtonWidget* ButtonWidget1;
laWidget* MEB2_Board;
laButtonWidget* Display_S1;
laTextFieldWidget* D6_Label;
laTextFieldWidget* D7_Label;
laRectangleWidget* MEB2_LED_D6;
laRectangleWidget* MEB2_LED_D7;


static void ScreenCreate_default(laScreen* screen);


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

    laScheme_Initialize(&LED_OFF, GFX_COLOR_MODE_RGB_565);
    LED_OFF.base = 0xC67A;
    LED_OFF.highlight = 0xC67A;
    LED_OFF.highlightLight = 0xFFFF;
    LED_OFF.shadow = 0x8410;
    LED_OFF.shadowDark = 0x4208;
    LED_OFF.foreground = 0x0;
    LED_OFF.foregroundInactive = 0xD71C;
    LED_OFF.foregroundDisabled = 0x8410;
    LED_OFF.background = 0xFFFF;
    LED_OFF.backgroundInactive = 0xD71C;
    LED_OFF.backgroundDisabled = 0xC67A;
    LED_OFF.text = 0x0;
    LED_OFF.textHighlight = 0x1F;
    LED_OFF.textHighlightText = 0xFFFF;
    LED_OFF.textInactive = 0xD71C;
    LED_OFF.textDisabled = 0x8C92;

    laScheme_Initialize(&LED_ON, GFX_COLOR_MODE_RGB_565);
    LED_ON.base = 0xF800;
    LED_ON.highlight = 0xC67A;
    LED_ON.highlightLight = 0xFFFF;
    LED_ON.shadow = 0x8410;
    LED_ON.shadowDark = 0x4208;
    LED_ON.foreground = 0x0;
    LED_ON.foregroundInactive = 0xD71C;
    LED_ON.foregroundDisabled = 0x8410;
    LED_ON.background = 0xFFFF;
    LED_ON.backgroundInactive = 0xD71C;
    LED_ON.backgroundDisabled = 0xC67A;
    LED_ON.text = 0x0;
    LED_ON.textHighlight = 0x1F;
    LED_ON.textHighlightText = 0xFFFF;
    LED_ON.textInactive = 0xD71C;
    LED_ON.textDisabled = 0x8C92;

    GFX_Set(GFXF_DRAW_PIPELINE_MODE, GFX_PIPELINE_GCUGPU);
    laContext_SetStringTable(&stringTable);

    screen = laScreen_New(LA_FALSE, LA_FALSE, &ScreenCreate_default);
    laContext_AddScreen(screen);

    laContext_SetActiveScreen(0);

	return 0;
}

static void ScreenCreate_default(laScreen* screen)
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
    laWidget_SetPosition((laWidget*)LabelWidget1, 146, 1);
    laWidget_SetSize((laWidget*)LabelWidget1, 179, 39);
    laWidget_SetBackgroundType((laWidget*)LabelWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget1, laString_CreateFromID(string_GFX_Quickstart));
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget1);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 113, 39);
    laWidget_SetSize((laWidget*)ImageWidget1, 242, 150);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &NewHarmonyLogo);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    ButtonWidget1 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget1, 101, 201);
    laWidget_SetSize((laWidget*)ButtonWidget1, 270, 40);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ButtonWidget1, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(ButtonWidget1, laString_CreateFromID(string_Instructions));
    laButtonWidget_SetPressedEventCallback(ButtonWidget1, &ButtonWidget1_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(ButtonWidget1, &ButtonWidget1_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)ButtonWidget1);

    MEB2_Board = laWidget_New();
    laWidget_SetPosition((laWidget*)MEB2_Board, 1, 0);
    laWidget_SetSize((laWidget*)MEB2_Board, 130, 108);
    laWidget_SetBackgroundType((laWidget*)MEB2_Board, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)MEB2_Board, LA_WIDGET_BORDER_LINE);
    laWidget_AddChild((laWidget*)layer0, MEB2_Board);

    Display_S1 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)Display_S1, 3, 3);
    laWidget_SetSize((laWidget*)Display_S1, 74, 100);
    laWidget_SetBackgroundType((laWidget*)Display_S1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)Display_S1, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(Display_S1, laString_CreateFromID(string_MEB2_S1));
    laButtonWidget_SetPressedEventCallback(Display_S1, &Display_S1_PressedEvent);
    laButtonWidget_SetReleasedEventCallback(Display_S1, &Display_S1_ReleasedEvent);

    laWidget_AddChild((laWidget*)MEB2_Board, (laWidget*)Display_S1);

    D6_Label = laTextFieldWidget_New();
    laWidget_SetPosition((laWidget*)D6_Label, 80, 37);
    laWidget_SetSize((laWidget*)D6_Label, 20, 14);
    laWidget_SetBackgroundType((laWidget*)D6_Label, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)D6_Label, LA_WIDGET_BORDER_NONE);
    laTextFieldWidget_SetText(D6_Label, laString_CreateFromID(string_MEB2_D6));
    laTextFieldWidget_SetCursorEnabled(D6_Label, LA_TRUE);
    laWidget_AddChild((laWidget*)MEB2_Board, (laWidget*)D6_Label);

    D7_Label = laTextFieldWidget_New();
    laWidget_SetPosition((laWidget*)D7_Label, 105, 38);
    laWidget_SetSize((laWidget*)D7_Label, 21, 12);
    laWidget_SetBackgroundType((laWidget*)D7_Label, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)D7_Label, LA_WIDGET_BORDER_NONE);
    laTextFieldWidget_SetText(D7_Label, laString_CreateFromID(string_MEB2_D7));
    laTextFieldWidget_SetCursorEnabled(D7_Label, LA_TRUE);
    laWidget_AddChild((laWidget*)MEB2_Board, (laWidget*)D7_Label);

    MEB2_LED_D6 = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)MEB2_LED_D6, 80, 5);
    laWidget_SetSize((laWidget*)MEB2_LED_D6, 20, 31);
    laWidget_SetScheme((laWidget*)MEB2_LED_D6, &LED_OFF);
    laWidget_SetBackgroundType((laWidget*)MEB2_LED_D6, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)MEB2_LED_D6, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)MEB2_Board, (laWidget*)MEB2_LED_D6);

    MEB2_LED_D7 = laRectangleWidget_New();
    laWidget_SetPosition((laWidget*)MEB2_LED_D7, 106, 5);
    laWidget_SetSize((laWidget*)MEB2_LED_D7, 20, 31);
    laWidget_SetScheme((laWidget*)MEB2_LED_D7, &LED_OFF);
    laWidget_SetBackgroundType((laWidget*)MEB2_LED_D7, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)MEB2_LED_D7, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)MEB2_Board, (laWidget*)MEB2_LED_D7);

}



