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
laWidget* PanelSettings;
laGroupBoxWidget* GroupBoxWidget5;
laRadioButtonWidget* RadioButtonWidget6;
laRadioButtonWidget* RadioButtonWidget7;
laButtonWidget* ButtonWidget8;
laWidget* PanelMain;
laGroupBoxWidget* CanRxMessageGB;
laLabelWidget* CAN_MESSAGE_1;
laLabelWidget* CAN_MESSAGE_2;
laLabelWidget* CAN_MESSAGE_3;
laLabelWidget* CAN_MESSAGE_5;
laLabelWidget* CAN_MESSAGE_4;
laLabelWidget* CAN_MESSAGE_6;
laLabelWidget* CAN_MESSAGE_7;
laLabelWidget* CAN_MESSAGE_8;
laLabelWidget* CAN_Address;
laLabelWidget* CAN_DLC;
laLabelWidget* LabelWidget14;
laLabelWidget* LabelWidget12;
laLabelWidget* LabelWidget11;
laLabelWidget* LabelWidget10;
laLabelWidget* LabelWidget13;
laLabelWidget* LabelWidget15;
laLabelWidget* LabelWidget16;
laLabelWidget* LabelWidget19;
laLabelWidget* LabelWidget18;
laLabelWidget* LabelWidget17;
laButtonWidget* TxMessage1;
laGroupBoxWidget* TxGroupBox;
laLabelWidget* TX_DATA_1;
laLabelWidget* TX_DATA_2;
laLabelWidget* TX_DATA_3;
laLabelWidget* TX_DATA_5;
laLabelWidget* TX_DATA_4;
laLabelWidget* TX_DATA_6;
laLabelWidget* TX_DATA_7;
laLabelWidget* TX_DATA_8;
laLabelWidget* CAN_TX_ID;
laLabelWidget* CAN_TX_DLC;
laLabelWidget* LabelWidget21;
laLabelWidget* LabelWidget22;
laLabelWidget* lblTxDlc;
laLabelWidget* lblTxId;
laLabelWidget* LabelWidget25;
laLabelWidget* LabelWidget26;
laLabelWidget* LabelWidget27;
laLabelWidget* LabelWidget28;
laLabelWidget* LabelWidget29;
laLabelWidget* LabelWidget30;
laButtonWidget* TxMessage2;
laGradientWidget* GradientWidget2;
laImageWidget* ImageWidget1;
laLabelWidget* LabelWidget2;
laLabelWidget* VersionNumberlbl;
laButtonWidget* SettingsButton;
laButtonWidget* MainMenu;


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
    defaultScheme.foregroundInactive = 0xF800;
    defaultScheme.foregroundDisabled = 0x8410;
    defaultScheme.background = 0x0;
    defaultScheme.backgroundInactive = 0xD71C;
    defaultScheme.backgroundDisabled = 0xC67A;
    defaultScheme.text = 0xFFFF;
    defaultScheme.textHighlight = 0x1F;
    defaultScheme.textHighlightText = 0xFFFF;
    defaultScheme.textInactive = 0xD71C;
    defaultScheme.textDisabled = 0x8C92;

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
    laRadioButtonGroup* radioButtonGroup_1;

    layer0 = laLayer_New();
    laWidget_SetPosition((laWidget*)layer0, 0, 0);
    laWidget_SetSize((laWidget*)layer0, 800, 480);
    laWidget_SetBackgroundType((laWidget*)layer0, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetScheme((laWidget*)layer0, &defaultScheme);
    laLayer_SetBufferCount(layer0, 1);

    laScreen_SetLayer(screen, 0, layer0);

    PanelSettings = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelSettings, 0, 160);
    laWidget_SetSize((laWidget*)PanelSettings, 800, 320);
    laWidget_SetBackgroundType((laWidget*)PanelSettings, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelSettings, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelSettings);

    GroupBoxWidget5 = laGroupBoxWidget_New();
    laWidget_SetPosition((laWidget*)GroupBoxWidget5, 367, 55);
    laWidget_SetSize((laWidget*)GroupBoxWidget5, 404, 100);
    laWidget_SetBackgroundType((laWidget*)GroupBoxWidget5, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GroupBoxWidget5, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)PanelSettings, (laWidget*)GroupBoxWidget5);

    RadioButtonWidget6 = laRadioButtonWidget_New();
    laWidget_SetPosition((laWidget*)RadioButtonWidget6, 21, 14);
    laWidget_SetSize((laWidget*)RadioButtonWidget6, 100, 25);
    laWidget_SetBackgroundType((laWidget*)RadioButtonWidget6, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)RadioButtonWidget6, LA_WIDGET_BORDER_NONE);
    laRadioButtonWidget_SetHAlignment(RadioButtonWidget6, LA_HALIGN_LEFT);
    laRadioButtonGroup_Create(&radioButtonGroup_1);
    laRadioButtonGroup_AddButton(radioButtonGroup_1, RadioButtonWidget6);
    laRadioButtonWidget_SetSelected(RadioButtonWidget6);
    laWidget_AddChild((laWidget*)GroupBoxWidget5, (laWidget*)RadioButtonWidget6);

    RadioButtonWidget7 = laRadioButtonWidget_New();
    laWidget_SetPosition((laWidget*)RadioButtonWidget7, 21, 52);
    laWidget_SetSize((laWidget*)RadioButtonWidget7, 100, 25);
    laWidget_SetBackgroundType((laWidget*)RadioButtonWidget7, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)RadioButtonWidget7, LA_WIDGET_BORDER_NONE);
    laRadioButtonWidget_SetHAlignment(RadioButtonWidget7, LA_HALIGN_LEFT);
    laRadioButtonGroup_AddButton(radioButtonGroup_1, RadioButtonWidget7);
    laWidget_AddChild((laWidget*)GroupBoxWidget5, (laWidget*)RadioButtonWidget7);

    ButtonWidget8 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)ButtonWidget8, 368, 180);
    laWidget_SetSize((laWidget*)ButtonWidget8, 140, 87);
    laWidget_SetBackgroundType((laWidget*)ButtonWidget8, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)ButtonWidget8, LA_WIDGET_BORDER_BEVEL);
    laWidget_AddChild((laWidget*)PanelSettings, (laWidget*)ButtonWidget8);

    PanelMain = laWidget_New();
    laWidget_SetPosition((laWidget*)PanelMain, 0, 140);
    laWidget_SetSize((laWidget*)PanelMain, 800, 340);
    laWidget_SetBackgroundType((laWidget*)PanelMain, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)PanelMain, LA_WIDGET_BORDER_NONE);
    laWidget_AddChild((laWidget*)layer0, PanelMain);

    CanRxMessageGB = laGroupBoxWidget_New();
    laWidget_SetPosition((laWidget*)CanRxMessageGB, 14, 196);
    laWidget_SetSize((laWidget*)CanRxMessageGB, 780, 98);
    laWidget_SetBackgroundType((laWidget*)CanRxMessageGB, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CanRxMessageGB, LA_WIDGET_BORDER_NONE);
    laGroupBoxWidget_SetText(CanRxMessageGB, laString_CreateFromID(string_CanRxGroupBox));
    laWidget_AddChild((laWidget*)PanelMain, (laWidget*)CanRxMessageGB);

    CAN_MESSAGE_1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CAN_MESSAGE_1, 197, 47);
    laWidget_SetSize((laWidget*)CAN_MESSAGE_1, 60, 24);
    laWidget_SetBackgroundType((laWidget*)CAN_MESSAGE_1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CAN_MESSAGE_1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CAN_MESSAGE_1, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(CAN_MESSAGE_1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)CAN_MESSAGE_1);

    CAN_MESSAGE_2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CAN_MESSAGE_2, 261, 48);
    laWidget_SetSize((laWidget*)CAN_MESSAGE_2, 60, 21);
    laWidget_SetBackgroundType((laWidget*)CAN_MESSAGE_2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CAN_MESSAGE_2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CAN_MESSAGE_2, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(CAN_MESSAGE_2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)CAN_MESSAGE_2);

    CAN_MESSAGE_3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CAN_MESSAGE_3, 324, 46);
    laWidget_SetSize((laWidget*)CAN_MESSAGE_3, 60, 22);
    laWidget_SetBackgroundType((laWidget*)CAN_MESSAGE_3, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CAN_MESSAGE_3, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CAN_MESSAGE_3, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(CAN_MESSAGE_3, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)CAN_MESSAGE_3);

    CAN_MESSAGE_5 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CAN_MESSAGE_5, 454, 49);
    laWidget_SetSize((laWidget*)CAN_MESSAGE_5, 41, 23);
    laWidget_SetBackgroundType((laWidget*)CAN_MESSAGE_5, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CAN_MESSAGE_5, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CAN_MESSAGE_5, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(CAN_MESSAGE_5, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)CAN_MESSAGE_5);

    CAN_MESSAGE_4 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CAN_MESSAGE_4, 385, 48);
    laWidget_SetSize((laWidget*)CAN_MESSAGE_4, 38, 22);
    laWidget_SetBackgroundType((laWidget*)CAN_MESSAGE_4, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CAN_MESSAGE_4, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CAN_MESSAGE_4, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(CAN_MESSAGE_4, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)CAN_MESSAGE_4);

    CAN_MESSAGE_6 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CAN_MESSAGE_6, 516, 50);
    laWidget_SetSize((laWidget*)CAN_MESSAGE_6, 39, 21);
    laWidget_SetBackgroundType((laWidget*)CAN_MESSAGE_6, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CAN_MESSAGE_6, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CAN_MESSAGE_6, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(CAN_MESSAGE_6, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)CAN_MESSAGE_6);

    CAN_MESSAGE_7 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CAN_MESSAGE_7, 588, 51);
    laWidget_SetSize((laWidget*)CAN_MESSAGE_7, 45, 18);
    laWidget_SetBackgroundType((laWidget*)CAN_MESSAGE_7, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CAN_MESSAGE_7, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CAN_MESSAGE_7, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(CAN_MESSAGE_7, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)CAN_MESSAGE_7);

    CAN_MESSAGE_8 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CAN_MESSAGE_8, 665, 51);
    laWidget_SetSize((laWidget*)CAN_MESSAGE_8, 60, 25);
    laWidget_SetBackgroundType((laWidget*)CAN_MESSAGE_8, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CAN_MESSAGE_8, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CAN_MESSAGE_8, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(CAN_MESSAGE_8, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)CAN_MESSAGE_8);

    CAN_Address = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CAN_Address, 24, 46);
    laWidget_SetSize((laWidget*)CAN_Address, 74, 26);
    laWidget_SetBackgroundType((laWidget*)CAN_Address, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CAN_Address, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CAN_Address, laString_CreateFromID(string_CAN_Address));
    laLabelWidget_SetHAlignment(CAN_Address, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)CAN_Address);

    CAN_DLC = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CAN_DLC, 120, 46);
    laWidget_SetSize((laWidget*)CAN_DLC, 46, 25);
    laWidget_SetBackgroundType((laWidget*)CAN_DLC, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CAN_DLC, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CAN_DLC, laString_CreateFromID(string_CAN_DLC));
    laLabelWidget_SetHAlignment(CAN_DLC, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)CAN_DLC);

    LabelWidget14 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget14, 320, 25);
    laWidget_SetSize((laWidget*)LabelWidget14, 60, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget14, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget14, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget14, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget14, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)LabelWidget14);

    LabelWidget12 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget12, 197, 25);
    laWidget_SetSize((laWidget*)LabelWidget12, 61, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget12, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget12, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget12, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget12, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)LabelWidget12);

    LabelWidget11 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget11, 120, 25);
    laWidget_SetSize((laWidget*)LabelWidget11, 60, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget11, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget11, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget11, laString_CreateFromID(string_String_DLC));
    laLabelWidget_SetHAlignment(LabelWidget11, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)LabelWidget11);

    LabelWidget10 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget10, 24, 25);
    laWidget_SetSize((laWidget*)LabelWidget10, 60, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget10, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget10, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget10, laString_CreateFromID(string_String_ID));
    laLabelWidget_SetHAlignment(LabelWidget10, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)LabelWidget10);

    LabelWidget13 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget13, 259, 25);
    laWidget_SetSize((laWidget*)LabelWidget13, 61, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget13, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget13, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget13, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget13, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)LabelWidget13);

    LabelWidget15 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget15, 382, 25);
    laWidget_SetSize((laWidget*)LabelWidget15, 60, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget15, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget15, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget15, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget15, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)LabelWidget15);

    LabelWidget16 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget16, 448, 25);
    laWidget_SetSize((laWidget*)LabelWidget16, 60, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget16, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget16, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget16, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget16, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)LabelWidget16);

    LabelWidget19 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget19, 663, 25);
    laWidget_SetSize((laWidget*)LabelWidget19, 60, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget19, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget19, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget19, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget19, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)LabelWidget19);

    LabelWidget18 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget18, 585, 25);
    laWidget_SetSize((laWidget*)LabelWidget18, 60, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget18, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget18, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget18, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget18, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)LabelWidget18);

    LabelWidget17 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget17, 513, 25);
    laWidget_SetSize((laWidget*)LabelWidget17, 60, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget17, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget17, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget17, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget17, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)CanRxMessageGB, (laWidget*)LabelWidget17);

    TxMessage1 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)TxMessage1, 16, 12);
    laWidget_SetSize((laWidget*)TxMessage1, 365, 65);
    laWidget_SetBackgroundType((laWidget*)TxMessage1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TxMessage1, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(TxMessage1, laString_CreateFromID(string_Message1));
    laButtonWidget_SetReleasedEventCallback(TxMessage1, &TxMessage1_ReleasedEvent);

    laWidget_AddChild((laWidget*)PanelMain, (laWidget*)TxMessage1);

    TxGroupBox = laGroupBoxWidget_New();
    laWidget_SetPosition((laWidget*)TxGroupBox, 14, 88);
    laWidget_SetSize((laWidget*)TxGroupBox, 778, 85);
    laWidget_SetBackgroundType((laWidget*)TxGroupBox, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TxGroupBox, LA_WIDGET_BORDER_NONE);
    laGroupBoxWidget_SetText(TxGroupBox, laString_CreateFromID(string_TX_GROUP_LABEL));
    laWidget_AddChild((laWidget*)PanelMain, (laWidget*)TxGroupBox);

    TX_DATA_1 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TX_DATA_1, 199, 47);
    laWidget_SetSize((laWidget*)TX_DATA_1, 60, 25);
    laWidget_SetBackgroundType((laWidget*)TX_DATA_1, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TX_DATA_1, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TX_DATA_1, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(TX_DATA_1, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)TX_DATA_1);

    TX_DATA_2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TX_DATA_2, 257, 47);
    laWidget_SetSize((laWidget*)TX_DATA_2, 60, 25);
    laWidget_SetBackgroundType((laWidget*)TX_DATA_2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TX_DATA_2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TX_DATA_2, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(TX_DATA_2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)TX_DATA_2);

    TX_DATA_3 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TX_DATA_3, 320, 47);
    laWidget_SetSize((laWidget*)TX_DATA_3, 60, 25);
    laWidget_SetBackgroundType((laWidget*)TX_DATA_3, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TX_DATA_3, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TX_DATA_3, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(TX_DATA_3, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)TX_DATA_3);

    TX_DATA_5 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TX_DATA_5, 450, 47);
    laWidget_SetSize((laWidget*)TX_DATA_5, 60, 25);
    laWidget_SetBackgroundType((laWidget*)TX_DATA_5, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TX_DATA_5, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TX_DATA_5, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(TX_DATA_5, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)TX_DATA_5);

    TX_DATA_4 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TX_DATA_4, 381, 47);
    laWidget_SetSize((laWidget*)TX_DATA_4, 60, 25);
    laWidget_SetBackgroundType((laWidget*)TX_DATA_4, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TX_DATA_4, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TX_DATA_4, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(TX_DATA_4, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)TX_DATA_4);

    TX_DATA_6 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TX_DATA_6, 512, 47);
    laWidget_SetSize((laWidget*)TX_DATA_6, 60, 25);
    laWidget_SetBackgroundType((laWidget*)TX_DATA_6, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TX_DATA_6, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TX_DATA_6, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(TX_DATA_6, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)TX_DATA_6);

    TX_DATA_7 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TX_DATA_7, 584, 47);
    laWidget_SetSize((laWidget*)TX_DATA_7, 60, 25);
    laWidget_SetBackgroundType((laWidget*)TX_DATA_7, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TX_DATA_7, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TX_DATA_7, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(TX_DATA_7, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)TX_DATA_7);

    TX_DATA_8 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)TX_DATA_8, 661, 47);
    laWidget_SetSize((laWidget*)TX_DATA_8, 60, 25);
    laWidget_SetBackgroundType((laWidget*)TX_DATA_8, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TX_DATA_8, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(TX_DATA_8, laString_CreateFromID(string_Def));
    laLabelWidget_SetHAlignment(TX_DATA_8, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)TX_DATA_8);

    CAN_TX_ID = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CAN_TX_ID, 24, 45);
    laWidget_SetSize((laWidget*)CAN_TX_ID, 70, 26);
    laWidget_SetBackgroundType((laWidget*)CAN_TX_ID, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CAN_TX_ID, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CAN_TX_ID, laString_CreateFromID(string_CAN_Address));
    laLabelWidget_SetHAlignment(CAN_TX_ID, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)CAN_TX_ID);

    CAN_TX_DLC = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)CAN_TX_DLC, 120, 45);
    laWidget_SetSize((laWidget*)CAN_TX_DLC, 46, 25);
    laWidget_SetBackgroundType((laWidget*)CAN_TX_DLC, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)CAN_TX_DLC, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(CAN_TX_DLC, laString_CreateFromID(string_CAN_DLC));
    laLabelWidget_SetHAlignment(CAN_TX_DLC, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)CAN_TX_DLC);

    LabelWidget21 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget21, 316, 24);
    laWidget_SetSize((laWidget*)LabelWidget21, 59, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget21, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget21, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget21, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget21, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)LabelWidget21);

    LabelWidget22 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget22, 193, 24);
    laWidget_SetSize((laWidget*)LabelWidget22, 60, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget22, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget22, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget22, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget22, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)LabelWidget22);

    lblTxDlc = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)lblTxDlc, 120, 24);
    laWidget_SetSize((laWidget*)lblTxDlc, 44, 25);
    laWidget_SetBackgroundType((laWidget*)lblTxDlc, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)lblTxDlc, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(lblTxDlc, laString_CreateFromID(string_String_DLC));
    laLabelWidget_SetHAlignment(lblTxDlc, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)lblTxDlc);

    lblTxId = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)lblTxId, 24, 24);
    laWidget_SetSize((laWidget*)lblTxId, 28, 25);
    laWidget_SetBackgroundType((laWidget*)lblTxId, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)lblTxId, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(lblTxId, laString_CreateFromID(string_String_ID));
    laLabelWidget_SetHAlignment(lblTxId, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)lblTxId);

    LabelWidget25 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget25, 255, 24);
    laWidget_SetSize((laWidget*)LabelWidget25, 58, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget25, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget25, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget25, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget25, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)LabelWidget25);

    LabelWidget26 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget26, 378, 24);
    laWidget_SetSize((laWidget*)LabelWidget26, 61, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget26, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget26, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget26, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget26, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)LabelWidget26);

    LabelWidget27 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget27, 444, 24);
    laWidget_SetSize((laWidget*)LabelWidget27, 61, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget27, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget27, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget27, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget27, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)LabelWidget27);

    LabelWidget28 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget28, 659, 24);
    laWidget_SetSize((laWidget*)LabelWidget28, 61, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget28, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget28, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget28, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget28, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)LabelWidget28);

    LabelWidget29 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget29, 581, 24);
    laWidget_SetSize((laWidget*)LabelWidget29, 59, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget29, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget29, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget29, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget29, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)LabelWidget29);

    LabelWidget30 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget30, 509, 24);
    laWidget_SetSize((laWidget*)LabelWidget30, 61, 25);
    laWidget_SetBackgroundType((laWidget*)LabelWidget30, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)LabelWidget30, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget30, laString_CreateFromID(string_String_1));
    laLabelWidget_SetHAlignment(LabelWidget30, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)TxGroupBox, (laWidget*)LabelWidget30);

    TxMessage2 = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)TxMessage2, 425, 12);
    laWidget_SetSize((laWidget*)TxMessage2, 365, 65);
    laWidget_SetBackgroundType((laWidget*)TxMessage2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)TxMessage2, LA_WIDGET_BORDER_BEVEL);
    laButtonWidget_SetText(TxMessage2, laString_CreateFromID(string_Message2));
    laButtonWidget_SetReleasedEventCallback(TxMessage2, &TxMessage2_ReleasedEvent);

    laWidget_AddChild((laWidget*)PanelMain, (laWidget*)TxMessage2);

    GradientWidget2 = laGradientWidget_New();
    laWidget_SetSize((laWidget*)GradientWidget2, 800, 50);
    laWidget_SetScheme((laWidget*)GradientWidget2, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)GradientWidget2, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)GradientWidget2, LA_WIDGET_BORDER_NONE);
    laGradientWidget_SetDirection((laGradientWidget*)GradientWidget2, LA_GRADIENT_DIRECTION_UP);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)GradientWidget2);

    ImageWidget1 = laImageWidget_New();
    laWidget_SetPosition((laWidget*)ImageWidget1, 1, 0);
    laWidget_SetSize((laWidget*)ImageWidget1, 128, 42);
    laWidget_SetBackgroundType((laWidget*)ImageWidget1, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)ImageWidget1, LA_WIDGET_BORDER_NONE);
    laImageWidget_SetImage(ImageWidget1, &MCHP_LOGO);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)ImageWidget1);

    LabelWidget2 = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)LabelWidget2, 137, 7);
    laWidget_SetSize((laWidget*)LabelWidget2, 205, 25);
    laWidget_SetScheme((laWidget*)LabelWidget2, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)LabelWidget2, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)LabelWidget2, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(LabelWidget2, laString_CreateFromID(string_Demo_Name));
    laLabelWidget_SetHAlignment(LabelWidget2, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)LabelWidget2);

    VersionNumberlbl = laLabelWidget_New();
    laWidget_SetPosition((laWidget*)VersionNumberlbl, 350, 5);
    laWidget_SetSize((laWidget*)VersionNumberlbl, 100, 25);
    laWidget_SetScheme((laWidget*)VersionNumberlbl, &defaultScheme);
    laWidget_SetBackgroundType((laWidget*)VersionNumberlbl, LA_WIDGET_BACKGROUND_NONE);
    laWidget_SetBorderType((laWidget*)VersionNumberlbl, LA_WIDGET_BORDER_NONE);
    laLabelWidget_SetText(VersionNumberlbl, laString_CreateFromID(string_Version_Number));
    laLabelWidget_SetHAlignment(VersionNumberlbl, LA_HALIGN_LEFT);
    laWidget_AddChild((laWidget*)layer0, (laWidget*)VersionNumberlbl);

    SettingsButton = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)SettingsButton, 626, 65);
    laWidget_SetSize((laWidget*)SettingsButton, 130, 65);
    laWidget_SetEnabled((laWidget*)SettingsButton, LA_FALSE);
    laWidget_SetVisible((laWidget*)SettingsButton, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)SettingsButton, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)SettingsButton, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetText(SettingsButton, laString_CreateFromID(string_SettingsB));
    laButtonWidget_SetReleasedEventCallback(SettingsButton, &SettingsButton_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)SettingsButton);

    MainMenu = laButtonWidget_New();
    laWidget_SetPosition((laWidget*)MainMenu, 10, 65);
    laWidget_SetSize((laWidget*)MainMenu, 130, 65);
    laWidget_SetEnabled((laWidget*)MainMenu, LA_FALSE);
    laWidget_SetVisible((laWidget*)MainMenu, LA_FALSE);
    laWidget_SetBackgroundType((laWidget*)MainMenu, LA_WIDGET_BACKGROUND_FILL);
    laWidget_SetBorderType((laWidget*)MainMenu, LA_WIDGET_BORDER_NONE);
    laButtonWidget_SetText(MainMenu, laString_CreateFromID(string_Main));
    laButtonWidget_SetReleasedEventCallback(MainMenu, &MainMenu_ReleasedEvent);

    laWidget_AddChild((laWidget*)layer0, (laWidget*)MainMenu);

}



