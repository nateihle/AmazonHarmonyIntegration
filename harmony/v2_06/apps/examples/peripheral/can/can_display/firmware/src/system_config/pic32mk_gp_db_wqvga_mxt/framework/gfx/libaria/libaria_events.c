/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Implementation File

  File Name:
    libaria_events.c

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

#include "gfx/libaria/libaria_events.h"


//custom code- Do Not Delete (Start)
void TxScreenUpDate(int Message);
//custom code- Do Not Delete (End)

// TxMessage1 - ReleasedEvent
void TxMessage1_ReleasedEvent(laButtonWidget* btn)
{
    // Send Message
    TxScreenUpDate(1);
}

// TxMessage2 - ReleasedEvent
void TxMessage2_ReleasedEvent(laButtonWidget* btn)
{
    // SendMessage
    TxScreenUpDate(2);
}

// SettingsButton - ReleasedEvent
void SettingsButton_ReleasedEvent(laButtonWidget* btn)
{
    // HideMain - Set Visible - PanelMain
    laWidget_SetVisible((laWidget*)PanelMain, LA_FALSE);

    // ShowSettings - Set Visible - PanelSettings
    laWidget_SetVisible((laWidget*)PanelSettings, LA_TRUE);
}

// MainMenu - ReleasedEvent
void MainMenu_ReleasedEvent(laButtonWidget* btn)
{
    // HideSettings - Set Visible - PanelSettings
    laWidget_SetVisible((laWidget*)PanelSettings, LA_FALSE);

    // ShowMain - Set Visible - PanelMain
    laWidget_SetVisible((laWidget*)PanelMain, LA_TRUE);
}





