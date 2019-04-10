/*******************************************************************************
  MPLAB Harmony Graphics Composer Generated Definitions Header

  File Name:
    libaria_macros.h

  Summary:
    Build-time generated definitions header based on output by the MPLAB Harmony
    Graphics Composer.

  Description:
    Build-time generated definitions header based on output by the MPLAB Harmony
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

#ifndef _LIBARIA_INIT_H
#define _LIBARIA_INIT_H

#ifndef NATIVE
#include "system_config.h"
#include "system_definitions.h"
#endif

#include "gfx/libaria/libaria.h"
#include "gfx/libaria/libaria_events.h"

#include "gfx/gfx_assets.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

#define LIBARIA_SCREEN_COUNT   3

// reference IDs for generated libaria screens
// screen "InfoScreen"
#define InfoScreen_ID    2

// screen "MainScreen"
#define MainScreen_ID    1

// screen "SplashScreen"
#define SplashScreen_ID    0



extern laScheme defaultScheme;
extern laScheme trayScheme;
extern laScheme ButtonWhiteScheme;
extern laScheme ClearScheme;
extern laScheme WhiteScheme;
extern laImageWidget* ImageWidget1;
extern laImageWidget* ImageWidget3;
extern laImageWidget* ImageWidget2;
extern laImageWidget* ImageWidget4;
extern laImageWidget* BackgroundImage;
extern laLabelWidget* RoastLabel;
extern laLabelWidget* AppTitleLabel;
extern laWidget* DragPanelRight;
extern laListWheelWidget* SizeSelect;
extern laWidget* PanelWidget4;
extern laButtonWidget* BrewButton;
extern laWidget* PanelWidget5;
extern laWidget* PanelWidget6;
extern laWidget* PanelWidget7;
extern laButtonWidget* RightTrayLid;
extern laImageWidget* ImageWidget6;
extern laWidget* DragPanel;
extern laWidget* PanelWidget2;
extern laButtonWidget* ChangeLanguage;
extern laButtonWidget* CoffeeButton;
extern laButtonWidget* CoffeeBeanButton;
extern laButtonWidget* TeaButton;
extern laButtonWidget* InfoPageButton;
extern laButtonWidget* GPUButton;
extern laLabelWidget* GPUButtonLabel;
extern laButtonWidget* LeftTrayLid;
extern laImageWidget* ImageWidget5;
extern laImageWidget* InfoPageHarmonyLogo;
extern laImageWidget* ImageWidget7;
extern laImageWidget* ImageWidget;
extern laWidget* InfoTextDragPanel;
extern laImageWidget* InfoTextImage;
extern laWidget* PanelWidget1;
extern laLabelWidget* TextTitle;
extern laButtonWidget* ReturnToMainButton;
extern laButtonWidget* ButtonWidget1;


int32_t libaria_initialize(void);

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif // _LIBARIA_INIT_H
/*******************************************************************************
 End of File
*/
