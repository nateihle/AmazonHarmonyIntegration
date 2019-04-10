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

#define LIBARIA_SCREEN_COUNT   6

// reference IDs for generated libaria screens
// screen "AlbumScreen"
#define AlbumScreen_ID    3

// screen "BusinessScreen"
#define BusinessScreen_ID    4

// screen "InfoScreen"
#define InfoScreen_ID    2

// screen "MainScreen"
#define MainScreen_ID    1

// screen "PortraitScreen"
#define PortraitScreen_ID    5

// screen "SplashScreen"
#define SplashScreen_ID    0



extern laScheme defaultScheme;
extern laScheme clearScheme;
extern laScheme labelModeScheme1;
extern laScheme titleLabelScheme;
extern laScheme labelModeScheme0;
extern laScheme labelModeScheme3;
extern laScheme GradientScheme;
extern laScheme labelModeScheme2;
extern laScheme blackScheme;
extern laImageWidget* ImageWidget1;
extern laImageWidget* ImageWidget3;
extern laImageWidget* ImageWidget2;
extern laImageWidget* ImageWidget4;
extern laImageWidget* ImageWidget;
extern laLabelWidget* LabelWidget3;
extern laRadialMenuWidget* RadialMenuWidget_Main;
extern laImagePlusWidget* RadialMenuWidget_Main_Item0;
extern laImagePlusWidget* RadialMenuWidget_Main_Item1;
extern laImagePlusWidget* RadialMenuWidget_Main_Item2;
extern laImagePlusWidget* RadialMenuWidget_Main_Item3;
extern laButtonWidget* InfoButton;
extern laLabelWidget* ModeLabel;
extern laImageWidget* InfoPageHarmonyLogo;
extern laImageWidget* ImageWidget7;
extern laImageWidget* ImageWidget;
extern laWidget* InfoTextDragPanel;
extern laImageWidget* InfoTextImage;
extern laWidget* PanelWidget1;
extern laLabelWidget* TextTitle;
extern laButtonWidget* ReturnToMainButton;
extern laButtonWidget* ButtonWidget;
extern laImageWidget* ImageWidget;
extern laLabelWidget* LabelWidget;
extern laButtonWidget* ButtonWidget4;
extern laRadialMenuWidget* RadialMenuWidget_Album;
extern laImagePlusWidget* RadialMenuWidget_Album_Item0;
extern laImagePlusWidget* RadialMenuWidget_Album_Item1;
extern laImagePlusWidget* RadialMenuWidget_Album_Item2;
extern laImagePlusWidget* RadialMenuWidget_Album_Item3;
extern laImagePlusWidget* RadialMenuWidget_Album_Item4;
extern laImagePlusWidget* RadialMenuWidget_Album_Item5;
extern laImagePlusWidget* RadialMenuWidget_Album_Item6;
extern laImagePlusWidget* RadialMenuWidget_Album_Item7;
extern laImagePlusWidget* RadialMenuWidget_Album_Item8;
extern laImagePlusWidget* RadialMenuWidget_Album_Item9;
extern laButtonWidget* ButtonWidget2;
extern laImageWidget* ImageWidget;
extern laLabelWidget* LabelWidget7;
extern laRadialMenuWidget* RadialMenuWidget_Business;
extern laImagePlusWidget* RadialMenuWidget_Business_Item0;
extern laImagePlusWidget* RadialMenuWidget_Business_Item1;
extern laImagePlusWidget* RadialMenuWidget_Business_Item2;
extern laImagePlusWidget* RadialMenuWidget_Business_Item3;
extern laImagePlusWidget* RadialMenuWidget_Business_Item4;
extern laImagePlusWidget* RadialMenuWidget_Business_Item5;
extern laButtonWidget* ButtonWidget8;
extern laButtonWidget* ButtonWidget6;
extern laLabelWidget* LabelWidget2;
extern laRadialMenuWidget* RadialMenuWidget4;
extern laImagePlusWidget* RadialMenuWidget4_Item0;
extern laImagePlusWidget* RadialMenuWidget4_Item1;
extern laImagePlusWidget* RadialMenuWidget4_Item2;
extern laImagePlusWidget* RadialMenuWidget4_Item3;
extern laImagePlusWidget* RadialMenuWidget4_Item4;
extern laImagePlusWidget* RadialMenuWidget4_Item5;
extern laImagePlusWidget* RadialMenuWidget4_Item6;
extern laButtonWidget* ButtonWidget3;
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
