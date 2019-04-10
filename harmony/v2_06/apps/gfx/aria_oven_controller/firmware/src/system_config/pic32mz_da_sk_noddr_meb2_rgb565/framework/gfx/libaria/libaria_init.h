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

#define LIBARIA_SCREEN_COUNT   5

// reference IDs for generated libaria screens
// screen "AssetLayout"
#define AssetLayout_ID    4

// screen "controllerScreen"
#define controllerScreen_ID    2

// screen "homeScreen"
#define homeScreen_ID    1

// screen "infoScreen"
#define infoScreen_ID    3

// screen "splashScreen"
#define splashScreen_ID    0



extern laScheme controllerTitle;
extern laScheme instructionScheme;
extern laScheme RedScheme;
extern laScheme helpScheme;
extern laScheme SettingsScheme;
extern laScheme GreenScheme;
extern laScheme infoScreen;
extern laScheme WhiteScheme;
extern laScheme YellowScheme;
extern laScheme clockScheme;
extern laImageWidget* ImageWidget1;
extern laImageWidget* ImageWidget2;
extern laWidget* PanelWidget1;
extern laImageWidget* ImageWidget3;
extern laImageWidget* ImageWidget4;
extern laButtonWidget* ButtonWidget3;
extern laLabelWidget* CenterClockLabel;
extern laImageWidget* ImageWidget6;
extern laButtonWidget* StartButton;
extern laLabelWidget* LabelWidget;
extern laLabelWidget* LabelWidget2;
extern laButtonWidget* ButtonWidget;
extern laButtonWidget* FishButtonWidget;
extern laButtonWidget* PizzaButtonWidget;
extern laButtonWidget* VegeButtonWidget;
extern laButtonWidget* StartStopButton;
extern laButtonWidget* DoneButton;
extern laCircularGaugeWidget* FishGaugeWidget;
extern laCircularGaugeWidget* PizzaGaugeWidget;
extern laCircularGaugeWidget* TurkeyGaugeWidget;
extern laLabelWidget* LabelWidget7;
extern laButtonWidget* ButtonWidget22;
extern laLabelWidget* LabelWidget9;
extern laLabelWidget* LabelWidget13;
extern laImageWidget* ImageWidget;
extern laImageWidget* Vegetables;
extern laImageWidget* Pizza;
extern laImageWidget* ImageWidget5;
extern laImageWidget* ImageWidget;
extern laImageWidget* ImageWidget7;
extern laImageWidget* ImageWidget11;
extern laLabelWidget* LabelWidget12;
extern laLabelWidget* LabelWidget8;
extern laLabelWidget* LabelWidget10;
extern laCircularGaugeWidget* CircularGaugeWidget3;
extern laLabelWidget* LabelWidget4;
extern laImageWidget* ImageWidget;
extern laImageWidget* ImageWidget;
extern laImageWidget* ImageWidget;
extern laImageWidget* ImageWidget;
extern laImageWidget* ImageWidget;
extern laImageWidget* ImageWidget;
extern laImageWidget* ImageWidget;
extern laImageWidget* ImageWidget8;
extern laImageWidget* ImageWidget9;
extern laImageWidget* ImageWidget10;
extern laImageWidget* ImageWidget12;
extern laImageWidget* ImageWidget13;


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
