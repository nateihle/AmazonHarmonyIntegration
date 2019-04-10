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
extern laScheme redScheme;
extern laScheme yellowScheme;
extern laScheme ClearScheme;
extern laScheme greenScheme;
extern laImageWidget* ImageWidget1;
extern laImageWidget* ImageWidget3;
extern laImageWidget* ImageWidget2;
extern laImageWidget* ImageWidget4;
extern laWidget* BackPanel;
extern laImageWidget* BackImage0;
extern laImageWidget* BackImage1;
extern laImageWidget* BackImage2;
extern laWidget* MiddlePanel;
extern laImageWidget* MiddleImage0;
extern laImageWidget* MiddleImage1;
extern laImageWidget* MiddleImage2;
extern laWidget* FrontPanel;
extern laImageWidget* FrontImage0;
extern laImageWidget* FrontImage1;
extern laImageWidget* FrontImage2;
extern laButtonWidget* MicrochipLogoWidget;
extern laCircularGaugeWidget* CircularGaugeWidget;
extern laWidget* SpriteAnchor;
extern laImageSequenceWidget* RunRightSequence;
extern laImageSequenceWidget* RunLeftSequence;
extern laImageSequenceWidget* IdleRightSequence;
extern laImageSequenceWidget* IdleLeftSequence;
extern laImageSequenceWidget* BlazeRightSequence;
extern laImageSequenceWidget* BlazeLeftSequence;
extern laImageSequenceWidget* DizzySequence;
extern laImageSequenceWidget* DizzyLeftSequence;
extern laImageSequenceWidget* HurtSequence;
extern laImageSequenceWidget* HurtLeftSequence;
extern laImageSequenceWidget* FallSequence;
extern laImageSequenceWidget* FallLeftSequence;
extern laWidget* TopScoreAnchor;
extern laLabelWidget* TopScoreLabel;
extern laLabelWidget* TopDigit5;
extern laLabelWidget* TopDigit4;
extern laLabelWidget* TopDigit3;
extern laLabelWidget* TopDigit2;
extern laLabelWidget* TopDigit1;
extern laLabelWidget* TopDigit0;
extern laWidget* ScoreAnchor;
extern laLabelWidget* ScoreLabel;
extern laLabelWidget* Digit5;
extern laLabelWidget* Digit4;
extern laLabelWidget* Digit3;
extern laLabelWidget* Digit2;
extern laLabelWidget* Digit1;
extern laLabelWidget* Digit0;
extern laWidget* GPUUsageAnchor;
extern laLabelWidget* GPUUsageLabel;
extern laLabelWidget* GPUScore6;
extern laLabelWidget* GPUScore5;
extern laLabelWidget* GPUScore4;
extern laLabelWidget* GPUScore3;
extern laLabelWidget* GPUScore2;
extern laLabelWidget* PixelLabel;
extern laLabelWidget* GPUScore1;
extern laLabelWidget* GPUScore0;
extern laButtonWidget* ButtonWidget1;
extern laButtonWidget* ButtonWidget0;
extern laImageWidget* InfoPageHarmonyLogo;
extern laImageWidget* TouchUpDownImage;
extern laImageWidget* ImageWidget;
extern laWidget* InfoTextDragPanel;
extern laImageWidget* InfoTextImage;
extern laWidget* InfoTitlePanelWidget;
extern laLabelWidget* TextTitle;
extern laButtonWidget* ReturnToMainButton;
extern laButtonWidget* EasterEggButton;


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
