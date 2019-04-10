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
// screen "New_appScreen"
#define New_appScreen_ID    1

// screen "SplashScreen"
#define SplashScreen_ID    0

// screen "helpscreen"
#define helpscreen_ID    2



extern laScheme InstructionScheme;
extern laScheme whiteScheme;
extern laScheme ListWheelImage1;
extern laScheme ListWheelType;
extern laScheme Clearscheme;
extern laImageWidget* ImageWidget1;
extern laImageWidget* ImageWidget3;
extern laImageWidget* ImageWidget2;
extern laImageWidget* ImageWidget4;
extern laImageWidget* ImageWidget_newapp;
extern laWidget* PanelWidget_timebackgrnd;
extern laLabelWidget* Label_Usb_time;
extern laLabelWidget* Label_Int_time;
extern laLabelWidget* Label_Sqi_time;
extern laLabelWidget* LabelWidget1_IntTimeLabel;
extern laLabelWidget* LabelWidget_SqiTimeLabel;
extern laLabelWidget* LabelWidget_UsbTimeLabel;
extern laLabelWidget* label_ms_1;
extern laLabelWidget* label_ms_2;
extern laLabelWidget* label_ms_3;
extern laLabelWidget* LabelWidget_source;
extern laWidget* PanelWidget_loadtimeslides;
extern laLabelWidget* LabelWidget_load_time;
extern laLabelWidget* LabelWidget_time;
extern laLabelWidget* LabelWidget_ImageType;
extern laButtonWidget* ButtonUSB;
extern laButtonWidget* ButtonSQI;
extern laButtonWidget* ButtonIntMem;
extern laButtonWidget* ButtonInfo;
extern laButtonWidget* ButtonSlides;
extern laButtonWidget* ButtonAllPlay;
extern laImageWidget* ImageWidget_help;
extern laButtonWidget* ButtonWidget_back;


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
