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

#include "gfx/gfx_resources_ext.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

#define LIBARIA_SCREEN_COUNT   1

// reference IDs for generated libaria screens
// screen "DemoScreen"
#define DemoScreen_ID    0



extern laScheme orangeText;
extern laScheme whiteText;
extern laScheme labelText;
extern laScheme blueblackbackground;
extern laScheme blackBoxes;
extern laScheme redBoxes;
extern laRectangleWidget* GFX_BACKGROUND;
extern laRectangleWidget* RectangleWidget3;
extern laImageWidget* GFX_MCHPLOGO;
extern laLabelWidget* GFX_TITLE;
extern laGroupBoxWidget* GroupBoxWidget5;
extern laLabelWidget* GFX_TONEMODE;
extern laLabelWidget* GFX_SAMPLEFREQ;
extern laButtonWidget* sinechirp;
extern laLabelWidget* GFX_SAMPLEFREQ_VALUE;
extern laLabelWidget* GFX_RESOLUTION;
extern laLabelWidget* GFX_RESOLUTION_VALUE;
extern laGroupBoxWidget* GroupBoxWidget1;
extern laLabelWidget* GFX_F1HZ;
extern laRectangleWidget* GFX_F1_BOX;
extern laLabelWidget* GFX_F1_VALUE;
extern laButtonWidget* f1plus;
extern laButtonWidget* f1mnus;
extern laGroupBoxWidget* GroupBoxWidget2;
extern laLabelWidget* GFX_F2HZ;
extern laRectangleWidget* GFX_F2_BOX;
extern laLabelWidget* GFX_F2_VALUE;
extern laButtonWidget* f2plus;
extern laButtonWidget* f2minus;
extern laGroupBoxWidget* GroupBoxWidget3;
extern laLabelWidget* GFX_TMS;
extern laRectangleWidget* GFX_TMS_BOX;
extern laLabelWidget* GFX_TMS_VALUE;
extern laButtonWidget* tplus;
extern laButtonWidget* tminus;
extern laGroupBoxWidget* GroupBoxWidget4;
extern laLabelWidget* GFX_VOLUME;
extern laRectangleWidget* GFX_VOLUME_BOX;
extern laLabelWidget* GFX_VOLUME_VALUE;
extern laButtonWidget* volumeplus;
extern laButtonWidget* volumeminus;
extern laButtonWidget* GFX_PLAYPAUSE;


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
