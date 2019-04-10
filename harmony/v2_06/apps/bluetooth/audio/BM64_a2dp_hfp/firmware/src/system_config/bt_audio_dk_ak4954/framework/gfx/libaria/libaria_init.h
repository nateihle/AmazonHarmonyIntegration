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

#include "gfx/gfx_assets.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

#define LIBARIA_SCREEN_COUNT   1

// reference IDs for generated libaria screens
// screen "default"
#define default_ID    0



extern laScheme defaultScheme;
extern laScheme tealtext;
extern laScheme orangetext;
extern laScheme redtext;
extern laScheme blueblackBackground;
extern laRectangleWidget* GFX_BACKGROUND;
extern laImageWidget* GFX_MCHPLOGO;
extern laLabelWidget* GFX_TITLE;
extern laWidget* GFX_DEVPANEL;
extern laLabelWidget* GFX_BTNAME;
extern laLabelWidget* GFX_BTADDRESS;
extern laLabelWidget* GFX_BTNAME_VALUE;
extern laLabelWidget* GFX_BTADDRESS_VALUE;
extern laWidget* GFX_MIDPANEL;
extern laImageWidget* GFX_AUDIO_MUTE;
extern laImageWidget* GFX_AUDIO_PLAY;
extern laWidget* GFX_LEFTPANEL;
extern laImageWidget* GFX_PREVIOUS;
extern laImageWidget* GFX_REWIND;
extern laWidget* GFX_RIGHTPANEL;
extern laImageWidget* GFX_FASTFORWARD;
extern laImageWidget* GFX_NEXT;
extern laWidget* GFX_VOLPANEL;
extern laLabelWidget* GFX_VOLUME_VALUE;
extern laWidget* GFX_CONNECT;
extern laImageWidget* GFX_CONNECTED;
extern laImageWidget* GFX_PAIRED;
extern laImageWidget* GFX_NOPAIR_NOCONNECTION;


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
