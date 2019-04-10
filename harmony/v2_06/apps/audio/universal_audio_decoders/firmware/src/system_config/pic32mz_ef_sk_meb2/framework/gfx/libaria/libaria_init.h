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

#include "gfx/gfx_resources_itn.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

#define LIBARIA_SCREEN_COUNT   3

// reference IDs for generated libaria screens
// screen "FileExplorer"
#define FileExplorer_ID    2

// screen "MainScreen"
#define MainScreen_ID    1

// screen "Welcome"
#define Welcome_ID    0



extern laScheme text_label;
extern laScheme _default;
extern laScheme flac_label_scheme;
extern laScheme track_time;
extern laScheme custom_progessbar;
extern laScheme wma_label_scheme;
extern laScheme mp3_label_scheme;
extern laScheme wav_label_scheme;
extern laScheme image_button;
extern laScheme pcm_label_scheme;
extern laScheme spx_label_scheme;
extern laScheme filled_circle;
extern laScheme opus_label_scheme;
extern laScheme track_info;
extern laScheme aac_label_scheme;
extern laRectangleWidget* RectangleWidget1;
extern laImageWidget* background1;
extern laLabelWidget* DemoNameLabel;
extern laWidget* PanelWidget3;
extern laLabelWidget* MessageLabel;
extern laRectangleWidget* background;
extern laImageWidget* LogoImage;
extern laLabelWidget* AACIndicator;
extern laLabelWidget* PCMIndicator;
extern laLabelWidget* LabelWidget1;
extern laLabelWidget* WAVIndicator;
extern laLabelWidget* WMAIndicator;
extern laLabelWidget* MP3Indicator;
extern laLabelWidget* FLACIndicator;
extern laLabelWidget* OPUSIndicator;
extern laLabelWidget* SPXIndicator;
extern laWidget* ProgressPanel;
extern laRectangleWidget* ProgressBarRect;
extern laImageWidget* CircleIndicator;
extern laWidget* TrackInfoPanel;
extern laLabelWidget* TrackNameLabel;
extern laLabelWidget* TrackArtistLabel;
extern laLabelWidget* TrackAlbumLabel;
extern laWidget* ControlPanel;
extern laButtonWidget* PlayButton;
extern laButtonWidget* NextTrackButton;
extern laButtonWidget* PrevTrackButton;
extern laWidget* PanelWidget1;
extern laLabelWidget* TrackLengthLabel;
extern laWidget* PanelWidget2;
extern laLabelWidget* PlaytimeLabel;
extern laButtonWidget* ToFileExplorerButton;
extern laRectangleWidget* RectangleWidget2;
extern laListWidget* MusicList;
extern laButtonWidget* ToPlayerButton;
extern laLabelWidget* LabelWidget2;


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
