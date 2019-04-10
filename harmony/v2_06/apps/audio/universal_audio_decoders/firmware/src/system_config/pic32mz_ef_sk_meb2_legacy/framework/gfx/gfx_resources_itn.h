/*******************************************************************************
  MPLAB Harmony Graphics Asset Header File

  File Name:
    gfx_resources_itn.h

  Summary:
    Header file containing a list of asset specifications for use with the
	MPLAB Harmony Graphics Stack.

  Description:
    Header file containing a list of asset specifications for use with the
	MPLAB Harmony Graphics Stack.

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

#ifndef GFX_RESOURCES_ITN_H
#define GFX_RESOURCES_ITN_H

#include "gfx/utils/gfx_utils.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif
// DOM-IGNORE-END 

/*** Generated Asset Descriptors ***/
/*****************************************************************************
 * MPLAB Harmony Graphics Asset Location IDs
 *****************************************************************************/
#define GFXU_ASSET_LOCATION_ID_INTERNAL    0
 
/*****************************************************************************
 * MPLAB Harmony Graphics Image Assets
 *****************************************************************************/
/*********************************
 * GFX Image Asset
 * Name:   MCHP_LOGO
 * Size:   122x30 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset MCHP_LOGO;
	
/*********************************
 * GFX Image Asset
 * Name:   Play
 * Size:   29x33 pixels
 * Mode:   INDEX_4
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset Play;
	
/*********************************
 * GFX Image Asset
 * Name:   Pause
 * Size:   29x33 pixels
 * Mode:   INDEX_1
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset Pause;
	
/*********************************
 * GFX Image Asset
 * Name:   Prev
 * Size:   29x19 pixels
 * Mode:   INDEX_4
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset Prev;
	
/*********************************
 * GFX Image Asset
 * Name:   Next
 * Size:   29x19 pixels
 * Mode:   INDEX_4
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset Next;
	
/*********************************
 * GFX Image Asset
 * Name:   progress_indicator_circle
 * Size:   15x15 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset progress_indicator_circle;
	
/*********************************
 * GFX Image Asset
 * Name:   NextShrink
 * Size:   24x16 pixels
 * Mode:   INDEX_4
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset NextShrink;
	
/*********************************
 * GFX Image Asset
 * Name:   PrevShrink
 * Size:   24x16 pixels
 * Mode:   INDEX_4
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset PrevShrink;
	
/*********************************
 * GFX Image Asset
 * Name:   MUSIC_ICON
 * Size:   25x25 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset MUSIC_ICON;
	
/*********************************
 * GFX Image Asset
 * Name:   RIGHT_NAV
 * Size:   44x44 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset RIGHT_NAV;
	
/*********************************
 * GFX Image Asset
 * Name:   LEFT_NAV
 * Size:   44x44 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset LEFT_NAV;
	
/*****************************************************************************
 * MPLAB Harmony Graphics Palette Assets
 *****************************************************************************/
/*********************************
 * GFX Palette Asset
 * Name:   MCHP_LOGO_palette
 * Colors: 228
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset MCHP_LOGO_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   Play_palette
 * Colors: 3
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset Play_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   Pause_palette
 * Colors: 2
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset Pause_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   Prev_palette
 * Colors: 3
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset Prev_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   progress_indicator_circle_palette
 * Colors: 35
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset progress_indicator_circle_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   MUSIC_ICON_palette
 * Colors: 79
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset MUSIC_ICON_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   RIGHT_NAV_palette
 * Colors: 23
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset RIGHT_NAV_palette;
	
/*****************************************************************************
 * MPLAB Harmony Graphics Font Assets
 *****************************************************************************/
/*********************************
 * GFX Font Asset
 * Name:         Arial
 * Height:       14
 * Style:        Bold
 * Glyph Count:  96
 * Range Count:  1
 * Glyph Ranges: 0x20-0x7F
 ***********************************/
extern GFXU_FontAsset Arial;
	
/*********************************
 * GFX Font Asset
 * Name:         Arial_16
 * Height:       19
 * Style:        Bold
 * Glyph Count:  96
 * Range Count:  1
 * Glyph Ranges: 0x20-0x7F
 ***********************************/
extern GFXU_FontAsset Arial_16;
	
/*********************************
 * GFX Font Asset
 * Name:         Arial_14
 * Height:       17
 * Style:        Bold
 * Glyph Count:  21
 * Range Count:  13
 * Glyph Ranges: 0x20
			     0x28-0x29
			     0x41
			     0x44
			     0x4D
			     0x55
			     0x5A
			     0x61
			     0x63-0x65
			     0x69
			     0x6C-0x6F
			     0x72-0x73
			     0x75-0x76
 ***********************************/
extern GFXU_FontAsset Arial_14;
	
/*********************************
 * GFX Font Asset
 * Name:         Calibri
 * Height:       17
 * Style:        Bold
 * Glyph Count:  96
 * Range Count:  1
 * Glyph Ranges: 0x20-0x7F
 ***********************************/
extern GFXU_FontAsset Calibri;
	
/*********************************
 * GFX Font Asset
 * Name:         Calibri_Numbers
 * Height:       17
 * Style:        Plain
 * Glyph Count:  11
 * Range Count:  1
 * Glyph Ranges: 0x30-0x3A
 ***********************************/
extern GFXU_FontAsset Calibri_Numbers;
	
/*****************************************************************************
 * MPLAB Harmony Graphics String Table
 *****************************************************************************/
/*********************************
 * GFX String Table
 * Name:         stringTable
 * Encoding:     ASCII
 * Languages:    default
 * String Count: 18
 ***********************************/
// language IDs
#define language_default    0

// string IDs
#define string_AAC_DECODER    0
#define string_ADPCM_DECODER    1
#define string_ALBUM    2
#define string_DEMO_NAME    3
#define string_FLAC_DECODER    4
#define string_FileExplorer    5
#define string_InsertUSB    6
#define string_MP3_DECODER    7
#define string_MSD_STATUS    8
#define string_NO_AUDIO_FILE    9
#define string_OPUS_DECODER    10
#define string_PLAY_TIME    11
#define string_SPEEX_DECODER    12
#define string_TRACK_ARTIST    13
#define string_TRACK_LENGTH    14
#define string_TRACK_NAME    15
#define string_WAV_DECODER    16
#define string_WMA_DECODER    17
 
extern GFXU_StringTableAsset stringTable;

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* GFX_RESOURCES_ITN_H */

