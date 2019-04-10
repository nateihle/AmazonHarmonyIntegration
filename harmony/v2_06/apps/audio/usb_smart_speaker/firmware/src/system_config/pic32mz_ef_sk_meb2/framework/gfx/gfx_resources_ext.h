/*******************************************************************************
  MPLAB Harmony Graphics Asset Header File

  File Name:
    gfx_resources_ext.h

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

#ifndef GFX_RESOURCES_EXT_H
#define GFX_RESOURCES_EXT_H

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
	
/*****************************************************************************
 * MPLAB Harmony Graphics Font Assets
 *****************************************************************************/
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans11
 * Height:       14
 * Style:        Bold
 * Glyph Count:  41
 * Range Count:  17
 * Glyph Ranges: 0x20
			     0x30-0x35
			     0x38-0x3A
			     0x41
			     0x43-0x46
			     0x48-0x49
			     0x4B
			     0x4D-0x50
			     0x54
			     0x5A
			     0x61
			     0x63-0x65
			     0x67-0x69
			     0x6B-0x70
			     0x72-0x73
			     0x76
			     0x79
 ***********************************/
extern GFXU_FontAsset LiberationSans11;
	
/*********************************
 * GFX Font Asset
 * Name:         LiberationMono11
 * Height:       14
 * Style:        Plain
 * Glyph Count:  4
 * Range Count:  3
 * Glyph Ranges: 0x30-0x31
			     0x36
			     0x3A
 ***********************************/
extern GFXU_FontAsset LiberationMono11;
	
/*********************************
 * GFX Font Asset
 * Name:         LiberationSerif11
 * Height:       13
 * Style:        Plain
 * Glyph Count:  7
 * Range Count:  4
 * Glyph Ranges: 0x43
			     0x63-0x65
			     0x6E-0x6F
			     0x74
 ***********************************/
extern GFXU_FontAsset LiberationSerif11;
	
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans14
 * Height:       17
 * Style:        Plain
 * Glyph Count:  0
 * Range Count:  0
 ***********************************/
extern GFXU_FontAsset LiberationSans14;
	
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans12Bold
 * Height:       15
 * Style:        Bold
 * Glyph Count:  0
 * Range Count:  0
 ***********************************/
extern GFXU_FontAsset LiberationSans12Bold;
	
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans12Italic
 * Height:       15
 * Style:        Italic
 * Glyph Count:  20
 * Range Count:  16
 * Glyph Ranges: 0x20
			     0x2D-0x2E
			     0x30
			     0x32
			     0x36
			     0x42-0x43
			     0x48
			     0x4D
			     0x53
			     0x55-0x56
			     0x63
			     0x65
			     0x69
			     0x6E-0x6F
			     0x72
			     0x74
 ***********************************/
extern GFXU_FontAsset LiberationSans12Italic;
	
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans16
 * Height:       19
 * Style:        Plain
 * Glyph Count:  0
 * Range Count:  0
 ***********************************/
extern GFXU_FontAsset LiberationSans16;
	
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans16Bold
 * Height:       19
 * Style:        Bold
 * Glyph Count:  11
 * Range Count:  11
 * Glyph Ranges: 0x20
			     0x42
			     0x53
			     0x55
			     0x61
			     0x65
			     0x6B
			     0x6D
			     0x70
			     0x72
			     0x74
 ***********************************/
extern GFXU_FontAsset LiberationSans16Bold;
	
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans16Italic
 * Height:       19
 * Style:        Italic
 * Glyph Count:  0
 * Range Count:  0
 ***********************************/
extern GFXU_FontAsset LiberationSans16Italic;
	
/*****************************************************************************
 * MPLAB Harmony Graphics String Table
 *****************************************************************************/
/*********************************
 * GFX String Table
 * Name:         stringTable
 * Encoding:     ASCII
 * Languages:    default
 * String Count: 14
 ***********************************/
// language IDs
#define language_default    0

// string IDs
#define string_AllLibSans12Ital    0
#define string_CodecType    1
#define string_Converged    2
#define string_DEMO_NAME    3
#define string_DT    4
#define string_Echo    5
#define string_InsertUSB    6
#define string_MHVersion    7
#define string_PLAY_TIME    8
#define string_ProcType    9
#define string_TRACK_LENGTH    10
#define string_TRACK_NAME    11
#define string_USB_STATUS    12
#define string_aecEn    13
 
extern GFXU_StringTableAsset stringTable;

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* GFX_RESOURCES_EXT_H */

