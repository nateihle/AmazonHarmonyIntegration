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
 * Name:   CONNECTED_1
 * Size:   30x30 pixels
 * Mode:   RGB_888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset CONNECTED_1;
	
/*********************************
 * GFX Image Asset
 * Name:   NO_PAIR_NO_CONNECTION_1
 * Size:   30x30 pixels
 * Mode:   RGB_888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset NO_PAIR_NO_CONNECTION_1;
	
/*********************************
 * GFX Image Asset
 * Name:   PAIRED_1
 * Size:   30x30 pixels
 * Mode:   RGB_888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset PAIRED_1;
	
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
	
/*****************************************************************************
 * MPLAB Harmony Graphics Font Assets
 *****************************************************************************/
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans11
 * Height:       14
 * Style:        Plain
 * Glyph Count:  94
 * Range Count:  2
 * Glyph Ranges: 0x20-0x2C
			     0x2E-0x7E
 ***********************************/
extern GFXU_FontAsset LiberationSans11;
	
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
 * Glyph Count:  12
 * Range Count:  7
 * Glyph Ranges: 0x20
			     0x31-0x35
			     0x42
			     0x44-0x45
			     0x47
			     0x4C
			     0x52
 ***********************************/
extern GFXU_FontAsset LiberationSans12Bold;
	
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans12Italic
 * Height:       15
 * Style:        Italic
 * Glyph Count:  95
 * Range Count:  1
 * Glyph Ranges: 0x20-0x7E
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
 * Glyph Count:  13
 * Range Count:  12
 * Glyph Ranges: 0x20
			     0x43
			     0x50
			     0x53
			     0x56
			     0x63
			     0x65
			     0x69
			     0x6C
			     0x6E-0x6F
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
	
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans12
 * Height:       15
 * Style:        Plain
 * Glyph Count:  29
 * Range Count:  9
 * Glyph Ranges: 0x20
			     0x2D
			     0x30-0x3A
			     0x41-0x46
			     0x49
			     0x52
			     0x54
			     0x56
			     0x61-0x66
 ***********************************/
extern GFXU_FontAsset LiberationSans12;
	
/*****************************************************************************
 * MPLAB Harmony Graphics String Table
 *****************************************************************************/
/*********************************
 * GFX String Table
 * Name:         stringTable
 * Encoding:     ASCII
 * Languages:    default
 * String Count: 15
 ***********************************/
// language IDs
#define language_default    0

// string IDs
#define string_AlphaNum11    0
#define string_Blank1    1
#define string_BtDemoName    2
#define string_CodecType    3
#define string_DEMO_NAME    4
#define string_InsertUSB    5
#define string_LEDstring    6
#define string_MHVersion    7
#define string_MacAddr    8
#define string_PLAY_TIME    9
#define string_RGBLabel    10
#define string_RecvTest    11
#define string_TRACK_LENGTH    12
#define string_USB_STATUS    13
#define string_alphaNumLibSans12Italic    14
 
extern GFXU_StringTableAsset stringTable;

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* GFX_RESOURCES_EXT_H */

