/*******************************************************************************
  MPLAB Harmony Graphics Asset Header File

  File Name:
    gfx_assets.h

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

#ifndef GFX_ASSETS_H
#define GFX_ASSETS_H

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
 * Name:   MCHP_LOGO2
 * Size:   122x30 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset MCHP_LOGO2;
	
/*********************************
 * GFX Image Asset
 * Name:   fastforward
 * Size:   28x19 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset fastforward;
	
/*********************************
 * GFX Image Asset
 * Name:   previous
 * Size:   15x19 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset previous;
	
/*********************************
 * GFX Image Asset
 * Name:   next
 * Size:   15x19 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset next;
	
/*********************************
 * GFX Image Asset
 * Name:   rewind_1
 * Size:   28x19 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset rewind_1;
	
/*********************************
 * GFX Image Asset
 * Name:   AudioMute16_2
 * Size:   32x25 pixels
 * Mode:   INDEX_4
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset AudioMute16_2;
	
/*********************************
 * GFX Image Asset
 * Name:   AudioPlay16_2
 * Size:   32x25 pixels
 * Mode:   INDEX_4
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset AudioPlay16_2;
	
/*********************************
 * GFX Image Asset
 * Name:   CONNECTED
 * Size:   30x30 pixels
 * Mode:   INDEX_1
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset CONNECTED;
	
/*********************************
 * GFX Image Asset
 * Name:   NO_PAIR_NO_CONNECTION
 * Size:   30x30 pixels
 * Mode:   INDEX_1
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset NO_PAIR_NO_CONNECTION;
	
/*********************************
 * GFX Image Asset
 * Name:   PAIRED
 * Size:   30x30 pixels
 * Mode:   INDEX_1
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset PAIRED;
	
/*****************************************************************************
 * MPLAB Harmony Graphics Palette Assets
 *****************************************************************************/
/*********************************
 * GFX Palette Asset
 * Name:   MCHP_LOGO2_palette
 * Colors: 228
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset MCHP_LOGO2_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   fastforward_palette
 * Colors: 42
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset fastforward_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   previous_palette
 * Colors: 29
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset previous_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   next_palette
 * Colors: 32
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset next_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   rewind_1_palette
 * Colors: 34
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset rewind_1_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   AudioMute16_2_palette
 * Colors: 6
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset AudioMute16_2_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   AudioPlay16_2_palette
 * Colors: 5
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset AudioPlay16_2_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   CONNECTED_palette
 * Colors: 2
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset CONNECTED_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   NO_PAIR_NO_CONNECTION_palette
 * Colors: 2
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset NO_PAIR_NO_CONNECTION_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   PAIRED_palette
 * Colors: 2
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset PAIRED_palette;
	
/*****************************************************************************
 * MPLAB Harmony Graphics Font Assets
 *****************************************************************************/
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans14
 * Height:       17
 * Style:        Plain
 * Glyph Count:  18
 * Range Count:  15
 * Glyph Ranges: 0x20
			     0x2F
			     0x32
			     0x34
			     0x36
			     0x41-0x44
			     0x46
			     0x48
			     0x4D
			     0x50
			     0x52
			     0x56
			     0x65
			     0x6D
			     0x6F
 ***********************************/
extern GFXU_FontAsset LiberationSans14;
	
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans12
 * Height:       15
 * Style:        Plain
 * Glyph Count:  83
 * Range Count:  6
 * Glyph Ranges: 0x20-0x2E
			     0x30-0x3B
			     0x40-0x5B
			     0x5D
			     0x5F
			     0x61-0x7A
 ***********************************/
extern GFXU_FontAsset LiberationSans12;
	
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans11
 * Height:       14
 * Style:        Plain
 * Glyph Count:  0
 * Range Count:  0
 ***********************************/
extern GFXU_FontAsset LiberationSans11;
	
/*****************************************************************************
 * MPLAB Harmony Graphics String Table
 *****************************************************************************/
/*********************************
 * GFX String Table
 * Name:         stringTable
 * Encoding:     ASCII
 * Languages:    default
 * String Count: 4
 ***********************************/
// language IDs
#define language_default    0

// string IDs
#define string_AlphaNumericPunc    0
#define string_DeviceAddress    1
#define string_DeviceName    2
#define string_Title    3
 
extern GFXU_StringTableAsset stringTable;

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* GFX_ASSETS_H */

