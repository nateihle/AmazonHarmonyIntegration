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
 * Name:   MCHP_LOGO
 * Size:   122x30 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset MCHP_LOGO;
	
/*********************************
 * GFX Image Asset
 * Name:   cross
 * Size:   16x16 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset cross;
	
/*********************************
 * GFX Image Asset
 * Name:   check
 * Size:   16x16 pixels
 * Mode:   INDEX_4
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset check;
	
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
 * Name:   cross_palette
 * Colors: 18
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset cross_palette;
	
/*********************************
 * GFX Palette Asset
 * Name:   check_palette
 * Colors: 12
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset check_palette;
	
/*****************************************************************************
 * MPLAB Harmony Graphics Font Assets
 *****************************************************************************/
/*********************************
 * GFX Font Asset
 * Name:         Arial_14
 * Height:       17
 * Style:        Plain
 * Glyph Count:  29
 * Range Count:  17
 * Glyph Ranges: 0x20
			     0x28-0x29
			     0x41
			     0x43-0x45
			     0x4D
			     0x4F-0x50
			     0x53
			     0x55-0x57
			     0x61
			     0x63-0x65
			     0x67
			     0x69
			     0x6C
			     0x6E-0x70
			     0x72-0x73
			     0x75-0x76
			     0x78
 ***********************************/
extern GFXU_FontAsset Arial_14;
	
/*********************************
 * GFX Font Asset
 * Name:         Arial
 * Height:       15
 * Style:        Bold
 * Glyph Count:  96
 * Range Count:  1
 * Glyph Ranges: 0x20-0x7F
 ***********************************/
extern GFXU_FontAsset Arial;
	
/*****************************************************************************
 * MPLAB Harmony Graphics String Table
 *****************************************************************************/
/*********************************
 * GFX String Table
 * Name:         stringTable
 * Encoding:     ASCII
 * Languages:    default
 * String Count: 9
 ***********************************/
// language IDs
#define language_default    0

// string IDs
#define string_ADPCMEncoder    0
#define string_DEMO_NAME    1
#define string_InsertUSB    2
#define string_OPUSEncoder    3
#define string_PCMEncoder    4
#define string_SPEEXEncoder    5
#define string_SaveFile    6
#define string_StartRecord    7
#define string_StopRecord    8
 
extern GFXU_StringTableAsset stringTable;

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* GFX_ASSETS_H */

