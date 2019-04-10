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
 * Name:   Bar
 * Size:   480x65 pixels
 * Mode:   RGBA_8888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset Bar;
	
/*********************************
 * GFX Image Asset
 * Name:   HarmonyLogo
 * Size:   197x139 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset HarmonyLogo;
	
/*********************************
 * GFX Image Asset
 * Name:   PIC32Logo
 * Size:   240x62 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset PIC32Logo;
	
/*********************************
 * GFX Image Asset
 * Name:   MicrochipLogo
 * Size:   144x39 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset MicrochipLogo;
	
/*********************************
 * GFX Image Asset
 * Name:   road480x300
 * Size:   480x300 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset road480x300;
	
/*********************************
 * GFX Image Asset
 * Name:   NewHarmonyLogo_VerySmall
 * Size:   80x56 pixels
 * Mode:   RGB_888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset NewHarmonyLogo_VerySmall;
	
/*****************************************************************************
 * MPLAB Harmony Graphics Font Assets
 *****************************************************************************/
/*********************************
 * GFX Font Asset
 * Name:         TimesNewRoman28
 * Height:       33
 * Style:        Bold
 * Glyph Count:  10
 * Range Count:  1
 * Glyph Ranges: 0x30-0x39
 ***********************************/
extern GFXU_FontAsset TimesNewRoman28;
	
/*********************************
 * GFX Font Asset
 * Name:         TimesNewRoman20
 * Height:       24
 * Style:        Plain
 * Glyph Count:  3
 * Range Count:  3
 * Glyph Ranges: 0x52
			     0x6E
			     0x75
 ***********************************/
extern GFXU_FontAsset TimesNewRoman20;
	
/*********************************
 * GFX Font Asset
 * Name:         ArialBig
 * Height:       43
 * Style:        Plain
 * Glyph Count:  10
 * Range Count:  1
 * Glyph Ranges: 0x30-0x39
 ***********************************/
extern GFXU_FontAsset ArialBig;
	
/*********************************
 * GFX Font Asset
 * Name:         ArialMed_Bold
 * Height:       19
 * Style:        Bold
 * Glyph Count:  15
 * Range Count:  11
 * Glyph Ranges: 0x2D
			     0x42
			     0x44
			     0x53
			     0x62
			     0x64-0x67
			     0x69
			     0x6C
			     0x6E-0x6F
			     0x72
			     0x75
 ***********************************/
extern GFXU_FontAsset ArialMed_Bold;
	
/*********************************
 * GFX Font Asset
 * Name:         ArialMed
 * Height:       19
 * Style:        Plain
 * Glyph Count:  15
 * Range Count:  11
 * Glyph Ranges: 0x2D
			     0x42
			     0x44
			     0x53
			     0x62
			     0x64-0x67
			     0x69
			     0x6C
			     0x6E-0x6F
			     0x72
			     0x75
 ***********************************/
extern GFXU_FontAsset ArialMed;
	
/*****************************************************************************
 * MPLAB Harmony Graphics String Table
 *****************************************************************************/
/*********************************
 * GFX String Table
 * Name:         stringTable
 * Encoding:     ASCII
 * Languages:    default
 * String Count: 17
 ***********************************/
// language IDs
#define language_default    0

// string IDs
#define string_DoubleBuffered_Bold    0
#define string_DoubleBuffered_Norm    1
#define string_Eight    2
#define string_Five    3
#define string_Four    4
#define string_Instructions    5
#define string_NewTxt    6
#define string_Nine    7
#define string_One    8
#define string_RoadVal    9
#define string_Seven    10
#define string_SingleBuffered_Bold    11
#define string_SingleBuffered_Norm    12
#define string_Six    13
#define string_Three    14
#define string_Two    15
#define string_Zero    16
 
extern GFXU_StringTableAsset stringTable;

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* GFX_ASSETS_H */

