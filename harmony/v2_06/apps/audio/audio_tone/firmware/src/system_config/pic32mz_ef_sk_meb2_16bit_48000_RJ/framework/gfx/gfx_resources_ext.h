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
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset MCHP_LOGO;
	
/*********************************
 * GFX Image Asset
 * Name:   AudioMute16_3
 * Size:   64x50 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset AudioMute16_3;
	
/*********************************
 * GFX Image Asset
 * Name:   AudioMute16_3p
 * Size:   64x50 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset AudioMute16_3p;
	
/*********************************
 * GFX Image Asset
 * Name:   AudioPlay16_3
 * Size:   64x50 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset AudioPlay16_3;
	
/*********************************
 * GFX Image Asset
 * Name:   AudioPlay16_3p
 * Size:   64x50 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset AudioPlay16_3p;
	
/*****************************************************************************
 * MPLAB Harmony Graphics Font Assets
 *****************************************************************************/
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans14
 * Height:       19
 * Style:        Plain
 * Glyph Count:  66
 * Range Count:  6
 * Glyph Ranges: 0x20
			     0x2B
			     0x2D
			     0x2F-0x39
			     0x41-0x5A
			     0x61-0x7A
 ***********************************/
extern GFXU_FontAsset LiberationSans14;
	
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans12
 * Height:       16
 * Style:        Plain
 * Glyph Count:  67
 * Range Count:  7
 * Glyph Ranges: 0x20
			     0x25
			     0x28-0x29
			     0x2D
			     0x30-0x39
			     0x41-0x5A
			     0x61-0x7A
 ***********************************/
extern GFXU_FontAsset LiberationSans12;
	
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans11
 * Height:       15
 * Style:        Plain
 * Glyph Count:  0
 * Range Count:  0
 ***********************************/
extern GFXU_FontAsset LiberationSans11;
	
/*********************************
 * GFX Font Asset
 * Name:         LiberationSans18
 * Height:       24
 * Style:        Plain
 * Glyph Count:  9
 * Range Count:  7
 * Glyph Ranges: 0x20
			     0x41
			     0x54
			     0x64-0x65
			     0x69
			     0x6E-0x6F
			     0x75
 ***********************************/
extern GFXU_FontAsset LiberationSans18;
	
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
#define string_Minus    0
#define string_Mode    1
#define string_PlayPause    2
#define string_Plus    3
#define string_Resolution    4
#define string_Resolution_value    5
#define string_Sample_Frequency    6
#define string_Sample_Frequency_value    7
#define string_String12    8
#define string_String14    9
#define string_Title    10
#define string_Volume    11
#define string_f1_Hz    12
#define string_f2_Hz    13
#define string_t_ms    14
 
extern GFXU_StringTableAsset stringTable;

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* GFX_RESOURCES_EXT_H */

