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
 * Name:   harmony_faded
 * Size:   180x126 pixels
 * Mode:   GS_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset harmony_faded;

/*********************************
 * GFX Image Asset
 * Name:   mchpLogo
 * Size:   32x32 pixels
 * Mode:   GS_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset mchpLogo;

/*********************************
 * GFX Image Asset
 * Name:   sdcard_sm
 * Size:   32x32 pixels
 * Mode:   GS_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset sdcard_sm;

/*********************************
 * GFX Image Asset
 * Name:   usb_icon_sm
 * Size:   32x32 pixels
 * Mode:   GS_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset usb_icon_sm;

/*****************************************************************************
 * MPLAB Harmony Graphics Font Assets
 *****************************************************************************/
/*********************************
 * GFX Font Asset
 * Name:         Arial
 * Height:       22
 * Style:        Bold
 * Glyph Count:  27
 * Range Count:  13
 * Glyph Ranges: 0x20
			     0x41-0x44
			     0x46
			     0x4D
			     0x4F
			     0x53
			     0x55
			     0x61
			     0x63-0x65
			     0x67-0x69
			     0x6B-0x70
			     0x72-0x74
			     0x79
 ***********************************/
extern GFXU_FontAsset Arial;

/*********************************
 * GFX Font Asset
 * Name:         Arial_sm
 * Height:       19
 * Style:        Plain
 * Glyph Count:  48
 * Range Count:  14
 * Glyph Ranges: 0x20-0x21
			     0x27
			     0x2E
			     0x30-0x39
			     0x41-0x44
			     0x46
			     0x49
			     0x4E
			     0x50-0x51
			     0x53-0x55
			     0x61
			     0x63-0x69
			     0x6B-0x70
			     0x72-0x79
 ***********************************/
extern GFXU_FontAsset Arial_sm;

/*****************************************************************************
 * MPLAB Harmony Graphics String Table
 *****************************************************************************/
/*********************************
 * GFX String Table
 * Name:         stringTable
 * Encoding:     ASCII
 * Languages:    default
 * String Count: 19
 ***********************************/
// language IDs
#define language_default    0

// string IDs
#define string_Done    0
#define string_FileNotFound1    1
#define string_FileNotFound2    2
#define string_Flashing    3
#define string_FlashingComplete    4
#define string_InvalidFile    5
#define string_NoMedium    6
#define string_NoValidMedium    7
#define string_NoValidMedium2    8
#define string_Numbers    9
#define string_Of    10
#define string_Ok    11
#define string_RecordCount    12
#define string_SDCard    13
#define string_SelectSource    14
#define string_Title    15
#define string_USB    16
#define string_UnknownError    17
#define string_stringNumberFiller    18

extern GFXU_StringTableAsset stringTable;

/*****************************************************************************
 * MPLAB Harmony Graphics Global Palette
 *****************************************************************************/

extern uint16_t globalColorPalette[256];
//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* GFX_ASSETS_H */

