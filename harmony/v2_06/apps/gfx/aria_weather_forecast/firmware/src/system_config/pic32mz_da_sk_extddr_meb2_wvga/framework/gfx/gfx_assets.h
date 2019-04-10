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
 * Name:   cloud_icon
 * Size:   128x128 pixels
 * Mode:   RGB_888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset cloud_icon;
	
/*********************************
 * GFX Image Asset
 * Name:   rain_icon_sm
 * Size:   128x128 pixels
 * Mode:   RGB_888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset rain_icon_sm;
	
/*********************************
 * GFX Image Asset
 * Name:   sun_icon_sm
 * Size:   128x128 pixels
 * Mode:   RGB_888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset sun_icon_sm;
	
/*********************************
 * GFX Image Asset
 * Name:   mchpLogo
 * Size:   69x65 pixels
 * Mode:   RGB_888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset mchpLogo;
	
/*********************************
 * GFX Image Asset
 * Name:   clouds3_large
 * Size:   800x177 pixels
 * Mode:   RGB_888
 * Format: JPEG
 ***********************************/
extern GFXU_ImageAsset clouds3_large;
	
/*********************************
 * GFX Image Asset
 * Name:   Bar
 * Size:   800x91 pixels
 * Mode:   RGB_888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset Bar;
	
/*********************************
 * GFX Image Asset
 * Name:   HarmonyLogo
 * Size:   307x214 pixels
 * Mode:   RGBA_8888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset HarmonyLogo;
	
/*********************************
 * GFX Image Asset
 * Name:   MicrochipLogo
 * Size:   202x55 pixels
 * Mode:   RGBA_8888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset MicrochipLogo;
	
/*********************************
 * GFX Image Asset
 * Name:   PIC32Logo
 * Size:   372x96 pixels
 * Mode:   RGBA_8888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset PIC32Logo;
	
/*****************************************************************************
 * MPLAB Harmony Graphics Font Assets
 *****************************************************************************/
/*********************************
 * GFX Font Asset
 * Name:         ArialUnicodeMS28
 * Height:       49
 * Style:        Plain
 * Glyph Count:  86
 * Range Count:  38
 * Glyph Ranges: 0x20
			     0x25
			     0x2B
			     0x2F-0x35
			     0x38-0x3A
			     0x41-0x45
			     0x48-0x49
			     0x4B-0x59
			     0x61-0x65
			     0x67-0x70
			     0x72-0x7A
			     0xED
			     0xF1
			     0x4E91
			     0x4F4E
			     0x524D
			     0x53D6
			     0x591A
			     0x592A
			     0x5B9A
			     0x5EA6
			     0x5F53
			     0x6307
			     0x6570
			     0x6B61
			     0x6D88
			     0x6E2F
			     0x6E7F
			     0x7684
			     0x76EE
			     0x78BA
			     0x8FCE
			     0x901F
			     0x9633
			     0x96E8
			     0x9805
			     0x98CE
			     0x9999
 ***********************************/
extern GFXU_FontAsset ArialUnicodeMS28;
	
/*********************************
 * GFX Font Asset
 * Name:         ArialUnicodeMS_Large
 * Height:       97
 * Style:        Plain
 * Glyph Count:  8
 * Range Count:  7
 * Glyph Ranges: 0x20
			     0x33
			     0x36-0x37
			     0x39
			     0x43
			     0x46
			     0xB0
 ***********************************/
extern GFXU_FontAsset ArialUnicodeMS_Large;
	
/*****************************************************************************
 * MPLAB Harmony Graphics String Table
 *****************************************************************************/
/*********************************
 * GFX String Table
 * Name:         stringTable
 * Encoding:     UTF8
 * Languages:    English, Chinese, Spanish
 * String Count: 30
 ***********************************/
// language IDs
#define language_English    0
#define language_Chinese    1
#define language_Spanish    2

// string IDs
#define string_Cancel    0
#define string_Chandler    1
#define string_Cloudy    2
#define string_Colon    3
#define string_Current    4
#define string_Item1    5
#define string_Item2    6
#define string_Item3    7
#define string_Item4    8
#define string_Item5    9
#define string_LanguageID    10
#define string_LayerTitle    11
#define string_LocalizationTitle    12
#define string_Ok    13
#define string_Quit    14
#define string_Rain    15
#define string_SmallString    16
#define string_Start    17
#define string_Stop    18
#define string_Sunny    19
#define string_TimeHour    20
#define string_TimeMinute    21
#define string_Welcome    22
#define string_degrees    23
#define string_humidity    24
#define string_ninemph    25
#define string_thirteenpercent    26
#define string_uvindex    27
#define string_uvlow    28
#define string_windspeed    29
 
extern GFXU_StringTableAsset stringTable;

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* GFX_ASSETS_H */

