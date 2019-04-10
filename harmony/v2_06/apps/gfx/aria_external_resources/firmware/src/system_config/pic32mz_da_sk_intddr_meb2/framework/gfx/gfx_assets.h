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
#define GFXU_ASSET_LOCATION_ID_SQI    1
#define GFXU_ASSET_LOCATION_ID_USBBin    2
#define GFXU_ASSET_LOCATION_ID_USBFile    3

/*****************************************************************************
 * MPLAB Harmony Graphics Image Assets
 *****************************************************************************/
/*********************************
 * GFX Image Asset
 * Name:   MicrochipLogo
 * Size:   144x39 pixels
 * Mode:   ARGB_8888
 * Format: PNG
 ***********************************/
extern GFXU_ImageAsset MicrochipLogo;

/*********************************
 * GFX Image Asset
 * Name:   HarmonyLogo
 * Size:   57x40 pixels
 * Mode:   RGB_888
 * Format: PNG
 ***********************************/
extern GFXU_ImageAsset HarmonyLogo;

/*********************************
 * GFX Image Asset
 * Name:   CrossFade0
 * Size:   240x139 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset CrossFade0;

/*********************************
 * GFX Image Asset
 * Name:   CrossFade1
 * Size:   240x139 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset CrossFade1;

/*********************************
 * GFX Image Asset
 * Name:   CrossFade2
 * Size:   240x139 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset CrossFade2;

/*********************************
 * GFX Image Asset
 * Name:   CrossFade3
 * Size:   240x139 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset CrossFade3;

/*********************************
 * GFX Image Asset
 * Name:   Bar
 * Size:   480x65 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset Bar;

/*********************************
 * GFX Image Asset
 * Name:   HarmonyLogo_1
 * Size:   197x139 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset HarmonyLogo_1;

/*********************************
 * GFX Image Asset
 * Name:   MicrochipLogo_1
 * Size:   144x39 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset MicrochipLogo_1;

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
 * Name:   Image_int_1
 * Size:   480x272 pixels
 * Mode:   RGB_888
 * Format: JPEG
 ***********************************/
extern GFXU_ImageAsset Image_int_1;

/*********************************
 * GFX Image Asset
 * Name:   Image_int_3
 * Size:   480x272 pixels
 * Mode:   RGB_888
 * Format: JPEG
 ***********************************/
extern GFXU_ImageAsset Image_int_3;

/*********************************
 * GFX Image Asset
 * Name:   slides_icon_pause_70
 * Size:   70x70 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset slides_icon_pause_70;

/*********************************
 * GFX Image Asset
 * Name:   slides_icon_play_70
 * Size:   70x70 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset slides_icon_play_70;

/*********************************
 * GFX Image Asset
 * Name:   Image_int_2
 * Size:   480x272 pixels
 * Mode:   RGB_888
 * Format: JPEG
 ***********************************/
extern GFXU_ImageAsset Image_int_2;

/*********************************
 * GFX Image Asset
 * Name:   intMemrect_pic32_gimp_70
 * Size:   70x70 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset intMemrect_pic32_gimp_70;

/*********************************
 * GFX Image Asset
 * Name:   loadtimes_gimp_70
 * Size:   70x70 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset loadtimes_gimp_70;

/*********************************
 * GFX Image Asset
 * Name:   intMemrect_pic32_gimp_70_1
 * Size:   70x70 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset intMemrect_pic32_gimp_70_1;

/*********************************
 * GFX Image Asset
 * Name:   slides_icon_gimp_70
 * Size:   70x70 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset slides_icon_gimp_70;

/*********************************
 * GFX Image Asset
 * Name:   slides_pause_gimp_70
 * Size:   70x70 pixels
 * Mode:   RGB_565
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset slides_pause_gimp_70;

/*********************************
 * GFX Image Asset
 * Name:   sqiFlashrect_gimp_70
 * Size:   70x70 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset sqiFlashrect_gimp_70;

/*********************************
 * GFX Image Asset
 * Name:   sqiFlashrect_red_gimp_70
 * Size:   70x70 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset sqiFlashrect_red_gimp_70;

/*********************************
 * GFX Image Asset
 * Name:   usbrect_gimp_70
 * Size:   70x70 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset usbrect_gimp_70;

/*********************************
 * GFX Image Asset
 * Name:   sqiFlashrect_null_70
 * Size:   70x70 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset sqiFlashrect_null_70;

/*********************************
 * GFX Image Asset
 * Name:   Image_int_4
 * Size:   240x136 pixels
 * Mode:   RGB_888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset Image_int_4;

/*********************************
 * GFX Image Asset
 * Name:   ext_res_help
 * Size:   480x272 pixels
 * Mode:   RGB_888
 * Format: JPEG
 ***********************************/
extern GFXU_ImageAsset ext_res_help;

/*********************************
 * GFX Image Asset
 * Name:   back_gimp_70
 * Size:   70x70 pixels
 * Mode:   RGB_888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset back_gimp_70;

/*********************************
 * GFX Image Asset
 * Name:   questionmark_gimp_70
 * Size:   70x70 pixels
 * Mode:   INDEX_8
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset questionmark_gimp_70;

/*********************************
 * GFX Image Asset
 * Name:   Image_sqi_3
 * Size:   480x272 pixels
 * Mode:   RGB_888
 * Format: JPEG
 ***********************************/
extern GFXU_ImageAsset Image_sqi_3;

/*********************************
 * GFX Image Asset
 * Name:   Image_sqi_1
 * Size:   480x272 pixels
 * Mode:   RGB_888
 * Format: JPEG
 ***********************************/
extern GFXU_ImageAsset Image_sqi_1;

/*********************************
 * GFX Image Asset
 * Name:   bigben
 * Size:   480x272 pixels
 * Mode:   RGB_888
 * Format: JPEG
 ***********************************/
extern GFXU_ImageAsset bigben;

/*********************************
 * GFX Image Asset
 * Name:   Image_sqi_2
 * Size:   480x272 pixels
 * Mode:   RGB_888
 * Format: JPEG
 ***********************************/
extern GFXU_ImageAsset Image_sqi_2;

/*********************************
 * GFX Image Asset
 * Name:   sasmit
 * Size:   480x272 pixels
 * Mode:   RGB_888
 * Format: JPEG
 ***********************************/
extern GFXU_ImageAsset sasmit;

/*********************************
 * GFX Image Asset
 * Name:   Image_sqi_4
 * Size:   240x136 pixels
 * Mode:   RGB_888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset Image_sqi_4;

/*********************************
 * GFX Image Asset
 * Name:   Image_usb_3
 * Size:   480x272 pixels
 * Mode:   RGB_888
 * Format: JPEG
 ***********************************/
extern GFXU_ImageAsset Image_usb_3;

/*********************************
 * GFX Image Asset
 * Name:   Image_usb_1
 * Size:   480x272 pixels
 * Mode:   RGB_888
 * Format: JPEG
 ***********************************/
extern GFXU_ImageAsset Image_usb_1;

/*********************************
 * GFX Image Asset
 * Name:   Image_usb_2
 * Size:   480x272 pixels
 * Mode:   RGB_888
 * Format: JPEG
 ***********************************/
extern GFXU_ImageAsset Image_usb_2;

/*********************************
 * GFX Image Asset
 * Name:   Image_usb_4
 * Size:   240x136 pixels
 * Mode:   RGB_888
 * Format: RAW
 ***********************************/
extern GFXU_ImageAsset Image_usb_4;

/*****************************************************************************
 * MPLAB Harmony Graphics Palette Assets
 *****************************************************************************/
/*********************************
 * GFX Palette Asset
 * Name:   intMemrect_pic32_gimp_70_palette
 * Colors: 237
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset intMemrect_pic32_gimp_70_palette;

/*********************************
 * GFX Palette Asset
 * Name:   sqiFlashrect_gimp_70_palette
 * Colors: 206
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset sqiFlashrect_gimp_70_palette;

/*********************************
 * GFX Palette Asset
 * Name:   sqiFlashrect_red_gimp_70_palette
 * Colors: 222
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset sqiFlashrect_red_gimp_70_palette;

/*********************************
 * GFX Palette Asset
 * Name:   usbrect_gimp_70_palette
 * Colors: 251
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset usbrect_gimp_70_palette;

/*********************************
 * GFX Palette Asset
 * Name:   sqiFlashrect_null_70_palette
 * Colors: 226
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset sqiFlashrect_null_70_palette;

/*********************************
 * GFX Palette Asset
 * Name:   questionmark_gimp_70_palette
 * Colors: 78
 * Format: RGB_565
 ***********************************/
extern GFXU_PaletteAsset questionmark_gimp_70_palette;

/*****************************************************************************
 * MPLAB Harmony Graphics Font Assets
 *****************************************************************************/
/*********************************
 * GFX Font Asset
 * Name:         ArialNarrow
 * Height:       33
 * Style:        Plain
 * Glyph Count:  0
 * Range Count:  0
 ***********************************/
extern GFXU_FontAsset ArialNarrow;

/*********************************
 * GFX Font Asset
 * Name:         Arial
 * Height:       24
 * Style:        Plain
 * Glyph Count:  41
 * Range Count:  15
 * Glyph Ranges: 0x20
			     0x2F-0x3A
			     0x42
			     0x46
			     0x49-0x4A
			     0x4D
			     0x51
			     0x53
			     0x55
			     0x61
			     0x63-0x6A
			     0x6C-0x70
			     0x72-0x75
			     0x77
			     0x79
 ***********************************/
extern GFXU_FontAsset Arial;

/*********************************
 * GFX Font Asset
 * Name:         Arial_1
 * Height:       17
 * Style:        Bold
 * Glyph Count:  6
 * Range Count:  6
 * Glyph Ranges: 0x42
			     0x45
			     0x47
			     0x4A
			     0x4D
			     0x50
 ***********************************/
extern GFXU_FontAsset Arial_1;

/*********************************
 * GFX Font Asset
 * Name:         Georgia_IntRAM
 * Height:       14
 * Style:        Bold
 * Glyph Count:  20
 * Range Count:  10
 * Glyph Ranges: 0x20
			     0x49
			     0x4C-0x4D
			     0x54
			     0x61
			     0x64-0x67
			     0x69
			     0x6C-0x6F
			     0x72-0x74
			     0x78-0x79
 ***********************************/
extern GFXU_FontAsset Georgia_IntRAM;

/*********************************
 * GFX Font Asset
 * Name:         ErasDemiITC_SQI
 * Height:       16
 * Style:        Plain
 * Glyph Count:  21
 * Range Count:  11
 * Glyph Ranges: 0x20
			     0x46
			     0x49
			     0x4C
			     0x51
			     0x53
			     0x61
			     0x64-0x69
			     0x6C-0x6F
			     0x72-0x74
			     0x78
 ***********************************/
extern GFXU_FontAsset ErasDemiITC_SQI;

/*********************************
 * GFX Font Asset
 * Name:         SegoePrint_USB
 * Height:       23
 * Style:        Bold
 * Glyph Count:  23
 * Range Count:  13
 * Glyph Ranges: 0x20
			     0x42
			     0x44
			     0x49
			     0x4C
			     0x53-0x55
			     0x61
			     0x64-0x67
			     0x69
			     0x6C-0x6F
			     0x72-0x74
			     0x76
			     0x78
 ***********************************/
extern GFXU_FontAsset SegoePrint_USB;

/*********************************
 * GFX Font Asset
 * Name:         ArialNarrow_SQI
 * Height:       33
 * Style:        Plain
 * Glyph Count:  0
 * Range Count:  0
 ***********************************/
extern GFXU_FontAsset ArialNarrow_SQI;

/*********************************
 * GFX Font Asset
 * Name:         ArialNarrow_USB_BIN
 * Height:       33
 * Style:        Plain
 * Glyph Count:  0
 * Range Count:  0
 ***********************************/
extern GFXU_FontAsset ArialNarrow_USB_BIN;

/*****************************************************************************
 * MPLAB Harmony Graphics String Table
 *****************************************************************************/
/*********************************
 * GFX String Table
 * Name:         stringTable
 * Encoding:     ASCII
 * Languages:    default, New_Language
 * String Count: 22
 ***********************************/
// language IDs
#define language_default    0
#define language_New_Language    1

// string IDs
#define string_DisplayText_IntFlash    0
#define string_DisplayText_SQI    1
#define string_DisplayText_USBBin    2
#define string_ImageType_bmp    3
#define string_ImageType_jpeg    4
#define string_Instruction    5
#define string_Instruction1    6
#define string_Instruction2    7
#define string_IntFLash    8
#define string_Int_Mem_load_time    9
#define string_JPEGFile    10
#define string_Load_time_intmem    11
#define string_Load_time_sqi    12
#define string_Load_time_usb    13
#define string_Resourcefrom    14
#define string_SQI    15
#define string_Sqi_load_time    16
#define string_Support    17
#define string_USBFile    18
#define string_USB_load_time    19
#define string_draw_time    20
#define string_ms    21

extern GFXU_StringTableAsset stringTable;

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* GFX_ASSETS_H */

