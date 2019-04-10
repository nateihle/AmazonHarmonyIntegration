/*******************************************************************************
  MPLAB Harmony Graphics Object Library Asset Source File

  File Name:
    gfx_resources_ext.c

  Summary:
    Source file containing asset data for use with the MPLAB Harmony Graphics
	Object Library.

  Description:
    Source file containing asset data for use with the MPLAB Harmony Graphics
	Object Library.

    Created with MPLAB Harmony Version 2.03
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

#include "gfx/gfx_resources_ext.h"

/*****************************************************************************
 Asset Manifest
 -------------------
 
 Fonts
 -------
    Arial14pt
    Arial12pt


 *****************************************************************************/

/*****************************************************************************
 * SECTION:  Fonts
 
 - font lookup table data description -
1 byte - size of the address offset values in this table, 1-4 possible
1 byte - size of the address width values in this table, 1-2 possible
  for each glyph entry in lookup table:
    1-4 bytes - glyph data offset in bytes
    1-2 bytes - glyph raster width in pixels	
 
 *****************************************************************************/
uint8_t Arial14pt_lookup_0[4] =
{
    0x01,0x01,0x00,0x0B,
};


GFXU_FontGlyphIndexTable Arial14pt_index_table =
{
	1, // range count
    {
	    /* 0x0 */
        {
		    1, // glyph count
		    0x0, // starting glyph id
		    0x0, // ending glyph id
		    Arial14pt_lookup_0 // glyph lookup table
        },
    }
};
		
// 1 glyphs @ 1 bpp
const uint8_t Arial14pt_data[34] =
{
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3F,0x80,0x20,0x80,0x20,
    0x80,0x20,0x80,0x20,0x80,0x20,0x80,0x20,0x80,0x20,0x80,0x3F,0x80,0x00,0x00,
    0x00,0x00,0x00,0x00,
};

GFXU_FontAsset Arial14pt =
{
	{
        GFXU_ASSET_TYPE_FONT, // asset type
	    GFXU_ASSET_LOCATION_ID_INTERNAL, // data location id
	    (void*)Arial14pt_data, // data address pointer
	    34, // data size
    },	
	17, // font height
	13, // font max ascent
	3, // font max descent
	4, // font baseline
	GFXU_FONT_BPP_1, // bits per pixel
	&Arial14pt_index_table // glyph index table
};
		
uint8_t Arial12pt_lookup_0[4] =
{
    0x01,0x01,0x00,0x09,
};


GFXU_FontGlyphIndexTable Arial12pt_index_table =
{
	1, // range count
    {
	    /* 0x0 */
        {
		    1, // glyph count
		    0x0, // starting glyph id
		    0x0, // ending glyph id
		    Arial12pt_lookup_0 // glyph lookup table
        },
    }
};
		
// 1 glyphs @ 1 bpp
const uint8_t Arial12pt_data[30] =
{
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0x00,0x41,0x00,0x41,0x00,0x41,
    0x00,0x41,0x00,0x41,0x00,0x41,0x00,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};

GFXU_FontAsset Arial12pt =
{
	{
        GFXU_ASSET_TYPE_FONT, // asset type
	    GFXU_ASSET_LOCATION_ID_INTERNAL, // data location id
	    (void*)Arial12pt_data, // data address pointer
	    30, // data size
    },	
	15, // font height
	12, // font max ascent
	3, // font max descent
	3, // font baseline
	GFXU_FONT_BPP_1, // bits per pixel
	&Arial12pt_index_table // glyph index table
};
		


