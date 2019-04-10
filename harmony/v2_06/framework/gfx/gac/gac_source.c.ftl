<#macro GAC_LOCATION_ID NAME>GFXU_ASSET_LOCATION_ID_${NAME}</#macro>
<#macro GAC_ASSET_HEADER
	TYPE
	LOCATION
	ADDRESS
	SIZE>
	{
        GFXU_ASSET_TYPE_${TYPE}, // asset type
	    GFXU_ASSET_LOCATION_ID_${LOCATION}, // data location id
<#if ADDRESS != "NULL">
	    (void*)${ADDRESS}, // data address pointer
<#else>
	    ${ADDRESS}, // data address pointer
</#if>
	    ${SIZE}, // data size
    },	
</#macro>
<#macro GAC_ASSET_DATA
	NAME
	SIZE
	DESC
	DATA>
// ${DESC}
<#if SIZE?number gt 0>
uint8_t ${NAME}_data[${SIZE}] =
${DATA};
</#if>
</#macro>
<#macro GAC_CONST_ASSET_DATA
	NAME
	SIZE
	DESC
	DATA>
// ${DESC}
<#if SIZE?number gt 0>
const uint8_t ${NAME}_data[${SIZE}] =
${DATA};
</#if>
</#macro>
<#macro GAC_IMAGE_ASSET_PALETTE
	NAME
	SIZE
	DESC
	DATA>
// ${DESC}
const uint8_t ${NAME}_palette[${SIZE}] =
${DATA};
</#macro>
<#macro GAC_IMAGE_ASSET_STRUCT
	NAME
	ADDRESS
	LOCATION_ID
	SIZE
	FORMAT
	WIDTH
	HEIGHT
	BPP
	COMPTYPE
	MODE
	FLAGS
	MASK_COLOR
	PALETTE>
GFXU_ImageAsset ${NAME} =
{
	<@GAC_ASSET_HEADER
		TYPE = "IMAGE"
		ADDRESS = ADDRESS
		LOCATION = LOCATION_ID
		SIZE = SIZE/>
	GFXU_IMAGE_FORMAT_${FORMAT}, // image format type
	${WIDTH}, // image width
	${HEIGHT}, // image height
	${WIDTH}, // buffer width
	${HEIGHT}, // buffer height
	GFX_COLOR_MODE_${MODE}, // image color mode
	GFXU_IMAGE_COMPRESSION_${COMPTYPE}, // image compression type (raw only)
	${FLAGS}, // image flags
	${MASK_COLOR}, // image mask color
<#if PALETTE?has_content>
	&${PALETTE} // image palette
<#else>
	GFX_NULL // no palette
</#if>
};
</#macro>
<#macro GAC_PALETTE_ASSET_STRUCT
	NAME
	ADDRESS
	LOCATION_ID
	SIZE
	COUNT
	BPP
	FORMAT>
GFXU_PaletteAsset ${NAME} =
{
	<@GAC_ASSET_HEADER
		TYPE = "PALETTE"
		ADDRESS = ADDRESS
		LOCATION = LOCATION_ID
		SIZE = SIZE/>
	${COUNT}, // color count
	GFX_COLOR_MODE_${FORMAT} // image color mode
};
</#macro>
<#macro GAC_FONT_ASSET_RANGE_LOOKUP_TABLE
	NAME
	DATA
	SIZE>
uint8_t ${NAME}[${SIZE}] =
${DATA};

</#macro>
<#macro GAC_FONT_ASSET_INDEX_TABLE
	INDEX
	NAME
	RANGE_COUNT>
<#if RANGE_COUNT gt 0>
<#list 0..RANGE_COUNT-1 as i>
<@GAC_FONT_ASSET_RANGE_LOOKUP_TABLE
	NAME = "CONFIG_GAC_FONT_RANGE_LOOKUP_NAME_${INDEX}_${i}"?eval
	DATA = "CONFIG_GAC_FONT_RANGE_LOOKUP_DATA_${INDEX}_${i}"?eval
	SIZE = "CONFIG_GAC_FONT_RANGE_LOOKUP_DATA_SIZE_${INDEX}_${i}"?eval/>
</#list>

GFXU_FontGlyphIndexTable ${NAME}_index_table =
<#noparse>{</#noparse>
	${RANGE_COUNT}, // range count
<#noparse>    {</#noparse>
<#list 0..RANGE_COUNT-1 as i>
<@GAC_FONT_ASSET_INDEX_TABLE_ENTRY
	DESC = "CONFIG_GAC_FONT_RANGE_STRING_${INDEX}_${i}"?eval
	COUNT = "CONFIG_GAC_FONT_RANGE_GLYPH_COUNT_${INDEX}_${i}"?eval
	START = "CONFIG_GAC_FONT_RANGE_START_${INDEX}_${i}"?eval
	END = "CONFIG_GAC_FONT_RANGE_END_${INDEX}_${i}"?eval
	LOOKUP = "CONFIG_GAC_FONT_RANGE_LOOKUP_NAME_${INDEX}_${i}"?eval/>
</#list>
<#noparse>    }</#noparse>
<#noparse>};</#noparse>
</#if>
</#macro>
<#macro GAC_FONT_ASSET_INDEX_TABLE_ENTRY
	DESC
	COUNT
	START
	END
	LOOKUP>
	    /* ${DESC} */
<#noparse>        {</#noparse>
		    ${COUNT}, // glyph count
		    ${START}, // starting glyph id
		    ${END}, // ending glyph id
		    ${LOOKUP} // glyph lookup table
<#noparse>        },</#noparse>
</#macro>
<#macro GAC_FONT_ASSET_STRUCT
	NAME
	INDEX_TABLE
	ADDRESS
	LOCATION_ID
	SIZE
	HEIGHT
	ASCENT
	DESCENT
	BASELINE
	BPP>
GFXU_FontAsset ${NAME} =
{
	<@GAC_ASSET_HEADER
		TYPE = "FONT"
		ADDRESS = ADDRESS
		LOCATION = LOCATION_ID
		SIZE = SIZE/>
	${HEIGHT}, // font height
	${ASCENT}, // font max ascent
	${DESCENT}, // font max descent
	${BASELINE}, // font baseline
	GFXU_FONT_BPP_${BPP}, // bits per pixel
<#if INDEX_TABLE != "NULL">
	&${INDEX_TABLE} // glyph index table
<#else>
    ${INDEX_TABLE} // glyph index table
</#if>
};
</#macro>
<#macro GAC_BINARY_ASSET_STRUCT
	NAME
	ADDRESS
	LOCATION_ID
	SIZE>
GFXU_BinaryAsset ${NAME} =
{
	<@GAC_ASSET_HEADER
		TYPE = "BINARY"
		ADDRESS = ADDRESS
		LOCATION = LOCATION_ID
		SIZE = SIZE/>
};
</#macro>
<#macro GAC_STRING_FONT_INDEX_TABLE
	INDEX
	NAME
	RANGE_COUNT>
<#list 0..RANGE_COUNT-1 as i>
<@GAC_FONT_ASSET_RANGE_LOOKUP_TABLE
	NAME = "CONFIG_GAC_FONT_RANGE_LOOKUP_NAME_${INDEX}_${i}"?eval
	DATA = "CONFIG_GAC_FONT_RANGE_LOOKUP_DATA_${INDEX}_${i}"?eval
	SIZE = "CONFIG_GAC_FONT_RANGE_LOOKUP_DATA_SIZE_${INDEX}_${i}"?eval/>
</#list>
GFXU_FontGlyphIndexTable ${NAME}_index_table =
<#noparse>{</#noparse>
	${RANGE_COUNT}, // range count
<#noparse>    {</#noparse>
<#list 0..RANGE_COUNT-1 as i>
<@GAC_FONT_ASSET_INDEX_TABLE_ENTRY
	DESC = "CONFIG_GAC_FONT_RANGE_STRING_${INDEX}_${i}"?eval
	COUNT = "CONFIG_GAC_FONT_RANGE_GLYPH_COUNT_${INDEX}_${i}"?eval
	START = "CONFIG_GAC_FONT_RANGE_START_${INDEX}_${i}"?eval
	END = "CONFIG_GAC_FONT_RANGE_END_${INDEX}_${i}"?eval
	LOOKUP = "CONFIG_GAC_FONT_RANGE_LOOKUP_NAME_${INDEX}_${i}"?eval/>
</#list>
<#noparse>    }</#noparse>
<#noparse>};</#noparse>
</#macro>
<#macro GAC_FONT_ASSET_INDEX_TABLE_ENTRY
	DESC
	COUNT
	START
	END
	LOOKUP>
	    /* ${DESC} */
<#noparse>        {</#noparse>
		    ${COUNT}, // glyph count
		    ${START}, // starting glyph id
		    ${END}, // ending glyph id
		    ${LOOKUP} // glyph lookup table
<#noparse>        },</#noparse>
</#macro>
<#macro GAC_STRINGTABLE_ASSET_STRUCT
	NAME
	ADDRESS
	LOCATION_ID
	SIZE
	LANGUAGE_COUNT
	STRING_COUNT
	ENCODING>
GFXU_StringTableAsset ${NAME} =
{
	<@GAC_ASSET_HEADER
		TYPE = "STRINGTABLE"
		ADDRESS = ADDRESS
		LOCATION = LOCATION_ID
		SIZE = SIZE/>
	${LANGUAGE_COUNT}, // language count
	${STRING_COUNT}, // string count
	GFXU_STRING_ENCODING_${ENCODING} // encoding standard
};
</#macro>
/*******************************************************************************
  MPLAB Harmony Graphics Object Library Asset Source File

  File Name:
    ${CONFIG_GAC_INTERNAL_FILE_NAME}.c

  Summary:
    Source file containing asset data for use with the MPLAB Harmony Graphics
	Object Library.

  Description:
    Source file containing asset data for use with the MPLAB Harmony Graphics
	Object Library.

    Created with MPLAB Harmony Version ${CONFIG_MPLAB_HARMONY_VERSION_STRING}
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

#include "gfx/${CONFIG_GAC_INTERNAL_FILE_NAME}.h"

/*****************************************************************************
 Asset Manifest
 -------------------
 
<#if CONFIG_GAC_NUM_IMAGES != 0>
 Images
 -------
<#assign x=CONFIG_GAC_NUM_IMAGES>
<#list 0..x-1 as i>
	${"CONFIG_GAC_IMAGE_NAME_${i}"?eval}
</#list>

</#if>
<#if CONFIG_GAC_NUM_PALETTES != 0>
 Palettes
 -------
<#assign x=CONFIG_GAC_NUM_PALETTES>
<#list 0..x-1 as i>
	${"CONFIG_GAC_PALETTE_NAME_${i}"?eval}
</#list>

</#if>
<#if CONFIG_GAC_NUM_FONTS != 0>
 Fonts
 -------
<#assign x=CONFIG_GAC_NUM_FONTS>
<#list 0..x-1 as i>
    ${"CONFIG_GAC_FONT_NAME_${i}"?eval}
</#list>

</#if>
<#if CONFIG_GAC_NUM_BINARIES != 0>
 Binaries
 -------
<#assign x=CONFIG_GAC_NUM_BINARIES>
<#list 0..x-1 as i>
     ${"CONFIG_GAC_BINARY_NAME_${i}"?eval}
</#list>

</#if>

<#if CONFIG_GAC_STRING_TABLE_EXISTS??>
 String Table
 -------
    ${CONFIG_GAC_STRINGTABLE_NAME}
 
</#if>
 *****************************************************************************/

<#if CONFIG_GAC_NUM_IMAGES != 0>
/*****************************************************************************
 * SECTION:  Images
 *****************************************************************************/
<#assign x=CONFIG_GAC_NUM_IMAGES>
<#list 0..x-1 as i>
/****** ${"CONFIG_GAC_IMAGE_NAME_${i}"?eval} ******/
<#if "CONFIG_GAC_IMAGE_DATA_${i}"?eval?? = true>
    <@GAC_CONST_ASSET_DATA
		NAME = "CONFIG_GAC_IMAGE_NAME_${i}"?eval
		SIZE = "CONFIG_GAC_IMAGE_DATA_SIZE_${i}"?eval
		DESC = "CONFIG_GAC_IMAGE_DESC_${i}"?eval
		DATA = "CONFIG_GAC_IMAGE_DATA_${i}"?eval/>
		
</#if>	
<#if "CONFIG_GAC_IMAGE_ADDRESS_${i}"?eval?? = true>
	<@GAC_IMAGE_ASSET_STRUCT
		NAME = "CONFIG_GAC_IMAGE_NAME_${i}"?eval
		ADDRESS = "CONFIG_GAC_IMAGE_ADDRESS_${i}"?eval
		LOCATION_ID = "CONFIG_GAC_IMAGE_LOCATION_ID_${i}"?eval
		SIZE = "CONFIG_GAC_IMAGE_DATA_SIZE_${i}"?eval
		FORMAT = "CONFIG_GAC_IMAGE_FORMAT_${i}"?eval
		WIDTH = "CONFIG_GAC_IMAGE_WIDTH_${i}"?eval
		HEIGHT = "CONFIG_GAC_IMAGE_HEIGHT_${i}"?eval
		BPP = "CONFIG_GAC_IMAGE_BPP_${i}"?eval
		COMPTYPE = "CONFIG_GAC_IMAGE_COMPTYPE_${i}"?eval
		MODE = "CONFIG_GAC_IMAGE_MODE_${i}"?eval
		FLAGS = "CONFIG_GAC_IMAGE_FLAGS_${i}"?eval
		MASK_COLOR = "CONFIG_GAC_IMAGE_MASK_COLOR_${i}"?eval
		PALETTE = "CONFIG_GAC_IMAGE_PALETTE_${i}"?eval/>
		
</#if>	
</#list>
</#if>
<#if CONFIG_GAC_NUM_PALETTES != 0>
/*****************************************************************************
 * SECTION:  Palettes
 *****************************************************************************/
<#assign x=CONFIG_GAC_NUM_PALETTES>
<#list 0..x-1 as i>
/****** ${"CONFIG_GAC_PALETTE_NAME_${i}"?eval} ******/
<#if "CONFIG_GAC_PALETTE_DATA_${i}"?eval?? = true>
    <@GAC_ASSET_DATA
		NAME = "CONFIG_GAC_PALETTE_NAME_${i}"?eval
		SIZE = "CONFIG_GAC_PALETTE_DATA_SIZE_${i}"?eval
		DESC = "CONFIG_GAC_PALETTE_DESC_${i}"?eval
		DATA = "CONFIG_GAC_PALETTE_DATA_${i}"?eval/>
		
</#if>			
<#if "CONFIG_GAC_PALETTE_ADDRESS_${i}"?eval?? = true>
	<@GAC_PALETTE_ASSET_STRUCT
		NAME = "CONFIG_GAC_PALETTE_NAME_${i}"?eval
		ADDRESS = "CONFIG_GAC_PALETTE_ADDRESS_${i}"?eval
		LOCATION_ID = "CONFIG_GAC_PALETTE_LOCATION_ID_${i}"?eval
		SIZE = "CONFIG_GAC_PALETTE_SIZE_${i}"?eval
		COUNT = "CONFIG_GAC_PALETTE_COUNT_${i}"?eval
		BPP = "CONFIG_GAC_PALETTE_BPP_${i}"?eval
		FORMAT = "CONFIG_GAC_PALETTE_FORMAT_${i}"?eval/>

</#if>	
</#list>
</#if>
<#if CONFIG_GAC_NUM_FONTS != 0>
/*****************************************************************************
 * SECTION:  Fonts
 
 - font lookup table data description -
1 byte - size of the address offset values in this table, 1-4 possible
1 byte - size of the address width values in this table, 1-2 possible
  for each glyph entry in lookup table:
    1-4 bytes - glyph data offset in bytes
    1-2 bytes - glyph raster width in pixels	
 
 *****************************************************************************/
<#assign x=CONFIG_GAC_NUM_FONTS>
<#list 0..x-1 as i>
<#if "CONFIG_GAC_FONT_RANGE_COUNT_${i}"?eval gt 0>
    <@GAC_FONT_ASSET_INDEX_TABLE
		INDEX = i
		NAME = "CONFIG_GAC_FONT_NAME_${i}"?eval
		RANGE_COUNT = "CONFIG_GAC_FONT_RANGE_COUNT_${i}"?eval
		/>
</#if>
		
<#if "CONFIG_GAC_FONT_DATA_${i}"?eval??>
	<@GAC_CONST_ASSET_DATA
		NAME = "CONFIG_GAC_FONT_NAME_${i}"?eval
		SIZE = "CONFIG_GAC_FONT_DATA_SIZE_${i}"?eval
		DESC = "CONFIG_GAC_FONT_DESC_${i}"?eval
		DATA = "CONFIG_GAC_FONT_DATA_${i}"?eval/>
</#if>

	<@GAC_FONT_ASSET_STRUCT
		NAME = "CONFIG_GAC_FONT_NAME_${i}"?eval
		INDEX_TABLE = "CONFIG_GAC_FONT_INDEX_TABLE_${i}"?eval
		ADDRESS = "CONFIG_GAC_FONT_ADDRESS_${i}"?eval
		LOCATION_ID = "CONFIG_GAC_FONT_LOCATION_ID_${i}"?eval
		SIZE = "CONFIG_GAC_FONT_DATA_SIZE_${i}"?eval
		HEIGHT = "CONFIG_GAC_FONT_HEIGHT_${i}"?eval
		ASCENT = "CONFIG_GAC_FONT_ASCENT_${i}"?eval
		DESCENT = "CONFIG_GAC_FONT_DESCENT_${i}"?eval
		BASELINE = "CONFIG_GAC_FONT_BASELINE_${i}"?eval
		BPP = "CONFIG_GAC_FONT_BPP_${i}"?eval/>
		
</#list>
</#if>

<#if CONFIG_GAC_NUM_BINARIES != 0>
/*****************************************************************************
 * SECTION:  Binaries
 *****************************************************************************/
<#assign x=CONFIG_GAC_NUM_BINARIES>
<#list 0..x-1 as i>
<#if "CONFIG_GAC_BINARY_DATA_${i}"?eval??>
	<@GAC_CONST_ASSET_DATA
		NAME = "CONFIG_GAC_BINARY_NAME_${i}"?eval
		SIZE = "CONFIG_GAC_BINARY_SIZE_${i}"?eval
		DESC = "CONFIG_GAC_BINARY_DESC_${i}"?eval
		DATA = "CONFIG_GAC_BINARY_DATA_${i}"?eval/>
</#if>
		
    <@GAC_BINARY_ASSET_STRUCT
		NAME = "CONFIG_GAC_BINARY_NAME_${i}"?eval
		ADDRESS = "CONFIG_GAC_BINARY_ADDRESS_${i}"?eval
		LOCATION_ID = "CONFIG_GAC_BINARY_LOCATION_ID_${i}"?eval
		SIZE = "CONFIG_GAC_BINARY_SIZE_${i}"?eval/>
		
</#list>
</#if>

<#if CONFIG_GAC_STRING_TABLE_EXISTS??>
/*****************************************************************************
 * SECTION:  Strings
 
 - String table data format description -

 2 bytes - number of unique string values
 
 for each string value:
   2 bytes - size of the string in bytes
   n bytes - character code point data, 1-4 bytes each character per encoding	 
 *****************************************************************************/
<#if CONFIG_GAC_STRINGTABLE_DATA??>
    <@GAC_ASSET_DATA
		NAME = CONFIG_GAC_STRINGTABLE_NAME
		SIZE = CONFIG_GAC_STRINGTABLE_DATA_SIZE
		DESC = CONFIG_GAC_STRINGTABLE_DESC
		DATA = CONFIG_GAC_STRINGTABLE_DATA/>
	
</#if>
/*****************************************************************************
  - String index table data format description -

 2 bytes - number of strings in the string table
 1 byte - number of languages in the string table
 1 byte - size of the string indicies, equals 2 if there are more than 254
          strings
		  
 for each string in table:
    for each language in table:
	    1-2 bytes - string data table entry	 
 *****************************************************************************/
<#if CONFIG_GAC_FONTINDEX_DATA??>
    <@GAC_ASSET_DATA
		NAME = "stringIndexTable"
		SIZE = CONFIG_GAC_STRINGINDEX_DATA_SIZE
		DESC = "Lookup table for associating string and language IDs to string data."
		DATA = CONFIG_GAC_STRINGINDEX_DATA/>
	
</#if>
<#if CONFIG_GAC_NUM_FONTS != 0>
GFXU_FontAsset* fontList[${CONFIG_GAC_NUM_FONTS}] =
{
<#assign x=CONFIG_GAC_NUM_FONTS>
<#list 0..x-1 as i>
	&${"CONFIG_GAC_FONT_ID_${i}"?eval},
</#list>
};

</#if>
/*****************************************************************************
  - Font index table data format description -

 2 bytes - number of strings in the string table
 1 byte - number of languages in the string table
		  
 for each string in table:
    for each language in table:
	    1 byte - the font to use for the string	 
		
 id = 0xFF if no font association
 *****************************************************************************/
<#if CONFIG_GAC_FONTINDEX_DATA??>
    <@GAC_ASSET_DATA
		NAME = "fontIndexTable"
		SIZE = CONFIG_GAC_FONTINDEX_DATA_SIZE
		DESC = "Lookup table for associating strings, languages, and fonts"
		DATA = CONFIG_GAC_FONTINDEX_DATA/>
	
</#if>	
GFXU_StringTableAsset ${CONFIG_GAC_STRINGTABLE_NAME} =
{
	<@GAC_ASSET_HEADER
		TYPE = "STRINGTABLE"
		ADDRESS = CONFIG_GAC_STRINGTABLE_ADDRESS
		LOCATION = CONFIG_GAC_STRINGTABLE_LOCATION_ID
		SIZE = CONFIG_GAC_STRINGTABLE_DATA_SIZE/>
	${CONFIG_GAC_STRINGTABLE_LANGUAGECOUNT}, // language count
	${CONFIG_GAC_STRINGTABLE_COUNT}, // string count
<#if CONFIG_GAC_STRINGTABLE_DATA??>
    stringIndexTable_data, // font lookup table
<#else>
	GFX_NULL, // no font lookup table
</#if>
<#if CONFIG_GAC_NUM_FONTS != 0>
    fontList, // font lookup table
<#else>
	GFX_NULL, // no font lookup table
</#if>
<#if CONFIG_GAC_FONTINDEX_DATA??>
    fontIndexTable_data, // font index table
<#else>
	GFX_NULL, // no font index data
</#if>
	GFXU_STRING_ENCODING_${CONFIG_GAC_STRINGTABLE_ENCODING} // encoding standard
};
		
</#if>

<#if CONFIG_GAC_GLOBAL_PALETTE_EXISTS??>
/*****************************************************************************
  Global Color Lookup Table
 *****************************************************************************/
<#if CONFIG_GAC_GLOBAL_PALETTE_SIZE = "1">
uint8_t globalColorPalette[256] =
<#elseif CONFIG_GAC_GLOBAL_PALETTE_SIZE = "2">
uint16_t globalColorPalette[256] =
<#elseif CONFIG_GAC_GLOBAL_PALETTE_SIZE = "4">
uint32_t globalColorPalette[256] =
</#if>
{
<#if CONFIG_GAC_GLOBAL_PALETTE_SIZE = "1" ||  CONFIG_GAC_GLOBAL_PALETTE_SIZE = "2">
<#list 0..31 as i>
<#assign idx = (i * 8)>
	0x${"CONFIG_GAC_GLOBAL_PALETTE_VAL_${idx}"?eval}, 0x${"CONFIG_GAC_GLOBAL_PALETTE_VAL_${idx+1}"?eval}, 0x${"CONFIG_GAC_GLOBAL_PALETTE_VAL_${idx+2}"?eval}, 0x${"CONFIG_GAC_GLOBAL_PALETTE_VAL_${idx+3}"?eval}, 0x${"CONFIG_GAC_GLOBAL_PALETTE_VAL_${idx+4}"?eval}, 0x${"CONFIG_GAC_GLOBAL_PALETTE_VAL_${idx+5}"?eval}, 0x${"CONFIG_GAC_GLOBAL_PALETTE_VAL_${idx+6}"?eval}, 0x${"CONFIG_GAC_GLOBAL_PALETTE_VAL_${idx+7}"?eval},
</#list>
<#else>
<#list 0..63 as i>
<#assign idx = (i * 4)>
	0x${"CONFIG_GAC_GLOBAL_PALETTE_VAL_${idx}"?eval}, 0x${"CONFIG_GAC_GLOBAL_PALETTE_VAL_${idx+1}"?eval}, 0x${"CONFIG_GAC_GLOBAL_PALETTE_VAL_${idx+2}"?eval}, 0x${"CONFIG_GAC_GLOBAL_PALETTE_VAL_${idx+3}"?eval},
</#list>
</#if>
};
</#if>