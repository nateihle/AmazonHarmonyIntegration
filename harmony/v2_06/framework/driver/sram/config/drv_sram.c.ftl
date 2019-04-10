<#--
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
 -->

/*** SRAM Driver Initialization Data ***/
<#if CONFIG_USE_DRV_SRAM == true>
<#if CONFIG_DRV_SRAM_INST_IDX0 == true>
SYS_FS_MEDIA_REGION_GEOMETRY sramMedia0GeometryTable[3] = 
{
    {
        .blockSize = ${CONFIG_DRV_SRAM_MEDIA_BLOCK_SIZE_IDX0},
        .numBlocks = (${CONFIG_DRV_SRAM_MEDIA_SIZE_IDX0} * (1024/${CONFIG_DRV_SRAM_MEDIA_BLOCK_SIZE_IDX0})),
    },
    {
       .blockSize = ${CONFIG_DRV_SRAM_MEDIA_BLOCK_SIZE_IDX0},
       .numBlocks = (${CONFIG_DRV_SRAM_MEDIA_SIZE_IDX0} * (1024/${CONFIG_DRV_SRAM_MEDIA_BLOCK_SIZE_IDX0})),
    },
    {
       .blockSize = ${CONFIG_DRV_SRAM_MEDIA_BLOCK_SIZE_IDX0},
       .numBlocks = (${CONFIG_DRV_SRAM_MEDIA_SIZE_IDX0} * (1024/${CONFIG_DRV_SRAM_MEDIA_BLOCK_SIZE_IDX0})),
    }
};

const SYS_FS_MEDIA_GEOMETRY sramMedia0Geometry = 
{
    .mediaProperty = SYS_FS_MEDIA_WRITE_IS_BLOCKING | SYS_FS_MEDIA_READ_IS_BLOCKING,
    .numReadRegions = 1,
    .numWriteRegions = 1,
    .numEraseRegions = 1,
    .geometryTable = (SYS_FS_MEDIA_REGION_GEOMETRY *)&sramMedia0GeometryTable
};

extern uint8_t ${CONFIG_DRV_SRAM_MEDIA_START_ADDRESS_IDX0}[];
const DRV_SRAM_INIT drvSram0Init =
{
<#if CONFIG_USE_DRV_SRAM_SYS_FS_REGISTER_IDX0 == true>
    .registerWithFs = true,
<#else>
    .registerWithFs = false,
</#if>
    .mediaStartAddress = (uint8_t *)${CONFIG_DRV_SRAM_MEDIA_START_ADDRESS_IDX0},
    .sramMediaGeometry = (SYS_FS_MEDIA_GEOMETRY *)&sramMedia0Geometry
};
</#if>

<#if CONFIG_DRV_SRAM_INST_IDX1 == true>
SYS_FS_MEDIA_REGION_GEOMETRY sramMedia1GeometryTable[3] = 
{
    {
        .blockSize = ${CONFIG_DRV_SRAM_MEDIA_BLOCK_SIZE_IDX1},
        .numBlocks = (${CONFIG_DRV_SRAM_MEDIA_SIZE_IDX1} * (1024/${CONFIG_DRV_SRAM_MEDIA_BLOCK_SIZE_IDX1})),
    },
    {
       .blockSize = ${CONFIG_DRV_SRAM_MEDIA_BLOCK_SIZE_IDX1},
       .numBlocks = (${CONFIG_DRV_SRAM_MEDIA_SIZE_IDX1} * (1024/${CONFIG_DRV_SRAM_MEDIA_BLOCK_SIZE_IDX1})),
    },
    {
       .blockSize = ${CONFIG_DRV_SRAM_MEDIA_BLOCK_SIZE_IDX1},
       .numBlocks = (${CONFIG_DRV_SRAM_MEDIA_SIZE_IDX1} * (1024/${CONFIG_DRV_SRAM_MEDIA_BLOCK_SIZE_IDX1}))
    }
};

const SYS_FS_MEDIA_GEOMETRY sramMedia1Geometry = 
{
    .mediaProperty = SYS_FS_MEDIA_WRITE_IS_BLOCKING | SYS_FS_MEDIA_READ_IS_BLOCKING,
    .numReadRegions = 1,
    .numWriteRegions = 1,
    .numEraseRegions = 1,
    .geometryTable = (SYS_FS_MEDIA_REGION_GEOMETRY *)&sramMedia1GeometryTable
};

extern uint8_t ${CONFIG_DRV_SRAM_MEDIA_START_ADDRESS_IDX1}[];
const DRV_SRAM_INIT drvSram1Init =
{
<#if CONFIG_USE_DRV_SRAM_SYS_FS_REGISTER_IDX1 == true>
    .registerWithFs = true,
<#else>
    .registerWithFs = false,
</#if>
    .mediaStartAddress = (uint8_t *)${CONFIG_DRV_SRAM_MEDIA_START_ADDRESS_IDX1},
    .sramMediaGeometry = (SYS_FS_MEDIA_GEOMETRY *)&sramMedia1Geometry
};
</#if>

</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
