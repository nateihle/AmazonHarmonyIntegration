<#--
/*******************************************************************************
Copyright (c) 2016 released Microchip Technology Inc.  All rights reserved.

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

/*** AR1021 Driver Initialization Data ***/

<#if CONFIG_USE_DRV_TOUCH_AR1021 == true>
const DRV_TOUCH_INIT drvAr1021InitData =
{
    .moduleInit                  = {0},
    .touchId                     = DRV_TOUCH_INDEX_0,
    .drvInitialize               = NULL,
    .drvOpen                     = NULL,
<#if CONFIG_USE_DRV_GFX_DISPLAY == true>
    .orientation                 = DISP_ORIENTATION,
    .horizontalResolution        = DISP_HOR_RESOLUTION,
    .verticalResolution          = DISP_VER_RESOLUTION,
</#if>
    .pReadFunc                   = NULL,
    .pWriteFunc                  = NULL,
    .pSectorErase                = NULL,
};
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
