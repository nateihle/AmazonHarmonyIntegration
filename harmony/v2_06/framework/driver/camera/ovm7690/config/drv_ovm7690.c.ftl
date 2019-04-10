<#--
/*******************************************************************************
Copyright (c) 2015 released Microchip Technology Inc.  All rights reserved.

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
// <editor-fold defaultstate="collapsed" desc="DRV_OVM7690 Configuration">
/*** OVM7690 Driver Initialization Data ***/

<#if CONFIG_USE_DRV_OVM7690 == true>
const DRV_CAMERA_OVM7690_INIT drvCameraInit =
{
    .cameraID                = CAMERA_MODULE_OVM7690,
    .sourcePort              = (void *)&PORTK,
    .hsyncChannel            = PORT_CHANNEL_A,
    .hsyncPosition           = PORTS_BIT_POS_1,
    .vsyncChannel            = PORT_CHANNEL_J,
    .vsyncPosition           = PORTS_BIT_POS_2,
    .hsyncInterruptSource    = INT_SOURCE_CHANGE_NOTICE_A,
    .vsyncInterruptSource    = INT_SOURCE_CHANGE_NOTICE_J,
    .dmaChannel              = DRV_CAMERA_OVM7690_DMA_CHANNEL_INDEX,
    .dmaTriggerSource        = DMA_TRIGGER_EXTERNAL_2,
    .bpp                     = GFX_CONFIG_COLOR_DEPTH,
};
</#if>
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
