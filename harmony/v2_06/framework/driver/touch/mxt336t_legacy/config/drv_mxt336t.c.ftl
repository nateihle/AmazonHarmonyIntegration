<#--
/*******************************************************************************
Copyright (c) 2014 released Microchip Technology Inc.  All rights reserved.

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
// <editor-fold defaultstate="collapsed" desc="DRV_TOUCH_MXT336T Initialization Data">
/*** MXT336T Driver Initialization Data ***/

<#if CONFIG_USE_DRV_TOUCH_MXT336T == true>
const DRV_MXT336T_INIT drvTouchInitData =
{
    .moduleInit                  = {0},
    .touchId                     = DRV_TOUCH_INDEX_0,
    .drvInitialize               = NULL,
    .drvOpen                     = DRV_I2C_Open,
<#if CONFIG_USE_GFX_STACK == true>
    .orientation                 = ${CONFIG_DRV_GFX_DISPLAY_ORIENTATION},
    .horizontalResolution        = ${CONFIG_DRV_GFX_DISPLAY_WIDTH},
    .verticalResolution          = ${CONFIG_DRV_GFX_DISPLAY_HEIGHT},
</#if>
<#if CONFIG_DRV_TOUCH_MXT336T_INTERRUPT_TYPE == "EXTERNAL INTERRUPT">
    .interruptSource             = ${CONFIG_DRV_TOUCH_MXT336T_INTERRUPT_SOURCE},
<#else>
	.interruptSource             = ${CONFIG_DRV_TOUCH_MXT336T_INTERRUPT_GPIO},
</#if>	

    .interruptChannel            = ${CONFIG_DRV_TOUCH_MXT336T_CHANGE_PIN_CHANNEL},
    .interruptPin                = ${CONFIG_DRV_TOUCH_MXT336T_CHANGE_PIN_NUMBER},
};


const DRV_MXT_INIT drvMxtInitData =
{
    .moduleInit                  = {0},
    .mxtId                     	 = DRV_MXT_INDEX_0,
    .drvInitialize               = NULL,
<#if CONFIG_USE_GFX_STACK == true>
    .orientation                 = ${CONFIG_DRV_GFX_DISPLAY_ORIENTATION},
    .horizontalResolution        = ${CONFIG_DRV_GFX_DISPLAY_WIDTH},
    .verticalResolution          = ${CONFIG_DRV_GFX_DISPLAY_HEIGHT},
</#if>
};

</#if>
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
