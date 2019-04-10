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
// <editor-fold defaultstate="collapsed" desc="SYS_TOUCH Initialization Data">
// *****************************************************************************
/* System Touch Initialization Data
*/

const DRV_TOUCH_INIT sysTouchInit0 =
{
<#if CONFIG_USE_DRV_TOUCH_MTCH6301 == true>
    .drvInitialize           = DRV_TOUCH_MTCH6301_Initialize,
    .drvOpen                 = DRV_TOUCH_MTCH6301_Open,
    .drvTouchStatus          = DRV_TOUCH_MTCH6301_TouchStatus,
    .drvTouchDataRead        = DRV_TOUCH_MTCH6301_TouchDataRead,
    .drvTouchGetX            = DRV_TOUCH_MTCH6301_TouchGetX,
    .drvTouchGetY            = DRV_TOUCH_MTCH6301_TouchGetY,
    .drvTouchPenGet          = NULL,
<#elseif CONFIG_USE_DRV_TOUCH_MTCH6303 == true>
    .drvInitialize           = DRV_TOUCH_MTCH6303_Initialize,
    .drvOpen                 = DRV_TOUCH_MTCH6303_Open,
    .drvTouchStatus          = DRV_TOUCH_MTCH6303_TouchStatus,
    .drvTouchDataRead        = DRV_TOUCH_MTCH6303_TouchDataRead,
    .drvTouchGetX            = DRV_TOUCH_MTCH6303_TouchGetX,
    .drvTouchGetY            = DRV_TOUCH_MTCH6303_TouchGetY,
    .drvTouchPenGet          = NULL,
<#elseif CONFIG_USE_DRV_TOUCH_ADC10BIT == true>
    .drvInitialize           = DRV_TOUCH_ADC10BIT_Initialize,
    .drvOpen                 = DRV_TOUCH_ADC10BIT_Open,
    .drvTouchStatus          = DRV_TOUCH_ADC10BIT_TouchStatus,
    .drvTouchDataRead        = DRV_TOUCH_ADC10BIT_TouchDataRead,
    .drvCalibrationSet       = DRV_TOUCH_ADC10BIT_CalibrationSet,
    .drvTouchGetX            = DRV_TOUCH_ADC10BIT_TouchGetX,
    .drvTouchGetY            = DRV_TOUCH_ADC10BIT_TouchGetY,
    .drvTouchPenGet          = NULL,
<#elseif CONFIG_USE_DRV_TOUCH_AR1021 == true>
    .drvInitialize           = DRV_TOUCH_AR1021_Initialize,
    .drvOpen                 = DRV_TOUCH_AR1021_Open,
    .drvTouchStatus          = DRV_TOUCH_AR1021_TouchStatus,
    .drvTouchDataRead        = DRV_TOUCH_AR1021_TouchDataRead,
    .drvCalibrationSet       = DRV_TOUCH_AR1021_CalibrationSet,
    .drvTouchGetX            = DRV_TOUCH_AR1021_TouchGetX,
    .drvTouchGetY            = DRV_TOUCH_AR1021_TouchGetY,
    .drvTouchPenGet          = DRV_TOUCH_AR1021_TouchPenGet,
<#elseif CONFIG_USE_DRV_TOUCH_ADC == true>
    .drvInitialize           = DRV_TOUCH_ADC_Initialize,
    .drvOpen                 = DRV_TOUCH_ADC_Open,
    .drvTouchStatus          = DRV_TOUCH_ADC_TouchStatus,
    .drvTouchDataRead        = DRV_TOUCH_ADC_TouchDataRead,
    .drvCalibrationSet       = DRV_TOUCH_ADC_CalibrationSet,
    .drvTouchGetX            = DRV_TOUCH_ADC_TouchGetX,
    .drvTouchGetY            = DRV_TOUCH_ADC_TouchGetY,
    .drvTouchPenGet          = 0,
<#elseif CONFIG_USE_DRV_TOUCH_MXT336T == true>
    .drvInitialize           = DRV_MXT_Initialize,
    .drvOpen                 = DRV_MXT_Open,
    .drvTouchStatus          = DRV_MXT_TouchStatus,
    .drvTouchDataRead        = DRV_MXT_TouchDataRead,
    .drvTouchGetX            = DRV_MXT_TouchGetX,
    .drvTouchGetY            = DRV_MXT_TouchGetY,
    .drvTouchPenGet          = NULL,
	<#elseif CONFIG_USE_DRV_TOUCH_MXT336T_UPDATED == true>
    .drvInitialize           = DRV_MXT_Initialize,
    .drvOpen                 = DRV_MXT_Open,
    .drvTouchStatus          = DRV_MXT_TouchStatus,
    .drvTouchDataRead        = DRV_MXT_TouchDataRead,
    .drvTouchGetX            = DRV_MXT_TouchGetX,
    .drvTouchGetY            = DRV_MXT_TouchGetY,
    .drvTouchPenGet          = NULL,
<#elseif CONFIG_USE_DRV_TOUCH_GENERIC == true>
    .drvInitialize           = DRV_TOUCH_GENERIC_Initialize,
    .drvOpen                 = DRV_TOUCH_GENERIC_Open,
    .drvTouchStatus          = DRV_TOUCH_GENERIC_TouchStatus,
    .drvTouchDataRead        = DRV_TOUCH_GENERIC_TouchDataRead,
    .drvTouchGetX            = DRV_TOUCH_GENERIC_TouchGetX,
    .drvTouchGetY            = DRV_TOUCH_GENERIC_TouchGetY,
    .drvTouchPenGet          = NULL,
</#if>
    .orientation             = ${CONFIG_DRV_GFX_DISPLAY_ORIENTATION},
    .horizontalResolution    = ${CONFIG_DRV_GFX_DISPLAY_WIDTH},
    .verticalResolution      = ${CONFIG_DRV_GFX_DISPLAY_HEIGHT},
	.minTouchDetectDelta     = ${CONFIG_SYS_TOUCH_SENSITIVITY_DELTA},
};
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->

