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
// <editor-fold defaultstate="collapsed" desc="DRV_S25FL Initialization Data">
/*** S25FL FLASH Driver Initialization Data ***/
<#if CONFIG_USE_DRV_S25FL == true>
<#if CONFIG_DRV_S25FL_SQI_DEVICE_IDX0?has_content>
const DRV_S25FL_INIT drvS25flInitData0 =
{
    .sqiDevice = ${CONFIG_DRV_S25FL_SQI_DEVICE_IDX0},
};
</#if>
<#if CONFIG_DRV_S25FL_SQI_DEVICE_IDX1?has_content>
const DRV_S25FL_INIT drvS25flInitData1 =
{
    .sqiDevice = ${CONFIG_DRV_S25FL_SQI_DEVICE_IDX1},
};
</#if>
</#if>
// </editor-fold>
<#--
/*******************************************************************************
 End of File
*/
-->
