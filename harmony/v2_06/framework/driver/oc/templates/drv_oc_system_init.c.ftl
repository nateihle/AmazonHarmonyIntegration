<#--
/*******************************************************************************
  OC Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_oc_system_init.c.ftl

  Summary:
    OC Driver Freemarker Template File

 *******************************************************************************/

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
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTOCULAR PURPOSE.
IN NO EVENT SHALL MOCROCHIP OR ITS LOCENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STROCT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVOCES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
 -->
<#if CONFIG_USE_DRV_OC == true>
    /* Initialize the OC Driver */
</#if>
<#if CONFIG_DRV_OC_INST_IDX0 == true>
    DRV_OC0_Initialize();
</#if>
<#if CONFIG_DRV_OC_INST_IDX1 == true>
    DRV_OC1_Initialize();
</#if>
<#if CONFIG_DRV_OC_INST_IDX2 == true>
    DRV_OC2_Initialize();
</#if>
<#if CONFIG_DRV_OC_INST_IDX3 == true>
    DRV_OC3_Initialize();
</#if>
<#if CONFIG_DRV_OC_INST_IDX4 == true>
    DRV_OC4_Initialize();
</#if>
<#if CONFIG_DRV_OC_INST_IDX5 == true>
    DRV_OC5_Initialize();
</#if>
<#if CONFIG_DRV_OC_INST_IDX6 == true>
    DRV_OC6_Initialize();
</#if>
<#if CONFIG_DRV_OC_INST_IDX7 == true>
    DRV_OC7_Initialize();
</#if>
<#if CONFIG_DRV_OC_INST_IDX8 == true>
    DRV_OC8_Initialize();
</#if>
<#if CONFIG_DRV_OC_INST_IDX9 == true>
    DRV_OC9_Initialize();
</#if>
<#if CONFIG_DRV_OC_INST_IDX10 == true>
    DRV_OC10_Initialize();
</#if>
<#if CONFIG_DRV_OC_INST_IDX11 == true>
    DRV_OC11_Initialize();
</#if>
<#if CONFIG_DRV_OC_INST_IDX12 == true>
    DRV_OC12_Initialize();
</#if>
<#if CONFIG_DRV_OC_INST_IDX13 == true>
    DRV_OC13_Initialize();
</#if>
<#if CONFIG_DRV_OC_INST_IDX14 == true>
    DRV_OC14_Initialize();
</#if>
<#if CONFIG_DRV_OC_INST_IDX15 == true>
    DRV_OC15_Initialize();
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
