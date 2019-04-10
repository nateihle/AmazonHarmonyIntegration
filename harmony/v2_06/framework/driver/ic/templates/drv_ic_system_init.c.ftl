<#--
/*******************************************************************************
  IC Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_ic_system_init.c.ftl

  Summary:
    IC Driver Freemarker Template File

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
<#if CONFIG_USE_DRV_IC == true>
    /* Initialize the IC Driver */
</#if>
<#if CONFIG_DRV_IC_INST_IDX0 == true>
    DRV_IC0_Initialize();
</#if>
<#if CONFIG_DRV_IC_INST_IDX1 == true>
    DRV_IC1_Initialize();
</#if>
<#if CONFIG_DRV_IC_INST_IDX2 == true>
    DRV_IC2_Initialize();
</#if>
<#if CONFIG_DRV_IC_INST_IDX3 == true>
    DRV_IC3_Initialize();
</#if>
<#if CONFIG_DRV_IC_INST_IDX4 == true>
    DRV_IC4_Initialize();
</#if>
<#if CONFIG_DRV_IC_INST_IDX5 == true>
    DRV_IC5_Initialize();
</#if>
<#if CONFIG_DRV_IC_INST_IDX6 == true>
    DRV_IC6_Initialize();
</#if>
<#if CONFIG_DRV_IC_INST_IDX7 == true>
    DRV_IC7_Initialize();
</#if>
<#if CONFIG_DRV_IC_INST_IDX8 == true>
    DRV_IC8_Initialize();
</#if>
<#if CONFIG_DRV_IC_INST_IDX9 == true>
    DRV_IC9_Initialize();
</#if>
<#if CONFIG_DRV_IC_INST_IDX10 == true>
    DRV_IC10_Initialize();
</#if>
<#if CONFIG_DRV_IC_INST_IDX11 == true>
    DRV_IC11_Initialize();
</#if>
<#if CONFIG_DRV_IC_INST_IDX12 == true>
    DRV_IC12_Initialize();
</#if>
<#if CONFIG_DRV_IC_INST_IDX13 == true>
    DRV_IC13_Initialize();
</#if>
<#if CONFIG_DRV_IC_INST_IDX14 == true>
    DRV_IC14_Initialize();
</#if>
<#if CONFIG_DRV_IC_INST_IDX15 == true>
    DRV_IC15_Initialize();
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
