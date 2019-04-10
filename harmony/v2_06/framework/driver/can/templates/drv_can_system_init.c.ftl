<#--
/*******************************************************************************
  CAN Driver Freemarker Template File

  Company:
    Microchip Technology Inc.

  File Name:
    drv_can_system_init.c.ftl

  Summary:
    CAN Driver Freemarker Template File

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
<#macro DRV_CAN_STATIC_API DRV_INSTANCE>
    /* Initialize CAN Driver ${DRV_INSTANCE} */
    DRV_CAN${DRV_INSTANCE}_Initialize();

</#macro>
<#if CONFIG_DRV_CAN_INST_IDX0!false>
<@DRV_CAN_STATIC_API DRV_INSTANCE="0"/>
</#if>
<#if CONFIG_DRV_CAN_INST_IDX1!false>
<@DRV_CAN_STATIC_API DRV_INSTANCE="1"/>
</#if>
<#if CONFIG_DRV_CAN_INST_IDX2!false>
<@DRV_CAN_STATIC_API DRV_INSTANCE="2"/>
</#if>
<#if CONFIG_DRV_CAN_INST_IDX3!false>
<@DRV_CAN_STATIC_API DRV_INSTANCE="3"/>
</#if>
<#--
/*******************************************************************************
 End of File
*/
-->
