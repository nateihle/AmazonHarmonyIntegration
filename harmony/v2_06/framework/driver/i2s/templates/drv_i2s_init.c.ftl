<#--
/*******************************************************************************
  I2S Driver Initialization File

  File Name:
    drv_i2s_init.c

  Summary:
    This file contains source code necessary to initialize the system.

  Description:
    This file contains source code necessary to initialize the system.  It
    implements the "SYS_Initialize" function, configuration bits, and allocates
    any necessary global system resources, such as the systemObjects structure
    that contains the object handles to all the MPLAB Harmony module objects in
    the system.
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
 <#if CONFIG_DRV_I2S_INST_IDX0 == true>
    sysObj.drvI2S0 = DRV_I2S_Initialize(DRV_I2S_INDEX_0, (SYS_MODULE_INIT *)&drvI2S0InitData);
</#if>
<#if CONFIG_DRV_I2S_INST_IDX1 == true>
    sysObj.drvI2S1 = DRV_I2S_Initialize(DRV_I2S_INDEX_1, (SYS_MODULE_INIT *)&drvI2S1InitData);
</#if>
<#if CONFIG_DRV_I2S_INST_IDX2 == true>
    sysObj.drvI2S2 = DRV_I2S_Initialize(DRV_I2S_INDEX_2, (SYS_MODULE_INIT *)&drvI2S2InitData);
</#if>
<#if CONFIG_DRV_I2S_INST_IDX3 == true>
    sysObj.drvI2S3 = DRV_I2S_Initialize(DRV_I2S_INDEX_3, (SYS_MODULE_INIT *)&drvI2S3InitData);
</#if>
<#if CONFIG_DRV_I2S_INST_IDX4 == true>
    sysObj.drvI2S4 = DRV_I2S_Initialize(DRV_I2S_INDEX_4, (SYS_MODULE_INIT *)&drvI2S4InitData);
</#if>
<#if CONFIG_DRV_I2S_INST_IDX5 == true>
    sysObj.drvI2S5 = DRV_I2S_Initialize(DRV_I2S_INDEX_5, (SYS_MODULE_INIT *)&drvI2S5InitData);
</#if>

<#if CONFIG_USE_SYS_INT == true && CONFIG_DRV_I2S_INTERRUPT_MODE == true>
<#if CONFIG_DRV_I2S_INST_IDX0 == true>
<#if CONFIG_PIC32MX == true>
    SYS_INT_VectorPrioritySet(DRV_I2S_INT_VECTOR_IDX0, DRV_I2S_INT_PRIORITY_IDX0);
    SYS_INT_VectorSubprioritySet(DRV_I2S_INT_VECTOR_IDX0, DRV_I2S_INT_SUB_PRIORITY_IDX0);
</#if>
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_TX_INT_SRC_IDX0}, ${CONFIG_DRV_I2S_TX_INT_PRIORITY_IDX0});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_TX_INT_SRC_IDX0}, ${CONFIG_DRV_I2S_TX_INT_SUB_PRIORITY_IDX0});
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_RX_INT_SRC_IDX0}, ${CONFIG_DRV_I2S_RX_INT_PRIORITY_IDX0});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_RX_INT_SRC_IDX0}, ${CONFIG_DRV_I2S_RX_INT_SUB_PRIORITY_IDX0});
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_ERR_INT_SRC_IDX0}, ${CONFIG_DRV_I2S_ERR_INT_PRIORITY_IDX0});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_ERR_INT_SRC_IDX0}, ${CONFIG_DRV_I2S_ERR_INT_SUB_PRIORITY_IDX0});
</#if>
</#if>
<#if CONFIG_DRV_I2S_INST_IDX1 == true>
<#if CONFIG_PIC32MX == true>
    SYS_INT_VectorPrioritySet(DRV_I2S_INT_VECTOR_IDX1, DRV_I2S_INT_PRIORITY_IDX1);
    SYS_INT_VectorSubprioritySet(DRV_I2S_INT_VECTOR_IDX1, DRV_I2S_INT_SUB_PRIORITY_IDX1);
</#if>
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_TX_INT_SRC_IDX1}, ${CONFIG_DRV_I2S_TX_INT_PRIORITY_IDX1});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_TX_INT_SRC_IDX1}, ${CONFIG_DRV_I2S_TX_INT_SUB_PRIORITY_IDX1});
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_RX_INT_SRC_IDX1}, ${CONFIG_DRV_I2S_RX_INT_PRIORITY_IDX1});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_RX_INT_SRC_IDX1}, ${CONFIG_DRV_I2S_RX_INT_SUB_PRIORITY_IDX1});
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_ERR_INT_SRC_IDX1}, ${CONFIG_DRV_I2S_ERR_INT_PRIORITY_IDX1});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_ERR_INT_SRC_IDX1}, ${CONFIG_DRV_I2S_ERR_INT_SUB_PRIORITY_IDX1});
</#if>
</#if>
<#if CONFIG_DRV_I2S_INST_IDX2 == true>
<#if CONFIG_PIC32MX == true>
    SYS_INT_VectorPrioritySet(DRV_I2S_INT_VECTOR_IDX2, DRV_I2S_INT_PRIORITY_IDX2);
    SYS_INT_VectorSubprioritySet(DRV_I2S_INT_VECTOR_IDX2, DRV_I2S_INT_SUB_PRIORITY_IDX2);
</#if>
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_TX_INT_SRC_IDX2}, ${CONFIG_DRV_I2S_TX_INT_PRIORITY_IDX2});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_TX_INT_SRC_IDX2}, ${CONFIG_DRV_I2S_TX_INT_SUB_PRIORITY_IDX2});
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_RX_INT_SRC_IDX2}, ${CONFIG_DRV_I2S_RX_INT_PRIORITY_IDX2});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_RX_INT_SRC_IDX2}, ${CONFIG_DRV_I2S_RX_INT_SUB_PRIORITY_IDX2});
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_ERR_INT_SRC_IDX2}, ${CONFIG_DRV_I2S_ERR_INT_PRIORITY_IDX2});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_ERR_INT_SRC_IDX2}, ${CONFIG_DRV_I2S_ERR_INT_SUB_PRIORITY_IDX2});
</#if>
</#if>
<#if CONFIG_DRV_I2S_INST_IDX3 == true>
<#if CONFIG_PIC32MX == true>
    SYS_INT_VectorPrioritySet(DRV_I2S_INT_VECTOR_IDX3, DRV_I2S_INT_PRIORITY_IDX3);
    SYS_INT_VectorSubprioritySet(DRV_I2S_INT_VECTOR_IDX3, DRV_I2S_INT_SUB_PRIORITY_IDX3);
</#if>
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_TX_INT_SRC_IDX3}, ${CONFIG_DRV_I2S_TX_INT_PRIORITY_IDX3});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_TX_INT_SRC_IDX3}, ${CONFIG_DRV_I2S_TX_INT_SUB_PRIORITY_IDX3});
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_RX_INT_SRC_IDX3}, ${CONFIG_DRV_I2S_RX_INT_PRIORITY_IDX3});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_RX_INT_SRC_IDX3}, ${CONFIG_DRV_I2S_RX_INT_SUB_PRIORITY_IDX3});
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_ERR_INT_SRC_IDX3}, ${CONFIG_DRV_I2S_ERR_INT_PRIORITY_IDX3});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_ERR_INT_SRC_IDX3}, ${CONFIG_DRV_I2S_ERR_INT_SUB_PRIORITY_IDX3});
</#if>
</#if>
<#if CONFIG_DRV_I2S_INST_IDX4 == true>
<#if CONFIG_PIC32MX == true>
    SYS_INT_VectorPrioritySet(DRV_I2S_INT_VECTOR_IDX4, DRV_I2S_INT_PRIORITY_IDX4);
    SYS_INT_VectorSubprioritySet(DRV_I2S_INT_VECTOR_IDX4, DRV_I2S_INT_SUB_PRIORITY_IDX4);
</#if>
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_TX_INT_SRC_IDX4}, ${CONFIG_DRV_I2S_TX_INT_PRIORITY_IDX4});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_TX_INT_SRC_IDX4}, ${CONFIG_DRV_I2S_TX_INT_SUB_PRIORITY_IDX4});
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_RX_INT_SRC_IDX4}, ${CONFIG_DRV_I2S_RX_INT_PRIORITY_IDX4});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_RX_INT_SRC_IDX4}, ${CONFIG_DRV_I2S_RX_INT_SUB_PRIORITY_IDX4});
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_ERR_INT_SRC_IDX4}, ${CONFIG_DRV_I2S_ERR_INT_PRIORITY_IDX4});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_ERR_INT_SRC_IDX4}, ${CONFIG_DRV_I2S_ERR_INT_SUB_PRIORITY_IDX4});
</#if>
</#if>
<#if CONFIG_DRV_I2S_INST_IDX5 == true>
<#if CONFIG_PIC32MX == true>
    SYS_INT_VectorPrioritySet(DRV_I2S_INT_VECTOR_IDX5, DRV_I2S_INT_PRIORITY_IDX5);
    SYS_INT_VectorSubprioritySet(DRV_I2S_INT_VECTOR_IDX5, DRV_I2S_INT_SUB_PRIORITY_IDX5);
</#if>
<#if CONFIG_PIC32MZ == true || CONFIG_PIC32MK == true>
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_TX_INT_SRC_IDX5}, ${CONFIG_DRV_I2S_TX_INT_PRIORITY_IDX5});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_TX_INT_SRC_IDX5}, ${CONFIG_DRV_I2S_TX_INT_SUB_PRIORITY_IDX5});
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_RX_INT_SRC_IDX5}, ${CONFIG_DRV_I2S_RX_INT_PRIORITY_IDX5});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_RX_INT_SRC_IDX5}, ${CONFIG_DRV_I2S_RX_INT_SUB_PRIORITY_IDX5});
    SYS_INT_VectorPrioritySet(${CONFIG_DRV_I2S_ERR_INT_SRC_IDX5}, ${CONFIG_DRV_I2S_ERR_INT_PRIORITY_IDX5});
    SYS_INT_VectorSubprioritySet(${CONFIG_DRV_I2S_ERR_INT_SRC_IDX5}, ${CONFIG_DRV_I2S_ERR_INT_SUB_PRIORITY_IDX5});
</#if>
</#if>
</#if>

<#--
/*******************************************************************************
 End of File
*/
-->
